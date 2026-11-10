#include <Utils/Segmenter.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <map>

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <opencv2/imgproc.hpp>

#ifdef WITH_SEGMENTATION
#include <dlfcn.h>
#include <onnxruntime_cxx_api.h>
#endif

namespace {
    // ImageNet normalisation used by SegFormer (preprocessor_config.json)
    constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    constexpr float STD[3] = {0.229f, 0.224f, 0.225f};
    // The network input is a multiple of this (SegFormer downsamples 32x)
    constexpr int INPUT_MULTIPLE = 32;

    // Recognisable colours for the classes that matter outdoors; the other classes get a generated colour
    const std::map<std::string, cv::Vec3b> NAMED_COLORS = {
        {"sky", {110, 170, 235}},     {"tree", {30, 130, 40}},      {"plant", {100, 200, 60}},
        {"grass", {160, 225, 100}},   {"field", {200, 205, 110}},   {"earth", {150, 105, 65}},
        {"sand", {220, 200, 140}},    {"floor", {175, 145, 115}},   {"road", {120, 120, 120}},
        {"sidewalk", {185, 185, 185}}, {"person", {230, 40, 40}},   {"car", {40, 80, 220}},
        {"truck", {20, 50, 160}},     {"tractor", {240, 130, 20}},  {"building", {170, 110, 90}},
        {"wall", {205, 195, 175}},    {"ceiling", {235, 235, 210}}, {"fence", {225, 160, 30}},
        {"pole", {250, 225, 0}},      {"water", {40, 150, 200}},    {"rock", {130, 115, 105}},
    };

    // Generated colours: hues spread by the golden ratio, so neighbouring class ids differ
    cv::Vec3b generatedColor(size_t index)
    {
        const float hue = std::fmod(static_cast<float>(index) * 0.618034f, 1.0f) * 180.0f;
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 150 + 60 * (index % 2), 170 + 50 * ((index / 2) % 2)));
        cv::Mat rgb;
        cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);
        return rgb.at<cv::Vec3b>(0, 0);
    }

    // id2label of the Hugging Face config.json
    std::vector<std::string> readClassNames(const std::string& configFile, std::string& error)
    {
        QFile file(QString::fromStdString(configFile));
        if (!file.open(QIODevice::ReadOnly)) {
            error = "Cannot open " + configFile;
            return {};
        }
        const QJsonObject id2label = QJsonDocument::fromJson(file.readAll()).object()["id2label"].toObject();
        std::vector<std::string> names(id2label.size());
        for (auto it = id2label.begin(); it != id2label.end(); ++it) {
            const int id = it.key().toInt();
            if (id >= 0 && id < static_cast<int>(names.size())) {
                names[id] = it.value().toString().trimmed().toStdString();
            }
        }
        if (names.empty()) {
            error = "No id2label in " + configFile;
        }
        return names;
    }

#ifdef WITH_SEGMENTATION
    // Why there is no usable CUDA GPU (driver not installed or not loaded, no GPU); empty when there is one.
    // Asks the driver library directly, which is quicker and clearer than a failing ONNX Runtime session.
    std::string cudaDeviceError()
    {
        void* driver = dlopen("libcuda.so.1", RTLD_NOW | RTLD_GLOBAL);
        if (!driver) {
            return "NVIDIA driver not installed (libcuda.so.1 not found)";
        }
        using InitFunction = int (*)(unsigned int);
        using CountFunction = int (*)(int*);
        const auto init = reinterpret_cast<InitFunction>(dlsym(driver, "cuInit"));
        const auto count = reinterpret_cast<CountFunction>(dlsym(driver, "cuDeviceGetCount"));
        if (!init || !count) {
            return "Unexpected NVIDIA driver library (no cuInit)";
        }
        if (const int result = init(0); result != 0) {
            return "No CUDA GPU: the NVIDIA driver is not loaded (cuInit error " + std::to_string(result) + "), check nvidia-smi";
        }
        int devices = 0;
        if (count(&devices) != 0 || devices == 0) {
            return "No CUDA GPU found";
        }
        return {};
    }

    // The GPU build of ONNX Runtime needs the CUDA 12 / cuDNN 9 libraries. scripts/download_models.sh --gpu puts them in
    // SEGMENTATION_CUDA_DIR, which is not on the library search path: loading them here (RTLD_GLOBAL) lets the CUDA
    // provider find them by name. Without that folder the system libraries are used (e.g. /usr/local/cuda via ldconfig).
    void loadCudaLibraries()
    {
        static const bool loaded = []() {
#ifdef SEGMENTATION_CUDA_DIR
            const std::filesystem::path dir = SEGMENTATION_CUDA_DIR;
            // Dependencies first: cuBLAS needs cuBLASLt, the cuDNN parts need libcudnn_graph and libcudnn_ops, ...
            const char* libraries[] = {
                "libcudart.so.12", "libnvrtc.so.12", "libcublasLt.so.12", "libcublas.so.12", "libcufft.so.11",
                "libcurand.so.10", "libcudnn_graph.so.9", "libcudnn_engines_precompiled.so.9",
                "libcudnn_engines_runtime_compiled.so.9", "libcudnn_heuristic.so.9", "libcudnn_ops.so.9",
                "libcudnn_cnn.so.9", "libcudnn_adv.so.9", "libcudnn.so.9",
            };
            for (const char* library : libraries) {
                const std::filesystem::path file = dir / library;
                if (std::filesystem::exists(file)) {
                    dlopen(file.c_str(), RTLD_NOW | RTLD_GLOBAL);
                }
            }
#endif
            return true;
        }();
        (void)loaded;
    }

    bool hasCudaProvider()
    {
        const std::vector<std::string> providers = Ort::GetAvailableProviders();
        return std::find(providers.begin(), providers.end(), "CUDAExecutionProvider") != providers.end();
    }
#endif
}

struct Segmenter::Impl {
    SegmentationConfig config;
    std::string error;
    Device device = Device::Cpu;
    std::string gpuError;
    std::vector<std::string> names;
    std::vector<cv::Vec3b> palette;
#ifdef WITH_SEGMENTATION
    std::unique_ptr<Ort::Session> session;
    std::string inputName;
    std::string outputName;
#endif
};

Segmenter::Segmenter(const SegmentationConfig& config)
    : impl(std::make_unique<Impl>())
{
    impl->config = config;
    if (config.modelDir.empty()) {
        impl->error = "No segmentation model configured";
        return;
    }

    impl->names = readClassNames(config.modelDir + "/config.json", impl->error);
    if (impl->names.empty()) {
        return;
    }
    for (size_t i = 0; i < impl->names.size(); ++i) {
        const auto named = NAMED_COLORS.find(impl->names[i]);
        impl->palette.push_back(named != NAMED_COLORS.end() ? named->second : generatedColor(i));
    }

#ifdef WITH_SEGMENTATION
    // One environment for the whole program
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "segmentation");
    const std::string modelFile = config.modelDir + "/model.onnx";
    auto makeOptions = [&config]() {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(config.threads);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        return options;
    };

    // 1. GPU, unless the CPU is asked for
    if (config.device != "cpu") {
        if (!hasCudaProvider()) {
            impl->gpuError = "ONNX Runtime is the CPU build (get the GPU build with scripts/download_models.sh --gpu)";
        } else if (impl->gpuError = cudaDeviceError(); impl->gpuError.empty()) {
            try {
                loadCudaLibraries();
                Ort::SessionOptions options = makeOptions();
                OrtCUDAProviderOptions cuda;
                cuda.device_id = 0;
                options.AppendExecutionProvider_CUDA(cuda);
                impl->session = std::make_unique<Ort::Session>(env, modelFile.c_str(), options);
                impl->device = Device::Gpu;
            } catch (const Ort::Exception& e) {
                impl->gpuError = std::string("Cannot run the model on the GPU: ") + e.what();
                impl->session.reset();
            }
        }
    }

    // 2. CPU
    if (!impl->session) {
        try {
            impl->session = std::make_unique<Ort::Session>(env, modelFile.c_str(), makeOptions());
            impl->device = Device::Cpu;
        } catch (const Ort::Exception& e) {
            impl->error = std::string("Cannot load the segmentation model: ") + e.what();
            impl->session.reset();
            return;
        }
    }

    Ort::AllocatorWithDefaultOptions allocator;
    impl->inputName = impl->session->GetInputNameAllocated(0, allocator).get();
    impl->outputName = impl->session->GetOutputNameAllocated(0, allocator).get();
#else
    impl->error = "Built without ONNX Runtime (thirdparty/onnxruntime): segmentation is not available";
#endif
}

Segmenter::~Segmenter() = default;

bool Segmenter::isAvailable() const
{
#ifdef WITH_SEGMENTATION
    return impl->session != nullptr;
#else
    return false;
#endif
}

const std::string& Segmenter::error() const
{
    return impl->error;
}

Segmenter::Device Segmenter::device() const
{
    return impl->device;
}

const std::string& Segmenter::gpuError() const
{
    return impl->gpuError;
}

const std::vector<std::string>& Segmenter::classNames() const
{
    return impl->names;
}

const std::vector<cv::Vec3b>& Segmenter::palette() const
{
    return impl->palette;
}

cv::Mat Segmenter::segment(const cv::Mat& imageRGB)
{
    return segment(std::vector<cv::Mat>{imageRGB}).front();
}

std::vector<cv::Mat> Segmenter::segment(const std::vector<cv::Mat>& imagesRGB)
{
    std::vector<cv::Mat> results(imagesRGB.size());
#ifdef WITH_SEGMENTATION
    if (!impl->session) {
        return results;
    }

    // Images of the same size form one batch
    std::map<std::pair<int, int>, std::vector<size_t>> batches;
    for (size_t i = 0; i < imagesRGB.size(); ++i) {
        if (!imagesRGB[i].empty() && imagesRGB[i].type() == CV_8UC3) {
            batches[{imagesRGB[i].cols, imagesRGB[i].rows}].push_back(i);
        }
    }

    for (const auto& [imageSize, indices] : batches) {
        // 1. Network input: long side inputSize, aspect ratio kept, both sides a multiple of 32
        const double scale = static_cast<double>(impl->config.inputSize) / std::max(imageSize.first, imageSize.second);
        auto roundedSide = [scale](int side) {
            return std::max(INPUT_MULTIPLE, static_cast<int>(std::lround(side * scale / INPUT_MULTIPLE)) * INPUT_MULTIPLE);
        };
        const int width = roundedSide(imageSize.first);
        const int height = roundedSide(imageSize.second);
        const size_t n = indices.size();
        const size_t inputPlane = static_cast<size_t>(height) * width;

        // 2. HWC uint8 -> NCHW float, normalised
        std::vector<float> input(n * 3 * inputPlane);
        for (size_t b = 0; b < n; ++b) {
            cv::Mat resized;
            cv::resize(imagesRGB[indices[b]], resized, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
            for (int c = 0; c < 3; ++c) {
                float* plane = input.data() + (b * 3 + c) * inputPlane;
                for (int y = 0; y < height; ++y) {
                    const cv::Vec3b* row = resized.ptr<cv::Vec3b>(y);
                    for (int x = 0; x < width; ++x) {
                        plane[y * width + x] = (row[x][c] / 255.0f - MEAN[c]) / STD[c];
                    }
                }
            }
        }

        // 3. Inference
        const std::array<int64_t, 4> shape = {static_cast<int64_t>(n), 3, height, width};
        const Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value tensor = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(), shape.data(), shape.size());
        const char* inputNames[] = {impl->inputName.c_str()};
        const char* outputNames[] = {impl->outputName.c_str()};
        std::vector<Ort::Value> outputs;
        try {
            outputs = impl->session->Run(Ort::RunOptions{nullptr}, inputNames, &tensor, 1, outputNames, 1);
        } catch (const Ort::Exception& e) {
            impl->error = std::string("Segmentation failed: ") + e.what();
            continue;
        }

        // 4. Logits [n, classes, h, w] -> class with the highest score per pixel
        const std::vector<int64_t> outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        if (outShape.size() != 4 || outShape[0] != static_cast<int64_t>(n)) {
            continue;
        }
        const int nClasses = static_cast<int>(outShape[1]);
        const int h = static_cast<int>(outShape[2]);
        const int w = static_cast<int>(outShape[3]);
        const size_t planeSize = static_cast<size_t>(h) * w;
        const float minConfidence = impl->config.minConfidence;

        for (size_t b = 0; b < n; ++b) {
            const float* logits = outputs[0].GetTensorData<float>() + b * nClasses * planeSize;
            cv::Mat labels(h, w, CV_8U);
            for (size_t p = 0; p < planeSize; ++p) {
                int best = 0;
                float bestScore = logits[p];
                for (int k = 1; k < nClasses; ++k) {
                    const float score = logits[k * planeSize + p];
                    if (score > bestScore) {
                        bestScore = score;
                        best = k;
                    }
                }
                if (minConfidence > 0.0f) {
                    // Softmax probability of the best class
                    float sum = 0.0f;
                    for (int k = 0; k < nClasses; ++k) {
                        sum += std::exp(logits[k * planeSize + p] - bestScore);
                    }
                    if (1.0f / sum < minConfidence) {
                        best = UNKNOWN;
                    }
                }
                labels.data[p] = static_cast<uint8_t>(std::min(best, static_cast<int>(UNKNOWN)));
            }

            // 5. Back to the image size (the output covers the whole image at a quarter of the input resolution)
            cv::resize(labels, results[indices[b]], cv::Size(imageSize.first, imageSize.second), 0, 0, cv::INTER_NEAREST);
        }
    }
#else
    (void)imagesRGB;
#endif
    return results;
}

cv::Mat Segmenter::overlay(const cv::Mat& imageRGB, const cv::Mat& labels, const std::vector<cv::Vec3b>& palette, float alpha)
{
    if (imageRGB.empty() || labels.size() != imageRGB.size() || labels.type() != CV_8U) {
        return imageRGB.clone();
    }
    // Class colour per label value; UNKNOWN (and labels outside the palette) keep the image colour (weight 0)
    cv::Mat lut(1, 256, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat weight(1, 256, CV_8U, cv::Scalar(0));
    for (size_t k = 0; k < palette.size() && k < UNKNOWN; ++k) {
        lut.at<cv::Vec3b>(0, static_cast<int>(k)) = palette[k];
        weight.at<uint8_t>(0, static_cast<int>(k)) = 255;
    }
    cv::Mat labels3, colors, mask;
    cv::cvtColor(labels, labels3, cv::COLOR_GRAY2RGB);
    cv::LUT(labels3, lut, colors);
    cv::LUT(labels, weight, mask);

    cv::Mat blended;
    cv::addWeighted(imageRGB, 1.0 - alpha, colors, alpha, 0.0, blended);
    cv::Mat result = imageRGB.clone();
    blended.copyTo(result, mask);
    return result;
}
