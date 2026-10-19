#include <Utils/Segmenter.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <opencv2/imgproc.hpp>

#ifdef WITH_SEGMENTATION
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
}

struct Segmenter::Impl {
    SegmentationConfig config;
    std::string error;
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
    try {
        // One environment for the whole program
        static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "segmentation");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(config.threads);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        const std::string modelFile = config.modelDir + "/model.onnx";
        impl->session = std::make_unique<Ort::Session>(env, modelFile.c_str(), options);

        Ort::AllocatorWithDefaultOptions allocator;
        impl->inputName = impl->session->GetInputNameAllocated(0, allocator).get();
        impl->outputName = impl->session->GetOutputNameAllocated(0, allocator).get();
    } catch (const Ort::Exception& e) {
        impl->error = std::string("Cannot load the segmentation model: ") + e.what();
        impl->session.reset();
    }
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
#ifdef WITH_SEGMENTATION
    if (!impl->session || imageRGB.empty() || imageRGB.type() != CV_8UC3) {
        return {};
    }

    // 1. Network input: long side inputSize, aspect ratio kept, both sides a multiple of 32
    const double scale = static_cast<double>(impl->config.inputSize) / std::max(imageRGB.cols, imageRGB.rows);
    auto roundedSide = [scale](int side) {
        return std::max(INPUT_MULTIPLE, static_cast<int>(std::lround(side * scale / INPUT_MULTIPLE)) * INPUT_MULTIPLE);
    };
    const int width = roundedSide(imageRGB.cols);
    const int height = roundedSide(imageRGB.rows);
    cv::Mat resized;
    cv::resize(imageRGB, resized, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);

    // 2. HWC uint8 -> CHW float, normalised
    std::vector<float> input(static_cast<size_t>(3) * height * width);
    for (int c = 0; c < 3; ++c) {
        float* plane = input.data() + static_cast<size_t>(c) * height * width;
        for (int y = 0; y < height; ++y) {
            const cv::Vec3b* row = resized.ptr<cv::Vec3b>(y);
            for (int x = 0; x < width; ++x) {
                plane[y * width + x] = (row[x][c] / 255.0f - MEAN[c]) / STD[c];
            }
        }
    }

    // 3. Inference
    const std::array<int64_t, 4> shape = {1, 3, height, width};
    const Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value tensor = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(), shape.data(), shape.size());
    const char* inputNames[] = {impl->inputName.c_str()};
    const char* outputNames[] = {impl->outputName.c_str()};
    std::vector<Ort::Value> outputs;
    try {
        outputs = impl->session->Run(Ort::RunOptions{nullptr}, inputNames, &tensor, 1, outputNames, 1);
    } catch (const Ort::Exception& e) {
        impl->error = std::string("Segmentation failed: ") + e.what();
        return {};
    }

    // 4. Logits [1, classes, h, w] -> class with the highest score per pixel
    const std::vector<int64_t> outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (outShape.size() != 4) {
        return {};
    }
    const int nClasses = static_cast<int>(outShape[1]);
    const int h = static_cast<int>(outShape[2]);
    const int w = static_cast<int>(outShape[3]);
    const float* logits = outputs[0].GetTensorData<float>();
    const size_t planeSize = static_cast<size_t>(h) * w;
    const float minConfidence = impl->config.minConfidence;

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
    cv::Mat full;
    cv::resize(labels, full, imageRGB.size(), 0, 0, cv::INTER_NEAREST);
    return full;
#else
    (void)imageRGB;
    return {};
#endif
}
