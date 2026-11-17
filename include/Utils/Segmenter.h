#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <Utils/SensorConfig.h>

// Semantic segmentation of camera images with an ONNX model, e.g. SegFormer trained on ADE20K as exported by
// Hugging Face (input pixel_values [N, 3, H, W]: RGB with ImageNet normalisation; output logits [N, classes, h, w]).
// Needs ONNX Runtime: without it (built without WITH_SEGMENTATION) isAvailable() is false.
// Runs on the GPU (CUDA) with the GPU build of ONNX Runtime and a working NVIDIA driver, see config.device.
// Not thread safe: use one Segmenter per thread.
class Segmenter {
public:
    static constexpr uint8_t UNKNOWN = 255;

    enum class Device { Cpu, Gpu };
    static const char* deviceName(Device device) { return device == Device::Gpu ? "GPU" : "CPU"; }

    // config.device: "auto" or "gpu" try the GPU first and fall back to the CPU (gpuError() says why), "cpu" = CPU only
    explicit Segmenter(const SegmentationConfig& config);
    ~Segmenter();

    bool isAvailable() const;
    // Why the model is not available
    const std::string& error() const;
    // Where the model runs
    Device device() const;
    // Why the model does not run on the GPU; empty when it does or when the CPU was asked for
    const std::string& gpuError() const;

    const std::vector<std::string>& classNames() const;
    // RGB colour per class
    const std::vector<cv::Vec3b>& palette() const;

    // Class id per pixel (CV_8U, size of the image), UNKNOWN where the model is not confident enough.
    // Returns an empty Mat when the model is not available or the image is not RGB (CV_8UC3).
    cv::Mat segment(const cv::Mat& imageRGB);
    // Several images at once: images of the same size go through the model in one batch (faster on the GPU).
    // One label map per image, empty for an empty or non-RGB image.
    std::vector<cv::Mat> segment(const std::vector<cv::Mat>& imagesRGB);

    // The image with the class colours blended on top (alpha = weight of the class colour); UNKNOWN pixels unchanged
    static cv::Mat overlay(const cv::Mat& imageRGB, const cv::Mat& labels, const std::vector<cv::Vec3b>& palette, float alpha = 0.55f);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
