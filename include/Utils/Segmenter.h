#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <Utils/SensorConfig.h>

// Semantic segmentation of camera images with an ONNX model, e.g. SegFormer trained on ADE20K as exported by
// Hugging Face (input pixel_values [N, 3, H, W]: RGB with ImageNet normalisation; output logits [N, classes, h, w]).
// Needs ONNX Runtime: without it (built without WITH_SEGMENTATION) isAvailable() is false.
// Not thread safe: use one Segmenter per thread.
class Segmenter {
public:
    static constexpr uint8_t UNKNOWN = 255;

    explicit Segmenter(const SegmentationConfig& config);
    ~Segmenter();

    bool isAvailable() const;
    // Why the model is not available
    const std::string& error() const;

    const std::vector<std::string>& classNames() const;
    // RGB colour per class
    const std::vector<cv::Vec3b>& palette() const;

    // Class id per pixel (CV_8U, size of the image), UNKNOWN where the model is not confident enough.
    // Returns an empty Mat when the model is not available or the image is not RGB (CV_8UC3).
    cv::Mat segment(const cv::Mat& imageRGB);

    // The image with the class colours blended on top (alpha = weight of the class colour); UNKNOWN pixels unchanged
    static cv::Mat overlay(const cv::Mat& imageRGB, const cv::Mat& labels, const std::vector<cv::Vec3b>& palette, float alpha = 0.55f);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
