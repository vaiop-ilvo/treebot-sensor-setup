// Segments camera images with the segmentation model, as the main application does in segmentation mode.
// Usage: segment_image <model dir> <out dir> <image.png> [<image.png> ...] [--size N] [--threads N]
// Writes <out dir>/<image name>_seg.png: the image with the class colours on top, and prints the classes found.
#include <Utils/Segmenter.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <model dir> <out dir> <image.png> [<image.png> ...] [--size N] [--threads N]\n";
        return 1;
    }
    SegmentationConfig config;
    config.modelDir = argv[1];
    const std::filesystem::path outDir = argv[2];
    std::vector<std::string> images;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--size" && i + 1 < argc) {
            config.inputSize = std::stoi(argv[++i]);
        } else if (arg == "--threads" && i + 1 < argc) {
            config.threads = std::stoi(argv[++i]);
        } else {
            images.push_back(arg);
        }
    }

    Segmenter segmenter(config);
    if (!segmenter.isAvailable()) {
        std::cerr << segmenter.error() << "\n";
        return 1;
    }
    std::filesystem::create_directories(outDir);
    const auto& names = segmenter.classNames();
    const auto& palette = segmenter.palette();

    for (const std::string& path : images) {
        cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
        if (bgr.empty()) {
            std::cerr << "Cannot read " << path << "\n";
            continue;
        }
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        segmenter.segment(rgb);     // Warm-up: the first run allocates the buffers
        const auto tic = std::chrono::steady_clock::now();
        const cv::Mat labels = segmenter.segment(rgb);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tic).count();

        // Class colours blended over the image, as the image overlay of the application
        std::vector<int> counts(256, 0);
        for (int y = 0; y < labels.rows; ++y) {
            for (int x = 0; x < labels.cols; ++x) {
                counts[labels.at<uint8_t>(y, x)]++;
            }
        }
        cv::Mat blended = Segmenter::overlay(rgb, labels, palette);
        cv::cvtColor(blended, blended, cv::COLOR_RGB2BGR);
        const std::filesystem::path out = outDir / (std::filesystem::path(path).stem().string() + "_seg.png");
        cv::imwrite(out.string(), blended);

        std::vector<int> order(256);
        for (int i = 0; i < 256; ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&counts](int a, int b) { return counts[a] > counts[b]; });
        std::cout << path << ": " << ms << " ms, " << labels.total() << " px ->";
        for (int i = 0; i < 6 && counts[order[i]] > 0; ++i) {
            const int label = order[i];
            const std::string name = label < static_cast<int>(names.size()) ? names[label] : "unknown";
            std::cout << " " << name << " " << (100 * counts[label] / static_cast<int>(labels.total())) << "%";
        }
        std::cout << "\n";
    }
    return 0;
}
