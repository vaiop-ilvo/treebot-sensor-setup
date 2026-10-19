#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <Lidar/QtPoint.h>
#include <Utils/PointCloudColor.h>
#include <Utils/Segmenter.h>

// Segments the camera images and colours the point cloud by class on a background thread, so the GUI never
// waits for the model. Only the newest job is kept: a job submitted while the model is busy replaces the one
// that is waiting, so the view shows the latest scan instead of falling behind.
class SegmentationWorker {
public:
    struct Job {
        uint64_t tag = 0;   // Returned with the result, to recognise outdated results
        std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud;
        std::vector<cv::Mat> imagesRGB;     // In the order of the calibrations; empty = no image
    };
    struct Result {
        uint64_t tag = 0;
        std::shared_ptr<std::vector<vaiop::QtPoint>> cloud;    // Copy of the job cloud, coloured by class
        std::vector<uint8_t> labels;                           // Class per point, CameraColorizer::UNSEEN_LABEL when not seen
        std::vector<size_t> classPoints;                       // Number of points per class
        double ms = 0.0;                                       // Time for segmenting and colouring
    };

    // onResult is called on the worker thread
    SegmentationWorker(const SegmentationConfig& config, CameraColorizer colorizer, std::function<void(Result)> onResult);
    ~SegmentationWorker();

    bool isAvailable() const;
    // Why the model is not available
    const std::string& error() const;
    const std::vector<std::string>& classNames() const;
    const std::vector<cv::Vec3b>& palette() const;

    void submit(Job job);

private:
    Segmenter segmenter;
    const CameraColorizer colorizer;
    const std::function<void(Result)> onResult;

    std::mutex mutex;
    std::condition_variable wakeUp;
    std::optional<Job> waiting;
    bool stopping = false;
    std::thread thread;

    void run();
    Result process(const Job& job);
};
