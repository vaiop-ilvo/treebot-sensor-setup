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
// The model can be moved between GPU and CPU while running (setDevice); that also happens on the worker thread.
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
        std::vector<cv::Mat> overlays;                         // Camera images (RGB) with the classes on top; empty = no image
        double ms = 0.0;                                       // Time for segmenting and colouring
        Segmenter::Device device = Segmenter::Device::Cpu;     // Where the model ran
    };
    // Where the model runs, and why not on the GPU (empty when it does or when the CPU was asked for)
    struct DeviceStatus {
        Segmenter::Device device = Segmenter::Device::Cpu;
        std::string gpuError;
    };

    // onResult and onDeviceChanged are called on the worker thread. The model is loaded on config.device.
    SegmentationWorker(const SegmentationConfig& config, CameraColorizer colorizer, std::function<void(Result)> onResult,
                       std::function<void(DeviceStatus)> onDeviceChanged = {});
    ~SegmentationWorker();

    bool isAvailable() const;
    // Why the model is not available
    const std::string& error() const;
    const std::vector<std::string>& classNames() const;
    const std::vector<cv::Vec3b>& palette() const;

    DeviceStatus deviceStatus() const;
    // Reloads the model on this device (GPU falls back to the CPU); onDeviceChanged reports the outcome
    void setDevice(Segmenter::Device device);

    void submit(Job job);

private:
    const SegmentationConfig config;
    std::unique_ptr<Segmenter> segmenter;   // Replaced on the worker thread by setDevice
    // Copies of the segmenter's, so the GUI thread can read them while the segmenter is replaced
    const bool available;
    const std::string loadError;
    const std::vector<std::string> names;
    const std::vector<cv::Vec3b> colors;
    const CameraColorizer colorizer;
    const std::function<void(Result)> onResult;
    const std::function<void(DeviceStatus)> onDeviceChanged;

    mutable std::mutex mutex;
    std::condition_variable wakeUp;
    std::optional<Job> waiting;
    std::optional<Segmenter::Device> requestedDevice;
    DeviceStatus status;
    bool stopping = false;
    std::thread thread;

    void run();
    void loadOn(Segmenter::Device device);
    Result process(const Job& job);
};
