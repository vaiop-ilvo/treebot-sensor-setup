#include <Utils/SegmentationWorker.h>

#include <chrono>

SegmentationWorker::SegmentationWorker(const SegmentationConfig& config, CameraColorizer colorizer,
                                       std::function<void(Result)> onResult, std::function<void(DeviceStatus)> onDeviceChanged)
    : config(config),
      segmenter(std::make_unique<Segmenter>(config)),
      available(segmenter->isAvailable()),
      loadError(segmenter->error()),
      names(segmenter->classNames()),
      colors(segmenter->palette()),
      colorizer(std::move(colorizer)),
      onResult(std::move(onResult)),
      onDeviceChanged(std::move(onDeviceChanged)),
      status{segmenter->device(), segmenter->gpuError()}
{
    if (available) {
        thread = std::thread(&SegmentationWorker::run, this);
    }
}

SegmentationWorker::~SegmentationWorker()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = true;
    }
    wakeUp.notify_one();
    if (thread.joinable()) {
        thread.join();
    }
}

bool SegmentationWorker::isAvailable() const
{
    return available;
}

const std::string& SegmentationWorker::error() const
{
    return loadError;
}

const std::vector<std::string>& SegmentationWorker::classNames() const
{
    return names;
}

const std::vector<cv::Vec3b>& SegmentationWorker::palette() const
{
    return colors;
}

SegmentationWorker::DeviceStatus SegmentationWorker::deviceStatus() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return status;
}

void SegmentationWorker::setDevice(Segmenter::Device device)
{
    if (!available) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        requestedDevice = device;
    }
    wakeUp.notify_one();
}

void SegmentationWorker::submit(Job job)
{
    if (!available || !job.cloud) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        waiting = std::move(job);
    }
    wakeUp.notify_one();
}

void SegmentationWorker::run()
{
    while (true) {
        std::optional<Job> job;
        std::optional<Segmenter::Device> device;
        {
            std::unique_lock<std::mutex> lock(mutex);
            wakeUp.wait(lock, [this]() { return stopping || waiting.has_value() || requestedDevice.has_value(); });
            if (stopping) {
                return;
            }
            std::swap(device, requestedDevice);
            std::swap(job, waiting);
        }
        if (device) {
            loadOn(*device);
        }
        if (job) {
            onResult(process(*job));
        }
    }
}

void SegmentationWorker::loadOn(Segmenter::Device device)
{
    SegmentationConfig deviceConfig = config;
    deviceConfig.device = device == Segmenter::Device::Gpu ? "gpu" : "cpu";
    auto next = std::make_unique<Segmenter>(deviceConfig);
    if (next->isAvailable()) {
        segmenter = std::move(next);    // Frees the previous session (and its GPU memory)
    }
    DeviceStatus newStatus{segmenter->device(), next ? next->gpuError() : segmenter->gpuError()};
    {
        std::lock_guard<std::mutex> lock(mutex);
        status = newStatus;
    }
    if (onDeviceChanged) {
        onDeviceChanged(newStatus);
    }
}

SegmentationWorker::Result SegmentationWorker::process(const Job& job)
{
    const auto tic = std::chrono::steady_clock::now();

    Result result;
    result.device = segmenter->device();
    // All cameras in one go: on the GPU they form one batch
    const std::vector<cv::Mat> labelMaps = segmenter->segment(job.imagesRGB);
    result.overlays.resize(job.imagesRGB.size());
    for (size_t c = 0; c < job.imagesRGB.size(); ++c) {
        if (!job.imagesRGB[c].empty()) {
            result.overlays[c] = Segmenter::overlay(job.imagesRGB[c], labelMaps[c], colors);
        }
    }

    result.tag = job.tag;
    result.stamp = job.stamp;
    result.plane = job.plane;
    result.record = job.record;
    result.cloud = std::make_shared<std::vector<vaiop::QtPoint>>(*job.cloud);
    colorizer.colorizeLabels(*result.cloud, labelMaps, colors, &result.labels);

    result.classPoints.assign(names.size(), 0);
    for (const uint8_t label : result.labels) {
        if (label < result.classPoints.size()) {
            result.classPoints[label]++;
        }
    }
    result.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tic).count();
    return result;
}
