#include <Utils/SegmentationWorker.h>

#include <chrono>

SegmentationWorker::SegmentationWorker(const SegmentationConfig& config, CameraColorizer colorizer,
                                       std::function<void(Result)> onResult)
    : segmenter(config),
      colorizer(std::move(colorizer)),
      onResult(std::move(onResult))
{
    if (segmenter.isAvailable()) {
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
    return segmenter.isAvailable();
}

const std::string& SegmentationWorker::error() const
{
    return segmenter.error();
}

const std::vector<std::string>& SegmentationWorker::classNames() const
{
    return segmenter.classNames();
}

const std::vector<cv::Vec3b>& SegmentationWorker::palette() const
{
    return segmenter.palette();
}

void SegmentationWorker::submit(Job job)
{
    if (!segmenter.isAvailable() || !job.cloud) {
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
        Job job;
        {
            std::unique_lock<std::mutex> lock(mutex);
            wakeUp.wait(lock, [this]() { return stopping || waiting.has_value(); });
            if (stopping) {
                return;
            }
            job = std::move(*waiting);
            waiting.reset();
        }
        onResult(process(job));
    }
}

SegmentationWorker::Result SegmentationWorker::process(const Job& job)
{
    const auto tic = std::chrono::steady_clock::now();

    std::vector<cv::Mat> labelMaps(job.imagesRGB.size());
    for (size_t c = 0; c < job.imagesRGB.size(); ++c) {
        if (!job.imagesRGB[c].empty()) {
            labelMaps[c] = segmenter.segment(job.imagesRGB[c]);
        }
    }

    Result result;
    result.tag = job.tag;
    result.cloud = std::make_shared<std::vector<vaiop::QtPoint>>(*job.cloud);
    colorizer.colorizeLabels(*result.cloud, labelMaps, segmenter.palette(), &result.labels);

    result.classPoints.assign(segmenter.classNames().size(), 0);
    for (const uint8_t label : result.labels) {
        if (label < result.classPoints.size()) {
            result.classPoints[label]++;
        }
    }
    result.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tic).count();
    return result;
}
