#include <Utils/Processor.h>

#include <Utils/Recorder.h>
#include <Utils/SegmentationWorker.h>

#include <algorithm>

namespace {
    // Frames that may wait; older ones that are not recorded are skipped (the view only needs the newest)
    constexpr size_t MAX_WAITING = 3;
}

Processor::Processor(const OccupancyMapConfig& occupancyConfig, CameraColorizer colorizer, Recorder* recorder,
                     SegmentationWorker* segmentation, Output output)
    : occupancyConfig(occupancyConfig),
      colorizer(std::move(colorizer)),
      recorder(recorder),
      segmentation(segmentation),
      output(std::move(output)),
      thread(&Processor::run, this)
{
}

Processor::~Processor()
{
    stop();
}

void Processor::submit(std::shared_ptr<const Frame> frame)
{
    if (!frame) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (stopping) {
            return;
        }
        queue.push_back(std::move(frame));
        while (queue.size() > MAX_WAITING) {
            const auto skippable = std::find_if(queue.begin(), queue.end() - 1, [](const auto& f) { return !f->record; });
            if (skippable == queue.end() - 1) {
                break;      // Only recorded frames waiting: process them all
            }
            queue.erase(skippable);
        }
    }
    wakeUp.notify_one();
}

void Processor::setColorMode(ColorMode mode, uint64_t newTag)
{
    std::lock_guard<std::mutex> lock(mutex);
    colorMode = mode;
    tag = newTag;
}

void Processor::stop()
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

void Processor::run()
{
    while (true) {
        std::shared_ptr<const Frame> frame;
        ColorMode mode;
        uint64_t frameTag;
        {
            std::unique_lock<std::mutex> lock(mutex);
            wakeUp.wait(lock, [this]() { return stopping || !queue.empty(); });
            if (queue.empty()) {
                return;     // Stopping, and everything is processed
            }
            frame = std::move(queue.front());
            queue.pop_front();
            mode = colorMode;
            frameTag = tag;
        }
        process(frame, mode, frameTag);
    }
}

void Processor::process(const std::shared_ptr<const Frame>& frame, ColorMode mode, uint64_t frameTag)
{
    auto result = std::make_shared<ProcessedFrame>();
    result->frame = frame;
    result->tag = frameTag;

    // Ground plane and occupancy map: once, for the view and the recording
    if (frame->cloud) {
        if (frame->live != planeLive) {
            plane = GroundPlane{};      // From live to replay or back: do not smooth across
            planeLive = frame->live;
        }
        plane = fitGroundPlane(*frame->cloud, plane.valid ? &plane : nullptr);
        result->plane = plane;
        if (plane.valid) {
            result->occupancy = std::make_shared<const OccupancyMap>(
                buildOccupancyMap(projectToGround(*frame->cloud, plane), occupancyConfig, false));
        }
    }

    const bool segmenting = mode == ColorMode::Segmentation && segmentation && segmentation->isAvailable();
    if (frame->record && recorder) {
        // In segmentation mode the GridMap (with the class layer) follows from the segmentation
        recorder->writeFrame(*frame, result->plane, result->occupancy, !segmenting);
    }

    if (frame->cloud) {
        if (segmenting) {
            // The worker keeps only the newest job, so it never falls behind the scans
            segmentation->submit({frameTag, frame, result->plane});
        } else if (mode == ColorMode::Rgb) {
            std::vector<cv::Mat> imagesRGB(frame->cameras.size());
            for (size_t i = 0; i < frame->cameras.size(); ++i) {
                imagesRGB[i] = frame->cameras[i].colourImage;
            }
            auto colored = std::make_shared<std::vector<vaiop::QtPoint>>(*frame->cloud);
            colorizer.colorize(*colored, imagesRGB);
            result->displayCloud = colored;
        } else {
            result->displayCloud = frame->cloud;
        }
    }
    output(result);
}
