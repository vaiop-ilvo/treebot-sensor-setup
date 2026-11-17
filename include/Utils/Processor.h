#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <Utils/Frame.h>
#include <Utils/PointCloudColor.h>
#include <Utils/SensorConfig.h>

class Recorder;
class RosPublisher;
class SegmentationWorker;

// Processes the frames on its own thread, once for the view and the recording: fits the ground plane (smoothed over
// time), builds the occupancy map, colours the cloud with the cameras (RGB mode), records the frame and submits it to
// the segmentation (segmentation mode) and publishes it on the ROS topics. Frames are processed in order; when the processing falls behind, frames that
// are not recorded are skipped (recorded frames never are).
class Processor {
public:
    using Output = std::function<void(std::shared_ptr<const ProcessedFrame>)>;

    // recorder, segmentation and ros may be nullptr; output is called on the processor thread. ros publishes the
    // live frames (not the replay).
    Processor(const OccupancyMapConfig& occupancyConfig, CameraColorizer colorizer, Recorder* recorder,
              SegmentationWorker* segmentation, Output output, RosPublisher* ros = nullptr);
    ~Processor();

    void submit(std::shared_ptr<const Frame> frame);
    // Colour mode of the next frames, and the tag they carry (results with an older tag are outdated)
    void setColorMode(ColorMode mode, uint64_t tag);
    // Waits for the queued frames and stops the thread
    void stop();

private:
    const OccupancyMapConfig occupancyConfig;
    const CameraColorizer colorizer;
    Recorder* const recorder;
    SegmentationWorker* const segmentation;
    const Output output;
    RosPublisher* const ros;

    std::mutex mutex;
    std::condition_variable wakeUp;
    std::deque<std::shared_ptr<const Frame>> queue;
    bool stopping = false;
    ColorMode colorMode = ColorMode::Distance;
    uint64_t tag = 0;

    // Processor thread only
    GroundPlane plane;                  // Previous fit, for smoothing over time
    bool planeLive = true;              // plane belongs to the live frames (or the replay)

    std::thread thread;                 // Last: started in the constructor, after the members it uses

    void run();
    void process(const std::shared_ptr<const Frame>& frame, ColorMode mode, uint64_t frameTag);
};
