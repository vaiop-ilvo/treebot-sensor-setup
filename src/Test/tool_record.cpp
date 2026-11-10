// Records a rosbag without the GUI, through the same pipeline as the application (Acquisition -> Processor ->
// Recorder, with --segment also the SegmentationWorker). Stops after --seconds or on Ctrl+C.
// Usage: treebot_tool record <folder> [--seconds N] [--segment]
// Prints per second the rows, the processing delay and the write queue; at the end the totals.
#include <Utils/Acquisition.h>
#include <Utils/Processor.h>
#include <Utils/Recorder.h>
#include <Utils/SegmentationWorker.h>
#include <Utils/SensorConfig.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace {
    std::atomic<bool> interrupted{false};

    uint64_t hostTimeMicros()
    {
        return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
}

int recordCommand(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <folder> [--seconds N] [--segment]\n";
        return 1;
    }
    const std::string folder = argv[1];
    double seconds = 10.0;
    bool segment = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else if (arg == "--segment") {
            segment = true;
        }
    }
    std::signal(SIGINT, [](int) { interrupted = true; });
    std::signal(SIGTERM, [](int) { interrupted = true; });

    const SensorConfig& config = sensorConfig();
    std::vector<CameraCalib> calibs;
    for (const CameraInfo& camInfo : config.cameras) {
        calibs.push_back(loadCameraCalib(camInfo.calibFileName, camInfo.name));
    }
    const CameraColorizer colorizer(calibs);
    Recorder recorder(config, calibs);

    std::unique_ptr<SegmentationWorker> segmentation;
    std::atomic<uint64_t> segmented{0};
    if (segment) {
        segmentation = std::make_unique<SegmentationWorker>(config.segmentation, config.occupancyMap, colorizer,
            [&recorder, &segmented](SegmentationWorker::Result result) {
                if (result.frame && result.frame->record) {
                    recorder.writeSegmentation(*result.frame, result.cloud, result.labels, result.classMap);
                }
                segmented++;
            });
        if (!segmentation->isAvailable()) {
            std::cerr << "No segmentation: " << segmentation->error() << "\n";
            return 1;
        }
        recorder.setClassNames(segmentation->classNames());
        std::cout << "Segmentation on the " << Segmenter::deviceName(segmentation->deviceStatus().device) << "\n";
    }

    // Processing delay: from the trigger to the end of the processing
    std::mutex statsMutex;
    uint64_t processed = 0, delaySumUs = 0, delayMaxUs = 0;
    Processor processor(config.occupancyMap, colorizer, &recorder, segmentation.get(),
        [&](std::shared_ptr<const ProcessedFrame> frame) {
            const uint64_t delay = hostTimeMicros() - frame->frame->triggerTs;
            std::lock_guard<std::mutex> lock(statsMutex);
            processed++;
            delaySumUs += delay;
            delayMaxUs = std::max(delayMaxUs, delay);
        });
    processor.setColorMode(segment ? ColorMode::Segmentation : ColorMode::Distance, 0);

    std::atomic<bool> connected{false};
    Acquisition acquisition(config, calibs, [&recorder]() { return recorder.isRecording(); },
        [&processor](std::shared_ptr<const Frame> frame) { processor.submit(std::move(frame)); },
        [&connected](Acquisition::Connection connection) {
            for (size_t i = 0; i < connection.cameraErrors.size(); ++i) {
                std::cout << "camera " << i + 1 << ": " << (connection.cameraErrors[i].empty() ? "open" : connection.cameraErrors[i]) << "\n";
            }
            std::cout << "lidar: " << (connection.lidarError.empty() ? "connected" : connection.lidarError) << "\n";
            connected = true;
        });
    acquisition.start();
    while (!connected && !interrupted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));   // Let the cameras settle their exposure

    std::string error;
    if (interrupted || !recorder.start(folder, error)) {
        std::cerr << error << "\n";
        acquisition.stop();
        return 1;
    }
    std::cout << "Recording to " << recorder.uri() << "\n";
    const auto start = std::chrono::steady_clock::now();
    while (!interrupted && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::lock_guard<std::mutex> lock(statsMutex);
        std::cout << "  " << recorder.rows() << " rows · processing delay avg " << (processed ? delaySumUs / processed / 1000 : 0)
                  << " ms, max " << delayMaxUs / 1000 << " ms · segmented " << segmented << " · queue " << recorder.pending()
                  << " · dropped " << recorder.dropped() << "\n";
    }

    // From the sensors down: every stage finishes what it has
    acquisition.stop();
    processor.stop();
    segmentation.reset();
    const uint64_t rows = recorder.rows();
    recorder.stop();
    std::cout << "Saved " << rows << " rows to " << recorder.uri() << " (" << recorder.dropped() << " messages dropped)\n";
    return 0;
}
