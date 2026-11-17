#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <QLabel>
#include <QMainWindow>
#include <QStackedWidget>
#include <QString>
#include <QTimer>

#include <Gui/CameraWidget.h>
#include <Gui/LidarWidget.h>
#include <Gui/RecordWidget.h>
#include <Gui/ReplayWidget.h>
#include <Gui/RobotWidget.h>
#include <Utils/Acquisition.h>
#include <Utils/Frame.h>
#include <Utils/PointCloudColor.h>
#include <Utils/Processor.h>
#include <Utils/RosPublisher.h>
#include <Utils/Recorder.h>
#include <Utils/SegmentationWorker.h>
#include <Utils/SensorConfig.h>

// The window: builds the pipeline and shows its results. All work happens on other threads:
//   Acquisition (sensors -> rows) -> Processor (ground plane, occupancy map, colours; Recorder) -> this window
//                                              -> SegmentationWorker (classes)                     -> this window
// The window only draws, the newest result first; it never delays the acquisition or the recording.
class MainWindow : public QMainWindow {
public:
    explicit MainWindow(const SensorConfig& config = SensorConfig::load(), QWidget *parent = nullptr);
    ~MainWindow();

private:
    SensorConfig config;
    CameraColorizer colorizer;                    // Calibrations in the order of config.cameras

    // Widgets
    QStackedWidget* headerStack;    // Record controls when live, replay controls when replaying
    RecordWidget* recordWidget;
    ReplayWidget* replayWidget;
    std::vector<CameraWidget*> cameraWidgets;
    LidarWidget* lidarWidget;
    RobotWidget* robotWidget;
    QLabel* triggerStatus;
    QLabel* writerStatus;
    QLabel* configStatus;
    QTimer statusTimer;

    QString lidarOfflineReason;                   // Shown again when returning from a replay
    QString liveTriggerText;
    bool replaying = false;                       // The sensors keep running in the background, but are not shown
    RecordedFrame lastReplayFrame;                // Processed again when the colour mode changes
    ColorMode colorMode = ColorMode::Distance;
    uint64_t tag = 0;                             // Changes with the colour mode and live/replay: older results are dropped
    std::shared_ptr<const ProcessedFrame> shownFrame;

    // Newest results of the other threads, shown by the GUI thread (one queued call at a time)
    std::mutex latestMutex;
    std::shared_ptr<const ProcessedFrame> latestFrame;
    bool frameScheduled = false;
    std::optional<SegmentationWorker::Result> latestSegmentation;
    bool segmentationScheduled = false;

    void setupUI();
    void onConnected(const Acquisition::Connection& connection);
    void onProcessed(std::shared_ptr<const ProcessedFrame> frame);          // Processor thread
    void onSegmented(SegmentationWorker::Result result);                    // Segmentation thread
    void showFrame(const ProcessedFrame& processed);
    void showSegmentation(const SegmentationWorker::Result& result);
    void showSegmentationDevice(const SegmentationWorker::DeviceStatus& status);
    void startReplay();
    void stopReplay();
    void processReplayFrame(const RecordedFrame& recorded);
    void setColorMode(ColorMode mode);
    void startRecording();
    void stopRecording();
    // The camera tiles show the segmentation overlays instead of the camera frames
    bool showingOverlay() const;
    void updateStatus();

    // The pipeline, in the order it is built; the destructor stops it from the sensors down
    std::unique_ptr<Recorder> recorder;
    std::unique_ptr<SegmentationWorker> segmentation;
    std::unique_ptr<RosPublisher> rosPublisher;    // Live ROS topics; before the processor that uses it
    std::unique_ptr<Processor> processor;
    std::unique_ptr<Acquisition> acquisition;
};
