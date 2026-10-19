#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <vector>

#include <QLabel>
#include <QMainWindow>
#include <QStackedWidget>
#include <QString>
#include <QTimer>

#include <Gui/LidarWidget.h>
#include <Gui/CameraWidget.h>
#include <Gui/RecordWidget.h>
#include <Gui/ReplayWidget.h>
#include <Gui/RobotWidget.h>
#include <Lidar/Ouster.h>
#include <Camera/Daheng.h>
#include <Robot/ArtofRobot.h>
#include <Utils/PointCloudColor.h>
#include <Utils/SensorConfig.h>

class MainWindow : public QMainWindow {
public:
    explicit MainWindow(const SensorConfig& config = SensorConfig::load(), QWidget *parent = nullptr);
    ~MainWindow();

private:
    // A row being collected: started by a trigger (lidar scan), completed when every camera has a frame
    // after the trigger timestamp (or timed out). Each camera gets the frame closest to the trigger timestamp.
    struct PendingCapture {
        std::chrono::steady_clock::time_point deadline;
        uint64_t triggerTs = 0;     // Host time in microseconds since epoch
        uint64_t fileIndex = 0;
        bool record = false;
        OusterMetaData lidar;
        RobotMetaData robot;
        std::shared_ptr<std::vector<vaiop::QtPoint>> pointCloud;  // Coloured with the camera frames when the row completes
    };

    SensorConfig config;

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

    // Sensors
    std::unique_ptr<Ouster> lidarPtr;
    std::vector<std::shared_ptr<Daheng>> cameras;
    std::vector<uint64_t> lastCameraSequence;     // Frame used by the previous row, so a frame is never logged twice
    std::unique_ptr<ArtofRobot> robotPtr;

    QTimer acquisitionTimer;
    QTimer statusTimer;
    std::chrono::steady_clock::time_point lastSoftTrigger;
    std::deque<PendingCapture> pending;           // Oldest first; rows are logged in trigger order
    QString lidarOfflineReason;                   // Shown again when returning from a replay
    QString liveTriggerText;

    // While replaying, the sensors keep running in the background but are not shown
    bool replaying = false;
    RecordedFrame lastReplayFrame;                // Shown again when the colour mode changes

    // Point cloud colouring
    ColorMode colorMode = ColorMode::Distance;
    CameraColorizer colorizer;                    // Calibrations in the order of config.cameras
    std::vector<bool> rotationWarned;             // A rotation mismatch is reported once per camera

    void setupUI();
    void connectSensors();
    void onTick();
    bool waitForTrigger();
    void startCapture();
    bool captureComplete(const PendingCapture& capture) const;
    // showCloud: colour and show the point cloud (RGB mode); false when a newer scan is already waiting
    void finishCapture(const PendingCapture& capture, bool showCloud);
    void rotateCamera(size_t index);
    void startReplay();
    void stopReplay();
    void showRecordedFrame(const RecordedFrame& frame);
    void setColorMode(ColorMode mode);
    // Copy of the cloud coloured with the camera images (RGB, index = camera index, empty = no image)
    std::shared_ptr<std::vector<vaiop::QtPoint>> colorWithCameras(const std::vector<vaiop::QtPoint>& cloud,
                                                                  const std::vector<cv::Mat>& imagesRGB) const;
    void updateStatus();
    std::string recordPath(const QString& relative) const;
};
