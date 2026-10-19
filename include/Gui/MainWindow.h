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

    void setupUI();
    void connectSensors();
    void onTick();
    bool waitForTrigger();
    void startCapture();
    bool captureComplete(const PendingCapture& capture) const;
    void finishCapture(const PendingCapture& capture);
    void rotateCamera(size_t index);
    void startReplay();
    void stopReplay();
    void showRecordedFrame(const RecordedFrame& frame);
    void updateStatus();
    std::string recordPath(const QString& relative) const;
};
