#include <Gui/MainWindow.h>
#include <Gui/Theme.h>

#include <QApplication>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QDebug>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QSplitter>
#include <QStatusBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cstdlib>

namespace {
    uint64_t hostTimeMicros()
    {
        return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }

    // The configured Redis keys that the application does not know: extra CSV columns
    std::vector<std::string> extraRobotKeys(const std::vector<RedisKey>& keys)
    {
        const std::vector<RedisKey> known = SensorConfig::defaultRobotKeys();
        std::vector<std::string> extra;
        for (const RedisKey& key : keys) {
            if (std::none_of(known.begin(), known.end(), [&key](const RedisKey& k) { return k.key == key.key; })) {
                extra.push_back(key.key);
            }
        }
        return extra;
    }

    // /record/row: everything of one CSV row, so scripts/bag_to_folder.py can write the folder format from the bag.
    // cloudStamp / imageStamps: stamps of the messages in the bag, 0 when not written.
    std::string rowJson(uint64_t row, const QString& logTime, uint64_t triggerTs, const OusterMetaData& lidar, uint64_t cloudStamp,
                        const std::vector<CameraMetaData>& cameras, const std::vector<uint64_t>& imageStamps,
                        const RobotMetaData& robot)
    {
        auto stamp = [](uint64_t us) { return us ? QJsonValue(static_cast<double>(us)) : QJsonValue(); };
        QJsonObject lidarJson{{"valid", lidar.valid}, {"ip", QString::fromStdString(lidar.ip)}, {"ts", static_cast<double>(lidar.ts)},
                              {"frame_id", static_cast<double>(lidar.frameId)}, {"fps", lidar.fps}, {"cloud_stamp", stamp(cloudStamp)}};
        QJsonArray camerasJson;
        for (size_t i = 0; i < cameras.size(); ++i) {
            const CameraMetaData& cam = cameras[i];
            camerasJson.append(QJsonObject{{"valid", cam.valid}, {"ip", QString::fromStdString(cam.ip)}, {"ts", static_cast<double>(cam.ts)},
                                           {"frame_id", static_cast<double>(cam.frameId)}, {"fps", cam.fps},
                                           {"image_stamp", stamp(i < imageStamps.size() ? imageStamps[i] : 0)}});
        }
        QJsonObject redis;
        for (const auto& [key, raw] : robot.redis) {
            redis.insert(QString::fromStdString(key), raw ? QJsonValue(QString::fromStdString(*raw)) : QJsonValue());
        }
        const QJsonObject robotJson{{"valid", robot.valid}, {"x", robot.x}, {"y", robot.y}, {"z", robot.z}, {"roll", robot.roll},
                                    {"pitch", robot.pitch}, {"yaw", robot.yaw}, {"lat", robot.lat}, {"lon", robot.lon},
                                    {"gps_fix", robot.gpsFix}, {"gps_lng_speed", robot.gpsLngSpeed},
                                    {"odom_lng_speed", robot.odomLngSpeed}, {"odom_ang_speed", robot.odomAngSpeed}, {"redis", redis}};
        const QJsonObject json{{"row", static_cast<double>(row)}, {"log_time", logTime}, {"trigger_ts", static_cast<double>(triggerTs)},
                               {"lidar", lidarJson}, {"cameras", camerasJson}, {"robot", robotJson}};
        return QJsonDocument(json).toJson(QJsonDocument::Compact).toStdString();
    }
}

namespace {
    // Maximum time to wait for a camera frame after the trigger (a frame normally arrives within one camera period)
    constexpr auto MAX_CAMERA_WAIT = std::chrono::milliseconds(150);
    // A camera frame further than this from the trigger timestamp is not logged (half the 10 Hz period)
    constexpr uint64_t MAX_SYNC_OFFSET_US = 50000;
    // Trigger period when no lidar is connected (preview + recording continue at 10 Hz)
    constexpr auto SOFT_TRIGGER_PERIOD = std::chrono::milliseconds(100);
    // Acquisition poll period
    constexpr int TICK_MS = 5;
}


MainWindow::MainWindow(const SensorConfig& config, QWidget *parent)
    : QMainWindow(parent),
      config(config)
{
    // Calibrations for colouring the point cloud with the cameras
    std::vector<CameraCalib> calibs;
    for (const CameraInfo& camInfo : config.cameras) {
        calibs.push_back(loadCameraCalib(camInfo.calibFileName, camInfo.name));
    }
    colorizer = CameraColorizer(std::move(calibs));
    rotationWarned.assign(config.cameras.size(), false);

    // Segmentation model on a background thread; its results are shown on the GUI thread
    segmentation = std::make_unique<SegmentationWorker>(config.segmentation, colorizer,
        [this](SegmentationWorker::Result result) {
            // Recorded on the worker thread: also the results that are not shown any more
            if (result.record) {
                bag->writeSegmentedScan(result.stamp, result.cloud, result.labels, result.plane, this->config.occupancyMap);
            }
            QMetaObject::invokeMethod(this, [this, result]() { showSegmentation(result); }, Qt::QueuedConnection);
        },
        [this](SegmentationWorker::DeviceStatus status) {
            QMetaObject::invokeMethod(this, [this, status]() { showSegmentationDevice(status); }, Qt::QueuedConnection);
        });
    if (!config.segmentation.modelDir.empty() && !segmentation->isAvailable()) {
        qWarning().noquote() << QString::fromStdString(segmentation->error());
    } else if (segmentation->isAvailable()) {
        const SegmentationWorker::DeviceStatus status = segmentation->deviceStatus();
        qInfo().noquote() << "Segmentation runs on the" << Segmenter::deviceName(status.device);
        if (!status.gpuError.empty() && config.segmentation.device != "cpu") {
            qWarning().noquote() << "Segmentation not on the GPU:" << QString::fromStdString(status.gpuError);
        }
    }

    colorMode = colorModeFromString(config.pointCloudColor);
    if (colorMode == ColorMode::Segmentation && !segmentation->isAvailable()) {
        qWarning() << "No segmentation model available, colouring the point cloud by distance";
        colorMode = ColorMode::Distance;
    }

    setupUI();

    // Connect the sensors after the window is shown, so the user sees the progress
    QTimer::singleShot(0, this, [this]() { connectSensors(); });
}

void MainWindow::setupUI()
{
    setWindowTitle("Treebot sensor");
    setMinimumSize(1100, 700);

    QWidget* central = new QWidget(this);
    QVBoxLayout* mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(12, 12, 12, 8);
    mainLayout->setSpacing(10);

    // Header: title, recording controls, robot state
    auto* titleBox = new QVBoxLayout();
    titleBox->setSpacing(0);
    auto* appTitle = new QLabel("Treebot sensor", central);
    appTitle->setObjectName("appTitle");
    auto* appSubtitle = new QLabel(QString("VAIOP · %1 cameras · Ouster lidar").arg(config.cameras.size()), central);
    appSubtitle->setObjectName("appSubtitle");
    titleBox->addStretch();
    titleBox->addWidget(appTitle);
    titleBox->addWidget(appSubtitle);
    titleBox->addStretch();

    recordWidget = new RecordWidget(central);
    replayWidget = new ReplayWidget(central);
    headerStack = new QStackedWidget(central);
    headerStack->addWidget(recordWidget);
    headerStack->addWidget(replayWidget);
    connect(recordWidget, &RecordWidget::replayRequested, this, [this]() { startReplay(); });
    recordWidget->setRecordingAvailable(BagWriter::isAvailable(), "Built without ROS 2 (/opt/ros/jazzy): no recording");
    connect(recordWidget, &RecordWidget::recordingChanged, this, [this](bool recording) {
        if (recording) {
            startBag();
        } else {
            stopBag();
        }
    });
    connect(replayWidget, &ReplayWidget::liveRequested, this, [this]() { stopReplay(); });
    connect(replayWidget, &ReplayWidget::frameLoaded, this, [this](const RecordedFrame& frame) { showRecordedFrame(frame); });
    robotWidget = new RobotWidget(central);
    robotWidget->setHost(QString("%1:%2").arg(QString::fromStdString(config.robotHost)).arg(config.robotPort));

    auto* headerLayout = new QHBoxLayout();
    headerLayout->setSpacing(10);
    headerLayout->addLayout(titleBox);
    headerLayout->addSpacing(12);
    headerLayout->addWidget(headerStack, 3);
    headerLayout->addWidget(robotWidget, 2);

    // Camera tiles
    auto* cameraRow = new QWidget(central);
    auto* cameraLayout = new QHBoxLayout(cameraRow);
    cameraLayout->setContentsMargins(0, 0, 0, 0);
    cameraLayout->setSpacing(10);
    for (const CameraInfo& camInfo : config.cameras) {
        auto* cam = new CameraWidget(QString::fromStdString(camInfo.name), cameraRow);
        cameraWidgets.push_back(cam);
        cameraLayout->addWidget(cam);
    }

    // Lidar
    lidarWidget = new LidarWidget(central);
    lidarWidget->setSegmentationAvailable(segmentation->isAvailable(), QString::fromStdString(segmentation->error()));
    lidarWidget->setColorMode(colorMode);
    if (colorMode == ColorMode::Segmentation) {
        lidarWidget->setLegend({}, "Segmenting…");
    }
    lidarWidget->onColorModeChanged([this](ColorMode mode) { setColorMode(mode); });
    lidarWidget->setOccupancyHeightRange(config.occupancyMap.minHeight, config.occupancyMap.maxHeight);
    const SegmentationWorker::DeviceStatus deviceStatus = segmentation->deviceStatus();
    lidarWidget->setSegmentationDevice(deviceStatus.device, QString::fromStdString(deviceStatus.gpuError));
    lidarWidget->onSegmentationDeviceChanged([this](Segmenter::Device device) {
        segmentation->setDevice(device);
        statusBar()->showMessage(QString("Loading the segmentation model on the %1…").arg(Segmenter::deviceName(device)));
    });
    lidarWidget->onImageOverlayChanged([this](bool) {
        // Live: the next scan shows the change; a replay shows the current row again
        if (replaying && lastReplayFrame.pointCloud) {
            showRecordedFrame(lastReplayFrame);
        }
    });

    // Cameras and lidar share the space; the user can drag the divider
    auto* splitter = new QSplitter(Qt::Vertical, central);
    splitter->addWidget(cameraRow);
    splitter->addWidget(lidarWidget);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    splitter->setHandleWidth(8);
    splitter->setChildrenCollapsible(false);

    mainLayout->addLayout(headerLayout);
    mainLayout->addWidget(splitter, 1);
    setCentralWidget(central);

    // Status bar
    triggerStatus = new QLabel(this);
    writerStatus = new QLabel(this);
    configStatus = new QLabel(QString("%1 · calibration: %2/%3 cameras · segmentation: %4")
        .arg(config.sourceFile.empty() ? QString("config: built-in defaults") : QString("config: %1").arg(QString::fromStdString(config.sourceFile)))
        .arg(colorizer.validCount())
        .arg(config.cameras.size())
        .arg(segmentation->isAvailable() ? QFileInfo(QString::fromStdString(config.segmentation.modelDir)).fileName() : QString("off")), this);
    configStatus->setToolTip(QString::fromStdString(config.calibFileName));
    statusBar()->addWidget(triggerStatus);
    statusBar()->addWidget(writerStatus);
    statusBar()->addPermanentWidget(configStatus);
    statusBar()->setSizeGripEnabled(false);
    triggerStatus->setText("Connecting to the sensors…");
}

void MainWindow::connectSensors()
{
    // Robot (polled in the background, never blocks the GUI)
    robotPtr = std::make_unique<ArtofRobot>(config.robotHost, config.robotPort, config.robotKeys);
    robotPtr->startPolling();

    // Cameras
    for (size_t i = 0; i < config.cameras.size(); ++i) {
        const CameraInfo& camInfo = config.cameras[i];
        QApplication::processEvents();
        auto camPtr = std::make_shared<Daheng>(camInfo.name, camInfo.ip, camInfo.rFlag, camInfo.calibFileName);
        if (camPtr->isOpen()) {
            camPtr->start();
            cameraWidgets[i]->setInfo(camInfo.ip, 0.0, 0, Theme::Status::Waiting);
            cameraWidgets[i]->setRotation(camPtr->getRotation());
            cameraWidgets[i]->onRotateClicked([this, i]() { rotateCamera(i); });
        } else {
            cameraWidgets[i]->setOffline(camInfo.ip, QString("Cannot open camera at %1").arg(QString::fromStdString(camInfo.ip)));
        }
        cameras.push_back(camPtr);
    }
    lastCameraSequence.assign(cameras.size(), 0);

    // Lidar
    QApplication::processEvents();
    try {
        lidarPtr = std::make_unique<Ouster>(config.lidarIp);
        lidarWidget->setInfo(config.lidarIp, 0.0, 0, 0, Theme::Status::Waiting);
    } catch (const std::exception& e) {
        qWarning() << "Lidar not available:" << e.what();
        lidarPtr.reset();
        lidarOfflineReason = QString("Lidar not reachable at %1\n%2\n\nCameras are triggered at 10 Hz instead.")
            .arg(QString::fromStdString(config.lidarIp), e.what());
        if (!replaying) {
            lidarWidget->setOffline(config.lidarIp, lidarOfflineReason);
        }
    }

    // Reset them back-to-back to sync their timers
    if (lidarPtr) {
        lidarPtr->resetTimestamp();
    }
    for (auto& camPtr : cameras) {
        camPtr->resetTimestamp();
    }

    liveTriggerText = lidarPtr ? "Trigger: lidar scans" : "Trigger: 10 Hz timer (no lidar)";
    if (!replaying) {
        triggerStatus->setText(liveTriggerText);
    }

    connect(&acquisitionTimer, &QTimer::timeout, this, [this]() { onTick(); });
    acquisitionTimer.start(TICK_MS);
    connect(&statusTimer, &QTimer::timeout, this, [this]() { updateStatus(); });
    statusTimer.start(250);
}

void MainWindow::onTick()
{
    // Keep reading the lidar while replaying, so live starts from a fresh scan instead of a backlog
    if (replaying) {
        waitForTrigger();
        return;
    }

    // A new trigger starts a row, also while earlier rows are still waiting for their camera frames
    if (waitForTrigger()) {
        startCapture();
    }

    // Finish the rows in trigger order, without blocking the event loop.
    // Only the newest scan is coloured: an older one would be replaced right away, and colouring every
    // row of a backlog keeps the event loop busy.
    while (!pending.empty() && captureComplete(pending.front())) {
        const bool newerScan = std::any_of(pending.begin() + 1, pending.end(), [](const PendingCapture& c) { return c.pointCloud != nullptr; });
        finishCapture(pending.front(), !newerScan);
        pending.pop_front();
    }
}

void MainWindow::rotateCamera(size_t index)
{
    auto& camPtr = cameras[index];
    camPtr->setRotation(camPtr->getRotation() + 90);
    cameraWidgets[index]->setRotation(camPtr->getRotation());
}

void MainWindow::startReplay()
{
    const QString startFolder = replayWidget->recording().size() > 0 ? replayWidget->recording().folder() : recordWidget->folder();
    if (!replayWidget->chooseRecording(startFolder)) {
        return;
    }

    replaying = true;
    segmentationTag++;
    pending.clear();    // Rows waiting for camera frames are only previews: replay never starts while recording
    headerStack->setCurrentWidget(replayWidget);
    for (CameraWidget* cameraWidget : cameraWidgets) {
        cameraWidget->setRotateEnabled(false);
    }
    triggerStatus->setText(QString("Replay: %1").arg(replayWidget->recording().logFile()));
}

void MainWindow::stopReplay()
{
    replaying = false;
    segmentationTag++;
    headerStack->setCurrentWidget(recordWidget);

    // Back to the live state of each sensor; the next trigger shows fresh data
    for (size_t i = 0; i < cameraWidgets.size(); ++i) {
        cameraWidgets[i]->setRotateEnabled(true);
        if (i >= cameras.size()) {
            cameraWidgets[i]->showMessage("Connecting…");
        } else if (!cameras[i]->isOpen()) {
            cameraWidgets[i]->setOffline(cameras[i]->getIp(), QString("Cannot open camera at %1").arg(QString::fromStdString(cameras[i]->getIp())));
        } else {
            cameraWidgets[i]->showMessage("Waiting for a frame…");
            cameraWidgets[i]->setInfo(cameras[i]->getIp(), cameras[i]->getFps(), cameras[i]->getFrameId(), Theme::Status::Waiting);
        }
    }
    if (lidarPtr) {
        lidarWidget->showMessage("Waiting for a scan…");
        lidarWidget->setInfo(lidarPtr->getIp(), lidarPtr->getFps(), lidarPtr->getFrameId(), 0, Theme::Status::Waiting);
    } else if (!lidarOfflineReason.isEmpty()) {
        lidarWidget->setOffline(config.lidarIp, lidarOfflineReason);
    } else {
        lidarWidget->showMessage("Connecting…");
    }
    if (robotPtr == nullptr) {
        robotWidget->setOffline("Connecting…");
    }
    triggerStatus->setText(liveTriggerText.isEmpty() ? "Connecting to the sensors…" : liveTriggerText);
}

void MainWindow::setColorMode(ColorMode mode)
{
    colorMode = mode;
    segmentationTag++;
    if (mode == ColorMode::Segmentation) {
        lidarWidget->setLegend({}, "Segmenting…");
    } else {
        lidarWidget->clearLegend();
    }
    if (mode != ColorMode::Distance && colorizer.validCount() == 0) {
        statusBar()->showMessage(QString("No camera calibration in %1: the point cloud stays grey")
            .arg(QString::fromStdString(config.calibFileName)), 8000);
    }
    // Live rows pick up the new mode with the next scan; a replay shows the current row again
    if (replaying && lastReplayFrame.pointCloud) {
        showRecordedFrame(lastReplayFrame);
    }
}

std::shared_ptr<std::vector<vaiop::QtPoint>> MainWindow::colorWithCameras(const std::vector<vaiop::QtPoint>& cloud,
                                                                          const std::vector<cv::Mat>& imagesRGB) const
{
    auto colored = std::make_shared<std::vector<vaiop::QtPoint>>(cloud);
    colorizer.colorize(*colored, imagesRGB);
    return colored;
}

bool MainWindow::showingOverlay() const
{
    return colorMode == ColorMode::Segmentation && segmentation->isAvailable() && lidarWidget->imageOverlay();
}

void MainWindow::showSegmentation(const SegmentationWorker::Result& result)
{
    // Outdated: another colour mode, or switched between live and replay since the job was submitted
    if (result.tag != segmentationTag || colorMode != ColorMode::Segmentation) {
        return;
    }
    lidarWidget->setPointCloud(result.cloud, result.labels, segmentation->palette());
    if (showingOverlay()) {
        for (size_t i = 0; i < result.overlays.size() && i < cameraWidgets.size(); ++i) {
            const cv::Mat& overlay = result.overlays[i];
            if (!overlay.empty()) {
                cameraWidgets[i]->setImage(QImage(overlay.data, overlay.cols, overlay.rows, static_cast<qsizetype>(overlay.step),
                                                  QImage::Format_RGB888).copy());
            }
        }
    }

    // Legend: the classes with at least 1% of the labelled points, largest first
    constexpr double MIN_SHARE = 0.01;
    constexpr size_t MAX_ENTRIES = 10;
    size_t labelled = 0;
    for (const size_t n : result.classPoints) {
        labelled += n;
    }
    std::vector<size_t> order(result.classPoints.size());
    for (size_t k = 0; k < order.size(); ++k) {
        order[k] = k;
    }
    std::sort(order.begin(), order.end(), [&result](size_t a, size_t b) { return result.classPoints[a] > result.classPoints[b]; });

    std::vector<LidarWidget::LegendEntry> entries;
    const auto& names = segmentation->classNames();
    const auto& palette = segmentation->palette();
    for (const size_t k : order) {
        const double share = labelled > 0 ? static_cast<double>(result.classPoints[k]) / labelled : 0.0;
        if (share < MIN_SHARE || entries.size() >= MAX_ENTRIES) {
            break;
        }
        entries.push_back({QString::fromStdString(names[k]), QColor(palette[k][0], palette[k][1], palette[k][2]), share});
    }
    lidarWidget->setLegend(entries, QString("%1k of %2k points labelled · %3 ms (%4)")
        .arg(labelled / 1000).arg(result.cloud->size() / 1000).arg(result.ms, 0, 'f', 0).arg(Segmenter::deviceName(result.device)));
}

void MainWindow::showSegmentationDevice(const SegmentationWorker::DeviceStatus& status)
{
    const QString gpuError = QString::fromStdString(status.gpuError);
    lidarWidget->setSegmentationDevice(status.device, gpuError);
    if (status.device == Segmenter::Device::Gpu || gpuError.isEmpty()) {
        statusBar()->showMessage(QString("Segmentation runs on the %1").arg(Segmenter::deviceName(status.device)), 5000);
    } else {
        // The GPU was asked for but is not usable: the model stays on the CPU
        qWarning().noquote() << "Segmentation not on the GPU:" << gpuError;
        statusBar()->showMessage("GPU not available, segmentation runs on the CPU: " + gpuError, 15000);
    }
}

void MainWindow::showRecordedFrame(const RecordedFrame& frame)
{
    if (!replaying) {
        return;
    }
    lastReplayFrame = frame;

    if (frame.pointCloud) {
        if (colorMode == ColorMode::Rgb || colorMode == ColorMode::Segmentation) {
            std::vector<cv::Mat> imagesRGB(frame.images.size());
            for (size_t i = 0; i < frame.images.size(); ++i) {
                if (!frame.images[i].isNull()) {
                    const QImage rgb = frame.images[i].convertToFormat(QImage::Format_RGB888);
                    imagesRGB[i] = cv::Mat(rgb.height(), rgb.width(), CV_8UC3, const_cast<uchar*>(rgb.constBits()),
                                           static_cast<size_t>(rgb.bytesPerLine())).clone();
                }
            }
            if (colorMode == ColorMode::Rgb) {
                lidarWidget->setPointCloud(colorWithCameras(*frame.pointCloud, imagesRGB));
            } else {
                // Shown when the worker is done (showSegmentation)
                segmentation->submit({segmentationTag, frame.pointCloud, std::move(imagesRGB)});
            }
        } else {
            lidarWidget->setPointCloud(frame.pointCloud);
        }
        lidarWidget->setInfo(frame.lidar.ip, frame.lidar.fps, frame.lidar.frameId, frame.pointCloud->size(), Theme::Status::Ok);
    } else {
        lidarWidget->showMessage(frame.lidar.pcPath.empty() ? "No point cloud in this row" : QString("Cannot read %1").arg(QString::fromStdString(frame.lidar.pcPath)));
        lidarWidget->setInfo(frame.lidar.ip, frame.lidar.fps, frame.lidar.frameId, 0, Theme::Status::Waiting);
    }

    for (size_t i = 0; i < cameraWidgets.size(); ++i) {
        if (i >= frame.cameras.size()) {
            cameraWidgets[i]->showMessage("Not in this recording");
            cameraWidgets[i]->setInfo("", 0.0, 0, Theme::Status::Offline);
            continue;
        }
        const CameraMetaData& camMetaData = frame.cameras[i];
        if (!frame.images[i].isNull()) {
            if (!(showingOverlay() && frame.pointCloud)) {
                cameraWidgets[i]->setImage(frame.images[i]);
            }
            cameraWidgets[i]->setInfo(camMetaData.ip, camMetaData.fps, camMetaData.frameId, Theme::Status::Ok);
        } else {
            cameraWidgets[i]->showMessage(camMetaData.imPath.empty() ? "No image in this row" : QString("Cannot read %1").arg(QString::fromStdString(camMetaData.imPath)));
            cameraWidgets[i]->setInfo(camMetaData.ip, camMetaData.fps, camMetaData.frameId, Theme::Status::Waiting);
        }
    }

    if (frame.robot.valid) {
        robotWidget->setMetaData(frame.robot);
    } else {
        robotWidget->setOffline("No robot data in this row");
    }
}

bool MainWindow::waitForTrigger()
{
    // All image collection is initiated by the lidar scan
    if (lidarPtr) {
        return lidarPtr->scan();
    }

    // Without lidar: software trigger
    const auto now = std::chrono::steady_clock::now();
    if (now - lastSoftTrigger >= SOFT_TRIGGER_PERIOD) {
        lastSoftTrigger = now;
        return true;
    }
    return false;
}

void MainWindow::startCapture()
{
    PendingCapture capture;
    capture.deadline = std::chrono::steady_clock::now() + MAX_CAMERA_WAIT;
    capture.record = recordWidget->isRecording() && bag->isOpen();
    // Rows still waiting for their cameras will take the indices before this one
    const auto pendingRecordRows = std::count_if(pending.begin(), pending.end(), [](const PendingCapture& c) { return c.record; });
    capture.fileIndex = recordedRows + pendingRecordRows;

    // Lidar
    if (lidarPtr) {
        auto pc = lidarPtr->getPointCloud();
        lidarPtr->getMetaData(capture.lidar);
        if (pc != nullptr) {
            // RGB and segmentation: shown when the camera frames of the row are in (finishCapture)
            capture.pointCloud = pc;
            if (colorMode == ColorMode::Distance) {
                lidarWidget->setPointCloud(pc);
            }
            lidarWidget->setInfo(lidarPtr->getIp(), lidarPtr->getFps(), lidarPtr->getFrameId(), pc->size(), Theme::Status::Ok);
        }
    } else {
        capture.lidar.ip = config.lidarIp;
    }
    capture.triggerTs = capture.lidar.ts != 0 ? capture.lidar.ts : hostTimeMicros();

    // Rosbag: the scan, its ground plane and the occupancy map. In segmentation mode the GridMap (with the class
    // layer) is written when the worker has segmented the scan.
    if (capture.record && capture.pointCloud) {
        capture.cloudStamp = capture.triggerTs;
        recordPlane = fitGroundPlane(*capture.pointCloud, recordPlane.valid ? &recordPlane : nullptr);
        capture.plane = recordPlane;
        const bool segmenting = colorMode == ColorMode::Segmentation && segmentation->isAvailable();
        bag->writeScan(capture.triggerTs, capture.pointCloud, recordPlane, config.occupancyMap, !segmenting);
    }

    // Robot: latest state from the background poller
    if (robotPtr) {
        capture.robot = robotPtr->getLatest();
        if (capture.robot.valid) {
            robotWidget->setMetaData(capture.robot);
            if (capture.record) {
                bag->writeRobot(capture.triggerTs, capture.robot);
            }
        } else {
            const std::string error = robotPtr->getLastError();
            robotWidget->setOffline(error.empty() ? "Connecting…" : QString("Offline: %1").arg(QString::fromStdString(error)));
        }
    }

    pending.push_back(std::move(capture));
}

bool MainWindow::captureComplete(const PendingCapture& capture) const
{
    if (std::chrono::steady_clock::now() >= capture.deadline) {
        return true;
    }

    // Done when every streaming camera has a frame at or after the trigger: later frames can only be further away
    return std::all_of(cameras.begin(), cameras.end(), [&capture](const std::shared_ptr<Daheng>& camPtr) {
        return !camPtr->isAcquiring() || (camPtr->getFrameSequence() > 0 && camPtr->getTimeStamp() >= capture.triggerTs);
    });
}

void MainWindow::finishCapture(const PendingCapture& capture, bool showCloud)
{
    const bool record = capture.record && recordWidget->isRecording() && bag->isOpen();
    std::vector<CameraMetaData> cameraMetaData(cameras.size());
    std::vector<uint64_t> imageStamps(cameras.size(), 0);
    std::vector<cv::Mat> imagesRGB(cameras.size());

    for (size_t i = 0; i < cameras.size(); i++) {
        auto& camPtr = cameras[i];
        CameraMetaData& camMetaData = cameraMetaData[i];
        camMetaData.ip = camPtr->getIp();

        // The frame closest to the trigger, if it is close enough and not logged in the previous row
        Daheng::Frame frame;
        const bool found = camPtr->isAcquiring() && camPtr->getFrameNear(capture.triggerTs, frame);
        const uint64_t offset = found ? static_cast<uint64_t>(std::llabs(static_cast<int64_t>(frame.ts - capture.triggerTs))) : 0;

        // Colour the point cloud with every frame close to the trigger (also one already logged in the previous row),
        // as long as the camera is rotated as during the calibration
        if (found && offset <= MAX_SYNC_OFFSET_US && i < colorizer.calibs().size()) {
            const int calibRotation = colorizer.calibs()[i].rotationDeg;
            if (calibRotation < 0 || calibRotation == camPtr->getRotation()) {
                imagesRGB[i] = frame.image;
            } else if (!rotationWarned[i]) {
                rotationWarned[i] = true;
                qWarning().noquote() << QString("%1 is rotated %2°, but calibrated at %3°: not used to colour the point cloud")
                    .arg(QString::fromStdString(camPtr->getName())).arg(camPtr->getRotation()).arg(calibRotation);
            }
        }
        if (!found || offset > MAX_SYNC_OFFSET_US || frame.sequence <= lastCameraSequence[i]) {
            if (camPtr->isOpen()) {
                cameraWidgets[i]->setInfo(camPtr->getIp(), camPtr->getFps(), camPtr->getFrameId(), Theme::Status::Waiting);
            }
            continue;
        }
        lastCameraSequence[i] = frame.sequence;

        // With the image overlay the tile shows the segmented frame when the worker is done (showSegmentation)
        if (!(showingOverlay() && capture.pointCloud)) {
            const QImage view(frame.image.data, frame.image.cols, frame.image.rows, static_cast<qsizetype>(frame.image.step), QImage::Format_RGB888);
            cameraWidgets[i]->setImage(view.copy());
        }
        cameraWidgets[i]->setInfo(camPtr->getIp(), camPtr->getFps(), frame.frameId, Theme::Status::Ok);

        camPtr->getMetaData(frame, camMetaData);
        camMetaData.valid = true;
        if (record) {
            bag->writeImage(i, frame.ts, frame.image);
            imageStamps[i] = frame.ts;
        }
    }

    if (showCloud && capture.pointCloud) {
        if (colorMode == ColorMode::Rgb) {
            lidarWidget->setPointCloud(colorWithCameras(*capture.pointCloud, imagesRGB));
        } else if (colorMode == ColorMode::Segmentation) {
            // The worker keeps only the newest job, so it never falls behind the scans
            segmentation->submit({segmentationTag, capture.pointCloud, std::move(imagesRGB), capture.triggerTs, capture.plane, record});
        }
    }

    if (record) {
        // The row of the CSV log (scripts/bag_to_folder.py); "timestamp" = when the row is complete
        const QString logTime = QDateTime::currentDateTime().toString("yyyy-MM-ddTHH:mm:ss.zzz");
        bag->writeRow(capture.triggerTs, rowJson(capture.fileIndex, logTime, capture.triggerTs, capture.lidar, capture.cloudStamp,
                                                 cameraMetaData, imageStamps, capture.robot));
        recordWidget->setRowCount(++recordedRows);
    }
}

void MainWindow::updateStatus()
{
    QString text;
    if (bag->isOpen()) {
        text = QString("Rosbag queue: %1").arg(bag->pending());
        if (const size_t dropped = bag->dropped(); dropped > 0) {
            text += QString(" · <span style='color:%1'>%2 messages dropped (disk too slow)</span>").arg(Theme::error.name()).arg(dropped);
        }
    }
    writerStatus->setText(text);
}

void MainWindow::startBag()
{
    const QString uri = recordWidget->folder() + "/rosbag_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    std::string error;
    if (!bag->open(uri.toStdString(), error)) {
        qWarning().noquote() << QString::fromStdString(error);
        QMessageBox::warning(this, "Recording", QString::fromStdString(error));
        recordWidget->stop();
        return;
    }
    recordedRows = 0;
    const uint64_t now = hostTimeMicros();
    // CSV name and header for scripts/bag_to_folder.py: those of this folder's log, or a new name for a bag-only recording
    QJsonArray header;
    for (const std::string& column : folderLogHeader(static_cast<int>(config.cameras.size()), extraRobotKeys(config.robotKeys))) {
        header.append(QString::fromStdString(column));
    }
    const QString csvName = "sensor_log_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".csv";
    QJsonArray cameraNames;
    for (const CameraInfo& camera : config.cameras) {
        cameraNames.append(QString::fromStdString(camera.name));
    }
    bag->writeRecordInfo(now, QJsonDocument(QJsonObject{{"format_version", 1}, {"csv_name", csvName}, {"csv_header", header},
                                                         {"cameras", cameraNames}}).toJson(QJsonDocument::Compact).toStdString());
    bag->writeCalibration(now, colorizer.calibs());
    if (segmentation->isAvailable()) {
        bag->writeClassNames(now, segmentation->classNames());
    }
    recordPlane = GroundPlane{};
    statusBar()->showMessage("Recording to the rosbag " + uri, 5000);
}

void MainWindow::stopBag()
{
    if (!bag->isOpen()) {
        return;
    }
    const QString uri = QString::fromStdString(bag->uri());
    const size_t dropped = bag->dropped();
    bag->close();   // Writes what is still queued
    statusBar()->showMessage(QString("Rosbag saved: %1%2").arg(uri)
        .arg(dropped > 0 ? QString(" (%1 messages dropped)").arg(dropped) : QString()), 10000);
}

MainWindow::~MainWindow()
{
    acquisitionTimer.stop();
    statusTimer.stop();

    if (robotPtr) {
        robotPtr->stopPolling();
    }

    // Stop all camera acquisition
    for (auto& camPtr : cameras) {
        camPtr->stop();
    }

    // The bag (when recording) writes what is still queued when it is destroyed
}
