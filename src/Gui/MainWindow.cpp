#include <Gui/MainWindow.h>
#include <Gui/Theme.h>
#include <Utils/AsyncWriter.h>

#include <QApplication>
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
    segmentation = std::make_unique<SegmentationWorker>(config.segmentation, colorizer, [this](SegmentationWorker::Result result) {
        QMetaObject::invokeMethod(this, [this, result]() { showSegmentation(result); }, Qt::QueuedConnection);
    });
    if (!config.segmentation.modelDir.empty() && !segmentation->isAvailable()) {
        qWarning().noquote() << QString::fromStdString(segmentation->error());
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

    recordWidget = new RecordWidget(central, static_cast<int>(config.cameras.size()));
    replayWidget = new ReplayWidget(central);
    headerStack = new QStackedWidget(central);
    headerStack->addWidget(recordWidget);
    headerStack->addWidget(replayWidget);
    connect(recordWidget, &RecordWidget::replayRequested, this, [this]() { startReplay(); });
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
    robotPtr = std::make_unique<ArtofRobot>(config.robotHost, config.robotPort);
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
    lidarWidget->setLegend(entries, QString("%1k of %2k points labelled · %3 ms")
        .arg(labelled / 1000).arg(result.cloud->size() / 1000).arg(result.ms, 0, 'f', 0));
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

std::string MainWindow::recordPath(const QString& relative) const
{
    return (recordWidget->folder() + "/" + relative).toStdString();
}

void MainWindow::startCapture()
{
    PendingCapture capture;
    capture.deadline = std::chrono::steady_clock::now() + MAX_CAMERA_WAIT;
    capture.record = recordWidget->isRecording();
    // Rows still waiting for their cameras will take the indices before this one
    const auto pendingRecordRows = std::count_if(pending.begin(), pending.end(), [](const PendingCapture& c) { return c.record; });
    capture.fileIndex = recordWidget->rowCount() + pendingRecordRows;

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
            if (capture.record) {
                const QString relativeFileName = QString("lidar/%1.ply").arg(capture.fileIndex, 6, 10, QChar('0'));
                if (lidarPtr->savePLY(recordPath(relativeFileName))) {
                    capture.lidar.pcPath = relativeFileName.toStdString();
                }
            }
        }
    } else {
        capture.lidar.ip = config.lidarIp;
    }
    capture.triggerTs = capture.lidar.ts != 0 ? capture.lidar.ts : hostTimeMicros();

    // Robot: latest state from the background poller
    if (robotPtr) {
        capture.robot = robotPtr->getLatest();
        if (capture.robot.valid) {
            robotWidget->setMetaData(capture.robot);
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
    const bool record = capture.record && recordWidget->isRecording();
    std::vector<CameraMetaData> cameraMetaData(cameras.size());
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
            const QString relativeFileName = QString("cam%1/%2.png").arg(i + 1).arg(capture.fileIndex, 6, 10, QChar('0'));
            if (Daheng::saveFrame(frame, recordPath(relativeFileName))) {
                camMetaData.imPath = relativeFileName.toStdString();
            }
        }
    }

    if (showCloud && capture.pointCloud) {
        if (colorMode == ColorMode::Rgb) {
            lidarWidget->setPointCloud(colorWithCameras(*capture.pointCloud, imagesRGB));
        } else if (colorMode == ColorMode::Segmentation) {
            // The worker keeps only the newest job, so it never falls behind the scans
            segmentation->submit({segmentationTag, capture.pointCloud, std::move(imagesRGB)});
        }
    }

    if (record) {
        recordWidget->logData(capture.robot, cameraMetaData, capture.lidar);
    }
}

void MainWindow::updateStatus()
{
    AsyncWriter& writer = AsyncWriter::instance();
    const size_t queued = writer.pending();
    const size_t dropped = writer.dropped();
    QString text = QString("Write queue: %1").arg(queued);
    if (dropped > 0) {
        text += QString(" · <span style='color:%1'>%2 files dropped (disk too slow)</span>").arg(Theme::error.name()).arg(dropped);
    }
    writerStatus->setText(text);
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

    // Make sure every queued image / point cloud is written before exiting
    AsyncWriter::instance().flush();
}
