#include <Gui/MainWindow.h>
#include <Gui/Theme.h>

#include <QApplication>
#include <QDebug>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>
#include <QVBoxLayout>

#include <algorithm>

namespace {
    QImage toQImage(const cv::Mat& imageRGB)
    {
        return QImage(imageRGB.data, imageRGB.cols, imageRGB.rows, static_cast<qsizetype>(imageRGB.step), QImage::Format_RGB888).copy();
    }

    // A row of a recording folder as a frame of the pipeline (not live, not recorded)
    std::shared_ptr<const Frame> toFrame(const RecordedFrame& recorded)
    {
        auto frame = std::make_shared<Frame>();
        frame->live = false;
        frame->lidar = recorded.lidar;
        frame->triggerTs = recorded.lidar.ts;
        frame->cloud = recorded.pointCloud;
        frame->robot = recorded.robot;
        frame->cameras.resize(recorded.cameras.size());
        for (size_t i = 0; i < recorded.cameras.size(); ++i) {
            CameraSample& sample = frame->cameras[i];
            sample.meta = recorded.cameras[i];
            sample.ip = sample.meta.ip;
            sample.open = true;
            sample.fps = sample.meta.fps;
            sample.frameId = sample.meta.frameId;
            if (i < recorded.images.size() && !recorded.images[i].isNull()) {
                const QImage rgb = recorded.images[i].convertToFormat(QImage::Format_RGB888);
                sample.image = cv::Mat(rgb.height(), rgb.width(), CV_8UC3, const_cast<uchar*>(rgb.constBits()),
                                       static_cast<size_t>(rgb.bytesPerLine())).clone();
                sample.colourImage = sample.image;
                sample.newFrame = true;
            }
        }
        return frame;
    }
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
    colorizer = CameraColorizer(calibs);

    // The pipeline, from the recording up to the sensors
    recorder = std::make_unique<Recorder>(config, calibs);
    segmentation = std::make_unique<SegmentationWorker>(config.segmentation, config.occupancyMap, colorizer,
        [this](SegmentationWorker::Result result) { onSegmented(std::move(result)); },
        [this](SegmentationWorker::DeviceStatus status) {
            QMetaObject::invokeMethod(this, [this, status]() { showSegmentationDevice(status); }, Qt::QueuedConnection);
        });
    if (!config.segmentation.modelDir.empty() && !segmentation->isAvailable()) {
        qWarning().noquote() << QString::fromStdString(segmentation->error());
    } else if (segmentation->isAvailable()) {
        recorder->setClassNames(segmentation->classNames());
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

    processor = std::make_unique<Processor>(config.occupancyMap, colorizer, recorder.get(), segmentation.get(),
        [this](std::shared_ptr<const ProcessedFrame> frame) { onProcessed(std::move(frame)); });
    processor->setColorMode(colorMode, tag);

    acquisition = std::make_unique<Acquisition>(config, calibs,
        [this]() { return recorder->isRecording(); },
        [this](std::shared_ptr<const Frame> frame) { processor->submit(std::move(frame)); },
        [this](Acquisition::Connection connection) {
            QMetaObject::invokeMethod(this, [this, connection]() { onConnected(connection); }, Qt::QueuedConnection);
        });
    acquisition->start();

    connect(&statusTimer, &QTimer::timeout, this, [this]() { updateStatus(); });
    statusTimer.start(250);
}

MainWindow::~MainWindow()
{
    statusTimer.stop();
    // From the sensors down: every stage finishes what it has before the next one stops
    acquisition->stop();
    processor->stop();
    segmentation.reset();
    if (recorder->isRecording()) {
        recorder->stop();   // Writes what is still queued
    }
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
    recordWidget->setRecordingAvailable(Recorder::isAvailable(), "Built without ROS 2 (/opt/ros/jazzy): no recording");
    connect(recordWidget, &RecordWidget::replayRequested, this, [this]() { startReplay(); });
    connect(recordWidget, &RecordWidget::recordingChanged, this, [this](bool recording) {
        if (recording) {
            startRecording();
        } else {
            stopRecording();
        }
    });
    connect(replayWidget, &ReplayWidget::liveRequested, this, [this]() { stopReplay(); });
    connect(replayWidget, &ReplayWidget::frameLoaded, this, [this](const RecordedFrame& frame) { processReplayFrame(frame); });
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
        // Live: the next scan shows the change; a replay processes the current row again
        if (replaying && lastReplayFrame.pointCloud) {
            processReplayFrame(lastReplayFrame);
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

void MainWindow::onConnected(const Acquisition::Connection& connection)
{
    for (size_t i = 0; i < cameraWidgets.size() && i < connection.cameraErrors.size(); ++i) {
        const std::string& ip = config.cameras[i].ip;
        if (connection.cameraErrors[i].empty()) {
            cameraWidgets[i]->setInfo(ip, 0.0, 0, Theme::Status::Waiting);
            cameraWidgets[i]->setRotation(connection.cameraRotations[i]);
            cameraWidgets[i]->onRotateClicked([this, i]() { cameraWidgets[i]->setRotation(acquisition->rotateCamera(i)); });
        } else {
            cameraWidgets[i]->setOffline(ip, QString::fromStdString(connection.cameraErrors[i]));
        }
    }
    if (connection.lidarError.empty()) {
        lidarWidget->setInfo(config.lidarIp, 0.0, 0, 0, Theme::Status::Waiting);
    } else {
        lidarOfflineReason = QString("Lidar not reachable at %1\n%2\n\nCameras are triggered at 10 Hz instead.")
            .arg(QString::fromStdString(config.lidarIp), QString::fromStdString(connection.lidarError));
        if (!replaying) {
            lidarWidget->setOffline(config.lidarIp, lidarOfflineReason);
        }
    }
    liveTriggerText = connection.lidarError.empty() ? "Trigger: lidar scans" : "Trigger: 10 Hz timer (no lidar)";
    if (!replaying) {
        triggerStatus->setText(liveTriggerText);
    }
}

void MainWindow::onProcessed(std::shared_ptr<const ProcessedFrame> frame)
{
    std::lock_guard<std::mutex> lock(latestMutex);
    latestFrame = std::move(frame);
    if (!frameScheduled) {
        frameScheduled = true;
        QMetaObject::invokeMethod(this, [this]() {
            std::shared_ptr<const ProcessedFrame> newest;
            {
                std::lock_guard<std::mutex> lock(latestMutex);
                newest = std::move(latestFrame);
                frameScheduled = false;
            }
            if (newest) {
                showFrame(*newest);
            }
        }, Qt::QueuedConnection);
    }
}

void MainWindow::onSegmented(SegmentationWorker::Result result)
{
    // Recorded here, on the worker thread: also the results that are not shown any more
    if (result.frame && result.frame->record) {
        recorder->writeSegmentation(*result.frame, result.cloud, result.labels, result.classMap);
    }
    std::lock_guard<std::mutex> lock(latestMutex);
    latestSegmentation = std::move(result);
    if (!segmentationScheduled) {
        segmentationScheduled = true;
        QMetaObject::invokeMethod(this, [this]() {
            std::optional<SegmentationWorker::Result> newest;
            {
                std::lock_guard<std::mutex> lock(latestMutex);
                newest.swap(latestSegmentation);
                segmentationScheduled = false;
            }
            if (newest) {
                showSegmentation(*newest);
            }
        }, Qt::QueuedConnection);
    }
}

bool MainWindow::showingOverlay() const
{
    return colorMode == ColorMode::Segmentation && segmentation->isAvailable() && lidarWidget->imageOverlay();
}

void MainWindow::showFrame(const ProcessedFrame& processed)
{
    const Frame& frame = *processed.frame;
    if (frame.live == replaying) {
        return;     // A live frame during a replay, or a replay frame after returning to live
    }

    for (size_t i = 0; i < cameraWidgets.size(); ++i) {
        CameraWidget* tile = cameraWidgets[i];
        if (i >= frame.cameras.size()) {
            if (!frame.live) {
                tile->showMessage("Not in this recording");
                tile->setInfo("", 0.0, 0, Theme::Status::Offline);
            }
            continue;
        }
        const CameraSample& camera = frame.cameras[i];
        if (frame.live && !camera.open) {
            continue;   // Shows why since connecting
        }
        if (camera.newFrame) {
            // With the image overlay the tile shows the segmented frame when the worker is done (showSegmentation)
            if (!(showingOverlay() && frame.cloud)) {
                tile->setImage(toQImage(camera.image));
            }
            tile->setInfo(camera.ip, camera.meta.fps, camera.meta.frameId, Theme::Status::Ok);
        } else {
            if (!frame.live) {
                tile->showMessage(camera.meta.imPath.empty() ? "No image in this row"
                                                             : QString("Cannot read %1").arg(QString::fromStdString(camera.meta.imPath)));
            }
            tile->setInfo(camera.ip, camera.fps, camera.frameId, Theme::Status::Waiting);
        }
    }

    if (frame.cloud) {
        // In segmentation mode the cloud is shown when the worker is done (showSegmentation)
        if (processed.displayCloud) {
            lidarWidget->setScene(processed.displayCloud, processed.plane, processed.occupancy);
        }
        lidarWidget->setInfo(frame.lidar.ip, frame.lidar.fps, frame.lidar.frameId, frame.cloud->size(), Theme::Status::Ok);
    } else if (!frame.live) {
        lidarWidget->showMessage(frame.lidar.pcPath.empty() ? "No point cloud in this row"
                                                            : QString("Cannot read %1").arg(QString::fromStdString(frame.lidar.pcPath)));
        lidarWidget->setInfo(frame.lidar.ip, frame.lidar.fps, frame.lidar.frameId, 0, Theme::Status::Waiting);
    }

    if (frame.robot.valid) {
        robotWidget->setMetaData(frame.robot);
    } else if (frame.live) {
        robotWidget->setOffline(frame.robotError.empty() ? "Connecting…" : QString("Offline: %1").arg(QString::fromStdString(frame.robotError)));
    } else {
        robotWidget->setOffline("No robot data in this row");
    }
}

void MainWindow::showSegmentation(const SegmentationWorker::Result& result)
{
    // Outdated: another colour mode, or switched between live and replay since the job was submitted
    if (result.tag != tag || colorMode != ColorMode::Segmentation || !result.frame || result.frame->live == replaying) {
        return;
    }
    lidarWidget->setScene(result.cloud, result.plane, result.classMap, result.labels, segmentation->palette());
    if (showingOverlay()) {
        for (size_t i = 0; i < result.overlays.size() && i < cameraWidgets.size(); ++i) {
            if (!result.overlays[i].empty()) {
                cameraWidgets[i]->setImage(toQImage(result.overlays[i]));
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

void MainWindow::startReplay()
{
    if (!replayWidget->chooseRecording(replayWidget->recording().size() > 0 ? replayWidget->recording().folder() : recordWidget->folder())) {
        return;
    }
    replaying = true;
    acquisition->setPaused(true);   // Replay never starts while recording (the button is disabled then)
    processor->setColorMode(colorMode, ++tag);
    headerStack->setCurrentWidget(replayWidget);
    for (CameraWidget* cameraWidget : cameraWidgets) {
        cameraWidget->setRotateEnabled(false);
    }
    triggerStatus->setText(QString("Replay: %1").arg(replayWidget->recording().logFile()));
}

void MainWindow::stopReplay()
{
    replaying = false;
    acquisition->setPaused(false);
    processor->setColorMode(colorMode, ++tag);
    headerStack->setCurrentWidget(recordWidget);

    // Back to the live state of each sensor; the next row shows fresh data
    for (CameraWidget* cameraWidget : cameraWidgets) {
        cameraWidget->setRotateEnabled(true);
        cameraWidget->showMessage("Waiting for a frame…");
    }
    if (!lidarOfflineReason.isEmpty()) {
        lidarWidget->setOffline(config.lidarIp, lidarOfflineReason);
    } else {
        lidarWidget->showMessage("Waiting for a scan…");
    }
    triggerStatus->setText(liveTriggerText.isEmpty() ? "Connecting to the sensors…" : liveTriggerText);
}

void MainWindow::processReplayFrame(const RecordedFrame& recorded)
{
    if (!replaying) {
        return;
    }
    lastReplayFrame = recorded;
    processor->submit(toFrame(recorded));
}

void MainWindow::setColorMode(ColorMode mode)
{
    colorMode = mode;
    processor->setColorMode(mode, ++tag);
    if (mode == ColorMode::Segmentation) {
        lidarWidget->setLegend({}, "Segmenting…");
    } else {
        lidarWidget->clearLegend();
    }
    if (mode != ColorMode::Distance && colorizer.validCount() == 0) {
        statusBar()->showMessage(QString("No camera calibration in %1: the point cloud stays grey")
            .arg(QString::fromStdString(config.calibFileName)), 8000);
    }
    // Live rows pick up the new mode with the next scan; a replay processes the current row again
    if (replaying && lastReplayFrame.pointCloud) {
        processReplayFrame(lastReplayFrame);
    }
}

void MainWindow::startRecording()
{
    std::string error;
    if (!recorder->start(recordWidget->folder().toStdString(), error)) {
        qWarning().noquote() << QString::fromStdString(error);
        QMessageBox::warning(this, "Recording", QString::fromStdString(error));
        recordWidget->stop();
        return;
    }
    statusBar()->showMessage("Recording to " + QString::fromStdString(recorder->uri()), 5000);
}

void MainWindow::stopRecording()
{
    if (!recorder->isRecording()) {
        return;
    }
    const QString uri = QString::fromStdString(recorder->uri());
    recorder->stop();   // Writes what is still queued
    const uint64_t rows = recorder->rows();
    const size_t dropped = recorder->dropped();
    recordWidget->setRowCount(rows);
    statusBar()->showMessage(QString("Saved %1 rows: %2%3").arg(rows).arg(uri)
        .arg(dropped > 0 ? QString(" (%1 messages dropped)").arg(dropped) : QString()), 10000);
}

void MainWindow::updateStatus()
{
    QString text;
    if (recorder->isRecording()) {
        recordWidget->setRowCount(recorder->rows());
        text = QString("Rosbag queue: %1").arg(recorder->pending());
        if (const size_t dropped = recorder->dropped(); dropped > 0) {
            text += QString(" · <span style='color:%1'>%2 messages dropped (disk too slow)</span>").arg(Theme::error.name()).arg(dropped);
        }
    }
    writerStatus->setText(text);
}
