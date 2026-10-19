#include <Gui/RecordWidget.h>
#include <Gui/Theme.h>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QVBoxLayout>

namespace {
    QString toIsoTime(uint64_t tsMicro)
    {
        return QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(tsMicro / 1000)).toString("yyyy-MM-ddTHH:mm:ss.zzz");
    }

    QString number(double value)
    {
        return QString::number(value, 'g', 17);
    }

    // Quote a CSV field when needed
    QString csvField(const QString& value)
    {
        if (value.contains(',') || value.contains('"') || value.contains('\n')) {
            QString escaped = value;
            escaped.replace("\"", "\"\"");
            return "\"" + escaped + "\"";
        }
        return value;
    }
}

RecordWidget::RecordWidget(QWidget* parent, int nCameras)
    : QFrame(parent)
    , nCameras(nCameras)
{
    setupUI();
    setupCsvHeader();

    connect(recordButton, &QPushButton::clicked, this, &RecordWidget::onRecordClicked);
    connect(stopButton, &QPushButton::clicked, this, &RecordWidget::onStopClicked);
    connect(folderButton, &QPushButton::clicked, this, &RecordWidget::onFolderClicked);
    connect(replayButton, &QPushButton::clicked, this, &RecordWidget::replayRequested);
    connect(&statusTimer, &QTimer::timeout, this, &RecordWidget::updateStatus);
    statusTimer.start(500);
    updateStatus();
}

RecordWidget::~RecordWidget()
{
    closeLog();
}

bool RecordWidget::isRecording() const
{
    return recording;
}

QString RecordWidget::folder() const
{
    return recordFolderName;
}

uint64_t RecordWidget::rowCount() const
{
    return rows;
}

void RecordWidget::setupCsvHeader()
{
    csvHeader = {"timestamp", "lidar_ip", "lidar_ts", "lidar_frame_id", "lidar_fps", "lidar_pc_path"};
    for (int i = 1; i <= nCameras; ++i) {
        const std::string cam = "cam" + std::to_string(i);
        for (const char* field : {"_ip", "_ts", "_frame_id", "_fps", "_im_path"}) {
            csvHeader.push_back(cam + field);
        }
    }
    for (const char* field : {"robot_x", "robot_y", "robot_z", "robot_roll", "robot_pitch", "robot_yaw",
                              "robot_lat", "robot_lon", "robot_gps_fix", "robot_gps_lng_speed",
                              "robot_odom_lng_speed", "robot_odom_ang_speed"}) {
        csvHeader.push_back(field);
    }
}

void RecordWidget::setupUI() {
    setObjectName("card");

    auto* title = new QLabel("RECORD", this);
    title->setObjectName("cardTitle");

    folderButton = new QPushButton("Folder…", this);
    folderButton->setToolTip("Select the output folder");
    folderButton->setCursor(Qt::PointingHandCursor);

    recordButton = new QPushButton("●  Record", this);
    recordButton->setObjectName("recordButton");
    recordButton->setToolTip("Start recording");
    recordButton->setCursor(Qt::PointingHandCursor);
    recordButton->setEnabled(false);

    stopButton = new QPushButton("■  Stop", this);
    stopButton->setToolTip("Stop recording");
    stopButton->setCursor(Qt::PointingHandCursor);
    stopButton->setEnabled(false);

    replayButton = new QPushButton("Replay…", this);
    replayButton->setToolTip("Open a recording and play it back");
    replayButton->setCursor(Qt::PointingHandCursor);

    folderLabel = new QLabel("No folder selected", this);
    folderLabel->setObjectName("muted");
    folderLabel->setMinimumWidth(160);
    folderLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    recDot = new QLabel(this);
    recDot->setFixedSize(10, 10);

    statusLabel = new QLabel(this);
    QFont mono("monospace");
    mono.setStyleHint(QFont::Monospace);
    statusLabel->setFont(mono);

    auto* titleRow = new QHBoxLayout();
    titleRow->setSpacing(6);
    titleRow->addWidget(title);
    titleRow->addStretch();
    titleRow->addWidget(recDot);
    titleRow->addWidget(statusLabel);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->setSpacing(6);
    buttonRow->addWidget(folderButton);
    buttonRow->addWidget(recordButton);
    buttonRow->addWidget(stopButton);
    buttonRow->addWidget(folderLabel, 1);
    buttonRow->addWidget(replayButton);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(6);
    layout->addLayout(titleRow);
    layout->addLayout(buttonRow);
}

void RecordWidget::updateStatus()
{
    if (recording) {
        blink = !blink;
        const qint64 secs = elapsed.elapsed() / 1000;
        statusLabel->setText(QString("REC %1:%2:%3 · %4 rows")
            .arg(secs / 3600, 2, 10, QChar('0'))
            .arg((secs / 60) % 60, 2, 10, QChar('0'))
            .arg(secs % 60, 2, 10, QChar('0'))
            .arg(rows));
        statusLabel->setStyleSheet(QString("color: %1; font-weight: 700;").arg(Theme::error.name()));
        recDot->setStyleSheet(QString("background: %1; border-radius: 5px;")
            .arg(blink ? Theme::error.name() : Theme::panel.name()));
    } else {
        statusLabel->setText(recordFolderName.isEmpty() ? "IDLE" : QString("READY · %1 rows").arg(rows));
        statusLabel->setStyleSheet(QString("color: %1;").arg(Theme::textMuted.name()));
        recDot->setStyleSheet(QString("background: %1; border-radius: 5px;").arg(Theme::textMuted.name()));
    }

    // Elide the folder path from the left so the end of the path stays visible
    if (!recordFolderName.isEmpty()) {
        folderLabel->setText(folderLabel->fontMetrics().elidedText(recordFolderName, Qt::ElideLeft, folderLabel->width()));
        folderLabel->setToolTip(recordFolderName);
    }
}

void RecordWidget::onRecordClicked() {
    if (!logFile.isOpen() && !logFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
        QMessageBox::warning(this, "Recording", QString("Could not open the log file:\n%1\n\n%2")
            .arg(logFile.fileName(), logFile.errorString()));
        return;
    }
    stream.setDevice(&logFile);

    recording = true;
    elapsed.start();
    recordButton->setEnabled(false);
    stopButton->setEnabled(true);
    folderButton->setEnabled(false);    // Never switch folders in the middle of a recording
    replayButton->setEnabled(false);
    updateStatus();
    emit recordingChanged(true);
}

void RecordWidget::onStopClicked() {
    recording = false;
    closeLog();
    recordButton->setEnabled(true);
    stopButton->setEnabled(false);
    folderButton->setEnabled(true);
    replayButton->setEnabled(true);
    updateStatus();
    qDebug() << "Recording stopped. Log file closed.";
    emit recordingChanged(false);
}

void RecordWidget::closeLog()
{
    if (logFile.isOpen()) {
        // Flush the text stream first, QFile::close does not know about its buffer
        stream.flush();
        logFile.close();
    }
}

bool RecordWidget::prepareFolder(const QString& selectedFolder)
{
    QString folderName = selectedFolder;

    // Only create numbered version if folder exists AND is not empty
    QDir recordDir(folderName);
    for (int index = 1; recordDir.exists() && !recordDir.isEmpty(); ++index) {
        folderName = selectedFolder + "_" + QString::number(index);
        recordDir.setPath(folderName);
    }

    // Create the folder and the sub directories
    QStringList subdirs;
    for (int i = 1; i <= nCameras; ++i) {
        subdirs << QString("cam%1").arg(i);
    }
    subdirs << "lidar";
    for (const QString& subdir : subdirs) {
        if (!recordDir.mkpath(subdir)) {
            QMessageBox::warning(this, "Recording", QString("Could not create folder:\n%1").arg(recordDir.filePath(subdir)));
            return false;
        }
    }

    // Create logging file
    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    logFile.setFileName(recordDir.filePath("sensor_log_" + timestamp + ".csv"));
    if (!logFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
        QMessageBox::warning(this, "Recording", QString("Could not create the log file:\n%1\n\n%2")
            .arg(logFile.fileName(), logFile.errorString()));
        return false;
    }
    stream.setDevice(&logFile);

    // Write header
    QStringList header;
    for (const std::string& name : csvHeader) {
        header << QString::fromStdString(name);
    }
    stream << header.join(',') << "\n";
    stream.flush();

    recordFolderName = recordDir.absolutePath();
    rows = 0;
    return true;
}

void RecordWidget::onFolderClicked() {
    if (recording) {
        return;
    }

    QFileDialog dialog(this, "Select output folder");
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setOption(QFileDialog::ShowDirsOnly);
    dialog.setOption(QFileDialog::DontUseNativeDialog); // Otherwise it does not show up
    if (!recordFolderName.isEmpty()) {
        QDir parent(recordFolderName);
        parent.cdUp();
        dialog.setDirectory(parent);
    }

    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return;
    }

    // Close the previous log before switching to the new folder
    closeLog();
    if (!prepareFolder(dialog.selectedFiles().first())) {
        recordFolderName.clear();
        folderLabel->setText("No folder selected");
        recordButton->setEnabled(false);
        updateStatus();
        return;
    }

    // Activate start recording button
    recordButton->setEnabled(true);
    stopButton->setEnabled(false);
    updateStatus();
    emit folderChanged(recordFolderName);
}

void RecordWidget::logData(const RobotMetaData& robotMetaData,
                const std::vector<CameraMetaData>& cameraMetaData,
                const OusterMetaData& lidarMetaData)
{
    if (!recording || !logFile.isOpen()) {
        return;
    }

    QStringList row;
    row.reserve(static_cast<int>(csvHeader.size()));

    // Timestamp
    row << QDateTime::currentDateTime().toString("yyyy-MM-ddTHH:mm:ss.zzz");

    // Lidar metadata
    if (lidarMetaData.valid) {
        row << QString::fromStdString(lidarMetaData.ip)
            << toIsoTime(lidarMetaData.ts)
            << QString::number(lidarMetaData.frameId)
            << QString::number(lidarMetaData.fps)
            << csvField(QString::fromStdString(lidarMetaData.pcPath));
    } else {
        row << QString::fromStdString(lidarMetaData.ip) << "" << "" << "" << "";
    }

    // Camera metadata, empty cells for cameras without a new frame
    for (int i = 0; i < nCameras; ++i) {
        if (i < static_cast<int>(cameraMetaData.size()) && cameraMetaData[i].valid) {
            const CameraMetaData& cam = cameraMetaData[i];
            row << QString::fromStdString(cam.ip)
                << toIsoTime(cam.ts)
                << QString::number(cam.frameId)
                << QString::number(cam.fps)
                << csvField(QString::fromStdString(cam.imPath));
        } else {
            const QString ip = i < static_cast<int>(cameraMetaData.size()) ? QString::fromStdString(cameraMetaData[i].ip) : "";
            row << ip << "" << "" << "" << "";
        }
    }

    // Robot metadata
    if (robotMetaData.valid) {
        row << number(robotMetaData.x) << number(robotMetaData.y) << number(robotMetaData.z)
            << number(robotMetaData.roll) << number(robotMetaData.pitch) << number(robotMetaData.yaw)
            << number(robotMetaData.lat) << number(robotMetaData.lon)
            << QString::number(robotMetaData.gpsFix)
            << number(robotMetaData.gpsLngSpeed)
            << number(robotMetaData.odomLngSpeed)
            << number(robotMetaData.odomAngSpeed);
    } else {
        for (int i = 0; i < 12; ++i) {
            row << "";
        }
    }

    // Write and flush every row, so nothing is lost when the application stops unexpectedly
    stream << row.join(',') << "\n";
    stream.flush();
    rows++;
}
