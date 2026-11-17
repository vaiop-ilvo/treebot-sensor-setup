#include <Gui/RecordWidget.h>
#include <Gui/Theme.h>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QVBoxLayout>

RecordWidget::RecordWidget(QWidget* parent)
    : QFrame(parent)
{
    setupUI();

    connect(recordButton, &QPushButton::clicked, this, &RecordWidget::onRecordClicked);
    connect(stopButton, &QPushButton::clicked, this, &RecordWidget::onStopClicked);
    connect(folderButton, &QPushButton::clicked, this, &RecordWidget::onFolderClicked);
    connect(replayButton, &QPushButton::clicked, this, &RecordWidget::replayRequested);
    connect(&statusTimer, &QTimer::timeout, this, &RecordWidget::updateStatus);
    statusTimer.start(500);
    updateStatus();
}

bool RecordWidget::isRecording() const
{
    return recording;
}

QString RecordWidget::folder() const
{
    return recordFolderName;
}

void RecordWidget::setRowCount(uint64_t newRows)
{
    rows = newRows;
}

void RecordWidget::setRecordingAvailable(bool isAvailable, const QString& reason)
{
    available = isAvailable;
    recordButton->setEnabled(available && !recording && !recordFolderName.isEmpty());
    recordButton->setToolTip(available ? "Start recording a rosbag in the folder" : reason);
}

void RecordWidget::setupUI() {
    setObjectName("card");

    auto* title = new QLabel("RECORD", this);
    title->setObjectName("cardTitle");

    folderButton = new QPushButton("Folder…", this);
    folderButton->setToolTip("Select the output folder: every recording is a rosbag_YYYYMMDD_hhmmss folder in it");
    folderButton->setCursor(Qt::PointingHandCursor);

    recordButton = new QPushButton("●  Record", this);
    recordButton->setObjectName("recordButton");
    recordButton->setToolTip("Start recording a rosbag in the folder");
    recordButton->setCursor(Qt::PointingHandCursor);
    recordButton->setEnabled(false);

    stopButton = new QPushButton("■  Stop", this);
    stopButton->setToolTip("Stop recording");
    stopButton->setCursor(Qt::PointingHandCursor);
    stopButton->setEnabled(false);

    replayButton = new QPushButton("Replay…", this);
    replayButton->setToolTip("Open a recording folder and play it back (a rosbag: convert it with scripts/bag_to_folder.py)");
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
        statusLabel->setText(recordFolderName.isEmpty() ? "IDLE" : QString("READY · last: %1 rows").arg(rows));
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
    recording = true;
    rows = 0;
    elapsed.start();
    recordButton->setEnabled(false);
    stopButton->setEnabled(true);
    folderButton->setEnabled(false);    // Never switch folders in the middle of a recording
    replayButton->setEnabled(false);
    updateStatus();
    emit recordingChanged(true);
}

void RecordWidget::onStopClicked() {
    stop();
}

void RecordWidget::stop() {
    if (!recording) {
        return;
    }
    recording = false;
    recordButton->setEnabled(available);
    stopButton->setEnabled(false);
    folderButton->setEnabled(true);
    replayButton->setEnabled(true);
    updateStatus();
    emit recordingChanged(false);
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
        dialog.setDirectory(recordFolderName);
    }
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return;
    }

    const QString selected = dialog.selectedFiles().first();
    if (!QDir().mkpath(selected)) {
        QMessageBox::warning(this, "Recording", QString("Could not create folder:\n%1").arg(selected));
        return;
    }
    recordFolderName = QDir(selected).absolutePath();
    rows = 0;
    recordButton->setEnabled(available);
    stopButton->setEnabled(false);
    updateStatus();
}
