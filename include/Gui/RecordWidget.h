#pragma once

#include <QElapsedTimer>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

// Recording controls: the output folder, Record / Stop and the status. Every Record ... Stop writes one rosbag
// (folder rosbag_YYYYMMDD_hhmmss) in the output folder; MainWindow does the writing.
class RecordWidget : public QFrame {
    Q_OBJECT
public:
    explicit RecordWidget(QWidget* parent = nullptr);

    bool isRecording() const;
    QString folder() const;
    // Rows written to the current bag, shown in the status
    void setRowCount(uint64_t rows);
    // Disables recording, with the reason as tooltip (e.g. built without ROS 2)
    void setRecordingAvailable(bool available, const QString& reason = {});
    // Stops the recording, as the Stop button (e.g. when the bag cannot be created)
    void stop();

signals:
    void recordingChanged(bool recording);
    void replayRequested();

private:
    bool recording = false;
    bool available = true;
    QString recordFolderName;
    uint64_t rows = 0;

    QPushButton* recordButton;
    QPushButton* stopButton;
    QPushButton* folderButton;
    QPushButton* replayButton;
    QLabel* folderLabel;
    QLabel* recDot;
    QLabel* statusLabel;
    QTimer statusTimer;
    QElapsedTimer elapsed;
    bool blink = false;

    void setupUI();
    void updateStatus();

private slots:
    void onRecordClicked();
    void onStopClicked();
    void onFolderClicked();
};
