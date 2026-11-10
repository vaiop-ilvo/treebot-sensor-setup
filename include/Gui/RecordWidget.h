#pragma once

#include <QComboBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QTextStream>
#include <QTimer>

#include <string>
#include <vector>

#include <Utils/MetaData.hpp>


class RecordWidget : public QFrame {
    Q_OBJECT
public:
    explicit RecordWidget(QWidget* parent = nullptr, int nCameras = 4);
    ~RecordWidget();

    bool isRecording() const;
    // Folder the current recording is written to (with the _N suffix when one was added)
    QString folder() const;
    // Number of rows written to the current log; used as index for the file names
    uint64_t rowCount() const;

    // Recording format selector: the images and point clouds as files in the folder, a rosbag in the folder, or both.
    // The CSV log is always written.
    bool savesFiles() const;
    bool savesBag() const;
    // format: folder, rosbag or both
    void setFormat(const QString& format);
    // Enables the rosbag formats; reason is shown as tooltip when they are not available
    void setBagAvailable(bool available, const QString& reason = {});

    void logData(const RobotMetaData& robotMetaData,
                 const std::vector<CameraMetaData>& cameraMetaData,
                 const OusterMetaData& lidarMetaData);

signals:
    void recordingChanged(bool recording);
    void folderChanged(const QString& folder);
    void replayRequested();

private:
    int nCameras;
    bool recording = false;
    QString recordFolderName;
    uint64_t rows = 0;

    QPushButton* recordButton;
    QPushButton* stopButton;
    QPushButton* folderButton;
    QPushButton* replayButton;
    QComboBox* formatBox;
    QLabel* folderLabel;
    QLabel* recDot;
    QLabel* statusLabel;
    QTimer statusTimer;
    QElapsedTimer elapsed;
    bool blink = false;

    QFile logFile;
    QTextStream stream;
    std::vector<std::string> csvHeader;

    void setupUI();
    void setupCsvHeader();
    void closeLog();
    void updateStatus();
    bool prepareFolder(const QString& selectedFolder);

private slots:
    void onRecordClicked();
    void onStopClicked();
    void onFolderClicked();
};
