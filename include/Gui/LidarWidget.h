#pragma once

#include <QFrame>
#include <QLabel>
#include <Gui/PointCloudWidget.h>
#include <Gui/SensorHeader.h>

class LidarWidget : public QFrame {

public:
    LidarWidget(QWidget* parent = nullptr);

    void setPointCloud(std::shared_ptr<std::vector<vaiop::QtPoint>> pc);
    void setInfo(const std::string& ip, double fps, uint64_t frameId, size_t nPoints, Theme::Status status);
    void setOffline(const std::string& ip, const QString& reason);
    // Removes the point cloud and shows the text instead
    void showMessage(const QString& text);

private:
    SensorHeader* header;
    PointCloudWidget* pointCloudWidget;
    QLabel* messageLabel;
};
