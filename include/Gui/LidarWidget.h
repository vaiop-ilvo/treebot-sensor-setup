#pragma once

#include <QComboBox>
#include <QFrame>
#include <QLabel>

#include <functional>

#include <Gui/PointCloudWidget.h>
#include <Gui/SensorHeader.h>
#include <Utils/PointCloudColor.h>

class LidarWidget : public QFrame {

public:
    LidarWidget(QWidget* parent = nullptr);

    void setPointCloud(std::shared_ptr<std::vector<vaiop::QtPoint>> pc);
    void setInfo(const std::string& ip, double fps, uint64_t frameId, size_t nPoints, Theme::Status status);
    void setOffline(const std::string& ip, const QString& reason);
    // Removes the point cloud and shows the text instead
    void showMessage(const QString& text);

    // Colour mode selector in the header
    ColorMode colorMode() const;
    void setColorMode(ColorMode mode);
    // Called when the user picks another colour mode
    void onColorModeChanged(std::function<void(ColorMode)> callback);

private:
    SensorHeader* header;
    PointCloudWidget* pointCloudWidget;
    QLabel* messageLabel;
    QComboBox* colorBox;
};
