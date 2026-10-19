#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QLabel>

#include <functional>
#include <vector>

#include <Gui/BirdEyeWidget.h>
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
    // Enables the segmentation mode; reason is shown as tooltip when it is not available
    void setSegmentationAvailable(bool available, const QString& reason = {});

    // Classes shown under the point cloud in segmentation mode: (name, colour, share of the labelled points)
    struct LegendEntry {
        QString name;
        QColor color;
        double share = 0.0;
    };
    void setLegend(const std::vector<LegendEntry>& entries, const QString& note = {});
    void clearLegend();

    // "Image overlay" check box (segmentation mode only): show the classes on the camera images
    bool imageOverlay() const;
    void onImageOverlayChanged(std::function<void(bool)> callback);

private:
    SensorHeader* header;
    PointCloudWidget* pointCloudWidget;
    BirdEyeWidget* birdEyeWidget;
    QCheckBox* birdEyeBox;      // 3D view or bird's-eye view on the ground plane
    QCheckBox* overlayBox;
    QLabel* hintLabel;

    std::shared_ptr<std::vector<vaiop::QtPoint>> lastCloud;   // Shown again when the view is switched
    GroundPlane groundPlane;                                    // Previous fit, for temporal smoothing

    void showCloud();
    void updateViewControls();
    QLabel* messageLabel;
    QComboBox* colorBox;
    QLabel* legendLabel;
};
