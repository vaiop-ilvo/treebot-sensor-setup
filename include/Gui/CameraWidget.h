#pragma once

#include <QFrame>
#include <QToolButton>

#include <functional>

#include <Gui/ImageWidget.h>
#include <Gui/SensorHeader.h>

class CameraWidget : public QFrame {

public:
    CameraWidget(const QString& name = "Camera", QWidget* parent = nullptr);

    void setImage(const QImage& image);
    void setInfo(const std::string& ip, double fps, uint64_t frameId, Theme::Status status);
    void setOffline(const std::string& ip, const QString& reason);
    // Removes the image and shows the text instead
    void showMessage(const QString& text);
    // Shows the current rotation (degrees clockwise) on the rotate button
    void setRotation(int degrees);
    // Called when the user clicks the rotate button
    void onRotateClicked(std::function<void()> callback);
    // Rotation only applies to a connected camera, not to a replay
    void setRotateEnabled(bool enabled);

private:
    SensorHeader* header;
    ImageWidget* imageWidget;
    QToolButton* rotateButton;
    bool rotateConnected = false;
};
