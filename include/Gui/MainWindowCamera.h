#pragma once

#include <memory>

#include <QMainWindow>
#include <Gui/CameraWidget.h>
#include <Camera/Daheng.h>

class MainWindowCamera : public QMainWindow {
public:
    MainWindowCamera(QWidget *parent = nullptr);
    ~MainWindowCamera();

private:
    CameraWidget *cameraWidget;
    std::unique_ptr<Daheng> daheng;
};
