#pragma once

#include <memory>

#include <QMainWindow>
#include <Gui/LidarWidget.h>
#include <Lidar/Ouster.h>

class MainWindowLidar : public QMainWindow {
public:
    MainWindowLidar(QWidget *parent = nullptr);

private:
    LidarWidget *lidarWidget;
    std::unique_ptr<Ouster> ouster;
};
