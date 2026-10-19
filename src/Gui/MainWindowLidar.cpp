#include <Gui/MainWindowLidar.h>
#include <Utils/SensorConfig.h>

#include <QTimer>

MainWindowLidar::MainWindowLidar(QWidget *parent)
    : QMainWindow(parent)
{
    const std::string ip = sensorConfig().lidarIp;
    setWindowTitle(QString("Lidar %1").arg(QString::fromStdString(ip)));

    lidarWidget = new LidarWidget(this);
    setCentralWidget(lidarWidget);

    // Ouster
    try {
        ouster = std::make_unique<Ouster>(ip);
    } catch (const std::exception& e) {
        lidarWidget->setOffline(ip, QString("Lidar not reachable at %1\n%2").arg(QString::fromStdString(ip), e.what()));
        return;
    }

    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        if (!ouster->scan()) {
            return;
        }
        auto pc = ouster->getPointCloud();
        if (pc != nullptr) {
            lidarWidget->setPointCloud(pc);
            lidarWidget->setInfo(ouster->getIp(), ouster->getFps(), ouster->getFrameId(), pc->size(), Theme::Status::Ok);
        }
    });
    timer->start(5);
}
