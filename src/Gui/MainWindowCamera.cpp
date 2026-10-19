#include <Gui/MainWindowCamera.h>
#include <Utils/SensorConfig.h>

#include <QTimer>

MainWindowCamera::MainWindowCamera(QWidget *parent)
    : QMainWindow(parent)
{
    // First camera of the config
    const CameraInfo camInfo = sensorConfig().cameras.empty() ? CameraInfo{"cam1", "192.168.2.14"} : sensorConfig().cameras.front();
    setWindowTitle(QString("Camera %1").arg(QString::fromStdString(camInfo.ip)));

    cameraWidget = new CameraWidget(QString::fromStdString(camInfo.name), this);
    setCentralWidget(cameraWidget);

    // Daheng
    daheng = std::make_unique<Daheng>(camInfo.name, camInfo.ip, camInfo.rFlag);
    if (!daheng->isOpen()) {
        cameraWidget->setOffline(camInfo.ip, QString("Cannot open camera at %1").arg(QString::fromStdString(camInfo.ip)));
        return;
    }
    daheng->start();

    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        auto imPtr = daheng->getQtImage();
        if (imPtr != nullptr) {
            cameraWidget->setImage(*imPtr);
            cameraWidget->setInfo(daheng->getIp(), daheng->getFps(), daheng->getFrameId(), Theme::Status::Ok);
        }
    });
    timer->start(16); // ~60 FPS
}

MainWindowCamera::~MainWindowCamera() {
    if (daheng) {
        daheng->stop();
    }
}
