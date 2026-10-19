#include <Gui/CameraWidget.h>
#include <QVBoxLayout>

CameraWidget::CameraWidget(const QString& name, QWidget* parent)
    : QFrame(parent)
{
    setObjectName("card");

    header = new SensorHeader(name, this);
    imageWidget = new ImageWidget(this);
    imageWidget->setPlaceholderText("Connecting…");

    rotateButton = new QToolButton(this);
    rotateButton->setObjectName("rotateButton");
    rotateButton->setText("⟳");
    rotateButton->setCursor(Qt::PointingHandCursor);
    rotateButton->setEnabled(false);    // Until the camera is connected
    header->addTool(rotateButton);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);
    layout->addWidget(header);
    layout->addWidget(imageWidget, 1);
}

void CameraWidget::setImage(const QImage& image) {
    imageWidget->setImage(image);
}

void CameraWidget::setRotation(int degrees) {
    rotateButton->setToolTip(QString("Rotate 90° clockwise (now %1°)\nApplies to the view and the saved images").arg(degrees));
}

void CameraWidget::onRotateClicked(std::function<void()> callback) {
    connect(rotateButton, &QToolButton::clicked, this, [callback]() { callback(); });
    rotateConnected = true;
    rotateButton->setEnabled(true);
}

void CameraWidget::setRotateEnabled(bool enabled) {
    rotateButton->setEnabled(enabled && rotateConnected);
}

void CameraWidget::showMessage(const QString& text) {
    imageWidget->clearImage();
    imageWidget->setPlaceholderText(text);
}

void CameraWidget::setInfo(const std::string& ip, double fps, uint64_t frameId, Theme::Status status) {
    header->setAddress(QString::fromStdString(ip));
    header->setStatus(status, status == Theme::Status::Ok ? "Streaming" : "No new frame");
    header->setStats(QString("%1 fps  #%2").arg(fps, 4, 'f', 1).arg(frameId));
}

void CameraWidget::setOffline(const std::string& ip, const QString& reason) {
    header->setAddress(QString::fromStdString(ip));
    header->setStatus(Theme::Status::Offline, reason);
    header->setStats("offline");
    imageWidget->clearImage();
    imageWidget->setPlaceholderText(reason);
}
