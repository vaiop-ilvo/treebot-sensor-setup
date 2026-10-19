#include <Gui/LidarWidget.h>
#include <QStackedLayout>
#include <QVBoxLayout>

LidarWidget::LidarWidget(QWidget* parent)
    : QFrame(parent)
{
    setObjectName("card");

    header = new SensorHeader("Lidar", this);
    pointCloudWidget = new PointCloudWidget(this);

    // Message shown on top of the point cloud (offline / help text)
    messageLabel = new QLabel(this);
    messageLabel->setAlignment(Qt::AlignCenter);
    messageLabel->setWordWrap(true);
    messageLabel->setStyleSheet(QString("color: %1; font-size: 11pt; background: rgba(%2, %3, %4, 215); border-radius: 8px;")
        .arg(Theme::viewportText.name()).arg(Theme::viewport.red()).arg(Theme::viewport.green()).arg(Theme::viewport.blue()));
    messageLabel->setText("Connecting…");

    auto* hint = new QLabel("drag: rotate · right-drag: pan · scroll: zoom · double-click: reset", this);
    hint->setObjectName("muted");
    hint->setAlignment(Qt::AlignRight);
    QFont hintFont = hint->font();
    hintFont.setPointSize(8);
    hint->setFont(hintFont);

    auto* viewContainer = new QWidget(this);
    auto* stacked = new QStackedLayout(viewContainer);
    stacked->setStackingMode(QStackedLayout::StackAll);
    stacked->addWidget(messageLabel);
    stacked->addWidget(pointCloudWidget);
    messageLabel->raise();

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);
    layout->addWidget(header);
    layout->addWidget(viewContainer, 1);
    layout->addWidget(hint);
}

void LidarWidget::setPointCloud(std::shared_ptr<std::vector<vaiop::QtPoint>> pc) {
    messageLabel->hide();
    pointCloudWidget->setPointCloud(pc);
}

void LidarWidget::setInfo(const std::string& ip, double fps, uint64_t frameId, size_t nPoints, Theme::Status status) {
    header->setAddress(QString::fromStdString(ip));
    header->setStatus(status, status == Theme::Status::Ok ? "Streaming" : "Waiting for scans");
    header->setStats(QString("%1 fps  #%2  %3k pts")
        .arg(fps, 4, 'f', 1)
        .arg(frameId)
        .arg(nPoints / 1000));
}

void LidarWidget::setOffline(const std::string& ip, const QString& reason) {
    header->setAddress(QString::fromStdString(ip));
    header->setStatus(Theme::Status::Offline, reason);
    header->setStats("offline");
    messageLabel->setText(reason);
    messageLabel->show();
}

void LidarWidget::showMessage(const QString& text) {
    pointCloudWidget->setPointCloud(std::make_shared<std::vector<vaiop::QtPoint>>());
    messageLabel->setText(text);
    messageLabel->show();
}
