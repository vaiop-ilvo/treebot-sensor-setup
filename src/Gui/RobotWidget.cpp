#include <Gui/RobotWidget.h>
#include <Gui/Theme.h>
#include <QGridLayout>
#include <QHBoxLayout>

RobotWidget::RobotWidget(QWidget* parent)
    : QFrame(parent)
{
    setObjectName("card");

    auto* title = new QLabel("ROBOT", this);
    title->setObjectName("cardTitle");

    statusDot = new QLabel(this);
    statusDot->setFixedSize(10, 10);

    hostLabel = new QLabel(this);
    hostLabel->setObjectName("muted");

    fixBadge = new QLabel(this);
    fixBadge->setAlignment(Qt::AlignCenter);

    latLonLabel = new QLabel("Lat ---  Lon ---", this);
    QFont mono("monospace");
    mono.setStyleHint(QFont::Monospace);
    latLonLabel->setFont(mono);
    latLonLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

    speedLabel = new QLabel(this);
    speedLabel->setObjectName("muted");

    auto* titleRow = new QHBoxLayout();
    titleRow->setSpacing(6);
    titleRow->addWidget(statusDot);
    titleRow->addWidget(title);
    titleRow->addWidget(hostLabel);
    titleRow->addStretch();
    titleRow->addWidget(fixBadge);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(4);
    layout->addLayout(titleRow);
    layout->addWidget(latLonLabel);
    layout->addWidget(speedLabel);

    setFixState(0);
    setOffline("Connecting…");
}

void RobotWidget::setHost(const QString& host)
{
    hostLabel->setText(host);
}

void RobotWidget::setMetaData(const RobotMetaData& metaData)
{
    if (!metaData.valid) {
        setOffline("No data");
        return;
    }

    statusDot->setStyleSheet(QString("background: %1; border-radius: 5px;").arg(Theme::ok.name()));
    statusDot->setToolTip("Connected to Redis");
    setLatLon(metaData.lat, metaData.lon);
    setFixState(metaData.gpsFix);
    speedLabel->setText(QString("GNSS %1 m/s · odom %2 m/s · %3 rad/s")
        .arg(metaData.gpsLngSpeed, 0, 'f', 2)
        .arg(metaData.odomLngSpeed, 0, 'f', 2)
        .arg(metaData.odomAngSpeed, 0, 'f', 2));
}

void RobotWidget::setOffline(const QString& reason)
{
    statusDot->setStyleSheet(QString("background: %1; border-radius: 5px;").arg(Theme::error.name()));
    statusDot->setToolTip(reason);
    speedLabel->setText(reason);
    setFixState(0);
}

void RobotWidget::setLatLon(double lat, double lon)
{
    latLonLabel->setText(QString("Lat %1  Lon %2").arg(lat, 0, 'f', 8).arg(lon, 0, 'f', 8));
}

void RobotWidget::setFixState(int fix)
{
    const QColor color = fixStateToColor(fix);
    fixBadge->setText(fixStateToString(fix));
    fixBadge->setStyleSheet(QString(
        "QLabel { color: %1; background: rgba(%2, %3, %4, 40); border: 1px solid %1;"
        " border-radius: 9px; padding: 1px 8px; font-weight: 600; font-size: 8pt; }")
        .arg(color.name()).arg(color.red()).arg(color.green()).arg(color.blue()));
}

QString RobotWidget::fixStateToString(int fix)
{
    switch (fix) {
        case 4: return "RTK FIX";
        case 5: return "RTK FLOAT";
        case 2: return "DGPS";
        case 3: return "2D FIX";
        case 1: return "NO RTK";
        default: return "NO FIX";
    }
}

QColor RobotWidget::fixStateToColor(int fix)
{
    switch (fix) {
        case 4: return Theme::ok;
        case 5: return Theme::warning;
        case 2: return QColor(0x1b, 0x8a, 0x93);
        case 3: return Theme::accent;
        case 1: return Theme::error;
        default: return Theme::textMuted;
    }
}
