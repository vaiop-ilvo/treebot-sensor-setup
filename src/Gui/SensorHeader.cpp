#include <Gui/SensorHeader.h>

SensorHeader::SensorHeader(const QString& title, QWidget* parent)
    : QWidget(parent)
{
    layout = new QHBoxLayout(this);
    layout->setContentsMargins(4, 0, 4, 0);
    layout->setSpacing(8);

    dotLabel = new QLabel(this);
    dotLabel->setFixedSize(10, 10);

    titleLabel = new QLabel(title, this);
    titleLabel->setStyleSheet("font-weight: 700;");

    addressLabel = new QLabel(this);
    addressLabel->setObjectName("muted");

    statsLabel = new QLabel(this);
    statsLabel->setObjectName("muted");
    statsLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    // Monospaced digits so the numbers do not jump around
    QFont mono("monospace");
    mono.setStyleHint(QFont::Monospace);
    mono.setPointSize(9);
    statsLabel->setFont(mono);

    layout->addWidget(dotLabel);
    layout->addWidget(titleLabel);
    layout->addWidget(addressLabel);
    layout->addStretch();
    layout->addWidget(statsLabel);

    setStatus(Theme::Status::Offline);
}

void SensorHeader::setTitle(const QString& title)
{
    titleLabel->setText(title);
}

void SensorHeader::setAddress(const QString& address)
{
    addressLabel->setText(address);
}

void SensorHeader::setStatus(Theme::Status status, const QString& tooltip)
{
    dotLabel->setStyleSheet(QString("background: %1; border-radius: 5px;").arg(Theme::statusColor(status).name()));
    dotLabel->setToolTip(tooltip);
}

void SensorHeader::setStats(const QString& stats)
{
    statsLabel->setText(stats);
}

void SensorHeader::addTool(QWidget* tool)
{
    layout->addWidget(tool);
}
