#include <Gui/LidarWidget.h>
#include <QHBoxLayout>
#include <QStackedLayout>
#include <QStandardItemModel>
#include <QVBoxLayout>

LidarWidget::LidarWidget(QWidget* parent)
    : QFrame(parent)
{
    setObjectName("card");

    header = new SensorHeader("Lidar", this);

    // Colour mode, in the order of ColorMode
    colorBox = new QComboBox(this);
    colorBox->addItem("Colour: distance", static_cast<int>(ColorMode::Distance));
    colorBox->addItem("Colour: camera RGB", static_cast<int>(ColorMode::Rgb));
    colorBox->addItem("Colour: segmentation", static_cast<int>(ColorMode::Segmentation));
    colorBox->setItemData(0, "Turbo colormap on the distance to the lidar (0-15 m)", Qt::ToolTipRole);
    colorBox->setItemData(1, "Colour of the camera pixels (needs config/calib.json); grey = not seen by a camera", Qt::ToolTipRole);
    setSegmentationAvailable(false, "No segmentation model configured");
    header->addTool(colorBox);
    pointCloudWidget = new PointCloudWidget(this);

    // Message shown on top of the point cloud (offline / help text)
    messageLabel = new QLabel(this);
    messageLabel->setAlignment(Qt::AlignCenter);
    messageLabel->setWordWrap(true);
    messageLabel->setStyleSheet(QString("color: %1; font-size: 11pt; background: rgba(%2, %3, %4, 215); border-radius: 8px;")
        .arg(Theme::viewportText.name()).arg(Theme::viewport.red()).arg(Theme::viewport.green()).arg(Theme::viewport.blue()));
    messageLabel->setText("Connecting…");

    legendLabel = new QLabel(this);
    legendLabel->setTextFormat(Qt::RichText);
    legendLabel->setWordWrap(true);
    QFont legendFont = legendLabel->font();
    legendFont.setPointSize(8);
    legendLabel->setFont(legendFont);
    // Same background as the point cloud, so the class colours look the same as on the points
    legendLabel->setStyleSheet(QString("color: %1; background: %2; border-radius: 6px; padding: 3px 8px;")
        .arg(Theme::viewportText.name(), Theme::viewport.name()));
    legendLabel->hide();

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
    auto* footer = new QHBoxLayout();
    footer->setSpacing(12);
    footer->addWidget(legendLabel, 1);
    footer->addWidget(hint, 0, Qt::AlignRight | Qt::AlignBottom);
    layout->addLayout(footer);
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

ColorMode LidarWidget::colorMode() const {
    return static_cast<ColorMode>(colorBox->currentData().toInt());
}

void LidarWidget::setColorMode(ColorMode mode) {
    const int index = colorBox->findData(static_cast<int>(mode));
    if (index >= 0) {
        colorBox->setCurrentIndex(index);
    }
}

void LidarWidget::onColorModeChanged(std::function<void(ColorMode)> callback) {
    connect(colorBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, callback](int) {
        callback(colorMode());
    });
}

void LidarWidget::showMessage(const QString& text) {
    pointCloudWidget->setPointCloud(std::make_shared<std::vector<vaiop::QtPoint>>());
    messageLabel->setText(text);
    messageLabel->show();
}

void LidarWidget::setSegmentationAvailable(bool available, const QString& reason) {
    const int index = colorBox->findData(static_cast<int>(ColorMode::Segmentation));
    if (auto* model = qobject_cast<QStandardItemModel*>(colorBox->model()); model && index >= 0) {
        model->item(index)->setEnabled(available);
    }
    colorBox->setItemData(index, available ? "Class colours of the segmentation model on the camera images; grey = not seen by a camera"
                                           : reason, Qt::ToolTipRole);
}

void LidarWidget::setLegend(const std::vector<LegendEntry>& entries, const QString& note) {
    QStringList items;
    for (const LegendEntry& entry : entries) {
        items << QString("<span style='color:%1; font-size:11pt'>&#9632;</span>&nbsp;%2&nbsp;<span style='color:%3'>%4%</span>")
            .arg(entry.color.name(), entry.name.toHtmlEscaped(), Theme::textMuted.lighter(150).name())
            .arg(entry.share * 100.0, 0, 'f', 0);
    }
    if (!note.isEmpty()) {
        items << QString("<span style='color:%1'>%2</span>").arg(Theme::textMuted.lighter(150).name(), note.toHtmlEscaped());
    }
    legendLabel->setText(items.join("&nbsp;&nbsp; "));
    legendLabel->show();
}

void LidarWidget::clearLegend() {
    legendLabel->clear();
    legendLabel->hide();
}
