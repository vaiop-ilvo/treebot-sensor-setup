#include <Gui/LidarWidget.h>
#include <QHBoxLayout>
#include <QStackedLayout>
#include <QStandardItemModel>
#include <QVBoxLayout>

#include <cmath>

namespace {
    // Blob view: points of a class closer than this are connected [m] (the OS1-128 points are ~0.2 m apart at 30 m)
    constexpr float BLOB_LINK_DISTANCE = 0.3f;
    // Points up to this height above the ground plane are ground, left out of the blobs [m]
    constexpr float BLOB_GROUND_MARGIN = 0.15f;
}

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

    overlayBox = new QCheckBox("Image overlay", this);
    overlayBox->setToolTip("Show the segmentation classes on the camera images (segmentation mode)");
    header->addTool(overlayBox);
    blobButton = new QToolButton(this);
    blobButton->setText("Blobs");
    blobButton->setCheckable(true);
    blobButton->setToolTip("Blob view (segmentation mode): connected points of the same class form a blob, shown with a circle around it.\n"
                           "The slider under the view sets the minimum blob size (points): smaller blobs are filtered out. The ground is left out.");
    header->addTool(blobButton);
    connect(blobButton, &QToolButton::toggled, this, [this](bool) {
        updateViewControls();
        showCloud();
    });
    birdEyeBox = new QCheckBox("Bird's-eye view", this);
    birdEyeBox->setToolTip("Project the point cloud on the ground plane (detected in every scan) and show it from above");
    header->addTool(birdEyeBox);
    connect(birdEyeBox, &QCheckBox::toggled, this, [this](bool) {
        updateViewControls();
        showCloud();
    });
    connect(colorBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) { updateViewControls(); });

    pointCloudWidget = new PointCloudWidget(this);
    birdEyeWidget = new BirdEyeWidget(this);

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

    // Blob size, shown while the blob view is on
    blobControls = new QWidget(this);
    auto* blobLayout = new QHBoxLayout(blobControls);
    blobLayout->setContentsMargins(0, 0, 0, 0);
    blobLayout->setSpacing(6);
    auto* blobTitle = new QLabel("Min. blob size", blobControls);
    blobTitle->setObjectName("muted");
    blobSlider = new QSlider(Qt::Horizontal, blobControls);
    blobSlider->setRange(0, 100);
    blobSlider->setValue(40);
    blobSlider->setFixedWidth(160);
    blobSlider->setToolTip("Minimum number of points in a blob: smaller blobs are filtered out");
    blobSizeLabel = new QLabel(blobControls);
    blobSizeLabel->setObjectName("muted");
    blobSizeLabel->setMinimumWidth(150);
    blobLayout->addWidget(blobTitle);
    blobLayout->addWidget(blobSlider);
    blobLayout->addWidget(blobSizeLabel);
    connect(blobSlider, &QSlider::valueChanged, this, [this](int) { showCloud(); });
    updateBlobLabel();

    hintLabel = new QLabel(this);
    hintLabel->setObjectName("muted");
    hintLabel->setAlignment(Qt::AlignRight);
    QFont hintFont = hintLabel->font();
    hintFont.setPointSize(8);
    hintLabel->setFont(hintFont);

    auto* viewContainer = new QWidget(this);
    auto* stacked = new QStackedLayout(viewContainer);
    stacked->setStackingMode(QStackedLayout::StackAll);
    stacked->addWidget(messageLabel);
    stacked->addWidget(pointCloudWidget);
    stacked->addWidget(birdEyeWidget);
    messageLabel->raise();

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);
    layout->addWidget(header);
    layout->addWidget(viewContainer, 1);
    auto* footer = new QHBoxLayout();
    footer->setSpacing(12);
    footer->addWidget(legendLabel, 1);
    footer->addWidget(blobControls, 0, Qt::AlignBottom);
    footer->addWidget(hintLabel, 0, Qt::AlignRight | Qt::AlignBottom);
    layout->addLayout(footer);

    updateViewControls();
}

void LidarWidget::setPointCloud(std::shared_ptr<std::vector<vaiop::QtPoint>> pc) {
    setPointCloud(std::move(pc), {}, {});
}

void LidarWidget::setPointCloud(std::shared_ptr<std::vector<vaiop::QtPoint>> pc, std::vector<uint8_t> labels,
                                std::vector<cv::Vec3b> palette) {
    messageLabel->hide();
    lastCloud = std::move(pc);
    lastLabels = std::move(labels);
    lastPalette = std::move(palette);
    showCloud();
}

bool LidarWidget::showingBlobs() const {
    return blobButton->isChecked() && blobButton->isEnabled() && lastCloud && lastLabels.size() == lastCloud->size();
}

uint32_t LidarWidget::minBlobPoints() const {
    // Logarithmic: 5 points at the left end, 5000 at the right end
    return static_cast<uint32_t>(std::lround(5.0 * std::pow(1000.0, blobSlider->value() / 100.0)));
}

void LidarWidget::updateBlobLabel() {
    blobSizeLabel->setText(QString("≥ %1 points · %2 blobs").arg(minBlobPoints()).arg(blobCount));
}

void LidarWidget::showCloud() {
    const bool birdEye = birdEyeBox->isChecked();
    if (!lastCloud || lastCloud->empty()) {
        pointCloudWidget->setPointCloud(std::make_shared<std::vector<vaiop::QtPoint>>());
        pointCloudWidget->setBlobs({});
        birdEyeWidget->clear();
        return;
    }

    // ~1 ms for the plane fit; needed by the bird's-eye view and to leave the ground out of the blobs
    const bool blobView = showingBlobs();
    if (birdEye || blobView) {
        groundPlane = fitGroundPlane(*lastCloud, &groundPlane);
    }

    // The cloud to show: everything, or only the points of the blobs that are large enough
    std::shared_ptr<std::vector<vaiop::QtPoint>> shown = lastCloud;
    std::vector<Blob> blobs;
    if (blobView) {
        // The ground would connect everything on it into one blob
        std::vector<uint8_t> include;
        if (groundPlane.valid) {
            include.resize(lastCloud->size());
            for (size_t i = 0; i < lastCloud->size(); ++i) {
                const QVector3D& p = (*lastCloud)[i].position;
                include[i] = groundPlane.height(Eigen::Vector3f(p.x(), p.y(), p.z())) > BLOB_GROUND_MARGIN ? 1 : 0;
            }
        }
        BlobResult result = findBlobs(*lastCloud, lastLabels, lastPalette, minBlobPoints(), BLOB_LINK_DISTANCE, include);
        shown = std::make_shared<std::vector<vaiop::QtPoint>>();
        shown->reserve(lastCloud->size() / 2);
        for (size_t i = 0; i < lastCloud->size(); ++i) {
            if (result.pointBlob[i] >= 0) {
                shown->push_back((*lastCloud)[i]);
            }
        }
        blobs = std::move(result.blobs);
    }
    blobCount = blobs.size();
    updateBlobLabel();

    if (birdEye) {
        birdEyeWidget->setPoints(projectToGround(*shown, groundPlane), groundPlane, projectToGround(blobs, groundPlane));
    } else {
        pointCloudWidget->setPointCloud(shown);
        pointCloudWidget->setBlobs(std::move(blobs));
    }
}

void LidarWidget::updateViewControls() {
    const bool birdEye = birdEyeBox->isChecked();
    pointCloudWidget->setVisible(!birdEye);
    birdEyeWidget->setVisible(birdEye);
    hintLabel->setText(birdEye ? "drag: pan · scroll: zoom · double-click: reset"
                               : "drag: rotate · right-drag: pan · scroll: zoom · double-click: reset");
    const bool segmentation = colorMode() == ColorMode::Segmentation;
    overlayBox->setEnabled(segmentation);
    blobButton->setEnabled(segmentation);
    blobControls->setVisible(segmentation && blobButton->isChecked());
}

bool LidarWidget::imageOverlay() const {
    return overlayBox->isChecked() && overlayBox->isEnabled();
}

void LidarWidget::onImageOverlayChanged(std::function<void(bool)> callback) {
    connect(overlayBox, &QCheckBox::toggled, this, [this, callback](bool) { callback(imageOverlay()); });
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
    lastCloud.reset();
    lastLabels.clear();
    pointCloudWidget->setPointCloud(std::make_shared<std::vector<vaiop::QtPoint>>());
    pointCloudWidget->setBlobs({});
    birdEyeWidget->clear();
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
