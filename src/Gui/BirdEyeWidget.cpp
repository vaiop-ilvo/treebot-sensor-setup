#include <Gui/BirdEyeWidget.h>
#include <Gui/Theme.h>

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace {
    // Default view: 15 m to the nearest edge, centred a few metres ahead (the lidar sees mostly forward)
    constexpr float DEFAULT_RADIUS = 15.0f;
    constexpr float DEFAULT_FORWARD = 4.0f;
    constexpr float MIN_RADIUS = 2.0f;
    constexpr float MAX_RADIUS = 60.0f;

    // Ring spacing so that there are a few rings in view: 1, 2, 5, 10, 20 m
    float ringSpacing(float radius)
    {
        for (const float spacing : {1.0f, 2.0f, 5.0f, 10.0f, 20.0f}) {
            if (radius / spacing <= 6.0f) {
                return spacing;
            }
        }
        return 50.0f;
    }
}

BirdEyeWidget::BirdEyeWidget(QWidget* parent)
    : QWidget(parent)
{
    setMouseTracking(false);
    setCursor(Qt::OpenHandCursor);
    resetView();
}

void BirdEyeWidget::setPoints(std::vector<BirdEyePoint> newPoints, const GroundPlane& newPlane)
{
    points = std::move(newPoints);
    plane = newPlane;
    imageDirty = true;
    update();
}

void BirdEyeWidget::clear()
{
    setPoints({}, GroundPlane{});
}

void BirdEyeWidget::resetView()
{
    radius = DEFAULT_RADIUS;
    center = QPointF(DEFAULT_FORWARD, 0.0);
    imageDirty = true;
    update();
}

BirdEyeView BirdEyeWidget::view() const
{
    BirdEyeView v;
    v.size = cv::Size(std::max(1, width()), std::max(1, height()));
    v.metersPerPixel = radius / (0.5f * std::max(1, std::min(width(), height())));
    v.center = cv::Point2f(static_cast<float>(center.x()), static_cast<float>(center.y()));
    // Points a bit larger when zoomed in, so the surfaces stay closed
    v.pointSize = std::clamp(static_cast<int>(std::lround(0.06f / v.metersPerPixel)), 2, 6);
    return v;
}

void BirdEyeWidget::paintEvent(QPaintEvent*)
{
    const BirdEyeView v = view();
    if (imageDirty || image.size() != size()) {
        const cv::Vec3b bg(Theme::viewport.red(), Theme::viewport.green(), Theme::viewport.blue());
        const cv::Mat rgb = renderBirdEye(points, v, bg);
        image = QImage(rgb.data, rgb.cols, rgb.rows, static_cast<qsizetype>(rgb.step), QImage::Format_RGB888).copy();
        imageDirty = false;
    }

    QPainter painter(this);
    painter.drawImage(0, 0, image);
    painter.setRenderHint(QPainter::Antialiasing);

    auto toWidget = [&v](float forward, float left) {
        const cv::Point2f px = v.toPixel(forward, left);
        return QPointF(px.x, px.y);
    };
    const QPointF origin = toWidget(0.0f, 0.0f);
    const float pixelsPerMeter = 1.0f / v.metersPerPixel;

    // Distance rings with labels
    const float spacing = ringSpacing(radius);
    const float farthest = radius * 1.5f + static_cast<float>(std::hypot(center.x(), center.y()));
    QColor ringColor = Theme::viewportText;
    ringColor.setAlpha(70);
    QFont small = font();
    small.setPointSize(8);
    painter.setFont(small);
    for (float r = spacing; r <= farthest; r += spacing) {
        painter.setPen(QPen(ringColor, 1));
        painter.drawEllipse(origin, r * pixelsPerMeter, r * pixelsPerMeter);
        painter.setPen(Theme::viewportText);
        painter.drawText(origin + QPointF(4, -r * pixelsPerMeter - 3), QString("%1 m").arg(r));
    }

    // Lidar: a triangle pointing forward
    QPainterPath marker;
    marker.moveTo(origin + QPointF(0, -9));
    marker.lineTo(origin + QPointF(6, 6));
    marker.lineTo(origin + QPointF(-6, 6));
    marker.closeSubpath();
    painter.setPen(QPen(Theme::viewport, 1.5));
    painter.setBrush(Theme::ilvoGreen);
    painter.drawPath(marker);

    // Ground plane information
    painter.setPen(Theme::viewportText);
    const QString text = plane.valid
        ? QString("Ground plane: lidar %1 m above the ground · tilt %2° · forward ↑")
              .arg(plane.sensorHeight(), 0, 'f', 2).arg(plane.tiltDeg(), 0, 'f', 1)
        : QString("No ground plane found");
    painter.drawText(rect().adjusted(8, 6, -8, -6), Qt::AlignLeft | Qt::AlignTop, text);
}

void BirdEyeWidget::resizeEvent(QResizeEvent*)
{
    imageDirty = true;
}

void BirdEyeWidget::mousePressEvent(QMouseEvent* event)
{
    lastMousePos = event->pos();
    setCursor(Qt::ClosedHandCursor);
}

void BirdEyeWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (!(event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton))) {
        return;
    }
    const QPoint delta = event->pos() - lastMousePos;
    lastMousePos = event->pos();
    const float metersPerPixel = view().metersPerPixel;
    // Image up = forward, image left = left
    center += QPointF(delta.y() * metersPerPixel, delta.x() * metersPerPixel);
    imageDirty = true;
    update();
}

void BirdEyeWidget::mouseDoubleClickEvent(QMouseEvent*)
{
    resetView();
}

void BirdEyeWidget::wheelEvent(QWheelEvent* event)
{
    // Zoom around the cursor: the ground position under it stays in place
    const BirdEyeView before = view();
    const QPointF pos = event->position();
    const QPointF ground(center.x() - (pos.y() - before.size.height * 0.5) * before.metersPerPixel,
                         center.y() - (pos.x() - before.size.width * 0.5) * before.metersPerPixel);
    const float factor = std::pow(0.999f, static_cast<float>(event->angleDelta().y()));
    radius = std::clamp(radius * factor, MIN_RADIUS, MAX_RADIUS);
    const float metersPerPixel = view().metersPerPixel;
    center = QPointF(ground.x() + (pos.y() - before.size.height * 0.5) * metersPerPixel,
                     ground.y() + (pos.x() - before.size.width * 0.5) * metersPerPixel);
    imageDirty = true;
    update();
}
