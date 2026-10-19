#pragma once

#include <QImage>
#include <QPoint>
#include <QWidget>

#include <vector>

#include <Utils/GroundPlane.h>

// Top-down view of the point cloud projected on the ground plane: the lidar at the origin, forward up,
// rings every few metres. Wheel = zoom, drag = pan, double click = reset view.
class BirdEyeWidget : public QWidget {
public:
    explicit BirdEyeWidget(QWidget* parent = nullptr);

    void setPoints(std::vector<BirdEyePoint> points, const GroundPlane& plane);
    void clear();
    void resetView();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    std::vector<BirdEyePoint> points;
    GroundPlane plane;

    // View: metres visible from the centre to the nearest widget edge, and the ground position at the centre
    float radius;
    QPointF center;     // (forward, left) [m]
    QPoint lastMousePos;

    QImage image;       // Rendered points, redrawn when the points or the view change
    bool imageDirty = true;

    BirdEyeView view() const;
};
