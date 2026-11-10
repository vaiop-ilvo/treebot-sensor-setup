#pragma once

#include <QImage>
#include <QPoint>
#include <QWidget>

#include <vector>

#include <Utils/GroundPlane.h>

// Occupancy map: top-down view of the point cloud projected on the ground plane, only the points in a height range
// above the ground. The lidar at the origin, forward up, rings every few metres. Wheel = zoom, drag = pan,
// double click = reset view.
class BirdEyeWidget : public QWidget {
public:
    explicit BirdEyeWidget(QWidget* parent = nullptr);

    // blobs: circles drawn around the points (blob view), empty = none
    void setPoints(std::vector<BirdEyePoint> points, const GroundPlane& plane, std::vector<BirdEyeBlob> blobs = {});
    void clear();
    void resetView();
    // Class grid instead of points: cells filled with their most common class (needs labelled points)
    void setClassGrid(bool on);
    // Only points from minHeight up to maxHeight above the ground plane are shown [m]
    void setHeightRange(float minHeight, float maxHeight);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    std::vector<BirdEyePoint> points;
    std::vector<BirdEyeBlob> blobs;
    GroundPlane plane;
    bool classGrid = false;
    float minHeight;
    float maxHeight;

    // View: metres visible from the centre to the nearest widget edge, and the ground position at the centre
    float radius;
    QPointF center;     // (forward, left) [m]
    QPoint lastMousePos;

    QImage image;       // Rendered points, redrawn when the points or the view change
    bool imageDirty = true;

    BirdEyeView view() const;
    float gridCellSize(const BirdEyeView& v) const;
};
