#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QPoint>
#include <QVector3D>
#include <Lidar/QtPoint.h>
#include <Utils/Blobs.h>
#include <Utils/GroundPlane.h>
#include <memory>
#include <vector>


// OpenGL point cloud viewer with an orbit camera:
// left drag = rotate, right/middle drag = pan, wheel = zoom, double click = reset view
class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
public:
    explicit PointCloudWidget(QWidget* parent = nullptr);
    ~PointCloudWidget();

    void setPointCloud(std::shared_ptr<const std::vector<vaiop::QtPoint>> newPoints);
    // Blobs drawn as translucent discs facing the viewer, with their radius in metres; empty = none
    void setBlobs(std::vector<Blob> newBlobs);
    // Ground plane of the cloud: the grid lies on it, and the robot marker (an arrow pointing forward with a ring,
    // on the ground below the lidar, and a line up to the lidar) shows where the treebot is. Invalid plane: 1 m below
    // the lidar, in the lidar frame.
    void setGround(const GroundPlane& plane);
    void resetView();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void setupVertexArray(QOpenGLVertexArrayObject& vao, QOpenGLBuffer& vbo);
    void initializeBlobs();
    QMatrix4x4 viewMatrix() const;

    bool glReady = false;
    QOpenGLShaderProgram shaderProgram;
    QOpenGLBuffer vbo;
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer gridVbo;
    QOpenGLVertexArrayObject gridVao;
    int gridVertexCount = 0;

    // Robot marker: triangles (arrow, ring) followed by lines (pole, lidar axes), rebuilt with the ground plane
    QOpenGLBuffer markerVbo;
    QOpenGLVertexArrayObject markerVao;
    int markerTriangleVertices = 0;
    int markerLineVertices = 0;
    GroundPlane ground;
    bool groundDirty = true;
    void uploadGround();

    // Blobs: one quad per blob (instanced), turned into a disc in the fragment shader
    QOpenGLShaderProgram blobProgram;
    QOpenGLBuffer quadVbo;
    QOpenGLBuffer blobVbo;
    QOpenGLVertexArrayObject blobVao;
    std::vector<Blob> blobs;
    bool blobsReady = false;

    // Points are uploaded to the GPU in paintGL (when the context is current)
    std::shared_ptr<const std::vector<vaiop::QtPoint>> points;
    bool pointsDirty = false;
    int uploadedPointCount = 0;

    // Orbit camera
    float yaw;          // around z [deg]
    float pitch;        // elevation [deg]
    float distance;     // [m]
    QVector3D target;
    QPoint lastMousePos;
    QMatrix4x4 projection;
};

#endif // POINTCLOUDWIDGET_H
