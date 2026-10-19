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
#include <memory>
#include <vector>


// OpenGL point cloud viewer with an orbit camera:
// left drag = rotate, right/middle drag = pan, wheel = zoom, double click = reset view
class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
public:
    explicit PointCloudWidget(QWidget* parent = nullptr);
    ~PointCloudWidget();

    void setPointCloud(std::shared_ptr<std::vector<vaiop::QtPoint>> newPoints);
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
    QMatrix4x4 viewMatrix() const;

    bool glReady = false;
    QOpenGLShaderProgram shaderProgram;
    QOpenGLBuffer vbo;
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer gridVbo;
    QOpenGLVertexArrayObject gridVao;
    int gridVertexCount = 0;

    // Points are uploaded to the GPU in paintGL (when the context is current)
    std::shared_ptr<std::vector<vaiop::QtPoint>> points;
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
