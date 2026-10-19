#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QVector3D>
#include <vector>

struct Point {
    QVector3D position;
    QVector3D color;
};

class PointCloudWidgetExample : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
public:
    explicit PointCloudWidgetExample(QWidget* parent = nullptr);
    ~PointCloudWidgetExample();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void generatePoints(int numPoints);

    QOpenGLShaderProgram shaderProgram;
    QOpenGLBuffer vbo;
    QOpenGLVertexArrayObject vao;

    std::vector<Point> points;

    float angle = 0.0f;
    QMatrix4x4 projection;
};

#endif // POINTCLOUDWIDGET_H
