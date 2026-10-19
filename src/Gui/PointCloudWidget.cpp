#include <Gui/PointCloudWidget.h>
#include <Gui/Theme.h>
#include <QDebug>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace {
    // Ground grid: 1 m cells, GRID_HALF_SIZE m in every direction around the lidar
    constexpr int GRID_HALF_SIZE = 10;
    constexpr float GRID_Z = -1.0f;
}

PointCloudWidget::PointCloudWidget(QWidget* parent)
    : QOpenGLWidget(parent),
      vbo(QOpenGLBuffer::VertexBuffer),
      gridVbo(QOpenGLBuffer::VertexBuffer)
{
    setFocusPolicy(Qt::StrongFocus);
    setMinimumHeight(200);

    // OpenGL 3.3 core (matches the shaders and QOpenGLFunctions_3_3_Core) with anti-aliasing
    QSurfaceFormat fmt = format();
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSamples(4);
    setFormat(fmt);

    resetView();
}

PointCloudWidget::~PointCloudWidget() {
    makeCurrent();
    vbo.destroy();
    vao.destroy();
    gridVbo.destroy();
    gridVao.destroy();
    shaderProgram.removeAllShaders();
    doneCurrent();
}

void PointCloudWidget::resetView()
{
    // Looking from behind and above the lidar
    yaw = 180.0f;
    pitch = 25.0f;
    distance = 8.0f;
    target = QVector3D(2.0f, 0.0f, 0.0f);
    update();
}

void PointCloudWidget::setupVertexArray(QOpenGLVertexArrayObject& vertexArray, QOpenGLBuffer& buffer)
{
    vertexArray.create();
    vertexArray.bind();
    buffer.create();
    buffer.bind();

    shaderProgram.bind();
    // position attribute
    shaderProgram.enableAttributeArray(0);
    shaderProgram.setAttributeBuffer(0, GL_FLOAT, offsetof(vaiop::QtPoint, position), 3, sizeof(vaiop::QtPoint));
    // color attribute
    shaderProgram.enableAttributeArray(1);
    shaderProgram.setAttributeBuffer(1, GL_FLOAT, offsetof(vaiop::QtPoint, color), 3, sizeof(vaiop::QtPoint));
    shaderProgram.release();

    vertexArray.release();
    buffer.release();
}

void PointCloudWidget::initializeGL() {
    // Fails when no OpenGL 3.3 context is available (e.g. X forwarding, offscreen); never call GL functions then
    if (!initializeOpenGLFunctions()) {
        qWarning() << "OpenGL 3.3 is not available, the point cloud view is disabled";
        return;
    }

    glClearColor(Theme::viewport.redF(), Theme::viewport.greenF(), Theme::viewport.blueF(), 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Compile shaders
    shaderProgram.addShaderFromSourceCode(QOpenGLShader::Vertex,
        R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;

        uniform mat4 mvp;
        uniform float pointSize;

        out vec3 vertexColor;

        void main() {
            gl_Position = mvp * vec4(aPos, 1.0);
            gl_PointSize = pointSize;
            vertexColor = aColor;
        }
        )");

    shaderProgram.addShaderFromSourceCode(QOpenGLShader::Fragment,
        R"(
        #version 330 core
        in vec3 vertexColor;
        out vec4 FragColor;

        void main() {
            FragColor = vec4(vertexColor, 1.0);
        }
        )");

    if (!shaderProgram.link()) {
        qWarning() << "Shader link error:" << shaderProgram.log();
        return;
    }

    setupVertexArray(vao, vbo);
    setupVertexArray(gridVao, gridVbo);

    // Ground grid with the x (red) and y (green) axes
    std::vector<vaiop::QtPoint> grid;
    const QVector3D gridColor(0.18f, 0.21f, 0.26f);
    for (int i = -GRID_HALF_SIZE; i <= GRID_HALF_SIZE; ++i) {
        const QVector3D colorX = (i == 0) ? QVector3D(0.55f, 0.25f, 0.25f) : gridColor;
        const QVector3D colorY = (i == 0) ? QVector3D(0.25f, 0.5f, 0.3f) : gridColor;
        grid.push_back({QVector3D(-GRID_HALF_SIZE, i, GRID_Z), colorY});
        grid.push_back({QVector3D(GRID_HALF_SIZE, i, GRID_Z), colorY});
        grid.push_back({QVector3D(i, -GRID_HALF_SIZE, GRID_Z), colorX});
        grid.push_back({QVector3D(i, GRID_HALF_SIZE, GRID_Z), colorX});
    }
    gridVertexCount = static_cast<int>(grid.size());
    gridVbo.bind();
    gridVbo.allocate(grid.data(), static_cast<int>(grid.size() * sizeof(vaiop::QtPoint)));
    gridVbo.release();

    glReady = true;
}


void PointCloudWidget::setPointCloud(std::shared_ptr<std::vector<vaiop::QtPoint>> newPoints)
{
    // Only keep a reference here: the upload happens in paintGL where the context is guaranteed to be ready
    points = std::move(newPoints);
    pointsDirty = true;
    update();
}

void PointCloudWidget::resizeGL(int w, int h) {
    projection.setToIdentity();
    projection.perspective(45.0f, float(w) / float(std::max(h, 1)), 0.1f, 300.0f);
}

QMatrix4x4 PointCloudWidget::viewMatrix() const
{
    const float yawRad = qDegreesToRadians(yaw);
    const float pitchRad = qDegreesToRadians(pitch);
    const QVector3D offset(
        distance * std::cos(pitchRad) * std::cos(yawRad),
        distance * std::cos(pitchRad) * std::sin(yawRad),
        distance * std::sin(pitchRad));

    QMatrix4x4 view;
    view.lookAt(target + offset, target, QVector3D(0, 0, 1));
    return view;
}

void PointCloudWidget::paintGL() {
    if (!glReady) return;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (pointsDirty && points) {
        vbo.bind();
        vbo.allocate(points->data(), static_cast<int>(points->size() * sizeof(vaiop::QtPoint)));
        vbo.release();
        uploadedPointCount = static_cast<int>(points->size());
        pointsDirty = false;
    }

    shaderProgram.bind();
    shaderProgram.setUniformValue("mvp", projection * viewMatrix());

    // Grid
    shaderProgram.setUniformValue("pointSize", 1.0f);
    gridVao.bind();
    glDrawArrays(GL_LINES, 0, gridVertexCount);
    gridVao.release();

    // Points
    if (uploadedPointCount > 0) {
        shaderProgram.setUniformValue("pointSize", static_cast<float>(2.0 * devicePixelRatioF()));
        vao.bind();
        glDrawArrays(GL_POINTS, 0, uploadedPointCount);
        vao.release();
    }

    shaderProgram.release();
}

void PointCloudWidget::mousePressEvent(QMouseEvent* event)
{
    lastMousePos = event->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent* event)
{
    const QPoint delta = event->pos() - lastMousePos;
    lastMousePos = event->pos();

    if (event->buttons() & Qt::LeftButton) {
        yaw -= delta.x() * 0.4f;
        pitch = std::clamp(pitch + delta.y() * 0.4f, -89.0f, 89.0f);
        update();
    } else if (event->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
        // Pan in the view plane, scaled with the distance
        const QMatrix4x4 view = viewMatrix();
        const QVector3D right(view(0, 0), view(0, 1), view(0, 2));
        const QVector3D up(view(1, 0), view(1, 1), view(1, 2));
        const float scale = distance * 0.0015f;
        target += (-right * delta.x() + up * delta.y()) * scale;
        update();
    }
}

void PointCloudWidget::mouseDoubleClickEvent(QMouseEvent*)
{
    resetView();
}

void PointCloudWidget::wheelEvent(QWheelEvent* event)
{
    const float steps = event->angleDelta().y() / 120.0f;
    distance = std::clamp(distance * std::pow(0.85f, steps), 0.5f, 150.0f);
    update();
}
