#include <Gui/PointCloudWidget.h>
#include <Gui/Theme.h>
#include <QDebug>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace {
    // Ground grid: 1 m cells, GRID_HALF_SIZE m in every direction around the robot
    constexpr int GRID_HALF_SIZE = 10;
    // Without a ground plane: the ground this far below the lidar
    constexpr float DEFAULT_GROUND_Z = -1.0f;
    // The marker lies this far above the ground, so the ground points do not cover it
    constexpr float MARKER_LIFT = 0.03f;
    constexpr float PI = 3.14159265f;

    // Ground coordinates (forward, left, up) of the lidar frame: origin on the ground below the lidar
    struct GroundFrame {
        QVector3D origin, forward, left, up;
        QVector3D at(float f, float l, float u = 0.0f) const { return origin + forward * f + left * l + up * u; }
    };

    GroundFrame groundFrame(const GroundPlane& plane)
    {
        GroundFrame frame;
        if (!plane.valid) {
            frame.up = QVector3D(0, 0, 1);
            frame.forward = QVector3D(1, 0, 0);
            frame.left = QVector3D(0, 1, 0);
            frame.origin = QVector3D(0, 0, DEFAULT_GROUND_Z);
            return frame;
        }
        frame.up = QVector3D(plane.normal.x(), plane.normal.y(), plane.normal.z()).normalized();
        frame.forward = (QVector3D(1, 0, 0) - frame.up.x() * frame.up).normalized();
        frame.left = QVector3D::crossProduct(frame.up, frame.forward);
        frame.origin = -plane.d * frame.up;     // The point of the plane below the lidar
        return frame;
    }
}

PointCloudWidget::PointCloudWidget(QWidget* parent)
    : QOpenGLWidget(parent),
      vbo(QOpenGLBuffer::VertexBuffer),
      gridVbo(QOpenGLBuffer::VertexBuffer),
      markerVbo(QOpenGLBuffer::VertexBuffer),
      quadVbo(QOpenGLBuffer::VertexBuffer),
      blobVbo(QOpenGLBuffer::VertexBuffer)
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
    markerVbo.destroy();
    markerVao.destroy();
    quadVbo.destroy();
    blobVbo.destroy();
    blobVao.destroy();
    shaderProgram.removeAllShaders();
    blobProgram.removeAllShaders();
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
    setupVertexArray(markerVao, markerVbo);
    groundDirty = true;     // Grid and marker are built in paintGL

    glReady = true;
    initializeBlobs();
}

void PointCloudWidget::initializeBlobs()
{
    // The quad corner (-1..1) is offset from the blob centre in view space, so the disc always faces the viewer
    blobProgram.addShaderFromSourceCode(QOpenGLShader::Vertex,
        R"(
        #version 330 core
        layout(location = 0) in vec2 aCorner;
        layout(location = 1) in vec3 aCenter;
        layout(location = 2) in vec3 aColor;
        layout(location = 3) in float aRadius;

        uniform mat4 view;
        uniform mat4 projection;

        out vec2 corner;
        out vec3 blobColor;

        void main() {
            vec4 center = view * vec4(aCenter, 1.0);
            center.xy += aCorner * aRadius;
            gl_Position = projection * center;
            corner = aCorner;
            blobColor = aColor;
        }
        )");
    blobProgram.addShaderFromSourceCode(QOpenGLShader::Fragment,
        R"(
        #version 330 core
        in vec2 corner;
        in vec3 blobColor;
        out vec4 FragColor;

        void main() {
            float r = length(corner);
            if (r > 1.0) discard;
            // Translucent fill, so the points of the blob stay visible, with a solid rim
            float rim = smoothstep(0.90, 0.95, r);
            FragColor = vec4(blobColor * mix(0.8, 1.15, rim), mix(0.22, 0.95, rim));
        }
        )");
    if (!blobProgram.link()) {
        qWarning() << "Blob shader link error:" << blobProgram.log();
        return;
    }

    blobVao.create();
    blobVao.bind();

    const GLfloat corners[] = {-1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};
    quadVbo.create();
    quadVbo.bind();
    quadVbo.allocate(corners, sizeof(corners));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);

    blobVbo.create();
    blobVbo.bind();
    auto instanceAttribute = [this](GLuint location, GLint components, size_t offset) {
        glEnableVertexAttribArray(location);
        glVertexAttribPointer(location, components, GL_FLOAT, GL_FALSE, sizeof(Blob), reinterpret_cast<const void*>(offset));
        glVertexAttribDivisor(location, 1);
    };
    instanceAttribute(1, 3, offsetof(Blob, center));
    instanceAttribute(2, 3, offsetof(Blob, color));
    instanceAttribute(3, 1, offsetof(Blob, radius));

    blobVao.release();
    blobVbo.release();
    blobsReady = true;
}

void PointCloudWidget::setGround(const GroundPlane& plane)
{
    ground = plane;
    groundDirty = true;
    update();
}

void PointCloudWidget::uploadGround()
{
    const GroundFrame frame = groundFrame(ground);

    // Grid on the ground with the forward (red) and left (green) axes through the robot
    std::vector<vaiop::QtPoint> grid;
    const QVector3D gridColor(0.18f, 0.21f, 0.26f);
    for (int i = -GRID_HALF_SIZE; i <= GRID_HALF_SIZE; ++i) {
        const QVector3D colorForward = (i == 0) ? QVector3D(0.55f, 0.25f, 0.25f) : gridColor;
        const QVector3D colorLeft = (i == 0) ? QVector3D(0.25f, 0.5f, 0.3f) : gridColor;
        grid.push_back({frame.at(-GRID_HALF_SIZE, i), colorLeft});
        grid.push_back({frame.at(GRID_HALF_SIZE, i), colorLeft});
        grid.push_back({frame.at(i, -GRID_HALF_SIZE), colorForward});
        grid.push_back({frame.at(i, GRID_HALF_SIZE), colorForward});
    }
    gridVertexCount = static_cast<int>(grid.size());
    gridVbo.bind();
    gridVbo.allocate(grid.data(), static_cast<int>(grid.size() * sizeof(vaiop::QtPoint)));
    gridVbo.release();

    // Robot marker in the ILVO green: arrow pointing forward and a ring on the ground, a line up to the lidar
    const QVector3D green(Theme::ilvoGreen.redF(), Theme::ilvoGreen.greenF(), Theme::ilvoGreen.blueF());
    const QVector3D darkGreen = green * 0.6f;
    std::vector<vaiop::QtPoint> marker;
    auto triangle = [&](QVector3D a, QVector3D b, QVector3D c, QVector3D color) {
        marker.push_back({a, color});
        marker.push_back({b, color});
        marker.push_back({c, color});
    };
    auto ground2 = [&frame](float f, float l) { return frame.at(f, l, MARKER_LIFT); };
    // Arrow: shaft from 0.5 m behind to 0.35 m ahead of the lidar, head up to 0.9 m ahead
    triangle(ground2(-0.5f, -0.12f), ground2(0.35f, -0.12f), ground2(0.35f, 0.12f), green);
    triangle(ground2(-0.5f, -0.12f), ground2(0.35f, 0.12f), ground2(-0.5f, 0.12f), green);
    triangle(ground2(0.35f, -0.35f), ground2(0.9f, 0.0f), ground2(0.35f, 0.35f), green);
    // Ring of 0.6 to 0.7 m around the lidar
    constexpr int SEGMENTS = 48;
    for (int i = 0; i < SEGMENTS; ++i) {
        const float a0 = 2.0f * PI * i / SEGMENTS;
        const float a1 = 2.0f * PI * (i + 1) / SEGMENTS;
        const QVector3D in0 = ground2(0.6f * std::cos(a0), 0.6f * std::sin(a0));
        const QVector3D out0 = ground2(0.7f * std::cos(a0), 0.7f * std::sin(a0));
        const QVector3D in1 = ground2(0.6f * std::cos(a1), 0.6f * std::sin(a1));
        const QVector3D out1 = ground2(0.7f * std::cos(a1), 0.7f * std::sin(a1));
        triangle(in0, out0, out1, darkGreen);
        triangle(in0, out1, in1, darkGreen);
    }
    markerTriangleVertices = static_cast<int>(marker.size());
    // Line from the ground up to the lidar, and small axes at the lidar (forward red, left green, up blue)
    marker.push_back({frame.origin, green});
    marker.push_back({QVector3D(0, 0, 0), green});
    constexpr float AXIS = 0.3f;
    marker.push_back({QVector3D(0, 0, 0), QVector3D(0.9f, 0.3f, 0.3f)});
    marker.push_back({frame.forward * AXIS, QVector3D(0.9f, 0.3f, 0.3f)});
    marker.push_back({QVector3D(0, 0, 0), QVector3D(0.3f, 0.85f, 0.4f)});
    marker.push_back({frame.left * AXIS, QVector3D(0.3f, 0.85f, 0.4f)});
    marker.push_back({QVector3D(0, 0, 0), QVector3D(0.35f, 0.55f, 1.0f)});
    marker.push_back({frame.up * AXIS, QVector3D(0.35f, 0.55f, 1.0f)});
    markerLineVertices = static_cast<int>(marker.size()) - markerTriangleVertices;
    markerVbo.bind();
    markerVbo.allocate(marker.data(), static_cast<int>(marker.size() * sizeof(vaiop::QtPoint)));
    markerVbo.release();
    groundDirty = false;
}

void PointCloudWidget::setBlobs(std::vector<Blob> newBlobs)
{
    blobs = std::move(newBlobs);
    update();
}


void PointCloudWidget::setPointCloud(std::shared_ptr<const std::vector<vaiop::QtPoint>> newPoints)
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

    if (groundDirty) {
        uploadGround();
    }

    shaderProgram.bind();
    shaderProgram.setUniformValue("mvp", projection * viewMatrix());

    // Grid
    shaderProgram.setUniformValue("pointSize", 1.0f);
    gridVao.bind();
    glDrawArrays(GL_LINES, 0, gridVertexCount);
    gridVao.release();

    // Robot marker
    markerVao.bind();
    glDrawArrays(GL_TRIANGLES, 0, markerTriangleVertices);
    glDrawArrays(GL_LINES, markerTriangleVertices, markerLineVertices);
    markerVao.release();

    // Points
    if (uploadedPointCount > 0) {
        shaderProgram.setUniformValue("pointSize", static_cast<float>(2.0 * devicePixelRatioF()));
        vao.bind();
        glDrawArrays(GL_POINTS, 0, uploadedPointCount);
        vao.release();
    }

    shaderProgram.release();

    // Blobs: translucent, so drawn back to front (sorted for the current view) without writing depth
    if (blobsReady && !blobs.empty()) {
        const QMatrix4x4 view = viewMatrix();
        std::vector<Blob> sorted = blobs;
        std::sort(sorted.begin(), sorted.end(), [&view](const Blob& a, const Blob& b) {
            return view.map(a.center).z() < view.map(b.center).z();
        });
        blobVbo.bind();
        blobVbo.allocate(sorted.data(), static_cast<int>(sorted.size() * sizeof(Blob)));
        blobVbo.release();

        glDepthMask(GL_FALSE);
        blobProgram.bind();
        blobProgram.setUniformValue("view", view);
        blobProgram.setUniformValue("projection", projection);
        blobVao.bind();
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(sorted.size()));
        blobVao.release();
        blobProgram.release();
        glDepthMask(GL_TRUE);
    }
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
