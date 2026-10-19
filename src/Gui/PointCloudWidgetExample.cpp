#include <Gui/PointCloudWidgetExample.h>
#include <QTimer>
#include <QtMath>
#include <cmath>

PointCloudWidgetExample::PointCloudWidgetExample(QWidget* parent)
    : QOpenGLWidget(parent),
      vbo(QOpenGLBuffer::VertexBuffer)
{
    setFocusPolicy(Qt::StrongFocus);

    // Animation timer for rotation
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        angle += 1.0f;
        if (angle >= 360.0f) angle -= 360.0f;
        update();
    });
    timer->start(16); // ~60 FPS
}

PointCloudWidgetExample::~PointCloudWidgetExample() {
    makeCurrent();
    vbo.destroy();
    vao.destroy();
    shaderProgram.release();
    shaderProgram.removeAllShaders();
    doneCurrent();
}

void PointCloudWidgetExample::generatePoints(int numPoints) {
    points.clear();
    points.reserve(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float theta = 2.0f * M_PI * float(rand()) / RAND_MAX;
        float phi = acos(1.0f - 2.0f * float(rand()) / RAND_MAX);
        float radius = 1.0f + 0.3f * float(rand()) / RAND_MAX;

        Point p;
        p.position.setX(radius * sin(phi) * cos(theta));
        p.position.setY(radius * sin(phi) * sin(theta));
        p.position.setZ(radius * cos(phi));

        // Color based on height
        p.color.setX((p.position.z() + 1.3f) / 2.6f);
        p.color.setY(0.5f + 0.5f * sin(p.position.x()));
        p.color.setZ(0.5f + 0.5f * cos(p.position.y()));

        points.push_back(p);
    }
}

void PointCloudWidgetExample::initializeGL() {
    initializeOpenGLFunctions();

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    // Compile shaders
    shaderProgram.addShaderFromSourceCode(QOpenGLShader::Vertex,
        R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;

        uniform mat4 mvp;

        out vec3 vertexColor;

        void main() {
            gl_Position = mvp * vec4(aPos, 1.0);
            gl_PointSize = 3.0;
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

    // Generate point cloud data
    generatePoints(5000);

    // Setup VAO & VBO
    vao.create();
    vao.bind();

    vbo.create();
    vbo.bind();
    vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo.allocate(points.data(), static_cast<int>(points.size() * sizeof(Point)));

    // Setup vertex attributes
    shaderProgram.bind();

    // position attribute
    shaderProgram.enableAttributeArray(0);
    shaderProgram.setAttributeBuffer(0, GL_FLOAT, offsetof(Point, position), 3, sizeof(Point));

    // color attribute
    shaderProgram.enableAttributeArray(1);
    shaderProgram.setAttributeBuffer(1, GL_FLOAT, offsetof(Point, color), 3, sizeof(Point));

    shaderProgram.release();
    vao.release();
    vbo.release();
}

void PointCloudWidgetExample::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    projection.setToIdentity();
    projection.perspective(45.0f, float(w) / float(h), 0.1f, 100.0f);
}

void PointCloudWidgetExample::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (points.empty()) return;

    shaderProgram.bind();
    vao.bind();

    // Simple rotating camera around origin
    QMatrix4x4 view;
    float camX = 3.0f * qCos(qDegreesToRadians(angle));
    float camZ = 3.0f * qSin(qDegreesToRadians(angle));
    float camY = 1.0f;
    QVector3D cameraPos(camX, camY, camZ);
    QVector3D target(0, 0, 0);
    QVector3D up(0, 1, 0);

    view.lookAt(cameraPos, target, up);

    QMatrix4x4 mvp = projection * view;

    shaderProgram.setUniformValue("mvp", mvp);

    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points.size()));

    vao.release();
    shaderProgram.release();
}
