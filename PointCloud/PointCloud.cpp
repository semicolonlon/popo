#include "PointCloud.h"
#include "setupController.h"

//コンストラクター 明示的な引数が必要なものはここで行う
PointCloud::PointCloud(QWidget* parent)
    : QOpenGLWidget(parent),
    m_shaderProgram(nullptr)
{

}

PointCloud::~PointCloud() {
    makeCurrent();
    chunks.clear();
    delete m_shaderProgram;
    doneCurrent();
}

void PointCloud::initializeGL()
{
    initializeOpenGLFunctions();

    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    m_shaderProgram = new QOpenGLShaderProgram(this);
    m_shaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex,
        "#version 330 core\n"
        "layout(location = 0) in vec3 vertexPosition;\n"
        "layout(location = 1) in vec3 vertexColor;\n"
        "uniform mat4 modelViewProjection;\n"
        "uniform float pointSize;\n"
        "out vec3 fragColor;\n"
        "void main() {\n"
        "    fragColor = vertexColor;\n"
        "    gl_Position = modelViewProjection * vec4(vertexPosition, 1.0);\n"
        "    gl_PointSize = pointSize;\n"
        "}");
    m_shaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment,
        "#version 330 core\n"
        "in vec3 fragColor;\n"
        "out vec4 outColor;\n"
        "void main() {\n"
        "    outColor = vec4(fragColor, 1.0);\n"
        "}");
    m_shaderProgram->link();

    glClearColor(backgroundColor.redF(), backgroundColor.greenF(), backgroundColor.blueF(), backgroundColor.alphaF());

    loadPointCloud(PATH);
     
    setupController(&controller);
}

void PointCloud::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

    QMatrix4x4 projection;
    float aspect = static_cast<float>(w) / static_cast<float>(h);
    projection.perspective(45.0f, aspect, 0.001f, 10.0f);

    QMatrix4x4 model;
    model.setToIdentity();

    m_modelViewProjection = projection * m_viewMatrix * model;

    m_shaderProgram->bind();
    m_shaderProgram->setUniformValue("modelViewProjection", m_modelViewProjection);
    m_shaderProgram->release();
}

void PointCloud::paintGL()
{
    moveCamera();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_shaderProgram->bind();

    m_shaderProgram->setUniformValue("modelViewProjection", m_modelViewProjection);

    for (int i = 0; i < chunks.size(); ++i)
    {
        auto chunk = chunks[i];
 
        const int lodLevel = getLODLevelForDistance(chunk.center);

        if (lodLevel < 4) {
            auto vao = chunk.vaos[lodLevel];
            vao->bind();
            glDrawArrays(GL_POINTS, 0, chunk.pointNum[lodLevel]);
            vao->release();
        }
    }

    m_shaderProgram->release();
    update();
}

int PointCloud::getLODLevelForDistance(QVector3D chunkPos)
{
    int lodLevel = 4;
    float distance = (chunkPos - m_cameraPos).length();
    QVector3D toChunk = (chunkPos - m_cameraPos).normalized();
    float angle = acos(QVector3D::dotProduct(m_cameraFront.normalized(), toChunk));

    if (distance < 50.0f) {
        lodLevel = 0; // 最高精度のLOD
        m_shaderProgram->setUniformValue("pointSize", 10.0f);
    }
    else if (distance < 100.0f && angle < qDegreesToRadians(50.0f)) {
        lodLevel = 1; // 高精度のLOD
        m_shaderProgram->setUniformValue("pointSize", 5.0f);
    }
    else if (distance < 150.0f && angle < qDegreesToRadians(45.0f)) {
        lodLevel = 2; // 中精度のLOD
        m_shaderProgram->setUniformValue("pointSize", 2.0f);
    }
    else if (distance < 300.0f && angle < qDegreesToRadians(40.0f)) {
        lodLevel = 3; // 最低精度LOD
        m_shaderProgram->setUniformValue("pointSize",1.0f);
    }
    else {
        lodLevel = 4; // 描画のスキップ
    }

    return lodLevel;
}

void PointCloud::loadPointCloud(const std::string& path)
{
    if (context()) {
        makeCurrent();
    }
    else {
        qDebug() << "No OpenGL context available!";
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(path, *cloud) == -1) {
        qDebug() << "Error loading point cloud!";
        return;
    }

    QMatrix4x4 rotation;
    rotation.rotate(-90, 1, 0, 0);
    QVector3D minPt(FLT_MAX, FLT_MAX, FLT_MAX);
    QVector3D maxPt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (auto& p : *cloud) {
        QVector3D pt = rotation * QVector3D(p.x, p.y, p.z);
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();

        minPt.setX(std::min(minPt.x(), p.x));
        minPt.setY(std::min(minPt.y(), p.y));
        minPt.setZ(std::min(minPt.z(), p.z));
        maxPt.setX(std::max(maxPt.x(), p.x));
        maxPt.setY(std::max(maxPt.y(), p.y));
        maxPt.setZ(std::max(maxPt.z(), p.z));
    }

    const QVector3D center = (maxPt + minPt) * 0.5f;
    for (auto& p : *cloud) {
        p.x -= center.x();
        p.y -= center.y();
        p.z -= center.z();
    }

    pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree(10.0f);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    for (auto it = octree.leaf_breadth_begin(); it != octree.leaf_breadth_end(); ++it)
    {
        Chunk chunk;
        std::vector<int> indices;
        it.getLeafContainer().getPointIndices(indices);

        Eigen::Vector3f center(0, 0, 0);
        for (int idx : indices) {
            const auto& p = cloud->points[idx];
            center += Eigen::Vector3f(p.x, p.y, p.z);
        }
        chunk.center = QVector3D(center.x(), center.y(), center.z()) / static_cast<float>(indices.size());

        for (int lod = 0; lod < 4; ++lod)
        {
            QOpenGLVertexArrayObject* vao = new QOpenGLVertexArrayObject();
            QOpenGLBuffer* vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);

            std::vector<float> data;
            const int step = 1 << lod;
            for (size_t i = 0; i < indices.size(); i += step) {
                const auto& p = cloud->points[indices[i]];
                data.insert(data.end(), { p.x, p.y, p.z, p.r / 255.0f, p.g / 255.0f, p.b / 255.0f });
            }

            vao->create();
            vao->bind();
            m_shaderProgram->bind();
            vbo->create();
            vbo->bind();
            vbo->allocate(data.data(), static_cast<int>(data.size() * sizeof(float)));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
            m_shaderProgram->release();
            vbo->release();
            vao->release();
            chunk.vaos.push_back(vao);
            chunk.pointNum.push_back(data.size()/6);
        }
        chunks.push_back(chunk);
    }
}

void PointCloud::moveCamera() {
    if (!controller) return;

    float moveSpeed = 0.05f;            // カメラ移動速度
    const float lookSensitivity = 1.0f; // 視点移動の感度
    const int deadZone = 8000;          // デッドゾーン

    //左
    int LaxisX = SDL_GetGamepadAxis(controller, SDL_GAMEPAD_AXIS_LEFTX);
    int LaxisY = SDL_GetGamepadAxis(controller, SDL_GAMEPAD_AXIS_LEFTY);

    if (abs(LaxisX) > deadZone || abs(LaxisY) > deadZone) {
        float offsetX = LaxisX / 32767.0f * lookSensitivity;
        float offsetY = LaxisY / 32767.0f * lookSensitivity;

        m_yaw -= offsetX;
        m_pitch -= offsetY;

        if (m_pitch > 89.0f)  m_pitch = 89.0f;
        if (m_pitch < -89.0f) m_pitch = -89.0f;

        // カメラの向き
        QVector3D direction;
        direction.setX(cos(qDegreesToRadians(m_pitch)) * sin(qDegreesToRadians(m_yaw)));
        direction.setY(sin(qDegreesToRadians(m_pitch)));
        direction.setZ(cos(qDegreesToRadians(m_pitch)) * cos(qDegreesToRadians(m_yaw)));
        m_cameraFront = direction.normalized();
    }

    //右
    int RaxisX = SDL_GetGamepadAxis(controller, SDL_GAMEPAD_AXIS_RIGHTX);
    int RaxisY = SDL_GetGamepadAxis(controller, SDL_GAMEPAD_AXIS_RIGHTY);

    if (abs(RaxisX) > deadZone || abs(RaxisY) > deadZone) {
        float normalizedX = RaxisX / 32767.0f;
        float normalizedY = RaxisY / 32767.0f;

        QVector3D right = QVector3D::crossProduct(m_cameraFront, m_cameraUp).normalized();
        QVector3D forward = m_cameraFront;

        m_cameraPos += right * normalizedX * moveSpeed;
        m_cameraPos -= forward * normalizedY * moveSpeed;
    }

    //この処理自体はいらないけどこれをけしたら動かない
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN) {
            switch (event.gbutton.button) {
            case SDL_GAMEPAD_BUTTON_DPAD_UP:
                m_cameraPos += m_cameraUp * moveSpeed;
                break;
            case SDL_GAMEPAD_BUTTON_DPAD_DOWN:
                m_cameraPos -= m_cameraUp * moveSpeed;
                break;
            case SDL_GAMEPAD_BUTTON_DPAD_LEFT:
                m_cameraPos -= QVector3D::crossProduct(m_cameraFront, m_cameraUp).normalized() * moveSpeed;
                break;
            case SDL_GAMEPAD_BUTTON_DPAD_RIGHT:
                m_cameraPos += QVector3D::crossProduct(m_cameraFront, m_cameraUp).normalized() * moveSpeed;
                break;
            }
        }
    }

    m_viewMatrix.setToIdentity(); //initialize
    m_viewMatrix.lookAt(m_cameraPos, m_cameraPos + m_cameraFront, m_cameraUp);

    QMatrix4x4 model;
    model.setToIdentity();

    QMatrix4x4 projection;
    float aspect = static_cast<float>(width()) / static_cast<float>(height());
    projection.perspective(45.0f, aspect, 0.1f, 1000.0f);

    m_modelViewProjection = projection * m_viewMatrix * model;
}