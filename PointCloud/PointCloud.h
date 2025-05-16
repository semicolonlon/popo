#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QVBoxLayout>
#include <QPushButton>
#include <QApplication>
#include <QFile>
#include <QDebug>
#include <QTextStream>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram> 
#include <QOpenGLBuffer> 
#include <QOpenGLVertexArrayObject>

#include <QPinchGesture>

#include <SDL3/SDL.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>

#include <QKeyEvent>                 
#include <QMatrix4x4> 
#include <vector>

class PointCloud : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit PointCloud(QWidget* parent = nullptr);
    ~PointCloud();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void loadPointCloud(const std::string& path);
    void moveCamera();
    int getLODLevelForDistance(QVector3D chunkPos);

    SDL_Gamepad* controller = nullptr;

    QOpenGLShaderProgram* m_shaderProgram;
    QPoint m_lastMousePosition;
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_modelViewProjection;

    //点群のパスはここに
    const std::string PATH = "ここに必ず点郡ファイルへの絶対パスを記述して";
    //背景色
    QColor backgroundColor = QColor(0.1f, 0.1f, 0.1f, 1.0f);
    // カメラ座標
    QVector3D m_cameraPos = QVector3D(0.0f, 0.0f, 3.0f);
    // カメラアングル
    float m_yaw = 180.0f;
    float m_pitch = 0.0f;
    QVector3D m_cameraFront = m_cameraPos + QVector3D(sin(m_yaw), 0.0f, cos(m_yaw));
    // カメラの上方向(変更するな)
    const QVector3D m_cameraUp = QVector3D(0.0f, 1.0f, 0.0f);

    // チャンクのクラス(チャンクラス)
    class Chunk {
    public:
        QVector3D center;
        std::vector<QOpenGLVertexArrayObject*> vaos;
        std::vector<int> pointNum;

        ~Chunk() {
            center = QVector3D(0.0f, 0.0f, 0.0f);
        }
    };
    
    std::vector<Chunk> chunks;
    // チャンクサイズ(^3)
    float resolution = 10.0f;
};

#endif // POINTCLOUD_H
