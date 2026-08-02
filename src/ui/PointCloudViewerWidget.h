#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>

#include "core/Types.h"

// fwd
class MmapPlyReader;
class PointOctree;
namespace open3d { namespace geometry { class PointCloud; } }

/// GPU LOD ������Ⱦ����
/// ˫·��: С�ļ��� Open3D VBO, ���ļ��� mmap + Octree LOD��
class PointCloudViewerWidget
    : public QOpenGLWidget,
      protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT

public:
    explicit PointCloudViewerWidget(QWidget *parent = nullptr);
    ~PointCloudViewerWidget() override;

    // ---- С�ļ�·�� (<100MB) ----
    void setPointCloud(std::shared_ptr<open3d::geometry::PointCloud> cloud);

    // ---- ���ļ�·�� (mmap + octree) ----
    void setMmapSource(std::shared_ptr<MmapPlyReader> reader,
                       std::shared_ptr<PointOctree> octree,
                       const CloudStats &stats);

    void setRenderCount(int count);       // С�ļ�: ֱ�ӵ���
    void setSseThreshold(float threshold); // ���ļ�: SSE ��ֵ
    void setRenderMode(RenderMode mode);

    int  totalPoints()    const;
    int  renderedPoints() const;
    const CloudStats &stats() const { return m_stats; }
    bool isMmapMode()     const { return m_useMmap; }

signals:
    void renderModeChanged(RenderMode mode);
    void vramUsageChanged(int64_t mb);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void setupShaders();
    void uploadSmallCloudVbo();
    void renderSmallCloud();
    void renderMmapLod();

    QMatrix4x4 computeMVP() const;
    void updateVramEstimate();

    QOpenGLShaderProgram   m_program;
    QOpenGLBuffer          m_vbo;
    QOpenGLVertexArrayObject m_vao;

    // С�ļ�
    std::shared_ptr<open3d::geometry::PointCloud> m_cloud;
    int m_totalPoints  = 0;
    int m_vboCapacity  = 0;
    int m_renderCount  = 0;

    // ���ļ�
    std::shared_ptr<MmapPlyReader> m_mmapReader;
    std::shared_ptr<PointOctree>   m_octree;
    bool m_useMmap = false;
    float m_sseThreshold = 0.5f;
    LodConfig m_lodConfig;

    // ����
    CloudStats m_stats;
    RenderMode m_renderMode = RenderMode::DepthColor;
    int  m_pointSize = 2;
    float m_yaw = 30.0f, m_pitch = -20.0f, m_zoom = 1.0f;
    QPoint m_lastMousePos;
    bool   m_dragging = false;
    int64_t m_vramEstimateMB = 0;
    bool m_renderModeChangedByPerformance = false;
    int m_renderedThisFrame = 0;
    std::unordered_map<uint32_t, GLuint> m_nodeVboCache;
};
