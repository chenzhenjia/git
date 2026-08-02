#include "PointCloudViewerWidget.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QMouseEvent>
#include <QWheelEvent>

#include <open3d/geometry/PointCloud.h>

#include "core/MmapPlyReader.h"
#include "core/Octree.h"
#include "core/PointCloudLoader.h"

// ---- GLSL ��ɫ�� ----
static const char *kVtx = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uMVP;
uniform float uPointSize;
out vec3 vColor;
void main(){gl_Position=uMVP*vec4(aPos,1.0);gl_PointSize=uPointSize;vColor=aColor;}
)";

static const char *kFrag = R"(
#version 330 core
in vec3 vColor;out vec4 fragColor;
void main(){vec2 c=gl_PointCoord-vec2(.5);if(dot(c,c)>.25)discard;fragColor=vec4(vColor,1.0);}
)";

static QVector3D depthCol(double t) {
    t=std::clamp(t,0.0,1.0);
    QColor c=QColor::fromHsv(static_cast<int>((1.0-t)*240.0),220,235);
    return QVector3D(c.redF(),c.greenF(),c.blueF());
}

// ===================================================================
PointCloudViewerWidget::PointCloudViewerWidget(QWidget *p):QOpenGLWidget(p){
    setMinimumSize(320,240);setFocusPolicy(Qt::StrongFocus);
}
PointCloudViewerWidget::~PointCloudViewerWidget(){
    makeCurrent();m_vbo.destroy();m_vao.destroy();
    for(auto&[k,v]:m_nodeVboCache){glDeleteBuffers(1,&v);}m_nodeVboCache.clear();
    doneCurrent();
}

// ===================================================================
//  Small-file path
// ===================================================================
void PointCloudViewerWidget::setPointCloud(
    std::shared_ptr<open3d::geometry::PointCloud> cloud){
    m_useMmap=false;m_mmapReader.reset();m_octree.reset();
    m_cloud=std::move(cloud);
    if(!m_cloud||m_cloud->points_.empty()){m_totalPoints=0;m_renderCount=0;update();return;}
    m_totalPoints=static_cast<int>(m_cloud->points_.size());
    m_renderCount=std::min(m_totalPoints,50000);
    m_stats=PointCloudLoader::computeStats(*m_cloud);
    m_yaw=30;m_pitch=-20;m_zoom=1;
    makeCurrent();uploadSmallCloudVbo();doneCurrent();
    updateVramEstimate();update();
}

// ===================================================================
//  Large-file mmap path
// ===================================================================
void PointCloudViewerWidget::setMmapSource(
    std::shared_ptr<MmapPlyReader> reader,
    std::shared_ptr<PointOctree> octree,
    const CloudStats &stats){
    m_useMmap=true;m_cloud.reset();
    for(auto&[k,v]:m_nodeVboCache){glDeleteBuffers(1,&v);}m_nodeVboCache.clear();
    m_mmapReader=std::move(reader);
    m_octree=std::move(octree);
    m_stats=stats;
    m_totalPoints=static_cast<int>(stats.total_points);
    m_renderCount=50000;
    m_yaw=30;m_pitch=-20;m_zoom=1;
    // Octree already built; no VBO pre-upload needed
    makeCurrent();doneCurrent();
    updateVramEstimate();update();
}

void PointCloudViewerWidget::setSseThreshold(float t){
    m_sseThreshold=std::clamp(t,0.01f,10.0f);update();
}

void PointCloudViewerWidget::setRenderCount(int count){
    m_renderCount=std::clamp(count,1000,m_totalPoints);
    if(m_renderCount>200000&&m_renderMode!=RenderMode::FlatColor){
        m_renderModeChangedByPerformance=true;setRenderMode(RenderMode::FlatColor);
    }else if(m_renderCount<=200000&&m_renderModeChangedByPerformance){
        m_renderModeChangedByPerformance=false;setRenderMode(RenderMode::DepthColor);
    }
    update();
}

void PointCloudViewerWidget::setRenderMode(RenderMode m){
    if(m_renderMode==m)return;m_renderMode=m;
    emit renderModeChanged(m);
    if(!m_useMmap&&m_totalPoints>0){makeCurrent();uploadSmallCloudVbo();doneCurrent();}
    update();
}

int PointCloudViewerWidget::totalPoints() const {return m_totalPoints;}
int PointCloudViewerWidget::renderedPoints() const {return m_useMmap?m_renderedThisFrame:m_renderCount;}

// ===================================================================
//  OpenGL
// ===================================================================
void PointCloudViewerWidget::initializeGL(){
    initializeOpenGLFunctions();
    setupShaders();
    m_vao.create();m_vbo.create();
    glClearColor(.086f,.094f,.110f,1);glEnable(GL_PROGRAM_POINT_SIZE);
}

void PointCloudViewerWidget::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    if(m_totalPoints==0)return;
    if(m_useMmap)renderMmapLod();
    else renderSmallCloud();
}

void PointCloudViewerWidget::resizeGL(int w,int h){glViewport(0,0,w,h);}

// ---- mouse ----
void PointCloudViewerWidget::mousePressEvent(QMouseEvent*e){if(e->button()==Qt::LeftButton){m_dragging=true;m_lastMousePos=e->pos();}}
void PointCloudViewerWidget::mouseMoveEvent(QMouseEvent*e){
    if(!m_dragging||m_totalPoints==0)return;
    QPoint d=e->pos()-m_lastMousePos;m_lastMousePos=e->pos();
    m_yaw=fmodf(m_yaw+d.x()*.3f,360.f);m_pitch=std::clamp(m_pitch-d.y()*.3f,-89.f,89.f);update();
}
void PointCloudViewerWidget::mouseReleaseEvent(QMouseEvent*e){if(e->button()==Qt::LeftButton)m_dragging=false;}
void PointCloudViewerWidget::wheelEvent(QWheelEvent*e){
    if(m_totalPoints==0)return;
    float f=e->angleDelta().y()>0?1.12f:1.f/1.12f;m_zoom=std::clamp(m_zoom*f,.05f,15.f);update();
}

// ===================================================================
//  private: small cloud rendering
// ===================================================================
void PointCloudViewerWidget::setupShaders(){
    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,kVtx);
    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment,kFrag);
    m_program.link();m_program.bind();
}

void PointCloudViewerWidget::uploadSmallCloudVbo(){
    if(!m_cloud||m_totalPoints<=0)return;
    const auto&pts=m_cloud->points_;int n=std::min(m_totalPoints,static_cast<int>(pts.size()));
    auto lo=m_cloud->GetMinBound(),hi=m_cloud->GetMaxBound();
    auto center=(lo+hi)*.5;double sc=(hi-lo).maxCoeff();if(sc<1e-9)sc=1;
    std::vector<float>buf(n*6);
    bool useVC=m_renderMode==RenderMode::VertexColor&&m_cloud->HasColors()&&(int)m_cloud->colors_.size()==n;
    for(int i=0;i<n;++i){
        auto p=pts[i];float*d=buf.data()+i*6;
        d[0]=(p.x()-center.x())/sc;d[1]=(p.y()-center.y())/sc;d[2]=(p.z()-center.z())/sc;
        QVector3D col;
        if(useVC){auto&c=m_cloud->colors_[i];col=QVector3D(std::clamp(c.x(),0.,1.),std::clamp(c.y(),0.,1.),std::clamp(c.z(),0.,1.));}
        else if(m_renderMode==RenderMode::FlatColor)col=QVector3D(.85,.85,.85);
        else col=depthCol(d[1]+.5);
        d[3]=col.x();d[4]=col.y();d[5]=col.z();
    }
    m_vbo.bind();m_vbo.allocate(buf.data(),n*6*sizeof(float));m_vboCapacity=n;
    m_vao.bind();m_program.bind();
    m_program.enableAttributeArray(0);m_program.setAttributeBuffer(0,GL_FLOAT,0,3,6*sizeof(float));
    m_program.enableAttributeArray(1);m_program.setAttributeBuffer(1,GL_FLOAT,12,3,6*sizeof(float));
    m_vao.release();m_vbo.release();
}

void PointCloudViewerWidget::renderSmallCloud(){
    m_program.bind();m_vao.bind();
    m_program.setUniformValue("uMVP",computeMVP());
    m_program.setUniformValue("uPointSize",m_pointSize*m_zoom);
    glDrawArrays(GL_POINTS,0,m_renderCount);
    m_vao.release();m_program.release();
}

// ===================================================================
//  private: mmap LOD rendering
// ===================================================================
static float heightNormForColor(float yVal,float yMin,float yMax){
    float rng=yMax-yMin;if(rng<1e-6f)return .5f;return std::clamp((yVal-yMin)/rng,0.f,1.f);
}

void PointCloudViewerWidget::renderMmapLod(){
    if(!m_mmapReader||!m_octree)return;

    QMatrix4x4 mvp=computeMVP();
    const float mvpArr[16]={mvp(0,0),mvp(1,0),mvp(2,0),mvp(3,0),
                             mvp(0,1),mvp(1,1),mvp(2,1),mvp(3,1),
                             mvp(0,2),mvp(1,2),mvp(2,2),mvp(3,2),
                             mvp(0,3),mvp(1,3),mvp(2,3),mvp(3,3)};

    std::vector<std::pair<uint32_t,int>> vis;
    m_octree->query(mvpArr,m_sseThreshold,vis);
    if(vis.empty())return;

    int drawnThisFrame=0;
    m_program.bind();m_vao.bind();

    // 节点列表发生变化时才重新收集可见节点
    bool nodesChanged=false;
    for(const auto&[nodeIdx,lod]:vis){
        if(!m_nodeVboCache.count(nodeIdx)){nodesChanged=true;break;}
    }
    if(nodesChanged&&m_nodeVboCache.size()>64){
        for(auto&[k,v]:m_nodeVboCache){glDeleteBuffers(1,&v);}
        m_nodeVboCache.clear();
    }

    for(const auto&[nodeIdx,lod]:vis){
        const OctreeNode&nd=m_octree->node(nodeIdx);
        uint64_t cnt=std::min<uint64_t>(nd.pointCount,150000);
        if(cnt==0)continue;
        uint64_t stride=std::max<uint64_t>(1,cnt/30000);
        uint64_t nDraw=(cnt+stride-1)/stride;
        if(nDraw==0)continue;

        // 使用缓存或创建新 VBO
        auto cacheIt=m_nodeVboCache.find(nodeIdx);
        if(cacheIt!=m_nodeVboCache.end()){
            glBindBuffer(GL_ARRAY_BUFFER,cacheIt->second);
            m_program.setAttributeBuffer(0,GL_FLOAT,0,3,6*sizeof(float));
            m_program.setAttributeBuffer(1,GL_FLOAT,12,3,6*sizeof(float));
            m_program.setUniformValue("uMVP",mvp);
            m_program.setUniformValue("uPointSize",m_pointSize*m_zoom);
            glDrawArrays(GL_POINTS,0,static_cast<GLsizei>(nDraw));
            drawnThisFrame+=static_cast<int>(nDraw);
            continue;
        }

        // 首次: 构建 VBO 并缓存
        std::vector<float>buf(nDraw*6);
        float yMin=nd.aabbMin[1],yMax=nd.aabbMax[1];
        uint64_t end=nd.firstPoint+cnt,out=0;
        for(uint64_t i=nd.firstPoint;i<end&&out<nDraw;i+=stride,++out){
            float xyz[3],rgb[3];m_mmapReader->readPoint(i,xyz,rgb);
            float*d=buf.data()+out*6;
            d[0]=xyz[0];d[1]=xyz[1];d[2]=xyz[2];
            if(m_renderMode==RenderMode::DepthColor){
                float t=heightNormForColor(xyz[1],yMin,yMax);
                QVector3D c=depthCol(t);
                d[3]=c.x();d[4]=c.y();d[5]=c.z();
            }else if(m_renderMode==RenderMode::FlatColor){
                d[3]=d[4]=d[5]=.85f;
            }else{
                d[3]=rgb[0];d[4]=rgb[1];d[5]=rgb[2];
            }
        }
        if(out==0)continue;

        GLuint vbo=0;glGenBuffers(1,&vbo);
        glBindBuffer(GL_ARRAY_BUFFER,vbo);
        glBufferData(GL_ARRAY_BUFFER,out*6*sizeof(float),buf.data(),GL_STATIC_DRAW);
        m_nodeVboCache[nodeIdx]=vbo;

        m_program.setAttributeBuffer(0,GL_FLOAT,0,3,6*sizeof(float));
        m_program.setAttributeBuffer(1,GL_FLOAT,12,3,6*sizeof(float));
        m_program.setUniformValue("uMVP",mvp);
        m_program.setUniformValue("uPointSize",m_pointSize*m_zoom);
        glDrawArrays(GL_POINTS,0,static_cast<GLsizei>(out));
        drawnThisFrame+=static_cast<int>(out);
    }
    m_vao.release();m_program.release();
    m_renderedThisFrame=drawnThisFrame;
}

// ===================================================================
//  common helpers
// ===================================================================
QMatrix4x4 PointCloudViewerWidget::computeMVP() const {
    QMatrix4x4 proj;proj.perspective(45.f,float(width())/std::max(height(),1),.001f,100.f);
    QMatrix4x4 view;view.translate(0,0,-3.f/m_zoom);view.rotate(m_pitch,1,0,0);view.rotate(m_yaw,0,0,1);
    return proj*view;
}

void PointCloudViewerWidget::updateVramEstimate(){
    int64_t bytes=m_totalPoints*6*sizeof(float);
    m_vramEstimateMB=bytes/(1024*1024);emit vramUsageChanged(m_vramEstimateMB);
}