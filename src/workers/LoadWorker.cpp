#include "LoadWorker.h"

#include <chrono>
#include <functional>

#include <QDebug>
#include <QThread>

#include <open3d/geometry/PointCloud.h>

#include "core/MmapPlyReader.h"
#include "core/Octree.h"
#include "core/PointCloudLoader.h"

LoadWorker::LoadWorker(QObject *parent):QObject(parent){}
LoadWorker::~LoadWorker(){cancel();}

// ===================================================================
// 小文件: Open3D path
// ===================================================================
void LoadWorker::startLoad(const QString &path, const LoadConfig &config){
    m_cancelled.store(false,std::memory_order_relaxed);
    emit loadStarted();emit progressUpdated(0);

    auto t0=std::chrono::steady_clock::now();
    // toLocal8Bit: compatible with Windows Chinese paths
    std::string localPath=path.toLocal8Bit().toStdString();
    std::string err;auto cloud=PointCloudLoader::load(localPath,config,err);

    if(m_cancelled.load()){emit loadFailed("Cancelled");return;}
    if(!cloud){emit loadFailed(QString::fromStdString(err));return;}
    emit progressUpdated(30);

    CloudStats s=PointCloudLoader::computeStats(*cloud);
    auto t1=std::chrono::steady_clock::now();
    s.load_time_ms=std::chrono::duration<double,std::milli>(t1-t0).count();
    emit statsReady(s);emit progressUpdated(60);

    if(config.auto_preview&&s.total_points>config.auto_preview_threshold){
        auto prev=PointCloudLoader::downsampleTo(cloud,config.preview_target_points);
        if(prev)emit previewReady(prev);
    }
    if(m_cancelled.load()){emit loadFailed("Cancelled");return;}
    emit progressUpdated(100);emit fullCloudReady(cloud);
}

// ===================================================================
// 大文件: mmap + Octree path
// ===================================================================
void LoadWorker::startMmapLoad(const QString &path, const LoadConfig &config){
    m_cancelled.store(false,std::memory_order_relaxed);
    emit loadStarted();emit progressUpdated(0);

    auto t0=std::chrono::steady_clock::now();
    std::string localPath=path.toLocal8Bit().toStdString();

    // ---- 1. mmap open + header parse ----
    auto reader=std::make_shared<MmapPlyReader>();
    std::string err;
    if(!reader->open(localPath.c_str(),err)){
        qDebug()<<"[Mmap] open failed:"<<err.c_str();
        emit loadFailed(QString::fromStdString(err));return;
    }
    qDebug()<<"[Mmap] opened, points:"<<reader->pointCount()
            <<"stride:"<<reader->header().stride
            <<"dataOffset:"<<reader->header().dataOffset;

    // diagnostic: first 3 points
    float xyz[3];
    for(uint64_t i=0;i<std::min<uint64_t>(3,reader->pointCount());++i){
        reader->readPosition(i,xyz);
        qDebug()<<"[Mmap] point"<<i<<":"<<xyz[0]<<xyz[1]<<xyz[2];
    }

    if(m_cancelled.load()){reader->close();emit loadFailed("Cancelled");return;}
    emit progressUpdated(15);

    // ---- 2. stats (AABB via sampling) ----
    CloudStats stats{};
    stats.total_points=static_cast<int64_t>(reader->pointCount());
    stats.is_mmap=true;
    stats.file_size_bytes=static_cast<int64_t>(reader->fileSize());

    float gMin[3]={1e30f,1e30f,1e30f},gMax[3]={-1e30f,-1e30f,-1e30f};
    uint64_t stride=std::max<uint64_t>(1,reader->pointCount()/100000);
    for(uint64_t i=0;i<reader->pointCount();i+=stride){
        reader->readPosition(i,xyz);
        for(int d=0;d<3;++d){if(xyz[d]<gMin[d])gMin[d]=xyz[d];if(xyz[d]>gMax[d])gMax[d]=xyz[d];}
    }
    for(int d=0;d<3;++d){stats.aabb_min[d]=gMin[d];stats.aabb_max[d]=gMax[d];}
    auto t1=std::chrono::steady_clock::now();
    stats.load_time_ms=std::chrono::duration<double,std::milli>(t1-t0).count();
    emit statsReady(stats);emit progressUpdated(30);

    if(m_cancelled.load()){reader->close();emit loadFailed("Cancelled");return;}

    // ---- 3. octree build ----
    qDebug()<<"[Octree] building...";
    auto octree=std::make_shared<PointOctree>();
    auto readFn=[&reader](uint64_t idx,float* xyz){reader->readPosition(idx,xyz);};
    octree->build(readFn,reader->pointCount(),10,5000,200000);
    qDebug()<<"[Octree] nodes:"<<octree->nodeCount();

    if(m_cancelled.load()){reader->close();emit loadFailed("Cancelled");return;}
    emit progressUpdated(90);

    auto t2=std::chrono::steady_clock::now();
    qDebug()<<"[Octree] build time:"<<std::chrono::duration<double,std::milli>(t2-t1).count()<<"ms";

    // ---- 4. preview: 50k points for instant display ----
    {
        auto previewCloud=std::make_shared<open3d::geometry::PointCloud>();
        uint64_t n=reader->pointCount(),ps=std::max<uint64_t>(1,n/50000);
        for(uint64_t i=0;i<n;i+=ps){
            float xyz[3],rgb[3];reader->readPoint(i,xyz,rgb);
            previewCloud->points_.emplace_back(xyz[0],xyz[1],xyz[2]);
            previewCloud->colors_.emplace_back(rgb[0],rgb[1],rgb[2]);
        }
        emit previewReady(previewCloud);
    }
    if(m_cancelled.load()){reader->close();emit loadFailed("Cancelled");return;}

    emit progressUpdated(100);
    emit mmapReady(reader,octree,stats);
}