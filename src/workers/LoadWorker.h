#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <QObject>
#include <QString>

#include "core/Types.h"

class MmapPlyReader;
class PointOctree;
namespace open3d { namespace geometry { class PointCloud; } }

// 二阶段异步加载 + mmap 大文件路径。
class LoadWorker : public QObject {
    Q_OBJECT

public:
    explicit LoadWorker(QObject *parent = nullptr);
    ~LoadWorker() override;

    void cancel() { m_cancelled.store(true, std::memory_order_relaxed); }

public slots:
    // 小文件路径 (<100MB)
    void startLoad(const QString &path, const LoadConfig &config);

    // 大文件 mmap 路径
    void startMmapLoad(const QString &path, const LoadConfig &config);

signals:
    void loadStarted();
    void progressUpdated(int percent);

    // 小文件
    void previewReady(std::shared_ptr<open3d::geometry::PointCloud> cloud);
    void fullCloudReady(std::shared_ptr<open3d::geometry::PointCloud> cloud);

    // 大文件: mmap reader + 八叉树已就绪
    void mmapReady(std::shared_ptr<MmapPlyReader> reader,
                   std::shared_ptr<PointOctree>   octree,
                   CloudStats                     stats);

    void statsReady(const CloudStats &stats);
    void loadFailed(const QString &errorMsg);

    // 诊断日志 (内部 qDebug)
    void diagnosticLog(const QString &msg);

private:
    std::atomic<bool> m_cancelled{false};
};