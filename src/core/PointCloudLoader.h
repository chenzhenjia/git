#pragma once

#include <memory>
#include <string>

#include "Types.h"

namespace open3d {
namespace geometry {
class PointCloud;
}
}

class PointCloudLoader {
public:
    PointCloudLoader() = delete;

    /// 加载任意 PLY 文件 (ASCII/Binary), 统一走 Open3D io::ReadPointCloud。
    /// 调用前自动做: 格式探测、内存预估、异常捕获。
    static std::shared_ptr<open3d::geometry::PointCloud>
    load(const std::string &path, const LoadConfig &config,
         std::string &error_msg);

    static CloudStats computeStats(
        const open3d::geometry::PointCloud &cloud);

    static std::shared_ptr<open3d::geometry::PointCloud>
    downsampleTo(std::shared_ptr<open3d::geometry::PointCloud> cloud,
                 int target_points);

private:
    /// 从文件头快速解析 vertex count, 用于内存预分配
    static int64_t peekVertexCount(const std::string &path,
                                   std::string &formatType);

    static double computeVoxelSizeForTarget(
        const open3d::geometry::PointCloud &cloud, int target);

    static std::shared_ptr<open3d::geometry::PointCloud>
    applyVoxelDownsample(std::shared_ptr<open3d::geometry::PointCloud> cloud,
                         double voxel_size);
};