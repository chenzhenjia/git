#include "PointCloudLoader.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <set>

#include <open3d/geometry/PointCloud.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/utility/Logging.h>

namespace {

static std::string lower(std::string s) {
    for (auto &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

static std::string sizeStr(int64_t bytes) {
    std::ostringstream ss; ss.precision(1); ss << std::fixed;
    if (bytes >= 1024*1024*1024) ss << bytes/(1024.0*1024*1024) << " GB";
    else if (bytes >= 1024*1024) ss << bytes/(1024.0*1024) << " MB";
    else ss << bytes/1024.0 << " KB";
    return ss.str();
}

} // namespace

// ===================================================================
//  peekVertexCount: 预读 PLY header, 提取格式类型和顶点数
// ===================================================================
int64_t PointCloudLoader::peekVertexCount(const std::string &path,
                                          std::string &formatType) {
    formatType.clear();
    std::ifstream f(path, std::ios::binary);
    if (!f) return -1;

    char buf[8192] = {};
    f.read(buf, sizeof(buf) - 1);
    std::string hdr(buf, static_cast<size_t>(f.gcount()));

    // 检测格式
    if (hdr.find("format ascii") != std::string::npos)
        formatType = "ascii";
    else if (hdr.find("binary_little_endian") != std::string::npos)
        formatType = "binary_little_endian";
    else if (hdr.find("binary_big_endian") != std::string::npos)
        formatType = "binary_big_endian";
    else
        return -1;

    // 提取 element vertex N
    auto pos = hdr.find("element vertex ");
    if (pos == std::string::npos) return -1;
    pos += 15; // skip "element vertex "
    auto end = hdr.find_first_of(" \t\r\n", pos);
    std::string numStr = hdr.substr(pos, end - pos);
    try { return std::stoll(numStr); }
    catch (...) { return -1; }
}

// ===================================================================
//  load()
// ===================================================================
std::shared_ptr<open3d::geometry::PointCloud>
PointCloudLoader::load(const std::string &path, const LoadConfig &config,
                       std::string &error_msg) {
    error_msg.clear();
    if (path.empty()) { error_msg = "File path is empty"; return nullptr; }

    // ---- 文件存在性 ----
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) {
        error_msg = "File not found: " + path; return nullptr;
    }
    const auto fsize = std::filesystem::file_size(path, ec);
    if (ec) { error_msg = "Cannot get file size: " + path; return nullptr; }

    // ---- 预处理: 格式探测 + 顶点数预读 ----
    std::string fmtType;
    int64_t vc = peekVertexCount(path, fmtType);
    if (vc <= 0) {
        error_msg = "Cannot parse PLY header or vertexCount=0";
        return nullptr;
    }

    // ---- 内存预估 ----
    const int64_t estMB = vc * 28 / (1024 * 1024);
    if (estMB > 3000) {
        std::ostringstream ss;
        ss << "Estimated " << estMB << " MB RAM for " << vc << " points.\n"
           << "Try voxel_size > 0 or use a binary PLY.";
        error_msg = ss.str(); return nullptr;
    }

    // ---- Open3D 加载 ----
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    // 预分配: 帮助 Open3D 内部避免多次 realloc
    cloud->points_.reserve(static_cast<size_t>(vc));
    if (fmtType == "ascii") cloud->colors_.reserve(static_cast<size_t>(vc));

    try {
        if (!open3d::io::ReadPointCloud(path, *cloud)) {
            std::ostringstream ss;
            ss << "Open3D ReadPointCloud failed on " << fmtType << " PLY\n"
               << "File: " << path << " (" << sizeStr(fsize) << ", " << vc << " vertices)\n"
               << "Tip: if ASCII, try CloudCompare > Export as binary PLY (little-endian)";
            error_msg = ss.str();
            open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);
            return nullptr;
        }
    } catch (const std::bad_alloc &) {
        error_msg = "Out of memory: " + std::to_string(estMB) + " MB needed";
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);
        return nullptr;
    } catch (const std::exception &e) {
        error_msg = std::string("Exception: ") + e.what();
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);
        return nullptr;
    }

    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);

    if (cloud->points_.empty()) {
        error_msg = "Empty cloud: " + path; return nullptr;
    }

    if (config.voxel_size > 0.0)
        cloud = applyVoxelDownsample(cloud, config.voxel_size);

    return cloud;
}

// ===================================================================
CloudStats PointCloudLoader::computeStats(
    const open3d::geometry::PointCloud &cloud) {
    CloudStats s;
    s.total_points = static_cast<int64_t>(cloud.points_.size());
    if (s.total_points == 0) return s;
    const auto lo = cloud.GetMinBound(), hi = cloud.GetMaxBound();
    for (int i = 0; i < 3; ++i) { s.aabb_min[i] = lo[i]; s.aabb_max[i] = hi[i]; }
    const auto sz = hi - lo;
    double vol = sz.x() * sz.y() * sz.z();
    s.point_density = (vol > 1e-12) ? s.total_points / vol : s.total_points;
    return s;
}

std::shared_ptr<open3d::geometry::PointCloud>
PointCloudLoader::downsampleTo(
    std::shared_ptr<open3d::geometry::PointCloud> cloud, int target) {
    if (!cloud || cloud->points_.empty() || target <= 0) return cloud;
    int64_t n = static_cast<int64_t>(cloud->points_.size());
    if (n <= target) return cloud;
    return applyVoxelDownsample(cloud, computeVoxelSizeForTarget(*cloud, target));
}

double PointCloudLoader::computeVoxelSizeForTarget(
    const open3d::geometry::PointCloud &cloud, int target) {
    const auto lo = cloud.GetMinBound(), hi = cloud.GetMaxBound();
    const auto sz = hi - lo;
    return std::max(std::pow(sz.x() * sz.y() * sz.z() / target, 1.0 / 3.0), 1e-6);
}

std::shared_ptr<open3d::geometry::PointCloud>
PointCloudLoader::applyVoxelDownsample(
    std::shared_ptr<open3d::geometry::PointCloud> cloud, double vx) {
    if (!cloud || cloud->points_.empty() || vx <= 0.0) return cloud;
    return cloud->VoxelDownSample(vx);
}