#pragma once

#include <cstdint>
#include <string>

// ---- 加载配置 ----
struct LoadConfig {
    double voxel_size = -1.0;
    int    preview_target_points  = 50000;
    bool   auto_preview           = true;
    int    auto_preview_threshold = 500000;
    // mmap 模式: 超此大小的文件使用内存映射加载 (字节)
    int64_t mmap_threshold_bytes  = 100 * 1024 * 1024; // 100 MB
};

// ---- 点云统计 ----
struct CloudStats {
    int64_t total_points = 0;
    int64_t displayed_points = 0;
    double aabb_min[3] = {};
    double aabb_max[3] = {};
    double point_density = 0.0;
    double load_time_ms = 0.0;
    int64_t vram_usage_mb = 0;
    bool    is_mmap = false;          // 是否使用 mmap 模式
    int64_t file_size_bytes = 0;
};

// ---- LOD / 渲染配置 ----
struct LodConfig {
    float sse_threshold = 0.5f;       // 屏幕空间误差阈值 (像素)
    int   min_points_per_node = 5000; // 八叉树叶节点最小点数
    int   max_octree_depth    = 10;   // 八叉树最大深度
    int   max_octree_nodes    = 200000;
    int   max_visible_nodes   = 256;  // 每帧最多渲染的节点数
};

// ---- 渲染模式 ----
enum class RenderMode {
    DepthColor,
    VertexColor,
    FlatColor,
};