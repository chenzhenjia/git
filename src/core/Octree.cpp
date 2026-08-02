#include "Octree.h"

#include <algorithm>
#include <cmath>
#include <cstring>

void PointOctree::build(
    std::function<void(uint64_t, float*)> readPos,
    uint64_t totalPoints, int maxDepth, int minPtsPerNode,
    uint32_t nodeLimit) {

    m_readPos = std::move(readPos);
    m_nodes.clear();
    m_nodes.reserve(1024);

    // ---- Pass 1: ����ȫ�� AABB ----
    float gMin[3] = { 1e30f, 1e30f, 1e30f };
    float gMax[3] = {-1e30f,-1e30f,-1e30f };

    const uint64_t step = std::max<uint64_t>(1, totalPoints / 200000);
    for (uint64_t i = 0; i < totalPoints; i += step) {
        float xyz[3];
        m_readPos(i, xyz);
        for (int d = 0; d < 3; ++d) {
            if (xyz[d] < gMin[d]) gMin[d] = xyz[d];
            if (xyz[d] > gMax[d]) gMax[d] = xyz[d];
        }
    }

    // ---- Pass 2: �����˲��� ----
    auto computeError = [](const float *lo, const float *hi) {
        float dx = hi[0]-lo[0], dy = hi[1]-lo[1], dz = hi[2]-lo[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    // ���ڵ�
    OctreeNode root{};
    std::memcpy(root.aabbMin, gMin, 12);
    std::memcpy(root.aabbMax, gMax, 12);
    root.firstPoint  = 0;
    root.pointCount  = totalPoints;
    root.childMask   = 0;
    for (int i = 0; i < 8; ++i) root.children[i] = -1;
    root.geometricError = computeError(gMin, gMax);
    root.depth = 0;
    m_nodes.push_back(root);

    // �ݹ�ϸ��
    uint32_t limit = nodeLimit;
    subdivide(0, 0, minPtsPerNode, limit);
}

void PointOctree::subdivide(uint32_t parentIdx, int depth,
                            int minPts, uint32_t &nodeLimit) {
    if (depth >= 10) return;  // hard limit
    OctreeNode &p = m_nodes[parentIdx];
    if (p.pointCount <= static_cast<uint64_t>(minPts)) return;
    if (m_nodes.size() >= nodeLimit) return;

    const float cx = (p.aabbMin[0] + p.aabbMax[0]) * 0.5f;
    const float cy = (p.aabbMin[1] + p.aabbMax[1]) * 0.5f;
    const float cz = (p.aabbMin[2] + p.aabbMax[2]) * 0.5f;

    // Child bounding boxes
    float childAabb[8][6];
    for (int i = 0; i < 8; ++i) {
        childAabb[i][0] = (i & 1) ? cx : p.aabbMin[0];
        childAabb[i][1] = (i & 1) ? p.aabbMax[0] : cx;
        childAabb[i][2] = (i & 2) ? cy : p.aabbMin[1];
        childAabb[i][3] = (i & 2) ? p.aabbMax[1] : cy;
        childAabb[i][4] = (i & 4) ? cz : p.aabbMin[2];
        childAabb[i][5] = (i & 4) ? p.aabbMax[2] : cz;
    }

    // ͳ��ÿ���ӽڵ�ĵ��� (����)
    uint64_t counts[8] = {};
    const uint64_t stride = std::max<uint64_t>(1, p.pointCount / 5000);
    const uint64_t end = p.firstPoint + p.pointCount;
    for (uint64_t i = p.firstPoint; i < end; i += stride) {
        float xyz[3];
        m_readPos(i, xyz);
        int child = 0;
        if (xyz[0] >= cx) child |= 1;
        if (xyz[1] >= cy) child |= 2;
        if (xyz[2] >= cz) child |= 4;
        counts[child]++;
    }

    // ��һ������
    uint64_t totalSamples = 0;
    for (int i = 0; i < 8; ++i) totalSamples += counts[i];
    if (totalSamples == 0) return;

    for (int i = 0; i < 8; ++i) {
        uint64_t approx = p.pointCount * counts[i] / totalSamples;
        if (approx < static_cast<uint64_t>(minPts / 4)) continue;
        if (m_nodes.size() >= nodeLimit) return;

        OctreeNode child{};
        std::memcpy(child.aabbMin, childAabb[i], 12);
        std::memcpy(child.aabbMax, childAabb[i] + 3, 12);
        child.firstPoint = p.firstPoint; // ���� mmap ��Χ
        child.pointCount = approx;
        child.childMask  = 0;
        for (int j = 0; j < 8; ++j) child.children[j] = -1;
        float dx = child.aabbMax[0]-child.aabbMin[0];
        float dy = child.aabbMax[1]-child.aabbMin[1];
        float dz = child.aabbMax[2]-child.aabbMin[2];
        child.geometricError = std::sqrt(dx*dx+dy*dy+dz*dz);
        child.depth = static_cast<int8_t>(depth + 1);

        int32_t childIdx = static_cast<int32_t>(m_nodes.size());
        p.childMask |= (1u << i);
        p.children[i] = childIdx;
        m_nodes.push_back(child);

        subdivide(childIdx, depth + 1, minPts, nodeLimit);
    }
}

void PointOctree::query(
    const float *mvp, float sseThreshold,
    std::vector<std::pair<uint32_t, int>> &visible) const {
    visible.clear();
    if (m_nodes.empty()) return;
    traverseVisible(0, mvp, sseThreshold, visible);
}

void PointOctree::traverseVisible(
    uint32_t idx, const float *mvp, float sseThresh,
    std::vector<std::pair<uint32_t, int>> &out) const {
    const OctreeNode &n = m_nodes[idx];

    // SSE check: geometricError / distance
    // mvp[11] = -near (perspective) or approximate distance from mvp
    float cx = (n.aabbMin[0] + n.aabbMax[0]) * 0.5f;
    float cy = (n.aabbMin[1] + n.aabbMax[1]) * 0.5f;
    float cz = (n.aabbMin[2] + n.aabbMax[2]) * 0.5f;
    // Transform center by MVP to get clip distance
    float w = mvp[3]*cx + mvp[7]*cy + mvp[11]*cz + mvp[15];
    float dist = std::abs(w);
    float sse = n.geometricError / std::max(dist, 1e-6f);

    if (sse < sseThresh || n.childMask == 0 || n.depth >= 9) {
        out.emplace_back(idx, n.depth);
        return;
    }

    for (int i = 0; i < 8; ++i) {
        if (n.childMask & (1u << i)) {
            traverseVisible(static_cast<uint32_t>(n.children[i]),
                           mvp, sseThresh, out);
        }
    }
}

void PointOctree::totalAABB(float lo[3], float hi[3]) const {
    if (m_nodes.empty()) return;
    std::memcpy(lo, m_nodes[0].aabbMin, 12);
    std::memcpy(hi, m_nodes[0].aabbMax, 12);
}
// ---- buildWithProgress ----
void PointOctree::buildWithProgress(std::function<void(uint64_t,float*)> readPos,uint64_t totalPoints,int maxDepth,int minPtsPerNode,uint32_t nodeLimit,std::function<void(int)> progressCb){
    build(std::move(readPos),totalPoints,maxDepth,minPtsPerNode,nodeLimit);
    if(progressCb)progressCb(100);
}
