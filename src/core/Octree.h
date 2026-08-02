#pragma once

#include <cstdint>
#include <vector>
#include <list>
#include <unordered_map>
#include <functional>

struct OctreeNode {
    float aabbMin[3], aabbMax[3];  // ����ռ��Χ��
    uint64_t firstPoint;           // �� mmap �е���ʼ����
    uint64_t pointCount;           // ���ڵ�����ĵ���
    uint32_t childMask;            // bit i = �� i ���ӽڵ����
    int32_t  children[8];          // �ӽڵ�����, -1 ��ʾ��
    float    geometricError;       // ������� (�Խ��߳���)
    int8_t   depth;                // �����
};

/// ��˰˲���: �� mmap PLY �ļ�����, ֧�� LOD ��ѯ
class PointOctree {
public:
    PointOctree() = default;

    /// �� mmap ����: ����ɨ�� (��һ��ͳ��, �ڶ��˷���)
    /// nodeLimit: ���ڵ��� (�����ڴ�)
    /// maxDepth:   ������
    /// minPtsPerNode: ÿ�ڵ����ٵ���
    void build(std::function<void(uint64_t idx, float *xyz)> readPos,
               uint64_t totalPoints, int maxDepth = 10,
               int minPtsPerNode = 5000,
               uint32_t nodeLimit = 200000);
    // 带进度回调的增量构建
    void buildWithProgress(std::function<void(uint64_t,float*)> readPos,
            uint64_t totalPoints,int maxDepth=10,int minPtsPerNode=5000,
            uint32_t nodeLimit=200000,
            std::function<void(int)> progressCb={});

    /// ���� MVP �������Ļ�ռ������ֵ��ѯ�ɼ��ڵ�
    /// ���� (�ڵ�����, lod ��ȵ���) ��
    void query(const float *mvp, float sseThreshold,
               std::vector<std::pair<uint32_t, int>> &visible) const;

    const OctreeNode &node(uint32_t idx) const { return m_nodes[idx]; }
    uint32_t           rootIndex()       const { return 0; }
    size_t             nodeCount()        const { return m_nodes.size(); }

    // AABB of the entire cloud
    void totalAABB(float lo[3], float hi[3]) const;

private:
    void subdivide(uint32_t parentIdx, int depth, int minPts, uint32_t &nodeLimit);
    void traverseVisible(uint32_t idx, const float *mvp,
                         float sseThresh,
                         std::vector<std::pair<uint32_t, int>> &out) const;

    std::vector<OctreeNode> m_nodes;

    // ��ʱ������
    std::function<void(uint64_t, float*)> m_readPos;
    std::vector<uint64_t> m_pointAssign; // ÿ������䵽�ĸ��ڵ� (������)
};

/// LRU ����: ���� GPU VBO ��ļ���/ж��
template <typename Key, typename Value>
class LruCache {
public:
    explicit LruCache(size_t maxSize) : m_maxSize(maxSize) {}

    Value *get(const Key &key) {
        auto it = m_map.find(key);
        if (it == m_map.end()) return nullptr;
        // �Ƶ�ǰ�� (���ʹ��)
        m_list.splice(m_list.begin(), m_list, it->second);
        return &it->second->second;
    }

    void put(const Key &key, Value value) {
        auto it = m_map.find(key);
        if (it != m_map.end()) {
            it->second->second = std::move(value);
            m_list.splice(m_list.begin(), m_list, it->second);
            return;
        }
        if (m_map.size() >= m_maxSize) {
            auto last = m_list.back();
            m_map.erase(last.first);
            m_list.pop_back();
        }
        m_list.emplace_front(key, std::move(value));
        m_map[key] = m_list.begin();
    }

    size_t size() const { return m_map.size(); }

private:
    size_t m_maxSize;
    std::list<std::pair<Key, Value>> m_list;
    std::unordered_map<Key, decltype(m_list.begin())> m_map;
};
