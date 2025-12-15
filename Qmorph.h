#pragma once
#include <vector>
#include <map>
#include <queue>
#include <glm/glm.hpp>
#include "CGALMeshGenerator.h"

// 为了避免循环引用，我们只进行前向声明
class CGALMeshGenerator;

class Qmorph {
public:
    // 定义输出结果的结构体
    struct Result {
        std::vector<CGALMeshGenerator::Quad> quads;
        std::vector<CGALMeshGenerator::Triangle> remaining_triangles;
    };

    Qmorph() = default;

    // 核心函数：接收一个三角网格，返回一个四边形为主的网格
    Result run(const CGALMeshGenerator& delaunay_mesh);

private:
    // --- 内部数据结构 ---
    // 使用别名让代码更清晰
    using Tri_idx = size_t;
    using Vert_idx = unsigned int;
    using Edge = std::pair<Vert_idx, Vert_idx>;

    // 存储每个三角形的邻接信息
    struct Tri_adj {
        // *** FIX: 使用 SIZE_MAX 代替 -1 来表示无效索引 ***
        Tri_idx neighbors[3] = { SIZE_MAX, SIZE_MAX, SIZE_MAX };
        // 邻居是通过哪条边连接的
        Edge edges[3];
    };

    // Structure for Priority-Based Merging
    struct PotentialQuad {
        float quality;
        Tri_idx t1, t2;
        Vert_idx v1_shared, v2_shared; // The shared edge
        Vert_idx v1_opp, v2_opp;       // The opposite vertices

        // Priority queue orders by quality (max first)
        bool operator<(const PotentialQuad& other) const {
            return quality < other.quality;
        }
    };

    // --- 升级后的多阶段算法 ---
    void build_adjacency(const std::vector<CGALMeshGenerator::Triangle>& triangles);
    void initial_pair_merging(const std::vector<glm::vec2>& vertices, const std::vector<CGALMeshGenerator::Triangle>& triangles);

    // Replaces initial_pair_merging with a threshold-based pass
    int priority_merge_pass(const std::vector<glm::vec2>& vertices,
        const std::vector<CGALMeshGenerator::Triangle>& triangles,
        float quality_threshold);
    // Laplacian smoothing for unmerged triangles
    void smooth_vertices(std::vector<glm::vec2>& vertices,
        const std::vector<CGALMeshGenerator::Triangle>& triangles);
    // Calculate quality for a potential merge
    PotentialQuad evaluate_merge(Tri_idx t1_idx, Tri_idx t2_idx,
        const std::vector<CGALMeshGenerator::Triangle>& triangles,
        const std::vector<glm::vec2>& vertices,
        const Edge& shared_edge);

    void iterative_cleanup(const std::vector<glm::vec2>& vertices, std::vector<CGALMeshGenerator::Triangle>& triangles);
    void final_smoothing(std::vector<glm::vec2>& vertices, const Boundary& boundary); // 注意：需要边界信息

    // --- 质量评估与辅助函数 ---
    float calculate_quad_quality(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3);
    Edge make_sorted_edge(Vert_idx v1, Vert_idx v2);

    // --- 成员变量 ---
    std::vector<bool> merged_triangles_;
    std::vector<Tri_adj> adj_list_;
    std::map<Edge, std::vector<Tri_idx>> edge_to_tri_map_;
    Result result_;

    std::vector<bool> fixed_vertices_; // Vertices that shouldn't move during smoothing
};
