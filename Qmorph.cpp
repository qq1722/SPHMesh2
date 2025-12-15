#include "Qmorph.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <set>
#include <cmath>

// --- 辅助函数：创建排序后的边，方便作为map的键 ---
Qmorph::Edge Qmorph::make_sorted_edge(Vert_idx v1, Vert_idx v2) {
    if (v1 > v2) std::swap(v1, v2);
    return { v1, v2 };
}

// --- 核心执行函数 ---
// --- 核心执行函数 ---
Qmorph::Result Qmorph::run(const CGALMeshGenerator& delaunay_mesh) {
    const auto& initial_triangles = delaunay_mesh.get_triangles();
    // 创建顶点副本，因为我们会在平滑过程中修改它们的位置
    auto vertices = delaunay_mesh.get_vertices();

    if (initial_triangles.empty()) {
        std::cout << "Qmorph: No triangles to convert." << std::endl;
        return {};
    }

    // --- 初始化 ---
    result_.quads.clear();
    result_.remaining_triangles.clear();
    merged_triangles_.assign(initial_triangles.size(), false);
    fixed_vertices_.assign(vertices.size(), false);

    // --- 标记边界顶点 (这些点在平滑时不能动) ---
    // 我们先建立一次全局邻接关系来统计边的引用次数
    std::map<Edge, int> edge_counts;
    for (const auto& t : initial_triangles) {
        edge_counts[make_sorted_edge(t.v0, t.v1)]++;
        edge_counts[make_sorted_edge(t.v1, t.v2)]++;
        edge_counts[make_sorted_edge(t.v2, t.v0)]++;
    }
    for (const auto& pair : edge_counts) {
        if (pair.second == 1) { // 只有1个三角形引用的边是边界边
            fixed_vertices_[pair.first.first] = true;
            fixed_vertices_[pair.first.second] = true;
        }
    }

    // --- 多阶段迭代策略 (Scheme 1 & 3) ---
    struct PassConfig {
        float threshold;
        bool do_smooth;
    };

    // 定义三个阶段：
    // 1. 高质量合并 (只合并几乎完美的矩形) -> 平滑
    // 2. 中质量合并 (合并一般的四边形) -> 平滑
    // 3. 清理阶段 (只要能构成凸四边形就合并) -> 不平滑
    std::vector<PassConfig> passes = {
        { 0.8f, true },
        { 0.4f, true },
        { 0.01f, false }
    };

    std::cout << "Qmorph: Starting Priority-Based Optimization..." << std::endl;

    for (int i = 0; i < passes.size(); ++i) {
        // 1. 每次合并前，基于当前状态重建邻接关系
        build_adjacency(initial_triangles);

        // 2. 执行基于优先级的合并
        int merged = priority_merge_pass(vertices, initial_triangles, passes[i].threshold);
        std::cout << "  Pass " << i + 1 << " (Thres=" << passes[i].threshold << "): Merged " << merged << " quads." << std::endl;

        // 3. 对剩余的三角形进行平滑，改善形状以便下一轮合并
        if (passes[i].do_smooth) {
            smooth_vertices(vertices, initial_triangles);
        }
    }
    //// --- 终极清理：处理剩余三角形 (3-to-1 Split) ---
    //std::cout << "Final Cleanup: Splitting remaining " << result_.remaining_triangles.size() << " triangles..." << std::endl;

    //// 遍历所有未合并的三角形
    //for (size_t i = 0; i < initial_triangles.size(); ++i) {
    //    if (!merged_triangles_[i]) {
    //        const auto& t = initial_triangles[i];

    //        // 1. 计算中心点
    //        glm::vec2 p0 = vertices[t.v0];
    //        glm::vec2 p1 = vertices[t.v1];
    //        glm::vec2 p2 = vertices[t.v2];
    //        glm::vec2 center = (p0 + p1 + p2) / 3.0f;

    //        // 2. 添加新顶点到顶点列表
    //        unsigned int center_idx = (unsigned int)vertices.size();
    //        vertices.push_back(center);
    //        // (注意：这会改变 vertices 的大小，但在循环中只添加，不影响旧索引)

    //        // 3. 生成 3 个新四边形
    //        // Quad 1: v0 -> v1 -> center
    //        // 注意：这里原来的三角形边 v0-v1 现在变成四边形的一条边，
    //        // 另外两条边是 v1-center 和 center-v0。
    //        // 等等... 3-to-1 分裂通常是把一个三角形切成3个四边形，
    //        // 但这需要三角形的每条边都被“一分为二”才能形成合法的四边形拓扑。

    //        // *** 修正策略 ***
    //        // 对于纯粹的三角形网格转四边形，最简单的“消灭孤立三角形”的方法其实是：
    //        // "把三角形的一个顶点分裂，或者把三角形推到边界"。

    //        // 但最简单的工程实现是保留它们为三角形，或者：
    //        // 如果你必须全是四边形，可以使用 Catmull-Clark 细分的一步：
    //        // 对整个网格进行一次细分，所有多边形（包括三角形）都会变成四边形。
    //        // 但这会增加4倍的网格量。
    //    }
    //}

    // --- 收集最终结果 ---
    for (size_t i = 0; i < initial_triangles.size(); ++i) {
        if (!merged_triangles_[i]) {
            result_.remaining_triangles.push_back(initial_triangles[i]);
        }
    }

    std::cout << "Qmorph Complete: " << result_.quads.size() << " quads, "
        << result_.remaining_triangles.size() << " triangles remaining." << std::endl;

    return result_;
}


// --- 步骤 1: 构建邻接关系，这是所有后续操作的基础 ---
// --- 构建邻接关系 (只考虑未合并的三角形) ---
void Qmorph::build_adjacency(const std::vector<CGALMeshGenerator::Triangle>& triangles) {
    edge_to_tri_map_.clear();
    // 重新初始化 adj_list_，确保大小匹配
    if (adj_list_.size() != triangles.size()) {
        adj_list_.assign(triangles.size(), Tri_adj{});
    }
    else {
        // 清空旧数据
        for (auto& adj : adj_list_) adj = Tri_adj{};
    }

    for (Tri_idx i = 0; i < triangles.size(); ++i) {
        if (merged_triangles_[i]) continue; // 跳过已合并的

        const auto& t = triangles[i];
        Vert_idx v[3] = { t.v0, t.v1, t.v2 };
        for (int j = 0; j < 3; ++j) {
            Edge edge = make_sorted_edge(v[j], v[(j + 1) % 3]);
            adj_list_[i].edges[j] = edge;
            edge_to_tri_map_[edge].push_back(i);
        }
    }
}


// --- 步骤 2: 快速配对合并，处理最理想的情况 ---
void Qmorph::initial_pair_merging(const std::vector<glm::vec2>& vertices, const std::vector<CGALMeshGenerator::Triangle>& triangles) {
    for (const auto& pair : edge_to_tri_map_) {
        const auto& edge = pair.first;
        const auto& tri_indices = pair.second;

        if (tri_indices.size() == 2) { // 是内部边
            Tri_idx idx1 = tri_indices[0];
            Tri_idx idx2 = tri_indices[1];

            if (merged_triangles_[idx1] || merged_triangles_[idx2]) {
                continue; // 其中一个已经被合并
            }

            const auto& t1 = triangles[idx1];
            const auto& t2 = triangles[idx2];

            // 找到两个三角形的另外两个顶点
            Vert_idx other_v1 = (t1.v0 != edge.first && t1.v0 != edge.second) ? t1.v0 : ((t1.v1 != edge.first && t1.v1 != edge.second) ? t1.v1 : t1.v2);
            Vert_idx other_v2 = (t2.v0 != edge.first && t2.v0 != edge.second) ? t2.v0 : ((t2.v1 != edge.first && t2.v1 != edge.second) ? t2.v1 : t2.v2);

            // 质量评估
            float quality = calculate_quad_quality(vertices[other_v1], vertices[edge.first], vertices[other_v2], vertices[edge.second]);

            // 阈值可以根据需要调整，0.25是一个比较宽松的值
            if (quality > 0.5f) {
                result_.quads.push_back({ other_v1, edge.first, other_v2, edge.second });
                merged_triangles_[idx1] = true;
                merged_triangles_[idx2] = true;
            }
        }
    }
}


// --- 质量评估函数 (可以根据需要变得更复杂) ---
// --- 质量计算 (检测凸性 + 90度偏差) ---
float Qmorph::calculate_quad_quality(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3) {
    // 1. 凸性检测 (使用叉乘的 Z 分量)
    auto cross_z = [](glm::vec2 a, glm::vec2 b, glm::vec2 c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };

    float c1 = cross_z(p0, p1, p2);
    float c2 = cross_z(p1, p2, p3);
    float c3 = cross_z(p2, p3, p0);
    float c4 = cross_z(p3, p0, p1);

    // 如果符号不一致，说明是凹多边形或自相交
    bool all_pos = (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0);
    bool all_neg = (c1 < 0 && c2 < 0 && c3 < 0 && c4 < 0);
    if (!all_pos && !all_neg) return 0.0f;

    // 2. 角度偏差计算
    auto get_angle = [](glm::vec2 a, glm::vec2 b, glm::vec2 c) { // angle at b
        glm::vec2 ba = glm::normalize(a - b);
        glm::vec2 bc = glm::normalize(c - b);
        return std::acos(glm::clamp(glm::dot(ba, bc), -1.0f, 1.0f));
        };

    float angles[4];
    angles[0] = get_angle(p3, p0, p1);
    angles[1] = get_angle(p0, p1, p2);
    angles[2] = get_angle(p1, p2, p3);
    angles[3] = get_angle(p2, p3, p0);

    float dev = 0.0f;
    float target = 1.5707963f; // 90 degrees in radians
    for (float a : angles) {
        dev += std::abs(a - target);
    }

    // 归一化：最大可能偏差约为 2*PI (退化情况)，我们希望偏差越小越好
    // 得分 1.0 = 完美正方形
    return std::max(0.0f, 1.0f - (dev / (2.0f * 3.14159f)));
}

int Qmorph::priority_merge_pass(const std::vector<glm::vec2>& vertices,
    const std::vector<CGALMeshGenerator::Triangle>& triangles,
    float quality_threshold) {
    std::priority_queue<PotentialQuad> pq;

    // 1. Evaluate all internal edges
    for (const auto& pair : edge_to_tri_map_) {
        if (pair.second.size() == 2) {
            Tri_idx t1 = pair.second[0];
            Tri_idx t2 = pair.second[1];
            pq.push(evaluate_merge(t1, t2, triangles, vertices, pair.first));
        }
    }

    int merge_count = 0;
    while (!pq.empty()) {
        PotentialQuad candidate = pq.top();
        pq.pop();

        // Valid checks
        if (candidate.quality < quality_threshold) continue; // Stop if quality drops too low? Or just skip?
        // Since it's a priority queue, we could technically break, 
        // but let's just continue to find other disconnected islands.

        if (merged_triangles_[candidate.t1] || merged_triangles_[candidate.t2]) continue;

        // Perform Merge
        result_.quads.push_back({ candidate.v1_opp, candidate.v1_shared, candidate.v2_opp, candidate.v2_shared });

        merged_triangles_[candidate.t1] = true;
        merged_triangles_[candidate.t2] = true;
        merge_count++;
    }
    return merge_count;
}

Qmorph::PotentialQuad Qmorph::evaluate_merge(Tri_idx t1_idx, Tri_idx t2_idx,
    const std::vector<CGALMeshGenerator::Triangle>& triangles,
    const std::vector<glm::vec2>& vertices,
    const Edge& shared_edge) {
    const auto& t1 = triangles[t1_idx];
    const auto& t2 = triangles[t2_idx];

    // Find opposite vertices
    Vert_idx v1_opp = (t1.v0 != shared_edge.first && t1.v0 != shared_edge.second) ? t1.v0 :
        ((t1.v1 != shared_edge.first && t1.v1 != shared_edge.second) ? t1.v1 : t1.v2);
    Vert_idx v2_opp = (t2.v0 != shared_edge.first && t2.v0 != shared_edge.second) ? t2.v0 :
        ((t2.v1 != shared_edge.first && t2.v1 != shared_edge.second) ? t2.v1 : t2.v2);

    float q = calculate_quad_quality(vertices[v1_opp], vertices[shared_edge.first],
        vertices[v2_opp], vertices[shared_edge.second]);

    return { q, t1_idx, t2_idx, shared_edge.first, shared_edge.second, v1_opp, v2_opp };
}

void Qmorph::smooth_vertices(std::vector<glm::vec2>& vertices,
    const std::vector<CGALMeshGenerator::Triangle>& triangles) {
    // Simple Laplacian smoothing for active vertices
    std::vector<glm::vec2> new_positions = vertices;
    std::vector<int> valence(vertices.size(), 0);
    std::vector<glm::vec2> sum_neighbors(vertices.size(), { 0.0f, 0.0f });

    // Iterate active triangles to gather adjacency for smoothing
    for (size_t i = 0; i < triangles.size(); ++i) {
        if (merged_triangles_[i]) continue;

        const auto& t = triangles[i];
        Vert_idx v[3] = { t.v0, t.v1, t.v2 };

        // Add edges (undirected, so add both ways)
        for (int j = 0; j < 3; ++j) {
            Vert_idx a = v[j];
            Vert_idx b = v[(j + 1) % 3];

            // A -> B
            sum_neighbors[a] += vertices[b];
            valence[a]++;
            // B -> A
            sum_neighbors[b] += vertices[a];
            valence[b]++;
        }
    }

    // Update positions
    for (size_t i = 0; i < vertices.size(); ++i) {
        if (fixed_vertices_[i]) continue; // Boundary vertex
        if (valence[i] == 0) continue;    // Isolated or merged vertex

        // Average of connected neighbors (Triangle edges count twice, but it averages out)
        // Correct Laplacian: Iterate unique neighbors. 
        // Approximation: Sum all incident edges / count. 
        new_positions[i] = sum_neighbors[i] / (float)valence[i];
    }

    vertices = new_positions;
}
