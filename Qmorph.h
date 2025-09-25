#pragma once
#include <vector>
#include <map>
#include <glm/glm.hpp>
#include "CGALMeshGenerator.h"

// Ϊ�˱���ѭ�����ã�����ֻ����ǰ������
class CGALMeshGenerator;

class Qmorph {
public:
    // �����������Ľṹ��
    struct Result {
        std::vector<CGALMeshGenerator::Quad> quads;
        std::vector<CGALMeshGenerator::Triangle> remaining_triangles;
    };

    Qmorph() = default;

    // ���ĺ���������һ���������񣬷���һ���ı���Ϊ��������
    Result run(const CGALMeshGenerator& delaunay_mesh);

private:
    // --- �ڲ����ݽṹ ---
    // ʹ�ñ����ô��������
    using Tri_idx = size_t;
    using Vert_idx = unsigned int;
    using Edge = std::pair<Vert_idx, Vert_idx>;

    // �洢ÿ�������ε��ڽ���Ϣ
    struct Tri_adj {
        // *** FIX: ʹ�� SIZE_MAX ���� -1 ����ʾ��Ч���� ***
        Tri_idx neighbors[3] = { SIZE_MAX, SIZE_MAX, SIZE_MAX };
        // �ھ���ͨ�����������ӵ�
        Edge edges[3];
    };

    // --- ������Ķ�׶��㷨 ---
    void build_adjacency(const std::vector<CGALMeshGenerator::Triangle>& triangles);
    void initial_pair_merging(const std::vector<glm::vec2>& vertices, const std::vector<CGALMeshGenerator::Triangle>& triangles);
    void iterative_cleanup(const std::vector<glm::vec2>& vertices, std::vector<CGALMeshGenerator::Triangle>& triangles);
    void final_smoothing(std::vector<glm::vec2>& vertices, const Boundary& boundary); // ע�⣺��Ҫ�߽���Ϣ

    // --- ���������븨������ ---
    float calculate_quad_quality(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3);
    Edge make_sorted_edge(Vert_idx v1, Vert_idx v2);

    // --- ��Ա���� ---
    std::vector<bool> merged_triangles_;
    std::vector<Tri_adj> adj_list_;
    std::map<Edge, std::vector<Tri_idx>> edge_to_tri_map_;
    Result result_;
};