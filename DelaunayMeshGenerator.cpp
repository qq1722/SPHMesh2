#include "DelaunayMeshGenerator.h"
#include "Cdd.h"
#include <iostream>
#include <map>

// �������������ڼ�����������ƽ�����ȼ�����뱾�����
static float distance_sq(const CDT::V2d<float>& p1, const CDT::V2d<float>& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

void DelaunayMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary, float min_particle_spacing) {
    vertices_.clear();
    triangles_.clear();

    const auto& boundary_vertices_in = boundary.get_vertices();
    if (particles.empty() || boundary_vertices_in.empty()) return;

    // --- 1. ������ϴ��׼�� ---
    std::vector<CDT::V2d<float>> final_vertices;
    // ʹ�� map �����ұ߽綥�����¶����б��е�����
    std::map<int, CDT::VertInd> boundary_idx_map;
    // ȥ����ֵ������Ϊ��С���Ӽ���һ����С�ı���
    float min_dist_sq = (min_particle_spacing * 0.1f) * (min_particle_spacing * 0.1f);

    // a) ������ӱ߽綥�㣬����֤���������ظ�
    for (int i = 0; i < boundary_vertices_in.size(); ++i) {
        CDT::V2d<float> current_v = { boundary_vertices_in[i].x, boundary_vertices_in[i].y };
        // ���������ӵ����һ�������Ƿ�̫��
        if (final_vertices.empty() || distance_sq(current_v, final_vertices.back()) > min_dist_sq) {
            boundary_idx_map[i] = static_cast<CDT::VertInd>(final_vertices.size());
            final_vertices.push_back(current_v);
        }
        else {
            boundary_idx_map[i] = static_cast<CDT::VertInd>(final_vertices.size() - 1);
        }
    }

    // b) ����ڲ����ӣ�ͬʱ�������������Ӷ���ľ��룬ʵ��ȥ��
    for (const auto& p : particles) {
        if (p.is_boundary) continue; // �߽������Ѿ�ͨ�� boundary_vertices ���

        CDT::V2d<float> particle_v = { p.position.x, p.position.y };
        bool is_duplicate = false;
        for (const auto& v_final : final_vertices) {
            if (distance_sq(particle_v, v_final) < min_dist_sq) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            final_vertices.push_back(particle_v);
        }
    }

    // --- 2. ���� CDT ���󲢲�����ϴ��Ķ��� ---
    CDT::Triangulation<float> cdt;
    cdt.insertVertices(final_vertices);

    // --- 3. ׼�������뾫ȷ�ı߽�Լ���� ---
    std::vector<CDT::Edge<float>> cdt_edges;
    cdt_edges.reserve(boundary_vertices_in.size());
    for (size_t i = 0; i < boundary_vertices_in.size(); ++i) {
        CDT::VertInd idx1 = boundary_idx_map[i];
        CDT::VertInd idx2 = boundary_idx_map[(i + 1) % boundary_vertices_in.size()];
        // ������ȥ�ص��µ������ӱ�
        if (idx1 != idx2) {
            cdt_edges.emplace_back(CDT::Edge<float>{idx1, idx2});
        }
    }
    cdt.insertEdges(cdt_edges);

    // --- 4. �Ƴ�������������� ---
    cdt.eraseOuterTrianglesAndHoles();

    // --- 5. ��ȡ��� ---
    vertices_.clear();
    const auto& result_vertices = cdt.vertices;
    vertices_.reserve(result_vertices.size());
    for (const auto& v : result_vertices) {
        vertices_.emplace_back(v.x, v.y);
    }

    triangles_.clear();
    const auto& result_triangles = cdt.triangles;
    triangles_.reserve(result_triangles.size());
    for (const auto& t : result_triangles) {
        triangles_.push_back({ t.vertices[0], t.vertices[1], t.vertices[2] });
    }

    std::cout << "CDT generated: " << vertices_.size() << " vertices, " << triangles_.size() << " triangles." << std::endl;
}