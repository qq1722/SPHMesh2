#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Simulation2D.h"
#include "Boundary.h"

// --- �ؼ��޸���������ȷ��CGAL���� ---
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h> // <-- ����������ͷ�ļ�

// ������/����
struct FaceInfo2 {
    bool in_domain() const { return nesting_level % 2 == 1; }
    int nesting_level = 0;
};

// CGAL �ں�
using K = CGAL::Exact_predicates_inexact_constructions_kernel;

// ����һ�����ϵ�����ࣺ������Լ���ߵ���Ϣ��Ҳ�������Զ������Ϣ
using Cfb = CGAL::Constrained_triangulation_face_base_2<K>;
using Fbb = CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K, Cfb>;

// ��������ݽṹ
using Vb = CGAL::Triangulation_vertex_base_2<K>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fbb>;

// ���յ�Լ�������������ʷ�����
using CDT = CGAL::Constrained_Delaunay_triangulation_2<K, Tds>;
using Point = CDT::Point;
// --- ���� ---


class CGALMeshGenerator {
public:
    struct Triangle {
        unsigned int v0, v1, v2;
    };
    // Quad �ṹ�嶨���Ƶ� Qmorph.h �и����ʣ���Ϊ�� Viewer �ܷ��ʣ���ʱ����
    struct Quad {
        unsigned int v0, v1, v2, v3;
    };

    CGALMeshGenerator() = default;
    ~CGALMeshGenerator() = default;

    // ����ǩ����������ȷ��
    void generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary);

    const std::vector<glm::vec2>& get_vertices() const { return vertices_; }
    const std::vector<Triangle>& get_triangles() const { return triangles_; }
    const std::vector<Quad>& get_quads() const { return quads_; }

private:
    std::vector<glm::vec2> vertices_;
    std::vector<Triangle> triangles_;
    std::vector<Quad> quads_; // ����

};