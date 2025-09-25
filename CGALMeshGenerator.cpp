#include "CGALMeshGenerator.h"
#include <iostream>
#include <map>
#include <vector>

void CGALMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary) {
    vertices_.clear();
    triangles_.clear();

    const auto& boundary_vertices = boundary.get_vertices();
    if (particles.empty() || boundary_vertices.empty()) return;

    CDT cdt;

    // --- ���� 1: ֻʹ���ڲ����ӽ��������ʷ� ---
    std::vector<Point> interior_points;
    for (const auto& p : particles) {
        // ����ֻ������Щ�����ƶ������ӣ���Ϊ���ǵ�λ�þ������Ż�
        if (!p.is_boundary) {
            interior_points.emplace_back(p.position.x, p.position.y);
        }
    }

    if (interior_points.empty()) {
        std::cerr << "Warning: No interior particles to generate mesh from." << std::endl;
        return;
    }

    cdt.insert(interior_points.begin(), interior_points.end());


    // --- ���� 2 & 3: ɸѡ����ȡ�ڲ������� (�����жϷ�) ---
    // ���map��CGAL���ڲ�������ӳ�䵽�����Լ�����������
    std::map<CDT::Vertex_handle, unsigned int> vertex_map;

    for (auto face_it = cdt.finite_faces_begin(); face_it != cdt.finite_faces_end(); ++face_it) {
        // a) ���������ε�����
        Point p0 = face_it->vertex(0)->point();
        Point p1 = face_it->vertex(1)->point();
        Point p2 = face_it->vertex(2)->point();
        glm::vec2 centroid((p0.x() + p1.x() + p2.x()) / 3.0f, (p0.y() + p1.y() + p2.y()) / 3.0f);

        // b) ʹ�ñ߽��ж������Ƿ���������
        if (boundary.is_inside(centroid)) {
            unsigned int v_indices[3];
            for (int i = 0; i < 3; ++i) {
                CDT::Vertex_handle vh = face_it->vertex(i);

                // c) ������¶��㣬��ӵ����ǵĶ����б���¼������
                if (vertex_map.find(vh) == vertex_map.end()) {
                    vertex_map[vh] = (unsigned int)vertices_.size();
                    vertices_.emplace_back(vh->point().x(), vh->point().y());
                }
                v_indices[i] = vertex_map[vh];
            }
            // d) �������ϸ��������
            triangles_.push_back({ v_indices[0], v_indices[1], v_indices[2] });
        }
    }

    std::cout << "CGAL generated (Clip Method): " << vertices_.size() << " vertices, " << triangles_.size() << " triangles." << std::endl;
}