#include "CGALMeshGenerator.h"
#include <iostream>
#include <map>
#include <vector>
void mark_domains(CDT& cdt);
void CGALMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary) {
    vertices_.clear();
    triangles_.clear();
    quads_.clear(); // 確保四邊形列表也被清空

    const auto& outer_boundary_verts = boundary.get_outer_boundary();
    const auto& holes_list = boundary.get_holes();

    if (outer_boundary_verts.empty()) return;

    CDT cdt;

    // --- 步驟 1: 插入外邊界約束 ---
    CDT::Vertex_handle v_start;
    CDT::Vertex_handle v_prev;
    for (const auto& p : outer_boundary_verts) {
        CDT::Vertex_handle vh = cdt.insert(Point(p.x, p.y));
        if (v_prev != nullptr) {
            cdt.insert_constraint(v_prev, vh);
        }
        else {
            v_start = vh;
        }
        v_prev = vh;
    }
    cdt.insert_constraint(v_prev, v_start); // 閉合外環

    // --- 步驟 2: 插入所有內洞約束 ---
    for (const auto& hole_verts : holes_list) {
        if (hole_verts.empty()) continue;
        CDT::Vertex_handle h_v_start;
        CDT::Vertex_handle h_v_prev;
        for (const auto& p : hole_verts) {
            CDT::Vertex_handle vh = cdt.insert(Point(p.x, p.y));
            if (h_v_prev != nullptr) {
                cdt.insert_constraint(h_v_prev, vh);
            }
            else {
                h_v_start = vh;
            }
            h_v_prev = vh;
        }
        cdt.insert_constraint(h_v_prev, h_v_start); // 閉合內洞
    }

    // --- 步驟 3: 插入所有 SPH 粒子 ---
    // (我们仍然只插入內部粒子，因為邊界粒子已经作為約束頂點被插入了)
    std::vector<Point> interior_points;
    for (const auto& p : particles) {
        if (!p.is_boundary) {
            interior_points.emplace_back(p.position.x, p.position.y);
        }
    }
    cdt.insert(interior_points.begin(), interior_points.end());

 
    mark_domains(cdt);

    // --- 步驟 5: 提取位於域內部的三角形 ---
    std::map<CDT::Vertex_handle, unsigned int> vertex_map;

    for (auto face_it = cdt.finite_faces_begin(); face_it != cdt.finite_faces_end(); ++face_it) {
        // `in_domain()` 檢查 nesting_level 是否為奇數
        // (0=外部, 1=内部, 2=在内洞中, ...)
        if (face_it->info().in_domain()) {
            unsigned int v_indices[3];
            for (int i = 0; i < 3; ++i) {
                CDT::Vertex_handle vh = face_it->vertex(i);

                // 如果是新頂點，添加到我們的頂點列表並記錄其索引
                if (vertex_map.find(vh) == vertex_map.end()) {
                    vertex_map[vh] = (unsigned int)vertices_.size();
                    vertices_.emplace_back(vh->point().x(), vh->point().y());
                }
                v_indices[i] = vertex_map[vh];
            }
            // 添加这个合格的三角形
            triangles_.push_back({ v_indices[0], v_indices[1], v_indices[2] });
        }
    }

    std::cout << "CGAL generated (Constrained Domain Method): " << vertices_.size() << " vertices, " << triangles_.size() << " triangles." << std::endl;
}



void mark_domains(CDT& cdt) {
    for (auto fit = cdt.all_faces_begin(); fit != cdt.all_faces_end(); ++fit) {
        fit->info().nesting_level = 0;
    }
    std::vector<CDT::Face_handle> q;
    q.push_back(cdt.infinite_face());
    q.front()->info().nesting_level = 0; // 外部域

    std::size_t head = 0;
    while (head < q.size()) {
        CDT::Face_handle fh = q[head++];
        int current_level = fh->info().nesting_level;
        for (int i = 0; i < 3; ++i) {
            CDT::Face_handle nfh = fh->neighbor(i);
            if (nfh->info().nesting_level == 0 && nfh != cdt.infinite_face()) {
                // 嵌套级别在穿过约束边时 +1
                if (cdt.is_constrained(CDT::Edge(fh, i))) {
                    nfh->info().nesting_level = current_level + 1;
                }
                else {
                    nfh->info().nesting_level = current_level;
                }
                q.push_back(nfh);
            }
        }
    }
}