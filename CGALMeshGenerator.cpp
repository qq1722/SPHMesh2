#include "CGALMeshGenerator.h"
#include <iostream>
#include <map>
#include <vector>
void mark_domains(CDT& cdt);
// [修改] generate_mesh 
void CGALMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary) {
    vertices_.clear();
    triangles_.clear();
    quads_.clear();

    const auto& outer_boundary_verts = boundary.get_outer_boundary();
    const auto& holes_list = boundary.get_holes();

    if (outer_boundary_verts.empty()) return;

    CDT cdt;

    // --- 步驟 1: 插入外邊界約束 (定义几何形状) ---
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

    // --- 步驟 3: 插入所有 SPH 粒子 (核心修复) ---
    // [修复] 我们必须插入所有粒子，包括边界粒子！
    // 即使边界粒子的位置可能与约束边重合，CGAL 也会自动处理（分裂约束边）。
    // 这样才能保证生成的网格边界顶点与 SPH 粒子一一对应。
    std::vector<Point> all_points;
    for (const auto& p : particles) {
        // 移除 if (!p.is_boundary) 的判断
        all_points.emplace_back(p.position.x, p.position.y);
    }
    // 批量插入比逐个插入效率更高
    cdt.insert(all_points.begin(), all_points.end());

    // --- 步驟 4: 標記域 (哪些三角形在 "內部") ---
    mark_domains(cdt);

    // --- 步驟 5: 提取位於域內部的三角形 ---
    // 我们需要一个映射来将 CGAL 的顶点句柄转换为我们自己的索引
    std::map<CDT::Vertex_handle, unsigned int> vertex_map;

    for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
        // `in_domain()` 逻辑：奇数层级在内部 (1, 3...), 偶数层级在外部 (0, 2...)
        // 0: 外部无限域
        // 1: 外环内部
        // 2: 内洞内部 (即多边形外部)
        if (fit->info().nesting_level % 2 == 1) {
            unsigned int v_indices[3];
            for (int i = 0; i < 3; ++i) {
                CDT::Vertex_handle vh = fit->vertex(i);

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



//void mark_domains(CDT& cdt) {
//    for (auto fit = cdt.all_faces_begin(); fit != cdt.all_faces_end(); ++fit) {
//        fit->info().nesting_level = 0;
//    }
//    std::vector<CDT::Face_handle> q;
//    q.push_back(cdt.infinite_face());
//    q.front()->info().nesting_level = 0; // 外部域
//
//    std::size_t head = 0;
//    while (head < q.size()) {
//        CDT::Face_handle fh = q[head++];
//        int current_level = fh->info().nesting_level;
//        for (int i = 0; i < 3; ++i) {
//            CDT::Face_handle nfh = fh->neighbor(i);
//            if (nfh->info().nesting_level == 0 && nfh != cdt.infinite_face()) {
//                // 嵌套级别在穿过约束边时 +1
//                if (cdt.is_constrained(CDT::Edge(fh, i))) {
//                    nfh->info().nesting_level = current_level + 1;
//                }
//                else {
//                    nfh->info().nesting_level = current_level;
//                }
//                q.push_back(nfh);
//            }
//        }
//    }
//}

// --- [新增] CGAL 域标记辅助函数 ---
// 標記 Cdt 域的嵌套級別 (这是CGAL处理带孔多边形的标准方法)
// 0 = 外部无限区域, 1 = 内部, 2 = 内部的洞, 3 = 洞里的岛...
void mark_domains(CDT& cdt) {
    for (auto fit = cdt.all_faces_begin(); fit != cdt.all_faces_end(); ++fit) {
        fit->info().nesting_level = 0;
    }

    // 从无限面（外部）开始广度优先搜索
    std::vector<CDT::Face_handle> q;
    q.push_back(cdt.infinite_face());
    q.front()->info().nesting_level = 0;

    std::size_t head = 0;
    while (head < q.size()) {
        CDT::Face_handle fh = q[head++];
        int current_level = fh->info().nesting_level;

        for (int i = 0; i < 3; ++i) {
            CDT::Face_handle nfh = fh->neighbor(i);
            // 如果邻居还没有被标记
            if (nfh->info().nesting_level == 0 && nfh != cdt.infinite_face()) {
                // 检查公共边是否是约束边
                if (cdt.is_constrained(CDT::Edge(fh, i))) {
                    // 穿过约束边，嵌套级别 +1
                    nfh->info().nesting_level = current_level + 1;
                }
                else {
                    // 未穿过约束边，级别保持不变
                    nfh->info().nesting_level = current_level;
                }
                q.push_back(nfh);
            }
        }
    }
}
