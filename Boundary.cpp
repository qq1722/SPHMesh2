#include "Boundary.h"
#include "Utils.h"
#include <algorithm> // for std::min/max
#include <cfloat>

// 构造函数: 從 Chart2D 初始化
Boundary::Boundary(const Chart2D& chart)
    : m_outer_boundary(chart.boundary), m_holes(chart.holes) // <-- [修改]
{
    calculate_aabb();
}

// get_outer_boundary() 的实现
const std::vector<glm::vec2>& Boundary::get_outer_boundary() const
{
    return m_outer_boundary;
}

// [新增] get_holes() 的实现
const std::vector<std::vector<glm::vec2>>& Boundary::get_holes() const
{
    return m_holes;
}

const glm::vec4& Boundary::get_aabb() const
{
    return aabb_;
}

void Boundary::calculate_aabb()
{
    // AABB 僅由 *外* 邊界決定
    if (m_outer_boundary.empty()) { // <-- [修改]
        aabb_ = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    glm::vec2 min_coords = m_outer_boundary[0]; // <-- [修改]
    glm::vec2 max_coords = m_outer_boundary[0]; // <-- [修改]

    for (size_t i = 1; i < m_outer_boundary.size(); ++i) // <-- [修改]
    {
        min_coords.x = std::min(min_coords.x, m_outer_boundary[i].x);
        min_coords.y = std::min(min_coords.y, m_outer_boundary[i].y);
        max_coords.x = std::max(max_coords.x, m_outer_boundary[i].x);
        max_coords.y = std::max(max_coords.y, m_outer_boundary[i].y);
    }
    aabb_ = glm::vec4(min_coords.x, min_coords.y, max_coords.x, max_coords.y);
}

// [新增] 靜態輔助函數: 原始的 is_inside 邏輯
// (我们把原来的 is_inside 逻辑提取出来)
bool Boundary::is_inside_polygon(const glm::vec2& point, const std::vector<glm::vec2>& polygon)
{
    if (polygon.empty())
    {
        return false;
    }
    int num_vertices = polygon.size();
    bool inside = false;

    for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++)
    {
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[j];
        if (((p1.y > point.y) != (p2.y > point.y)) &&
            (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x))
        {
            inside = !inside;
        }
    }
    return inside;
}

// [修改] 修改後的 is_inside: 
// 必须在外环内，且在所有内洞外
bool Boundary::is_inside(const glm::vec2& point) const
{
    // 1. 必須在外環內
    if (!is_inside_polygon(point, m_outer_boundary)) {
        return false;
    }

    // 2. 必須在所有內洞的 *外部*
    for (const auto& hole : m_holes) {
        if (is_inside_polygon(point, hole)) {
            return false; // 點在內洞裡
        }
    }

    // 在外環內，且不在任何內洞內
    return true;
}

// [新增] 实现计算最近点逻辑
glm::vec2 Boundary::get_closest_point(const glm::vec2& p) const {
    // 1. 先计算到外环的最近点
    glm::vec2 best_point = closest_point_on_polygon(p, m_outer_boundary);
    float min_dist_sq = glm::dot(p - best_point, p - best_point);

    // 2. 遍历所有内洞，看有没有更近的
    for (const auto& hole : m_holes) {
        if (hole.empty()) continue;
        glm::vec2 hole_pt = closest_point_on_polygon(p, hole);
        float dist_sq = glm::dot(p - hole_pt, p - hole_pt);

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_point = hole_pt;
        }
    }

    return best_point;
}

// [新增] 核心算法：同时寻找最近点和切线
void Boundary::get_closest_point_and_tangent(const glm::vec2& p, glm::vec2& out_closest, glm::vec2& out_tangent) const
{
    float min_dist_sq = FLT_MAX;

    // 定义一个 Lambda 函数来处理单个环（外环或内洞）
    auto process_ring = [&](const std::vector<glm::vec2>& ring) {
        if (ring.empty()) return;
        int n = (int)ring.size();
        for (int i = 0; i < n; ++i) {
            glm::vec2 a = ring[i];
            glm::vec2 b = ring[(i + 1) % n]; // 循环连接到下一个点

            glm::vec2 ab = b - a;
            float len_sq = glm::dot(ab, ab);
            if (len_sq < 1e-9f) continue; // 忽略退化边

            // 计算点 p 在线段 ab 上的投影比例 t
            float t = glm::dot(p - a, ab) / len_sq;
            t = std::max(0.0f, std::min(1.0f, t)); // 限制在线段范围内

            glm::vec2 pt = a + t * ab;
            float dist_sq = glm::dot(p - pt, p - pt);

            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                out_closest = pt;
                // 切线就是边的方向 (归一化)
                out_tangent = glm::normalize(ab);
            }
        }
        };

    // 1. 检查外环
    process_ring(m_outer_boundary);

    // 2. 检查所有内洞
    for (const auto& hole : m_holes) {
        process_ring(hole);
    }
}