#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <string>
#include "models.h" // <-- [新增] 包含 Chart2D 結構定義

class Boundary
{
public:
    // 构造函数: 接收一个 Chart2D 对象 (包含外环和内洞)
    Boundary(const Chart2D& chart); // <-- [修改]

    // 判断一个点是否在边界内部 (现在会考虑内洞)
    bool is_inside(const glm::vec2& point) const;

    // 获取 *外* 边界的所有顶点 (用于渲染)
    // 注意：旧的 get_vertices() 已重命名为 get_outer_boundary()
    const std::vector<glm::vec2>& get_outer_boundary() const; // <-- [修改]

    // 获取内洞列表 (用于網格生成)
    const std::vector<std::vector<glm::vec2>>& get_holes() const; // <-- [新增]

    // 获取边界的轴对齐包围盒 (AABB)
    const glm::vec4& get_aabb() const;

    // --- 为了兼容旧代码 ---
    // 旧的 get_vertices() 函数现在只返回外环
    const std::vector<glm::vec2>& get_vertices() const {
        return m_outer_boundary;
    }
    // --- [新增] 核心修复功能 ---
    // 计算点 p 到整个边界系统（外环或任意内洞）的最近点
    glm::vec2 get_closest_point(const glm::vec2& p) const;
    // [新增] 获取最近点 *以及* 该处的切线单位向量
    void get_closest_point_and_tangent(const glm::vec2& p, glm::vec2& out_closest, glm::vec2& out_tangent) const;

private:
    std::vector<glm::vec2> m_outer_boundary; // <-- [修改]
    std::vector<std::vector<glm::vec2>> m_holes; // <-- [新增]
    glm::vec4 aabb_; // x_min, y_min, x_max, y_max

    void calculate_aabb(); // 私有輔助函數

    // 新增: 靜態輔助函數，用於 Ray-Casting
    static bool is_inside_polygon(const glm::vec2& point, const std::vector<glm::vec2>& polygon);
};