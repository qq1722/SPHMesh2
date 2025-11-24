#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Boundary.h"

class BackgroundGrid {
public:
    // [修改] 构造函数增加一个参数
   // [修改] 构造函数签名
    BackgroundGrid(const Boundary& boundary, float grid_cell_size, float refinement_level, float h_min, float h_max);

    float get_target_size(const glm::vec2& pos) const;
    // 新增：获取指定位置的目标方向 D_t
    glm::vec2 get_target_direction(const glm::vec2& pos) const;
    // --- 新增公共接口 ---
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    float get_cell_size() const { return cell_size_; }
    glm::vec2 get_min_coords() const { return min_coords_; }
    const std::vector<float>& get_target_size_field() const { return target_size_field_; }
    // --- 结束 ---
    float get_min_target_size() const { return h_min_; } // <-- 新增

private:
    void compute_fields(const Boundary& boundary);

    glm::vec2 min_coords_;
    float cell_size_;
    int width_, height_;
    std::vector<float> target_size_field_;     // 存储 h_t
    std::vector<glm::vec2> target_direction_field_; // 新增：存储 D_t
    // [新增] 存储 h_min 和 h_max
    float h_min_ = 0.1f;
    float h_max_ = 0.4f;

    // [新增] 存储用户自定义的加密层数
    float refinement_level_ = 5.0f; // 默认值
};