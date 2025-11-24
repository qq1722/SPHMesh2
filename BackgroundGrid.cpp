#include "BackgroundGrid.h"
#include "Utils.h"
#include <algorithm>
#include <vector>
#include <glm/gtc/constants.hpp> // 为了 glm::pi

// [修改] 构造函数实现
BackgroundGrid::BackgroundGrid(const Boundary& boundary, float grid_cell_size, float refinement_level, float h_min, float h_max)
    : refinement_level_(refinement_level), h_min_(h_min), h_max_(h_max) // 存储传入的值
{
    cell_size_ = grid_cell_size;
    const auto& aabb = boundary.get_aabb();
    min_coords_ = { aabb.x, aabb.y };
    width_ = static_cast<int>((aabb.z - aabb.x) / cell_size_) + 3;
    height_ = static_cast<int>((aabb.w - aabb.y) / cell_size_) + 3;

    target_size_field_.resize(width_ * height_);
    target_direction_field_.resize(width_ * height_, { 1.0f, 0.0f });

    // 现在 h_min_ 和 h_max_ 已经有正确的值了
    compute_fields(boundary);
}

// 核心修改：计算 h_t 和 D_t
void BackgroundGrid::compute_fields(const Boundary& boundary) {
   /* const auto& boundary_vertices = boundary.get_vertices();
    if (boundary_vertices.size() < 2) return;*/

    //// --- 1. 计算 h_min 和 h_max ---
    //h_min_ = cell_size_ * 0.5f; // 边界最密集的间距 (保存为成员)

    //// 内部最稀疏的间距：如您所说 "比边缘密集稍微多一点点"
    //float h_max = h_min_ * 1.5f;

    // --- 2. 计算SDF和尺寸场 h_t (恢复您原来的 t*t 逻辑) ---
    std::vector<float> sdf(width_ * height_, FLT_MAX);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
         /*   glm::vec2 grid_pos = min_coords_ + glm::vec2(x * cell_size_, y * cell_size_);
            float dist_to_boundary = glm::distance(grid_pos, closest_point_on_polygon(grid_pos, boundary_vertices));

            float sdf_value = boundary.is_inside(grid_pos) ? dist_to_boundary : -dist_to_boundary;
            sdf[y * width_ + x] = sdf_value;*/
            glm::vec2 grid_pos = min_coords_ + glm::vec2(x * cell_size_, y * cell_size_);

            // [核心修复]
            // 使用我们之前写的、能感知内洞的 get_closest_point 函数
            glm::vec2 closest_pt = boundary.get_closest_point(grid_pos);
            float dist_to_boundary = glm::distance(grid_pos, closest_pt);

            // SDF 计算
            sdf[y * width_ + x] = boundary.is_inside(grid_pos) ? dist_to_boundary : -dist_to_boundary;

            // --- [核心修改] ---
                 // influence_radius 现在由 h_max 和你自定义的 "加密层数" 决定
                 // 比如 h_max=1.0, refinement_level=5, 则影响半径为 5.0
            float influence_radius = h_max_ * refinement_level_;
            float t = std::min(dist_to_boundary / influence_radius, 1.0f);

            // t*t 使得靠近边界(t=0)时尺寸增长缓慢 (保持h_min)
            target_size_field_[y * width_ + x] = glm::mix(h_min_, h_max_, t * t);
        }
    }

    // --- 3. 计算方向场 D_t (关键修改：使用SDF梯度，而不是切线) ---
    // 这个场从边界指向中心，对于湖泊和正方形都是稳定的。
    for (int y = 1; y < height_ - 1; ++y) {
        for (int x = 1; x < width_ - 1; ++x) {
            // 使用中心差分计算SDF梯度
            float grad_x = (sdf[y * width_ + (x + 1)] - sdf[y * width_ + (x - 1)]) / (2.0f * cell_size_);
            float grad_y = (sdf[(y + 1) * width_ + x] - sdf[(y - 1) * width_ + x]) / (2.0f * cell_size_);
            glm::vec2 grad = { grad_x, grad_y };

            if (glm::length(grad) > 1e-6f) {
                // 方向场 D_t 直接设为归一化的SDF梯度 (径向)
               /* target_direction_field_[y * width_ + x] = glm::normalize(grad);*/
                glm::vec2 tangent = { -grad.y, grad.x };
                target_direction_field_[y * width_ + x] = glm::normalize(tangent);
            }
            // 如果梯度为0 (例如在中心), 它将保持默认的 (1,0)
        }
    }
}

// 双线性插值获取任意位置的目标数据
float BackgroundGrid::get_target_size(const glm::vec2& pos) const {
    // ... (代码与上一版相同) ...
    glm::vec2 local_pos = (pos - min_coords_) / cell_size_;
    int x0 = static_cast<int>(local_pos.x);
    int y0 = static_cast<int>(local_pos.y);
    x0 = std::max(0, std::min(x0, width_ - 2));
    y0 = std::max(0, std::min(y0, height_ - 2));
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    float tx = local_pos.x - x0;
    float ty = local_pos.y - y0;
    float s00 = target_size_field_[y0 * width_ + x0];
    float s10 = target_size_field_[y0 * width_ + x1];
    float s01 = target_size_field_[y1 * width_ + x0];
    float s11 = target_size_field_[y1 * width_ + x1];
    float s_y0 = glm::mix(s00, s10, tx);
    float s_y1 = glm::mix(s01, s11, tx);
    return glm::mix(s_y0, s_y1, ty);
}

glm::vec2 BackgroundGrid::get_target_direction(const glm::vec2& pos) const {
    // ... (双线性插值) ...
    glm::vec2 local_pos = (pos - min_coords_) / cell_size_;
    int x0 = static_cast<int>(local_pos.x);
    int y0 = static_cast<int>(local_pos.y);
    x0 = std::max(0, std::min(x0, width_ - 2));
    y0 = std::max(0, std::min(y0, height_ - 2));
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    float tx = local_pos.x - x0;
    float ty = local_pos.y - y0;
    glm::vec2 d00 = target_direction_field_[y0 * width_ + x0];
    glm::vec2 d10 = target_direction_field_[y0 * width_ + x1];
    glm::vec2 d01 = target_direction_field_[y1 * width_ + x0];
    glm::vec2 d11 = target_direction_field_[y1 * width_ + x1];
    glm::vec2 d_y0 = glm::mix(d00, d10, tx);
    glm::vec2 d_y1 = glm::mix(d01, d11, tx);
    return glm::normalize(glm::mix(d_y0, d_y1, ty));
}