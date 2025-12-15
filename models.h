#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <algorithm> // for std::min/max
#include <iostream>
#include <cmath> // for sin, cos
#include <random> // for random numbers

// 新增的头文件
#include <fstream>   // 用于文件输入 (std::ifstream)
#include <sstream>   // 用于解析每行 (std::stringstream)
#include <string>    // 用于处理文件名 (std::string)


// --- 新增数据结构 ---
// 用于存储一个2D图表（面），包含一个外边界和多个（或零个）内洞
struct Chart2D {
    std::vector<glm::vec2> boundary; // 外边界
    std::vector<std::vector<glm::vec2>> holes; // 洞的列表（每个洞都是一个顶点列表）

    // 辅助函数，检查此图表是否有效加载
    bool IsValid() const {
        return !boundary.empty();
    }
};


// ----------------------------------------------------------------
// 内部辅助函数和旧函数
// ----------------------------------------------------------------

// 一个简单的函数，用于生成0到1之间的随机浮点数
inline float random_float(float min, float max) {
    static std::mt19937 generator(std::random_device{}());
    std::uniform_real_distribution<float> distribution(min, max);
    return distribution(generator);
}

// (旧函数 - 已更新以返回 Chart2D)
inline Chart2D create_complex_lake(int num_vertices, float avg_radius, float irregularity, float spikeyness) {
    Chart2D chart; // 新的返回类型
    float angle_step = 2.0f * 3.1415926535f / num_vertices;

    for (int i = 0; i < num_vertices; ++i) {
        float angle = i * angle_step;
        float base_radius = avg_radius;
        float radius = base_radius + random_float(-irregularity, irregularity);
        if (random_float(0.0f, 1.0f) < spikeyness) {
            radius *= random_float(1.1f, 1.3f);
        }
        else if (random_float(0.0f, 1.0f) < spikeyness) {
            radius *= random_float(0.7f, 0.9f);
        }
        float x = radius * cos(angle);
        float y = radius * sin(angle);
        chart.boundary.push_back({ x, y });
    }
    // .holes 列表默认为空
    return chart;
}

// (旧函数 - 已更新以返回 Chart2D 并进行归一化)
inline Chart2D get_lake_shape_vertices() {
    Chart2D chart; // 新的返回类型
    chart.boundary = {
        {2.2f, 8.1f}, {4.5f, 9.2f}, {7.3f, 8.5f}, {9.1f, 6.7f},
        {10.5f, 4.2f}, {11.2f, 1.5f}, {10.1f, -1.1f}, {8.3f, -3.2f},
        {6.1f, -4.5f}, {3.5f, -5.1f}, {1.2f, -4.2f}, {-1.5f, -3.5f},
        {-3.1f, -2.1f}, {-4.5f, 0.5f}, {-5.2f, 2.8f}, {-4.1f, 5.3f},
        {-2.5f, 7.1f}, {0.1f, 6.5f}
    };

    // 归一化处理
    if (chart.boundary.empty()) return chart;

    glm::vec2 min_coords = chart.boundary[0];
    glm::vec2 max_coords = chart.boundary[0];
    for (const auto& v : chart.boundary) {
        min_coords.x = std::min(min_coords.x, v.x);
        min_coords.y = std::min(min_coords.y, v.y);
        max_coords.x = std::max(max_coords.x, v.x);
        max_coords.y = std::max(max_coords.y, v.y);
    }
    glm::vec2 center = (min_coords + max_coords) * 0.5f;
    glm::vec2 size = max_coords - min_coords;
    float max_dim = std::max(size.x, size.y);
    float target_size = 10.0f; // 目标尺寸
    float scale = (max_dim > 1e-6) ? (target_size / max_dim) : 1.0f;
    for (auto& v : chart.boundary) {
        v = (v - center) * scale;
    }

    // .holes 列表默认为空
    return chart;
}


// --- 内部辅助函数 ---
// 从单个文件加载顶点，不进行归一化
// 假设最后一个点是与第一个点重复的，并移除它
inline std::vector<glm::vec2> load_polygon_from_file_raw(const std::string& full_path) {
    std::vector<glm::vec2> vertices;
    std::ifstream file(full_path);

    // 注意：如果文件打不开（例如，在探测hole文件时），只返回空向量是正常的
    if (!file.is_open()) {
        return vertices;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        float x, y;
        if (ss >> x >> y) {
            vertices.push_back({ x, y });
        }
    }
    file.close();

    // 移除最后一个多余的点（闭合点）
    if (!vertices.empty()) {
        vertices.pop_back();
    }
    return vertices;
}


// --- 新增的主功能函数 ---
//
// 根据模型名和图表索引，加载一个完整的2D图表（包含外环和所有内洞）
// 并对所有部分进行*统一*的归一化处理
//
// 参数:
//   model_name: 模型名称 (例如 "model", "cube")
//   chart_index: 图表索引 (例如 0, 1, ...)
//
// *** 重要假设 ***
// 根据你之前的示例文件（例如 "model_chart_0_boundary.txt"）
// 我假设完整的文件名格式是使用下划线 "_" 连接的，如下所示：
//   - 外环: exportdata/模型名_chart_索引_boundary.txt
//   - 内洞: exportdata/模型名_chart_索引_hole_K.txt  (K=0, 1, 2...)
//
//inline Chart2D load_chart_by_index(const std::string& model_name, int chart_index) {
//
//    Chart2D chart;
//    std::stringstream ss_filename;
//
//    // 1. 加载外边界
//    // 格式: exportdata/model_name_chart_index_boundary.txt
//    ss_filename << "exportdata/" << model_name << "_chart_" << chart_index << "_boundary.txt";
//    std::string boundary_path = ss_filename.str();
//
//    chart.boundary = load_polygon_from_file_raw(boundary_path);
//
//    // 如果连外边界都找不到，就没必要继续了
//    if (!chart.IsValid()) {
//        std::cerr << "Error: Could not load boundary file: " << boundary_path << std::endl;
//        return chart; // 返回无效的 chart
//    }
//
//    // 2. 循环加载所有的洞
//    int hole_k = 0;
//    while (true) {
//        ss_filename.str(""); // 清空 stringstream
//        ss_filename.clear();
//        // 格式: exportdata/model_name_chart_index_hole_k.txt
//        ss_filename << "exportdata/" << model_name << "_chart_" << chart_index << "_hole_" << hole_k << ".txt";
//        std::string hole_path = ss_filename.str();
//
//        std::vector<glm::vec2> hole_vertices = load_polygon_from_file_raw(hole_path);
//
//        if (hole_vertices.empty()) {
//            // 找不到文件，意味着这个chart的洞已经加载完毕
//            break;
//        }
//        else {
//            chart.holes.push_back(hole_vertices);
//            hole_k++;
//        }
//    }
//
//    // 3. 对所有加载的顶点（外环 + 所有内洞）进行统一归一化
//
//    glm::vec2 min_coords = chart.boundary[0];
//    glm::vec2 max_coords = chart.boundary[0];
//
//    // 用于收集所有顶点的 lambda 函数
//    auto find_bounds = [&](const std::vector<glm::vec2>& vertices) {
//        for (const auto& v : vertices) {
//            min_coords.x = std::min(min_coords.x, v.x);
//            min_coords.y = std::min(min_coords.y, v.y);
//            max_coords.x = std::max(max_coords.x, v.x);
//            max_coords.y = std::max(max_coords.y, v.y);
//        }
//        };
//
//    // 查找外环的边界
//    find_bounds(chart.boundary);
//    // 查找所有内洞的边界（这能确保内洞也被包含在总包围盒内）
//    for (const auto& hole_v : chart.holes) {
//        find_bounds(hole_v);
//    }
//
//    // 计算归一化参数
//    glm::vec2 center = (min_coords + max_coords) * 0.5f;
//    glm::vec2 size = max_coords - min_coords;
//    float max_dim = std::max(size.x, size.y);
//    float target_size = 10.0f; // 目标尺寸 (与 get_lake_shape_vertices 保持一致)
//    float scale = (max_dim > 1e-6) ? (target_size / max_dim) : 1.0f;
//
//    // 用于应用归一化的 lambda
//    auto normalize = [&](glm::vec2& v) {
//        v = (v - center) * scale;
//        };
//
//    // 对外环应用归一化
//    for (auto& v : chart.boundary) {
//        normalize(v);
//    }
//    // 【关键】对所有内洞应用 *完全相同* 的归一化
//    for (auto& hole_v : chart.holes) {
//        for (auto& v : hole_v) {
//            normalize(v);
//        }
//    }
//
//    std::cout << "Successfully loaded chart " << chart_index << " for model '" << model_name << "'." << std::endl;
//    std::cout << "  - Boundary: 1 (with " << chart.boundary.size() << " vertices)" << std::endl;
//    std::cout << "  - Holes: " << chart.holes.size() << std::endl;
//
//    return chart;
//}


// [修改] 增加 scale_override 参数，默认值为 -1.0 (表示自动计算局部缩放)
inline Chart2D load_chart_by_index(const std::string& model_name, int chart_index, float scale_override = -1.0f) {

    Chart2D chart;
    std::stringstream ss_filename;

    // ... (加载 boundary 和 holes 的代码保持不变) ...
    // 1. 加载外边界
    ss_filename << "exportdata/" << model_name << "_chart_" << chart_index << "_boundary.txt";
    std::string boundary_path = ss_filename.str();
    chart.boundary = load_polygon_from_file_raw(boundary_path);
    if (!chart.IsValid()) {
        std::cerr << "Error: Could not load boundary file: " << boundary_path << std::endl;
        return chart;
    }

    // 2. 循环加载所有的洞
    int hole_k = 0;
    while (true) {
        ss_filename.str(""); ss_filename.clear();
        ss_filename << "exportdata/" << model_name << "_chart_" << chart_index << "_hole_" << hole_k << ".txt";
        std::vector<glm::vec2> hole_vertices = load_polygon_from_file_raw(ss_filename.str());
        if (hole_vertices.empty()) break;
        chart.holes.push_back(hole_vertices);
        hole_k++;
    }

    // 3. 归一化逻辑 [核心修改]
    glm::vec2 min_coords = chart.boundary[0];
    glm::vec2 max_coords = chart.boundary[0];

    auto find_bounds = [&](const std::vector<glm::vec2>& vertices) {
        for (const auto& v : vertices) {
            min_coords.x = std::min(min_coords.x, v.x);
            min_coords.y = std::min(min_coords.y, v.y);
            max_coords.x = std::max(max_coords.x, v.x);
            max_coords.y = std::max(max_coords.y, v.y);
        }
        };

    find_bounds(chart.boundary);
    for (const auto& hole_v : chart.holes) find_bounds(hole_v);

    glm::vec2 center = (min_coords + max_coords) * 0.5f;

    float scale;
    if (scale_override > 0.0f) {
        // [新增] 如果提供了全局缩放比例，直接使用它
        scale = scale_override;
    }
    else {
        // [旧逻辑] 局部自动缩放
        glm::vec2 size = max_coords - min_coords;
        float max_dim = std::max(size.x, size.y);
        float target_size = 10.0f;
        scale = (max_dim > 1e-6) ? (target_size / max_dim) : 1.0f;
    }

    auto normalize = [&](glm::vec2& v) {
        v = (v - center) * scale; // 注意：center 仍然是局部的，保证每个图表都居中显示
        };

    for (auto& v : chart.boundary) normalize(v);
    for (auto& hole_v : chart.holes) {
        for (auto& v : hole_v) normalize(v);
    }

    return chart;
}
