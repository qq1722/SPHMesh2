#include "Viewer.h"
#include "Boundary.h"
#include "Simulation2D.h"
#include "models.h"
#include "qmorph.h"
#include "CGALMeshGenerator.h"
#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <string>
#include <filesystem> // C++17 标准库，用于文件系统操作
#include <map>
#include <set>

namespace fs = std::filesystem;

// 用于存储扫描到的模型信息
struct ChartInfo {
    bool has_holes = false;
};

struct ModelData {
    std::string name;
    std::map<int, ChartInfo> charts; // Key: Chart Index, Value: Info
};

// 扫描 exportdata 文件夹并解析文件
std::map<std::string, ModelData> scan_export_data() {
    std::map<std::string, ModelData> models;
    std::string dir_path = "exportdata";

    if (!fs::exists(dir_path)) {
        std::cerr << "Warning: Directory '" << dir_path << "' does not exist. Creating it." << std::endl;
        fs::create_directory(dir_path);
        return models;
    }

    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (!entry.is_regular_file()) continue;

        std::string filename = entry.path().filename().string();

        // 简单的文件名解析：查找 "_chart_"
        // 假设格式: <ModelName>_chart_<Index>_<Type>.txt
        size_t chart_pos = filename.find("_chart_");
        if (chart_pos == std::string::npos) continue;

        std::string model_name = filename.substr(0, chart_pos);
        std::string rest = filename.substr(chart_pos + 7); // 跳过 "_chart_" (7 chars)

        // 解析 Index
        size_t underscore_pos = rest.find('_');
        if (underscore_pos == std::string::npos) continue;

        try {
            int chart_idx = std::stoi(rest.substr(0, underscore_pos));
            std::string suffix = rest.substr(underscore_pos + 1);

            // 标记模型和 Chart 存在
            models[model_name].name = model_name;

            // 确保 Chart 条目存在
            if (models[model_name].charts.find(chart_idx) == models[model_name].charts.end()) {
                models[model_name].charts[chart_idx] = ChartInfo{};
            }

            // 检查是否为孔洞文件
            if (suffix.find("hole") != std::string::npos) {
                models[model_name].charts[chart_idx].has_holes = true;
            }
        }
        catch (...) {
            continue; // 解析数字失败，跳过
        }
    }
    return models;
}

int main() {
    // --- 1. 扫描模型 ---
    std::cout << "Scanning 'exportdata' folder..." << std::endl;
    auto models = scan_export_data();

    if (models.empty()) {
        std::cout << "No models found in 'exportdata'. Please ensure files are named 'Name_chart_N_boundary.txt'." << std::endl;
        std::cout << "Press Enter to exit...";
        std::cin.get();
        return -1;
    }

    // --- 2. 用户选择模型 ---
    std::vector<std::string> model_names;
    int idx = 1;
    std::cout << "\nAvailable Models:" << std::endl;
    for (const auto& pair : models) {
        std::cout << "  [" << idx << "] " << pair.first << std::endl;
        model_names.push_back(pair.first);
        idx++;
    }

    int model_choice = 0;
    while (true) {
        std::cout << "\nSelect a model (1-" << model_names.size() << "): ";
        std::cin >> model_choice;
        if (std::cin.fail() || model_choice < 1 || model_choice > model_names.size()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Please try again." << std::endl;
        }
        else {
            break;
        }
    }

    std::string selected_model_name = model_names[model_choice - 1];
    ModelData& selected_model = models[selected_model_name];

    // --- 3. 用户选择 Chart ---
    std::vector<int> chart_indices;
    idx = 1;
    std::cout << "\nAvailable Charts for '" << selected_model_name << "':" << std::endl;
    for (const auto& pair : selected_model.charts) {
        int c_idx = pair.first;
        bool holes = pair.second.has_holes;
        std::cout << "  [" << idx << "] Chart " << c_idx;
        if (holes) {
            std::cout << " (With Holes / 有环)";
        }
        std::cout << std::endl;
        chart_indices.push_back(c_idx);
        idx++;
    }

    int chart_choice = 0;
    while (true) {
        std::cout << "\nSelect a chart (1-" << chart_indices.size() << "): ";
        std::cin >> chart_choice;
        if (std::cin.fail() || chart_choice < 1 || chart_choice > chart_indices.size()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input." << std::endl;
        }
        else {
            break;
        }
    }

    int selected_chart_index = chart_indices[chart_choice - 1];

    // --- 4. 加载数据 ---
    std::cout << "\nLoading " << selected_model_name << " / Chart " << selected_chart_index << " ..." << std::endl;

    Chart2D active_chart = load_chart_by_index(selected_model_name, selected_chart_index);

    if (!active_chart.IsValid()) {
        std::cerr << "Error: Failed to load boundary data." << std::endl;
        return -1;
    }

    // --- 5. 创建并运行模拟 ---
    Boundary boundary(active_chart);
    Simulation2D sim(boundary);
    CGALMeshGenerator generator;
    Qmorph qmorph_converter;

    Viewer viewer(1280, 720, "SPH Remeshing - Dynamic Mesh Generation");

    viewer.set_boundary(&boundary);
    viewer.set_simulation2d(&sim);
    viewer.set_background_grid(sim.get_background_grid());
    viewer.set_cgal_generator(&generator);
    viewer.set_qmorph_generator(&qmorph_converter);

    viewer.run();

    return 0;
}