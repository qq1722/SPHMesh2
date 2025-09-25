#pragma once
#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <glm/glm.hpp>
#include <string>
#include <fstream>

#include "Shader.h"
#include "Boundary.h"
#include "Simulation2D.h"
#include "MeshGenerator2D.h"
#include "BackgroundGrid.h" 
//#include "DelaunayMeshGenerator.h" 
#include "CGALMeshGenerator.h"
#include "qmorph.h"

class Viewer {
public:
    Viewer(int width, int height, const std::string& title);
    ~Viewer();

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    void set_boundary(Boundary* boundary);
    void set_simulation2d(Simulation2D* sim);
    void set_mesh_generator2d(MeshGenerator2D* generator);
    void run();
    void set_background_grid(BackgroundGrid* grid); // ����
    void set_cgal_generator(CGALMeshGenerator* generator); // ����
    void set_qmorph_generator(Qmorph* converter);

private:
    void init();
    void main_loop();
    //void process_input();
    // ���������̻ص�����
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    void update_camera_vectors();

    void setup_boundary_buffers();
    void update_particle_buffers();
    void update_mesh_buffers();

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    // ���������ӻ�ģʽ�л������ݵ���
    void toggle_view_mode();
    void save_particle_snapshot();
    // ������Ϊ��С�����û�����
    void setup_size_field_buffers();

private:
    GLFWwindow* window_;
    int width_, height_;
    std::string title_;

    Shader* shader_ = nullptr;
    Shader* point_shader_ = nullptr;

    Shader* size_field_shader_ = nullptr;
    unsigned int VAO_size_field_ = 0, VBO_size_field_ = 0;
    BackgroundGrid* grid_ = nullptr;
    // �����߼�
   // bool show_size_field_ = false;
    int step_count_ = 0;
    std::ofstream convergence_log_;

    CGALMeshGenerator* cgal_generator_ = nullptr;
    unsigned int VAO_mesh_ = 0, VBO_mesh_ = 0, EBO_mesh_ = 0;
  //  bool show_mesh_ = false; // ����������������ʾ

    Qmorph* qmorph_converter_ = nullptr; // ����
    // �������洢Q-Morphת�����
    std::vector<CGALMeshGenerator::Quad> quads_;
    std::vector<CGALMeshGenerator::Triangle> remaining_triangles_;


    unsigned int VAO_boundary_ = 0, VBO_boundary_ = 0;
    unsigned int VAO_particles_ = 0, VBO_particles_ = 0;
   

    Boundary* boundary_ = nullptr;
    Simulation2D* sim2d_ = nullptr;
   // MeshGenerator2D* generator2d_ = nullptr;

    glm::vec3 camera_target_ = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 camera_up_ = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 camera_pos_;
    float camera_radius_ = 15.0f;
    float camera_yaw_ = -90.0f;
    float camera_pitch_ = 0.0f;

    bool mouse_left_pressed_ = false;
    double last_x_, last_y_;

    float h_ = 0.10f;

    enum class ViewMode { Particles, SizeField, Triangles, Quads }; // <-- ���� Quads ģʽ
    ViewMode current_view_ = ViewMode::Particles;
};