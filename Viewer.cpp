#include "Viewer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

// --- ���캯��������־�ļ� ---
Viewer::Viewer(int width, int height, const std::string& title)
    : width_(width), height_(height), title_(title), window_(nullptr) {
    init();
    convergence_log_.open("convergence_log.csv");
    if (convergence_log_.is_open()) {
        convergence_log_ << "Step,KineticEnergy\n"; // д��CSV��ͷ
    }
}

// --- �����������ر���־�ļ� ---
Viewer::~Viewer() {
    delete shader_;
    delete point_shader_;
    delete size_field_shader_;
    glDeleteVertexArrays(1, &VAO_boundary_); glDeleteBuffers(1, &VBO_boundary_);
    glDeleteVertexArrays(1, &VAO_particles_); glDeleteBuffers(1, &VBO_particles_);
    glDeleteVertexArrays(1, &VAO_size_field_); glDeleteBuffers(1, &VBO_size_field_);
    if (window_) { glfwDestroyWindow(window_); }
    if (convergence_log_.is_open()) {
        convergence_log_.close();
    }
    glfwTerminate();
}

void Viewer::set_background_grid(BackgroundGrid* grid) {
    grid_ = grid;
    if (grid_) {
        setup_size_field_buffers();
    }
}






// --- ��ѭ����������߼� ---
//void Viewer::main_loop() {
//    while (!glfwWindowShouldClose(window_)) {
//        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//            glfwSetWindowShouldClose(window_, true);
//
//        if (sim2d_) {
//            sim2d_->step();
//            step_count_++;
//            if (step_count_ % 10 == 0 && convergence_log_.is_open()) {
//                convergence_log_ << step_count_ << "," << sim2d_->get_kinetic_energy() << "\n";
//            }
//        }
//
//        update_particle_buffers();
//
//        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        glm::mat4 model = glm::mat4(1.0f);
//        glm::mat4 view = glm::lookAt(camera_pos_, camera_target_, camera_up_);
//        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)width_ / (float)height_, 0.1f, 100.0f);
//
//        if (show_mesh_ && cgal_generator_ && shader_) {
//            if (cgal_generator_ && shader_) {
//                shader_->use();
//                shader_->setMat4("model", model);
//                shader_->setMat4("view", view);
//                shader_->setMat4("projection", projection);
//                shader_->setVec4("color", glm::vec4(0.9f, 0.9f, 0.9f, 1.0f));
//                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//                glLineWidth(1.0f);
//
//                glBindVertexArray(VAO_mesh_);
//
//                // ����ʣ���������
//                if (!cgal_generator_->get_triangles().empty()) {
//                    glDrawElements(GL_TRIANGLES, cgal_generator_->get_triangles().size() * 3, GL_UNSIGNED_INT, 0);
//                }
//
//                // �����ı��� (��Ҫ���� EBO)
//                if (!cgal_generator_->get_quads().empty()) {
//                    std::vector<unsigned int> quad_line_indices;
//                    for (const auto& q : cgal_generator_->get_quads()) {
//                        quad_line_indices.push_back(q.v0); quad_line_indices.push_back(q.v1);
//                        quad_line_indices.push_back(q.v1); quad_line_indices.push_back(q.v2);
//                        quad_line_indices.push_back(q.v2); quad_line_indices.push_back(q.v3);
//                        quad_line_indices.push_back(q.v3); quad_line_indices.push_back(q.v0);
//                    }
//                    glBufferData(GL_ELEMENT_ARRAY_BUFFER, quad_line_indices.size() * sizeof(unsigned int), quad_line_indices.data(), GL_DYNAMIC_DRAW);
//                    glDrawElements(GL_LINES, quad_line_indices.size(), GL_UNSIGNED_INT, 0);
//                }
//
//                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//            }
//        }
//        
//        else if (show_size_field_) {
//            // --- ���ƴ�С�� ---
//            if (grid_ && size_field_shader_) {
//                size_field_shader_->use();
//                size_field_shader_->setMat4("view", view);
//                size_field_shader_->setMat4("projection", projection);
//                const auto& field = grid_->get_target_size_field();
//                if (!field.empty()) {
//                    float min_val = *std::min_element(field.begin(), field.end());
//                    float max_val = *std::max_element(field.begin(), field.end());
//                    size_field_shader_->setFloat("vmin", min_val);
//                    size_field_shader_->setFloat("vmax", max_val);
//                }
//                glBindVertexArray(VAO_size_field_);
//                glDrawArrays(GL_TRIANGLES, 0, grid_->get_width() * grid_->get_height() * 6);
//            }
//            // --- �ڴ�С��֮�ϵ��ӱ߽��� ---
//            if (boundary_ && shader_) {
//                shader_->use();
//                shader_->setMat4("view", view);
//                shader_->setMat4("projection", projection);
//                glLineWidth(1.0f);
//                glBindVertexArray(VAO_boundary_);
//                glDrawArrays(GL_LINE_LOOP, 0, boundary_->get_vertices().size());
//            }
//        }
//        
//        else {
//            // ���Ʊ߽�
//            if (boundary_ && shader_) {
//                shader_->use();
//                shader_->setMat4("model", model);
//                shader_->setMat4("view", view);
//                shader_->setMat4("projection", projection);
//                glLineWidth(1.0f);
//                glBindVertexArray(VAO_boundary_);
//                glDrawArrays(GL_LINE_LOOP, 0, boundary_->get_vertices().size());
//                glLineWidth(1.0f);
//            }
//            // ��������
//            if (sim2d_ && point_shader_) {
//                point_shader_->use();
//                shader_->setMat4("model", model);
//                point_shader_->setMat4("view", view);
//                point_shader_->setMat4("projection", projection);
//                glPointSize(1.0f);
//                glBindVertexArray(VAO_particles_);
//                if (!sim2d_->get_particle_positions().empty()) {
//                    glDrawArrays(GL_POINTS, 0, sim2d_->get_particle_positions().size());
//                }
//            }
//        }
//
//        glfwSwapBuffers(window_);
//        glfwPollEvents();
//    }
//}
void Viewer::main_loop() {
    while (!glfwWindowShouldClose(window_)) {
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window_, true);

        if (current_view_ == ViewMode::Particles || current_view_ == ViewMode::SizeField) {
            if (sim2d_) {
                sim2d_->step();
                step_count_++;
                if (step_count_ % 10 == 0 && convergence_log_.is_open()) {
                    convergence_log_ << step_count_ << "," << sim2d_->get_kinetic_energy() << "\n";
                }
                update_particle_buffers();
            }
        }

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view = glm::lookAt(camera_pos_, camera_target_, camera_up_);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)width_ / (float)height_, 0.1f, 100.0f);
        glm::mat4 model = glm::mat4(1.0f);

        switch (current_view_) {
        case ViewMode::Particles: {
            if (boundary_ && shader_) {
                shader_->use();
                shader_->setMat4("model", model); shader_->setMat4("view", view); shader_->setMat4("projection", projection);
                shader_->setVec4("color", glm::vec4(0.8f, 0.8f, 0.8f, 1.0f));
                glLineWidth(2.0f);
                glBindVertexArray(VAO_boundary_);
                glDrawArrays(GL_LINE_LOOP, 0, boundary_->get_vertices().size());
            }
            if (sim2d_ && point_shader_) {
                point_shader_->use();
                point_shader_->setMat4("model", model); point_shader_->setMat4("view", view); point_shader_->setMat4("projection", projection);
                glPointSize(4.0f);
                glBindVertexArray(VAO_particles_);
                if (!sim2d_->get_particle_positions().empty()) {
                    glDrawArrays(GL_POINTS, 0, sim2d_->get_particle_positions().size());
                }
            }
            break;
        }
        case ViewMode::SizeField: {
            if (grid_ && size_field_shader_) {
                size_field_shader_->use();
                size_field_shader_->setMat4("model", model); size_field_shader_->setMat4("view", view); size_field_shader_->setMat4("projection", projection);
                const auto& field = grid_->get_target_size_field();
                if (!field.empty()) {
                    auto minmax = std::minmax_element(field.begin(), field.end());
                    size_field_shader_->setFloat("vmin", *minmax.first);
                    size_field_shader_->setFloat("vmax", *minmax.second);
                }
                glBindVertexArray(VAO_size_field_);
                glDrawArrays(GL_TRIANGLES, 0, grid_->get_width() * grid_->get_height() * 6);
            }
            if (boundary_ && shader_) {
                shader_->use();
                shader_->setMat4("model", model); shader_->setMat4("view", view); shader_->setMat4("projection", projection);
                shader_->setVec4("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
                glLineWidth(2.5f);
                glBindVertexArray(VAO_boundary_);
                glDrawArrays(GL_LINE_LOOP, 0, boundary_->get_vertices().size());
            }
            break;
        }
        case ViewMode::Triangles: {
            if (cgal_generator_ && shader_ && !cgal_generator_->get_vertices().empty()) {
                shader_->use();
                shader_->setMat4("model", model); shader_->setMat4("view", view); shader_->setMat4("projection", projection);
                shader_->setVec4("color", glm::vec4(0.9f, 0.9f, 0.9f, 1.0f));
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(1.0f);
                glBindVertexArray(VAO_mesh_);
                if (!cgal_generator_->get_triangles().empty()) {
                    glDrawElements(GL_TRIANGLES, cgal_generator_->get_triangles().size() * 3, GL_UNSIGNED_INT, 0);
                }
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
            break;
        }
        case ViewMode::Quads: {
            if (cgal_generator_ && shader_ && !cgal_generator_->get_vertices().empty()) {
                shader_->use();
                shader_->setMat4("model", model); shader_->setMat4("view", view); shader_->setMat4("projection", projection);
                shader_->setVec4("color", glm::vec4(0.9f, 0.9f, 0.9f, 1.0f));
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(1.0f);
                glBindVertexArray(VAO_mesh_);
                size_t total_indices = (quads_.size() * 4 + remaining_triangles_.size() * 3) * 2;
                if (total_indices > 0) {
                    glDrawElements(GL_LINES, total_indices, GL_UNSIGNED_INT, 0);
                }
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
            break;
        }
        }
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
}



// --- �������� ---

void Viewer::update_mesh_buffers() {
    if (!cgal_generator_ || cgal_generator_->get_vertices().empty()) return;

    glBindVertexArray(VAO_mesh_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_mesh_);
    glBufferData(GL_ARRAY_BUFFER, cgal_generator_->get_vertices().size() * sizeof(glm::vec2), cgal_generator_->get_vertices().data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_mesh_);

    // �ؼ���ֻ���ݵ�ǰӦ����ʾ��������׼��EBO
    if (current_view_ == ViewMode::Triangles) {
        const auto& tris = cgal_generator_->get_triangles();
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, tris.size() * sizeof(CGALMeshGenerator::Triangle), tris.data(), GL_STATIC_DRAW);
    }
    else if (current_view_ == ViewMode::Quads) {
        std::vector<unsigned int> line_indices;
        line_indices.reserve(quads_.size() * 8 + remaining_triangles_.size() * 6);
        for (const auto& q : quads_) {
            line_indices.push_back(q.v0); line_indices.push_back(q.v1);
            line_indices.push_back(q.v1); line_indices.push_back(q.v2);
            line_indices.push_back(q.v2); line_indices.push_back(q.v3);
            line_indices.push_back(q.v3); line_indices.push_back(q.v0);
        }
        for (const auto& t : remaining_triangles_) {
            line_indices.push_back(t.v0); line_indices.push_back(t.v1);
            line_indices.push_back(t.v1); line_indices.push_back(t.v2);
            line_indices.push_back(t.v2); line_indices.push_back(t.v0);
        }
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, line_indices.size() * sizeof(unsigned int), line_indices.data(), GL_DYNAMIC_DRAW);
    }
    glBindVertexArray(0);
}

void Viewer::set_cgal_generator(CGALMeshGenerator* generator) {
    cgal_generator_ = generator;
    if (cgal_generator_) {
        glGenVertexArrays(1, &VAO_mesh_);
        glGenBuffers(1, &VBO_mesh_);
        glGenBuffers(1, &EBO_mesh_);

        glBindVertexArray(VAO_mesh_);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_mesh_);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }
}

//void Viewer::toggle_view_mode() {
//    show_size_field_ = !show_size_field_;
//    if (show_size_field_) {
//        std::cout << "Displaying: Size Field (h_t). Press V to switch back to particles." << std::endl;
//    }
//    else {
//        std::cout << "Displaying: Particles. Press V to switch to Size Field." << std::endl;
//    }
//}

void Viewer::save_particle_snapshot() {
    if (!sim2d_) return;
    std::string filename = "particles_step_" + std::to_string(step_count_) + ".txt";
    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << "x,y\n"; // CSV header
        for (const auto& pos : sim2d_->get_particle_positions()) {
            outfile << pos.x << "," << pos.y << "\n";
        }
        outfile.close();
        std::cout << "Saved particle snapshot to " << filename << std::endl;
    }
}

void Viewer::setup_size_field_buffers() {
    if (!grid_) return;
    size_field_shader_ = new Shader("shaders/size_field.vert", "shaders/size_field.frag");

    struct Vertex {
        glm::vec2 pos;
        float value;
    };

    std::vector<Vertex> vertices;
    int w = grid_->get_width();
    int h = grid_->get_height();
    float cs = grid_->get_cell_size();
    glm::vec2 min_c = grid_->get_min_coords();
    const auto& field = grid_->get_target_size_field();

    for (int y = 0; y < h - 1; ++y) {
        for (int x = 0; x < w - 1; ++x) {
            // ÿ�����Ӵ�������������
            Vertex v0 = { min_c + glm::vec2(x * cs, y * cs),       field[y * w + x] };
            Vertex v1 = { min_c + glm::vec2((x + 1) * cs, y * cs),   field[y * w + x + 1] };
            Vertex v2 = { min_c + glm::vec2(x * cs, (y + 1) * cs),   field[(y + 1) * w + x] };
            Vertex v3 = { min_c + glm::vec2((x + 1) * cs, (y + 1) * cs), field[(y + 1) * w + x + 1] };
            vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2);
            vertices.push_back(v1); vertices.push_back(v3); vertices.push_back(v2);
        }
    }

    glGenVertexArrays(1, &VAO_size_field_);
    glGenBuffers(1, &VBO_size_field_);
    glBindVertexArray(VAO_size_field_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_size_field_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
    // position
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);
    // value (h_t)
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, value));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}
// --- ��������̬���̻ص����� ---

void Viewer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (!viewer || action != GLFW_PRESS) return;

    if (key == GLFW_KEY_V) {
        // ������ģʽ��ѭ���л�
        viewer->current_view_ = static_cast<ViewMode>((static_cast<int>(viewer->current_view_) + 1) % 4);
        // --- �ؼ��޸����л����κ�������ͼ�󣬶�������»����� ---
        if (viewer->current_view_ == ViewMode::Triangles || viewer->current_view_ == ViewMode::Quads) {
            viewer->update_mesh_buffers();
        }
    }
  

    if (key == GLFW_KEY_S) {
        viewer->save_particle_snapshot();
    }

    if (key == GLFW_KEY_C) {
        if (!viewer->cgal_generator_ || !viewer->sim2d_ || !viewer->boundary_) return;

        // �����ǰ����������ģʽ����ת��Ϊ�ı���
        if (viewer->current_view_ == ViewMode::Triangles) {
            if (viewer->qmorph_converter_) {
                auto result = viewer->qmorph_converter_->run(*viewer->cgal_generator_);
                viewer->quads_ = result.quads;
                viewer->remaining_triangles_ = result.remaining_triangles;
                viewer->current_view_ = ViewMode::Quads;
                viewer->update_mesh_buffers(); // �ڸı�״̬�����
            }
        }
        // ���򣬴�����/��С�����ɳ�ʼ��������
        else {
            viewer->cgal_generator_->generate_mesh(viewer->sim2d_->get_particles(), *viewer->boundary_);
            viewer->quads_.clear();
            viewer->remaining_triangles_.clear();
            viewer->current_view_ = ViewMode::Triangles;
            viewer->update_mesh_buffers(); // �ڸı�״̬�����
        }
    }
}



void Viewer::set_boundary(Boundary* boundary) {
    boundary_ = boundary;
    if (boundary_) setup_boundary_buffers();
}

void Viewer::set_simulation2d(Simulation2D* sim) {
    sim2d_ = sim;
    if (sim2d_) {
        glGenVertexArrays(1, &VAO_particles_);
        glGenBuffers(1, &VBO_particles_);
        glBindVertexArray(VAO_particles_);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_particles_);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }
}

//void Viewer::set_mesh_generator2d(MeshGenerator2D* generator) {
//    generator2d_ = generator;
//    if (generator2d_) {
//        glGenVertexArrays(1, &VAO_mesh_);
//        glGenBuffers(1, &VBO_mesh_);
//        glGenBuffers(1, &EBO_mesh_);
//        glBindVertexArray(VAO_mesh_);
//        glBindBuffer(GL_ARRAY_BUFFER, VBO_mesh_);
//        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
//        glEnableVertexAttribArray(0);
//        glBindVertexArray(0);
//    }
//}

void Viewer::set_qmorph_generator(Qmorph* converter) {
    qmorph_converter_ = converter;
}


// --- �o�B���{���� ---

void Viewer::setup_boundary_buffers() {
    if (!boundary_) return;
    glGenVertexArrays(1, &VAO_boundary_);
    glGenBuffers(1, &VBO_boundary_);
    glBindVertexArray(VAO_boundary_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_boundary_);
    glBufferData(GL_ARRAY_BUFFER, boundary_->get_vertices().size() * sizeof(glm::vec2), boundary_->get_vertices().data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}


void Viewer::run() {
    shader_ = new Shader("shaders/simple.vert", "shaders/simple.frag");
    point_shader_ = new Shader("shaders/point.vert", "shaders/point.frag");
    update_camera_vectors();
    main_loop();
}
void Viewer::init() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { return; }
    glfwSetWindowUserPointer(window_, this); // �ؼ�����thisָ���봰�ڹ���

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, key_callback); // <-- ע����̻ص�
    glfwSetFramebufferSizeCallback(window_, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, cursor_pos_callback);
    glfwSetScrollCallback(window_, scroll_callback);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void Viewer::update_camera_vectors() {
    float x = camera_target_.x + camera_radius_ * cos(glm::radians(camera_yaw_)) * cos(glm::radians(camera_pitch_));
    float y = camera_target_.y + camera_radius_ * sin(glm::radians(camera_pitch_));
    float z = camera_target_.z + camera_radius_ * sin(glm::radians(camera_yaw_)) * cos(glm::radians(camera_pitch_));
    camera_pos_ = glm::vec3(x, y, z);
}
void Viewer::update_particle_buffers() {
    if (!sim2d_ || sim2d_->get_particle_positions().empty()) return;
    glBindVertexArray(VAO_particles_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_particles_);
    glBufferData(GL_ARRAY_BUFFER, sim2d_->get_particle_positions().size() * sizeof(glm::vec2), sim2d_->get_particle_positions().data(), GL_DYNAMIC_DRAW);
    glBindVertexArray(0);
}

void Viewer::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer) { viewer->width_ = width; viewer->height_ = height; }
}
void Viewer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer && button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            viewer->mouse_left_pressed_ = true;
            glfwGetCursorPos(window, &viewer->last_x_, &viewer->last_y_);
        }
        else if (action == GLFW_RELEASE) {
            viewer->mouse_left_pressed_ = false;
        }
    }
}
void Viewer::cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer && viewer->mouse_left_pressed_) {
        float xoffset = xpos - viewer->last_x_;
        float yoffset = viewer->last_y_ - ypos;
        viewer->last_x_ = xpos;
        viewer->last_y_ = ypos;
        float sensitivity = 0.2f;
        viewer->camera_yaw_ += xoffset * sensitivity;
        viewer->camera_pitch_ += yoffset * sensitivity;
        if (viewer->camera_pitch_ > 89.0f) viewer->camera_pitch_ = 89.0f;
        if (viewer->camera_pitch_ < -89.0f) viewer->camera_pitch_ = -89.0f;
        viewer->update_camera_vectors();
    }
}
void Viewer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer) {
        viewer->camera_radius_ -= (float)yoffset * 0.5f;
        if (viewer->camera_radius_ < 1.0f) viewer->camera_radius_ = 1.0f;
        if (viewer->camera_radius_ > 40.0f) viewer->camera_radius_ = 40.0f;
        viewer->update_camera_vectors();
    }
}