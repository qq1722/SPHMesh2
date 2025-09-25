#pragma once
#include <vector>
#include <glm/glm.hpp>

class Simulation2D; // ǰ����

class MeshGeneratorDelaunay {
public:
    // ��춃���һ�������ε�������c����
    struct Triangle {
        unsigned int v0, v1, v2;
    };

    MeshGeneratorDelaunay() = default;

    // ���Ĺ��ܣ����������c�K���еڃ����ǻ�
    void triangulate1(const Simulation2D& sim);

    const std::vector<glm::vec2>& get_vertices() const { return vertices_; }
    const std::vector<Triangle>& get_triangles() const { return triangles_; }

private:
    std::vector<glm::vec2> vertices_;
    std::vector<Triangle> triangles_;
};