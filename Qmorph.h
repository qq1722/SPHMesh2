#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "CGALMeshGenerator.h"

// Ϊ�˱���ѭ�����ã�����ֻ����ǰ������
class CGALMeshGenerator;

class Qmorph {
public:
    // �����������Ľṹ��
    struct Result {
        std::vector<CGALMeshGenerator::Quad> quads;
        std::vector<CGALMeshGenerator::Triangle> remaining_triangles;
    };

    Qmorph() = default;

    // ���ĺ���������һ���������񣬷���һ���ı���Ϊ��������
    Result run(const CGALMeshGenerator& delaunay_mesh);

private:
    // ˽�и�������
    float calculate_quad_quality(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3, const glm::vec2& p4);
};