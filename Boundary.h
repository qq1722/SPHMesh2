#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <string>

class Boundary 
{
public:
    // ���캯��������һ������߽���״�Ķ����б�
    Boundary(const std::vector<glm::vec2>& vertices);

    // �ж�һ�����Ƿ��ڱ߽��ڲ� (�����㷨)
    bool is_inside(const glm::vec2& point) const;

    // ��ȡ�߽�����ж��㣬������Ⱦ
    const std::vector<glm::vec2>& get_vertices() const;

    // ��ȡ�߽��������Χ�� (AABB)�������������
    const glm::vec4& get_aabb() const;

private:
    std::vector<glm::vec2> vertices_;
    glm::vec4 aabb_; // x_min, y_min, x_max, y_max

    void calculate_aabb(); // ˽�и������������ڼ����Χ��
};