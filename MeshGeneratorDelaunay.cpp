#include "MeshGeneratorDelaunay.h"
#include "Simulation2D.h"
#include "triangle.h"
#include <vector>

// *** �����ޏͣ��ڰ����^�ļ�֮ǰ�����x���б�Ҫ�ĺ� ***
#define TRILIBRARY
#define REAL double
extern "C" {
#include "triangle.h"
}

void MeshGeneratorDelaunay::triangulate1(const Simulation2D& sim) {
    vertices_.clear();
    triangles_.clear();

    const auto& particles = sim.get_particle_positions();
    if (particles.empty()) return;

    vertices_ = particles;

    // --- �ʂ� Triangle �������ݔ�딵�� ---
    std::vector<REAL> points_for_triangle; // ʹ�� REAL ������ double
    points_for_triangle.reserve(vertices_.size() * 2);
    for (const auto& v : vertices_) {
        points_for_triangle.push_back(v.x);
        points_for_triangle.push_back(v.y);
    }

    // --- ��ʼ��ݔ���ݔ���Y���w ---
    struct triangulateio in, out, vorout;

    // ��ʼ������ṹ��
    in.numberofpoints = static_cast<int>(vertices_.size());
    in.pointlist = points_for_triangle.data();
    in.numberofpointattributes = 0;
    in.pointattributelist = nullptr;
    in.pointmarkerlist = nullptr;
    in.numberofsegments = 0;
    in.segmentlist = nullptr;
    in.segmentmarkerlist = nullptr;
    in.numberofholes = 0;
    in.holelist = nullptr;
    in.numberofregions = 0;
    in.regionlist = nullptr;

    // ��ʼ������ṹ��
    out.pointlist = nullptr;
    out.pointattributelist = nullptr;
    out.pointmarkerlist = nullptr;
    out.trianglelist = nullptr;
    out.triangleattributelist = nullptr;
    out.neighborlist = nullptr;
    out.segmentlist = nullptr;
    out.segmentmarkerlist = nullptr;
    out.edgelist = nullptr;
    out.edgemarkerlist = nullptr;

    // ��ʼ��Voronoi����ṹ�壨��ʹ��ʹ�ã�
    vorout.pointlist = nullptr;
    vorout.pointattributelist = nullptr;
    vorout.pointmarkerlist = nullptr;
    vorout.trianglelist = nullptr;
    vorout.triangleattributelist = nullptr;
    vorout.neighborlist = nullptr;
    vorout.segmentlist = nullptr;
    vorout.segmentmarkerlist = nullptr;
    vorout.edgelist = nullptr;
    vorout.edgemarkerlist = nullptr;
    char* zq = NULL;
    // --- ʹ��4�������{�� triangulate ---
    ::triangulate(const_cast<char*>("zQ"), &in, &out, &vorout);

    // --- ��ݔ���Y������ȡ���������� ---
    triangles_.reserve(out.numberoftriangles);
    for (int i = 0; i < out.numberoftriangles; ++i) {
        triangles_.push_back({
            static_cast<unsigned int>(out.trianglelist[i * 3 + 0]),
            static_cast<unsigned int>(out.trianglelist[i * 3 + 1]),
            static_cast<unsigned int>(out.trianglelist[i * 3 + 2])
            });
    }

    // --- ጷŃȴ� ---
    free(out.pointlist);
    free(out.pointattributelist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.triangleattributelist);
    free(out.neighborlist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);
    free(out.edgelist);
    free(out.edgemarkerlist);

    // ጷ�Voronoiݔ���ȴ�
    free(vorout.pointlist);
    free(vorout.pointattributelist);
    free(vorout.pointmarkerlist);
    free(vorout.trianglelist);
    free(vorout.triangleattributelist);
    free(vorout.neighborlist);
    free(vorout.segmentlist);
    free(vorout.segmentmarkerlist);
    free(vorout.edgelist);
    free(vorout.edgemarkerlist);
}