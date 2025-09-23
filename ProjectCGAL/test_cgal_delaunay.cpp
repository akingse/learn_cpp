#include "pch.h"

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <Eigen/Dense>
//#include <iostream>
//#include <vector>

using namespace std;
using namespace clash;
using namespace Eigen;
using namespace std::chrono;
//#define  CGAL_USE_BASIC_VIEWER

// ��Delaunay�����ʷֵĽ��ת�浽TriMesh��
//TriMesh createTriMesh(const std::vector<Eigen::Vector3d>& points) 

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;

// ��Eigen::Vector2dת��ΪCGAL��
Point_2 eigen_to_cgal(const Eigen::Vector2d& p) {
    return Point_2(p.x(), p.y());
}

// ��CGAL��ת��ΪEigen::Vector2d
Eigen::Vector2d cgal_to_eigen(const Point_2& p) {
    return Eigen::Vector2d(p.x(), p.y());
}

// Delaunay�����ʷ�������
std::vector<Eigen::Vector3i> delaunay_triangulation(const std::vector<Eigen::Vector2d>& points)
{

    // 1. ����CGAL�㼯
    std::vector<Point_2> cgal_points;
    cgal_points.reserve(points.size());

    for (const auto& p : points) {
        cgal_points.push_back(eigen_to_cgal(p));
    }

    // 2. ����Delaunay�����ʷ�
    Delaunay dt;
    dt.insert(cgal_points.begin(), cgal_points.end());

    // 3. ��ȡ����������
    std::vector<Eigen::Vector3i> triangles;
    triangles.reserve(dt.number_of_faces());

    // ����������ӳ��
    std::map<Point_2, int> point_index_map;
    int index = 0;
    for (const auto& p : cgal_points) {
        point_index_map[p] = index++;
    }

    // ��������������
    for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
        // ��ȡ�����ε���������
        const Point_2& p0 = face->vertex(0)->point();
        const Point_2& p1 = face->vertex(1)->point();
        const Point_2& p2 = face->vertex(2)->point();

        // ��ȡ��������
        int i0 = point_index_map[p0];
        int i1 = point_index_map[p1];
        int i2 = point_index_map[p2];

        // ����������������ȷ����ʱ��˳��
        triangles.emplace_back(i0, i1, i2);
    }
    return triangles;
}

static void readTerrainDataToMesh_csv1()
{
    string filename = R"(C:\Users\Aking\source\repos\bimbase\src\P3d2Stl\OutputObj\modelmesh_terrain0.obj)";
    std::vector<ModelMesh> meshVct = ModelMesh::readFromFile(filename);
    if (meshVct.empty())
        return;
    ModelMesh mesh = meshVct[0];
    mesh.to2D();
    std::chrono::steady_clock::time_point timestart, timeend;

    timestart = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::Vector3i> ibo = delaunay_triangulation(mesh.vbo2_);
    timeend = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double, std::milli>(timeend - timestart).count();
    cout << "duration=" << duration << endl;
    mesh.ibo_ = ibo;
    ModelMesh::writeToFile({ mesh });

    return;
}

static void readTerrainDataToMesh_csv2()
{
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2d> points2d;
    for (int y = 0; y < 20; y++)
    {
        for (int x = 0; x < 20; x++)
        {
            points.push_back(Vector3d(x, y, 0));
            points2d.push_back(Vector2d(x, y));
        }
    }
    ModelMesh mesh;
    mesh.vbo_ = points;
    std::vector<Eigen::Vector3i> ibo = delaunay_triangulation(points2d);
    mesh.ibo_ = ibo;
    ModelMesh::writeToFile({ mesh });
    //TriMesh trimesh = createTriMesh(points);
}


static int _enrol = []()
    {
        readTerrainDataToMesh_csv1();
        //readTerrainDataToMesh_csv2();
        return 0;
    }();
