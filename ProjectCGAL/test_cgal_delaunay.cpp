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

// 将Delaunay三角剖分的结果转存到TriMesh中
//TriMesh createTriMesh(const std::vector<Eigen::Vector3d>& points) 

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;

// 将Eigen::Vector2d转换为CGAL点
Point_2 eigen_to_cgal(const Eigen::Vector2d& p) {
    return Point_2(p.x(), p.y());
}

// 将CGAL点转换为Eigen::Vector2d
Eigen::Vector2d cgal_to_eigen(const Point_2& p) {
    return Eigen::Vector2d(p.x(), p.y());
}

// Delaunay三角剖分主函数
std::vector<Eigen::Vector3i> delaunay_triangulation(const std::vector<Eigen::Vector2d>& points)
{

    // 1. 创建CGAL点集
    std::vector<Point_2> cgal_points;
    cgal_points.reserve(points.size());

    for (const auto& p : points) {
        cgal_points.push_back(eigen_to_cgal(p));
    }

    // 2. 创建Delaunay三角剖分
    Delaunay dt;
    dt.insert(cgal_points.begin(), cgal_points.end());

    // 3. 提取三角形索引
    std::vector<Eigen::Vector3i> triangles;
    triangles.reserve(dt.number_of_faces());

    // 创建点索引映射
    std::map<Point_2, int> point_index_map;
    int index = 0;
    for (const auto& p : cgal_points) {
        point_index_map[p] = index++;
    }

    // 遍历所有有限面
    for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
        // 获取三角形的三个顶点
        const Point_2& p0 = face->vertex(0)->point();
        const Point_2& p1 = face->vertex(1)->point();
        const Point_2& p2 = face->vertex(2)->point();

        // 获取顶点索引
        int i0 = point_index_map[p0];
        int i1 = point_index_map[p1];
        int i2 = point_index_map[p2];

        // 保存三角形索引（确保逆时针顺序）
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
    double area2 = computeTriangleArea(array<Vector2d, 3>{ points2d[0], points2d[22], points2d[33] });
    double area3 = computeTriangleArea(array<Vector3d, 3>{ points[0], points[22], points[33] });
    ModelMesh mesh;
    mesh.vbo_ = points;
    std::vector<Eigen::Vector3i> ibo = delaunay_triangulation(points2d);
    mesh.ibo_ = ibo;
    ModelMesh::writeToFile({ mesh });
    //TriMesh trimesh = createTriMesh(points);
}

static ModelMesh read_obj_mesh(std::string filename) //to python
{
    std::vector<ModelMesh> meshVct = ModelMesh::readFromFile(filename);
    return meshVct[0];
}

//简化
static void readTerrainDataToMesh_csv3()
{
    string filename = R"(C:\Users\Aking\source\repos\learn_cpp\ProjectCGAL\OutputObj\modelmesh_528808828.obj)";
    ModelMesh mesh = read_obj_mesh(filename);
    ModelMesh simmesh;
    simmesh.vbo_ = mesh.vbo_;
    for (const auto& face : mesh.ibo_)
    {
        array<Eigen::Vector3d, 3> trigon = {
            mesh.vbo_[face[0]],
            mesh.vbo_[face[1]],
            mesh.vbo_[face[2]] };
        double area = 0.5 * (trigon[1]- trigon[0]).cross(trigon[2] - trigon[1]).norm();
        if (600 < area)
            continue;
        double edge = max(max((trigon[1] - trigon[0]).norm(), (trigon[2] - trigon[0]).norm()), (trigon[2] - trigon[1]).norm());
        if (100 < edge)
            continue;
        if (600 > max(max(trigon[0][0], trigon[1][0]), trigon[2][0]))
            continue;
        const Vector3d edge0 = mesh.vbo_[face[1]] - mesh.vbo_[face[0]];
        const Vector3d edge1 = mesh.vbo_[face[2]] - mesh.vbo_[face[1]];
        double side0 = edge0.norm();
        double side1 = edge1.norm();
        double cos01 = edge0.dot(edge1) / (side0 * side1);
        if (fabs(fabs(cos01) - 1.0) < 5 / 180 * M_PI)
            continue;
        simmesh.ibo_.push_back(face);
    }
    ModelMesh::writeToFile({ simmesh });
    cout << "writeToFile finish" << endl;
}

static int _enrol = []()
    {
        readTerrainDataToMesh_csv1();
        //readTerrainDataToMesh_csv2();
        //readTerrainDataToMesh_csv3();
        return 0;
    }();
