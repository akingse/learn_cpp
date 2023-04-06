//https://blog.csdn.net/NUX_A/article/details/126133245
#include "pch.h"

//几何图元，如点、线等都定义在内核`Kernel`中
#include <iostream>
#include <CGAL/Simple_cartesian.h>

/*
- `Point_2()`:一个2d的点
- `Segment_2`:一个2d的扇形，若只有两点，则为直线
- `CGAL::squared_distance(p, q)`：两点距离or点到直线的距离
- `CGAL::orientation(p, q, m)`:三点是否共线，若不共线，则m在p，q哪一侧
- `CGAL::midpoint(p, q)`:中点坐标
*/
int test_1()
{
    typedef CGAL::Simple_cartesian<double> Kernel;
    typedef Kernel::Point_2 Point_2;
    typedef Kernel::Segment_2 Segment_2;

    Point_2 p(1, 1), q(10, 10);
    std::cout << "p = " << p << std::endl;// p = 1 1
    std::cout << "q = " << q.x() << " " << q.y() << std::endl;// q = 10 10
    std::cout << "sqdist(p,q) = "
        << CGAL::squared_distance(p, q) << std::endl;// sqdist(p,q) = 162
    Segment_2 s(p, q);
    Point_2 m(5, 9);
    std::cout << "m = " << m << std::endl;// m = 5 9
    std::cout << "sqdist(Segment_2(p,q), m) = "
        << CGAL::squared_distance(s, m) << std::endl;// sqdist(Segment_2(p,q), m) = 8
    std::cout << "p, q, and m ";
    switch (CGAL::orientation(p, q, m)) {
    case CGAL::COLLINEAR:
        std::cout << "are collinear\n";
        break;
    case CGAL::LEFT_TURN:
        std::cout << "make a left turn\n";
        break;
    case CGAL::RIGHT_TURN:
        std::cout << "make a right turn\n";
        break;
    } // p, q, and m make a left turn
    std::cout << " midpoint(p,q) = " << CGAL::midpoint(p, q) << std::endl;// midpoint(p,q) = 5.5 5.5
    return 0;
}

#include <iostream>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>


//凸包：在高维空间中有一群散布各处的点，凸包是包覆这群点的所有外壳当中，表面积or容积最小的一个外壳，而最小的外壳一定是凸的。
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <vector>

int test_2()
{
    typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
    typedef Kernel::Point_2 Point_2;
    typedef std::vector<Point_2> Points;

    Points points, result;
    points.push_back(Point_2(0, 0));
    points.push_back(Point_2(10, 0));
    points.push_back(Point_2(10, 10));
    points.push_back(Point_2(5, 6));
    points.push_back(Point_2(4, 1));
    CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(result));
    std::cout << result.size() << " points on the convex hull" << std::endl;
    for (int i = 0; i < result.size(); i++) {
        std::cout << result[i] << std::endl;
    }
    return 0;
}
/*
4 points on the convex hull
0 0
10 0
10 10
5 6
*/
//delaunay三角剖分,一个小demo
#include<vector>
#include<algorithm>
#include<CGAL/point_generators_2.h>
#include<CGAL/algorithm.h>
#include<CGAL/random_selection.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include<CGAL/Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>

using namespace CGAL;



int test_3() 
{
    typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
    typedef Kernel::Point_2 Point;
    typedef Creator_uniform_2<double, Point> Creator;
    typedef std::vector<Point> Vector;
    typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay;
    typedef Delaunay::Vertex_handle Vertex_handle;
    Vector points;
    points.reserve(10);

    Random_points_in_disc_2<Point, Creator> g(150.0);
    CGAL::cpp11::copy_n(g, 10, std::back_inserter(points));

    Delaunay dt;

    dt.insert(points.begin(), points.end());
    CGAL::draw(dt);
    system("pause");
    return EXIT_SUCCESS;
}
