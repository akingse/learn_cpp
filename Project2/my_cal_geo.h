#pragma once
namespace chatgpt
{
	// 生成Delaunay三角形Voronoi图
    struct Point 
    {
        double x, y;
        bool operator==(const Point& rhs) const
        {
            return memcmp(this, &rhs, 2 * sizeof(double)) == 0;
        }
    };

    struct Triangle 
    {
        Point p1, p2, p3;
        bool operator==(const Triangle& rhs) const
        {
            return memcmp(this, &rhs, 3 * sizeof(Point)) == 0;
        }
    };

    struct Edge 
    {
        Point start, end;
    };

    struct LineSegment
    {
        Point start, end;
    };

}
