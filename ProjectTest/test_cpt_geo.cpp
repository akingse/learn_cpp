#include "pch.h"
#include "test_cpt_geo.h"
using namespace std;
using namespace chatgpt;

std::vector<Point> generateRandomPoints(int numPoints) {
    std::vector<Point> points;
    // 在这里生成随机点的代码
    return points;
}

double distance(const Point& p1, const Point& p2) {
    // 计算两点之间的距离
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

bool isPointInCircle(const Point& center, double radius, const Point& point) {
    // 检查点是否在圆内
    double dx = center.x - point.x;
    double dy = center.y - point.y;
    double distanceSquared = dx * dx + dy * dy;
    return distanceSquared <= radius * radius;
}

void calculateCircumcircle(const Triangle& triangle, Point& center, double& radius) {
    double ax = triangle.p1.x;
    double ay = triangle.p1.y;
    double bx = triangle.p2.x;
    double by = triangle.p2.y;
    double cx = triangle.p3.x;
    double cy = triangle.p3.y;

    double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
    double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;

    center.x = ux;
    center.y = uy;
    radius = distance(center, triangle.p1);
}

void removeDuplicateEdges(std::vector<Edge>& edges) {
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        if (a.start.x != b.start.x) {
            return a.start.x < b.start.x;
        }
        else {
            return a.start.y < b.start.y;
        }
        });

    edges.erase(std::unique(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return (a.start.x == b.start.x && a.start.y == b.start.y && a.end.x == b.end.x && a.end.y == b.end.y) ||
            (a.start.x == b.end.x && a.start.y == b.end.y && a.end.x == b.start.x && a.end.y == b.start.y);
        }), edges.end());
}

bool isTriangleIntersectingSuperTriangle(const Triangle& triangle, const Triangle& superTriangle) {
    std::vector<Point> trianglePoints = { triangle.p1, triangle.p2, triangle.p3 };
    std::vector<Point> superTrianglePoints = { superTriangle.p1, superTriangle.p2, superTriangle.p3 };

    for (const auto& point : trianglePoints) {
        if (std::find(superTrianglePoints.begin(), superTrianglePoints.end(), point) != superTrianglePoints.end()) {
            return true;
        }
    }

    return false;
}


std::vector<Triangle> generateDelaunayTriangulation(const std::vector<Point>& points) {
    std::vector<Triangle> triangles;

    // 递归终止条件
    if (points.size() == 3) {
        Triangle triangle;
        triangle.p1 = points[0];
        triangle.p2 = points[1];
        triangle.p3 = points[2];
        triangles.push_back(triangle);
        return triangles;
    }

    // 找到点集的最小外接矩形
    double minX = points[0].x, minY = points[0].y;
    double maxX = minX, maxY = minY;
    for (const auto& point : points) {
        minX = std::min(minX, point.x);
        minY = std::min(minY, point.y);
        maxX = std::max(maxX, point.x);
        maxY = std::max(maxY, point.y);
    }

    // 计算外接矩形的中心点
    double centerX = (minX + maxX) / 2;
    double centerY = (minY + maxY) / 2;

    // 构造外接矩形的三角形
    Triangle superTriangle;
    double deltaX = maxX - minX;
    double deltaY = maxY - minY;
    double deltaMax = std::max(deltaX, deltaY);
    superTriangle.p1 = { centerX - 20 * deltaMax, centerY - deltaMax };
    superTriangle.p2 = { centerX, centerY + 20 * deltaMax };
    superTriangle.p3 = { centerX + 20 * deltaMax, centerY - deltaMax };
    triangles.push_back(superTriangle);

    // 逐点插入
    for (const auto& point : points) {
        std::vector<Edge> polygonEdges;

        // 查找点所在的三角形
        for (auto it = triangles.begin(); it != triangles.end(); ) {
            const Triangle& triangle = *it;

            // 检查点是否在三角形的外接圆内
            Point circleCenter;
            double circleRadius;
            calculateCircumcircle(triangle, circleCenter, circleRadius);
            if (isPointInCircle(circleCenter, circleRadius, point)) {
                // 保存三角形的边
                polygonEdges.push_back({ triangle.p1, triangle.p2 });
                polygonEdges.push_back({ triangle.p2, triangle.p3 });
                polygonEdges.push_back({ triangle.p3, triangle.p1 });

                // 删除该三角形
                it = triangles.erase(it);
            }
            else {
                ++it;
            }
        }

        // 去除重复的边
        removeDuplicateEdges(polygonEdges);

        // 重新构建三角形
        for (const auto& edge : polygonEdges) {
            Triangle triangle;
            triangle.p1 = edge.start;
            triangle.p2 = edge.end;
            triangle.p3 = point;
            triangles.push_back(triangle);
        }
    }

    // 删除与外接矩形相交的三角形
    for (auto it = triangles.begin(); it != triangles.end(); ) {
        const Triangle& triangle = *it;
        if (isTriangleIntersectingSuperTriangle(triangle, superTriangle)) {
            it = triangles.erase(it);
        }
        else {
            ++it;
        }
    }

    // 删除外接矩形的三角形
    triangles.erase(std::find(triangles.begin(), triangles.end(), superTriangle));

    return triangles;
}

void calculatePerpendicularBisector(const Point& p1, const Point& p2, LineSegment& bisector) {
    double midX = (p1.x + p2.x) / 2;
    double midY = (p1.y + p2.y) / 2;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    bisector.start.x = midX - dy;
    bisector.start.y = midY + dx;
    bisector.end.x = midX + dy;
    bisector.end.y = midY - dx;
}

bool calculateLineIntersection(const LineSegment& line1, const LineSegment& line2, Point& intersection) {
    double x1 = line1.start.x;
    double y1 = line1.start.y;
    double x2 = line1.end.x;
    double y2 = line1.end.y;
    double x3 = line2.start.x;
    double y3 = line2.start.y;
    double x4 = line2.end.x;
    double y4 = line2.end.y;

    double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    if (denominator == 0) {
        // 两线段平行或共线，没有交点
        return false;
    }

    double intersectionX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
	double intersectionY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y2 - y2) * (x3 * y4 - y3 * x4)) / denominator;

    intersection.x = intersectionX;
    intersection.y = intersectionY;

    return true;
}

std::vector<LineSegment> generateVoronoiDiagram(const std::vector<Point>& points) {
    // 生成Delaunay三角形
    std::vector<Triangle> triangles = generateDelaunayTriangulation(points);

    // 计算Voronoi边
    std::vector<LineSegment> voronoiEdges;

    for (const auto& triangle : triangles) {
        // 计算三角形的外接圆心
        Point circleCenter;
        double circleRadius;
        calculateCircumcircle(triangle, circleCenter, circleRadius);

        // 计算三角形的外接圆心到三个顶点的垂直平分线
        LineSegment edge1, edge2, edge3;
        calculatePerpendicularBisector(triangle.p1, triangle.p2, edge1);
        calculatePerpendicularBisector(triangle.p2, triangle.p3, edge2);
        calculatePerpendicularBisector(triangle.p3, triangle.p1, edge3);

        // 计算Voronoi边的交点
        Point intersection1, intersection2;
        bool hasIntersection1 = calculateLineIntersection(edge1, edge2, intersection1);
        bool hasIntersection2 = calculateLineIntersection(edge2, edge3, intersection2);

        // 添加Voronoi边
        if (hasIntersection1) {
            voronoiEdges.push_back({ intersection1, circleCenter });
        }
        if (hasIntersection2) {
            voronoiEdges.push_back({ intersection2, circleCenter });
        }
    }

    return voronoiEdges;
}