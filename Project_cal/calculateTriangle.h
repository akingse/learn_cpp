#pragma once
namespace psykronix
{
    // intersect of triangle
    DLLEXPORT bool isPointInTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon);
    DLLEXPORT bool isPointInTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
    DLLEXPORT bool isTwoSegmentsIntersect(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, bool preBox = false);
    DLLEXPORT bool isEdgeCrossTriangle(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
    DLLEXPORT bool isSegmentCrossTriangleSurface(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
    DLLEXPORT bool isTwoTrianglesIntersection(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
    DLLEXPORT bool isTwoTrianglesIntersection2(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
    DLLEXPORT bool TriangleIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
    DLLEXPORT bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
    // preprocess
    DLLEXPORT bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
    DLLEXPORT bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance);
    DLLEXPORT bool isTriangleAndBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box);
    DLLEXPORT bool isTriangleAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box);
    DLLEXPORT std::tuple<Eigen::Vector3d, double> getTriangleBoundingCircle(const std::array<Eigen::Vector3d, 3>& trigon);
    // soft-clash
    DLLEXPORT void getSegmentsPoints(Eigen::Vector3d& VEC, Eigen::Vector3d& X, Eigen::Vector3d& Y, const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& Q, const Eigen::Vector3d& B);
    DLLEXPORT double getTrianglesDistance(Eigen::Vector3d& P, Eigen::Vector3d& Q, const std::array<Eigen::Vector3d, 3>& S, const std::array<Eigen::Vector3d, 3>& T);

}


