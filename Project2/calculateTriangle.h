#pragma once
namespace psykronix
{
	// vector
	inline bool isNaN(const Eigen::Vector3d& vec)
	{
		return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
	}
	bool isParallel(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB); // dynamic accuracy
	bool isPerpendi(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB);

	// intersect of triangle
	DLLEXPORT bool isPointInTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon);
	DLLEXPORT bool isPointInTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isTwoSegmentsIntersect(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	DLLEXPORT bool isTwoSegmentsIntersect(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB);
	DLLEXPORT bool isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector2d, 2>& segment, const Eigen::AlignedBox2d& box);
	DLLEXPORT bool isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 2>& segment, const Eigen::AlignedBox3d& box);
	DLLEXPORT bool isSegmentCrossTriangle(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isSegmentCrossTriangleSurface(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isTwoTrianglesIntersectPIT(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
	DLLEXPORT bool isTwoTrianglesIntersectEIT(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
	//DLLEXPORT bool isTwoTrianglesIntersectionSAT(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
	// preprocess
	DLLEXPORT RelationOfTwoTriangles getRelationOfTwoTrianglesSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
	DLLEXPORT std::tuple<Eigen::Vector3d, double> getTriangleBoundingCircle(const std::array<Eigen::Vector3d, 3>& trigon);
	// distance
	//DLLEXPORT double getDistanceOfPointAndSegmentINF(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& segm);
	//DLLEXPORT double getDistanceOfTwoSegmentsINF(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB);
	//DLLEXPORT double getDistanceOfPointAndPlaneINF(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& plane);
	//other
	bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, double toleAng = 0, double toleDis = 0);
	std::tuple<bool, std::array<double, 4>> getTwoSegmentsCollinearCoincidentPoints(const Segment& segmA, const Segment& segmB, double toleAng = 0, double toleDis = 0);
	void mergeIntersectRegionOfSegment(std::vector<double>& _range, const std::array<double, 2>& prop);
}

//simplify global
bool isPointOnTriangleSurface(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
bool isSegmentAndTriangleIntersctSAT(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
bool isPointRayAcrossTriangleSAT(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
//triangle about
bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance = 0.0);
bool isTriangleAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box);
bool isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
double getTrianglesDistanceSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
std::array<Eigen::Vector3d, 2> getTwoTrianglesNearestPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
std::array<Eigen::Vector3d, 2> getTwoTrianglesIntersectPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);

