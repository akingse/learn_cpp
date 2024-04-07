#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple triangle calculation methods			   *
* License   :  MIT									                           *
*******************************************************************************/

namespace clash
{
	// vector
	//inline bool isNaN(const Eigen::Vector3d& vec)
	//{
	//	return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
	//}
	double computeTriangleArea(const std::array<Eigen::Vector2d, 3>& triangle);
	double computeTriangleArea(const std::array<Eigen::Vector3d, 3>& triangle, bool is2D = true);
	DLLEXPORT_CAL double calculatePolygonArea(const std::vector<Eigen::Vector2d>& polygon); //using Shoelace(Gauss) method

	// intersect of triangle
	DLLEXPORT_CAL bool isPointInTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon);
	DLLEXPORT_CAL bool isPointInTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT_CAL bool isTwoSegmentsIntersect(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	DLLEXPORT_CAL bool isTwoSegmentsIntersect(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB);
	DLLEXPORT_CAL bool isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector2d, 2>& segment, const Eigen::AlignedBox2d& box);
	DLLEXPORT_CAL bool isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 2>& segment, const Eigen::AlignedBox3d& box);
	DLLEXPORT_CAL Eigen::Vector2d getTwoSegmentsIntersectPoint(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	DLLEXPORT_CAL Eigen::Vector3d getTwoSegmentsIntersectPoint(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB);
	bool isSegmentCrossTriangle(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
	bool isSegmentCrossTriangleSurface(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
	bool isTwoTrianglesIntersectPIT(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
	bool isTwoTrianglesIntersectEIT(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
	
	// preprocess
	RelationOfTwoTriangles getRelationOfTwoTrianglesSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
	std::tuple<Eigen::Vector3d, double> getTriangleBoundingCircle(const std::array<Eigen::Vector3d, 3>& trigon);
	// distance
	//DLLEXPORT double getDistanceOfPointAndSegmentINF(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& segm);
	//DLLEXPORT double getDistanceOfTwoSegmentsINF(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB);
	//DLLEXPORT double getDistanceOfPointAndPlaneINF(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& plane);
	
	// for profile fusion
	DLLEXPORT_CAL bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	DLLEXPORT_CAL bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, double toleAng = 0, double toleDis = 0);
	DLLEXPORT_CAL std::tuple<bool, std::array<double, 4>> getTwoSegmentsCollinearCoincidentPoints(const Segment& segmA, const Segment& segmB, double toleAng = 0, double toleDis = 0);
	DLLEXPORT_CAL void mergeIntersectIntervalOfSegment(std::vector<double>& _range, const std::array<double, 2>& prop);

	//inline in common use
	inline bool isPointOnLine(const Eigen::Vector3d& point, const Segment& line)
	{
		return eigen::isParallel3d(point - line[0], point - line[1]);
	}
	inline bool isPointOnPlane(const Eigen::Vector3d& point, const Plane3d& plane)
	{
		return eigen::isPerpendi3d(point - plane.m_origin, plane.m_normal);
	}
}

//simplify global
DLLEXPORT_CAL bool isPointOnTriangleSurface(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
DLLEXPORT_CAL bool isSegmentAndTriangleIntersctSAT(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
DLLEXPORT_CAL bool isPointRayAcrossTriangleSAT(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
DLLEXPORT_CAL int isRayLineCrossTriangleMTA(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const clash::Triangle& trigon);
//triangle about
DLLEXPORT_CAL bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance = 0.0);
DLLEXPORT_CAL bool isTriangleAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box);
DLLEXPORT_CAL bool isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
DLLEXPORT_CAL bool isTwoTrianglesPenetrationSAT(const std::array<Eigen::Vector2d, 3>& triA, const std::array<Eigen::Vector2d, 3>& triB);
DLLEXPORT_CAL bool isTwoTrianglesPenetrationSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
DLLEXPORT_CAL double getTrianglesDistanceSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
DLLEXPORT_CAL std::array<Eigen::Vector3d, 2> getTwoTrianglesNearestPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
DLLEXPORT_CAL std::array<Eigen::Vector3d, 2> getTwoTrianglesIntersectPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
DLLEXPORT_CAL double getDistanceOfPointAndTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon);
DLLEXPORT_CAL double getDistanceOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);

// pnpoly
DLLEXPORT_CAL bool isPointInPolygon2D(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);
DLLEXPORT_CAL bool isPointInPolygon2D(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& polygon);
//for convex polygon
DLLEXPORT_CAL bool isTwoPolygonsIntersectSAT(const std::vector<Eigen::Vector2d>& polygonA, const std::vector<Eigen::Vector2d>& polygonB);
