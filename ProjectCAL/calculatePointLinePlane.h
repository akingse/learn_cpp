#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common point and line and plane calculation methods		   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_POINTLINEPLANE_H
#define CALCULATE_POINTLINEPLANE_H

namespace clash
{
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

namespace eigen
{
	DLLEXPORT_CAL double getDistanceOfPointAndLine(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& line); //get nearest point
	DLLEXPORT_CAL double getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& plane); //get nearest projection point
	DLLEXPORT_CAL double getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const clash::Plane3d& plane); //overload
	DLLEXPORT_CAL double getDistanceOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB); //get two nearest points
	DLLEXPORT_CAL Eigen::Vector3d getIntersectPointOfLineAndPlane(const std::array<Eigen::Vector3d, 2>& line, const std::array<Eigen::Vector3d, 2>& plane);
	DLLEXPORT_CAL Eigen::Vector2d getIntersectPointOfTwoLines(const std::array<Eigen::Vector2d, 2>& lineA, const std::array<Eigen::Vector2d, 2>& lineB);
	DLLEXPORT_CAL clash::Segment3d getIntersectLineOfTwoPlanes(const std::array<Eigen::Vector3d, 3>& planeA, const std::array<Eigen::Vector3d, 3>& planeB);
	DLLEXPORT_CAL clash::Segment3d getIntersectLineOfTwoPlanes(const clash::Plane3d& planeA, const clash::Plane3d& planeB); //overload
	DLLEXPORT_CAL clash::Segment3d getIntersectLineOfTwoPlanesP3D(const clash::Plane3d& planeA, const clash::Plane3d& planeB); 
}

//segment
namespace clash
{
	using namespace Eigen;

	//isTwoSegmentsIntersect2D
	inline bool isTwoSegmentsIntersect(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
	{
		// segmA's two point on both sides of segmB
		// (segmB[0] - segmA[0])x(segmA[1] - segmA[0])*(segmA[1] - segmA[0])x(segmB[1] - segmA[0])>=0
		// (segmA[0] - segmB[0])x(segmB[1] - segmB[0])*(segmB[1] - segmB[0])x(segmA[1] - segmB[0])>=0
		if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) ||
			std::min(segmA[0][0], segmA[1][0]) > std::max(segmB[0][0], segmB[1][0]) ||
			std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) ||
			std::min(segmA[0][1], segmA[1][1]) > std::max(segmB[0][1], segmB[1][1]))
			return false;
#ifdef USING_THRESHOLD_GEOMETRIC
		bool isBetweenB = ((segmB[0] - segmA[0]).x() * (segmA[1] - segmA[0]).y() - (segmA[1] - segmA[0]).x() * (segmB[0] - segmA[0]).y()) *
			((segmA[1] - segmA[0]).x() * (segmB[1] - segmA[0]).y() - (segmB[1] - segmA[0]).x() * (segmA[1] - segmA[0]).y()) > _epsF;
		bool isBetweenA = ((segmA[0] - segmB[0]).x() * (segmB[1] - segmB[0]).y() - (segmB[1] - segmB[0]).x() * (segmA[0] - segmB[0]).y()) *
			((segmB[1] - segmB[0]).x() * (segmA[1] - segmB[0]).y() - (segmA[1] - segmB[0]).x() * (segmB[1] - segmB[0]).y()) > _epsF;
		return isBetweenB && isBetweenA;
#else
		//Vector2d vecA = segmA[1] - segmA[0];
		//Vector2d AB_0 = segmB[0] - segmA[0];
		//Vector2d AB_1 = segmB[1] - segmA[0];
		//Vector2d vecB = segmB[1] - segmB[0];
		//Vector2d BA_0 = segmA[0] - segmB[0];
		//Vector2d BA_1 = segmA[1] - segmB[0];
		//return
		//	(AB_0[0] * vecA[1] - AB_0[1] * vecA[0]) * (AB_1[0] * vecA[1] - AB_1[1] * vecA[0]) <= 0.0 &&
		//	(BA_0[0] * vecB[1] - BA_0[1] * vecB[0]) * (BA_1[0] * vecB[1] - BA_1[1] * vecB[0]) <= 0.0;
		return // double point on line judge
			((segmB[0] - segmA[0])[0] * (segmA[1] - segmA[0])[1] - (segmA[1] - segmA[0])[0] * (segmB[0] - segmA[0])[1]) *
			((segmA[1] - segmA[0])[0] * (segmB[1] - segmA[0])[1] - (segmB[1] - segmA[0])[0] * (segmA[1] - segmA[0])[1]) >= 0.0 &&
			((segmA[0] - segmB[0])[0] * (segmB[1] - segmB[0])[1] - (segmB[1] - segmB[0])[0] * (segmA[0] - segmB[0])[1]) *
			((segmB[1] - segmB[0])[0] * (segmA[1] - segmB[0])[1] - (segmA[1] - segmB[0])[0] * (segmB[1] - segmB[0])[1]) >= 0.0;
#endif
	}

	// must coplanar
	inline bool isTwoSegmentsIntersect(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
	{
		// quick reject box
	  //if (std::max(segmA[0].x(), segmA[1].x()) < std::min(segmB[0].x(), segmB[1].x()) ||
		//	std::min(segmA[0].x(), segmA[1].x()) > std::max(segmB[0].x(), segmB[1].x()) ||
		//	std::max(segmA[0].y(), segmA[1].y()) < std::min(segmB[0].y(), segmB[1].y()) ||
		//	std::min(segmA[0].y(), segmA[1].y()) > std::max(segmB[0].y(), segmB[1].y()) ||
		//	std::max(segmA[0].z(), segmA[1].z()) < std::min(segmB[0].z(), segmB[1].z()) ||
		//	std::min(segmA[0].z(), segmA[1].z()) > std::max(segmB[0].z(), segmB[1].z()))
		if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) ||
			std::min(segmA[0][0], segmA[1][0]) > std::max(segmB[0][0], segmB[1][0]) ||
			std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) ||
			std::min(segmA[0][1], segmA[1][1]) > std::max(segmB[0][1], segmB[1][1]) ||
			std::max(segmA[0][2], segmA[1][2]) < std::min(segmB[0][2], segmB[1][2]) ||
			std::min(segmA[0][2], segmA[1][2]) > std::max(segmB[0][2], segmB[1][2])) //index will be fast
			return false;
		Eigen::Vector3d vecA = segmA[1] - segmA[0];
		Eigen::Vector3d vecB = segmB[1] - segmB[0];
		//judge coplanar first, without tolerance
		Eigen::Vector3d vecN = vecA.cross(vecB).normalized();
		//double dotPro = vecN.dot(segmB[0] - segmA[0]);
        if (!vecN.isZero() && 0 != std::fabs(vecN.dot(segmB[0] - segmA[0])))
			return false;
		return  //double straddling test
			0.0 <= (segmB[0] - segmA[0]).cross(vecA).dot(vecA.cross(segmB[1] - segmA[0])) && //double separate
			0.0 <= (segmA[0] - segmB[0]).cross(vecB).dot(vecB.cross(segmA[1] - segmB[0]));
	}

	inline Eigen::Vector2d getTwoSegmentsIntersectPoint(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
	{
		if (!isTwoSegmentsIntersect(segmA, segmB))
			return Vector2d(std::nan("0"), std::nan("0"));
		//get intersect point 2D
		const Eigen::Vector2d& A1 = segmA[0];
		const Eigen::Vector2d& A2 = segmA[1];
		const Eigen::Vector2d& B1 = segmB[0];
		const Eigen::Vector2d& B2 = segmB[1];
		double kA = -((B2.x() - B1.x()) * (A1.y() - B1.y()) - (B2.y() - B1.y()) * (A1.x() - B1.x())) /
			((B2.x() - B1.x()) * (A2.y() - A1.y()) - (B2.y() - B1.y()) * (A2.x() - A1.x())); //not parallel
		return A1 + kA * (A2 - A1);
		//double kB = -((A2.x() - A1.x()) * (B1.y() - A1.y()) - (A2.y() - A1.y()) * (B1.x() - A1.x())) /
		//	((A2.x() - A1.x()) * (B2.y() - B1.y()) - (A2.y() - A1.y()) * (B2.x() - B1.x())); //not parallel
		//return B1 + kB * (B2 - B1);
	}

	inline Eigen::Vector3d getTwoSegmentsIntersectPoint(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
	{
		if (!isTwoSegmentsIntersect(segmA, segmB))
			return gVecNaN;
		Vector3d vecSeg = segmA[1] - segmA[0];
		Vector3d normal = (vecSeg).cross(segmB[1] - segmB[0]);
		if (normal.isZero(epsF)) // intersect cause collinear
			return segmA[0];
		//double k = (segmA[0] - segmB[0]).cross(segmA[0] - segmB[1]).norm() / normal.norm(); //triangle area method
		double k = (segmB[0] - segmA[0]).cross(vecSeg).dot(normal) / normal.squaredNorm(); //k
		return segmA[0] + k * vecSeg;
	}

	// for soft clash
	//inline Vector3d getIntersectOfPointAndLine(const Vector3d& point, const std::array<Vector3d, 2>& segm)
	//{
	//	Eigen::Vector3d direction = (segm[1] - segm[0]);// not zero
	//	double projection = direction.dot(point);
	//	//the projection must on segment
	//	if (direction.dot(segm[1]) < projection || projection < direction.dot(segm[0]))
	//		return gVecNaN;// DBL_MAX;
	//	double k = direction.dot(point - segm[0]) / direction.dot(direction);
	//	return (segm[0] - point + k * direction);//.squaredNorm();
	//}
	//inline Vector3d getIntersectOfTwoSegments(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
	//{
	//	Vector3d vectA = segmA[1] - segmA[0];
	//	Vector3d vectB = segmB[1] - segmB[0];
	//	double delta1 = (segmB[0] - segmA[0]).dot(vectA);
	//	double delta2 = (segmB[0] - segmA[0]).dot(vectB);
	//	// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
	//	double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
	//	if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
	//		return gVecNaN;// DBL_MAX;
	//	double kA = 1 / deno * (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2);
	//	double kB = 1 / deno * (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2);
	//	//	Vector3d pointA = segmA[0] + kA * vectA;
	//	//	Vector3d pointB = segmB[0] + kB * vectB;
	//	//whether two intersect-point inside segments
	//	if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
	//		return (segmA[0] + kA * vectA - segmB[0] - kB * vectB);//.squaredNorm();
	//	return gVecNaN;// DBL_MAX; // nearest point outof segments
	//}
	//inline Eigen::Vector3d getIntersectPointOfLineAndPlane(const PosVec3d& ray, const Plane3d& plane)
	//{
	//	double k = (plane.m_origin - ray[0]).dot(plane.m_normal) / (ray[1].dot(plane.m_normal));
	//	return ray[0] + k * ray[1];
	//}

	inline bool isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector2d, 2>& segment, const Eigen::AlignedBox2d& box)
	{
		if (box.contains(segment[0]) || box.contains(segment[1]))
			return true;
		const Vector2d& _min = box.min();
		const Vector2d& _max = box.max();
		if (std::max(segment[0].x(), segment[1].x()) < _min.x() ||
			std::min(segment[0].x(), segment[1].x()) > _max.x() ||
			std::max(segment[0].y(), segment[1].y()) < _min.y() ||
			std::min(segment[0].y(), segment[1].y()) > _max.y())
			return false;
		std::array<Eigen::Vector2d, 3> axes = { {
			Vector2d(1,0),
			Vector2d(0,1),
			Vector2d(-(segment[1] - segment[0]).y(),(segment[1] - segment[0]).x())} }; //canbe zero
		std::array<Eigen::Vector2d, 4> boxVtx = { {
			box.min(),
			Vector2d(box.max().x(),box.min().y()),
			box.max(),
			Vector2d(box.min().x(),box.max().y())} };
		double minA, maxA, minB, maxB, projection;
		for (const auto& axis : axes) //fast than index
		{
			if (axis.isZero())
				continue;
			minA = DBL_MAX;
			maxA = -DBL_MAX;
			minB = DBL_MAX;
			maxB = -DBL_MAX;
			for (const auto& vertex : boxVtx) //fast than list
			{
				projection = axis.dot(vertex);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			for (const auto& vertex : segment)
			{
				projection = axis.dot(vertex);
				minB = std::min(minB, projection);
				maxB = std::max(maxB, projection);
			}
			if (maxA < minB || maxB < minA) // absolute zero
				return false;
		}
		return true;
	}

	inline bool isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 2>& segment, const Eigen::AlignedBox3d& box)
	{
		//whlie segment with tolerance, using large box
		if (box.contains(segment[0]) || box.contains(segment[1]))
			return true;
		const Vector3d& _min = box.min();
		const Vector3d& _max = box.max();
		if (std::max(segment[0].x(), segment[1].x()) < _min.x() ||
			std::min(segment[0].x(), segment[1].x()) > _max.x() ||
			std::max(segment[0].y(), segment[1].y()) < _min.y() ||
			std::min(segment[0].y(), segment[1].y()) > _max.y() ||
			std::max(segment[0].z(), segment[1].z()) < _min.z() ||
			std::min(segment[0].z(), segment[1].z()) > _max.z())
			return false;
		Vector3d vecSeg = segment[1] - segment[0];//segment direction
		std::array<Eigen::Vector3d, 7> axes = { {
			vecSeg, //maybe not work
			Vector3d(1,0,0),
			Vector3d(0,1,0),
			Vector3d(0,0,1),
			vecSeg.cross(Vector3d(1,0,0)),
			vecSeg.cross(Vector3d(0,1,0)),
			vecSeg.cross(Vector3d(0,0,1)) } };
		Vector3d vecBox = box.max() - box.min();
		std::array<Eigen::Vector3d, 8> boxVtx = { { //box.min() is origin
			Vector3d(0,0,0),
			Vector3d(vecBox[0],0,0),
			Vector3d(vecBox[0],vecBox[1],0),
			Vector3d(0,vecBox[1],0),
			Vector3d(0,0,vecBox[2]),
			Vector3d(vecBox[0],0,vecBox[2]),
			vecBox, //Vector3d(vecBox[0],vecBox[1],vecBox[2])
			Vector3d(0,vecBox[1],vecBox[2]) } };
		double minA, maxA, minB, maxB, projection;
		for (const auto& axis : axes) //fast than index
		{
			if (axis.isZero())
				continue;
			minA = DBL_MAX;
			maxA = -DBL_MAX;
			minB = DBL_MAX;
			maxB = -DBL_MAX;
			for (const auto& vertex : boxVtx) //fast than list
			{
				projection = axis.dot(vertex);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			for (const auto& vertex : segment) //only two point
			{
				projection = axis.dot(vertex - box.min());
				minB = std::min(minB, projection);
				maxB = std::max(maxB, projection);
			}
			if (maxA < minB || maxB < minA) // absolute zero
				return false;
		}
		return true;
	}
}

//merge
namespace clash
{
	// distance
	//DLLEXPORT double getDistanceOfPointAndSegmentINF(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& segm);
	//DLLEXPORT double getDistanceOfTwoSegmentsINF(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB);
	//DLLEXPORT double getDistanceOfPointAndPlaneINF(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& plane);
	// for profile fusion
	DLLEXPORT_CAL bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	DLLEXPORT_CAL bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, double toleAng = 0, double toleDis = 0);
	DLLEXPORT_CAL std::tuple<bool, std::array<double, 4>> getTwoSegmentsCollinearCoincidentPoints(const Segment& segmA, const Segment& segmB, double toleAng = 0, double toleDis = 0);
	DLLEXPORT_CAL void mergeIntersectIntervalOfSegment(std::vector<double>& _range, const std::array<double, 2>& prop);

}
#endif// CALCULATE_POINTLINEPLANE_H

