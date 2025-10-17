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

namespace eigen
{
	DLLEXPORT_CAL double getDistanceOfPointAndLine(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 2>& line); //get nearest point
	DLLEXPORT_CAL double getDistanceOfPointAndLine(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& line); //get nearest point
	DLLEXPORT_CAL double getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal);
	DLLEXPORT_CAL double getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& plane); //get nearest projection point
	//DLLEXPORT_CAL double getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const clash::Plane3d& plane); //overload

	DLLEXPORT_CAL double getDistanceOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB); //get two nearest points
	DLLEXPORT_CAL Eigen::Vector3d getNearestPointOfPointAndLine(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& line);
	DLLEXPORT_CAL Eigen::Vector3d getNearestPointOfPointAndPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal);
	
	DLLEXPORT_CAL clash::Segment3d getNearestPointOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB); //get nearest projection point
	DLLEXPORT_CAL Eigen::Vector3d getIntersectPointOfLineAndPlane(const std::array<Eigen::Vector3d, 2>& line, const std::array<Eigen::Vector3d, 2>& plane);
	DLLEXPORT_CAL clash::Segment3d getIntersectLineOfTwoPlanes(const Eigen::Vector3d& originA, const Eigen::Vector3d& normalA, const Eigen::Vector3d& originB, const Eigen::Vector3d& normalB);
	DLLEXPORT_CAL clash::Segment3d getIntersectLineOfTwoPlanes(const std::array<Eigen::Vector3d, 3>& planeA, const std::array<Eigen::Vector3d, 3>& planeB);
	
	DLLEXPORT_CAL Eigen::Vector2d getIntersectPointOfTwoLines(const std::array<Eigen::Vector2d, 2>& lineA, const std::array<Eigen::Vector2d, 2>& lineB);
	DLLEXPORT_CAL Eigen::Vector3d getIntersectPointOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB);
}

//segment
namespace clash
{
	using namespace Eigen;

	//inline in common use
	inline bool isPointOnLine(const Eigen::Vector3d& point, const Segment& line)
	{
		return eigen::isParallel3d(point - line[0], point - line[1]);
	}

	inline bool isPointOnPlane(const Eigen::Vector3d& point, const Plane3d& plane)
	{
		return eigen::isPerpendi3d(point - plane.m_origin, plane.m_normal);
	}

	inline bool isTwoSegmentsIntersect(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB, double tolerance = 0)
	{
		// segmA's two point on both sides of segmB
		if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) + tolerance ||
            std::max(segmB[0][0], segmB[1][0]) < std::min(segmA[0][0], segmA[1][0]) + tolerance ||
			std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) + tolerance ||
            std::max(segmB[0][1], segmB[1][1]) < std::min(segmA[0][1], segmA[1][1]) + tolerance)
			return false;
		return // double point on line judge
			((segmB[0] - segmA[0])[0] * (segmA[1] - segmA[0])[1] - (segmA[1] - segmA[0])[0] * (segmB[0] - segmA[0])[1]) *
			((segmA[1] - segmA[0])[0] * (segmB[1] - segmA[0])[1] - (segmB[1] - segmA[0])[0] * (segmA[1] - segmA[0])[1]) >= 0.0 &&
			((segmA[0] - segmB[0])[0] * (segmB[1] - segmB[0])[1] - (segmB[1] - segmB[0])[0] * (segmA[0] - segmB[0])[1]) *
			((segmB[1] - segmB[0])[0] * (segmA[1] - segmB[0])[1] - (segmA[1] - segmB[0])[0] * (segmB[1] - segmB[0])[1]) >= 0.0;
	}

    inline bool isTwoSegmentsIntersect(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, double tolerance = 0)
	{
		if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) + tolerance ||
			std::max(segmB[0][0], segmB[1][0]) < std::min(segmA[0][0], segmA[1][0]) + tolerance ||
			std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) + tolerance ||
			std::max(segmB[0][1], segmB[1][1]) < std::min(segmA[0][1], segmA[1][1]) + tolerance ||
			std::max(segmA[0][2], segmA[1][2]) < std::min(segmB[0][2], segmB[1][2]) + tolerance ||
			std::max(segmB[0][2], segmB[1][2]) < std::min(segmA[0][2], segmA[1][2]) + tolerance)
			return false;
		Eigen::Vector3d point = eigen::getIntersectPointOfTwoLines(segmA, segmB);
		if (std::isnan(point[0]))
			return false;
        if (0 < (point - segmA[0]).dot(point - segmA[1]) || 0 < (point - segmB[0]).dot(point - segmB[1]))
			return false;
		return true;
	}

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

