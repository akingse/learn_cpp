#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple 2d polygon calculation methods		   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_POLYGON2D_H
#define CALCULATE_POLYGON2D_H

namespace clash
{
	// pnpoly
    inline bool isPointInPolygon2D(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon, 
        double tolerance = 0, bool usingBox = true)// pnpoly
	{
		auto _getDistanceOfPointAndSegmentINF = [](const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 2>& segm)->double
			{
				Eigen::Vector2d vectseg = segm[1] - segm[0];// canbe zero, next willbe zero
				double projection = vectseg.dot(point);
				//the projection must on segment
				if (vectseg.dot(segm[1]) <= projection || projection <= vectseg.dot(segm[0]))
					return DBL_MAX;
				double k = vectseg.dot(point - segm[0]) / vectseg.dot(vectseg);
				return (segm[0] - point + k * vectseg).norm();
			};
		if (tolerance != 0)
		{
			int nvert = (int)polygon.size();
			for (int i = 0; i < (int)polygon.size(); ++i)
			{
                if ((point - polygon[i]).norm() <= tolerance)
					return true;
				std::array<Eigen::Vector2d, 2> segment = { polygon[i], polygon[(i + 1) % nvert] };
                if (_getDistanceOfPointAndSegmentINF(point, segment) < tolerance)
					return true;
			}
		}
		if (usingBox)
		{
			Eigen::AlignedBox2d box;
			for (int i = 0; i < (int)polygon.size(); ++i)//(const auto& iter : polygon)
				box.extend(polygon[i]);
			if (!box.contains(point))
				return false;
		}
		//origin algorithm
		bool isIn = false;
		int nvert = (int)polygon.size(); // polygon need not close
		int i, j;
		for (i = 0, j = nvert - 1; i < nvert; j = i++)
		{
			if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) &&
				(point[0] < (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]))
				isIn = !isIn;
		}
		return isIn;
	}

	inline bool isPointInPolygon2D(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& polygon)// pnpoly
	{
		// point on polygon means false
		bool isIn = false;
		int nvert = (int)polygon.size();
		int i, j;
		for (i = 0, j = nvert - 1; i < nvert; j = i++)
		{
			if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) &&
				(point[0] < (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]))
				isIn = !isIn;
		}
		return isIn;
	}

	inline bool isTwoPolygonsIntersectSAT(const std::vector<Eigen::Vector2d>& polygonA, const std::vector<Eigen::Vector2d>& polygonB)
	{
		// has been boundbox pre-judge
		std::vector<Eigen::Vector2d> axes;
		size_t i;
		Eigen::Vector2d edge;
		for (i = 0; i < polygonA.size() - 1; i++)
		{
			edge = polygonA[i + 1] - polygonA[i];
			if (!edge.isZero())
				axes.push_back(Eigen::Vector2d(-edge[1], edge[0]));
		}
		edge = polygonA[i] - polygonA[0];
		if (!edge.isZero())
			axes.push_back(Eigen::Vector2d(-edge[1], edge[0]));
		for (i = 0; i < polygonB.size() - 1; i++)
		{
			edge = polygonB[i + 1] - polygonB[i];
			if (!edge.isZero())
				axes.push_back(Eigen::Vector2d(-edge[1], edge[0]));
		}
		edge = polygonB[i] - polygonB[0];
		if (!edge.isZero())
			axes.push_back(Eigen::Vector2d(-edge[1], edge[0]));
		//do judge
		double minA, maxA, minB, maxB, projection;
		for (const auto& axis : axes)
		{
			if (axis.isZero())
				continue;
			minA = DBL_MAX;
			maxA = -DBL_MAX;
			minB = DBL_MAX;
			maxB = -DBL_MAX;
			for (const auto& vertex : polygonA)
			{
				projection = axis.dot(vertex);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			for (const auto& vertex : polygonB)
			{
				projection = axis.dot(vertex);
				minB = std::min(minB, projection);
				maxB = std::max(maxB, projection);
			}
			if (maxA < minB + epsF || maxB < minA + epsF)
				return false;
		}
		return true;
	}
	
	inline bool isPolygonSelfIntersect(const std::vector<Eigen::Vector2d>& polygon)
	{
		if (polygon.size() <= 3)
			return false;
		std::vector<Segment2d> sortEdges;
		size_t n = polygon.size();
		for (int i = 0; i < n; ++i)
		{
			int j = (i + 1) % n;
			if (polygon[i][0] < polygon[j][0])
				sortEdges.push_back({ polygon[i],polygon[j] });
			else
				sortEdges.push_back({ polygon[j],polygon[i] });
		}
		//sorted by start coord-x
		std::sort(sortEdges.begin(), sortEdges.end(),
			[&](const Segment2d& segA, const Segment2d& segB) {return segA[0][0] < segB[0][0]; });
		for (int i = 0; i < sortEdges.size() - 1; ++i) //whether judge last 
		{
			double edgeXmax = sortEdges[i][1][0];
			for (int j = i + 1; j < sortEdges.size(); ++j)
			{
				if (edgeXmax < sortEdges[j][0][0])
					continue;
				//preInter
				if (std::max(sortEdges[i][0][1], sortEdges[i][0][1]) < std::min(sortEdges[j][0][1], sortEdges[j][0][1]) ||
					std::max(sortEdges[j][0][1], sortEdges[j][0][1]) < std::min(sortEdges[i][0][1], sortEdges[i][0][1]))
					continue;
				if (isTwoSegmentsIntersect(sortEdges[i], sortEdges[i]))
					return true;
			}
		}
		return false;
	}

	//equal area < 0
	inline double isContourCCW(const std::vector<Eigen::Vector2d>& contour)
	{
		const size_t n = contour.size();
		if (n < 3)
			return false;
		double area = 0.0;
		for (size_t i = 0; i < n; ++i)
		{
			const size_t j = (i + 1) % n;
			const Eigen::Vector2d& p1 = contour[i];
			const Eigen::Vector2d& p2 = contour[j];
			area += (p2.x() - p1.x()) * (p2.y() + p1.y());
		}
		return area; // area < 0;
	}

	//area is fabs
	inline double calculatePolygonArea(const std::vector<Eigen::Vector2d>& polygon) //using Shoelace-Gauss method
	{
		double area = 0.0;
		int n = (int)polygon.size();
		if (n < 3)
			return 0;
		for (int i = 0; i < n; ++i)
		{
			int j = (i + 1) % n; //avoid index over bound
			area += (polygon[i][0] + polygon[j][0]) * (polygon[i][1] - polygon[j][1]);
			//area += polygon[i][0] * polygon[j][1] - polygon[i][1] * polygon[j][0]; //cross2d
		}
		return 0.5 * std::fabs(area);
	}

	inline Eigen::Vector2d computeCentroid(const std::array<Eigen::Vector2d, 3>& trigon)
	{
		Eigen::Vector2d centroid = (trigon[0] + trigon[1] + trigon[2]) / 3.0;
		return centroid;
	}
	inline Eigen::Vector3d computeCentroid(const std::array<Eigen::Vector3d, 3>& trigon)
	{
		Eigen::Vector3d centroid = (trigon[0] + trigon[1] + trigon[2]) / 3.0;
		return centroid;
	}

	inline Eigen::Vector2d computeCentroid(const std::vector<Eigen::Vector2d>& polygon)
	{
		double area = 0.0;
		Eigen::Vector2d centroid(0.0, 0.0);
		int n = (int)polygon.size();
		if (n < 3) 
		{
			throw std::invalid_argument("Polygon must have at least 3 vertices.");
			return Eigen::Vector2d(std::nan("0"), 0);
		}
		for (int i = 0; i < n; ++i) 
		{
			int j = (i + 1) % n; // Next vertex index
			double crossProduct = polygon[i].x() * polygon[j].y() - polygon[j].x() * polygon[i].y();
			area += crossProduct;
			centroid += (polygon[i] + polygon[j]) * crossProduct; // Accumulate weighted centroid
		}
		area *= 0.5;
		centroid /= (6.0 * area); // Final centroid calculation
		return centroid;
	}

	//for FillProfile
	DLLEXPORT_CAL std::vector<std::vector<int>> getPathsClassify(const std::vector<std::vector<Eigen::Vector2d>>& profile);

}
#endif// CALCULATE_POLYGON2D_H
