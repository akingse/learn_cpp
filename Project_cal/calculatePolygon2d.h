#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple 2d polygon methods					   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_POLYGON2D_H
#define CALCULATE_POLYGON2D_H
namespace clash
{
	// pnpoly
	inline bool isPointInPolygon2D(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon)// pnpoly
	{
		Eigen::AlignedBox2d box;
		for (const auto& iter : polygon)
			box.extend(iter);
		if (!box.contains(point))
			return false;
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
	
	inline double calculatePolygonArea(const std::vector<Eigen::Vector2d>& polygon) //using Shoelace-Gauss method
	{
		double area = 0.0;
		size_t n = polygon.size();
		for (int i = 0; i < n; ++i)
		{
			int j = (i + 1) % n; //avoid over bound
			area += (polygon[i][0] + polygon[j][0]) * (polygon[i][1] - polygon[j][1]); //wond
			//area += polygon[i][0] * polygon[j][1] - polygon[i][1] * polygon[j][0]; //cross2d
		}
		return 0.5 * std::fabs(area);
	}

}
#endif// CALCULATE_POLYGON2D_H
