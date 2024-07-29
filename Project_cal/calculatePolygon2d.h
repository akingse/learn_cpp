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


	//for curvearray Classify
	struct PathsPart
	{
		int m_index;
		double m_ymin = DBL_MAX;
		//Eigen::Vector2d m_point;
		Eigen::AlignedBox2d m_box2d;
		std::vector<int> m_parity;
		bool operator<(const PathsPart& rhs) const
		{
			return m_ymin < rhs.m_ymin;
		}
	};

	inline std::vector<PathsPart> _createPathsPartVct(const std::vector<std::vector<Eigen::Vector2d>>& profile)
	{
		std::vector<PathsPart> pathsPartVct(profile.size());
		for (int i = 0; i < profile.size(); ++i)
		{
			PathsPart& pathsPart = pathsPartVct[i];
			pathsPart.m_index = i;
			//pathsPart.m_point = profile[i][0];// profile must no empty
			Eigen::AlignedBox2d box2d;
			for (const auto& iter : profile[i])
			{
				box2d.extend(iter);
				if (iter[1] < pathsPart.m_ymin)
					pathsPart.m_ymin = iter[1];
			}
			pathsPart.m_box2d = box2d;
		}
		std::sort(pathsPartVct.begin(), pathsPartVct.end());
		return pathsPartVct;
	}

	//using plane sweep algorithm, using to convertPathsEigenToCurveArray
	inline std::vector<std::vector<int>> getPathsClassify(const std::vector<std::vector<Eigen::Vector2d>>& profile)
	{
		std::vector<PathsPart> pathsPartVct = _createPathsPartVct(profile);
		//calculate profile whether inside
		for (int i = 0; i < pathsPartVct.size(); ++i)
		{
			PathsPart& partI = pathsPartVct[i];
			// ymin -> ymax
			if (partI.m_index == -1) //not support inner nest inner
				continue;
			double ymax = partI.m_box2d.max()[1];
			for (int j = i + 1; j < pathsPartVct.size(); ++j)
			{
				PathsPart& partJ = pathsPartVct[j];
				if (ymax < partJ.m_ymin)
					break;
				//boundbox separate -> must separate
                //boundbox intersect -> not always intersect
				if (partI.m_box2d.intersects(partJ.m_box2d) &&
					isPointInPolygon2D(profile[partJ.m_index][0], profile[partI.m_index]))
				{
					partI.m_parity.push_back(partJ.m_index);
					partJ.m_index = -1; //means inner
				}
			}
		}
		// process index according sign
		std::vector<std::vector<int>> unionVct;
		for (int i = 0; i < pathsPartVct.size(); ++i)
		{
			PathsPart& partI = pathsPartVct[i];
			if (partI.m_index == -1)
				continue;
			if (partI.m_parity.empty())
			{
				unionVct.push_back({ partI.m_index });
				continue;
			}
			std::vector<int> parity = { partI.m_index };
			parity.insert(parity.end(), partI.m_parity.begin(), partI.m_parity.end());
			unionVct.push_back(parity);
		}
		return unionVct;
	}


}
#endif// CALCULATE_POLYGON2D_H
