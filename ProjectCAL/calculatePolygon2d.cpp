#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;

//for curvearray classify
struct PathsPart
{
	int m_index = -2;//default init
	double m_ymin = DBL_MAX;
	//Eigen::Vector2d m_point;
	Eigen::AlignedBox2d m_box2d;
	std::vector<int> m_parity;
	bool operator<(const PathsPart& rhs) const
	{
		return m_ymin < rhs.m_ymin;
	}
};

static std::vector<PathsPart> _createPathsPartVct(const std::vector<std::vector<Eigen::Vector2d>>& profile)
{
	std::vector<PathsPart> pathsPartVct(profile.size());
	for (int i = 0; i < profile.size(); ++i)
	{
		PathsPart& pathsPart = pathsPartVct[i];
		pathsPart.m_index = i;
		//pathsPart.m_point = profile[i][0];// profile must no empty
		Eigen::AlignedBox2d box2d;
		for (int j = 0; j < profile[i].size(); ++j)//(const auto& iter : profile[i])
		{
			const Eigen::Vector2d& point = profile[i][j];
			box2d.extend(point);
			if (point[1] < pathsPart.m_ymin)
				pathsPart.m_ymin = point[1];
		}
		pathsPart.m_box2d = box2d;
	}
	std::sort(pathsPartVct.begin(), pathsPartVct.end());
	return pathsPartVct;
}

//using plane sweep algorithm, using to convertPathsEigenToCurveArray
std::vector<std::vector<int>> clash::getPathsClassify(const std::vector<std::vector<Eigen::Vector2d>>& profile)
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
			if (partJ.m_index == -1)
				continue;
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

static bool isPolygonSelfIntersect_doubleLoop(const std::vector<Eigen::Vector2d>& polygon)
{
	if (polygon.size() <= 3)
		return false;
	int n = (int)polygon.size(); //even if n==3
	for (int i = 0; i < n - 2; ++i) //skip adjacent
	{
		std::array<Eigen::Vector2d, 2> segmA = { polygon[i], polygon[i + 1] };
		for (int j = i + 2; j < n; ++j)
		{
			//include i(0,1)-j(n-1,0)
			std::array<Eigen::Vector2d, 2> segmB = { polygon[j], polygon[(j + 1) % n] };
			if (clash::isTwoSegmentsIntersect(segmA, segmB))
				return true;
		}
	}
	return false;
}

