#include "pch.h"
#include "calculateDataTree.h"

using namespace std;
using namespace psykronix;
using namespace Eigen;
//static constexpr double eps = FLT_EPSILON; //1e-7

size_t Polygon2d::m_id = 0;
#ifdef min
#undef min
#endif // 
#ifdef max
#undef max
#endif // 


bool psykronix::isTwoSegmentsCollinearCoincident(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
{
	//double operator^(const Vector2d& vec1, const Vector2d& vec2)
	auto _cross2d = [](const Vector2d& vec1, const Vector2d& vec2)->double
	{
		//return vec1.x() * vec2.y() - vec2.x() * vec1.y();
		return vec1[0] * vec2[1] - vec2[0] * vec1[1];
	};
	if (!isTwoSegmentsIntersect(segmA, segmB))
		return false;
	//return (segmA[1] - segmA[0]).cross(segmB[1] - segmB[0]).isZero(eps);
	return fabs(_cross2d(segmA[1] - segmA[0], segmB[1] - segmB[0])) < eps;
}

//bool psykronix::isTwoSegmentsCollinearCoincident(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
//{
//	if (!isTwoSegmentsIntersect(segmA, segmB))
//		return false;
//	return (segmA[1] - segmA[0]).cross(segmB[1] - segmB[0]).isZero(eps);
//}
//double _getDistanceOfPointAndSegmentINF(const Vector3d& point, const std::array<Vector3d, 2>& segm)
//{
//	Vector3d vecSeg = segm[1] - segm[0];// not zero
//	double projection = vecSeg.dot(point);
//	//the projection must on segment
//	if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
//		return DBL_MAX;
//	double k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
//	return (segm[0] - point + k * vecSeg).squaredNorm();
//};

#define USING_NORMALIZED_VECTOR
bool psykronix::isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, 
	double toleAng /*= 0*/, double toleDis /*= 0*/)
{
	// |A¡ÁB| <= |A||B|sin¦È, near zero sin¦È approx ¦È
#ifndef USING_NORMALIZED_VECTOR
	Vector3d segmVecA = (segmA[1] - segmA[0]).normalized();
	Vector3d segmVecB = (segmB[1] - segmB[0]).normalized();
	if (!segmVecA.cross(segmVecB).isZero(toleAng)) //cross product max component is toleAng
		return false;
#else
	Vector3d segmVecA = segmA[1] - segmA[0];
	Vector3d segmVecB = segmB[1] - segmB[0];
	//if (segmVecA.cross(segmVecB).squaredNorm() > segmVecA.squaredNorm() * segmVecB.squaredNorm() * toleAng * toleAng)
	if (segmVecA.norm() * segmVecB.norm() * toleAng < segmVecA.cross(segmVecB).norm())
		return false;
#endif // USING_NORMALIZED_VECTOR
	int interEnd = 0;
#ifdef USING_NORMALIZED_VECTOR
	for (const auto& endA : segmA)
	{
		segmVecA= segmB[0] - endA; // re-using variable name
		segmVecB= segmB[1] - endA;
		if (segmVecA.norm() <= toleDis || segmVecB.norm() <= toleDis) // point concident
		{
			interEnd++;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm()) //norm fast than squaredNorm
			interEnd++;
	}
	for (const auto& endB : segmB)
	{
		segmVecA = segmA[0] - endB;
		segmVecB = segmA[1] - endB;
		if (segmVecA.norm() <= toleDis || segmVecB.norm() <= toleDis) // point concident
		{
			interEnd++;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm())
			interEnd++;
	}
#else
	Vector3d vecSeg;
	double projection, k;
	auto _getDistanceOfPointAndSegmentINF = [&](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double
	{
		vecSeg = segm[1] - segm[0];// not zero
		projection = vecSeg.dot(point);
		//the projection must on segment
		if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
			return DBL_MAX;
		k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
		return (segm[0] - point + k * vecSeg).squaredNorm();
};
	for (const auto& endA: segmA)
	{
		if (_getDistanceOfPointAndSegmentINF(endA, segmB) <= toleDis * toleDis)
			interEnd++;
	}
	for (const auto& endB: segmB)
	{
		if (_getDistanceOfPointAndSegmentINF(endB, segmA) <= toleDis * toleDis)
			interEnd++;
	}
#endif // USING_NORMALIZED_VECTOR
	return 1 < interEnd; //interEnd == 2/3/4
}

std::tuple<bool, array<double, 4>> psykronix::getTwoSegmentsCollinearCoincidentPoints(const array<Vector3d, 2>& segmA, const array<Vector3d, 2>& segmB,
	double toleAng /*= 0*/, double toleDis /*= 0*/)
{
	Vector3d segmVecA = segmA[1] - segmA[0];
	Vector3d segmVecB = segmB[1] - segmB[0];
	double propA0 = std::nan("0"), propA1 = std::nan("0"), propB0 = std::nan("0"), propB1 = std::nan("0");
	if (toleAng * segmVecA.norm() * segmVecB.norm() < segmVecA.cross(segmVecB).norm())
		return { false, { propA0, propA1, propB0, propB1 } }; //not collinear
	bool sameDirect = (segmVecA).dot(segmVecB) > 0;
	bool midInter = false;
	int endInter = 0;
	for (const auto& endA : segmA) //pointA on segmentB
	{
		segmVecA = endA - segmB[0]; // re-using variable name
		segmVecB = endA - segmB[1];
		if (segmVecA.norm() <= toleDis) // point concident
		{
			endInter++; // same as segmentB 
			propB0 = 0.0;
			continue;
		}
		if (segmVecB.norm() <= toleDis) // point concident
		{
			endInter++;
			propB1 = 1.0;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm()) //norm fast than squaredNorm
		{
			midInter = true;
			if (sameDirect)
			{
				if ((segmA[0] - segmB[0]).dot(segmA[0] - segmB[1]) < 0) // pointA0 in segmB
					propB0 = (segmA[0] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
				if ((segmA[1] - segmB[0]).dot(segmA[1] - segmB[1]) < 0) // pointA1 in segmB
					propB1 = (segmA[1] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
			}
			else //swap segmentA
			{
				if ((segmA[1] - segmB[0]).dot(segmA[1] - segmB[1]) < 0) // pointA0 in segmB
					propB0 = (segmA[1] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
				if ((segmA[0] - segmB[0]).dot(segmA[0] - segmB[1]) < 0) // pointA1 in segmB
					propB1 = (segmA[0] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
			}
		}
	}
	for (const auto& endB : segmB) //pointB on segmentA
	{
		segmVecA = endB - segmA[0];
		segmVecB = endB - segmA[1];
		if (segmVecA.norm() <= toleDis) // point concident
		{
			propA0 = 0.0;
			continue;
		}
		if (segmVecB.norm() <= toleDis) // point concident
		{
			propA1 = 1.0;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm())
		{
			midInter = true;
			if (sameDirect)
			{
				if ((segmB[0] - segmA[0]).dot(segmB[0] - segmA[1]) < 0) // pointB0 in segmA
					propA0 = (segmB[0] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
				if ((segmB[1] - segmA[0]).dot(segmB[1] - segmA[1]) < 0) // pointB1 in segmA
					propA1 = (segmB[1] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
			}
			else//swap segmentB
			{
				if ((segmB[1] - segmA[0]).dot(segmB[1] - segmA[1]) < 0) // pointB0 in segmA
					propA0 = (segmB[1] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
				if ((segmB[0] - segmA[0]).dot(segmB[0] - segmA[1]) < 0) // pointB1 in segmA
					propA1 = (segmB[0] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
			}
		}
	}
	//bool isInter = 1 < endInter; //interPoints == 2/3/4
	if (!midInter && endInter < 2) // no intersect
		return { false, { propA0, propA1, propB0, propB1  } };
	// propA0 means intersect part start, propA1 means end
	if (isnan(propA0)) 
		propA0 = 0.0;
	if (isnan(propA1))
		propA1 = 1.0;
	if (isnan(propB0))
		propB0 = 0.0;
	if (isnan(propB1))
		propB1 = 1.0;
	return { true, { propA0, propA1, propB0, propB1  }}; //
}


//--------------------------------------------------------------------------------------------------
//  K-dimensional Tree 2d
//--------------------------------------------------------------------------------------------------


//sort the input polygons
std::shared_ptr<KdTreeNode2d> _createKdTree2d(std::vector<std::pair<size_t, Polygon2d>>& polygons, int dimension = 0)
{
	// the kd-tree crud create read update delete
	auto _getTotalBounding = [&](/*const std::vector<Polygon2d>& polygons*/)->Eigen::AlignedBox2d
	{
		AlignedBox2d fullBox;
		for (const auto& iter : polygons)
		{
			fullBox.extend(iter.second.bounding().min());
			fullBox.extend(iter.second.bounding().max());
		}
		return fullBox;
	};

	if (polygons.empty()) //no chance
		return nullptr;

	int direction = dimension % 2;  // the direction of xy, x=0/y=1
	//double splitValue = calculateSplitValue(/*polygons,*/ dimension); 
	//KdTreeNode* currentNode = new KdTreeNode();
	std::shared_ptr<KdTreeNode2d> currentNode = std::make_shared<KdTreeNode2d>();
	//copy polygons
	//std::vector<Polygon2d> polySort = polygons;
	//std::sort(polygons.begin(), polygons.end(), [](const Polygon2d& a, const Polygon2d& b) { return a.boungding().min()[0] < b.boungding().min()[0]; });
	if (polygons.size() != 1) //middle node
	{
		currentNode->m_bound = _getTotalBounding();// calculateBoundingBox(polygons);  
		if (direction % 2 == 0)
			std::sort(polygons.begin(), polygons.end(), [](const pair<size_t, Polygon2d>& a, const pair<size_t, Polygon2d>& b) 
				{ return a.second.bounding().min()[0] < b.second.bounding().min()[0]; }); //x
		else
			std::sort(polygons.begin(), polygons.end(), [](const pair<size_t, Polygon2d>& a, const pair<size_t, Polygon2d>& b) 
				{ return a.second.bounding().min()[1] < b.second.bounding().min()[1]; }); //y
		//splitPolygons(/*polygons, leftPolygons, rightPolygons */ dimension, splitValue);
		size_t dichotomy = polygons.size() / 2; // less | more
		vector<pair<size_t, Polygon2d>> leftPolygons(polygons.begin(), polygons.begin() + dichotomy); //for new child node
		vector<pair<size_t, Polygon2d>> rightPolygons(polygons.begin() + dichotomy, polygons.end());
		// using recursion
		currentNode->m_left = _createKdTree2d(leftPolygons, dimension + 1);
		currentNode->m_right = _createKdTree2d(rightPolygons, dimension + 1);
		currentNode->m_index = -1; //not leaf node
	}
	else // end leaf node
	{
		currentNode->m_bound = polygons[0].second.bounding();// short cut way
		currentNode->m_left = nullptr;
		currentNode->m_right = nullptr;
		currentNode->m_index = polygons[0].first;
	}
	currentNode->m_dimension = direction; //record the xy direciton, alse canbe depth, then use depth % 2
	return currentNode;
}

KdTree2d::KdTree2d(const std::vector<Polygon2d>& _polygons/*, int depth = 0*/)
{
	std::vector<std::pair<size_t, Polygon2d>> polygons;
	for (size_t i = 0; i < _polygons.size(); ++i)//(const auto& iter : _polygons)
	{
		polygons.emplace_back(std::pair<size_t, Polygon2d>{ i, _polygons[i] });
	}
	//return _createKdTree2d(polygons);
	m_kdTree = _createKdTree2d(polygons);
}

std::vector<size_t> KdTree2d::findIntersect(const Polygon2d& polygon)
{
	if (!m_kdTree->isValid() || !polygon.isValid())
		return {}; //test whether is working
	std::vector<size_t> indexes;
	//auto _searchKdTree = [&](shared_ptr<KdTreeNode2d> node)->void
	std::function<void(const shared_ptr<KdTreeNode2d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode2d>& node)->void
	{
		//using recursion
		if (node->m_bound.intersects(polygon.bounding()))
		{
			if (node->m_index == -1) // isnot leaf node
			{
				_searchKdTree(node->m_left);
				_searchKdTree(node->m_right);
			}
			else
			{
				indexes.push_back(node->m_index);
				return;
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

//--------------------------------------------------------------------------------------------------
//  K-dimensional Tree 3d
//--------------------------------------------------------------------------------------------------

//sort the input polygons // Polyface3d self include index,
std::shared_ptr<KdTreeNode3d> _createKdTree3d(std::vector<Polyface3d>& polyfaces, int dimension = 0)
{
	// the kd-tree crud create read update delete
	auto _getTotalBounding = [&](/*const std::vector<Polygon2d>& polygons*/)->Eigen::AlignedBox3d
	{
		AlignedBox3d fullBox;
		for (const auto& iter : polyfaces)
		{
			fullBox.extend(iter.m_bound.min());
			fullBox.extend(iter.m_bound.max());
		}
		return fullBox;
	};
	if (polyfaces.empty()) //no chance
		return nullptr;
	const int direction = dimension % 3;  // the direction of xyz, x=0/y=1/z=2
	std::shared_ptr<KdTreeNode3d> currentNode = std::make_shared<KdTreeNode3d>();
	if (polyfaces.size() != 1) //middle node
	{
		currentNode->m_bound = _getTotalBounding();// calculateBoundingBox(polygons);
		std::sort(polyfaces.begin(), polyfaces.end(),
				[=](const Polyface3d& a, const Polyface3d& b) { return a.m_bound.min()[direction] < b.m_bound.min()[direction]; }); // index of Vector3d
		//splitPolygons(/*polygons, leftPolygons, rightPolygons */ dimension, splitValue);
		size_t dichotomy = polyfaces.size() / 2; // less | more
		vector<Polyface3d> leftPolygons(polyfaces.begin(), polyfaces.begin() + dichotomy); //for new child node
		vector<Polyface3d> rightPolygons(polyfaces.begin() + dichotomy, polyfaces.end());
		// using recursion
		currentNode->m_left = _createKdTree3d(leftPolygons, dimension + 1);
		currentNode->m_right = _createKdTree3d(rightPolygons, dimension + 1);
		currentNode->m_index = -1; //not leaf node
	}
	else // end leaf node
	{
		currentNode->m_bound = polyfaces[0].m_bound;// short cut way
		currentNode->m_left = nullptr;
		currentNode->m_right = nullptr;
		currentNode->m_index = polyfaces[0].m_index;
	}
	currentNode->m_dimension = direction; //record the xy direciton, alse canbe depth, then use depth % 2
	return currentNode;
}

KdTree3d::KdTree3d(std::vector<Polyface3d>& polyfaces)
{
	//return _createKdTree3d(polyfaces);
	m_kdTree = _createKdTree3d(polyfaces);
}

std::vector<size_t> KdTree3d::findIntersect(const Polyface3d& polygon, double tolerance /*= 0.0*/) const
{
	if (m_kdTree.get() == nullptr || polygon.m_index == -1) // cannot be external polyface
		return {}; 
	std::vector<size_t> indexes;
	Eigen::AlignedBox3d curBox = polygon.m_bound;
	if (tolerance != 0.0)
	{
		Vector3d tole(tolerance, tolerance, tolerance);
		curBox.min() -= tole;
		curBox.max() += tole;
	}
	std::function<void(const shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode3d>& node)->void
	{
		//using recursion
		if (node->m_bound.intersects(curBox))
		{
			if (node->m_index == -1) // isnot leaf node
			{
				_searchKdTree(node->m_left);
				_searchKdTree(node->m_right);
			}
			else
			{
				if (polygon.m_index != node->m_index) //exclude self
					indexes.push_back(node->m_index);
				//return;
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

// only for clash, distinguish soft and hard
std::vector<std::tuple<size_t, bool>> KdTree3d::findIntersectClash(const Polyface3d& polygon, double tolerance /*= 0.0*/) const
{
	if (m_kdTree.get() == nullptr || polygon.m_index == -1) // cannot be external polyface
		return {};
	std::vector<std::tuple<size_t, bool>> indexes;
	Eigen::AlignedBox3d curBox = polygon.m_bound;
	if (tolerance != 0.0)
	{
		Vector3d tole(tolerance, tolerance, tolerance);
		curBox.min() -= tole;
		curBox.max() += tole;
	}
	std::function<void(const shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode3d>& node)->void
	{
		//using recursion
		if (node->m_bound.intersects(curBox))
		{
			if (node->m_index == -1) // isnot leaf node
			{
				_searchKdTree(node->m_left);
				_searchKdTree(node->m_right);
			}
			else
			{
				bool is_soft = tolerance > 0.0 && (node->m_bound.intersection(polygon.m_bound).sizes().array() < 0).any(); // origin box not intersect
				if (polygon.m_index != node->m_index) //exclude self
					indexes.push_back({ node->m_index, is_soft });
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

