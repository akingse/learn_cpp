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

double _getDistanceOfPointAndSegmentINF(const Vector3d& point, const std::array<Vector3d, 2>& segm)
{
	Vector3d vecSeg = segm[1] - segm[0];// not zero
	double projection = vecSeg.dot(point);
	//the projection must on segment
	if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
		return DBL_MAX;
	double k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
	return (segm[0] - point + k * vecSeg).squaredNorm();
};

#define USING_NORMALIZED_VECTOR
bool psykronix::isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, 
	double toleDis /*= 0*/, double toleAng /*= 0*/)
{
	// |A×B| <= |A||B|sinθ, near zero sinθ approx θ
#ifdef USING_NORMALIZED_VECTOR
	Vector3d segmVecA = (segmA[1] - segmA[0]).normalized();
	Vector3d segmVecB = (segmB[1] - segmB[0]).normalized();
	if (!segmVecA.cross(segmVecB).isZero(toleAng)) //cross product max component is toleAng
		return false;
#else
	Vector3d segmVecA = segmA[1] - segmA[0];
	Vector3d segmVecB = segmB[1] - segmB[0];
	if (segmVecA.cross(segmVecB).squaredNorm() > segmVecA.squaredNorm() * segmVecB.squaredNorm() * toleAng * toleAng)
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
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm()) //or using squaredNorm
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
	return (interEnd == 2 || interEnd == 4);
	//return false;
}

bool psykronix::BooleanOpIntersect(Polygon2d& polyA, Polygon2d& polyB)
{
	if (!polyA.boungding().intersects(polyB.boungding())) // pre-intersect
		return false;
	// using polygonA, iterate polyB
	const std::vector<Eigen::Vector2d>& polygonA = polyA.get();
	const std::vector<Eigen::Vector2d>& polygonB = polyB.get();
	std::array<Eigen::Vector2d, 2> segmentA, segmentB;
	bool isIntersect = false;
#ifdef USING_AUTO_CLOSE
	for (size_t iB = 0; iB < polygonB.size() - 1; ++iB)
	{
		// pre-intersect
		segmentB = { polygonB[iB], polygonB[iB + 1] };
		if (!isSegmentAndBoundingBoxIntersectSAT(segmentB, polyA.boungding()))
			continue;
		for (size_t iA = 0; iA < polygonA.size() - 1; ++iA)
		{
			segmentA = { polygonA[iA], polygonA[iA + 1] };
			if (isTwoSegmentsCollinearCoincident(segmentA, segmentB))
			{
				polyA.m_intersect.push_back({ iA,0,0 });
				polyB.m_intersect.push_back({ iB,0,0 });
				isIntersect = true;
				//return true;
			}
		}
	}
#else
	for (size_t iB = 0; iB < polygonB.size(); ++iB)
	{
		// pre-intersect
		segmentB = (iB != polygonB.size() - 1) ? array<Vector2d, 2>{polygonB[iB], polygonB[iB + 1]} : array<Vector2d, 2>{polygonB[iB], polygonB[0]};
		if (!isSegmentAndBoundingBoxIntersectSAT(segmentB, polyA.boungding()))
			continue;
		for (size_t iA = 0; iA < polygonA.size(); ++iA)
		{
			segmentA = (iA != polygonA.size() - 1) ? array<Vector2d, 2>{polygonA[iA], polygonA[iA + 1]} : array<Vector2d, 2>{polygonA[iA], polygonA[0]};
			if (isTwoSegmentsCollinearCoincident(segmentA, segmentB))
			{
				polyA.m_intersect.push_back({ iA,0,0 });
				polyB.m_intersect.push_back({ iB,0,0 });
				isIntersect = true;
				//return true;
			}
		}
	}
#endif
	return isIntersect;
}

void psykronix::BooleanOpIntersect(std::vector<Polygon2d>& polyVct)
{
	for (size_t i = 0; i < polyVct.size(); ++i)
	{
		for (size_t j = 0; j < polyVct.size(); ++j)
		{
			if (i >= j)
				continue;
			BooleanOpIntersect(polyVct[i], polyVct[j]);
		}
	}
}

void psykronix::BooleanOpIntersect(std::vector<Polygon2d>& polyVctA, std::vector<Polygon2d>& polyVctB)
{
	for (size_t i = 0; i < polyVctA.size(); ++i)
	{
		for (size_t j = 0; j < polyVctB.size(); ++j)
		{
			if (polyVctA[i] == polyVctB[j])
				continue;
			BooleanOpIntersect(polyVctA[i], polyVctB[j]);
		}
	}
}

//--------------------------------------------------------------------------------------------------
//  K-dimensional Tree
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
			fullBox.extend(iter.second.boungding().min());
			fullBox.extend(iter.second.boungding().max());
		}
		return fullBox;
	};

	//auto calculateSplitValue = [&](int dimension)->double
	//{
	//	std::vector<double> values;
	//	// 收集多边形在划分维度上的坐标值
	//	for (const auto& polygon : polygons) 
	//	{
	//		for (const auto& point : polygon.get()) 
	//			values.push_back((dimension == 0) ? point.x() : point.y());  // 根据划分维度选择x或y坐标
	//	}
	//	// 对坐标值进行排序
	//	std::sort(values.begin(), values.end());
	//	// 计算中值
	//	double splitValue;
	//	if (values.size() % 2 == 0) 
	//		// 偶数个值，取中间两个值的平均值作为划分值
	//		splitValue = (values[values.size() / 2 - 1] + values[values.size() / 2]) / 2.0;
	//	else 
	//		// 奇数个值，取中间值作为划分值
	//		splitValue = values[values.size() / 2];
	//	return splitValue;
	//};
	//auto splitPolygons = [&](int dimension, double splitValue)->void
	//{
	//	for (const auto& polygon : polygons) 
	//	{
	//		bool isLeft = true;
	//		bool isRight = true;
	//		for (const auto& point : polygon.get()) 
	//		{
	//			if ((dimension == 0 && point.x() > splitValue) || (dimension == 1 && point.y() > splitValue))
	//				isLeft = false;
	//			if ((dimension == 0 && point.x() < splitValue) || (dimension == 1 && point.y() < splitValue))
	//				isRight = false;
	//		}
	//		//if (isLeft) 
	//		//	leftPolygons.push_back(polygon);
	//		//if (isRight) 
	//		//	rightPolygons.push_back(polygon);
	//	}
	//};

	if (polygons.empty()) //no chance
		return nullptr;

	int direction = dimension % 2;  // the direction of xy, x=0/y=1
	//double splitValue = calculateSplitValue(/*polygons,*/ dimension); 
	//KdTreeNode* currentNode = new KdTreeNode();
	std::shared_ptr<KdTreeNode2d> currentNode = std::make_shared<KdTreeNode2d>();
	//copy polygons
	//std::vector<Polygon2d> polySort = polygons;
	//std::sort(polygons.begin(), polygons.end(), [](const Polygon2d& a, const Polygon2d& b) { return a.boungding().min()[0] < b.boungding().min()[0]; }); //x
	if (polygons.size() != 1) //middle node
	{
		currentNode->m_bound = _getTotalBounding();// calculateBoundingBox(polygons);  
		if (direction % 2 == 0)
			std::sort(polygons.begin(), polygons.end(),
				[](const pair<size_t, Polygon2d>& a, const pair<size_t, Polygon2d>& b) { return a.second.boungding().min()[0] < b.second.boungding().min()[0]; }); //x
		else
			std::sort(polygons.begin(), polygons.end(),
				[](const pair<size_t, Polygon2d>& a, const pair<size_t, Polygon2d>& b) { return a.second.boungding().min()[1] < b.second.boungding().min()[1]; }); //y
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
		currentNode->m_bound = polygons[0].second.boungding();// short cut way
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
		if (node->m_bound.intersects(polygon.boungding()))
		{
			if (node->m_index == -1) // is leaf node
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

KdTree3d::KdTree3d(std::vector<Polyface3d>& polyfaces/*, int depth = 0*/)
{
	//return _createKdTree3d(polyfaces);
	m_kdTree = _createKdTree3d(polyfaces);
}

std::vector<size_t> KdTree3d::findIntersect(const Polyface3d& polygon)
{
	if (m_kdTree.get() == nullptr || !polygon.m_index == -1)
		return {}; 
	std::vector<size_t> indexes;
	std::function<void(const shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode3d>& node)->void
	{
		//using recursion
		if (node->m_bound.intersects(polygon.m_bound))
		{
			if (node->m_index == -1) // is leaf node
			{
				_searchKdTree(node->m_left);
				_searchKdTree(node->m_right);
			}
			else
			{
				if (polygon.m_index != node->m_index) //exclude self
					indexes.push_back(node->m_index);
				return;
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}



