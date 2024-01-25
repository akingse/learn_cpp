#include "pch.h"
#include "calculateDataTree.h"

using namespace std;
using namespace psykronix;
using namespace Eigen;
using namespace eigen;

#ifdef min
#undef min
#endif // 
#ifdef max
#undef max
#endif // 

//--------------------------------------------------------------------------------------------------
//  K-dimensional Tree 2d
//--------------------------------------------------------------------------------------------------

#ifdef RESERVE_USING_POLYGON2D
//size_t Polygon2d::m_id = 0;
//sort the input polygons
std::shared_ptr<KdTreeNode2d> _createKdTree2d(std::vector<Polygon2d>& polygons, int dimension = 0)
{
	// the kd-tree crud create read update delete
	auto _getTotalBounding = [&polygons](/*const std::vector<Polygon2d>& polygons*/)->Eigen::AlignedBox2d
	{
		AlignedBox2d fullBox;
		for (const auto& iter : polygons)
		{
			fullBox.extend(iter.bounding().min());
			fullBox.extend(iter.bounding().max());
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
			std::sort(polygons.begin(), polygons.end(), [](const Polygon2d& a, const Polygon2d& b)
				{ return a.bounding().min()[0] < b.bounding().min()[0]; }); //x
		else
			std::sort(polygons.begin(), polygons.end(), [](const Polygon2d& a, const Polygon2d& b)
				{ return a.bounding().min()[1] < b.bounding().min()[1]; }); //y
		//splitPolygons(/*polygons, leftPolygons, rightPolygons */ dimension, splitValue);
		size_t dichotomy = polygons.size() / 2; // less | more
		vector<Polygon2d> leftPolygons(polygons.begin(), polygons.begin() + dichotomy); //for new child node
		vector<Polygon2d> rightPolygons(polygons.begin() + dichotomy, polygons.end());
		// using recursion
		currentNode->m_left = _createKdTree2d(leftPolygons, dimension + 1);
		currentNode->m_right = _createKdTree2d(rightPolygons, dimension + 1);
		currentNode->m_index = -1; //not leaf node
	}
	else // end leaf node
	{
		currentNode->m_bound = polygons[0].bounding();// short cut way
		currentNode->m_left = nullptr;
		currentNode->m_right = nullptr;
		currentNode->m_index = polygons[0].m_index;
	}
	currentNode->m_dimension = direction; //record the xy direciton, alse canbe depth, then use depth % 2
	return currentNode;
}

KdTree2d::KdTree2d(const std::vector<Polygon2d>& _polygons/*, int depth = 0*/)
{
	//std::vector<std::pair<size_t, Polygon2d>> polygons;
	//for (size_t i = 0; i < _polygons.size(); ++i)//(const auto& iter : _polygons)
	//{
	//	polygons.emplace_back(std::pair<size_t, Polygon2d>{ i, _polygons[i] });
	//}
	std::vector<Polygon2d> polygons = _polygons; //copy
	m_kdTree = _createKdTree2d(polygons);
}

//template<class T>
std::shared_ptr<KdTreeNode2d> _createKdTree2d(std::vector<TrigonPart>& triangles, int dimension = 0)
{
	// the kd-tree crud create read update delete
	auto _getTotalBounding = [&triangles](/*const std::vector<TrigonPart>& polygons*/)->Eigen::AlignedBox2d
	{
		AlignedBox2d fullBox;
		for (const auto& iter : triangles)
		{
			fullBox.extend(iter.m_box2d.min());
			fullBox.extend(iter.m_box2d.max());
		}
		return fullBox;
	};
	if (triangles.empty()) //no chance
		return nullptr;
	int direction = dimension % 2;  // the direction of xy, x=0/y=1
	std::shared_ptr<KdTreeNode2d> currentNode = std::make_shared<KdTreeNode2d>();
	if (triangles.size() != 1) //middle node
	{
		currentNode->m_bound = _getTotalBounding();// calculateBoundingBox(polygons);  
		if (direction % 2 == 0)
			std::sort(triangles.begin(), triangles.end(), [](const TrigonPart& a, const TrigonPart& b)
				{ return a.m_box2d.min()[0] < b.m_box2d.min()[0]; }); //x
		else
			std::sort(triangles.begin(), triangles.end(), [](const TrigonPart& a, const TrigonPart& b)
				{ return a.m_box2d.min()[1] < b.m_box2d.min()[1]; }); //y
		//splitPolygons(/*polygons, leftPolygons, rightPolygons */ dimension, splitValue);
		size_t dichotomy = triangles.size() / 2; // less | more
		vector<TrigonPart> leftPolygons(triangles.begin(), triangles.begin() + dichotomy); //for new child node
		vector<TrigonPart> rightPolygons(triangles.begin() + dichotomy, triangles.end());
		// using recursion
		currentNode->m_left = _createKdTree2d(leftPolygons, dimension + 1);
		currentNode->m_right = _createKdTree2d(rightPolygons, dimension + 1);
		//currentNode->m_index = -1; //not leaf node
	}
	else // end leaf node
	{
		currentNode->m_bound = triangles[0].m_box2d;
		currentNode->m_left = nullptr;
		currentNode->m_right = nullptr;
		currentNode->m_index2 = triangles[0].m_index;
		currentNode->m_index = triangles[0].m_number;
	}
	currentNode->m_dimension = direction; //record the xy direciton, alse canbe depth, then use depth % 2
	return currentNode;
}

KdTree2d::KdTree2d(const std::vector<TrigonPart>& _triangles)
{
	std::vector<TrigonPart> triangles = _triangles; //copy
	m_kdTree = _createKdTree2d(triangles);
}

KdTree2d::KdTree2d(const std::vector<ProfilePart>& profiles)
{
	std::vector<Polygon2d> polygons;
	for (size_t i = 0; i < profiles.size(); ++i)
	{
		Polygon2d polygon;
		polygon.m_index = i;
		const Vector3d& min = profiles[i].m_box3d.min();
		const Vector3d& max = profiles[i].m_box3d.max();
		polygon.m_bound = AlignedBox2d(Vector2d(min[0], min[1]), Vector2d(max[0], max[1]));
		polygons.push_back(polygon);
	}
	m_kdTree = _createKdTree2d(polygons);
}

std::vector<size_t> KdTree2d::findIntersect(const Polygon2d& polygon)
{
	if (m_kdTree == nullptr || !polygon.isValid())
		return {}; //test whether is working
	std::vector<size_t> indexes;
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
				if (polygon.m_index != node->m_index) //exclude self
					indexes.push_back(node->m_index);
				//return;
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

std::vector<size_t> KdTree2d::findIntersect(const TrigonPart& trigon)
{
	if (m_kdTree == nullptr)
		return {}; //test whether is working
	std::vector<size_t> indexes;
	std::function<void(const shared_ptr<KdTreeNode2d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode2d>& node)->void
	{
		//using recursion
		if (node->m_bound.intersects(trigon.m_box2d))
		{
			if (node->m_index != -1)
			{
				//vector index, only small to large
				if (trigon.m_number < node->m_index) //exclude self
					indexes.push_back(node->m_index);
				return;
			}
			else // isnot leaf node
			{
				_searchKdTree(node->m_left);
				_searchKdTree(node->m_right);
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

std::vector<size_t> KdTree2d::findIntersect(const ProfilePart& profile)
{
	if (m_kdTree == nullptr || profile.isEmpty())
		return {}; //test whether is working
	std::vector<size_t> indexes;
	//const Vector3d& min = profile.m_box3d.min();
	//const Vector3d& max = profile.m_box3d.max();
	//AlignedBox2d bound = AlignedBox2d(Vector2d(min[0], min[1]), Vector2d(max[0], max[1]));
	std::function<void(const shared_ptr<KdTreeNode2d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode2d>& node)->void
	{
		//using recursion
		if (node->m_bound.intersects(profile.m_box2d))
		{
			if (node->m_index == -1) // isnot leaf node
			{
				_searchKdTree(node->m_left);
				_searchKdTree(node->m_right);
			}
			else
			{
				if (profile.m_index < node->m_index) //exclude self
					indexes.push_back(node->m_index);
				//return;
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

#endif RESERVE_USING_POLYGON2D
//--------------------------------------------------------------------------------------------------
//  K-dimensional Tree 3d
//--------------------------------------------------------------------------------------------------

bool KdTree3d::insert(const psykronix::Polyface3d& polyface)
{
	//must be new index of polyface
	std::function<bool(const shared_ptr<KdTreeNode3d>&)> _findIndex = [&](const shared_ptr<KdTreeNode3d>& node)->bool
	{
		//using recursion
		if (node->m_index == -1) // isnot leaf node
		{
			if (_findIndex(node->m_left))
				return true;
			if (_findIndex(node->m_right))
				return true;
		}
		return node->m_index == polyface.m_index;
	};
	if (_findIndex(m_kdTree))
		return false;
	// insert and update bound box
	std::function<AlignedBox3d(shared_ptr<KdTreeNode3d>, shared_ptr<KdTreeNode3d>)> _searchKdTree = [&]
	(shared_ptr<KdTreeNode3d> node, shared_ptr<KdTreeNode3d> father)->AlignedBox3d// return father node bound-box
	{
		//using recursion
		if (node->m_index == -1) // isnot leaf node
		{
			if (polyface.m_bound.min()[node->m_dimension] < node->m_bound.min()[node->m_dimension]) //compare xyz
			{
				node->m_bound = _searchKdTree(node->m_left, node);
			}
			else
			{
				node->m_bound = _searchKdTree(node->m_right, node);
			}
		}
		else //if (node->m_index==-1)
		{
			if (father->single()) // right leaf is empty
			{
				father->m_right = std::make_shared<KdTreeNode3d>(KdTreeNode3d(polyface, node->m_dimension));
				return father->m_left->m_bound.merged(father->m_right->m_bound);
			}
			else
			{
				node->m_left = std::make_shared<KdTreeNode3d>(KdTreeNode3d(polyface, (node->m_dimension + 1) % 3));
				return father->m_bound.merged(polyface.m_bound); //change the box
			}
		}
		return {};
	};
	_searchKdTree(m_kdTree, nullptr);
	return true;
}

//bool KdTree3d::remove(size_t index)
bool KdTree3d::remove(const psykronix::Polyface3d& polyface)
{
	//retrieve first
	//bool isFind = false;
	long long index = polyface.m_index;
	//std::function<bool(shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](shared_ptr<KdTreeNode3d>& node)->bool
	std::function<tuple<bool, AlignedBox3d>(shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](shared_ptr<KdTreeNode3d>& node)
	{
		//using recursion
		if (node->m_index == -1) // isnot leaf node
		{
			if (std::get<0>(_searchKdTree(node->m_left)))
			{
				if (node->m_right != nullptr)
				{
					node->m_bound = std::get<1>(_searchKdTree(node->m_left));
					return tuple<bool, AlignedBox3d>{true, node->m_bound};
				}
				else
				{
					node.reset();
					return tuple<bool, AlignedBox3d>{false, {}};
				}
			}
			if (std::get<0>(_searchKdTree(node->m_right)))
			{
				if (node->m_left != nullptr)
				{
					node->m_bound = std::get<1>(_searchKdTree(node->m_right));
					return tuple<bool, AlignedBox3d>{true, node->m_bound};
				}
				else
				{
					node.reset();
					return tuple<bool, AlignedBox3d>{false, {}};
				}
			}
		}
		if (node->m_index == index)
		{
			//node = nullptr;
			//node->m_index = -1;
			node.reset(); //release the pointer
			return tuple<bool, AlignedBox3d>{true, {}};
		}
		return tuple<bool, AlignedBox3d>{false, {}};
	};
	return std::get<0>(_searchKdTree(m_kdTree));
}

bool KdTree3d::update(const psykronix::Polyface3d& polyface)
{
	std::function<tuple<bool, AlignedBox3d>(shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](shared_ptr<KdTreeNode3d>& node)//->bool
	{
		//using recursion
		if (node->m_index == -1) // isnot leaf node
		{
			if (std::get<0>(_searchKdTree(node->m_left)))
			{
				node->m_bound = std::get<1>(_searchKdTree(node->m_left));
				node->m_bound.merged(node->m_right->m_bound);
				return tuple<bool, AlignedBox3d>{true, node->m_bound};
			}
			if (std::get<0>(_searchKdTree(node->m_right)))
			{
				node->m_bound = std::get<1>(_searchKdTree(node->m_right));
				node->m_bound.merged(node->m_left->m_bound);
				return tuple<bool, AlignedBox3d>{true, node->m_bound};
			}
		}
		if (node->m_index == polyface.m_index)
		{
			node->m_bound = polyface.m_bound; //change the box
			return tuple<bool, AlignedBox3d>{true, polyface.m_bound};
		}
		return tuple<bool, AlignedBox3d>{false, {}};
	};
	return std::get<0>(_searchKdTree(m_kdTree));
	//return false;
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
		currentNode->m_bound = polyfaces.front().m_bound;// size is 1
		currentNode->m_left = nullptr;
		currentNode->m_right = nullptr;
		currentNode->m_index = polyfaces.front().m_index;
	}
	currentNode->m_dimension = direction; //record the xy direciton, alse canbe depth, then use depth % 2
	return currentNode;
}

// record count and depth
std::shared_ptr<KdTreeNode3d> _createKdTree3d(std::vector<Polyface3d>& polyfaces, size_t& count, size_t& depth, int dimension = 0)
{
	// the kd-tree crud create read update delete
	auto _getTotalBounding = [&polyfaces](/*const std::vector<Polygon2d>& polygons*/)->Eigen::AlignedBox3d
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
	depth = dimension;
	std::shared_ptr<KdTreeNode3d> currentNode = std::make_shared<KdTreeNode3d>();
	if (polyfaces.size() != 1) //middle node
	{
		currentNode->m_bound = _getTotalBounding();// calculateBoundingBox(polygons);
		std::sort(polyfaces.begin(), polyfaces.end(),
			[=](const Polyface3d& a, const Polyface3d& b) { return a.m_bound.min()[direction] < b.m_bound.min()[direction]; }); // index of Vector3d
		size_t dichotomy = polyfaces.size() / 2; // less | more
		vector<Polyface3d> leftPolygons(polyfaces.begin(), polyfaces.begin() + dichotomy); //for new child node
		vector<Polyface3d> rightPolygons(polyfaces.begin() + dichotomy, polyfaces.end());
		// using recursion
		currentNode->m_left = _createKdTree3d(leftPolygons, count, depth, dimension + 1);// using recursion
		currentNode->m_right = _createKdTree3d(rightPolygons, count, depth, dimension + 1);
		currentNode->m_index = -1; //not leaf node
	}
	else // end leaf node
	{
		currentNode->m_bound = polyfaces.front().m_bound;// last polyfaces.size()==1
		currentNode->m_left = nullptr;
		currentNode->m_right = nullptr;
		currentNode->m_index = polyfaces.front().m_index;
		count++;
	}
	currentNode->m_dimension = direction; //record the xy direciton, alse canbe depth, then use depth % 2
	return currentNode;
}

// constructor
KdTree3d::KdTree3d(const std::vector<Polyface3d>& polyfaces)
{
	std::vector<Polyface3d> _polyfaces = polyfaces;
#ifdef CLASH_DETECTION_DEBUG_TEMP
	m_kdTree = _createKdTree3d(_polyfaces, m_count, m_depth);
#else
	m_kdTree = _createKdTree3d(_polyfaces);
#endif // DEBUG
}

#ifdef RESERVE_USING_BOUNDBOX_3D
std::shared_ptr<KdTreeNode3d> _createKdTree3d(std::vector<TrigonPart>& triangles, int dimension = 0)
{
	// the kd-tree crud create read update delete
	auto _getTotalBounding = [&](/*const std::vector<Polygon2d>& polygons*/)->Eigen::AlignedBox3d
	{
		AlignedBox3d fullBox;
		for (const auto& iter : triangles)
		{
			fullBox.extend(iter.m_box3d.min());
			fullBox.extend(iter.m_box3d.max());
		}
		return fullBox;
	};
	if (triangles.empty()) //no chance
		return nullptr;
	const int direction = dimension % 3;  // the direction of xyz, x=0/y=1/z=2
	std::shared_ptr<KdTreeNode3d> currentNode = std::make_shared<KdTreeNode3d>();
	if (triangles.size() != 1) //middle node
	{
		currentNode->m_bound = _getTotalBounding();// calculateBoundingBox(polygons);
		std::sort(triangles.begin(), triangles.end(),
			[=](const TrigonPart& a, const TrigonPart& b) { return a.m_box3d.min()[direction] < b.m_box3d.min()[direction]; }); // index of Vector3d
		//splitPolygons(/*polygons, leftPolygons, rightPolygons */ dimension, splitValue);
		size_t dichotomy = triangles.size() / 2; // less | more
		vector<TrigonPart> leftPolygons(triangles.begin(), triangles.begin() + dichotomy); //for new child node
		vector<TrigonPart> rightPolygons(triangles.begin() + dichotomy, triangles.end());
		// using recursion
		currentNode->m_left = _createKdTree3d(leftPolygons, dimension + 1);
		currentNode->m_right = _createKdTree3d(rightPolygons, dimension + 1);
		//currentNode->m_index = -1; //not leaf node
	}
	else // end leaf node
	{
		currentNode->m_bound = triangles.front().m_box3d;// size is 1
		currentNode->m_left = nullptr;
		currentNode->m_right = nullptr;
		currentNode->m_index2 = triangles.front().m_index;
		currentNode->m_index = triangles.front().m_number;
	}
	currentNode->m_dimension = direction; //record the xy direciton, alse canbe depth, then use depth % 2
	return currentNode;
}
#endif

KdTree3d::KdTree3d(const std::vector<TrigonPart>& _triangles)
{
	std::vector<TrigonPart> triangles = _triangles;
	//m_kdTree = _createKdTree3d(triangles);
}

std::vector<size_t> KdTree3d::findIntersect(const Polyface3d& polygon, double tolerance /*= 0.0*/) const
{
	if (m_kdTree == nullptr) // || polygon.m_index == -1 means cannot be external polyface
		return {};
	std::vector<size_t> indexes;
	Eigen::AlignedBox3d curBox = polygon.m_bound;
	if (tolerance != 0.0) // not using default eps
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
					indexes.push_back(node->m_index); //if double loop, index canbe small to large
				return;
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

#ifdef RESERVE_USING_BOUNDBOX_3D
std::vector<size_t> KdTree3d::findIntersect(const TrigonPart& trigon)
{
	if (m_kdTree == nullptr) // || polygon.m_index == -1 means cannot be external polyface
		return {};
	//std::vector<array<int, 2>> indexes;
	std::vector<size_t> indexes;
	std::function<void(const shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode3d>& node)->void
	{
		//using recursion
		if (node->m_bound.intersects(trigon.m_box3d))
		{
			if (node->m_index != -1) // is leaf node
			{
				if (trigon.m_number < node->m_index && trigon.m_index[0] != node->m_index2[0]) //same mesh, must 3d-sepa
					indexes.push_back(node->m_index); //if double loop, index canbe small to large
				return;
			}
			else
			{
				_searchKdTree(node->m_left);
				_searchKdTree(node->m_right);
			}
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}
#endif

// only for clash, distinguish soft and hard
std::vector<std::tuple<size_t, bool>> KdTree3d::findIntersectClash(const Polyface3d& polygon, double tolerance) const
{
	Vector3d copy = m_tolerance;
	m_tolerance = Vector3d(tolerance, tolerance, tolerance);
	std::vector<std::tuple<size_t, bool>> resInter = findIntersectClash(polygon);
	m_tolerance = copy;
	return resInter;
}

std::vector<std::tuple<size_t, bool>> KdTree3d::findIntersectClash(const Polyface3d& polygon) const
{
	if (m_kdTree.get() == nullptr || polygon.m_index == -1) // cannot be external polyface
		return {};
	std::vector<std::tuple<size_t, bool>> indexes;
	Eigen::AlignedBox3d toleBox = polygon.m_bound; //using extra eps
	//Vector3d tole = (m_tolerance == 0.0) ? Vector3d(eps, eps, eps) : Vector3d(m_tolerance, m_tolerance, m_tolerance);
	toleBox.min() -= m_tolerance;
	toleBox.max() += m_tolerance;
	std::function<void(const shared_ptr<KdTreeNode3d>&)> _searchKdTree = [&](const shared_ptr<KdTreeNode3d>& node)->void
	{
		//using recursion
		if (!node->m_bound.intersects(toleBox))
			return;
		if (node->m_index == -1) // isnot leaf node
		{
			_searchKdTree(node->m_left);
			_searchKdTree(node->m_right);
		}
		else
		{
			if (polygon.m_index >= node->m_index) //vector index, only small to large
				return;
			bool is_soft = 0.0 < m_tolerance[0] && !node->m_bound.intersects(polygon.m_bound); // origin box not intersect
			//if (polygon.m_index != node->m_index) //exclude self
			indexes.push_back({ node->m_index, is_soft });
		}
	};
	_searchKdTree(m_kdTree);
	return indexes;
}

