#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from November 2023											   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple tree data structure methods			   *
* License   :  MIT									                           *
*******************************************************************************/

#define USING_AUTO_CLOSE
#define RESERVE_USING_POLYGON2D

namespace clash
{
	class RectBase2d
	{
	public:
		Eigen::AlignedBox2d m_bound;
		long long m_index = -1;
		RectBase2d() = default;
		virtual ~RectBase2d() {};
	};
	class RectBase3d
	{
	public:
		Eigen::AlignedBox3d m_bound;
		long long m_index = -1;
		RectBase3d() = default;
		virtual ~RectBase3d() {};
	};

#ifdef RESERVE_USING_POLYGON2D
	class Polygon2d
	{
	public:
		std::vector<Eigen::Vector2d> m_polygon;
		Eigen::AlignedBox2d m_bound;
		long long m_index = -1; // the index in the vector container
		//record the number of polygon's segment, and the proportion of intersect point, and the number of other polygon
		std::vector<std::tuple<size_t, double, size_t>> m_intersect;
		//static size_t m_id;
	public:
		Polygon2d() = default;
		Polygon2d(const std::vector<Eigen::Vector2d>& polyline)
		{
			m_polygon = polyline;
			//m_id++;
			for (const auto& iter : m_polygon)
				m_bound.extend(iter);
#ifdef USING_AUTO_CLOSE
			if (isValid() && m_polygon[0] != m_polygon[m_polygon.size() - 1]) // !m_polygon.empty(), auto close
				m_polygon.push_back(m_polygon[0]);
#endif
		}
		const std::vector<Eigen::Vector2d>& get() const
		{
			return m_polygon;
		}
		std::vector<std::tuple<size_t, double, size_t>>& intersection()
		{
			return m_intersect;
		}
		Eigen::AlignedBox2d bounding() const
		{
			return m_bound;
		}
		inline bool isValid() const
		{
			if (m_polygon.size() < 4) //at least is trigon
				return false;
			// no self intersect
#ifdef USING_EXTRA_SAFE_CHECK
			for (size_t i = 0; i < m_polygon.size() - 1; ++i)
			{
				for (size_t j = i; i < m_polygon.size() - 1; ++j)
				{
					if (clash::isTwoSegmentsIntersect(std::array<Eigen::Vector2d, 2>{m_polygon[i], m_polygon[i + 1]},
						std::array<Eigen::Vector2d, 2>{m_polygon[i], m_polygon[i + 1]}))
						return false;
				}
			}
#endif
			return true;
		}
		inline bool operator==(const Polygon2d& rhs) const
		{
			if (m_polygon.size() != rhs.m_polygon.size())
				return false;
			return memcmp(m_polygon.data(), rhs.m_polygon.data(), sizeof(Polygon2d) * m_polygon.size()) == 0;
		}
		inline bool operator<(const Polygon2d& rhs) const
		{
			if (m_polygon.size() != rhs.m_polygon.size())
				return false;
			return memcmp(m_polygon.data(), rhs.m_polygon.data(), sizeof(Polygon2d) * m_polygon.size()) == -1;
		}
	};

#endif // RESERVE_USING_POLYGON2D

	// for PolyfaceHandlePtr, to fill into k-d tree
	struct Polyface3d
	{
		long long m_index = -1; // the index in the vector container
		Eigen::AlignedBox3d m_bound;  // current polyface bounding box
#ifdef CLASH_DETECTION_DEBUG_TEMP
		UnifiedIdentify m_identify; // extra information
#endif
	};

}
//----------------------------------------------------------------------------------------------------------------
//  2d
//----------------------------------------------------------------------------------------------------------------

//Bounding Volum Hierarchy
struct BVHNode2d
{
	Eigen::AlignedBox2d m_bound;  // 
	std::shared_ptr<BVHNode2d> m_left;	//BVHNode2d* m_left;  
	std::shared_ptr<BVHNode2d> m_right; //BVHNode2d* m_right; 
	long long m_index = -1; // the middle node's index
#ifdef TEST_CALCULATION_DEBUG
	// other data
	//std::array<int, 2> m_index2 = { -1,-1 }; //for TrigonPart
	size_t m_dimension = 0; // also means m_depth
#endif
	//BVHNode2d() : m_box(), m_left(nullptr), m_right(nullptr) {}
	//BVHNode2d(const Polygon2d& poly) : m_box(poly.boungding()), m_left(nullptr), m_right(nullptr) {}
	bool isValid() const
	{
		return m_bound.isEmpty();
	}

};

class BVHTree2d
{
private:
	std::shared_ptr<BVHNode2d> m_tree;
public:
	BVHTree2d() = delete;
	BVHTree2d(const std::vector<clash::Polygon2d>& polygons);
	BVHTree2d(const std::vector<eigen::TrigonPart>& triangles);
	BVHTree2d(const std::vector<eigen::ContourPart>& profiles);
	std::shared_ptr<BVHNode2d> get() const
	{
		return m_tree;
	}
	std::vector<size_t> findIntersect(const clash::Polygon2d& polygon); //searchFromTree
	std::vector<size_t> findIntersect(const eigen::ContourPart& profile);
	std::vector<size_t> findIntersectOpLess(const eigen::TrigonPart& trigon);//operator less
	std::vector<size_t> findIntersect(const eigen::TrigonPart& trigon); 
};

//----------------------------------------------------------------------------------------------------------------
//  3d
//----------------------------------------------------------------------------------------------------------------

struct BVHNode3d
{
	Eigen::AlignedBox3d m_bound;  // the extend bounding box
	std::shared_ptr<BVHNode3d> m_left;	//BVHNode3d* m_left;  
	std::shared_ptr<BVHNode3d> m_right;  //BVHNode3d* m_right; 
	// other data
	long long m_index = -1; // the middle node's index
	//std::array<int, 2> m_index2 = { -1,-1 }; //for TrigonPart
	size_t m_dimension = 0; // also means m_depth
	BVHNode3d() = default;
	BVHNode3d(const clash::Polyface3d& polyface, size_t dimension)
	{
		m_dimension = dimension;
		m_bound = polyface.m_bound;
		m_index = polyface.m_index;
		//m_left = nullptr; m_right = nullptr;
	}
	bool single() const
	{
		return m_left != nullptr && m_right == nullptr;
	}
	//bool isLeaf1() const
	//{
	//	return m_index != -1;
	//}
	//bool isLeaf2() const
	//{
	//	return m_index2 != std::array<int, 2>{ -1, -1 };
	//}
};

// the BVH tree of 3d polyface
class BVHTree3d
{
private:
	std::shared_ptr<BVHNode3d> m_tree;
#ifdef CLASH_DETECTION_DEBUG_TEMP //for debug
	size_t m_count = 0; //count total leafs
	size_t m_depth = 0; //the max depth
#endif
	//mutable double m_tolerance = 0;
	mutable Eigen::Vector3d m_tolerance = Eigen::Vector3d(clash::epsF, clash::epsF, clash::epsF); //default with threshold epsF
public:
	BVHTree3d() = delete;
	BVHTree3d(const std::vector<clash::Polyface3d>& polyfaceVct); // using origin bound-box
	BVHTree3d(const std::vector<eigen::TrigonPart>& triangles);
	void setTolerance(double tolerance)
	{
		//Vector3d tole = (m_tolerance == 0.0) ? Vector3d(epsF, epsF, epsF) : Vector3d(m_tolerance, m_tolerance, m_tolerance);
		m_tolerance = Eigen::Vector3d(tolerance, tolerance, tolerance);
	}
	std::shared_ptr<BVHNode3d> getTree() const
	{
		return m_tree;
	}
	std::shared_ptr<BVHNode3d>& getTree()
	{
		return m_tree;
	}
	// the tree crud create read update delete
	bool insert(const clash::Polyface3d& polyface); //only insert the not exsit index
	//bool remove(size_t index);
	bool remove(const clash::Polyface3d& polyface); //find by polyface index
	bool update(const clash::Polyface3d& polyface); //using polyface self index
	std::vector<size_t> findIntersect(const clash::Polyface3d& polyface, double tolerance = 0.0) const; //searchFromTree
	std::vector<size_t> findIntersect(const eigen::TrigonPart& trigon);
	std::vector<std::tuple<size_t, bool>> findIntersectClash(const clash::Polyface3d& polyface) const; // bool means soft-clash
	std::vector<std::tuple<size_t, bool>> findIntersectClash(const clash::Polyface3d& polyface, double tolerance) const; //custom tolerance
};

//--------------------------------------------------------------------------------------------------
//  spatial
//--------------------------------------------------------------------------------------------------

namespace spatial
{
	//template<class T>
	struct QuadtreeNode 
	{
		//Eigen::AlignedBox2d data; //T data;
		//QuadtreeNode* children[2][2]; //std::shared_ptr<QuadtreeNode>
		//int divide;  //
		Eigen::Vector2d position;  
		double size;    
		std::vector<int> points;  
		std::array<QuadtreeNode*, 4> children; 

		QuadtreeNode(const Eigen::Vector2d& pos, double sz) : position(pos), size(sz)
		{
			for (int i = 0; i < 4; ++i) 
				children[i] = nullptr;
		}
		~QuadtreeNode() 
		{
			for (int i = 0; i < 4; ++i) 
				delete children[i];
		}
	};

	class Quadtree //4
	{
	public:
		QuadtreeNode* m_tree;
		int m_depth;
		//void build(const std::vector<clash::RectBase2d>& rect)
		//{
		//	auto _quadCreate = []()
		//	{
		//	};
		//}
		QuadtreeNode* createQuadtreeNode(const Eigen::Vector2d& position, double size);
		std::vector<int> intersectSearch(const QuadtreeNode* node, const Eigen::AlignedBox2d& box);

	};
	
	template<class T>
	struct OctreeNode
	{
		T data;
		OctreeNode* children[2][2][2]; //std::shared_ptr<OctreeNode>
		int divide;  //
	};

	template<class T>
	class Octree //8
	{
	public:
		OctreeNode<T>* m_tree;

	};

	struct RTreeNode
	{
		int m_index; 
		Eigen::AlignedBox3d m_bound; //Minimum Bounding Rectangles
		std::vector<std::shared_ptr<RTreeNode>> m_child;
	};

	class RTree3d
	{
	public:
		std::vector<std::shared_ptr<RTreeNode>> m_tree;

	};

	struct KDTreeNode 
	{
		Eigen::AlignedBox3d bounds; 
		int index = -1;
		std::shared_ptr<KDTreeNode> left = nullptr;
		std::shared_ptr<KDTreeNode> right = nullptr;
		//KDTreeNode() = default;
		//KDTreeNode(const KDTreeNode&) = default;
		//KDTreeNode(KDTreeNode&&) = default;
		//KDTreeNode& operator=(const KDTreeNode&) = default;
		//KDTreeNode& operator=(KDTreeNode&&) = default;
	};

	//typedef std::vector<KdTreeNode, Eigen::aligned_allocator<KdTreeNode>> NodeVector;
	typedef std::vector<KDTreeNode> NodeVector;

	class KDTree
	{
	public:
		std::shared_ptr<KDTreeNode> m_tree;
		KDTree(const std::vector<clash::RectBase3d>& data);
		void searchIntersect(const Eigen::AlignedBox3d& queryBox, std::vector<int>& results) const;

	};

}