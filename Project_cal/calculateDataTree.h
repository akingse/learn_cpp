#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from November 2023											   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple k-dimensional tree methods			   *
* License   :  MIT									                           *
*******************************************************************************/

#define USING_AUTO_CLOSE
#define RESERVE_USING_POLYGON2D

namespace psykronix
{
#ifdef RESERVE_USING_POLYGON2D
	class Polygon2d
	{
	public:
		std::vector<Eigen::Vector2d> m_polygon;
		Eigen::AlignedBox2d m_bound;
		long long m_index = -1; // the index in the vector container
		//record the number of polygon's segment, and the proportion of intersect point, and the number of other polygon
		std::vector<std::tuple<size_t, double, size_t>> m_intersect;
		static size_t m_id;
	public:
		Polygon2d(const std::vector<Eigen::Vector2d>& polyline)
		{
			m_polygon = polyline;
			m_id++;
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
					if (psykronix::isTwoSegmentsIntersect(std::array<Eigen::Vector2d, 2>{m_polygon[i], m_polygon[i + 1]},
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

struct KdTreeNode2d
{
	Eigen::AlignedBox2d m_bound;  // 
	std::shared_ptr<KdTreeNode2d> m_left;	//KdTreeNode* m_left;  
	std::shared_ptr<KdTreeNode2d> m_right; //KdTreeNode* m_right; 
	// other data
	long long m_index = -1; // the middle node's index
	std::array<int, 2> m_index2 = { -1,-1 }; //for TrigonPart
	size_t m_dimension = 0; // also means m_depth
	//KdTreeNode() : m_box(), m_left(nullptr), m_right(nullptr) {}
	//KdTreeNode(const Polygon2d& poly) : m_box(poly.boungding()), m_left(nullptr), m_right(nullptr) {}
	bool isValid() const
	{
		return m_bound.isEmpty();
	}
	bool isLeaf() const  //for TrigonPart
	{
		return m_index2 != std::array<int, 2>{ -1, -1 };
	}
};

class KdTree2d
{
private:
	std::shared_ptr<KdTreeNode2d> m_kdTree;
	//static std::shared_ptr<KdTreeNode2d> createKdTree(const std::vector<psykronix::Polygon2d>& polygons);
public:
	KdTree2d() = delete;
	KdTree2d(const std::vector<psykronix::Polygon2d>& polygons);
	KdTree2d(const std::vector<TrigonPart>& triangles);
	std::shared_ptr<KdTreeNode2d> get() const
	{
		return m_kdTree;
	}
	std::vector<size_t> findIntersect(const psykronix::Polygon2d& polygon); //searchFromKdTree
	std::vector<size_t> findIntersect(const TrigonPart& trigon);
};
//----------------------------------------------------------------------------------------------------------------
//  3d
//----------------------------------------------------------------------------------------------------------------

// the k-dimensional tree
struct KdTreeNode3d
{
	Eigen::AlignedBox3d m_bound;  // the extend bounding box
	std::shared_ptr<KdTreeNode3d> m_left;	//KdTreeNode* m_left;  
	std::shared_ptr<KdTreeNode3d> m_right;  //KdTreeNode* m_right; 
	// other data
	long long m_index = -1; // the middle node's index
	std::array<int, 2> m_index2 = { -1,-1 }; //for TrigonPart
	size_t m_dimension = 0; // also means m_depth
	KdTreeNode3d() = default;
	KdTreeNode3d(const psykronix::Polyface3d& polyface, size_t dimension)
	{
		m_dimension = dimension;
		m_bound = polyface.m_bound;
		m_index = polyface.m_index;
		//m_left = nullptr; m_right = nullptr;
	}
	bool isLeaf1() const
	{
		return m_index != -1;
	}
	bool isLeaf2() const
	{
		return m_index2 != std::array<int, 2>{ -1, -1 };
	}
	bool single() const
	{
		return m_left != nullptr && m_right == nullptr;
	}
};

// the k-dimensional tree of 3d polyface, only build kd-tree when construct, only include research function
class KdTree3d
{
private:
	std::shared_ptr<KdTreeNode3d> m_kdTree;
#ifdef CLASH_DETECTION_DEBUG_TEMP //for debug
	size_t m_count = 0; //count total leafs
	size_t m_depth = 0; //the max depth
#endif
	//mutable double m_tolerance = 0;
	mutable Eigen::Vector3d m_tolerance = Eigen::Vector3d(psykronix::eps, psykronix::eps, psykronix::eps); //default with threshold eps

public:
	//static std::shared_ptr<KdTreeNode3d> createKdTree(std::vector<psykronix::Polyface3d>& PolyfaceVct);
	KdTree3d() = delete;
	KdTree3d(const std::vector<psykronix::Polyface3d>& polyfaceVct); // using origin bound-box
	KdTree3d(const std::vector<TrigonPart>& triangles);
	void setTolerance(double tolerance)
	{
		//Vector3d tole = (m_tolerance == 0.0) ? Vector3d(eps, eps, eps) : Vector3d(m_tolerance, m_tolerance, m_tolerance);
		m_tolerance = Eigen::Vector3d(tolerance, tolerance, tolerance);
	}
	std::shared_ptr<KdTreeNode3d> getTree() const
	{
		return m_kdTree;
	}
	std::shared_ptr<KdTreeNode3d>& getTree()
	{
		return m_kdTree;
	}
	// CRUD
	bool insert(const psykronix::Polyface3d& polyface); //only insert the not exsit index
	//bool remove(size_t index);
	bool remove(const psykronix::Polyface3d& polyface); //find by polyface index
	bool update(const psykronix::Polyface3d& polyface); //using polyface self index
	std::vector<size_t> findIntersect(const psykronix::Polyface3d& polyface, double tolerance = 0.0) const; //searchFromKdTree
	std::vector<size_t> findIntersect(const TrigonPart& trigon);
	std::vector<std::tuple<size_t, bool>> findIntersectClash(const psykronix::Polyface3d& polyface) const; // bool means soft-clash
	std::vector<std::tuple<size_t, bool>> findIntersectClash(const psykronix::Polyface3d& polyface, double tolerance) const; //custom tolerance
};

class RTree3d
{

};
