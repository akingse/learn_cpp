#pragma once

#define USING_AUTO_CLOSE
namespace psykronix
{
	bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	//bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB);
	bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, double toleAng = 0, double toleDis = 0);
	std::tuple<bool, std::array<double, 4>> getTwoSegmentsCollinearCoincidentPoints(const Segment& segmA, const Segment& segmB, double toleAng = 0,	double toleDis = 0);
	void mergeIntersectRegion(std::vector<double>& _range, const std::array<double, 2>& prop);

	class Polygon2d
	{
	private:
		std::vector<Eigen::Vector2d> m_polygon;
		Eigen::AlignedBox2d m_bound;
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

	// for PolyfaceHandlePtr, to fill into k-d tree
	struct Polyface3d
	{
		long long m_index = -1; // the index in the vector container
		Eigen::AlignedBox3d m_bound;  // current polyface bounding box
		// ifdef
		UnifiedIdentify m_identify; // extra information
	};

}

// the k-dimensional tree
struct KdTreeNode2d
{
	Eigen::AlignedBox2d m_bound;  // 
	std::shared_ptr<KdTreeNode2d> m_left;	//KdTreeNode* m_left;  
	std::shared_ptr<KdTreeNode2d> m_right; //KdTreeNode* m_right; 
	// other data
	long long m_index = -1; // the middle node's index
	size_t m_dimension = 0; // also means m_depth
	//KdTreeNode() : m_box(), m_left(nullptr), m_right(nullptr) {}
	//KdTreeNode(const Polygon2d& poly) : m_box(poly.boungding()), m_left(nullptr), m_right(nullptr) {}
	bool isValid() const
	{
		return m_bound.isEmpty();
	}
};

class KdTree2d
{
private:
	std::shared_ptr<KdTreeNode2d> m_kdTree;
	//static std::shared_ptr<KdTreeNode2d> createKdTree(const std::vector<psykronix::Polygon2d>& polygons);
public:
	KdTree2d(const std::vector<psykronix::Polygon2d>& polygons);
	std::shared_ptr<KdTreeNode2d> get() const
	{
		return m_kdTree;
	}
	std::vector<size_t> findIntersect(const psykronix::Polygon2d& polygon); //searchFromKdTree

};

//----------------------------------------------------------------------------------------------------------------
//  3d
//----------------------------------------------------------------------------------------------------------------

struct KdTreeNode3d
{
	Eigen::AlignedBox3d m_bound;  // the extend bounding box
	std::shared_ptr<KdTreeNode3d> m_left;	//KdTreeNode* m_left;  
	std::shared_ptr<KdTreeNode3d> m_right;  //KdTreeNode* m_right; 
	// other data
	long long m_index = -1; // the middle node's index
	size_t m_dimension = 0; // also means m_depth
};


// the k-dimensional tree of 3d polyface, only build kd-tree when construct, only include research function
class KdTree3d
{
private:
	std::shared_ptr<KdTreeNode3d> m_kdTree;
	//static std::shared_ptr<KdTreeNode3d> createKdTree(std::vector<psykronix::Polyface3d>& PolyfaceVct);
public:
	KdTree3d(std::vector<psykronix::Polyface3d>& polyfaceVct);
	std::shared_ptr<KdTreeNode3d> get() const
	{
		return m_kdTree;
	}
	std::vector<size_t> findIntersect(const psykronix::Polyface3d& polyface, double tolerance = 0.0) const; //searchFromKdTree
	std::vector<std::tuple<size_t, bool>> findIntersectClash(const psykronix::Polyface3d& polyface, double tolerance) const; // bool means soft-clash

};
