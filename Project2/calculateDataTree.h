#pragma once

#define USING_AUTO_CLOSE
namespace psykronix
{
	class Polygon2d;
	bool isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB);
	bool BooleanOpIntersect(Polygon2d& polyA, Polygon2d& polyB);
	void BooleanOpIntersect(std::vector<Polygon2d>& polyVct);
	void BooleanOpIntersect(std::vector<Polygon2d>& polyVctA, std::vector<Polygon2d>& polyVctB);

	class Polygon2d
	{
		friend bool BooleanOpIntersect(Polygon2d& polyA, Polygon2d& polyB);
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
		Eigen::AlignedBox2d boungding() const
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
}

// the k-dimensional tree
struct KdTreeNode
{
	Eigen::AlignedBox2d m_box;  // 
	std::shared_ptr<KdTreeNode> m_left;	//KdTreeNode* m_left;  
	std::shared_ptr<KdTreeNode> m_right; //KdTreeNode* m_right; 
	// other data
	long long m_index = -1; // the middle node's index
	size_t m_dimension = 0; // also means m_depth
	//KdTreeNode() : m_box(), m_left(nullptr), m_right(nullptr) {}
	//KdTreeNode(const Polygon2d& poly) : m_box(poly.boungding()), m_left(nullptr), m_right(nullptr) {}
	bool isValid() const
	{
		return m_box.isEmpty();
	}
};

class KdTree
{
private:
	std::shared_ptr<KdTreeNode> m_kdTree;
	static std::shared_ptr<KdTreeNode> createKdTree(const std::vector<psykronix::Polygon2d>& polygons);
public:
	KdTree(const std::vector<psykronix::Polygon2d>& polygons)
	{
		m_kdTree = createKdTree(polygons);
	}
	std::shared_ptr<KdTreeNode> get() const
	{
		return m_kdTree;
	}
	std::vector<size_t> findIntersect(const psykronix::Polygon2d& polygon); //searchFromKdTree

};


