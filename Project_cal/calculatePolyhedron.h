#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple polyhedron calculation methods		   *
* License   :  MIT									                           *
*******************************************************************************/

namespace psykronix
{
	int isRayLineCrossTriangleMTA(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Triangle& trigon);
	// polyhedron
	bool isMeshConvexPolyhedron(const ModelMesh& mesh);
	int getMeshGenusNumber(const ModelMesh& mesh);
	RelationOfPointAndMesh isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const ModelMesh& mesh); //using random rotate
	bool isPointInsidePolyhedronMTA(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//bool isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	//RayOnTrigon relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	//RelationOfPointAndMesh isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	// caution, the input point is relative or absolute coordinate in mesh
	//bool isPointOnPolyhedronSurface(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//bool isPointInsidePolyhedronAZ(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//bool isPointInsidePolyhedronCL(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//bool isPointInsidePolyhedronFL(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//mesh
	std::vector<Eigen::Vector3d> getNormalVectorOfMeshFace(const ModelMesh& mesh); //using ray method
	std::vector<Eigen::Vector3d> getProfileOutOfMesh(const ModelMesh& mesh, const Plane3d& plane);

}

// collision detection
//std::array<std::vector<size_t>, 2> getReducedIntersectTrianglesOfMesh(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance, const Eigen::Affine3d& matrix);
//Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvex(const ModelMesh& meshA, const ModelMesh& meshB,
//	const std::set<size_t>& faceSetA, const std::set<size_t>& faceSetB, const std::vector<size_t>& vertexVectA, const std::vector<size_t>& vertexVectB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvexALL(const ModelMesh& meshA, const ModelMesh& meshB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> _getPenetrationDepthOfTwoConvexBOX(const ModelMesh& meshA, const ModelMesh& meshB);

bool isTwoMeshsIntersectSAT(const ModelMesh& meshA, const ModelMesh& meshB);
Eigen::Vector3d getPenetrationDepthOfTwoMeshsParts(const ModelMesh& meshA, const ModelMesh& meshB, const std::vector<Eigen::Vector3d>& axesSepa,
	const std::set<size_t>& vboSetA, const std::set<size_t>& vboSetB);
std::tuple<psykronix::RelationOfTwoMesh, Eigen::Vector3d> getTwoMeshsIntersectRelation(const ModelMesh& meshA, const ModelMesh& meshB);
std::tuple<double, std::array<size_t, 2>> getTwoMeshsSeparationDistanceSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance);

namespace games
{
	struct QEMVertex
	{
		Eigen::Vector3d m_vertex;
		double m_error = 0.0;
		//Vertex(double x, double y, double z) : x(x), y(y), z(z), error(0.0) {}
		bool operator<(const QEMVertex& rhs) const
		{
			return m_error > rhs.m_error; //for small root heap
		}
	};

	struct QEMEdge
	{
		std::array<int, 2> m_edge; //unique index
		int m_index; //index of HeMesh
		Eigen::Vector3d m_vertex; //using in first average method
		Eigen::Vector4d m_vbar; //using in QEM
		double m_error = 0.0;
		bool operator<(const QEMEdge& rhs) const
		{
			return m_error > rhs.m_error; //for small root heap
		}
	};

	//struct CompareVertex //using for priority_queue template
	//{
	//	bool operator()(const Vertex& v1, const Vertex& v2) 
	//	{
	//		return v1.m_error < v2.m_error;
	//	}
	//};
	////operator<
	//bool cmpless_QuadricError(const Vertex& vA, const Vertex& vB) //using as function-pointer
	//{
	//	return vA.m_error < vB.m_error;
	//}

	//The half edge data structure
	struct HeEdge;
	struct HeFace;

	struct HeVertex
	{
		HeVertex() = default;
		HeVertex(const Eigen::Vector3d& v) :m_coord(v) {}
		Eigen::Vector3d m_coord;// start vertex coordinate of HalfEdge
		HeEdge* m_incEdge = nullptr; //incident edge, the start of edge
		int m_index = -1;
	};

	struct HeEdge //the half edge
	{
		HeEdge() = default;
		//HeEdge(HeVertex* v0, HeVertex* v1) :m_oriVertex(v0), m_desVertex(v1) {}
		HeVertex* m_oriVertex = nullptr; //start origin vertex
		//HeVertex* m_desVertex = nullptr; //end destination vertex, get by nextEdge
		HeEdge* m_twinEdge = nullptr; //means pair opposite halfedge, get by index
		HeEdge* m_prevEdge = nullptr;
		HeEdge* m_nextEdge = nullptr;
		HeFace* m_incFace = nullptr; //incident face
		int m_index = -1;
		//int m_edge; //the unique edge index
	};

	struct HeFace
	{
		HeFace() = default;
		HeFace(HeEdge* edge) : m_incEdge(edge) {}
		HeEdge* m_incEdge = nullptr; //any one of edges
		Eigen::Vector3d m_normal;
		int m_index = -1;
		inline std::array<int, 3> ibo() const
		{
			std::array<int, 3> face = {
				m_incEdge->m_oriVertex->m_index,
				m_incEdge->m_nextEdge->m_oriVertex->m_index, 
				m_incEdge->m_prevEdge->m_oriVertex->m_index };
			return face;
		}
		inline std::array<Eigen::Vector3d, 3> ibo_v() const
		{
			std::array<Eigen::Vector3d, 3> face = {
				m_incEdge->m_oriVertex->m_coord,
				m_incEdge->m_nextEdge->m_oriVertex->m_coord,
				m_incEdge->m_prevEdge->m_oriVertex->m_coord };
			return face;
		}
		inline bool include(const HeVertex* vt) const
		{
			return //using pointer judge
				vt == m_incEdge->m_oriVertex ||
				vt == m_incEdge->m_nextEdge->m_oriVertex ||
				vt == m_incEdge->m_prevEdge->m_oriVertex;
		}
	};

#define USING_POINTER_VERION
	struct HeMesh
	{
#ifdef USING_POINTER_VERION
		std::vector<HeVertex*> m_vertexes; //vbo
		std::vector<HeEdge*> m_edges;
		std::vector<HeFace*> m_faces; //ibo
#else //using_vector_index
		std::vector<HeVertex> m_vertexes;
		std::vector<HeEdge> m_edges;
		std::vector<HeFace> m_faces;
#endif
		bool isManifold = true;
		//std::vector<bool> is_border;
		HeMesh() = default;
		void clear()
		{
			//Reset();
			for (size_t i = 0; i != m_vertexes.size(); ++i)
			{
				if (m_vertexes[i])
				{
					delete m_vertexes[i];
					m_vertexes[i] = nullptr;
				}
			}
			for (size_t i = 0; i != m_edges.size(); ++i)
			{
				if (m_edges[i])
				{
					delete m_edges[i];
					m_edges[i] = nullptr;
				}
			}
			for (size_t i = 0; i != m_faces.size(); ++i)
			{
				if (m_faces[i])
				{
					delete m_faces[i];
					m_faces[i] = nullptr;
				}
			}
			m_vertexes.clear();
			m_edges.clear();
			m_faces.clear();
		}
		~HeMesh()
		{
			clear();
		}
		//convert
		HeMesh(const ModelMesh& mesh); //fromTriangleMesh
		operator ModelMesh() const; //toTriangleMesh

		////Retrieve/CRUD
		//std::vector<HeEdge*> findEdges(const HeVertex*) const;
		//std::vector<HeEdge*> findEdges(const HeFace*) const;
		//std::vector<HeFace*> findFaces(const HeEdge*) const;
		////function
		//void update();
		//void rearrangeFaceIncEdge();
		//std::vector<int> findConnectedFaces(size_t fid);

	};

	//utility
	ModelMesh meshLoopSubdivision(const ModelMesh& mesh);
	ModelMesh meshQuadricSimpIification(const ModelMesh& mesh, size_t collapseEdgeCount = 0);
	ModelMesh meshQuadricErrorMetricsSimpIification(const ModelMesh& mesh, size_t collapseEdgeCount = 0);
	//halfedge
	HeMesh meshQuadricSimpIification(const HeMesh& mesh, size_t edgeCollapseTarget = 0);

}
