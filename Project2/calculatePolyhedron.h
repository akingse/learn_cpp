#pragma once
namespace psykronix
{
	// polyhedron
	DLLEXPORT bool isMeshConvexPolyhedron(const ModelMesh& mesh);
	DLLEXPORT RelationOfPointAndMesh isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const ModelMesh& mesh); //using random rotate
	//DLLEXPORT bool isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	//DLLEXPORT RayOnTrigon relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	//DLLEXPORT RelationOfPointAndMesh isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	// caution, the input point is relative or absolute coordinate in mesh
	//DLLEXPORT bool isPointOnPolyhedronSurface(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//DLLEXPORT bool isPointInsidePolyhedronAZ(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//DLLEXPORT bool isPointInsidePolyhedronCL(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//DLLEXPORT bool isPointInsidePolyhedronFL(const Eigen::Vector3d& point, const ModelMesh& mesh);
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
	struct Vertex 
	{
		Eigen::Vector3d m_vertex;
		double m_error = 0.0;
		//Vertex(double x, double y, double z) : x(x), y(y), z(z), error(0.0) {}
		bool operator<(const Vertex& rhs) const
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

	struct Edge //unique
	{
		std::array<int, 2> m_edge;
		Eigen::Vector3d m_vertex;
		double m_error = 0.0;
		bool operator<(const Edge& rhs) const
		{
			return m_error > rhs.m_error; //for small root heap
		}
	};

	//utility
	ModelMesh meshLoopSubdivision(const ModelMesh& mesh);
	ModelMesh meshQuadricErrorMetricsSimpIification(const ModelMesh& mesh, size_t collapseEdgeCount = 0);
}
