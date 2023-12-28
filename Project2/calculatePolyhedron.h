#pragma once
namespace psykronix
{
	// polyhedron
	DLLEXPORT bool isMeshConvexPolyhedron(const ModelMesh& mesh);
	//DLLEXPORT bool isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	//DLLEXPORT RayOnTrigon relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	//DLLEXPORT RelationOfPointAndMesh isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	// caution, the input point is relative or absolute coordinate in mesh
	DLLEXPORT RelationOfPointAndMesh isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const ModelMesh& mesh);
	DLLEXPORT bool isPointInsidePolyhedronAZ(const Eigen::Vector3d& point, const ModelMesh& mesh);
	DLLEXPORT bool isPointInsidePolyhedronCL(const Eigen::Vector3d& point, const ModelMesh& mesh);
	DLLEXPORT bool isPointInsidePolyhedronFL(const Eigen::Vector3d& point, const ModelMesh& mesh);
	DLLEXPORT bool isPointOnPolyhedronSurface(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//DLLEXPORT Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB);
}

// collision detection
//DLLEXPORT std::array<std::vector<size_t>, 2> getReducedIntersectTrianglesOfMesh(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance, const Eigen::Affine3d& matrix);
std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvex(const ModelMesh& meshA, const ModelMesh& meshB,
	const std::set<size_t>& faceSetA, const std::set<size_t>& faceSetB, const std::vector<size_t>& vertexVectA, const std::vector<size_t>& vertexVectB);
Eigen::Vector3d getPenetrationDepthOfTwoMeshsParts(const ModelMesh& meshA, const ModelMesh& meshB, const std::vector<Eigen::Vector3d>& axesSepa,
	const std::set<size_t>& vboSetA, const std::set<size_t>& vboSetB);
std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvexALL(const ModelMesh& meshA, const ModelMesh& meshB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> _getPenetrationDepthOfTwoConvexBOX(const ModelMesh& meshA, const ModelMesh& meshB);
DLLEXPORT bool isTwoMeshsIntersectSAT(const ModelMesh& meshA, const ModelMesh& meshB);
DLLEXPORT std::tuple<psykronix::RelationOfTwoMesh, Eigen::Vector3d> getTwoMeshsIntersectRelation(const ModelMesh& meshA, const ModelMesh& meshB);
DLLEXPORT std::tuple<double, std::array<size_t, 2>> getTwoMeshsSeparationDistanceSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance);

namespace games
{
	struct Vertex 
	{
		Eigen::Vector3d m_vertex;
		double m_error = 0.0;
		//Vertex(double x, double y, double z) : x(x), y(y), z(z), error(0.0) {}
	};

	struct CompareVertex 
	{
		bool operator()(const Vertex& v1, const Vertex& v2) 
		{
			return v1.m_error > v2.m_error;
		}
	};
	ModelMesh meshLoopSubdivision(const ModelMesh& mesh);
	ModelMesh meshQuadricErrorSimpIification(const ModelMesh& mesh, size_t targetVertexCount = 0);
}
