#pragma once

enum class RelationOfPointAndMesh : int
{
	SURFACE = 0,
	INNER,
	OUTER,
};

enum class RelationOfTwoMesh : int
{
	SEPARATE = 0,
	INTRUSIVE, //d>0
	CONTACT_OUTER, //d==0
	INSEDE_AINB, //total inside
	INSEDE_AINB_CONT, // partly cont 
	INSEDE_AINB_FIT, //all vertex cont
	INSEDE_BINA,
	INSEDE_BINA_CONT,
	INSEDE_BINA_FIT,
};

enum class RelationOfRayAndTrigon : int
{
	CROSS_OUTER = 0,
	CROSS_INNER,
	CROSS_VERTEX_0,
	CROSS_VERTEX_1,
	CROSS_VERTEX_2,
	CROSS_EDGE_01,
	CROSS_EDGE_12,
	CROSS_EDGE_20,
	COIN_EDGE_01, //collinear
	COIN_EDGE_12,
	COIN_EDGE_20,
};

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
std::tuple<Eigen::Vector3d, std::array<size_t, 2>> _getPenetrationDepthOfTwoConvexALL(const ModelMesh& meshA, const ModelMesh& meshB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> _getPenetrationDepthOfTwoConvexBOX(const ModelMesh& meshA, const ModelMesh& meshB);
DLLEXPORT bool isTwoMeshsIntersectSAT(const ModelMesh& meshA, const ModelMesh& meshB);
DLLEXPORT std::tuple<RelationOfTwoMesh, Eigen::Vector3d> getTwoMeshsIntersectRelation(const ModelMesh& meshA, const ModelMesh& meshB);
DLLEXPORT std::tuple<double, std::array<size_t, 2>> getTwoMeshsSeparationDistanceSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance);

namespace mesh
{
	ModelMesh meshLoopSubdivision(const ModelMesh& mesh);

}
