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

namespace psykronix
{
	// polyhedron
	DLLEXPORT bool isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT bool isMeshConvexPolyhedron(const ModelMesh& mesh);
	//DLLEXPORT RayOnTrigon relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT RelationOfPointAndMesh isPointInsidePolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT RelationOfPointAndMesh isPointInsidePolyhedron(const Eigen::Vector3d& point, const ModelMesh& mesh);
	DLLEXPORT bool isPointContainedInPolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT bool isPointContainedInPolyhedron(const Eigen::Vector3d& point, const ModelMesh& mesh);
	DLLEXPORT Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB);

}

// collision detection
DLLEXPORT std::array<std::vector<size_t>, 2> getReducedIntersectTrianglesOfMesh(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, const Eigen::Affine3d& matrix);
DLLEXPORT bool isTwoMeshsIntersectSAT(const ModelMesh& mesh_a, const ModelMesh& mesh_b);
DLLEXPORT RelationOfTwoMesh getTwoMeshsIntersectRelation(const ModelMesh& mesh_a, const ModelMesh& mesh_b);
DLLEXPORT std::tuple<double, std::array<size_t, 2>> getTwoMeshsDistanceSAT(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance);