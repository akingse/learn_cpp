#pragma once

enum class RelationOfPointAndMesh : int
{
	SURFACE = 0,
	INNER,
	OUTER,
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
DLLEXPORT bool isTwoMeshsIntersectClashHard(const ModelMesh& mesh_a, const ModelMesh& mesh_b);
DLLEXPORT std::tuple<double, std::array<size_t, 2>> getTwoMeshsDistanceClashSoft(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance);