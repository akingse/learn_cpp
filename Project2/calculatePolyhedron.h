#pragma once
namespace psykronix
{
	// polyhedron
	DLLEXPORT bool isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT bool isPointInsidePolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	//DLLEXPORT RayOnTrigon relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isPointContainedInPolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const std::vector<Eigen::Vector3d>& vboA, const std::vector<std::array<int, 3>>& iboA, const std::vector<Eigen::Vector3d>& vboB, const std::vector<std::array<int, 3>>& iboB);
	// collision detection
	DLLEXPORT std::array<std::vector<size_t>, 2> getReducedIntersectTrianglesOfMesh(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, const Eigen::Affine3d& matrix = Eigen::Affine3d::Identity());
	DLLEXPORT bool isTwoMeshsIntersectHard(const ModelMesh& mesh_a, const ModelMesh& mesh_b, const Eigen::Affine3d& matrix = Eigen::Affine3d::Identity());
	DLLEXPORT double getTwoMeshsDistanceSoft(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, Eigen::Vector3d& P, Eigen::Vector3d& Q, const Eigen::Affine3d& matrix = Eigen::Affine3d::Identity());


}
