#pragma once
namespace psykronix
{
	// polyhedron
	DLLEXPORT bool isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT bool isMeshConvexPolyhedron(const ModelMesh& mesh);
	DLLEXPORT bool isPointInsidePolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT bool isPointInsidePolyhedron(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//DLLEXPORT RayOnTrigon relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isPointContainedInPolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB);
	// collision detection
	DLLEXPORT std::array<std::vector<size_t>, 2> getReducedIntersectTrianglesOfMesh(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, const Eigen::Affine3d& matrix);
	DLLEXPORT bool isTwoMeshsIntersectHard(const ModelMesh& mesh_a, const ModelMesh& mesh_b);
	DLLEXPORT double getTwoMeshsDistanceSoft(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, Eigen::Vector3d& P, Eigen::Vector3d& Q);

}
