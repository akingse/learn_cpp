#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple polyhedron calculation methods		   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_POLYHEDRON_H
#define CALCULATE_POLYHEDRON_H
namespace clash
{
	// polyhedron
	DLLEXPORT_CAL bool isMeshConvexPolyhedron(const ModelMesh& mesh);
	int getMeshGenusNumber(const ModelMesh& mesh);
	RelationOfPointAndMesh isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const ModelMesh& mesh); //using random rotate
	bool isPointInsidePolyhedronMTA(const Eigen::Vector3d& point, const ModelMesh& mesh);

	// caution, the input point is relative or absolute coordinate in mesh
	//bool isPointOnPolyhedronSurface(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//bool isPointInsidePolyhedronAZ(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//bool isPointInsidePolyhedronCL(const Eigen::Vector3d& point, const ModelMesh& mesh);
	//bool isPointInsidePolyhedronFL(const Eigen::Vector3d& point, const ModelMesh& mesh);
	
	//mesh
	std::vector<Eigen::Vector3d> getNormalVectorOfMeshFace(const ModelMesh& mesh); //using ray method
	std::vector<Eigen::Vector3d> getProfileOutOfMesh(const ModelMesh& mesh, const Plane3d& plane);
	ModelMesh mergeMultiMeshsToOneMesh(const std::vector<ModelMesh>& meshVct);

	//clash using
	bool isTwoMeshsIntersectSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance = 0.0);
	Eigen::Vector3d getPenetrationDepthOfTwoMeshsParts(const ModelMesh& meshA, const ModelMesh& meshB, const std::vector<Eigen::Vector3d>& axesSepa, const std::set<int>& vboSetA, const std::set<int>& vboSetB);
	std::tuple<clash::RelationOfTwoMesh, Eigen::Vector3d> getTwoMeshsIntersectRelation(const ModelMesh& meshA, const ModelMesh& meshB);
	std::tuple<double, std::array<int, 2>> getTwoMeshsSeparationDistanceSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance);

}

// collision detection
//std::array<std::vector<size_t>, 2> getReducedIntersectTrianglesOfMesh(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance, const Eigen::Affine3d& matrix);
//Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvex(const ModelMesh& meshA, const ModelMesh& meshB,
//	const std::set<size_t>& faceSetA, const std::set<size_t>& faceSetB, const std::vector<size_t>& vertexVectA, const std::vector<size_t>& vertexVectB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvexALL(const ModelMesh& meshA, const ModelMesh& meshB);
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> _getPenetrationDepthOfTwoConvexBOX(const ModelMesh& meshA, const ModelMesh& meshB);

inline clash::TriMesh createTriMesh_UnitCube()
{
	clash::TriMesh mesh;
	mesh.vbo_ = {
		Eigen::Vector3d(1.0,  1.0,  1.0), // 0
		Eigen::Vector3d(-1.0,  1.0,  1.0), // 1
		Eigen::Vector3d(-1.0, -1.0,  1.0), // 2
		Eigen::Vector3d(1.0, -1.0,  1.0), // 3
		Eigen::Vector3d(1.0,  1.0, -1.0), // 4
		Eigen::Vector3d(-1.0,  1.0, -1.0), // 5
		Eigen::Vector3d(-1.0, -1.0, -1.0), // 6
		Eigen::Vector3d(1.0, -1.0, -1.0)  // 7
	};
	mesh.ibo_ = {
		// 前脸 (z = 1)
		{0, 1, 2}, {0, 2, 3},
		// 后脸 (z = -1)
		{4, 5, 6}, {4, 6, 7},
		// 顶脸 (y = 1)
		{0, 4, 5}, {0, 5, 1},
		// 底脸 (y = -1)
		{2, 6, 7}, {2, 7, 3},
		// 右脸 (x = 1)
		{0, 3, 7}, {0, 7, 4},
		// 左脸 (x = -1)
		{1, 5, 6}, {1, 6, 2}
	};
	mesh.fno_ = {
		// 前脸法线 (0,0,1)
		{0, 0, 1}, {0, 0, 1},
		// 后脸法线 (0,0,-1)
		{0, 0, -1}, {0, 0, -1},
		// 顶脸法线 (0,1,0)
		{0, 1, 0}, {0, 1, 0},
		// 底脸法线 (0,-1,0)
		{0, -1, 0}, {0, -1, 0},
		// 右脸法线 (1,0,0)
		{1, 0, 0}, {1, 0, 0},
		// 左脸法线 (-1,0,0)
		{-1, 0, 0}, {-1, 0, 0}
	};
	mesh.bounding_ = Eigen::AlignedBox3d(Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d(1.0, 1.0, 1.0));
	return mesh;
}


#endif// CALCULATE_POLYHEDRON_H
