#include "pch.h"
#include "calculatePolyhedron.h"
using namespace std;
using namespace Eigen;
using namespace psykronix;
static constexpr double eps = FLT_EPSILON; //1e-7
static constexpr double _eps = -FLT_EPSILON;
static Eigen::Vector3d gVecNaN(std::nan("0"), std::nan("0"), std::nan("0"));
#undef max 
#undef min
#define USING_METHOD_SAT
#define USING_RELATIVE_MATRIX_RECTIFY

#ifdef STATISTIC_DATA_COUNT
extern std::atomic<size_t> count_mesh_inside_mesh;
extern std::atomic<size_t> count_reduced_exclude_pre, count_triA_inter, count_triB_inter,
count_err_empty_mesh, count_tri_box_exclude_pre;
extern std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> triPairList, triRecordHard;
extern std::vector<InterTriInfo> interTriInfoList;

#endif //STATISTIC_DATA_COUNT

enum class RayOnTrigon :unsigned int
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

bool psykronix::isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	for (const auto& iter : ibo)
	{
		Vector3d normal = (vbo[iter[1]] - vbo[iter[0]]).cross(vbo[iter[2]] - vbo[iter[0]]).normalized();
		bool isFirst = true, isLeft /*= false*/, temp /*= false*/;
		for (size_t i = 0; i < vbo.size(); ++i)
		{
			if (i == iter[0] || i == iter[1] || i == iter[2] || fabs(normal.dot(vbo[i] - vbo[iter[0]])) < FLT_EPSILON) // self or coplanar
				continue;
			temp = normal.dot(vbo[i] - vbo[iter[0]]) < 0.0;
			if (isFirst)
			{
				isLeft = temp;
				isFirst = false;
			}
			else
			{
				if (temp != isLeft)
					return false;
			}
		}
	}
	return true;
}

bool psykronix::isMeshConvexPolyhedron(const ModelMesh& mesh)
{
	return isMeshConvexPolyhedron(mesh.vbo_, mesh.ibo_);
}

RayOnTrigon _relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon) // axisZ direction
{
	//if (!isPointRayAcrossTriangle(point, trigon))
	//	return PointOnTrigon::CROSS_OUTER;
	// must intersect
	if (point.x() == trigon[0].x() && point.y() == trigon[0].y()) // include ray and edge collinear
		return RayOnTrigon::CROSS_VERTEX_0;
	else if (point.x() == trigon[1].x() && point.y() == trigon[1].y())
		return RayOnTrigon::CROSS_VERTEX_1;
	else if (point.x() == trigon[2].x() && point.y() == trigon[2].y())
		return RayOnTrigon::CROSS_VERTEX_2;
	else if ((point - trigon[1]).cross(point - trigon[0]).z() == 0.0) // point on edge's projection
		return RayOnTrigon::CROSS_EDGE_01;
	else if ((point - trigon[2]).cross(point - trigon[1]).z() == 0.0)
		return RayOnTrigon::CROSS_EDGE_12;
	else if ((point - trigon[0]).cross(point - trigon[2]).z() == 0.0)
		return RayOnTrigon::CROSS_EDGE_20;
	return RayOnTrigon::CROSS_INNER;
}

bool psykronix::isPointInsidePolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
#ifdef STATISTIC_DATA_COUNT
	count_mesh_inside_mesh++;
#endif  
	auto isLeftAll = [&](const std::array<int, 3>& trigon)->bool // buildin lambda function
	{
		Vector3d normal = (vbo[trigon[1]] - vbo[trigon[0]]).cross(vbo[trigon[2]] - vbo[trigon[0]]).normalized(); // for precision
		bool isFirst = true, isLeft /*= false*/, temp /*= false*/;
		for (size_t i = 0; i < vbo.size(); ++i)
		{
			if (i == trigon[0] || i == trigon[1] || i == trigon[2] || fabs(normal.dot(vbo[i] - vbo[trigon[0]])) < FLT_EPSILON) // self and coplanar
				continue;
			temp = normal.dot(vbo[i] - vbo[trigon[0]]) < 0.0;
			if (isFirst)
			{
				isLeft = temp;
				isFirst = false;
			}
			else
			{
				if (temp != isLeft)
					return false;
			}
		}
		return true;
	};
	size_t count = 0;
	set<int> vertexSet;
	set<array<int, 2>> edgeSet;
	bool isConvex = isMeshConvexPolyhedron(vbo, ibo);
	for (const auto& iter : ibo)
	{
		Triangle trigon = { { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] } };
		if (isPointRayAcrossTriangleSAT(point, trigon))
		{
			RayOnTrigon relation = _relationOfPointAndTriangle(point, trigon);
			if (RayOnTrigon::CROSS_INNER == relation)
				count++;
			else if (RayOnTrigon::CROSS_VERTEX_0 == relation && vertexSet.find(iter[0]) == vertexSet.end())
			{
				if (isConvex || isLeftAll(iter)) // isConvex or Concave outer face
				{
					count++;
					vertexSet.insert(iter[0]);
				}
			}
			else if (RayOnTrigon::CROSS_VERTEX_1 == relation && vertexSet.find(iter[1]) == vertexSet.end())
			{
				if (isConvex || isLeftAll(iter)) // isConvex or Concave outer face
				{
					count++;
					vertexSet.insert(iter[1]);
				}
			}
			else if (RayOnTrigon::CROSS_VERTEX_2 == relation && vertexSet.find(iter[2]) == vertexSet.end())
			{
				if (isConvex || isLeftAll(iter)) // isConvex or Concave outer face
				{
					count++;
					vertexSet.insert(iter[2]);
				}
			}
			else if (RayOnTrigon::CROSS_EDGE_01 == relation &&
				edgeSet.find({ iter[0], iter[1] }) == edgeSet.end() && edgeSet.find({ iter[1], iter[0] }) == edgeSet.end())
			{
				count++;
				edgeSet.insert({ iter[0], iter[1] });
			}
			else if (RayOnTrigon::CROSS_EDGE_12 == relation &&
				edgeSet.find({ iter[1], iter[2] }) == edgeSet.end() && edgeSet.find({ iter[2], iter[1] }) == edgeSet.end())
			{
				count++;
				edgeSet.insert({ iter[1], iter[2] });
			}
			else if (RayOnTrigon::CROSS_EDGE_20 == relation &&
				edgeSet.find({ iter[2], iter[0] }) == edgeSet.end() && edgeSet.find({ iter[0], iter[2] }) == edgeSet.end())
			{
				count++;
				edgeSet.insert({ iter[2], iter[0] });
			}
		}
	}
	return count % 2 != 0;
}

bool psykronix::isPointContainedInPolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	//one mesh all-in other mesh, the bounding-box must all-in other mesh
	int count = 0;
	for (const auto& iter : ibo)
	{
		std::array<Eigen::Vector3d, 3> trigon = { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] };
		if (isPointOnTriangleSurface(point, trigon))
		{
			count++; // ray across is true
		}
		else
		{
			Vector3d rayX = Vector3d(1.0, 0.0, 0.0);
			Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[0]);
			double dotPro = normal.dot(rayX); //ray.direction
			if (dotPro == 0.0) // ray direction is parallel
				continue;
			double t = normal.dot(trigon[0] - point) / dotPro; //ray.origin
			if (t > 0.0)
			{
				Vector3d inter = point + rayX * t;
				if (isPointInTriangle(inter, trigon))
					count++; // ray across is true
			}
		}
	}
	return count % 2 != 0;
}

bool psykronix::isPointInsidePolyhedron(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	Eigen::Vector3d pointR = mesh.pose_.inverse() * point;
	return isPointInsidePolyhedron(pointR, mesh.vbo_, mesh.ibo_);
}

bool _isNormalVectorOutwardsConvex(const std::array<int, 3>& face, const Eigen::Vector3d& normal,
	const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	// must be Convex Polyhedron, default create normal
	//Vector3d normal = (vbo[face[1]] - vbo[face[0]]).cross(vbo[face[2]] - vbo[face[0]]).normalized();
	for (size_t i = 0; i < vbo.size(); ++i)
	{
		if (i == face[0] || i == face[1] || i == face[2] || fabs(normal.dot(vbo[i] - vbo[face[0]])) < FLT_EPSILON) // self and coplanar
			continue;
		return normal.dot(vbo[i] - vbo[face[0]]) < 0.0;
	}
}

//using method SAT
Eigen::Vector3d _getPenetrationDepthOfTwoConvex(const std::vector<Eigen::Vector3d>& vboA, const std::vector<std::array<int, 3>>& iboA,
	const std::vector<Eigen::Vector3d>& vboB, const std::vector<std::array<int, 3>>& iboB)
{
	double dminA = DBL_MAX, dminB = DBL_MAX;
	Eigen::Vector3d direction;
	//std::tuple<std::array<int, 3>,bool> faceRecord; // bool fixed FaceA
	// the minimum distance mush on the direction of face normal
	bool isFixedFaceA = true;
	for (const auto& iterA : iboA) // iterate every face
	{
		double minA = DBL_MAX, minB = DBL_MAX, maxA = -DBL_MAX, maxB = -DBL_MAX; //refresh min and max
		Vector3d normal = (vboA[iterA[1]] - vboA[iterA[0]]).cross(vboA[iterA[2]] - vboA[iterA[0]]).normalized();
		//bool isOut = _isNormalVectorOutwardsConvex(iterA, normal, vboA, iboA);
		for (const auto& vertexA : vboA)
		{
			double projection = normal.dot(vertexA);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboB)
		{
			double projection = normal.dot(vertexB);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminA)//refresh 
		{
			dminA = std::min(maxA - minB, maxB - minA);
			direction = normal;
			if (maxA - minB > maxB - minA)
				isFixedFaceA = false;
		}
	}
	for (const auto& iterB : iboB) // iterate every face
	{
		double minA = DBL_MAX, minB = DBL_MAX, maxA = -DBL_MAX, maxB = -DBL_MAX;
		Vector3d normal = (vboB[iterB[1]] - vboB[iterB[0]]).cross(vboB[iterB[2]] - vboB[iterB[0]]).normalized();
		for (const auto& vertexA : vboA)
		{
			double projection = normal.dot(vertexA);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboB)
		{
			double projection = normal.dot(vertexB);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminB)//refresh 
		{
			dminB = std::min(maxA - minB, maxB - minA);
			direction = normal;
			if (maxA - minB < maxB - minA)
				isFixedFaceA = false;
		}
	}
	if (!isFixedFaceA)
		isFixedFaceA = -isFixedFaceA;
	return std::min(dminA, dminB) * direction;
}

// fix mehsA, move distance of meshB
Eigen::Vector3d psykronix::getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB)
{
	//must intersect
	bool isConvexA = isMeshConvexPolyhedron(meshA.vbo_, meshA.ibo_); // matrix not work on convex
	bool isConvexB = isMeshConvexPolyhedron(meshB.vbo_, meshB.ibo_);
	double dminA = DBL_MAX, dminB = DBL_MAX;
	Eigen::Vector3d direction;
	if (isConvexA && isConvexB)
	{

	}


	return direction;
}

//---------------------------------------------------------------------------
// mesh
//---------------------------------------------------------------------------

// get the index param of mesh's ibo, return index-vector(ibo) of two mesh
std::array<std::vector<size_t>, 2> psykronix::getReducedIntersectTrianglesOfMesh(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, const Eigen::Affine3d& matrix)
{
	//Eigen::AlignedBox3d boxMag(box.min() - 0.5 * Vector3d(tolerance, tolerance, tolerance), box.max() + 0.5 * Vector3d(tolerance, tolerance, tolerance));
	Eigen::AlignedBox3d box = mesh_a.bounding_.intersection(mesh_b.bounding_);
	Vector3d toleSize = Vector3d(tolerance, tolerance, tolerance);
	Eigen::AlignedBox3d boxMag(box.min() - toleSize, box.max() + toleSize);
	std::vector<size_t> triA_Index; // using index of mesh IBO
	Eigen::AlignedBox3d triA_Box; // iterate to lessen box
	for (size_t i = 0; i < mesh_a.ibo_.size(); ++i)
	{
		std::array<Eigen::Vector3d, 3> triIter = { // input matrix to avoid repeat calculate
				matrix * mesh_a.vbo_[mesh_a.ibo_[i][0]],
				matrix * mesh_a.vbo_[mesh_a.ibo_[i][1]],
				matrix * mesh_a.vbo_[mesh_a.ibo_[i][2]] };
		//enlarge box while has tolerance
		if (isTriangleAndBoundingBoxIntersectSAT(triIter, boxMag))
		{
			triA_Index.push_back(i);
			triA_Box.extend(triIter[0]);
			triA_Box.extend(triIter[1]);
			triA_Box.extend(triIter[2]);
		}
	}
	if (triA_Index.empty())
		return {};
	std::vector<size_t> triB_Index;
	Eigen::AlignedBox3d triB_Box;
	for (size_t j = 0; j < mesh_b.ibo_.size(); ++j)
	{
		std::array<Eigen::Vector3d, 3> triIter = {
				mesh_b.vbo_[mesh_b.ibo_[j][0]],
				mesh_b.vbo_[mesh_b.ibo_[j][1]],
				mesh_b.vbo_[mesh_b.ibo_[j][2]] };
		if (isTriangleAndBoundingBoxIntersectSAT(triIter, boxMag))
		{
			triB_Index.push_back(j);
			triB_Box.extend(triIter[0]);
			triB_Box.extend(triIter[1]);
			triB_Box.extend(triIter[2]);
		}
	}
	if (triB_Index.empty())
		return {};
	// second bounding-box intersect
	triA_Box.min() = triA_Box.min() - toleSize;
	triA_Box.max() = triA_Box.max() + toleSize;
	triB_Box.min() = triB_Box.min() - toleSize;
	triB_Box.max() = triB_Box.max() + toleSize;
	if (!triA_Box.intersects(triB_Box))
		return {};
#ifdef STATISTIC_DATA_COUNT
	count_triA_inter += triA_Index.size();
	count_triB_inter += triB_Index.size();
#endif
	return { triA_Index, triB_Index };
}

bool psykronix::isTwoMeshsIntersectHard(const ModelMesh& mesh_a, const ModelMesh& mesh_b)
{
	Eigen::Affine3d relative_matrix = Eigen::Affine3d::Identity();
#ifdef USING_RELATIVE_MATRIX_RECTIFY
	// get relative matrix
	relative_matrix = mesh_b.pose_.inverse() * mesh_a.pose_; // model_a * a / b
	for (size_t i = 0; i < 3; i++)// machine error process
	{
		for (size_t j = 0; j < 3; j++)
		{
			relative_matrix(i, j) = abs(relative_matrix(i, j)) < 1e-14 ? 0.0 : relative_matrix(i, j);
			relative_matrix(i, j) = abs(relative_matrix(i, j) - 1.0) < 1e-14 ? 1.0 : relative_matrix(i, j);
		}
		relative_matrix(i, 3) = std::round(relative_matrix(i, 3) * 1e9) / 1e9;
	}
#endif 
	// get the index param of mesh's ibo
	std::array<std::vector<size_t>, 2> indexAB = getReducedIntersectTrianglesOfMesh(mesh_a, mesh_b, 0.0, relative_matrix);
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		for (const auto& iA : indexAB[0])
		{
			std::array<Eigen::Vector3d, 3> triA = {
					relative_matrix * mesh_a.vbo_[mesh_a.ibo_[iA][0]],
					relative_matrix * mesh_a.vbo_[mesh_a.ibo_[iA][1]],
					relative_matrix * mesh_a.vbo_[mesh_a.ibo_[iA][2]] };
			for (const auto& iB : indexAB[1])
			{
				std::array<Eigen::Vector3d, 3> triB = {
						mesh_b.vbo_[mesh_b.ibo_[iB][0]],
						mesh_b.vbo_[mesh_b.ibo_[iB][1]],
						mesh_b.vbo_[mesh_b.ibo_[iB][2]] };
				if (!isTwoTrianglesBoundingBoxIntersect(triA, triB)) // second pre-judge
				{
#ifdef STATISTIC_DATA_COUNT
					count_tri_box_exclude_pre++;
#endif  
					continue;
				}
				if (isTwoTrianglesIntersectSAT(triA, triB))
				{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
					triRecordHard.push_back({ triA, triB });
					interTriInfoList.push_back({ { triA, triB }, {}, 0.0 });
#endif
					return true;
				}
			}
		}
	}
	//judge whether mesh entirely inside mesh
	if (mesh_a.vbo_.empty() || mesh_b.vbo_.empty())
	{
#ifdef STATISTIC_DATA_COUNT
		count_err_empty_mesh++;
#endif  
		return false;
	}
	if (mesh_a.bounding_.contains(mesh_b.bounding_)) //boundingbox is world coord
	{
		// precondition: boundingbox inside, polyface not intersect, meshin not empty
		if (isPointInsidePolyhedron(mesh_a.pose_.inverse() * mesh_b.pose_ * mesh_b.vbo_[0], mesh_a.vbo_, mesh_a.ibo_)) //not empty
			return true;
	}
	else if (mesh_b.bounding_.contains(mesh_a.bounding_))
	{
		if (isPointInsidePolyhedron(mesh_b.pose_.inverse() * mesh_a.pose_ * mesh_a.vbo_[0], mesh_b.vbo_, mesh_b.ibo_)) //not empty
			return true;
	}
	return false;
}

double psykronix::getTwoMeshsDistanceSoft(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, Eigen::Vector3d& P, Eigen::Vector3d& Q)
{
#ifdef STATISTIC_DATA_RECORD
	std::array<std::array<Eigen::Vector3d, 3>, 2> triDistPair;
#endif    
	Eigen::Affine3d relative_matrix = Eigen::Affine3d::Identity();
#ifdef USING_RELATIVE_MATRIX_RECTIFY
	// get relative matrix
	relative_matrix = mesh_b.pose_.inverse() * mesh_a.pose_; // model_a * a / b
	for (size_t i = 0; i < 3; i++)// machine error process
	{
		for (size_t j = 0; j < 3; j++)
		{
			relative_matrix(i, j) = abs(relative_matrix(i, j)) < 1e-14 ? 0.0 : relative_matrix(i, j);
			relative_matrix(i, j) = abs(relative_matrix(i, j) - 1.0) < 1e-14 ? 1.0 : relative_matrix(i, j);
		}
		relative_matrix(i, 3) = std::round(relative_matrix(i, 3) * 1e9) / 1e9;
	}
#endif 
	// distance > tolerance, return double-max, to decrease calculate
	double d = DBL_MAX; // the res
	std::array<vector<size_t>, 2> indexAB = getReducedIntersectTrianglesOfMesh(mesh_a, mesh_b, tolerance, relative_matrix);
	if (indexAB[0].empty() || indexAB[1].empty())
		return d;
	for (const auto& iA : indexAB[0])
	{
		std::array<Eigen::Vector3d, 3> triA = {
				relative_matrix * mesh_a.vbo_[mesh_a.ibo_[iA][0]],
				relative_matrix * mesh_a.vbo_[mesh_a.ibo_[iA][1]],
				relative_matrix * mesh_a.vbo_[mesh_a.ibo_[iA][2]] };
		for (const auto& iB : indexAB[1])
		{
			std::array<Eigen::Vector3d, 3> triB = {
					mesh_b.vbo_[mesh_b.ibo_[iB][0]],
					mesh_b.vbo_[mesh_b.ibo_[iB][1]],
					mesh_b.vbo_[mesh_b.ibo_[iB][2]] };
			if (!isTwoTrianglesBoundingBoxIntersect(triA, triB, tolerance))
			{
#ifdef STATISTIC_DATA_COUNT
				count_tri_box_exclude_pre++;
#endif                    
				continue;
			}
			double temp = getTrianglesDistance(P, Q, triA, triB);// two trigons distance calculate
			if (temp <= tolerance && temp < d) // update d
			{
				d = temp;
				P = mesh_b.pose_ * P;
				Q = mesh_b.pose_ * Q;
#ifdef STATISTIC_DATA_RECORD
				triDistPair = { triA, triB }; //record last tri-pair
#endif    
			}
		}
	}
#ifdef STATISTIC_DATA_RECORD
	if (d <= tolerance)
		interTriInfoList.push_back({ triDistPair ,{}, d });
#endif   
	return d;
}

// only for convex polytope 
bool isTwoMeshsIntersectGJK(const ModelMesh& meshA, const ModelMesh& meshB)
{

	return false;
}

// only for convex polytope
double getTwoMeshsPenetrationDepthEPA(const ModelMesh& meshA, const ModelMesh& meshB)
{

	return 0.0;
}

//---------------------------------------------------------------------------
//  penetration depth
//---------------------------------------------------------------------------

