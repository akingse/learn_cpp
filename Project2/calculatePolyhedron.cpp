#include "pch.h"
#include "calculatePolyhedron.h"
using namespace std;
using namespace Eigen;
using namespace psykronix;
static constexpr double eps = FLT_EPSILON; //1e-7
static constexpr double eps_d = 10 * DBL_EPSILON; // double
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

bool psykronix::isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	for (const auto& iter : ibo)
	{
		Vector3d normal = (vbo[iter[1]] - vbo[iter[0]]).cross(vbo[iter[2]] - vbo[iter[0]]).normalized();
		bool isFirst = true, isLeft /*= false*/, temp /*= false*/;
		for (size_t i = 0; i < vbo.size(); ++i)
		{
			if (i == iter[0] || i == iter[1] || i == iter[2] || fabs(normal.dot((vbo[i] - vbo[iter[0]]).normalized())) < eps) // self or coplanar
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

RelationOfRayAndTrigon _relationOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon) // axisZ direction
{
	//if (!isPointRayAcrossTriangle(point, trigon))
	//	return PointOnTrigon::CROSS_OUTER;
	// must intersect
	if (point.x() == trigon[0].x() && point.y() == trigon[0].y()) // include ray and edge collinear
		return RelationOfRayAndTrigon::CROSS_VERTEX_0;
	else if (point.x() == trigon[1].x() && point.y() == trigon[1].y())
		return RelationOfRayAndTrigon::CROSS_VERTEX_1;
	else if (point.x() == trigon[2].x() && point.y() == trigon[2].y())
		return RelationOfRayAndTrigon::CROSS_VERTEX_2;
	else if ((point - trigon[1]).cross(point - trigon[0]).z() == 0.0) // point on edge's projection
		return RelationOfRayAndTrigon::CROSS_EDGE_01;
	else if ((point - trigon[2]).cross(point - trigon[1]).z() == 0.0)
		return RelationOfRayAndTrigon::CROSS_EDGE_12;
	else if ((point - trigon[0]).cross(point - trigon[2]).z() == 0.0)
		return RelationOfRayAndTrigon::CROSS_EDGE_20;
	return RelationOfRayAndTrigon::CROSS_INNER;
}

// include point on face
RelationOfPointAndMesh psykronix::isPointInsidePolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	//one mesh inside other mesh, the bounding-box must inside other mesh
	Vector3d rayX = Vector3d(1.0, 0.0, 0.0);
	Vector3d normal, local;
	int count = 0;
	double angle = 0.0, deno, k;
	while (true)
	{
		for (const auto& iter : ibo) // iterate every trigon
		{
			std::array<Eigen::Vector3d, 3> trigon = { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] };
			if (isPointOnTriangleSurface(point, trigon))
				return RelationOfPointAndMesh::SURFACE;// false; // ray across is false
			normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[0]);
			deno = rayX.dot(normal); //ray.direction
			if (fabs(deno) < eps) // ray direction is parallel
				continue; // or redo circulation
			k = (trigon[0] - point).dot(normal) / deno;
			if (0.0 < k) // positive direction
			{
				local = point + k * rayX;
				if (!isPointInTriangle(local, trigon))
					continue;
				if ((local - trigon[0]).isZero() ||
					(local - trigon[1]).isZero() ||
					(local - trigon[2]).isZero() ||
					(local - trigon[0]).cross(local - trigon[1]).isZero() ||
					(local - trigon[1]).cross(local - trigon[2]).isZero() ||
					(local - trigon[2]).cross(local - trigon[0]).isZero()) //Singularity
				{
					angle += 0.1; // 0.1(rad)~5.73(deg)
					Affine3d rotation = Affine3d::Identity();
					rotation.rotate(AngleAxisd(angle, Vector3d::UnitZ()));
					rayX = rotation * rayX;
					break;
				}
				count++; // ray across is true
			}
		}
		break;//end while
	}
	return (count % 2 != 0) ? RelationOfPointAndMesh::INNER : RelationOfPointAndMesh::OUTER;
}

RelationOfPointAndMesh psykronix::isPointInsidePolyhedron(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	Eigen::Vector3d pointR = mesh.pose_.inverse() * point;
	return isPointInsidePolyhedron(pointR, mesh.vbo_, mesh.ibo_);
}

// exclude point on face
bool psykronix::isPointContainedInPolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
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
			if (i == trigon[0] || i == trigon[1] || i == trigon[2] || fabs(normal.dot((vbo[i] - vbo[trigon[0]]).normalized())) < eps) // self and coplanar
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
			RelationOfRayAndTrigon relation = _relationOfPointAndTriangle(point, trigon);
			if (RelationOfRayAndTrigon::CROSS_INNER == relation)
				count++;
			else if (RelationOfRayAndTrigon::CROSS_VERTEX_0 == relation && vertexSet.find(iter[0]) == vertexSet.end())
			{
				if (isConvex || isLeftAll(iter)) // isConvex or Concave outer face
				{
					count++;
					vertexSet.insert(iter[0]);
				}
			}
			else if (RelationOfRayAndTrigon::CROSS_VERTEX_1 == relation && vertexSet.find(iter[1]) == vertexSet.end())
			{
				if (isConvex || isLeftAll(iter)) // isConvex or Concave outer face
				{
					count++;
					vertexSet.insert(iter[1]);
				}
			}
			else if (RelationOfRayAndTrigon::CROSS_VERTEX_2 == relation && vertexSet.find(iter[2]) == vertexSet.end())
			{
				if (isConvex || isLeftAll(iter)) // isConvex or Concave outer face
				{
					count++;
					vertexSet.insert(iter[2]);
				}
			}
			else if (RelationOfRayAndTrigon::CROSS_EDGE_01 == relation &&
				edgeSet.find({ iter[0], iter[1] }) == edgeSet.end() && edgeSet.find({ iter[1], iter[0] }) == edgeSet.end())
			{
				count++;
				edgeSet.insert({ iter[0], iter[1] });
			}
			else if (RelationOfRayAndTrigon::CROSS_EDGE_12 == relation &&
				edgeSet.find({ iter[1], iter[2] }) == edgeSet.end() && edgeSet.find({ iter[2], iter[1] }) == edgeSet.end())
			{
				count++;
				edgeSet.insert({ iter[1], iter[2] });
			}
			else if (RelationOfRayAndTrigon::CROSS_EDGE_20 == relation &&
				edgeSet.find({ iter[2], iter[0] }) == edgeSet.end() && edgeSet.find({ iter[0], iter[2] }) == edgeSet.end())
			{
				count++;
				edgeSet.insert({ iter[2], iter[0] });
			}
		}
	}
	return count % 2 != 0;
}

bool psykronix::isPointContainedInPolyhedron(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	Eigen::Vector3d pointR = mesh.pose_.inverse() * point;
	return isPointContainedInPolyhedron(pointR, mesh.vbo_, mesh.ibo_);
}

bool _isNormalVectorOutwardsConvex(const std::array<int, 3>& face, const Eigen::Vector3d& normal,
	const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	// must be Convex Polyhedron, default create normal
	//Vector3d normal = (vbo[face[1]] - vbo[face[0]]).cross(vbo[face[2]] - vbo[face[0]]).normalized();
	for (size_t i = 0; i < vbo.size(); ++i)
	{
		if (i == face[0] || i == face[1] || i == face[2] || fabs(normal.dot((vbo[i] - vbo[face[0]]).normalized())) < eps) // self and coplanar
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
std::array<std::vector<size_t>, 2> getReducedIntersectTrianglesOfMesh(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance, const Eigen::Affine3d& matrix)
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

bool isTwoMeshsIntersectClashHard(const ModelMesh& mesh_a, const ModelMesh& mesh_b)
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
		// precondition: boundingbox inside, polyface not intersect, mesh isnot empty
		if (isPointContainedInPolyhedron(mesh_a.pose_.inverse() * mesh_b.pose_ * mesh_b.vbo_[0], mesh_a.vbo_, mesh_a.ibo_))
			return true;
	}
	else if (mesh_b.bounding_.contains(mesh_a.bounding_))
	{
		if (isPointContainedInPolyhedron(mesh_b.pose_.inverse() * mesh_a.pose_ * mesh_a.vbo_[0], mesh_b.vbo_, mesh_b.ibo_))
			return true;
	}
	return false;
}

RelationOfTwoMesh getTwoMeshsIntersectRelation(const ModelMesh& mesh_a, const ModelMesh& mesh_b)
{
	auto _isTwoIntersectTrianglesCoplanar = [](const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB)->bool //must intersect
	{
		Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[0]);
		Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[0]);
		return normalA.cross(normalB).normalized().isZero(eps);
	};
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
	bool isContact = false; // vertex or edge or face contact
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
					isContact = true;
					if (_isTwoIntersectTrianglesCoplanar(triA, triB))
						continue;
					std::array<Eigen::Vector3d, 2> pInter = getTwoTrianglesIntersectPoints(triA, triB);
					if (!(pInter[1] - pInter[0]).isZero(eps)) //((pInter[1].normalized() - pInter[0].normalized())
						return RelationOfTwoMesh::INTRUSIVE;
					//return true;
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
		return RelationOfTwoMesh::SEPARATE;
	}
	//lefted, total inside, inner-contact or outer-contact
	if (mesh_a.bounding_.contains(mesh_b.bounding_)) //boundingbox is world coord
	{
		// precondition: boundingbox inside, polyface not intersect, mesh isnot empty
		for (const auto& vertexB : mesh_b.vbo_)
		{
			RelationOfPointAndMesh relation = isPointInsidePolyhedron(mesh_a.pose_.inverse() * mesh_b.pose_ * vertexB, mesh_a.vbo_, mesh_a.ibo_);
			if (RelationOfPointAndMesh::INNER == relation)
				return isContact ? RelationOfTwoMesh::INSEDE_BINA_CONT : RelationOfTwoMesh::INSEDE_BINA;
			else if (RelationOfPointAndMesh::OUTER == relation)
				return isContact ? RelationOfTwoMesh::CONTACT_OUTER : RelationOfTwoMesh::SEPARATE; // concave separate
		}
		return RelationOfTwoMesh::INSEDE_BINA_FIT;// RelationOfPointAndMesh::SURFACE fit all vertex
	}
	else if (mesh_b.bounding_.contains(mesh_a.bounding_))
	{
		for (const auto& vertexA : mesh_a.vbo_)
		{
			RelationOfPointAndMesh relation = isPointInsidePolyhedron(mesh_b.pose_.inverse() * mesh_a.pose_ * mesh_a.vbo_[0], mesh_b.vbo_, mesh_b.ibo_);
			if (RelationOfPointAndMesh::INNER == relation)
				return isContact ? RelationOfTwoMesh::INSEDE_AINB_CONT : RelationOfTwoMesh::INSEDE_AINB;
			else if (RelationOfPointAndMesh::OUTER == relation)
				return isContact ? RelationOfTwoMesh::CONTACT_OUTER : RelationOfTwoMesh::SEPARATE; // concave separate
		}
		return RelationOfTwoMesh::INSEDE_BINA_FIT;// RelationOfPointAndMesh::SURFACE fit all vertex
	}
	return RelationOfTwoMesh::SEPARATE;
}

// return index of mesh_a and mesh_b ibo
std::tuple<double, std::array<size_t, 2>> getTwoMeshsDistanceClashSoft(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance)
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
	std::array<size_t, 2> index;
	std::array<vector<size_t>, 2> indexAB = getReducedIntersectTrianglesOfMesh(mesh_a, mesh_b, tolerance, relative_matrix);
	if (indexAB[0].empty() || indexAB[1].empty())
		return { d, {} };
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
			double temp = getTrianglesDistanceSAT(triA, triB);// two trigons distance calculate
			if (temp <= tolerance && temp < d) // update d
			{
				d = temp;
				index = { iA, iB };
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
	return { d, index };
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

// return the vertex of meshB
std::vector<Eigen::Vector3d> _getInsideVertexSet(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = Eigen::Affine3d::Identity();
#ifdef USING_RELATIVE_MATRIX_RECTIFY
	// get relative matrix
	relative_matrix = meshB.pose_.inverse() * meshA.pose_; // model_a * a / b
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
	// the vertex of meshB inside meshA
	std::vector<Eigen::Vector3d> res;
	for (const auto& iter : meshB.vbo_)
	{
		if (RelationOfPointAndMesh::INNER == isPointInsidePolyhedron(iter, meshA))
			res.push_back(iter);
	}
	return res;
}

std::vector<Eigen::Vector3d> _getIntersectVertexSet(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = Eigen::Affine3d::Identity();
#ifdef USING_RELATIVE_MATRIX_RECTIFY
	// get relative matrix
	relative_matrix = meshB.pose_.inverse() * meshA.pose_; // model_a * a / b
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



}
