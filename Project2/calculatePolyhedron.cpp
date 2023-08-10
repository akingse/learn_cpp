#include "pch.h"
#include "calculatePolyhedron.h"
using namespace std;
using namespace Eigen;
using namespace psykronix;
//static constexpr double eps = FLT_EPSILON; //1e-7
//static constexpr double _eps = -FLT_EPSILON;
static constexpr double eps_d = 10 * DBL_EPSILON; // double

#undef max 
#undef min
#define USING_METHOD_SAT
#define USING_RELATIVE_MATRIX_RECTIFY

#ifdef STATISTIC_DATA_COUNT
extern std::atomic<size_t> count_mesh_inside_mesh;
extern std::atomic<size_t> count_reduced_exclude_pre, count_triA_inter, count_triB_inter,
count_err_empty_mesh, count_tri_box_exclude_pre, count_trigon_coplanar;
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

bool psykronix::isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	Vector3d normal;
	for (const auto& iter : ibo)
	{
		normal = (vbo[iter[1]] - vbo[iter[0]]).cross(vbo[iter[2]] - vbo[iter[0]]);// .normalized();
		bool isFirst = true, isLeft /*= false*/, temp /*= false*/;
		for (size_t i = 0; i < vbo.size(); ++i)
		{
			if (i == iter[0] || i == iter[1] || i == iter[2] || fabs(normal.dot((vbo[i] - vbo[iter[0]]))) < eps) // self or coplanar
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

// exclude point on face, ray is rotz
RelationOfPointAndMesh psykronix::isPointInsidePolyhedronRZ(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	//one mesh inside other mesh, the bounding-box must inside other mesh
	Vector3d rayX = Vector3d(1.0, 0.0, 0.0);
	Vector3d normal, local;
	int count = 0;
	double angle = 0.0, deno, k;
	auto _isRayAndTriangleIntersectParallel = [&](std::array<Eigen::Vector3d, 3 >& trigon)->bool
	{
		//if (fabs((point - trigon[0]).dot(normal)) > eps) // not coplanar
		if ((point - trigon[0]).dot(normal) != 0.0) // not coplanar
			return false;
		// negetive direction ray cause cross product result opposite
		return ((trigon[0] - point).cross(rayX).dot(rayX.cross(trigon[1] - point)) >= 0.0 && (trigon[0] - point).cross(rayX).dot((trigon[0] - point).cross(trigon[1] - point)) >= 0.0) ||
			((trigon[1] - point).cross(rayX).dot(rayX.cross(trigon[2] - point)) >= 0.0 && (trigon[1] - point).cross(rayX).dot((trigon[1] - point).cross(trigon[2] - point)) >= 0.0) ||
			((trigon[2] - point).cross(rayX).dot(rayX.cross(trigon[0] - point)) >= 0.0 && (trigon[2] - point).cross(rayX).dot((trigon[2] - point).cross(trigon[0] - point)) >= 0.0);
	};
	auto _correctRayX = [&]()
	{
		angle += 0.1; // 0.1(rad)~5.73(deg)
		Affine3d rotation = Affine3d::Identity();
		rotation.rotate(AngleAxisd(angle, Vector3d::UnitZ()));
		rayX = rotation * rayX;
	};
	while (true)
	{
		bool isNew = false;//new rayX
		for (const auto& iter : ibo) // iterate every trigon
		{
			std::array<Eigen::Vector3d, 3> trigon = { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] };
			if (isPointOnTriangleSurface(point, trigon))
				return RelationOfPointAndMesh::SURFACE; // ray across is false
			normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[0]);
			deno = rayX.dot(normal); //ray.direction
			if (deno == 0.0)//(fabs(deno) < eps) // ray direction is parallel, redo circulation
			{
				if (_isRayAndTriangleIntersectParallel(trigon)) //coplanar
				{
					_correctRayX();
					isNew = true;
					break;
				}
				continue;
			}
			k = (trigon[0] - point).dot(normal) / deno;
			if (k < 0.0) // only positive direction
				continue;
			local = point + k * rayX;
			if (!isPointInTriangle(local, trigon))
				continue;
			if ((local - trigon[0]).isZero() ||
				(local - trigon[1]).isZero() ||
				(local - trigon[2]).isZero() ||
				(local - trigon[0]).cross(local - trigon[1]).isZero() ||
				(local - trigon[1]).cross(local - trigon[2]).isZero() ||
				(local - trigon[2]).cross(local - trigon[0]).isZero()) //singularity
			{
				_correctRayX();
				isNew = true;
				break;
			}
			count++; // ray across is true
		}
		if (!isNew)
			break;//end while
	}
	return (count % 2 == 1) ? RelationOfPointAndMesh::INNER : RelationOfPointAndMesh::OUTER;
}

RelationOfPointAndMesh psykronix::isPointInsidePolyhedronRZ(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	Eigen::Vector3d pointR = mesh.pose_.inverse() * point;
	return isPointInsidePolyhedronRZ(pointR, mesh.vbo_, mesh.ibo_);
}

// include point on face, 
//bool psykronix::isPointInsidePolyhedronAZ(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
bool psykronix::isPointInsidePolyhedronAZ(const Eigen::Vector3d& _point, const ModelMesh& mesh)
{
#ifdef STATISTIC_DATA_COUNT
	count_mesh_inside_mesh++;
#endif  
	Eigen::Vector3d point = mesh.pose_.inverse() * _point;
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	auto _relationOfPointAndTriangle = [&](/*const Vector3d& point,*/ const std::array<Vector3d, 3>& trigon)->RelationOfRayAndTrigon // axisZ direction
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
	};
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
	//bool isConvex = isMeshConvexPolyhedron(vbo, ibo);
	for (const auto& iter : ibo)
	{
		Triangle trigon = { { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] } };
		if (!isPointRayAcrossTriangleSAT(point, trigon))
			continue;
		RelationOfRayAndTrigon relation = _relationOfPointAndTriangle(trigon);
		if (RelationOfRayAndTrigon::CROSS_INNER == relation)
			count++;
		else if (RelationOfRayAndTrigon::CROSS_VERTEX_0 == relation && vertexSet.find(iter[0]) == vertexSet.end())
		{
			if (mesh.convex_ || isLeftAll(iter)) // isConvex or Concave outer face
			{
				count++;
				vertexSet.insert(iter[0]);
			}
		}
		else if (RelationOfRayAndTrigon::CROSS_VERTEX_1 == relation && vertexSet.find(iter[1]) == vertexSet.end())
		{
			if (mesh.convex_ || isLeftAll(iter)) // isConvex or Concave outer face
			{
				count++;
				vertexSet.insert(iter[1]);
			}
		}
		else if (RelationOfRayAndTrigon::CROSS_VERTEX_2 == relation && vertexSet.find(iter[2]) == vertexSet.end())
		{
			if (mesh.convex_ || isLeftAll(iter)) // isConvex or Concave outer face
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
	return count % 2 == 1;
}

//bool psykronix::isPointInsidePolyhedronAZ(const Eigen::Vector3d& point, const ModelMesh& mesh)
//{
//	Eigen::Vector3d pointR = mesh.pose_.inverse() * point;
//	return isPointInsidePolyhedronAZ(pointR, mesh.vbo_, mesh.ibo_);
//}

bool psykronix::isPointInsidePolyhedronCEIL(const Eigen::Vector3d& _point, const ModelMesh& mesh) // more judge
{
	Eigen::Vector3d point = mesh.pose_.inverse() * _point;
	//auto _isPointInTriangle2D = [&](const std::array<Vector3d, 3>& trigon)->bool
	//{
	//	if (point.x() > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
	//		point.x() < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
	//		point.y() > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) || 
	//		point.y() < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]))
	//		return false;
	//	double az = (trigon[1][0] - trigon[0][0]) * (trigon[2][1] - trigon[0][1]) - (trigon[2][0] - trigon[0][0]) * (trigon[1][1] - trigon[0][1]);
	//	return //when vertical, az==0, always true
	//		0.0 <= az * ((trigon[1][0] - trigon[0][0]) * (point[1] - trigon[0][1]) - (point[0] - trigon[0][0]) * (trigon[1][1] - trigon[0][1])) &&
	//		0.0 <= az * ((trigon[2][0] - trigon[1][0]) * (point[1] - trigon[1][1]) - (point[0] - trigon[1][0]) * (trigon[2][1] - trigon[1][1])) &&
	//		0.0 <= az * ((trigon[0][0] - trigon[2][0]) * (point[1] - trigon[2][1]) - (point[0] - trigon[2][0]) * (trigon[0][1] - trigon[2][1]));
	//};
	auto _isPointInTriangle2D = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)->bool
	{
		if (point.x() > std::max(std::max(p0[0], p1[0]), p2[0]) ||
			point.x() < std::min(std::min(p0[0], p1[0]), p2[0]) ||
			point.y() > std::max(std::max(p0[1], p1[1]), p2[1]) ||
			point.y() < std::min(std::min(p0[1], p1[1]), p2[1]) ||
			point.z() > std::max(std::max(p0[2], p1[2]), p2[2])) //include z
			return false;
		double az = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
		return //when vertical, az==0, always true
			0.0 <= az * ((p1[0] - p0[0]) * (point[1] - p0[1]) - (point[0] - p0[0]) * (p1[1] - p0[1])) &&
			0.0 <= az * ((p2[0] - p1[0]) * (point[1] - p1[1]) - (point[0] - p1[0]) * (p2[1] - p1[1])) &&
			0.0 <= az * ((p0[0] - p2[0]) * (point[1] - p2[1]) - (point[0] - p2[0]) * (p0[1] - p2[1]));
	};
	if (!mesh.bounding_.contains(point))
		return false;
	// using axisZ
	Eigen::Vector3d normal;
	size_t count = 0;
	double projection;
	for (const auto& face : mesh.ibo_)
	{
		//Triangle trigon = { { mesh.vbo_[face[0]] ,mesh.vbo_[face[1]] ,mesh.vbo_[face[2]] } };
		const Vector3d& p0 = mesh.vbo_[face[0]];
		const Vector3d& p1 = mesh.vbo_[face[1]];
		const Vector3d& p2 = mesh.vbo_[face[2]];
		if (!_isPointInTriangle2D(p0, p1, p2))
			continue;
		normal = (p1 - p0).cross(p2 - p0);
		if (normal.z() == 0.0) // point-ray through trigon, return true
		{
			// point in segment collinear judge 
			if ((point.x() - p0.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (point.y() - p0.y()) == 0.0)
				return true;
			continue; //judge point under segment
		}
		projection = normal.dot(point - p0);
		if (projection == 0.0) //isPointOnTriangleSurface
			return true; 
		if (projection * normal.z() < 0.0) // point under plane
			count++;
	}
	return count % 2 == 1;
}

//--------------------------------------------------------------------------------------------------------
// mesh
//--------------------------------------------------------------------------------------------------------


Eigen::Affine3d _getRelativeMatrixRectify(const Eigen::Affine3d& matA, const Eigen::Affine3d& matB)
{
	//Eigen::Affine3d::Identity()
	Eigen::Affine3d relative_matrix = matB.inverse() * matA; // model_a * a / b
	for (size_t i = 0; i < 3; i++)// machine error process
	{
		for (size_t j = 0; j < 3; j++)
		{
			relative_matrix(i, j) = abs(relative_matrix(i, j)) < 1e-14 ? 0.0 : relative_matrix(i, j);
			relative_matrix(i, j) = abs(relative_matrix(i, j) - 1.0) < 1e-14 ? 1.0 : relative_matrix(i, j);
		}
		relative_matrix(i, 3) = std::round(relative_matrix(i, 3) * 1e9) / 1e9;
	}
	return relative_matrix;
}

// get the index param of mesh's ibo, return index-vector(ibo) of two mesh
std::array<std::vector<size_t>, 2> _getReducedIntersectTrianglesOfMesh(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance)
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
				mesh_a.pose_ * mesh_a.vbo_[mesh_a.ibo_[i][0]],
				mesh_a.pose_ * mesh_a.vbo_[mesh_a.ibo_[i][1]],
				mesh_a.pose_ * mesh_a.vbo_[mesh_a.ibo_[i][2]] };
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
				mesh_b.pose_* mesh_b.vbo_[mesh_b.ibo_[j][0]],
				mesh_b.pose_* mesh_b.vbo_[mesh_b.ibo_[j][1]],
				mesh_b.pose_* mesh_b.vbo_[mesh_b.ibo_[j][2]] };
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

// ClashHard
bool isTwoMeshsIntersectSAT(const ModelMesh& mesh_a, const ModelMesh& mesh_b)
{
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(mesh_a.pose_, mesh_b.pose_);// get relative matrix
	// get the index param of mesh's ibo
	std::array<std::vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(mesh_a, mesh_b, 0.0);
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
		if (isPointInsidePolyhedronAZ(mesh_a.pose_.inverse() * mesh_b.pose_ * mesh_b.vbo_[0], mesh_a))
		{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			triRecordHard.push_back({ gTirNaN, gTirNaN });
			interTriInfoList.push_back({ { gTirNaN, gTirNaN }, {}, 0.0 });
#endif
			return true;
		}
	}
	else if (mesh_b.bounding_.contains(mesh_a.bounding_))
	{
		if (isPointInsidePolyhedronAZ(mesh_b.pose_.inverse() * mesh_a.pose_ * mesh_a.vbo_[0], mesh_b))
		{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			triRecordHard.push_back({ gTirNaN, gTirNaN });
			interTriInfoList.push_back({ { gTirNaN, gTirNaN }, {}, 0.0 });
#endif
			return true;
		}
	}
	return false;
}

RelationOfTwoMesh getTwoMeshsIntersectRelation(const ModelMesh& mesh_a, const ModelMesh& mesh_b)
{
	//auto _isTwoIntersectTrianglesCoplanar = [](const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB)->bool //must intersect
	//{
	//	Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[0]);
	//	Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[0]);
	//	return normalA.cross(normalB).isZero(eps);
	//};
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(mesh_a.pose_, mesh_b.pose_);// get relative matrix
	// get the index param of mesh's ibo
	std::array<std::vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(mesh_a, mesh_b, 0.0);
	bool isContact = false; // vertex or edge or face contact
	bool isIntrusive = false;
	std::map<size_t, bool> vertexAINB, vertexBINA;
	std::set<size_t> faceA, faceB; // intersect and inside // or using vector
	set<size_t> vertexSetA, vertexSetB;
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		//for (const auto& iA : indexAB[0])
		//{
		//	vertexAINB.insert({ mesh_a.ibo_[iA][0], false });
		//	vertexAINB.insert({ mesh_a.ibo_[iA][1], false });
		//	vertexAINB.insert({ mesh_a.ibo_[iA][2], false });
		//}
		//for (const auto& iB : indexAB[1])
		//{
		//	vertexAINB.insert({ mesh_b.ibo_[iB][0], false });
		//	vertexAINB.insert({ mesh_b.ibo_[iB][1], false });
		//	vertexAINB.insert({ mesh_b.ibo_[iB][2], false });
		//}

		for (const auto& iA : indexAB[0])
		{
			if (isPointInsidePolyhedronCEIL(mesh_a.pose_ * mesh_a.vbo_[mesh_a.ibo_[iA][0]], mesh_b) ||
				isPointInsidePolyhedronCEIL(mesh_a.pose_ * mesh_a.vbo_[mesh_a.ibo_[iA][1]], mesh_b) ||
				isPointInsidePolyhedronCEIL(mesh_a.pose_ * mesh_a.vbo_[mesh_a.ibo_[iA][2]], mesh_b))
				faceA.insert(iA);
		}		
		for (const auto& iB : indexAB[1])
		{
			if (isPointInsidePolyhedronCEIL(mesh_b.pose_ * mesh_b.vbo_[mesh_b.ibo_[iB][0]], mesh_a) ||
				isPointInsidePolyhedronCEIL(mesh_b.pose_ * mesh_b.vbo_[mesh_b.ibo_[iB][1]], mesh_a) ||
				isPointInsidePolyhedronCEIL(mesh_b.pose_ * mesh_b.vbo_[mesh_b.ibo_[iB][2]], mesh_a))
				faceB.insert(iB);
		}


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
				if (!isTwoTrianglesIntersectSAT(triA, triB)) //separate
					continue;
				//intersect
				isContact = true;
				if ((triA[1] - triA[0]).cross(triA[2] - triA[0]).cross((triB[1] - triB[0]).cross(triB[2] - triB[0])).isZero(eps)) // intersect-coplanar
				{
#ifdef STATISTIC_DATA_COUNT
					count_trigon_coplanar++;
#endif  
					continue;
				}
				std::array<Eigen::Vector3d, 2> pInter = getTwoTrianglesIntersectPoints(triA, triB);
				if (!(pInter[1] - pInter[0]).isZero(eps)) //((pInter[1].normalized() - pInter[0].normalized())
				{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
					triRecordHard.push_back({ triA, triB });
					interTriInfoList.push_back({ { triA, triB }, {}, 0.0 });
#endif
					//return RelationOfTwoMesh::INTRUSIVE;
					isIntrusive = true;
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
		Eigen::Affine3d mat = mesh_a.pose_.inverse() * mesh_b.pose_;
		for (const auto& vertexB : mesh_b.vbo_)
		{
			RelationOfPointAndMesh relation = isPointInsidePolyhedronRZ(mat * vertexB, mesh_a.vbo_, mesh_a.ibo_);
			if (RelationOfPointAndMesh::INNER == relation)
			{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
				triRecordHard.push_back({ gTirNaN, gTirNaN });
				interTriInfoList.push_back({ { gTirNaN, gTirNaN }, {}, 0.0 });
#endif
				return isContact ? RelationOfTwoMesh::INSEDE_BINA_CONT : RelationOfTwoMesh::INSEDE_BINA;
			}
			else if (RelationOfPointAndMesh::OUTER == relation)
				return isContact ? RelationOfTwoMesh::CONTACT_OUTER : RelationOfTwoMesh::SEPARATE; // concave separate
			// else SURFACE all
		}
		return RelationOfTwoMesh::INSEDE_BINA_FIT;// RelationOfPointAndMesh::SURFACE fit all vertex
	}
	else if (mesh_b.bounding_.contains(mesh_a.bounding_))
	{
		Eigen::Affine3d mat = mesh_b.pose_.inverse() * mesh_a.pose_;
		for (const auto& vertexA : mesh_a.vbo_)
		{
			RelationOfPointAndMesh relation = isPointInsidePolyhedronRZ(mat * vertexA, mesh_b.vbo_, mesh_b.ibo_);
			if (RelationOfPointAndMesh::INNER == relation)
			{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
				triRecordHard.push_back({ gTirNaN, gTirNaN });
				interTriInfoList.push_back({ { gTirNaN, gTirNaN }, {}, 0.0 });
#endif
				return isContact ? RelationOfTwoMesh::INSEDE_AINB_CONT : RelationOfTwoMesh::INSEDE_AINB;
			}
			else if (RelationOfPointAndMesh::OUTER == relation)
				return isContact ? RelationOfTwoMesh::CONTACT_OUTER : RelationOfTwoMesh::SEPARATE; // concave separate
			// else SURFACE all
		}
		return RelationOfTwoMesh::INSEDE_BINA_FIT;// RelationOfPointAndMesh::SURFACE fit all vertex
	}
	return RelationOfTwoMesh::SEPARATE;
}

// ClashSoft // return index of mesh_a and mesh_b ibo
std::tuple<double, std::array<size_t, 2>> getTwoMeshsDistanceSAT(const ModelMesh& mesh_a, const ModelMesh& mesh_b, double tolerance)
{
#ifdef STATISTIC_DATA_RECORD
	std::array<std::array<Eigen::Vector3d, 3>, 2> triDistPair;
#endif    
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(mesh_a.pose_, mesh_b.pose_);// get relative matrix
	// distance > tolerance, return double-max, to decrease calculate
	double d = DBL_MAX; // the res
	std::array<size_t, 2> index;
	std::array<vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(mesh_a, mesh_b, tolerance);
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

//// only for convex polytope 
//bool isTwoMeshsIntersectGJK(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	return false;
//}
//// only for convex polytope
//double getTwoMeshsPenetrationDepthEPA(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	return 0.0;
//}

//---------------------------------------------------------------------------
//  penetration depth
//---------------------------------------------------------------------------

//using method SAT
//Eigen::Vector3d _getPenetrationDepthOfTwoConvex(const ModelMesh& meshA, const ModelMesh& meshB)
Eigen::Vector3d getPenetrationDepthOfTwoConvex(const ModelMesh& meshA, const set<size_t>& faceA, const ModelMesh& meshB, const set<size_t>& faceB)
{
	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
	const std::vector<std::array<int, 3>>& iboA = meshA.ibo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	const std::vector<std::array<int, 3>>& iboB = meshB.ibo_;
	const Eigen::Affine3d& matA = meshA.pose_;
	const Eigen::Affine3d& matB = meshB.pose_;
	double dminA = DBL_MAX, dminB = DBL_MAX;
	Eigen::Vector3d direction, normal;
	// the minimum distance mush on the direction of face normal
	bool isFixedFaceA = true;
	double minA, maxA, minB, maxB, projection;//refresh min and max
	std::array<size_t, 2> indexAB;
	for (const auto& iterA : faceA) // iterate every face
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboA[iboA[iterA][1]] - vboA[iboA[iterA][0]]).cross(vboA[iboA[iterA][2]] - vboA[iboA[iterA][0]]).normalized(); //pose is same
		for (const auto& vertexA : vboA)
		{
			projection = normal.dot(matA * vertexA);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboB)
		{
			projection = normal.dot(matB * vertexB);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminA)//refresh 
		{
			dminA = std::min(maxA - minB, maxB - minA);
			direction = normal;
			indexAB = { iterA ,0 };
			if (maxA - minB > maxB - minA)
				isFixedFaceA = false;
		}
	}
	for (const auto& iterB : faceB) // iterate every face
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboB[iboB[iterB][1]] - vboB[iboB[iterB][0]]).cross(vboB[iboB[iterB][2]] - vboB[iboB[iterB][0]]).normalized(); //pose is same
		for (const auto& vertexA : vboA)
		{
			projection = normal.dot(matA * vertexA);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboB)
		{
			projection = normal.dot(matB * vertexB);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminB)//refresh 
		{
			dminB = std::min(maxA - minB, maxB - minA);
			direction = normal;
			indexAB = { 0 , iterB };
			if (maxA - minB < maxB - minA)
				isFixedFaceA = false;
		}
	}
	if (!isFixedFaceA)
		direction = -direction;
	return std::min(dminA, dminB) * direction;
}

// return the vertex of meshA
std::vector<Eigen::Vector3d> _getInsideVertexSet(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	// the vertex of meshA inside meshB
	std::vector<Eigen::Vector3d> res;
	for (const auto& iter : meshA.vbo_)
	{
		if (RelationOfPointAndMesh::INNER == isPointInsidePolyhedronRZ(relative_matrix * iter, meshB.vbo_, meshB.ibo_))
			res.push_back(meshA.pose_ * iter);
	}
	return res;
}

std::vector<Eigen::Vector3d> _getIntersectVertexSet(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	std::vector<Eigen::Vector3d> res;
	for (size_t i = 0; i < meshA.ibo_.size(); i++)
	{
		std::array<Eigen::Vector3d, 3> triA = {
				relative_matrix * meshA.vbo_[meshA.ibo_[i][0]],
				relative_matrix * meshA.vbo_[meshA.ibo_[i][1]],
				relative_matrix * meshA.vbo_[meshA.ibo_[i][2]] };
		for (size_t j = 0; j < meshA.ibo_.size(); j++)
		{
			std::array<Eigen::Vector3d, 3> triB = {
					meshB.vbo_[meshB.ibo_[j][0]],
					meshB.vbo_[meshB.ibo_[j][1]],
					meshB.vbo_[meshB.ibo_[j][2]] };
			if (!isTwoTrianglesIntersectSAT(triA, triB))
				continue;
			std::array<Eigen::Vector3d, 2> pInter = getTwoTrianglesIntersectPoints(triA, triB);
			res.push_back(meshB.pose_ * pInter[0]);
			res.push_back(meshB.pose_ * pInter[1]);
		}
	}
	return res;
}

//manual body boolean
vector<Polyhedron> _getBooleanIntersectOfTwoMesh(const ModelMesh& meshA, const ModelMesh& meshB)
{
	vector<Polyhedron> interBodys;


	return {};
}


Eigen::Vector3d _getPartialDepthtOfTwoConcave(const ModelMesh& meshA, const ModelMesh& meshB)
{


	return {};
}

// fixed mehsB, move distance of meshA
Eigen::Vector3d psykronix::getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB)
{
	//must intersect
	if (meshA.convex_ && meshB.convex_) // already intrusive
	{
		//return getPenetrationDepthOfTwoConvex(meshA, meshB);
	}
	Eigen::Vector3d direction = gVecNaN;
	// V==null, I!=nul, 

	// V!=null


	return direction;
}

double getMoveDistanceOfAssignedDirection(const ModelMesh& meshA, const ModelMesh& meshB, const Eigen::Vector3d& direction)
{
	return 0;
}

