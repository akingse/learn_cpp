#include "pch.h"
#include "calculatePolyhedron.h"
using namespace std;
using namespace Eigen;
using namespace psykronix;
//static constexpr double eps_d = 10 * DBL_EPSILON; // double
static const Triangle gTriXOY = { Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0) };
static const Triangle gTriXOZ = { Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,0,1) };
static const Triangle gTriYOZ = { Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1) };

#undef max 
#undef min
#define USING_METHOD_SAT
#define USING_RELATIVE_MATRIX_RECTIFY

//replace .x() => [0]
//replace .y() => [1]
//replace .z() => [2]

#ifdef STATISTIC_DATA_COUNT
extern std::atomic<size_t> count_mesh_inside_mesh, count_getTrisDistance;
extern std::atomic<size_t> count_reduced_exclude_pre, count_triA_inter, count_triB_inter,
count_err_empty_mesh, count_tri_box_exclude_pre, count_trigon_coplanar, count_error_tris_sepa,
count_point_inside_mesh;

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
	double projection, d_eps;
	bool isFirst /*= true*/, isLeft /*= false*/, temp /*= false*/;
	for (const auto& face : ibo)
	{
		normal = (vbo[face[1]] - vbo[face[0]]).cross(vbo[face[2]] - vbo[face[1]]);// .normalized();
		d_eps = normal.squaredNorm() * eps; // wide threshold
		isFirst = true;
		for (size_t i = 0; i < vbo.size(); ++i)
		{
			projection = normal.dot(vbo[i] - vbo[face[0]]);
			if (i == face[0] || i == face[1] || i == face[2] || fabs(projection) < d_eps) // self or coplanar
				continue;
			temp = projection < 0.0;
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
RelationOfPointAndMesh psykronix::isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	//one mesh inside other mesh, the bounding-box must inside other mesh
	Vector3d rayLine = Vector3d(1.0, 0.0, 0.0);
	Vector3d normal, local;
	std::array<Eigen::Vector3d, 3> trigon;
	int count = 0;
	double angle = 0.0, deno, k;
#ifdef STATISTIC_DATA_TESTFOR
	clock_t startT = clock(), endT;
	size_t countCr = 0, countPr = 0;
#endif
	auto _isRayAndTriangleIntersectParallel = [&](std::array<Eigen::Vector3d, 3 >& trigon)->bool
	{
		//if (fabs((point - trigon[0]).dot(normal)) > eps) // not coplanar
		if ((point - trigon[0]).dot(normal) != 0.0) // not coplanar
			return false;
		// negetive direction ray cause cross product result opposite
		return 
			((trigon[0] - point).cross(rayLine).dot(rayLine.cross(trigon[1] - point)) >= 0.0 && (trigon[0] - point).cross(rayLine).dot((trigon[0] - point).cross(trigon[1] - point)) >= 0.0) ||
			((trigon[1] - point).cross(rayLine).dot(rayLine.cross(trigon[2] - point)) >= 0.0 && (trigon[1] - point).cross(rayLine).dot((trigon[1] - point).cross(trigon[2] - point)) >= 0.0) ||
			((trigon[2] - point).cross(rayLine).dot(rayLine.cross(trigon[0] - point)) >= 0.0 && (trigon[2] - point).cross(rayLine).dot((trigon[2] - point).cross(trigon[0] - point)) >= 0.0);
	};
	auto _correctRayLine = [&]()
	{
		angle += 1.0; // 0.1(rad)~5.73(deg)
		Affine3d rotation = Affine3d::Identity();
		rotation.rotate(AngleAxisd(angle, Vector3d(rand(), rand(), rand()))); //Vector3d::UnitZ() //Vector3d(1, 1, 1)
		rayLine = rotation * rayLine;
		//countCr++;
	};
	while (true)
	{
		bool isNew = false;//new rayX
		for (const auto& iter : ibo) // iterate every trigon
		{
			trigon = { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] };
			if (isPointOnTriangleSurface(point, trigon))
				return RelationOfPointAndMesh::SURFACE; // ray across is false
			normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]);
			deno = rayLine.dot(normal); //ray.direction
			if (deno == 0.0)// (fabs(deno) < eps) //ray direction is parallel, redo circulation
			{
				if (_isRayAndTriangleIntersectParallel(trigon)) //coplanar
				{
					_correctRayLine();
					isNew = true;
					break;
				}
				continue;
			}
			k = (trigon[0] - point).dot(normal) / deno;
			if (k < 0.0) // only positive direction
				continue;
			local = point + k * rayLine;
			if (!isPointInTriangle(local, trigon))
				continue;
			if ((local - trigon[0]).isZero() ||
				(local - trigon[1]).isZero() ||
				(local - trigon[2]).isZero() ||
				(local - trigon[0]).cross(local - trigon[1]).isZero() ||
				(local - trigon[1]).cross(local - trigon[2]).isZero() ||
				(local - trigon[2]).cross(local - trigon[0]).isZero()) //singularity
			{
				_correctRayLine();
				isNew = true;
				break;
			}
			count++; // ray across is true
		}
		if (!isNew)
			break;//end while
#ifdef STATISTIC_DATA_TESTFOR
		endT = clock();
		if (1.0 < double(endT - startT) / CLOCKS_PER_SEC)
		{
			cout << "timeout!" << endl;
			//return {};
		}
#endif
	}
	return (count % 2 == 1) ? RelationOfPointAndMesh::INNER : RelationOfPointAndMesh::OUTER;
}

RelationOfPointAndMesh psykronix::isPointInsidePolyhedronROT(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	Eigen::Vector3d pointR = mesh.pose_.inverse() * point;
	return isPointInsidePolyhedronROT(pointR, mesh.vbo_, mesh.ibo_);
}

// include point on face
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
		Vector3d normal = (vbo[trigon[1]] - vbo[trigon[0]]).cross(vbo[trigon[2]] - vbo[trigon[1]]).normalized(); // for precision
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

bool psykronix::isPointInsidePolyhedronCL(const Eigen::Vector3d& _point, const ModelMesh& mesh) // more judge like ceil
{
#ifdef STATISTIC_DATA_COUNT
	count_point_inside_mesh++;
#endif
	Eigen::Vector3d point = mesh.pose_.inverse() * _point; // all vertex using self coordinate
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
	{	//pre-box
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
	if (!mesh.bounding_.contains(_point)) //mesh boundingbox is world coordinate
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
		normal = (p1 - p0).cross(p2 - p1);
		if (normal.z() == 0.0) // trigon is vertical
		{
			// point in segment collinear judge 
			if ((point.x() - p0.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (point.y() - p0.y()) == 0.0) //cross product is zero
			{
				Vector3d normalA = normal.cross(p1 - p0);
				Vector3d normalB = normal.cross(p2 - p1);
				Vector3d normalC = normal.cross(p0 - p2);
				// point under segment judge
				if (normalA.dot(point - p0) * normalA.z() < 0.0 || normalB.dot(point - p1) * normalB.z() < 0.0 || normalC.dot(point - p2) * normalC.z() < 0.0)
					return true; //count++; //
			}
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

// exclude point on surface
bool psykronix::isPointInsidePolyhedronFL(const Eigen::Vector3d& _point, const ModelMesh& mesh) // less judge like floor
{
#ifdef STATISTIC_DATA_COUNT
	count_point_inside_mesh++;
#endif
	Eigen::Vector3d point = mesh.pose_.inverse() * _point; // all vertex using self coordinate
	auto _isPointInTriangle2D = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)->bool
	{	//pre-box
		if (point.x() > std::max(std::max(p0[0], p1[0]), p2[0]) ||
			point.x() < std::min(std::min(p0[0], p1[0]), p2[0]) ||
			point.y() > std::max(std::max(p0[1], p1[1]), p2[1]) ||
			point.y() < std::min(std::min(p0[1], p1[1]), p2[1]) ||
			point.z() > std::max(std::max(p0[2], p1[2]), p2[2]) + eps) // isPointOnSurface with threshold
			return false;
		double az = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
		return //when vertical, az==0, always true
			0.0 <= az * ((p1[0] - p0[0]) * (point[1] - p0[1]) - (point[0] - p0[0]) * (p1[1] - p0[1])) &&
			0.0 <= az * ((p2[0] - p1[0]) * (point[1] - p1[1]) - (point[0] - p1[0]) * (p2[1] - p1[1])) &&
			0.0 <= az * ((p0[0] - p2[0]) * (point[1] - p2[1]) - (point[0] - p2[0]) * (p0[1] - p2[1]));
	};
	if (!mesh.bounding_.contains(_point)) //mesh boundingbox is world coordinate
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
		normal = (p1 - p0).cross(p2 - p1);
		if (normal.z() == 0.0) // trigon is vertical
		{
			// point in segment collinear judge 
			if ((point.x() - p0.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (point.y() - p0.y()) == 0.0) //cross product is zero
			{
				Vector3d normalA = normal.cross(p1 - p0);
				Vector3d normalB = normal.cross(p2 - p1);
				Vector3d normalC = normal.cross(p0 - p2);
				// point under segment judge
				if (normalA.dot(point - p0) * normalA.z() < 0.0 || normalB.dot(point - p1) * normalB.z() < 0.0 || normalC.dot(point - p2) * normalC.z() < 0.0)
					count++; //
			}
			continue; //judge point under segment
		}
		projection = normal.dot(point - p0);
		if (fabs(projection) < normal.squaredNorm() * eps) //isPointOnTriangleSurface exclude
			return false;
		if (projection * normal.z() < 0.0) // point under plane
			count++;
	}
	return count % 2 == 1;
}

//using method SAT
std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvexALL(const ModelMesh& meshA, const ModelMesh& meshB)
{
	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
	const std::vector<std::array<int, 3>>& iboA = meshA.ibo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	const std::vector<std::array<int, 3>>& iboB = meshB.ibo_;
	const Eigen::Affine3d& matA = meshA.pose_;
	const Eigen::Affine3d& matB = meshB.pose_;
	double dminA = DBL_MAX, dminB = DBL_MAX;
	double minA, maxA, minB, maxB, projection;//refresh min and max
	Eigen::Vector3d direction, normal;
	std::array<size_t, 2> indexAB = { 0, 0 };// { ULL_MAX, ULL_MAX };
	for (const auto& faceA : iboA) // iterate every face
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboA[faceA[1]] - vboA[faceA[0]]).cross(vboA[faceA[2]] - vboA[faceA[1]]).normalized(); //pose is same
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
			dminA = std::min(maxA - minB, maxB - minA); // intersect meshs, dmin is positive
			direction = normal;
		}
	}
	for (const auto& faceB : iboB) // iterate every face
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboB[faceB[1]] - vboB[faceB[0]]).cross(vboB[faceB[2]] - vboB[faceB[1]]).normalized(); //pose is same
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
			dminB = std::min(maxA - minB, maxB - minA); // intersect meshs, dmin is positive
			direction = normal;
		}
	}
	return { std::min(dminA, dminB) * direction, indexAB };
}

std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvexBOX(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::AlignedBox3d boxA = meshA.bounding_;
	Eigen::AlignedBox3d boxB = meshB.bounding_;
	// no contain sequence
	double dmin = DBL_MAX, dtemp;
	Vector3d coord;
	size_t index;
	Triangle unitThree = { Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1) };
	for (int i = 0; i < 3; i++)
	{
		dtemp = std::min(boxA.max()[i] - boxB.min()[i], boxB.max()[i] - boxA.min()[i]); //x direction
		if (dtemp < dmin)
		{
			dmin = dtemp;
			coord = unitThree[i]; //Eigen::Vector3d::UnitX();
			index = i;
		}
	}
	return { dmin * coord,{index, ULL_MAX} };
}

// for intrusive meshs, using major method SAT
std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvex(const ModelMesh& meshA, const ModelMesh& meshB, 
	const set<size_t>& faceSetA, const set<size_t>& faceSetB, const vector<size_t>& vertexVectA, const vector<size_t>& vertexVectB)
{
	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
	const std::vector<std::array<int, 3>>& iboA = meshA.ibo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	const std::vector<std::array<int, 3>>& iboB = meshB.ibo_;
	const Eigen::Affine3d& matA = meshA.pose_;
	const Eigen::Affine3d& matB = meshB.pose_;
	double dminA = DBL_MAX, dminB = DBL_MAX, dminA_V, dminB_V, tmpMaxA, tmpMaxB;
	double minA, maxA, minB, maxB, projection, origin;//refresh min and max
	Eigen::Vector3d directionA, directionB, directionA_V, directionB_V, normal;
	// the minimum distance must on the direction of face normal
	bool isFixedFaceA = true;
	size_t indexA = ULL_MAX, indexB = ULL_MAX, indexA_V = ULL_MAX, indexB_V = ULL_MAX;
	//std::array<size_t, 2> indexAB = { ULL_MAX , ULL_MAX };
	std::set<int> vboSetA, vboSetB; // to remove repeat vertex
	for (const auto& iterA : faceSetA) //merge vertex
	{
		vboSetA.insert(iboA[iterA][0]);
		vboSetA.insert(iboA[iterA][1]);
		vboSetA.insert(iboA[iterA][2]);
	}
	for (const auto& iterB : faceSetB)
	{
		vboSetB.insert(iboB[iterB][0]);
		vboSetB.insert(iboB[iterB][1]);
		vboSetB.insert(iboB[iterB][2]);
	}
	for (const auto& iA : faceSetA) // iterate every faceA
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboA[iboA[iA][1]] - vboA[iboA[iA][0]]).cross(vboA[iboA[iA][2]] - vboA[iboA[iA][1]]).normalized();
		for (const auto& vertexA : vboSetA)
		{
			projection = normal.dot(matA * vboA[vertexA]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboSetB)
		{
			projection = normal.dot(matB * vboB[vertexB]);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminA)// the intersect mesh-part prejection segment
		{
			dminA = std::min(maxA - minB, maxB - minA); //always positive, but impacted by accuracy, forexample cross edges and normalized
			directionA = normal;
			indexA = iA ;
#ifdef STATISTIC_DATA_RECORD
			if (dminA < 0.0)
				count_error_tris_sepa++;
			if (maxA - minB > maxB - minA)
				isFixedFaceA = false;
#endif
		}
		dminA_V = 0.0;
		tmpMaxA = 0.0;
		origin = normal.dot(matA * vboA[iboA[iA][0]]); // relative projection value
		for (const auto& iB : vertexVectB) // the inside vertex's prejection
		{
			projection = normal.dot(matB * vboB[iB]) - origin;
			if (projection < dminA_V)
			{
				dminA_V = projection; // always negative
				directionA_V = normal;
				indexA_V = iA;
			}
			if (!meshA.convex_ && tmpMaxA < projection) // the direction not exact
				tmpMaxA = projection;
		}
		if (dminA < tmpMaxA - dminA_V) // choose max, without vertex, dminV is zero
		{
			dminA = tmpMaxA - dminA_V;
			directionA = directionA_V;
			indexA = indexA_V;
		}
	}
	for (const auto& iB : faceSetB) // iterate every faceB
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboB[iboB[iB][1]] - vboB[iboB[iB][0]]).cross(vboB[iboB[iB][2]] - vboB[iboB[iB][1]]).normalized();
		for (const auto& vertexA : vboSetA)
		{
			projection = normal.dot(matA * vboA[vertexA]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboSetB)
		{
			projection = normal.dot(matB * vboB[vertexB]);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminB)//refresh 
		{
			dminB = std::min(maxA - minB, maxB - minA);
			directionB = normal;
			indexB = iB;
#ifdef STATISTIC_DATA_RECORD
			if (dminB < 0.0)
				count_error_tris_sepa++;
			if (maxA - minB < maxB - minA)
				isFixedFaceA = false;
#endif
		}
		dminB_V = 0.0;
		tmpMaxB = 0.0;
		origin = normal.dot(matA * vboB[iboB[iB][0]]); // relative projection value
		for (const auto& iA : vertexVectA) // the inside vertex's prejection
		{
			projection = normal.dot(matA * vboA[iA]) - origin;
			if (projection < dminB_V)
			{
				dminB_V = projection;  // always negative
				directionB_V = normal;
				indexB_V = iB;
			}
			if (!meshB.convex_ && tmpMaxB < projection) // the direction not exact
				tmpMaxB = projection;
		}
		if (dminB < tmpMaxB - dminB_V) // choose max
		{
			dminB = tmpMaxB - dminB_V;
			directionB = directionB_V;
			indexB = indexB_V;
		}
	}
	//if (!isFixedFaceA)
	//	direction = -direction;
	return (dminA < dminB) ?
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ dminA * directionA, { indexA, ULL_MAX }} :
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ dminB * directionB, { ULL_MAX, indexB } };
}

std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfVertexAndFace(const ModelMesh& meshA, const ModelMesh& meshB, 
	const vector<size_t>& faceVectA, const vector<size_t>& faceVectB, const vector<size_t>& vertexVectA, const vector<size_t>& vertexVectB)
{
	if ((faceVectA.empty() || vertexVectB.empty()) && (faceVectB.empty() || vertexVectA.empty())) // any empty cause zero
		return { gVecZero, { ULL_MAX , ULL_MAX } }; // or gVecNaN separate
	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
	const std::vector<std::array<int, 3>>& iboA = meshA.ibo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	const std::vector<std::array<int, 3>>& iboB = meshB.ibo_;
	const Eigen::Affine3d& matA = meshA.pose_;
	const Eigen::Affine3d& matB = meshB.pose_;
	Eigen::Vector3d directionA, directionB, normal;
	double dminA = 0, dminB = 0, projection, origin, tmpMaxA = 0, tmpMaxB = 0;
	size_t indexA = ULL_MAX, indexB = ULL_MAX;
#ifdef STATISTIC_DATA_RECORD
	set<double> projectionA, projectionB;
#endif
	for (const auto& iA : faceVectA) // the face normal direction always outwards
	{
		if (vertexVectB.empty())
		{
			dminA = -DBL_MAX;
			break;
		}
		normal = (vboA[iboA[iA][1]] - vboA[iboA[iA][0]]).cross(vboA[iboA[iA][2]] - vboA[iboA[iA][1]]).normalized();
		origin = normal.dot(matA * vboA[iboA[iA][0]]); // relative projection value
		for (const auto& iB : vertexVectB)
		{
			//projection = normal.dot(matB * vboB[iB] - vboA[iboA[iA][0]]);
			projection = normal.dot(matB * vboB[iB]) - origin;
#ifdef STATISTIC_DATA_RECORD
			projectionA.insert(projection);
#endif
			if (projection < dminA) // always negative
			{
				dminA = projection;
				directionA = normal;
				indexA = iA;
			}
			if (!meshA.convex_ && tmpMaxA < projection) // the direction not exact
			{
				tmpMaxA = projection;
			}
		}
	}
	for (const auto& iB : faceVectB) // the face normal direction always outwards
	{
		if (vertexVectA.empty())
		{
			dminB = -DBL_MAX;
			break;
		}
		normal = (vboB[iboB[iB][1]] - vboB[iboB[iB][0]]).cross(vboB[iboB[iB][2]] - vboB[iboB[iB][1]]).normalized();
		origin = normal.dot(matB * vboB[iboB[iB][0]]);
		for (const auto& iA : vertexVectA)
		{
			projection = normal.dot(matA * vboA[iA]) - origin;
#ifdef STATISTIC_DATA_RECORD
			projectionB.insert(projection);
#endif
			if (projection < dminB)
			{
				dminB = projection;
				directionB = normal;
				indexB = iB;
			}
			if (!meshA.convex_ && tmpMaxB < projection) // the direction not exact
			{
				tmpMaxB = projection;
			}
		}
	}
	return (tmpMaxA - dminA < tmpMaxB - dminB) ? //return min
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ (tmpMaxA - dminA) * directionA, { indexA, ULL_MAX }} :
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ (tmpMaxB - dminB) * directionB, { ULL_MAX, indexB } };
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

// get the index number of mesh's ibo
std::array<std::vector<size_t>, 2> _getReducedIntersectTrianglesOfMesh(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance)
{
	//Eigen::AlignedBox3d boxMag(box.min() - 0.5 * Vector3d(tolerance, tolerance, tolerance), box.max() + 0.5 * Vector3d(tolerance, tolerance, tolerance));
	Eigen::AlignedBox3d box = meshA.bounding_.intersection(meshB.bounding_);
	Vector3d toleSize = Vector3d(tolerance, tolerance, tolerance);
	Eigen::AlignedBox3d boxMag(box.min() - toleSize, box.max() + toleSize);
	std::vector<size_t> triA_Index; // using index of mesh IBO
	Eigen::AlignedBox3d triA_Box; // iterate to lessen box
	for (size_t i = 0; i < meshA.ibo_.size(); ++i)
	{
		std::array<Eigen::Vector3d, 3> triIter = { // input matrix to avoid repeat calculate
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][0]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][1]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][2]] };
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
	for (size_t j = 0; j < meshB.ibo_.size(); ++j)
	{
		std::array<Eigen::Vector3d, 3> triIter = {
				meshB.pose_* meshB.vbo_[meshB.ibo_[j][0]],
				meshB.pose_* meshB.vbo_[meshB.ibo_[j][1]],
				meshB.pose_* meshB.vbo_[meshB.ibo_[j][2]] };
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
bool isTwoMeshsIntersectSAT(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	// get the index param of mesh's ibo
	std::array<std::vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(meshA, meshB, 0.0);
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		std::array<Eigen::Vector3d, 3> triA, triB;
		for (const auto& iA : indexAB[0])
		{
			triA = { relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
					relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
					relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
			for (const auto& iB : indexAB[1])
			{
				triB = { meshB.vbo_[meshB.ibo_[iB][0]],
						meshB.vbo_[meshB.ibo_[iB][1]],
						meshB.vbo_[meshB.ibo_[iB][2]] };
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
	if (meshA.vbo_.empty() || meshB.vbo_.empty()) // extra safe check
	{
#ifdef STATISTIC_DATA_COUNT
		count_err_empty_mesh++;
#endif  
		return false;
	}
	//judge whether mesh entirely inside mesh
	if (meshA.bounding_.contains(meshB.bounding_)) //boundingbox is world coord
	{
		// precondition: boundingbox inside, polyface not intersect, mesh isnot empty
		if (isPointInsidePolyhedronAZ(meshA.pose_.inverse() * meshB.pose_ * meshB.vbo_[0], meshA))
		{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			triRecordHard.push_back({ gTirNaN, gTirNaN });
			interTriInfoList.push_back({ { gTirNaN, gTirNaN }, {}, 0.0 });
#endif
			return true;
		}
	}
	else if (meshB.bounding_.contains(meshA.bounding_))
	{
		if (isPointInsidePolyhedronAZ(meshA.pose_.inverse() * meshB.pose_ * meshA.vbo_[0], meshB))
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

// clash judge include penetration depth
std::tuple<RelationOfTwoMesh, Eigen::Vector3d> getTwoMeshsIntersectRelation(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	// get the index param of mesh's ibo
	std::array<std::vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(meshA, meshB, 0.0);
	bool isContact = false; // vertex or edge or face contact
	bool isIntrusive = false;
	std::set<size_t> faceSetA, faceSetB; // intersect and inside, to remove repeat
	std::vector<size_t> vertexVectA, vertexVectB;
	//std::array<Eigen::Vector3d, 2> pInter;
#ifdef STATISTIC_DATA_TESTFOR
	vector<double> cpVect;
#endif
#ifdef STATISTIC_DATA_RECORD
	Triangle trigon;
#endif
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		std::array<Eigen::Vector3d, 3> triA, triB;
		for (const auto& iA : indexAB[0]) //faces intersect
		{
			triA = { relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
					relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
					relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
			for (const auto& iB : indexAB[1])
			{
				triB = { meshB.vbo_[meshB.ibo_[iB][0]],
						meshB.vbo_[meshB.ibo_[iB][1]],
						meshB.vbo_[meshB.ibo_[iB][2]] };
				if (!isTwoTrianglesBoundingBoxIntersect(triA, triB)) // second pre-judge
				{
#ifdef STATISTIC_DATA_COUNT
					count_tri_box_exclude_pre++;
#endif  
					continue;
				}
				if (!isTwoTrianglesIntersectSAT(triA, triB)) //separate
					continue;
				isContact = true; //isIntersect// maybe intrusive or atleast contact
				//intersect
#ifdef STATISTIC_DATA_TESTFOR
				//double cp = (triA[1] - triA[0]).cross(triA[2] - triA[1]).cross((triB[1] - triB[0]).cross(triB[2] - triB[1])).squaredNorm();
				//cpVect.push_back(cp);
				//sort(cpVect.begin(), cpVect.end());
#endif
				if ((triA[1] - triA[0]).cross(triA[2] - triA[1]).cross((triB[1] - triB[0]).cross(triB[2] - triB[1])).isZero(eps)) // intersect-coplanar
				{
#ifdef STATISTIC_DATA_COUNT
					count_trigon_coplanar++;
#endif  
					continue;
				}
				isIntrusive = true; 
				faceSetA.insert(iA); //faceVectA.push_back(iA);
				faceSetB.insert(iB); //faceVectB.push_back(iB);
				//pInter = getTwoTrianglesIntersectPoints(triA, triB);
				//if (!(pInter[1] - pInter[0]).isZero()) // also using eps
				//{
				//	//return RelationOfTwoMesh::INTRUSIVE;
				//	isIntrusive = true; //for count all intersect
				//	faceSetA.insert(iA); // only intrusive triangle insert, exclude contact triangle
				//	faceSetB.insert(iB);
				//}
			}
		}
	}
	if (isIntrusive)
	{
		//for (const auto& iA : indexAB[0]) //only insert intersect mesh
		//{
		//	if (isPointInsidePolyhedronCL(meshA.pose_ * meshA.vbo_[meshA.ibo_[iA][0]], meshB) ||
		//		isPointInsidePolyhedronCL(meshA.pose_ * meshA.vbo_[meshA.ibo_[iA][1]], meshB) ||
		//		isPointInsidePolyhedronCL(meshA.pose_ * meshA.vbo_[meshA.ibo_[iA][2]], meshB))
		//		faceSetA.insert(iA);
		//}
		//for (const auto& iB : indexAB[1])
		//{
		//	if (isPointInsidePolyhedronCL(meshB.pose_ * meshB.vbo_[meshB.ibo_[iB][0]], meshA) ||
		//		isPointInsidePolyhedronCL(meshB.pose_ * meshB.vbo_[meshB.ibo_[iB][1]], meshA) ||
		//		isPointInsidePolyhedronCL(meshB.pose_ * meshB.vbo_[meshB.ibo_[iB][2]], meshA))
		//		faceSetA.insert(iB);
		//}
		for (size_t i = 0; i < meshA.vbo_.size(); ++i)
		{
			if (isPointInsidePolyhedronFL(meshA.pose_ * meshA.vbo_[i], meshB))
				vertexVectA.push_back(i);
		}
		for (size_t i = 0; i < meshB.vbo_.size(); ++i)
		{
			if (isPointInsidePolyhedronFL(meshB.pose_ * meshB.vbo_[i], meshA))
				vertexVectB.push_back(i);
		}
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>> depth = getPenetrationDepthOfTwoConvex(meshA, meshB, faceSetA, faceSetB, vertexVectA, vertexVectB);
		//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> depth = getPenetrationDepthOfVertexAndFace(meshA, meshB, faceVectA, faceVectB, vertexVectA, vertexVectB);
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
		if (std::get<1>(depth)[0] == ULL_MAX && std::get<1>(depth)[1] == ULL_MAX)
			interTriInfoList.push_back({ { std::move(gTirNaN), std::move(gTirNaN) }, {}, -std::get<0>(depth).norm() });
		else if (std::get<1>(depth)[0] != ULL_MAX) //bool isLeft 
		{
			trigon = { meshA.pose_ * meshA.vbo_[meshA.ibo_[get<1>(depth)[0]][0]],
					meshA.pose_ * meshA.vbo_[meshA.ibo_[get<1>(depth)[0]][1]],
					meshA.pose_ * meshA.vbo_[meshA.ibo_[get<1>(depth)[0]][2]] };
			interTriInfoList.push_back({ { std::move(trigon), std::move(gTirNaN) }, {}, -std::get<0>(depth).norm() });
		}
		else
		{
			trigon = { meshB.pose_ * meshB.vbo_[meshB.ibo_[get<1>(depth)[1]][0]],
					meshB.pose_ * meshB.vbo_[meshB.ibo_[get<1>(depth)[1]][1]],
					meshB.pose_ * meshB.vbo_[meshB.ibo_[get<1>(depth)[1]][2]] };
			interTriInfoList.push_back({ { std::move(gTirNaN), std::move(trigon) }, {}, -std::get<0>(depth).norm() });
		}
#endif
		//RelationOfTwoMesh::INTRUSIVE 
		return { RelationOfTwoMesh::INTRUSIVE , std::get<0>(depth) };
	}
	//judge whether mesh entirely inside mesh 
	//lefted, total inside, inner-contact or outer-contact
	if (meshA.bounding_.contains(meshB.bounding_)) //boundingbox is world coord
	{
		// precondition: boundingbox inside, polyface not intersect, mesh isnot empty
		Eigen::Affine3d mat = meshA.pose_.inverse() * meshB.pose_;
		RelationOfPointAndMesh relation = isPointInsidePolyhedronROT(mat * (*meshB.vbo_.begin()), meshA.vbo_, meshA.ibo_);// already pre judge, mesh isnot empty
		if (RelationOfPointAndMesh::OUTER == relation)
		{
			return isContact ?
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::CONTACT_OUTER, gVecZero} :
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::SEPARATE , gVecNaN };
		}
		else //if (RelationOfPointAndMesh::INNER == relation)
		{
			std::tuple<Eigen::Vector3d, std::array<size_t, 2>> depth = getPenetrationDepthOfTwoConvexBOX(meshA, meshB);
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			std::array<Triangle, 3> triThree = { gTriYOZ ,gTriXOZ ,gTriXOY };
			interTriInfoList.push_back({ { std::move(triThree[std::get<1>(depth)[0]]), std::move(gTirNaN)}, {}, -std::get<0>(depth).norm()});
#endif
			return isContact ?
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA_CONT, std::get<0>(depth) } :
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA , std::get<0>(depth) };
		}
	}
	else if (meshB.bounding_.contains(meshA.bounding_))
	{
		Eigen::Affine3d mat = meshB.pose_.inverse() * meshA.pose_;
		RelationOfPointAndMesh relation = isPointInsidePolyhedronROT(mat * (*meshA.vbo_.begin()), meshB.vbo_, meshB.ibo_);
		if (RelationOfPointAndMesh::OUTER == relation)
		{
			return isContact ?
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::CONTACT_OUTER, gVecZero} :
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::SEPARATE , gVecNaN };
		}
		else //if (RelationOfPointAndMesh::INNER == relation)
		{
			std::tuple<Eigen::Vector3d, std::array<size_t, 2>> depth = getPenetrationDepthOfTwoConvexBOX(meshA, meshB);
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			std::array<Triangle, 3> triThree = { gTriYOZ ,gTriXOZ ,gTriXOY };
			interTriInfoList.push_back({ { std::move(triThree[std::get<1>(depth)[0]]), std::move(gTirNaN)}, {}, -std::get<0>(depth).norm() });
#endif
			return isContact ?
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA_CONT, std::get<0>(depth) } :
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA , std::get<0>(depth) };
		}
	}
	return { RelationOfTwoMesh::SEPARATE, gVecZero }; 
}

// ClashSoft // return index of mesh_a and mesh_b ibo
std::tuple<double, std::array<size_t, 2>> getTwoMeshsSeparationDistanceSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance)
{
#ifdef STATISTIC_DATA_RECORD
	std::array<std::array<Eigen::Vector3d, 3>, 2> triDistPair;
#endif    
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	// distance > tolerance, return double-max, to decrease calculate
	double d = DBL_MAX, temp; // the res
	std::array<size_t, 2> index = { ULL_MAX, ULL_MAX };
	std::array<vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(meshA, meshB, tolerance);
	if (indexAB[0].empty() || indexAB[1].empty())
		return { d, index };
	for (const auto& iA : indexAB[0])
	{
		std::array<Eigen::Vector3d, 3> triA = {
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
		for (const auto& iB : indexAB[1])
		{
			std::array<Eigen::Vector3d, 3> triB = {
					meshB.vbo_[meshB.ibo_[iB][0]],
					meshB.vbo_[meshB.ibo_[iB][1]],
					meshB.vbo_[meshB.ibo_[iB][2]] };
			if (!isTwoTrianglesBoundingBoxIntersect(triA, triB, tolerance))
			{
#ifdef STATISTIC_DATA_COUNT
				count_tri_box_exclude_pre++;
#endif                    
				continue;
			}
			temp = getTrianglesDistanceSAT(triA, triB);// two trigons distance calculate
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

//---------------------------------------------------------------------------
//  penetration depth
//---------------------------------------------------------------------------

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

// return the vertex of meshA
std::vector<Eigen::Vector3d> _getInsideVertexSet(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	// the vertex of meshA inside meshB
	std::vector<Eigen::Vector3d> res;
	//for (const auto& iter : meshA.vbo_)
	//{
	//	if (RelationOfPointAndMesh::INNER == isPointInsidePolyhedronROT(relative_matrix * iter, meshB.vbo_, meshB.ibo_))
	//		res.push_back(meshA.pose_ * iter);
	//}
	return res;
}

std::vector<Eigen::Vector3d> _getIntersectVertexSet(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	std::vector<Eigen::Vector3d> res;
	//for (size_t i = 0; i < meshA.ibo_.size(); i++)
	//{
	//	std::array<Eigen::Vector3d, 3> triA = {
	//			relative_matrix * meshA.vbo_[meshA.ibo_[i][0]],
	//			relative_matrix * meshA.vbo_[meshA.ibo_[i][1]],
	//			relative_matrix * meshA.vbo_[meshA.ibo_[i][2]] };
	//	for (size_t j = 0; j < meshA.ibo_.size(); j++)
	//	{
	//		std::array<Eigen::Vector3d, 3> triB = {
	//				meshB.vbo_[meshB.ibo_[j][0]],
	//				meshB.vbo_[meshB.ibo_[j][1]],
	//				meshB.vbo_[meshB.ibo_[j][2]] };
	//		if (!isTwoTrianglesIntersectSAT(triA, triB))
	//			continue;
	//		std::array<Eigen::Vector3d, 2> pInter = getTwoTrianglesIntersectPoints(triA, triB);
	//		res.push_back(meshB.pose_ * pInter[0]);
	//		res.push_back(meshB.pose_ * pInter[1]);
	//	}
	//}
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
Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB)
{
	//must intersect
	Eigen::Vector3d direction = gVecNaN;
	return direction;
}

double getMoveDistanceOfAssignedDirection(const ModelMesh& meshA, const ModelMesh& meshB, const Eigen::Vector3d& direction)
{
	return 0;
}

