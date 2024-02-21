#include "pch.h"
#include "calculatePolyhedron.h"
using namespace std;
using namespace Eigen;
using namespace games;
using namespace clash;
//static constexpr double eps_d = 10 * DBL_EPSILON; // double
static const Triangle gTriXOY = { Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0) };
static const Triangle gTriXOZ = { Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,0,1) };
static const Triangle gTriYOZ = { Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1) };

#ifdef max
#undef max 
#endif // max
#ifdef min
#undef min 
#endif // min
#define USING_METHOD_SAT
#define USING_RELATIVE_MATRIX_RECTIFY
#define DEBUG_POLYHEDRON_MESH

//replace .x() => [0]
//replace .y() => [1]
//replace .z() => [2]

#ifdef STATISTIC_DATA_COUNT
extern std::atomic<size_t> count_mesh_inside_mesh,
count_reduced_exclude_pre, count_triA_inter, count_triB_inter,
count_err_empty_mesh, count_tri_box_exclude_pre, count_error_tris_sepa,
count_point_inside_mesh, count_facesetA, count_facesetB, count_triAB_nan, count_triB_nan,
count_meshs_isContact, count_meshs_isIntrusive, count_err_inter_mesh_sepa, count_repeat_sepa_axis,
count_cal_sepa_axis;
#endif //STATISTIC_DATA_COUNT

#ifdef DEBUG_POLYHEDRON_MESH
static int count_edgediag_full = 0;
#endif // DEBUG_POLYHEDRON_MESH


#ifdef STATISTIC_DATA_RECORD
//container
extern std::vector<std::array<Triangle, 2>> triPairList, triRecordHard; //std::array<Eigen::Vector3d, 3>
extern std::vector<Triangle> triFaceVctA, triFaceVctB;
extern std::vector<Vector3d> triVertexVctA, triVertexVctB;
extern std::vector<InterTriInfo> interTriInfoList;
#endif // STATISTIC_DATA_RECORD

bool clash::isMeshConvexPolyhedron(const ModelMesh& mesh)
//bool clash::isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	bool isLeft = false;
	for (const auto& face : ibo)
	{
		Vector3d normal = (vbo[face[1]] - vbo[face[0]]).cross(vbo[face[2]] - vbo[face[1]]);// .normalized();
		double d_eps = normal.squaredNorm() * epsF; // wide threshold
		bool isFirst = true;
		for (size_t i = 0; i < vbo.size(); ++i)
		{
			double projection = normal.dot(vbo[i] - vbo[face[0]]);
			if (i == face[0] || i == face[1] || i == face[2] || fabs(projection) < d_eps) // self or coplanar
				continue;
			bool temp = projection < 0.0;
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

int clash::getMeshGenusNumber(const ModelMesh& mesh)
{
	set<array<int, 2>> uniqueEdge;
	for (const auto& iter : mesh.ibo_)
	{
		array<int, 4> tri = { iter[0], iter[1], iter[2], iter[0] };
		for (int i = 0; i < 3; i++)
		{
			array<int, 2> edge = (tri[i] < tri[i + 1]) ?
				array<int, 2>{tri[i], tri[i + 1]} : array<int, 2>{tri[i + 1], tri[i]};
			uniqueEdge.insert(edge);
		}
	}
	size_t V = mesh.vbo_.size();
	size_t F = mesh.ibo_.size();
	size_t E = uniqueEdge.size();
	return int(1 - (V - E + F) / 2); //V - E + F = 2(1-g)
}

vector<Vector3d> clash::getNormalVectorOfMeshFace(const ModelMesh& mesh) //using ray method
{
	vector<Vector3d> fno; //res
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	Vector3d bary;//barycentric coordinates
	auto _correctRayLine = [&bary](const Triangle& face)->void
		{
			double a = (double)std::rand() / RAND_MAX;
			double b = 0.5 * (1 - a);
			bary = a * face[0] + b * face[1] + b * face[2];
		};
	for (const auto& faceid : ibo)
	{
		Triangle face = { vbo[faceid[0]] ,vbo[faceid[1]] ,vbo[faceid[2]] };
		bary = 1 / 3 * (face[0] + face[1] + face[2]);
		Vector3d normal = (face[1] - face[0]).cross(face[2] - face[1]);
		int count = 0;
		while (true)
		{
			bool isNew = false;//new ray-line
			for (const auto& iter : ibo) // iterate every trigon
			{
				Triangle trigon = { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] };
				int cross = isRayLineCrossTriangleMTA(bary, normal, trigon);
				if (cross == 0)
					continue;
				else if (cross == 1)
					count++;
				else// if (cross == -1)// on edge
				{
					_correctRayLine(trigon);
					isNew = true;
					break;
				}
			}
			if (isNew == false)
				break;//end while
		}
		if (count % 2 == 1) //inner
			normal = -normal;
		fno.push_back(normal);
	}
	return fno;
}

vector<Eigen::Vector3d> clash::getProfileOutOfMesh(const ModelMesh& mesh, const Plane3d& plane)
{
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	vector<array<int, 3>> faceVisible;
	vector<Vector3d> fno = getNormalVectorOfMeshFace(mesh); //correct face normal
	for (int i = 0; i < ibo.size(); ++i) //fno_ size equal
	{
		Triangle face = { vbo[ibo[i][0]] ,vbo[ibo[i][1]] ,vbo[ibo[i][2]] };
		Vector3d normal = (face[1] - face[0]).cross(face[2] - face[1]);
		if (std::acos(plane.m_normal.dot(fno[i]) / (plane.m_normal.norm() * fno[i].norm())) < M_PI_2)
			faceVisible.push_back(mesh.ibo_[i]);
	}
	// for convex polyhedron, no shielded, no genus
	set<array<int, 2>> profileEdge;
	for (const auto& iter : faceVisible)
	{
		array<int, 4> tri = { iter[0], iter[1], iter[2], iter[0] };
		for (int i = 0; i < 3; i++)
		{
			array<int, 2> edge = (tri[i] < tri[i + 1]) ? array<int, 2>{tri[i], tri[i + 1]} : array<int, 2>{tri[i + 1], tri[i]};
			if (profileEdge.find(edge) != profileEdge.end())
				profileEdge.erase(edge);
			else
				profileEdge.insert(edge);
		}
	}
	if (profileEdge.empty())
		return {};
	// merge to close profile
	vector<int> profileIndex = { (*profileEdge.begin())[0], (*profileEdge.begin())[1] };
	profileEdge.erase(profileEdge.begin());
	while (!profileEdge.empty())
	{
		for (const auto& iter : profileEdge)
		{
			if (iter[0] == profileIndex.back() || iter[1] == profileIndex.back())
			{
				(iter[0] == profileIndex.back()) ?
					profileIndex.push_back(iter[1]) : profileIndex.push_back(iter[0]);
				profileEdge.erase(iter);
			}
		}
	}
	vector<Vector3d> profilePoints;
	for (const int& i : profileIndex)
		profilePoints.push_back(vbo[i]);
	Matrix4d matSha = getProjectionMatrixByPlane(plane);
	for (auto& iter : profilePoints)
		iter = (matSha * iter.homogeneous()).hnormalized();
	return profilePoints;
}

bool clash::isPointInsidePolyhedronMTA(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	Vector3d direction = Vector3d(1.0, 0.0, 0.0);
	int count = 0;
	while (true)
	{
		bool isNew = false;//new ray-line
		for (const auto& iter : ibo) // iterate every trigon
		{
			Triangle trigon = { vbo[iter[0]] ,vbo[iter[1]] ,vbo[iter[2]] };
			int cross = isRayLineCrossTriangleMTA(point, direction, trigon);
			if (cross == 0)
				continue;
			else if (cross == 1)
				count++;
			else// if (cross == -1)// on edge
			{
				direction = Vector3d(rand(), rand(), rand());
				isNew = true;
				break;
			}
		}
		if (isNew == false)
			break;//end while
	}
	return count % 2 == 1;
}

// exclude point on face, ray is rotate random
RelationOfPointAndMesh clash::isPointInsidePolyhedronROT(const Eigen::Vector3d& _point, const ModelMesh& mesh)
{
	Eigen::Vector3d point = mesh.pose_.inverse() * _point; // to relative coordinate
	if (!mesh.bounding_.contains(point))
		return RelationOfPointAndMesh::OUTER;
	//auto _isPointOnPolyhedronSurface = [&]()->bool
	bool isSurface = false;
	for (const auto& face : mesh.ibo_)
	{
		if (isPointOnTriangleSurface(point, { mesh.vbo_[face[0]], mesh.vbo_[face[1]], mesh.vbo_[face[2]] }))
		{
			isSurface = true;
			break;
		}
	}
	if (isSurface)
		return RelationOfPointAndMesh::SURFACE;
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	//one mesh inside other mesh, the bounding-box must inside other mesh
	Vector3d rayLine = Vector3d(1.0, 0.0, 0.0);
	Vector3d normal;
	std::array<Eigen::Vector3d, 3> trigon;
	int count = 0;
	double angle = 0.0;
#ifdef STATISTIC_DATA_TESTFOR
	clock_t startT = clock(), endT;
#endif
	auto _isRayAndTriangleIntersectParallel = [&point, &normal, &rayLine](std::array<Eigen::Vector3d, 3 >& trigon)->bool
		{
			//if (fabs((point - trigon[0]).dot(normal)) > epsF) // not coplanar
			if ((point - trigon[0]).dot(normal) != 0.0) // not coplanar
				return false;
			// negetive direction ray cause cross product result opposite
			return
				((trigon[0] - point).cross(rayLine).dot(rayLine.cross(trigon[1] - point)) >= 0.0 && (trigon[0] - point).cross(rayLine).dot((trigon[0] - point).cross(trigon[1] - point)) >= 0.0) ||
				((trigon[1] - point).cross(rayLine).dot(rayLine.cross(trigon[2] - point)) >= 0.0 && (trigon[1] - point).cross(rayLine).dot((trigon[1] - point).cross(trigon[2] - point)) >= 0.0) ||
				((trigon[2] - point).cross(rayLine).dot(rayLine.cross(trigon[0] - point)) >= 0.0 && (trigon[2] - point).cross(rayLine).dot((trigon[2] - point).cross(trigon[0] - point)) >= 0.0);
		};
	auto _correctRayLine = [&angle, &rayLine]()
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
			double deno = rayLine.dot(normal); //ray.direction
			if (deno == 0.0)// (fabs(deno) < epsF) //ray direction is parallel, redo circulation
			{
				if (_isRayAndTriangleIntersectParallel(trigon)) //coplanar
				{
					_correctRayLine();
					isNew = true;
					break;
				}
				continue;
			}
			double k = (trigon[0] - point).dot(normal) / deno;
			if (k < 0.0) // only positive direction
				continue;
			Vector3d local = point + k * rayLine;
			if (!isPointInTriangle(local, trigon))
				continue;
			if ((local - trigon[0]).isZero(epsF) ||
				(local - trigon[1]).isZero(epsF) ||
				(local - trigon[2]).isZero(epsF) ||
				(local - trigon[0]).cross(local - trigon[1]).isZero(epsF) ||
				(local - trigon[1]).cross(local - trigon[2]).isZero(epsF) ||
				(local - trigon[2]).cross(local - trigon[0]).isZero(epsF)) //singularity
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

// include point on face
bool isPointInsidePolyhedronAZ(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	//Eigen::Vector3d point = mesh.pose_.inverse() * _point;
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	auto _relationOfPointAndTriangle = [&point](/*const Vector3d& point,*/ const std::array<Vector3d, 3>& trigon)->RelationOfRayAndTrigon // axisZ direction
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
	auto isLeftAll = [&vbo](const std::array<int, 3>& trigon)->bool // buildin lambda function
		{
			Vector3d normal = (vbo[trigon[1]] - vbo[trigon[0]]).cross(vbo[trigon[2]] - vbo[trigon[1]]).normalized(); // for precision
			bool isFirst = true, isLeft = false, temp /*= false*/;
			for (size_t i = 0; i < vbo.size(); ++i)
			{
				if (i == trigon[0] || i == trigon[1] || i == trigon[2] || fabs(normal.dot((vbo[i] - vbo[trigon[0]]).normalized())) < epsF) // self and coplanar
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

// include point on surface
bool isPointInsidePolyhedronCL(const Eigen::Vector3d& _point, const ModelMesh& mesh) // more judge like ceil
{
#ifdef STATISTIC_DATA_COUNT
	count_point_inside_mesh++;
#endif
	Eigen::Vector3d point = mesh.pose_.inverse() * _point; // all vertex using self coordinate
	//auto _isPointInTriangle2D = [&point](const std::array<Vector3d, 3>& trigon)->bool
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
	auto _isPointInTriangle2D = [&point](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)->bool
		{	//pre-box
			if (point.x() > std::max(std::max(p0[0], p1[0]), p2[0]) ||
				point.x() < std::min(std::min(p0[0], p1[0]), p2[0]) ||
				point.y() > std::max(std::max(p0[1], p1[1]), p2[1]) ||
				point.y() < std::min(std::min(p0[1], p1[1]), p2[1]) ||
				point.z() > std::max(std::max(p0[2], p1[2]), p2[2]) + epsF) //include z
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
	size_t count = 0;
	for (const auto& face : mesh.ibo_)
	{
		//Triangle trigon = { { mesh.vbo_[face[0]] ,mesh.vbo_[face[1]] ,mesh.vbo_[face[2]] } };
		const Vector3d& p0 = mesh.vbo_[face[0]];
		const Vector3d& p1 = mesh.vbo_[face[1]];
		const Vector3d& p2 = mesh.vbo_[face[2]];
		if (!_isPointInTriangle2D(p0, p1, p2))
			continue;
		Eigen::Vector3d normal = (p1 - p0).cross(p2 - p1);
		if (normal.z() == 0.0) // trigon is vertical
		{
			if ((point - p0).dot(normal) == 0.0) //point on plane judge, also using 2d dot
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
		double projection = normal.dot(point - p0);
		if (projection == 0.0) //isPointOnTriangleSurface
			return true;
		if (projection * normal.z() < 0.0) // point under plane
			count++;
	}
	return count % 2 == 1;
}

// exclude point on surface
bool isPointInsidePolyhedronFL(const Eigen::Vector3d& _point, const ModelMesh& mesh) // less judge like floor
{
#ifdef STATISTIC_DATA_COUNT
	count_point_inside_mesh++;
#endif
	Eigen::Vector3d point = mesh.pose_.inverse() * _point; // all vertex using self coordinate
	auto _isPointInTriangle2D = [&point](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)->bool
		{	//pre-box
			if (point.x() > std::max(std::max(p0[0], p1[0]), p2[0]) ||
				point.x() < std::min(std::min(p0[0], p1[0]), p2[0]) ||
				point.y() > std::max(std::max(p0[1], p1[1]), p2[1]) ||
				point.y() < std::min(std::min(p0[1], p1[1]), p2[1]) ||
				point.z() > std::max(std::max(p0[2], p1[2]), p2[2]) + epsF) // isPointOnSurface with threshold
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
	size_t count = 0;
	for (const auto& face : mesh.ibo_)
	{
		//Triangle trigon = { { mesh.vbo_[face[0]] ,mesh.vbo_[face[1]] ,mesh.vbo_[face[2]] } };
		const Vector3d& p0 = mesh.vbo_[face[0]];
		const Vector3d& p1 = mesh.vbo_[face[1]];
		const Vector3d& p2 = mesh.vbo_[face[2]];
		if (!_isPointInTriangle2D(p0, p1, p2))
			continue;
		Eigen::Vector3d normal = (p1 - p0).cross(p2 - p1);
		if (normal.z() == 0.0) // trigon is vertical
		{
			if ((point - p0).dot(normal) == 0.0 && isPointInTriangle(point, { p0, p1 ,p2 })) //point on plane judge
				return false; // exclude point on surface
			//	//(point.x() - p0.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (point.y() - p0.y()) == 0.0) //cross product is zero in XoY
			//{
			//	Vector3d normalA = normal.cross(p1 - p0);
			//	Vector3d normalB = normal.cross(p2 - p1);
			//	Vector3d normalC = normal.cross(p0 - p2);
			//	// point under segment judge
			//	if (normalA.dot(point - p0) * normalA.z() < 0.0 || normalB.dot(point - p1) * normalB.z() < 0.0 || normalC.dot(point - p2) * normalC.z() < 0.0)
			//		count++; //
			//}
			continue;
		}
		double projection = normal.dot(point - p0);
		if (fabs(projection) < normal.squaredNorm() * epsF) //isPointOnTriangleSurface exclude
			return false;
		if (projection * normal.z() < 0.0) // point under plane
			count++;
	}
	return count % 2 == 1;
}

bool isPointOnPolyhedronSurface(const Eigen::Vector3d& _point, const ModelMesh& mesh)
{
	Eigen::Vector3d point = mesh.pose_.inverse() * _point;
	for (const auto& face : mesh.ibo_)
	{
		if (isPointOnTriangleSurface(point, { mesh.vbo_[face[0]], mesh.vbo_[face[1]], mesh.vbo_[face[2]] }))
			return true;
	}
	return false;
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
	Eigen::Affine3d matIAB = matA.inverse() * matB;
	Eigen::Affine3d matIBA = matB.inverse() * matA;
	double dminA = DBL_MAX, dminB = DBL_MAX;
	double minA, maxA, minB, maxB, projection;//refresh min and max
	Eigen::Vector3d direction, normal;
	std::array<size_t, 2> indexAB = { 0, 0 };// { ULLONG_MAX, ULLONG_MAX };
	for (const auto& faceA : iboA) // iterate every face
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboA[faceA[1]] - vboA[faceA[0]]).cross(vboA[faceA[2]] - vboA[faceA[1]]).normalized(); //pose is same
		for (const auto& vertexA : vboA)
		{
			projection = normal.dot(vertexA);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboB)
		{
			projection = normal.dot(matIAB * vertexB);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminA)//refresh 
		{
			dminA = std::min(maxA - minB, maxB - minA); // intersect meshs, dmin is positive
			direction = matA.rotation() * normal; //get the rotation part
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
			projection = normal.dot(matIBA * vertexA);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboB)
		{
			projection = normal.dot(vertexB);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (std::min(maxA - minB, maxB - minA) < dminB)//refresh 
		{
			dminB = std::min(maxA - minB, maxB - minA); // intersect meshs, dmin is positive
			direction = matB.rotation() * normal;
		}
	}
	return { std::min(dminA, dminB) * direction, indexAB };
}

// for inner depth
//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> _getPenetrationDepthOfTwoConvexBOX(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	Eigen::AlignedBox3d boxA = meshA.bounding_;
//	Eigen::AlignedBox3d boxB = meshB.bounding_;
//	// no contain sequence
//	double dmin = DBL_MAX;
//	Vector3d coord;
//	size_t index;
//	Triangle unitThree = { Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1) };
//	for (int i = 0; i < 3; i++)
//	{
//		double dtemp = std::min(boxA.max()[i] - boxB.min()[i], boxB.max()[i] - boxA.min()[i]); //x direction
//		if (dtemp < dmin)
//		{
//			dmin = dtemp;
//			coord = unitThree[i]; //Eigen::Vector3d::UnitX();
//			index = i;
//		}
//	}
//	return { dmin * coord,{index, ULLONG_MAX} };
//}

//simplify
Eigen::Vector3d _getPenetrationDepthOfTwoMeshsBOX(const ModelMesh& meshA, const ModelMesh& meshB)
{
	Eigen::AlignedBox3d boxA = meshA.bounding_;
	Eigen::AlignedBox3d boxB = meshB.bounding_;
	// no contain sequence
	double dmin = DBL_MAX;
	Vector3d coord;
	Triangle unitThree = { Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1) }; //choose one direction from xyz
	for (int i = 0; i < 3; i++)
	{
		double dtemp = std::min(boxA.max()[i] - boxB.min()[i], boxB.max()[i] - boxA.min()[i]); //x direction
		if (dtemp < dmin)
		{
			dmin = dtemp;
			coord = unitThree[i]; //Eigen::Vector3d::UnitX();
		}
	}
	return dmin * coord;
}

// for intrusive meshs, using major method SAT, return normal vector and face index
std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfTwoConvex(const ModelMesh& meshA, const ModelMesh& meshB,
	const set<size_t>& faceSetA, const set<size_t>& faceSetB, const vector<size_t>& vertexVectA, const vector<size_t>& vertexVectB)
{
	if (faceSetA.empty() && faceSetB.empty())
        return { Vector3d::Zero(), {ULLONG_MAX, ULLONG_MAX} };
	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
	const std::vector<std::array<int, 3>>& iboA = meshA.ibo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	const std::vector<std::array<int, 3>>& iboB = meshB.ibo_;
	const Eigen::Affine3d& matA = meshA.pose_;
	const Eigen::Affine3d& matB = meshB.pose_;
	Eigen::Affine3d matIAB = matA.inverse() * matB;
	Eigen::Affine3d matIBA = matB.inverse() * matA;
	std::set<size_t> vboSetA, vboSetB; // to remove repeat vertex
	for (const auto& iterA : faceSetA) //merge vertex
	{
		vboSetA.insert(iboA[iterA][0]);
		vboSetA.insert(iboA[iterA][1]);
		vboSetA.insert(iboA[iterA][2]);
#ifdef STATISTIC_DATA_RECORD
		//output calculate intersect meshs triangless
		triFaceVctA.push_back({ matA * vboA[iboA[iterA][0]], matA * vboA[iboA[iterA][1]], matA * vboA[iboA[iterA][2]] }); //world coord
#endif
	}
	for (const auto& iterB : faceSetB)
	{
		vboSetB.insert(iboB[iterB][0]);
		vboSetB.insert(iboB[iterB][1]);
		vboSetB.insert(iboB[iterB][2]);
#ifdef STATISTIC_DATA_RECORD
		triFaceVctB.push_back({ matB * vboB[iboB[iterB][0]], matB * vboB[iboB[iterB][1]], matB * vboB[iboB[iterB][2]] });
#endif
	}
	// append all vertex
	for (const auto& iterA : vertexVectA)
		vboSetA.insert(iterA);
	for (const auto& iterB : vertexVectB)
		vboSetB.insert(iterB);
#ifdef STATISTIC_DATA_RECORD
	for (const auto& iA : vertexVectA)
		triVertexVctA.push_back(matA * vboA[iA]);
	for (const auto& iB : vertexVectB)
		triVertexVctB.push_back(matB * vboB[iB]);
#endif
#ifdef STATISTIC_DATA_RECORD
	bool isFixedFaceA = true;
#endif
	double dminA = DBL_MAX, dminB = DBL_MAX;// , dminA_V, dminB_V, tmpMaxA, tmpMaxB;
	double minA, maxA, minB, maxB, projection, dtemp;//refresh min and max
	Eigen::Vector3d directionA, directionB, normal;// directionA_V, directionB_V;
	// the minimum distance must on the direction of face normal
	size_t indexA = ULLONG_MAX, indexB = ULLONG_MAX;// indexA_V = ULLONG_MAX, indexB_V = ULLONG_MAX;
	//std::array<size_t, 2> indexAB = { ULLONG_MAX , ULLONG_MAX };
	// calculate depth using SAT
	for (const auto& iA : faceSetA) // iterate every faceA
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		normal = (vboA[iboA[iA][1]] - vboA[iboA[iA][0]]).cross(vboA[iboA[iA][2]] - vboA[iboA[iA][1]]).normalized();
		for (const auto& vertexA : vboSetA)
		{
			projection = normal.dot(vboA[vertexA]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboSetB)
		{
			projection = normal.dot(matIAB * vboB[vertexB]);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		dtemp = std::min(maxA - minB, maxB - minA);
		if (dtemp < dminA)// the intersect mesh-part prejection segment
		{
			dminA = dtemp; //always positive, but impacted by accuracy, forexample cross edges and normalized
			directionA = matA.rotation() * normal;
			indexA = iA;
#ifdef STATISTIC_DATA_RECORD
			if (dminA < 0.0)
				count_error_tris_sepa++;
			if (maxA - minB > maxB - minA)
				isFixedFaceA = false;
#endif
		}
		//dminA_V = 0.0;
		//tmpMaxA = 0.0;
		//origin = normal.dot(vboA[iboA[iA][0]]); // relative projection value
		//for (const auto& iB : vertexVectB) // the inside vertex's prejection
		//{
		//	projection = normal.dot(matIAB * vboB[iB]) - origin;
		//	if (projection < dminA_V)
		//	{
		//		dminA_V = projection; // always negative
		//		directionA_V = matA.rotation() * normal;
		//		indexA_V = iA;
		//	}
		//	if (!meshA.convex_ && tmpMaxA < projection) // the direction not exact
		//		tmpMaxA = projection;
		//}
		//if (dminA < tmpMaxA - dminA_V) // choose max, without vertex, dminV is zero
		//{
		//	dminA = tmpMaxA - dminA_V;
		//	directionA = directionA_V;
		//	indexA = indexA_V;
		//}
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
			projection = normal.dot(matIBA * vboA[vertexA]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboSetB)
		{
			projection = normal.dot(vboB[vertexB]);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		dtemp = std::min(maxA - minB, maxB - minA);
		if (dtemp < dminB)//refresh 
		{
			dminB = dtemp;
			directionB = matB.rotation() * normal;
			indexB = iB;
#ifdef STATISTIC_DATA_RECORD
			if (dminB < 0.0)
				count_error_tris_sepa++;
			if (maxA - minB < maxB - minA)
				isFixedFaceA = false;
#endif
		}
		//dminB_V = 0.0;
		//tmpMaxB = 0.0;
		//origin = normal.dot(matA * vboB[iboB[iB][0]]); // relative projection value
		//for (const auto& iA : vertexVectA) // the inside vertex's prejection
		//{
		//	projection = normal.dot(matA * vboA[iA]) - origin;
		//	if (projection < dminB_V)
		//	{
		//		dminB_V = projection;  // always negative
		//		directionB_V = matB.rotation() * normal;
		//		indexB_V = iB;
		//	}
		//	if (!meshB.convex_ && tmpMaxB < projection) // the direction not exact
		//		tmpMaxB = projection;
		//}
		//if (dminB < tmpMaxB - dminB_V) // choose max
		//{
		//	dminB = tmpMaxB - dminB_V;
		//	directionB = directionB_V;
		//	indexB = indexB_V;
		//}
	}
	//if (!isFixedFaceA)
	//	direction = -direction;
	return (dminA < dminB) ?
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ dminA* directionA, { indexA, ULLONG_MAX }} :
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ dminB * directionB, { ULLONG_MAX, indexB } };
}

// calculate depth self with tiny errors
Eigen::Vector3d getPenetrationDepthOfTwoMeshsParts(const ModelMesh& meshA, const ModelMesh& meshB, const std::vector<Vector3d>& axesSepa,
	const std::set<size_t>& vboSetA, const std::set<size_t>& vboSetB)
{
	if (axesSepa.empty() || vboSetA.empty() || vboSetB.empty())
		return Vector3d::Zero();
	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	//const std::vector<std::array<int, 3>>& iboA = meshA.ibo_;
	//const std::vector<std::array<int, 3>>& iboB = meshB.ibo_;
	const Eigen::Affine3d& matA = meshA.pose_;
	const Eigen::Affine3d& matB = meshB.pose_;
	double dmin = DBL_MAX;// dminB = DBL_MAX;
	double minA, maxA, minB, maxB, projection, dtemp;//refresh min and max
	Eigen::Vector3d direction; //iteration
	// calculate depth using SAT
	for (const auto& axis : axesSepa) // every axis isnot zero-vector
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertexA : vboSetA)
		{
			projection = axis.dot(matA * vboA[vertexA]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertexB : vboSetB)
		{
			projection = axis.dot(matB * vboB[vertexB]);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		dtemp = std::min(maxA - minB, maxB - minA);
#ifdef STATISTIC_DATA_COUNT
		if (dtemp < 0.0)
			count_err_inter_mesh_sepa++;
#endif  
		if (dtemp < dmin)// the intersect mesh-part prejection segment
		{
			dmin = dtemp; //always positive, but impacted by accuracy, forexample cross edges and normalized
			direction = axis;
		}
	}
	return dmin * direction;
}

//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> getPenetrationDepthOfVertexAndFace(const ModelMesh& meshA, const ModelMesh& meshB, 
//	const vector<size_t>& faceVectA, const vector<size_t>& faceVectB, const vector<size_t>& vertexVectA, const vector<size_t>& vertexVectB)
//{
//	if ((faceVectA.empty() || vertexVectB.empty()) && (faceVectB.empty() || vertexVectA.empty())) // any empty cause zero
//		return { gVecZero, { ULLONG_MAX , ULLONG_MAX } }; // or gVecNaN separate
//	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
//	const std::vector<std::array<int, 3>>& iboA = meshA.ibo_;
//	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
//	const std::vector<std::array<int, 3>>& iboB = meshB.ibo_;
//	const Eigen::Affine3d& matA = meshA.pose_;
//	const Eigen::Affine3d& matB = meshB.pose_;
//	Eigen::Affine3d matIAB = matA.inverse() * matB;
//	Eigen::Affine3d matIBA = matB.inverse() * matA;
//	Eigen::Vector3d directionA, directionB, normal;
//	double dminA = 0, dminB = 0, projection, origin, tmpMaxA = 0, tmpMaxB = 0;
//	size_t indexA = ULLONG_MAX, indexB = ULLONG_MAX;
//#ifdef STATISTIC_DATA_RECORD
//	set<double> projectionA, projectionB;
//#endif
//	for (const auto& iA : faceVectA) // the face normal direction always outwards
//	{
//		if (vertexVectB.empty()) //cause no distance
//		{
//			dminA = -DBL_MAX;
//			break;
//		}
//		normal = (vboA[iboA[iA][1]] - vboA[iboA[iA][0]]).cross(vboA[iboA[iA][2]] - vboA[iboA[iA][1]]).normalized();
//		origin = normal.dot(vboA[iboA[iA][0]]); // relative projection value
//		for (const auto& iB : vertexVectB)
//		{
//			//projection = normal.dot(matB * vboB[iB] - vboA[iboA[iA][0]]);
//			projection = normal.dot(matIAB * vboB[iB]) - origin;
//#ifdef STATISTIC_DATA_RECORD
//			projectionA.insert(projection);
//#endif
//			if (projection < dminA) // always negative
//			{
//				dminA = projection;
//				directionA = matA.rotation() * normal;
//				indexA = iA;
//			}
//			if (!meshA.convex_ && tmpMaxA < projection) // the direction not exact
//			{
//				tmpMaxA = projection;
//			}
//		}
//	}
//	for (const auto& iB : faceVectB) // the face normal direction always outwards
//	{
//		if (vertexVectA.empty())
//		{
//			dminB = -DBL_MAX;
//			break;
//		}
//		normal = (vboB[iboB[iB][1]] - vboB[iboB[iB][0]]).cross(vboB[iboB[iB][2]] - vboB[iboB[iB][1]]).normalized();
//		origin = normal.dot(vboB[iboB[iB][0]]);
//		for (const auto& iA : vertexVectA)
//		{
//			projection = normal.dot(matIBA * vboA[iA]) - origin;
//#ifdef STATISTIC_DATA_RECORD
//			projectionB.insert(projection);
//#endif
//			if (projection < dminB)
//			{
//				dminB = projection;
//				directionB = matB.rotation() * normal;
//				indexB = iB;
//			}
//			if (!meshA.convex_ && tmpMaxB < projection) // the direction not exact
//			{
//				tmpMaxB = projection;
//			}
//		}
//	}
//	return (tmpMaxA - dminA < tmpMaxB - dminB) ? //return min
//		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ (tmpMaxA - dminA) * directionA, { indexA, ULLONG_MAX }} :
//		std::tuple<Eigen::Vector3d, std::array<size_t, 2>>{ (tmpMaxB - dminB) * directionB, { ULLONG_MAX, indexB } };
//}

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
	//Vector3d toleSize = Vector3d(tolerance + epsF, tolerance + epsF, tolerance + epsF);
	Vector3d toleSize = Vector3d(tolerance, tolerance, tolerance);
	Eigen::AlignedBox3d boxMag(box.min() - toleSize, box.max() + toleSize);
	std::vector<size_t> triA_Index; // using index of mesh IBO
	Eigen::AlignedBox3d triA_Box; // iterate to lessen box
	std::array<Eigen::Vector3d, 3> triIter;
	for (size_t i = 0; i < meshA.ibo_.size(); ++i)
	{
		triIter = { // input matrix to avoid repeat calculate
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
		triIter = { meshB.pose_ * meshB.vbo_[meshB.ibo_[j][0]],
					meshB.pose_ * meshB.vbo_[meshB.ibo_[j][1]],
					meshB.pose_ * meshB.vbo_[meshB.ibo_[j][2]] };
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
	triA_Box.min() -= toleSize;
	triA_Box.max() += toleSize;
	triB_Box.min() -= toleSize;
	triB_Box.max() += toleSize;
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
	//Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	Eigen::Affine3d relative_matrix = meshB.pose_.inverse() * meshA.pose_; //without revise
	// get the index param of mesh's ibo
	std::array<std::vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(meshA, meshB, 0.0);
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		std::array<Eigen::Vector3d, 3> triA, triB;
		for (const auto& iA : indexAB[0])
		{
			triA = { 
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
			for (const auto& iB : indexAB[1])
			{
				triB = { 
					meshB.vbo_[meshB.ibo_[iB][0]],
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
					interTriInfoList.push_back({ { triA, triB }, {}, std::nan("0") }); // only no-depth call, default distance is nan
#endif
					return true;
				}
			}
		}
	}
#ifdef STATISTIC_DATA_COUNT
	if (meshA.vbo_.empty() || meshB.vbo_.empty()) // extra safe check
	{
		count_err_empty_mesh++;
		//return false;
	}
#endif  
	//judge whether mesh entirely inside mesh
	if (meshA.bounding_.contains(meshB.bounding_)) //boundingbox is world coord
	{
		// precondition: boundingbox inside, polyface not intersect, mesh isnot empty
		if (!meshB.vbo_.empty() && isPointInsidePolyhedronROT(meshB.pose_ * meshB.vbo_[0], meshA) != RelationOfPointAndMesh::OUTER)
		{
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			triRecordHard.push_back({ gTirNaN, gTirNaN }); //two trinan means inside
			interTriInfoList.push_back({ { gTirNaN, gTirNaN }, {}, 0.0 });
#endif
			return true;
		}
	}
	else if (meshB.bounding_.contains(meshA.bounding_))
	{
		if (!meshA.vbo_.empty() && isPointInsidePolyhedronROT(meshA.pose_ * meshA.vbo_[0], meshB) != RelationOfPointAndMesh::OUTER)
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
#define USING_ALL_SEPARATE_AXES_FOR_DEPTH
std::tuple<RelationOfTwoMesh, Eigen::Vector3d> getTwoMeshsIntersectRelation(const ModelMesh& meshA, const ModelMesh& meshB)
{
	//Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	Eigen::Affine3d relative_matrix = meshB.pose_.inverse() * meshA.pose_;
	// get the index param of mesh's ibo
	std::array<std::vector<size_t>, 2> indexAB = _getReducedIntersectTrianglesOfMesh(meshA, meshB, 0.0);
	bool isContact = false; // vertex or edge or face contact
	bool isIntrusive = false;
#ifndef USING_ALL_SEPARATE_AXES_FOR_DEPTH
	std::set<size_t> faceSetA, faceSetB; // intersect and inside, to remove repeat
	std::vector<size_t> vertexVectA, vertexVectB;
#endif
	// all axes version
	std::vector<Vector3d> axesSepa; // in world coordinate
	std::set<size_t> vboSetA, vboSetB; // to remove repeat vertex
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		std::array<Eigen::Vector3d, 3> triA, triB;
		std::array<Eigen::Vector3d, 2> pInter;
		std::array<Eigen::Vector3d, 3> edgesA, edgesB;
		Eigen::Vector3d normalA, normalB;
		std::array<Eigen::Vector3d, 11> axesPoten; //remove 6 edge normal
#ifdef STATISTIC_DATA_RECORD
		for (const auto& i : indexAB[0]) //for test
		{
			Triangle triIter = {
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][0]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][1]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][2]] };
			triFaceVctA.push_back(triIter); //world coord
		}
		for (const auto& j : indexAB[1])
		{
			Triangle triIter = {
				meshB.pose_ * meshB.vbo_[meshB.ibo_[j][0]],
				meshB.pose_ * meshB.vbo_[meshB.ibo_[j][1]],
				meshB.pose_ * meshB.vbo_[meshB.ibo_[j][2]] };
			triFaceVctB.push_back(triIter); //world coord
		}
#endif
		for (const auto& iA : indexAB[0]) //faces intersect
		{
			triA = { 
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
			for (const auto& iB : indexAB[1])
			{
				triB = { 
					meshB.vbo_[meshB.ibo_[iB][0]],
					meshB.vbo_[meshB.ibo_[iB][1]],
					meshB.vbo_[meshB.ibo_[iB][2]] };
#ifdef STATISTIC_DATA_RECORD
				if (!isTwoTrianglesBoundingBoxIntersect(triA, triB))
					count_tri_box_exclude_pre++;
#endif  
				if (!isTwoTrianglesBoundingBoxIntersect(triA, triB) || !isTwoTrianglesIntersectSAT(triA, triB)) // second pre-judge, merge
					continue;
				isContact = true; //isIntersect// maybe intrusive or atleast contact
				//intersect
				//if ((triA[1] - triA[0]).cross(triA[2] - triA[1]).cross((triB[1] - triB[0]).cross(triB[2] - triB[1])).isZero(epsF)) // intersect-coplanar judge
				//{
				//	continue;
				//}
				//faceSetA.insert(iA); //faceVectA.push_back(iA);
				//faceSetB.insert(iB); //faceVectB.push_back(iB);
#ifdef USING_ALL_SEPARATE_AXES_FOR_DEPTH
				// add the separate axis
				//RelationOfTwoTriangles relation = getRelationOfTwoTrianglesSAT(triA, triB);
				if (RelationOfTwoTriangles::INTRUSIVE != getRelationOfTwoTrianglesSAT(triA, triB))
					continue;
				isIntrusive = true;
				// add the faces vertex index
				for (int i = 0; i < 3; ++i)
				{
					vboSetA.insert(meshA.ibo_[iA][i]);
					vboSetB.insert(meshB.ibo_[iB][i]);
				}
				// caution, both mesh with matrix, axis direciton in self local coordinate, need transform to world
				edgesA = { 
					meshA.pose_ * (triA[1] - triA[0]),
					meshA.pose_ * (triA[2] - triA[1]),
					meshA.pose_ * (triA[0] - triA[2]) };
				edgesB = { 
					meshB.pose_ * (triB[1] - triB[0]),
					meshB.pose_ * (triB[2] - triB[1]),
					meshB.pose_ * (triB[0] - triB[2]) };
				normalA = edgesA[0].cross(edgesA[1]); //normalA,
				normalB = edgesB[0].cross(edgesB[1]); //normalB,
				axesPoten = { {
					normalA,
					normalB,
					edgesA[0].cross(edgesB[0]),
					edgesA[0].cross(edgesB[1]),
					edgesA[0].cross(edgesB[2]),
					edgesA[1].cross(edgesB[0]),
					edgesA[1].cross(edgesB[1]),
					edgesA[1].cross(edgesB[2]),
					edgesA[2].cross(edgesB[0]),
					edgesA[2].cross(edgesB[1]),
					edgesA[2].cross(edgesB[2]) } };
				for (auto& axis : axesPoten) // revise zero vector
				{
					if (axis.isZero(epsF)) // clean data
						axis = Vector3d(1, 0, 0);
				}
				for (const auto& axis : axesPoten)
				{
                    bool isExist = false; //to remove repeat
					for (const auto& iter : axesSepa)
					{
						if (iter.cross(axis).isZero(epsF))
						{
#ifdef STATISTIC_DATA_COUNT
							count_repeat_sepa_axis++;
#endif	
							isExist = true;
							break;
						}
					}
					if (!isExist)
					{
#ifdef STATISTIC_DATA_COUNT
						count_cal_sepa_axis++;
#endif	
						axesSepa.push_back(axis.normalized());
					}
				}
#else
				pInter = getTwoTrianglesIntersectPoints(triA, triB);
#ifdef STATISTIC_DATA_COUNT
				if (isnan(pInter[0][0]) && isnan(pInter[1][0])) //triangles intersect but no intersect points
					count_triAB_nan++;
				if (!isnan(pInter[0][0]) && isnan(pInter[1][0]))
					count_triB_nan++;
#endif  
				// only intrusive triangle insert, exclude contact triangle
				if (!isnan(pInter[0][0]) && !isnan(pInter[1][0]) && !(pInter[1] - pInter[0]).isZero(epsF) && // also using epsF
					((pInter[0] != triA[0] && pInter[0] != triA[1] && pInter[0] != triA[2] && pInter[0] != triB[0] && pInter[0] != triB[1] && pInter[0] != triB[2]) ||
						(pInter[1] != triA[0] && pInter[1] != triA[1] && pInter[1] != triA[2] && pInter[1] != triB[0] && pInter[1] != triB[1] && pInter[1] != triB[2])))
				{
					//another case, two triangle edge collinear, using triangle vertex coincident judge
					faceSetA.insert(iA);
					faceSetB.insert(iB);
				}
#endif //USING_ALL_SEPARATE_AXES_FOR_DEPTH
			}
		}
	}
#ifdef STATISTIC_DATA_COUNT
	if (isContact)
		count_meshs_isContact++;
	if (isIntrusive)
		count_meshs_isIntrusive++;
#endif	
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
#ifdef USING_ALL_SEPARATE_AXES_FOR_DEPTH
			if (vboSetA.find(i) == vboSetA.end() && RelationOfPointAndMesh::INNER == isPointInsidePolyhedronROT(meshA.pose_ * meshA.vbo_[i], meshB))
				vboSetA.insert(i);
#else
			if (isPointInsidePolyhedronFL(meshA.pose_ * meshA.vbo_[i], meshB))
				vertexVectA.push_back(i);
#endif // USING_ALL_SEPARATE_AXES_FOR_DEPTH
		}
		for (size_t i = 0; i < meshB.vbo_.size(); ++i)
		{
#ifdef USING_ALL_SEPARATE_AXES_FOR_DEPTH
			if (vboSetB.find(i) == vboSetB.end() && RelationOfPointAndMesh::INNER == isPointInsidePolyhedronROT(meshB.pose_ * meshB.vbo_[i], meshA))
				vboSetB.insert(i);
#else
			if (isPointInsidePolyhedronFL(meshB.pose_ * meshB.vbo_[i], meshA))
				vertexVectB.push_back(i);
#endif // USING_ALL_SEPARATE_AXES_FOR_DEPTH
		}
#ifdef USING_ALL_SEPARATE_AXES_FOR_DEPTH
		Eigen::Vector3d depth = getPenetrationDepthOfTwoMeshsParts(meshA, meshB, axesSepa, vboSetA, vboSetB);
#else
		std::tuple<Eigen::Vector3d, std::array<size_t, 2>> depth = getPenetrationDepthOfTwoConvex(meshA, meshB, faceSetA, faceSetB, vertexVectA, vertexVectB);
#endif
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
#ifndef USING_ALL_SEPARATE_AXES_FOR_DEPTH
		Triangle trigon;
		if (std::get<1>(depth)[0] == ULLONG_MAX && std::get<1>(depth)[1] == ULLONG_MAX) // means no distance, generally due to surface contact
			interTriInfoList.push_back({ { std::move(gTirNaN), std::move(gTirNaN) }, {}, -std::get<0>(depth).norm() });
		else if (std::get<1>(depth)[0] != ULLONG_MAX) //bool isLeft 
		{
			trigon = { 
				meshA.pose_ * meshA.vbo_[meshA.ibo_[get<1>(depth)[0]][0]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[get<1>(depth)[0]][1]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[get<1>(depth)[0]][2]] };
			interTriInfoList.push_back({ { std::move(trigon), std::move(gTirNaN) }, {}, -std::get<0>(depth).norm() });
		}
		else
		{
			trigon = { 
				meshB.pose_ * meshB.vbo_[meshB.ibo_[get<1>(depth)[1]][0]],
				meshB.pose_ * meshB.vbo_[meshB.ibo_[get<1>(depth)[1]][1]],
				meshB.pose_ * meshB.vbo_[meshB.ibo_[get<1>(depth)[1]][2]] };
			interTriInfoList.push_back({ { std::move(gTirNaN), std::move(trigon) }, {}, -std::get<0>(depth).norm() });
		}
#endif // !USING_ALL_SEPARATE_AXES
#endif//STATISTIC_DATA_RECORD
#ifdef USING_ALL_SEPARATE_AXES_FOR_DEPTH
		return { RelationOfTwoMesh::INTRUSIVE , depth };
#else
		return { RelationOfTwoMesh::INTRUSIVE , std::get<0>(depth) };
#endif
	}
	//judge whether mesh entirely inside mesh 
	RelationOfPointAndMesh relation = RelationOfPointAndMesh::OUTER; //to avoid wholly enfold, exactly calculate depth
	if (meshA.bounding_.contains(meshB.bounding_)) //boundingbox is world coord
	{
		// precondition: boundingbox inside, polyface not intersect, mesh isnot empty
		for (const auto& iterB : meshB.vbo_)
		{
			relation = isPointInsidePolyhedronROT(meshB.pose_ * iterB, meshA);
			if (RelationOfPointAndMesh::SURFACE != relation)
				break;
		}
		if (RelationOfPointAndMesh::OUTER != relation)//all-inner or contact-inner or all-contact
		{
			//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> depth = _getPenetrationDepthOfTwoConvexBOX(meshA, meshB);
			Eigen::Vector3d depth = _getPenetrationDepthOfTwoMeshsBOX(meshA, meshB);
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			std::array<Triangle, 3> triThree = { gTriYOZ ,gTriXOZ ,gTriXOY };
			interTriInfoList.push_back({ { std::move(triThree[0]), std::move(gTirNaN)}, {}, -depth.norm() });
#endif
#ifdef STATISTIC_DATA_COUNT
			count_mesh_inside_mesh++;
#endif	
			return isContact ? //equal some surface
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA_CONT, depth } :
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA , depth };
		}
	}
	else if (meshB.bounding_.contains(meshA.bounding_))
	{
		for (const auto& iterA : meshA.vbo_)
		{
			relation = isPointInsidePolyhedronROT(meshA.pose_ * iterA, meshB);
			if (RelationOfPointAndMesh::SURFACE != relation)
				break;
		}
		if (RelationOfPointAndMesh::OUTER != relation)
		{
			//std::tuple<Eigen::Vector3d, std::array<size_t, 2>> depth = _getPenetrationDepthOfTwoConvexBOX(meshA, meshB);
			Eigen::Vector3d depth = _getPenetrationDepthOfTwoMeshsBOX(meshA, meshB);
#ifdef STATISTIC_DATA_RECORD //record all trigon-intersect
			std::array<Triangle, 3> triThree = { gTriYOZ ,gTriXOZ ,gTriXOY };
			interTriInfoList.push_back({ { std::move(triThree[0]), std::move(gTirNaN)}, {}, -depth.norm() });
#endif
#ifdef STATISTIC_DATA_COUNT
			count_mesh_inside_mesh++;
#endif		
			return isContact ?
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA_CONT, depth } :
				tuple<RelationOfTwoMesh, Vector3d>{ RelationOfTwoMesh::INSEDE_BINA , depth };
		}
	}
	if (isContact)
	{
		return { RelationOfTwoMesh::CONTACT_OUTER, Vector3d::Zero() };
	}
	return { RelationOfTwoMesh::SEPARATE, Vector3d::Zero() };
}

// ClashSoft // return index of mesh_a and mesh_b ibo
std::tuple<double, std::array<size_t, 2>> getTwoMeshsSeparationDistanceSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance)
{
#ifdef STATISTIC_DATA_RECORD
	std::array<std::array<Eigen::Vector3d, 3>, 2> triDistPair;
#endif    
	//Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	Eigen::Affine3d relative_matrix = meshB.pose_.inverse() * meshA.pose_; //without revise
	// distance > tolerance, return double-max, to decrease calculate
	double d = DBL_MAX; // the res
	std::array<size_t, 2> index = { ULLONG_MAX, ULLONG_MAX };
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

//---------------------------------------------------------------------------
//  penetration depth
//---------------------------------------------------------------------------

//#ifdef USING_DISCARD_FUNCTION
// only for convex polytope 
//bool isTwoMeshsIntersectGJK(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	return false;
//}
//
//// only for convex polytope
//double getTwoMeshsPenetrationDepthEPA(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	return 0.0;
//}
//
////manual body boolean
//vector<Polyhedron> _getBooleanIntersectOfTwoMesh(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	vector<Polyhedron> interBodys;
//	return {};
//}
//
//Eigen::Vector3d _getPartialDepthtOfTwoConcave(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	return {};
//}
//
//// fixed mehsB, move distance of meshA
//Eigen::Vector3d getPenetrationDepthOfTwoMeshs(const ModelMesh& meshA, const ModelMesh& meshB)
//{
//	//must intersect
//	Eigen::Vector3d direction = gVecNaN;
//	return direction;
//}
//
//double getMoveDistanceOfAssignedDirection(const ModelMesh& meshA, const ModelMesh& meshB, const Eigen::Vector3d& direction)
//{
//	return 0;
//}

// interconversion
HeMesh::HeMesh(const ModelMesh& mesh)
{
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	HeMesh heMesh;
	for (int i = 0; i < mesh.vbo_.size(); ++i)
	{
		HeVertex* heVertex = new HeVertex;
		heVertex->m_index = i;
		heVertex->m_coord= mesh.vbo_[i];
		heMesh.m_vertexes.push_back(heVertex);
	}
	int countEdge = 0;
	map<array<int, 2>, int> uniqueEdge;
	for (int i = 0; i < mesh.ibo_.size(); ++i)
	{
		HeFace* heFace = new HeFace;
		heFace->m_index = i;
		//heFace->m_normal = (v1 - v0).cross(v2 - v1); // with normalized
		heFace->m_normal = (vbo[ibo[i][1]] - vbo[ibo[i][0]]).cross(vbo[ibo[i][2]] - vbo[ibo[i][1]]);
		int firstEdge = countEdge;
		for (int j = 0; j < 3; ++j) //create 3 edges from 1 face
		{
			HeEdge* heEdge = new HeEdge;
			heEdge->m_index = countEdge;
			heEdge->m_incFace = heFace;
			heEdge->m_oriVertex = m_vertexes[ibo[i][j]];
			m_edges.push_back(heEdge);
			countEdge++;
		}
		array<int, 4> face = { ibo[i][0], ibo[i][1], ibo[i][2], ibo[i][0] };
		for (int j = 0; j < 3; ++j) // add adjacent relation after create edges
		{
			//m_prevEdge
			int prev = j - 1 < 0 ? 2 : j - 1;
			m_edges[firstEdge + j]->m_prevEdge = m_edges[firstEdge + prev];
			//m_nextEdge
			int next = j + 1 > 2 ? 0 : j + 1;
			m_edges[firstEdge + j]->m_nextEdge = m_edges[firstEdge + next];
			//m_twinEdge
			array<int, 2> edge = (face[j] < face[j + 1]) ?
				array<int, 2>{face[j], face[j + 1]} :
				array<int, 2>{face[j + 1], face[j]};
			if (uniqueEdge.find(edge) == uniqueEdge.end())
				uniqueEdge.emplace(edge, firstEdge + j);
			else //means twinEdge added
			{
				m_edges[firstEdge + j]->m_twinEdge = m_edges[uniqueEdge[edge]];
				m_edges[uniqueEdge[edge]]->m_twinEdge = m_edges[firstEdge + j];
			}
		}
		heFace->m_incEdge = m_edges[firstEdge];
		m_faces.push_back(heFace);
	}
}

ModelMesh HeMesh::toMesh() const
{
	ModelMesh mesh;
	std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	std::vector<std::array<int, 3>>& ibo = mesh.ibo_;
	for (const auto& v : m_vertexes)
		vbo.push_back(v->m_coord);
	for (const auto& iter : m_faces)
		ibo.push_back(iter->ibo());
	return mesh;
}

HeMesh::operator ModelMesh() const
{
	return toMesh();
}

void HeMesh::clear()
{
	//Reset();
	for (size_t i = 0; i != m_vertexes.size(); ++i)
	{
		if (m_vertexes[i])
		{
			delete m_vertexes[i];
			m_vertexes[i] = nullptr;
		}
	}
	for (size_t i = 0; i != m_edges.size(); ++i)
	{
		if (m_edges[i])
		{
			delete m_edges[i];
			m_edges[i] = nullptr;
		}
	}
	for (size_t i = 0; i != m_faces.size(); ++i)
	{
		if (m_faces[i])
		{
			delete m_faces[i];
			m_faces[i] = nullptr;
		}
	}
	m_vertexes.clear();
	m_edges.clear();
	m_faces.clear();
}

bool HeMesh::isValid() const //is mainfold mesh
{
	// without nullptr
	for (const auto& iter : m_vertexes)
	{
		if (iter->m_index == -1 || iter->m_incEdge == nullptr || std::isnan(iter->m_coord[0]))
			return false;
	}
	for (const auto& iter : m_edges)
	{
		if (iter->m_index == -1 || !iter->m_oriVertex || !iter->m_twinEdge || !iter->m_prevEdge || !iter->m_nextEdge || !iter->m_incFace)
			return false;
		if (iter->m_twinEdge->m_oriVertex != iter->m_nextEdge->m_oriVertex ||
			iter->m_twinEdge->m_nextEdge->m_oriVertex != iter->m_oriVertex)
			return false;
	}
	for (const auto& iter : m_faces)
	{
		if (iter->m_index == -1 || iter->m_incEdge == nullptr || iter->m_normal.isZero())
			return false;
		// other methods
	}

	return true;
}

// the diagonal vertex of face | the surround vertex of vertex
tuple<vector<std::array<int, 3>>, vector<set<int>>> _getMeshVertexLinkedInfo(const ModelMesh& mesh)
{
	//get diagonal vertex index of each face-edge
	vector<std::array<int, 3>> edgeDiag(mesh.ibo_.size(), { -1,-1,-1 }); //init
	// generate new middle vertex
	for (int i = 0; i < mesh.ibo_.size(); ++i)
	{
		if (edgeDiag[i][0] != -1 && edgeDiag[i][1] != -1 && edgeDiag[i][2] != -1) //has been assemble
			continue;
		std::array<int, 3> faceA = mesh.ibo_[i]; //copy
		std::sort(faceA.begin(), faceA.end());
		int find = 0;
		for (int j = 0; j < mesh.ibo_.size(); ++j)
		{
			if (i >= j) //edgeDiag[j] been revised
				continue;
			std::array<int, 3> faceB = mesh.ibo_[j]; //copy
			std::sort(faceB.begin(), faceB.end());
			if (faceB[2] < faceA[0] || faceB[0]> faceA[2]) //bound box
				continue;
			int coin = 0;
			for (const int vtB : faceB) //find two common vertex
			{
				if (vtB == faceA[0] || vtB == faceA[1] || vtB == faceA[2])
					coin++;
			}
			if (coin == 2) //means common edge
			{
				int diaA, diaB; // diagonal vertex index
				if (faceA[0] != faceB[0] && faceA[0] != faceB[1] && faceA[0] != faceB[2])
					diaA = faceA[0];
				else if (faceA[1] != faceB[0] && faceA[1] != faceB[1] && faceA[1] != faceB[2])
					diaA = faceA[1];
				else
					diaA = faceA[2];
				if (faceB[0] != faceA[0] && faceB[0] != faceA[1] && faceB[0] != faceA[2])
					diaB = faceB[0];
				else if (faceB[1] != faceA[0] && faceB[1] != faceA[1] && faceB[1] != faceA[2])
					diaB = faceB[1];
				else
					diaB = faceB[2];
				int indexI, indexJ; // faces origin index
				if (diaA == mesh.ibo_[i][0])
					indexI = 1;
				else if (diaA == mesh.ibo_[i][1])
					indexI = 2;
				else
					indexI = 0;
				if (diaB == mesh.ibo_[j][0])
					indexJ = 1;
				else if (diaB == mesh.ibo_[j][1])
					indexJ = 2;
				else
					indexJ = 0;
				//record
				edgeDiag[i][indexI] = diaB;
				edgeDiag[j][indexJ] = diaA;
				find++;
			}
			if (find == 3)
				break;
		}
	}
	// update origin vertex
	vector<set<int>> roundVct(mesh.vbo_.size());
	for (int i = 0; i < mesh.vbo_.size(); ++i)
	{
		set<int> round;// vector<Vector3d> round;
		for (const auto& face : mesh.ibo_)
		{
			if (face[0] != i && face[1] != i && face[2] != i)
				continue;
			for (int j = 0; j < 3; j++)
			{
				if (face[j] != i)
					round.insert(face[j]);
			}
		}
		roundVct[i] = round;
	}
	return { edgeDiag, roundVct };
}

// every new vertex
Vector3d _getNewVertex(const Vector3d& A, const Vector3d& B, const Vector3d& C, const Vector3d& D)
{
	// AB on edge, CD is diagonal
	return 0.375 * (A + B) + 0.125 * (C + D);// 3/8*(A+B)+1/8*(A+D)
}

// update old vertex
Vector3d _updateOldVertex(const Vector3d& origin, const vector<Vector3d>& round)
{
	size_t n = round.size(); //vertex degree
	double u = (n == 3) ? 0.1875 : 3.0 / (8 * n); //double
	Vector3d sum = Vector3d::Zero(); //neighbor position sum
	for (const auto& iter : round)
		sum += iter;
	return (1 - n * u) * origin + u * sum;
}

// practice of games101
ModelMesh games::meshLoopSubdivision(const ModelMesh& mesh)
{
	// invent wheel and optimize wheel
	ModelMesh meshNew = mesh; //copy
	std::vector<Eigen::Vector3d>& vboNew = meshNew.vbo_;
	std::vector<std::array<int, 3>> iboNew;// reload
	tuple<vector<std::array<int, 3>>, vector<set<int>>> info = _getMeshVertexLinkedInfo(mesh);
	const vector<std::array<int, 3>>& edgeDiag = get<0>(info);
	map<array<int, 2>, int> uniqueEdge;
	for (int i = 0; i < mesh.ibo_.size(); ++i)
	{
		const std::array<int, 3>& face = mesh.ibo_[i];
		int mid01, mid12, mid20; //the new vertex index of edge middle
		//the unique edge, small->large
		array<int, 2> edge01 = (face[0] < face[1]) ? array<int, 2>{ face[0], face[1] } : array<int, 2>{face[1], face[0] };
		if (uniqueEdge.find(edge01) != uniqueEdge.end())
		{
			mid01 = uniqueEdge.at(edge01);
		}
		else
		{
			mid01 = (int)vboNew.size();
			uniqueEdge.insert({ edge01, mid01 }); //index of new edge middle
			Vector3d pointNew = _getNewVertex(vboNew[face[0]], vboNew[face[1]], vboNew[face[2]], vboNew[edgeDiag[i][0]]);
			vboNew.push_back(pointNew);
		}
		array<int, 2> edge12 = (face[1] < face[2]) ? array<int, 2>{ face[1], face[2] } : array<int, 2>{face[2], face[1] };
		if (uniqueEdge.find(edge12) != uniqueEdge.end())
		{
			mid12 = uniqueEdge.at(edge12);
		}
		else
		{
			mid12 = (int)vboNew.size();
			uniqueEdge.insert({ edge12, mid12 });
			Vector3d pointNew = _getNewVertex(vboNew[face[1]], vboNew[face[2]], vboNew[face[0]], vboNew[edgeDiag[i][1]]);
			vboNew.push_back(pointNew);
		}
		array<int, 2> edge20 = (face[2] < face[0]) ? array<int, 2>{ face[2], face[0] } : array<int, 2>{face[0], face[2] };
		if (uniqueEdge.find(edge20) != uniqueEdge.end())
		{
			mid20 = uniqueEdge.at(edge20);
		}
		else
		{
			mid20 = (int)vboNew.size();
			uniqueEdge.insert({ edge20, mid20 });
			Vector3d pointNew = _getNewVertex(vboNew[face[2]], vboNew[face[0]], vboNew[face[1]], vboNew[edgeDiag[i][2]]);
			vboNew.push_back(pointNew);
		}
		iboNew.push_back({ face[0],mid01,mid20 });
		iboNew.push_back({ face[1],mid12,mid01 });
		iboNew.push_back({ face[2],mid20,mid12 });
		iboNew.push_back({ mid01,mid12,mid20 });
	}
	const vector<set<int>>& roundIndex = get<1>(info);
	for (int i = 0; i < mesh.vbo_.size(); ++i)
	{
		vector<Vector3d> round;
		for (const auto& j : roundIndex[i])
			round.push_back(mesh.vbo_[j]);
		vboNew[i] = _updateOldVertex(mesh.vbo_[i], round);
	}
	meshNew.ibo_ = iboNew;
	return meshNew;
}

HeMesh games::meshLoopSubdivision(const HeMesh& mesh)
{
	HeMesh meshNew; //create new empty HeMesh
	int indexFace = 0;// mesh.m_faces.size();
	int indexVertex = 0;//mesh.m_vertexes.size();
	int indexEdge = 0;//mesh.m_edges.size();
	std::map<int, int> new2oldEdge; // the new outer edge | incident origin edge(two edge use one index)
	std::map<int, int> old2newEdge; // old origin edge index | two new statr and end edge (index-1, index)
	for (const auto& iter : mesh.m_faces)
	{
		// the face's index0 vertex
		HeVertex* vtFace0 = new HeVertex; 
		vtFace0->m_index = indexVertex++;
		vtFace0->m_coord = iter->m_incEdge->m_oriVertex->m_coord;
		meshNew.m_vertexes.push_back(vtFace0);
		HeEdge* edgeFaceS0 = new HeEdge; // 1/2 start
		edgeFaceS0->m_index = indexEdge++;
		edgeFaceS0->m_oriVertex = vtFace0;
		vtFace0->m_incEdge = edgeFaceS0;
		meshNew.m_edges.push_back(edgeFaceS0);
		HeVertex* vtMiddle0 = new HeVertex;;
		vtMiddle0->m_index = indexVertex++;
		vtMiddle0->m_coord = _getNewVertex(
			iter->m_incEdge->m_oriVertex->m_coord, 
			iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_prevEdge->m_oriVertex->m_coord, 
			iter->m_incEdge->m_twinEdge->m_prevEdge->m_oriVertex->m_coord);
		meshNew.m_vertexes.push_back(vtMiddle0);
		HeEdge* edgeFaceE0 = new HeEdge; // 2/2 end
		edgeFaceE0->m_index = indexEdge++;
		edgeFaceE0->m_oriVertex = vtMiddle0;
		meshNew.m_edges.push_back(edgeFaceE0);
		vtMiddle0->m_incEdge = edgeFaceE0;
		new2oldEdge.emplace(indexEdge, iter->m_incEdge->m_index);
		old2newEdge.emplace(iter->m_incEdge->m_index, indexEdge);
		// the face's index1 vertex
		HeVertex* vtFace1 = new HeVertex; 
		vtFace1->m_index = indexVertex++;
		vtFace1->m_coord = iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord;
		meshNew.m_vertexes.push_back(vtFace1);
		HeEdge* edgeFaceS1 = new HeEdge; // 1/2 start
		edgeFaceS1->m_index = indexEdge++;
		edgeFaceS1->m_oriVertex = vtFace1;
		vtFace1->m_incEdge = edgeFaceS1;
		meshNew.m_edges.push_back(edgeFaceS1);
		HeVertex* vtMiddle1 = new HeVertex;;
		vtMiddle1->m_index = indexVertex++;
		vtMiddle1->m_coord = _getNewVertex(
			iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_prevEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_nextEdge->m_twinEdge->m_prevEdge->m_oriVertex->m_coord);
		meshNew.m_vertexes.push_back(vtMiddle1);
		HeEdge* edgeFaceE1 = new HeEdge; // 2/2 end
		edgeFaceE1->m_index = indexEdge++;
		edgeFaceE1->m_oriVertex = vtMiddle1;
		meshNew.m_edges.push_back(edgeFaceE1);
		vtMiddle1->m_incEdge = edgeFaceE1;
		new2oldEdge.emplace(indexEdge, iter->m_incEdge->m_nextEdge->m_index);
		old2newEdge.emplace(iter->m_incEdge->m_nextEdge->m_index, indexEdge);
		// the face's index2 vertex
		HeVertex* vtFace2 = new HeVertex; 
		vtFace2->m_index = indexVertex++;
		vtFace2->m_coord = iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord;
		edgeFaceS1->m_oriVertex = vtFace2;
		meshNew.m_vertexes.push_back(vtFace2);
		HeEdge* edgeFaceS2 = new HeEdge; // 1/2 start
		edgeFaceS2->m_index = indexEdge++;
		edgeFaceS2->m_oriVertex = vtFace2;
		vtFace2->m_incEdge = edgeFaceS2;
		meshNew.m_edges.push_back(edgeFaceS2);
		HeVertex* vtMiddle2 = new HeVertex;;
		vtMiddle2->m_index = indexVertex++;
		vtMiddle2->m_coord = _getNewVertex(
			iter->m_incEdge->m_prevEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_prevEdge->m_twinEdge->m_prevEdge->m_oriVertex->m_coord);
		meshNew.m_vertexes.push_back(vtMiddle2);
		HeEdge* edgeFaceE2 = new HeEdge; // 2/2 end
		edgeFaceE2->m_index = indexEdge++;
		edgeFaceE2->m_oriVertex = vtMiddle2;
		meshNew.m_edges.push_back(edgeFaceE2);
		vtMiddle2->m_incEdge = edgeFaceE2;
		new2oldEdge.emplace(indexEdge, iter->m_incEdge->m_prevEdge->m_index);
		old2newEdge.emplace(iter->m_incEdge->m_prevEdge->m_index, indexEdge);
		// create 4 triangles of half-edge
		// new triangle-0
		HeEdge* edgeMidO0 = new HeEdge; //outer
		HeEdge* edgeMidI0 = new HeEdge; //inner
		edgeMidO0->m_index = indexEdge++;
		edgeMidI0->m_index = indexEdge++;
		edgeMidO0->m_twinEdge = edgeMidI0;
		edgeMidI0->m_twinEdge = edgeMidO0;
		edgeMidO0->m_oriVertex = vtMiddle0;
		edgeMidI0->m_oriVertex = vtMiddle2;
		edgeMidO0->m_prevEdge = edgeFaceS0;
		edgeMidO0->m_nextEdge = edgeFaceE2;
		meshNew.m_edges.push_back(edgeMidO0);
		meshNew.m_edges.push_back(edgeMidI0);
		HeFace* face0 = new HeFace;
		face0->m_index = indexFace++;
		face0->m_normal = (vtMiddle0->m_coord - vtFace0->m_coord).cross(vtMiddle2->m_coord - vtFace0->m_coord);
		face0->m_incEdge = edgeMidO0;
		meshNew.m_faces.push_back(face0);
		edgeMidO0->m_incFace = face0;
		edgeFaceS0->m_incFace = face0;
		edgeFaceE2->m_incFace = face0;
		// new triangle-1
		HeEdge* edgeMidO1 = new HeEdge; //outer
		HeEdge* edgeMidI1 = new HeEdge; //inner
		edgeMidO1->m_index = indexEdge++;
		edgeMidI1->m_index = indexEdge++;
		edgeMidO1->m_twinEdge = edgeMidI1;
		edgeMidI1->m_twinEdge = edgeMidO1;
		edgeMidO1->m_oriVertex = vtMiddle1;
		edgeMidI1->m_oriVertex = vtMiddle0;
		edgeMidO1->m_prevEdge = edgeFaceS1;
		edgeMidO1->m_nextEdge = edgeFaceE0;
		meshNew.m_edges.push_back(edgeMidO1);
		meshNew.m_edges.push_back(edgeMidI1);
		HeFace* face1 = new HeFace;
		face1->m_index = indexFace++;
		face1->m_normal = (vtMiddle1->m_coord - vtFace1->m_coord).cross(vtMiddle0->m_coord - vtFace1->m_coord);
		face1->m_incEdge = edgeMidO1;
		meshNew.m_faces.push_back(face1);
		edgeMidO1->m_incFace = face1;
		edgeFaceS1->m_incFace = face1;
		edgeFaceE0->m_incFace = face1;
		// new triangle-2
		HeEdge* edgeMidO2 = new HeEdge; //outer
		HeEdge* edgeMidI2 = new HeEdge; //inner
		edgeMidO2->m_index = indexEdge++;
		edgeMidI2->m_index = indexEdge++;
		edgeMidO2->m_twinEdge = edgeMidI2;
		edgeMidI2->m_twinEdge = edgeMidO2;
		edgeMidO2->m_oriVertex = vtMiddle2;
		edgeMidI2->m_oriVertex = vtMiddle1;
		edgeMidO2->m_prevEdge = edgeFaceS2;
		edgeMidO2->m_nextEdge = edgeFaceE1;
		meshNew.m_edges.push_back(edgeMidO2);
		meshNew.m_edges.push_back(edgeMidI2);
		HeFace* face2 = new HeFace;
		face2->m_index = indexFace++;
		face2->m_normal = (vtMiddle2->m_coord - vtFace2->m_coord).cross(vtMiddle1->m_coord - vtFace2->m_coord);
		face2->m_incEdge = edgeMidO0;
		meshNew.m_faces.push_back(face2);
		edgeMidO2->m_incFace = face2;
		edgeFaceS2->m_incFace = face2;
		edgeFaceE1->m_incFace = face2;
		// new triangle-inner
		HeFace* faceIn = new HeFace;
		faceIn->m_index = indexFace++;
		faceIn->m_normal = (vtMiddle1->m_coord - vtMiddle0->m_coord).cross(vtMiddle2->m_coord - vtMiddle2->m_coord);
		faceIn->m_incEdge = edgeMidI0;
		meshNew.m_faces.push_back(faceIn);
		//relation of inner triangle's edge
		edgeMidI0->m_prevEdge = edgeMidI2;
		edgeMidI0->m_nextEdge = edgeMidI1;
		edgeMidI0->m_incFace = faceIn;
		edgeMidI1->m_prevEdge = edgeMidI0;
		edgeMidI1->m_nextEdge = edgeMidI2;
		edgeMidI1->m_incFace = faceIn;
		edgeMidI2->m_prevEdge = edgeMidI1;
		edgeMidI2->m_nextEdge = edgeMidI0;
		edgeMidI2->m_incFace = faceIn;
		//relation of origin face triangle's edge
		edgeFaceS0->m_prevEdge = edgeFaceE2;
		edgeFaceS0->m_nextEdge = edgeMidO0;
		edgeFaceE0->m_prevEdge = edgeMidO1;
		edgeFaceE0->m_nextEdge = edgeFaceS1;
		edgeFaceS1->m_prevEdge = edgeFaceE0;
		edgeFaceS1->m_nextEdge = edgeMidO1;
		edgeFaceE1->m_prevEdge = edgeMidO2;
		edgeFaceE1->m_nextEdge = edgeFaceS2;
		edgeFaceS2->m_prevEdge = edgeFaceE1;
		edgeFaceS2->m_nextEdge = edgeMidO2;
		edgeFaceE2->m_prevEdge = edgeMidO0;
		edgeFaceE2->m_nextEdge = edgeFaceS0;
	}
	for (const auto& edge : new2oldEdge) // edge pair
	{
		HeEdge* edgeS = meshNew.m_edges[edge.first - 1];
		HeEdge* edgeE = meshNew.m_edges[edge.first];
		edgeS->m_twinEdge = meshNew.m_edges[old2newEdge[edge.second]];
		edgeE->m_twinEdge = meshNew.m_edges[old2newEdge[edge.second - 1]];
		// not using double assign
	}
	return meshNew;
}

//ModelMesh meshGeneralSubdivision(const ModelMesh& mesh) //catmull-clark subdivision
//{
//	return mesh;
//}

////without update
//ModelMesh meshQuadricErrorMetricsSimplification(const ModelMesh& mesh, size_t collapseEdgeCount /*= 0*/) //edge collapse and quadirc error metrics
//{
//	if (collapseEdgeCount == 0)
//		collapseEdgeCount = mesh.ibo_.size() / 2;
//	if (collapseEdgeCount >= mesh.ibo_.size())
//		return {};
//	//get edge
//	set<array<int, 2>> uniqueEdge;
//	for (const auto& iter : mesh.ibo_)
//	{
//		array<int, 4> tri = { iter[0], iter[1], iter[2], iter[0] };
//		for (int i = 0; i < 3; i++)
//		{
//			array<int, 2> edge = (tri[i] < tri[i + 1]) ?
//				array<int, 2>{tri[i], tri[i + 1]} : 
//				array<int, 2>{tri[i + 1], tri[i]};
//			uniqueEdge.insert(edge);
//		}
//	}
//	map<array<int, 2>, array<vector<int>, 2>> edgeNeighborFace; // edge vertex index | two vertex NeighborFace index
//	map<array<int, 2>, array<int, 2>> edgeOnFace; // edge vertex index | two faces index
//	for (const auto& edge : uniqueEdge)
//	{
//		//get edgeNeighborFace
//		vector<int> edge0, edge1;
//		for (int i = 0; i < mesh.ibo_.size(); i++)
//		{
//			const array<int, 3>& face = mesh.ibo_[i];
//			if (face[0] == edge[0] || face[1] == edge[0] || face[2] == edge[0])
//				edge0.push_back(i);
//			if (face[0] == edge[1] || face[1] == edge[1] || face[2] == edge[1])
//				edge1.push_back(i);
//		}
//		edgeNeighborFace.insert({ edge, { edge0, edge1 } });
//		// get edgeOnFace
//		array<int, 2> faceTwo;
//		int find = 0;
//		for (int i = 0; i < mesh.ibo_.size(); i++)
//		{
//			int coin = 0;
//			const array<int, 3>& face = mesh.ibo_[i];
//			for (const int& vt : face) //find two common vertex
//			{
//				if (vt == edge[0] || vt == edge[1])
//					coin++;
//			}
//			if (coin == 2) //means common edge
//			{
//				faceTwo[find] = i;
//				find++;
//			}
//			if (find == 2)
//			{
//				edgeOnFace.insert({ edge, faceTwo });
//				break;
//			}
//		}
//	}
//	//process come on
//	ModelMesh meshNew = mesh;//copy
//	std::vector<Eigen::Vector3d>& vbo = meshNew.vbo_;
//	std::vector<std::array<int, 3>>& ibo = meshNew.ibo_;
//	auto _computeEdgeError = [&](const Vector3d& v, const array<vector<int>, 2>& neighbor)->double
//		{
//			double totalError = 0.0;
//			for (const auto& faces : neighbor) //two vertex neighbor face
//			{
//				for (const int& iter : faces)
//				{
//					Vector3d v0 = vbo[ibo[iter][0]];
//					Vector3d v1 = vbo[ibo[iter][1]];
//					Vector3d v2 = vbo[ibo[iter][2]];
//					Vector3d n = (v1 - v0).cross(v2 - v1);
//					totalError += (v0 - v).dot(n) * ((v0 - v).dot(n)) / n.dot(n);
//				}
//			}
//			return totalError;
//		};
//	std::priority_queue<QEMEdge> collapseEdgeQueue;
//	auto _updateCollapseEdgeQueue = [&]()->void
//		{
//			for (const auto& iter : uniqueEdge)
//			{
//				QEMEdge edge;
//				edge.m_edge = iter;
//				edge.m_vertex = 0.5 * (vbo[iter[0]] + vbo[iter[1]]);
//				edge.m_error = _computeEdgeError(edge.m_vertex, edgeNeighborFace[iter]);
//				collapseEdgeQueue.push(edge);
//			}
//		};
//	_updateCollapseEdgeQueue();
//	//contract edge
//	size_t collaCout = 0;
//	while (collaCout < collapseEdgeCount)
//	{
//		QEMEdge edge = collapseEdgeQueue.top();
//		collapseEdgeQueue.pop(); //delete edge
//		uniqueEdge.erase(edge.m_edge);
//		ibo[edgeOnFace[edge.m_edge][0]] = { -1,-1,-1 };
//		ibo[edgeOnFace[edge.m_edge][1]] = { -1,-1,-1 };
//		vbo[edge.m_edge[0]] = edge.m_vertex;
//		vbo[edge.m_edge[1]] = gVecNaN;
//		// change record map
//		for (auto& face : ibo)
//		{
//			for (int i = 0; i < 3; i++)
//			{
//				//if (face[i] == edge.m_edge[0]) continue;
//				if (face[i] == edge.m_edge[1])
//					face[i] = edge.m_edge[0];
//			}
//		}
//		_updateCollapseEdgeQueue();
//		collaCout++;
//	}
//	ModelMesh meshSim;
//	//map<int, int> indexMap;
//	vector<int> indexMap;
//	int j = 0;
//	for (int i = 0; i < vbo.size(); i++)
//	{
//		indexMap.push_back(j);
//		if (!isnan(vbo[i][0]))
//		{
//			meshSim.vbo_.push_back(vbo[i]);
//			j++;
//		}
//	}
//	for (const auto& iter : ibo)
//	{
//		if (iter[0] != -1) //valid
//		{
//			std::array<int, 3> face = { indexMap[iter[0]], indexMap[iter[1]], indexMap[iter[2]] };
//			meshSim.ibo_.push_back(face);
//		}
//	}
//	return meshSim;
//}

// all function of QEM
inline void _getPlaneCoefficient(const array<Vector3d,3>& trigon, double& a, double& b, double& c, double& d)
{
	// a*x + b*y + c*z + d = 0
	//Vector3d N = (A - B).cross(C - B); // N_x*(x-A_x) + N_y*(y-A_y) + N_z*(y-A_z) = 0
	//assert(!N.isZero());
	Vector3d normal = (trigon[0] - trigon[1]).cross(trigon[2] - trigon[1]);
	a = normal[0];
	b = normal[1];
	c = normal[2];
	d = -normal.dot(trigon[0]);
	double len = std::sqrt(a * a + b * b + c * c); //a^2 + b^2 + c^2 = 1
	a /= len;
	b /= len;
	c /= len;
	d /= len;
	//return;
}

// calculate Q of every vertex
inline Matrix4d _getQMatrixOfVertex(const std::vector<std::array<int, 3>>& ibo, const std::vector<Eigen::Vector3d>& vbo, int i)
{
	Matrix4d Q = Eigen::Matrix4d::Zero();
	for (const auto& face : ibo)
	{
		if (face[0] != i && face[1] != i && face[2] != i) //find all adjacent faces 
			continue;
		double a, b, c, d;
		_getPlaneCoefficient({ vbo[face[0]], vbo[face[1]], vbo[face[2]] }, a, b, c, d);
		Vector4d p(a, b, c, d);
		Q += p * p.transpose(); //Kp matrix sigma
	}
	return Q;
}

inline Matrix4d _getQMatrixOfVertex(const HeMesh& mesh, int i)
{
	Matrix4d Q = Eigen::Matrix4d::Zero();
	for (const auto& face : mesh.m_faces)
	{
		if (!face->isinclude(mesh.m_vertexes[i]))
			continue;
		double a, b, c, d;
		_getPlaneCoefficient(face->ibo_v(), a, b, c, d);
		Vector4d p(a, b, c, d);
		Q += p * p.transpose(); //Kp matrix sigma
	}
	return Q;
}

inline QEMEdge _getCostAndVbarOfEdge(const std::vector<Matrix4d>& Qs, const std::vector<Vector3d>& vbo, const array<int, 2 >& i) 
{
	QEMEdge edge;
	edge.m_edge = i;
	//calculate cost and v_bar
	Eigen::Matrix4d Q_bar = Qs[i[0]] + Qs[i[1]];
	Eigen::Matrix4d Q_h = Q_bar;//homogeneous
	Q_h.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
	Eigen::Matrix4d inverse;
	bool invertible;
	Q_h.computeInverseWithCheck(inverse, invertible);
	Eigen::Vector4d b(0, 0, 0, 1);
	Eigen::Vector4d v_bar = (invertible) ?
		v_bar = inverse * b : //v_bar = (1 / v_bar[3]) * v_bar; //hnormalized
		v_bar = (0.5 * (vbo[i[0]] + vbo[i[0]])).homogeneous();
	double error = v_bar.transpose() * Q_bar * v_bar; // the cost
	edge.m_vbar = v_bar;
	edge.m_error = error;
	return edge;
}

inline QEMEdge _getCostAndVbarOfEdge(const std::vector<Matrix4d>& Qs, const HeEdge* i)
{
	QEMEdge edge;
	edge.m_index = i->m_index;
	//calculate cost and v_bar
	Eigen::Matrix4d Q_bar = Qs[i->m_oriVertex->m_index] + Qs[i->m_nextEdge->m_oriVertex->m_index]; //get vertex index
	Eigen::Matrix4d Q_h = Q_bar;//homogeneous
	Q_h.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
	Eigen::Matrix4d inverse;
	bool invertible;
	Q_h.computeInverseWithCheck(inverse, invertible);
	Eigen::Vector4d b(0, 0, 0, 1);
	Eigen::Vector4d v_bar = (invertible) ?
		v_bar = inverse * b : //v_bar = (1 / v_bar[3]) * v_bar; //hnormalized
		v_bar = (0.5 * (i->m_oriVertex->m_coord + i->m_nextEdge->m_oriVertex->m_coord)).homogeneous(); //get vertex coord
	double error = v_bar.transpose() * Q_bar * v_bar; // the cost
	edge.m_vbar = v_bar;
	edge.m_error = error;
	return edge;
}

//#ifndef USING_HALFEDGE_STRUCTURE
ModelMesh games::meshQEMSimplification(const ModelMesh& mesh, size_t collapseEdgeCount /*= 0*/)
{
	//get edge
	set<array<int, 2>> uniqueEdge;
	for (const auto& iter : mesh.ibo_)
	{
		array<int, 4> tri = { iter[0], iter[1], iter[2], iter[0] };
		for (int i = 0; i < 3; i++)
		{
			array<int, 2> edge = (tri[i] < tri[i + 1]) ?
				array<int, 2>{tri[i], tri[i + 1]} : array<int, 2>{tri[i + 1], tri[i]};
			uniqueEdge.insert(edge);
		}
	}
	//get edge located face2
	map<array<int, 2>, array<int, 2>> edgeOnFace; // edge vertex index | two faces index
	for (const auto& edge : uniqueEdge)
	{
		array<int, 2> faceTwo;
		int find = 0;
		for (int i = 0; i < mesh.ibo_.size(); i++)
		{
			int coin = 0;
			const array<int, 3>& face = mesh.ibo_[i];
			for (const int& vt : face) //find two common vertex
			{
				if (vt == edge[0] || vt == edge[1])
					coin++;
			}
			if (coin == 2) //means common edge
			{
				faceTwo[find] = i;
				find++;
			}
			if (find == 2)
			{
				edgeOnFace.emplace(edge, faceTwo);
				break;
			}
		}
	}
	std::vector<Eigen::Matrix4d> Qs;
	for (int i = 0; i < mesh.vbo_.size(); ++i)
	{
		Qs.push_back(_getQMatrixOfVertex(mesh.ibo_, mesh.vbo_, i));
	}
	// place edge into heap
	std::priority_queue<QEMEdge> heap;
	for (const auto& iter : uniqueEdge)
	{
		QEMEdge edge = _getCostAndVbarOfEdge(Qs, mesh.vbo_, iter);
		heap.push(edge);
	}
	ModelMesh meshC = mesh;//copy
	std::vector<Eigen::Vector3d>& vbo = meshC.vbo_;
	std::vector<std::array<int, 3>>& ibo = meshC.ibo_;
	auto _updateCollapseEdgeHeap = [&](int i_bar) ->void
	{
		set<int> adjacentVertex;
		set<array<int, 2>> adjacentEdge;
		//Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
		for (const auto& face : ibo)
		{
			if (face[0] != i_bar && face[1] != i_bar && face[2] != i_bar) //find the adjacent faces 
				continue;
			if (face[0] == i_bar)
			{
				adjacentVertex.insert(face[1]);
				adjacentVertex.insert(face[2]);
				(face[0] < face[1]) ? adjacentEdge.insert({ face[0], face[1] }) : adjacentEdge.insert({ face[1], face[0] });
				(face[0] < face[2]) ? adjacentEdge.insert({ face[0], face[2] }) : adjacentEdge.insert({ face[2], face[0] });
			}
			else if (face[1] == i_bar)
			{
				adjacentVertex.insert(face[0]);
				adjacentVertex.insert(face[2]);
				(face[1] < face[0]) ? adjacentEdge.insert({ face[1], face[0] }) : adjacentEdge.insert({ face[0], face[1] });
				(face[1] < face[2]) ? adjacentEdge.insert({ face[1], face[2] }) : adjacentEdge.insert({ face[2], face[1] });
			}
			else//if (face[2] == i_bar)
			{
				adjacentVertex.insert(face[1]);
				adjacentVertex.insert(face[0]);
				(face[2] < face[1]) ? adjacentEdge.insert({ face[2], face[1] }) : adjacentEdge.insert({ face[1], face[2] });
				(face[2] < face[0]) ? adjacentEdge.insert({ face[2], face[0] }) : adjacentEdge.insert({ face[0], face[2] });
			}
		}
		// update Q of adjacent Vertex
		Qs[i_bar] = _getQMatrixOfVertex(ibo, vbo, i_bar);
		for (const int& iter : adjacentVertex)
		{
			Qs[iter] = _getQMatrixOfVertex(ibo, vbo, iter);
		}
		// add new edges
		for (const auto& iter : adjacentEdge)
		{
			QEMEdge edge = _getCostAndVbarOfEdge(Qs, vbo, iter);
			heap.push(edge);
		}
	};
	//contract edge
	size_t collaCout = 0;
	while (collaCout < collapseEdgeCount)
	{
		const QEMEdge& edge = heap.top();
		ibo[edgeOnFace[edge.m_edge][0]] = { -1,-1,-1 }; //delete two face
		ibo[edgeOnFace[edge.m_edge][1]] = { -1,-1,-1 };
		vbo[edge.m_edge[0]] = edge.m_vbar.hnormalized();
		vbo[edge.m_edge[1]] = gVecNaN; //delete one vertex
		for (auto& face : ibo) //change face who own deleted vertex
		{
			for (int i = 0; i < 3; i++)
			{
				if (face[i] == edge.m_edge[1])
					face[i] = edge.m_edge[0];
			}
		}
		heap.pop(); //delete one edge
		_updateCollapseEdgeHeap(edge.m_edge[0]); //新的edge的error会不会小于受vbar影响而改变的edge
		collaCout++;
	}
	ModelMesh meshSim; //new mesh
	vector<int> indexMap; // origin face's vertex index -> new index
	int j = 0;
	for (int i = 0; i < vbo.size(); i++)
	{
		indexMap.push_back(j);
		if (!isnan(vbo[i][0]))
		{
			meshSim.vbo_.push_back(vbo[i]);
			j++;
		}
	}
	for (const auto& iter : ibo)
	{
		if (iter[0] != -1) //valid
		{
			std::array<int, 3> face = { indexMap[iter[0]], indexMap[iter[1]], indexMap[iter[2]] };
			meshSim.ibo_.push_back(face);
		}
	}
	return meshSim;
}
//#endif

// mesh operation with halfedge
#ifdef USING_HALFEDGE_STRUCTURE
HeMesh games::meshQEMSimplification(const HeMesh& mesh, size_t edgeCollapseTarget /*= 0*/)
{
	std::vector<Eigen::Matrix4d> Qs;
	for (int i = 0; i < mesh.m_vertexes.size(); ++i)
	{
		Qs.push_back(_getQMatrixOfVertex(mesh, i));
	}
	// place edge into heap
	std::priority_queue<QEMEdge> heap;
	std::set<int> uniqueEdge;
	for (const auto& iter : mesh.m_edges) // only unique edge, halfedge means double amount
	{
		if (uniqueEdge.find(iter->m_index) == uniqueEdge.end() &&
			uniqueEdge.find(iter->m_twinEdge->m_index) == uniqueEdge.end())
		{
			uniqueEdge.insert(iter->m_index);
			QEMEdge edge = _getCostAndVbarOfEdge(Qs, iter);
			heap.push(edge);
		}
	}
	size_t edgeCollapseCurrent = 0;
	HeMesh meshC = mesh;//copy
	auto _updateCollapseEdgeHeap = [&](const HeVertex* vbar)->void
	{
		//update neibor vertex qme-value
		set<int> adjacentVertex;
		for (const auto iter : meshC.m_faces)
		{
			if (iter->isinclude(vbar))
			{
				const std::array<int, 3>& ibo = iter->ibo();
				for (const auto i : ibo)
					adjacentVertex.insert(i); //include vbar->m_index self
			}
		}
		for (const auto iter : adjacentVertex)
			Qs[iter] = _getQMatrixOfVertex(meshC, iter);
		//update neibor edge qme-value
		for (const auto iter : meshC.m_edges)
		{
			if (iter->m_oriVertex == vbar) //keep half amout
			{
				QEMEdge edge = _getCostAndVbarOfEdge(Qs, iter);
				heap.push(edge);
			}
		}
	};
	//contract edge
	while (edgeCollapseCurrent < edgeCollapseTarget)
	{
		const QEMEdge& mini = heap.top();
		HeEdge* heEdge = meshC.m_edges[mini.m_index];
		//update first vertex, delete second vertex
		heEdge->m_oriVertex->m_coord = mini.m_vbar.hnormalized(); //update merged vertex
		for (auto iter : mesh.m_edges)
		{
			if (iter->m_oriVertex->m_index == heEdge->m_twinEdge->m_oriVertex->m_index)
				iter->m_oriVertex = heEdge->m_oriVertex;
		}
		if (heEdge->m_oriVertex->m_incEdge == heEdge) //avoid heVertex hold nullptr
			heEdge->m_oriVertex->m_incEdge = heEdge->m_prevEdge->m_twinEdge;
		if (heEdge->m_twinEdge->m_oriVertex)
		{
			delete heEdge->m_twinEdge->m_oriVertex; //also heEdge->m_nextEdge->m_oriVertex
			heEdge->m_twinEdge->m_oriVertex = nullptr;
		}
		//change the twin edge of deleted face
		int merge0 = heEdge->m_nextEdge->m_twinEdge->m_index;
		int merge1 = heEdge->m_prevEdge->m_twinEdge->m_index;
		meshC.m_edges[merge0]->m_twinEdge = meshC.m_edges[merge1];
		meshC.m_edges[merge1]->m_twinEdge = meshC.m_edges[merge0];
		merge0 = heEdge->m_twinEdge->m_nextEdge->m_twinEdge->m_index;
		merge1 = heEdge->m_twinEdge->m_prevEdge->m_twinEdge->m_index;
		meshC.m_edges[merge0]->m_twinEdge = meshC.m_edges[merge1];
		meshC.m_edges[merge1]->m_twinEdge = meshC.m_edges[merge0];
		// delete two face
		if (heEdge->m_incFace)
		{
			delete heEdge->m_incFace;
			heEdge->m_incFace = nullptr;
		}
		if (heEdge->m_twinEdge->m_incFace)
		{
			delete heEdge->m_twinEdge->m_incFace;
			heEdge->m_twinEdge->m_incFace = nullptr;
		}
		_updateCollapseEdgeHeap(heEdge->m_oriVertex);
		//delete one edge
		if (heEdge)
		{
			delete heEdge;
			heEdge = nullptr;
		}
		if (heEdge->m_twinEdge)
		{
			delete heEdge->m_twinEdge;
			heEdge->m_twinEdge = nullptr;
		}
		heap.pop();
		edgeCollapseCurrent++;
	}
	HeMesh meshSim; //new mesh
	int j = 0;
	for (int i = 0; i < meshC.m_vertexes.size(); i++)
	{
		if (meshC.m_vertexes[i] != nullptr)
		{
			meshC.m_vertexes[i]->m_index = j;
			meshSim.m_vertexes.emplace_back(meshC.m_vertexes[i]);
			j++;
		}
	}
	j = 0;
	for (int i = 0; i < meshC.m_edges.size(); i++)
	{
		if (meshC.m_edges[i] != nullptr)
		{
			meshC.m_edges[i]->m_index = j;
			meshSim.m_edges.emplace_back(meshC.m_edges[i]);
			j++;
		}
	}
	j = 0;
	for (int i = 0; i < meshC.m_faces.size(); i++)
	{
		if (meshC.m_faces[i] != nullptr)
		{
			meshC.m_faces[i]->m_index = j;
			meshSim.m_faces.emplace_back(meshC.m_faces[i]);
			j++;
		}
	}
	return meshSim;
}
#endif //USING_HALFEDGE_STRUCTURE
