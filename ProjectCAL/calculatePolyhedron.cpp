#include "pch.h"
#include "calculatePolyhedron.h"
using namespace std;
using namespace Eigen;
using namespace clash;
using namespace eigen;
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
//static int count_edgediag_full = 0;
#endif // DEBUG_POLYHEDRON_MESH
#ifdef STATISTIC_DATA_RECORD
//container
extern std::vector<std::array<Triangle, 2>> triPairList, triRecordHard; //std::array<Eigen::Vector3d, 3>
extern std::vector<Triangle> triFaceVctA, triFaceVctB;
extern std::vector<Vector3d> triVertexVctA, triVertexVctB;
extern std::vector<InterTriInfo> interTriInfoList;
#endif // STATISTIC_DATA_RECORD

//bool clash::isMeshConvexPolyhedron(const std::vector<Eigen::Vector3d>& vbo, const std::vector<Eigen::Vector3i>& ibo)
bool clash::isMeshConvexPolyhedron(const ModelMesh& mesh)
{
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<Eigen::Vector3i>& ibo = mesh.ibo_;
	bool isLeft = false;
    for (int i = 0; i < (int)ibo.size(); ++i) //(const auto& face : ibo)
	{
		const Vector3d& normal = mesh.fno_[i];//(vbo[face[1]] - vbo[face[0]]).cross(vbo[face[2]] - vbo[face[1]]);// .normalized();
		double d_eps = normal.squaredNorm() * epsF; // wide threshold
		bool isFirst = true;
		for (int j = 0; j < (int)vbo.size(); ++j)
		{
			double projection = normal.dot(vbo[j] - vbo[ibo[i][0]]);
			if (j == ibo[i][0] || j == ibo[i][1] || j == ibo[i][2] || fabs(projection) <= d_eps) // self or coplanar
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
	const std::vector<Eigen::Vector3i>& ibo = mesh.ibo_;
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
	const std::vector<Eigen::Vector3i>& ibo = mesh.ibo_;
	vector<Eigen::Vector3i> faceVisible;
	vector<Eigen::Vector3d> fno = getNormalVectorOfMeshFace(mesh); //correct face normal
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
	Matrix4d matSha = eigen::getProjectionMatrixByPlane(plane);
	for (auto& iter : profilePoints)
		iter = (matSha * iter.homogeneous()).hnormalized();
	return profilePoints;
}

ModelMesh clash::mergeMultiMeshsToOneMesh(const std::vector<ModelMesh>& meshVct)
{
	if (meshVct.size() == 0)
		return {};
	else if (meshVct.size() == 1)
		return meshVct[0];
    ModelMesh mesh = meshVct[0]; //res
	mesh.convex_ = false;
	mesh.number_ = -1;//reset
	if (!mesh.pose_.isApprox(Eigen::Affine3d::Identity()))
	{
		for (size_t i = 0; i < mesh.vbo_.size(); ++i)
			mesh.vbo_[i] = mesh.pose_ * mesh.vbo_[i];
		for (size_t i = 0; i < mesh.fno_.size(); ++i)
			mesh.fno_[i] = mesh.pose_ * mesh.fno_[i];
		mesh.pose_ = Eigen::Affine3d::Identity();
	}
	for (size_t i = 1; i < meshVct.size(); ++i)
	{
		ModelMesh iter = meshVct[i];//copy
		mesh.bounding_.extend(iter.bounding_); //bounding box is world
		if (!iter.pose_.isApprox(Eigen::Affine3d::Identity()))
		{
			for (size_t j = 0; j < iter.vbo_.size(); ++j)
				iter.vbo_[j] = iter.pose_ * iter.vbo_[j];
			for (size_t j = 0; j < iter.fno_.size(); ++j)
				iter.fno_[j] = iter.pose_ * iter.fno_[j];
		}
		for (size_t j = 0; j < iter.ibo_.size(); ++j)
			iter.ibo_[j] = iter.ibo_[j] + Vector3i((int)mesh.vbo_.size(), (int)mesh.vbo_.size(), (int)mesh.vbo_.size());
		mesh.vbo_.insert(mesh.vbo_.end(), iter.vbo_.begin(), iter.vbo_.end());
		mesh.fno_.insert(mesh.fno_.end(), iter.fno_.begin(), iter.fno_.end());
		mesh.ibo_.insert(mesh.ibo_.end(), iter.ibo_.begin(), iter.ibo_.end());
	}
	return mesh;
}

bool clash::isPointInsidePolyhedronMTA(const Eigen::Vector3d& point, const ModelMesh& mesh)
{
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<Eigen::Vector3i>& ibo = mesh.ibo_;
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
	const std::vector<Eigen::Vector3i>& ibo = mesh.ibo_;
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
	const std::vector<Eigen::Vector3i>& ibo = mesh.ibo_;
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
	auto isLeftAll = [&vbo](const Eigen::Vector3i& trigon)->bool // buildin lambda function
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
	const std::vector<Eigen::Vector3i>& iboA = meshA.ibo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	const std::vector<Eigen::Vector3i>& iboB = meshB.ibo_;
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
	const std::vector<Eigen::Vector3i>& iboA = meshA.ibo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	const std::vector<Eigen::Vector3i>& iboB = meshB.ibo_;
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
Eigen::Vector3d clash::getPenetrationDepthOfTwoMeshsParts(const ModelMesh& meshA, const ModelMesh& meshB, const std::vector<Eigen::Vector3d>& axesSepa,
	const std::set<int>& vboSetA, const std::set<int>& vboSetB)
{
	if (axesSepa.empty() || vboSetA.empty() || vboSetB.empty())
		return Vector3d::Zero();
	const std::vector<Eigen::Vector3d>& vboA = meshA.vbo_;
	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
	//const std::vector<Eigen::Vector3i>& iboA = meshA.ibo_;
	//const std::vector<Eigen::Vector3i>& iboB = meshB.ibo_;
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
//	const std::vector<Eigen::Vector3i>& iboA = meshA.ibo_;
//	const std::vector<Eigen::Vector3d>& vboB = meshB.vbo_;
//	const std::vector<Eigen::Vector3i>& iboB = meshB.ibo_;
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

static Eigen::Affine3d _getRelativeMatrixRectify(const Eigen::Affine3d& matA, const Eigen::Affine3d& matB)
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
static std::array<std::vector<int>, 2> _triangleAndCommonBoxPreclash(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance)
{
	Eigen::AlignedBox3d box = meshA.bounding_.intersection(meshB.bounding_); //common box
	Vector3d toleSize = Vector3d(tolerance, tolerance, tolerance);
	Eigen::AlignedBox3d boxMag(box.min() - toleSize, box.max() + toleSize); //magnify
	std::vector<int> triA_Index; // using index of mesh IBO
	Eigen::AlignedBox3d triA_Box; // iterate to lessen box
	std::array<Eigen::Vector3d, 3> triIter;
	for (int i = 0; i < (int)meshA.ibo_.size(); ++i)
	{
		triIter = { // input matrix to avoid repeat calculate
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][0]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][1]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][2]] };
		//enlarge box while has tolerance
		if (isTriangleAndBoxIntersectSAT(triIter, boxMag))
		{
			triA_Index.push_back(i);
			triA_Box.extend(triIter[0]);
			triA_Box.extend(triIter[1]);
			triA_Box.extend(triIter[2]);
		}
	}
	if (triA_Index.empty())
		return {};
	std::vector<int> triB_Index;
	Eigen::AlignedBox3d triB_Box;
	for (int j = 0; j < (int)meshB.ibo_.size(); ++j)
	{
		triIter = { meshB.pose_ * meshB.vbo_[meshB.ibo_[j][0]],
					meshB.pose_ * meshB.vbo_[meshB.ibo_[j][1]],
					meshB.pose_ * meshB.vbo_[meshB.ibo_[j][2]] };
		if (isTriangleAndBoxIntersectSAT(triIter, boxMag))
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
	{
#ifdef STATISTIC_DATA_COUNT
		//count_tris_box_not_inter++;
		MACRO_EXPANSION_DATA_COUNT("count_tris_box_not_inter");
		MACRO_EXPANSION_DATA_PAIR(meshA.number_, meshB.number_)
#endif  
		return {};//exist situation
	}
	return { triA_Index, triB_Index };
}

// ClashHard
bool clash::isTwoMeshsIntersectSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance /*= 0.0*/)
{
	//Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	Eigen::Affine3d relative_matrix = meshB.pose_.inverse() * meshA.pose_; //without revise
	// get the index param of mesh's ibo
	std::array<std::vector<int>, 2> indexAB = _triangleAndCommonBoxPreclash(meshA, meshB, tolerance);
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		for (const int& iA : indexAB[0])
		{
			std::array<Eigen::Vector3d, 3> triA = {
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
			for (const int& iB : indexAB[1])
			{
				std::array<Eigen::Vector3d, 3> triB = {
					meshB.vbo_[meshB.ibo_[iB][0]],
					meshB.vbo_[meshB.ibo_[iB][1]],
					meshB.vbo_[meshB.ibo_[iB][2]] };
				if (!isTwoTrianglesBoxIntersect(triA, triB, tolerance)) // second pre-judge
				{
#ifdef STATISTIC_DATA_COUNT
					count_tri_box_exclude_pre++;
#endif  
					continue;
				}
				if (isTwoTrianglesIntersectSAT(triA, triB))
				{
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
#ifdef STATISTIC_DATA_COUNT
			count_mesh_inside_mesh++;
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
#ifdef STATISTIC_DATA_COUNT
			count_mesh_inside_mesh++;
#endif
			return true;
		}
	}
	return false;
}

// clash judge include penetration depth
#define USING_ALL_SEPARATE_AXES_FOR_DEPTH
std::tuple<RelationOfTwoMesh, Eigen::Vector3d> clash::getTwoMeshsIntersectRelation(const ModelMesh& meshA, const ModelMesh& meshB)
{
	//Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	Eigen::Affine3d relative_matrix = meshB.pose_.inverse() * meshA.pose_;
	// get the index param of mesh's ibo
	std::array<std::vector<int>, 2> indexAB = _triangleAndCommonBoxPreclash(meshA, meshB, 0.0);
	bool isContact = false; // vertex or edge or face contact
	bool isIntrusive = false;
#ifndef USING_ALL_SEPARATE_AXES_FOR_DEPTH
	std::set<size_t> faceSetA, faceSetB; // intersect and inside, to remove repeat
	std::vector<size_t> vertexVectA, vertexVectB;
#endif
	// all axes version
	std::vector<Vector3d> axesSepa; // in world coordinate
	std::set<int> vboSetA, vboSetB; // to remove repeat vertex
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		std::array<Eigen::Vector3d, 3> triA, triB;
		std::array<Eigen::Vector3d, 2> pInter;
		std::array<Eigen::Vector3d, 3> edgesA, edgesB;
		Eigen::Vector3d normalA, normalB;
		std::array<Eigen::Vector3d, 11> axesPoten; //remove 6 edge normal
#ifdef STATISTIC_DATA_RECORD
		for (const int& i : indexAB[0]) //for test
		{
			Triangle triIter = {
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][0]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][1]],
				meshA.pose_ * meshA.vbo_[meshA.ibo_[i][2]] };
			triFaceVctA.push_back(triIter); //world coord
		}
		for (const int& j : indexAB[1])
		{
			Triangle triIter = {
				meshB.pose_ * meshB.vbo_[meshB.ibo_[j][0]],
				meshB.pose_ * meshB.vbo_[meshB.ibo_[j][1]],
				meshB.pose_ * meshB.vbo_[meshB.ibo_[j][2]] };
			triFaceVctB.push_back(triIter); //world coord
		}
#endif
		for (const int& iA : indexAB[0]) //faces intersect
		{
			triA = { 
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
				relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
			for (const int& iB : indexAB[1])
			{
				triB = { 
					meshB.vbo_[meshB.ibo_[iB][0]],
					meshB.vbo_[meshB.ibo_[iB][1]],
					meshB.vbo_[meshB.ibo_[iB][2]] };
#ifdef STATISTIC_DATA_RECORD
				if (!isTwoTrianglesBoxIntersect(triA, triB))
					count_tri_box_exclude_pre++;
#endif  
				if (!isTwoTrianglesBoxIntersect(triA, triB) || !isTwoTrianglesIntersectSAT(triA, triB)) // second pre-judge, merge
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
		for (int i = 0; i < (int)meshA.vbo_.size(); ++i)
		{
#ifdef USING_ALL_SEPARATE_AXES_FOR_DEPTH
			if (vboSetA.find(i) == vboSetA.end() && RelationOfPointAndMesh::INNER == isPointInsidePolyhedronROT(meshA.pose_ * meshA.vbo_[i], meshB))
				vboSetA.insert(i);
#else
			if (isPointInsidePolyhedronFL(meshA.pose_ * meshA.vbo_[i], meshB))
				vertexVectA.push_back(i);
#endif // USING_ALL_SEPARATE_AXES_FOR_DEPTH
		}
		for (int i = 0; i < (int)meshB.vbo_.size(); ++i)
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
std::tuple<double, std::array<int, 2>> clash::getTwoMeshsSeparationDistanceSAT(const ModelMesh& meshA, const ModelMesh& meshB, double tolerance)
{
#ifdef STATISTIC_DATA_RECORD
	std::array<std::array<Eigen::Vector3d, 3>, 2> triDistPair;
#endif    
	//Eigen::Affine3d relative_matrix = _getRelativeMatrixRectify(meshA.pose_, meshB.pose_);// get relative matrix
	Eigen::Affine3d relative_matrix = meshB.pose_.inverse() * meshA.pose_; //without revise
	// distance > tolerance, return double-max, to decrease calculate
	double d = DBL_MAX; // the res
	std::array<int, 2> index = { -1, -1 };//INT_MAX
	std::array<vector<int>, 2> indexAB = _triangleAndCommonBoxPreclash(meshA, meshB, tolerance);
	if (indexAB[0].empty() || indexAB[1].empty())
		return { d, index };
	for (const int& iA : indexAB[0])
	{
		std::array<Eigen::Vector3d, 3> triA = {
			relative_matrix * meshA.vbo_[meshA.ibo_[iA][0]],
			relative_matrix * meshA.vbo_[meshA.ibo_[iA][1]],
			relative_matrix * meshA.vbo_[meshA.ibo_[iA][2]] };
		for (const int& iB : indexAB[1])
		{
			std::array<Eigen::Vector3d, 3> triB = {
				meshB.vbo_[meshB.ibo_[iB][0]],
				meshB.vbo_[meshB.ibo_[iB][1]],
				meshB.vbo_[meshB.ibo_[iB][2]] };
			if (!isTwoTrianglesBoxIntersect(triA, triB, tolerance))
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
