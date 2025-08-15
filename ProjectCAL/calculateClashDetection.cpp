#include "pch.h"
using namespace std;
using namespace clash;
using namespace test;
using namespace sat;
//#define STATISTIC_DATA_COUNT

std::vector<TriMesh> ClashDetection::sm_meshStore;

#ifdef STATISTIC_DATA_COUNT
static std::atomic<size_t> count_clash = 0, count_mesh = 0, count_triangle = 0, count_vertex = 0,
count_pre_clash = 0, count_mesh_intersect = 0, count_triangle_and_box_intersect = 0, count_two_triangle_intersect = 0, count_trianglebox_intersect = 0,
count_preclash_A = 0, count_preclash_B = 0, count_judge_inside = 0, count_mesh_inside = 0,
count_ = 0;

//debug test
static void _output_data()
{
	DataRecordSingleton& instace = DataRecordSingleton::getInstance();
	std::vector<std::pair<std::string, int>>& dataM = instace.getData().m_dataItemVct;
    dataM.push_back({ "count_clash", count_clash });
    dataM.push_back({ "count_mesh", count_mesh });
    dataM.push_back({ "count_triangle", count_triangle });
    dataM.push_back({ "count_vertex", count_vertex });
    dataM.push_back({ "count_pre_clash", count_pre_clash });
    dataM.push_back({ "count_preclash_A", count_preclash_A });
    dataM.push_back({ "count_preclash_B", count_preclash_B });
    dataM.push_back({ "count_judge_inside", count_judge_inside });
    dataM.push_back({ "count_mesh_inside", count_mesh_inside });
	instace.writeDataToCsv();
	return;
}

static void _clear_atomic()
{
	count_pre_clash = 0, count_mesh_intersect = 0, count_triangle_and_box_intersect = 0, count_two_triangle_intersect = 0, count_trianglebox_intersect = 0,
	count_preclash_A = 0, count_preclash_B = 0, count_judge_inside = 0, count_mesh_inside = 0;
}
#endif

//inside clash without depth, without tolerance
bool sat::isMeshInsideOtherMesh(const TriMesh& meshIn, const TriMesh& meshOut)
{
#ifdef STATISTIC_DATA_COUNT
	count_judge_inside++;
#endif
	constexpr double toleDist = 1e-8;
	auto _isPointOnTriangleSurface = [](const Vector3d& point, const std::array<Vector3d, 3>& trigon, const Vector3d& normal, double toleDist)->bool
		{
			//_isPointInTriangle
			if (point[0] + toleDist < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				point[0] - toleDist > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				point[1] + toleDist < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				point[1] - toleDist > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				point[2] + toleDist < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
				point[2] - toleDist > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
				toleDist < fabs(normal.dot(point - trigon[0])))
				return false;
			return
				-toleDist * (trigon[1] - trigon[0]).norm() < (trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) && //bool isLeftA
				-toleDist * (trigon[2] - trigon[1]).norm() < (trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) && //bool isLeftB
				-toleDist * (trigon[0] - trigon[2]).norm() < (trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal);   //bool isLeftC
		};
	auto _isRayAndTriangleIntersectParallel = [](const Vector3d& pnt, const Vector3d& dir, const std::array<Vector3d, 3 >& tri)->bool
		{
			//ray only one direction, negetive direction cause cross product result opposite
			return
				(0.0 <= (tri[0] - pnt).cross(dir).dot(dir.cross(tri[1] - pnt)) && 0.0 <= (tri[0] - pnt).cross(dir).dot((tri[0] - pnt).cross(tri[1] - pnt))) ||
				(0.0 <= (tri[1] - pnt).cross(dir).dot(dir.cross(tri[2] - pnt)) && 0.0 <= (tri[1] - pnt).cross(dir).dot((tri[1] - pnt).cross(tri[2] - pnt))) ||
				(0.0 <= (tri[2] - pnt).cross(dir).dot(dir.cross(tri[0] - pnt)) && 0.0 <= (tri[2] - pnt).cross(dir).dot((tri[2] - pnt).cross(tri[0] - pnt)));
		};
	auto _isPointInTriangle = [](const Vector3d& point, const std::array<Vector3d, 3>& trigon, const Vector3d& normal, double toleDist)->bool
		{
			if (point[0] + toleDist < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				point[0] - toleDist > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				point[1] + toleDist < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				point[1] - toleDist > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				point[2] + toleDist < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
				point[2] - toleDist > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]))
				return false; //using smaller eps
			return
				-toleDist * (trigon[1] - trigon[0]).norm() < (trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) && //bool isLeftA
				-toleDist * (trigon[2] - trigon[1]).norm() < (trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) && //bool isLeftB
				-toleDist * (trigon[0] - trigon[2]).norm() < (trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal);   //bool isLeftC
		};
	auto _isPointOnTriangleVertexOrEdge = [](const Vector3d& point, const std::array<Vector3d, 3 >& trigon, double toleDist)->bool
		{
			return //two points coincident, point on edge_line
				(point - trigon[0]).isZero(toleDist) ||
				(point - trigon[1]).isZero(toleDist) ||
				(point - trigon[2]).isZero(toleDist) ||
				(point - trigon[0]).cross(point - trigon[1]).isZero(toleDist * (trigon[0] - trigon[1]).norm()) ||
				(point - trigon[1]).cross(point - trigon[2]).isZero(toleDist * (trigon[1] - trigon[2]).norm()) ||
				(point - trigon[2]).cross(point - trigon[0]).isZero(toleDist * (trigon[2] - trigon[0]).norm());
		};
    if (meshIn.vbo_.empty() || meshIn.ibo_.empty())
		return false;
	Eigen::Vector3d point = gVecNaN;
	//get one inner point
	for (int i = 0; i < (int)meshIn.vbo_.size(); ++i)
	{
		bool isOnFace = false;
		//const Eigen::Vector3d& iter = meshIn.vbo_[i];
		for (int j = 0; j < (int)meshOut.ibo_.size(); ++j)
		{
			const std::array<Eigen::Vector3d, 3> trigon = {
				meshOut.vbo_[meshOut.ibo_[j][0]],
				meshOut.vbo_[meshOut.ibo_[j][1]],
				meshOut.vbo_[meshOut.ibo_[j][2]] };
			const Eigen::Vector3d& normal = meshOut.fno_[j];
			if (_isPointOnTriangleSurface(meshIn.vbo_[i], trigon, normal, toleDist))
			{
				isOnFace = true;
				break;
			}
		}
        if (isOnFace == false)
		{
			point = meshIn.vbo_[i];
			break;
		}
	}
	if (std::isnan(point[0])) //meshIn.vbo_ not empty
		return true; //all face coincide
	//ray method
	Vector3d direction = Vector3d(1.0, 1.0, 1.0); //random ray direction
	constexpr int maxCircle = 10;
	int newRayCount = 0;
	int countInter = 0;
	while (newRayCount < maxCircle) //avoid while-true
	{
		newRayCount++;
		bool isNewRay = false;//new create rand rayDir
		for (int j = 0; j < (int)meshOut.ibo_.size(); ++j)// iterate every trigon
		{
			const std::array<Eigen::Vector3d, 3> trigon = {
				meshOut.vbo_[meshOut.ibo_[j][0]],
				meshOut.vbo_[meshOut.ibo_[j][1]],
				meshOut.vbo_[meshOut.ibo_[j][2]] };
			const Eigen::Vector3d& normal = meshOut.fno_[j];
			double deno = direction.dot(normal); //ray.direction
			if (deno == 0.0)//ray direction is parallel, redo circulation
			{
				if ((point - trigon[0]).dot(normal) != 0.0) // not coplanar
					continue;
				if (!_isRayAndTriangleIntersectParallel(point, direction, trigon)) //coplanar
                    continue;
				direction = Vector3d(rand() - RAND_MAX, rand() - RAND_MAX, rand() - RAND_MAX).normalized();//move RAND_MAX/2
				isNewRay = true;
				countInter = 0;//reset
				break;
			}
			double k = (trigon[0] - point).dot(normal) / deno;
			if (k < 0.0) // only positive direction
				continue;
			Vector3d local = point + k * direction;
			if (!_isPointInTriangle(local, trigon, normal, toleDist))
				continue;
			if (_isPointOnTriangleVertexOrEdge(local, trigon, toleDist)) //singularity
			{
				direction = Vector3d(rand() - RAND_MAX, rand() - RAND_MAX, rand() - RAND_MAX).normalized();
				isNewRay = true;
				countInter = 0;//reset
				break;
			}
			countInter++; // ray across is true
		}
		if (isNewRay == false)
			break;//end while
	}
	return (countInter % 2 == 1);
}

//hard clash without tolerance
bool sat::isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	//isTwoTrianglesBoundingBoxIntersect
	if (std::max(std::max(triA[0][0], triA[1][0]), triA[2][0]) < std::min(std::min(triB[0][0], triB[1][0]), triB[2][0]) ||
		std::max(std::max(triB[0][0], triB[1][0]), triB[2][0]) < std::min(std::min(triA[0][0], triA[1][0]), triA[2][0]) ||
		std::max(std::max(triA[0][1], triA[1][1]), triA[2][1]) < std::min(std::min(triB[0][1], triB[1][1]), triB[2][1]) ||
		std::max(std::max(triB[0][1], triB[1][1]), triB[2][1]) < std::min(std::min(triA[0][1], triA[1][1]), triA[2][1]) ||
		std::max(std::max(triA[0][2], triA[1][2]), triA[2][2]) < std::min(std::min(triB[0][2], triB[1][2]), triB[2][2]) ||
		std::max(std::max(triB[0][2], triB[1][2]), triB[2][2]) < std::min(std::min(triA[0][2], triA[1][2]), triA[2][2]))
		return false;
	std::array<Eigen::Vector3d, 3> edgesA = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = {
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	Eigen::Vector3d normalA = edgesA[0].cross(edgesA[1]);
	Eigen::Vector3d normalB = edgesB[0].cross(edgesB[1]);
	if (normalA.cross(normalB).isZero())//isParallel3d, means not intrusive
		return false;
	std::array<Eigen::Vector3d, 11> axes = { {
		normalA, //normal direction projection
		normalB,
		edgesA[0].cross(edgesB[0]),//cross edge pair to get normal
		edgesA[0].cross(edgesB[1]),
		edgesA[0].cross(edgesB[2]),
		edgesA[1].cross(edgesB[0]),
		edgesA[1].cross(edgesB[1]),
		edgesA[1].cross(edgesB[2]),
		edgesA[2].cross(edgesB[0]),
		edgesA[2].cross(edgesB[1]),
		edgesA[2].cross(edgesB[2]) } };
	double minA, maxA, minB, maxB, projection;
	for (const Vector3d& axis : axes)
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Vector3d& vertex : triA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Vector3d& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA  < minB || maxB  < minA) //include equal make more return
			return false; //as long as one axis gap is separate
	}
	return true;
}

//must intersect, negative tolerance to filter tiny intersect
bool sat::isTwoTrianglesIntrusionSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance /*= 0.0*/)
{
	//isTwoTrianglesBoundingBoxIntersect
	if (std::max(std::max(triA[0][0], triA[1][0]), triA[2][0]) + tolerance < std::min(std::min(triB[0][0], triB[1][0]), triB[2][0]) ||
		std::max(std::max(triB[0][0], triB[1][0]), triB[2][0]) + tolerance < std::min(std::min(triA[0][0], triA[1][0]), triA[2][0]) ||
		std::max(std::max(triA[0][1], triA[1][1]), triA[2][1]) + tolerance < std::min(std::min(triB[0][1], triB[1][1]), triB[2][1]) ||
		std::max(std::max(triB[0][1], triB[1][1]), triB[2][1]) + tolerance < std::min(std::min(triA[0][1], triA[1][1]), triA[2][1]) ||
		std::max(std::max(triA[0][2], triA[1][2]), triA[2][2]) + tolerance < std::min(std::min(triB[0][2], triB[1][2]), triB[2][2]) ||
		std::max(std::max(triB[0][2], triB[1][2]), triB[2][2]) + tolerance < std::min(std::min(triA[0][2], triA[1][2]), triA[2][2]))
		return false;
	std::array<Eigen::Vector3d, 3> edgesA = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = {
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	Eigen::Vector3d normalA = edgesA[0].cross(edgesA[1]);
	Eigen::Vector3d normalB = edgesB[0].cross(edgesB[1]);
	if (normalA.cross(normalB).isZero())//isParallel3d, means not intrusive
		return false;
	std::array<Eigen::Vector3d, 11> axes = { {
		normalA.normalized(), //normal direction projection
		normalB.normalized(),
		edgesA[0].cross(edgesB[0]).normalized(),//cross edge pair to get normal
		edgesA[0].cross(edgesB[1]).normalized(),
		edgesA[0].cross(edgesB[2]).normalized(),
		edgesA[1].cross(edgesB[0]).normalized(),
		edgesA[1].cross(edgesB[1]).normalized(),
		edgesA[1].cross(edgesB[2]).normalized(),
		edgesA[2].cross(edgesB[0]).normalized(),
		edgesA[2].cross(edgesB[1]).normalized(),
		edgesA[2].cross(edgesB[2]).normalized() } };
	double minA, maxA, minB, maxB, projection;
	for (const Vector3d& axis : axes)
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Vector3d& vertex : triA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Vector3d& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA + tolerance < minB || maxB + tolerance < minA) //include equal make more return
			return false; //as long as one axis gap is separate
	}
	return true;
}

//without tolerance, critical contact is separate
bool sat::isTriangleAndBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& _box, double tolerance /*= 0.0*/)
{
#ifdef STATISTIC_DATA_COUNT
	count_triangle_and_box_intersect++;
#endif
	Eigen::AlignedBox3d box = _box; //boxMagnify
	if (tolerance != 0.0)
	{
		Vector3d toleSize = Vector3d(tolerance, tolerance, tolerance);
		box = Eigen::AlignedBox3d(_box.min() - toleSize, _box.max() + toleSize);//boxExtendTolerance
	}
	//pre-judge
	if (box.contains(trigon[0]) || box.contains(trigon[1]) || box.contains(trigon[2]))
		return true;
	const Vector3d& min = box.min();
	const Vector3d& max = box.max();
	//extreme value filter
	if (std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) < min[0] ||
		std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) > max[0] ||
		std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) < min[1] ||
		std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) > max[1] ||
		std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]) < min[2] ||
		std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) > max[2])
		return false;
	// Separating Axis Theorem
	std::array<Eigen::Vector3d, 3> edges = {
		trigon[1] - trigon[0],
		trigon[2] - trigon[1],
		trigon[0] - trigon[2] };
	std::array<Eigen::Vector3d, 3> coords = {
		Vector3d(1.,0.,0.),
		Vector3d(0.,1.,0.),
		Vector3d(0.,0.,1.) };
	std::array<Eigen::Vector3d, 10> axes = { {
		edges[0].cross(edges[1]), //trigon normal
		coords[0].cross(edges[0]),
		coords[0].cross(edges[1]),
		coords[0].cross(edges[2]),
		coords[1].cross(edges[0]),
		coords[1].cross(edges[1]),
		coords[1].cross(edges[2]),
		coords[2].cross(edges[0]),
		coords[2].cross(edges[1]),
		coords[2].cross(edges[2]) } };
	const Vector3d& origin = box.min();
	const Vector3d vertex = box.sizes(); //m_max - m_min
	std::array<Vector3d, 8> vertexes = { {
		Vector3d(0, 0, 0),
		Vector3d(vertex[0], 0, 0),
		Vector3d(vertex[0], vertex[1], 0),
		Vector3d(0, vertex[1], 0),
		Vector3d(0, 0, vertex[2]),
		Vector3d(vertex[0], 0, vertex[2]),
		Vector3d(vertex[0], vertex[1], vertex[2]),
		Vector3d(0, vertex[1], vertex[2]) } };
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) //fast than index
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& iter : trigon)
		{
			projection = (iter - origin).dot(axis);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& iter : vertexes)
		{
			projection = iter.dot(axis);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB || maxB < minA) // absolute zero, contact not intersect
			return false;
	}
	return true;
}

//tolerance keep with hardclash
static std::array<std::vector<int>, 2> trianglesAndCommonBoxPreclash(const TriMesh& meshA, const TriMesh& meshB, double tolerance)
{
//#define USING_SECOND_PRECLASH //test shows not faster
	Eigen::AlignedBox3d boxCom = meshA.bounding_.intersection(meshB.bounding_); // box common
	Eigen::AlignedBox3d boxMag = boxCom;
	if (0.0 < tolerance)
	{
		Vector3d toleSize = Vector3d(tolerance, tolerance, tolerance);  //to accelerate calculation, pre extend box
		boxMag = Eigen::AlignedBox3d(boxCom.min() - toleSize, boxCom.max() + toleSize);//boxExtendTolerance
	}
#ifdef USING_SECOND_PRECLASH
	Eigen::AlignedBox3d boxA;
	Eigen::AlignedBox3d boxB;
#endif
	std::vector<int> indexA;
	for (int i = 0; i < (int)meshA.ibo_.size(); ++i)
	{
		const std::array<Eigen::Vector3d, 3> triA = { 
			meshA.vbo_[meshA.ibo_[i][0]],
			meshA.vbo_[meshA.ibo_[i][1]],
			meshA.vbo_[meshA.ibo_[i][2]] };
		if (sat::isTriangleAndBoxIntersectSAT(triA, boxMag))
		{
			indexA.push_back(i);
#ifdef USING_SECOND_PRECLASH
			boxA.extend(triA[0]);
			boxA.extend(triA[1]);
			boxA.extend(triA[2]);
#endif
		}
	}
	if (indexA.empty())
		return {};
	std::vector<int> indexB;
	for (int j = 0; j < (int)meshB.ibo_.size(); ++j)
	{
		const std::array<Eigen::Vector3d, 3> triB= { 
			meshB.vbo_[meshB.ibo_[j][0]],
			meshB.vbo_[meshB.ibo_[j][1]],
			meshB.vbo_[meshB.ibo_[j][2]] };
		if (sat::isTriangleAndBoxIntersectSAT(triB, boxMag))
		{
			indexB.push_back(j);
#ifdef USING_SECOND_PRECLASH
			boxB.extend(triB[0]);
			boxB.extend(triB[1]);
			boxB.extend(triB[2]);
#endif
		}
	}
	if (indexB.empty())
		return {};
#ifdef USING_SECOND_PRECLASH
	std::vector<int> indexARes;
	for (const int& iA : indexA)
	{
		std::array<Eigen::Vector3d, 3> triA = {
			meshA.vbo_[meshA.ibo_[iA][0]],
			meshA.vbo_[meshA.ibo_[iA][1]],
			meshA.vbo_[meshA.ibo_[iA][2]] };
		if (isTriangleAndBoxIntersectSAT(triA, boxB, tolerance))
			indexARes.push_back(iA);
	}
	std::vector<int> indexBRes;
	for (const int& iB : indexB)
	{
		std::array<Eigen::Vector3d, 3> triB = {
			meshB.vbo_[meshB.ibo_[iB][0]],
			meshB.vbo_[meshB.ibo_[iB][1]],
			meshB.vbo_[meshB.ibo_[iB][2]] };
		if (isTriangleAndBoxIntersectSAT(triB, boxA, tolerance))
			indexBRes.push_back(iB);
	}
	return { indexARes, indexBRes };
#endif
#ifndef USING_SECOND_PRECLASH
	return { indexA, indexB };
#endif
}

bool sat::isTwoMeshsIntersectSAT(const TriMesh& meshA, const TriMesh& meshB, double tolerance /*= 0.0*/)
{
	const std::array<std::vector<int>, 2> indexAB = trianglesAndCommonBoxPreclash(meshA, meshB, tolerance);// first pre-judge
#ifdef STATISTIC_DATA_COUNT
	count_preclash_A += indexAB[0].size();
	count_preclash_B += indexAB[1].size();
#endif
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		for (const int& iA : indexAB[0])
		{
			const std::array<Eigen::Vector3d, 3> triA = {
				meshA.vbo_[meshA.ibo_[iA][0]],
				meshA.vbo_[meshA.ibo_[iA][1]],
				meshA.vbo_[meshA.ibo_[iA][2]] };
			std::vector<int> contactFace;
			for (const int& iB : indexAB[1])
			{
				const std::array<Eigen::Vector3d, 3> triB = {
					meshB.vbo_[meshB.ibo_[iB][0]],
					meshB.vbo_[meshB.ibo_[iB][1]],
					meshB.vbo_[meshB.ibo_[iB][2]] };
#ifdef STATISTIC_DATA_COUNT
				count_two_triangle_intersect++;
#endif
				if (sat::isTwoTrianglesIntrusionSAT(triA, triB, tolerance)) //origin isTwoTrianglesIntersectSAT
				{
					return true;
				}
				// to distinguish only face-contact intersect
				if (tolerance < 0.0 && meshA.convex_ && meshB.convex_ && 
					sat::isTwoTrianglesIntersectSAT(triA, triB))
				{
					contactFace.push_back(iB);
				}
			}
			if (contactFace.empty()) //both convex
				continue;
			//judge whether two side using sat
			const Vector3d& normalA = meshA.fno_[iA];
			bool hasPosi = false, hasNega = false;
			for (const int& fB : contactFace)
			{
				for (int i = 0; i < 3; ++i)
				{
					double projection = normalA.dot(meshB.vbo_[meshB.ibo_[fB][i]] - triA[0]);
					if (0.0 == projection) //on face
						continue;
					if (0.0 < projection)
						hasPosi = true;
					else if (0.0 > projection && projection <= tolerance) //default normal direction out
						hasNega = true;
                    if (hasPosi && hasNega)
						return true;
				}
			}
		}
	}
	//judge whether mesh entirely inside other mesh
	if (meshA.bounding_.contains(meshB.bounding_))
	{
        if (sat::isMeshInsideOtherMesh(meshB, meshA))
		{
#ifdef STATISTIC_DATA_COUNT
			count_mesh_inside++;
#endif
			return true;
		}
	}
	else if (meshB.bounding_.contains(meshA.bounding_))
	{
		if (sat::isMeshInsideOtherMesh(meshA, meshB))
		{
#ifdef STATISTIC_DATA_COUNT
			count_mesh_inside++;
#endif			
			return true;
		}
	}
	return false;
}

// tolerance > 0 is soft clash, include distance < tolerance
// tolerance <= 0 is hard clash, exclude abs(distance) < abs(tolerance)
std::vector<std::pair<int, int>> ClashDetection::executeAssignClashDetection(const std::vector<int>& indexesLeft, const std::vector<int>& indexesRight, 
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
#ifdef STATISTIC_DATA_COUNT
	count_mesh = sm_meshStore.size();
#endif
	if (sm_meshStore.empty())
		return {};
	std::vector<Polyface3d> polyfaceVct(sm_meshStore.size());// simpified TriMesh
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)sm_meshStore.size(); ++i)
	{
		polyfaceVct[i].m_index = i; //keep order
		polyfaceVct[i].m_bound = sm_meshStore[i].bounding_;
#ifdef STATISTIC_DATA_COUNT
        count_triangle += sm_meshStore[i].ibo_.size();
		count_vertex += sm_meshStore[i].vbo_.size();
#endif
	}
	bvh::BVHTree3d bvhtree(polyfaceVct);//create spatial search tree
	set<std::pair<int, int>> signRecord;
	std::vector<std::pair<int, int>> clashRes;
#ifdef _OPENMP 
	int numThreads = omp_get_max_threads() - 1;
	int maxThreads = std::max(numThreads, 1);
	omp_set_num_threads(maxThreads);
#endif //_OPENMP
#pragma omp parallel for //schedule(dynamic)
	for (int i = 0; i < (int)indexesLeft.size(); ++i)
	{
#pragma omp critical
		{
			if (callback != nullptr)
				callback(100.f * i / (float)indexesLeft.size(), (int)clashRes.size()); // progress bar | result count
		}
		const int iL = indexesLeft[i];
		if (iL < 0 || sm_meshStore.size() <= iL) //check
			continue;
		const std::vector<int> preClash = bvhtree.findIntersect(polyfaceVct[iL], tolerance);
#ifdef STATISTIC_DATA_COUNT
		count_pre_clash += preClash.size();
#endif
		const TriMesh& meshL = sm_meshStore[iL];
		for (int j = 0; j < (int)indexesRight.size(); ++j)
		{
			const int iR = indexesRight[j];
			if (iR < 0 || sm_meshStore.size() <= iR || //check
				iL == iR || //skip self
				std::find(preClash.begin(), preClash.end(), iR) == preClash.end())
			{
				continue;
			}
			std::pair<int, int> current = (iL < iR) ? pair<int, int>{iL, iR} : pair<int, int>{ iR, iL };
			if (signRecord.find(current) != signRecord.end()) //avoid repeat
				continue;
#pragma omp critical
			{
				signRecord.insert(current);
			}
			const TriMesh& meshR = sm_meshStore[iR];
#ifdef STATISTIC_DATA_COUNT
			count_mesh_intersect++;
#endif
			if (!sat::isTwoMeshsIntersectSAT(meshL, meshR, tolerance))
				continue;
#pragma omp critical
			{
				if (meshL.index_ == -1 || meshR.index_ == -1) //invalid index
					clashRes.push_back(current);
				else
					clashRes.push_back({ meshL.index_, meshR.index_ });
			}
		}
	}
#ifdef STATISTIC_DATA_COUNT
	count_clash = clashRes.size();
	_output_data();
	_clear_atomic();
#endif
	return clashRes;
}

std::vector<std::pair<int, int>> ClashDetection::executeFullClashDetection(const std::vector<TriMesh>& meshVct, 
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	sm_meshStore = meshVct; //copy data
	std::vector<Polyface3d> polyfaceVct(sm_meshStore.size());// simpified TriMesh
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)sm_meshStore.size(); ++i)
	{
		polyfaceVct[i].m_index = i; //keep order
		polyfaceVct[i].m_bound = sm_meshStore[i].bounding_;
	}
	bvh::BVHTree3d bvhtree(polyfaceVct);//create spatial search tree
	std::vector<std::pair<int, int>> clashRes;
#ifdef _OPENMP 
	int numThreads = omp_get_max_threads() - 1;
	int maxThreads = std::max(numThreads, 1);
	omp_set_num_threads(maxThreads);
#endif //_OPENMP
#pragma omp parallel for //schedule(dynamic)
	for (int i = 0; i < (int)sm_meshStore.size(); ++i)
	{
#pragma omp critical
		{
			if (callback != nullptr)
				callback(100.f * i / (float)sm_meshStore.size(), (int)clashRes.size()); // progress bar | result count
		}
		const TriMesh& meshL = sm_meshStore[i];
		const std::vector<int> preClash = bvhtree.findIntersect(polyfaceVct[i], tolerance);
        for (const int& j : preClash)
		{
            if (j <= i) //also canbe exclude in findIntersect;
				continue;
			const TriMesh& meshR = sm_meshStore[j];
			if (!sat::isTwoMeshsIntersectSAT(meshL, meshR, tolerance))
				continue;
#pragma omp critical
			{
				if (meshL.index_ == -1 || meshR.index_ == -1) //invalid index
                    clashRes.push_back({ i,j });
				else
					clashRes.push_back({ meshL.index_, meshR.index_ });
			}
		}
	}
	sm_meshStore.clear(); //clearMeshs
	return clashRes;
}

std::vector<std::pair<int, int>> ClashDetection::executePairClashDetection(const std::vector<TriMesh>& meshsLeft, const std::vector<TriMesh>& meshsRight,
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	sm_meshStore = meshsLeft; //addMeshs
	sm_meshStore.insert(sm_meshStore.end(), meshsRight.begin(), meshsRight.end());
	std::vector<int> indexesLeft(meshsLeft.size());
	for (int i = 0; i < (int)meshsLeft.size(); ++i)
		indexesLeft[i] = i;
	std::vector<int> indexesRight(meshsRight.size());
	int begin = (int)indexesLeft.size();
	for (int i = 0; i < (int)meshsRight.size(); ++i)
		indexesRight[i] = i + begin;
	std::vector<std::pair<int, int>> res = ClashDetection::executeAssignClashDetection(indexesLeft, indexesRight, tolerance, callback);
	sm_meshStore.clear(); //clearMeshs
	return res;
}

//create a whole trigons tree, contain all meshs trigon (maybe there is wrong way)
std::vector<std::pair<int, int>> ClashDetection::executeFullClashDetectionByTrigon(const std::vector<TriMesh>& meshVct,
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	std::vector<RectBase3d> allTrgions;
//#pragma omp parallel for schedule(dynamic)
	int k = 0; //global index
	vector<vector<int>> meshTrigon(meshVct.size());
	for (int i = 0; i < (int)meshVct.size(); ++i) //without mesh store
	{
		const TriMesh& mesh = meshVct[i];
		std::vector<RectBase3d> tempTri(mesh.ibo_.size()); //add ibo face to boundbox
		vector<int> tempId(mesh.ibo_.size());
		for (int j = 0; j < (int)mesh.ibo_.size(); ++j)
		{
			AlignedBox3d box;
			box.extend(mesh.vbo_[mesh.ibo_[j][0]]);
			box.extend(mesh.vbo_[mesh.ibo_[j][1]]);
			box.extend(mesh.vbo_[mesh.ibo_[j][2]]);
			tempTri[j].m_bound = box;
			tempTri[j].m_number = i; //mesh index
			tempTri[j].m_index = k++; //unique index
			tempId[j] = tempTri[j].m_index;
		}
		allTrgions.insert(allTrgions.end(), tempTri.begin(), tempTri.end());
		meshTrigon[i] = tempId;
	}
	//create tree
	bvh::RectBaseTree3d tree3d(allTrgions);
	std::vector<std::pair<int, int>> res;
	for (int i = 0; i < (int)meshTrigon.size(); ++i)
	{
		bool isInter = false;
		set<int> interMesh;
		for (int j = 0; j < (int)meshTrigon[i].size(); ++j)
		{
			const TriMesh& meshA = meshVct[i];
			std::array<Eigen::Vector3d, 3> triA = {
				meshA.vbo_[meshA.ibo_[j][0]],
				meshA.vbo_[meshA.ibo_[j][1]],
				meshA.vbo_[meshA.ibo_[j][2]] };
			const RectBase3d& iter = allTrgions[meshTrigon[i][j]];
            const std::vector<std::array<int, 2>> preInter = tree3d.findIntersect(iter, tolerance); //trigon index
			for (const auto& k : preInter)
			{
				const TriMesh& meshB = meshVct[k[1]];
				std::array<Eigen::Vector3d, 3> triB = {
					meshB.vbo_[meshB.ibo_[k[0]][0]],
					meshB.vbo_[meshB.ibo_[k[0]][1]],
					meshB.vbo_[meshB.ibo_[k[0]][2]] };
                if (interMesh.find(k[1]) != interMesh.end() ||
					!isTwoTrianglesIntrusionSAT(triA, triB))
					continue;
				//intersect
				isInter = true;
				interMesh.insert(k[1]);
				res.push_back({ i,k[1] });//record
				break;
			}
			if (isInter)
				break;
		}
	}
	//inside judge

	return res;
}


//--------------------------------------------------------------------------------------------------
//  namespace accura
//--------------------------------------------------------------------------------------------------

bool accura::isTwoTrianglesPenetrationSAT(const std::array<Vector2d, 3>& triA, const std::array<Vector2d, 3>& triB, double toleDist, double toleAngle)
{
	// bounding-box been judged
	std::array<Eigen::Vector2d, 6> edgesAB = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2],
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	std::array<Vector2d, 3> triA_R, triB_R; //using relative coordinate
	double spec = 0; // using max-coord k as coeff
	for (int i = 0; i < 3; ++i)
	{
		triA_R[i] = triA[i] - triA[0]; //triA_R[0]=Vector3d(0,0,0)
		triB_R[i] = triB[i] - triA[0];
		spec = std::max(std::max(fabs(triA_R[i][0]), fabs(triA_R[i][1])), spec);
		spec = std::max(std::max(fabs(triB_R[i][0]), fabs(triB_R[i][1])), spec);
	}
	spec = spec * toleDist;
	for (auto& iter : edgesAB)
		iter = Vector2d(-iter[1], iter[0]); //rotz(pi/2)
	double minA, maxA, minB, maxB, projection;
	for (auto& axis : edgesAB)
	{
		if (axis.isZero(toleAngle)) //degeneracy triangle
			continue;
		axis.normalize();
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : triA_R)
		{
			projection = axis.dot(vertex - triA[0]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB_R)
		{
			projection = axis.dot(vertex - triA[0]);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + spec || maxB < minA + spec)
			return false;
	}
	return true;
}

//#define USING_COMPLETE_SEPARATION_AXIS
bool accura::isTwoTrianglesPenetrationSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB,
	const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB, double toleDist, double toleAngle)
{
	// bounding-box been judged, both triangle is legal
	std::array<Eigen::Vector3d, 3> edgesA = {
		(triA[1] - triA[0]).normalized(),
		(triA[2] - triA[1]).normalized(),
		(triA[0] - triA[2]).normalized() };
	std::array<Eigen::Vector3d, 3> edgesB = {
		(triB[1] - triB[0]).normalized(),
		(triB[2] - triB[1]).normalized(),
		(triB[0] - triB[2]).normalized() };
	std::array<Vector3d, 3> triA_R, triB_R; //using relative coordinate
	double spec = 0; // using max-coord k as coeff
	for (int i = 0; i < 3; ++i)
	{
		triA_R[i] = triA[i] - triA[0]; //triA_R[0]=Vector3d(0,0,0)
		triB_R[i] = triB[i] - triA[0];
		for (int j = 0; j < 3; ++j)
			spec = std::max(std::max(fabs(triA_R[i][j]), fabs(triB_R[i][j])), spec);
	}
	spec = spec * toleDist;
	double minA, maxA, minB, maxB, projection;
	//Eigen::Vector3d normalA = edgesA[0].cross(edgesA[1]).normalized();
	//Eigen::Vector3d normalB = edgesB[0].cross(edgesB[1]).normalized();
	if (normalA.cross(normalB).isZero(toleAngle)) //is parallel
	{
#ifdef USING_COMPLETE_SEPARATION_AXIS 
		if (!fabs(normalA.dot(triB_R[0])) < spec) //not coplanar
			return false;
		std::array<Eigen::Vector3d, 6> edgesAB = { edgesA[0],edgesA[1],edgesA[2],edgesB[0],edgesB[1],edgesB[2] };
		for (const auto& axis : edgesAB)
		{
			if (axis.isZero(tolerance))
				continue;
			minA = DBL_MAX;
			maxA = -DBL_MAX;
			minB = DBL_MAX;
			maxB = -DBL_MAX;
			for (const auto& vertex : triA_R)
			{
				projection = axis.dot(vertex);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			for (const auto& vertex : triB_R)
			{
				projection = axis.dot(vertex);
				minB = std::min(minB, projection);
				maxB = std::max(maxB, projection);
			}
			if (maxA < minB + spec || maxB < minA + spec)
				return false;
		}
		return true;
#else
		return false;//exclude coplanar
#endif
	}
	//separation axis theorem
	std::array<Eigen::Vector3d, 11> axes = { {
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
	for (const auto& axis : axes)
	{
		if (axis.isZero(toleAngle))
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : triA_R)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB_R)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + spec || maxB < minA + spec)
			return false;
	}
	return true;
}

bool accura::isTwoTrianglesPenetrationSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB, double toleDist, double toleAngle)
{
	Eigen::Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[1]).normalized();
	Eigen::Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[1]).normalized();
	return isTwoTrianglesPenetrationSAT(triA, triB, normalA, normalB, toleDist, toleAngle);
}

bool accura::isPointInTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon, double tolerance) // 2D
{
	//tole>0 less judge, tole<0 more judge
	std::array<Eigen::Vector2d, 3> trigonR; // is origin inside trigonR
	double spec = 0;
	for (int i = 0; i < 3; ++i)
	{
		trigonR[i] = trigon[i] - point;
		spec = std::max(std::max(fabs(trigonR[i][0]), fabs(trigonR[i][1])), spec);
	}
	spec = spec * tolerance; // assert(spec > tolerance);
	if (-spec < std::min(std::min(trigonR[0][0], trigonR[1][0]), trigonR[2][0]) ||
		spec > std::max(std::max(trigonR[0][0], trigonR[1][0]), trigonR[2][0]) ||
		-spec < std::min(std::min(trigonR[0][1], trigonR[1][1]), trigonR[2][1]) ||
		spec > std::max(std::max(trigonR[0][1], trigonR[1][1]), trigonR[2][1])) //box judge
		return false;
	// been legal judge, no zero vector
	Eigen::Vector2d v0 = (trigon[1] - trigon[0]).normalized();
	Eigen::Vector2d v1 = (trigon[2] - trigon[1]).normalized();
	Eigen::Vector2d v2 = (trigon[0] - trigon[2]).normalized();
	// (p1-p0).cross(p2-p1)
	double axisz = v0[0] * v1[1] - v0[1] * v1[0];
	axisz = (0.0 < axisz) ? 1.0 : -1.0;
	return //cross2d isLeft judge, (trigon[i] - point) is negative direction
		spec <= axisz * (v0[1] * trigonR[0][0] - v0[0] * trigonR[0][1]) &&
		spec <= axisz * (v1[1] * trigonR[1][0] - v1[0] * trigonR[1][1]) &&
		spec <= axisz * (v2[1] * trigonR[2][0] - v2[0] * trigonR[2][1]); // = decide whether include point on edge
}

bool accura::isTwoSegmentsIntersect(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB, double tolerance)
{
	//tole<0 less judge, tole>0 more judge
	Vector2d vecA = segmA[1] - segmA[0];
	Vector2d vecB = segmB[1] - segmB[0];
	double spec = 0;
	spec = std::max(std::max(fabs(vecA[0]), fabs(vecA[1])), spec);
	spec = std::max(std::max(fabs(vecB[0]), fabs(vecB[1])), spec);
	spec = spec * tolerance;// assert(spec > tolerance);
	if (spec < std::min(segmB[0][0], segmB[1][0]) - std::max(segmA[0][0], segmA[1][0]) ||
		spec < std::min(segmA[0][0], segmA[1][0]) - std::max(segmB[0][0], segmB[1][0]) ||
		spec < std::min(segmB[0][1], segmB[1][1]) - std::max(segmA[0][1], segmA[1][1]) ||
		spec < std::min(segmA[0][1], segmA[1][1]) - std::max(segmB[0][1], segmB[1][1]))
		return false;
	Vector2d AB_0 = segmB[0] - segmA[0];
	Vector2d AB_1 = segmB[1] - segmA[0];
	Vector2d BA_0 = segmA[0] - segmB[0];
	Vector2d BA_1 = segmA[1] - segmB[0];
	return //double straddle test, cross2d opposite direction
		(AB_0[0] * vecA[1] - AB_0[1] * vecA[0]) * (AB_1[0] * vecA[1] - AB_1[1] * vecA[0]) <= spec &&
		(BA_0[0] * vecB[1] - BA_0[1] * vecB[0]) * (BA_1[0] * vecB[1] - BA_1[1] * vecB[0]) <= spec; //not support both near zero
}

