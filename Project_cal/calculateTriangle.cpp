#include "pch.h"
#include "calculateTriangle.h"
using namespace std;
using namespace Eigen;
using namespace psykronix;
#ifdef max
#undef max 
#endif // max
#ifdef min
#undef min 
#endif // min
#define USING_METHOD_SAT
//#define USING_THRESHOLD_CUSTOMIZE
//#define USING_ACCURATE_NORMALIZED
//#define USING_THRESHOLD_GEOMETRIC // to process collinar and coplanar

//--------------------------------------------------------------------------------------------------
//  triangle
//--------------------------------------------------------------------------------------------------

#ifdef STATISTIC_DATA_COUNT
std::atomic<size_t> count_isTwoTrisInter = 0, count_getTrisDistance = 0, count_isTrisBoundBoxInter = 0, count_isTrisAndBoxInter = 0,
count_pointInTri = 0, count_err_degen_tri = 0,
count_err_inputbox = 0, count_err_inter_dist = 0, count_err_repeat_tri = 0,
count_err_tris_sepa = 0, count_err_tris_inter = 0;
extern std::atomic<size_t> count_err_degen_tri, count_trigon_coplanar;
#endif //STATISTIC_DATA_COUNT
#ifdef STATISTIC_DATA_TESTFOR
extern std::atomic<size_t> count_gRTT;
#endif

bool psykronix::isParallel(const Vector2d& vecA, const Vector2d& vecB/*, double tole = 0*/)
{
	// dynamic accuracy
	double tole = DBL_EPSILON * std::max(vecA.squaredNorm(), vecB.squaredNorm() + 1.0);
	return fabs(vecA[0] * vecB[1] - vecA[1] * vecB[0]) < tole;
}
bool psykronix::isParallel(const Vector3d& vecA, const Vector3d& vecB/*, double tole = 0*/)
{
	// dynamic accuracy
	double tole = DBL_EPSILON * std::max(vecA.squaredNorm(), vecB.squaredNorm() + 1.0);
	return vecA.cross(vecB).isZero(tole);
}

bool psykronix::isPerpendi(const Vector2d& vecA, const Vector2d& vecB/*, double tole = 0*/)
{
	// dynamic accuracy
	double tole = DBL_EPSILON * std::max(vecA.squaredNorm(), vecB.squaredNorm() + 1.0); //avoid all zero
	return vecA.dot(vecB) < tole;
}
bool psykronix::isPerpendi(const Vector3d& vecA, const Vector3d& vecB/*, double tole = 0*/)
{
	// dynamic accuracy
	double tole = DBL_EPSILON * std::max(vecA.squaredNorm(), vecB.squaredNorm() + 1.0); //avoid all zero
	return vecA.dot(vecB) < tole;
}

double psykronix::computeTriangleArea(const std::array<Vector2d, 3>& triangle)
{
	Vector2d a = triangle[1] - triangle[0];
	Vector2d b = triangle[2] - triangle[0];
	return 0.5 * fabs(a[0] * b[1] - a[1] * b[0]); //cross value
}

double psykronix::computeTriangleArea(const std::array<Vector3d, 3>& triangle, bool is2D /*= true*/) //is2D means OnXoY
{
	Vector3d AB = triangle[1] - triangle[0];
	Vector3d AC = triangle[2] - triangle[0];
	if (is2D)
	{
		AB[2] = 0.0; //to xoy plane
		AC[2] = 0.0;
	}
	return 0.5 * AB.cross(AC).norm();
}

//isPointInTriangle2D
bool psykronix::isPointInTriangle(const Vector2d& point, const std::array<Vector2d, 3>& trigon) // 2D
{
	// using isLeft test
	const Vector2d& p0 = trigon[0];
	const Vector2d& p1 = trigon[1];
	const Vector2d& p2 = trigon[2];
	// compare z-value
#ifdef USING_THRESHOLD_GEOMETRIC
	double axisz = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
	return !(
		axisz * ((p1.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p1.y() - p0.y()) < _eps) || //bool isLeftA
		axisz * ((p2.x() - p1.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (p2.y() - p1.y()) < _eps) || //bool isLeftB
		axisz * ((p0.x() - p2.x()) * (point.y() - p2.y()) - (point.x() - p2.x()) * (p0.y() - p2.y()) < _eps));  //bool isLeftC
#else
	// (p1-p0).cross(p2-p1)
	double axisz = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
	return
		0.0 < axisz * ((p1[0] - p0[0]) * (point[1] - p0[1]) - (point[0] - p0[0]) * (p1[1] - p0[1])) &&
		0.0 < axisz * ((p2[0] - p1[0]) * (point[1] - p1[1]) - (point[0] - p1[0]) * (p2[1] - p1[1])) &&
		0.0 < axisz * ((p0[0] - p2[0]) * (point[1] - p2[1]) - (point[0] - p2[0]) * (p0[1] - p2[1]));
#endif
}

// using BarycentricCoordinates
bool isPointInTriangleBC(const Vector2d& point, const std::array<Vector2d, 3>& triangle)
{
	Eigen::Vector2d v0 = triangle[1] - triangle[0];
	Eigen::Vector2d v1 = triangle[2] - triangle[0];
	Eigen::Vector2d v2 = point - triangle[0];
	double d00 = v0.dot(v0);
	double d01 = v0.dot(v1);
	double d11 = v1.dot(v1);
	double d20 = v2.dot(v0);
	double d21 = v2.dot(v1);
	double denom = d00 * d11 - d01 * d01;
	//if (denom==0) //willnot happen, unless triangle degeneracy
	double a = (d11 * d20 - d01 * d21) / denom;
	double b = (d00 * d21 - d01 * d20) / denom;
	double c = 1.0 - a - b;
	return 0.0 < a && 0.0 < b && 0.0 < c;
}

bool isPointInTriangleSAT(const Vector2d& point, const std::array<Vector2d, 3>& triangle)
{
	std::array<Vector2d, 3> axes = {
		triangle[1] - triangle[0],
		triangle[2] - triangle[1],
		triangle[0] - triangle[2] };
	for (auto& axis : axes)
		axis = Vector2d(-axis[1], axis[0]);
	double min = DBL_MAX, max = -DBL_MAX, projection;
	for (const auto& axis : axes)
	{
		for (const auto& vertex : triangle)
		{
			projection = axis.dot(vertex);
			min = std::min(min, projection);
			max = std::max(max, projection);
		}
		projection = axis.dot(point);
		if (max < projection || projection < min)
			return false;
	}
	return true;
}

//must coplanar
bool psykronix::isPointInTriangle(const Vector3d& point, const std::array<Vector3d, 3>& trigon) //3D
{
#ifdef STATISTIC_DATA_COUNT
	count_pointInTri++;
#endif
	// precision problem, cause misjudge without tolerance
	//if (point.x() < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) /*+ _eps*/ ||
	//	point.x() > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) /*+ eps */ ||
	//	point.y() < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) /*+ _eps*/ || 
	//	point.y() > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) /*+ eps */ ||
	//	point.z() < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) /*+ _eps*/ || 
	//	point.z() > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]) /*+ eps */ )
	//	return false; // random data test fast 10% about
#ifdef USING_ACCURATE_NORMALIZED
	Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]).normalized();// using isLeft test
#else
	Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]);// using isLeft test
#endif // USING_ACCURATE_NORMALIZED
#ifdef USING_THRESHOLD_GEOMETRIC
	// 3D triangle, input point must coplanar
	return !(
		(trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) < _eps || //bool isNotLeftA
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) < _eps || //bool isNotLeftB
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal) < _eps);  //bool isNotLeftC
#else
	return
		0.0 <= (trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) && //bool isLeftA
		0.0 <= (trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) && //bool isLeftB
		0.0 <= (trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal);   //bool isLeftC
#endif //USING_THRESHOLD_GEOMETRIC
	//+-*x< 0,5,1.5,2.5,1.5
	//if (((trigon[1] - trigon[0]).cross(point - trigon[0])).dot(normal) < _eps) //bool isLeftA
	//	return false;
	//if (((trigon[2] - trigon[0]).cross(point - trigon[0])).dot(normal) > eps) //bool isLeftB
	//	return false;
	//if (((trigon[2] - trigon[1]).cross(point - trigon[1])).dot(normal) < _eps) //bool isLeftC
	//	return false;
	//return true;
	// area method
	//double dot1 = normal.dot((trigon[1] - trigon[0]).cross(point - trigon[0]));
	//double dot2 = normal.dot((point - trigon[0]).cross(trigon[2] - trigon[0]));
	//return 0.0 <= dot1 && 0.0 <= dot2 && dot1 + dot2 <= normal.dot(normal);
}

bool isPointOnPlane(const Eigen::Vector3d& point, const psykronix::Plane3d& plane)
{
	return isPerpendi(point - plane.m_origin, plane.m_normal);
}

bool isPointOnTriangleSurface(const Vector3d& point, const std::array<Vector3d, 3>& trigon)
{
	if (point[0] < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
		point[0] > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
		point[1] < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
		point[1] > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
		point[2] < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
		point[2] > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]))
		return false;
#ifdef USING_ACCURATE_NORMALIZED
	Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]).normalized();// using isLeft test
#else
	Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]);// using isLeft test
#endif // USING_ACCURATE_NORMALIZED
#ifdef USING_THRESHOLD_GEOMETRIC
	if ((trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) < _eps ||
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) < _eps ||
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal) < _eps)
		return false;
	return fabs(normal.dot(point - trigon[0])) < eps;
#else
	if ((trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) < 0.0 ||
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) < 0.0 ||
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal) < 0.0)
		return false;
	return normal.dot(point - trigon[0]) == 0.0;
#endif
}

//isTwoSegmentsIntersect2D
bool psykronix::isTwoSegmentsIntersect(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
{
	// segmA's two point on both sides of segmB
	// (segmB[0] - segmA[0])x(segmA[1] - segmA[0])*(segmA[1] - segmA[0])x(segmB[1] - segmA[0])>=0
	// (segmA[0] - segmB[0])x(segmB[1] - segmB[0])*(segmB[1] - segmB[0])x(segmA[1] - segmB[0])>=0
	if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) ||
		std::min(segmA[0][0], segmA[1][0]) > std::max(segmB[0][0], segmB[1][0]) ||
		std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) ||
		std::min(segmA[0][1], segmA[1][1]) > std::max(segmB[0][1], segmB[1][1]))
		return false;
#ifdef USING_THRESHOLD_GEOMETRIC
	bool isBetweenB = ((segmB[0] - segmA[0]).x() * (segmA[1] - segmA[0]).y() - (segmA[1] - segmA[0]).x() * (segmB[0] - segmA[0]).y()) *
		((segmA[1] - segmA[0]).x() * (segmB[1] - segmA[0]).y() - (segmB[1] - segmA[0]).x() * (segmA[1] - segmA[0]).y()) > _eps;
	bool isBetweenA = ((segmA[0] - segmB[0]).x() * (segmB[1] - segmB[0]).y() - (segmB[1] - segmB[0]).x() * (segmA[0] - segmB[0]).y()) *
		((segmB[1] - segmB[0]).x() * (segmA[1] - segmB[0]).y() - (segmA[1] - segmB[0]).x() * (segmB[1] - segmB[0]).y()) > _eps;
	return isBetweenB && isBetweenA;
#else
	return // double point on line judge
		((segmB[0] - segmA[0]).x() * (segmA[1] - segmA[0]).y() - (segmA[1] - segmA[0]).x() * (segmB[0] - segmA[0]).y()) *
		((segmA[1] - segmA[0]).x() * (segmB[1] - segmA[0]).y() - (segmB[1] - segmA[0]).x() * (segmA[1] - segmA[0]).y()) >= 0.0 &&
		((segmA[0] - segmB[0]).x() * (segmB[1] - segmB[0]).y() - (segmB[1] - segmB[0]).x() * (segmA[0] - segmB[0]).y()) *
		((segmB[1] - segmB[0]).x() * (segmA[1] - segmB[0]).y() - (segmA[1] - segmB[0]).x() * (segmB[1] - segmB[0]).y()) >= 0.0;
#endif
}

//double straddling test, must coplanar
bool psykronix::isTwoSegmentsIntersect(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
{
	// quick reject box
  //if (std::max(segmA[0].x(), segmA[1].x()) < std::min(segmB[0].x(), segmB[1].x()) ||
	//	std::min(segmA[0].x(), segmA[1].x()) > std::max(segmB[0].x(), segmB[1].x()) ||
	//	std::max(segmA[0].y(), segmA[1].y()) < std::min(segmB[0].y(), segmB[1].y()) ||
	//	std::min(segmA[0].y(), segmA[1].y()) > std::max(segmB[0].y(), segmB[1].y()) ||
	//	std::max(segmA[0].z(), segmA[1].z()) < std::min(segmB[0].z(), segmB[1].z()) ||
	//	std::min(segmA[0].z(), segmA[1].z()) > std::max(segmB[0].z(), segmB[1].z()))
	if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) ||
		std::min(segmA[0][0], segmA[1][0]) > std::max(segmB[0][0], segmB[1][0]) ||
		std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) ||
		std::min(segmA[0][1], segmA[1][1]) > std::max(segmB[0][1], segmB[1][1]) ||
		std::max(segmA[0][2], segmA[1][2]) < std::min(segmB[0][2], segmB[1][2]) ||
		std::min(segmA[0][2], segmA[1][2]) > std::max(segmB[0][2], segmB[1][2])) //index will be fast
		return false;
#ifdef USING_THRESHOLD_GEOMETRIC
	return !(
		(segmB[0] - segmA[0]).cross(segmA[1] - segmA[0]).dot((segmA[1] - segmA[0]).cross(segmB[1] - segmA[0])) < _eps || //double separate
		(segmA[0] - segmB[0]).cross(segmB[1] - segmB[0]).dot((segmB[1] - segmB[0]).cross(segmA[1] - segmB[0])) < _eps);
#else
	return 
		0.0 <= (segmB[0] - segmA[0]).cross(segmA[1] - segmA[0]).dot((segmA[1] - segmA[0]).cross(segmB[1] - segmA[0])) &&
		0.0 <= (segmA[0] - segmB[0]).cross(segmB[1] - segmB[0]).dot((segmB[1] - segmB[0]).cross(segmA[1] - segmB[0]));
#endif
}

Eigen::Vector2d psykronix::getTwoSegmentsIntersectPoint(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
{
	if (!isTwoSegmentsIntersect(segmA, segmB))
		return Vector2d(std::nan("0"), std::nan("0"));
	//get intersect point 2D
	const Eigen::Vector2d& A1 = segmA[0];
	const Eigen::Vector2d& A2 = segmA[1];
	const Eigen::Vector2d& B1 = segmB[0];
	const Eigen::Vector2d& B2 = segmB[1];
	double kA = -((B2.x() - B1.x()) * (A1.y() - B1.y()) - (B2.y() - B1.y()) * (A1.x() - B1.x())) /
		((B2.x() - B1.x()) * (A2.y() - A1.y()) - (B2.y() - B1.y()) * (A2.x() - A1.x())); //not parallel
	return A1 + kA * (A2 - A1); 
	//double kB = -((A2.x() - A1.x()) * (B1.y() - A1.y()) - (A2.y() - A1.y()) * (B1.x() - A1.x())) /
	//	((A2.x() - A1.x()) * (B2.y() - B1.y()) - (A2.y() - A1.y()) * (B2.x() - B1.x())); //not parallel
	//return B1 + kB * (B2 - B1);
}

Eigen::Vector3d psykronix::getTwoSegmentsIntersectPoint(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
{
	if (!isTwoSegmentsIntersect(segmA, segmB))
		return gVecNaN;
	Vector3d vecSeg = segmA[0] - segmA[1];
	Vector3d normal = (segmB[0] - segmB[1]).cross(vecSeg);
	if (normal.isZero(eps)) // intersect cause collinear
		return segmA[0];
	double k = (segmA[0] - segmB[0]).cross(segmA[0] - segmB[1]).norm() / normal.norm(); //k
	return segmA[0] + k * vecSeg;
}

bool psykronix::isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector2d, 2>& segment, const Eigen::AlignedBox2d& box)
{
	if (box.contains(segment[0]) || box.contains(segment[1]))
		return true;
	const Vector2d& _min = box.min();
	const Vector2d& _max = box.max();
	if (std::max(segment[0].x(), segment[1].x()) < _min.x() ||
		std::min(segment[0].x(), segment[1].x()) > _max.x() ||
		std::max(segment[0].y(), segment[1].y()) < _min.y() ||
		std::min(segment[0].y(), segment[1].y()) > _max.y())
		return false;
	std::array<Eigen::Vector2d, 3> axes = { {
		Vector2d(1,0),
		Vector2d(0,1),
		Vector2d(-(segment[1] - segment[0]).y(),(segment[1] - segment[0]).x())} }; //canbe zero
	std::array<Eigen::Vector2d, 4> boxVtx = { {
		box.min(),
		Vector2d(box.max().x(),box.min().y()),
		box.max(),
		Vector2d(box.min().x(),box.max().y())} };
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) //fast than index
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : boxVtx) //fast than list
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : segment)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB || maxB < minA) // absolute zero
			return false;
	}
	return true;
}

bool psykronix::isSegmentAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 2>& segment, const Eigen::AlignedBox3d& box)
{
	//whlie segment with tolerance, using large box
	if (box.contains(segment[0]) || box.contains(segment[1]))
		return true;
	const Vector3d& _min = box.min();
	const Vector3d& _max = box.max();
	if (std::max(segment[0].x(), segment[1].x()) < _min.x() ||
		std::min(segment[0].x(), segment[1].x()) > _max.x() ||
		std::max(segment[0].y(), segment[1].y()) < _min.y() ||
		std::min(segment[0].y(), segment[1].y()) > _max.y() ||
		std::max(segment[0].z(), segment[1].z()) < _min.z() ||
		std::min(segment[0].z(), segment[1].z()) > _max.z())
		return false;
	Vector3d vecSeg = segment[1] - segment[0];//segment direction
	std::array<Eigen::Vector3d, 7> axes = { {
		vecSeg, //maybe not work
		Vector3d(1,0,0),
		Vector3d(0,1,0),
		Vector3d(0,0,1),
		vecSeg.cross(Vector3d(1,0,0)),
		vecSeg.cross(Vector3d(0,1,0)),
		vecSeg.cross(Vector3d(0,0,1)) } };
	Vector3d vecBox = box.max() - box.min();
	std::array<Eigen::Vector3d, 8> boxVtx = { { //box.min() is origin
		Vector3d(0,0,0),
		Vector3d(vecBox[0],0,0),
		Vector3d(vecBox[0],vecBox[1],0),
		Vector3d(0,vecBox[1],0),
		Vector3d(0,0,vecBox[2]),
		Vector3d(vecBox[0],0,vecBox[2]),
		vecBox, //Vector3d(vecBox[0],vecBox[1],vecBox[2])
		Vector3d(0,vecBox[1],vecBox[2]) } };
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) //fast than index
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : boxVtx) //fast than list
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : segment) //only two point
		{
			projection = axis.dot(vertex - box.min());
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB || maxB < minA) // absolute zero
			return false;
	}
	return true;
}

//must coplanar
bool psykronix::isSegmentCrossTriangle(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) // must coplanar
{
	//judge box
	//Eigen::AlignedBox3d boxSeg(segment[0], segment[1]);
	//Eigen::AlignedBox3d boxTri(
	//	Vector3d(
	//		std::min(trigon[0].x(), trigon[1].x(), trigon[2].x()),
	//		std::min(trigon[0].y(), trigon[1].y(), trigon[2].y()),
	//		std::min(trigon[0].z(), trigon[1].z(), trigon[2].z())),
	//	Vector3d(
	//		std::max(trigon[0].x(), trigon[1].x(), trigon[2].x()),
	//		std::max(trigon[0].y(), trigon[1].y(), trigon[2].y()),
	//		std::max(trigon[0].z(), trigon[1].z(), trigon[2].z()))); //box of trigon
	//if (!boxSeg.intersects(boxTri))
	//	return false;
	//judge trigon point
	if (isPointInTriangle(segment[0], trigon) || isPointInTriangle(segment[1], trigon))
		return true;
	Vector3d vecX = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]).cross(segment[1] - segment[0]); // axisZ cross axisY
	double dot0 = vecX.dot(trigon[0] - segment[0]);
	double dot1 = vecX.dot(trigon[1] - segment[0]);
	double dot2 = vecX.dot(trigon[2] - segment[0]);
	// triangle sub point is all left // small sat 
	if ((dot0 > eps && dot1 > eps && dot2 > eps) || (dot0 < _eps && dot1 < _eps && dot2 < _eps))
		return false; //+-*x< 0,6,3,2,4.5
	// double straddling test x3
	return 
		isTwoSegmentsIntersect(segment, { trigon[0], trigon[1] }) ||
		isTwoSegmentsIntersect(segment, { trigon[1], trigon[2] }) ||
		isTwoSegmentsIntersect(segment, { trigon[2], trigon[0] });

	//bool edgea2segm = ((segment[0] - trigon[0]).cross(trigon[1] - trigon[0])).dot((segment[1] - trigon[0]).cross(trigon[1] - trigon[0])) < eps;
	//bool segm2edgea = ((trigon[1] - segment[0]).cross(vecSeg)).dot((trigon[0] - segment[0]).cross(vecSeg)) < eps;
	//if (edgea2segm && segm2edgea)
	//	return true;
	//bool edgeb2segm = ((segment[0] - trigon[1]).cross(trigon[2] - trigon[1])).dot((segment[1] - trigon[1]).cross(trigon[2] - trigon[1])) < eps;
	//bool segm2edgeb = ((trigon[2] - segment[0]).cross(vecSeg)).dot((trigon[1] - segment[0]).cross(vecSeg)) < eps;
	//if (edgeb2segm && segm2edgeb)
	//	return true;
	//bool edgec2segm = ((segment[0] - trigon[2]).cross(trigon[0] - trigon[2])).dot((segment[1] - trigon[2]).cross(trigon[0] - trigon[2])) < eps;
	//bool segm2edgec = ((trigon[0] - segment[0]).cross(vecSeg)).dot((trigon[2] - segment[0]).cross(vecSeg)) < eps;
	//if (edgec2segm && segm2edgec)
	//	return true;
	//return false;
}

// segment must through trigon inner plane
bool psykronix::isSegmentCrossTriangleSurface(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) //start point outof plane
{
	// compare angle of normal-vector
	Vector3d vecSeg = segment[1] - segment[0];
	double dotA = (trigon[0] - segment[0]).cross(trigon[1] - segment[0]).dot(vecSeg);
	double dotB = (trigon[1] - segment[0]).cross(trigon[2] - segment[0]).dot(vecSeg);
	double dotC = (trigon[2] - segment[0]).cross(trigon[0] - segment[0]).dot(vecSeg);
	return (dotA < eps && dotB < eps && dotC < eps) || (dotA > _eps && dotB > _eps && dotC > _eps);
}

// Method: judge intersect point in triangle
bool psykronix::isTwoTrianglesIntersectPIT(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{
	//#ifdef STATISTIC_DATA_COUNT
	//	isTwoTrianglesInter++;
	//#endif
		// pre box check
		//if (!isTwoTrianglesBoundingBoxIntersect(triL, triR))
		//	return false;
		// right edge through left plane
	Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[1]);
	bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point on plane(dot==0)
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;
	// left edge through right plane
	Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[1]);
	bool acrossL2R_A = (veczR.dot(triL[0] - triR[0])) * (veczR.dot(triL[1] - triR[0])) < eps;
	bool acrossL2R_B = (veczR.dot(triL[1] - triR[0])) * (veczR.dot(triL[2] - triR[0])) < eps;
	bool acrossL2R_C = (veczR.dot(triL[2] - triR[0])) * (veczR.dot(triL[0] - triR[0])) < eps;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;
	// forward
	double dotPro;// = 0;
	if (acrossR2L_A) // R0 
	{
		//edgeA={triR[0], triR[1]}
		dotPro = veczL.dot(triR[0] - triR[1]); //dotA
		if (fabs(dotPro) < eps) // perpendi to veczL (veczL dot edgeA is zero)
		{
			if (isSegmentCrossTriangle({ triR[0], triR[1] }, triL)) //segment on plane
				return true;
		}
		else
		{
			Vector3d point = triR[0] + (veczL.dot(triR[0] - triL[0]) / dotPro) * (triR[1] - triR[0]);
			if (isPointInTriangle(point, triL))
				return true;
		}
	}
	if (acrossR2L_B) // R1
	{
		dotPro = (veczL.dot(triR[1] - triR[2]));
		if (fabs(dotPro) < eps) // perpendi to veczL
		{
			if (isSegmentCrossTriangle({ triR[1], triR[2] }, triL)) //segment on plane
				return true;
		}
		else
		{
			Vector3d point = triR[1] + (veczL.dot(triR[1] - triL[0]) / dotPro) * (triR[2] - triR[1]);
			if (isPointInTriangle(point, triL))
				return true;
		}
	}
	if (acrossR2L_C) // R2
	{
		dotPro = (veczL.dot(triR[2] - triR[0]));
		if (fabs(dotPro) < eps) // perpendi to veczL
		{
			if (isSegmentCrossTriangle({ triR[2], triR[0] }, triL)) //segment on plane
				return true;
		}
		else
		{
			Vector3d point = triR[2] + (veczL.dot(triR[2] - triL[0]) / dotPro) * (triR[0] - triR[2]);
			if (isPointInTriangle(point, triL))
				return true;
		}
	}
	// reversal
	if (acrossL2R_A) // L0
	{
		dotPro = veczR.dot(triL[0] - triL[1]);
		if (fabs(dotPro) < eps) // perpendi to veczR
		{
			if (isSegmentCrossTriangle({ triL[0], triL[1] }, triR)) //segment on plane
				return true;
		}
		else
		{
			Vector3d point = triL[0] + (veczR.dot(triL[0] - triR[0]) / dotPro) * (triL[1] - triL[0]);
			if (isPointInTriangle(point, triR))
				return true;
		}
	}
	if (acrossL2R_B) // L1
	{
		dotPro = veczR.dot(triL[1] - triL[2]);
		if (fabs(dotPro) < eps) // perpendi to veczR
		{
			if (isSegmentCrossTriangle({ triL[1], triL[2] }, triR)) //segment on plane
				return true;
		}
		else
		{
			Vector3d point = triL[1] + (veczR.dot(triL[1] - triR[0]) / dotPro) * (triL[2] - triL[1]);
			if (isPointInTriangle(point, triR))
				return true;
		}
	}
	if (acrossL2R_C) // L2
	{
		dotPro = veczR.dot(triL[2] - triL[0]);
		if (fabs(dotPro) < eps) // perpendi to veczR
		{
			if (isSegmentCrossTriangle({ triL[2], triL[0] }, triR)) //segment on plane
				return true;
		}
		else
		{
			Vector3d point = triL[2] + (veczR.dot(triL[2] - triR[0]) / dotPro) * (triL[0] - triL[2]);
			if (isPointInTriangle(point, triR))
				return true;
		}
	}
	return false;
}

// Method: edge in tetra-hedron
bool psykronix::isTwoTrianglesIntersectEIT(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{
	Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[1]); // triL close face triangle
	bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point-on-plane
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;
	Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[1]); // triR close face triangle
	bool acrossL2R_A = (veczR.dot(triL[0] - triR[0])) * (veczR.dot(triL[1] - triR[0])) < eps; //include point-on-plane
	bool acrossL2R_B = (veczR.dot(triL[1] - triR[0])) * (veczR.dot(triL[2] - triR[0])) < eps;
	bool acrossL2R_C = (veczR.dot(triL[2] - triL[0])) * (veczR.dot(triL[0] - triL[0])) < eps;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;
	// using face to-left test
	bool pointOnfaceS /*= false*/;
	bool pointOnfaceE /*= false*/;
	if (acrossR2L_A) // first filter
	{
		pointOnfaceS = fabs(veczL.dot(triR[0] - triL[0])) < eps; //start point
		pointOnfaceE = fabs(veczL.dot(triR[1] - triL[0])) < eps; //end point
		if (!pointOnfaceS)
		{
			if (isSegmentCrossTriangleSurface({ triR[0], triR[1] }, triL))
				return true;
		}
		else if (!pointOnfaceE)
		{
			if (isSegmentCrossTriangleSurface({ triR[1], triR[0] }, triL))
				return true;
		}
		else if (pointOnfaceS && pointOnfaceE)
		{
			if (isSegmentCrossTriangle({ triR[0], triR[1] }, triL)) //segment on plane
				return true;
		}
	}
	if (acrossR2L_B) // first filter
	{
		pointOnfaceS = fabs(veczL.dot(triR[1] - triL[0])) < eps; //start point
		pointOnfaceE = fabs(veczL.dot(triR[2] - triL[0])) < eps; //end point
		if (!pointOnfaceS)
		{
			if (isSegmentCrossTriangleSurface({ triR[1], triR[2] }, triL))
				return true;
		}
		else if (!pointOnfaceE)
		{
			if (isSegmentCrossTriangleSurface({ triR[2], triR[1] }, triL))
				return true;
		}
		else if (pointOnfaceS && pointOnfaceE)
		{
			if (isSegmentCrossTriangle({ triR[1], triR[2] }, triL)) //segment on plane
				return true;
		}
	}
	if (acrossR2L_C) // first filter
	{
		pointOnfaceS = fabs(veczL.dot(triR[2] - triL[0])) < eps; //start point
		pointOnfaceE = fabs(veczL.dot(triR[0] - triL[0])) < eps; //end point
		if (!pointOnfaceS)
		{
			if (isSegmentCrossTriangleSurface({ triR[2], triR[1] }, triL))
				return true;
		}
		else if (!pointOnfaceE)
		{
			if (isSegmentCrossTriangleSurface({ triR[0], triR[0] }, triL))
				return true;
		}
		else if (pointOnfaceS && pointOnfaceE)
		{
			if (isSegmentCrossTriangle({ triR[2], triR[0] }, triL)) //segment on plane
				return true;
		}
	}
	// exchange two triangles
	if (acrossL2R_A) // first filter
	{
		pointOnfaceS = fabs(veczR.dot(triL[0] - triR[0])) < eps; //start point
		pointOnfaceE = fabs(veczR.dot(triL[1] - triR[0])) < eps; //end point
		if (!pointOnfaceS)
		{
			if (isSegmentCrossTriangleSurface({ triL[0], triL[1] }, triR))
				return true;
		}
		else if (!pointOnfaceE)
		{
			if (isSegmentCrossTriangleSurface({ triL[1], triL[0] }, triR))
				return true;
		}
		else if (pointOnfaceS && pointOnfaceE)
		{
			if (isSegmentCrossTriangle({ triL[0], triL[1] }, triR)) //segment on plane
				return true;
		}
	}
	if (acrossL2R_B) // first filter
	{
		pointOnfaceS = fabs(veczR.dot(triL[1] - triR[0])) < eps; //start point
		pointOnfaceE = fabs(veczR.dot(triL[2] - triR[0])) < eps; //end point
		if (!pointOnfaceS)
		{
			if (isSegmentCrossTriangleSurface({ triL[1], triL[2] }, triR))
				return true;
		}
		else if (!pointOnfaceE)
		{
			if (isSegmentCrossTriangleSurface({ triL[2], triL[1] }, triR))
				return true;
		}
		else if (pointOnfaceS && pointOnfaceE)
		{
			if (isSegmentCrossTriangle({ triL[1], triL[2] }, triR)) //segment on plane
				return true;
		}
	}
	if (acrossL2R_C) // first filter
	{
		pointOnfaceS = fabs(veczR.dot(triL[2] - triR[0])) < eps; //start point
		pointOnfaceE = fabs(veczR.dot(triL[0] - triR[0])) < eps; //end point
		if (!pointOnfaceS)
		{
			if (isSegmentCrossTriangleSurface({ triL[2], triL[1] }, triR))
				return true;
		}
		else if (!pointOnfaceE)
		{
			if (isSegmentCrossTriangleSurface({ triL[0], triL[0] }, triR))
				return true;
		}
		else if (pointOnfaceS && pointOnfaceE)
		{
			if (isSegmentCrossTriangle({ triL[2], triL[0] }, triR)) //segment on plane
				return true;
		}
	}
	return false;
}

//must intersect after SAT, using threshold
RelationOfTwoTriangles psykronix::getRelationOfTwoTrianglesSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
#ifdef STATISTIC_DATA_TESTFOR
	count_gRTT++;
#endif
	std::array<Eigen::Vector3d, 3> edgesA = { { 
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] } };
	std::array<Eigen::Vector3d, 3> edgesB = { { 
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] } };
	if (edgesA[0].cross(edgesA[1]).cross(edgesB[0].cross(edgesB[1])).isZero(eps))// && fabs(normalA.dot(triB[1] - triA[1])) < eps) //using approx
	{
		//Eigen::Vector3d normalA = edgesA[0].cross(edgesA[1]);
		//Eigen::Vector3d normalB = edgesB[0].cross(edgesB[1]);
		//std::array<Eigen::Vector3d, 6> axes = { {
		//	normalA.cross(edgesA[0]),
		//	normalA.cross(edgesA[1]),
		//	normalA.cross(edgesA[2]),
		//	normalB.cross(edgesB[0]),
		//	normalB.cross(edgesB[1]),
		//	normalB.cross(edgesB[2]) } };
#ifdef STATISTIC_DATA_COUNT
		count_trigon_coplanar++;
#endif  
		return RelationOfTwoTriangles::COPLANAR;
	}
	// using SAT, next not coplanar
	std::array<Eigen::Vector3d, 9> axes = { {
		edgesA[0].cross(edgesB[0]),
		edgesA[0].cross(edgesB[1]),
		edgesA[0].cross(edgesB[2]),
		edgesA[1].cross(edgesB[0]),
		edgesA[1].cross(edgesB[1]),
		edgesA[1].cross(edgesB[2]),
		edgesA[2].cross(edgesB[0]),
		edgesA[2].cross(edgesB[1]),
		edgesA[2].cross(edgesB[2]) } };
	for (auto& axis : axes) // revise zero vector
	{
		if (axis.isZero())
			axis = Vector3d(1, 0, 0);
	}
	double dmin = DBL_MAX, minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) //fast than index
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : triA) //fast than list
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		dmin = std::min(std::min(maxA - minB, maxB - minA), dmin);
	}
	if (fabs(dmin) < eps)
		return RelationOfTwoTriangles::CONTACT;
	return RelationOfTwoTriangles::INTRUSIVE;
}

// decompose box to twelve triangles
//bool psykronix::isTriangleAndBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box)
//{
//#ifdef STATISTIC_DATA_COUNT
//	count_isTrisAndBoxInter++;
//	if ((box.sizes().array() < 0.0).any()) // input box illegal
//		count_err_inputbox++; // nothing error
//#endif
//	const Vector3d& p0 = trigon[0];
//	const Vector3d& p1 = trigon[1];
//	const Vector3d& p2 = trigon[2];
//	if (box.contains(p0) || box.contains(p1) || box.contains(p2))
//		return true;
//	const Vector3d& min = box.min();
//	const Vector3d& max = box.max();
//	if ((p0.x() < min.x() && p1.x() < min.x() && p2.x() < min.x()) ||
//		(p0.x() > max.x() && p1.x() > max.x() && p2.x() > max.x()) ||
//		(p0.y() < min.y() && p1.y() < min.y() && p2.y() < min.y()) ||
//		(p0.y() > max.y() && p1.y() > max.y() && p2.y() > max.y()) ||
//		(p0.z() < min.z() && p1.z() < min.z() && p2.z() < min.z()) ||
//		(p0.z() > max.z() && p1.z() > max.z() && p2.z() > max.z()))
//		return false;
//	Vector3d vertex = max - min;
//	std::array<std::array<Vector3d, 3>, 12> triTwelve = { {
//	{ Vector3d(0, 0, 0), Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), vertex.y(), 0) },
//	{ Vector3d(0, 0, 0), Vector3d(0, vertex.y(), 0), Vector3d(vertex.x(), vertex.y(), 0) },
//	{ Vector3d(0, 0, vertex.z()), Vector3d(vertex.x(), 0, vertex.z()), vertex },
//	{ Vector3d(0, 0, vertex.z()), Vector3d(0, vertex.y(), vertex.z()), vertex },
//	{ Vector3d(0, 0, 0), Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), 0, vertex.z()) },
//	{ Vector3d(0, 0, 0), Vector3d(0, 0, vertex.z()), Vector3d(vertex.x(), 0, vertex.z()) },
//	{ Vector3d(0, vertex.y(), 0), Vector3d(vertex.x(), vertex.y(), 0), vertex },
//	{ Vector3d(0, vertex.y(), 0), Vector3d(0, vertex.y(), vertex.z()), vertex },
//	{ Vector3d(0, 0, 0), Vector3d(0, vertex.y(), 0), Vector3d(0, vertex.y(), vertex.z()) },
//	{ Vector3d(0, 0, 0), Vector3d(0, 0, vertex.z()), Vector3d(0, vertex.y(), vertex.z()) },
//	{ Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), vertex.y(), 0), vertex },
//	{ Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), 0, vertex.z()), vertex } } };
//	for (const auto& iter : triTwelve)
//	{
//		if (isTwoTrianglesIntersectSAT({ p0 - min, p1 - min, p2 - min }, iter))
//			return true;
//	}
//	return false;
//}

std::tuple<Vector3d, double> psykronix::getTriangleBoundingCircle(const std::array<Vector3d, 3>& trigon) //return center and radius
{
	auto _inverse3x3 = [](const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const Vector3d& B)->Vector3d //set as row
	{
		//fast than eigon
		double cofactor00 = v1[1] * v2[2] - v1[2] * v2[1];
		double cofactor01 = -(v1[0] * v2[2] - v1[2] * v2[0]);
		double cofactor02 = v1[0] * v2[1] - v1[1] * v2[0];
		double cofactor10 = -(v0[1] * v2[2] - v0[2] * v2[1]);
		double cofactor11 = v0[0] * v2[2] - v0[2] * v2[0];
		double cofactor12 = -(v0[0] * v2[1] - v0[1] * v2[0]);
		double cofactor20 = v0[1] * v1[2] - v0[2] * v1[1];
		double cofactor21 = -(v0[0] * v1[2] - v0[2] * v1[0]);
		double cofactor22 = v0[0] * v1[1] - v0[1] * v1[0];
		double det = v0[0] * cofactor00 + v1[0] * cofactor01 + v2[0] * cofactor02;
		det = 1.0 / det;
		Matrix3d inv;
		inv << 
			det * cofactor00, det * cofactor10, det * cofactor20,
			det * cofactor01, det * cofactor11, det * cofactor21,
			det * cofactor02, det * cofactor12, det * cofactor22;
		return inv * B;
	};
	Vector3d vecA = trigon[1] - trigon[0];
	Vector3d vecB = trigon[2] - trigon[1];
	Vector3d vecC = trigon[0] - trigon[2];
	Vector3d normal = vecA.cross(vecB);
	//if (normal.squaredNorm() > eps) //trigon legal
	// illegal and obtuse angle
	double a2 = vecA.squaredNorm();
	double b2 = vecB.squaredNorm();
	double c2 = vecC.squaredNorm();
	if (a2 >= b2 + c2) //two points coin >=
		return { 0.5 * (trigon[0] + trigon[1]), 0.5 * sqrt(a2) };
	else if (b2 >= a2 + c2)
		return { 0.5 * (trigon[0] + trigon[2]), 0.5 * sqrt(b2) };
	else if (c2 >= a2 + b2)
		return { 0.5 * (trigon[1] + trigon[2]), 0.5 * sqrt(c2) };
	//mat = set_matrix_by_row_vectors(vecA, vecB, normal)
	Eigen::Matrix3d mat;
	mat.row(0) = vecA;
	mat.row(1) = vecB;
	mat.row(2) = normal;
	const Vector3d& p0 = trigon[0];
	const Vector3d& p1 = trigon[1];
	const Vector3d& p2 = trigon[2];
	Vector3d p = mat.inverse() * Vector3d(
		0.5 * (p1.x() * p1.x() - p0.x() * p0.x() + p1.y() * p1.y() - p0.y() * p0.y() + p1.z() * p1.z() - p0.z() * p0.z()),
		0.5 * (p2.x() * p2.x() - p0.x() * p0.x() + p2.y() * p2.y() - p0.y() * p0.y() + p2.z() * p2.z() - p0.z() * p0.z()),
		normal.dot(p0));
	return { p, (p - p0).norm() };
}

bool isSegmentAndTriangleIntersctSAT(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon)
{
	//pre-box
	if (std::max(segment[0][0], segment[1][0]) < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
		std::max(segment[0][0], segment[1][0]) > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
		std::max(segment[0][1], segment[1][1]) < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
		std::max(segment[0][1], segment[1][1]) > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
		std::max(segment[0][2], segment[1][2]) < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
		std::max(segment[0][2], segment[1][2]) > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]))
		return false;
	Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]);
	Vector3d vecSeg = segment[1] - segment[0];
#ifdef USING_METHOD_SAT
	double minA, maxA, projection, dotB0, dotB1;
	if (fabs(normal.dot(segment[0] - trigon[0])) < eps && fabs(normal.dot(segment[1] - trigon[0])) < eps)// 2D-SAT, coplanar
	{
		std::array<Eigen::Vector3d, 4> axes = { {
			normal.cross(vecSeg),
			normal.cross(trigon[1] - trigon[0]),
			normal.cross(trigon[2] - trigon[1]),
			normal.cross(trigon[0] - trigon[2]) } };
		for (auto& axis : axes) // revise zero vector
		{
			if (axis.isZero())
				axis = Vector3d(1, 0, 0);
		}
		for (const auto& axis : axes)
		{
			minA = DBL_MAX;
			maxA = -DBL_MAX;
			for (const auto& vertex : trigon)
			{
				projection = vertex.dot(axis);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			dotB0 = segment[0].dot(axis);
			dotB1 = segment[1].dot(axis);
			if (maxA < std::min(dotB0, dotB1) || std::max(dotB0, dotB1) < minA) // absolute zero
				return false;
		}
		return true;
	}
	else // 3D-SAT, not coplanar
	{
		std::array<Eigen::Vector3d, 4> axes = { {
			normal,
			vecSeg.cross(trigon[1] - trigon[0]),
			vecSeg.cross(trigon[2] - trigon[1]),
			vecSeg.cross(trigon[0] - trigon[2])} };
		for (auto& axis : axes) // revise zero vector
		{
			if (axis.isZero())
				axis = Vector3d(1, 0, 0);
		}
		for (const auto& axis : axes)
		{
			minA = DBL_MAX;
			maxA = -DBL_MAX;
			for (const auto& vertex : trigon)
			{
				projection = vertex.dot(axis);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			dotB0 = segment[0].dot(axis);
			dotB1 = segment[1].dot(axis);
			if (maxA < std::min(dotB0, dotB1) || std::max(dotB0, dotB1) < minA) // absolute zero
				return false;
		}
		return true;
	}
#else
	double onPlaneS = (normal.dot(segment[0] - trigon[0]));
	double onPlaneE = (normal.dot(segment[1] - trigon[0]));
	if (onPlaneS * onPlaneE > 0.0)//include point on plane(dot==0)
		return false;
	//point on plane
	double dotA, dotB, dotC;
	if (onPlaneS != 0.0)
	{
		dotA = (trigon[0] - segment[0]).cross(trigon[1] - segment[0]).dot(vecSeg);
		dotB = (trigon[1] - segment[0]).cross(trigon[2] - segment[0]).dot(vecSeg);
		dotC = (trigon[2] - segment[0]).cross(trigon[0] - segment[0]).dot(vecSeg);
		return (dotA <= 0.0 && dotB <= 0.0 && dotC <= 0.0) || (dotA >= 0.0 && dotB >= 0.0 && dotC >= 0.0);
	}
	if (onPlaneE != 0.0)
	{
		dotA = (trigon[0] - segment[1]).cross(trigon[1] - segment[1]).dot(vecSeg);
		dotB = (trigon[1] - segment[1]).cross(trigon[2] - segment[1]).dot(vecSeg);
		dotC = (trigon[2] - segment[1]).cross(trigon[0] - segment[1]).dot(vecSeg);
		return (dotA <= 0.0 && dotB <= 0.0 && dotC <= 0.0) || (dotA >= 0.0 && dotB >= 0.0 && dotC >= 0.0);
	}
	return isSegmentCrossTriangle(segment, trigon);
#endif
}

// whlie ray across same edge of two triangles
bool isPointRayAcrossTriangleSAT(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon)
{
	//first pre-process, bounding box
	double rayZ = std::max(std::max(trigon[0].z(), trigon[1].z()), trigon[2].z());
	if (point.z() > rayZ || // ray direction to axisZ
		point.x() > std::max(std::max(trigon[0].x(), trigon[1].x()), trigon[2].x()) ||
		point.x() < std::min(std::min(trigon[0].x(), trigon[1].x()), trigon[2].x()) ||
		point.y() > std::max(std::max(trigon[0].y(), trigon[1].y()), trigon[2].y()) ||
		point.y() < std::min(std::min(trigon[0].y(), trigon[1].y()), trigon[2].y()))
		return false; // range box judge
	//using SAT
	Vector3d rayPt(point.x(), point.y(), rayZ), axisZ(0, 0, 1);
	std::array<Eigen::Vector3d, 3> m_edges = { trigon[1] - trigon[0],
										   trigon[2] - trigon[1],
										   trigon[0] - trigon[2] };
	Vector3d normal = m_edges[0].cross(m_edges[1]);
	std::array<Eigen::Vector3d, 7> axes = { {
			//axisZ, // using pre-box
			axisZ.cross(m_edges[0]),
			axisZ.cross(m_edges[1]),
			axisZ.cross(m_edges[2]),
			normal,
			normal.cross(m_edges[0]),
			normal.cross(m_edges[1]),
			normal.cross(m_edges[2]) } };//add
	for (auto& axis : axes) // revise zero vector
	{
		if (axis.isZero())
			axis = Vector3d(1, 0, 0);
	}
	double minTri, maxTri, projection, dotP, dotR;
	for (const auto& axis : axes) //fast than index
	{
		minTri = DBL_MAX;
		maxTri = -DBL_MAX;
		for (const auto& vertex : trigon) //fast than list
		{
			projection = axis.dot(vertex);
			minTri = std::min(minTri, projection);
			maxTri = std::max(maxTri, projection);
		}
		dotP = axis.dot(point);
		dotR = axis.dot(rayPt);
#ifdef USING_THRESHOLD_CUSTOMIZE
		if (minTri + eps < std::min(dotP, dotR) || std::max(dotP, dotR) + eps < minTri)
#else
		if (maxTri < std::min(dotP, dotR) || std::max(dotP, dotR) < minTri) // absolute zero
#endif
			return false;
	}
	return true;
	// another method, judge 2D point in triangle, point under plane
}

//Moller Trumbore Algorithm
int isRayLineCrossTriangleMTA(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Triangle& trigon)
{
	//(Cost = 0 div, 27 mul, 17 add)
	Vector3d E1 = trigon[1] - trigon[0];
	Vector3d E2 = trigon[2] - trigon[0];
	Vector3d S = origin - trigon[0];
	Vector3d S1 = direction.cross(E2);
	Vector3d S2 = S.cross(E1);
	double k = S1.dot(E1);
	if (fabs(k) < eps)// ray parallel with trigon plane (include through, both illegal)
	{
		// through edge, should change ray
		Vector3d normal = E1.cross(E2);
		if (fabs(normal.dot(S)) < eps && (
			(0 <= (trigon[0] - origin).cross(direction).dot((direction).cross(trigon[1] - origin)) && 0 <= (trigon[0] - origin).cross(trigon[1] - trigon[0]).dot(normal) / direction.cross(trigon[1] - trigon[0]).dot(normal)) ||
			(0 <= (trigon[1] - origin).cross(direction).dot((direction).cross(trigon[2] - origin)) && 0 <= (trigon[1] - origin).cross(trigon[2] - trigon[1]).dot(normal) / direction.cross(trigon[2] - trigon[1]).dot(normal)) ||
			(0 <= (trigon[2] - origin).cross(direction).dot((direction).cross(trigon[0] - origin)) && 0 <= (trigon[2] - origin).cross(trigon[0] - trigon[2]).dot(normal) / direction.cross(trigon[0] - trigon[2]).dot(normal))))
			return -4;
		return -3;
	}
	k = 1.0 / k;
	double t = k * S2.dot(E2);
	if (t < 0) //check 0 <= t < oo, exclude nan
		return -2;// negative direction
	double b1 = k * S1.dot(S); //b1, b2 are barycentric coordinates
	double b2 = k * S2.dot(direction);
	double b3 = 1.0 - b1 - b2;
	if (b1 < 0 || b2 < 0 || b3 < 0)
		return -1;//false, out of triangle
	if (t == 0 || b1 == 0 || b2 == 0 || b3 == 0)
		return 0;// intersect point on plane | intersect point on edge
	return 1;
}

//// for soft clash
//Vector3d getIntersectOfPointAndLine(const Vector3d& point, const std::array<Vector3d, 2>& segm)
//{
//	Eigen::Vector3d direction = (segm[1] - segm[0]);// not zero
//	double projection = direction.dot(point);
//	//the projection must on segment
//	if (direction.dot(segm[1]) < projection || projection < direction.dot(segm[0]))
//		return DBL_MAX;
//	double k = direction.dot(point - segm[0]) / direction.dot(direction);
//	return (segm[0] - point + k * direction);//.squaredNorm();
//}
//Vector3d getIntersectOfTwoSegments(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
//{
//	Vector3d vectA = segmA[1] - segmA[0];
//	Vector3d vectB = segmB[1] - segmB[0];
//	double delta1 = (segmB[0] - segmA[0]).dot(vectA);
//	double delta2 = (segmB[0] - segmA[0]).dot(vectB);
//	// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
//	double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
//	if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
//		return DBL_MAX;
//	double kA = 1 / deno * (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2);
//	double kB = 1 / deno * (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2);
//	//	Vector3d pointA = segmA[0] + kA * vectA;
//	//	Vector3d pointB = segmB[0] + kB * vectB;
//	//whether two intersect-point inside segments
//	if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
//		return (segmA[0] + kA * vectA - segmB[0] - kB * vectB);//.squaredNorm();
//	return DBL_MAX; // nearest point outof segments
//}
//Vector3d getIntersectOfPointAndPlane(const Vector3d& point, const std::array<Vector3d, 3>& plane)
//{
//	Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
//	//if (normal.isZero()) // error triangle plane
//	//	return DBL_MAX;
//	double k = (plane[0] - point).dot(normal) / normal.dot(normal);
//	Vector3d local = point + k * normal;
//	if (!isPointInTriangle(local, plane))
//		return DBL_MAX;
//	return local;//(k * normal).squaredNorm();
//}
//Vector3d getIntersectPointOfPointAndLineMIN(const Vector3d& point, const std::array<Vector3d, 2>& segm)
//{
//	return {};
//}
//Vector3d getIntersectPointOfPointAndPlaneMIN(const Vector3d& point, const std::array<Vector3d, 3>& plane)
//{
//	return {};
//}
//Segment getIntersectPointsOfTwoLineMIN(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
//{
//	return {};
//}
//Vector3d getIntersectPointOfLineAndPlaneMIN(const std::array<Vector3d, 2>& segm, const std::array<Vector3d, 3>& plane)
//{
//	return {};
//}
//Segment getIntersectPointsOfTwoPlaneMIN(const std::array<Vector3d, 3>& planeA, const std::array<Vector3d, 3>& planeB)
//{
//	return {};
//}

bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance /*= 0.0*/)
{
#ifdef STATISTIC_DATA_COUNT
	count_isTrisBoundBoxInter++;
#endif
	//get min and max of two trigons
	return !(
		std::max(std::max(triB[0][0], triB[1][0]), triB[2][0]) < std::min(std::min(triA[0][0], triA[1][0]), triA[2][0]) - tolerance ||
		std::max(std::max(triA[0][0], triA[1][0]), triA[2][0]) + tolerance < std::min(std::min(triB[0][0], triB[1][0]), triB[2][0]) ||
		std::max(std::max(triB[0][1], triB[1][1]), triB[2][1]) < std::min(std::min(triA[0][1], triA[1][1]), triA[2][1]) - tolerance ||
		std::max(std::max(triA[0][1], triA[1][1]), triA[2][1]) + tolerance < std::min(std::min(triB[0][1], triB[1][1]), triB[2][1]) ||
		std::max(std::max(triB[0][2], triB[1][2]), triB[2][2]) < std::min(std::min(triA[0][2], triA[1][2]), triA[2][2]) - tolerance ||
		std::max(std::max(triA[0][2], triA[1][2]), triA[2][2]) + tolerance < std::min(std::min(triB[0][2], triB[1][2]), triB[2][2]));
	//if (std::max(std::max(triB[0].x(), triB[1].x()), triB[2].x()) < std::min(std::min(triA[0].x(), triA[1].x()), triA[2].x()) - tolerance)
	//	return false;
	//if (std::max(std::max(triA[0].x(), triA[1].x()), triA[2].x()) + tolerance < std::min(std::min(triB[0].x(), triB[1].x()), triB[2].x()))
	//	return false;
	//if (std::max(std::max(triB[0].y(), triB[1].y()), triB[2].y()) < std::min(std::min(triA[0].y(), triA[1].y()), triA[2].y()) - tolerance)
	//	return false;
	//if (std::max(std::max(triA[0].y(), triA[1].y()), triA[2].y()) + tolerance < std::min(std::min(triB[0].y(), triB[1].y()), triB[2].y()))
	//	return false;
	//if (std::max(std::max(triB[0].z(), triB[1].z()), triB[2].z()) < std::min(std::min(triA[0].z(), triA[1].z()), triA[2].z()) - tolerance)
	//	return false;
	//if (std::max(std::max(triA[0].z(), triA[1].z()), triA[2].z()) + tolerance < std::min(std::min(triB[0].z(), triB[1].z()), triB[2].z()))
	//	return false;
	//return true;
	//double xminA = std::min(std::min(triA[0].x(), triA[1].x()), triA[2].x()) - tolerance;
	//double xmaxA = std::max(std::max(triA[0].x(), triA[1].x()), triA[2].x()) + tolerance;
	//double yminA = std::min(std::min(triA[0].y(), triA[1].y()), triA[2].y()) - tolerance;
	//double ymaxA = std::max(std::max(triA[0].y(), triA[1].y()), triA[2].y()) + tolerance;
	//double zminA = std::min(std::min(triA[0].z(), triA[1].z()), triA[2].z()) - tolerance;
	//double zmaxA = std::max(std::max(triA[0].z(), triA[1].z()), triA[2].z()) + tolerance;
	//double xminB = std::min(std::min(triB[0].x(), triB[1].x()), triB[2].x());
	//double xmaxB = std::max(std::max(triB[0].x(), triB[1].x()), triB[2].x());
	//double yminB = std::min(std::min(triB[0].y(), triB[1].y()), triB[2].y());
	//double ymaxB = std::max(std::max(triB[0].y(), triB[1].y()), triB[2].y());
	//double zminB = std::min(std::min(triB[0].z(), triB[1].z()), triB[2].z());
	//double zmaxB = std::max(std::max(triB[0].z(), triB[1].z()), triB[2].z());
	//return AlignedBox3d(Vector3d(xminA, yminA, zminA), Vector3d(xmaxA, ymaxA, zmaxA)).intersects(
	//	AlignedBox3d(Vector3d(xminB, yminB, zminB), Vector3d(xmaxB, ymaxB, zmaxB)));
	//m_min.array()<=(b.max)().array()).all() && ((b.min)().array()<=m_max.array()).all();
	//return !(xmaxB < xminA || ymaxB < yminA || zmaxB < zminA || xmaxA < xminB || ymaxA < yminB || zmaxA < zminB);
}

bool isTriangleAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box)
{
#ifdef STATISTIC_DATA_COUNT
	count_isTrisAndBoxInter++;
#endif
	//pre-judge
	const Vector3d& p0 = trigon[0];
	const Vector3d& p1 = trigon[1];
	const Vector3d& p2 = trigon[2];
	if (box.contains(p0) || box.contains(p1) || box.contains(p2))
		return true;
	const Vector3d& _min = box.min();
	const Vector3d& _max = box.max();
	if (std::max(std::max(p0.x(), p1.x()), p2.x()) < _min.x() ||
		std::min(std::min(p0.x(), p1.x()), p2.x()) > _max.x() ||
		std::max(std::max(p0.y(), p1.y()), p2.y()) < _min.y() ||
		std::min(std::min(p0.y(), p1.y()), p2.y()) > _max.y() ||
		std::max(std::max(p0.z(), p1.z()), p2.z()) < _min.z() ||
		std::min(std::min(p0.z(), p1.z()), p2.z()) > _max.z())
		return false;
	//if ((p0.x() < min.x() && p1.x() < min.x() && p2.x() < min.x()) ||
	//	(p0.x() > max.x() && p1.x() > max.x() && p2.x() > max.x()) ||
	//	(p0.y() < min.y() && p1.y() < min.y() && p2.y() < min.y()) ||
	//	(p0.y() > max.y() && p1.y() > max.y() && p2.y() > max.y()) ||
	//	(p0.z() < min.z() && p1.z() < min.z() && p2.z() < min.z()) ||
	//	(p0.z() > max.z() && p1.z() > max.z() && p2.z() > max.z()))
	//	return false;
	// Separating Axis Theorem
	std::array<Eigen::Vector3d, 3> m_edges = { 
		trigon[1] - trigon[0],
		trigon[2] - trigon[1],
		trigon[0] - trigon[2] };
	std::array<Eigen::Vector3d, 3> coords = { 
		Vector3d(1,0,0),
		Vector3d(0,1,0),
		Vector3d(0,0,1) };
	std::array<Eigen::Vector3d, 13> axes = { {
		coords[0],
		coords[1],
		coords[2],
		m_edges[0].cross(m_edges[1]), //normal
		//normal.cross(edges[]),
		coords[0].cross(m_edges[0]),
		coords[0].cross(m_edges[1]),
		coords[0].cross(m_edges[2]),
		coords[1].cross(m_edges[0]),
		coords[1].cross(m_edges[1]),
		coords[1].cross(m_edges[2]),
		coords[2].cross(m_edges[0]),
		coords[2].cross(m_edges[1]),
		coords[2].cross(m_edges[2]) } };
	//for (auto& axis : axes) // absolute can give up
	//{
	//	if (axis.isZero())
	//		axis = Vector3d(1, 0, 0);
	//}
	const Vector3d& origin = box.min();
	Vector3d vertex = box.sizes();
	std::array<Vector3d, 8> vertexes = { {
		Vector3d(0, 0, 0),
		Vector3d(vertex.x(), 0, 0),
		Vector3d(vertex.x(), vertex.y(), 0),
		Vector3d(0, vertex.y(), 0),
		Vector3d(0, 0, vertex.z()),
		Vector3d(vertex.x(), 0, vertex.z()),
		Vector3d(vertex.x(), vertex.y(), vertex.z()),
		Vector3d(0, vertex.y(), vertex.z()) } };
	// iterate
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) //fast than index
	{
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
#ifdef USING_THRESHOLD_CUSTOMIZE
		if (maxA + eps < minB || maxB + eps < minA)
#else
		if (maxA < minB || maxB < minA) // absolute zero
#endif
			return false;
	}
	return true;
}

bool isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	std::array<Eigen::Vector3d, 3> edgesA = { 
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = { 
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
#ifdef STATISTIC_DATA_COUNT
	count_isTwoTrisInter++;
	if (triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2])
		count_err_repeat_tri++;
	if (edgesA[0].cross(edgesA[1]).isZero() || edgesB[0].cross(edgesB[1]).isZero())
		count_err_degen_tri++;
#endif
	Eigen::Vector3d normalA = edgesA[0].cross(edgesA[1]);
	Eigen::Vector3d normalB = edgesB[0].cross(edgesB[1]);
	//// to avoid precision error, cause normal isnot unique, different normal lead to two result
	//double projection0 = normalA.dot(triB[0] - triA[1]);
	//double projection1 = normalA.dot(triB[1] - triA[1]);
	//double projection2 = normalA.dot(triB[2] - triA[1]);
	//if ((0.0 < projection0 && 0.0 < projection1 && 0.0 < projection2) || (0.0 > projection0 && 0.0 > projection1 && 0.0 > projection2))
	//	return false;
	//projection0 = normalB.dot(triA[0] - triB[1]);
	//projection1 = normalB.dot(triA[1] - triB[1]);
	//projection2 = normalB.dot(triA[2] - triB[1]);
	//if ((0.0 < projection0 && 0.0 < projection1 && 0.0 < projection2) || (0.0 > projection0 && 0.0 > projection1 && 0.0 > projection2))
	//	return false;
	std::array<Eigen::Vector3d, 17> axes = { { // compat zero-vector from parallel edges
		//normalA,
		//normalB,
		normalA,
		normalB,
		normalA.cross(edgesA[0]),
		normalA.cross(edgesA[1]),
		normalA.cross(edgesA[2]),
		normalB.cross(edgesB[0]),
		normalB.cross(edgesB[1]),
		normalB.cross(edgesB[2]),
		//cross edge pair
		edgesA[0].cross(edgesB[0]),
		edgesA[0].cross(edgesB[1]),
		edgesA[0].cross(edgesB[2]),
		edgesA[1].cross(edgesB[0]),
		edgesA[1].cross(edgesB[1]),
		edgesA[1].cross(edgesB[2]),
		edgesA[2].cross(edgesB[0]),
		edgesA[2].cross(edgesB[1]),
		edgesA[2].cross(edgesB[2]) } };
	// Check for overlap along each axis
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) //fast than index
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : triA) //fast than list
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
#ifdef USING_THRESHOLD_CUSTOMIZE
		if (maxA + eps < minB || maxB + eps < minA) //only no threshold can avoid edge-pair parallel, that lead to zero vector
#else
		if (maxA < minB || maxB < minA) // absolute zero
#endif
			return false;
	}
	// special handling degenerate triangle
	//Eigen::Vector3d croA = edgesA[0].cross(edgesA[1]);
	//Eigen::Vector3d croB = edgesB[0].cross(edgesB[1]);
	//return !(edgesA[0].cross(edgesA[1]).isZero() || edgesB[0].cross(edgesB[1]).isZero());
	return true;
}

// intersect and penetration judge
bool isTwoTrianglesPenetrationSAT(const std::array<Vector2d, 3>& triA, const std::array<Vector2d, 3>& triB, double tolerance /*= eps*/)
{
	// bound box been judged
	//if (std::max(std::max(triA[0][0], triA[1][0]), triA[2][0]) <= std::min(std::min(triB[0][0], triB[1][0]), triB[2][0]) ||
	//	std::max(std::max(triA[0][1], triA[1][1]), triA[2][1]) <= std::min(std::min(triB[0][1], triB[1][1]), triB[2][1]) ||
	//	std::max(std::max(triB[0][0], triB[1][0]), triB[2][0]) <= std::min(std::min(triA[0][0], triA[1][0]), triA[2][0]) ||
	//	std::max(std::max(triB[0][1], triB[1][1]), triB[2][1]) <= std::min(std::min(triA[0][1], triA[1][1]), triA[2][1]))
	//	return false;
	std::array<Eigen::Vector2d, 6> edgesAB = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2],
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	double k = 0;
	for (auto& iter : edgesAB)
	{
		k = std::max(k, std::max(iter[0], iter[1]));// iter.squaredNorm());
		iter = Vector2d(-iter[1], iter[0]); //rotz(pi/2)
	}
	tolerance = k * tolerance;
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : edgesAB)
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : triA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + tolerance || maxB < minA + tolerance)
			return false; //without normlized
	}
	return true;
}

//must intersect
bool isTwoTrianglesPenetrationSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB, double tolerance /*= eps*/)
{
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
	if (normalA.cross(normalB).isZero(eps)) //is parallel
		return false;//exclude coplanar
	std::array<Eigen::Vector3d, 9> axes = { { 
		// only cross edge pair
		edgesA[0].cross(edgesB[0]),
		edgesA[0].cross(edgesB[1]),
		edgesA[0].cross(edgesB[2]),
		edgesA[1].cross(edgesB[0]),
		edgesA[1].cross(edgesB[1]),
		edgesA[1].cross(edgesB[2]),
		edgesA[2].cross(edgesB[0]),
		edgesA[2].cross(edgesB[1]),
		edgesA[2].cross(edgesB[2]) } };
	double k = 0;
	for (const auto& iter : edgesA)
		k = std::max(k, iter.squaredNorm());
	for (const auto& iter : edgesB)
		k = std::max(k, iter.squaredNorm());
	tolerance = k * tolerance;
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) 
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : triA) 
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + tolerance || maxB < minA + tolerance)
			return false; //without normlized
	}
	return true;
}

// must separate
double getTrianglesDistanceSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
#ifdef STATISTIC_DATA_COUNT
	count_getTrisDistance++;
#endif
	//if (isTwoTrianglesIntersectSAT(triA, triB))
	//	return 0.0;
	double dmin = DBL_MAX, dtemp;
	Vector3d direction, vecSeg, vectA, vectB; // to iterate, get nearest direction
	std::array<array<Vector3d, 2>, 3> edgesA = { {
		{ triA[0], triA[1] },
		{ triA[1], triA[2] },
		{ triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { { 
		{ triB[0], triB[1] },
		{ triB[1], triB[2] },
		{ triB[2], triB[0] } } };
	auto _getDistanceOfPointAndSegmentINF = [&vecSeg](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double
	{
		vecSeg = segm[1] - segm[0];// not zero
		double projection = vecSeg.dot(point);
		//the projection must on segment
		if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
			return DBL_MAX;
		double k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
		return (segm[0] - point + k * vecSeg).squaredNorm();
	};
	auto _getDistanceOfTwoSegmentsINF = [&vectA, &vectB](const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)->double
	{
		//Vector3d vectA = segmA[1] - segmA[0];
		//Vector3d vectB = segmB[1] - segmB[0];
		double delta1 = (segmB[0] - segmA[0]).dot(vectA);
		double delta2 = (segmB[0] - segmA[0]).dot(vectB);
		// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
		double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
		if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
			return DBL_MAX;
		double kA = 1 / deno * (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2);
		double kB = 1 / deno * (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2);
		//	Vector3d pointA = segmA[0] + kA * vectA;
		//	Vector3d pointB = segmB[0] + kB * vectB;
		//whether two intersect-point inside segments
		if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
			return (segmA[0] + kA * vectA - segmB[0] - kB * vectB).squaredNorm();
		return DBL_MAX; // nearest point outof segments
	};
#define EDGE_PAIR_CREATE_AXIS_OPTMIZE
#ifdef EDGE_PAIR_CREATE_AXIS_OPTMIZE
	auto _getNearestAxisOfTwoSegments = [&/*many*/](const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB)// major capture axis
	{
		//enum RelationTwoSegment
		//{
		//	NearestMiddleMiddle, // edgeA.corss(edgeB)
		//	NearestMiddleVertex, // (vertexA-vertexB).cross(edgesB).cross(edgesB)
		//	NearestVertexVertex, // vertexA-vertexB
		//};
		vectA = segmA[1] - segmA[0];
		vectB = segmB[1] - segmB[0];
		dtemp = _getDistanceOfTwoSegmentsINF(segmA, segmB);
		if (dtemp < dmin) //!= DBL_MAX)
		{
			direction = vectA.cross(vectB);
			dmin = dtemp;
			return;
		}
		for (const auto& iterA : segmA)
		{
			dtemp = _getDistanceOfPointAndSegmentINF(iterA, segmB);
			if (dtemp == DBL_MAX)
			{
				dtemp = (iterA - segmB[1]).squaredNorm();// total 4 times compare
				if (dtemp < dmin)
				{
					direction = iterA - segmB[1];
					dmin = dtemp;
				}
			}
			else if (dtemp < dmin)
			{
				direction = (iterA - segmB[0]).cross(vectB).cross(vectB);
				dmin = dtemp;
			}
		}
		for (const auto& iterB : segmB)
		{
			dtemp = _getDistanceOfPointAndSegmentINF(iterB, segmA);
			if (dtemp == DBL_MAX)
			{
				dtemp = (iterB - segmA[1]).squaredNorm();
				if (dtemp < dmin)
				{
					direction = iterB - segmA[1];
					dmin = dtemp;
				}
			}
			else if (dtemp < dmin)
			{
				direction = (iterB - segmA[0]).cross(vectA).cross(vectA);
				dmin = dtemp;
			}
		}
	};
	for (const auto& iterA : edgesA)
	{
		for (const auto& iterB : edgesB)
		{
			_getNearestAxisOfTwoSegments(iterA, iterB); //iterate update axis
		}
	}
	// next reduce axis will cause speed slow
	std::array<Eigen::Vector3d, 3> axes = { {
		(triA[1] - triA[0]).cross(triA[2] - triA[1]).normalized(), //normalA
		(triB[1] - triB[0]).cross(triB[2] - triB[1]).normalized(), //normalB
		direction.normalized() } };
	double dmax = -DBL_MAX, minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) // check for overlap along each axis
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : triA) //fast than list
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		dmax = std::max(std::max(minB - maxA, minA - maxB), dmax);
	}
	return dmax;
#else
	auto _getDistanceOfPointAndPlaneINF = [](const Vector3d& point, const std::array<Vector3d, 3>& plane)->double
	{
		Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
		//if (normal.isZero()) // error triangle plane
		//	return DBL_MAX;
		double k = (plane[0] - point).dot(normal) / normal.dot(normal);
		Vector3d local = point + k * normal;
		if (!isPointInTriangle(local, plane))
			return DBL_MAX;
		return (k * normal).squaredNorm();
	};
	for (const auto& iterA : triA)  //vertex to vertex
	{
		for (const auto& iterB : triB)
		{
			dtemp = (iterA - iterB).squaredNorm();
			if (dtemp < dmin)
			{
				dmin = dtemp;
				direction = iterA - iterB;
			}
		}
	}
	for (const auto& iterA : edgesA) // edge to edge 
	{
		for (const auto& iterB : edgesB)
		{
			dtemp = _getDistanceOfTwoSegmentsINF(iterA, iterB);
			if (dtemp < dmin)
			{
				dmin = dtemp;
				direction = (iterA[1] - iterA[0]).cross(iterB[1] - iterB[0]);
			}
		}
	}
	for (const auto& iterV : triA)
	{
		dtemp = _getDistanceOfPointAndPlaneINF(iterV, triB);  // vertex to face
		if (dtemp < dmin)
		{
			dmin = dtemp;
			direction = (triB[1] - triB[0]).cross(triB[2] - triB[1]);
		}
		for (const auto& edge : edgesB) // vertex to edge
		{
			dtemp = _getDistanceOfPointAndSegmentINF(iterV, edge);
			if (dtemp < dmin)
			{
				dmin = dtemp;
				direction = (iterV - edge[0]).cross(edge[1] - edge[0]).cross(edge[1] - edge[0]);
			}
		}
	}
	for (const auto& iterV : triB) // vertex to edge
	{
		dtemp = _getDistanceOfPointAndPlaneINF(iterV, triA);  // vertex to face
		if (dtemp < dmin)
		{
			dmin = dtemp;
			direction = (triA[1] - triA[0]).cross(triA[2] - triA[1]);
		}
		for (const auto& edge : edgesA)
		{
			dtemp = _getDistanceOfPointAndSegmentINF(iterV, edge);
			if (dtemp < dmin)
			{
				dmin = dtemp;
				direction = (iterV - edge[0]).cross(edge[1] - edge[0]).cross(edge[1] - edge[0]);
			}
		}
	}
	direction.normalize();
	double minA = DBL_MAX, minB = DBL_MAX, maxA = -DBL_MAX, maxB = -DBL_MAX, projection;
	for (const auto& vertex : triA) //fast than list
	{
		projection = direction.dot(vertex);
		minA = std::min(minA, projection);
		maxA = std::max(maxA, projection);
	}
	for (const auto& vertex : triB)
	{
		projection = direction.dot(vertex);
		minB = std::min(minB, projection);
		maxB = std::max(maxB, projection);
	}
	return  std::max(minB - maxA, minA - maxB);
#endif //REDUCED_AXIS_OPTMIZE
}

//must separate
std::array<Eigen::Vector3d, 2> getTwoTrianglesNearestPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	std::array<Vector3d, 2> res;
#ifdef STATISTIC_DATA_COUNT
	if (isTwoTrianglesIntersectSAT(triA, triB))
		count_err_tris_inter++;
	return res;
#endif
	double dmin = DBL_MAX, dtemp;
	Eigen::Vector3d local, local2;
	auto _getDistanceOfPointAndPlaneINF = [&local](const Vector3d& point, const std::array<Vector3d, 3>& plane)->double
	{
		Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
		//if (normal.isZero()) // error triangle plane
		//	return DBL_MAX;
		double k = (plane[0] - point).dot(normal) / normal.dot(normal);
		local = point + k * normal; // reference closure
		if (!isPointInTriangle(local, plane))
			return DBL_MAX;
		return (k * normal).squaredNorm(); //to be fast
	};
	auto _getDistanceOfTwoSegmentsINF = [&local, &local2](const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)->double
	{
		Vector3d vectA = segmA[1] - segmA[0];
		Vector3d vectB = segmB[1] - segmB[0];
		double deltaA = (segmB[0] - segmA[0]).dot(vectA);
		double deltaB = (segmB[0] - segmA[0]).dot(vectB);
		double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
		if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
			return DBL_MAX;
		double kA = 1 / deno * (-vectB.dot(vectB) * deltaA + vectB.dot(vectA) * deltaB);
		double kB = 1 / deno * (-vectA.dot(vectB) * deltaA + vectA.dot(vectA) * deltaB);
		if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
		{
			local = segmA[0] + kA * vectA;
			local2 = segmB[0] + kB * vectB;
			return (local - local2).squaredNorm();
		}
		return DBL_MAX; // means nearest point outof segments
	};
	auto _getDistanceOfPointAndSegmentINF = [&local](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double
	{
		Eigen::Vector3d vecSeg = (segm[1] - segm[0]);
		double projection = vecSeg.dot(point);
		//the projection must on segment
		if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
			return DBL_MAX;
		double k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
		local = segm[0] + k * vecSeg;
		return (local - point).squaredNorm();
	};
	std::array<array<Vector3d, 2>, 3> edgesA = { {
		{ triA[0], triA[1] },
		{ triA[1], triA[2] },
		{ triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { { 
		{ triB[0], triB[1] },
		{ triB[1], triB[2] },
		{ triB[2], triB[0] } } };
	// iterate all
	for (const auto& iterA : triA)  //vertex to vertex
	{
		for (const auto& iterB : triB)
		{
			dtemp = (iterA - iterB).squaredNorm();
			if (dtemp < dmin)
			{
				dmin = dtemp;
				res = { iterA, iterB };
			}
			//if (fabs(dmin) < eps)
			//	return res;
		}
	}
	for (const auto& edgeA : edgesA) // edge to edge
	{
		for (const auto& edgeB : edgesB)
		{
			dtemp = _getDistanceOfTwoSegmentsINF(edgeA, edgeB);
			if (dtemp < dmin)
			{
				dmin = dtemp;
				res = { local, local2 };
			}
		}
	}
	for (const auto& vertex : triA)
	{
		dtemp = _getDistanceOfPointAndPlaneINF(vertex, triB);  // vertex to face
		if (dtemp < dmin)
		{
			dmin = dtemp;
			res = { vertex, local };
		}
		for (const auto& edge : edgesB) // vertex to edge
		{
			dtemp = _getDistanceOfPointAndSegmentINF(vertex, edge);
			if (dtemp < dmin)
			{
				dmin = dtemp;
				res = { vertex,local };
			}
		}
	}
	for (const auto& vertex : triB)
	{
		dtemp = _getDistanceOfPointAndPlaneINF(vertex, triA);  // vertex to face
		if (dtemp < dmin)
		{
			dmin = dtemp;
			res = { local, vertex };
		}
		for (const auto& edge : edgesA) // vertex to edge
		{
			dtemp = _getDistanceOfPointAndSegmentINF(vertex, edge);
			if (dtemp < dmin)
			{
				dmin = dtemp;
				res = { local, vertex };
			}
		}
	}
	return res;
}

//must intersect
std::array<Eigen::Vector3d, 2> getTwoTrianglesIntersectPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	std::array<Vector3d, 2> res = { gVecNaN , gVecNaN }; // avoid separate
#ifdef STATISTIC_DATA_COUNT
	if (!isTwoTrianglesIntersectSAT(triA, triB))
		count_err_tris_sepa++;
	//	return res;
#endif
	Vector3d vecSeg, normal, local;
	auto _getIntersectOfSegmentAndPlaneINF = [&vecSeg, &normal](const std::array<Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& plane)->double
	{
		vecSeg = segment[1] - segment[0];
		normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]).normalized(); //avoid over threshold
		if (fabs(vecSeg.dot(normal)) < eps)
			return (fabs((segment[0] - plane[0]).dot(normal)) < eps) ? DBL_MAX : -DBL_MAX; // positive infinity means separate
		return (plane[0] - segment[0]).dot(normal) / vecSeg.dot(normal); //k
		//return segment[0] + k * vecSeg; // intersect point
	};
	auto _getPointOfTwoIntersectSegments = [&vecSeg, &normal](const std::array<Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB)->double
	{
		normal = (segmB[0] - segmB[1]).cross(vecSeg);
		if (normal.isZero(eps)) // intersect cause collinear
			return DBL_MAX;
		return (segmA[0] - segmB[0]).cross(segmA[0] - segmB[1]).norm() / normal.norm(); //k
		//return (segmB[0] - segmA[0]).cross(vecSeg).dot(normal) / normal.squaredNorm();
		//return segmA[0] + k * vecSeg;
	};
	auto _getEndPointsOfTwoCollinearSegments = [&res](const std::array<Vector3d, 2>& edgeA, const std::array<Eigen::Vector3d, 2>& edgeB)
	{
		if ((edgeA[0] - edgeB[0]).dot(edgeA[0] - edgeB[1]) <= 0 && (edgeA[1] - edgeB[0]).dot(edgeA[1] - edgeB[1]) <= 0)
			res = edgeA;
		else if ((edgeB[0] - edgeA[0]).dot(edgeB[0] - edgeA[1]) <= 0 && (edgeB[1] - edgeA[0]).dot(edgeB[1] - edgeA[1]) <= 0)
			res = edgeB;
		else
			res = { ((edgeA[0] - edgeB[0]).dot(edgeA[0] - edgeB[1])) <= 0.0 ? edgeA[0] : edgeA[1],
					((edgeB[0] - edgeA[0]).dot(edgeB[0] - edgeA[1])) <= 0.0 ? edgeB[0] : edgeB[1] };
	};
	std::array<array<Vector3d, 2>, 3> edgesA = { { 
		{ triA[0], triA[1] },
		{ triA[1], triA[2] },
		{ triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { { 
		{ triB[0], triB[1] },
		{ triB[1], triB[2] },
		{ triB[2], triB[0] } } };
	int count = 0;
	double k, k2;
	for (const auto& edgeA : edgesA)
	{
		k = _getIntersectOfSegmentAndPlaneINF(edgeA, triB); // refer revise direction of edgeA
		if (DBL_MAX == k) //positive infinity means coplanar
		{
			if (isPointInTriangle(edgeA[0], triB) && isPointInTriangle(edgeA[1], triB))
				return edgeA;
			for (const auto& edgeB : edgesB)
			{
				if (!isTwoSegmentsIntersect(edgeA, edgeB))
					continue;
				k2 = _getPointOfTwoIntersectSegments(edgeA, edgeB);
				if (DBL_MAX == k2) //collinear
				{
					_getEndPointsOfTwoCollinearSegments(edgeA, edgeB);
					return res;
				}
				res[count++] = edgeA[0] + k2 * (vecSeg); // first use second add
				if (count == 2)
					return res;
			}
		}
		else if (0 <= k && k <= 1)
		{
			local = edgeA[0] + k * vecSeg;
			if (!isPointInTriangle(local, triB))
				continue;
			res[count++] = local;
			if (count == 2)
				return res;
		}
	}
	for (const auto& edgeB : edgesB)
	{
		k = _getIntersectOfSegmentAndPlaneINF(edgeB, triA);
		if (DBL_MAX == k) //coplanar
		{
			if (isPointInTriangle(edgeB[0], triA) && isPointInTriangle(edgeB[1], triA))
				return edgeB;
			for (const auto& edgeA : edgesA)
			{
				if (!isTwoSegmentsIntersect(edgeB, edgeA))
					continue;
				k2 = _getPointOfTwoIntersectSegments(edgeB, edgeA);
				if (DBL_MAX == k2) //collinear
				{
					_getEndPointsOfTwoCollinearSegments(edgeA, edgeB);
					return res;
				}
				res[count++] = edgeB[0] + k2 * (vecSeg);
				if (count == 2)
					return res;
			}
		}
		else if (0 <= k && k <= 1)
		{
			local = edgeB[0] + k * vecSeg;
			if (!isPointInTriangle(local, triA))
				continue;
			res[count++] = local;
			if (count == 2)
				return res;
		}
	}
	return res;
}

//RelationOfTrigon getTwoTrianglesIntersectRelation(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
//{
//	if (!isTwoTrianglesIntersectSAT(triA, triB))
//		return RelationOfTrigon::SEPARATE;
//	Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[0]).normalized();
//	Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[0]).normalized();
//	if (normalA.cross(normalB).isZero(eps)) // parallel
//	{
//		if (isPointInTriangle(triA[0], triB) && isPointInTriangle(triA[1], triB) && isPointInTriangle(triA[2], triB))
//			return RelationOfTrigon::COPLANAR_A_INSIDE_B;
//		else if (isPointInTriangle(triB[0], triA) && isPointInTriangle(triB[1], triA) && isPointInTriangle(triB[2], triA))
//			return RelationOfTrigon::COPLANAR_B_INSIDE_A;
//		else
//			return RelationOfTrigon::COPLANAR_INTERSECT;
//	}
//	return RelationOfTrigon::INTERSECT;
//}

//the distance of two triangle no more than this number
//double getApproDistanceOfTwoTrianglesMax(const Triangle& triA, const Triangle& triB)
//{
//	double disSquare = 0.0;
//	for (const auto& vtA : triA)
//	{
//		for (const auto& vtB : triB)
//		{
//			double disTmp = (vtA - vtB).squaredNorm();
//			if (disSquare < disTmp)
//				disSquare = disTmp;
//		}
//	}
//	return std::sqrt(disSquare);
//}

// for profile section
bool psykronix::isTwoSegmentsCollinearCoincident(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
{
	//double operator^(const Vector2d& vec1, const Vector2d& vec2)
	auto _cross2d = [](const Vector2d& vec1, const Vector2d& vec2)->double
	{
		//return vec1.x() * vec2.y() - vec2.x() * vec1.y();
		return vec1[0] * vec2[1] - vec2[0] * vec1[1];
	};
	if (!isTwoSegmentsIntersect(segmA, segmB))
		return false;
	//return (segmA[1] - segmA[0]).cross(segmB[1] - segmB[0]).isZero(eps);
	return fabs(_cross2d(segmA[1] - segmA[0], segmB[1] - segmB[0])) < eps;
}

//bool psykronix::isTwoSegmentsCollinearCoincident(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
//{
//	if (!isTwoSegmentsIntersect(segmA, segmB))
//		return false;
//	return (segmA[1] - segmA[0]).cross(segmB[1] - segmB[0]).isZero(eps);
//}
//double _getDistanceOfPointAndSegmentINF(const Vector3d& point, const std::array<Vector3d, 2>& segm)
//{
//	Vector3d vecSeg = segm[1] - segm[0];// not zero
//	double projection = vecSeg.dot(point);
//	//the projection must on segment
//	if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
//		return DBL_MAX;
//	double k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
//	return (segm[0] - point + k * vecSeg).squaredNorm();
//};

#define USING_NORMALIZED_VECTOR
bool psykronix::isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB,
	double toleAng /*= 0*/, double toleDis /*= 0*/)
{
	// |A¡ÁB| <= |A||B|sin¦È, near zero sin¦È approx ¦È
#ifndef USING_NORMALIZED_VECTOR
	Vector3d segmVecA = (segmA[1] - segmA[0]).normalized();
	Vector3d segmVecB = (segmB[1] - segmB[0]).normalized();
	if (!segmVecA.cross(segmVecB).isZero(toleAng)) //cross product max component is toleAng
		return false;
#else
	Vector3d segmVecA = segmA[1] - segmA[0];
	Vector3d segmVecB = segmB[1] - segmB[0];
	//if (segmVecA.cross(segmVecB).squaredNorm() > segmVecA.squaredNorm() * segmVecB.squaredNorm() * toleAng * toleAng)
	if (segmVecA.norm() * segmVecB.norm() * toleAng < segmVecA.cross(segmVecB).norm())
		return false;
#endif // USING_NORMALIZED_VECTOR
	int interEnd = 0;
#ifdef USING_NORMALIZED_VECTOR
	for (const auto& endA : segmA)
	{
		segmVecA = segmB[0] - endA; // re-using variable name
		segmVecB = segmB[1] - endA;
		if (segmVecA.norm() <= toleDis || segmVecB.norm() <= toleDis) // point concident
		{
			interEnd++;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm()) //norm fast than squaredNorm
			interEnd++;
	}
	for (const auto& endB : segmB)
	{
		segmVecA = segmA[0] - endB;
		segmVecB = segmA[1] - endB;
		if (segmVecA.norm() <= toleDis || segmVecB.norm() <= toleDis) // point concident
		{
			interEnd++;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm())
			interEnd++;
	}
#else
	Vector3d vecSeg;
	double projection, k;
	auto _getDistanceOfPointAndSegmentINF = [&](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double
	{
		vecSeg = segm[1] - segm[0];// not zero
		projection = vecSeg.dot(point);
		//the projection must on segment
		if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
			return DBL_MAX;
		k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
		return (segm[0] - point + k * vecSeg).squaredNorm();
	};
	for (const auto& endA : segmA)
	{
		if (_getDistanceOfPointAndSegmentINF(endA, segmB) <= toleDis * toleDis)
			interEnd++;
	}
	for (const auto& endB : segmB)
	{
		if (_getDistanceOfPointAndSegmentINF(endB, segmA) <= toleDis * toleDis)
			interEnd++;
	}
#endif // USING_NORMALIZED_VECTOR
	return 1 < interEnd; //interEnd == 2/3/4
}

std::tuple<bool, array<double, 4>> psykronix::getTwoSegmentsCollinearCoincidentPoints(const array<Vector3d, 2>& segmA, const array<Vector3d, 2>& segmB,
	double toleAng /*= 0*/, double toleDis /*= 0*/)
{
	Vector3d segmVecA = segmA[1] - segmA[0];
	Vector3d segmVecB = segmB[1] - segmB[0];
	double propA0 = std::nan("0"), propA1 = std::nan("0"), propB0 = std::nan("0"), propB1 = std::nan("0");
	if (toleAng * segmVecA.norm() * segmVecB.norm() < segmVecA.cross(segmVecB).norm())
		return { false, { propA0, propA1, propB0, propB1 } }; //not collinear
	bool sameDirect = (segmVecA).dot(segmVecB) > 0;
	bool midInter = false;
	int endInter = 0;
	for (const auto& endA : segmA) //pointA on segmentB
	{
		segmVecA = endA - segmB[0]; // re-using variable name
		segmVecB = endA - segmB[1];
		if (segmVecA.norm() <= toleDis) // point concident
		{
			endInter++; // same as segmentB 
			propB0 = 0.0;
			continue;
		}
		if (segmVecB.norm() <= toleDis) // point concident
		{
			endInter++;
			propB1 = 1.0;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm()) //norm fast than squaredNorm
		{
			midInter = true;
			if (sameDirect)
			{
				if ((segmA[0] - segmB[0]).dot(segmA[0] - segmB[1]) < 0) // pointA0 in segmB
					propB0 = (segmA[0] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
				if ((segmA[1] - segmB[0]).dot(segmA[1] - segmB[1]) < 0) // pointA1 in segmB
					propB1 = (segmA[1] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
			}
			else //swap segmentA
			{
				if ((segmA[1] - segmB[0]).dot(segmA[1] - segmB[1]) < 0) // pointA0 in segmB
					propB0 = (segmA[1] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
				if ((segmA[0] - segmB[0]).dot(segmA[0] - segmB[1]) < 0) // pointA1 in segmB
					propB1 = (segmA[0] - segmB[0]).norm() / (segmB[1] - segmB[0]).norm();
			}
		}
	}
	for (const auto& endB : segmB) //pointB on segmentA
	{
		segmVecA = endB - segmA[0];
		segmVecB = endB - segmA[1];
		if (segmVecA.norm() <= toleDis) // point concident
		{
			propA0 = 0.0;
			continue;
		}
		if (segmVecB.norm() <= toleDis) // point concident
		{
			propA1 = 1.0;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDis
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm())
		{
			midInter = true;
			if (sameDirect)
			{
				if ((segmB[0] - segmA[0]).dot(segmB[0] - segmA[1]) < 0) // pointB0 in segmA
					propA0 = (segmB[0] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
				if ((segmB[1] - segmA[0]).dot(segmB[1] - segmA[1]) < 0) // pointB1 in segmA
					propA1 = (segmB[1] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
			}
			else//swap segmentB
			{
				if ((segmB[1] - segmA[0]).dot(segmB[1] - segmA[1]) < 0) // pointB0 in segmA
					propA0 = (segmB[1] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
				if ((segmB[0] - segmA[0]).dot(segmB[0] - segmA[1]) < 0) // pointB1 in segmA
					propA1 = (segmB[0] - segmA[0]).norm() / (segmA[1] - segmA[0]).norm();
			}
		}
	}
	//bool isInter = 1 < endInter; //interPoints == 2/3/4
	if (!midInter && endInter < 2) // no intersect
		return { false, { propA0, propA1, propB0, propB1  } };
	// propA0 means intersect part start, propA1 means end
	if (isnan(propA0))
		propA0 = 0.0;
	if (isnan(propA1))
		propA1 = 1.0;
	if (isnan(propB0))
		propB0 = 0.0;
	if (isnan(propB1))
		propB1 = 1.0;
	//for (int i = 0; i < 4; i++)
	//{
	//	if (isnan(propArr[i]))
	//		propArr[i] = (i % 2 == 0) ? 0.0 : 1.0;
	//}
	return { true, { propA0, propA1, propB0, propB1  } }; //
}

void psykronix::mergeIntersectRegionOfSegment(std::vector<double>& _range, const std::array<double, 2>& prop)//->void
{
	//if (range.empty())
	//	range = { { prop[0], prop[1] } };
	if (prop[1] < _range.front()) //prop left
	{
		//range.push_back({ prop[0], prop[1] });
		_range.push_back(prop[0]);
		_range.push_back(prop[1]);
		sort(_range.begin(), _range.end());
		return;
	}
	if (prop[0] > _range.back())//prop right
	{
		//range.push_back({ prop[0], prop[1] });
		_range.push_back(prop[0]);
		_range.push_back(prop[1]);
		return;
	}
	//transform vector<> to vector<pair>
	std::vector<pair<double, double>> range;
	for (size_t i = 0; i < _range.size() / 2; ++i)
		range.push_back({ _range[2 * i], _range[2 * i + 1] });
	vector<pair<double, double>> mergeRes;
	array<double, 2> segmIter = prop; // iterator segment
	bool isCover = false; //prop cover _range
	for (const auto& iter : range)
	{
		// merge // propSegm={p0, p1}
		if (prop[0] <= iter.first) //p0 left
		{
			if (prop[1] < iter.first) //p1 left
			{
				mergeRes.push_back(iter); //p1 right
				continue;// not intersect
			}
			if (prop[1] > iter.second)
			{
				isCover = true;
				continue; //cover iter
			}
			segmIter = { segmIter[0], iter.second };// p1 bettwen
			continue;
		}
		if (prop[0] > iter.second) //p0 right
		{
			mergeRes.push_back(iter);
			continue;
		}
		// p0 bettwen
		if (prop[1] > iter.second) //p1 right
		{
			segmIter = { iter.first, segmIter[1] };
		}
		else // p1 bettwen
		{
			mergeRes.push_back(iter);
		}
	}
	if (prop != segmIter || isCover)
		mergeRes.push_back({ segmIter[0], segmIter[1] });
	sort(mergeRes.begin(), mergeRes.end()); //ascending order
	//range = mergeRes;
	// transform vector<pair> to vector<>
	if (!mergeRes.empty())
	{
		_range.clear();
		for (const auto& iter : mergeRes)
		{
			_range.push_back(iter.first);
			_range.push_back(iter.second);
		}
	}
}

bool isPointInPolygon2D(const Eigen::Vector2d& point, const vector<Eigen::Vector2d>& polygon)// pnpoly
{
	AlignedBox2d box;
	for (const auto& iter : polygon)
		box.extend(iter);
	if (!box.contains(point))
		return false;
	bool isIn = false;
	int nvert = (int)polygon.size(); // polygon need not close
	int i, j;
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) &&
			(point[0] < (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]))
			isIn = !isIn;
	}
	return isIn;
}

bool isPointInPolygon2D(const Eigen::Vector3d& point, const vector<Eigen::Vector3d>& polygon)// pnpoly
{
	// point on polygon means false
	bool isIn = false;
	int nvert = (int)polygon.size();
	int i, j;
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) &&
			(point[0] < (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]))
			isIn = !isIn;
	}
	return isIn;
}

bool isTwoPolygonsIntersectSAT(const std::vector<Eigen::Vector2d>& polygonA, const std::vector<Eigen::Vector2d>& polygonB)
{
	// has been boundbox pre-judge
	vector<Eigen::Vector2d> axes;
	size_t i;
	Vector2d edge;
	for (i = 0; i < polygonA.size() - 1; i++)
	{
		edge = polygonA[i + 1] - polygonA[i];
		if (!edge.isZero())
			axes.push_back(Vector2d(-edge[1], edge[0]));
	}
	edge = polygonA[i] - polygonA[0];
	if (!edge.isZero())
		axes.push_back(Vector2d(-edge[1], edge[0]));
	for (i = 0; i < polygonB.size() - 1; i++)
	{
		edge = polygonB[i + 1] - polygonB[i];
		if (!edge.isZero())
			axes.push_back(Vector2d(-edge[1], edge[0]));
	}
	edge = polygonB[i] - polygonB[0];
	if (!edge.isZero())
		axes.push_back(Vector2d(-edge[1], edge[0]));
	//do judge
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes)
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& vertex : polygonA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : polygonB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + eps || maxB < minA + eps)
			return false;
	}
	return true;
}

Eigen::Matrix4d getProjectionMatrixByPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
{
	//get rot matrix from one vector
	const Vector3d& axisz = normal.normalized();
	Vector3d axisx = Vector3d(0,0,1).cross(axisz).normalized();
	//get the point on normal that through world origin
	double o_n = point.dot(normal);
	Vector3d origin = (o_n == 0.0) ? //correct plane's origin point
		point : o_n / normal.squaredNorm() * normal;
	if (axisx.isZero())
	{
		Matrix4d projection; // projection
		projection << 
			1, 0, 0, origin[0],
			0, 1, 0, origin[1],
			0, 0, 0, origin[2],
			0, 0, 0, 1;
		return projection;
	}
	Vector3d axisy = axisz.cross(axisx);
	Matrix4d matOri; //unit and orth
	matOri <<
		axisx[0], axisy[0], axisz[0], origin[0],
		axisx[1], axisy[1], axisz[1], origin[1],
		axisx[2], axisy[2], axisz[2], origin[2],
		0, 0, 0, 1;
	Matrix4d invPro; // projection
	invPro << // shadow_xoy * inverse
		axisx[0], axisx[1], axisx[2], 0,
		axisy[0], axisy[1], axisy[2], 0,
		0, 0, 0, 0, // dimension reduction
		0, 0, 0, 1;
	return matOri * invPro;
}

Matrix4d getProjectionMatrixByPlane(const Plane3d& plane)
{
	return getProjectionMatrixByPlane(plane.m_origin, plane.m_normal);
}

std::array<Eigen::Matrix4d, 2> getRelativeMatrixByProjectionPlane(const Eigen::Vector3d& origin, const Eigen::Vector3d& normal)
{
	Vector3d axisz = normal.normalized();
	Vector3d axisx = Vector3d(0, 0, 1).cross(axisz).normalized();
	if (axisx.isZero())
		return { Matrix4d::Identity(), Matrix4d::Identity() };
	Vector3d axisy = axisz.cross(axisx);
	Matrix4d matFor, matInv;// = Eigen::Affine3d::Identity();
	//mat.setByOriginAndVectors();
	matFor << //forword matrix
		axisx[0], axisy[0], axisz[0], origin[0],
		axisx[1], axisy[1], axisz[1], origin[1],
		axisx[2], axisy[2], axisz[2], origin[2],
		0, 0, 0, 1;
	matInv << //inverse matrix, transpose and negation
		axisx[0], axisx[1], axisx[2], -origin[0],
		axisy[0], axisy[1], axisy[2], -origin[1],
		axisz[0], axisz[1], axisz[2], -origin[2],
		0, 0, 0, 1;
	return { matFor,matInv };
}

std::array<Eigen::Matrix4d, 2> getRelativeMatrixByProjectionPlane(const Plane3d& plane)
{
	return getRelativeMatrixByProjectionPlane(plane.m_origin, plane.m_normal);
}
