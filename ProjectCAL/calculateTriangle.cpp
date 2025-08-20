#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;
#define USING_METHOD_SAT
//#define USING_THRESHOLD_CUSTOMIZE
//#define USING_ACCURATE_NORMALIZED
//#define USING_THRESHOLD_GEOMETRIC // to process collinar and coplanar
//#define USING_COMPLETE_SEPARATION_AXIS //3d=17

//--------------------------------------------------------------------------------------------------
//  triangle
//--------------------------------------------------------------------------------------------------

#ifdef STATISTIC_DATA_COUNT
std::atomic<size_t> 
count_isTwoTrisInter = 0, count_getTrisDistance = 0, count_isTrisBoundBoxInter = 0, count_isTrisAndBoxInter = 0,
count_pointInTri = 0, count_err_degen_tri = 0,
count_err_inputbox = 0, count_err_inter_dist = 0, count_err_repeat_tri = 0,
count_err_tris_sepa = 0, count_err_tris_inter = 0;
extern std::atomic<size_t> count_err_degen_tri, count_trigon_coplanar;
#endif //STATISTIC_DATA_COUNT
#ifdef STATISTIC_DATA_TESTFOR
extern std::atomic<size_t> count_gRTT;
#endif

double clash::computeTriangleArea(const std::array<Vector2d, 3>& triangle)
{
	Vector2d a = triangle[1] - triangle[0];
	Vector2d b = triangle[2] - triangle[0];
	return 0.5 * fabs(a[0] * b[1] - a[1] * b[0]); //cross value
}

double clash::computeTriangleArea(const std::array<Vector3d, 3>& triangle, bool is2D /*= true*/) //is2D means OnXoY
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
bool clash::isPointInTriangle(const Vector2d& point, const std::array<Vector2d, 3>& trigon) // 2D
{
	// using isLeft test
	const Vector2d& p0 = trigon[0];
	const Vector2d& p1 = trigon[1];
	const Vector2d& p2 = trigon[2];
	// compare z-value
#ifdef USING_THRESHOLD_GEOMETRIC
	double axisz = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
	axisz = (0.0 < axisz) ? 1.0 : -1.0;
	return !(
		axisz * ((p1.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p1.y() - p0.y()) < -epsF) || //bool isLeftA
		axisz * ((p2.x() - p1.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (p2.y() - p1.y()) < -epsF) || //bool isLeftB
		axisz * ((p0.x() - p2.x()) * (point.y() - p2.y()) - (point.x() - p2.x()) * (p0.y() - p2.y()) < -epsF));  //bool isLeftC
#else
	// (p1-p0).cross(p2-p1); -:20, *:11
	double axisz = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
	axisz = (0.0 < axisz) ? 1.0 : -1.0;
	return
		0.0 <= axisz * ((p1[0] - p0[0]) * (point[1] - p0[1]) - (point[0] - p0[0]) * (p1[1] - p0[1])) &&
		0.0 <= axisz * ((p2[0] - p1[0]) * (point[1] - p1[1]) - (point[0] - p1[0]) * (p2[1] - p1[1])) &&
		0.0 <= axisz * ((p0[0] - p2[0]) * (point[1] - p2[1]) - (point[0] - p2[0]) * (p0[1] - p2[1]));
#endif
}

bool clash::isPointInTriangleCCW(const Vector2d& point, const std::array<Vector2d, 3>& trigon) // 2D
{
	// using isLeft test
	const Vector2d& p0 = trigon[0];
	const Vector2d& p1 = trigon[1];
	const Vector2d& p2 = trigon[2];
	return
		0.0 <= ((p1[0] - p0[0]) * (point[1] - p0[1]) - (point[0] - p0[0]) * (p1[1] - p0[1])) &&
		0.0 <= ((p2[0] - p1[0]) * (point[1] - p1[1]) - (point[0] - p1[0]) * (p2[1] - p1[1])) &&
		0.0 <= ((p0[0] - p2[0]) * (point[1] - p2[1]) - (point[0] - p2[0]) * (p0[1] - p2[1]));
}

// using Barycentric Coordinates
bool isPointInTriangleBC(const Vector2d& point, const std::array<Vector2d, 3>& triangle)
{
	// -:14, *:6, /:2, dot:5(+:5, *:10)
	Eigen::Vector2d v0 = triangle[1] - triangle[0];
	Eigen::Vector2d v1 = triangle[2] - triangle[0];
	Eigen::Vector2d v2 = point - triangle[0];
	double d00 = v0.dot(v0);
	double d01 = v0.dot(v1);
	double d11 = v1.dot(v1);
	double d20 = v2.dot(v0);
	double d21 = v2.dot(v1);
	double deno = d00 * d11 - d01 * d01;
	//if (denom==0) //willnot happen, unless triangle degeneracy
	double a = (d11 * d20 - d01 * d21) / deno;
	double b = (d00 * d21 - d01 * d20) / deno;
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
	for (const auto& axis : axes) //none zero
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
bool clash::isPointInTriangle(const Vector3d& point, const std::array<Vector3d, 3>& trigon) //3D
{
#ifdef STATISTIC_DATA_COUNT
	count_pointInTri++;
#endif
	// precision problem, cause misjudge without tolerance
	//if (point.x() < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) /*+ -epsF*/ ||
	//	point.x() > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) /*+ epsF */ ||
	//	point.y() < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) /*+ -epsF*/ || 
	//	point.y() > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) /*+ epsF */ ||
	//	point.z() < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) /*+ -epsF*/ || 
	//	point.z() > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]) /*+ epsF */ )
	//	return false; // random data test fast 10% about
#ifdef USING_ACCURATE_NORMALIZED
	Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]).normalized();// using isLeft test
#else
	Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]);// using isLeft test
#endif // USING_ACCURATE_NORMALIZED
#ifdef USING_THRESHOLD_GEOMETRIC
	// 3D triangle, input point must coplanar
	return !(
		(trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) < -epsF || //bool isNotLeftA
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) < -epsF || //bool isNotLeftB
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal) < -epsF);  //bool isNotLeftC
#else
	return
		0.0 <= (trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) && //bool isLeftA
		0.0 <= (trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) && //bool isLeftB
		0.0 <= (trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal);   //bool isLeftC
#endif //USING_THRESHOLD_GEOMETRIC
	//+-*x< 0,5,1.5,2.5,1.5
	//if (((trigon[1] - trigon[0]).cross(point - trigon[0])).dot(normal) < -epsF) //bool isLeftA
	//	return false;
	//if (((trigon[2] - trigon[0]).cross(point - trigon[0])).dot(normal) > epsF) //bool isLeftB
	//	return false;
	//if (((trigon[2] - trigon[1]).cross(point - trigon[1])).dot(normal) < -epsF) //bool isLeftC
	//	return false;
	//return true;
	// area method
	//double dot1 = normal.dot((trigon[1] - trigon[0]).cross(point - trigon[0]));
	//double dot2 = normal.dot((point - trigon[0]).cross(trigon[2] - trigon[0]));
	//return 0.0 <= dot1 && 0.0 <= dot2 && dot1 + dot2 <= normal.dot(normal);
}

bool clash::isPointOnTriangleSurface(const Vector3d& point, const std::array<Vector3d, 3>& trigon)
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
	if ((trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) < -epsF ||
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) < -epsF ||
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal) < -epsF)
		return false;
	return fabs(normal.dot(point - trigon[0])) < epsF;
#else
	if ((trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) < 0.0 ||
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) < 0.0 ||
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal) < 0.0)
		return false;
	return normal.dot(point - trigon[0]) == 0.0;
#endif
}

//must coplanar
bool clash::isSegmentCrossTriangle(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) // must coplanar
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
	if ((dot0 > epsF && dot1 > epsF && dot2 > epsF) || (dot0 < -epsF && dot1 < -epsF && dot2 < -epsF))
		return false; //+-*x< 0,6,3,2,4.5
	// double straddling test x3
	return 
		isTwoSegmentsIntersect(segment, { trigon[0], trigon[1] }) ||
		isTwoSegmentsIntersect(segment, { trigon[1], trigon[2] }) ||
		isTwoSegmentsIntersect(segment, { trigon[2], trigon[0] });

	//bool edgea2segm = ((segment[0] - trigon[0]).cross(trigon[1] - trigon[0])).dot((segment[1] - trigon[0]).cross(trigon[1] - trigon[0])) < epsF;
	//bool segm2edgea = ((trigon[1] - segment[0]).cross(vecSeg)).dot((trigon[0] - segment[0]).cross(vecSeg)) < epsF;
	//if (edgea2segm && segm2edgea)
	//	return true;
	//bool edgeb2segm = ((segment[0] - trigon[1]).cross(trigon[2] - trigon[1])).dot((segment[1] - trigon[1]).cross(trigon[2] - trigon[1])) < epsF;
	//bool segm2edgeb = ((trigon[2] - segment[0]).cross(vecSeg)).dot((trigon[1] - segment[0]).cross(vecSeg)) < epsF;
	//if (edgeb2segm && segm2edgeb)
	//	return true;
	//bool edgec2segm = ((segment[0] - trigon[2]).cross(trigon[0] - trigon[2])).dot((segment[1] - trigon[2]).cross(trigon[0] - trigon[2])) < epsF;
	//bool segm2edgec = ((trigon[0] - segment[0]).cross(vecSeg)).dot((trigon[2] - segment[0]).cross(vecSeg)) < epsF;
	//if (edgec2segm && segm2edgec)
	//	return true;
	//return false;
}

// segment must through trigon inner plane //start point outof plane
bool clash::isSegmentCrossTriangleSurface(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon)
{
	// compare angle of normal-vector
	Vector3d vecSeg = segment[1] - segment[0];
	double dotA = (trigon[0] - segment[0]).cross(trigon[1] - segment[0]).dot(vecSeg);
	double dotB = (trigon[1] - segment[0]).cross(trigon[2] - segment[0]).dot(vecSeg);
	double dotC = (trigon[2] - segment[0]).cross(trigon[0] - segment[0]).dot(vecSeg);
	return (dotA < epsF && dotB < epsF && dotC < epsF) || (dotA > -epsF && dotB > -epsF && dotC > -epsF);
}

// Method: judge intersect point in triangle
bool clash::isTwoTrianglesIntersectPIT(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{
	//#ifdef STATISTIC_DATA_COUNT
	//	isTwoTrianglesInter++;
	//#endif
		// pre box check
		//if (!isTwoTrianglesBoundingBoxIntersect(triL, triR))
		//	return false;
		// right edge through left plane
	Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[1]);
	bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < epsF; //include point on plane(dot==0)
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < epsF;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < epsF;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;
	// left edge through right plane
	Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[1]);
	bool acrossL2R_A = (veczR.dot(triL[0] - triR[0])) * (veczR.dot(triL[1] - triR[0])) < epsF;
	bool acrossL2R_B = (veczR.dot(triL[1] - triR[0])) * (veczR.dot(triL[2] - triR[0])) < epsF;
	bool acrossL2R_C = (veczR.dot(triL[2] - triR[0])) * (veczR.dot(triL[0] - triR[0])) < epsF;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;
	// forward
	double dotPro;// = 0;
	if (acrossR2L_A) // R0 
	{
		//edgeA={triR[0], triR[1]}
		dotPro = veczL.dot(triR[0] - triR[1]); //dotA
		if (fabs(dotPro) < epsF) // perpendi to veczL (veczL dot edgeA is zero)
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
		if (fabs(dotPro) < epsF) // perpendi to veczL
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
		if (fabs(dotPro) < epsF) // perpendi to veczL
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
		if (fabs(dotPro) < epsF) // perpendi to veczR
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
		if (fabs(dotPro) < epsF) // perpendi to veczR
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
		if (fabs(dotPro) < epsF) // perpendi to veczR
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
bool clash::isTwoTrianglesIntersectEIT(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{
	Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[1]); // triL close face triangle
	bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < epsF; //include point-on-plane
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < epsF;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < epsF;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;
	Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[1]); // triR close face triangle
	bool acrossL2R_A = (veczR.dot(triL[0] - triR[0])) * (veczR.dot(triL[1] - triR[0])) < epsF; //include point-on-plane
	bool acrossL2R_B = (veczR.dot(triL[1] - triR[0])) * (veczR.dot(triL[2] - triR[0])) < epsF;
	bool acrossL2R_C = (veczR.dot(triL[2] - triL[0])) * (veczR.dot(triL[0] - triL[0])) < epsF;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;
	// using face to-left test
	bool pointOnfaceS /*= false*/;
	bool pointOnfaceE /*= false*/;
	if (acrossR2L_A) // first filter
	{
		pointOnfaceS = fabs(veczL.dot(triR[0] - triL[0])) < epsF; //start point
		pointOnfaceE = fabs(veczL.dot(triR[1] - triL[0])) < epsF; //end point
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
		pointOnfaceS = fabs(veczL.dot(triR[1] - triL[0])) < epsF; //start point
		pointOnfaceE = fabs(veczL.dot(triR[2] - triL[0])) < epsF; //end point
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
		pointOnfaceS = fabs(veczL.dot(triR[2] - triL[0])) < epsF; //start point
		pointOnfaceE = fabs(veczL.dot(triR[0] - triL[0])) < epsF; //end point
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
		pointOnfaceS = fabs(veczR.dot(triL[0] - triR[0])) < epsF; //start point
		pointOnfaceE = fabs(veczR.dot(triL[1] - triR[0])) < epsF; //end point
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
		pointOnfaceS = fabs(veczR.dot(triL[1] - triR[0])) < epsF; //start point
		pointOnfaceE = fabs(veczR.dot(triL[2] - triR[0])) < epsF; //end point
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
		pointOnfaceS = fabs(veczR.dot(triL[2] - triR[0])) < epsF; //start point
		pointOnfaceE = fabs(veczR.dot(triL[0] - triR[0])) < epsF; //end point
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
RelationOfTwoTriangles clash::getRelationOfTwoTrianglesSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB)
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
	if (edgesA[0].cross(edgesA[1]).cross(edgesB[0].cross(edgesB[1])).isZero(epsF))//&& fabs(normalA.dot(triB[1] - triA[1])) < epsF)
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
		if (axis.isZero())
			continue;
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
	if (fabs(dmin) < epsF)
		return RelationOfTwoTriangles::CONTACT;
	return RelationOfTwoTriangles::INTRUSIVE;
}

std::tuple<Vector3d, double> clash::getTriangleBoundingCircle(const std::array<Vector3d, 3>& trigon) //return center and radius
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
	//if (normal.squaredNorm() > epsF) //trigon legal
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

bool clash::isSegmentAndTriangleIntersctSAT(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon)
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
	if (fabs(normal.dot(segment[0] - trigon[0])) < epsF && fabs(normal.dot(segment[1] - trigon[0])) < epsF)// 2D-SAT, coplanar
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
			if (axis.isZero())
				continue;
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
			if (axis.isZero())
				continue;
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
bool clash::isPointRayAcrossTriangleSAT(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon)
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
		if (axis.isZero())
			continue;
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
		if (minTri + epsF < std::min(dotP, dotR) || std::max(dotP, dotR) + epsF < minTri)
#else
		if (maxTri < std::min(dotP, dotR) || std::max(dotP, dotR) < minTri) // absolute zero
#endif
			return false;
	}
	return true;
	// another method, judge 2D point in triangle, point under plane
}

//Moller Trumbore Algorithm
int clash::isRayLineCrossTriangleMTA(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Triangle& trigon)
{
	//(Cost = 0 div, 27 mul, 17 add)
	Vector3d E1 = trigon[1] - trigon[0];
	Vector3d E2 = trigon[2] - trigon[0];
	Vector3d S = origin - trigon[0];
	Vector3d S1 = direction.cross(E2);
	Vector3d S2 = S.cross(E1);
	double k = S1.dot(E1);
	if (fabs(k) < epsF)// ray parallel with trigon plane (include through, both illegal)
	{
		// through edge, should change ray
#ifdef RAY_METHOD_INSIDE_JUDGE
		Vector3d normal = E1.cross(E2);
		if (fabs(normal.dot(S)) < epsF && ( // point on plane
			// single straddle test && intersect point t<=0
			(0 <= (trigon[0] - origin).cross(direction).dot((direction).cross(trigon[1] - origin)) && 
				0 <= (trigon[0] - origin).cross(trigon[1] - trigon[0]).dot(normal) / direction.cross(trigon[1] - trigon[0]).dot(normal)) ||
			(0 <= (trigon[1] - origin).cross(direction).dot((direction).cross(trigon[2] - origin)) && 
				0 <= (trigon[1] - origin).cross(trigon[2] - trigon[1]).dot(normal) / direction.cross(trigon[2] - trigon[1]).dot(normal)) ||
			(0 <= (trigon[2] - origin).cross(direction).dot((direction).cross(trigon[0] - origin)) && 
				0 <= (trigon[2] - origin).cross(trigon[0] - trigon[2]).dot(normal) / direction.cross(trigon[0] - trigon[2]).dot(normal))))
			return -4;
#endif
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

//--------------------------------------------------------------------------------------------------
//  namespace eigen
//--------------------------------------------------------------------------------------------------

bool eigen::isTwoTrianglesBoxIntersect(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB, double tolerance /*= 0.0*/)
{
#ifdef STATISTIC_DATA_COUNT
	count_isTrisBoundBoxInter++;
#endif
	//get min and max of two trigons
	return 
        std::min(std::min(triA[0][0], triA[1][0]), triA[2][0]) <= std::max(std::max(triB[0][0], triB[1][0]), triB[2][0]) + tolerance &&
        std::min(std::min(triB[0][0], triB[1][0]), triB[2][0]) <= std::max(std::max(triA[0][0], triA[1][0]), triA[2][0]) + tolerance &&
        std::min(std::min(triA[0][1], triA[1][1]), triA[2][1]) <= std::max(std::max(triB[0][1], triB[1][1]), triB[2][1]) + tolerance &&
        std::min(std::min(triB[0][1], triB[1][1]), triB[2][1]) <= std::max(std::max(triA[0][1], triA[1][1]), triA[2][1]) + tolerance &&
        std::min(std::min(triA[0][2], triA[1][2]), triA[2][2]) <= std::max(std::max(triB[0][2], triB[1][2]), triB[2][2]) + tolerance &&
        std::min(std::min(triB[0][2], triB[1][2]), triB[2][2]) <= std::max(std::max(triA[0][2], triA[1][2]), triA[2][2]) + tolerance;
}

//without tolerance, canbe extend boundbox
bool eigen::isTriangleAndBoxIntersectSAT(const std::array<Eigen::Vector2d, 3>& trigon, const Eigen::AlignedBox2d& box)
{
	std::array<Eigen::Vector2d, 6> edges = {
		trigon[1] - trigon[0],
		trigon[2] - trigon[1],
		trigon[0] - trigon[2], };
	for (auto& axis : edges)
		axis = Vector2d(-axis[1], axis[0]);
	std::array<Eigen::Vector2d, 4> vertexes = {
		box.min(),
		box.max(),
		box.min() + Vector2d(box.sizes().x(), 0),
		box.min() + Vector2d(0, box.sizes().y()) };
	std::array<Eigen::Vector2d, 5> axes = { {
		Vector2d(1,0),
		Vector2d(0,1),
		edges[0],
		edges[1],
		edges[2],
	} };
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : axes) //none zero
	{
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const auto& iter : trigon)
		{
			projection = iter.dot(axis);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& iter : vertexes)
		{
			projection = iter.dot(axis);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB || maxB < minA) // absolute zero
			return false;
	}
	return true;
}

bool eigen::isTriangleAndBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box)
{
#ifdef STATISTIC_DATA_COUNT
	count_isTrisAndBoxInter++;
#endif
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
		//coords[0], //been excluded by extreme value
		//coords[1],
		//coords[2],
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
	//for (auto& axis : axes) // absolute can give up
	//{
	//	if (axis.isZero())
	//		axis = Vector3d(1, 0, 0);
	//}
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
	// iterate
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
		if (maxA < minB || maxB < minA) // absolute zero
			return false;
	}
	return true;
}

//for shield record
bool eigen::isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector2d, 3>& triA, const std::array<Eigen::Vector2d, 3>& triB, double tolerance /*= 0.0*/)
{
	std::array<Eigen::Vector2d, 6> edgesAB = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2],
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2], };
	for (Eigen::Vector2d& axis : edgesAB)
		axis = Eigen::Vector2d(-axis[1], axis[0]); //rotz(pi/2)
	// Check for overlap along each axis
	double minA, maxA, minB, maxB, projection;
	for (const Eigen::Vector2d& axis : edgesAB) //fast than index
	{
		//if (axis.isZero()) //degeneracy triangle, regard as not shield
		//	continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Eigen::Vector2d& vertex : triA) //fast than list
		{
			projection = axis.dot(vertex - triA[0]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Eigen::Vector2d& vertex : triB)
		{
			projection = axis.dot(vertex - triA[0]);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA <= minB + tolerance || maxB <= minA + tolerance) //contact, regard as not shield
			return false;
	}
	return true;
}

bool eigen::isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB,
	const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB, double tolerance /*= 0.0*/)
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

	double minA, maxA, minB, maxB, projection;
#ifdef USING_COMPLETE_SEPARATION_AXIS 
	std::array<Eigen::Vector3d, 17> axes = { { // compat zero-vector from parallel edges
			edgesA[0].cross(edgesB[0]),//cross edge pair to get normal
			edgesA[0].cross(edgesB[1]),
			edgesA[0].cross(edgesB[2]),
			edgesA[1].cross(edgesB[0]),
			edgesA[1].cross(edgesB[1]),
			edgesA[1].cross(edgesB[2]),
			edgesA[2].cross(edgesB[0]),
			edgesA[2].cross(edgesB[1]),
			edgesA[2].cross(edgesB[2]),
			normalA, //normal direction projection
			normalB,
			normalA.cross(edgesA[0]), //perpendi to edge when coplanar
			normalA.cross(edgesA[1]),
			normalA.cross(edgesA[2]),
			normalB.cross(edgesB[0]),
			normalB.cross(edgesB[1]),
			normalB.cross(edgesB[2])
		} };
#else
	if (normalA.cross(normalB).isZero())//is parallel
	{
		double toleDist = (tolerance < 0.0) ? -tolerance : 1e-8;
		if (toleDist < fabs(normalA.dot(triB[0] - triA[0]))) //not coplanar
			return false;
		std::array<Eigen::Vector3d, 6> edgesAB = {
			normalA.cross(edgesA[0]), //perpendi to edge when coplanar
			normalA.cross(edgesA[1]),
			normalA.cross(edgesA[2]),
			normalB.cross(edgesB[0]),
			normalB.cross(edgesB[1]),
			normalB.cross(edgesB[2]) };
		for (const Vector3d& axis : edgesAB)
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
			if (maxA < minB + tolerance || maxB < minA + tolerance)
				return false;
		}
		return true;
	}
	std::array<Eigen::Vector3d, 11> axes = {
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
		edgesA[2].cross(edgesB[2]),
	};
#endif
	// Check for overlap along each axis
	for (const Eigen::Vector3d& axis : axes) //fast than index
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Eigen::Vector3d& vertex : triA) //fast than list
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Eigen::Vector3d& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + tolerance || maxB < minA + tolerance) //contact is intersect
			return false; //one axis gap is separate
	}
	// special handling degenerate triangle
	//Eigen::Vector3d croA = edgesA[0].cross(edgesA[1]);
	//Eigen::Vector3d croB = edgesB[0].cross(edgesB[1]);
	//return !(edgesA[0].cross(edgesA[1]).isZero() || edgesB[0].cross(edgesB[1]).isZero());
	return true;
}

bool eigen::isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance /*= 0.0*/)
{
	Eigen::Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[1]);
	Eigen::Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[1]);
	return isTwoTrianglesIntersectSAT(triA, triB, normalA, normalB, tolerance);
}

// intersect and penetration judge
bool eigen::isTwoTrianglesIntrusionSAT(const std::array<Vector2d, 3>& triA, const std::array<Vector2d, 3>& triB, double tolerance /*= 0.0*/)
{
	// bound box been judged
	//if (std::max(std::max(triA[0][0], triA[1][0]), triA[2][0]) <= std::min(std::min(triB[0][0], triB[1][0]), triB[2][0]) ||
	//	std::max(std::max(triA[0][1], triA[1][1]), triA[2][1]) <= std::min(std::min(triB[0][1], triB[1][1]), triB[2][1]) ||
	//	std::max(std::max(triB[0][0], triB[1][0]), triB[2][0]) <= std::min(std::min(triA[0][0], triA[1][0]), triA[2][0]) ||
	//	std::max(std::max(triB[0][1], triB[1][1]), triB[2][1]) <= std::min(std::min(triA[0][1], triA[1][1]), triA[2][1]))
	//	return false;
	std::array<Eigen::Vector2d, 6> edgesAB = {
		(triA[1] - triA[0]).normalized(),
		(triA[2] - triA[1]).normalized(),
		(triA[0] - triA[2]).normalized(),
		(triB[1] - triB[0]).normalized(),
		(triB[2] - triB[1]).normalized(),
		(triB[0] - triB[2]).normalized() };
	for (auto& iter : edgesAB)
		iter = Vector2d(-iter[1], iter[0]); //rotz(pi/2)
	double minA, maxA, minB, maxB, projection;
	for (const auto& axis : edgesAB)
	{
		//if (axis.isZero()) //zero vector cause misjudgment, but preproccessed
		//	continue;
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
		if (maxA < minB + tolerance || maxB < minA + tolerance) //tolerance>0 means less
			return false; //without normlized
	}
	return true;
}

bool eigen::isTwoTrianglesIntrusionSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB, 
	const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB, double tolerance /*= 0.0*/)
{
	std::array<Eigen::Vector3d, 3> edgesA = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = {
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
#ifdef _DEBUG
	int i = -1;
#endif
	double minA, maxA, minB, maxB, projection;
#ifdef USING_COMPLETE_SEPARATION_AXIS 
	std::array<Eigen::Vector3d, 17> axes = { {
		edgesA[0].cross(edgesB[0]).normalized(),
		edgesA[0].cross(edgesB[1]).normalized(),
		edgesA[0].cross(edgesB[2]).normalized(),
		edgesA[1].cross(edgesB[0]).normalized(),
		edgesA[1].cross(edgesB[1]).normalized(),
		edgesA[1].cross(edgesB[2]).normalized(),
		edgesA[2].cross(edgesB[0]).normalized(),
		edgesA[2].cross(edgesB[1]).normalized(),
		edgesA[2].cross(edgesB[2]).normalized(),
		normalA, //normal direction projection
		normalB,
		normalA.cross(edgesA[0]).normalized(), //perpendi to edge when coplanar
		normalA.cross(edgesA[1]).normalized(),
		normalA.cross(edgesA[2]).normalized(),
		normalB.cross(edgesB[0]).normalized(),
		normalB.cross(edgesB[1]).normalized(),
		normalB.cross(edgesB[2]).normalized()
		} };
#else
	if (normalA.cross(normalB).isZero())//is parallel
	{
#ifdef _DEBUG
		double dotpro = fabs(normalA.dot(triB[0] - triA[0]));
#endif
		double toleDist = (tolerance < 0.0) ? -tolerance : 1e-8;
        if (toleDist < fabs(normalA.dot(triB[0] - triA[0]))) //not coplanar
			return false;
		std::array<Eigen::Vector3d, 6> edgesAB = { 
			normalA.cross(edgesA[0]).normalized(), //perpendi to edge when coplanar
			normalA.cross(edgesA[1]).normalized(),
			normalA.cross(edgesA[2]).normalized(),
			normalB.cross(edgesB[0]).normalized(),
			normalB.cross(edgesB[1]).normalized(),
			normalB.cross(edgesB[2]).normalized() };
		for (const Vector3d& axis : edgesAB)
		{
#ifdef _DEBUG
			i++;
#endif
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
			if (maxA < minB + tolerance || maxB < minA + tolerance)
				return false;
		}
		return true;
	}
	std::array<Eigen::Vector3d, 11> axes = { 
		normalA,
		normalB,
		edgesA[0].cross(edgesB[0]).normalized(),
		edgesA[0].cross(edgesB[1]).normalized(),
		edgesA[0].cross(edgesB[2]).normalized(),
		edgesA[1].cross(edgesB[0]).normalized(),
		edgesA[1].cross(edgesB[1]).normalized(),
		edgesA[1].cross(edgesB[2]).normalized(),
		edgesA[2].cross(edgesB[0]).normalized(),
		edgesA[2].cross(edgesB[1]).normalized(),
		edgesA[2].cross(edgesB[2]).normalized() };
#endif
	for (const Vector3d& axis : axes)
	{
#ifdef _DEBUG
		i++;
#endif
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
		if (maxA < minB + tolerance || maxB < minA + tolerance)
			return false;
	}
	return true;
}

bool eigen::isTwoTrianglesIntrusionSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB, double tolerance /*= 0.0*/)
{
	Eigen::Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[1]).normalized();
	Eigen::Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[1]).normalized();
	return isTwoTrianglesIntrusionSAT(triA, triB, normalA, normalB, tolerance);
}

//#define WITHOUT_COPLANAR_VERSION
#ifdef WITHOUT_COPLANAR_VERSION
bool eigen::isTwoTrianglesIntrusionSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB, double tolerance /*= 0.0*/)
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
	if (normalA.cross(normalB).isZero(tolerance)) //is parallel
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
#endif

//origin Penetration
bool eigen::isTwoTrianglesIntrusionSAT(double toleDist, const std::array<Vector2d, 3>& triA, const std::array<Vector2d, 3>& triB)
{
	// bounding-box been judged
	std::array<Eigen::Vector2d, 6> edgesAB = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2],
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	double spec = 0; // using max-coord k as coeff //specification 
	for (Vector2d& iter: edgesAB)
	{
		spec = std::max(std::max(fabs(iter[0]), fabs(iter[1])), spec);
		iter.normalize();
	}
	spec = spec * toleDist;
	for (Eigen::Vector2d& iter : edgesAB)
		iter = Eigen::Vector2d(-iter[1], iter[0]); //rotz(pi/2)
	double minA, maxA, minB, maxB, projection;
	for (Eigen::Vector2d& axis : edgesAB)
	{
		if (axis.isZero()) //degeneracy triangle
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Eigen::Vector2d& vertex : triA)
		{
			projection = axis.dot(vertex - triA[0]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Eigen::Vector2d& vertex : triB)
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
bool eigen::isTwoTrianglesIntrusionSAT(double toleDist, const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB,
	const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB)
{
	// bounding-box been judged, both triangle is legal
	std::array<Eigen::Vector3d, 3> edgesA = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = {
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	double spec = 0; // using max-coord k as coeff
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			spec = std::max(std::max(fabs(edgesA[i][j]), fabs(edgesB[i][j])), spec);
	}
	spec = spec * toleDist;
	double minA, maxA, minB, maxB, projection;
	//Eigen::Vector3d normalA = edgesA[0].cross(edgesA[1]).normalized();
	//Eigen::Vector3d normalB = edgesB[0].cross(edgesB[1]).normalized();
	//separation axis theorem
	std::array<Eigen::Vector3d, 17> axes = { {
		edgesA[0].cross(edgesB[0]).normalized(),
		edgesA[0].cross(edgesB[1]).normalized(),
		edgesA[0].cross(edgesB[2]).normalized(),
		edgesA[1].cross(edgesB[0]).normalized(),
		edgesA[1].cross(edgesB[1]).normalized(),
		edgesA[1].cross(edgesB[2]).normalized(),
		edgesA[2].cross(edgesB[0]).normalized(),
		edgesA[2].cross(edgesB[1]).normalized(),
		edgesA[2].cross(edgesB[2]).normalized(),
		normalA,
		normalB,
		normalA.cross(edgesA[0]).normalized(),
		normalA.cross(edgesA[1]).normalized(),
		normalA.cross(edgesA[2]).normalized(),
		normalB.cross(edgesB[0]).normalized(),
		normalB.cross(edgesB[1]).normalized(),
		normalB.cross(edgesB[2]).normalized()
		} };
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
			projection = axis.dot(vertex - triA[0]);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Vector3d& vertex : triB)
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

bool eigen::isTwoTrianglesIntrusionSAT(double toleDist, const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB)
{
	Eigen::Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[1]).normalized();
	Eigen::Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[1]).normalized();
	return isTwoTrianglesIntrusionSAT(toleDist, triA, triB, normalA, normalB);
}

//must intersect before
double eigen::getTrianglesIntrusionSAT(const std::array<Eigen::Vector2d, 3>& triA, const std::array<Eigen::Vector2d, 3>& triB)
{
	std::array<Eigen::Vector2d, 6> edgesAB = {
		(triA[1] - triA[0]).normalized(),
		(triA[2] - triA[1]).normalized(),
		(triA[0] - triA[2]).normalized(),
		(triB[1] - triB[0]).normalized(),
		(triB[2] - triB[1]).normalized(),
		(triB[0] - triB[2]).normalized() };
	for (Vector2d& iter : edgesAB)
		iter = Vector2d(-iter[1], iter[0]); //rotz(pi/2)
	double intrusion = DBL_MAX;
	double minA, maxA, minB, maxB, projection;
	for (const Vector2d& axis : edgesAB)
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Vector2d& vertex : triA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Vector2d& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (minB < maxA || minA < maxB) //without tolerance
			intrusion = std::min(std::min(maxA - minB, maxB - minA), intrusion);
	}
	return intrusion;
}

double eigen::getTrianglesIntrusionSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
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
	if (normalA.cross(normalB).isZero())//isParallel3d, means not intrusive
		return 0.0;
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
	double intrusion = DBL_MAX;
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
		if (minB < maxA || minA < maxB) //without tolerance
			intrusion = std::min(std::min(maxA - minB, maxB - minA), intrusion);
	}
	return intrusion;
}

double eigen::getDistanceOfPointAndTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon)
{
	//https://www.bilibili.com/video/BV1MF4m1V7e3/
	if (isPointInTriangle(point, trigon))
		return -1;
	double distance = DBL_MAX;
	//for (const auto& vertex : trigon)
	for (int i = 0; i < 3; ++i)
	{
		distance = std::min((trigon[i] - point).norm(), distance);
		int j = (i + 1) % 3;
		Vector2d direction = trigon[j] - trigon[i];
		if (0.0 < direction.dot(point - trigon[i]) && direction.dot(point - trigon[j]) < 0.0) //relative and projection
		{
			//double h = (point - trigon[i]).cross(direction).norm() / direction.norm();
			//double h = (trigon[i] - point + direction.dot(point - trigon[i]) / direction.dot(direction) * direction).norm();
			distance = std::min(cross2d(point - trigon[i], direction) / direction.norm(), distance);
		}
	}
	return distance;
}

double eigen::getDistanceOfPointAndTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon)
{
	bool isInTri;
	auto _getDistanceOfPointAndPlane = [&isInTri](const Vector3d& vertex, const std::array<Vector3d, 3>& plane)->double
		{
			Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
			double k = (plane[0] - vertex).dot(normal) / normal.dot(normal);
			//_getProjectionOfPointAndPlane
			Vector3d local = vertex + k * normal; // reference closure
			isInTri = isPointInTriangle(local, plane);
			return (k * normal).norm();
		};
	double perpendi = _getDistanceOfPointAndPlane(point, trigon);
	if (isInTri)
		return perpendi;
	// distance of point to frame line
	double distance = DBL_MAX;
	for (int i = 0; i < 3; ++i)
	{
		distance = std::min((trigon[i] - point).norm(), distance);
		int j = (i + 1) % 3;
		Vector3d direction = trigon[j] - trigon[i];
		if (0.0 < direction.dot(point - trigon[i]) && direction.dot(point - trigon[j]) < 0.0) //relative and projection
			distance = std::min((point - trigon[i]).cross(direction).norm() / direction.norm(), distance);
	}
	return distance;
}

// must separate
double eigen::getTrianglesDistanceSAT(const std::array<Eigen::Vector2d, 3>& triA, const std::array<Eigen::Vector2d, 3>& triB)
{
	auto _getDistanceOfPointAndSegmentINF = [](const Vector2d& point, const std::array<Vector2d, 2>& segm)->double
		{
			Vector2d vectseg = segm[1] - segm[0];// canbe zero, next willbe zero
			double projection = vectseg.dot(point);
			//the projection must on segment
			if (vectseg.dot(segm[1]) <= projection || projection <= vectseg.dot(segm[0]))
				return DBL_MAX;
			double k = vectseg.dot(point - segm[0]) / vectseg.dot(vectseg);
			return (segm[0] - point + k * vectseg).squaredNorm();
		};
	std::array<array<Vector2d, 2>, 3> edgesA = { {
		{ triA[0], triA[1] },
		{ triA[1], triA[2] },
		{ triA[2], triA[0] } } };
	std::array<array<Vector2d, 2>, 3> edgesB = { {
		{ triB[0], triB[1] },
		{ triB[1], triB[2] },
		{ triB[2], triB[0] } } };
	double distance = DBL_MAX;
	for (const Vector2d& vertexA : triA)
	{
		//point to point
		for (const Vector2d& vertexB : triB)
		{
			double norm = (vertexA - vertexB).squaredNorm();
			distance = std::min(distance, norm);
		}
		//point to edge
		for (const auto& edgeB : edgesB)
		{
			double temp = _getDistanceOfPointAndSegmentINF(vertexA, edgeB);
			distance = std::min(distance, temp);
		}
	}
	for (const Vector2d& vertexB : triB)
	{
		//point to edge
		for (const auto& edgeA : edgesA)
		{
			double temp = _getDistanceOfPointAndSegmentINF(vertexB, edgeA);
			distance = std::min(distance, temp);
		}
	}
	return std::sqrt(distance);
}

//old version, confused
double eigen::getTrianglesDistanceSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
#ifdef STATISTIC_DATA_COUNT
	count_getTrisDistance++;
#endif
	//if (isTwoTrianglesIntersectSAT(triA, triB))
	//	return 0.0;
	double dmin = DBL_MAX, dtemp;
	Vector3d direction, vecSeg, vectA, vectB; // to iterate, get nearest direction
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
			double deltaA = (segmB[0] - segmA[0]).dot(vectA);
			double deltaB = (segmB[0] - segmA[0]).dot(vectB);
			// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
			double deno = vectA.dot(vectB) * vectB.dot(vectA) - vectA.dot(vectA) * vectB.dot(vectB);//a*d-b*c
			if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
				return DBL_MAX;
			double kA = (vectB.dot(vectA) * deltaB - vectB.dot(vectB) * deltaA) / deno;
			double kB = (vectA.dot(vectA) * deltaB - vectA.dot(vectB) * deltaA) / deno;
			//	Vector3d pointA = segmA[0] + kA * vectA;
			//	Vector3d pointB = segmB[0] + kB * vectB;
			//whether two intersect-point inside segments
			if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
				return (segmA[0] + kA * vectA - segmB[0] - kB * vectB).squaredNorm();
			return DBL_MAX; // nearest point outof segments
		};
	auto _getNearestAxisOfTwoSegments = [&](const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)// major capture axis
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
	std::array<array<Vector3d, 2>, 3> edgesA = { {
		{ triA[0], triA[1] },
		{ triA[1], triA[2] },
		{ triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { {
		{ triB[0], triB[1] },
		{ triB[1], triB[2] },
		{ triB[2], triB[0] } } };
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
}

//simple version
double eigen::getTrianglesDistanceSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB)
{
	auto _getDistanceOfPointAndSegmentINF = [](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double
		{
			Vector3d vectseg = segm[1] - segm[0];// canbe zero, next willbe zero
			//if (vectseg.isZero())
			//	return DBL_MAX;
			double projection = vectseg.dot(point);
			//the projection must on segment
			if (vectseg.dot(segm[1]) <= projection || projection <= vectseg.dot(segm[0]))
				return DBL_MAX;
			double k = vectseg.dot(point - segm[0]) / vectseg.dot(vectseg);
			return (segm[0] - point + k * vectseg).squaredNorm();
		};
	auto _getDistanceOfPointAndPlaneINF = [](const Vector3d& point, const std::array<Vector3d, 3>& trigon, const Vector3d& normal)->double
		{
			//if (normal.isZero()) // error triangle plane
			//	return DBL_MAX;
			double k = (trigon[0] - point).dot(normal) / normal.dot(normal);
			Vector3d local = point + k * normal; // reference closure
			//if (!isPointInTriangle(local, plane))
			constexpr double toleDist = 1e-8; //fixed tolerance
			if (local[0] + toleDist < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				local[0] - toleDist > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				local[1] + toleDist < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				local[1] - toleDist > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				local[2] + toleDist < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
				local[2] - toleDist > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
				(trigon[1] - trigon[0]).cross(local - trigon[0]).dot(normal) <= 0.0 || //bool isLeftA
				(trigon[2] - trigon[1]).cross(local - trigon[1]).dot(normal) <= 0.0 || //bool isLeftB
				(trigon[0] - trigon[2]).cross(local - trigon[2]).dot(normal) <= 0.0)   //bool isLeftC
				return DBL_MAX;
			return (local - point).squaredNorm(); //to be fast
		};
	auto _getDistanceOfTwoSegmentsINF = [](const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)->double
		{
			Vector3d vectA = segmA[1] - segmA[0];
			Vector3d vectB = segmB[1] - segmB[0];
			double deltaA = (segmB[0] - segmA[0]).dot(vectA);
			double deltaB = (segmB[0] - segmA[0]).dot(vectB);
			// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
            double deno = vectA.dot(vectB) * vectB.dot(vectA) - vectA.dot(vectA) * vectB.dot(vectB);//a*d-b*c
			if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
                return DBL_MAX;
            double kA = (vectB.dot(vectA) * deltaB - vectB.dot(vectB) * deltaA) / deno;
            double kB = (vectA.dot(vectA) * deltaB - vectA.dot(vectB) * deltaA) / deno;
            //whether two intersect-point inside segments
			if (0.0 < kA && kA < 1.0 && 0.0 < kB && kB < 1.0)
				return (segmA[0] + kA * vectA - segmB[0] - kB * vectB).squaredNorm();
			return DBL_MAX; // nearest point outof segments
		};
	double distance = DBL_MAX;
	std::array<array<Vector3d, 2>, 3> edgesA = { {
		{ triA[0], triA[1] },
		{ triA[1], triA[2] },
		{ triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { {
		{ triB[0], triB[1] },
		{ triB[1], triB[2] },
		{ triB[2], triB[0] } } };
	for (const Vector3d& vertexA : triA)
	{
		//point to point
		for (const Vector3d& vertexB : triB)
		{
			double norm = (vertexA - vertexB).squaredNorm();
			distance = std::min(distance, norm);
		}
		//point to edge
		for (const auto& edgeB : edgesB)
		{
			double temp = _getDistanceOfPointAndSegmentINF(vertexA, edgeB);
			distance = std::min(distance, temp);
		}
		//point to plane
		double temp = _getDistanceOfPointAndPlaneINF(vertexA, triB, normalB);
		distance = std::min(distance, temp);
	}
	for (const Vector3d& vertexB : triB)
	{
		//point to edge
		for (const auto& edgeA : edgesA)
		{
			double temp = _getDistanceOfPointAndSegmentINF(vertexB, edgeA);
			distance = std::min(distance, temp);
		}
		//point to plane
		double temp = _getDistanceOfPointAndPlaneINF(vertexB, triA, normalA);
		distance = std::min(distance, temp);
	}
	//edge to edge
	for (const auto& edgeA : edgesA)
	{
		for (const auto& edgeB : edgesB)
		{
			double temp = _getDistanceOfTwoSegmentsINF(edgeA, edgeB);
			distance = std::min(distance, temp);
		}
	}
	return std::sqrt(distance);
}

//#define EDGE_PAIR_CREATE_AXIS_OPTMIZE
#ifdef EDGE_PAIR_CREATE_AXIS_OPTMIZE
double getTrianglesDistanceSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	double dmin = DBL_MAX, dtemp;
	Vector3d direction, vecSeg, vectA, vectB; // to iterate, get nearest direction
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
			double delta1 = (segmB[0] - segmA[0]).dot(vectA);
			double delta2 = (segmB[0] - segmA[0]).dot(vectB);
			// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
			double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
			if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
				return DBL_MAX;
			double kA = (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2) / deno;
			double kB = (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2) / deno;
			if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
				return (segmA[0] + kA * vectA - segmB[0] - kB * vectB).squaredNorm();
			return DBL_MAX; // nearest point outof segments
		};
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
	std::array<array<Vector3d, 2>, 3> edgesA = { {
		{ triA[0], triA[1] },
		{ triA[1], triA[2] },
		{ triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { {
		{ triB[0], triB[1] },
		{ triB[1], triB[2] },
		{ triB[2], triB[0] } } };
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
}
#endif //REDUCED_AXIS_OPTMIZE

//must separate
std::array<Eigen::Vector3d, 2> eigen::getTwoTrianglesNearestPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
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
			double deno = vectA.dot(vectB) * vectB.dot(vectA) - vectA.dot(vectA) * vectB.dot(vectB);//a*d-b*c
			if (deno == 0.0) // parallel, must exclude, then distance of point to segment in next function
				return DBL_MAX;
			double kA = (-vectB.dot(vectB) * deltaA + vectB.dot(vectA) * deltaB) / deno ;
			double kB = (-vectA.dot(vectB) * deltaA + vectA.dot(vectA) * deltaB) / deno ;
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
				return DBL_MAX; //means projection point out of segment
			//return std::min((point - segm[0]).squaredNorm(), (point - segm[1]).squaredNorm());
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
	return res;
}

//must intersect
std::array<Eigen::Vector3d, 2> eigen::getTwoTrianglesIntersectPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	std::array<Vector3d, 2> res = { gVecNaN , gVecNaN }; // avoid separate
#ifdef STATISTIC_DATA_COUNT
	if (!isTwoTrianglesIntersectSAT(triA, triB))
		count_err_tris_sepa++;
	//	return res;
#endif
	Vector3d vecSeg, normal, local;
	auto _getIntersectOfSegmentAndPlaneINF = [&vecSeg, &normal](const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& plane)->double
		{
			vecSeg = segment[1] - segment[0];
			normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]).normalized(); //avoid over threshold
			if (fabs(vecSeg.dot(normal)) < epsF)
				return (fabs((segment[0] - plane[0]).dot(normal)) < epsF) ? DBL_MAX : -DBL_MAX; // positive infinity means separate
			return (plane[0] - segment[0]).dot(normal) / vecSeg.dot(normal); //k
			//return segment[0] + k * vecSeg; // intersect point
		};
	auto _getPointOfTwoIntersectSegments = [&vecSeg, &normal](const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)->double
		{
			normal = (segmB[0] - segmB[1]).cross(vecSeg);
			if (normal.isZero(epsF)) // intersect cause collinear
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

