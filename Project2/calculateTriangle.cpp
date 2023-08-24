#include "pch.h"
#include "calculateTriangle.h"
using namespace std;
using namespace Eigen;
using namespace psykronix;
#undef max //AlignedBox3d mem-fun
#undef min
#define USING_METHOD_SAT
//#define USING_ACCURATE_NORMALIZED
//#define USING_THRESHOLD_GEOMETRIC // to process collinar and coplanar

//--------------------------------------------------------------------------------------------------
//  triangle
//--------------------------------------------------------------------------------------------------

#ifdef STATISTIC_DATA_COUNT
std::atomic<size_t> count_isTwoTrisInter = 0, count_getTrisDistance = 0, count_isTrisBoundBoxInter = 0, count_isTrisAndBoxInter = 0,
count_pointInTri = 0, 
count_err_inputbox = 0, count_err_inter_dist = 0, count_err_repeat_tri = 0, 
count_err_tris_sepa = 0, count_err_tris_inter = 0;
extern std::atomic<size_t> count_err_degen_tri;
#endif //STATISTIC_DATA_COUNT

enum class RelationOfTrigon : int
{
	SEPARATE = 0,
	INTERSECT, //intersect point local one or two trigon
	COPLANAR_AINB, //A_INSIDE_B
	COPLANAR_BINA, //B_INSIDE_A
	COPLANAR_INTERSECT,
};

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
	return !(axisz * ((p1.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p1.y() - p0.y()) < _eps) ||//bool isLeftA
		axisz * ((p2.x() - p1.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (p2.y() - p1.y()) < _eps) || //bool isLeftB
		axisz * ((p0.x() - p2.x()) * (point.y() - p2.y()) - (point.x() - p2.x()) * (p0.y() - p2.y()) < _eps)); //bool isLeftC
#else
	double axisz = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
	return
		0.0 <= axisz * ((p1.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p1.y() - p0.y())) &&
		0.0 <= axisz * ((p2.x() - p1.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (p2.y() - p1.y())) &&
		0.0 <= axisz * ((p0.x() - p2.x()) * (point.y() - p2.y()) - (point.x() - p2.x()) * (p0.y() - p2.y()));
#endif
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
	return !((trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) < _eps || //bool isNotLeftA
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) < _eps || //bool isNotLeftB
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal) < _eps); //bool isNotLeftC
#else
	return
		0.0 <= (trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) && //bool isLeftA
		0.0 <= (trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) && //bool isLeftB
		0.0 <= (trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal);	 //bool isLeftC
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

bool psykronix::isPointOnTriangleSurface(const Vector3d& point, const std::array<Vector3d, 3>& trigon)
{
	if (point.x() < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
		point.x() > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
		point.y() < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
		point.y() > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
		point.z() < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
		point.z() > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]))
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
	return !((segmB[0] - segmA[0]).cross(segmA[1] - segmA[0]).dot((segmA[1] - segmA[0]).cross(segmB[1] - segmA[0])) < _eps || //double separate
		(segmA[0] - segmB[0]).cross(segmB[1] - segmB[0]).dot((segmB[1] - segmB[0]).cross(segmA[1] - segmB[0])) < _eps);
#else
	return 0.0 <= (segmB[0] - segmA[0]).cross(segmA[1] - segmA[0]).dot((segmA[1] - segmA[0]).cross(segmB[1] - segmA[0])) &&
		0.0 <= (segmA[0] - segmB[0]).cross(segmB[1] - segmB[0]).dot((segmB[1] - segmB[0]).cross(segmA[1] - segmB[0]));
#endif
}

//must coplanar
bool psykronix::isSegmentCrossTriangle(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) // must coplanar
{
	//judge box
	//Eigen::AlignedBox3d boxSeg(segment[0], segment[1]);
	//Eigen::AlignedBox3d boxTri(Vector3d(
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
	if (isTwoSegmentsIntersect(segment, { trigon[0], trigon[1] }))
		return true;
	if (isTwoSegmentsIntersect(segment, { trigon[1], trigon[2] }))
		return true;
	if (isTwoSegmentsIntersect(segment, { trigon[2], trigon[0] }))
		return true;
	return false;
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
	return true;
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

// decompose box to twelve triangles
bool psykronix::isTriangleAndBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box)
{
#ifdef STATISTIC_DATA_COUNT
	count_isTrisAndBoxInter++;
	if ((box.sizes().array() < 0.0).any()) // input box illegal
		count_err_inputbox++; // nothing error
#endif
	const Vector3d& p0 = trigon[0];
	const Vector3d& p1 = trigon[1];
	const Vector3d& p2 = trigon[2];
	if (box.contains(p0) || box.contains(p1) || box.contains(p2))
		return true;
	const Vector3d& min = box.min();
	const Vector3d& max = box.max();
	if ((p0.x() < min.x() && p1.x() < min.x() && p2.x() < min.x()) ||
		(p0.x() > max.x() && p1.x() > max.x() && p2.x() > max.x()) ||
		(p0.y() < min.y() && p1.y() < min.y() && p2.y() < min.y()) ||
		(p0.y() > max.y() && p1.y() > max.y() && p2.y() > max.y()) ||
		(p0.z() < min.z() && p1.z() < min.z() && p2.z() < min.z()) ||
		(p0.z() > max.z() && p1.z() > max.z() && p2.z() > max.z()))
		return false;
	Vector3d vertex = max - min;
	std::array<std::array<Vector3d, 3>, 12> triTwelve = { {
	{ Vector3d(0, 0, 0), Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), vertex.y(), 0) },
	{ Vector3d(0, 0, 0), Vector3d(0, vertex.y(), 0), Vector3d(vertex.x(), vertex.y(), 0) },
	{ Vector3d(0, 0, vertex.z()), Vector3d(vertex.x(), 0, vertex.z()), vertex },
	{ Vector3d(0, 0, vertex.z()), Vector3d(0, vertex.y(), vertex.z()), vertex },
	{ Vector3d(0, 0, 0), Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), 0, vertex.z()) },
	{ Vector3d(0, 0, 0), Vector3d(0, 0, vertex.z()), Vector3d(vertex.x(), 0, vertex.z()) },
	{ Vector3d(0, vertex.y(), 0), Vector3d(vertex.x(), vertex.y(), 0), vertex },
	{ Vector3d(0, vertex.y(), 0), Vector3d(0, vertex.y(), vertex.z()), vertex },
	{ Vector3d(0, 0, 0), Vector3d(0, vertex.y(), 0), Vector3d(0, vertex.y(), vertex.z()) },
	{ Vector3d(0, 0, 0), Vector3d(0, 0, vertex.z()), Vector3d(0, vertex.y(), vertex.z()) },
	{ Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), vertex.y(), 0), vertex },
	{ Vector3d(vertex.x(), 0, 0), Vector3d(vertex.x(), 0, vertex.z()), vertex } } };
	for (const auto& iter : triTwelve)
	{
		if (isTwoTrianglesIntersectSAT({ p0 - min, p1 - min, p2 - min }, iter))
			return true;
	}
	return false;
}

std::tuple<Vector3d, double> psykronix::getTriangleBoundingCircle(const std::array<Vector3d, 3>& trigon) //return center and radius
{
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

bool psykronix::isSegmentAndTriangleIntersctSAT(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon)
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
bool psykronix::isPointRayAcrossTriangleSAT(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon)
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
	std::array<Eigen::Vector3d, 3> edges = { trigon[1] - trigon[0],
										   trigon[2] - trigon[1],
										   trigon[0] - trigon[2] };
	Vector3d normal = edges[0].cross(edges[1]);
	std::array<Eigen::Vector3d, 7> axes = { { //order matters speed
			//axisZ, // using pre-box
			axisZ.cross(edges[0]),
			axisZ.cross(edges[1]),
			axisZ.cross(edges[2]),
			normal,
			normal.cross(edges[0]),
			normal.cross(edges[1]),
			normal.cross(edges[2]) } };//add
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
		double minRay = std::min(axis.dot(point), axis.dot(rayPt));
		double maxRay = std::max(axis.dot(point), axis.dot(rayPt));
		if (minTri + eps < minRay || maxRay + eps < minTri)
#else
		if (maxTri < std::min(dotP, dotR) || std::max(dotP, dotR) < minTri) // absolute zero
#endif
			return false;
	}
	return true;
	// another method, judge 2D point in triangle, point under plane
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
	if (std::max(std::max(triB[0].x(), triB[1].x()), triB[2].x()) < std::min(std::min(triA[0].x(), triA[1].x()), triA[2].x()) - tolerance)
		return false;
	if (std::max(std::max(triA[0].x(), triA[1].x()), triA[2].x()) + tolerance < std::min(std::min(triB[0].x(), triB[1].x()), triB[2].x()))
		return false;
	if (std::max(std::max(triB[0].y(), triB[1].y()), triB[2].y()) < std::min(std::min(triA[0].y(), triA[1].y()), triA[2].y()) - tolerance)
		return false;
	if (std::max(std::max(triA[0].y(), triA[1].y()), triA[2].y()) + tolerance < std::min(std::min(triB[0].y(), triB[1].y()), triB[2].y()))
		return false;
	if (std::max(std::max(triB[0].z(), triB[1].z()), triB[2].z()) < std::min(std::min(triA[0].z(), triA[1].z()), triA[2].z()) - tolerance)
		return false;
	if (std::max(std::max(triA[0].z(), triA[1].z()), triA[2].z()) + tolerance < std::min(std::min(triB[0].z(), triB[1].z()), triB[2].z()))
		return false;
	return true;
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
	std::array<Eigen::Vector3d, 3> edges = { trigon[1] - trigon[0],
										   trigon[2] - trigon[1],
										   trigon[0] - trigon[2] };
	std::array<Eigen::Vector3d, 3> coords = { Vector3d(1,0,0),
											   Vector3d(0,1,0),
											   Vector3d(0,0,1) };
	std::array<Eigen::Vector3d, 13> axes = { {
			coords[0],
			coords[1],
			coords[2],
			edges[0].cross(edges[1]), //normal
			//normal.cross(edges[]),
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
		for (const auto& vertex : trigon)
		{
			projection = (vertex - origin).dot(axis);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : vertexes)
		{
			projection = vertex.dot(axis);
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
	std::array<Eigen::Vector3d, 3> edgesA = { triA[1] - triA[0],
										   triA[2] - triA[1],
										   triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = { triB[1] - triB[0],
											triB[2] - triB[1],
											triB[0] - triB[2] };
#ifdef STATISTIC_DATA_COUNT
	count_isTwoTrisInter++;
	if (triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2])
		count_err_repeat_tri++;
	if (edgesA[0].cross(edgesA[1]).isZero() || edgesB[0].cross(edgesB[1]).isZero())
		count_err_degen_tri++;
#endif
	Vector3d normalA = edgesA[0].cross(edgesA[1]);
	Vector3d normalB = edgesB[0].cross(edgesB[1]);
	std::array<Eigen::Vector3d, 17> axes = { { //order matters speed
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
		if (maxA + eps < minB || maxB + eps < minA)
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
	std::array<array<Vector3d, 2>, 3> edgesA = { { {triA[0], triA[1]},
												{triA[1], triA[2]},
												{triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { { {triB[0], triB[1]},
												{ triB[1], triB[2] },
												{ triB[2], triB[0] } } };
	auto _getDistanceOfPointAndSegmentINF = [&](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double
	{
		vecSeg = segm[1] - segm[0];// not zero
		double projection = vecSeg.dot(point);
		//the projection must on segment
		if (vecSeg.dot(segm[1]) < projection || projection < vecSeg.dot(segm[0]))
			return DBL_MAX;
		double k = vecSeg.dot(point - segm[0]) / vecSeg.dot(vecSeg);
		return (segm[0] - point + k * vecSeg).squaredNorm();
	};
	auto _getDistanceOfTwoSegmentsINF = [&](const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)->double
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
#define EDGE_PAIR_CREATE_AXIS_OPTMIZE
#ifdef EDGE_PAIR_CREATE_AXIS_OPTMIZE
	auto _getNearestAxisOfTwoSegments = [&](const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB)// major capture axis
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
		direction.normalized(),
		(triA[1] - triA[0]).cross(triA[2] - triA[1]).normalized(),
		(triB[1] - triB[0]).cross(triB[2] - triB[1]).normalized() } };
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
	double minA = DBL_MAX, minB = DBL_MAX, maxA = -DBL_MAX, maxB = -DBL_MAX;
	for (const auto& vertex : triA) //fast than list
	{
		double projection = direction.dot(vertex);
		minA = std::min(minA, projection);
		maxA = std::max(maxA, projection);
	}
	for (const auto& vertex : triB)
	{
		double projection = direction.dot(vertex);
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
	//	return res;
#endif
	double dmin = DBL_MAX, dtemp;
	Eigen::Vector3d local, local2;
	auto _getDistanceOfPointAndPlaneINF = [&](const Vector3d& point, const std::array<Vector3d, 3>& plane)->double
	{
		Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
		//if (normal.isZero()) // error triangle plane
		//	return DBL_MAX;
		double k = (plane[0] - point).dot(normal) / normal.dot(normal);
		local = point + k * normal; // reference closure
		if (!isPointInTriangle(local, plane))
			return DBL_MAX;
		return (k * normal).squaredNorm();
	};
	auto _getDistanceOfTwoSegmentsINF = [&](const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)->double
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
	auto _getDistanceOfPointAndSegmentINF = [&](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double
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
	std::array<array<Vector3d, 2>, 3> edgesA = { { {triA[0], triA[1]},
												{triA[1], triA[2]},
												{triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { { {triB[0], triB[1]},
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
	auto _getIntersectOfSegmentAndPlaneINF = [&](const std::array<Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& plane)->double
	{
		vecSeg = segment[1] - segment[0];
#ifdef USING_ACCURATE_NORMALIZED
		normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]).normalized(); // plane's normal isnot Zero
		if (fabs(vecSeg.normalized().dot(normal)) < eps)// line parallel to plane, not always coplanar
			return (fabs((segment[0] - plane[0]).normalized().dot(normal)) < eps) ? DBL_MAX : -DBL_MAX; // positive infinity means separate
#else
		normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);// normalized();
		if (fabs(vecSeg.dot(normal)) < eps)
			return (fabs((segment[0] - plane[0]).dot(normal)) < eps) ? DBL_MAX : -DBL_MAX;
#endif // USING_ACCURATE_NORMALIZED
		return (plane[0] - segment[0]).dot(normal) / vecSeg.dot(normal); //k
		//return segment[0] + k * vecSeg; // intersect point
	};
	auto _getPointOfTwoIntersectSegments = [&](const std::array<Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB)->double
	{
		normal = (segmB[0] - segmB[1]).cross(vecSeg);
		if (normal.isZero(eps)) // intersect cause collinear
			return DBL_MAX;
		return (segmA[0] - segmB[0]).cross(segmA[0] - segmB[1]).norm() / normal.norm(); //k
		//return segmA[0] + k * vecSeg;
	};
	auto _getEndPointsOfTwoCollinearSegments = [&](const std::array<Vector3d, 2>& edgeA, const std::array<Eigen::Vector3d, 2>& edgeB)
	{
		if ((edgeA[0] - edgeB[0]).dot(edgeA[0] - edgeB[1]) <= 0 && (edgeA[1] - edgeB[0]).dot(edgeA[1] - edgeB[1]) <= 0)
			res = edgeA;
		else if ((edgeB[0] - edgeA[0]).dot(edgeB[0] - edgeA[1]) <= 0 && (edgeB[1] - edgeA[0]).dot(edgeB[1] - edgeA[1]) <= 0)
			res = edgeB;
		else
			res = { ((edgeA[0] - edgeB[0]).dot(edgeA[0] - edgeB[1])) <= 0.0 ? edgeA[0] : edgeA[1],
					((edgeB[0] - edgeA[0]).dot(edgeB[0] - edgeA[1])) <= 0.0 ? edgeB[0] : edgeB[1] };
	};
	std::array<array<Vector3d, 2>, 3> edgesA = { { {triA[0], triA[1]},
												{triA[1], triA[2]},
												{triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { { {triB[0], triB[1]},
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


