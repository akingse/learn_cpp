#include "pch.h"
#include "calculateTriangle.h"
using namespace std;
using namespace Eigen;
using namespace psykronix;
#undef max //AlignedBox3d mem-fun
#undef min

//--------------------------------------------------------------------------------------------------
//  triangle
//--------------------------------------------------------------------------------------------------
static constexpr double eps = 1e-6; //DBL_EPSILON
static constexpr double _eps = -eps;
std::atomic<size_t> getTriangleBoundC = 0, TriangularIntersectC = 0, getTriDistC = 0, isTriangleBoundC = 0,
count_pointInTri = 0, count_edgeCrossTri = 0, count_segCrossTri = 0, count_across = 0,
isTwoTrianglesC = 0, TriangularIntersectC_in = 0, TriangularIntersectC_all = 0,
count_err_inputbox = 0, count_err_inter_dist = 0, count_err_repeat_tri = 0; // count_err_degen_tri = 0


//_isPointInTriangle2D
bool psykronix::isPointInTriangle(const Vector2d& point, const std::array<Vector2d, 3>& trigon) // 2D
{
	// using isLeft test
	const Vector2d& p0 = trigon[0];
	const Vector2d& p1 = trigon[1];
	const Vector2d& p2 = trigon[2];
	// compare z-value
	return !(((p1.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p1.y() - p0.y()) < _eps) ||//bool isLeftA
		((p2.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p2.y() - p0.y()) > eps) || //bool isLeftB
		((p2.x() - p1.x()) * (point.y() - p1.y()) - (point.x() - p1.x()) * (p2.y() - p1.y()) < _eps)); //bool isLeftC
}

//_isPointInTriangle3D
bool psykronix::isPointInTriangle(const Vector3d& point, const std::array<Vector3d, 3>& trigon) //must coplanar
{
#ifdef STATISTIC_DATA_COUNT
	count_pointInTri++;
#endif
	// 3D triangle, input point must coplanar
	Vector3d vecZ = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[0]);// using isLeft test
	return !((trigon[1] - trigon[0]).cross(point - trigon[0]).dot(vecZ) < _eps || //bool isNotLeftA
		(trigon[2] - trigon[1]).cross(point - trigon[1]).dot(vecZ) < _eps || //bool isNotLeftB
		(trigon[0] - trigon[2]).cross(point - trigon[2]).dot(vecZ) < _eps); //bool isNotLeftC
	//+-*x< 0,5,1.5,2.5,1.5
	//if (((trigon[1] - trigon[0]).cross(point - trigon[0])).dot(vecZ) < _eps) //bool isLeftA
	//	return false;
	//if (((trigon[2] - trigon[0]).cross(point - trigon[0])).dot(vecZ) > eps) //bool isLeftB
	//	return false;
	//if (((trigon[2] - trigon[1]).cross(point - trigon[1])).dot(vecZ) < _eps) //bool isLeftC
	//	return false;
	//return true;
}

//double straddling test
bool psykronix::isTwoSegmentsIntersect(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB, bool preBox /*= false*/)
{
	if (preBox)// quick reject box
	{
		if (std::max(segmB[0].x(), segmB[0].x()) < std::min(segmB[1].x(), segmB[1].x()) ||
			std::min(segmB[0].x(), segmB[0].x()) > std::max(segmB[1].x(), segmB[1].x()) ||
			std::max(segmB[0].y(), segmB[0].y()) < std::min(segmB[1].y(), segmB[1].y()) ||
			std::min(segmB[0].y(), segmB[0].y()) > std::max(segmB[1].y(), segmB[1].y()) ||
			std::max(segmB[0].z(), segmB[0].z()) < std::min(segmB[1].z(), segmB[1].z()) ||
			std::min(segmB[0].z(), segmB[0].z()) > std::max(segmB[1].z(), segmB[1].z()))
			return false;
	}
	return !((segmB[0] - segmA[0]).cross(segmA[1] - segmA[0]).dot((segmA[1] - segmA[0]).cross(segmB[1] - segmA[0])) < _eps || //double separate
		(segmA[0] - segmB[0]).cross(segmB[1] - segmB[0]).dot((segmB[1] - segmB[0]).cross(segmA[1] - segmB[0])) < _eps);
}

bool psykronix::isEdgeCrossTriangle(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) // must coplanar
{
#ifdef STATISTIC_DATA_COUNT
	count_edgeCrossTri++;
#endif
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
	Vector3d vecX = (segment[1] - segment[0]).cross((trigon[1] - trigon[0]).cross(trigon[2] - trigon[0])); // axisY cross axisZ
	double dot0 = vecX.dot(trigon[0] - segment[0]);
	double dot1 = vecX.dot(trigon[1] - segment[0]);
	double dot2 = vecX.dot(trigon[2] - segment[0]);
	if ((dot0 > eps && dot1 > eps && dot2 > eps) || (dot0 < _eps && dot1 < _eps && dot2 < _eps))// triangle-point is all left
		return false; //+-*x< 0,6,3,2,4.5
	// judge segment point
	if (isPointInTriangle(segment[0], trigon) || isPointInTriangle(segment[1], trigon))
		return true;
	// double straddling test x3
	Vector3d vecSeg = segment[1] - segment[0];
	if (!(std::min(segment[0].x(), segment[1].x()) > std::max(trigon[0].x(), trigon[1].x()) || std::max(segment[0].x(), segment[1].x()) < std::min(trigon[0].x(), trigon[1].x()) ||
		std::min(segment[0].y(), segment[1].y()) > std::max(trigon[0].y(), trigon[1].y()) || std::max(segment[0].y(), segment[1].y()) < std::min(trigon[0].y(), trigon[1].y()) ||
		std::min(segment[0].z(), segment[1].z()) > std::max(trigon[0].z(), trigon[1].z()) || std::max(segment[0].z(), segment[1].z()) < std::min(trigon[0].z(), trigon[1].z())))
	{
		if (!((trigon[0] - segment[0]).cross(vecSeg).dot(vecSeg.cross(trigon[1] - segment[0])) < _eps ||
			(segment[0] - trigon[0]).cross(trigon[1] - trigon[0]).dot((trigon[1] - trigon[0]).cross(segment[1] - trigon[0])) < _eps))
			return true;
	}
	if (!(std::min(segment[0].x(), segment[1].x()) > std::max(trigon[1].x(), trigon[2].x()) || std::max(segment[0].x(), segment[1].x()) < std::min(trigon[1].x(), trigon[2].x()) ||
		std::min(segment[0].y(), segment[1].y()) > std::max(trigon[1].y(), trigon[2].y()) || std::max(segment[0].y(), segment[1].y()) < std::min(trigon[1].y(), trigon[2].y()) ||
		std::min(segment[0].z(), segment[1].z()) > std::max(trigon[1].z(), trigon[2].z()) || std::max(segment[0].z(), segment[1].z()) < std::min(trigon[1].z(), trigon[2].z())))
	{
		if (!((trigon[1] - segment[0]).cross(vecSeg).dot(vecSeg.cross(trigon[2] - segment[0])) < _eps ||
			(segment[0] - trigon[1]).cross(trigon[2] - trigon[1]).dot((trigon[2] - trigon[1]).cross(segment[1] - trigon[1])) < _eps))
			return true;
	}
	if (!(std::min(segment[0].x(), segment[1].x()) > std::max(trigon[2].x(), trigon[0].x()) || std::max(segment[0].x(), segment[1].x()) < std::min(trigon[2].x(), trigon[0].x()) ||
		std::min(segment[0].y(), segment[1].y()) > std::max(trigon[2].y(), trigon[0].y()) || std::max(segment[0].y(), segment[1].y()) < std::min(trigon[2].y(), trigon[0].y()) ||
		std::min(segment[0].z(), segment[1].z()) > std::max(trigon[2].z(), trigon[0].z()) || std::max(segment[0].z(), segment[1].z()) < std::min(trigon[2].z(), trigon[0].z())))
	{
		if (!((trigon[2] - segment[0]).cross(vecSeg).dot(vecSeg.cross(trigon[0] - segment[0])) < _eps ||
			(segment[0] - trigon[2]).cross(trigon[0] - trigon[2]).dot((trigon[0] - trigon[2]).cross(segment[1] - trigon[2])) < _eps))
			return true;
	}
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

// must through plane
bool psykronix::isSegmentCrossTriangleSurface(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) //start point outof plane
{
#ifdef STATISTIC_DATA_COUNT
	count_segCrossTri++;
#endif

	// compare angle of normal-vector
	Vector3d vecSeg = segment[1] - segment[0];
	double dotA = (trigon[0] - segment[0]).cross(trigon[1] - segment[0]).dot(vecSeg);
	double dotB = (trigon[1] - segment[0]).cross(trigon[2] - segment[0]).dot(vecSeg);
	double dotC = (trigon[2] - segment[0]).cross(trigon[0] - segment[0]).dot(vecSeg);
	return (dotA < eps && dotB < eps && dotC < eps) || (dotA > _eps && dotB > _eps && dotC > _eps);
	// condition exclude
	//bool isLeft = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[0]).dot(segment[0] - trigon[0]) > eps;
	//Vector3d vecSeg = segment[1] - segment[0];
	//if (isLeft)
	//{
	//    if (vecSeg.dot((trigon[1] - segment[0]).cross(trigon[0] - segment[0])) < _eps)
	//        return false;
	//    if (vecSeg.dot((trigon[2] - segment[0]).cross(trigon[1] - segment[0])) < _eps)
	//        return false;
	//    if (vecSeg.dot((trigon[0] - segment[0]).cross(trigon[2] - segment[0])) < _eps)
	//        return false;
	//    return true;
	//}
	//else
	//{
	//    if (vecSeg.dot((trigon[1] - segment[0]).cross(trigon[0] - segment[0])) > eps)
	//        return false;
	//    if (vecSeg.dot((trigon[2] - segment[0]).cross(trigon[1] - segment[0])) > eps)
	//        return false;
	//    if (vecSeg.dot((trigon[0] - segment[0]).cross(trigon[2] - segment[0])) > eps)
	//        return false;
	//    return true;
	//}
	//bool isLeftSa = vec.dot((trigon[0] - segment[0]).cross(trigon[1] - segment[0])) < eps;
	//bool isLeftSb = vec.dot((trigon[1] - segment[0]).cross(trigon[2] - segment[0])) < eps;
	//bool isLeftSc = vec.dot((trigon[2] - segment[0]).cross(trigon[0] - segment[0])) < eps;
	//return (isLeftSa && isLeftSb && isLeftSc) || (!isLeftSa && !isLeftSb && !isLeftSc);
	//bool isLeftEa = (segment[0] - segment[1]).dot((trigon[0] - segment[1]).cross(trigon[1] - segment[1])) < eps;
	//bool isLeftEb = (segment[0] - segment[1]).dot((trigon[1] - segment[1]).cross(trigon[2] - segment[1])) < eps;
	//bool isLeftEc = (segment[0] - segment[1]).dot((trigon[2] - segment[1]).cross(trigon[0] - segment[1])) < eps;
	//if ((isLeftEa && isLeftEb && isLeftEc) || (!isLeftEa && !isLeftEb && !isLeftEc))
	//    return true;
	//return false;
}

// Method: every intersect point in triangle
bool psykronix::isTwoTrianglesIntersection(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{
	//#ifdef STATISTIC_DATA_COUNT
	//	TriangularIntersectC++;
	//#endif
		// pre box check
		//if (!isTwoTrianglesBoundingBoxIntersect(triL, triR))
		//	return false;
		// right edge through left plane
	Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]);
	bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point on plane(dot==0)
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;
	// left edge through right plane
	Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[0]);
	bool acrossL2R_A = (veczR.dot(triL[0] - triR[0])) * (veczR.dot(triL[1] - triR[0])) < eps;
	bool acrossL2R_B = (veczR.dot(triL[1] - triR[0])) * (veczR.dot(triL[2] - triR[0])) < eps;
	bool acrossL2R_C = (veczR.dot(triL[2] - triR[0])) * (veczR.dot(triL[0] - triR[0])) < eps;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;

#ifdef STATISTIC_DATA_COUNT
	TriangularIntersectC_in++;
#endif
	// forward
	double dotPro;// = 0;
	if (acrossR2L_A) // R0 
	{
		//edgeA={triR[0], triR[1]}
		dotPro = veczL.dot(triR[0] - triR[1]); //dotA
		if (fabs(dotPro) < eps) // perpendi to veczL (veczL dot edgeA is zero)
		{
			if (isEdgeCrossTriangle({ triR[0], triR[1] }, triL)) //segment on plane
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
			if (isEdgeCrossTriangle({ triR[1], triR[2] }, triL)) //segment on plane
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
			if (isEdgeCrossTriangle({ triR[2], triR[0] }, triL)) //segment on plane
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
			if (isEdgeCrossTriangle({ triL[0], triL[1] }, triR)) //segment on plane
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
			if (isEdgeCrossTriangle({ triL[1], triL[2] }, triR)) //segment on plane
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
			if (isEdgeCrossTriangle({ triL[2], triL[0] }, triR)) //segment on plane
				return true;
		}
		else
		{
			Vector3d point = triL[2] + (veczR.dot(triL[2] - triR[0]) / dotPro) * (triL[0] - triL[2]);
			if (isPointInTriangle(point, triR))
				return true;
		}
	}
#ifdef STATISTIC_DATA_COUNT
	TriangularIntersectC_all++;
#endif
	return false;
}

// Method: edge in tetra-hedron
bool psykronix::isTwoTrianglesIntersect(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{
	Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]); // triL close face triangle
	bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point-on-plane
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;
	Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[0]); // triR close face triangle
	bool acrossL2R_A = (veczR.dot(triL[0] - triR[0])) * (veczR.dot(triL[1] - triR[0])) < eps; //include point-on-plane
	bool acrossL2R_B = (veczR.dot(triL[1] - triR[0])) * (veczR.dot(triL[2] - triR[0])) < eps;
	bool acrossL2R_C = (veczR.dot(triL[2] - triL[0])) * (veczR.dot(triL[0] - triL[0])) < eps;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;
	return true;
	// using face to-left test
	bool pointOnfaceS = false;
	bool pointOnfaceE = false;
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
			if (isEdgeCrossTriangle({ triR[0], triR[1] }, triL)) //segment on plane
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
			if (isEdgeCrossTriangle({ triR[1], triR[2] }, triL)) //segment on plane
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
			if (isEdgeCrossTriangle({ triR[2], triR[0] }, triL)) //segment on plane
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
			if (isEdgeCrossTriangle({ triL[0], triL[1] }, triR)) //segment on plane
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
			if (isEdgeCrossTriangle({ triL[1], triL[2] }, triR)) //segment on plane
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
			if (isEdgeCrossTriangle({ triL[2], triL[0] }, triR)) //segment on plane
				return true;
		}
	}
	return false;
}

//bool psykronix::isTwoTrianglesIntersection1(const std::array<Eigen::Vector3f, 3>& triL, const std::array<Eigen::Vector3f, 3>& triR)
//{
//    std::array<Vector3d, 3>  triA = { triL[0].cast<double>(), triL[1].cast<double>(), triL[2].cast<double>() };
//    std::array<Vector3d, 3>  triB = { triR[0].cast<double>(), triR[1].cast<double>(), triR[2].cast<double>() };
//    return isTwoTrianglesIntersection1(triA, triB);
//}

bool psykronix::isTriangleAndBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box)
{
#ifdef STATISTIC_DATA_COUNT
	isTriangleBoundC++;
#endif
	//	if ((box.sizes().array() < 0).any()) // input box illegal
	//	{
	//#ifdef STATISTIC_DATA_COUNT
	//		count_err_inputbox++; // no-one error
	//#endif
	//		return true;
	//	}
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

bool psykronix::isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	//get min and max of two trigons
	const double& xminA = std::min(std::min(triA[0].x(), triA[1].x()), triA[2].x()); // a little fast
	const double& xmaxB = std::max(std::max(triB[0].x(), triB[1].x()), triB[2].x());
	if (xmaxB < xminA)
		return false;
	const double& xmaxA = std::max(std::max(triA[0].x(), triA[1].x()), triA[2].x());
	const double& xminB = std::min(std::min(triB[0].x(), triB[1].x()), triB[2].x());
	if (xmaxA < xminB)
		return false;
	const double& yminA = std::min(std::min(triA[0].y(), triA[1].y()), triA[2].y());
	const double& ymaxB = std::max(std::max(triB[0].y(), triB[1].y()), triB[2].y());
	if (ymaxB < yminA)
		return false;
	const double& ymaxA = std::max(std::max(triA[0].y(), triA[1].y()), triA[2].y());
	const double& yminB = std::min(std::min(triB[0].y(), triB[1].y()), triB[2].y());
	if (ymaxA < yminB)
		return false;
	const double& zminA = std::min(std::min(triA[0].z(), triA[1].z()), triA[2].z());
	const double& zmaxB = std::max(std::max(triB[0].z(), triB[1].z()), triB[2].z());
	if (zmaxB < zminA)
		return false;
	const double& zmaxA = std::max(std::max(triA[0].z(), triA[1].z()), triA[2].z());
	const double& zminB = std::min(std::min(triB[0].z(), triB[1].z()), triB[2].z());
	if (zmaxA < zminB)
		return false;
	return true;
	//return !(xminA > xmaxB || yminA > ymaxB || zminA > zmaxB || xminB > xmaxA || yminB > ymaxA || zminB > zmaxA);
}

bool psykronix::isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance)
{
	//get min and max of two trigons
	const double& xminA = std::min(std::min(triA[0].x(), triA[1].x()), triA[2].x()) - tolerance; // a little fast
	const double& xmaxB = std::max(std::max(triB[0].x(), triB[1].x()), triB[2].x());
	if (xmaxB < xminA)
		return false;
	const double& xmaxA = std::max(std::max(triA[0].x(), triA[1].x()), triA[2].x()) + tolerance;
	const double& xminB = std::min(std::min(triB[0].x(), triB[1].x()), triB[2].x());
	if (xmaxA < xminB)
		return false;
	const double& yminA = std::min(std::min(triA[0].y(), triA[1].y()), triA[2].y()) - tolerance;
	const double& ymaxB = std::max(std::max(triB[0].y(), triB[1].y()), triB[2].y());
	if (ymaxB < yminA)
		return false;
	const double& ymaxA = std::max(std::max(triA[0].y(), triA[1].y()), triA[2].y()) + tolerance;
	const double& yminB = std::min(std::min(triB[0].y(), triB[1].y()), triB[2].y());
	if (ymaxA < yminB)
		return false;
	const double& zminA = std::min(std::min(triA[0].z(), triA[1].z()), triA[2].z()) - tolerance;
	const double& zmaxB = std::max(std::max(triB[0].z(), triB[1].z()), triB[2].z());
	if (zmaxB < zminA)
		return false;
	const double& zmaxA = std::max(std::max(triA[0].z(), triA[1].z()), triA[2].z()) + tolerance;
	const double& zminB = std::min(std::min(triB[0].z(), triB[1].z()), triB[2].z());
	if (zmaxA < zminB)
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

std::tuple<Vector3d, double> psykronix::getTriangleBoundingCircle(const std::array<Vector3d, 3>& trigon) //return center and radius
{
	//getTriangleBoundC++;
	Vector3d vecA = trigon[1] - trigon[0];
	Vector3d vecB = trigon[2] - trigon[0];
	Vector3d vecC = trigon[2] - trigon[1];
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
	Vector3d vecZ = vecA.cross(vecB);
	//if (vecZ.squaredNorm() > eps) //trigon legal
	const Vector3d& p0 = trigon[0];
	const Vector3d& p1 = trigon[1];
	const Vector3d& p2 = trigon[2];
	//mat = set_matrix_by_row_vectors(vecA, vecB, vecZ)
	Eigen::Matrix3d mat;
	mat.row(0) = vecA;
	mat.row(1) = vecB;
	mat.row(2) = vecZ;
	Vector3d p = mat.inverse() * Vector3d(
		0.5 * (p1.x() * p1.x() - p0.x() * p0.x() + p1.y() * p1.y() - p0.y() * p0.y() + p1.z() * p1.z() - p0.z() * p0.z()),
		0.5 * (p2.x() * p2.x() - p0.x() * p0.x() + p2.y() * p2.y() - p0.y() * p0.y() + p2.z() * p2.z() - p0.z() * p0.z()),
		vecZ.dot(p0));
	//if (psykronix::_isPointInTriangle(p, trigon)) //is acute angle
	return { p, (p - p0).norm() };
}

//bool psykronix::TriangularIntersectionTest(const std::array<Eigen::Vector3f, 3>& T1, const std::array<Eigen::Vector3f, 3>& T2)
//{
//    std::array<Vector3d, 3>  triA = { T1[0].cast<double>(), T1[1].cast<double>(), T1[2].cast<double>()};
//    std::array<Vector3d, 3>  triB = { T2[0].cast<double>(), T2[1].cast<double>(), T2[2].cast<double>()};
//    return TriangularIntersectionTest(triA, triB);
//}

//---------------------------------------------------------------------------
// soft clash
//---------------------------------------------------------------------------
//DLLEXPORT void _getSegPoints(Eigen::Vector3d& VEC, Eigen::Vector3d& X, Eigen::Vector3d& Y, const std::array<Eigen::Vector3d, 2>& posvecA, const std::array<Eigen::Vector3d, 2>& posvecB);
void psykronix::getSegmentsPoints(Eigen::Vector3d& VEC, Eigen::Vector3d& X, Eigen::Vector3d& Y,
	const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& Q, const Eigen::Vector3d& B)
{
	Eigen::Vector3d TMP;
	double A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
	Eigen::Vector3d T = Q - P;
	A_dot_A = A.dot(A);
	B_dot_B = B.dot(B);
	A_dot_B = A.dot(B);
	A_dot_T = A.dot(T);
	B_dot_T = B.dot(T);
	double denom = A_dot_A * B_dot_B - A_dot_B * A_dot_B;
	double t = (A_dot_T * B_dot_B - B_dot_T * A_dot_B) / denom;
	if ((t < 0) || isnan(t))
		t = 0;
	else if (t > 1)
		t = 1;
	double u = (t * A_dot_B - B_dot_T) / B_dot_B;
	if ((u <= 0) || isnan(u))
	{
		Y = Q;
		t = A_dot_T / A_dot_A;
		if ((t <= 0) || isnan(t))
		{
			X = P;
			VEC = Q - P;
		}
		else if (t >= 1)
		{
			X = P + A;
			VEC = Q - X;
		}
		else
		{
			X = P + t * A;
			TMP = T.cross(A);
			VEC = A.cross(TMP);
		}
	}
	else if (u >= 1)
	{
		Y = Q + B;
		t = (A_dot_B + A_dot_T) / A_dot_A;
		if ((t <= 0) || isnan(t))
		{
			X = P;
			VEC = Y - P;
		}
		else if (t >= 1)
		{
			X = P + A;
			VEC = Y - X;
		}
		else
		{
			X = P + t * A;
			T = Y - P;
			TMP = T.cross(A);
			VEC = A.cross(TMP);
		}
	}
	else
	{
		Y = Q + u * B;
		if ((t <= 0) || isnan(t))
		{
			X = P;
			TMP = T.cross(B);
			VEC = B.cross(TMP);
		}
		else if (t >= 1)
		{
			X = P + A;
			T = Q - X;
			TMP = T.cross(B);
			VEC = B.cross(TMP);
		}
		else
		{
			X = P + t * A;
			VEC = A.cross(B);
			if (VEC.dot(T) < 0)
				VEC = -1.0 * VEC;
		}
	}
}

//#define USING_INNER_PRE_JUDGE
double psykronix::getTrianglesDistance(Eigen::Vector3d& P, Eigen::Vector3d& Q, const std::array<Eigen::Vector3d, 3>& S, const std::array<Eigen::Vector3d, 3>& T)
{
#ifdef STATISTIC_DATA_COUNT
	getTriDistC++;
#endif
#ifdef USING_INNER_PRE_JUDGE
	if (isTwoTrianglesIntersectSAT(S, T)) // pre-judge intersect
	{
#ifdef STATISTIC_DATA_COUNT
		count_err_inter_dist++;
#endif
		return 0.0;
	}
#endif
	// Compute vectors along the 6 sides
	std::array<Eigen::Vector3d, 3> Sv, Tv;
	Vector3d VEC;
	Sv[0] = S[1] - S[0];
	Sv[1] = S[2] - S[1];
	Sv[2] = S[0] - S[2];
	Tv[0] = T[1] - T[0];
	Tv[1] = T[2] - T[1];
	Tv[2] = T[0] - T[2];
	Vector3d V, Z, minP, minQ;
	bool shown_disjoint = false;
	double mindd = (S[0] - T[0]).squaredNorm() + 1;  // Set first minimum safely high
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			// Find closest points on edges i & j, plus the vector (and distance squared) between these points
			getSegmentsPoints(VEC, P, Q, S[i], Sv[i], T[j], Tv[j]);
			V = Q - P;
			double dd = V.dot(V);
			// Verify this closest point pair only if the distance squared is less than the minimum found thus far.
			if (dd <= mindd)
			{
				minP = P;
				minQ = Q;
				mindd = dd;
				Z = S[(i + 2) % 3] - P;
				double a = Z.dot(VEC);
				Z = T[(j + 2) % 3] - Q;
				double b = Z.dot(VEC);
				if ((a <= 0) && (b >= 0))
					return sqrt(dd);
				double p = V.dot(VEC);
				if (a < 0)
					a = 0;
				if (b > 0)
					b = 0;
				if ((p - a + b) > 0)
					shown_disjoint = true;
			}
		}
	}
	Vector3d Sn = Sv[0].cross(Sv[1]); // Compute normal to S triangle
	double Snl = Sn.dot(Sn);      // Compute square of length of normal
	// If cross product is long enough,
	if (Snl > 1e-15)
	{
		// Get projection lengths of T points
		Vector3d Tp; //double Tp[3]
		V = S[0] - T[0];
		Tp[0] = V.dot(Sn);
		V = S[0] - T[1];
		Tp[1] = V.dot(Sn);
		V = S[0] - T[2];
		Tp[2] = V.dot(Sn);
		int point = -1;
		if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
		{
			point = (Tp[0] < Tp[1]) ? 0 : 1;
			if (Tp[2] < Tp[point])
				point = 2;
		}
		else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
		{
			point = (Tp[0] > Tp[1]) ? 0 : 1;
			if (Tp[2] > Tp[point])
				point = 2;
		}
		// If Sn is a separating direction, 
		if (point >= 0)
		{
			shown_disjoint = true;
			// Test whether the point found, when projected onto the 
			// other triangle, lies within the face.
			V = T[point] - S[0];
			Z = Sn.cross(Sv[0]);
			if (V.dot(Z) > 0)
			{
				V = T[point] - S[1];
				Z = Sn.cross(Sv[1]);
				if (V.dot(Z) > 0)
				{
					V = T[point] - S[2];
					Z = Sn.cross(Sv[2]);
					if (V.dot(Z) > 0)
					{
						// T[point] passed the test - it's a closest point for 
						// the T triangle; the other point is on the face of S
						P = T[point] + (Tp[point] / Snl) * Sn;
						Q = T[point];
						return (P - Q).norm();
					}
				}
			}
		}
	}
	Vector3d Tn = Tv[0].cross(Tv[1]);
	double Tnl = Tn.dot(Tn);
	if (Tnl > 1e-15)
	{
		Vector3d Sp; //double Sp[3]
		V = T[0] - S[0];
		Sp[0] = V.dot(Tn);
		V = T[0] - S[1];
		Sp[1] = V.dot(Tn);
		V = T[0] - S[2];
		Sp[2] = V.dot(Tn);
		int point = -1;
		if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
		{
			point = (Sp[0] < Sp[1]) ? 0 : 1;
			if (Sp[2] < Sp[point])
				point = 2;
		}
		else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
		{
			point = (Sp[0] > Sp[1]) ? 0 : 1;
			if (Sp[2] > Sp[point])
				point = 2;
		}
		if (point >= 0)
		{
			shown_disjoint = true;
			V = S[point] - T[0];
			Z = Tn.cross(Tv[0]);
			if (V.dot(Z) > 0)
			{
				V = S[point] - T[1];
				Z = Tn.cross(Tv[1]);
				if (V.dot(Z) > 0)
				{
					V = S[point] - T[2];
					Z = Tn.cross(Tv[2]);
					if (V.dot(Z) > 0)
					{
						P = S[point];
						Q = S[point] + (Sp[point] / Tnl) * Tn;
						return (P - Q).norm();
					}
				}
			}
		}
	}
	if (shown_disjoint)
	{
		P = minP;
		Q = minQ;
		return sqrt(mindd);
	}
	else
		return 0;
}

bool psykronix::isTriangleAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box)
{
#ifdef STATISTIC_DATA_COUNT
	isTriangleBoundC++;
#endif
	//pre-judge
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
	// Separating Axis Theorem
	std::array<Eigen::Vector3d, 3> edges = { trigon[1] - trigon[0],
										   trigon[2] - trigon[1],
										   trigon[0] - trigon[2] };
	std::array<Eigen::Vector3d, 3> coords = { Vector3d(1,0,0),
											   Vector3d(0,1,0),
											   Vector3d(0,0,1) };
	std::array<Eigen::Vector3d, 15> axes = { {
			coords[0],
			coords[1],
			coords[2],
			edges[0],
			edges[1],
			edges[2],
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
			Vector3d(0, vertex.y(), vertex.z())} };
	// iterate
	for (const auto& axis : axes) //fast than index
	{
		double minA = DBL_MAX;
		double maxA = -DBL_MAX;
		double minB = DBL_MAX;
		double maxB = -DBL_MAX;
		for (const auto& vertex : trigon)
		{
			double projection = (vertex - origin).dot(axis);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : vertexes)
		{
			double projection = vertex.dot(axis);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
#ifdef USING_MACHINE_PERCESION_THRESHOLD
		if (maxA + eps < minB || maxB + eps < minA)
#else
		if (maxA < minB || maxB < minA) // absolute zero
#endif
			return false;
	}
	return true;
}

bool psykronix::isSegmentAndTriangleIntersctSAT(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon)
{
#ifdef USING_MACHINE_PERCESION_THRESHOLD
	Vector3d vecZ = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[0]);
	double dot0 = (vecZ.dot(segment[0] - trigon[0]));
	double dot1 = (vecZ.dot(segment[1] - trigon[0]));
	if (dot0 * dot1 > 0.0)//include point on plane(dot==0)
		return false;
	Vector3d vecSeg = segment[1] - segment[0];
	//point on plane
	if (dot0 != 0.0)
	{
		double dotA = (trigon[0] - segment[0]).cross(trigon[1] - segment[0]).dot(vecSeg);
		double dotB = (trigon[1] - segment[0]).cross(trigon[2] - segment[0]).dot(vecSeg);
		double dotC = (trigon[2] - segment[0]).cross(trigon[0] - segment[0]).dot(vecSeg);
		return (dotA <= 0.0 && dotB <= 0.0 && dotC <= 0.0) || (dotA >= 0.0 && dotB >= 0.0 && dotC >= 0.0);
	}
	if (dot1 != 0.0)
	{
		double dotA = (trigon[0] - segment[1]).cross(trigon[1] - segment[1]).dot(vecSeg);
		double dotB = (trigon[1] - segment[1]).cross(trigon[2] - segment[1]).dot(vecSeg);
		double dotC = (trigon[2] - segment[1]).cross(trigon[0] - segment[1]).dot(vecSeg);
		return (dotA <= 0.0 && dotB <= 0.0 && dotC <= 0.0) || (dotA >= 0.0 && dotB >= 0.0 && dotC >= 0.0);
	}
	return isEdgeCrossTriangle(segment, trigon);
#endif
	Vector3d vecZ = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[0]);
	double d0 = vecZ.dot(segment[0] - trigon[0]);
	double d1 = vecZ.dot(segment[1] - trigon[0]);
	if (fabs(vecZ.dot(segment[0] - trigon[0])) < eps && abs(vecZ.dot(segment[1] - trigon[0])) < eps) // 2D-SAT, coplanar
	{
		std::array<Eigen::Vector3d, 4> axes = { {
				vecZ.cross(segment[1] - segment[0]),
				vecZ.cross(trigon[1] - trigon[0]),
				vecZ.cross(trigon[2] - trigon[1]),
				vecZ.cross(trigon[0] - trigon[2]) } };
		for (const auto& axis : axes)
		{
			double minA = DBL_MAX;
			double maxA = -DBL_MAX;
			for (const auto& vertex : trigon)
			{
				double projection = vertex.dot(axis);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			double dotB0 = segment[0].dot(axis);
			double dotB1 = segment[1].dot(axis);
			if (maxA < std::min(dotB0, dotB1) || std::max(dotB0, dotB1) < minA) // absolute zero
				return false;
		}
		return true;
	}
	else // 3D-SAT, not coplanar
	{
		Eigen::Vector3d vecSeg = segment[1] - segment[0];
		std::array<Eigen::Vector3d, 3> edges = { trigon[1] - trigon[0],
											   trigon[2] - trigon[1],
											   trigon[0] - trigon[2] };
		std::array<Eigen::Vector3d, 7> axes = { {
				edges[0],
				edges[1],
				edges[2],
				vecSeg,
				vecSeg.cross(edges[0]),
				vecSeg.cross(edges[1]),
				vecSeg.cross(edges[2])} };
		for (const auto& axis : axes)
		{
			double minA = DBL_MAX;
			double maxA = -DBL_MAX;
			for (const auto& vertex : trigon)
			{
				double projection = vertex.dot(axis);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			double dotB0 = segment[0].dot(axis);
			double dotB1 = segment[1].dot(axis);
			if (maxA < std::min(dotB0, dotB1) || std::max(dotB0, dotB1) < minA) // absolute zero
				return false;
		}
		return true;
	}
}

// base on Separating Axis Theorem
bool psykronix::isTwoTrianglesIntersectionSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
#ifdef STATISTIC_DATA_COUNT
	TriangularIntersectC++;
#endif
	//if (!isTwoTrianglesBoundingBoxIntersect(triA, triB))
	//	return false;
	std::array<Eigen::Vector3d, 3> edgesA = { triA[1] - triA[0],
											   triA[2] - triA[1],
											   triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = { triB[1] - triB[0],
											   triB[2] - triB[1],
											   triB[0] - triB[2] };
	bool triA_legal = !(edgesA[0].isZero() || edgesA[1].isZero() || edgesA[2].isZero());
	bool triB_legal = !(edgesB[0].isZero() || edgesB[1].isZero() || edgesB[2].isZero());
	if (triA_legal && triB_legal)
	{
		// Compute cross products of edges (15 possible axes)
		std::array<Eigen::Vector3d, 15> axes = { { //order matters speed
				//edgesA[0].cross(edgesA[1]),
				//edgesB[0].cross(edgesB[1]),
				edgesA[0],
				edgesA[1],
				edgesA[2],
				edgesB[0],
				edgesB[1],
				edgesB[2],
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
		for (const auto& axis : axes) //fast than index
		{
			double minA = DBL_MAX; // std::numeric_limits<double>::max();
			double maxA = -DBL_MAX;//std::numeric_limits<double>::lowest();
			double minB = DBL_MAX; //std::numeric_limits<double>::max();
			double maxB = -DBL_MAX;//std::numeric_limits<double>::lowest();
			for (const auto& vertex : triA) //fast than list
			{
				double projection = vertex.dot(axis);
				minA = std::min(minA, projection);
				maxA = std::max(maxA, projection);
			}
			for (const auto& vertex : triB)
			{
				double projection = vertex.dot(axis);
				minB = std::min(minB, projection);
				maxB = std::max(maxB, projection);
			}
			//double dotA0 = axis.dot(triA[0]);
			//double dotA1 = axis.dot(triA[1]);
			//double dotA2 = axis.dot(triA[2]);
			//double dotB0 = axis.dot(triB[0]);
			//double dotB1 = axis.dot(triB[1]);
			//double dotB2 = axis.dot(triB[2]);
			//double minA = std::min(std::min(dotA0, dotA1), dotA2);
			//double maxA = std::max(std::max(dotA0, dotA1), dotA2);
			//double minB = std::min(std::min(dotB0, dotB1), dotB2);
			//double maxB = std::max(std::max(dotB0, dotB1), dotB2);
#ifdef USING_MACHINE_PERCESION_THRESHOLD
			if (maxA + eps < minB || maxB + eps < minA)
#else
			if (maxA < minB || maxB < minA) // absolute zero
#endif
				return false;
		}
		return true;
	}
	else if (triA_legal && !triB_legal)
	{
		if (edgesB[0].isZero())
			return isSegmentAndTriangleIntersctSAT({ triB[0], triB[2] }, triA);
		if (edgesB[1].isZero())
			return isSegmentAndTriangleIntersctSAT({ triB[0], triB[1] }, triA);
		if (edgesB[2].isZero())
			return isSegmentAndTriangleIntersctSAT({ triB[1], triB[2] }, triA);
	}
	else if (!triA_legal && triB_legal)
	{
		if (edgesA[0].isZero())
			return isSegmentAndTriangleIntersctSAT({ triA[0], triA[2] }, triB);
		if (edgesA[1].isZero())
			return isSegmentAndTriangleIntersctSAT({ triA[0], triA[1] }, triB);
		if (edgesA[2].isZero())
			return isSegmentAndTriangleIntersctSAT({ triA[1], triA[2] }, triB);
	}
	else
	{
		// if intersect, must coplanar
		return false;
	}
}

bool psykronix::isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
#ifdef STATISTIC_DATA_COUNT
	TriangularIntersectC++;
	if (triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2])
		count_err_repeat_tri++;
#endif	// for degenerate triangle, return false
	std::array<Eigen::Vector3d, 3> edgesA = { triA[1] - triA[0],
										   triA[2] - triA[1],
										   triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = { triB[1] - triB[0],
											triB[2] - triB[1],
											triB[0] - triB[2] };
//#ifdef STATISTIC_DATA_COUNT
//	if (edgesA[0].cross(edgesA[1]).isZero() || edgesB[0].cross(edgesB[1]).isZero())
//		count_err_degen_tri++;
//#endif
	std::array<Eigen::Vector3d, 15> axes = { { //order matters speed
			edgesA[0].cross(edgesB[0]),
			edgesA[0].cross(edgesB[1]),
			edgesA[0].cross(edgesB[2]),
			edgesA[1].cross(edgesB[0]),
			edgesA[1].cross(edgesB[1]),
			edgesA[1].cross(edgesB[2]),
			edgesA[2].cross(edgesB[0]),
			edgesA[2].cross(edgesB[1]),
			edgesA[2].cross(edgesB[2]),
			edgesA[0],
			edgesA[1],
			edgesA[2],
			edgesB[0],
			edgesB[1],
			edgesB[2]} };
	// Check for overlap along each axis
	for (const auto& axis : axes) //fast than index
	{
		double minA = DBL_MAX; // std::numeric_limits<double>::max();
		double maxA = -DBL_MAX;//std::numeric_limits<double>::lowest();
		double minB = DBL_MAX; //std::numeric_limits<double>::max();
		double maxB = -DBL_MAX;//std::numeric_limits<double>::lowest();
		for (const auto& vertex : triA) //fast than list
		{
			double projection = vertex.dot(axis);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const auto& vertex : triB)
		{
			double projection = vertex.dot(axis);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		//double maxB = std::max(std::max(dotB0, dotB1), dotB2);
#ifdef USING_MACHINE_PERCESION_THRESHOLD
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

//------------------------------------------------------------------------------------
// Voxel 
//------------------------------------------------------------------------------------

bool psykronix::TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2)
{
	//#ifdef STATISTIC_DATA_COUNT
	//	TriangularIntersectC++;
	//#endif
	const Eigen::Vector3d& A1 = *(const Eigen::Vector3d*)(&T1.at(0));
	const Eigen::Vector3d& B1 = *(const Eigen::Vector3d*)(&T1.at(1));
	const Eigen::Vector3d& C1 = *(const Eigen::Vector3d*)(&T1.at(2));
	const Eigen::Vector3d& A2 = *(const Eigen::Vector3d*)(&T2.at(0));
	const Eigen::Vector3d& B2 = *(const Eigen::Vector3d*)(&T2.at(1));
	const Eigen::Vector3d& C2 = *(const Eigen::Vector3d*)(&T2.at(2));
	// T1
	Eigen::Vector3d A1B1 = B1 - A1;
	Eigen::Vector3d B1C1 = C1 - B1;
	Eigen::Vector3d C1A1 = A1 - C1;
	// A1 T2
	Eigen::Vector3d A1A2 = A2 - A1;
	Eigen::Vector3d A1B2 = B2 - A1;
	Eigen::Vector3d A1C2 = C2 - A1;
	// B1 T2
	Eigen::Vector3d B1A2 = A2 - B1;
	Eigen::Vector3d B1B2 = B2 - B1;
	Eigen::Vector3d B1C2 = C2 - B1;
	// C1 T2
	Eigen::Vector3d C1A2 = A2 - C1;
	Eigen::Vector3d C1B2 = B2 - C1;
	Eigen::Vector3d C1C2 = C2 - C1;

	// T2
	Eigen::Vector3d A2B2 = B2 - A2;
	Eigen::Vector3d B2C2 = C2 - A2;
	Eigen::Vector3d C2A2 = A2 - C2;
	// A2 T1
	Eigen::Vector3d A2A1 = A1 - A2;
	Eigen::Vector3d A2B1 = B1 - A2;
	Eigen::Vector3d A2C1 = C1 - A2;
	// B2 T1
	Eigen::Vector3d B2A1 = A1 - B2;
	Eigen::Vector3d B2B1 = B1 - B2;
	Eigen::Vector3d B2C1 = C1 - B2;
	// C2 T1
	Eigen::Vector3d C2A1 = A1 - C2;
	Eigen::Vector3d C2B1 = B1 - C2;
	Eigen::Vector3d C2C1 = C1 - C2;

	// n1
	Eigen::Vector3d n1 = A1B1.cross(B1C1);
	double n1_t1 = A1A2.dot(n1);
	double n1_t2 = A1B2.dot(n1);
	double n1_t3 = A1C2.dot(n1);

	// n2
	Eigen::Vector3d n2 = A2B2.cross(B2C2);
	double n2_t1 = A2A1.dot(n2);
	double n2_t2 = A2B1.dot(n2);
	double n2_t3 = A2C1.dot(n2);
	unsigned int n1_ts = (n1_t1 > 0.0 ? 18 : (n1_t1 < 0.0 ? 0 : 9)) + (n1_t2 > 0.0 ? 6 : (n1_t2 < 0.0 ? 0 : 3)) + (n1_t3 > 0.0 ? 2 : (n1_t3 < 0.0 ? 0 : 1));
	unsigned int n2_ts = (n2_t1 > 0.0 ? 18 : (n2_t1 < 0.0 ? 0 : 9)) + (n2_t2 > 0.0 ? 6 : (n2_t2 < 0.0 ? 0 : 3)) + (n2_t3 > 0.0 ? 2 : (n2_t3 < 0.0 ? 0 : 1));
	if (n1_ts == 0 || n2_ts == 0 || n1_ts == 26 || n2_ts == 26)
		return false;   // +*/<  72,48,0,18
	// 共面
	if (n1_ts == 13 || n2_ts == 13)
	{
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		double A2_k1 = A1A2.dot(A1B1_outboard);
		double B2_k1 = A1B2.dot(A1B1_outboard);
		double C2_k1 = A1C2.dot(A1B1_outboard);
		if (A2_k1 > 0 && B2_k1 > 0 && C2_k1 > 0)
			return false; // +*/<  84,72,0,21
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		double A2_k2 = B1A2.dot(B1C1_outboard);
		double B2_k2 = B1B2.dot(B1C1_outboard);
		double C2_k2 = B1C2.dot(B1C1_outboard);
		if (A2_k2 > 0 && B2_k2 > 0 && C2_k2 > 0)
			return false; // +*/<  96,96,0,24
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double A2_k3 = C1A2.dot(C1A1_outboard);
		double B2_k3 = C1B2.dot(C1A1_outboard);
		double C2_k3 = C1C2.dot(C1A1_outboard);
		if (A2_k3 > 0 && B2_k3 > 0 && C2_k3 > 0)
			return false; // +*/<  108,120,0,27 
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n2);
		double A1_k1 = A2A1.dot(A2B2_outboard);
		double B1_k1 = A2B1.dot(A2B2_outboard);
		double C1_k1 = A2C1.dot(A2B2_outboard);
		if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
			return false; // +*/<  120,144,0,30
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n2);
		double A1_k2 = B2A1.dot(B2C2_outboard);
		double B1_k2 = B2B1.dot(B2C2_outboard);
		double C1_k2 = B2C1.dot(B2C2_outboard);
		if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
			return false; // +*/<  132,168,0,33
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n2);
		double A1_k3 = C2A1.dot(C2A2_outboard);
		double B1_k3 = C2B1.dot(C2A2_outboard);
		double C1_k3 = C2C1.dot(C2A2_outboard);
		if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
			return false; // +*/<  144,192,0,36
		return true; // +*/<  144,192,0,18
	}
	Eigen::Vector3d O1, P1, Q1, O2, P2, Q2;
	switch (n1_ts)
	{
	case 9:
	case 17:
	{
		// A2;
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double k1 = A1A2.dot(A1B1_outboard);
		double k2 = B1A2.dot(B1C1_outboard);
		double k3 = C1A2.dot(C1A1_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 3:
	case 23:
	{
		// B2;
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double k1 = A1B2.dot(A1B1_outboard);
		double k2 = B1B2.dot(B1C1_outboard);
		double k3 = C1B2.dot(C1A1_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 1:
	case 25:
	{
		// C2;
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double k1 = A1C2.dot(A1B1_outboard);
		double k2 = B1C2.dot(B1C1_outboard);
		double k3 = C1C2.dot(C1A1_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 12:
	case 14:
	{
		// A2B2;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n1);
		double kA = A2A1.dot(A2B2_outboard);
		double kB = A2B1.dot(A2B2_outboard);
		double kC = A2C1.dot(A2B2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 4:
	case 22:
	{
		// B2C2;
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n1);
		double kA = B2A1.dot(B2C2_outboard);
		double kB = B2B1.dot(B2C2_outboard);
		double kC = B2C1.dot(B2C2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 10:
	case 16:
	{
		// C2A2;
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n1);
		double kA = C2A1.dot(C2A2_outboard);
		double kB = C2B1.dot(C2A2_outboard);
		double kC = C2C1.dot(C2A2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 7:     // C2;      A2B2;
	case 19:    // C2;      A2B2;
	case 8:     // C2A2;    A2B2;
	case 18:    // C2A2;    A2B2;
		O2 = A2;
		P2 = B2;
		Q2 = C2;
		break;
	case 11:    // A2;      B2C2;
	case 15:    // A2;      B2C2;
	case 6:     // A2B2;    B2C2;
	case 20:    // A2B2;    B2C2;
		O2 = B2;
		P2 = A2;
		Q2 = C2;
		break;
	case 5:     // B2;      C2A2;
	case 21:    // B2;      C2A2;
	case 2:     // B2C2;    C2A2;
	case 24:    // B2C2;    C2A2;
		O2 = C2;
		P2 = B2;
		Q2 = A2;
		break;
	}
	switch (n2_ts)
	{
	case 9:
	case 17:
	{
		// A1;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n2);
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n2);
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n2);
		double k1 = A2A1.dot(A2B2_outboard);
		double k2 = B2A1.dot(B2C2_outboard);
		double k3 = C2A1.dot(C2A2_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 3:
	case 23:
	{
		// B1;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n2);
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n2);
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n2);
		double k1 = A2B1.dot(A2B2_outboard);
		double k2 = B2B1.dot(B2C2_outboard);
		double k3 = C2B1.dot(C2A2_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 1:
	case 25:
	{
		// C1;
		Eigen::Vector3d A2B2_outboard = A1B2.cross(n2);
		Eigen::Vector3d B2C2_outboard = B1C2.cross(n2);
		Eigen::Vector3d C2A2_outboard = C1A2.cross(n2);
		double k1 = A2C1.dot(A2B2_outboard);
		double k2 = B2C1.dot(B2C2_outboard);
		double k3 = C2C1.dot(C2A2_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 12:
	case 14:
	{
		// A1B1;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n1);
		double kA = A2A1.dot(A2B2_outboard);
		double kB = A2B1.dot(A2B2_outboard);
		double kC = A2C1.dot(A2B2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 4:
	case 22:
	{
		// B1C1;
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n1);
		double kA = B2A1.dot(B2C2_outboard);
		double kB = B2B1.dot(B2C2_outboard);
		double kC = B2C1.dot(B2C2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 10:
	case 16:
	{
		// C1A1;
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n1);
		double kA = C2A1.dot(C2A2_outboard);
		double kB = C2B1.dot(C2A2_outboard);
		double kC = C2C1.dot(C2A2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 7:     // C1;      A1B1;
	case 19:    // C1;      A1B1;
	case 8:     // C1A1;    A1B1;
	case 18:    // C1A1;    A1B1;
		O1 = A1;
		P1 = B1;
		Q1 = C1;
		break;
	case 11:    // A1;      B1C1;
	case 15:    // A1;      B1C1;
	case 6:     // A1B1;    B1C1;
	case 20:    // A1B1;    B1C1;
		O1 = B1;
		P1 = A1;
		Q1 = C1;
		break;
	case 5:     // B1;      C1A1;
	case 21:    // B1;      C1A1;
	case 2:     // B1C1;    C1A2;
	case 24:    // B1C1;    C1A2;
		O1 = C1;
		P1 = B1;
		Q1 = A1;
		break;
	}
	Eigen::Vector3d O1O2 = O2 - O1;
	Eigen::Vector3d O1P2 = P2 - O1;
	Eigen::Vector3d O1Q2 = Q2 - O1;
	Eigen::Vector3d O1P1 = P1 - O1;
	Eigen::Vector3d O1Q1 = Q1 - O1;

	Eigen::Vector3d NP = O1P1.cross(O1O2);
	double kpk = NP.dot(O1Q1);
	double kpp = NP.dot(O1P2);
	double kpq = NP.dot(O1Q2);
	if ((kpk > 0 && kpp < 0 && kpq < 0) || (kpk < 0 && kpp > 0 && kpq > 0))
		return false; // +*/<  89,74,0,24

	Eigen::Vector3d NQ = O1Q1.cross(O1O2);
	double kqk = NQ.dot(O1P1);
	double kqp = NQ.dot(O1P2);
	double kqq = NQ.dot(O1Q2);
	if ((kqk > 0 && kqp < 0 && kqq < 0) || (kqk < 0 && kqp > 0 && kqq > 0))
		return false; // +*/<  101,98,0,30
	return true;
}

#undef max
#undef min
#include <map>
#include <set>
#include <array>
#include <vector>
#include <memory>
#include <ranges>
#include <format>
#include <cassert>
#include <numeric>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <type_traits>
#include "Eigen/Geometry"    
#include <iostream>
class Grid
{
public:
	Grid(const Eigen::AlignedBox3d& bounding, std::array<unsigned int, 3> num_size) :
		bounding_(bounding),
		num_size_(num_size),
		fill_cell_(num_size.at(0)* num_size.at(1)* num_size.at(2))
	{
	}
	void push(const std::array<Eigen::Vector3d, 3>& triangle, unsigned char mark)
	{
		for (size_t i = 0; i < 3; i++)
			if (std::isnan(triangle.at(i).x()) || std::isnan(triangle.at(i).y()) || std::isnan(triangle.at(i).z()))
				return;
		/***********************************************************************
		* 对三个点沿着三个方向进行排序，沿边扫描时走三条路径: 0->2; 0->1; 1->2;
		*             x_1
		* y_2       __--_
		*       __--     -_
		* y_0 -------______-_ y_1
		*   x_0             x_2
		**********************************************************************/
		std::array<unsigned int, 3> order_x({ 0, 1, 2 });
		std::array<unsigned int, 3> order_y({ 0, 1, 2 });
		std::array<unsigned int, 3> order_z({ 0, 1, 2 });
		std::sort(order_x.begin(), order_x.end(), [&](int i, int j) {return triangle.at(i).x() < triangle.at(j).x(); });
		std::sort(order_y.begin(), order_y.end(), [&](int i, int j) {return triangle.at(i).y() < triangle.at(j).y(); });
		std::sort(order_z.begin(), order_z.end(), [&](int i, int j) {return triangle.at(i).z() < triangle.at(j).z(); });
		// 排除完全不相交的情况
		if (triangle.at(order_x[0]).x() > bounding_.max().x())
			return;
		if (triangle.at(order_x[0]).y() > bounding_.max().y())
			return;
		if (triangle.at(order_x[0]).z() > bounding_.max().z())
			return;
		if (triangle.at(order_x[2]).x() < bounding_.min().x())
			return;
		if (triangle.at(order_x[2]).y() < bounding_.min().y())
			return;
		if (triangle.at(order_x[2]).z() < bounding_.min().z())
			return;
		// 三个方向格子长度
		double cell_size_x = (bounding_.max().x() - bounding_.min().x()) / num_size_.at(0);
		double cell_size_y = (bounding_.max().y() - bounding_.min().y()) / num_size_.at(1);
		double cell_size_z = (bounding_.max().z() - bounding_.min().z()) / num_size_.at(2);
		double cell_size_x_fraction = 1.0 / cell_size_x;
		double cell_size_y_fraction = 1.0 / cell_size_y;
		double cell_size_z_fraction = 1.0 / cell_size_z;

		/**********************************************************************
		* 例如体素5个格子，遍历的隔板范围为1:4；故采取ceil
		*          __--_
		*      __--     -_
		*    -------______-_
		* |   |   |   |   |   |
		* 0   1   2   3   4   5
		**********************************************************************/
		int partition_x_min_index = triangle.at(order_x[0]).x() > bounding_.min().x() ? ceil((triangle.at(order_x[0]).x() - bounding_.min().x()) * cell_size_x_fraction) : 1;
		int partition_y_min_index = triangle.at(order_y[0]).y() > bounding_.min().y() ? ceil((triangle.at(order_y[0]).y() - bounding_.min().y()) * cell_size_y_fraction) : 1;
		int partition_z_min_index = triangle.at(order_z[0]).z() > bounding_.min().z() ? ceil((triangle.at(order_z[0]).z() - bounding_.min().z()) * cell_size_z_fraction) : 1;
		int partition_x_max_index = triangle.at(order_x[2]).x() > bounding_.max().x() ? num_size_.at(0) : ceil((triangle.at(order_x[2]).x() - bounding_.min().x()) * cell_size_x_fraction);
		int partition_y_max_index = triangle.at(order_y[2]).y() > bounding_.max().y() ? num_size_.at(1) : ceil((triangle.at(order_y[2]).y() - bounding_.min().y()) * cell_size_y_fraction);
		int partition_z_max_index = triangle.at(order_z[2]).z() > bounding_.max().z() ? num_size_.at(2) : ceil((triangle.at(order_z[2]).z() - bounding_.min().z()) * cell_size_z_fraction);
		Eigen::Vector3d vec_x_01 = triangle.at(order_x[1]) - triangle.at(order_x[0]);
		Eigen::Vector3d vec_x_12 = triangle.at(order_x[2]) - triangle.at(order_x[1]);
		Eigen::Vector3d vec_x_20 = triangle.at(order_x[0]) - triangle.at(order_x[2]);
		Eigen::Vector3d vec_y_01 = triangle.at(order_y[1]) - triangle.at(order_y[0]);
		Eigen::Vector3d vec_y_12 = triangle.at(order_y[2]) - triangle.at(order_y[1]);
		Eigen::Vector3d vec_y_20 = triangle.at(order_y[0]) - triangle.at(order_y[2]);
		Eigen::Vector3d vec_z_01 = triangle.at(order_z[1]) - triangle.at(order_z[0]);
		Eigen::Vector3d vec_z_12 = triangle.at(order_z[2]) - triangle.at(order_z[1]);
		Eigen::Vector3d vec_z_20 = triangle.at(order_z[0]) - triangle.at(order_z[2]);
		double y_partial_x_01 = vec_x_01.x() == 0.0 ? 0.0 : vec_x_01.y() / vec_x_01.x();
		double z_partial_x_01 = vec_x_01.x() == 0.0 ? 0.0 : vec_x_01.z() / vec_x_01.x();
		double y_partial_x_12 = vec_x_12.x() == 0.0 ? 0.0 : vec_x_12.y() / vec_x_12.x();
		double z_partial_x_12 = vec_x_12.x() == 0.0 ? 0.0 : vec_x_12.z() / vec_x_12.x();
		double y_partial_x_20 = vec_x_20.x() == 0.0 ? 0.0 : vec_x_20.y() / vec_x_20.x();
		double z_partial_x_20 = vec_x_20.x() == 0.0 ? 0.0 : vec_x_20.z() / vec_x_20.x();
		double z_partial_y_01 = vec_y_01.y() == 0.0 ? 0.0 : vec_y_01.z() / vec_y_01.y();
		double x_partial_y_01 = vec_y_01.y() == 0.0 ? 0.0 : vec_y_01.x() / vec_y_01.y();
		double z_partial_y_12 = vec_y_12.y() == 0.0 ? 0.0 : vec_y_12.z() / vec_y_12.y();
		double x_partial_y_12 = vec_y_12.y() == 0.0 ? 0.0 : vec_y_12.x() / vec_y_12.y();
		double z_partial_y_20 = vec_y_20.y() == 0.0 ? 0.0 : vec_y_20.z() / vec_y_20.y();
		double x_partial_y_20 = vec_y_20.y() == 0.0 ? 0.0 : vec_y_20.x() / vec_y_20.y();
		double x_partial_z_01 = vec_z_01.z() == 0.0 ? 0.0 : vec_z_01.x() / vec_z_01.z();
		double y_partial_z_01 = vec_z_01.z() == 0.0 ? 0.0 : vec_z_01.y() / vec_z_01.z();
		double x_partial_z_12 = vec_z_12.z() == 0.0 ? 0.0 : vec_z_12.x() / vec_z_12.z();
		double y_partial_z_12 = vec_z_12.z() == 0.0 ? 0.0 : vec_z_12.y() / vec_z_12.z();
		double x_partial_z_20 = vec_z_20.z() == 0.0 ? 0.0 : vec_z_20.x() / vec_z_20.z();
		double y_partial_z_20 = vec_z_20.z() == 0.0 ? 0.0 : vec_z_20.y() / vec_z_20.z();
		int kbc = num_size_.at(1) * num_size_.at(2);
		// x
		for (int i_x = partition_x_min_index; i_x < partition_x_max_index; i_x++)
		{
			double partition_x = bounding_.min().x() + i_x * cell_size_x; // 隔板x的坐标
			// 隔板会切割三角形，并交边界于两个交点 point_twist=(partition_x, twist_y, twist_z); point_straight=(partition_x, straight_y, straight_z)
			double twist_y, twist_z, straight_y, straight_z;
			double dx_p0 = partition_x - triangle.at(order_x[0]).x();
			straight_y = y_partial_x_20 * dx_p0 + triangle.at(order_x[0]).y();
			straight_z = z_partial_x_20 * dx_p0 + triangle.at(order_x[0]).z();
			if (partition_x < triangle.at(order_x[1]).x())
			{
				twist_y = y_partial_x_01 * dx_p0 + triangle.at(order_x[0]).y();
				twist_z = z_partial_x_01 * dx_p0 + triangle.at(order_x[0]).z();
			}
			else
			{
				double dx_p1 = partition_x - triangle.at(order_x[1]).x();
				twist_y = y_partial_x_12 * dx_p1 + triangle.at(order_x[1]).y();
				twist_z = z_partial_x_12 * dx_p1 + triangle.at(order_x[1]).z();
			}
			double z_partial_y = (straight_z - twist_z) / (straight_y - twist_y);
			double y_partial_z = (straight_y - twist_y) / (straight_z - twist_z);
			// 计算y,z隔板的范围
			double y_min = std::min(twist_y, straight_y);
			double y_max = std::max(twist_y, straight_y);
			double z_min = std::min(twist_z, straight_z);
			double z_max = std::max(twist_z, straight_z);
			int y_min_index = y_min > bounding_.min().y() ? ceil((y_min - bounding_.min().y()) * cell_size_y_fraction) : 1;
			int y_max_index = y_max > bounding_.max().y() ? num_size_.at(1) : y_max == bounding_.min().y() ? 1 : ceil((y_max - bounding_.min().y()) * cell_size_y_fraction);
			int z_min_index = z_min > bounding_.min().z() ? ceil((z_min - bounding_.min().z()) * cell_size_z_fraction) : 1;
			int z_max_index = z_max > bounding_.max().z() ? num_size_.at(2) : z_max == bounding_.min().z() ? 1 : ceil((z_max - bounding_.min().z()) * cell_size_z_fraction);
			// 对于在x方向投影非常小的三角形（尺寸小于格子）并且没有与格子相交
			if (y_min_index == y_max_index && z_min_index == z_max_index)
			{
				int offset_base = i_x * kbc + (y_min_index - 1) * num_size_.at(2) + z_min_index - 1;
				fill_cell_.at(offset_base) |= mark;
				fill_cell_.at(offset_base - kbc) |= mark;
			}
			else
			{
				// point_twist与point_straight的连线会切割y隔板
				for (int i_y = y_min_index; i_y < y_max_index; i_y++)
				{
					// 交点位置为(partition_x,partition_y,partition_z)
					double partition_y = bounding_.min().y() + i_y * cell_size_y;
					double partition_z = z_partial_y * (partition_y - twist_y) + twist_z;
					if (partition_z < bounding_.min().z())
						continue;
					if (partition_z > bounding_.max().z())
						continue;
					// 索引位置(i_x, i_y, [i_z,i_z+1));
					int i_z = floor((partition_z - bounding_.min().z()) * cell_size_z_fraction);
					i_z = i_z == num_size_.at(2) ? i_z - 1 : i_z;
					// 交点位置的几何意义为：位于i_x,i_y的一根柱与三角面相交，则该柱的四周的四个区域均与三角面相交
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - kbc - num_size_.at(2)) |= mark;
				}
				// point_twist与point_straight的连线会切割z隔板
				for (int i_z = z_min_index; i_z < z_max_index; i_z++)
				{
					double partition_z = bounding_.min().z() + i_z * cell_size_z;
					double partition_y = y_partial_z * (partition_z - twist_z) + twist_y;
					if (partition_y < bounding_.min().y())
						continue;
					if (partition_y > bounding_.max().y())
						continue;
					int i_y = floor((partition_y - bounding_.min().y()) * cell_size_y_fraction);
					i_y = i_y == num_size_.at(1) ? i_y - 1 : i_y;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - kbc - 1) |= mark;
				}
			}
		}
		// y
		for (int i_y = partition_y_min_index; i_y < partition_y_max_index; i_y++)
		{
			double partition_y = bounding_.min().y() + i_y * cell_size_y;
			double twist_z, twist_x, straight_z, straight_x;
			double dy_p0 = partition_y - triangle.at(order_y[0]).y();
			straight_z = z_partial_y_20 * dy_p0 + triangle.at(order_y[0]).z();
			straight_x = x_partial_y_20 * dy_p0 + triangle.at(order_y[0]).x();
			if (partition_y < triangle.at(order_y[1]).y())
			{
				twist_z = z_partial_y_01 * dy_p0 + triangle.at(order_y[0]).z();
				twist_x = x_partial_y_01 * dy_p0 + triangle.at(order_y[0]).x();
			}
			else
			{
				double dy_p1 = partition_y - triangle.at(order_y[1]).y();
				twist_z = z_partial_y_12 * dy_p1 + triangle.at(order_y[1]).z();
				twist_x = x_partial_y_12 * dy_p1 + triangle.at(order_y[1]).x();
			}
			double x_partial_z = (straight_x - twist_x) / (straight_z - twist_z);
			double z_partial_x = (straight_z - twist_z) / (straight_x - twist_x);
			double z_min = std::min(twist_z, straight_z);
			double z_max = std::max(twist_z, straight_z);
			double x_min = std::min(twist_x, straight_x);
			double x_max = std::max(twist_x, straight_x);
			int z_min_index = z_min > bounding_.min().z() ? ceil((z_min - bounding_.min().z()) * cell_size_z_fraction) : 1;
			int z_max_index = z_max > bounding_.max().z() ? num_size_.at(2) : z_max == bounding_.min().z() ? 1 : ceil((z_max - bounding_.min().z()) * cell_size_z_fraction);
			int x_min_index = x_min > bounding_.min().x() ? ceil((x_min - bounding_.min().x()) * cell_size_x_fraction) : 1;
			int x_max_index = x_max > bounding_.max().x() ? num_size_.at(0) : x_max == bounding_.min().x() ? 1 : ceil((x_max - bounding_.min().x()) * cell_size_x_fraction);
			if (z_min_index == z_max_index && x_min_index == x_max_index)
			{
				int offset_base = (x_min_index - 1) * kbc + i_y * num_size_.at(2) + z_min_index - 1;
				fill_cell_.at(offset_base) |= mark;
				fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
			}
			else
			{
				for (int i_z = z_min_index; i_z < z_max_index; i_z++)
				{
					double partition_z = bounding_.min().z() + i_z * cell_size_z;
					double partition_x = x_partial_z * (partition_z - twist_z) + twist_x;
					if (partition_x < bounding_.min().x())
						continue;
					if (partition_x > bounding_.max().x())
						continue;
					int i_x = floor((partition_x - bounding_.min().x()) * cell_size_x_fraction);
					i_x = i_x == num_size_.at(1) ? i_x - 1 : i_x;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2) - 1) |= mark;
				}
				for (int i_x = x_min_index; i_x < x_max_index; i_x++)
				{
					double partition_x = bounding_.min().x() + i_x * cell_size_x;
					double partition_z = z_partial_x * (partition_x - twist_x) + twist_z;
					if (partition_z < bounding_.min().z())
						continue;
					if (partition_z > bounding_.max().z())
						continue;
					int i_z = floor((partition_z - bounding_.min().z()) * cell_size_z_fraction);
					i_z = i_z == num_size_.at(1) ? i_z - 1 : i_z;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - kbc - num_size_.at(2)) |= mark;
				}
			}
		}
		// z
		for (int i_z = partition_z_min_index; i_z < partition_z_max_index; i_z++)
		{
			double partition_z = bounding_.min().z() + i_z * cell_size_z;
			double twist_x, twist_y, straight_x, straight_y;
			double dz_p0 = partition_z - triangle.at(order_z[0]).z();
			straight_x = x_partial_z_20 * dz_p0 + triangle.at(order_z[0]).x();
			straight_y = y_partial_z_20 * dz_p0 + triangle.at(order_z[0]).y();
			if (partition_z < triangle.at(order_z[1]).z())
			{
				twist_x = x_partial_z_01 * dz_p0 + triangle.at(order_z[0]).x();
				twist_y = y_partial_z_01 * dz_p0 + triangle.at(order_z[0]).y();
			}
			else
			{
				double dz_p1 = partition_z - triangle.at(order_z[1]).z();
				twist_x = x_partial_z_12 * dz_p1 + triangle.at(order_z[1]).x();
				twist_y = y_partial_z_12 * dz_p1 + triangle.at(order_z[1]).y();
			}
			double y_partial_x = (straight_y - twist_y) / (straight_x - twist_x);
			double x_partial_y = (straight_x - twist_x) / (straight_y - twist_y);
			double x_min = std::min(twist_x, straight_x);
			double x_max = std::max(twist_x, straight_x);
			double y_min = std::min(twist_y, straight_y);
			double y_max = std::max(twist_y, straight_y);
			int x_min_index = x_min > bounding_.min().x() ? ceil((x_min - bounding_.min().x()) * cell_size_x_fraction) : 1;
			int x_max_index = x_max > bounding_.max().x() ? num_size_.at(0) : x_max == bounding_.min().x() ? 1 : ceil((x_max - bounding_.min().x()) * cell_size_x_fraction);
			int y_min_index = y_min > bounding_.min().y() ? ceil((y_min - bounding_.min().y()) * cell_size_y_fraction) : 1;
			int y_max_index = y_max > bounding_.max().y() ? num_size_.at(1) : y_max == bounding_.min().y() ? 1 : ceil((y_max - bounding_.min().y()) * cell_size_y_fraction);
			if (x_min_index == x_max_index && y_min_index == y_max_index)
			{
				int offset_base = (x_min_index - 1) * kbc + (y_min_index - 1) * num_size_.at(2) + i_z;
				fill_cell_.at(offset_base) |= mark;
				fill_cell_.at(offset_base - 1) |= mark;
			}
			else
			{
				for (int i_x = x_min_index; i_x < x_max_index; i_x++)
				{
					double partition_x = bounding_.min().x() + i_x * cell_size_x;
					double partition_y = y_partial_x * (partition_x - twist_x) + twist_y;
					if (partition_y < bounding_.min().y())
						continue;
					if (partition_y > bounding_.max().y())
						continue;
					int i_y = floor((partition_y - bounding_.min().y()) * cell_size_y_fraction);
					i_y = i_y == num_size_.at(1) ? i_y - 1 : i_y;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - kbc - 1) |= mark;
				}
				for (int i_y = y_min_index; i_y < y_max_index; i_y++)
				{
					double partition_y = bounding_.min().y() + i_y * cell_size_y;
					double partition_x = x_partial_y * (partition_y - twist_y) + twist_x;
					if (partition_x < bounding_.min().x())
						continue;
					if (partition_x > bounding_.max().x())
						continue;
					int i_x = floor((partition_x - bounding_.min().x()) * cell_size_x_fraction);
					i_x = i_x == num_size_.at(1) ? i_x - 1 : i_x;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2) - 1) |= mark;
				}
			}
		}
	}
	std::vector<std::tuple<Eigen::Vector3d, unsigned char>> list() const
	{
		std::vector<std::tuple<Eigen::Vector3d, unsigned char>> res;
		double cell_size_x = (bounding_.max().x() - bounding_.min().x()) / num_size_.at(0);
		double cell_size_y = (bounding_.max().y() - bounding_.min().y()) / num_size_.at(1);
		double cell_size_z = (bounding_.max().z() - bounding_.min().z()) / num_size_.at(2);
		int kbc = num_size_.at(1) * num_size_.at(2);
		for (int i = 0; i < fill_cell_.size(); i++)
		{
			if (fill_cell_.at(i) == 0)
				continue;
			int ix = i / kbc;
			int t = i % kbc;
			int iy = t / num_size_.at(2);
			int iz = t % num_size_.at(2);
			res.push_back({ {
				  bounding_.min().x() + (ix + 0.5) * cell_size_x,
				  bounding_.min().y() + (iy + 0.5) * cell_size_y,
				  bounding_.min().z() + (iz + 0.5) * cell_size_z},
				  fill_cell_.at(i) });
		}
		return res;
	}
private:
	std::vector<unsigned char> fill_cell_;
	Eigen::AlignedBox3d bounding_;
	std::array<unsigned int, 3> num_size_;
};

static void test()
{
	Grid grid(Eigen::AlignedBox3d(Eigen::Vector3d{ 0, 0, 0 }, Eigen::Vector3d{ 10, 10, 10 }), { 100,100,100 });
	double k = 10.0 / RAND_MAX;
	const int num = (int)1e5; //stack overflow
	std::array<std::array<Eigen::Vector3d, 3>, num> coor;
	for (size_t i = 0; i < coor.size(); i++)
		coor.at(i) = { Eigen::Vector3d{k* rand(), k* rand(), k* rand()}, Eigen::Vector3d{k* rand(), k* rand(), k* rand()}, Eigen::Vector3d{k* rand(), k* rand(), k* rand()} };
	auto start = std::chrono::steady_clock::now(); // 开始计时
	for (size_t i = 0; i < num; i++)
	{
		grid.push(coor.at(i), 0xff);
		if (i % 100 == 0)
			std::cout << "\r" << i / double(num) * 100.0 << "%\t\t\t";
	}
	auto end = std::chrono::steady_clock::now();   // 结束计时
	std::cout << "\r" << 100.0 << "%\t\t\t" << std::endl;
	auto res = grid.list();
	std::cout << res.size() << std::endl;
	std::chrono::duration<double> elapsed = end - start;
	std::cout << elapsed.count() * 1000 << " ms." << std::endl;

}

