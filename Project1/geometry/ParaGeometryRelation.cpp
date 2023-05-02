#include "pch.h"
using namespace std;
namespace para
{
//----------------------------------------------------------------------------------------------
// 	                                        relation
//----------------------------------------------------------------------------------------------
	BPParaVec getNearestPointOfPointLine(const BPParaVec& point, const PGSegment& line) //get nearst point
	{
		BPParaVec line0 = line.start();
		BPParaVec line1 = line.end();
		if (isParallel(point - line0, point - line1))  //parallel
			return point;
		if (isCoincident(line1, line0))  // norm of line is zero.
			return line0;
		double a = norm(line0 - line1) * norm(line0 - line1);
		double b = (line0 - line1) * (line0 - point);
		return line0 + (b / a) * (line1 - line0);
	}

	bool is_shadow_matrix_on_xoy()
	{
		return true;
	}

	BPParaTransform get_full_rank_matrix_from_shadow(const BPParaTransform& M, bool withTrans = true)
	{
		BPParaVec n = getMatrixsAxisX(M);
		BPParaVec o = getMatrixsAxisY(M);
		BPParaVec p = (withTrans) ? getMatrixsPosition(M) : g_axisO;
		return setMatrixByColumnVectors(n, o, g_axisZ, p);
	}

	bool is_point_locate_on_plane(const BPParaVec& point, const BPParaTransform& plane)
	{
		BPParaVec pointO = getMatrixsPosition(plane);  // the original point
		BPParaVec vectorZ = getMatrixsAxisZ(getOrthogonalMatrix(plane));
		return isPerpendi(vectorZ, point - pointO);
	}

	double getDistanceOfPointLine(const BPParaVec& point, const PGSegment& line)
	{
		if (line.isCoincident()) // norm of line is zero.
			return norm(point - line.start());
		BPParaVec C = line.start() - point;
		BPParaVec vec = line.vector();
		return norm(C + (-1.0 * vec * C) / (vec * vec) * vec);
	}

	bool isPointOnArc(const BPParaVec& point, const BPParaTransform& arc)
	{
		BPParaTransform mat = arc;
		if (is_shadow_matrix_on_xoy())
			mat = get_full_rank_matrix_from_shadow(mat);
		if (!is_point_locate_on_plane(point, getOrthogonalMatrix(mat)))
			return false;
		BPParaVec relaP = inverse(mat) * point;
		double d = norm(relaP);
		if (abs(d - 1) < PL_Length)
			return true;
		return d <= 1;
	}

	//bool isTwoVectorsSameDirection(const BPParaVec& vecA, const BPParaVec& vecB)
	//{
	//	if (vecA.isOrigin() || vecB.isOrigin())
	//		return true;
	//	return (isParallel(vecA, vecB) && (vecA * vecB) > 0);
	//}
	bool isPointOnLine(const BPParaVec& point, const PGSegment& line)
	{
		return isParallel(point - line.start(), point - line.end());
	}
	GRTwoVectorsDirection isTwoVectorsSameDirection(const BPParaVec& vecA, const BPParaVec& vecB)
	{
		if (vecA.isOrigin() || vecB.isOrigin())
			return GRTwoVectorsDirection::Direction_Any;
		if (isParallel(vecA, vecB))
			return ((vecA * vecB) > 0) ? GRTwoVectorsDirection::Direction_Same :
			GRTwoVectorsDirection::Direction_Opposite;
		else if (isPerpendi(vecA, vecB))
			return GRTwoVectorsDirection::Direction_Vertical;
		else
			return GRTwoVectorsDirection::Direction_None;
	}


}
