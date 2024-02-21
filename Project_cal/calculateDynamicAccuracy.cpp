#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace accura;

bool accura::isTwoTrianglesPenetrationSAT(const std::array<Vector2d, 3>& triA, const std::array<Vector2d, 3>& triB, double tolerance)
{
	// kd-tree bounding-box been judged
	std::array<Eigen::Vector2d, 6> edgesAB = {
        (triA[1] - triA[0]).normalized(),
		(triA[2] - triA[1]).normalized(),
		(triA[0] - triA[2]).normalized(),
		(triB[1] - triB[0]).normalized(),
		(triB[2] - triB[1]).normalized(),
		(triB[0] - triB[2]).normalized() };
	for (auto& iter : edgesAB)
		iter = Vector2d(-iter[1], iter[0]); //rotz(pi/2)
	std::array<Vector2d, 3> triA_R, triB_R; //using relative coordinate
	double precision = 0; // using max-coord k as coeff
	for (int i = 0; i < 3; ++i)
	{
		triA_R[i] = triA[i] - triA[0]; //triA_R[0]=Vector3d(0,0,0)
		triB_R[i] = triB[i] - triA[0];
		precision = std::max(std::max(fabs(triA_R[i][0]), fabs(triA_R[i][1])), precision);
		precision = std::max(std::max(fabs(triB_R[i][0]), fabs(triB_R[i][1])), precision);
	}
	precision = precision * tolerance;
	double minA, maxA, minB, maxB, projection;
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
		if (maxA < minB + precision || maxB < minA + precision)
			return false;
	}
	return true;
}

bool accura::isTwoTrianglesPenetrationSAT(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB, double tolerance)
{
	std::array<Eigen::Vector3d, 3> edgesA = {
        (triA[1] - triA[0]).normalized(),
		(triA[2] - triA[1]).normalized(),
		(triA[0] - triA[2]).normalized() };
	std::array<Eigen::Vector3d, 3> edgesB = {
		(triB[1] - triB[0]).normalized(),
		(triB[2] - triB[1]).normalized(),
		(triB[0] - triB[2]).normalized() };
	Eigen::Vector3d normalA = edgesA[0].cross(edgesA[1]).normalized();
	Eigen::Vector3d normalB = edgesB[0].cross(edgesB[1]).normalized();
	std::array<Vector2d, 3> triA_R, triB_R; //using relative coordinate
	double precision = 0; // using max-coord k as coeff
	for (int i = 0; i < 3; ++i)
	{
		triA_R[i] = triA[i] - triA[0]; //triA_R[0]=Vector3d(0,0,0)
		triB_R[i] = triB[i] - triA[0];
		precision = std::max(std::max(fabs(triA_R[i][0]), fabs(triA_R[i][1])), precision);
		precision = std::max(std::max(fabs(triB_R[i][0]), fabs(triB_R[i][1])), precision);
	}
	precision = precision * tolerance;
	double minA, maxA, minB, maxB, projection;
	if (normalA.cross(normalB).isZero(tolerance)) //is parallel
	{
#ifdef USING_COMPLETE_SEPARATION_AXIS 
        if (!fabs(normalA.dot(triB_R[0])) < precision) //not coplanar
			return false;
		std::array<Eigen::Vector2d, 6> edgesAB = { edgesA[0],edgesA[1],edgesA[2],edgesB[0],edgesB[1],edgesB[2] };
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
			if (maxA < minB + precision || maxB < minA + precision)
				return false;
		}
		return true;
#else
		return false;//exclude coplanar
#endif
	}
	//separation axis theorem
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
	for (const auto& axis : axes)
	{
		if (axis.isZero(tolerance))
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
		if (maxA < minB + precision || maxB < minA + precision)
			return false;
	}
	return true;
}

