#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace accura;

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

//Precision
double Precision::sm_toleAngle = 1e-12;
double Precision::sm_toleConfusion = 1e-7;

bool Precision::isParallel(Eigen::Vector3d& vecA, Eigen::Vector3d& vecB, double tolerance /*= Angular()*/)
{
	//double area = vecA.cross(vecB).norm(); // 0=|a|*|b|*sin(theta)
	Eigen::Vector3d vecAU = vecA.normalized();
	Eigen::Vector3d vecBU = vecB.normalized();
    Vector3d croPro = vecAU.cross(vecBU);
    return croPro.norm() <= tolerance;//tobe optimize
    //double max = std::max(std::max(fabs(croPro[0]), fabs(croPro[1])), fabs(croPro[2]));
}

bool Precision::isPerpendi(Eigen::Vector3d& vecA, Eigen::Vector3d& vecB, double tolerance /*= Angular()*/)
{
	//double proj = vecA.dot(vecB); // 0=|a|*|b|*cos(theta)
	Eigen::Vector3d vecAU = vecA.normalized();
	Eigen::Vector3d vecBU = vecB.normalized();
	double dotPro = vecAU.dot(vecBU); //scalar products
	return dotPro <= tolerance;
}

bool Precision::isPointsCoincident(Eigen::Vector3d& pntA, Eigen::Vector3d& pntB, double tolerance /*= 10*Confusion()*/)
{
	double distance2 = (pntA - pntB).squaredNorm();
	return distance2 <= tolerance * tolerance;
}

bool Precision::isPointOnLine(Eigen::Vector3d& point, const clash::Segment3d& line, double tolerance /*= Confusion()*/)
{
	double distance = eigen::getDistanceOfPointAndLine(point, line);
	return distance <= tolerance;
}

bool Precision::isPointOnPlane(Eigen::Vector3d& point, const clash::Plane3d& plane, double tolerance /*= 0.1*Confusion()*/)
{
	double distance = eigen::getDistanceOfPointAndPlane(point, plane);
	return distance <= tolerance;
}
