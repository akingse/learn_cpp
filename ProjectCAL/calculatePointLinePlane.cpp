#include "pch.h"
#include "calculatePointLinePlane.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;

//2D
double eigen::getDistanceOfPointAndLine(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 2>& line)
{
	Vector2d vecSeg = line[1] - line[0];// not zero
	if (vecSeg.isZero())
		return (line[0] - point).norm();
	double k = vecSeg.dot(point - line[0]) / vecSeg.dot(vecSeg);
	Vector2d local = point + k * vecSeg;
	return (line[0] - local).norm();
}

static bool isTwoSegmentsIntersect_DA(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB, double toleDist)
{
	//tole<0 less judge, tole>0 more judge
	Vector2d vecA = segmA[1] - segmA[0];
	Vector2d vecB = segmB[1] - segmB[0];
	double spec = 0;
	spec = std::max(std::max(fabs(vecA[0]), fabs(vecA[1])), spec);
	spec = std::max(std::max(fabs(vecB[0]), fabs(vecB[1])), spec);
	spec = spec * toleDist;// assert(spec > toleDist);
	if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) + spec ||
		std::max(segmB[0][0], segmB[1][0]) < std::min(segmA[0][0], segmA[1][0]) + spec ||
		std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) + spec ||
		std::max(segmB[0][1], segmB[1][1]) < std::min(segmA[0][1], segmA[1][1]) + spec)
		return false;
	Vector2d AB_0 = segmB[0] - segmA[0];
	Vector2d AB_1 = segmB[1] - segmA[0];
	Vector2d BA_0 = segmA[0] - segmB[0];
	Vector2d BA_1 = segmA[1] - segmB[0];
	return //double straddle test, cross2d opposite direction
		(AB_0[0] * vecA[1] - AB_0[1] * vecA[0]) * (AB_1[0] * vecA[1] - AB_1[1] * vecA[0]) <= spec &&
		(BA_0[0] * vecB[1] - BA_0[1] * vecB[0]) * (BA_1[0] * vecB[1] - BA_1[1] * vecB[0]) <= spec; //not support both near zero
}

static bool isTwoSegmentsIntersect_DA(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB, double toleDist)
{
	//tole<0 less judge, tole>0 more judge
	Vector3d vecA = segmA[1] - segmA[0];
	Vector3d vecB = segmB[1] - segmB[0];
	double spec = 0;
    for (int i = 0; i < 3; i++)
		spec = std::max(std::max(fabs(vecA[i]), fabs(vecB[i])), spec);
	spec = spec * toleDist;// assert(spec > toleDist);
	if (std::max(segmA[0][0], segmA[1][0]) < std::min(segmB[0][0], segmB[1][0]) + spec ||
		std::max(segmB[0][0], segmB[1][0]) < std::min(segmA[0][0], segmA[1][0]) + spec ||
		std::max(segmA[0][1], segmA[1][1]) < std::min(segmB[0][1], segmB[1][1]) + spec ||
		std::max(segmB[0][1], segmB[1][1]) < std::min(segmA[0][1], segmA[1][1]) + spec ||
		std::max(segmA[0][2], segmA[1][2]) < std::min(segmB[0][2], segmB[1][2]) + spec ||
		std::max(segmB[0][2], segmB[1][2]) < std::min(segmA[0][2], segmA[1][2]) + spec)
		return false;
	Vector3d AB_0 = segmB[0] - segmA[0];
	Vector3d AB_1 = segmB[1] - segmA[0];
	Vector3d BA_0 = segmA[0] - segmB[0];
	Vector3d BA_1 = segmA[1] - segmB[0];
	return //double straddle test, cross2d opposite direction
		(AB_0[0] * vecA[1] - AB_0[1] * vecA[0]) * (AB_1[0] * vecA[1] - AB_1[1] * vecA[0]) <= spec &&
		(BA_0[0] * vecB[1] - BA_0[1] * vecB[0]) * (BA_1[0] * vecB[1] - BA_1[1] * vecB[0]) <= spec; //not support both near zero
}

Eigen::Vector2d eigen::getIntersectPointOfTwoLines(const std::array<Eigen::Vector2d, 2>& lineA, const std::array<Eigen::Vector2d, 2>& lineB)
{
	Eigen::Vector2d vecA = lineA[1] - lineA[0];
	Eigen::Vector2d vecB = lineB[1] - lineB[0];
	if (isParallel2d(vecA, vecB))
		return gVecNaN2d; //lines collinear
	double k = cross2d(lineB[0] - lineA[0], vecB) / cross2d(vecA, vecB);
	return lineA[0] + k * vecA;
}

Eigen::Vector3d eigen::getIntersectPointOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB)
{
	Eigen::Vector3d dirA = lineA[1] - lineA[0]; //direction
	Eigen::Vector3d dirB = lineB[1] - lineB[0];
	//if (isParallel2d(vecA, vecB))
	//	return gVecNaN2d; //lines collinear
	Eigen::Vector3d crossProd = dirA.cross(dirB); // plane normal
	double denom = crossProd.squaredNorm();
	if (denom == 0)
		return gVecNaN;// std::nullopt; // 平行或重合
	Eigen::Vector3d delta = lineB[0] - lineA[0];
    if (epsF < fabs(crossProd.normalized().dot(delta)))
        return gVecNaN;
	double t = (delta.cross(dirB).dot(crossProd)) / denom;
	//double s = (delta.cross(dirA).dot(crossProd)) / denom;
	//if (t < 0.0 || t > 1.0 || s < 0.0 || s > 1.0)
	//	return false;
	Eigen::Vector3d point = lineA[0] + t * dirA;
	return point;
}

static Eigen::Vector2d getTwoSegmentsIntersectPoint(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
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

static Eigen::Vector3d getTwoSegmentsIntersectPoint(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
{
	if (!isTwoSegmentsIntersect(segmA, segmB))
		return gVecNaN;
	Vector3d vecSeg = segmA[1] - segmA[0];
	Vector3d normal = (vecSeg).cross(segmB[1] - segmB[0]);
	if (normal.isZero(epsF)) // intersect cause collinear
		return segmA[0];
	//double k = (segmA[0] - segmB[0]).cross(segmA[0] - segmB[1]).norm() / normal.norm(); //triangle area method
	double k = (segmB[0] - segmA[0]).cross(vecSeg).dot(normal) / normal.squaredNorm(); //k
	return segmA[0] + k * vecSeg;
}

// for soft clash
//inline Vector3d getIntersectOfPointAndLine(const Vector3d& point, const std::array<Vector3d, 2>& segm)
//{
//	Eigen::Vector3d direction = (segm[1] - segm[0]);// not zero
//	double projection = direction.dot(point);
//	//the projection must on segment
//	if (direction.dot(segm[1]) < projection || projection < direction.dot(segm[0]))
//		return gVecNaN;// DBL_MAX;
//	double k = direction.dot(point - segm[0]) / direction.dot(direction);
//	return (segm[0] - point + k * direction);//.squaredNorm();
//}
//inline Vector3d getIntersectOfTwoSegments(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
//{
//	Vector3d vectA = segmA[1] - segmA[0];
//	Vector3d vectB = segmB[1] - segmB[0];
//	double delta1 = (segmB[0] - segmA[0]).dot(vectA);
//	double delta2 = (segmB[0] - segmA[0]).dot(vectB);
//	// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
//	double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
//	if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
//		return gVecNaN;// DBL_MAX;
//	double kA = 1 / deno * (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2);
//	double kB = 1 / deno * (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2);
//	//	Vector3d pointA = segmA[0] + kA * vectA;
//	//	Vector3d pointB = segmB[0] + kB * vectB;
//	//whether two intersect-point inside segments
//	if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
//		return (segmA[0] + kA * vectA - segmB[0] - kB * vectB);//.squaredNorm();
//	return gVecNaN;// DBL_MAX; // nearest point outof segments
//}
//inline Eigen::Vector3d getIntersectPointOfLineAndPlane(const PosVec3d& ray, const Plane3d& plane)
//{
//	double k = (plane.m_origin - ray[0]).dot(plane.m_normal) / (ray[1].dot(plane.m_normal));
//	return ray[0] + k * ray[1];
//}

//calculate distance
double eigen::getDistanceOfPointAndLine(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& line)
{
	Vector3d vecSeg = line[1] - line[0];// not zero
	if (vecSeg.isZero())
		return (line[0] - point).norm();
	double k = vecSeg.dot(point - line[0]) / vecSeg.dot(vecSeg);
	Vector3d local = point + k * vecSeg;
	return (line[0] - local).norm();
}

double eigen::getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal)
{
	if (normal.isZero()) // error triangle plane
        return (origin - point).norm();
    //double k = (origin - point).dot(normal) / normal.dot(normal); //normal not required normalize
    //Vector3d local = point + k * normal;
    //return (local - point).norm(); //fabs(k) * normal.norm();
    double dotPro = (origin - point).dot(normal.normalized());
	return fabs(dotPro);
}

double eigen::getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& plane)
{
    return getDistanceOfPointAndPlane(point, plane[0], (plane[1] - plane[0]).cross(plane[2] - plane[0]));
}

//double eigen::getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const clash::Plane3d& plane)
//{
//	return getDistanceOfPointAndPlane(point, plane.m_origin, plane.m_normal);
//}

//calculate intersect
double eigen::getDistanceOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB)
{
	Eigen::Vector3d vectA = lineA[1] - lineA[0];
	Eigen::Vector3d vectB = lineB[1] - lineB[0];
	double deltaA = (lineB[0] - lineA[0]).dot(vectA);
	double deltaB = (lineB[0] - lineA[0]).dot(vectB);
    double deno = vectA.dot(vectB) * vectB.dot(vectA) - vectA.dot(vectA) * vectB.dot(vectB);//a*d-b*c
	if (deno == 0.0) // parallel
		return getDistanceOfPointAndLine(lineA[0], lineB);
	double kA = (vectB.dot(vectA) * deltaB - vectB.dot(vectB) * deltaA) / deno;
	double kB = (vectA.dot(vectA) * deltaB - vectA.dot(vectB) * deltaA) / deno;
	Vector3d pointA = lineA[0] + kA * vectA;
	Vector3d pointB = lineB[0] + kB * vectB;
	return (pointB - pointA).norm();
}

Eigen::Vector3d eigen::getNearestPointOfPointAndLine(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& line)
{
	Vector3d vecSeg = line[1] - line[0];// not zero
	if (vecSeg.isZero())
		return line[0];
	double k = vecSeg.dot(point - line[0]) / vecSeg.dot(vecSeg);
	Vector3d local = point + k * vecSeg;
	return local;
}

Eigen::Vector3d eigen::getNearestPointOfPointAndPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal)
{
	double k = (origin - point).dot(normal) / normal.dot(normal); //normal not required normalize
	Vector3d local = point + k * normal;
	return local;
}

clash::Segment3d eigen::getNearestPointOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB)
{
	Eigen::Vector3d vectA = lineA[1] - lineA[0];
	Eigen::Vector3d vectB = lineB[1] - lineB[0];
	double deltaA = (lineB[0] - lineA[0]).dot(vectA);
	double deltaB = (lineB[0] - lineA[0]).dot(vectB);
	double deno = vectA.dot(vectB) * vectB.dot(vectA) - vectA.dot(vectA) * vectB.dot(vectB);//a*d-b*c
	if (deno == 0.0) // parallel
		return { lineA[0], getNearestPointOfPointAndLine(lineA[0], lineB) };
	double kA = (vectB.dot(vectA) * deltaB - vectB.dot(vectB) * deltaA) / deno;
	double kB = (vectA.dot(vectA) * deltaB - vectA.dot(vectB) * deltaA) / deno;
	Vector3d pointA = lineA[0] + kA * vectA;
	Vector3d pointB = lineB[0] + kB * vectB;
	return { pointA,pointB };
}

Eigen::Vector3d eigen::getIntersectPointOfLineAndPlane(const std::array<Eigen::Vector3d, 2>& line, const std::array<Eigen::Vector3d, 2>& plane)
{
	Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
	Eigen::Vector3d v = line[1] - line[0];
	if (isPerpendi3d(v, normal))
		return gVecNaN; //pointOnPlane
	double k = (plane[0] - line[0]).dot(normal) / (v.dot(normal));
	return line[0] + k * v;
}

clash::Segment3d eigen::getIntersectLineOfTwoPlanes(const Vector3d& originA, const Vector3d& normalA, const Vector3d& originB, const Vector3d& normalB)
{
	Vector3d normal = normalA.cross(normalB);// .normalized();
	if (normal.isZero())
		return { gVecNaN, gVecNaN };
    if ((originA - originB).isZero())
		return { originA, normal };
	Eigen::Matrix3d matrix;//matrix << normalA, normalB, normal; //is column
	matrix.row(0) = normalA;
	matrix.row(1) = normalB;
	matrix.row(2) = normal;
	Vector3d point = matrix.inverse() * Vector3d(originA.dot(normalA), originB.dot(normalA), 0.5 * (originA + originB).dot(normal)); //Eigen inverse fast enough
    return { point, normal }; //position and vector
	//Vector3d vx = nA.cross(Vector3d(0, 1, 0));
	//Vector3d iB = getIntersectPointOfLineAndPlane(PosVec3d{ pA,vx }, planeB);
	//Vector3d iV = (iB - p).normalized();
}

clash::Segment3d eigen::getIntersectLineOfTwoPlanes(const std::array<Eigen::Vector3d, 3>& planeA, const std::array<Eigen::Vector3d, 3>& planeB)
{
	return getIntersectLineOfTwoPlanes(
		planeA[0], (planeA[1] - planeA[0]).cross(planeA[2] - planeA[0]),
		planeB[0], (planeB[1] - planeB[0]).cross(planeB[2] - planeB[0]));
}

//clash::Segment3d eigen::getIntersectLineOfTwoPlanes(const clash::Plane3d& planeA, const clash::Plane3d& planeB)
//{
//	return getIntersectLineOfTwoPlanes(planeA.m_origin, planeA.m_normal, planeB.m_origin, planeB.m_normal);
//}

static Vector3d _intersectThreePlanes(const Plane3d& plane1, const Plane3d& plane2, const Plane3d& plane3)
{
	// all 3 planes normal been normalized
	const Vector3d& crV1 = plane1.m_normal;
	const Vector3d& crV2 = plane2.m_normal;
	const Vector3d& crV3 = plane3.m_normal;
	//determinant3Vectors(crV1, crV2, crV3);
	double dDet = crV1.x() * crV2.y() * crV3.z() - crV1.x() * crV2.z() * crV3.y();
	dDet -= crV1.y() * crV2.x() * crV3.z() - crV1.y() * crV2.z() * crV3.x();
	dDet += crV1.z() * crV2.x() * crV3.y() - crV1.z() * crV2.y() * crV3.x();
	if (dDet == 0)
		return gVecNaN;
	const Vector3d& crP1 = plane1.m_origin;
	const Vector3d& crP2 = plane2.m_origin;
	const Vector3d& crP3 = plane3.m_origin;
	Vector3d sWork = 
		(crP1.dot(crV1) * (crV2.cross(crV3))) +
		(crP2.dot(crV2) * (crV3.cross(crV1))) +
		(crP3.dot(crV3) * (crV1.cross(crV2)));
	return sWork / dDet;
}

//copy from P3dGuTsect
static clash::Segment3d getIntersectLineOfTwoPlanes_P3D(const clash::Plane3d& planeA, const clash::Plane3d& planeB)
{
	const Vector3d& P1 = planeA.m_origin;
	const Vector3d& P2 = planeB.m_origin;
	const Vector3d& V1 = planeA.m_normal.normalized();
	const Vector3d& V2 = planeB.m_normal.normalized();
	Vector3d V3 = V1.cross(V2).normalized();
	Vector3d P3 = 0.5 * (P1 + P2);
	if (V3.isZero())
		return { gVecNaN, gVecNaN };
	Vector3d O3 = _intersectThreePlanes(planeA, planeB, Plane3d(P3, V3));
	return { O3,V3 };
}

// for profile section
bool clash::isTwoSegmentsCollinearCoincident(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
{
	//double operator^(const Vector2d& vec1, const Vector2d& vec2)
	auto _cross2d = [](const Vector2d& vec1, const Vector2d& vec2)->double
		{
			//return vec1.x() * vec2.y() - vec2.x() * vec1.y();
			return vec1[0] * vec2[1] - vec2[0] * vec1[1];
		};
	if (!isTwoSegmentsIntersect(segmA, segmB))
		return false;
	//return (segmA[1] - segmA[0]).cross(segmB[1] - segmB[0]).isZero(epsF);
	return fabs(_cross2d(segmA[1] - segmA[0], segmB[1] - segmB[0])) < epsF;
}

#define USING_NORMALIZED_VECTOR
bool clash::isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB,
	double toleAng /*= 0*/, double toleDis /*= 0*/)
{
	// |A×B| <= |A||B|sinθ, near zero sinθ approx θ
#ifndef USING_NORMALIZED_VECTOR
	Vector3d segmVecA = (segmA[1] - segmA[0]).normalized();
	Vector3d segmVecB = (segmB[1] - segmB[0]).normalized();
	if (!segmVecA.cross(segmVecB).isZero(toleAng)) //cross product max component is toleAngle
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
		if (segmVecA.norm() <= toleDis || segmVecB.norm() <= toleDis) // point coincident
		{
			interEnd++;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDist
		if (segmVecA.dot(segmVecB) < 0 && segmVecA.cross(segmVecB).norm() <= toleDis * (segmVecA - segmVecB).norm()) //norm fast than squaredNorm
			interEnd++;
	}
	for (const auto& endB : segmB)
	{
		segmVecA = segmA[0] - endB;
		segmVecB = segmA[1] - endB;
		if (segmVecA.norm() <= toleDis || segmVecB.norm() <= toleDis) // point conicident
		{
			interEnd++;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDist
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

std::tuple<bool, array<double, 4>> clash::getTwoSegmentsCollinearCoincidentPoints(const array<Vector3d, 2>& segmA, const array<Vector3d, 2>& segmB,
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
		if (segmVecA.norm() <= toleDis) // point coincident
		{
			endInter++; // same as segmentB 
			propB0 = 0.0;
			continue;
		}
		if (segmVecB.norm() <= toleDis) // point coincident
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
		if (segmVecA.norm() <= toleDis) // point coincident
		{
			propA0 = 0.0;
			continue;
		}
		if (segmVecB.norm() <= toleDis) // point coincident
		{
			propA1 = 1.0;
			continue;
		}
		// point on segment, opposite direction //using triangles area to judge toleDist
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

void clash::mergeIntersectIntervalOfSegment(std::vector<double>& _range, const std::array<double, 2>& prop)//->void
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

