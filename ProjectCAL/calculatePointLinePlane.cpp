#include "pch.h"
#include "calculatePointLinePlane.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;

namespace eigen
{
	//calculate distance
	double getDistanceOfPointAndLine(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& line)
	{
		Vector3d vecSeg = line[1] - line[0];// not zero
		if (vecSeg.isZero())
			return (line[0] - point).norm();
		double k = vecSeg.dot(point - line[0]) / vecSeg.dot(vecSeg);
		Vector3d local = point + k * vecSeg;
		return (line[0] - local).norm();
	}

	double getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const clash::Plane3d& plane)
	{
		if (plane.m_normal.isZero()) // error triangle plane
            return (point - plane.m_origin).norm();
		Vector3d normal = plane.m_normal.normalized();
  //      double k = (point - plane.m_origin).dot(normal) / normal.dot(normal); //normal without normalize
  //      //Vector3d local = point + k * normal;
		//return fabs(k) * normal.norm(); // (local - point).norm();
		double dotPro = (point - plane.m_origin).dot(normal);
		return fabs(dotPro);
	}

	double getDistanceOfPointAndPlane(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& plane)
	{
		return getDistanceOfPointAndPlane(point, Plane3d(plane));
	}

	double getDistanceOfTwoLines(const std::array<Eigen::Vector3d, 2>& lineA, const std::array<Eigen::Vector3d, 2>& lineB)
	{
		Eigen::Vector3d vectA = lineA[1] - lineA[0];
		Eigen::Vector3d vectB = lineB[1] - lineB[0];
		double delta1 = (lineB[0] - lineA[0]).dot(vectA);
		double delta2 = (lineB[0] - lineA[0]).dot(vectB);
		double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
		if (deno == 0.0) // parallel
			return getDistanceOfPointAndLine(lineA[0], lineB);
		double kA = 1.0 / deno * (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2);
		double kB = 1.0 / deno * (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2);
		Vector3d pointA = lineA[0] + kA * vectA;
		Vector3d pointB = lineB[0] + kB * vectB;
		return (pointB - pointA).norm();
	}

	//calculate intersect
	Eigen::Vector3d getIntersectPointOfLineAndPlane(const std::array<Eigen::Vector3d, 2>& line, const std::array<Eigen::Vector3d, 2>& plane)
	{
		Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
		Eigen::Vector3d v = line[1] - line[0];
		if (isPerpendi3d(v, normal))
			return gVecNaN; //pointOnPlane
		double k = (plane[0] - line[0]).dot(normal) / (v.dot(normal));
		return line[0] + k * v;
	}

	Eigen::Vector2d getIntersectPointOfTwoLines(const std::array<Eigen::Vector2d, 2>& lineA, const std::array<Eigen::Vector2d, 2>& lineB)
	{
		Eigen::Vector2d vecA = lineA[1] - lineA[0];
		Eigen::Vector2d vecB = lineB[1] - lineB[0];
        if (isParallel2d(vecA, vecB))
			return gVecNaN2d; //lines collinear
		double k = cross2d(lineB[0] - lineA[0], vecB) / cross2d(vecA, vecB);
		return lineA[0] + k * vecA;
	}

	clash::Segment3d getIntersectLineOfTwoPlanes(const clash::Plane3d& planeA, const clash::Plane3d& planeB)
	{
		const Vector3d& pA = planeA.m_origin;
		const Vector3d& pB = planeB.m_origin;
		const Vector3d& nA = planeA.m_normal;
		const Vector3d& nB = planeB.m_normal;
		Vector3d v = nA.cross(nB).normalized();
		if (v.isZero())
			return { gVecNaN, gVecNaN };
		Eigen::Matrix3d matrix;
		matrix << nA, nB, v;
		Vector3d p = matrix.inverse() * Vector3d(pA.dot(nA), pB.dot(nB), 0.5 * (pA + pB).dot(v));
        return { p, p + v };
		//Vector3d vx = nA.cross(Vector3d(0, 1, 0));
		//Vector3d iB = getIntersectPointOfLineAndPlane(PosVec3d{ pA,vx }, planeB);
		//Vector3d iV = (iB - p).normalized();
	}

	Vector3d _intersectThreePlanes(const Plane3d& plane1, const Plane3d& plane2, const Plane3d& plane3)
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
	clash::Segment3d getIntersectLineOfTwoPlanesP3D(const clash::Plane3d& planeA, const clash::Plane3d& planeB)
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

	clash::Segment3d getIntersectLineOfTwoPlanes(const std::array<Eigen::Vector3d, 3>& planeA, const std::array<Eigen::Vector3d, 3>& planeB)
	{
		return getIntersectLineOfTwoPlanes(Plane3d(planeA), Plane3d(planeB));
	}

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
	// |A¡ÁB| <= |A||B|sin¦È, near zero sin¦È approx ¦È
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

