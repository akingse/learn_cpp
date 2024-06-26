#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;

Vector3d clash::getIntersectOfPointAndPlane(const Vector3d& point, const std::array<Vector3d, 3>& plane)
{
	Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
	//if (normal.isZero()) // error triangle plane
	//	return DBL_MAX;
	double k = (plane[0] - point).dot(normal) / normal.dot(normal);
	Vector3d local = point + k * normal;
	if (!isPointInTriangle(local, plane))
		return gVecNaN;// DBL_MAX;
	return local;//(k * normal).squaredNorm();
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

//bool clash::isTwoSegmentsCollinearCoincident(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
//{
//	if (!isTwoSegmentsIntersect(segmA, segmB))
//		return false;
//	return (segmA[1] - segmA[0]).cross(segmB[1] - segmB[0]).isZero(epsF);
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
bool clash::isTwoSegmentsCollinearCoincident(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB,
	double toleAng /*= 0*/, double toleDis /*= 0*/)
{
	// |A��B| <= |A||B|sin��, near zero sin�� approx ��
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

