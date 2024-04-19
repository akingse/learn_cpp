#pragma once

namespace clash
{
	// vector
	//inline bool isNaN(const Eigen::Vector3d& vec)
	//{
	//	return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
	//}
	// 
	//inline in common use
	inline bool isPointOnLine(const Eigen::Vector3d& point, const Segment& line)
	{
		return eigen::isParallel3d(point - line[0], point - line[1]);
	}
	inline bool isPointOnPlane(const Eigen::Vector3d& point, const Plane3d& plane)
	{
		return eigen::isPerpendi3d(point - plane.m_origin, plane.m_normal);
	}
}

//segment
namespace clash
{
	using namespace Eigen;

	//isTwoSegmentsIntersect2D
	inline bool isTwoSegmentsIntersect(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
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
			((segmA[1] - segmA[0]).x() * (segmB[1] - segmA[0]).y() - (segmB[1] - segmA[0]).x() * (segmA[1] - segmA[0]).y()) > _epsF;
		bool isBetweenA = ((segmA[0] - segmB[0]).x() * (segmB[1] - segmB[0]).y() - (segmB[1] - segmB[0]).x() * (segmA[0] - segmB[0]).y()) *
			((segmB[1] - segmB[0]).x() * (segmA[1] - segmB[0]).y() - (segmA[1] - segmB[0]).x() * (segmB[1] - segmB[0]).y()) > _epsF;
		return isBetweenB && isBetweenA;
#else
		//Vector2d vecA = segmA[1] - segmA[0];
		//Vector2d AB_0 = segmB[0] - segmA[0];
		//Vector2d AB_1 = segmB[1] - segmA[0];
		//Vector2d vecB = segmB[1] - segmB[0];
		//Vector2d BA_0 = segmA[0] - segmB[0];
		//Vector2d BA_1 = segmA[1] - segmB[0];
		//return
		//	(AB_0[0] * vecA[1] - AB_0[1] * vecA[0]) * (AB_1[0] * vecA[1] - AB_1[1] * vecA[0]) <= 0.0 &&
		//	(BA_0[0] * vecB[1] - BA_0[1] * vecB[0]) * (BA_1[0] * vecB[1] - BA_1[1] * vecB[0]) <= 0.0;
		return // double point on line judge
			((segmB[0] - segmA[0])[0] * (segmA[1] - segmA[0])[1] - (segmA[1] - segmA[0])[0] * (segmB[0] - segmA[0])[1]) *
			((segmA[1] - segmA[0])[0] * (segmB[1] - segmA[0])[1] - (segmB[1] - segmA[0])[0] * (segmA[1] - segmA[0])[1]) >= 0.0 &&
			((segmA[0] - segmB[0])[0] * (segmB[1] - segmB[0])[1] - (segmB[1] - segmB[0])[0] * (segmA[0] - segmB[0])[1]) *
			((segmB[1] - segmB[0])[0] * (segmA[1] - segmB[0])[1] - (segmA[1] - segmB[0])[0] * (segmB[1] - segmB[0])[1]) >= 0.0;
#endif
	}

	// must coplanar
	inline bool isTwoSegmentsIntersect(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
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
			(segmB[0] - segmA[0]).cross(segmA[1] - segmA[0]).dot((segmA[1] - segmA[0]).cross(segmB[1] - segmA[0])) < _epsF || //double separate
			(segmA[0] - segmB[0]).cross(segmB[1] - segmB[0]).dot((segmB[1] - segmB[0]).cross(segmA[1] - segmB[0])) < _epsF);
#else
		return //double straddling test
			0.0 <= (segmB[0] - segmA[0]).cross(segmA[1] - segmA[0]).dot((segmA[1] - segmA[0]).cross(segmB[1] - segmA[0])) &&
			0.0 <= (segmA[0] - segmB[0]).cross(segmB[1] - segmB[0]).dot((segmB[1] - segmB[0]).cross(segmA[1] - segmB[0]));
#endif
	}

	inline Eigen::Vector2d getTwoSegmentsIntersectPoint(const std::array<Vector2d, 2>& segmA, const std::array<Vector2d, 2>& segmB)
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

	inline Eigen::Vector3d getTwoSegmentsIntersectPoint(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
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
	inline Vector3d getIntersectOfPointAndLine(const Vector3d& point, const std::array<Vector3d, 2>& segm)
	{
		Eigen::Vector3d direction = (segm[1] - segm[0]);// not zero
		double projection = direction.dot(point);
		//the projection must on segment
		if (direction.dot(segm[1]) < projection || projection < direction.dot(segm[0]))
			return gVecNaN;// DBL_MAX;
		double k = direction.dot(point - segm[0]) / direction.dot(direction);
		return (segm[0] - point + k * direction);//.squaredNorm();
	}

	inline Vector3d getIntersectOfTwoSegments(const std::array<Vector3d, 2>& segmA, const std::array<Vector3d, 2>& segmB)
	{
		Vector3d vectA = segmA[1] - segmA[0];
		Vector3d vectB = segmB[1] - segmB[0];
		double delta1 = (segmB[0] - segmA[0]).dot(vectA);
		double delta2 = (segmB[0] - segmA[0]).dot(vectB);
		// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
		double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
		if (deno == 0.0) // parallel, must exclude, than distance of point to segment in next function
			return gVecNaN;// DBL_MAX;
		double kA = 1 / deno * (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2);
		double kB = 1 / deno * (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2);
		//	Vector3d pointA = segmA[0] + kA * vectA;
		//	Vector3d pointB = segmB[0] + kB * vectB;
		//whether two intersect-point inside segments
		if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
			return (segmA[0] + kA * vectA - segmB[0] - kB * vectB);//.squaredNorm();
		return gVecNaN;// DBL_MAX; // nearest point outof segments
	}
	
	inline Vector3d getIntersectOfPointAndPlane(const Vector3d& point, const std::array<Vector3d, 3>& plane);

	//inline Eigen::Vector3d getIntersectPointOfLineAndPlane(const Segment& line, const Plane3d& plane)
	//{
	//	Eigen::Vector3d v = line[1] - line[0];
	//	double k = (plane.m_origin - line[0]).dot(plane.m_normal) / (v.dot(plane.m_normal));
	//	return line[0] + k * v;
	//}
	
	inline Eigen::Vector3d getIntersectPointOfLineAndPlane(const PosVec3d& ray, const Plane3d& plane)
	{
		double k = (plane.m_origin - ray[0]).dot(plane.m_normal) / (ray[1].dot(plane.m_normal));
		return ray[0] + k * ray[1];
	}

	inline Segment getIntersectLineOfTwoPlane(const Plane3d& planeA, const Plane3d& planeB)
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
		return { p,p + v };
		Vector3d vx = nA.cross(Vector3d(0, 1, 0));
		Vector3d iB = getIntersectPointOfLineAndPlane(PosVec3d{ pA,vx }, planeB);
		Vector3d iV = (iB - p).normalized();
	}


}

//plane
namespace clash
{

}

