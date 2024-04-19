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