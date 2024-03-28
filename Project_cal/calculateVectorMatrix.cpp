#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;

Eigen::Matrix4d eigen::getProjectionMatrixByPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
{
	//get rot matrix from one vector
	const Vector3d& axisz = normal.normalized();
	Vector3d axisx = Vector3d(0, 0, 1).cross(axisz).normalized();
	//get the point on normal that through world origin
	double o_n = point.dot(normal);
	Vector3d origin = (o_n == 0.0) ? //correct plane's origin point
		point : o_n / normal.squaredNorm() * normal;
	if (axisx.isZero())
	{
		Matrix4d projection; // projection
		projection <<
			1, 0, 0, origin[0],
			0, 1, 0, origin[1],
			0, 0, 0, origin[2],
			0, 0, 0, 1;
		return projection;
	}
	Vector3d axisy = axisz.cross(axisx);
	Matrix4d matOri; //unit and orth
	matOri <<
		axisx[0], axisy[0], axisz[0], origin[0],
		axisx[1], axisy[1], axisz[1], origin[1],
		axisx[2], axisy[2], axisz[2], origin[2],
		0, 0, 0, 1;
	Matrix4d invPro; //inverse projection
	invPro << // shadow_xoy * inverse
		axisx[0], axisx[1], axisx[2], 0,
		axisy[0], axisy[1], axisy[2], 0,
		0, 0, 0, 0, // dimension reduction
		0, 0, 0, 1;
	return matOri * invPro;
}

Matrix4d eigen::getProjectionMatrixByPlane(const Plane3d& plane)
{
	return getProjectionMatrixByPlane(plane.m_origin, plane.m_normal);
}

std::array<Eigen::Matrix4d, 2> eigen::getRelativeMatrixByProjectionPlane(const Eigen::Vector3d& origin, const Eigen::Vector3d& normal)
{
	Vector3d axisz = normal.normalized();
	Vector3d axisx = Vector3d(0, 0, 1).cross(axisz).normalized();
	if (axisx.isZero())
		return { Matrix4d::Identity(), Matrix4d::Identity() };
	Vector3d axisy = axisz.cross(axisx);
	Matrix4d matFor, matInv;// = Eigen::Affine3d::Identity();
	//mat.setByOriginAndVectors();
	matFor << //forword matrix
		axisx[0], axisy[0], axisz[0], origin[0],
		axisx[1], axisy[1], axisz[1], origin[1],
		axisx[2], axisy[2], axisz[2], origin[2],
		0, 0, 0, 1;
	matInv << //inverse matrix, transpose and negation
		axisx[0], axisx[1], axisx[2], -origin[0],
		axisy[0], axisy[1], axisy[2], -origin[1],
		axisz[0], axisz[1], axisz[2], -origin[2],
		0, 0, 0, 1;
	return { matFor,matInv };
}

std::array<Eigen::Matrix4d, 2> eigen::getRelativeMatrixByProjectionPlane(const Plane3d& plane)
{
	return getRelativeMatrixByProjectionPlane(plane.m_origin, plane.m_normal);
}
