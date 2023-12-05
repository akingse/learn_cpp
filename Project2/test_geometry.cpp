#include "pch.h"
using namespace std;
using namespace psykronix;
using namespace Eigen;
using namespace std::chrono;
#ifdef min
#undef min
#endif // 
#ifdef max
#undef max
#endif // 

namespace psykronix
{
	typedef std::array<Eigen::Vector3d, 2> PosVec3d;

	bool isParallel(const Vector3d& vecA, const Vector3d& vecB/*, double tole = 0*/)
	{
		// dynamic accuracy
		double tole = DBL_EPSILON * std::max(vecA.squaredNorm(), vecB.squaredNorm() + 1);
		return vecA.cross(vecB).isZero(tole);
	}

	bool isPerpendi(const Vector3d& vecA, const Vector3d& vecB/*, double tole = 0*/)
	{
		// dynamic accuracy
		double tole = DBL_EPSILON * std::max(vecA.squaredNorm(), vecB.squaredNorm() + 1); //avoid all zero
		return vecA.dot(vecB) < tole;
	}

	class Plane3d
	{
	public:
		Vector3d m_origin;
		Vector3d m_normal;
		Plane3d()
		{
			m_origin = Vector3d(0, 0, 0);
			m_normal = Vector3d(0, 0, 1);
		}
		Plane3d(const Vector3d& origin, const Vector3d& normal)
		{
			m_origin = origin;
			if (normal.isApprox(Vector3d::Zero(), 0))
				m_normal = gVecNaN;
			else
				m_normal = normal;
		}
		const Vector3d& origin() const
		{
			return m_origin;
		}
		const Vector3d& normal() const
		{
			return m_normal;
		}

	};

	//两平面求交
	PosVec3d getIntersectLineOfTwoPlane(const Plane3d& planeA, const Plane3d& planeB)
	{
		if (isParallel(planeA.normal(), planeB.normal()))
		{
			//is_point_on_plane
			if (isPerpendi(planeA.normal(), planeB.origin() - planeA.origin()))
			{
				if (planeA.normal().isApprox(Vector3d(0, 0, 1)))
					return { planeA.origin(), Vector3d(1, 0, 0) }; //xoy plane
				return { planeA.origin(), planeA.normal().cross(Vector3d(0, 0, 1)) }; //on xoy plane
			}
			return { gVecNaN ,gVecNaN };
		}
		//else normals not parallel
		Vector3d normal = planeA.normal().cross(planeB.normal());
#ifdef using_relative_matrix
		// inverse matrix
		Vector3d B(planeA.origin().dot(planeA.normal()),
			planeB.origin().dot(planeB.normal()),
			0.5 * (planeA.origin() + planeB.origin()).dot(normal));//Ax=B
		Eigen::Matrix3d matrix;
		//matrix.row(0) << planeA.normal()[0], planeA.normal()[1], planeA.normal()[2];
		//matrix.row(1) << planeB.normal()[0], planeB.normal()[1], planeB.normal()[2];
		//matrix.row(2) << normal[0], normal[1], normal[2];
		matrix << planeA.normal()[0], planeB.normal()[0], normal[0],
			planeA.normal()[1], planeB.normal()[1], normal[1],
			planeA.normal()[2], planeB.normal()[2], normal[2];
		Vector3d origin = matrix.inverse() * B;
		return { origin, normal };
#else
		// relative matrix 
		Vector3d mz = planeA.normal().normalized();
		Vector3d my = normal.normalized();
		Vector3d mx = my.cross(mz);
		Eigen::Matrix3d matrix;
		matrix << mx[0], my[0], mz[0],
			mx[1], my[1], mz[1],
			mx[2], my[2], mz[2];
		Eigen::Matrix3d matInv = matrix.transpose();
		Vector3d relaBo = matInv * planeB.origin();
		Vector3d relaBn = matInv * planeB.normal();
		Vector3d origin(relaBo[2] * relaBn[0] / relaBn[2] + relaBo[0], relaBo[1], 0);
		return { matrix * origin, normal };
#endif
	}
}

//inner-core version
static void test1()
{

}
static void test2()
{

}

static void test3() //测试getIntersectLineOfTwoPlane
{
	Plane3d plane1(Vector3d(10, 20, 3), Vector3d(1, 2, 3));
	Plane3d plane2(Vector3d(30, 10, -3), Vector3d(4, 5, 6));
	auto res = getIntersectLineOfTwoPlane(plane1, plane2);

	return;
}
