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
//#define USING_RELATIVE_MATRIX


namespace psykronix
{
	typedef std::array<Eigen::Vector3d, 2> PosVec3d;
	static const PosVec3d gPVNaN = { gVecNaN ,gVecNaN };

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
			return gPVNaN;
		}
		//else normals not parallel
		Vector3d normal = planeA.normal().cross(planeB.normal());
#ifdef USING_RELATIVE_MATRIX
		// relative matrix //pair=500000 cost_time=81472 micro_seconds
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
#else
		// inverse matrix //pair=500000 cost_time=49825 micro_seconds
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
#endif
	}
}

//inner-core version
PosVec3d intersectWithTwoPlanes(const Plane3d& planeA, const Plane3d& planeB)
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
		return gPVNaN;
	}
	// old
	Vector3d normalA = planeA.normal().normalized();
	Vector3d normalB = planeB.normal().normalized();
	if (fabs(normalA.dot(normalA) - 1 > eps) ||
		fabs(normalB.dot(normalB) - 1 > eps))
		return gPVNaN;
	Vector3d normalC = normalA.cross(normalB);
	if (normalC.squaredNorm() < DBL_EPSILON)
		return gPVNaN;
	normalC.normalize(); //norm self
	Vector3d midlle = 0.5 * (planeA.origin() + planeB.origin());
	// intersectThreePlanes
	Vector3d origin;
	if (fabs(normalA.dot(normalA) - 1 > eps) ||
		fabs(normalB.dot(normalB) - 1 > eps) ||
		fabs(normalC.dot(normalC) - 1 > eps))
		return gPVNaN;
	auto _determinant3vectors = [](const Vector3d& v1, const Vector3d& v2, const Vector3d& v3)->double
	{
		double det = v1.x() * v2.y() * v3.z() - v1.x() * v2.z() * v3.y();
				det -= v1.y() * v2.x() * v3.z() - v1.y() * v2.z() * v3.x();
				det += v1.z() * v2.x() * v3.y() - v1.z() * v2.y() * v3.x();
		return det;
	};
	double det = _determinant3vectors(normalA, normalB, normalC);
	if (fabs(det) < eps)
		return gPVNaN;
	Vector3d work = normalA.dot(normalA) * normalB.cross(normalC) +
					normalB.dot(normalB) * normalC.cross(normalA) +
					normalC.dot(normalC) * normalA.cross(normalB);
	origin = 1 / det * work;
	return { origin, normalC };
}

static void test1()
{

}

static void test2()
{
	size_t N = N_10E_6;
	double* arr = _readBinFileAlignedBox(N); //读取随机数据
	std::vector<Plane3d> planeVct;// (N);
	for (int i = 0; i < N; i++)
	{
		double min_x = arr[6 * i + 0];
		double min_y = arr[6 * i + 1];
		double min_z = arr[6 * i + 2];
		double max_x = arr[6 * i + 3];
		double max_y = arr[6 * i + 4];
		double max_z = arr[6 * i + 5];
		planeVct.emplace_back(Plane3d(Vector3d(min_x, min_y, min_z), Vector3d(max_x, max_y, max_z)));
	}
	steady_clock::time_point start, end;
	microseconds duration;
	vector<PosVec3d> resSeg;
	PosVec3d res;
	for (int i = 0; i < 3; i++)
	{
		start = std::chrono::high_resolution_clock::now();
		for (int j = 0; j < N / 2; j++)
		{
			//resSeg.push_back(getIntersectLineOfTwoPlane(planeVct[2 * j], planeVct[2 * j + 1]));
			res = intersectWithTwoPlanes(planeVct[2 * j], planeVct[2 * j + 1]);
			if (!isNaN(res[0]))
				resSeg.push_back(res);
		}
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<microseconds>(end - start);
		cout << "pair=" << N / 2 << " cost_time=" << duration.count() << " micro_seconds" << endl;
		cout << "res_pair=" << resSeg.size() << endl;
		resSeg.clear();
	}

	return;
}

static void test3() //测试getIntersectLineOfTwoPlane
{
	Plane3d plane1(Vector3d(0, 20, 0), Vector3d(-1, 0, 0)); //可以共线
	Plane3d plane2(Vector3d(0, -10, 0), Vector3d(0, 0, 1));
	PosVec3d interLine = getIntersectLineOfTwoPlane(plane1, plane2);

	plane1 = Plane3d{ Vector3d(10, 20, 3), Vector3d(1, 2, 3) };
	plane2 = Plane3d{ Vector3d(10, 20, 3), Vector3d(1, 2, 3) };
	interLine = getIntersectLineOfTwoPlane(plane1, plane2);

	return;
}

static int enrol = []()->int
{
	//test0();
	//test1();
	test2();
	test3();
	cout << "test_geometry finished.\n" << endl;
	return 0;
}();

