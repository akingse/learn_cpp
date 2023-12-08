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
	//��ƽ����
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
		//Eigen::Matrix3d matrix;
		////matrix.row(0) << planeA.normal()[0], planeA.normal()[1], planeA.normal()[2];
		////matrix.row(1) << planeB.normal()[0], planeB.normal()[1], planeB.normal()[2];
		////matrix.row(2) << normal[0], normal[1], normal[2];
		//matrix << planeA.normal()[0], planeB.normal()[0], normal[0],
		//		planeA.normal()[1], planeB.normal()[1], normal[1],
		//		planeA.normal()[2], planeB.normal()[2], normal[2];
		//Vector3d origin = matrix.inverse() * B;
		// manual write matrix inverse
		auto _inverse3x3 = [&](const Vector3d& v1, const Vector3d& v2, const Vector3d& v3)->Vector3d
		{
			double det = v1.x() * v2.y() * v3.z() - v1.x() * v2.z() * v3.y();
			det -= v1.y() * v2.x() * v3.z() - v1.y() * v2.z() * v3.x();
			det += v1.z() * v2.x() * v3.y() - v1.z() * v2.y() * v3.x();
			//AdjugateMatrix �������
			Vector3d row0(v2[1] * v3[2] - v3[1] * v2[2], -(v2[1] * v3[2] - v2[2] * v3[1]), v2[1] * v3[2] - v2[2] * v3[1]);
			Vector3d row1(-(v2[0] * v3[2] - v2[2] * v3[1]), v2[1] * v3[2] - v2[2] * v3[1], -(v2[1] * v3[2] - v2[2] * v3[1]));
			Vector3d row2(v2[0] * v3[2] - v2[2] * v3[1], -(v2[1] * v3[2] - v2[2] * v3[1]), v2[1] * v3[2] - v2[2] * v3[1]);
			return 1 / det * Vector3d(row0.dot(B), row1.dot(B), row2.dot(B));
		};
		Vector3d origin = _inverse3x3(planeA.normal(), planeB.normal(), normal);
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
	//����forѭ��������������ܵ�Ӱ��
	size_t N = N_10E_6;
	double* arr = _readBinFileAlignedBox(N);

	steady_clock::time_point start, end;
	microseconds duration;
	vector<AlignedBox3d> boxVct;

	for (int i = 0; i < N; i++)
	{
		double min_x = arr[6 * i + 0];
		double min_y = arr[6 * i + 1];
		double min_z = arr[6 * i + 2];
		double max_x = arr[6 * i + 3];
		double max_y = arr[6 * i + 4];
		double max_z = arr[6 * i + 5];
		AlignedBox3d box = { Vector3d(min_x,min_y,min_z), Vector3d(max_x,max_y,max_z) };
		boxVct.emplace_back(box);
	}
	for (size_t t = 0; t < 3; t++)
	{
		start = std::chrono::high_resolution_clock::now();
		vector<int> interVct;
		vector<Vector3d> corVct; //Ч����һ���ģ�д�������׵��ԣ����ǻ�������������
		Vector3d cropro;
		Triangle tri = { Vector3d(200,300,400), Vector3d(1200,300,400), Vector3d(200,300,1400) };
		for (int i = 0; i < N; i++)
		{
			//AlignedBox3d tmp;
			//Vector3d cropro = boxVct[i].min().cross(boxVct[i].max()); //time = 2346
			//cropro = boxVct[i].min().cross(boxVct[i].max()); //time = 2342
			//corVct.push_back(cropro); //time=72736
			//corVct.emplace_back(cropro); //time=71642

			bool isI = isTriangleAndBoundingBoxIntersectSAT(tri, boxVct[i]); // time=11022
		}
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<microseconds>(end - start);
		cout << " time=" << duration.count() + duration.count() << " micro_seconds" << endl;
	}

}

static void test2()
{
	size_t N = N_10E_6;
	double* arr = _readBinFileAlignedBox(N); //��ȡ�������
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
			res = getIntersectLineOfTwoPlane(planeVct[2 * j], planeVct[2 * j + 1]);
			//res = intersectWithTwoPlanes(planeVct[2 * j], planeVct[2 * j + 1]);
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

static void test3() //����getIntersectLineOfTwoPlane
{
	Plane3d plane1(Vector3d(0, 10, 0), Vector3d(-1, 0, 0)); //���Թ���
	Plane3d plane2(Vector3d(0, -10, 0), Vector3d(0, 0, 1));
	PosVec3d interLine = getIntersectLineOfTwoPlane(plane1, plane2);

	plane1 = Plane3d{ Vector3d(10, 20, 3), Vector3d(1, 2, 3) };
	plane2 = Plane3d{ Vector3d(10, 20, 3), Vector3d(1, 2, 3) };
	interLine = getIntersectLineOfTwoPlane(plane1, plane2);

	return;
}

static int enrol = []()->int
{
	test1();
	//test2();
	//test3();
	cout << "test_geometry finished.\n" << endl;
	return 0;
}();
