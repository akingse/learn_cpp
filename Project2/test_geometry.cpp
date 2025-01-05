#include "pch.h"
using namespace std;
using namespace clash;
using namespace Eigen;
using namespace eigen;
using namespace accura;
using namespace std::chrono;
#ifdef min
#undef min
#endif // 
#ifdef max
#undef max
#endif // 
//#define USING_RELATIVE_MATRIX

namespace clash
{
	//��ƽ����
	PosVec3d _getIntersectLineOfTwoPlane(const Plane3d& planeA, const Plane3d& planeB)
	{
		if (isParallel3d(planeA.normal(), planeB.normal()))
		{
			//is_point_on_plane
			if (isPerpendi3d(planeA.normal(), planeB.origin() - planeA.origin()))
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
	if (isParallel3d(planeA.normal(), planeB.normal()))
	{
		//is_point_on_plane
		if (isPerpendi3d(planeA.normal(), planeB.origin() - planeA.origin()))
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
	if (fabs(normalA.dot(normalA) - 1 > epsF) ||
		fabs(normalB.dot(normalB) - 1 > epsF))
		return gPVNaN;
	Vector3d normalC = normalA.cross(normalB);
	if (normalC.squaredNorm() < DBL_EPSILON)
		return gPVNaN;
	normalC.normalize(); //norm self
	Vector3d midlle = 0.5 * (planeA.origin() + planeB.origin());
	// intersectThreePlanes
	Vector3d origin;
	if (fabs(normalA.dot(normalA) - 1 > epsF) ||
		fabs(normalB.dot(normalB) - 1 > epsF) ||
		fabs(normalC.dot(normalC) - 1 > epsF))
		return gPVNaN;
	auto _determinant3vectors = [](const Vector3d& v1, const Vector3d& v2, const Vector3d& v3)->double
	{
		double det = v1.x() * v2.y() * v3.z() - v1.x() * v2.z() * v3.y();
		det -= v1.y() * v2.x() * v3.z() - v1.y() * v2.z() * v3.x();
		det += v1.z() * v2.x() * v3.y() - v1.z() * v2.y() * v3.x();
		return det;
	};
	double det = _determinant3vectors(normalA, normalB, normalC);
	if (fabs(det) < epsF)
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

//Ч�ʲ���
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
	Segment3d res;
	for (int i = 0; i < 3; i++)
	{
		start = std::chrono::high_resolution_clock::now();
		for (int j = 0; j < N / 2; j++)
		{
			//res = intersectWithTwoPlanes(planeVct[2 * j], planeVct[2 * j + 1]);
			res = getIntersectLineOfTwoPlanes(planeVct[2 * j], planeVct[2 * j + 1]); //29905 faster
			//res = getIntersectLineOfTwoPlanesP3D(planeVct[2 * j], planeVct[2 * j + 1]); //53539
			//if (!isnan(res[0][0]))
			//	resSeg.push_back(res);
		}
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<microseconds>(end - start);
        cout << dec << "pair=" << N / 2 << " cost_time=" << double(duration.count()) << " micro_seconds" << endl;
		//cout << "res_pair=" << resSeg.size() << endl;
		resSeg.clear();
	}

	return;
}

static void test3() //����getIntersectLineOfTwoPlane
{
	Plane3d plane1(Vector3d(0, 10, 0), Vector3d(-1, 0, 0)); //���Թ���
	Plane3d plane2(Vector3d(0, -10, 0), Vector3d(0, 0, 1));
	PosVec3d interLine = getIntersectLineOfTwoPlanes(plane1, plane2);

	plane1 = Plane3d{ Vector3d(10, 20, 3), Vector3d(1, 2, 3) };
	plane2 = Plane3d{ Vector3d(10, 20, 3), Vector3d(1, 2, 3) };
	interLine = getIntersectLineOfTwoPlanes(plane1, plane2);

	return;
}

static void test4()
{
	//Eigen::Vector2d point(1-1e-8,0);// (0, 0);
	//Eigen::Vector2d point(-2, -1); //����
	Eigen::Vector2d point(0, 0); 
	vector<Eigen::Vector2d> polygon = { Vector2d(1,1),Vector2d(-1,1),Vector2d(-1,-1),Vector2d(1,-1) };
	//polygon.push_back(polygon.front());
	//polygon.push_back(Vector2d(2, -1));
	bool isin = isPointInPolygon2D(point, polygon);

	Plane3d plane(Vector3d(20, 20, 0), Vector3d(2, 1, 0));

	Matrix4d mat = getProjectionMatrixByPlane(plane);
	cout << mat << endl;

	//�����������ڲ�
	array<Eigen::Vector2d, 3> trigon = { Vector2d(-10, -10), Vector2d(30, 0), Vector2d(1, 10) };
	trigon = { Vector2d(-10, -10), Vector2d(1, 10), Vector2d(30, 0) };

    bool inter = isPointInTriangle(Vector2d(0, 0) + Vector2d(1000, 1000), to_vec2(translate(1000, 1000) * trigon), 0);
	//tole�ͱ�Ե
	trigon = { Vector2d(-10, 0), Vector2d(1, 10), Vector2d(30, 0) };
	trigon = { Vector2d(0, 0), Vector2d(10, 0), Vector2d(0, 10) };
	//tole < 0 more judge
	bool inter0 = isPointInTriangle(point, trigon, 0);
	bool interLess = isPointInTriangle(point, trigon, epsF);
	bool interMore = isPointInTriangle(point, trigon, -epsF);

	return;
}

//���Ծ�������
static void test5()
{
	//�������ϵ��Ӱ�� //����ϵ����Ӱ��
	Segment segmA = { Vector3d(300,0,0), Vector3d(0,101,0) };
	Segment segmB = { Vector3d(0,0,0), Vector3d(100,102,0) };
	//segmA = eigen::translate(12345678, 12345678, 0) * segmA;
	//segmB = eigen::translate(12345678, 12345678, 0) * segmB;
	Eigen::Vector3d inter = clash::getTwoSegmentsIntersectPoint(segmA, segmB);
	Vector3d error = (inter - segmA[0]).cross(inter - segmA[1]);
	//9.0949470177292824e-13
	//-1.4156103134155273e-07

	Vector3d v1(100, -1000, 200);
	double m1 = v1.maxCoeff();
	//v1.cwiseMax(v2) //merge two vector, to new one

	Segment2d segm2A = { Vector2d(0,0), Vector2d(100,0) };
	//Segment2d segm2B = { Vector2d(50,0), Vector2d(0,50) };
	Segment2d segm2B = { Vector2d(100-epsF,0), Vector2d(200,0) }; //test tole
	bool isinter = isTwoSegmentsIntersect(segm2A, segm2B, 0); //tole>0 more judge

	//3D
	Matrix4d mat = rotate(Vector3d(3.21, 0, 0), Vector3d(2, 1, 0), 1);
	Segment3d segm3A = { Vector3d(0,0,0), Vector3d(100,100,0) };
	Segment3d segm3B = { Vector3d(100.21,0,0), Vector3d(0,100.8,0) }; //test tole
	bool isinter3 = isTwoSegmentsIntersect(mat*segm3A, mat*segm3B); //tole>0 more judge

	return;
}

//���� ���潻��
static void test6()
{
	clash::Plane3d planeA = Plane3d(Vector3d(0, 0, 0), Vector3d(0, 0, 1));
	clash::Plane3d planeB = Plane3d(Vector3d(0, 0, 0), Vector3d(1, 1, 0));

	Segment3d interLine1 = getIntersectLineOfTwoPlanes(planeA, planeB);
	Segment3d interLine2 = getIntersectLineOfTwoPlanesP3D(planeA, planeB);

	return;
}

inline bool GetIntersectPoint(const Vector2i& ln1a, const Vector2i& ln1b,
	const Vector2i& ln2a, const Vector2i& ln2b, Vector2i& ip) //intersect point
{
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	double dx1 = double(ln1b.x() - ln1a.x());
	double dy1 = double(ln1b.y() - ln1a.y());
	double dx2 = double(ln2b.x() - ln2a.x());
	double dy2 = double(ln2b.y() - ln2a.y());
	double det = dy1 * dx2 - dy2 * dx1;
	if (det == 0.0) 
		return false;
	double t = ((ln1a.x() - ln2a.x()) * dy2 - (ln1a.y() - ln2a.y()) * dx2) / det;
	if (t <= 0.0) 
		ip = ln1a;        // ?? check further (see also #568)
	else if (t >= 1.0) 
		ip = ln1b;   // ?? check further
	else
	{
		ip.x() = int64_t(ln1a.x() + t * dx1);
		ip.y() = int64_t(ln1a.y() + t * dy1);
	}
	return true;
}

static void test7()
{
	//����
	vector<vector<Eigen::Vector2i>> poly0 = { {
		Vector2i(0,0),
		Vector2i(10,5),
		Vector2i(0,10),
		} };
	vector<vector<Eigen::Vector2i>> poly1 = { {
		Vector2i(5,0),
		Vector2i(5,10),
		//Vector2i(-5,5),
		Vector2i(-5,-1), //without intersect
		} };

	Vector2i ip;
	bool isinter = GetIntersectPoint(poly0[0][0], poly0[0][1], poly1[0][0], poly1[0][2], ip);

	return;
}

static int enrol = []()->int
{
	//test1();
	test2();
	//test3();
	//test4();
	//test5();
	//test6();
	cout << "test_geometry finished.\n" << endl;
	return 0;
}();

