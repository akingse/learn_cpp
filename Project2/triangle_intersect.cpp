#include "pch.h"
#include <iomanip>
#include <calculateTriangle.h>
#include <my_class_fun.h>
using namespace std;
using namespace psykronix;
using namespace std;
using namespace para;
using namespace Eigen;
using namespace psykronix;
#undef max
#undef min
static constexpr double eps = 1e-14; //DBL_EPSILON
static constexpr double _eps = -eps;

static Eigen::Vector3d P(std::nan("0"), std::nan("0"), std::nan("0"));
static Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));


class TriangleVec
{
public:
	Vec3 m_p0;
	Vec3 m_p1;
	Vec3 m_p2;
};
#pragma once
//GYDevillersTriangle.h
/*      ���ټ��ռ��������ཻ�㷨�Ĵ���ʵ��(Devillers & Guigue�㷨)
����ԭ��ַ:http://blog.csdn.net/fourierfeng/article/details/11969915#

Devillers & Guigue�㷨(���Devillers �㷨) ͨ�������θ����㹹�ɵ�����ʽ�����ļ����������ж��������е㡢�ߡ���֮������λ�ù�ϵ,
�Ӷ��ж����������Ƿ��ཻ�������ԭ������:�����ռ��ĸ��㣺a(ax, ay, az), b = (bx, by, bz), c = (cx, cy, cz), d = (dx, dy, dz), ��������ʽ���£�

[a, b, c, d] ���������������������ĸ��ռ���λ�ù�ϵ��
[a, b, c, d] > 0 ��ʾ d �� a��b��c ����ʱ��˳������ɵ������ε������߷���(���Ϸ�);
[a, b, c, d] < 0 ��ʾ d �� ��abc���·�; [a, b, c, d] = 0 ��ʾ�ĵ㹲�档

		������������T1��T2������ֱ�Ϊ��V10��V11��V12��V20��V21��V22��
	���������ڵ�ƽ��ֱ�Ϊ��1�ͦ�2���䷨�����ֱ�ΪN1��N2.�㷨���б������κ���һ�����������ڵ�ƽ����໥λ�ù�ϵ, ��ǰ�ų����ཻ�������
	ͨ������[V20, V21, V22, V1i].(i = 0, 1, 2)���ж�T1�ͦ�2�Ĺ�ϵ��������е�����ʽ��ֵ����Ϊ����ͬ�ţ���T1��T2���ཻ������T1�ͦ�2�ཻ��
	�ཻ�ַ�Ϊ���¼��������
		a)������е�����ʽ��ֵΪ�㣬��T1��T2���棬ת��Ϊ������߶��ཻ���⡣
		b)�������һ������ʽ��ֵΪ�㣬��������������ʽͬ�ţ���ֻ��һ������ƽ���ڣ����Զ����Ƿ���T2�ڲ��������ཻ�������ཻ��
		c)����T1�Ķ���λ��ƽ���2����(����T1��һ������ƽ���2�е����)��

		�ٰ������Ƶķ����� T 2 �� �� 1 ����һ���Ĳ��ԡ����ͨ������, ��ÿ�������α���ȷ����һ��λ����һ������������ƽ���һ��,
	����������λ������һ�ࡣ�㷨�ֱ�ѭ���û�ÿ�������εĶ���, ��ʹV10(V20)λ�ڦ�2(��1)��һ�࣬��������λ������һ�ࣻ
	ͬʱ�Զ���V21��V22(V11, V12)���н�����������ȷ��V10(V20)λ�ڦ�2(��1)���Ϸ����������߷���
	�������ϵ�Ԥ�ų����û�������V10���ڱ�V10V11��V10V12��V20���ڱ�V20V21��V20V22����ƽ��Ľ���L�ཻ�ڹ̶���ʽ�ĵ��ϣ�
	�ֱ��Ϊi��j��k��l(i<j, k<l), ��ͼ��(�ο�ԭ����)
	��Щ����L���γɵķ������Ϊi1 = [i, j], i2 = [k, l].���ˣ����������ε��ཻ��������ת��Ϊ�������i1��i2���ص����⡣
	���ص����ཻ�������ཻ�����ڽ�����ʽ�̶���ֻ����������k <= j��i <= l�����������ص����������ɽ�һ������Ϊ�б�ʽ
	(1)�Ƿ������
				[V10, V11, V20, V21] <= 0 && [V10, V12, V22, V20] <= 0        �б�ʽ(1)
*/

typedef float float3[];

enum TopologicalStructure
{
	INTERSECT, NONINTERSECT
};

//struct Triangle
//{
//	//float3 Normal_0;
//	//float3 Vertex_1, Vertex_2, Vertex_3;
//};

/*******************************************************************************************************/
//Devillers�㷨������
TopologicalStructure judge_triangle_topologicalStructure(Triangle* tri1, Triangle* tri2)
{
	return {};
}

//����boolֵ
bool isTriangleTntersect(Triangle* tri1, Triangle* tri2)
{
	TopologicalStructure  intersectSt = judge_triangle_topologicalStructure(tri1, tri2);
	if (intersectSt == INTERSECT)
		return true;
	return false;
}

typedef std::tuple<char, int, double> TC;
namespace std
{
	//template <class T,class U,class V>
	//class tuple<T, U, V>
	//{
	//public:
	//	T operator[](int i)
	//	{

	//	}
	//};
}

/*
һ����˵����������������������������Ƿ��ཻ:
ͶӰ�㷨:
����ͶӰ���ۣ����Խ���ά�ռ��е������Σ�ͶӰ������һ��ƽ
���ϣ��۲���ͶӰ֮���Ƿ�����ص����Ӷ��ƶ��Ƿ����ཻ;
�����ʷ��㷨:
�����������ν��������ʷ֣��Ƚ����������εıߡ������Ƿ���
���㣬���н��㣬�����������ο����϶�Ϊ�ཻ;

*/

enum class COLLISION :int
{
	T_SEPA,
	T_INTER,
	T_TANG,

};

COLLISION is_two_triangle_intersec(const TriangleVec& triA, const TriangleVec& triB)
{
	return {};
}

double getTrigonArea2(const Triangle& triA)
{
	double a = (triA[1] - triA[0]).squaredNorm();
	double b = (triA[2] - triA[1]).squaredNorm();
	double c = (triA[0] - triA[2]).squaredNorm();
	double p = 0.5 * (a + b + c);
	return p * (p - a) * (p - b) * (p - c);
}
bool isEqualTrigon(const Triangle& triA, const Triangle& triB)
{
	//if (isTwoTrianglesIntersectSAT(triA, triB))
	//{
	//	bool findFlag = false;
	//	for (const auto& iter : triInterList) //findTri
	//	{
	//		if (iter[0][0].x() == triA[0].x() && iter[0][0].y() == triA[0].y() && iter[0][0].z() == triA[0].z() &&
	//			iter[0][1].x() == triA[1].x() && iter[0][1].y() == triA[1].y() && iter[0][1].z() == triA[1].z() &&
	//			iter[0][2].x() == triA[2].x() && iter[0][2].y() == triA[2].y() && iter[0][2].z() == triA[2].z() &&
	//			iter[1][0].x() == triB[0].x() && iter[1][0].y() == triB[0].y() && iter[1][0].z() == triB[0].z() &&
	//			iter[1][1].x() == triB[1].x() && iter[1][1].y() == triB[1].y() && iter[1][1].z() == triB[1].z() &&
	//			iter[1][2].x() == triB[2].x() && iter[1][2].y() == triB[2].y() && iter[1][2].z() == triB[2].z())
	//		{
	//			findFlag = true;
	//			break;
	//		}
	//	}
	//	if (!findFlag) //record the trigons that detect by function, but exclude by pre-box
	//		triInterListExtra.push_back({ triA, triB }); //������ཻ������
	//	return true;
	//}

	double d = 1;
	Vector3d point0(1.1, 0, 0);
	Vector3d point1 = psykronix::translate(2.2, 0) * psykronix::roty(M_PI) *= point0;
	Vector3d point2 = psykronix::rotz(2*M_PI) *= point0;

	if (point0 == point1) //����������ȣ����Ӿ���
		d = 0;
	if (point0 == point2)
		d = 0;
	//size_t n = sizeof(Triangle); //72
	//return memcmp(&triA, &triB, sizeof(Triangle)) == 0;// triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2];

	return 
		(triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2]) ||
		(triA[0] == triB[0] && triA[1] == triB[2] && triA[2] == triB[1]) ||
		(triA[0] == triB[1] && triA[1] == triB[0] && triA[2] == triB[2]) || 
		(triA[0] == triB[1] && triA[1] == triB[2] && triA[2] == triB[0]) ||
		(triA[0] == triB[2] && triA[1] == triB[0] && triA[2] == triB[1]) ||
		(triA[0] == triB[2] && triA[1] == triB[1] && triA[2] == triB[0]) ;

	//area
	//return getTrigonArea2(triA) == getTrigonArea2(triB);
}

void printTrigon(const Triangle& trigon)
{
	cout << "" << endl;
}

static void _test0()
{
	std::array<std::array<double, 3>, 3> triangle = { { {0,0,0},{1,1,1}, {2,2,2}} };
	//int PQP_BV_TYPE = RSS_TYPE | OBB_TYPE;
	//int macro1 = PQP_BV_TYPE & RSS_TYPE;
	//int macro2 = PQP_BV_TYPE & OBB_TYPE;
	std::tuple<char, int, double> mytp;
	mytp = { 'a',1,2.0 };

	double nums[10] = { rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand() };
	//for (int i = 0; i < sizeof(nums)/sizeof(double); i++)
	//	cout << nums[i] << " | ";

		//bool isTI = TriangleIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//double td = getTrianglesDistance(P, Q, { triA0,triA1,triA2 }, { triB0,triB1,triB2 }); //43.905043670162655
	double a = 3;
	float b = 3;
	double c = a - b;
	double d = max(a, c);
	//psykronix::Vertex vec(1, 1, 1);
	//psykronix::Vertex vec2 = vec;

	double num = 1.123456789012345;

	std::stringstream ss;
	ss << setprecision(15) << num;
	string str = ss.str(); 
	cout << "keep:" << str << endl;
	ss.clear(15);
	ss << setprecision(15) << num;
	cout << "keep:" << str << endl;
	ss.clear(15);
	ss << setprecision(15) << M_E;
	cout << "keep:" << str << endl;

	std::stringstream ss1;
	ss1 << setprecision(15) << M_PI;
	cout << "keep:" << ss1.str() << endl;
	UINT64 max = (UINT64)~((UINT64)0);
	cout << max << endl;

}

static void _test1()
{

	double d=std::nan("0");
	Vector3d triA_0 = Vector3d(4924494.8122771187, -385870.18283433426, 5749.9999999999054);
	Vector3d triA_1 = Vector3d(4924599.8122771177, -385945.18283433421, 5749.9999999999054);
	Vector3d triA_2 = Vector3d(4924586.8713248633, -385946.88654301979, 5749.9999999999054);
	Vector3d triB_0 = Vector3d(4924601.8102601077, -385940.89359764993, 5750.0000000000000);
	Vector3d triB_1 = Vector3d(4924595.2577039087, -385951.32193110074, 5750.0000000000000);
	Vector3d triB_2 = Vector3d(4924589.8109916430, -385975.18553675216, 5750.0000000000000);
	bool isTI = isTwoTrianglesIntersection({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool isTIT = TriangularIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	triA_0 = Vector3d(4948618.6464014640, -378059.39893364342, 39.982199933911403); //x
	triA_1 = Vector3d(4948618.6464014640, -378066.89893364336, 39.982199933911403);	//x-
	triA_2 = Vector3d(4948608.3857971635, -378068.70815501985, 39.982184091814524);
	triB_2 = Vector3d(4948618.6464014640, -378066.89893364359, 39.982199933801859); //x-
	triB_0 = Vector3d(4948648.6464014640, -378096.89893364365, 39.982246252978271);
	triB_1 = Vector3d(4948627.9169112947, -378068.36723815475, 39.982214247214543);
	d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool int1=isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	//triA_0 = Vector3d(4948618.646401, -378059.398934, 39.982200);
	//triA_1 = Vector3d(4948618.646401, -378066.898934, 39.982200);
	//triA_2 = Vector3d(4948608.385797, -378068.708155, 39.982184);
	//triB_0 = Vector3d(4948648.646401, -378096.898934, 39.982246);
	//triB_1 = Vector3d(4948618.646401, -378066.898934, 39.982200);
	//triB_2 = Vector3d(4948609.375892, -378068.367238, 39.982186);
	//d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//bool int2 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	triA_0 = Vector3d(4948589.10216887,-378091.689488313,39.9821543184115);
	triA_1 = Vector3d(4948589.10216887,-378102.108378973,39.9821543184115);
	triA_2 = Vector3d(4948581.52809739,-378091.562127208,39.9821426242490);
	triB_0 = Vector3d(4948648.64640146,-378096.898933644,39.9822462529783);
	triB_1 = Vector3d(4948590.11470598,-378087.628423812,39.9821558816473);
	triB_2 = Vector3d(4948588.64640146,-378096.898933644,39.9821536146255);

	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };
	_wirteTrigonFile({ {triA, triB} }, "testTrisData.bin");
	auto res = _readTrigonFile("testTrisData.bin");

	double tolerance = 0.001;
	//bool inter = isTwoTrianglesBoundingBoxIntersect(triA, triB);
	//bool interR = isTwoTrianglesBoundingBoxIntersect(res[0][0], res[0][1]);
	//bool inter1 = isTwoTrianglesBoundingBoxIntersect(triA, triB, tolerance);
	bool isInt = isTwoTrianglesIntersection(triA, triB);
	bool isInt0 = isTwoTrianglesIntersectSAT(triA, triB);
	bool isInt1 = isTwoTrianglesIntersectSAT(res[0][0], res[0][1]);

	triA_0 = Vector3d(4934991.08492488,-380736.849323458,-266.330042529162); 
	triA_1 = Vector3d(4934984.36869635,-380736.849323732,-263.095677331456);	
	triA_2 = Vector3d(4934986.01043158,-380736.849323665,-271.249229247876);
	triB_0 = Vector3d(4934988.30653218,-380736.849323571,-265.705952052692);
	triB_1 = Vector3d(4934982.79133525,-380736.849323797,-262.020815280171);
	triB_2 = Vector3d(4935011.81215053,-380736.849322611,-250.000000000000); 

	bool res1 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool res2 = isTwoTrianglesIntersection({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	//RW test
	std::vector<std::array<uint64_t, 2>> entityIdList = { {1,2},{3,4} };
	_wirteTrigonFile(entityIdList, "entityIdList.bin");
	std::vector<std::array<uint64_t, 2>> entity = _readEntityIDFile("entityIdList.bin");

	bool res3 = isEqualTrigon(triA, triA);
	bool res4 = isEqualTrigon(triA, triB);

	triA_0 = Vector3d(4935003.6138694724, -380736.84932294575, -221.24922924757556); 
	triA_1 = Vector3d(4934991.0849248841, -380736.84932345786, -216.33004252886147);	
	triA_2 = Vector3d(4935003.6138694724, -380736.84932294575, -221.24922924757556);
	triB_0 = Vector3d(4934988.3065321781, -380736.84932357143, -215.70595205269194);
	triB_1 = Vector3d(4934982.7913352484, -380736.84932379687, -212.02081528017138);
	triB_2 = Vector3d(4935011.8121505287, -380736.84932261088, -200.00000000000006); 

	bool err1= isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool err2= isTwoTrianglesBoundingBoxIntersect({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	cout << "return 0" << endl;
}

static void _test2()
{
	string path = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/";
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris1 = _readTrigonFile(path+"triInterList_opt1.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris2 = _readTrigonFile(path+"triInterList_opt2.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris3 = _readTrigonFile(path+"triInterList_opt3.bin"); //4030
	//entity
	std::vector<std::array<uint64_t, 2>> entity1 = _readEntityIDFile(path + "entityIdList_off.bin");
	std::vector<std::array<uint64_t, 2>> entity2 = _readEntityIDFile(path + "entityIdList_on.bin");
	std::vector<std::array<uint64_t, 2>> entity3 = _readEntityIDFile(path + "entityIdList_opt.bin");

	bool isInt;
	double d;
	size_t count = 0;
	//for (auto& iter : tris)
	//{
	//	isInt = isTwoTrianglesIntersectSAT(iter[0], iter[1]);
	//	d = getTrianglesDistance(P,Q,iter[0], iter[1]);
	//	cout << d << endl;
	//}
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris1_extra;
	vector<size_t> numList;
	for (int i=0;i<tris1.size();++i)
	{
		if (!isTwoTrianglesIntersectSAT(tris1[i][0], tris1[i][1]))
			cout << "not intersect1" << endl;
		if (!isTwoTrianglesIntersectSAT(tris2[i][0], tris2[i][1]))
			cout << "not intersect2" << endl;
		if (!(isEqualTrigon(tris1[i][0], tris2[i][0]) && isEqualTrigon(tris1[i][1], tris2[i][1])) &&
			!(isEqualTrigon(tris1[i][0], tris2[i][1]) && isEqualTrigon(tris1[i][1], tris2[i][0])))
			numList.push_back(i);
	}
	cout << "diff-index:" << numList.size() << endl;
	cout << "intersect finish" << endl;

#ifdef DOUBLE_FOR_LOOP
	for (auto& iterA : tris1)
	{
		bool findFlag = false;
		for (auto& iterB : tris3) //findTri
		{
			if ((isEqualTrigon(iterA[0], iterB[0]) && isEqualTrigon(iterA[1], iterB[1])) ||
				(isEqualTrigon(iterA[0], iterB[1]) && isEqualTrigon(iterA[1], iterB[0])))
			{
				findFlag = true;
				break;
			}
		}
		if (!findFlag)
		{
			count++;
			tris1_extra.push_back(iterA);
			//cout << "no findFlag" << endl;
			//print_triangle(iter1[0], iter1[1]);
		}
	}
#endif

	for (auto& iterA : entity1)
	{
		bool findFlag = false;
		for (auto& iterB : entity3) //findTri
		{
			if (iterA[0] == iterB[0] && iterA[1] == iterB[1])
			{
				findFlag = true;
				break;
			}
		}
		if (!findFlag)
		{
			cout << "no findFlag" << endl;
			cout << iterA[0]<< "-"<< iterA[1] << endl; // 6509 - 6134
			//print_triangle(iter1[0], iter1[1]);
		}
	}

	cout << count << endl; //420

	cout << "return 0" << endl;
}

static int enrol = []()->int
{
	//_test0();
	_test1();
	_test2();
	return 0;
}();


//------------------------------------------------------------------------------------------------------------------------------
// 
//------------------------------------------------------------------------------------------------------------------------------
// 
//���ᶨ����Χ������������

/*
�ҵ���������ƽ��ƽ�е�����AABB����
���������ε���������AABB��������ľ������
���������εĶ����Ƿ���AABB��
����㷨�ٶȿ���׼ȷ��ʹ�÷����ᶨ������ֻ��Ҫ����13��Ǳ�ڵķ������Ƿ���ڡ�
������κ�һ����������ڣ���ô�����κ�AABB���ཻ���������Ǳ�ڵķ����ᶼ�����ڣ��������κ�AABB�ཻ��
*/

