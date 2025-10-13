#include "pch.h"
using namespace std;
using namespace clash;
using namespace Eigen;
using namespace eigen;
using namespace accura;

static void _test1()
{
	// large-large
	//Eigen::Vector3d vecA(2e6,1e6,0);
	//Eigen::Vector3d vecB(2e6,1e6+1,0);
	double d2 = 2.0 / 180 * M_PI;//0.035

	Eigen::Vector3d vecA(0, 0, 1e6);
	Eigen::Vector3d vecB(1, 0, 1e6);
	bool pall = Precision::isParallel(vecA, vecB);
    double croPro = vecA.cross(vecB).norm() / (vecA.norm() * vecB.norm());
	Eigen::Vector3d vecAU = vecA.normalized();
	Eigen::Vector3d vecBU = vecB.normalized();
	double croProU = vecAU.cross(vecBU).norm();
	double du = croPro - croProU; //always equal

	double angle1 = eigen::angle_two_vectors(vecA, vecB, false);
	double angle2 = eigen::angle_two_vectors(vecA, vecB, true);
	double d = angle1 - angle2;	// =1e-10 //单位化仅对acos有影响
	return;
}

static void _test2()
{
	// large-short
	Eigen::Vector3d vecA(2e6, 1e6+1, 0);
	Eigen::Vector3d vecB(2, 1, 0);
	bool pall = Precision::isParallel(vecA, vecB);
	double angle1 = eigen::angle_two_vectors(vecA, vecB, false);
	double angle2 = eigen::angle_two_vectors(vecA, vecB, true);
	double d = angle1 - angle2;

	// large-short
	Eigen::Vector3d vecC(2e6, 1e6, 0);
	Eigen::Vector3d vecD(2, 1+1e-6, 0);
	bool pall2 = Precision::isParallel(vecC, vecD);
	angle1 = eigen::angle_two_vectors(vecA, vecB, false);
	angle2 = eigen::angle_two_vectors(vecA, vecB, true);
	d = angle1 - angle2;

	return;
}

static void _test3()
{
	// short-short
	Eigen::Vector3d vecA(10, 11, 0);
	Eigen::Vector3d vecB(10, 11 + 1e-5, 0);
	bool pall = Precision::isParallel(vecA, vecB);
	double angle1 = eigen::angle_two_vectors(vecA, vecB, false);
	double angle2 = eigen::angle_two_vectors(vecA, vecB, true);
	double d = angle1 - angle2;	// =1e-10
	return;
}

static void _test4()
{
	Eigen::Vector3d vecA(0, 0, 1);
	Eigen::Vector3d vecB(0, 0, -1);
	Eigen::Vector3d vecC(0, 0, 0);
	bool pall = Precision::isParallel(vecA, vecA); //same
	bool pall2 = Precision::isParallel(vecA, vecB);//reverse
	bool pall3 = Precision::isParallel(vecA, vecC); //zero


	double d = 0;
	//对于随机数，是否单位化，差在1e-15
	for (int i = 0; i < 100; i++)
	{
		Eigen::Vector3d vecA(rand(), rand(), 0);
		Eigen::Vector3d vecB(rand(), rand(), 0);
		double angle1 = eigen::angle_two_vectors(vecA, vecB, false);
		double angle2 = eigen::angle_two_vectors(vecA, vecB, true);
		d = angle1 - angle2;
	}

}

//精度问题
static void _test5()
{
	//顶点是否在该平面上
	double delta = 0;
	for (int i = 0; i < 100; i++)
	{
		//Triangle3d trigon = {
  //          Eigen::Vector3d(rand() + double(rand()) / RAND_MAX, rand(), rand()),
		//	Eigen::Vector3d(rand(), rand() + double(rand()) / RAND_MAX, rand()),
		//	Eigen::Vector3d(rand(), rand(), rand() + double(rand()) / RAND_MAX),
		//};
		// 
		Triangle3d trigon = {
			Eigen::Vector3d(rand() + double(rand()) / RAND_MAX, rand() + double(rand()) / RAND_MAX, rand() + double(rand()) / RAND_MAX),
			Eigen::Vector3d(rand() + double(rand()) / RAND_MAX, rand() + double(rand()) / RAND_MAX, rand() + double(rand()) / RAND_MAX),
			Eigen::Vector3d(rand() + double(rand()) / RAND_MAX, rand() + double(rand()) / RAND_MAX, rand() + double(rand()) / RAND_MAX),
		};
		//向量是否单位化，结果相差很大
		Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]);
		//Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]).normalized();
		double dot0 = normal.dot(trigon[1] - trigon[0]);
		double dot1 = normal.dot(trigon[2] - trigon[1]);
		double dot2 = normal.dot(trigon[0] - trigon[2]);
		//double dot0 = normal.dot((trigon[1] - trigon[0]).normalized());
		//double dot1 = normal.dot((trigon[2] - trigon[1]).normalized());
		//double dot2 = normal.dot((trigon[0] - trigon[2]).normalized());
		delta += fabs(dot0) + fabs(dot1) + fabs(dot2);
	}


}

static int enrol = []()->int
	{
		//_test1();
		//_test2();
		//_test3();
		//_test4();
		_test5();
		cout << get_filepath_filename(__FILE__) << " finished.\n" << endl;
		return 0;
	}();
