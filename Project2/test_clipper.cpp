#include "pch.h"
#include <algorithm>
#include <random>
#include "windows.h"

using namespace std;
using namespace psykronix;
using namespace Eigen;

static void test0()
{
	std::array<Eigen::Vector2d, 2> segment{Vector2d(11, 0), Vector2d(11, 0)};
	Eigen::AlignedBox2d box{Vector2d(0,0), Vector2d(10,10)};
	bool isInter1 = isSegmentAndBoundingBoxIntersectSAT(segment, box);

	std::array<Eigen::Vector2d, 2> segmA{Vector2d(10, 10), Vector2d(20, 20)};
	//std::array<Eigen::Vector2d, 2> segmB{Vector2d(10, 10), Vector2d(20, 20)};
	std::array<Eigen::Vector2d, 2> segmB{Vector2d(15, 15), Vector2d(25, 28)};

	bool isInter2 = isTwoSegmentsIntersect(segmA, segmB);
	bool isInter3 = isTwoSegmentsCollinearCoincident(segmA, segmB);
	return;
}

//test cal time, by random rectange
#define USING_KDTREE_METHOD
static void test1()
{
	const size_t  totalNum = 10000;
	std::random_device rd_rct, rd_pair; //to genetate random number
	std::mt19937 gen_rct(rd_rct());
	std::mt19937 gen_pair(rd_pair());
	std::uniform_real_distribution<double> rectange(0.0, 10000.0);
	std::uniform_real_distribution<double> interPair(0.0, 10.0);
	std::vector<Polygon2d> polygons;

	for (int i = 0; i < totalNum; i++)
	{
		std::vector<Eigen::Vector2d> squareA, squareB;
		Vector2d point = Vector2d(rectange(gen_rct), rectange(gen_rct));
		squareA.push_back(point);
		double x1 = interPair(gen_pair);
		double y1 = interPair(gen_pair);
		squareA.push_back(point + Vector2d(0, y1));
		squareA.push_back(point - Vector2d(x1, y1));
		squareA.push_back(point - Vector2d(x1, 0));
		//squareB
		squareB.push_back(point);
		double x2 = interPair(gen_pair);
		double y2 = interPair(gen_pair);
		squareB.push_back(point + Vector2d(x2, 0));
		squareB.push_back(point + Vector2d(x2, y2));
		squareB.push_back(point + Vector2d(0, y2));
		polygons.push_back(Polygon2d(squareA));
		polygons.push_back(Polygon2d(squareB));
	}

	clock_t start, end;
	for (int i = 0; i < 3; i++)
	{
		start = clock();
#ifdef USING_KDTREE_METHOD
		KdTree2d kdtree(polygons);
		for (auto& iter : polygons)
		{
			std::vector<size_t> indexes = kdtree.findIntersect(iter);
			for (const auto& i:indexes)
				BooleanOpIntersect(iter, polygons[i]);
		}

#else
		BooleanOpIntersect(polygons);
#endif // USING_KDTREE_METHOD

		end = clock(); //completed
		cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
		//timecout += to_string(double(end - start) / CLOCKS_PER_SEC);
		//timecout += "\n";
		Sleep(1000);
	}
	/*
	��ʱָ������ 10
	time = 1.21s
	time = 1.265s
	time = 1.246s
	*/
}

//test for createKdTree
static void test2()
{
	std::vector<Polygon2d> polyVct;
	polyVct.emplace_back(Polygon2d({ Vector2d(0,0), Vector2d(0,0) + Vector2d(2,2) }));
	polyVct.emplace_back(Polygon2d({ Vector2d(2,2), Vector2d(2,2) + Vector2d(2,2) }));
	polyVct.emplace_back(Polygon2d({ Vector2d(1,1), Vector2d(1,1) + Vector2d(2,2) }));
	polyVct.emplace_back(Polygon2d({ Vector2d(3,3), Vector2d(3,3) + Vector2d(2,2) }));
	polyVct.emplace_back(Polygon2d({ Vector2d(4,4), Vector2d(4,4) + Vector2d(2,2) }));
	//std::shared_ptr<KdTreeNode> kdTree = KdTree::createKdTree(polyVct);
	KdTree2d kdtree(polyVct);
	std::shared_ptr<KdTreeNode2d> tree = kdtree.get();
	//int a = sizeof(tree); //double pointor ==16
	Polygon2d target({ Vector2d(3,1), Vector2d(5,3) });
	std::vector<size_t> indexes = kdtree.findIntersect(target);

	return;
}

static void test4(Eigen::Vector3d* boxVtx)
{

}
static void test3()
{
	//���Թ��ߺ���
	bool isCC = false;
	//std::array<Eigen::Vector3d, 2> segmA{Vector3d(10, 10, 0), Vector3d(20, 20, 0)};
	//std::array<Eigen::Vector3d, 2> segmB{Vector3d(15, 15, 0), Vector3d(30, 30, 0)};
	std::array<Eigen::Vector3d, 2> segmA{Vector3d(10, 10, 0), Vector3d(30, 30, 0)};
	std::array<Eigen::Vector3d, 2> segmB{Vector3d(15, 15, 0), Vector3d(20, 20, 0)};
	isCC=isTwoSegmentsCollinearCoincident(segmA, segmB);

	segmB = { Vector3d(15, 15, 0), Vector3d(30, 30.0001, 0) };
	isCC = isTwoSegmentsCollinearCoincident(segmA, segmB, 0.1, 0.1);
	segmB = { Vector3d(15, 15.0001, 0), Vector3d(30, 30, 0) };
	isCC = isTwoSegmentsCollinearCoincident(segmA, segmB, 0.1, 0.1);
	segmB = { Vector3d(16, 15, 0), Vector3d(31, 30, 0) };
	isCC = isTwoSegmentsCollinearCoincident(segmA, segmB, 0.1, 0.1);

	// res function
	segmA = { Vector3d(10, 0, 0), Vector3d(20, 0, 0) };
	segmB = { Vector3d(10, 0, 0), Vector3d(20, 0, 0) };
	std::tuple<bool, std::array<double, 4>> isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB);
	//����غ�
	segmB = { Vector3d(10, 0, 0), Vector3d(30, 0, 0) }; 
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0-1
	segmB = { Vector3d(10, 0, 0), Vector3d(15, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0-0.5
	//�յ��غ�
	segmB = { Vector3d(-10, 0, 0), Vector3d(20, 0, 0) }; 
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0-1
	segmB = { Vector3d(15, 0, 0), Vector3d(20, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0.5-1
	//segmB�ڲ�
	segmB = { Vector3d(15, 0, 0), Vector3d(18, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0.5-0.8
	//segmB�ⲿ
	segmB = { Vector3d(-10, 0, 0), Vector3d(30, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0-1
	//segmB���
	segmB = { Vector3d(5, 0, 0), Vector3d(15, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0-0.5 |
	//segmB�Ҳ�
	segmB = { Vector3d(15, 0, 0), Vector3d(30, 0, 0) };
	//segmB = { Vector3d(30, 0, 0), Vector3d(15, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB); //0.5-1 
	//���˵��غ�
	segmB = { Vector3d(20, 0, 0), Vector3d(30, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB);
	segmB = { Vector3d(-10, 0, 0), Vector3d(10, 0, 0) };
	isSCC = getTwoSegmentsCollinearCoincidentPoints(segmA, segmB);


	std::array<Eigen::Vector3d, 8> boxVtx;
	Eigen::Vector3d boxArr[8];
	test4(boxVtx.data());

	return;
}


static int enrol = []()->int
{
	//test0();
	//test1();
	//test2(); //for funciton
	test3();
	cout << "test_clipper" << endl;
	return 0;
}();
