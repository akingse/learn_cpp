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
//#define USING_CONTAINER_SET
#define USING_KDTREE_METHOD
#define USING_FINDINTERSECTCLASH

//测试kd-tree
static void test0()
{
	double g_tolerance = 0;
	size_t N = 1000; //1e3//1e4//1e5//1e6
	std::vector<Polyface3d> polyface3dVct;// (N);

//#pragma region custom
#if 0
	std::random_device rd;
	std::mt19937 gen(rd());
	//std::uniform_int_distribution<> dis(0, 10000); //int
	std::uniform_real_distribution<> dis(0, 10000); //double
	for (size_t i = 0; i < N; ++i)
	{
		AlignedBox3d box = { Vector3d(0,0,0), Vector3d(300,200,100) };
		Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
		//double x = rand() % (10000 + 1);
		//double y = rand() % (10000 + 1);
		//double z = rand() % (10000 + 1);
		double x = dis(gen);
		double y = dis(gen);
		double z = dis(gen);
		transformation.translation() << x, y, z;
		box.transform(transformation);
		polyface3dVct.emplace_back(Polyface3d{ long long(i), box });
	}
#endif // 0

	//读写bin文件以保证数据一致，用于对比（kd-trer&double-loop）结果的统一行
	double* arr = _readBinFileAlignedBox(N);
	for (int i = 0; i < N; i++)
	{
		double min_x = arr[6 * i + 0];
		double min_y = arr[6 * i + 1];
		double min_z = arr[6 * i + 2];
		double max_x = arr[6 * i + 3];
		double max_y = arr[6 * i + 4];
		double max_z = arr[6 * i + 5];
		//cout << "(" << min_x << "," << min_y << "," << "),(" << min_z << "," << max_x << "," << max_y << "," << max_z << ")" << endl;
		AlignedBox3d box = { Vector3d(min_x,min_y,min_z), Vector3d(max_x,max_y,max_z) };
		polyface3dVct.emplace_back(Polyface3d{ long long(i), box });
	}

	for (int i = 0; i < 3; i++)
	{
		steady_clock::time_point start, end;
		microseconds duration, duration2;
#ifdef USING_CONTAINER_SET
		set<array<size_t, 2>> findIndex;
#else
		vector<array<size_t, 2>> findIndex;
		std::vector<std::tuple<size_t, bool>> findIndexClash;
#endif // USING_CONTAINER_SET

		//compare
#ifdef USING_KDTREE_METHOD
		start = std::chrono::high_resolution_clock::now();
		// time of build tree
		KdTree3d kdtree(polyface3dVct); //时间线性增长
		kdtree.setTolerance(g_tolerance);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<microseconds>(end - start);
		cout << "N=" << N << " kdtree create time=" << duration.count() << " micro_seconds" << endl;
		//count find time
		start = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < N; ++i)
		{
#ifdef USING_FINDINTERSECTCLASH
			vector<std::tuple<size_t, bool>> temp = kdtree.findIntersectClash(polyface3dVct[i]);
			findIndexClash.insert(findIndexClash.end(), temp.begin(), temp.end());
#else
			std::vector<size_t> temp = kdtree.findIntersect(polyface3dVct[i]);
			for (auto& j : temp)
			{
				if (i < j)
#ifdef USING_CONTAINER_SET
					findIndex.insert({ i,j });
#else
					findIndex.emplace_back(array<size_t, 2>{ i, j }); // tiny fast than //findIndex.push_back({ i,j });
#endif // USING_CONTAINER_SET
			}
#endif //USING_FINDINTERSECTCLASH
		}
#else
		//double loop
		start = std::chrono::high_resolution_clock::now();
		AlignedBox3d box_tolerance;
		Vector3d tole_size = (g_tolerance == 0.0) ? Vector3d(eps, eps, eps) : Vector3d(g_tolerance, g_tolerance, g_tolerance);
		for (size_t i = 0; i < N; ++i)
		{
			for (size_t j = 0; j < N; ++j)
			{
				if (i >= j)
					continue;
				box_tolerance = { polyface3dVct[i].m_bound.min() - tole_size, polyface3dVct[i].m_bound.max() + tole_size };
				if (box_tolerance.intersects(polyface3dVct[j].m_bound))
#ifdef USING_CONTAINER_SET
					findIndex.insert({ i,j });
#else
					findIndex.emplace_back(array<size_t, 2>{ i, j }); // tiny fast than //findIndex.push_back({ i,j });
#endif // USING_CONTAINER_SET
			}
		}
#endif // USING_KDTREE_METHOD
		end = std::chrono::high_resolution_clock::now();
		duration2 = std::chrono::duration_cast<microseconds>(end - start);
		//cout << "count_finded=" << findIndex.size() << " time=" << duration2.count() << " micro_seconds" << endl;
		// total time
		cout << "count_finded=" << findIndex.size() + findIndexClash.size() << " time=" << duration.count() + duration2.count() << " micro_seconds" << endl;
		cout << endl;
	}
	return;
}

static void test1()
{
	size_t N = 1e4;
	std::vector<Polyface3d> polyface3dVct;// (N);
	double* arr = _readBinFileAlignedBox(N);

	steady_clock::time_point start, end;
	microseconds duration;
	start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < N; i++)
	{
		double min_x = arr[6 * i + 0];
		double min_y = arr[6 * i + 1];
		double min_z = arr[6 * i + 2];
		double max_x = arr[6 * i + 3];
		double max_y = arr[6 * i + 4];
		double max_z = arr[6 * i + 5];
		AlignedBox3d box = { Vector3d(min_x,min_y,min_z), Vector3d(max_x,max_y,max_z) };
		polyface3dVct.push_back(Polyface3d{ long long(i), box });
	}
	end = std::chrono::high_resolution_clock::now();
	//duration = std::chrono::duration_cast<microseconds>(end - start);
	//cout << "N=" << N << " load data time=" << duration.count() << " micro_seconds" << endl;

	//std::sort函数使用快速排序（Quick Sort）
	for (int i = 0; i < 3; i++)
	{
		std::vector<Polyface3d> polyface3dVctCopy = polyface3dVct;
		start = std::chrono::high_resolution_clock::now();
		std::sort(polyface3dVctCopy.begin(), polyface3dVctCopy.end(),
			[](const Polyface3d& a, const Polyface3d& b) { return a.m_bound.min()[0] < b.m_bound.min()[0]; }); // index of Vector3d
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<microseconds>(end - start);
		cout << "N=" << N << " sort time=" << duration.count() << " micro_seconds" << endl;
	}
	double o1 = N * log2(N);
	cout << "On=" << o1 << endl;
	//时间复杂度为O(n log n)
	//1e4=800 //1.32877
	//1e5=12000 //1.66096e+06
	//1e6=121302 //1.99316e+07
}

static int enrol = []()->int
{
	test0();
	//test1();
	cout << "test_data_tree finished.\n" << endl;
	return 0;
}();


