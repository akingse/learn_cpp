#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#ifdef _DEBUG
#pragma comment (lib, "pcl_commond.lib")
#pragma comment (lib, "pcl_featuresd.lib")
#pragma comment (lib, "pcl_filtersd.lib")
#pragma comment (lib, "pcl_iod.lib")
#pragma comment (lib, "pcl_io_plyd.lib")
#pragma comment (lib, "pcl_keypointsd.lib")
#pragma comment (lib, "pcl_mld.lib")
#pragma comment (lib, "pcl_octreed.lib")
#pragma comment (lib, "pcl_outofcored.lib")
#pragma comment (lib, "pcl_peopled.lib")
#endif

#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

#ifdef _DEBUG
#pragma comment (lib, "pcl_peopled.lib")
#pragma comment (lib, "pcl_recognitiond.lib")
#pragma comment (lib, "pcl_registrationd.lib")
#pragma comment (lib, "pcl_sample_consensusd.lib")
#pragma comment (lib, "pcl_searchd.lib")
#pragma comment (lib, "pcl_segmentationd.lib")
#pragma comment (lib, "pcl_stereod.lib")
#pragma comment (lib, "pcl_surfaced.lib")
#pragma comment (lib, "pcl_trackingd.lib")
#pragma comment (lib, "pcl_visualizationd.lib")

#pragma comment (lib, "libboost_thread-vc143-mt-gd-x64-1_87.lib")

#pragma comment (lib, "vtkcgns-9.4.lib")
#pragma comment (lib, "vtkcgns-9.4-gd.lib")
#pragma comment (lib, "vtkChartsCore-9.4-gd.lib")
#pragma comment (lib, "vtkCommonColor-9.4-gd.lib")
#pragma comment (lib, "vtkCommonComputationalGeometry-9.4-gd.lib")
#pragma comment (lib, "vtkCommonCore-9.4-gd.lib")
#pragma comment (lib, "vtkCommonDataModel-9.4-gd.lib")
#pragma comment (lib, "vtkCommonExecutionModel-9.4-gd.lib")
#pragma comment (lib, "vtkCommonMath-9.4-gd.lib")
#pragma comment (lib, "vtkCommonMisc-9.4-gd.lib")
#pragma comment (lib, "vtkCommonSystem-9.4-gd.lib")
#pragma comment (lib, "vtkCommonTransforms-9.4-gd.lib")
#pragma comment (lib, "vtkWrappingTools-9.4-gd.lib")
#pragma comment (lib, "vtksys-9.4-gd.lib")

#pragma comment (lib, "vtkRenderingAnnotation-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingContext2D-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingContextOpenGL2-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingCore-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingFreeType-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingGL2PSOpenGL2-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingImage-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingLabel-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingLOD-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingOpenGL2-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingSceneGraph-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingUI-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingVolume-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingVolumeOpenGL2-9.4-gd.lib")
#pragma comment (lib, "vtkRenderingVtkJS-9.4-gd.lib")

#endif

//inc
/*
C:\Program Files\OpenNI2\Include
C:\Program Files\PCL 1.15.1\include\pcl-1.15
C:\Program Files\PCL 1.15.1\3rdParty\Boost\include\boost-1_87
C:\Program Files\PCL 1.15.1\3rdParty\Eigen3\include\eigen3
C:\Program Files\PCL 1.15.1\3rdParty\FLANN\include\flann
C:\Program Files\PCL 1.15.1\3rdParty\Qhull\include
C:\Program Files\PCL 1.15.1\3rdParty\VTK\include\vtk-9.4
*/

//lib
/*
C:\Program Files\OpenNI2\Lib
C:\Program Files\PCL 1.15.1\lib
C:\Program Files\PCL 1.15.1\3rdParty\Boost\lib
C:\Program Files\PCL 1.15.1\3rdParty\FLANN\lib
C:\Program Files\PCL 1.15.1\3rdParty\Qhull\lib
C:\Program Files\PCL 1.15.1\3rdParty\VTK\lib
*/

//define
/*
_CRT_SECURE_NO_DEPRECATE
_CRT_SECURE_NO_WARNINGS
_SCL_SECURE_NO_WARNINGS
_SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING
BOOST_ALL_NO_LIB
BOOST_USE_WINDOWS_H
NOMINMAX

*/

//PATH
/*
PATH=C:\Program Files\PCL 1.15.1\bin;
C:\Program Files\OpenNI2\Tools;
C:\Program Files\PCL 1.15.1\3rdParty\FLANN\bin;
C:\Program Files\PCL 1.15.1\3rdParty\Qhull\bin;
C:\Program Files\PCL 1.15.1\3rdParty\VTK\bin
*/

static void test0()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    for (auto& point : cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto& point : cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
}

void test1()
{
	srand((unsigned int)time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 创建点云数据
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	//半径内近邻搜索
	vector<int>pointIdxRadiusSearch;
	vector<float>pointRadiusSquaredDistance;
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
	cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << endl;
	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << endl;
	}
	// 初始化点云可视化对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("display PCL"));
	viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
	// 对点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

}


int main()
{
    test1();

    return (0);
}
