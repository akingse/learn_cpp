#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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

int main()
{
    test0();

    return (0);
}
