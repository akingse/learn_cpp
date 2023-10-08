#pragma once
//stream
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm> 
#include <chrono>   
#include <iostream>
#include <memory>
#include <type_traits>
#include <time.h> //clock()
#include <ctime>  // random seed
#include <cassert>
//#include <windows.h> //min max
#define _AFXDLL
//#include <afx.h> // MFC <cstring>
#include <functional>  
#include <omp.h>
//math
#define _USE_MATH_DEFINES //using M_PI
#include <cmath>
#include <math.h>
#include <complex>

//stl
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <set>
#include <queue>
#include <stack>
#include <unordered_set>
#include <regex>

//thread
#include <thread>
#include <mutex> 
#include <condition_variable>

#include <process.h>
#include <cassert>
#include <typeinfo>
#include <typeindex> 
#include <type_traits>
#include <utility> 
#include <any> 
#include <string_view>
#include <functional>
#include <memory_resource>
#include <Eigen/Dense>
//#include <CGAL/>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

// my custom
typedef std::array<Eigen::Vector3d, 2> Segment;
typedef std::array<Eigen::Vector3d, 3> Triangle;
typedef std::tuple<std::vector<Eigen::Vector3d>, std::vector<std::array<int, 3>>> Polyhedron;
#include "Project1_API.h"
#include "my_geometry.h"			//export
#include "test_serialize.h"
#include "my_triangle_intersect.h"
#ifdef IS_EXPORT
#define DLLEXPORT __declspec(dllimport)
#else
#define DLLEXPORT __declspec(dllexport)
#endif
#define STATISTIC_DATA_TESTFOR
#define USING_FLATBUFFERS_SERIALIZATION
//#define STATISTIC_DATA_COUNT

#include "my_class_fun.h"		
#include "calculateTriangle.h"		//DLL-API
#include "calculatePolyhedron.h"
#include "test_file_rw.h" //file read and wirte

//Eigen
//typedef Vector3d Vec3;
//typedef Matrix4d BPTransfrom;
//typedef Matrix4d GeTransfrom;
//using Vec3 = Vector3d;
//using Eigen::MatrixXd;
//using Eigen::Matrix4d;
//using Eigen::VectorXd;
//using Eigen::Vector2d;
//using Eigen::Vector3d;