#pragma once
#define _CRT_SECURE_NO_WARNINGS
//stream
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm> 
//time
#include <chrono>   
#include <time.h> //clock()
#include <ctime>  // random seed
#include <cassert>
//math
#define _USE_MATH_DEFINES //using M_PI
#include <cmath>
#include <math.h>
#include <complex>
#include <random> //C++11 rand library

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
#include <thread>   // std::thread
#include <mutex>    // std::mutex
#include <future>	// std::future
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
#include <memory>
#include <memory_resource>

//windows
#define _AFXDLL
#define WIN32_LEAN_AND_MEAN //avoid _WINSOCKAPI_ warning
#include <windows.h> //min max
#include <afx.h> // MFC <cstring>
#if defined(min) || defined(min)
#undef min
#undef max
#endif

// third party
#include <omp.h>
#include <Eigen/Dense>
//#include <CGAL/>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

//import
#include "Project_CAL_API.h"
#include "Project1_API.h" //add head include: $(SolutionDir)Project1

#define PROJECT_2_DLLEXPORT_DEFINE
#ifdef PROJECT_2_DLLEXPORT_DEFINE
#define DLLEXPORT_2 __declspec(dllexport)
#else
#define DLLEXPORT_2 __declspec(dllimport)
#endif
#define STATISTIC_DATA_TESTFOR
#define USING_CONDITIONAL_COMPILE_PROJECT_2
//#define USING_FLATBUFFERS_SERIALIZATION //only open in ThinkPad
//#define STATISTIC_DATA_COUNT

#include "my_geometry.h" //export
#include "test_serialize.h"
#include "my_file_read_write.h" //file read and wirte

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