#pragma once
#define _CRT_SECURE_NO_WARNINGS
//stream
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm> 
#include <iostream>
//time
#include <chrono>   
#include <time.h> //clock()
#include <ctime>  // random seed
#include <cassert>

//windows
//#include <windows.h> //min max
#define _AFXDLL
//#include <afx.h> // MFC <cstring>
// 
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

// third party
#include <omp.h>
#include <Eigen/Dense>
//#include <CGAL/>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include "Project1_API.h" //add head include: $(SolutionDir)Project1
#include "Poject_CAL_API.h"
#include "my_geometry.h" //export
#include "test_serialize.h"
#ifdef PROJECT_2_DLLEXPORT_DEFINE
#define DLLEXPORT __declspec(dllimport)
#else
#define DLLEXPORT __declspec(dllexport)
#endif
#define STATISTIC_DATA_TESTFOR
#define USING_CONDITIONAL_COMPILE_PROJECT_2
//#define USING_FLATBUFFERS_SERIALIZATION //only open in ThinkPad
//#define STATISTIC_DATA_COUNT
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