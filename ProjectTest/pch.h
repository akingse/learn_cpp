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
#include <functional>
#include <memory>
//C++17
#ifdef USING_STANDARD_CPP_17
#include <any> //C++17
#include <string_view>
#include <memory_resource>
#endif

//windows
#define _AFXDLL //redefine
#define WIN32_LEAN_AND_MEAN //avoid _WINSOCKAPI_ warning
#include <windows.h> //min max
#include <afx.h> // MFC <cstring>
#if defined(min) || defined(max)
#undef min
#undef max
#endif

// third party
#include <omp.h>
#include <Eigen/Dense>
//#include <CGAL/>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

//import
#include "ProjectCAL_API.h"
#include "ProjectPPC_API.h" //add head include: $(SolutionDir)Project1

#define STATISTIC_DATA_TESTFOR
//#define USING_FLATBUFFERS_SERIALIZATION //only open in ThinkPad
//#define STATISTIC_DATA_COUNT

