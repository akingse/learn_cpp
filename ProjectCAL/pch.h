#pragma once
#include<iostream>
#include<fstream>
#include<sstream>
#define _USE_MATH_DEFINES //using M_PI
#define _AFXDLL
//#define WIN32_LEAN_AND_MEAN
//windows API
//#include <afx.h> //CString
#include<windows.h>
#include<direct.h> //_getcwd
#include<iomanip> //setprecision
#if defined(min) || defined(max)
#undef min
#undef max
#endif

#include<math.h>
#include<chrono>
#include<array>
#include<vector>
#include<map>
#include<unordered_map>
#include<set>
#include<unordered_set>
#include<stack>
#include<queue>
#include<Eigen/Dense>
#define USING_HALFEDGE_STRUCTURE
#define USING_POINTER_VERION
#define TEST_CALCULATION_DEBUG
//#define STATISTIC_DATA_COUNT
#define STORAGE_VERTEX_DATA_2D
#define USING_EIGEN_VERISON
#define FILL_PROFILE_DEBUG_TEMP

#define PROJECT_CAL_DLLEXPORT_DEFINE //set on pre-processor define
#include "auto_include/ProjectCAL_API.h"  //PublicAPI
//#include "test_triangle_intersect.h" //for test verify
