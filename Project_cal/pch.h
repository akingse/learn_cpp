#pragma once
#include<iostream>
#include<sstream>
#define _USE_MATH_DEFINES //using M_PI
#define _AFXDLL
#define WIN32_LEAN_AND_MEAN
//windows API
#include<windows.h>
#include <afx.h> //CString
#if defined(min) || defined(max)
#undef min
#undef max
#endif

#include<math.h>
#include<chrono>
#include<array>
#include<vector>
#include<map>
#include<set>
#include<queue>
#include<Eigen/Dense>
#define USING_HALFEDGE_STRUCTURE
#define USING_POINTER_VERION
#define TEST_CALCULATION_DEBUG

#define CAL_DLLEXPORT_DEFINE //set on pre-processor define
#include "auto_include/Project_CAL_API.h"  //PublicAPI
#include "test_triangle_intersect.h" //for test verify

