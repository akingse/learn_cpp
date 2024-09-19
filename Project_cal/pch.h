#pragma once
#include<iostream>
#include<sstream>
#define _USE_MATH_DEFINES //using M_PI
#define _AFXDLL
#define WIN32_LEAN_AND_MEAN
//windows API
#include<windows.h>
#include <afx.h> //CString
#if defined(min) || defined(min)
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
#define CAL_DLLEXPORT_DEFINE //set on pre-processor define
#ifdef CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif
#define USING_HALFEDGE_STRUCTURE
#define USING_POINTER_VERION
#define TEST_CALCULATION_DEBUG

#include "clashTypeDefine.h" // header only
#include "clashInterfaceUtility.h" // header only
#include "calculateVectorMatrix.h" // namespace eigen
#include "calculatePointLinePlane.h" // namespace clash
#include "calculatePolygon2d.h"  // namespace clash
#include "calculateTriangle.h"	 // namespace clash
#include "calculatePolyhedron.h"  // namespace clash
#include "calculateMeshTopology.h" //namespace games
#include "calculateDataStructure.h" // clash //spatial partition tree
#include "calculateDynamicAccuracy.h" //namespace accura
#include "my_triangle_intersect.h"	//for test verify

