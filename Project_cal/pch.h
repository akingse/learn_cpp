#pragma once
#include<iostream>
#define _USE_MATH_DEFINES //using M_PI
#define _AFXDLL

#include<math.h>
#include<chrono>
#include<array>
#include<vector>
#include<map>
#include<set>
#include<queue>
#include<Eigen/Dense>
//#define CAL_DLLEXPORT_DEFINE //set on pre-processor define
#ifdef CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllimport)
#else
#define DLLEXPORT_CAL __declspec(dllexport)
#endif


#include "psykronixTypeDefine.h"		
#include "my_class_fun.h"		
#include "calculateTriangle.h"	
#include "calculateDataTree.h"
#include "calculatePolyhedron.h"
