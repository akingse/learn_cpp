#pragma once
#ifdef CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif

#include "clashTypeDefine.h"		
#include "calculateVectorMatrix.h"		
#include "calculateTriangle.h"
#include "calculatePolyhedron.h"
#include "calculateDataStructure.h"
#include "calculateDynamicAccuracy.h"
#include "my_triangle_intersect.h" //origin
