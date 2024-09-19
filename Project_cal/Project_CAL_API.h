#pragma once
#ifdef CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif

#include "clashTypeDefine.h"		
#include "clashInterfaceUtility.h"		
#include "calculateVectorMatrix.h"		
#include "calculatePointLinePlane.h"
#include "calculateTriangle.h"
#include "calculatePolygon2d.h"
#include "calculatePolyhedron.h"
#include "calculateMeshTopology.h"
#include "calculateDataStructure.h"
#include "calculateDynamicAccuracy.h"
#include "my_triangle_intersect.h" //origin
