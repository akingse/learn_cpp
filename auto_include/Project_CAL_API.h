#pragma once
#ifdef CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif

#include "clashClashTypeDefine.h"   // header only
#include "clashInterfaceUtility.h"  // header only
#include "calculateVectorMatrix.h"  // namespace eigen
#include "calculatePointLinePlane.h"// namespace clash
#include "calculatePolygon2d.h"     // namespace clash
#include "calculateTriangle.h"	    // namespace clash
#include "calculatePolyhedron.h"    // namespace clash
#include "calculateMeshTopology.h"  //namespace games
#include "calculateDataStructure.h" // clash //spatial partition tree
#include "calculateDynamicAccuracy.h"//namespace accura
#include "test_triangle_intersect.h" //for test verify
