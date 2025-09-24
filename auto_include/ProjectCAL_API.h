#pragma once
#ifdef PROJECT_CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif
//#define DLLEXPORT_CAL

#include "clashInterfaceUtility.h"  // header only
#include "clashClassTypeDefine.h"   // header only
#include "DataRecordSingleton.h"    //singleton
#include "calculateVectorMatrix.h"  // namespace eigen
#include "calculatePointLinePlane.h"// namespace clash
#include "calculatePolygon2d.h"     // namespace clash
#include "calculateTriangle.h"	    // namespace clash
#include "calculatePolyhedron.h"    // namespace clash
#include "calculateMeshTopology.h"  //namespace games
#include "calculateSpatialSearchTree.h" // clash //spatial partition tree
#include "calculateDynamicAccuracy.h"//namespace accura
#include "calculateClashDetection.h"

#include "commonFileReadWrite.h" 
