#pragma once
#ifdef PROJECT_CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif
//#define DLLEXPORT_CAL

//#include "clashInterfaceUtility.h"  // header only
#include "DataClassTypeDefine.h"   
#include "DataRecordSingleton.h"    //singleton
#include "calculateVectorMatrix.h"  // namespace eigen
#include "calculatePointLinePlane.h"// namespace clash
#include "calculateSpatialSearchTree.h" // clash //spatial partition tree
#include "calculateTriangle.h"	    // namespace clash
#include "calculatePolygon2d.h"     // namespace clash
#include "calculatePolygon3d.h"     // namespace land
#include "calculatePolyhedron.h"    // namespace clash
#include "calculateMeshTopology.h"  //namespace games
#include "calculateDynamicAccuracy.h"//namespace accura
#include "calculateClashDetection.h"
#include "commonFileReadWrite.h" 
