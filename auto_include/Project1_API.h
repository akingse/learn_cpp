#pragma once
#ifdef PROJECT_1_DLLEXPORT_DEFINE
#define DLLEXPORT_1 __declspec(dllexport)
#else
#define DLLEXPORT_1 __declspec(dllimport)
#endif

//component
#include "Gnrc.h"					//export
#include "BPParaVec.h"				//export
#include "BPParaTransform.h"		//export
#include "UtilitySet.h"				//export

// my custom
#include "my_gnrc.h"				
#include "my_vec.h"					//export
#include "my_md5.h"					//export
#include "my_ref_count.h"			//export
#include "my_handle.h"				//export

//geometry
#include "geometry/ParaGeometryClass.h"              // public
#include "geometry/ParaGeometryCalculate.h"
#include "geometry/ParaGeometryRelation.h"         	// public