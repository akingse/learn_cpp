#pragma 

// set in unified macro export file
#ifdef PROJECT_1_DLLEXPORT_DEFINE
#define DLLEXPORT_1 __declspec(dllexport)
#else
#define DLLEXPORT_1 __declspec(dllimport)
#endif

//component
#include "BPParaGnrc.h"				
#include "BPParaVec.h"				
#include "BPParaTransform.h"		
#include "BPParaDependency.h"		

// my custom
#include "my_gnrc.h"				
#include "my_vec.h"					
#include "my_md5.h"					
#include "my_ref_count.h"			
#include "my_handle.h"				
#include "my_geometry.h"			

//geometry
#include "geometry/ParaGeometryClass.h"              // public
#include "geometry/ParaGeometryCalculate.h"          // public
#include "geometry/ParaGeometryRelation.h"         	 // public
