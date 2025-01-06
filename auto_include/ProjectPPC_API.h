#pragma 
//using namespace ppc; //ppc=python parametric component

// set in unified macro export file
#ifdef PROJECT_PPC_DLLEXPORT_DEFINE
#define DLLEXPORT_PPC __declspec(dllexport)
#else
#define DLLEXPORT_PPC __declspec(dllimport)
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
