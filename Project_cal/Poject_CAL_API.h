#pragma once
#ifdef CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif

#include "clashTypeDefine.h"		
#include "calculateVertorMatrix.h"		
#include "calculateTriangle.h"
#include "calculateDataTree.h"
#include "calculatePolyhedron.h"
