#pragma once
#ifdef CAL_DLLEXPORT_DEFINE
#define DLLEXPORT_CAL __declspec(dllexport)
#else
#define DLLEXPORT_CAL __declspec(dllimport)
#endif

#include "psykronixTypeDefine.h"		
#include "my_class_fun.h"		
#include "calculateTriangle.h"
#include "calculateDataTree.h"
#include "calculatePolyhedron.h"
