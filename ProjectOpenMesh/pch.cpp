// set in property page, create precompiled header
#include "pch.h"

#ifdef USING_LAPTOP_EXTREME
#endif

#ifdef _DEBUG
#pragma comment (lib, "OpenMeshCored.lib")
#pragma comment (lib, "OpenMeshToolsd.lib")
#else
#pragma comment (lib, "OpenMeshCore.lib")
#pragma comment (lib, "OpenMeshTools.lib")
#endif
