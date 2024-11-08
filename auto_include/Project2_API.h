#pragma once
//there is no any export, just using source code

#ifdef PROJECT_2_DLLEXPORT_DEFINE
#define DLLEXPORT_2 __declspec(dllexport)
#else
#define DLLEXPORT_2 __declspec(dllimport)
#endif

#include "commonFileReadWrite.h" //file read and wirte
