#pragma once
//there is no any export, just using source code

#ifdef PROJECT_2_DLLEXPORT_DEFINE
#define DLLEXPORT_2 __declspec(dllimport)
#else
#define DLLEXPORT_2 __declspec(dllexport)
#endif
#define STATISTIC_DATA_TESTFOR
#define USING_CONDITIONAL_COMPILE_PROJECT_2
//#define USING_FLATBUFFERS_SERIALIZATION //only open in ThinkPad
//#define STATISTIC_DATA_COUNT
