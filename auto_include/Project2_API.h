#pragma once
#include "my_geometry.h" //export
#include "test_serialize.h"
#ifdef PROJECT_2_DLLEXPORT_DEFINE
#define DLLEXPORT __declspec(dllimport)
#else
#define DLLEXPORT __declspec(dllexport)
#endif
#define STATISTIC_DATA_TESTFOR
#define USING_CONDITIONAL_COMPILE_PROJECT_2
//#define USING_FLATBUFFERS_SERIALIZATION //only open in ThinkPad
//#define STATISTIC_DATA_COUNT
#include "my_file_read_write.h" //file read and wirte