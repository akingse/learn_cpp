#pragma once
#include<iostream>
#include<Eigen/Dense>
//#include"Eigen/Geometry"   
#define _USE_MATH_DEFINES //using M_PI
#include<math.h>
#include<chrono>
#include<array>
#include<vector>
//#define IS_EXPORT
#ifdef IS_EXPORT
#define DLLEXPORT __declspec(dllimport)
#else
#define DLLEXPORT __declspec(dllexport)
#endif
#include "my_class_fun.h"
#include "calculateTriangle.h"

