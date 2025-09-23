#pragma once
#define _CRT_SECURE_NO_WARNINGS
//#define NDEBUG

//stream
#include<iostream>
#include<sstream>
#include<fstream>

//container
#include <vector>
#include <map>

#include <algorithm> 
#include <utility>
#include <random>
#include <chrono>

//win
#define _USE_MATH_DEFINES //using M_PI
#include <cstdlib>
#include <windows.h>
#include <stdio.h>
#include <direct.h>

//TPL
#include <Eigen/Dense>

//CGAL
#include <CGAL/point_generators_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/random_selection.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel	Kernel;
typedef CGAL::Point_2<Kernel>								Point2;
typedef Kernel::Point_2										Point2;
typedef CGAL::Creator_uniform_2<double, Point2>				Creator;
typedef std::vector<Point2>									Vector2;

//typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;
//typedef Eigen::Matrix<double, 3, 1> Vector3d;
//typedef Eigen::Matrix<double, 3, 3> Matrix3d;
//typedef Eigen::Matrix<double, 4, 4> Matrix4d;

#undef min
#undef max
#include "ProjectCAL_API.h"
