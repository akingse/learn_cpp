#pragma once
#define _USE_MATH_DEFINES //using M_PI

//stream
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm> 
#include <chrono>   
#include <iostream>
#include <memory>
#include <type_traits>
#include <time.h> //clock()函数
#include <ctime>    //用于产生随机数据的种子
#include <cassert>
#include <thread>
#include <windows.h>
#include <functional> //函数式编程

//math
#include <cmath>
#include <math.h>
#include <complex>

//stl
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <set>
#include <queue>
#include <stack>
#include <unordered_set>
#include <regex>

#include <process.h>
#include <cassert>
#include <typeinfo>
#include <typeindex> 
#include <type_traits>
#include <utility> 
#include <any> 
#include <mutex> 
#include <string_view>
#include <functional>
#include <memory_resource>
#include <Eigen/Dense>

// my custom
#include "Project1_API.h"
#include "test_serialize.h"
#include "my_geometry.h"			//export




//eigen
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::Matrix4d;

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

//typedef Vector3d Vec3;
//typedef Matrix4d BPTransfrom;
//typedef Matrix4d GeTransfrom;
//class None {};
//using Vec3 = Vector3d;