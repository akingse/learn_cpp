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

#define PROJECT_1_DLLEXPORT_DEFINE //set on pre-processor define
#ifdef PROJECT_1_DLLEXPORT_DEFINE // define both API and pch
#define DLLEXPORT_1 __declspec(dllexport)
#else
#define DLLEXPORT_1 __declspec(dllimport)
#endif

//component
#include "Gnrc.h"					//export
#include "BPParaVec.h"				//export
#include "BPParaTransform.h"		//export
#include "UtilitySet.h"


// my custom
#include "my_gnrc.h"				
#include "my_vec.h"					//export
#include "my_md5.h"					//export
#include "my_ref_count.h"			//export
#include "my_handle.h"				//export

//geometry
#include "geometry/ParaGeometryClass.h"             // public
#include "geometry/ParaGeometryCalculate.h"
#include "geometry/ParaGeometryRelation.h"         	// public

//delete python tool generate export API
//$(SolutionDir)tools\python3-embed\python.exe $(SolutionDir)tools\export_header.py  $(ProjectName)
// add head include
//$(ProjectDir)