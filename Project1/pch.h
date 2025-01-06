#pragma once

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
#define _USE_MATH_DEFINES //using M_PI
#include <math.h>
#include <complex>

//stl
#include <vector>
#include <map>
#include <array>
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

#define PROJECT_PPC_DLLEXPORT_DEFINE
#include "auto_include/ProjectPPC_API.h" //PublicAPI

//delete python tool generate export API
//$(SolutionDir)tools\python3-embed\python.exe $(SolutionDir)tools\export_header.py  $(ProjectName)
// add head include
//$(ProjectDir)