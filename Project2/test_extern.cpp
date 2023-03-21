//#include "pch.h"
using namespace std;
#include <map>
#include <string>

int main_ex()
{
    //func(); //1
    extern int num;
    printf("%d", num); //2

    map<int, string> amap;
    amap[0] = "0";
    amap[1] = "1";
    amap[2] = "2";

	string expre = "{2023} = {2022}";
	//analysisExpressionTest1(expre);

	if (amap.find(3) != amap.end()) //need check
        amap.at(3) = "3";
    return 0;
}


//int num = 3;
//void func()
//{
//    printf("%d\n", num);
//}


/*
extern 表明该变量在别的地方已经定义过了,在这里要使用那个变量.
static 表示静态的变量，分配内存的时候, 存储在静态区,不存储在栈上面.
首先，static与extern是一对“水火不容”的家伙，也就是说extern和static不能同时修饰一个变量；

其次，static修饰的全局变量声明与定义同时进行，也就是说当你在头文件中使用static声明了全局变量后，它也同时被定义了；
最后，static修饰全局变量的作用域只能是本身的编译单元，也就是说它的“全局”只对本编译单元有效，其他编译单元则看不到它,如:

*/