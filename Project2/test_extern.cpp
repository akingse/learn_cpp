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
extern �����ñ����ڱ�ĵط��Ѿ��������,������Ҫʹ���Ǹ�����.
static ��ʾ��̬�ı����������ڴ��ʱ��, �洢�ھ�̬��,���洢��ջ����.
���ȣ�static��extern��һ�ԡ�ˮ���ݡ��ļһҲ����˵extern��static����ͬʱ����һ��������

��Σ�static���ε�ȫ�ֱ��������붨��ͬʱ���У�Ҳ����˵������ͷ�ļ���ʹ��static������ȫ�ֱ�������Ҳͬʱ�������ˣ�
���static����ȫ�ֱ�����������ֻ���Ǳ���ı��뵥Ԫ��Ҳ����˵���ġ�ȫ�֡�ֻ�Ա����뵥Ԫ��Ч���������뵥Ԫ�򿴲�����,��:

*/