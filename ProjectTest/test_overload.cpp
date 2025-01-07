#include "pch.h"
using namespace std;

class Box
{
public:

    double getVolume(void)
    {
        return length * breadth * height;
    }
    void setLength(double len)
    {
        length = len;
    }

    void setBreadth(double bre) //宽
    {
        breadth = bre;
    }

    void setHeight(double hei)
    {
        height = hei;
    }
    // 重载 + 运算符，用于把两个 Box 对象相加
    Box operator+(const Box& b)
    {
        Box box;
        box.length = this->length + b.length;
        box.breadth = this->breadth + b.breadth;
        box.height = this->height + b.height;
        return box;
    }
    Box operator*(const int& b)
    {
        Box box;
        box.length = this->length * b;
        box.breadth = this->breadth * b;
        box.height = this->height * b;
        return box;
    }
    //friend Box operator*(const int& b, Box box); //如果是私有则需要加友元限定

//private:
public:

    double length;      // 长度
    double breadth;     // 宽度
    double height;      // 高度

    
};

Box operator*(const int& b, Box box)
{
    box.length = box.length * b;
    box.breadth = box.breadth * b;
    box.height = box.height * b;
    return box;
}

// 程序的主函数
int main_o()
{



    Box Box1;                // 声明 Box1，类型为 Box
    Box Box2;                // 声明 Box2，类型为 Box
    Box Box3;                // 声明 Box3，类型为 Box
    double volume = 0.0;     // 把体积存储在该变量中

    // Box1 详述
    Box1.setLength(1.0);
    Box1.setBreadth(1.0);
    Box1.setHeight(1.0);

    // Box2 详述
    Box2.setLength(2.0);
    Box2.setBreadth(2.0);
    Box2.setHeight(2.0);

    // Box1 的体积
    volume = Box1.getVolume();
    cout << "Volume of Box1 : " << volume << endl;

    // Box2 的体积
    volume = Box2.getVolume();
    cout << "Volume of Box2 : " << volume << endl;

    // 把两个对象相加，得到 Box3
    //Box3 = Box1 + Box2;

    //// Box3 的体积
    //volume = Box3.getVolume();
    //cout << "Volume of Box3 : " << volume << endl;
    Box3 = 3*Box2 ;
    //Box3 = Box2*2;
    volume = Box3.getVolume();
    cout << "Volume of Box3 : " << volume << endl;

    vector<int> v1 = { 1,2 };
    //int a, b = vec[0],vec[1];
    // vector拼接
    std::vector<int> v3={3, 4};
    v3.insert(v3.end(), v1.begin(), v1.end());
    int a = 1;
    cout << v3.back() << endl;
    cout << *v3.begin() << endl;
    cout << &a << endl;
    
    //排序
    sort(v3.begin(), v3.end());//从小到大
    reverse(v3.begin(), v3.end());//从大到小 

    return 0;
}