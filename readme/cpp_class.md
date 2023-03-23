[TOC]

---

## 类和对象



### 虚函数表指针 vtptr

1. 用virtual关键字申明的函数叫做虚函数，虚函数肯定是类的成员函数。

2. 存在虚函数的类都有一个一维的虚函数表叫做虚表。每一个类的对象都有一个指向虚表开始的虚指针。虚表是和类对应的，虚表指针是和对象对应的。

3. 虚函数表是一个存储 **类成员函数指针** 的数据结构

4. 虚函数表是由编译器自动生成和维护的，virtual成员函数会被编译器放入虚函数表中

5. 存在虚函数时，每个对象中都有一个指向虚函数表的指针（vptr指针）

6. 多继承会存在多个虚表指针

   
   
7. 虚表指针：指向虚函数表首地址的一个指针，存在于每个基类对象的内存中，在调用构造函数构造对象时，设置虚表指针__vfptr

8. 虚函数表：在`编译阶段生成`，编译器将类中虚函数的地址存放在虚函数表中，虚函数表存在于全局数据区.data，每个类仅有一个，供所有对象共享。

   

### C++多态

可分为静态多态和动态多态

- 静态多态就是在系统编译期间就可以确定程序执行到这里将要执行哪个函数，例如函数重载
- 动态多态则是利用虚函数实现，在系统编译的时候并不知道程序将要调用哪一个函数，只有在运行到这里的时候才能确定接下来会执行什么函数



### this指针

类的成员变量和成员函数是分开存储的，只有非静态成员变量才属于实例化出来的对象，每个对象都有一份。非静态成员函数只有一份。
类的每一个非静态成员函数都有一个this指针。而且是指针常量。即T* const this类型。T为类型。所以可以看出不能更改this指针的指向。而且是这个形参是隐式包含在形参列表中的。this指针指向调用这个成员函数的对象本身。

用途：
（1）区分与成员变量相同名称的形参。
即如果成员函数的形参与成员变量名相同，那么可以用this->“成员变量名”，以这种形式区分成员变量和形参。
（2）返回对象本身。
this是指向对象本身的，那么在成员函数体内，*this就是指对象本身。



### 静态成员函数

（1）静态成员函数没有this指针
（2）所有对象共享一个静态成员函数。
（3）静态成员函数只能访问静态成员变量（没有this指针）。
（4）静态成员函数类内类外都可以定义。
（5）静态成员函数受访问权限的限制
访问静态成员变量的方式：
（1）通过对象访问
（2）通过域名访问

[成员指针](https://blog.csdn.net/LaoJiu_/article/details/68946915?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-68946915-blog-76512298.pc_relevant_multi_platform_whitelistv3&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-68946915-blog-76512298.pc_relevant_multi_platform_whitelistv3&utm_relevant_index=1)

### 一：成员变量指针

1.1 非静态成员指针

[类成员](https://so.csdn.net/so/search?q=类成员&spm=1001.2101.3001.7020)变量指针，实际上并不是真正意义上的指针，即它并不是指向内存中某个地址，而是该成员变量与对象指针的偏移量。该偏移量只有附着在某个具体对象，才能指向对象成员变量的具体地址。

### 二：成员函数指针

2.1 非静态函数指针

C++非静态成员函数的指针，其值就是指向一块内存地址。但是指针不能直接调用，它需要绑定到一个对象才能调用。 例子程序如下：

```cpp
class A 
{
public:
    static void staticmember(){cout<<"static"<<endl;}   //static member
    void nonstatic(){cout<<"nonstatic"<<endl;}          //nonstatic member
    virtual void virtualmember(){cout<<"virtual"<<endl;};//virtual member
};
int main()
{
    A a;
    //static成员函数,取得的是该函数在内存中的实际地址，而且因为static成员是全局的，所以不能用A::限定符
    void (*ptrstatic)() = &A::staticmember;      
    //nonstatic成员函数 取得的是该函数在内存中的实际地址     
    void (A::*ptrnonstatic)() = &A::nonstatic;
    //虚函数取得的是虚函数表中的偏移值，这样可以保证能过指针调用时同样的多态效果
    void (A::*ptrvirtual)() = &A::virtualmember;
    //函数指针的使用方式
    ptrstatic();
    (a.*ptrnonstatic)();
    (a.*ptrvirtual)();
}




```

