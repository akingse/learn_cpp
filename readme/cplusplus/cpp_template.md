### 模板条件

std::void_t<>
使用场景，配合std::false_type 与std::true_type
判断一个类当中是否存在一个类型别名，成员变量，函数等。

### decltype()

类型说明符，它的作用是选择并返回操作数的数据类型。
decltype，在C++中，作为操作符，用于查询表达式的数据类型。decltype在C++11标准制定时引入，主要是为泛型编程而设计，以解决泛型编程中，由于有些类型由模板参数决定，而难以（甚至不可能）表示之的问题。

### std::declval<T>

std::declval的功能：返回某个类型T的右值引用，不管该类型是否有默认构造函数或者该类型是否可以创建对象。

布尔模板函数特化

```cpp
    template <bool B = __has_transform<T>>
    void __transform(BPGeometricPrimitive& imp, const BPParaTransform& transform, bool local) const;
    template <>
    void __transform<true>(BPGeometricPrimitive& imp, const BPParaTransform& transform, bool local) const
    {}
    template <>
    void __transform<false>(BPGeometricPrimitive& imp, const BPParaTransform& transform, bool local) const
    {}
```

### 1.模板特化

1.1 概述
模板特化（template specialization）不同于模板的实例化，模板参数在某种特定类型下的具体实现称为模板特化。模板特化有时也称之为模板的具体化，分别有函数模板特化和类模板特化。

2.模板偏特化
模板的偏特化是指需要根据模板的某些但不是全部的参数进行特化
2.1 概述
模板偏特化（Template Partitial Specialization）是模板特化的一种特殊情况，指显示指定部分模板参数而非全部模板参数，或者指定模板参数的部分特性分而非全部特性，也称为模板部分特化。与模板偏特化相对的是模板全特化，指对所有模板参数进行特化。模板全特化与模板偏特化共同组成模板特化。

对主版本模板类、全特化类、偏特化类的调用优先级从高到低进行排序是：全特化类>偏特化类>主版本模板类。这样的优先级顺序对性能也是最好的。



### 可变模板

C++11 可变参数模板

```cpp
template<class …Args>
返回类型 函数名(Args… args)
{
  //函数体
}
```

其特殊性在于 ... 的使用，可变参数模板，通过使用 ... 来帮助定义，其中，... 左侧为参数包（parameter pack ），右侧将参数包展开成多个单独的参数。