## C++11新特性

### 一、列表初始化

1.1 {}初始化

初始化列表(List Initialize)

以前{}只能用来初始化结构体和数组，而现在一切皆可用列表初始化。

 

1.2 initializer_list类型

### 二、类型推导

2.1 auto

auto的作用就是自动推导对象的类型。从这个意义上讲，auto并非一种"类型"声明，而是一个类型声明时的"占位符",编译器在编译时期会将auto替换为变量实际的类型。

 

2.2 auto注意事项

2.3 decltype

关键字decltype将变量的类型声明为表达式指定的类型。我前面用的typeid(x).name()可以拿到x类型的字符串，但是不能使用这个再去定义一个变量。

而decltype却可以拿到变量，后面还可以继续使用这个类型定义出变量。

 

### 三、新增与改进

3.1 nullptr

3.2 范围for

3.3 array

3.4 forward_list

3.5 unordered系列

3.6 final与override



---



bad_cast		: 通过dynamic_cast 抛出；
runtime_error	: 运行时异常，包括 3个自子类：
	overflow_error / underflow_error / range_error
logic_error		: 逻辑错误，包括 4个子类：
	domain_error / invalid_argument / out_of_range / length_error
bad_alloc		: new失败时，会抛出bad_alloc；