# C++四种cast操作符

[cast](https://blog.csdn.net/starryheavens/article/details/4617637)

C++标准定义了四个新的转换符：reinterpret_cast, static_cast, dynamic_cast和const_cast，目的在于控制类(class)之间的类型转换。

1.1       reinpreter_cast
用法：reinpreter_cast<type-id> (expression)

type-id必须是一个指针、引用、算术类型、函数指针或者成员指针。它可以把一个指针转换成一个整数，也可以把一个整数转换成一个指针。

这个操作符能够在非相关的类型之间转换。操作结果只是简单的从一个指针到别的指针的值的二进制拷贝。在类型之间指向的内容不做任何类型的检查和转换。reinpreter_cast是特意用于底层的强制转型，导致实现依赖（就是说，不可移植）的结果。

int n=9;

// reinterpret_cast 仅仅是复制 n 的比特位到 d，因此d 包含无用值。

double d=reinterpret_cast<double & > (n);

1.2       const_cast
用法：const_cast<type_id> (expression)

用于修改类型的const或volatile属性。除了const 或volatile修饰之外，type_id和expression的类型是一样的，一般用于强制消除对象的常量性。它是唯一能做到这一点的 C++ 风格的强制转型，而C不提供消除const的机制（已验证）。

常量指针被转化成非常量指针，并且仍然指向原来的对象；常量引用被转换成非常量引用，并且仍然指向原来的对象；常量对象被转换成非常量对象。

1.3       static_cast
用法：static_cast < type-id > ( expression )

该运算符把expression转换为type-id类型，但没有运行时类型检查来保证转换的安全性。它允许执行任意的隐式转换和相反转换动作。主要有如下几种用法：

1）用于基本数据类型之间的转换，如把int转换成char，non-const 对象转型为 const 对象（这里相反方向不可以，C++只有const_cast可以）。

2）把空指针转换成目标类型的指针。（之前的做法是用强制转换(type-id*)）

3）把任何类型的表达式转换成void类型。

4）应用到类的指针上，它允许子类类型的指针转换为父类类型的指针（upercasting这是一个有效的隐式转换）；也能够执行相反动作，即转换父类为它的子类（downcasting），这种转换的安全性需要开发人员来保证（主要是在非上下转型中）。

class Base {};

class Derived : public Base {};

Base *a = new Base;

Derived *b = NULL;

b = static_cast<Derived *>(a); //可以通过编译，但存在安全隐患(如访问//Derived的成员)

注意：

1．static_cast不能转换掉expression的const、volitale、或者__unaligned属性。

2．在非基本类型或上下转型中，被转换的父类需要检查是否与目的类型相一致，否则，如果在两个完全不相干的类之间进行转换，将会导致编译出错。

1.4       dynamic_cast
只用于对象的指针和引用，主要用于执行“安全的向下转型”，也就是说，要确定一个对象是否是一个继承体系中的一个特定类型。它是唯一不能用旧风格语法执行的强制转型，也是唯一可能有重大运行时代价的强制转型。

当用于多态类型时（包含虚函数），它允许任意的隐式类型转换以及相反过程。不过，与static_cast不同，在后一种情况里（即隐式转换的相反过程），dynamic_cast根据RTTI信息检查操作是否有效。即在转换时dynamic_cast会检查转换是否能返回一个被请求的有效的完整对象。这种检查不是语法上的，而是真实情况的检查。检测在运行时进行，如果被转换的指针不是一个被请求的有效完整的对象指针，返回值为NULL。

先看RTTI相关部分，通常，许多编译器都是通过vtable找到对象的RTTI信息的，这也就意味着，如果基类没有虚函数，也就无法判断一个基类指针变量所指对象的真实类型, 这时候dynamic_cast只能用来做安全的转换（upercasting），如从派生类指针转换成基类指针，而这种转换其实并不需要dynamic_cast参与。

class Base { virtual dummy() {} };

class Derived : public Base {};

class Other{} ;

Base* b1 = new Derived;

Base* b2 = new Base;

Derived* d1 = dynamic_cast<Derived *>(b1);  // succeeds

Derived* d2 = dynamic_cast<Derived *>(b2);  // fails: returns 'NULL'

//如果一个引用类型执行了类型转换并且这个转换是不可能的，运行时一个//bad_cast的异常类型会被抛出：

Derived d3 = dynamic_cast<Derived &>(*b1);  // succeeds

Derived d4 = dynamic_cast<Derived &>(*b2);  // fails: exception thrown

注意：Base需要有虚函数，否则会编译出错。

1.5       小结
四种类型转换操作符对于隐式的类型转换没有必要。

static_cast在更宽上范围内可以完成映射，这种不加限制的映射伴随着不安全性。在类层次间进行上行转换时，dynamic_cast和static_cast的效果是一样的；在进行下行转换时（基类需要包含虚函数），dynamic_cast具有类型检查的功能，牺牲了效率，但比static_cast安全。





---

const_cast 用于将const变成非cost

2、static_cast

　　用于各种隐式转换，比如非const转const，void*转指针等, static_cast能用于多态向上转化，如果向下转能成功但是不安全，结果未知。

3、 dynamic_cast

　　用于动态类型转换，只能用于含有虚函数的类，用于类层次之间的向上和向下转换，只能转指针或引用，如果是非法的对于指针返回NULL，对于引用抛异常。

4 、reinterpret_cast

　　几乎什么都可以转，比如将int转指针，可能会出问题，尽量少用；

