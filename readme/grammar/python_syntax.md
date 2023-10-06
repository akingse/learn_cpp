## [runboob](https://www.runoob.com/python/python-basic-syntax.html)

### Python 标识符

在 Python 里，标识符由字母、数字、下划线组成。

在 Python 中，所有标识符可以包括英文、数字以及下划线(_)，但不能以数字开头。

Python 中的标识符是区分大小写的。

以下划线开头的标识符是有特殊意义的。以单下划线开头 **_foo** 的代表不能直接访问的类属性，需通过类提供的接口进行访问，不能用 **from xxx import \*** 而导入。

以双下划线开头的 **__foo** 代表类的私有成员，以双下划线开头和结尾的 **__foo__** 代表 Python 里特殊方法专用的标识，如 **__init__()** 代表类的构造函数。

Python 可以同一行显示多条语句，方法是用分号 **;** 分开，如：

```python
print ('hello');print ('runoob');
```

### Python 保留字符（关键字）

```

```

多行语句

```python
# 使用斜杠（ \）将一行的语句分为多行显示
total = item_one + \
        item_two + \
        item_three
        
days = ['Monday', 'Tuesday', 'Wednesday',
        'Thursday', 'Friday']
```

Python 引号

```python
# 引号( ' )、双引号( " )、三引号( ''' 或 """ ) 来表示字符串
word = 'word'
sentence = "这是一个句子。"
paragraph = """这是一个段落。
包含了多个语句"""
# 三引号可以由多行组成，编写多行文本的快捷语法，常用于文档字符串，在文件的特定地点，被当做注释。
```



### 空行

记住：空行也是程序代码的一部分。（这是计划的一部分）

函数之间或类的方法之间用空行分隔，表示一段新的代码的开始。类和函数入口之间也用一行空行分隔，以突出函数入口的开始。

### print 输出

print 默认输出是换行的，如果要实现不换行需要在变量末尾加上逗号 **,**。



[list操作](https://www.runoob.com/python/python-lists.html)

---





### 函数传参

显式指定函数的参数类型及返回值类型：

```python
def function_demo(param_A: int, param_B: float, param_C: list, param_D: tuple) -> dict:
    pass
```



### 关键字 keyword.kwlist

['False', 'None', 'True', 'and', 'as', 'assert', 'break', 'class', 'continue', 'def', 'del', 'elif', 'else', 'except', 'finally', 'for', 'from', 'global', 'if', 'import', 'in', 'is', 'lambda', 'nonlocal', 'not', 'or', 'pass', 'raise', 'return', 'try', 'while', 'with', 'yield']

 

 r 指 raw，即 raw string，会自动将反斜杠转义

print(r'\n')   # 输出 \n

 

索引改值

不可变数据（3 个）：Number（数字）、String（字符串）、Tuple（元组）；

可变数据（3 个）：List（列表）、Dictionary（字典）、Set（集合）。

支持

索引查询，切片，append()增加、pop()删除；

string、list 和 tuple 都属于 sequence（序列），可索引；set不可索引、dict通过键（key）来进行索引；

 

type()不会认为子类是一种父类类型。

isinstance()会认为子类是一种父类类型。

 

/ 返回一个浮点数，// 返回一个整数。在混合计算时，Python会把整型转换成为浮点数。

函数返回单个值/多个值，默认为元组return ()，当然也可以手动 return [] 返回数组；

 

python 中一切都是对象，比如这个a=1

a=1 #变量 a 是没有类型，她仅仅是一个对象的引用（一个指针），可以是 List 类型对象，也可以指向 String 类型对象。

一看数据类型，二看修改方法

不可变数据

num 不可变类型：变量赋值 a=5 后再赋值 a=10，这里实际是新生成一个 int 值对象 10，再让 a 指向它，而 5 被丢弃，不是改变a的值，相当于新生成了a。

change(a)# 传递的只是a的值，没有影响a对象本身。比如在 fun（a）内部修改 a 的值，只是修改另一个复制的对象，不会影响 a 本身。

\# 调用函数前后，形参和实参指向的是同一个对象（对象 id 相同），在函数内部修改形参后，形参指向的是不同的 id。

 

### 函数参数

*星号，可变长参数，参数解包；

*args 自动处理为元组

**args 自动处理为字典

 

调用函数时可使用的正式参数类型：

必需参数 #def printme( str ):调用时的数量必须和声明时的一样。

关键字参数 #printme( str = "菜鸟教程")函数调用使用关键字参数来确定传入的参数值。

默认参数 #def printinfo( name, age = 35 ):调用函数时，如果没有传递参数，则会使用默认参数。

不定长参数 #加了星号 * 的参数会以元组(tuple)的形式导入，存放所有未命名的变量参数。

加了两个星号 ** 的参数会以字典的形式导入。def printinfo( arg1, **vardict ): printinfo(1, a=2,b=3)

如果单独出现星号 * 后的参数必须用关键字传入。def f(a,b,*,c): f(1,2,c=3) 

lambda 来创建匿名函数。sum = lambda arg1, arg2: arg1 + arg2

强制位置参数 / 用来指明函数形参必须使用指定位置参数，* 指明使用关键字参数的形式。

 

————————————————————————————————————————————————



### PY语法

def func->None() # ->

为函数添加元数据,描述函数的返回类型;

使用预期的类型来注释参数，然后在函数返回值验证时检验参数的类型或者将其强制转换成预期的类型。

 

self，当前类对象

_name 私有函数，外部无法调用；

__name__  python内置类属性

[常用属性](https://www.cnblogs.com/yangliguo/p/8178135.html) 

 



### @修饰器

Python一切皆对象

def hi():

he=hi

print(he)

print(he())

变量<->对象

小括号放在后面，这个函数就会执行；然而如果你不放括号在它后面，那它可以被到处传递

Python 中的函数和C++不太一样，Python 中的函数可以像普通变量一样当做参数传递给另外一个函数

装饰器

@property就是将定义的函数（方法）当作属性对象使用，不需要像调用函数那样去调用，而@xxx.setter是为@xxx的这样函数进行值的设置，

 

一切皆对象，包括函数

能在函数中定义一个函数

能作为参数传递

能作为返回值

@preprocess_function 装饰器

功能：在执行函数之前干预函数

 

### python 鸭子类型

python不支持多态也用不到多态，多态的概念是应用于java和C#这一类强类型语言中，而Python崇尚鸭子类型（Duck Typing）

鸭子类型：是一种动态类型的风格。一个对象有效的语义，不是由继承自特定的类或实现特定的接口，而是由当前方法和属性的集合决定。这个概念的名字来源于由James Whitcomb Riley提出的鸭子测试，“鸭子测试”可以这样表述：

“当看到一只鸟走起来像鸭子、游泳起来像鸭子、叫起来也像鸭子，那么这只鸟就可以被称为鸭子。”

在鸭子类型中，关注的不是对象类型本身，而是它是如何使用的。我们可以编写一个函数，它接受一个类型为鸭的对象，并调用它的走和叫方法。在使用鸭子类型的语言中，这样的一个函数可以接受一个任意类型的对象，并调用它的走和叫方法。如果这些需要被调用的方法不存在，那么将引发一个运行时错误。



In general, a value in a programming language is said to have First-class status if it can be passed as a parameter, returned from a subroutine, or assigned into a variable.

也就是说，在编程语言中，一等公民可以作为函数参数，可以作为函数返回值，也可以赋值给变量。

 

### list删除操作

1. pop()

1.默认删除最后一个元素.pop()中也可以传入参数,为list的索引

2.pop() 接收的是索引，无参的情况下删除的是最后一个元素（典型的栈的特性）

3.pop() 存在返回值，返回的是删除的元素值

 

2.del

删除指定索引位置

list=[11,12,13,14,15]

del(list[1])

print(list)

 

3.remove

remove() 的参数是具体的元素值，而不是索引，

 

list=[11,12,13,14,15]

list.remove(11)

 

---



