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







