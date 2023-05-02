

### python[切片](https://www.jianshu.com/p/15715d6f4dad)

```python
切片操作基本表达式：
object[start_index:end_index:step]
```

step：正负数均可，其绝对值大小决定了切取数据时的‘‘步长”，而正负号决定了“切取方向”，正表示“从左往右”取值，负表示“从右往左”取值。

start_index：表示起始索引（包含该索引对应值）；该参数省略时，表示从对象“端点”开始取值，至于是从“起点”还是从“终点”开始，则由step参数的正负决定，step为正从“起点”开始，为负从“终点”开始。

##### 多层切片操作

```python
a = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
a[:8]=[0, 1, 2, 3, 4, 5, 6, 7]
a[:8][2:5]= [2, 3, 4]
a[:8][2:5][-1:] = [4]

```

##### 其他对象的切片操作

```ruby
>>> (0, 1, 2, 3, 4, 5)[:3]
>>> (0, 1, 2)
元组的切片操作

>>>'ABCDEFG'[::2]
>>>'ACEG'
字符串的切片操作

```

需要注意的是：**[:]和.copy()都属于“浅拷贝”，只拷贝最外层元素，内层嵌套元素则通过引用方式共享，而非独立分配内存**，如果需要彻底拷贝则需采用“深拷贝”方式；







---

### [struct.pack()](https://www.cnblogs.com/gala/archive/2011/09/22/2184801.html)和[struct.unpack()](https://blog.csdn.net/weiwangchao_/article/details/80395941)

struct模块中最重要的三个函数是pack(), unpack(), calcsize()

```python
#  按照给定的格式(fmt)，把数据封装成字符串(实际上是类似于c结构体的字节流)
pack(fmt, v1, v2, ...) 

# 按照给定的格式(fmt)解析字节流string，返回解析出来的tuple
unpack(fmt, string)       

# 计算给定的格式(fmt)占用多少字节的内存
calcsize(fmt)
```

**struct中支持的格式如下表：**

| Format | C Type               | Python             | 字节数 |
| ------ | -------------------- | ------------------ | ------ |
| `x`    | pad byte             | no value           | 1      |
| `c`    | `char`               | string of length 1 | 1      |
| `b`    | `signed char`        | integer            | 1      |
| `B`    | `unsigned char`      | integer            | 1      |
| `?`    | `_Bool`              | bool               | 1      |
| `h`    | `short`              | integer            | 2      |
| `H`    | `unsigned short`     | integer            | 2      |
| `i`    | `int`                | integer            | 4      |
| `I`    | `unsigned int`       | integer or long    | 4      |
| `l`    | `long`               | integer            | 4      |
| `L`    | `unsigned long`      | long               | 4      |
| `q`    | `long long`          | long               | 8      |
| `Q`    | `unsigned long long` | long               | 8      |
| `f`    | `float`              | float              | 4      |
| `d`    | `double`             | float              | 8      |
| `s`    | `char[]`             | string             | 1      |
| `p`    | `char[]`             | string             | 1      |
| `P`    | `void *`             | long               |        |



为了同c中的结构体交换数据，还要考虑有的c或c++编译器使用了字节对齐，通常是以4个字节为单位的32位系统，故而struct根据本地机器字节顺序转换.可以用格式中的第一个字符来改变对齐方式.定义如下：

| Character | Byte order             | Size and alignment      |
| --------- | ---------------------- | ----------------------- |
| `@`       | native                 | native      凑够4个字节 |
| `=`       | native                 | standard    按原字节数  |
| `<`       | little-endian          | standard    按原字节数  |
| `>`       | big-endian             | standard    按原字节数  |
| `!`       | network (= big-endian) | standard    按原字节数  |



