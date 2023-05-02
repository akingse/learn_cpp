### C++



C++main函数的argc与argv

- argc 是 argument count的缩写，表示参数个数（包含可执行程序）
- argv 是 argument vector 的缩写，表示传入mian函数的参数序列或者指针

```
#include <iostream>
using namespace std;

int main(int argc,char* argv[]){
    cout << "argc = " << argc << endl;//打印argc
    for (int i = 0; i < argc;i++){    //打印argv[]
        cout << "argv[" << i << "] = " << argv[i] << endl;
    }
}

```

执行的时候参数生效

![在这里插入图片描述](https://img-blog.csdnimg.cn/07a061732057427ea402d238489e028c.png)



### 函数指针

1. 指针函数：类型说明符 * 函数名（参数）;
    是一个函数，返回一个指针，实际上就是返回一个地址给调用函数
    在调用指针函数时，需要一个同类型的指针来接收其函数的返回值。
    也可以将其返回值设为void * 类型，调用时强制转换返回值为自己想要的类型

2. 函数指针：类型说明符 (* 函数名)(参数) ;

  ```c
  int (*FunPointerName)(int a,int b);
  ```

  是一个指针，指向函数的指针，包含了函数的地址，可以用它来调用函数，本质是一个指针变量，该指针指向这个函数
  把函数地址赋值给函数指针

  ```c
  FunPoniterName = &FunctionName;
  FunPoniterName = FunctionName;
  ```

  调用函数指针

  ```c
  x = (*FunPointerName)(参数);
  x = FunPointerName(参数);
  ```

  



区别

```c
int * FunName (int a,int b);
int (*FunName)(int a,int b);

#include <iostream>
using namespace std;
int add(int a,int b){
    return a + b;
}
int main(){
    int (*fun)(int a,int b);
    fun = add;
    cout << fun(10,20) << endl;
	cout << (*fun)(10,20) << endl;
}
```

函数指针与指针函数的最大区别是函数指针的函数名是一个指针，即函数名前面有一个指针类型的标志型号“*”。




