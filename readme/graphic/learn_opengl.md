

## 学习OpenGL

红宝书的《OpenGL编程指南》

### visual studio 中搭建Open GL开发环境



## GLFW

> GLFW 是配合 OpenGL 使用的轻量级工具程序库，缩写自 Graphics Library Framework（图形库框架）。GLFW 的主要功能是创建并管理窗口和 OpenGL 上下文，同时还提供了处理手柄、键盘、鼠标输入的功能。有类似功能的库还有 GLUT和 SDL。
> 

由于在Windows平台下初始化OpenGL非常麻烦，不仅需要创建OpenGL上下文，而且更麻烦的是调用OpenGL的GPU驱动的函数(具体多麻烦查看我的博文)。所以推荐使用第三方已经写好的库GLFW，类似的库还有GLUT。
GLFW是跨平台的抽象层，所以同一份代码在Windows和Linux下都可以运行。
GLFW提供了创建窗口，创建OpenGL上下文，管理键盘和鼠标的事件，甚至还提供多线程管理等跨平台的功能。功能越来越多了。

官方下载源码，使用cmake编译，生成vs解决方案

[git](https://github.com/glfw/glfw)

项目配置

```shell
#head
C:\Users\Aking\source\repos\TPL\glfw\deps;C:\Users\Aking\source\repos\TPL\glfw\include;
C:\Users\Aking\source\repos\TPL\glad\include
#lib
..\src\Debug\glfw3.lib;kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib
C:\Users\Aking\source\repos\TPL\glfw\build\src\Debug\
```



### GLAD在线转换

GLAD是一个开源的库，它能解决我们上面提到的那个繁琐的问题。GLAD的配置与大多数的开源库有些许的不同，GLAD使用了一个在线服务。在这里我们能够告诉GLAD需要定义的OpenGL版本，并且根据这个版本加载所有相关的OpenGL函数。

[GLAD](https://glad.dav1d.de/)

学习[OpenGL](https://learnopengl-cn.github.io/)

---



[songho](http://www.songho.ca/opengl/)

[翻译](https://blog.csdn.net/u012501459/article/list/8)



## 学习OpenVR

[github](https://github.com/ValveSoftware/openvr)

### 计算机图形学基础–OpenGL的实现

[bilibili](https://www.bilibili.com/video/BV1rL411x7KC/?spm_id_from=333.337.search-card.all.click&vd_source=dcb8b113123affd8262c0bfb1bbcbac3)

创建空项目，使用NuGet下载nupenggl.core

```c++
#include <GL/glut.h>

void init()
{
    glClearColor(1,1,1,0); //black
    glMatrixMode();//projection
    glLoadIdentity();
    glutOrtho2D(-100,100,-100,100); //display range
    glEnd();
    glFlush(); //fresh
}

void myPoint()
{
	glClear(); //
    glPointSize(3); //3 pixel
    glBegin(); //assign geometry type
    glColor3f(1,0,0); //red
    glVertex2i(-3,3); //assign coordi
    glColor3f(1,0,0); //red
    glVertex2i(3,3); //assign coordi
    
}
int main(int argc, char* argv[])
{
    glutInit(&argc,argv);
    glutInitDisplayMode();
    glutInitWindowPosition(100,300);
    glutInitWindowSize(300,300);
    gltuCeateWindow("display points");
    init(); //custom
    glutDisplayFunc(); //custom
    glutMainLoop();
    
    return 0;
}

```

