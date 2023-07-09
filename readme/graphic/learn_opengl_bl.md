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