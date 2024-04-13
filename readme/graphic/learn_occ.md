#  Open Cascade

开放级联技术Open Cascade Technology

![img](https://dev.opencascade.org/doc/overview/html/occt_logo.png)

[官网文档](https://dev.opencascade.org/doc/overview/html/index.html) 

[参考手册](https://dev.opencascade.org/doc/occt-7.6.0/refman/html/index.html)

[下载地址]( https://dev.opencascade.org/release)



OpenCascade

> OpenCascade技术是目前世界上唯一的开源软件开发平台。这家法国公司为三维曲面和实体建模、可视化和CAD数据交换提供服务和工具。
>
> OCCT的功能分为几个大模块。每个模块定义工具包（库）列表。主要模块：
>
> 基础类：定义基本类、内存分配器、OS抽象层、集合（数据映射、数组等）、加速数据结构（BVH树）和其他模块使用的向量/矩阵数学。
>
> 建模数据：提供数据结构来表示二维和三维几何图元（分析曲线：直线、圆、椭圆、双曲线、抛物线、Bézier、B样条、偏移；分析曲面：平面、圆柱体、圆锥体、球体、圆环体、Bézier、B样条、旋转、拉伸、偏移）及其组合到B-Rep模型中。
>
> 建模算法：包含大量几何和拓扑算法（交集、布尔运算、曲面网格划分、圆角、形状修复）。
>
> 可视化：提供在3D Viewer中显示几何图形的交互式服务；实现一个紧凑的OpenGL/OpenGL ES渲染器，支持传统的Phong、实时PBR金属粗糙度着色模型以及交互式光线跟踪/路径跟踪引擎。
>
> 数据交换：提供导入/导出各种CAD格式的可能性。
>
> STEP、IGES、glTF、OBJ、STL和VRML本机支持。[24]可以使用插件导入其他格式。[25]扩展数据交换（XDE）组件依赖于统一的XCAF文档定义，其中包括CAD形状的装配结构，颜色/名称/材质/元数据/图层属性以及其他补充信息，如PMI。
>
> 应用程序框架：提供处理特定于应用程序的数据的解决方案。
>
> DRAW Test Harness：基于Tcl解释器实现OCCT算法的脚本接口，用于交互式使用、自动化流程、原型应用程序和测试目的。
>
>  

 

OpenCascade提供二维和三维几何体的生成、显示和分析。

### 主要功能有：

1．创建锥、柱、环等基本几何体；

2．对几何体进行布尔操作（相加，相减，相交运算）；

3．倒角，斜切，镂空，偏移，扫视；

4．几何空间关系计算（法线，点积，叉积，投影，拟合等）；

5．几何体分析（质心，体积，曲率等）；

6．空间变换（平移，缩放，旋转）。

高级功能：

1．应用框架服务； 

2．数据交换服务。

 ![img](https://pic4.zhimg.com/80/v2-f397b72d89bc8e15a56a2b45ab34541b_1440w.webp)



---

[基础知识](https://www.cnblogs.com/mrliu0515/p/16211012.html)

[bilibili](https://space.bilibili.com/659981)

![image-20230325225408093](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325225408093.png)

 

 Open CASCADE （简称OCC）是一开源的几何造型引擎。基于该建模引擎发展了若干CAD/CAE/CAM软件，如国外的FreeCAD、HeeksCAD，国内的AnyCAD。Open CASCADE（简称OCC）为开源社区比较成熟的基于BREP结构的建模引擎，能够满足二维三维实体造型和曲面造型，OCC可以分为建模、可视化和数据管理（OCAF）三大模块。

 拥有高水平的内核，是发展自主CAD/CAM/CAE和的核心工作。

 然而市场需求并不支持。最大的几何内核Parasolid一年的生意，也就是3000多万美元。如果做内核，是不可能活不下去的。单独研发CAD几何内核是没有太大市场的，需要与三维CAD软件系统整合起来做。

 统一的数据结构，同时支持线框、CSG、B-Rep3种模型

 

---

使用教程

下载occ7.8.0+vs2022+qt6.4.2

qt5.12 [官网](https://www.qt.io/offline-installers) [安装包](https://download.qt.io/archive/qt/5.12/5.12.12/qt-opensource-windows-x86-5.12.12.exe)

[occ](https://dev.opencascade.org/release) 下载7.6.0历史版本，GitHub授权登录

VS管理扩展安装Qt插件后，在Qt版本里设置依赖路径；

![image-20240406190201643](C:/Users/Aking/AppData/Roaming/Typora/typora-user-images/image-20240406190201643.png)

学习研究OCC

occ文档有html版本，chm版本，pdf版本；

### 文件目录

- **adm** This folder contains administration files, which allow rebuilding OCCT;
- **adm/cmake** This folder contains files of CMake building procedure;
- **adm/msvc** This folder contains Visual Studio projects for Visual C++ 2010, 2012, 2013, 2015, 2017 and 2019 which allow rebuilding OCCT under Windows platform in 32 and 64-bit mode;
- **adm/scripts** This folder contains auxiliary scripts for semi-automated building and packaging of OCCT for different platforms;
- **data** This folder contains CAD files in different formats, which can be used to test the OCCT functionality;
- **doc** This folder contains OCCT documentation in HTML and PDF format;
- **dox** This folder contains sources of OCCT documentation in plain text (MarkDown) format;
- **inc** This folder contains copies of all OCCT header files;
- **samples** This folder contains sample applications.
- **src** This folder contains OCCT source files. They are organized in folders, one per development unit;
- **tests** This folder contains scripts for OCCT testing.
- **tools** This folder contains sources of Inspector tool.
- **win64/vc10** This folder contains executable and library files built in optimize mode for Windows platform by Visual C++ 2010;





