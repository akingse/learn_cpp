# ComputerGraphic

计算机图形学-渲染
现代计算机图形学入门(闫令琪)
Overview of Computer Graphics
资源
[课程ppt]( https://games-cn.org/graphics-intro-ppt-video/)
[在线平台](https://games-cn.org/) 

### Applications

游戏，电影，动画，设计，可视化，虚拟现实(VR-AR)，数字插画Digital llustration，模拟simulation，图形UI，字体typography
Fundamental Intellectual Challenges
创建真实的虚拟世界并与之交互
需要了解物理世界的各个方面
新的计算方法、显示、技术

### Technical Challenges

(透视)投影、曲线、曲面的数学
照明和遮阳物理学
用3D表示/操作形状
动画/仿真
3D绘图软件编程和硬件

computer graphics is awesome

Course Topics 4

### Rasterization光栅化

Project geometry primitives (3D triangles / polygons) onto the screen
投影几何原语(三维三角形/多边形)到屏幕上
Break projected primitives into fragments (pixels)
将投影原语分解为片段(像素)
Gold standard in Video Games (Real-time Applications) 实时显示，30fps
电子游戏 (实时应用)
Curves and Meshes曲线和网格
How to represent geometry in Computer Graphics
如何在计算机图形学中表示几何
Ray Tracing射线追踪
Shoot rays from the camera though each pixel
通过每个像素从照相机上拍摄光线
Calculate intersection and shading
计算交点和阴影

-Continue to bounce the rays till they hit light sources
-继续反射光线，直到它们击中光源
Gold standard in Animations / Movies (Offline Applications)
动画/电影中的金本位(离线应用)
//tradeoff
Animation / Simulation动画/仿真
Key frame Animation
关键帧动画
Mass-spring System
质量弹簧系统

Using OpenGL / DirectX / Vulcan
The syntax of Shaders

Computer Vision / Deep Learning

![image-20230325222439660](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325222439660.png)



---

[课程](http://www.cs.ucsb.edu/~lingqi/teaching/games101.html)

## 课程大纲：

### 【1】：计算机图形学概述

[官网](https://learnopengl-cn.github.io/) 

### 【2】：向量与线性代数 Review of Linear Algebra

基础数学
线性代数、计算、数据处理
基础物理 ：光学、力学
信号处理、数值分析、美学
More dependent on Linear Algebra
更多依赖于线性代数
Vectors (dot products, cross products,)
Matrices (matrix-matrix, matrix-vector mult., ...)

#### 基础概念

默认列向量，模长，单位向量，加法
点乘 Dot (scalar) Product
性质：交换律、结合律、分配律
Find angle between two vectors寻找两个向量间的角度
Finding projection of one vector on another求一个向量在另一个向量上的投影
Measure how close测量有多近
Decompose a vector分解向量 （perp垂直向量）
Determine forward /backward 确定前进/倒向（光追判定用）
叉乘
Cross product is orthogonal to two initial vectors交叉积与两个初始向量正交
Direction determined by right-hand rule由右手法则决定的方向
Useful in constructing coordinate systems (later)在构造坐标系统方面很有用(稍后)
叉乘设定：x*y=z右手系，x*y=-z左手系
unity左手系，direct3D左手系，OpenGL右手系
Determine left / right确定左/右
Determine inside / outside确定内/外（点在三角形内部）
判断点在三角形内部，单边法则（corner case）
坐标系正交基 Orthonormal bases and coordinate frames
矩阵
数乘，矩阵乘矩阵 rows columns行列
无交换律，有结合律，分配律，转置，逆矩阵； 
叉乘矩阵

![image-20230325222706284](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325222706284.png)

### 【3】：基础变换（二维）

Why study transformation .为什么要学习转变。
modeling模型变换，view视图变换，坐标系运动学变换，投影成像
2D transformations: rotation, scale, shear 2D变换：旋转、缩放、剪切
scale
reflect反射
shear切变
horizontal 水平
vertical 竖直
Homogeneous coordinates齐次坐标
Why homogeneous coordinates为什么齐次坐标
tradeoff，为了表示平移引入齐次坐标；

![image-20230325222905142](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325222905142.png)

 齐次坐标系下的增加维度，齐次坐标系下的点相加，得到这两个点的中电； 
Affine transformation仿射变换
Composing transforms复合变换
3D transformations三维变换
projection 投影

#### 【4】：基础变换（三维）

Transformation Cont 续
正交矩阵（inverse=transpose）
公式 Rodrigues’ Rotation Formula
视图和投影变换 View/Projection
MVP变换（model-view-projection）
Viewing (观测) transformation

View (视图) / Camera transformation 
相机有一个默认设置，朝向z负方向
$$
M_{view}=R_{view} T_{view}
$$

相机变换公式，模型由相对位置，应用同一个矩阵

![image-20230325223352632](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325223352632.png)



- ### 【5】：三维到二维变换（模型、视图、投影）

#### Projection (投影) transformation 

三维到二维的投影

**Orthographic projection**   (正交投影) 

camera在origin，看向-z，

canonical cube标准立方体，[-1, 1]^3（包围盒？），右手系，左右lr，上下bt，远近nf

![image-20230725232511115](../../../../../AppData/Roaming/Typora/typora-user-images/image-20230725232511115.png)

 **Perspective** (透视) projection，近大远小，更接近人眼，应用更广泛；

 Frustum 截锥，这个n是相机距离near clip plane的距离，还是透视投影的参考点距离；

![image-20231005192156361](../../../../../AppData/Roaming/Typora/typora-user-images/image-20231005192156361.png)



- ### 【6】：光栅化（离散化三角形）

####  Rasterization

透视投影，定义一个视锥，长宽比aspect ratio，视角field of view，

![image-20231005234332985](../../../../../AppData/Roaming/Typora/typora-user-images/image-20231005234332985.png)

after MVP，model view projection

Raster=Screen，pixel

像素坐标系，左下角原点，[0,0]-[width-1, height-1]

视口变换矩阵， M_viewport=translate(w/2,h/2,0)**scale(w/2,h/2,1)



- ### 【7】：光栅化（深度测试与抗锯齿）

三角形，最基础的多边形，一定共面，内外清晰，内部渐变； 

三角形与像素的关系，判断像素中心是否在三角形内； is_inside_2d(tri, x, y)

using包围盒，逐行扫描；锯齿aliasing

#### Antialiasing and Z-Buffering 抗锯齿

信号处理 jaggies锯齿 aliasing学名 artifact-固有缺陷

先模糊后采样Filter then sample，傅里叶展开，周期函数都可以写成cossin的线性组合

![image-20231025221000773](../../../../../AppData/Roaming/Typora/typora-user-images/image-20231025221000773.png)

MSAA 多重采样抗锯齿MultiSampling Anti-Aliasing

Occlusions and Visibility 遮挡和可见性



- ### 【8】：着色（光照与基本着色模型）

 illumination 光照

shading 着色

pipeline 图形管线

可见性 visibility occlusion

解决方法：深度缓存 z-buffer

画家算法，逐步遮挡覆盖

逐个像素计算深度

```c
for (each triangle T)
    for (each sample (x,y,z) in T)
        if (z < zbuffer[x,y]) // closest sample so far
            framebuffer[x,y] = rgb; // update color
            zbuffer[x,y] = z; // update depth
        else
        	; // do nothing, this sample is occluded

```



- ### 【9】：着色（着色频率、图形管线、纹理映射）

 shading 着色，引入明暗和颜色，应用不同材质 material；

shading是局部的，不考虑遮挡的阴影

布林冯-着色模型（3种光项，漫反射，高光，环境光）

漫反射 diffuse reflection

![image-20231116230138963](../../../../../AppData/Roaming/Typora/typora-user-images/image-20231116230138963.png)

max(0,n·l)，只考虑同向夹角；

漫反射与观察方向无关，即与v无关；

高光 specular

相机和光源夹角在范围内；半程向量，用于简化计算；

![image-20231124235354238](../../../../../AppData/Roaming/Typora/typora-user-images/image-20231124235354238.png)

衡量向量是否接近，指数p控制cos曲线下降，最终用于控制高光区域大小；

环境光 ambient

被环境（中的物体）反射的光线照亮；假设任何一个点接收到来自环境光是相同的；是一个常数，是某一种颜色，保证没有地方是黑的；
$$
L_a=k_a I_a\\

L=L_a+L_d+L_s
$$
着色频率 shading frequency

flat shading面face着色，gouraud shading顶点vertex着色，phong shading像素pixel着色；

如何确定polyface顶点的法向量，周围面法向量的（加权）平均；

逐像素的平均，Barycentric interpolation 重心插值

#### 渲染管线

 Graphics (Real-time Rendering)  Pipeline 合起来叫（图形概念太大）实时渲染管线；

 shader代码，控制顶点和像素如何进行着色；OpenGL vertex顶点着色器，fragment像素着色器

#### texture mapping 纹理贴图

三角形顶点到内部插值-重心坐标；

纹理是在定义三角形的像素属性；

插值

插值原因：（triangle）顶点才有数据，内部需要平滑过渡

插值内容：纹理坐标，颜色，法向量；

插值方法：重心坐标
$$
(x,y,z)=αA+βB +γC\\
α+β+γ=1\\
α>0,β>0,γ>0
$$
面积公式，简化叉乘面积公式，可得α,β,γ；

投影时，重心坐标可能会变，插值必须在投影前做；投影到二维后，使用逆变换把二维的像素点变回三维点，对三维点进行深度插值；

纹理应用Applying Textures

遍历光栅像素点(x,y)，重心插值出像素点的值，查询对应uv，应用值；

纹理放大 texture magn

bilinear双线性插值，插值3次；简单但效果差一些；

bicubic双三次插值；运算量较大，但效果更好；好的画质往往伴随更大的计算开销；

纹理过大，摩尔纹+锯齿；

 范围查询range query，用一种数据结构实现快速平均数计算；

mipmap，快速、近似、方形；

trilinear interpolation三线性插值在mipmap层与层之间进行插值；

各向异性过滤；

纹理应用-环境光照

in GPU，texture=data+mipmap

环境贴图（将环境光照记录作为贴图），凹凸贴图（记录相对高度，深度贴图）；

位移贴图（真的移动模型顶点），directX动态曲面细分；



- ### 【10】：几何（基本表示方法）

 几何表象，显式和隐式Implicit Explicit

隐式：判断点位容易，表示不直观；
$$
f(x,y,z)=0
$$
CSG 体素法

signed Distance Function距离函数

- 【11】：几何（曲线与曲面）

 显式表示法：三角面mesh，bezier曲面，细分曲面，NURBS，点云

polygon mesh 最广泛的应用；obj文件中，f v/vt/vn

Bezier曲线



- 【12】：几何（前沿动态）

 三角形面

mesh细分

loop subdivision 路普细分

mesh简化，二次误差度量



mesh正则化

- 【13】：光线追踪（基本原理）GAMES101_Lecture_13

 光栅化无法解决全局效果 global effects  （软阴影，glossy反射，间接光照）

光线追踪，非常慢，适合离线渲染；

光线三公理：1光沿直线传播，2光线不会碰撞collide，3从光源出发到相机过程光路可逆 reciprocity

pinhole小孔相机模型

设定：eye是一个点；光源是点光源；光线完美折射和反射；

做法：从像素投射光线，与场景相交得到交点，交点和光源连线，判定可见性和可见度，计算着色写回像素值；

![image-20240216234359150](../../../../../AppData/Roaming/Typora/typora-user-images/image-20240216234359150.png)

whitted风格

- primary ray  主射光线，
- secondary rays  弹射光线，
- shadow rays  投影光线

渲染：visible判断

几何：inside判断

DLSS深度学习超采样

RTXGI 实时全局光照ray tracing execute

**Uniform grids 网格化** 

**Spatial partitions  空间划分**

空间划分：Oct-tree，KD-tree，BSP-tree

KD-tree的问题，1同一个mesh可能会被多个node包含，2三角形与包围盒相交判断较难；

Bounding Volume Hierarchy (BVH)  层次包围体，主流做法

原来我之前写的kd-tree就是BVH，尴了个尬；

优化：选择最长的轴进行二分，查找中位数（快速选择算法，o(n)）



- ## 【14】：光线追踪（加速结构）

 

**Basic radiometry** (辐射度量学)  

Radiant flux辐射通量, 单位时间内的光量，lm流明
$$
Q[J=Joule] \\
\phi=dQ/dt \quad [W=watt]
$$

Radiant intensity光强, 光源向各个方向辐射能量，方向性的能量

intersity单位立体角上的能量；= 能量每单位立体角 solid angle
$$
I(\omega)=d\Phi/d\omega \\
[W/sr] [lm/sr=cd=candela] \\
\Omega=A/r^2
$$
![image-20240226231228772](../../../../../AppData/Roaming/Typora/typora-user-images/image-20240226231228772.png)

irradiance辐射照度, 物体表面接收到的能量；

irradiance是单位面积的能量，垂直方向投影；cos(theta)

能量衰减，E=phi/4pi，E'=E/r^2，光源在传播过程中，intersity不变，irradiance在衰减；
$$
E(x)=d\Phi(x)/dA \\
[W/m^2]\quad [lm/m^2=lux]
$$
radiance  辐射亮度，光线在传播中如何度量能量；

能量-单位立体角，单位面积；
$$
L(p,\omega)=d^2\Phi(p,\omega)/d\omega dAcos\theta \\
[W/sr \  m^2]\quad [cd/m^2=lm/sr\ m^2=nit]
$$

- 
  incident radiance 入射

- exiting radiance 出射


E=irradiance：dA区域收到的全部能量（integral积分，energy能量）

L=radiance：dA区域在dw方向接受的能量(light)

**BRDF 双向反射分布函数**

反射（漫/镜面）现象：光线发出，打到物体上，被反射出去，这个过程的函数（物理量）描述；

假设已知入射光和受光点 求反射光分布的函数 反之亦然 所以叫双向分布函数；

物体表面一个点在接收一束光照射之后会将这束光反射到空间半球中的任何一个可能的方向（取决于物体表面材质），BRDF描述的就是向某一个方向反射出去的光的“量”与这束入射光总的“量”的比值；

$$
f_r(\omega_i \rightarrow \omega_r)=dL_r(\omega_r)/dE_i(\omega_i)
$$

递归方程：物体表面不仅被光源照亮，还被其他物体的反射光线照亮；

渲染方程 RenderEquation，现代图形学的基础

![image-20240303224050126](../../../../../AppData/Roaming/Typora/typora-user-images/image-20240303224050126.png)
$$
L_o(p,\omega_o)= \\
L_e：自发光，e=Emission 发光体 \\
\Omega^+=H^2,表示半球，平面上的点只接收半球范围内的radiance \\
cos\theta=n\cdot \omega_i \\

L_i=Light \  incident
$$

解渲染方程，L = E + KL  算子分解

![image-20240306225245053](../../../../../AppData/Roaming/Typora/typora-user-images/image-20240306225245053.png)

发光体+直接光源+间接光照+二次反射

路径追踪， resolve render equation

概率论回顾

- X随机变量
- X~p(x) 概率分布
- 数学期望：E(X)=Σxipi
- 连续函数：E(X)=∫xp(x)dx 概率密度函数 PDF
- 随机变量函数的，数学期望 E(Y)=F[fx]=∫f(x)p(x)dx

通过概率的方法，去解渲染方程；

**蒙特卡洛积分**

积分域(a,b)内N等分

![image-20240304231859541](../../../../../AppData/Roaming/Typora/typora-user-images/image-20240304231859541.png)

**路径追踪 path tracing**

- diffuse漫反射
- glossy 哑光
- specular 镜面反射

幂级数问题，N=1的蒙特卡洛积分，叫路径追踪；（N!=1分布式光线追踪）

无限递归问题，俄罗斯轮盘赌概率停递归止，

**光源采样**

改变积分域，dw->dA

![image-20240312230852870](../../../../../AppData/Roaming/Typora/typora-user-images/image-20240312230852870.png)

- 1，对于光源部分，直接积分
- 2，其他反射，间接RR轮盘赌

Side Notes 旁注

路径追踪 indeed difficult

Blinn-Phong 光栅化

ray tracing==whitted style的光线追踪

path tracing

敬畏科学，这仅仅是个入门介绍；



- 【15】：路径追踪与光的传播理论

 

- 【16】：复杂外观建模与光的传播、实时光线追踪（前沿动态）

 

- ## 【17】：材质和外观

 散热现象

Material==BRDF

glossy 哑光

镜面反射（入射角=出射角）Perfect Specular Reflection  

折射，（玻璃）refraction

折射定律，不同介质有不同的折射率
$$
\eta_i sin\theta_i=\eta_t sin\theta_t
$$
全反射现象，Total internal reflection，从大eta到小eta，可见视角变小；

URDF+UTDF=USDF 散射scattering（Transmit折射）

菲涅尔项 Fresnel Reflection  Term

简化近似公式，schlick appro

微表面 microfacet

法向分布-> 

- Concentrated <==> glossy  
- Spread <==> diffuse  

表面方向性

- 各向同性 Isotropic  
- 各向异性 Anisotropic  拉丝金属

性质

- Non-negativity  非负
- Linearity  线性
- Reciprocity principle  可逆性
- Energy conservation  能量守恒
- Isotropic vs. anisotropic  异同

测量 BRDF



### 18 先进渲染

高级光线传播

Unbiased light transport methods 无偏

- Bidirectional path tracing (BDPT)双向路径追踪，较单向更耗时
- Metropolis light transport (MLT) 人名，适合复杂场景，难以估计收敛



Biased light transport methods
- Photon mapping 光子映射（有偏）适合渲染caustics，光线聚集形成的亮点
- Vertex connection and merging (VCM)



Instant radiosity (VPL / many light methods) 实时辐射度

外观建模

• Non-surface models

- Participating media 云雾 散射函数
- Hair / fur / fiber (BCSDF) 头发，marschner模型
- Granular material 颗粒材质

• Surface models

- Translucent material (BSSRDF) 半透明，透射材质，dipole appro（皮肤，玉石）次表面反射； 
- Cloth 布料，常用的三种方法
- Detailed material (non-statistical BRDF) 微观领域，波动光学（相对几何光学），

• Procedural appearance  程序化生成，noise三维函数





### 19 相机 镜头 光场

成像=合成+捕捉

Field of View (FOV)  视场

可视角 FOV=2*arctan(h/2*f)

相机镜头，通常使用标准传感器36*24，计算出等效焦距；

Expose 曝光

H=T*E，time\*irradiance

aperture光圈，f-number，光圈越小越接近小孔成像，没有景深；f是直径的倒数；

shuterr speed快门时间，导致运动模糊，

iso gain感光度，像素乘一个数，会放大噪声；

Thin Lens Approximation  等效焦距



### 【20】：颜色与感知

 

### 【21】：动画与模拟（基本概念、逆运动学、质点弹簧系统）

 

### 【21】：物质点法（前沿动态）