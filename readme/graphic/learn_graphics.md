# Computer Graphics

计算机图形学(Computer Graphics，简称CG)是一种使用数学算法将二维或三维图形转化为计算机显示器的栅格形式的科学。简单地说，计算机图形学的主要研究内容就是研究如何在计算机中表示图形、以及利用计算机进行图形的计算、处理和显示的相关原理与算法。

《计算机图形学》[刘利刚](https://www.bilibili.com/video/BV1iT4y1o7oM/?spm_id_from=333.337.search-card.all.click&vd_source=dcb8b113123affd8262c0bfb1bbcbac3)

课程内容：三维几何模型是描述三维空间内物体的表面或内部的数学表达，是计算机图形学的主要研究对象，是建模、仿真、渲染等处理过程的基础操作对象。本课程将介绍几何模型的各种表达方式、建模手段及处理方法，使得初步接触该领域的学生能够快速入门并掌握几何处理的基础算法及处理方法。主要内容包括：曲线曲面的基本理论、参数样条曲线曲面、三角网格曲面、离散微分几何、细分曲面及隐式曲面、网格曲面处理（去噪、参数化、纹理合成、简化、编辑、修复） 、形状分析与理解等。

### Computer graphics

虚拟现实、增强现实、混合现实（ Virtual reality、 Augmented Reality、Mixed reality）

- 建模(modeling): deals with the mathematical specification of shape and appearance properties in a 
  way that can be stored on computer. 
- 渲染(rendering): is a term inherited from art and deals with the creation of shaded images from 3D computer models. 
- 动画(animation): is a technique to create an illusion of motion through sequences of images.


用计算的方法，将物理世界映射到虚拟数字世界（即，对物体的几何与物理属性与规律进行表达与仿真，并真实地呈现与交互），再通过虚拟世界的计算最终反向作用于现实世界；

[什么是计算机图形学?](http://staff.ustc.edu.cn/~lgliu) 
计算机图形学是一种使用数学算法将二维或三维图形转化为计算机显示器的栅格形式的科学。简单地说，计算机图形学的主要研究内容就是研究如何在计算机中表示图形、以及利用计算机进行图形的计算、处理和显示的相关原理与算法。虽然通常认为CG是指三维图形的处理，事实上也包括了二维图形及图像的处理。
狭义地理解，计算机图形学是数字图象处理或计算机视觉的逆过程：计算机图形学是用计算机来画图像的学科，数字图象处理是把外界获得的图象用计算机进行处理的学科，计算机视觉是根据获取的图像来理解和识别其中的物体的三维信息及其他信息。
计算机图形学主要包含四大部分的内容：建模(Modeling)、渲染(Rendering)、动画(Animation)和人机交互(Human–computer Interaction, HCI)。

1、 建模(Modeling)

要在计算机中表示一个三维物体，首先要有它的几何模型表达。因此，三维模型的建模是计算机图形学的基础，是其他内容的前提。表达一个几何物体可以是用数学上的样条函数或隐式函数来表达；也可以是用光滑曲面上的采样点及其连接关系所表达的三角网格来表达（即连续曲面的分片线性逼近）

计算机辅助设计(CAD)中的主流方法是采用NURBS（非均匀有理B-样条、Bezier曲线曲面）方法（已成为CAD工业领域的标准），这也是计算机辅助几何设计(CAGD)所研究的主要内容。此类表达方法有一些难点问题仍未解决，比如非正规情况下的曲面光滑拼合，复杂曲面表达等。这部分涉及的数学比较多，国内做这块的学者比较多些。

数学
计算机图形学里面用到的数学比较多，列举一些常用的，包括：微积分、线性代数、矩阵计算、微分几何、数值计算和分析、计算方法、偏微分方程、微分方程数值解、最优化、概率、统计、计算几何等。

看似枯燥的数学到看到它的实际应用的过程中，你会更容易享受数学的美妙。
计算机图形学中的很多算法是真实物理世界的模拟，因此，如果你要进行基于物理的建模和仿真，一些物理知识和理论也需要的，比如力学（动力学，运动学，流体力学）和光学等；

[老虎书](https://blog.csdn.net/HW140701/article/details/122380227)

---



学习笔记

 

### 渲染管线（Render Pipeline）

1. 什么是渲染管线

渲染管线（渲染流水线）是将三维场景模型转换到屏幕像素空间输出的过程。图形渲染管线接受一组3D坐标，然后把它们转变为屏幕上的有色2D像素输出。

![image-20230325222104884](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325222104884.png)

流程图中展现了几何阶段中几个常见的渲染步骤（不同的图像应用接口存在着些许不同，这里以OpenGL为例），其中，绿色表示开发者可以完全编程控制的部分，虚线外框表示此阶段不是必需的，黄色表示开发者无法完全控制的部分（但可以进行一些配置），紫色表示开发者无法控制的阶段（已经由GPU固定实现）。

 

计算机图形学的一项主要工作是将计算机中抽象的模型转换为人们直观可见、可以形象理解的图形。它综合利用数学、物理学、计算机等知识，将模型的形状、物理特性(如材料的折射率、反射率、物体发光温度等，机械强度、材料密度等对运动模拟的影响等)，以及物体间的相对位置、遮挡关系等性质在计算机屏幕上模拟出来，是一个将“几何”演绎到画面上的再创造过程，这就是渲染，也叫绘制。

### 什么是渲染？ 

当需要把模型或者场景输出成图像文件、视频信号或者电影胶片时就需要用到渲染（Render）。渲染是指软件由模型生成图像的过程。模型是用语言或者数据结构进行严格定义的三维物体或虚拟场景，它包括几何、视点、纹理、照明等信息。图像是数字图像或者位图图像。除去后期制作，渲染是计算机图形处理的最后一道工序，通过它得到模型与动画的最终显示效果。实现渲染依靠多种软件，如各种 CG 软件自带渲染引擎、RenderMan 等。



渲染技术

![image-20230325222132936](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325222132936.png)





# shader

Shader代码是用来控制图形处理单元（GPU）的程序，通过它可以实现各种图形效果和渲染技术。以下是一些基本的Shader代码编写步骤：

1. 了解各种Shader类型：包括顶点着色器(Vertex Shader)、片段着色器(Fragment Shader)、几何着色器(Geometry Shader)等。
2. 选择合适的编程语言：常用的编程语言有GLSL、HLSL和Cg。
3. 理解输入和输出：Shader的输入通常是顶点位置、纹理坐标和顶点颜色等，输出通常是颜色或深度值。
4. 定义Uniform变量：Uniform变量是可以在C++代码中动态修改的Shader变量。
5. 编写Shader代码：根据需要实现Shader功能，一般包括数学运算、纹理采样和光照计算等。
6. 编译Shader代码：使用GPU Shader编译器将Shader代码编译成GPU可执行的二进制形式。
7. 与程序集成：将Shader代码与程序集成，将Shader程序上传到GPU中进行渲染。

实现复杂的Shader效果需要深入理解图形编程、线性代数和物理学等方面的原理知识。建议初学者可以从基础着手，例如参考OpenGL或Unity官方文档中提供的Shader例子来学习。

### Shader定义

着色器（Shader）是用来实现图像渲染的，用来替代固定渲染管线的可编辑程序。其中Vertex Shader（顶点着色器）主要负责顶点的几何关系等的运算，Pixel Shader（像素着色器）主要负责片元颜色等的计算。

![图1](https://bkimg.cdn.bcebos.com/pic/e850352ac65c10387059cd72ba119313b07e895e?x-bce-process=image/resize,m_lfit,w_1280,limit_1)

学习Shader之前，了解计算机图形学的基础知识是必须的，如果没有图形学的基础，直接学习写Shader就好比建一座空中阁楼。

主流的Shader语言（HLSL和GLSL）

关于图形图像研究的学科很多，主要分为计算几何（computer geometry）、计算机图形学(computer graphy)、计算机视觉(computer vision)、数字图像处理(digital image processing)，各自关系如下

![img](https://pic3.zhimg.com/v2-360aa420916bbe1a340e04e87cfe290e_r.jpg)

**具体API（OpenGL / DirectX / Vulkan）**

**软件（Maya / 3DS MAX / Blender/ Unity / Unreal Engine）**



学习shader

了解计算机图形学的基础知识是必须的，如果没有图形学的基础，直接学习写Shader就好比建一座空中阁楼。

一边书学习基本概念（顶点、索引、空间变换、纹理映射、基本光照模型、GPU架构等），一边学习图形渲染API（OpenGL or DirectX）

《OpenGL编程指南》（俗称OpenGL红宝书）



---

[shadertoy](https://www.shadertoy.com/)

Shader（着色器）是一段能够针对3D对象进行操作、并被GPU所执行的程序。Shader并不是一个统一的标准，不同的图形接口的Shader并不相同。OpenGL的着色语言是GLSL， NVidia开发了Cg，而微软的Direct3D使用高级着色器语言（HLSL）。而Unity的Shader 是将传统的图形接口的Shader（由 Cg / HLSL编写）嵌入到独有的描述性结构中而形成的一种代码生成框架，最终会自动生成各硬件平台自己的Shader，从而实现跨平台。所以说Shaer其实就是一门语言，用来控制渲染的语言。

OpenGL-GLSL

NVidia-Cg

Direct3D-HLSL

**OpenGL和Direct3D都提供了三类着色器：**

顶点着色器：处理每个顶点，将顶点的空间位置投影在屏幕上，即计算顶点的二维坐标。同时，它也负责顶点的深度缓冲（Z-Buffer）的计算。顶点着色器可以掌控顶点的位置、颜色和纹理坐标等属性，但无法生成新的顶点。顶点着色器的输出传递到流水线的下一步。如果有之后定义了几何着色器，则几何着色器会处理顶点着色器的输出数据，否则，光栅化器继续流水线任务。

像素着色器(Direct3D)，常常又称为片断着色器(OpenGL)：处理来自光栅化器的数据。光栅化器已经将多边形填满并通过流水线传送至像素着色器，后者逐像素计算颜色。像素着色器常用来处理场景光照和与之相关的效果，如凸凹纹理映射和调色。名称片断着色器似乎更为准确，因为对于着色器的调用和屏幕上像素的显示并非一一对应。举个例子，对于一个像素，片断着色器可能会被调用若干次来决定它最终的颜色，那些被遮挡的物体也会被计算，直到最后的深度缓冲才将各物体前后排序。

几何着色器：可以从多边形网格中增删顶点。它能够执行对CPU来说过于繁重的生成几何结构和增加模型细节的工作。Direct3D版本10增加了支持几何着色器的API, 成为Shader Model 4.0的组成部分。OpenGL只可通过它的一个插件来使用几何着色器。



---

X-Ray特效是一种将物体内部的结构和材质呈现出来的效果。以下是一个简单的基于Unity Shader的X-Ray特效实现方法：

```c
Shader "Custom/X-Ray" {
    Properties {
        _MainTex ("Texture", 2D) = "white" {}
        _XRayColor ("X-Ray Color", Color) = (1,1,1,1)
        _XRayPower ("X-Ray Power", Range(0.1, 10)) = 1
        _RimPower ("Rim Power", Range(0.1, 10)) = 1
    }
    SubShader {
        Tags {"Queue"="Transparent" "RenderType"="Opaque"}
        LOD 100

        Pass {
            CGPROGRAM
            #pragma vertex vert 
            #pragma fragment frag 

            #include "UnityCG.cginc"
            
            struct appdata {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };
            
            struct v2f {
                float2 uv : TEXCOORD0;
                float4 pos : SV_POSITION;
            };
            
            sampler2D _MainTex;
            float4 _MainTex_ST;
            float _XRayPower;
            float _RimPower;
            fixed4 _XRayColor;
            
            v2f vert (appdata v) {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                return o;
            }
            
            fixed4 frag (v2f i) : SV_Target {
                // X-Ray Color
                fixed4 xray = _XRayColor;
                xray.a = 0.3;
                // Compute Fresnel Rim Lighting
                float rim = 1 - saturate(dot(normalize(i.uv - 0.5), _WorldSpaceCameraPos.xyz));
                // X-Ray Power
                float xrayPower = _XRayPower;
                
                // X-Ray Effect
                fixed4 col = tex2D(_MainTex, i.uv);
                col *= xrayPower * xray + (1 - xrayPower) * rim * rim * xray;
                return col;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}
```

该Shader代码包含以下几个部分：

1. 属性定义：定义了与该着色器相关的参数；

2. 输入输出结构体定义：定义了其输入输出的数据格式；

3. Shader调用部分：包含了vertex和fragment两个方法；

4. 定义变量：定义了该shader中所需要的变量以便于控制；

5. vertex函数：通过输入结构体中的appdata v返回一个自定义类型v2f；

6. frag函数：渲染时执行该函数部分代码；

7. 集成到Unity中，实现X-Ray特效。

注意，该Shader只是一个简单的示例，实现的效果并不十分真实，实现复杂且真实的X-Ray特效需要深入理解物理学和渲染编程原理。



