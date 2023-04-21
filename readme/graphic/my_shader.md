

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



定义

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



