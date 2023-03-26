[TOC]



# Geometry几何

计算机图形学的一部分（建模，渲染，动画）

### 数学基础-矩阵

[csdn](https://blog.csdn.net/weixin_50502862/article/details/129363271?spm=1001.2100.3001.7377&utm_medium=distribute.pc_feed_blog_category.none-task-blog-classify_tag-4-129363271-null-null.nonecase&depth_1-utm_source=distribute.pc_feed_blog_category.none-task-blog-classify_tag-4-129363271-null-null.nonecase
附加矩阵实现)

矩阵含义

noa三方向 normal orientation approach
$$
Matrix=\begin{bmatrix}
n_x & o_x & a_x & p_x\\
n_y & o_y & a_y & p_x \\
n_z & o_z & a_z & p_x \\
0 & 0 & 0 & 1 \\
\end{bmatrix} \tag{1}
$$

- 0，表示变换，表示状态
- 1，二维到三维，三维到二维的转换；
- 2，局部到世界，世界到局部的转换；

### 左乘右乘

左乘结果是 向量旋转 之后相对于原坐标系的位置， 右乘是参考系旋转移动后，向量(并未移动)相对于新参考系的坐标。

（齐次）矩阵既可以表示变换，也可以表示坐标系，像（三维）向量一样矩阵二义性；

### 矩阵分类

变换矩阵
刚性变换
T=translate() 
逆变换 T-1(t)= T(-t)
R=rotate()
任意旋转
arbitrary_rotate()
逆变换 R-1= RT
哈达马积（Hadamard product）
两个矩阵对应元素的乘积
matlab矩阵点乘
数字乘矩阵，逐个元素相乘；

线性变换
S=Scale()缩放
逆变换 S-1(s) = S(1/sx,1/sy,1/sz)
错切 shear() 
反射/镜像变换 reflect()  /mirror()

仿射affine 变换（线性变换（缩放、错切、反射、旋转）+平移）
projective 投影 / perspective 透视
rigid刚性/ linear 线性
isotropic scale 等比缩放
similitudes 相似变换
is_rigid_matrix



---

### 建模-造型接口