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











### 拓扑多边形自相交判断及纠正方法 

在原多边形内部构建新多边形， 通过判定新多边形的顶点是否为原多边形内点实现了多边形自相交的判断。  

f=Ax+By+C

当 f = 0 时， 点 p 在矢量边上; f ＞ 0 时， 点 p 在矢量边的左侧( 从 pi 看向 pi+1 ) ; f ＜ 0 时， 点 p 在矢量边的右侧。  

多边形面积公式



计算方法



### [判断点是否在多边形内部](https://www.cnblogs.com/luxiaoxun/p/3722358.html)

如何判断一个点是否在多边形内部？

（1）面积和判别法：判断目标点与多边形的每条边组成的三角形面积和是否等于该多边形，相等则在多边形内部。

（2）夹角和判别法：判断目标点与所有边的夹角和是否为360度，为360度则在多边形内部。

（3）引射线法：从目标点出发引一条射线，看这条射线和多边形所有边的交点数目。如果有奇数个交点，则说明在内部，如果有偶数个交点，则说明在外部。

具体做法：将测试点的Y坐标与多边形的每一个点进行比较，会得到一个测试点所在的行与多边形边的交点的列表。在下图的这个例子中有8条边与测试点所在的行相交，而有6条边没有相交。如果测试点的两边点的个数都是奇数个则该测试点在多边形内，否则在多边形外。在这个例子中测试点的左边有5个交点，右边有三个交点，它们都是奇数，所以点在多边形内。



```cpp
private static double SignedPolygonArea(List<PointLatLng> points)
{            // Add the first point to the end.
            int pointsCount = points.Count;
            PointLatLng[] pts = new PointLatLng[pointsCount + 1];
            points.CopyTo(pts, 0);
            pts[pointsCount] = points[0];        \
        for (int i = 0; i < pointsCount + 1; ++i)
        {
            pts[i].Lat = pts[i].Lat * (System.Math.PI * 6378137 / 180);
            pts[i].Lng = pts[i].Lng * (System.Math.PI * 6378137 / 180);
        }
        // Get the areas.
        double area = 0;
        for (int i = 0; i < pointsCount; i++)
        {
            area += (pts[i + 1].Lat - pts[i].Lat) * (pts[i + 1].Lng + pts[i].Lng) / 2;
        }
        // Return the result.
        return area;
}

    /// <summary>
    /// Get the area of a polygon
    /// </summary>
    /// <param name="points"></param>
    /// <returns></returns>
public static double GetPolygonArea(List<PointLatLng> points)
{
    // Return the absolute value of the signed area.
    // The signed area is negative if the polygon is oriented clockwise.
    return Math.Abs(SignedPolygonArea(points));
}
```

### 判断一个点是否在多边形内部 - [射线法思路](https://blog.csdn.net/qq_27161673/article/details/52973866)

原理

直线穿越多边形边界时，有且只有两种情况：进入多边形或穿出多边形。
在不考虑非欧空间的情况下，直线不可能从内部再次进入多边形，或从外部再次穿出多边形，即连续两次穿越边界的情况必然成对。
直线可以无限延伸，而闭合曲线包围的区域是有限的，因此最后一次穿越多边形边界，一定是穿出多边形，到达外部。



### 判断点是否处于多边形内的[三种方法](http://blog.sina.com.cn/s/blog_b347c8960101dvvz.html)

\1. 叉乘判别法（只适用于凸多边形）

 想象一个凸多边形，将凸多边形中每一个边AB，与被测点P，求PA×PB。判断结果的符号是否发生变化，如果没有变化，P在多边形内；反之点处于凸多边形外。但对于凹多边形不再适用。 其每一个边都将整个2D屏幕划分成为左右两边，连接每一边的第一个端点和要测试的点得到一个矢量v，将两个2维矢量扩展成3维的，然后将该边与v叉乘，判断结果3维矢量中Z分量的符号是否发生变化，进而推导出点是否处于凸多边形内外。这里要注意的是，多边形顶点究竟是左手序还是右手序，这对具体判断方式有影响。

\2. 面积判别法（只适用于凸多边形）

 第四点分别与三角形的两个点组成的面积分别设为S1,S2,S3，只要S1+S2+S3>原来的三角形面积就不在三角形范围中.可以使用海伦公式 。推广一下是否可以得到面向凸多边形的算法？（不确定）

\3. 角度和判别法（适用于任意多边形）

```cpp
double angle = 0;
realPointList::iterator iter1 = points.begin();
for (realPointList::iterator iter2 = (iter1 + 1); iter2 < points.end(); ++iter1, ++iter2)
 {
   double x1 = (*iter1).x - p.x;   
   double y1 = (*iter1).y - p.y;   
   double x2 = (*iter2).x - p.x;
   double y2 = (*iter2).y - p.y;   
   angle += angle2D(x1, y1, x2, y2);
 }

if (fabs(angle - span::PI2) < 0.01) return true;
else return false;
```

另外，可以使用bounding box来加速。

if (p.x < (*iter)->boundingBox.left ||

  p.x > (*iter)->boundingBox.right ||

  p.y < (*iter)->boundingBox.bottom ||

  p.y > (*iter)->boundingBox.top) 。。。。。。

对于多边形来说，计算bounding box非常的简单。只需要把水平和垂直方向上的最大最小值找出来就可以了。

对于三角形：第四点分别与三角形的两个点的交线组成的角度分别设为j1,j2,j3，只要j1+j2+j3>360就不在三角形范围中。

\4. 水平/垂直交叉点数判别法（适用于任意多边形）

  注意到如果从P作水平向左的射线的话，如果P在多边形内部，那么这条射线与多边形的交点必为奇数，如果P在多边形外部，则交点个数必为偶数（0也在内）。所以，我们可以顺序考虑多边形的每条边，求出交点的总个数。还有一些特殊情况要考虑。假如考虑边(P1,P2)，

1)如果射线正好穿过P1或者P2,那么这个交点会被算作2次，处理办法是如果P的从坐标与P1,P2中较小的纵坐标相同，则直接忽略这种情况

2)如果射线水平，则射线要么与其无交点，要么有无数个，这种情况也直接忽略。

3)如果射线竖直，而P0的横坐标小于P1,P2的横坐标，则必然相交。

4)再判断相交之前，先判断P是否在边(P1,P2)的上面，如果在，则直接得出结论：P再多边形内部。