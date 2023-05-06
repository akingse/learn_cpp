## Delaunay得劳内三角形算法

使用几个第三方库，写一个得劳内三角形算法，造个小轮子；

include

- eigon
- cgal
- boost

```
C:\Users\Aking\source\repos\TPL\eigen-3.4.0
```



---

Delaunay性质

- 最接近于规则化的的三角网
- 三角形外接圆内部没有其他点
- 如果不存在四点共圆则唯一



> 德劳内三角（DelaunayTriangulation）空圆特性和最大化最小角特性
> 维诺图（Voronoi diagram）：对相邻的两个点之间作垂直频平分线，所构成的图叫又称泰森多边形。
> 它和德劳内三角网是对偶（Duality）的关系。 维诺图中的点是德劳内三角网三角形外接圆的圆心。

[blog](https://blog.csdn.net/qq_49838656/article/details/119641392)





eigon构造矩阵和行列式