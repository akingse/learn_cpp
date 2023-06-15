

---

## 几何csg树设计

### 简介

计算机中表示三维形体的模型，按照几何特点进行分类，大体上可以分为三种：线框模型、表面模型和实体模型。如果按照表示物体的方法进行分类，实体模型基本上可以分为分解表示、构造实体几何CSG（Constructive Solid Geometry）和边界表示B-REP（Boundary Representation）三大类。

Solid  最简单的实体被称为体元或几何图元，通常是形状简单的物体，如立方体、圆柱体、棱柱、棱锥、球体、圆锥等。

Constructive 构造物体就是将体元根据集合论的布尔逻辑组合在一起，这些运算包括：并集、交集以及补集。

1，简单实体，2，嵌套基本体

![img](https://upload.wikimedia.org/wikipedia/commons/thumb/8/8b/Csg_tree.png/220px-Csg_tree.png)

- 碰撞检测-高效
- 渲染光追-点在内部
- 自带封闭-相对brep
- -没有拓扑信息



---

### CSG设计

class CSG_Tree

成员变量

 ```
  - 基本体（single & nest）
  - 矩阵 transformation
  - 布尔信息
 ```

  

接口

```c
	参数读写
getPropertyValue / setPropertyValue
	修改矩阵
setTransform / getTransform
    布尔信息
isHollow / setHollow
    序列化
serialize / deserialize
```







---



### 现有python-csg树设计

[processon](https://www.processon.com/diagraming/61722165f346fb01b906489b)

###### ![image-20230613141232958](C:/Users/Aking/AppData/Roaming/Typora/typora-user-images/image-20230613141232958.png)







### 原有C++-csg树设计





![image-20230613144920385](C:/Users/Aking/AppData/Roaming/Typora/typora-user-images/image-20230613144920385.png)