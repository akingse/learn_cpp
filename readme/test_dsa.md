数据结构



  ![dsa](https://raw.githubusercontent.com/akingse/my-picbed/main/file.png)



DataStructure数据结构

![image-20230325214328442](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325214328442.png)

邓俊辉



> C++语言程序设计基础：类、继承、重载、重写、虚方法、模板离散数学基础: 集合、偏序集、良序、数学归纳法、级数、递归、递推概率基础: 随机分布、概率、伯努利实验、数学期望、期望值的线性律
>
> DSA（数据结构和算法 Data Structure and Algorithms ）
>
> 数据结构和算法是相辅相成的。数据结构是为算法服务的，算法要作用在特定的数据结构之上。 因此，我们无法孤立数据结构来讲算法，也无法孤立算法来讲数据结构。

10 个数据结构：数组、链表、栈、队列、散列表、二叉树、堆、跳表、图、Trie 树；

10 个算法：递归、排序、二分查找、搜索、哈希算法、贪心算法、分治算法、回溯算法、动态规划、字符串匹配算法。

 

 

### 第一章 绪论

算法的特征

程序!=算法，有穷性

算法：正确，健壮，可读，效率

Algorithms + Data Structures = Programs

(Algorithms + Data Structures) X Efficiency = Computation

时间复杂度big O

![image-20230325220859365](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325220859365.png)

迭代乃人工，递归方神通

减而治之 decrease and conquer

分而治之 devide and conquer

Longest Common Subsequence 最长公共子序列

动态规划:消除重复计算。提高效率

将复杂度为幂级数的递归，采用减而治之，改写为线性复杂度；

数据访问尽量连续，符合操作系统缓存机制；



| **排序法**  | **平均时间** | **最差情形** | **稳定度** | **额外空间** | **备注**           |
| ----------- | ------------ | ------------ | ---------- | ------------ | ------------------ |
| 冒泡        | O(n2)        | O(n2)        | 稳定       | O(1)         | n小时较好          |
| 选择        | O(n2)        | O(n2)        | 不稳定     | O(1)         | n小时较好          |
| 插入        | O(n2)        | O(n2)        | 稳定       | O(1)         | 大部分已排序时较好 |
| Shell(希尔) | O(nlogn)     | O(ns)        | 不稳定     | O(1)         | s是所选分组        |
| 快速        | O(nlogn)     | O(n2)        | 不稳定     | O(nlogn)     | n大时较好          |
| 归并        | O(nlogn)     | O(nlogn)     | 稳定       | O(1)         | n大时较好          |
| 堆          | O(nlogn)     | O(nlogn)     | 不稳定     | O(1)         | n大时较好          |
| 桶式        | O(k+n)       | O(k+n)       | 稳定       | O(1)         | 只能排整形数组     |



第二章 向量

抽象数据类型ADT

 

第三章 列表

第四章 栈与队列

第五章 二叉树

第六章 图

第七章 图应用

---

第八章 二叉搜索树

第十章 高级搜索树

第十一章 词典

第十二章 优先级队列

第十三章 串

第十四章 排序