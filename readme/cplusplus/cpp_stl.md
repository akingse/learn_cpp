# stl(Standard Template Library)

STL的代码从广义上讲分为三类：algorithm（算法）、container（容器）和iterator（迭代器）

STL被组织为下面的13个头文件：<algorithm>、<deque>、<functional>、<iterator>、<vector>、<list>、<map>、<memory>、<numeric>、<queue>、<set>、<stack>和<utility>。

标准模板库包含了序列容器（sequence containers）与关系容器（associative containers）。

序列容器包括vector,list,forward_list,deque和array等。

关联容器包括set,multiset,map,multimap,unordered_set,bitset和valarray等。



---





emplace原位构造

```cpp
std::map<int, std::string>test;
test.insert(make_pair(1, str));								// (1)
test.emplace(make_pair(1, str));							// (2)
test.emplace(2, str);										// (3)  也可写作test.emplace<int, std::string>(2, str);	
test.try_emplace(3, str);									// (4)


```

对于不发生插入的情况，(4)中try_emplace不会进行参数构造，即不会去调用std::pair的构造函数，而(3)中emplace无论插入成功或失败都会调用std::pair的构造函数进行参数构造，导致消耗更多时间并产生更多垃圾数据，拉低性能





### C++ STL [十六大容器](https://zhuanlan.zhihu.com/p/542115773)





[list约瑟夫环](https://blog.csdn.net/weixin_65155181/article/details/126675169)
