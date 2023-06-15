#include "pch.h"
using namespace std;

//https://blog.csdn.net/weixin_51618380/article/details/115297385
class Tree
{
public:
	char m = '\0';
	std::vector<Tree> p;
	Tree() = default;
	void set(const std::vector<Tree>& _p)
	{
		p = _p;
	}
	void append(const Tree& _p)
	{
		p.push_back(_p);
	}
	void push_front(const Tree& _p)
	{
		p.insert(p.begin(), _p);
	}
	Tree(char _m, const std::vector<Tree>& _p = {}) :m(_m), p(_p) {};
	~Tree() {};
};

//template <typename T> //change lhs
//std::vector<T>& operator+(std::vector<T>& vct1, const std::vector<T>& vct2)
//{
//	vct1.insert(vct1.end(), vct2.begin(), vct2.end());
//	return vct1;
//}

//模板特化
//模板特化（template specialization）不同于模板的实例化，模板参数在某种特定类型下的具体实现称为模板特化。模板特化有时也称之为模板的具体化，分别有函数模板特化和类模板特化。
//模板偏特化（Template Partitial Specialization）是模板特化的一种特殊情况，指显示指定部分模板参数而非全部模板参数，或者指定模板参数的部分特性分而非全部特性，也称为模板部分特化。
//与模板偏特化相对的是模板全特化，指对所有模板参数进行特化。模板全特化与模板偏特化共同组成模板特化。
//模板偏特化主要分为两种，一种是指对部分模板参数进行全特化，另一种是对模板参数特性进行特化，包括将模板参数特化为指针、引用或是另外一个模板类。


template <typename T> //keep lhs
std::vector<T> operator+(const std::vector<T>& vct1, const std::vector<T>& vct2)
{
	std::vector<T> vct = vct1;
	vct.insert(vct.end(), vct2.begin(), vct2.end());
	return vct;
}

template <typename T>
void operator+=(std::vector<T>& vct1, const std::vector<T>& vct2)
{
	vct1.insert(vct1.end(), vct2.begin(), vct2.end());
}


vector<tuple<char,int>> recursion0(Tree tree, int layer=0)
{
	vector<tuple<char, int>> res;
	res.push_back({ tree.m,layer });
	for (auto& iter : tree.p)
	{
		res.push_back({ iter.m,layer+1 });
		vector<tuple<char, int>>  temp;
		for (auto& iterII : iter.p)
			temp =temp + recursion0(iterII, layer + 1);
		res = res + temp;
	}
	return res;
}

vector<tuple<char, int>> getFlatten(Tree tree, size_t& max, int layer = 0)
{
	vector<tuple<char, int>> res;
	res.push_back({ tree.m,layer });
	for (auto& iter : tree.p)
	{
		res += getFlatten(iter,max, layer + 1);
		if (layer+1 > max)
			max = layer+1;
	}
	return res;
}

vector<tuple<Tree, int>> recursion2(const Tree& tree, size_t& max, int layer = 0)
{
	vector<tuple<Tree, int>> res;
	res.push_back({ tree,layer });
	for (auto& iter : tree.p)
	{
		res = res + recursion2(iter, max, layer + 1);
		if (layer + 1 > max) // set max deep
			max = layer + 1;
	}
	return res;
}

//vector<tuple<char, int>> getFlattenPN(const vector<tuple<char, int>>& tree, size_t max)
vector<tuple<char, int>> getFlattenPN(Tree _tree)
{
	size_t max = 0;
	vector<tuple<char, int>> tree = getFlatten(_tree, max);
	vector<tuple<char, int>> res;
	if (tree.size() != 0)
		res.push_back({ std::get<0>(*tree.begin()),-1 });
	for (size_t i=1;i<max+1;++i)
	{
		//for (const auto& iterI : tree)
		for (auto iterI=tree.begin();iterI!=tree.end();++iterI)
		{
			if (std::get<1>(*iterI) == i)
			{
				auto iterII = iterI; //copy iterator
				while (std::get<1>(*iterII) != i-1 )
					iterII--; //find prev key
				for (size_t j = 0; j < res.size(); ++j) //find the index in array
				{
					if (std::get<0>(res[j]) == std::get<0>(*iterII))
						res.push_back({ std::get<0>(*iterI), j });
				}
				//auto a = iterII - tree.begin(); //the origin offset
			}
		}
		//auto& iter = tree.begin();
		//while (iter != tree.end())
	}
	return res;
}


	//反向生成树
Tree getAssemble0(const vector<tuple<char, int>>& tree)
{
	Tree res;
	if (tree.size() == 0)
		return res; //unvalid
	std::vector<Tree> tList;
	for (auto& iter : tree)
		tList.push_back(Tree(std::get<0>(iter)));
	size_t max = 0;
	int cmp = std::get<1>(*tree.begin());
	//get max nest layer
	for (auto& iter : tree)
	{
		if (std::get<1>(iter) != cmp)
		{
			max++;
			cmp = std::get<1>(iter);
		}
	}
	cmp = std::get<1>(*tree.rbegin());
	for (size_t i = 0; i < max; ++i)
	{
		std::vector<Tree> temp;
		//for (auto iterR = tree.rbegin(); iterR != tree.rend(); ++iterR) // using reverse_iterator
		for (int j=tree.size()-1;j>=0;--j) //using index
		{
			//if (cmp == std::get<1>(*iterR))
			if (cmp == std::get<1>(tree[j]))
				temp.push_back(std::move(tList[j])); // avoid copy
			else
			{
				if (cmp == -1)
					return tList[0];
				reverse(temp.begin(), temp.end()); //reverse
				tList[cmp].set(std::move(temp));
				temp.clear();
				temp.push_back(std::move(tList[j]));
				cmp = std::get<1>(tree[j]);
			}
		}
	}
	return res; //empty
}

//轮子优化，使用先序遍历列表
Tree getAssemble1(const vector<tuple<char, int>>& tree)
{
	Tree res;
	if (tree.size() == 0)
		return res; //unvalid
	std::vector<tuple<Tree,int>> tList; //create all nest tree
	for (auto& iter : tree)
	//for (auto& iter=tree.rbegin();iter!=tree.rend();++iter)//reverse , not work
		tList.push_back({ Tree(std::get<0>(iter)), std::get<1>(iter) });
	//auto iter = std::get<1>(*tree.rbegin());
	//for (auto iter = tList.rbegin(); iter != tList.rend(); iter++)
	//{
	//	auto iterB = iter;
	//	if (get<1>(*iterB) == 0)
	//		break;
	//	while (get<1>(*iterB) + 1 == get<1>(*iter))
	//		iterB--;
	//	get<0>(*iterB).append(get<0>(*iter));
	//}
	
	//must using index
	for (int i = tList.size()-1; i >=0; --i) //inverse order
	{
		int j = i;
		if (get<1>(tList[j]) == 0)
			break;
		while (get<1>(tList[j]) + 1 != get<1>(tList[i]))
			j--;
		get<0>(tList[j]).push_front(get<0>(tList[i]));

	}
	return get<0>(tList.front());
}



int main_tree()
{
	//双亲表示法
	Tree K('K');
	Tree L('L');
	Tree M('M');
	Tree N('N');
	Tree E('E');
	Tree G('G');
	Tree H('H');
	Tree J('J');

	Tree F('F', { K,L }); // using move cons
	Tree I('I', { M,N });

	Tree B('B', { E,F });
	Tree C('C', { G });
	Tree D('D', { H,I,J });

	Tree A('A', { B,C,D });

	vector<int> v1 = { 1,1 };
	v1.insert(v1.begin(), 0);
	vector<int> v2 = { 2,2 };
	vector<int> v3 = v1 + v2;
	v1 += v2;


	size_t max = 0;
	vector<tuple<char, int>>  res0 = getFlatten(A,max);
	//vector<tuple<Tree, int>>  res1 = recursion2(A,max);
	vector<tuple<char, int>> res2 = getFlattenPN(A);
	Tree res3 = getAssemble0(res2);
	Tree res4 = getAssemble1(res0);

	return 0;
}


//双亲表示法、孩子表示法、孩子兄弟表示法。
// 
//双亲表示法
//树的双亲表示法结点结构定义
//#define MAX_TREE_SIZE 100
//typedef int TElemType;   //树结点的数据类型，目前暂定整形
//typedef struct PTNode {    //结点结构
//	TElemType   data;  //结点数据域
//	int parent;              //双亲位置
//}PTNode;
//typedef struct {              //树结构
//	PTNode nodes[MAX_TREE_SIZE];    //结点数组
//	int  r, n;                     //根节点的位置和结点数
//}



//孩子表示法
//树的孩子表示法结点结构定义
#define MAX_TREE_SIZE 100
typedef int TElemType;   //树结点的数据类型，目前暂定整形
typedef struct CTNode {      //孩子结点
	int child;
	struct CTNode* next;
}*ChildPtr;
typedef struct {              //表头结构
	TElemType data;
	ChildPtr firstchild;
}CTBox;
typedef struct {              //树结构
	CTBox nodes[MAX_TREE_SIZE];    //结点数组
	int  r, n;                     //根节点的位置和结点数
};



//孩子兄弟表示法。


//-----------------------component--------------------------
class GeoCombine;
/*

// include max depth
vector<tuple<BPGeometricPrimitive, int>> getFlatten(const BPGeometricPrimitive& tree, size_t& max, int layer = 0)
{
	vector<tuple<BPGeometricPrimitive, int>> res; //child primitive and layer number
	res.push_back({ tree,layer });
	if (!tree.is<GeoCombine>())
		return res;
	for (auto& iter : tree.as<GeoCombine>().getParts())
	{
		res += getFlatten(iter, max, layer + 1);
		if (layer + 1 > max)
			max = layer + 1;
	}
	return res;
}

vector<tuple<BPGeometricPrimitive, int>> getFlattenPN(const BPGeometricPrimitive& tree)
{
	size_t max = 0;
	vector<tuple<BPGeometricPrimitive, int>> flat = getFlatten(tree, max);
	vector<tuple<BPGeometricPrimitive, int>> res;
	if (flat.size() == 0)
		return res;
	res.push_back({ std::get<0>(*flat.begin()),-1 });
	for (size_t i = 1; i < max + 1; ++i)
	{
		for (auto iterI = flat.begin(); iterI != flat.end(); ++iterI)
		{
			if (std::get<1>(*iterI) == i)
			{
				auto iterII = iterI; //copy iterator
				while (std::get<1>(*iterII) != i - 1)
					iterII--; //find prev key
				for (size_t j = 0; j < res.size(); ++j) //find the index in array
				{
					if (std::get<0>(res[j]) == std::get<0>(*iterII))
						res.push_back({ std::get<0>(*iterI), j });
				}
			}
		}
	}
	return res;
}


BPGeometricPrimitive getAssemblePN(vector<tuple<BPGeometricPrimitive, int>>& tree)
{
	if (tree.size() == 0)
		return BPGeometricPrimitive(); //unvalid
	size_t max = 0;
	int cmp = std::get<1>(*tree.begin());
	//get max nest layer
	for (auto& iter : tree)
	{
		if (std::get<1>(iter) != cmp)
		{
			max++;
			cmp = std::get<1>(iter);
		}
	}
	cmp = std::get<1>(*tree.rbegin());
	std::vector<BPGeometricPrimitive> res;
	for (int i = tree.size() - 1; i >= 0; --i) //using index
	{
		if (cmp == std::get<1>(tree[i]))
			res.push_back(std::move(get<0>(tree[i]))); // avoid copy
		else
		{
			reverse(res.begin(), res.end()); //keep order
			if (!get<0>(tree[cmp]).is<GeoCombine>())
				continue;
			get<0>(tree[cmp]).as<GeoCombine>().setParts(std::move(res));
			res.clear();
			res.push_back(std::move(get<0>(tree[cmp]))); //move cause clear *imp
			cmp = std::get<1>(tree[i]);
		}
	}
	return res[0];
}


// serialize
std::vector<unsigned char> getNestedGeometrySerialize(const std::vector<BPGeometricPrimitive>& parts)
{
	std::vector<unsigned char> res;
	for (const auto& iter : parts)
	{
		// serialize
		vector<tuple<BPGeometricPrimitive, int>> tree = getFlatten(iter);
		if (iter.is<GeoCombine>())
		{
			//         // call recursion
			GeoCombine primitive = iter.as<GeoCombine>();
			std::tuple<BPParaMD5, std::vector<unsigned char>, std::vector<unsigned char>> combine = __PrimitiveInterface::serialize(primitive);
			//         s_treeStore[std::get<0>(combine)] = { std::get<1>(combine) ,std::get<2>(combine) };
			//         std::vector<unsigned char> nest = getNestedGeometrySerialize(primitive.getParts());
			//         res.insert(res.end(), nest.begin(), nest.end());
		}
		else
		{
			// BPParaMD5    __PrimitiveInterface*    BPGeometricPrimitive
			std::tuple<BPParaMD5, std::vector<unsigned char>, std::vector<unsigned char>> single = __PrimitiveInterface::serialize(iter);
			BPParaMD5 repre = iter.getMD5();
			s_mapStore.try_emplace(repre, std::get<2>(single));
			s_mapStore.try_emplace(std::get<0>(single), std::get<1>(single));


			//BPGeometricPrimitive* pIter = (BPGeometricPrimitive*)&iter + 2*sizeof(void*); // muliti inherit with many pointor
			//__PrimitiveInterface* imp = *(__PrimitiveInterface**)(pIter);  pIter += sizeof(__PrimitiveInterface*);
			//BPParaTransform* m_transform = (BPParaTransform*)(pIter);  pIter += sizeof(BPParaTransform);
			//std::string* m_remark = (std::string*)(pIter);  pIter += sizeof(std::string);
			//std::vector<unsigned char>* m_attachData = (std::vector<unsigned char>*)(pIter);  pIter += sizeof(std::vector<unsigned char>);
			//bool* m_hollow = (bool*)(pIter);
   //         size_t pointor;
   //         memcpy(&pointor, &imp, sizeof(size_t)); // save iter owner __PrimitiveInterface*
   //         s_treeStore[{pointor,iter.getTransform(),iter.isHollow()}] = {std::get<1>(single) ,std::get<2>(single)};
   //         size_t szKey = sizeof(std::tuple<size_t, BPParaTransform, bool>);
			//res.resize(res.size() + szKey); // back append
			//memcpy(res.data() + res.size() - szKey, &std::tuple<size_t, BPParaTransform>(pointor, iter.getTransform()), szKey);
		}
	}
	//test 
	std::vector<BPGeometricPrimitive> partsDe = getNestedGeometryDeserialize(res);
	return res;
}

//deserialize
std::vector<BPGeometricPrimitive> getNestedGeometryDeserialize(const std::vector<unsigned char>& buf)
{
	BPGeometricPrimitive geo = __PrimitiveInterface::deserialize(buf, buf);
	std::vector<BPGeometricPrimitive> res;

	//  size_t szKey = sizeof(std::tuple<size_t, BPParaTransform, bool>);
	//  if (buf.size() % szKey != 0)
	//      return {};
	//  const unsigned char* ptr = buf.data();
	//  for (int i = 0; i < buf.size() / szKey; i++)
	//  {
	//      std::tuple<size_t, BPParaTransform, bool> temp;
	//      memcpy(&temp, ptr, szKey);
	//      ptr += szKey;
	//      if (s_treeStore.find(temp) == s_treeStore.end())
	//          continue;
		  //std::tuple<std::vector<unsigned char>, std::vector<unsigned char>> infor = s_treeStore.at(temp);
	//      BPGeometricPrimitive primitive = __PrimitiveInterface::deserialize(std::get<0>(infor), std::get<1>(infor));
	//      res.push_back(primitive);
	//  }
	return res;
}
*/

static int _enrol = []()->int 
{

	return 0;
}();