using namespace std;
#include "pch.h"

std::vector<unsigned char> serialize(const BPGeometricPrimitiveSer& primitive)
{
	//序列化 infor
	std::vector<unsigned char> infor(sizeof(bool) + sizeof(size_t) + primitive.m_remark.size()); //vector<bool> a(10);//指定size
	//using memcpy
	unsigned char* ptr = infor.data();
	memcpy(ptr, &primitive.m_hollow, sizeof(bool));						ptr += sizeof(bool);
	memcpy(ptr, &primitive.m_identification, sizeof(size_t));			ptr += sizeof(size_t);
	memcpy(ptr, primitive.m_remark.data(), primitive.m_remark.size());  ptr += primitive.m_remark.size();
	return infor;
}
BPGeometricPrimitiveSer deserialize(const std::vector<unsigned char>& infor)
{
	//反序列化 infor
	bool hollow;
	size_t identification;
	const unsigned char* ptr = infor.data();
	memcpy(&hollow, ptr, sizeof(bool));					ptr += sizeof(bool);
	memcpy(&identification, ptr, sizeof(size_t));		ptr += sizeof(size_t);
	long long n = infor.size()-(ptr - infor.data()); //计算string字长
	string remark = std::string((const char*)ptr, n);
	// cons
	BPGeometricPrimitiveSer res;
	res.m_hollow = hollow;
	res.m_identification = identification;
	res.m_remark = remark;
	return res;
}

template< typename T >
struct tree_node
{
	T t;
	std::vector<tree_node> children;
	void walk_depth_first() const;
};
//简单的遍历将使用递归…
template< typename T >
void tree_node<T>::walk_depth_first() const
{
	cout << t;
	for (auto& n : children) 
		n.walk_depth_first();
}

template <typename T>
struct TreeNode
{
	T* DATA; // data of type T to be stored at this TreeNode

	vector< TreeNode<T>* > children;

	// insertion logic for if an insert is asked of me.
	// may append to children, or may pass off to one of the child nodes
	void insert(T* newData);

};

template <typename T>
struct Tree
{
	TreeNode<T>* root;

	// TREE LEVEL functions
	void clear() { delete root; root = 0; }

	void insert(T* data) { if (root)root->insert(data); }
};


int main_ser()
{

	bool hollow = false;
	size_t identification = 1234567890;
	string remark = "string";
	BPGeometricPrimitiveSer bp(hollow, identification, remark);
	std::vector<unsigned char> data= serialize(bp);
	BPGeometricPrimitiveSer de = deserialize(data);

	// vector 整型构造
	//std::vector<unsigned char> infor1(remark.size()); //byte 6
	//std::vector<unsigned char> infor2(sizeof(bool));//byte 1
	//std::vector<unsigned char> infor3(sizeof(size_t));//byte 8
	std::vector<unsigned char> infor(15);

	//注意容器的 sizeof() 和 .size()
	size_t sz0 = sizeof(Vec3); //24
	size_t sz1 = infor.size(); //15
	size_t sz2 = sizeof(infor); //32
	size_t sz3 = sizeof(string); //40
	size_t sz6 = sizeof(remark); //40
	size_t sz4 = sizeof(vector<void*>); //32
	size_t sz5 = sizeof(map<bool,bool>); //24
	size_t sz7 = sizeof(map<size_t, size_t>); //24

	//类强转二进制
	Vec3 vec;
	const unsigned char* ptr = reinterpret_cast<const unsigned char*>(&vec);
	std::vector<unsigned char> res = std::vector<unsigned char>(ptr, ptr + sizeof(Vec3));
	Vec3 vecDe;
	memcpy(&vecDe, res.data(), sizeof(Vec3));

	//序列化容器
	vector<double> ori = { 1,2,4,1,2,4, 1,2,4,1,2,4, 1,2,4,1,2,4};
	size_t sz8 = sizeof(ori); //32
	const unsigned char* ser = reinterpret_cast<const unsigned char*>(&ori);
	std::vector<unsigned char> uchar = std::vector<unsigned char>(ser, ser + sizeof(ori));
	vector<double> tar(sizeof(ori));
	memcpy(&tar, uchar.data(), sizeof(ori));

	//嵌套容器
	vector<vector<double>> ori2 = { { 1,2,4}, { 1,2,4}, { 1,2,4} };
	size_t sz9 = sizeof(ori2); //32
	const unsigned char* ser2 = reinterpret_cast<const unsigned char*>(&ori2);
	std::vector<unsigned char> uchar2 = std::vector<unsigned char>(ser2, ser2 + sizeof(ori2));
	vector<vector<double>> tar2(sizeof(ori2));
	memcpy(&tar2, uchar2.data(), sizeof(ori2));


	size_t s1 = 111111;
	size_t s2 = 222222;
	size_t s3 = 333333;
	size_t s4 = 444444;
	std::vector<size_t> szList = { 1111,2222,3333,4444 };
	std::vector<unsigned char> seSZ;
	for (auto& iter : szList)
	{
		seSZ.resize(seSZ.size() + sizeof(size_t));
		memcpy(seSZ.data() + seSZ.size() - sizeof(size_t), &iter, sizeof(size_t));
	}

	std::vector<size_t> szListDe(seSZ.size() / sizeof(size_t));
	unsigned char* ptrs = seSZ.data();
	memcpy(&s1, ptrs, sizeof(size_t));    ptrs += sizeof(size_t);
	memcpy(&s2, ptrs, sizeof(size_t));    ptrs += sizeof(size_t);
	memcpy(&s3, ptrs, sizeof(size_t));    ptrs += sizeof(size_t);
	memcpy(&s4, ptrs, sizeof(size_t));    ptrs += sizeof(size_t);
	return 0;
}

	//fail
	//std::vector<char> chars = { 'h', 'e', 'l', 'l', 'o'};
	//std::vector<unsigned char> inform(chars.size());
	////using memcpy
	//unsigned char* ptrs = inform.data();
	//memcpy(ptrs, chars.data(), chars.size());  ptrs += chars.size();
	//de
	//const unsigned char* ptr2 = inform.data();
	//long long n = inform.size();
	//std::vector<char> remark2 = std::vector<char>(n,(char)ptr2);

	/*
	C++树结构
	C++ STL不提供任何“树”容器
	需要考虑的一些问题:
-节点的子节点数是固定的还是可变的?
-每个节点的开销是多少?-你需要父指针、兄弟指针等吗?
-提供什么算法?-不同的迭代器、搜索算法等。
	
	*/

	
    /*
    非嵌套类序列化

    //序列化 模板类强转 reinterpret_cast
    virtual std::vector<unsigned char> _serialize() const override
    {
        const unsigned char* ptr = reinterpret_cast<const unsigned char*>(&m_data);
        return std::vector<unsigned char>(ptr, ptr + sizeof(T));
    }
    //反序列化
    static __Primitive<T>* _deserialize(const std::vector<unsigned char> buf)
    {
        T data;
        memcpy(&data, buf.data(), sizeof(T));
        return new __Primitive<T>(data);
    }

    固定格式数据 using memcpy
    unsigned char* ptr = infor.data();
    memcpy(ptr, &identification, sizeof(size_t));       ptr += sizeof(size_t);
    memcpy(ptr, &hollow, sizeof(bool));                 ptr += sizeof(bool);

    ProjectManagerImplementation工程文件序列化
    std::tuple<BPParaMD5, std::vector<unsigned char>, std::vector<unsigned char>> temp = __PrimitiveInterface::serialize(obj);
    std::get<2>(temp).resize(std::get<2>(temp).size() + sizeof(BPParaMD5));
    memcpy(std::get<2>(temp).data() + std::get<2>(temp).size() - sizeof(BPParaMD5), &std::get<0>(temp), sizeof(BPParaMD5));
    return std::get<2>(temp);


    */
