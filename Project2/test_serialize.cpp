#include "pch.h"
using namespace std;

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

//
//template< typename T >
//struct tree_node
//{
//	T t;
//	std::vector<tree_node> children;
//	void walk_depth_first() const;
//};
////简单的遍历将使用递归…
//template< typename T >
//void tree_node<T>::walk_depth_first() const
//{
//	cout << t;
//	for (auto& n : children) 
//		n.walk_depth_first();
//}
//
//template <typename T>
//struct TreeNode
//{
//	T* DATA; // data of type T to be stored at this TreeNode
//
//	vector< TreeNode<T>* > children;
//
//	// insertion logic for if an insert is asked of me.
//	// may append to children, or may pass off to one of the child nodes
//	void insert(T* newData);
//
//};
//
//template <typename T>
//struct Tree
//{
//	TreeNode<T>* root;
//
//	// TREE LEVEL functions
//	void clear() { delete root; root = 0; }
//
//	void insert(T* data) { if (root)root->insert(data); }
//};

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
	size_t sz0 = sizeof(Vec3d); //24
	size_t sz1 = infor.size(); //15
	size_t sz2 = sizeof(infor); //32
	size_t sz3 = sizeof(string); //40
	size_t sz6 = sizeof(remark); //40
	size_t sz4 = sizeof(vector<void*>); //32
	size_t sz5 = sizeof(map<bool,bool>); //24
	size_t sz7 = sizeof(map<size_t, size_t>); //24

	//类强转二进制
	Vec3d vec;
	const unsigned char* ptr = reinterpret_cast<const unsigned char*>(&vec);
	std::vector<unsigned char> res = std::vector<unsigned char>(ptr, ptr + sizeof(Vec3d));
	Vec3d vecDe;
	memcpy(&vecDe, res.data(), sizeof(Vec3d));

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

// 序列化-二叉树

static atomic<int> s_allnodes_index = 1;
struct TreeNode 
{
	int m_nodeIndex;
	std::vector<int> m_geomIndexes;
	TreeNode* m_father;
	TreeNode* m_left;
	TreeNode* m_right;

	TreeNode() : m_nodeIndex(s_allnodes_index++), m_father(nullptr), m_left(nullptr), m_right(nullptr) {}
	TreeNode(int x) : m_nodeIndex(x), m_father(nullptr), m_left(nullptr), m_right(nullptr) {}
    static TreeNode* create(TreeNode* left, TreeNode* right, TreeNode* father = nullptr)
	{
		TreeNode* ptr = new TreeNode;
		ptr->m_father = father;
		ptr->m_left = left;
		ptr->m_right = right;
		return ptr;
	}
};

// 序列化二叉树为字符串
std::string serialize(TreeNode* root) {
	if (root == nullptr) 
		return "$";
    //std::vector<unsigned char> temp(
    std::string serialized(
		sizeof(int) + 
        sizeof(int) * (1 + root->m_geomIndexes.size()), 0);
    //unsigned char* ptr = temp.data();
    char* ptr = const_cast<char*>(serialized.data());
	//vector
    int count = root->m_geomIndexes.size();
    memcpy(ptr, &count, sizeof(int)); ptr += sizeof(int);
    memcpy(ptr, root->m_geomIndexes.data(), sizeof(int) * root->m_geomIndexes.size()); ptr += sizeof(int) * root->m_geomIndexes.size();
	//member
	memcpy(ptr, &root->m_nodeIndex, sizeof(int)); ptr += sizeof(int);
	//std::string serialized(temp.begin(), temp.end());
	serialized += char(-128) + serialize(root->m_left);
	serialized += char(-128) + serialize(root->m_right);
	return serialized;
}

// 反序列化字符串为二叉树
TreeNode* deserialize_istring(std::istringstream& iss, TreeNode* father = nullptr) {
	std::string token;
	//iss >> token;//默认情况下，istringstream空格（空白字符）被视为分隔符，逐个遍历；
	std::getline(iss, token, char(-128));//
	if (token == "$") 
		return nullptr;
	int count, nodeIndex;
	const char* ptr = token.data();
	memcpy(&count, ptr, sizeof(int)); ptr += sizeof(int);
	std::vector<int> geomIndexes(count);
	memcpy(geomIndexes.data(), ptr, sizeof(int) * geomIndexes.size()); ptr += sizeof(int) * geomIndexes.size();
	memcpy(&nodeIndex, ptr, sizeof(int)); ptr += sizeof(int);

	//TreeNode* root = new TreeNode(std::stoi(token));
	TreeNode* root = new TreeNode;
	root->m_father = father;	father = root;
	root->m_geomIndexes = geomIndexes;
	root->m_nodeIndex = nodeIndex;
	root->m_left = deserialize_istring(iss, father);
	root->m_right = deserialize_istring(iss, father);
	return root;
}
// 反序列化字符串为二叉树（辅助函数）
TreeNode* deserialize(const std::string& data) {
	//从序列化的字符串中按照特定规则提取数据，例如按空格分隔节点的值。字符串流提供了方便的方法来完成这个任务。
	//在反序列化过程中，我们需要将字符串表示的节点值转换为整数或其他类型。字符串流可以将字符串解析为适当的数据类型，如std::stoi用于将字符串转换为整数。
	std::istringstream iss(data);
	return deserialize_istring(iss);
}

// 将int值序列化为std::string
std::string serializeInt(int value) {
	std::stringstream ss;
	ss << value;
	return ss.str();
}

// 将bool值序列化为std::string
std::string serializeBool(bool value) {
	return value ? "true" : "false";
}

// 测试序列化函数
void testSerialization0() 
{
	// 构建一个二叉树
	TreeNode* root = new TreeNode(1);
	root->m_left = new TreeNode(2);
	root->m_left->m_geomIndexes = { 2,22,222 };
	root->m_right = new TreeNode(-1);
	root->m_left->m_left = new TreeNode(4);
	root->m_left->m_left->m_geomIndexes = { 4,44 };
	root->m_left->m_right = new TreeNode(5);

	// 序列化
	std::string serialized = serialize(root);
	std::cout << "Serialized: " << serialized << std::endl;

	std::vector<unsigned char> charVector(serialized.size());
	memcpy(charVector.data(), serialized.data(), serialized.size());
	//迭代器构造
	//std::string strVector(charVector.begin(), charVector.end());
	//memcpy
	std::string strVector(charVector.size(), 0);
	memcpy(const_cast<char*>(strVector.data()), charVector.data(), charVector.size());

	// 反序列化
	TreeNode* deserialized = deserialize(serialized);
	TreeNode* deserialized1 = deserialize(strVector);
	std::cout << "Deserialized: " << deserialized->m_nodeIndex << std::endl;

	// 空列表data为空指针
	vector<int> vecInt(0);
    int* ptr = vecInt.data();
	//iss >> token默认分隔符
	string str;
	str.push_back('0');
	str.push_back('1');
	str.push_back('0');
	for (int i = 0; i > -129; i--)
		str.push_back(i);
	std::istringstream iss(str);
	std::string word;
	vector<string> strRec;
	while (iss >> word) 
	{
		strRec.push_back(word);
	}

	return;
}

//string与std::vector<unsigned char>转换
void testSerialization3()
{
	//char -127~128
	string str;
	str.push_back(-1);
	str.push_back(-2);
	str.insert(0, "hello");

	//string token = "$";
	//auto token = "$";
	const char token[2] = "$";
	const char token1 = '$';

	//std::vector<unsigned char> bin;
	//bin.push_back(-1);
	//bin.push_back(-2);
	//bin.push_back('h');
	//bin.push_back('e');

	//Byte
	std::vector<unsigned char> bin(str.size());
	memcpy(bin.data(), str.data(), str.size());
	string str_de(bin.size(), 0);
	memcpy(const_cast<char*>(str_de.data()), bin.data(), bin.size());

	return;
}

//int memcpy是否为负
void testSerialization4()
{
	int num = 12345;
	char buffer[20]; // 20 是缓冲区大小，根据实际情况调整
	itoa(num, buffer, 10);
	string nums = to_string(num);
    int numde = std::stoi(buffer);
    int numde1 = std::stoi(nums);

	int count = 0;
	for (int i = 1; i < 1e6; i++)
	{
		vector<byte> tochar(4);
		memcpy(tochar.data(), &i, sizeof(int));
        char ci = static_cast<char>(i);
		
		for (int j = 0; j < 4; j++)
		{
			if (tochar[j] < 0)
				count++; //int 128==char -128
			if (tochar[j] == 0)
				count++;
		}

	}
	return;
}

//生成随机二叉树
TreeNode* generateRandomTree(int maxDepth, int currentDepth = 1)
{
	// 如果当前层级超过最大层级，返回空指针
	if (currentDepth > maxDepth) 
		return nullptr;
	// 随机生成节点值
	//int nodeValue = rand() % 100; // 生成随机值（0-99）
	TreeNode* node = new TreeNode;//
	// 随机决定是否创建左子树和右子树
	if (rand() % 2 == 0) 
	{ // 50% 的概率创建左子树
        node->m_left = generateRandomTree(maxDepth, currentDepth + 1);
        node->m_right = generateRandomTree(maxDepth, currentDepth + 1);
	}
	return node;
}

void testSerialization5()
{
	srand(int(time(0))); // 设置随机种子
	// 随机选择层级在 2 到 10 之间
	int maxDepth = 4;// +rand() % 9; // 最大深度（从 2 到 10）
	std::cout << "Generating a random binary tree with up to " << maxDepth << " levels." << std::endl;

	// 生成随机二叉树
	TreeNode* root = generateRandomTree(maxDepth);

	return;
}

//------------------------------------------------------------------------------------------

//转Json，调用接口
//C:\Users\Aking\source\repos\TPL\boost-1.81.0\libs\json\include\boost
//C:\Users\Aking\source\repos\TPL\boost-1.81.0\libs\json\include\boost\json
//#include <json.hpp>
//using namespace boost;
//using namespace boost::json;


//------------------------------------------------------------------------------------------

//指针传值
static void test_pointer_(byte* const data)
{
	//data += 4;
	*data = 'c';
	cout << *data << endl;
}

static void test_pointer(byte* data)
{
	data += 4;
	*data = 'c';
	cout << *data << endl;
}

//函数接收到的是指针 data 的一个副本。在函数内部，对 data 的偏移操作只会修改函数内部的副本，不会影响原始指针的值。
static void test_pointer_c(const byte* data)
{
	data += 4;
	//*data = 'c';
	cout << *data << endl;
}

//函数接收到的是指针 data 的引用。在函数内部，对 data 的偏移操作会直接修改原始指针的值，因为函数内部操作的是原始指针的引用。
static void test_pointer_r(byte*& data)
{
	data += 4;
	cout << *data << endl;
}

static void test_pointer_pp(byte** data)
{
	*data += 4;
	cout << *data << endl;
}

//测试指针改值
void testSerialization1()
{
	string str = "12345";
	byte* data = new byte;
	memcpy(data, str.data(), str.size());

	//test_point_c(data);
	//cout << *data << endl;
	//test_point(data);
	//cout << *data << endl;
	//test_point_r(data);
	test_pointer_pp(&data);
	cout << *data << endl;
	return;
}

//memcpy拷贝覆盖
class IGemetry
{

};

class ICsgTree
{
public:
	//int id = 0; //默认无对齐，各占8byte
	virtual ~ICsgTree() = 0;
};

//min(n in #pragma pack(n),结构体中的最长类型)
#pragma pack(1)
class GeCsgTree :public ICsgTree
{
private:
	int tolerance;
	int oparation;
	vector<shared_ptr<ICsgTree>> m_leftNodes;
	vector<shared_ptr<ICsgTree>> m_rightNodes;
public:
	virtual ~GeCsgTree() override {};
};
#pragma pack() //end

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
struct CsgTreeData
{
	vector<shared_ptr<ICsgTree>> m_leftNodes;
	vector<shared_ptr<ICsgTree>> m_rightNodes;
	//bool isLeft;
};

static void _test1()
{
	int sz0 = sizeof(ICsgTree);
	int sz1 = sizeof(GeCsgTree);
	int sz2 = sizeof(CsgTreeData);
	int sz3 = sizeof(vector<shared_ptr<ICsgTree>>);
	CsgTreeData data;
	// 生成GUID
	boost::uuids::random_generator generator;
	boost::uuids::uuid uuid = generator();

	// 将GUID转换为字符串
	std::string uuidStr = boost::uuids::to_string(uuid);

	// 输出GUID
	std::cout << "Generated GUID: " << uuidStr << std::endl;//Generated GUID: 57186994-bef7-4134-832b-fc3aa6866063
	/*
	UUID（Universally Unique Identifier）的标准表示形式是一个包含32个十六进制（hexadecimal）字符的字符串，
	分为五个部分，用连字符分隔，形如："xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"。

	前8个字符表示UUID的32位数据的前32位。
	第9至12个字符表示UUID的32位数据的接下来的16位。
	第13至16个字符表示UUID的32位数据的接下来的16位。
	第17至20个字符表示UUID的32位数据的接下来的16位。
	最后12个字符表示UUID的32位数据的接下来的48位。
	*/

	return;
}

static int enrol = []()->int
	{
		testSerialization0();
		testSerialization1();
		//testSerialization2();
		testSerialization3();
		testSerialization4();
		testSerialization5();
		_test1();
		cout << "test_serialize finished.\n" << endl;
		return 0;
	}();
