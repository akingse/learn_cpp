#include "pch.h"
#include "test_serialize.h"
using namespace std;
using namespace bin;
using namespace para;
//using namespace ppc;

std::vector<unsigned char> _bp_serialize(const BPGeometricPrimitiveSer& primitive)
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

BPGeometricPrimitiveSer _bp_deserialize(const std::vector<unsigned char>& infor)
{
	//反序列化 infor
	bool hollow;
	size_t identification;
	const unsigned char* ptr = infor.data();
	memcpy(&hollow, ptr, sizeof(bool));					ptr += sizeof(bool);
	memcpy(&identification, ptr, sizeof(size_t));		ptr += sizeof(size_t);
    long long n = infor.size() - (ptr - infor.data()); //计算string字长
	string remark = std::string((const char*)ptr, n);
	// cons
	BPGeometricPrimitiveSer res;
	res.m_hollow = hollow;
	res.m_identification = identification;
	res.m_remark = remark;
	return res;
}

int main_ser()
{

	bool hollow = false;
	size_t identification = 1234567890;
	string remark = "string";
	BPGeometricPrimitiveSer bp(hollow, identification, remark);
	std::vector<unsigned char> data= _bp_serialize(bp);
	BPGeometricPrimitiveSer de = _bp_deserialize(data);

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

//----------------------------------------------------------------------------------------------
// 序列化-二叉树
//----------------------------------------------------------------------------------------------

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

//重写序列化，修改序列化逻辑，返回智能指针
namespace ppc
{
	std::vector<byte> serializition(const shared_ptr<TreeNodePtr>& root)
	{
		if (root == nullptr)
			return {};
		std::vector<byte> serialized(
			sizeof(int) +
			sizeof(int) * (1 + root->m_geomIndexes.size()));
		unsigned char* ptr = serialized.data();
		//vector
		int count = (int)root->m_geomIndexes.size();
		memcpy(ptr, &count, sizeof(int)); ptr += sizeof(int);
		memcpy(ptr, root->m_geomIndexes.data(), sizeof(int) * root->m_geomIndexes.size()); ptr += sizeof(int) * root->m_geomIndexes.size();
		//member
		memcpy(ptr, &root->m_nodeIndex, sizeof(int)); ptr += sizeof(int);
		//leftNode
		std::vector<byte> bufferLeft = ppc::serializition(root->m_left);
		count = (int)bufferLeft.size();
		std::vector<byte> sizeLeft(sizeof(int));
		memcpy(sizeLeft.data(), &count, sizeof(int));
		serialized.insert(serialized.end(), sizeLeft.begin(), sizeLeft.end());
		serialized.insert(serialized.end(), bufferLeft.begin(), bufferLeft.end());
		//rightNode
		std::vector<byte> bufferRight = ppc::serializition(root->m_right);
		count = (int)bufferRight.size();
		std::vector<byte> sizeRight(sizeof(int));
		memcpy(sizeRight.data(), &count, sizeof(int));
		serialized.insert(serialized.end(), sizeRight.begin(), sizeRight.end());
		serialized.insert(serialized.end(), bufferRight.begin(), bufferRight.end());
		return serialized;
	}

	shared_ptr<TreeNodePtr> deserializition(const std::vector<byte>& data)
	{
		if (data.empty())
			return nullptr;
		int count, nodeIndex;
		const unsigned char* ptr = data.data();
		memcpy(&count, ptr, sizeof(int)); ptr += sizeof(int);
		std::vector<int> geomIndexes(count);
		memcpy(geomIndexes.data(), ptr, sizeof(int) * geomIndexes.size()); ptr += sizeof(int) * geomIndexes.size();
		memcpy(&nodeIndex, ptr, sizeof(int)); ptr += sizeof(int);
		//pointer
		shared_ptr<TreeNodePtr> root = make_shared<TreeNodePtr>(-1);
		//root->m_father = father;	father = root;
		root->m_geomIndexes = geomIndexes;
		root->m_nodeIndex = nodeIndex;
		//if (ptr - data.data() == data.size())
		//	return shared_ptr<TreeNode>(root);
		//leftNode
		memcpy(&count, ptr, sizeof(int)); ptr += sizeof(int);
		std::vector<byte> bufferLeft(count);
		memcpy(bufferLeft.data(), ptr, count); ptr += count;
		root->m_left = deserializition(bufferLeft);
		//rightNode
		memcpy(&count, ptr, sizeof(int)); ptr += sizeof(int);
		std::vector<byte> bufferRight(count);
		memcpy(bufferRight.data(), ptr, count); ptr += count;
		root->m_right = deserializition(bufferRight);
		return root;
	}
}

// 测试序列化函数
void testSerialization2() 
{
	int sz0 = sizeof(ICsgTree);
	int sz1 = sizeof(GeCsgTree);
	int sz2 = sizeof(CsgTreeData);
	int sz3 = sizeof(vector<shared_ptr<ICsgTree>>);
	CsgTreeData data;
	// 构建一个二叉树
	TreeNode* root = new TreeNode(1);
	root->m_left = new TreeNode(2);
	root->m_left->m_geomIndexes = { 2,22,222 };
	root->m_right = new TreeNode(-1);
	root->m_left->m_left = new TreeNode(4);
	root->m_left->m_left->m_geomIndexes = { 4,44 };
	root->m_left->m_right = new TreeNode(5);

	//// 序列化
	//std::string serialized = serialize(root);
	//std::cout << "Serialized: " << serialized << std::endl;

	//std::vector<unsigned char> charVector(serialized.size());
	//memcpy(charVector.data(), serialized.data(), serialized.size());
	////迭代器构造
	////std::string strVector(charVector.begin(), charVector.end());
	////memcpy
	//std::string strVector(charVector.size(), 0);
	//memcpy(const_cast<char*>(strVector.data()), charVector.data(), charVector.size());

	//// 反序列化
	//TreeNode* deserialized = deserialize(serialized);
	//TreeNode* deserialized1 = deserialize(strVector);
	//std::cout << "Deserialized: " << deserialized->m_nodeIndex << std::endl;

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
	_itoa(num, buffer, 10);
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

static void testSerialization6()
{
	// 定义一个 std::function，指向类型与 normalFunction 匹配的函数指针
	std::function<void()> func = testSerialization5;
	//std::cout << "typeid(func).name(): " << typeid(func).name() << std::endl;
	//std::cout << "typeid(&normalFunction).name(): " << typeid(&testSerialization5).name() << std::endl;
	// 比较 std::function 和 普通函数指针的 typeid
	if (typeid(func) == typeid(&testSerialization5))
		std::cout << "The types are the same." << std::endl;

	DependencyRegistry& reg = DependencyRegistry::getInstance();
	Implementation* ptr = reg.get<Implementation>("csg_node_class"); //父子类敏感
	std::shared_ptr<TreeNode> node;
	if (ptr == nullptr)
		node = Implementation().create();
	else
	{
		node = ptr->create();
		std::vector<unsigned char> data = ptr->serial(node);
		std::shared_ptr<TreeNode> nodeDe = ptr->deserial(data);
	}

	//function<std::string(TreeNode*)> serialize_ptr = serialize;
	//std::string buffer0 = serialize_ptr(node.get());
	//std::string(*serialize_ptr)(TreeNode*);

	//用法一：std::function
	function<std::string(TreeNode*)>* serialize_fun1 = reg.get<function<std::string(TreeNode*)>>("serialize_fun");
	std::string buffer1 = (*serialize_fun1)(node.get());

	//用法二：函数指针
	std::string buffer = Implementation::serialize(node.get());
	auto serialize_fun = reg.get<std::string(TreeNode*)>("serialize");
	std::string data2 = (*serialize_fun)(node.get());
	auto deserialize_fun = reg.get<TreeNode*(const std::string&)>("deserialize");
	TreeNode* nodeDe2 = (*deserialize_fun)(buffer);

	//将调用过程在项目1实现，导出封装后的函数
    std::shared_ptr<TreeNode> node3 = Implementation::createNew();
	//std::string data3 = ppc::serial(node3);
	//std::shared_ptr<TreeNode> nodeDe3 = ppc::deserial(data3);

	return;
}

//测试依赖反转，vector<byte>版本函数
static void testSerialization7()
{
	std::shared_ptr<TreeNodePtr> node1 = Implementation::createNewPtr();
	std::vector<byte> buffer1 = serializition(node1);
	std::shared_ptr<TreeNodePtr> nodeDe1 = deserializition(buffer1);
	//depend
	DependencyRegistry& reg = DependencyRegistry::getInstance();
	auto serialize_fun = reg.get<std::vector<byte>(const shared_ptr<TreeNodePtr>&)>("serializition");
	std::vector<byte> buffer2;
	if (serialize_fun!=nullptr)
		buffer2 = (*serialize_fun)(node1);
	auto deserialize_fun = reg.get<shared_ptr<TreeNodePtr>(const std::vector<byte>&)>("deserializition");
	shared_ptr<TreeNodePtr> nodeDe2;
	if (deserialize_fun != nullptr)
		nodeDe2 = (*deserialize_fun)(buffer2);
	//depinv
	std::vector<byte> buffer3= ppc::serializition(node1);
	std::shared_ptr<TreeNodePtr> nodeDe3 = ppc::deserializition(buffer3);

	return;
}

void exampleFunction(int m_value)
{
	cout << "exampleFunction" << endl;
}

//DependencyInversion
static void testSerialization8()
{
	DependencyInversion& registry = DependencyInversion::getInstance();
	//std::shared_ptr<TreeNodePtr> node1 = Implementation::createNewPtr();
	//auto serialize_fun = reg.get<std::vector<byte>(const shared_ptr<TreeNodePtr>&)>("serializition");
	//std::vector<byte> buffer2;
	//if (serialize_fun != nullptr)
	//	buffer2 = (*serialize_fun)(node1);
	//auto deserialize_fun = reg.get<shared_ptr<TreeNodePtr>(const std::vector<byte>&)>("deserializition");
	//shared_ptr<TreeNodePtr> nodeDe2;
	//if (deserialize_fun != nullptr)
	//	nodeDe2 = (*deserialize_fun)(buffer2);

	using FuncType = void(*)(int);
	registry.set<FuncType>("exampleFunc", exampleFunction);
	FuncType* retrievedFunc = registry.get<FuncType>("exampleFunc");
	(*retrievedFunc)(0);
	return;
}

#if 0
namespace bin
{
	static int s_allnode_index = 0;
	TreeNode::TreeNode(const BinaryTreePtr& tree) : m_value(s_allnode_index++), m_left(nullptr), m_right(nullptr)
	{
		m_owner = tree;
		//if (tree->m_root == nullptr)
            tree->m_root = shared_ptr<TreeNode>(this);
		//tree->m_root.reset();
	}

	TreeNode::TreeNode(const BinaryTreePtr& tree, const TreeNodePtr& left, const TreeNodePtr& right, int op):
		m_value(s_allnode_index++), m_left(left), m_right(right)
	{
		TreeNodePtr ptr = shared_ptr<TreeNode>(this);
		m_owner = tree;
		if (tree->m_root == nullptr) //avoid pointer destruct crash
			tree->m_root = ptr;
		if (left != nullptr)
			left->m_father = ptr;
		if (right != nullptr)
			right->m_father = ptr;
	}

	//不可使用递归构造
	//TreeNode::TreeNode(const BinaryTreePtr& tree, const TreeNodePtr& rhs, const TreeNodePtr& father /*= nullptr*/)
	//{
	//	if (tree == nullptr || rhs == nullptr)
	//		return;
	//	TreeNodePtr ptr = shared_ptr<TreeNode>(this);
 //       if (father == nullptr && tree->m_root == nullptr)
	//		tree->m_root = ptr;
	//	m_owner = tree;
	//	m_father = father;
	//	m_value = rhs->m_value;
	//	if (rhs->m_left != nullptr)
	//	{
	//		TreeNode* temp = new TreeNode(tree, rhs->m_left, ptr);
	//		m_left = shared_ptr<TreeNode>(temp);
	//	}
	//	if (rhs->m_right != nullptr)
	//	{
	//		TreeNode* temp = new TreeNode(tree, rhs->m_right, ptr);
	//		m_right = shared_ptr<TreeNode>(temp);
	//	}
	//}
	

	TreeNodePtr TreeNode::deepcopy(const BinaryTreePtr& tree, const TreeNodePtr& rhs, const TreeNodePtr& father /*= nullptr*/)
	{
		if (tree == nullptr || rhs == nullptr)
			return nullptr;
		TreeNodePtr ptr = make_shared<TreeNode>();//construct empty
		if (father == nullptr)
			tree->m_root = ptr;
		ptr->m_owner = tree;
		ptr->m_father = father;
		ptr->m_value = rhs->m_value;
		if (rhs->m_left != nullptr)
		{
			ptr->m_left = deepcopy(tree, rhs->m_left, ptr);
		}
		if (rhs->m_right != nullptr)
		{
			ptr->m_right = deepcopy(tree, rhs->m_right, ptr);
		}
		return ptr;
	}

	static void testSerialization8()
	{
		BinaryTreePtr tree = make_shared<BinaryTree>();
		//TreeNodePtr node1 = make_shared<TreeNode>(tree);//相互持有的智能指针，会重复析构
		//TreeNodePtr node2 = make_shared<TreeNode>(tree);
		//TreeNodePtr node3 = make_shared<TreeNode>(tree);
		//TreeNodePtr node4 = make_shared<TreeNode>(tree, node1, node2, 0);
		//TreeNodePtr node5 = make_shared<TreeNode>(tree, node4, node3, 0);

		BinaryTreePtr tree2 = make_shared<BinaryTree>();
		//TreeNodePtr copy_ptr = TreeNode::deepcopy(tree2, node5);

		BinaryTreePtr tree3 = make_shared<BinaryTree>();
		//TreeNode copy = TreeNode(tree3, node5); //使用递归构造函数，必然崩溃

		return;
	}
}
#endif

#define _DEBUG

static int enrol = []()->int
	{
//#if defined( _DEBUG ) //&& !defined( NDEBUG ) //多条件编译
		//DependencyRegistry::getInstance().set("csg_node_class", new Implementation());
		std::function<std::string(TreeNode*)> function = Implementation::serialize; //普通函数指针转std::function对象
		//DependencyRegistry::getInstance().set("serialize_fun", &function); //debug strange
		
		DependencyRegistry::getInstance().set("serialize", Implementation::serialize);
		DependencyRegistry::getInstance().set("deserialize", Implementation::deserialize);
		DependencyRegistry::getInstance().set("serializition", serializition);
		DependencyRegistry::getInstance().set("deserializition", deserializition);
		//DependencyInversion::getInstance().set("serializition", serializition); //why release crash
		//DependencyInversion::getInstance().set("deserializition", deserializition);
//#endif

		testSerialization2();
		testSerialization3();
		testSerialization4();
		testSerialization5();
		//testSerialization6();
		//testSerialization7();
		cout << "test_serialize finished.\n" << endl;
		return 0;
	}();
