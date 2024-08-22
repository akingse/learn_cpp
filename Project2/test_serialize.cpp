#include "pch.h"
using namespace std;

std::vector<unsigned char> _bp_serialize(const BPGeometricPrimitiveSer& primitive)
{
	//���л� infor
	std::vector<unsigned char> infor(sizeof(bool) + sizeof(size_t) + primitive.m_remark.size()); //vector<bool> a(10);//ָ��size
	//using memcpy
	unsigned char* ptr = infor.data();
	memcpy(ptr, &primitive.m_hollow, sizeof(bool));						ptr += sizeof(bool);
	memcpy(ptr, &primitive.m_identification, sizeof(size_t));			ptr += sizeof(size_t);
	memcpy(ptr, primitive.m_remark.data(), primitive.m_remark.size());  ptr += primitive.m_remark.size();
	return infor;
}

BPGeometricPrimitiveSer _bp_deserialize(const std::vector<unsigned char>& infor)
{
	//�����л� infor
	bool hollow;
	size_t identification;
	const unsigned char* ptr = infor.data();
	memcpy(&hollow, ptr, sizeof(bool));					ptr += sizeof(bool);
	memcpy(&identification, ptr, sizeof(size_t));		ptr += sizeof(size_t);
    long long n = infor.size() - (ptr - infor.data()); //����string�ֳ�
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

	// vector ���͹���
	//std::vector<unsigned char> infor1(remark.size()); //byte 6
	//std::vector<unsigned char> infor2(sizeof(bool));//byte 1
	//std::vector<unsigned char> infor3(sizeof(size_t));//byte 8
	std::vector<unsigned char> infor(15);

	//ע�������� sizeof() �� .size()
	size_t sz0 = sizeof(Vec3d); //24
	size_t sz1 = infor.size(); //15
	size_t sz2 = sizeof(infor); //32
	size_t sz3 = sizeof(string); //40
	size_t sz6 = sizeof(remark); //40
	size_t sz4 = sizeof(vector<void*>); //32
	size_t sz5 = sizeof(map<bool,bool>); //24
	size_t sz7 = sizeof(map<size_t, size_t>); //24

	//��ǿת������
	Vec3d vec;
	const unsigned char* ptr = reinterpret_cast<const unsigned char*>(&vec);
	std::vector<unsigned char> res = std::vector<unsigned char>(ptr, ptr + sizeof(Vec3d));
	Vec3d vecDe;
	memcpy(&vecDe, res.data(), sizeof(Vec3d));

	//���л�����
	vector<double> ori = { 1,2,4,1,2,4, 1,2,4,1,2,4, 1,2,4,1,2,4};
	size_t sz8 = sizeof(ori); //32
	const unsigned char* ser = reinterpret_cast<const unsigned char*>(&ori);
	std::vector<unsigned char> uchar = std::vector<unsigned char>(ser, ser + sizeof(ori));
	vector<double> tar(sizeof(ori));
	memcpy(&tar, uchar.data(), sizeof(ori));

	//Ƕ������
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

//------------------------------------------------------------------------------------------

//memcpy��������
class IGemetry
{
};

class ICsgTree
{
public:
	//int id = 0; //Ĭ���޶��룬��ռ8byte
	virtual ~ICsgTree() = 0;
};

//min(n in #pragma pack(n),�ṹ���е������)
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

static void testSerialization1()
{
	int sz0 = sizeof(ICsgTree);
	int sz1 = sizeof(GeCsgTree);
	int sz2 = sizeof(CsgTreeData);
	int sz3 = sizeof(vector<shared_ptr<ICsgTree>>);
	CsgTreeData data;
	// ����GUID
	boost::uuids::random_generator generator;
	boost::uuids::uuid uuid = generator();

	// ��GUIDת��Ϊ�ַ���
	std::string uuidStr = boost::uuids::to_string(uuid);

	// ���GUID
	std::cout << "Generated GUID: " << uuidStr << std::endl;//Generated GUID: 57186994-bef7-4134-832b-fc3aa6866063
	/*
	UUID��Universally Unique Identifier���ı�׼��ʾ��ʽ��һ������32��ʮ�����ƣ�hexadecimal���ַ����ַ�����
	��Ϊ������֣������ַ��ָ������磺"xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"��

	ǰ8���ַ���ʾUUID��32λ���ݵ�ǰ32λ��
	��9��12���ַ���ʾUUID��32λ���ݵĽ�������16λ��
	��13��16���ַ���ʾUUID��32λ���ݵĽ�������16λ��
	��17��20���ַ���ʾUUID��32λ���ݵĽ�������16λ��
	���12���ַ���ʾUUID��32λ���ݵĽ�������48λ��
	*/

	return;
}


//----------------------------------------------------------------------------------------------
// ���л�-������
//----------------------------------------------------------------------------------------------

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

struct TreeNodePtr
{
	int m_nodeIndex;
	std::vector<int> m_geomIndexes;
	std::shared_ptr<TreeNodePtr> m_father;
	std::shared_ptr<TreeNodePtr> m_left;
	std::shared_ptr<TreeNodePtr> m_right;
	TreeNodePtr(int x) :m_nodeIndex(x) {}

};

//�������������
TreeNode* generateRandomTree(int maxDepth, int currentDepth = 1)
{
	// �����ǰ�㼶�������㼶�����ؿ�ָ��
	if (currentDepth > maxDepth)
		return nullptr;
	// ������ɽڵ�ֵ
	//int nodeValue = rand() % 100; // �������ֵ��0-99��
	TreeNode* node = new TreeNode;//
	// ��������Ƿ񴴽���������������
	if (rand() % 2 == 0)
	{ // 50% �ĸ��ʴ���������
		node->m_left = generateRandomTree(maxDepth, currentDepth + 1);
		node->m_right = generateRandomTree(maxDepth, currentDepth + 1);
	}
	return node;
}

// ���л�������Ϊ�ַ���
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

// �����л��ַ���Ϊ������
TreeNode* deserialize_istring(std::istringstream& iss, TreeNode* father = nullptr) {
	std::string token;
	//iss >> token;//Ĭ������£�istringstream�ո񣨿հ��ַ�������Ϊ�ָ��������������
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
// �����л��ַ���Ϊ������������������
TreeNode* deserialize(const std::string& data) {
	//�����л����ַ����а����ض�������ȡ���ݣ����簴�ո�ָ��ڵ��ֵ���ַ������ṩ�˷���ķ���������������
	//�ڷ����л������У�������Ҫ���ַ�����ʾ�Ľڵ�ֵת��Ϊ�������������͡��ַ��������Խ��ַ�������Ϊ�ʵ����������ͣ���std::stoi���ڽ��ַ���ת��Ϊ������
	std::istringstream iss(data);
	return deserialize_istring(iss);
}

//��д���л����޸����л��߼�����������ָ��

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
	std::vector<byte> bufferLeft = serializition(root->m_left);
	count = (int)bufferLeft.size();
	std::vector<byte> sizeLeft(sizeof(int));
	memcpy(sizeLeft.data(), &count, sizeof(int));
	serialized.insert(serialized.end(), sizeLeft.begin(), sizeLeft.end());
	serialized.insert(serialized.end(), bufferLeft.begin(), bufferLeft.end());
	//rightNode
	std::vector<byte> bufferRight = serializition(root->m_right);
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

// �������л�����
void testSerialization2() 
{
	// ����һ��������
	TreeNode* root = new TreeNode(1);
	root->m_left = new TreeNode(2);
	root->m_left->m_geomIndexes = { 2,22,222 };
	root->m_right = new TreeNode(-1);
	root->m_left->m_left = new TreeNode(4);
	root->m_left->m_left->m_geomIndexes = { 4,44 };
	root->m_left->m_right = new TreeNode(5);

	// ���л�
	std::string serialized = serialize(root);
	std::cout << "Serialized: " << serialized << std::endl;

	std::vector<unsigned char> charVector(serialized.size());
	memcpy(charVector.data(), serialized.data(), serialized.size());
	//����������
	//std::string strVector(charVector.begin(), charVector.end());
	//memcpy
	std::string strVector(charVector.size(), 0);
	memcpy(const_cast<char*>(strVector.data()), charVector.data(), charVector.size());

	// �����л�
	TreeNode* deserialized = deserialize(serialized);
	TreeNode* deserialized1 = deserialize(strVector);
	std::cout << "Deserialized: " << deserialized->m_nodeIndex << std::endl;

	// ���б�dataΪ��ָ��
	vector<int> vecInt(0);
    int* ptr = vecInt.data();
	//iss >> tokenĬ�Ϸָ���
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

//string��std::vector<unsigned char>ת��
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

//int memcpy�Ƿ�Ϊ��
void testSerialization4()
{
	int num = 12345;
	char buffer[20]; // 20 �ǻ�������С������ʵ���������
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

void testSerialization5()
{
	srand(int(time(0))); // �����������
	// ���ѡ��㼶�� 2 �� 10 ֮��
	int maxDepth = 4;// +rand() % 9; // �����ȣ��� 2 �� 10��
	std::cout << "Generating a random binary tree with up to " << maxDepth << " levels." << std::endl;

	// �������������
	TreeNode* root = generateRandomTree(maxDepth);

	return;
}

class Implementation : public Interface 
{
public:
	virtual std::shared_ptr<TreeNode> create() override
	{
		TreeNode* node = new TreeNode(1);
		node->m_left= new TreeNode(2);
		node->m_right= new TreeNode(3);
		return shared_ptr<TreeNode>(node);
	}
	virtual std::vector<unsigned char> serial(const std::shared_ptr<TreeNode>& csg) override
	{
		if (csg == nullptr)
			return {};
		std::string data = serialize(csg.get());
		std::vector<unsigned char> res(data.size());
		memcpy(res.data(), data.data(), data.size());
		return res;
	}
	virtual std::shared_ptr<TreeNode> deserial(const std::vector<unsigned char>& data) override
	{
		if (data.empty())
			return nullptr;
		std::string buffer(data.size(),0);
		memcpy(const_cast<char*>(buffer.data()), data.data(), data.size());
		TreeNode* node = deserialize(buffer);
		return shared_ptr<TreeNode>(node);
	}
	//static
	static std::shared_ptr<TreeNode> createNew()
	{
		TreeNode* node = new TreeNode(1);
		node->m_left = new TreeNode(2);
		node->m_left->m_geomIndexes = { 22,222 };
		node->m_right = new TreeNode(3);
		node->m_left->m_left = new TreeNode(4);
		node->m_left->m_right = new TreeNode(5);
		return shared_ptr<TreeNode>(node);
	}
	static std::shared_ptr<TreeNodePtr> createNewPtr()
	{
		shared_ptr<TreeNodePtr> node = make_shared<TreeNodePtr>(1);
		node->m_left = make_shared<TreeNodePtr>(2);
		node->m_left->m_geomIndexes = { 22,222 };
		node->m_right = make_shared<TreeNodePtr>(3);
		node->m_left->m_left = make_shared<TreeNodePtr>(4);
		node->m_left->m_right = make_shared<TreeNodePtr>(5);
		return node;
	}

};

static void testSerialization6()
{
	// ����һ�� std::function��ָ�������� normalFunction ƥ��ĺ���ָ��
	std::function<void()> func = testSerialization5;
	//std::cout << "typeid(func).name(): " << typeid(func).name() << std::endl;
	//std::cout << "typeid(&normalFunction).name(): " << typeid(&testSerialization5).name() << std::endl;
	// �Ƚ� std::function �� ��ͨ����ָ��� typeid
	if (typeid(func) == typeid(&testSerialization5))
		std::cout << "The types are the same." << std::endl;

	DependencyRegistry& reg = DependencyRegistry::getInstance();
	Implementation* ptr = reg.get<Implementation>("csg_node_class"); //����������
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

	//�÷�һ��std::function
	function<std::string(TreeNode*)>* serialize_fun1 = reg.get<function<std::string(TreeNode*)>>("serialize_fun");
	std::string buffer1 = (*serialize_fun1)(node.get());

	//�÷���������ָ��
	std::string buffer = serialize(node.get());
	auto serialize_fun = reg.get<std::string(TreeNode*)>("serialize");
	std::string data2 = (*serialize_fun)(node.get());
	auto deserialize_fun = reg.get<TreeNode*(const std::string&)>("deserialize");
	TreeNode* nodeDe2 = (*deserialize_fun)(buffer);

	//�����ù�������Ŀ1ʵ�֣�������װ��ĺ���
    std::shared_ptr<TreeNode> node3 = Implementation::createNew();
	//std::string data3 = ppc::serial(node3);
	//std::shared_ptr<TreeNode> nodeDe3 = ppc::deserial(data3);

	return;
}

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

static int enrol = []()->int
	{
		std::function<std::string(TreeNode*)> function = serialize; //��ͨ����ָ��תstd::function����
		DependencyRegistry::getInstance().set("csg_node_class", new Implementation());
		DependencyRegistry::getInstance().set("serialize_fun", &function);
		DependencyRegistry::getInstance().set("serialize", serialize);
		DependencyRegistry::getInstance().set("deserialize", deserialize);
		DependencyRegistry::getInstance().set("serializition", serializition);
		DependencyRegistry::getInstance().set("deserializition", deserializition);

		testSerialization1();
		testSerialization2();
		testSerialization3();
		testSerialization4();
		testSerialization5();
		testSerialization6();
		testSerialization7();
		cout << "test_serialize finished.\n" << endl;
		return 0;
	}();
