#include "pch.h"
using namespace std;

std::vector<unsigned char> serialize(const BPGeometricPrimitiveSer& primitive)
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
BPGeometricPrimitiveSer deserialize(const std::vector<unsigned char>& infor)
{
	//�����л� infor
	bool hollow;
	size_t identification;
	const unsigned char* ptr = infor.data();
	memcpy(&hollow, ptr, sizeof(bool));					ptr += sizeof(bool);
	memcpy(&identification, ptr, sizeof(size_t));		ptr += sizeof(size_t);
	long long n = infor.size()-(ptr - infor.data()); //����string�ֳ�
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
////�򵥵ı�����ʹ�õݹ顭
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
    ��Ƕ�������л�

    //���л� ģ����ǿת reinterpret_cast
    virtual std::vector<unsigned char> _serialize() const override
    {
        const unsigned char* ptr = reinterpret_cast<const unsigned char*>(&m_data);
        return std::vector<unsigned char>(ptr, ptr + sizeof(T));
    }
    //�����л�
    static __Primitive<T>* _deserialize(const std::vector<unsigned char> buf)
    {
        T data;
        memcpy(&data, buf.data(), sizeof(T));
        return new __Primitive<T>(data);
    }

    �̶���ʽ���� using memcpy
    unsigned char* ptr = infor.data();
    memcpy(ptr, &identification, sizeof(size_t));       ptr += sizeof(size_t);
    memcpy(ptr, &hollow, sizeof(bool));                 ptr += sizeof(bool);

    ProjectManagerImplementation�����ļ����л�
    std::tuple<BPParaMD5, std::vector<unsigned char>, std::vector<unsigned char>> temp = __PrimitiveInterface::serialize(obj);
    std::get<2>(temp).resize(std::get<2>(temp).size() + sizeof(BPParaMD5));
    memcpy(std::get<2>(temp).data() + std::get<2>(temp).size() - sizeof(BPParaMD5), &std::get<0>(temp), sizeof(BPParaMD5));
    return std::get<2>(temp);


    */

// ���л�-������

struct TreeNode 
{
	int m_nodeIndex;
	std::vector<int> m_geomIndexes;
	TreeNode* m_father;
	TreeNode* m_left;
	TreeNode* m_right;

	TreeNode() : m_nodeIndex(-1), m_father(nullptr), m_left(nullptr), m_right(nullptr) {}
	TreeNode(int x) : m_nodeIndex(x), m_father(nullptr), m_left(nullptr), m_right(nullptr) {}
};

// ���л�������Ϊ�ַ���
std::string serialize(TreeNode* root) {
	if (root == nullptr) 
		return "$";
    std::vector<unsigned char> temp(sizeof(int) + sizeof(int) * root->m_geomIndexes.size() + sizeof(int));
    unsigned char* ptr = temp.data();
    int count = root->m_geomIndexes.size();
    memcpy(ptr, &count, sizeof(int)); ptr += sizeof(int);
    memcpy(ptr, root->m_geomIndexes.data(), sizeof(int) * root->m_geomIndexes.size()); ptr += sizeof(int) * root->m_geomIndexes.size();
    memcpy(ptr, &root->m_nodeIndex, sizeof(int)); ptr += sizeof(int);
    //data.insert(data.end(), temp.begin(), temp.end());

	//std::string serialized = std::to_string(root->m_nodeIndex);
	std::string serialized(temp.begin(), temp.end());
	serialized += " " + serialize(root->m_left);
	serialized += " " + serialize(root->m_right);

	return serialized;
}
// �����л��ַ���Ϊ������
TreeNode* deserialize_istring(std::istringstream& iss, TreeNode* father = nullptr) {
	std::string token;
	iss >> token;//Ĭ������£�istringstream�ո񣨿հ��ַ�������Ϊ�ָ��������������
	if (token == "$") 
		return nullptr;
	int count;
	int nodeIndex;
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

// ��intֵ���л�Ϊstd::string
std::string serializeInt(int value) {
	std::stringstream ss;
	ss << value;
	return ss.str();
}

// ��boolֵ���л�Ϊstd::string
std::string serializeBool(bool value) {
	return value ? "true" : "false";
}

// �������л�����
void testSerialization0() {
	// ����һ��������
	TreeNode* root = new TreeNode(1);
	root->m_left = new TreeNode(2);
	root->m_left->m_geomIndexes = { 2,22,222 };
	root->m_right = new TreeNode(3);
	root->m_left->m_left = new TreeNode(4);
	root->m_left->m_left->m_geomIndexes = { 4,44 };
	root->m_left->m_right = new TreeNode(5);

	// ���л�
	std::string serialized = serialize(root);
	std::cout << "Serialized: " << serialized << std::endl;

	//const std::vector<unsigned char>* charVector = reinterpret_cast<const vector<unsigned char>*>(serialized.data());
	//const std::string* strVector = reinterpret_cast<const string*>(charVector->data());
	std::vector<unsigned char> charVector(serialized.size());
	memcpy(charVector.data(), serialized.data(), serialized.size());
	std::string strVector(charVector.begin(), charVector.end());

	// �����л�
	TreeNode* deserialized = deserialize(serialized);
	TreeNode* deserialized1 = deserialize(strVector);
	std::cout << "Deserialized: " << deserialized->m_nodeIndex << std::endl;

	int intValue = -42;
	bool boolValue = true;

	//std::vector<unsigned char> charVector(sizeof(int));
	//memcpy(charVector.data(), &intValue, sizeof(int));
	//std::string str(charVector.begin(), charVector.end());
	//int intDeser;
	//memcpy(&intDeser, str.data(), sizeof(int));
	//std::istringstream iss(str);
	//std::string token;
	//iss >> token;
	//std::stoi(token);

	std::string serializedInt = serializeInt(intValue);
	std::string serializedBool = serializeBool(boolValue);

	std::cout << "Serialized int value: " << serializedInt << std::endl;
	std::cout << "Serialized bool value: " << serializedBool << std::endl;
}

//FatherLeftRight
void serialize(TreeNode* root, std::vector<unsigned char>& data) 
{
	if (root == nullptr) {
		data.push_back('$'); // �������ַ���ʾ�սڵ�
		return;
	}
	//// ���ڵ�ֵת��Ϊ�ֽڲ���ӵ�data��
	//std::vector<unsigned char> temp(sizeof(int) + sizeof(int) * root->m_geomIndexes.size() + sizeof(int));
	//unsigned char* ptr = temp.data();
	//int count = root->m_geomIndexes.size();
	//memcpy(ptr, &count, sizeof(int)); ptr += sizeof(int);
	//memcpy(ptr, root->m_geomIndexes.data(), sizeof(int) * root->m_geomIndexes.size()); ptr += sizeof(int) * root->m_geomIndexes.size();
	//memcpy(ptr, &root->m_nodeIndex, sizeof(int)); ptr += sizeof(int);
	//data.insert(data.end(),temp.begin(),temp.end());


	// ���ڵ�ֵת��Ϊ�ֽڲ���ӵ�data��
	data.push_back(static_cast<char>(root->m_nodeIndex));
	// �ݹ����л���������������
	serialize(root->m_left, data);
	serialize(root->m_right, data);
}

// �����л��ֽ���Ϊ������
TreeNode* deserialize(std::vector<unsigned char>& data, int& index, TreeNode* m_father) 
{
	if (index >= data.size() || data[index] == '$') {
		index++;
		return nullptr;
	}
	// ��data�ж�ȡ�ڵ�ֵ�������ڵ�
	TreeNode* node = new TreeNode(static_cast<int>(data[index]));
	node->m_father = m_father; // ���ø�ָ��

	// �ݹ鷴���л���������������
	index++;
	node->m_left = deserialize(data, index, node);
	index++;
	node->m_right = deserialize(data, index, node);

	return node;
}

void testSerialization2()
{
	TreeNode* root = new TreeNode(1);
	root->m_left = new TreeNode(2);
	root->m_right = new TreeNode(3);
	root->m_left->m_father = root;
	root->m_right->m_father = root;
	root->m_left->m_geomIndexes = { 1, 2, 3 }; // ʾ�� vector<int> �ĸ�ֵ

	std::vector<unsigned char> serializedData;
	serialize(root, serializedData);

	// ������л�����ֽ���
	std::cout << "Serialized data: ";
	for (char byte : serializedData)
		std::cout << byte << " ";
	std::cout << std::endl;

	int index = 0;
	TreeNode* deserializedRoot = deserialize(serializedData, index, nullptr);

	return;
}

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

//�������յ�����ָ�� data ��һ���������ں����ڲ����� data ��ƫ�Ʋ���ֻ���޸ĺ����ڲ��ĸ���������Ӱ��ԭʼָ���ֵ��
static void test_pointer_c(const byte* data)
{
	data += 4;
	//*data = 'c';
	cout << *data << endl;
}

//�������յ�����ָ�� data �����á��ں����ڲ����� data ��ƫ�Ʋ�����ֱ���޸�ԭʼָ���ֵ����Ϊ�����ڲ���������ԭʼָ������á�
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

//����ָ���ֵ
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

static void _test1()
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

static int enrol = []()->int
	{
		testSerialization0();
		//testSerialization1();
		testSerialization2();
		_test1();
		cout << "test_serialize finished.\n" << endl;
		return 0;
	}();
