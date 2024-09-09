#pragma once

//测试 序列化和反序列化
class BPGeometricPrimitiveSer
{
public:
	bool m_hollow = false;
	size_t m_identification = 0;
	std::string m_remark = "";
	BPGeometricPrimitiveSer() = default;
	BPGeometricPrimitiveSer(bool hollow, size_t identification, const std::string& remark)
	{
		m_hollow = hollow;
		m_identification = identification;
		m_remark = remark;
	}
	BPGeometricPrimitiveSer(const BPGeometricPrimitiveSer&) = default;
	~BPGeometricPrimitiveSer() = default;

	template<class T>
	bool is() 
	{
		return true;
	}
	template<class T>
	T as()
	{
		return T;
	}
};


//------------------------------------------------------------------------
//			for csgtree
//------------------------------------------------------------------------

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
	std::vector<std::shared_ptr<ICsgTree>> m_leftNodes;
	std::vector<std::shared_ptr<ICsgTree>> m_rightNodes;
public:
	virtual ~GeCsgTree() override {};
};
#pragma pack() //end

struct CsgTreeData
{
	std::vector<std::shared_ptr<ICsgTree>> m_leftNodes;
	std::vector<std::shared_ptr<ICsgTree>> m_rightNodes;
	//bool isLeft;
};

namespace ppc
{
	static std::atomic<int> s_allnodes_index = 1;

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
}

using namespace ppc;
class Implementation : public Interface
{
public:
	// 序列化二叉树为字符串
	static std::string serialize(TreeNode* root) {
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
	static TreeNode* deserialize_istring(std::istringstream& iss, TreeNode* father = nullptr) {
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
	static TreeNode* deserialize(const std::string& data) {
		//从序列化的字符串中按照特定规则提取数据，例如按空格分隔节点的值。字符串流提供了方便的方法来完成这个任务。
		//在反序列化过程中，我们需要将字符串表示的节点值转换为整数或其他类型。字符串流可以将字符串解析为适当的数据类型，如std::stoi用于将字符串转换为整数。
		std::istringstream iss(data);
		return deserialize_istring(iss);
	}


public:
	virtual std::shared_ptr<TreeNode> create() override
	{
		TreeNode* node = new TreeNode(1);
		node->m_left = new TreeNode(2);
		node->m_right = new TreeNode(3);
		return std::shared_ptr<TreeNode>(node);
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
		std::string buffer(data.size(), 0);
		memcpy(const_cast<char*>(buffer.data()), data.data(), data.size());
		TreeNode* node = deserialize(buffer);
		return std::shared_ptr<TreeNode>(node);
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
		return std::shared_ptr<TreeNode>(node);
	}
	static std::shared_ptr<TreeNodePtr> createNewPtr()
	{
		std::shared_ptr<TreeNodePtr> node = std::make_shared<TreeNodePtr>(1);
		node->m_left = std::make_shared<TreeNodePtr>(2);
		node->m_left->m_geomIndexes = { 22,222 };
		node->m_right = std::make_shared<TreeNodePtr>(3);
		node->m_left->m_left = std::make_shared<TreeNodePtr>(4);
		node->m_left->m_right = std::make_shared<TreeNodePtr>(5);
		return node;
	}

};
