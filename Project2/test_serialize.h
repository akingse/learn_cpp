#pragma once

//���� ���л��ͷ����л�
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
		static TreeNode* create(TreeNode* m_left, TreeNode* m_right, TreeNode* m_father = nullptr)
		{
			TreeNode* ptr = new TreeNode;
			ptr->m_father = m_father;
			ptr->m_left = m_left;
			ptr->m_right = m_right;
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
	// ���л�������Ϊ�ַ���
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

	// �����л��ַ���Ϊ������
	static TreeNode* deserialize_istring(std::istringstream& iss, TreeNode* m_father = nullptr) {
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
		root->m_father = m_father;	m_father = root;
		root->m_geomIndexes = geomIndexes;
		root->m_nodeIndex = nodeIndex;
		root->m_left = deserialize_istring(iss, m_father);
		root->m_right = deserialize_istring(iss, m_father);
		return root;
	}

	// �����л��ַ���Ϊ������������������
	static TreeNode* deserialize(const std::string& data) {
		//�����л����ַ����а����ض�������ȡ���ݣ����簴�ո�ָ��ڵ��ֵ���ַ������ṩ�˷���ķ���������������
		//�ڷ����л������У�������Ҫ���ַ�����ʾ�Ľڵ�ֵת��Ϊ�������������͡��ַ��������Խ��ַ�������Ϊ�ʵ����������ͣ���std::stoi���ڽ��ַ���ת��Ϊ������
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

namespace bin
{
	class TreeNode;
	class BinaryTree;
	typedef std::shared_ptr<TreeNode> TreeNodePtr;
	typedef std::shared_ptr<BinaryTree> BinaryTreePtr;

	// �������ڵ�ṹ
	struct TreeNode
	{
		int value = -1;
		BinaryTreePtr m_owner;
		TreeNodePtr m_father;
		TreeNodePtr m_left;
		TreeNodePtr m_right;

		// ���캯��
		TreeNode() = default;
		//create empty
		TreeNode(const BinaryTreePtr& tree);
		//create by operation
		TreeNode(const BinaryTreePtr& tree, const TreeNodePtr& left, const TreeNodePtr& right, int op);
		//deep copy
		TreeNode(const BinaryTreePtr& tree, const TreeNodePtr& rhs, const TreeNodePtr& father = nullptr);

	};

	// ��������
	class BinaryTree {
	public:
		TreeNodePtr m_root;

		BinaryTree() : m_root(nullptr) {}

		// ɾ����ǰ�ڵ�ķ���
		void deleteNode(TreeNodePtr& node)
		{
			if (node) {
				// �ݹ�ɾ����������
				deleteNode(node->m_left);
				deleteNode(node->m_right);

				// ����������ʱ��node������ָ������뿪������ʱ�Զ��ͷ�
				node.reset();
			}
		}

		// �������� - ���������
		void clear() {
			deleteNode(m_root);
		}
	};

}
