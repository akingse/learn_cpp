#include "pch.h"

std::map<std::string, DependencyRegistry::FunctionPointer> DependencyRegistry::sm_implementations;

std::string ppc::serial(const std::shared_ptr<TreeNode>& csg)
{
    DependencyRegistry& reg = DependencyRegistry::getInstance();
	auto serialize_fun = reg.get<std::string(TreeNode*)>("serialize");
	if (serialize_fun == nullptr)
		return {};
	std::string data = (*serialize_fun)(csg.get());
	return data;
}

std::shared_ptr<TreeNode> ppc::deserial(const std::string& data)
{
	DependencyRegistry& reg = DependencyRegistry::getInstance();
	auto deserialize_fun = reg.get<TreeNode*(const std::string&)>("deserialize");
	if (deserialize_fun == nullptr)
		return {};
	TreeNode* nodeDe = (*deserialize_fun)(data);
	return std::shared_ptr<TreeNode>(nodeDe);
}

static void test0()
{
	//DependencyRegistry& reg = DependencyRegistry::getInstance();
	//Interface* ptr = reg.get<Interface*>("csg");
	//if (ptr == nullptr)
	//	return;
	//std::shared_ptr<TreeNode> node = ptr->create();
	//std::vector<unsigned char> data = ptr->serial(node);
	//std::shared_ptr<TreeNode> nodeDe = ptr->deserial(data);
	//return;
}

static int enrol = []()->int
	{
		//test0();
  //      printf("test_serialize finished.\n");
		return 0;
	}();
