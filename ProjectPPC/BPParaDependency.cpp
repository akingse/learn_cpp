#include "pch.h"
using namespace para;
//init
std::map<std::string, DependencyRegistry::FunctionPointer> DependencyRegistry::sm_implementations;// = { {"default",DependencyRegistry::FunctionPointer() } };
std::map<std::string, std::shared_ptr<void>> DependencyInversion::sm_implementations;
std::mutex DependencyInversion::sm_mutex;


std::vector<unsigned char> para::serializition(const std::shared_ptr<TreeNodePtr>& node)
{
    DependencyRegistry& reg = DependencyRegistry::getInstance();
	auto serialize_fun = reg.get<std::vector<unsigned char>(const std::shared_ptr<TreeNodePtr>&)>("serializition");
	std::vector<byte> data;
	if (serialize_fun != nullptr)
		data = (*serialize_fun)(node);
	return data;
}

std::shared_ptr<TreeNodePtr> para::deserializition(const std::vector<unsigned char>& data)
{
	DependencyRegistry& reg = DependencyRegistry::getInstance();
	auto deserialize_fun = reg.get<std::shared_ptr<TreeNodePtr>(const std::vector<unsigned char>&)>("deserializition");
	std::shared_ptr<TreeNodePtr> node;
	if (deserialize_fun != nullptr)
		node = (*deserialize_fun)(data);
	return node;
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
