#include "pch.h"

//std::map<std::string, DependencyRegistry::FunctionPointer> DependencyRegistry::sm_implementations;

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
