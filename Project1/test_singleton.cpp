#include "pch.h"
#include "test_singleton.h"


static std::map<std::type_index, void*> m_id2ptr;
void* __getImplementation(std::type_index id)
{
    auto iter = m_id2ptr.find(id);
    if (iter == m_id2ptr.end())
        return nullptr;
    return iter->second;
}