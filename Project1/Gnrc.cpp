#include "pch.h"
namespace para
{
    typedef None BPParaNone;

    Gnrc::Gnrc()
    {
        m_imp = new _GeneTemplateClass<BPParaNone>(BPParaNone());
    }
    std::map<std::type_index, _GeneFactoryClassBase*> Gnrc::s_map;
    std::map<std::type_index, _GeneFactoryClassBase*>& Gnrc::_getFactoryMap()
    {
        return s_map;
    }
    Gnrc::Gnrc(std::type_index id, void* src /*= nullptr*/, bool isRef /*= false*/)
    {
        auto iter = _getFactoryMap().find(id);
        if (iter == _getFactoryMap().end())
        {
#ifdef _DEBUG
            throw std::runtime_error("classtype without enrol!");
#else
            m_imp = new _GeneTemplateClass<BPParaNone>(BPParaNone()); // default constructor
#endif
        }
        else
        {
            m_imp = iter->second->create(src, isRef);
        }
    }


    Gnrc::Gnrc(const Gnrc& lhs)
    {
        if (!lhs.m_imp)
            m_imp = new _GeneTemplateClass<BPParaNone>(BPParaNone());// nullptr;
        else
            m_imp = lhs.m_imp->_constructor();
    }

    Gnrc& Gnrc::operator=(const Gnrc& lhs) noexcept
    {
        if (!lhs.m_imp)
        {
            m_imp = new _GeneTemplateClass<BPParaNone>(BPParaNone());// nullptr;
        }
        else if (this != &lhs)
        {
            m_imp = lhs.m_imp->_constructor();
        }
        return *this;
    }

    Gnrc::Gnrc(Gnrc&& rhs) noexcept : m_imp(rhs.m_imp)
    {
        rhs.m_imp = new _GeneTemplateClass<BPParaNone>(BPParaNone());
    }

    Gnrc& Gnrc::operator=(Gnrc&& src) noexcept
    {
        if (this != &src)
        {
            delete m_imp;
            m_imp = src.m_imp;
            src.m_imp = new _GeneTemplateClass<BPParaNone>(BPParaNone());
        }
        return *this;
    }
    Gnrc* Gnrc::operator&()
    {
        return this;
    }
    const Gnrc* Gnrc::operator&() const
    {
        return this;
    }

    bool Gnrc::operator==(const Gnrc& other) const
    {
        if (!this->m_imp && !other.m_imp)
            return true;
        if (!this->m_imp || !other.m_imp)
            return false;
        return this->m_imp->_is_equal(*other.m_imp);
    }

    bool Gnrc::operator<(const Gnrc& other) const
    {
        if (!this->m_imp)
            return false;
        return this->m_imp->_is_less(*other.m_imp);
    }

    //original gnrc function
    const std::type_index Gnrc::_id() const
    {
        return m_imp->_get_id();
    }
    void* Gnrc::_imp() const
    {
        return m_imp->_get_imp();
        //return (void*)m_imp;
    }

    Gnrc::~Gnrc()
    {
        if (m_imp)
        {
            delete m_imp;
            m_imp = nullptr;
        }
    }

}