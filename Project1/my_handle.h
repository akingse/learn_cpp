#pragma once
class BPParaHandle
{
public:
    enum HandleType : unsigned char
    {
        enInvalid = 0,
        enPropertyHandle = 10,
    };
    size_t m_handle1 = 0;
    size_t m_handle2 = 0;
    unsigned char m_type = 0;

public:
    template<typename T>
    bool isHandleType() const
    {
        static_assert(std::is_base_of<BPParaHandle, T>::value, "Only judgement between handles is allowed.");
        //return m_handle[15] == T::getHandleType();
        return m_type == T::getHandleType();
    }
    template<typename T>
    inline T asHandleType() const
    {
        static_assert(std::is_base_of<BPParaHandle, T>::value, "Only judgement between handles is allowed.");
        T res;
        res.m_handle1 = m_handle1;
        res.m_handle2 = m_handle2;
        res.m_type = T::getHandleType();
        return res;
    }
    BPParaHandle()
    {
    }
    BPParaHandle(const BPParaHandle& rhs) :m_handle1(rhs.m_handle1), m_handle2(rhs.m_handle2), m_type(rhs.m_type)
    {
    }
    BPParaHandle(size_t handle1, size_t handle2, HandleType type):m_handle1(handle1), m_handle2(handle2), m_type(type)
    {
    }
    virtual ~BPParaHandle()
    {
    }
    inline bool operator<(const BPParaHandle& _r) const
    {
        //char* pOffset=(char*)(this);        pOffset += sizeof(void*); //vtptr
        //size_t* hd1 = (size_t*)(pOffset);   pOffset += sizeof(size_t);
        //size_t* hd2 = (size_t*)(pOffset);   pOffset += sizeof(size_t);

        size_t* pOffset = (size_t*)this;                   pOffset += 1;
        size_t* hd1 = (size_t*)(pOffset);       pOffset += 1;
        size_t* hd2 = (size_t*)(pOffset);       pOffset += 1;

        //BPParaHandle copy = *this;
        //memcpy(&copy, nullptr, sizeof(void*));

        //��һ����ӵ���������ϵĳ�Աʱ����Ҫдͳһ�ıȽϺ���
		int res = memcmp((char*)(this) + sizeof(void*), (char*)(&_r) + sizeof(void*), sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char));
		int res1 = memcmp((size_t*)(this) + 1, (size_t*)(&_r) + 1, sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char));
        //return res1;
        //return m_handle1 < _r.m_handle1 || m_handle2 < _r.m_handle2 || m_type < _r.m_type;

        size_t mhash = (std::hash<size_t>()(m_handle1)) ^ (std::hash<size_t>()(m_handle2) << 1) ^ (std::hash<unsigned char>()(m_type) << 2);
        size_t rhash = (std::hash<size_t>()(_r.m_handle1)) ^ (std::hash<size_t>()(_r.m_handle2) << 1) ^ (std::hash<unsigned char>()(_r.m_type) << 2);
        return mhash < rhash;
    }
    inline bool operator==(const BPParaHandle& _r) const
    {
        return m_handle1 == _r.m_handle1 && m_handle2 < _r.m_handle2 && m_type == _r.m_type;
    }
};


class BPPropertyHandle :
    public BPParaHandle
{
public:
    inline static HandleType getHandleType()
    {
        return BPParaHandle::enPropertyHandle;
    }
    BPPropertyHandle() {}
    BPPropertyHandle(size_t handle1, size_t handle2) //:m_handle(handle), m_type(type)
    {
        m_handle1 = handle1;
        m_handle2 = handle2;
		m_type = BPParaHandle::enPropertyHandle;
    }
    ~BPPropertyHandle() {}
};

