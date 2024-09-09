#include "pch.h"
#include <iomanip>
#include <Iphlpapi.h>
#pragma comment(lib, "Iphlpapi.lib")
using namespace std;
//typedef unsigned char byte;
struct Guid
{
    size_t s1;
    size_t s2;
    size_t s3;
    size_t s4;
};

std::ostream& operator<<(std::ostream& out, Guid guid)
{
    out << std::setw(16) << std::setfill('0') << std::hex << guid.s1;
    out << std::setw(16) << std::setfill('0') << std::hex << guid.s2;
    out << std::setw(16) << std::setfill('0') << std::hex << guid.s3;
    out << std::setw(16) << std::setfill('0') << std::hex << guid.s4;
    return out;
}

Guid make_guid()
{
    // random
    Guid guid;
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::uniform_int_distribution<size_t> dist;
    guid.s1 = dist(gen);
    // current_epoch_time
    guid.s2 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // machine_running_to_now_duration
    guid.s3 = std::chrono::steady_clock::now().time_since_epoch().count();
    // mac
    std::unique_ptr<IP_ADAPTER_INFO> pIPAdapterInfo(new IP_ADAPTER_INFO());
    ULONG size = sizeof(IP_ADAPTER_INFO);
    int nRet = GetAdaptersInfo(pIPAdapterInfo.get(), &size);
    if (ERROR_BUFFER_OVERFLOW == nRet)
    {
        pIPAdapterInfo.reset((PIP_ADAPTER_INFO)new byte[size]);
        nRet = GetAdaptersInfo(pIPAdapterInfo.get(), &size);
    }
    if (ERROR_SUCCESS != nRet)
        return guid;
    std::array<unsigned char, 8> mac;
    for (int i = 0; i < pIPAdapterInfo->AddressLength; i++)
        mac[i] = pIPAdapterInfo->Address[i];
    for (int i = pIPAdapterInfo->AddressLength; i < 8; i++)
        mac[i] = dist(gen);
    guid.s4 = 0;
    for (int i = 0; i < 8; i++)
        guid.s4 = guid.s4 | (size_t(mac[i]) << (i * 8));
    return guid;
}

int mainguid()
{

    for (size_t i = 0; i < 20; i++)
    {
        Guid guid = make_guid();
        std::cout << guid << std::endl;
    }
    return 0;
}

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
static void _test_guid0()
{
    // 生成GUID
    boost::uuids::random_generator generator;
    boost::uuids::uuid uuid = generator();

    // 将GUID转换为字符串
    std::string uuidStr = boost::uuids::to_string(uuid);

    // 输出GUID
    std::cout << "Generated GUID: " << uuidStr << std::endl;//Generated GUID: 57186994-bef7-4134-832b-fc3aa6866063
    /*
    UUID（Universally Unique Identifier）的标准表示形式是一个包含32个十六进制（hexadecimal）字符的字符串，
    分为五个部分，用连字符分隔，形如："xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"。

    前8个字符表示UUID的32位数据的前32位。
    第9至12个字符表示UUID的32位数据的接下来的16位。
    第13至16个字符表示UUID的32位数据的接下来的16位。
    第17至20个字符表示UUID的32位数据的接下来的16位。
    最后12个字符表示UUID的32位数据的接下来的48位。
    */
    return;
}


#include <windows.h>
#include <objbase.h> // 包含 CoCreateGuid 函数的头文件
//windows API
static void _test_guid1()
{
    GUID guid = {};
    // 初始化COM库
    HRESULT result = CoInitialize(NULL);
    if (FAILED(result)) {
        std::cerr << "COM库初始化失败: " << std::hex << result << std::endl;
        return;
    }
    map<GUID, double> gmap;

    // 创建GUID
    result = CoCreateGuid(&guid);
    if (FAILED(result)) {
        std::cerr << "生成GUID失败: " << std::hex << result << std::endl;
        CoUninitialize();
        return;
    }

    // 输出GUID
    std::cout << "生成的GUID是: {"
        << std::hex << std::uppercase
        << guid.Data1 << '-'
        << guid.Data2 << '-'
        << guid.Data3 << '-'
        << static_cast<int>(guid.Data4[0]) << static_cast<int>(guid.Data4[1]) << '-'
        << static_cast<int>(guid.Data4[2]) << static_cast<int>(guid.Data4[3])
        << static_cast<int>(guid.Data4[4]) << static_cast<int>(guid.Data4[5])
        << static_cast<int>(guid.Data4[6]) << static_cast<int>(guid.Data4[7])
        << "}" << std::endl;
    int n = sizeof(guid);
    // 清理COM库
    CoUninitialize();
}

static int enrol = []()->int
    {
        _test_guid0();
        _test_guid1();
        cout << "test_guid finished.\n" << endl;
        return 0;
    }();
