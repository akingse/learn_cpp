#include "pch.h"
#include <iostream>
#include <windows.h>
#include <random>
#include <chrono>
#include <array>
#include <iomanip>
#include <Iphlpapi.h>
#pragma comment(lib, "Iphlpapi.lib")
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

