#pragma once
#include <ctime>   
#include <iostream>
#include <fstream>
#include <sstream>
//#ifndef CLASH_DETECTION_SOLUTION
#pragma warning(disable: 4996) //ctime

struct InterTriInfo
{
    std::array<std::array<Eigen::Vector3d, 3>, 2> trianglePair;
    std::array<unsigned long long, 2> entityPair;
    double distance;
};

static size_t countFile = 0;
inline void writeDataContent(const std::string& context, const std::string& fileName = {})
{
    std::time_t currentTime = std::time(nullptr);
    std::string timeString = std::ctime(&currentTime);
    if (!timeString.empty() && timeString[timeString.length() - 1] == '\n') 
        timeString.erase(timeString.length() - 6); // 2024\n
    for (char& c : timeString) 
    {
        if (c == ' ' || c == ':')
            c = '_';
    }
    std::string fileNameO = fileName;
    if (fileNameO.empty())
        fileNameO = "../txt_file/LogTxt_" + timeString + "_N" + std::to_string(countFile) + ".txt";
    std::ofstream outputFile(fileNameO);
    if (outputFile.is_open()) 
    {
        outputFile << "| create time: |" << timeString << "| \n| ---- | ---- | \n";
        outputFile << context;
        outputFile.close();
        countFile++;
    }
    return;
}

//file write and read
void _wirteTrigonFile(const std::vector<std::array<uint64_t, 2>>& tris, const std::string& fileName);
void _wirteTrigonFile(const std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>>& tris, const std::string& fileName);
std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> _readTrigonFile(const std::string& fileName);
std::vector<std::array<uint64_t, 2>> _readEntityIDFile(const std::string& fileName);

int _wirteNumberFile(size_t n, const std::string& filename);
int _wirteNumberFile(size_t n, double* _array, const std::string& filename); // n = size(triA, triB)
DLLEXPORT_CAL double* _readNumberFile(size_t n, const std::string& filename);

// flatbuffers serialization
void write_InterTriInfo(const std::vector<InterTriInfo>& infos, const std::string& fileName);
void write_ModelMesh(const std::vector<clash::ModelMesh>& meshs, const std::string& fileName);
std::vector<InterTriInfo> read_InterTriInfo(const std::string& fileName);
inline std::vector<clash::ModelMesh> read_ModelMesh(const std::string& fileName) { return {}; }
std::vector<clash::ModelMesh> read_ModelMesh(const std::vector<clash::ModelMesh>& meshVct, const Eigen::Matrix4d& matRela);

// the AlignedBox data
int _writeBinFileAlignedBox(size_t N);

DLLEXPORT_CAL double* _readBinFileAlignedBox(size_t N); //call write inner

inline Eigen::Vector3d createRandVector()
{
    Eigen::Vector3d vec(
        rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,
        rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,
        rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX);
    return vec;
    /*
    int num = rand() % 100; //产生0~99这100个整数中的一个随机整数
    rand() % (b-a+1)+ a ;    //表示  a~b 之间的一个随机整数。
    通常rand()产生的随机数在每次运行的时候都是与上一次相同的，这样是为了便于程序的调试。若要产生每次不同的随机数，则可以使用srand( seed )函数进行产生随机化种子，随着seed的不同，就能够产生不同的随机数。

    rand() 会返回一随机数值，范围在 0 至 RAND_MAX 间。
    rand()产生的是假随机数字，每次执行时是相同的。若要不同,以不同的值来初始化它.初始化的函数就是 srand()。
    */
}

#ifdef USING_CONDITIONAL_COMPILE_PROJECT_2
inline Vec2 _get_rand()
{
    return Vec2(rand(), rand());
}

inline Eigen::Vector3d _to2D(const Eigen::Vector3d& vec3)
{
    return Eigen::Vector3d(vec3.x(), vec3.y(), 0.0);
}

inline std::array<para::BPParaVec, 3> _get_rand3v()
{
    // rand -16384 -> 16384 
    //srand((int)time(0));
    //Sleep(100);
    return std::array<para::BPParaVec, 3> {
            para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}
inline std::array<Eigen::Vector3d, 3> _get_rand3(int i = 0)
{
    if (i == 0)
        return std::array<Eigen::Vector3d, 3> {
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
    if (i == 1)
        return std::array<Eigen::Vector3d, 3> {
            Eigen::Vector3d(rand(), rand(), rand()),
            Eigen::Vector3d(rand(), rand(), rand()),
            Eigen::Vector3d(rand(), rand(), rand()) };
    if (i == -1)
        return std::array<Eigen::Vector3d, 3> {
            Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
            Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
            Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff) };
}

inline std::array<Eigen::Vector3d, 2> _get_rand2()
{
    return std::array<Eigen::Vector3d, 2> {
        Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
        Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}
#endif

//判断是否存在目录，如果不存在则创建目录
//#include <windows.h>
inline bool DirectoryExists(const std::string& dirPath)
{
    DWORD ftyp = GetFileAttributesA(dirPath.c_str());
    if (ftyp == INVALID_FILE_ATTRIBUTES)
        return false;
    return (ftyp & FILE_ATTRIBUTE_DIRECTORY) != 0;
}

//CreateDirectoryIfNotExists
inline void createDirectory(const std::string& dirPath)
{
    if (DirectoryExists(dirPath))
        return;
    if (CreateDirectoryA(dirPath.c_str(), nullptr) || GetLastError() == ERROR_ALREADY_EXISTS)
        std::cout << "Directory created: " << dirPath << std::endl;
}
