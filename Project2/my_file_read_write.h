#pragma once
#include<afx.h>
//inline std::wstring transfer_string_to_wstring(const std::string& str) //ANSIToUnicode
//{
//	int lengthW = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, NULL);
//    wchar_t* pUnicode = new wchar_t[lengthW * sizeof(wchar_t)];
//    memset(pUnicode, 0, lengthW * sizeof(wchar_t));
//    MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, pUnicode, lengthW);
//    std::wstring strw = pUnicode;
//    delete[] pUnicode;
//    return strw;
//}

#ifndef CLASH_DETECTION_SOLUTION
struct InterTriInfo
{
    std::array<std::array<Eigen::Vector3d, 3>, 2> trianglePair;
    std::array<unsigned long long, 2> entityPair;
    double distance;
};
#endif

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
double* _readNumberFile(size_t n, const std::string& filename);

// flatbuffers serialization
void write_InterTriInfo(const std::vector<InterTriInfo>& infos, const std::string& fileName);
void write_ModelMesh(const std::vector<clash::ModelMesh>& meshs, const std::string& fileName);
std::vector<InterTriInfo> read_InterTriInfo(const std::string& fileName);
std::vector<clash::ModelMesh> read_ModelMesh(const std::string& fileName);
std::vector<clash::ModelMesh> read_ModelMesh(const std::vector<clash::ModelMesh>& meshVct, const Eigen::Matrix4d& matRela);

// the AlignedBox data
int _writeBinFileAlignedBox(size_t N);

double* _readBinFileAlignedBox(size_t N); //call write inner

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
