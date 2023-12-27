#pragma once

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
void write_ModelMesh(const std::vector<ModelMesh>& meshs, const std::string& fileName);
std::vector<InterTriInfo> read_InterTriInfo(const std::string& fileName);
std::vector<ModelMesh> read_ModelMesh(const std::string& fileName);

// the AlignedBox data
int _writeBinFileAlignedBox(size_t N);
double* _readBinFileAlignedBox(size_t N); //call write inner

inline std::wstring transfer_string_to_wstring(const std::string& str) //ANSIToUnicode
{
    int lengthW = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, NULL);
    wchar_t* pUnicode = new wchar_t[lengthW * sizeof(wchar_t)];
    memset(pUnicode, 0, lengthW * sizeof(pUnicode));
    MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, pUnicode, lengthW);
    std::wstring strw = pUnicode;
    delete[] pUnicode;
    return strw;
}

inline std::string getExePath() // include<afx.h>
{
    TCHAR buff[MAX_PATH];
    GetModuleFileNameW(NULL, buff, MAX_PATH);
    CString path = buff;
    path = path.Left(path.ReverseFind('\\')); // delete exename
    return (std::string)(CStringA)path;
}
