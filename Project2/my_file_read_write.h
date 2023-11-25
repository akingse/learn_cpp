#pragma once
//ÎÄ¼þ¶ÁÐ´

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

