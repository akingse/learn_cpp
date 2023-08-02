#include "pch.h"
#include "test_file_rw.h"
using namespace std;
using namespace psykronix;
#undef min
#undef max
static std::string randNumName = "bin_file/random_1e8.bin";
static std::string randNumNameSepa = "bin_file/random_1e8_sepa.bin";
//wirte randnum file
int _wirteNumberFile(size_t n, const string& filename)
{
	n = 2 * 3 * 3 * n;
	double* arr = new double[n];
	for (int i = 0; i < n; ++i)
	{
		arr[i] = static_cast<double>(rand() - 0x3fff);// rand()) / RAND_MAX;
	}
	// 写入文件
	ofstream out(filename, ios::out | ios::binary);
	if (!out.is_open()) {
		cerr << "Error opening file" << endl;
		return -1;
	}
	out.write(reinterpret_cast<char*>(&n), sizeof(int)); //
	out.write(reinterpret_cast<char*>(arr), n * sizeof(double));
	out.close();
	return 0;
}

int _wirteNumberFile(const std::vector<double>& _array, const string& filename)
{
	ofstream out(filename, ios::out | ios::binary);
	if (!out.is_open()) {
		cerr << "Error opening file" << endl;
		return -1;
	}
	int n = _array.size();
	out.write(reinterpret_cast<char*>(&n), sizeof(int)); //
	out.write(reinterpret_cast<const char*>(_array.data()), n * sizeof(double));
	out.close();
	return 0;
}


int _wirteNumberFile(size_t n, double* _array, const string& filename) // n = size(triA, triB)
{
	n = 2 * 3 * 3 * n;
	//double* arr = new double[n];
	//for (int i = 0; i < n; ++i)
	//{
	//	arr[i] = static_cast<double>(_array[i]);
	//}
	ofstream out(filename, ios::out | ios::binary);
	if (!out.is_open()) {
		cerr << "Error opening file" << endl;
		return -1;
	}
	out.write(reinterpret_cast<char*>(&n), sizeof(int)); //
	out.write(reinterpret_cast<char*>(_array), n * sizeof(double));
	out.close();
	return 0;
}

double* _readNumberFile(size_t n, const string& filename)
{
	ifstream in(filename, ios::in | ios::binary);
	if (!in.is_open()) {
		//cerr << "Error opening file" << endl;
		//return nullptr;
		_wirteNumberFile(n, filename);
		in = ifstream(filename, ios::in | ios::binary);
		if (!in.is_open())
			return nullptr;
	}
	n = 2 * 3 * 3 * n;
	int read_n;
	in.read(reinterpret_cast<char*>(&read_n), sizeof(int));
	if (read_n != n) {
		cerr << "Incorrect data size read from the input file" << endl;
		return nullptr;
	}
	double* read_arr = new double[read_n];
	in.read(reinterpret_cast<char*>(read_arr), read_n * sizeof(double));
	return read_arr;
}

void _wirteTrigonFile(const std::vector<std::array<uint64_t, 2>>& tris, const std::string& fileName)
{
	//std::string fileName = "testTrisData.bin";
	size_t n = tris.size() * 2;
	uint64_t* arrayD = new uint64_t[n];
	for (size_t i = 0; i < tris.size(); ++i)
	{
		for (size_t j = 0; j < 2; ++j) // trigon pair
		{
			arrayD[i * 2 + j] = tris[i][j];
		}
	}
	ofstream out(fileName, ios::out | ios::binary);
	if (!out.is_open())
		cerr << "Error opening file" << endl;
	size_t num = tris.size();
	out.write(reinterpret_cast<char*>(&num), sizeof(size_t)); //
	out.write(reinterpret_cast<char*>(arrayD), n * sizeof(uint64_t));
	out.close();
	delete[] arrayD;
}

void _wirteTrigonFile(const std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>>& tris, const std::string& fileName)
{
	//std::string fileName = "testTrisData.bin";
	size_t n = tris.size() * 2 * 3 * 3;
	double* arrayD = new double[n];
	size_t count = 0;
	for (size_t i = 0; i < tris.size(); ++i)
	{
		for (size_t j = 0; j < 2; ++j) // trigon pair
		{
			for (size_t t = 0; t < 3; ++t) //trigon vertex
			{
				arrayD[i * 18 + j * 9 + t * 3 + 0] = tris[i][j][t].x();
				arrayD[i * 18 + j * 9 + t * 3 + 1] = tris[i][j][t].y();
				arrayD[i * 18 + j * 9 + t * 3 + 2] = tris[i][j][t].z();
				count += 3;
			}
		}
	}
	ofstream out(fileName, ios::out | ios::binary);
	if (!out.is_open())
		cerr << "Error opening file" << endl;
	size_t num = tris.size();
	out.write(reinterpret_cast<char*>(&num), sizeof(size_t)); //
	out.write(reinterpret_cast<char*>(arrayD), n * sizeof(double));
	out.close();
	delete[] arrayD;

}

// manual serial
std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> _readTrigonFile(const std::string& fileName)
{
	//std::string fileName = "testTrisData.bin";
	ifstream in(fileName, ios::in | ios::binary);
	if (!in.is_open())
		return {};
	int n;
	in.read(reinterpret_cast<char*>(&n), sizeof(size_t));
	double* arrayD = new double[n * 2 * 3 * 3];
	in.read(reinterpret_cast<char*>(arrayD), n * 2 * 3 * 3 * sizeof(double));
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris;
	for (size_t i = 0; i < n; ++i)
	{
		std::array<std::array<Eigen::Vector3d, 3>, 2> triPair;
		for (size_t j = 0; j < 2; ++j) // trigon pair
		{
			array<Eigen::Vector3d, 3> trigon;
			for (size_t t = 0; t < 3; ++t) //trigon vertex
			{
				Eigen::Vector3d point = {
					arrayD[i * 18 + j * 9 + t * 3 + 0],
					arrayD[i * 18 + j * 9 + t * 3 + 1],
					arrayD[i * 18 + j * 9 + t * 3 + 2] };
				trigon[t] = point;
			}
			triPair[j] = trigon;
		}
		tris.push_back(triPair);
	}
	delete[] arrayD;
	return tris;
}

std::vector<std::array<uint64_t, 2>> _readEntityIDFile(const std::string& fileName)
{
	ifstream in(fileName, ios::in | ios::binary);
	if (!in.is_open())
		return {};
	int n;
	in.read(reinterpret_cast<char*>(&n), sizeof(size_t));
	uint64_t* arrayD = new uint64_t[n * 2];
	in.read(reinterpret_cast<char*>(arrayD), n * 2 * sizeof(uint64_t));
	std::vector<std::array<uint64_t, 2>> tris;

	for (size_t i = 0; i < n; ++i)
	{
		tris.push_back({ arrayD[2 * i] ,arrayD[2 * i + 1] });
	}
	delete[] arrayD;
	return tris;
}

#define USING_FLATBUFFERS_SERIALIZATION
#ifdef USING_FLATBUFFERS_SERIALIZATION 
#include "flatbuffers/flatbuffers.h"
using namespace flatbuffers;

#include "C:/Users/Aking/source/repos/bimbase/Include/fbs/inter_triangels_info_generated.h"
std::vector<InterTriInfo> read_InterTriInfo(const std::string& fileName)
{
	//ifstream inFile(path + "interTriInfo_5108.bin", ios::in | ios::binary);
	std::ifstream inFile(fileName, std::ios::in | std::ios::binary);
	std::vector<InterTriInfo> res;
	if (inFile.is_open())
	{
		inFile.seekg(0, std::ios::end);
		std::streampos fileSize = inFile.tellg();
		inFile.seekg(0, std::ios::beg);
		std::vector<uint8_t> bufferPointer(fileSize);
		if (!inFile.read(reinterpret_cast<char*>(bufferPointer.data()), fileSize))
			return {};
		const TriList* triList = GetTriList(bufferPointer.data());// 反序列化 TriList 对象
		size_t n = triList->tri_infoes()->size();
		for (size_t i = 0; i < n; ++i)
		{
			const TriInfo* triinfo = triList->tri_infoes()->Get(i);
			auto tria = triinfo->tri_a();
			auto trib = triinfo->tri_b();
			typedef std::array<Eigen::Vector3d, 3> Triangle;
			Triangle triA = { Eigen::Vector3d(tria->p0()->x(),tria->p0()->y(),tria->p0()->z()),
				Eigen::Vector3d(tria->p1()->x(),tria->p1()->y(),tria->p1()->z()),
				Eigen::Vector3d(tria->p2()->x(),tria->p2()->y(),tria->p2()->z()) };
			Triangle triB = { Eigen::Vector3d(trib->p0()->x(),trib->p0()->y(),trib->p0()->z()),
				Eigen::Vector3d(trib->p1()->x(),trib->p1()->y(),trib->p1()->z()),
				Eigen::Vector3d(trib->p2()->x(),trib->p2()->y(),trib->p2()->z()) };
			uint64_t entityA = triinfo->entity_a();
			uint64_t entityB = triinfo->entity_b();
			double distance = triinfo->distance(); //max=9.3132257461547852e-10
			res.push_back(InterTriInfo{ std::array<std::array<Eigen::Vector3d, 3>, 2>{ triA, triB},
				std::array<unsigned long long, 2>{ entityA ,entityB }, distance });
		}
		inFile.close();
	}
	return res;
}

#include "C:/Users/Aking/source/repos/bimbase/Include/fbs/convert_to_mesh_generated.h"
std::vector<ModelMesh> read_ModelMesh(const std::string& fileName)
{
	std::ifstream inFile(fileName, std::ios::in | std::ios::binary);
	std::vector<ModelMesh> res;
	if (inFile.is_open())
	{
		inFile.seekg(0, std::ios::end);
		std::streampos fileSize = inFile.tellg();
		inFile.seekg(0, std::ios::beg);
		std::vector<uint8_t> bufferPointer(fileSize);
		if (!inFile.read(reinterpret_cast<char*>(bufferPointer.data()), fileSize))
			return {};
		// deseria
		const  MeshVct* meshVct = GetMeshVct(bufferPointer.data());
		size_t n = meshVct->meshs()->size();
		for (size_t i = 0; i < n; ++i)
		{
			const ConvertToMesh* mesh = meshVct->meshs()->Get(i);
			auto vbo = mesh->vbo();
			auto ibo = mesh->ibo();
			auto _min = Eigen::Vector3d(mesh->aabb_min()->x(), mesh->aabb_min()->y(), mesh->aabb_min()->z());
			auto _max = Eigen::Vector3d(mesh->aabb_max()->x(), mesh->aabb_max()->y(), mesh->aabb_max()->z());
			std::vector<Eigen::Vector3d> vbo_;
			std::vector<std::array<int, 3>> ibo_;
			for (int i = 0; i < vbo->vbos()->size(); i++)
			{
				auto pt = vbo->vbos()->Get(i);
				vbo_.push_back(Eigen::Vector3d(pt->x(), pt->y(), pt->z()));
			}
			for (int i = 0; i < ibo->ibos()->size(); i++)
			{
				auto id = ibo->ibos()->Get(i);
				ibo_.push_back(std::array<int, 3>{id->id0(), id->id1(), id->id2()});
			}
			std::vector<int> _iboRaw;
			auto iboRaw = mesh->ibo_raw();
			for (int i = 0; i < iboRaw->size(); i++)
			{
				_iboRaw.push_back(iboRaw->Get(i));
			}
			res.push_back(ModelMesh{ vbo_, ibo_, Eigen::AlignedBox3d(_min, _max), Eigen::Affine3d::Identity(),false, _iboRaw });
		}
		inFile.close();
	}
	return res;
}

// using flatbuffers serialization
void write_InterTriInfo(const std::vector<InterTriInfo>& infos, const std::string& fileName)
{
	//flat buffer
	flatbuffers::FlatBufferBuilder builder;
	std::vector<flatbuffers::Offset<TriInfo>> triInfos;
	for (const auto& iter : infos)
	{
		auto p0_A = CreatePoint3D(builder, iter.trianglePair[0][0].x(), iter.trianglePair[0][0].y(), iter.trianglePair[0][0].z());
		auto p1_A = CreatePoint3D(builder, iter.trianglePair[0][1].x(), iter.trianglePair[0][1].y(), iter.trianglePair[0][1].z());
		auto p2_A = CreatePoint3D(builder, iter.trianglePair[0][2].x(), iter.trianglePair[0][2].y(), iter.trianglePair[0][2].z());
		auto triangleA = CreateTriangular(builder, p0_A, p1_A, p2_A);
		auto p0_B = CreatePoint3D(builder, iter.trianglePair[1][0].x(), iter.trianglePair[1][0].y(), iter.trianglePair[1][0].z());
		auto p1_B = CreatePoint3D(builder, iter.trianglePair[1][1].x(), iter.trianglePair[1][1].y(), iter.trianglePair[1][1].z());
		auto p2_B = CreatePoint3D(builder, iter.trianglePair[1][2].x(), iter.trianglePair[1][2].y(), iter.trianglePair[1][2].z());
		auto triangleB = CreateTriangular(builder, p0_B, p1_B, p2_B);
		auto triInfo = CreateTriInfo(builder, triangleA, triangleB, iter.entityPair[0], iter.entityPair[1], iter.distance);
		triInfos.push_back(triInfo);
	}
	auto triInfosVector = builder.CreateVector(triInfos); // 创建 TriInfo 向量
	auto triList = CreateTriList(builder, triInfosVector); //创建 TriList 表对象并填充字段
	builder.Finish(triList); //结束构建并生成 FlatBuffer
	// 获取序列化后的数据缓冲区指针和大小
	uint8_t* bufferPointer = builder.GetBufferPointer();
	size_t bufferSize = builder.GetSize();
	ofstream out(fileName, ios::out | ios::binary);
	if (out.is_open())
	{
		out.write(reinterpret_cast<char*>(bufferPointer), bufferSize);
		out.close();
		builder.Clear();
	}
}

void write_ModelMesh(const std::vector<ModelMesh>& meshs, const std::string& fileName)
{
	flatbuffers::FlatBufferBuilder builder;
	std::vector<flatbuffers::Offset<ConvertToMesh>> meshVct;
	for (const auto& iter : meshs)
	{
		std::vector<flatbuffers::Offset<Point3D>> vbos;
		for (auto& iterV : iter.vbo_)
		{
			auto vbo = CreatePoint3D(builder, iterV.x(), iterV.y(), iterV.z());
			vbos.push_back(vbo);
		}
		auto _vbosVct = builder.CreateVector(vbos); //create fbs::Offset<fbs::Vector<fbs::Offset<T>>>
		auto _vbos = CreateVBO(builder, _vbosVct);
		std::vector<flatbuffers::Offset<IndexVt>> ibos;
		for (auto& iterI : iter.ibo_)
		{
			auto ibo = CreateIndexVt(builder, iterI[0], iterI[1], iterI[2]);
			ibos.push_back(ibo);
		}
		auto _ibosVct = builder.CreateVector(ibos); //create fbs::Offset<fbs::Vector<fbs::Offset<T>>>
		auto _ibos = CreateIBO(builder, _ibosVct);
		auto _ibos_raw = builder.CreateVector(iter.iboRaw_); //origin type
		auto _min = CreatePoint3D(builder, iter.bounding_.min().x(), iter.bounding_.min().y(), iter.bounding_.min().z());
		auto _max = CreatePoint3D(builder, iter.bounding_.max().x(), iter.bounding_.max().y(), iter.bounding_.max().z());
		auto mesh = CreateConvertToMesh(builder, _vbos, _ibos, _ibos_raw, _min, _max);
		meshVct.push_back(mesh);
	}
	auto _meshVct = builder.CreateVector(meshVct);
	auto triList = CreateMeshVct(builder, _meshVct);
	builder.Finish(triList);
	// get the pointer and size
	uint8_t* bufferPointer = builder.GetBufferPointer();
	size_t bufferSize = builder.GetSize();
	ofstream out(fileName, ios::out | ios::binary);
	if (out.is_open())
	{
		out.write(reinterpret_cast<char*>(bufferPointer), bufferSize);
		out.close();
		builder.Clear();
	}
}

#endif //USING_FLATBUFFERS_SERIALIZATION
