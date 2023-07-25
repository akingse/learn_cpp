#pragma once
#include <afx.h>
inline std::string getExePath() // include<afx.h>
{
	TCHAR buff[MAX_PATH];
	GetModuleFileNameW(NULL, buff, MAX_PATH);
	CString path = buff;
	path = path.Left(path.ReverseFind('\\')); // delete exename
	return (CStringA)path;
}

static std::string randNumName = "random_1e8.bin";
namespace std
{
	//wirte randnum file
	inline int _wirteNumberFile(size_t n)
	{
		n = 2 * 3 * 3 * n;
		double* arr = new double[n];
		for (int i = 0; i < n; ++i)
		{
			arr[i] = static_cast<double>(rand() - 0x3fff);// rand()) / RAND_MAX;
		}
		// 写入文件
		ofstream out(randNumName, ios::out | ios::binary);
		if (!out.is_open()) {
			cerr << "Error opening file" << endl;
			return -1;
		}
		out.write(reinterpret_cast<char*>(&n), sizeof(int)); //
		out.write(reinterpret_cast<char*>(arr), n * sizeof(double));
		out.close();
		return 0;
	}

	inline double* _readNumberFile(size_t n)
	{
		ifstream in(randNumName, ios::in | ios::binary);
		if (!in.is_open()) {
			//cerr << "Error opening file" << endl;
			//return nullptr;
			_wirteNumberFile(n);
			in = ifstream(randNumName, ios::in | ios::binary);
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


}

#ifndef CLASH_DETECTION_SOLUTION
struct InterTriInfo
{
	std::array<std::array<Eigen::Vector3d, 3>, 2> trianglePair;
	std::array<unsigned long long, 2> entityPair;
	double distance;
};

struct ModelMesh
{
	std::vector<Eigen::Vector3d> vbo_;
	std::vector<std::array<int, 3>> ibo_;
	Eigen::AlignedBox3d bounding_;
	Eigen::Affine3d pose_; // Eigen::Affine3d::Identity()
	std::vector<int> iboRaw_;
};
#endif

#define USING_FLATBUFFERS_SERIALIZATION

#ifdef USING_FLATBUFFERS_SERIALIZATION 
#include "C:/Users/Aking/source/repos/bimbase/Include/fbs/inter_triangels_info_generated.h"
inline std::vector<InterTriInfo> _read_GetTriList(const std::string& fileName)
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
inline std::vector<ModelMesh> _read_ModelMesh(const std::string& fileName)
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
			for (int i=0;i< vbo->vbos()->size();i++)
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
			res.push_back(ModelMesh{ vbo_ ,ibo_ , Eigen::AlignedBox3d(_min, _max), Eigen::Affine3d::Identity(),_iboRaw });
		}
		inFile.close();
	}
	return res;
}

#endif
