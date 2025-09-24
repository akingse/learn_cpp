#include "pch.h"
using namespace std;
using namespace eigen;
using namespace clash;

namespace clash
{
    enum class RelationOfTwoTriangles : int //two intersect triangle
    {
        COPLANAR = 0,   //intersect or separate
        CONTACT,        //intersect but depth is zero
        INTRUSIVE,      //sat intersect all
        //PARALLEL,       //but not coplanr, must separate
        //SEPARATE,
    };

    enum class RelationOfTrigon : int
    {
        SEPARATE = 0,
        INTERSECT, //intersect point local one or two trigon
        COPLANAR_AINB, //A_INSIDE_B
        COPLANAR_BINA, //B_INSIDE_A
        COPLANAR_INTERSECT,
    };

    enum class RelationOfRayAndTrigon : int
    {
        CROSS_OUTER = 0,
        CROSS_INNER,
        CROSS_VERTEX_0,
        CROSS_VERTEX_1,
        CROSS_VERTEX_2,
        CROSS_EDGE_01,
        CROSS_EDGE_12,
        CROSS_EDGE_20,
        COIN_EDGE_01, //collinear
        COIN_EDGE_12,
        COIN_EDGE_20,
    };

}


TriMesh clash::operator*(const Eigen::Matrix4d& mat, const TriMesh& mesh)
{
    TriMesh res = mesh;
    Eigen::AlignedBox3d box; //re calculate
    for (int i = 0; i < (int)mesh.vbo_.size(); ++i)
    {
        res.vbo_[i] = (mat * mesh.vbo_[i].homogeneous()).hnormalized();
        box.extend(res.vbo_[i]);
    }
    Eigen::Matrix3d mat3 = mat.block<3, 3>(0, 0);
    for (int i = 0; i < (int)mesh.fno_.size(); ++i)
        res.fno_[i] = mat3 * mesh.fno_[i];
    res.bounding_ = box;
    return res;
}

ModelMesh clash::operator*(const Eigen::Matrix4d& mat, const ModelMesh& mesh)
{
    ModelMesh res = mesh;
    Eigen::AlignedBox3d box; //re calculate
    for (int i = 0; i < (int)mesh.vbo_.size(); ++i)
    {
        res.vbo_[i] = (mat * mesh.vbo_[i].homogeneous()).hnormalized();
        box.extend(res.vbo_[i]);
    }
    Eigen::Matrix3d mat3 = mat.block<3, 3>(0, 0);
    for (int i = 0; i < (int)mesh.fno_.size(); ++i)
        res.fno_[i] = mat3 * mesh.fno_[i];
    res.bounding_ = box;
    return res;
}


bool ModelMesh::isEqualMesh(const ModelMesh& meshA, const ModelMesh& meshB)
{
    if (meshA.ibo_.size() != meshB.ibo_.size() || meshA.vbo_.size() != meshB.vbo_.size())
        return false;
    if (!meshA.bounding_.isApprox(meshB.bounding_))
        return false;
    if (meshA.ibo_.empty() || meshA.vbo_.empty() || meshB.ibo_.empty() || meshB.vbo_.empty())
        return false;
    if (meshA.ibo_.front() != meshB.ibo_.front())
        return false;
    if (!meshA.vbo_.front().isApprox(meshB.vbo_.front()))
        return false;
    constexpr double toleFixed = 1e-8;
    std::pair<double, Eigen::Vector3d> vmA = meshA.volume_moment();
    std::pair<double, Eigen::Vector3d> vmB = meshB.volume_moment();
    if (!vmA.second.isApprox(vmB.second))
        return false;
    if (toleFixed < fabs(vmA.first - vmB.first))
        return false;
    if (toleFixed < fabs(meshA.area() - meshB.area()))
        return false;
    return true;
}

bool ModelMesh::writeToFile(const std::vector<ModelMesh>& meshs, const std::string& _filename /*= {}*/) //obj format
{
    std::string filename = _filename;
    if (filename.empty())
    {
        filename = clash::get_exe_path();//clashInterfaceUtility
        filename += "/OutputObj/modelmesh_" + std::to_string(GetTickCount64()) + ".obj";
    }
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return false;
    }
    //for (const auto& mesh : meshs)
    for (int i = 0; i < meshs.size(); i++)
    {
        const ModelMesh& mesh = meshs[i];
        ofs << "# Mesh_" << std::to_string(i) << "\n";
        ofs << "g mesh" << std::to_string(i) << "\n";
        for (const auto& vertex : mesh.vbo_) {
            ofs << "v " << std::setprecision(16) << vertex.x() << " "
                << std::setprecision(16) << vertex.y() << " "
                << std::setprecision(16) << vertex.z() << "\n";
        }
        for (const auto& face : mesh.ibo_) {
            ofs << "f " << (face(0) + 1) << " " << (face(1) + 1) << " " << (face(2) + 1) << "\n";
        }
        ofs << "\n";
    }
    ofs.close();
    return true;
}

std::vector<ModelMesh> ModelMesh::readFromFile(const std::string& filename) //obj format
{
    std::vector<ModelMesh> meshVct;
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return meshVct;
    }
    std::string line;
    ModelMesh mesh;
    while (std::getline(ifs, line))
    {
        std::istringstream is(line);
        std::string prefix;
        is >> prefix;
        if (prefix == "v") {
            Eigen::Vector3d vertex;
            is >> vertex[0] >> vertex[1] >> vertex[2];
            mesh.vbo_.push_back(vertex);
        }
        else if (prefix == "f") {
            Eigen::Vector3i face;
            int index;
            for (int i = 0; i < 3; ++i) {
                is >> index;
                face[i] = index - 1; // OBJ index begin 1
            }
            mesh.ibo_.push_back(face);
        }
        else if (prefix == "g" || prefix == "o") {
            if (!mesh.vbo_.empty())// || !mesh.ibo_.empty()) 
            {
                meshVct.push_back(mesh);
                mesh = ModelMesh(); //reset clear
            }
        }
    }
    if (!mesh.vbo_.empty())// || !mesh.ibo_.empty())
        meshVct.push_back(mesh);
    ifs.close();
    return meshVct;
}
