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
    int index = -1;
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
            //if (!mesh.vbo_.empty())// || !mesh.ibo_.empty()) 
            if (index != -1)
            {
                mesh.index_ = index;
                meshVct.push_back(mesh);
                mesh = ModelMesh(); //reset clear
            }
            index++;
        }
    }
    if (index != -1)//(!mesh.vbo_.empty())// || !mesh.ibo_.empty())
    {
        mesh.index_ = index;
        meshVct.push_back(mesh);
    }
    ifs.close();
    return meshVct;
}

std::vector<int> ModelMesh::selfIntersectCheck() const
{
    auto _findDuplicate = [](const std::vector<int>& nums)->int
        {
            std::unordered_set<int> seen;
            for (const int num : nums)
            {
                if (seen.find(num) != seen.end())
                    return num;
                seen.insert(num);
            }
            return 0; //error
        };

    std::vector<int> res;
    for (int i = 0; i < (int)ibos_.size(); ++i)
    {
        std::set<int> unique;
        for (const int& j : ibos_[i])
            unique.insert(j);
        int differ = ibos_[i].size() - unique.size();
        if (differ == 0)
            continue;
        if (unique.size() < ibos_[i].size())
            res.push_back(i);
    }
    return res;
}

std::vector<int> ModelMesh::selfIntersectRepair() 
{
    auto _findDuplicate = [](const std::vector<int>& nums)->int
        {
            std::unordered_set<int> seen;
            for (const int num : nums)
            {
                if (seen.find(num) != seen.end())
                    return num;
                seen.insert(num);
            }
            return 0; //error
        };

    std::vector<int> res;
    for (int i = 0; i < (int)ibos_.size(); ++i)
    {
        std::set<int> unique;
        for (const int& j : ibos_[i])
            unique.insert(j);
        int differ = ibos_[i].size() - unique.size();
        if (differ == 0)
            continue;
        else if (differ == 1)
        {
            int com = _findDuplicate(ibos_[i]);
            std::vector<int> poly0;
            std::vector<int> poly1;
            bool just = true;
            for (const int& j : ibos_[i])
            {
                (just) ? poly0.push_back(j) : poly1.push_back(j);
                if (j == com)
                    just = !just;
            }
            ibos_[i] = (poly0.size() < poly1.size()) ? poly1 : poly0;
            test::DataRecordSingleton::dataCountAppend("count_split_poly");
            std::vector<int> inner = (poly0.size() < poly1.size()) ? poly0 : poly1;
            int sumI = 0;
            int mulI = 1;
            for (const int& k : inner)
            {
                sumI += k;
                mulI *= k;
            }
            for (int j = 0; j < (int)ibos_.size(); ++j)
            {
                if (ibos_[j].size() != inner.size())
                    continue;
                int sumJ = 0;
                int mulJ = 1;
                for (const auto& k : ibos_[j])
                {
                    sumJ += k;
                    mulJ *= k;
                }
                if (sumI != sumJ || mulI != mulJ)
                    continue;
                ibos_[j].clear();
                test::DataRecordSingleton::dataCountAppend("count_removeTriangle");
            }
            //continue;
        }
        if (unique.size() < ibos_[i].size())
            res.push_back(i);
    }
    return res;
}
