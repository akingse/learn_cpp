#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2025												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common and simple 3d polygon calculation methods		   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_POLYGON3D_H
#define CALCULATE_POLYGON3D_H

namespace clash
{
    struct Edge
    {
        //Eigen::Vector2i m_id;
        int v1, v2;
        Edge(int a, int b)
        {
            v1 = std::min(a, b);
            v2 = std::max(a, b);
        }
        bool operator==(const Edge& other) const
        {
            return (v1 == other.v1 && v2 == other.v2);
        }
    };
}

namespace std
{
    template<>
    struct std::hash<clash::Edge>
    {
        size_t operator()(const clash::Edge& edge) const
        {
            size_t v1 = std::hash<int>()(edge.v1);
            size_t v2 = std::hash<int>()(edge.v2);
            return v1 ^ (v2 + 0x9e3779b9 + (v1 << 6) + (v1 >> 2));
        }
    };
}

namespace clash
{

    inline std::pair<Eigen::Vector3d, Eigen::Vector3d> computePrincipalAxes3D(const clash::ModelMesh& mesh)
    {
        // 1. 提取顶点
        Eigen::MatrixXd points(mesh.vbo_.size(), 3);
        for (size_t i = 0; i < mesh.vbo_.size(); ++i)
            points.row(i) = mesh.vbo_[i].transpose();
        // 2. 计算均值
        Eigen::Vector3d mean = points.colwise().mean();
        // 3. 中心化数据
        Eigen::MatrixXd centered = points.rowwise() - mean.transpose();
        // 4. 计算协方差矩阵
        Eigen::MatrixXd covariance = (centered.transpose() * centered) / (points.rows() - 1);
        // 5. 特征值分解
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
        if (solver.info() != 0) {
            throw std::runtime_error("Eigen decomposition failed.");
        }
        // 6. 提取主轴和次轴
        Eigen::Vector3d principalAxis = solver.eigenvectors().col(2);  // 最大特征值对应的特征向量
        Eigen::Vector3d secondaryAxis = solver.eigenvectors().col(1);  // 次大特征值对应的特征向量
        return { principalAxis, secondaryAxis };
    }

#ifdef STORAGE_VERTEX_DATA_2D
    inline std::pair<Eigen::Vector2d, Eigen::Vector2d> computePrincipalAxes2D(const clash::ModelMesh& mesh)
    {
        Eigen::MatrixXd points(mesh.vbo2_.size(), 2);
        for (size_t i = 0; i < mesh.vbo2_.size(); ++i)
            points.row(i) = mesh.vbo2_[i].transpose();
        Eigen::Vector2d mean = points.colwise().mean();
        Eigen::MatrixXd centered = points.rowwise() - mean.transpose();
        Eigen::MatrixXd covariance = (centered.transpose() * centered) / (points.rows() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
        if (solver.info() != 0)
            throw std::runtime_error("Eigen decomposition failed.");
        Eigen::MatrixXd res = solver.eigenvectors();
        Eigen::Vector2d principalAxis = solver.eigenvectors().col(1);
        Eigen::Vector2d secondaryAxis = solver.eigenvectors().col(0);
        return { principalAxis, secondaryAxis };
        //return { to_vec3(principalAxis), to_vec3(secondaryAxis) };
    }

    //using relative matrix
    inline Eigen::Matrix4d meshRelativeTransform(clash::ModelMesh& mesh)
    {
        mesh.to2D();
        std::pair<Eigen::Vector2d, Eigen::Vector2d> axis2 = computePrincipalAxes2D(mesh);
        //double dotpro = axis2.first.dot(axis2.second);
        Eigen::Matrix3d mat = eigen::toMatrix3d(eigen::rotz90() * eigen::rotz90() * eigen::rotz90()); //avoid tiny float deviation
        if (axis2.first[0] != 0)
            mat = eigen::toMatrix3d(eigen::rotz(-atan(axis2.first[1] / axis2.first[0])));
        for (auto& iter : mesh.vbo_)
            iter = mat * iter;
        for (auto& iter : mesh.fno_)
            iter = mat * iter;
        Eigen::AlignedBox3d bounding;
        for (const auto& iter : mesh.vbo_)
            bounding.extend(iter);
        for (auto& iter : mesh.vbo_)
            iter = iter - bounding.min();
        //reset
        mesh.to2D();
        Eigen::AlignedBox3d range;
        for (const auto& iter : mesh.vbo_)
            range.extend(iter);
        mesh.bounding_ = range;
        //forward mat
        //mat.transpose();
        Eigen::Matrix4d mat4 = eigen::toMatrix4d(mat.inverse()) * eigen::translate(bounding.min());
        return mat4;
    }
#endif

    //ordered contour points and vertex index
    inline std::pair<std::vector<Eigen::Vector3d>, std::unordered_set<int>> computeBoundaryContour(
        const std::vector<Eigen::Vector3d>& mesh_vbo, const std::vector<Eigen::Vector3i>& mesh_ibo)//const ModelMesh& mesh)
    {
        //compute by ibo index count
        std::unordered_map<Edge, int> edgeCount;
        // count every edge
        for (const auto& triangle : mesh_ibo)
        {
            std::array<Edge, 3> edges = {
                Edge(triangle[0], triangle[1]),
                Edge(triangle[1], triangle[2]),
                Edge(triangle[2], triangle[0])
            };
            for (const auto& edge : edges)
                edgeCount[edge]++;
        }
        // collect
        std::vector<Edge> boundEdges;
        std::unordered_set<int> uniqueEdge;
        for (const auto& pair : edgeCount)
        {
            if (pair.second == 1)
            {
                boundEdges.push_back(pair.first);
                uniqueEdge.insert(pair.first.v1);
                uniqueEdge.insert(pair.first.v2);
            }
        }
        //extractboundContour
        std::unordered_map<int, std::vector<int>> adjacencyList;
        for (const auto& edge : boundEdges)
        {
            adjacencyList[edge.v1].push_back(edge.v2);
            adjacencyList[edge.v2].push_back(edge.v1);
        }
        std::vector<Eigen::Vector3d> boundContour;
        int startVertex = boundEdges[0].v1; // choose one random index
        int currentVertex = startVertex;
        int previousVertex = -1;
        //std::vector<int> check;
        //std::set<int> checkset;
        do {
            boundContour.push_back(mesh_vbo[currentVertex]);
            const std::vector<int>& neighbors = adjacencyList[currentVertex];
            for (const int neighbor : neighbors)
            {
                if (neighbor != previousVertex) {
                    previousVertex = currentVertex;
                    currentVertex = neighbor;
                    break;
                }
            }
            if (boundEdges.size() < boundContour.size())
                return {};// break;
        } while (currentVertex != startVertex);
        //boundContour.push_back(boundContour.front());//closed
        return { boundContour,uniqueEdge };
    }

    //filter ibo, distinguish inner and outer
    inline std::vector<Eigen::Vector3i> getRingMeshByBoundary(const std::vector<Eigen::Vector3i>& ibo, const std::unordered_set<int>& boundEdge, bool isInner = true)
    {
        std::vector<Eigen::Vector3i> innMesh; //inner
        //innMesh.vbo_ = mesh.vbo_;
        std::vector<Eigen::Vector3i> outMesh; //outer
        //outMesh.vbo_ = mesh.vbo_;
        std::unordered_set<int> outContour;
        std::unordered_set<Edge> outEdges;
        for (const auto& face : ibo)
        {
            bool isOut = false;
            for (const auto& i : face)
            {
                //if (std::find(boundEdges.begin(), boundEdges.end(), edge) != boundEdges.end())
                if (boundEdge.find(i) != boundEdge.end())
                {
                    isOut = true;
                    break;
                }
            }
            if (isOut)
            {
                outMesh.push_back(face);
            }
            else
            {
                innMesh.push_back(face);
            }
        }
        //_drawPolyface(getPolyfaceHandleFromModelMesh(innMesh));
        //_drawPolyface(getPolyfaceHandleFromModelMesh(outMesh), colorRand());
        return isInner ? innMesh : outMesh;
    }

    inline bool isContourCCW(const std::vector<Eigen::Vector3d>& contour)
    {
        const size_t n = contour.size();
        if (n < 3)
            return true;
        double area = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const size_t j = (i + 1) % n;
            const Eigen::Vector3d& p1 = contour[i];
            const Eigen::Vector3d& p2 = contour[j];
            area += (p2.x() - p1.x()) * (p2.y() + p1.y());
        }
        return area < 0;
    }

    //down-right-up-left, positive direction //first base on max anlge
    std::array<std::vector<Eigen::Vector3d>, 4> splitContourToEdge(
        const std::vector<Eigen::Vector3d>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints, bool isFirst = false);

    inline Eigen::Vector2d getIntersectPoint(const std::vector<Eigen::Vector2d>& lineA, const std::vector<Eigen::Vector2d>& lineB)
    {
        for (int i = 0; i < lineA.size() - 1; ++i)
        {
            std::array<Eigen::Vector2d, 2> segmA = { lineA[i],lineA[i + 1] };
            for (int j = 0; j < lineB.size() - 1; ++j)
            {
                std::array<Eigen::Vector2d, 2> segmB = { lineB[j],lineB[j + 1] };
                if (!isTwoSegmentsIntersect(segmA, segmB))
                    continue;
                Eigen::Vector2d point = eigen::getIntersectPointOfTwoLines(segmA, segmB);
                return point;
            }
        }
        return Eigen::Vector2d(std::nan("0"), std::nan("0"));
    }

    inline std::vector<Eigen::Vector2d> linspace(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, int n)
    {
        std::vector<Eigen::Vector2d> res(n);
        Eigen::Vector2d v = 1.0 / n * (p1 - p0);
        for (int i = 0; i < n; ++i)
            res[i] = p0 + double(i) * v;
        return res;
    }

    //double getDepthZ(const ModelMesh& mesh, const Vector2d& p)
    inline double getDepthZ(const ModelMesh& mesh, const std::vector<int>& inters, const Eigen::Vector2d& p)
    {
        for (const int& i : inters)//(int i = 0; i < mesh.ibo_.size(); ++i)
        {
            Triangle2d trigon2d = { //2D
                mesh.vbo2_[mesh.ibo_[i][0]],
                mesh.vbo2_[mesh.ibo_[i][1]],
                mesh.vbo2_[mesh.ibo_[i][2]] };
            if (!isPointInTriangle(p, trigon2d))
                continue;
            double deno = mesh.fno_[i][2];// Vector3d(0, 0, 1).dot(mesh.fno_[i]);
            if (deno == 0)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (0 < (p - trigon2d[j]).dot(p - trigon2d[(j + 1) % 3]))
                        continue;
                    Eigen::Vector3d dir = mesh.vbo_[mesh.ibo_[i][(j + 1) % 3]] - mesh.vbo_[mesh.ibo_[i][j]];
                    Eigen::Vector3d normal = Eigen::Vector3d(0, 0, 1).cross(dir); //Vector3d(-dir[2], dir[0], 0)
                    if (normal.isZero())
                        return mesh.vbo_[mesh.ibo_[i][j]].z();
                    double k = (mesh.vbo_[mesh.ibo_[i][j]] - eigen::to_vec3(p)).cross(dir).dot(normal) / (normal.dot(normal));
                    return k;
                }
            }
            double k = (mesh.vbo_[mesh.ibo_[i][0]] - eigen::to_vec3(p)).dot(mesh.fno_[i]) / deno;
            return k;
        }
        return std::nan("0");
    }


}



#endif// CALCULATE_POLYGON3D_H
