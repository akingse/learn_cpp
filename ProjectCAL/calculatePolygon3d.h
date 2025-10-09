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

namespace land
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
    struct std::hash<land::Edge>
    {
        size_t operator()(const land::Edge& edge) const
        {
            size_t v1 = std::hash<int>()(edge.v1);
            size_t v2 = std::hash<int>()(edge.v2);
            return v1 ^ (v2 + 0x9e3779b9 + (v1 << 6) + (v1 >> 2));
        }
    };
}

namespace land
{
    using namespace Eigen;
    using namespace eigen;

    //#ifdef STORAGE_VERTEX_DATA_2D
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

    //using relative matrix
    inline Eigen::Matrix4d meshRelativeTransform(clash::ModelMesh& mesh)
    {
        mesh.to2D();
        std::pair<Eigen::Vector2d, Eigen::Vector2d> axis2 = computePrincipalAxes2D(mesh);
        //double dotpro = axis2.first.dot(axis2.second);
        //double size = 2000;
        //_drawSegment(Vector3d(), 2* size*axis2.first);
        //_drawSegment(Vector3d(), 1* size*axis2.second);
        Eigen::Matrix3d mat = toMatrix3d(rotz90() * rotz90() * rotz90()); //avoid tiny float deviation
        if (axis2.first[0] != 0)
            mat = toMatrix3d(rotz(-atan(axis2.first[1] / axis2.first[0])));
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
        Eigen::Matrix4d mat4 = toMatrix4d(mat.inverse()) * translate(bounding.min());
        return mat4;
    }

    //ordered contour points and vertex index
    inline std::pair<std::vector<Vector3d>, std::unordered_set<int>> computeBoundaryContour(
        const std::vector<Vector3d>& mesh_vbo, const std::vector<Vector3i>& mesh_ibo)//const ModelMesh& mesh)
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
    inline std::vector<Vector3i> getInnerMeshByBoundary(const std::vector<Vector3i>& ibo, const std::unordered_set<int>& boundEdge, bool isInner = true)
    {
        std::vector<Vector3i> innMesh; //inner
        //innMesh.vbo_ = mesh.vbo_;
        std::vector<Vector3i> outMesh; //outer
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

    inline bool isContourCCW(const std::vector<Vector3d>& contour)
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

    //first base on max anlge
    inline std::array<std::vector<Eigen::Vector3d>, 4> splitContourToEdge(
            const std::vector<Eigen::Vector3d>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints, bool isFirst = false)
    {
        std::map<double, int> angleMap; //夹角排序，轮廓中有的地方夹角很大
        if (isFirst)
        {
            int n = (int)boundContour.size();
            double minCorner = 1;
            for (int i = 0; i < n; i++)
            {
                int h = (i == 0) ? n - 1 : i - 1;
                int j = (i + 1) % n;
                Eigen::Vector3d veci = boundContour[i] - boundContour[h];
                Eigen::Vector3d vecj = boundContour[j] - boundContour[i];
                double angle = angle_two_vectors(to_vec3(to_vec2(veci)), to_vec3(to_vec2(vecj)));
                if (angle < minCorner)
                    continue;
                angleMap.emplace(angle, i);
            }
            if (angleMap.size() < 4)
                return {};
        }
        int firstpoint = 0;
        double distance = DBL_MAX;
        if (isFirst)
            for (auto iter = angleMap.begin(); iter != angleMap.end(); iter++)
            {
                double temp = (cornerPoints[0] - to_vec2(boundContour[iter->second])).squaredNorm();
                if (temp < distance)
                {
                    distance = temp;
                    firstpoint = iter->second;
                }
            }
        else
            for (int j = 0; j < (int)boundContour.size(); j++)
            {
                double temp = (cornerPoints[0] - to_vec2(boundContour[j])).squaredNorm();
                if (temp < distance)
                {
                    distance = temp;
                    firstpoint = j;
                }
            }
        //std::vector<Eigen::Vector3d> ccwContour = boundContour;
        std::vector<Eigen::Vector3d> ccwContour(boundContour.size());
        bool isCCW = isContourCCW(boundContour);
        int n = (int)boundContour.size();
        for (int i = 0; i < n; i++)
            ccwContour[i] = boundContour[(i + firstpoint) % n];
        //ccwContour[n - 1] = ccwContour[0]; //closed, for reverse
        if (!isCCW)
        {
            ccwContour.push_back(ccwContour.front());
            std::reverse(ccwContour.begin(), ccwContour.end());
            ccwContour.pop_back();
        }
        std::array<int, 4> cornerIndex = { 0,0,0,0 };
        for (int i = 1; i < 4; i++)
        {
            double distance = DBL_MAX;
            if (isFirst)
                for (auto iter = angleMap.begin(); iter != angleMap.end(); iter++)
                {
                    // angleMap record origin index
                    double temp = (cornerPoints[i] - to_vec2(boundContour[iter->second])).squaredNorm();
                    if (temp < distance)
                    {
                        distance = temp;
                        cornerIndex[i] = (iter->second + n - firstpoint) % n; //avoid nega
                        if (!isCCW)
                            cornerIndex[i] = n - cornerIndex[i];
                    }
                }
            else
                for (int j = 0; j < (int)ccwContour.size(); j++)
                {
                    double temp = (cornerPoints[i] - to_vec2(ccwContour[j])).squaredNorm();
                    if (temp < distance)
                    {
                        if (j <= cornerIndex[i - 1])
                            continue;
                        distance = temp;
                        cornerIndex[i] = j;
                    }
                }
        }
        //left - right - down - up
        std::array<std::vector<Eigen::Vector3d>, 4> edgeUV4;
        //int countLess = 0;
        //for (int i = 0; i < 4; i++)
        //{
        //    if (cornerIndex[i] < cornerIndex[(i + 1) % 4])
        //        countLess++;
        //}
        //bool isCCW = countLess == 3;
        for (int i = 0; i < 4; i++)
        {
            int start = cornerIndex[i];
            int end = cornerIndex[(i + 1) % 4];
            std::vector<Eigen::Vector3d> edge;
            if (start < end)
            {
                for (int j = start; j <= end; j++)
                    edge.push_back(ccwContour[j]);
            }
            else
            {
                for (int j = start; j < ccwContour.size(); j++)
                    edge.push_back(ccwContour[j]);
                for (int j = 0; j <= end; j++)
                    edge.push_back(ccwContour[j]);
            }
            if (i == 2 || i == 3)//(!isCCW && (i == 0 || i == 1)))
                std::reverse(edge.begin(), edge.end());
            edgeUV4[i] = edge;
        }
        return edgeUV4;
    }


}



#endif// CALCULATE_POLYGON3D_H
