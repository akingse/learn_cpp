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
#ifdef USING_EIGEN_VERISON
        Eigen::Vector2i m_id;
        Edge(int a, int b)
        {
            //m_id[0] = std::min(a, b);
            //m_id[1] = std::max(a, b);
            m_id = (a < b) ? Eigen::Vector2i(a, b) : Eigen::Vector2i(b, a);
        }
        bool operator==(const Edge& other) const
        {
            return m_id == other.m_id;
        }
        bool operator<(const Edge& other) const
        {
            return memcmp(m_id.data(), other.m_id.data(), 8) == -1; //2*sizeof(int)
        }
        inline int first() const
        {
            return m_id[0];
        }
#else
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
        inline int first() const
        {
            return v1;
        }
#endif
    };
}

namespace std
{
    template<>
    struct std::hash<clash::Edge>
    {
        size_t operator()(const clash::Edge& edge) const
        {
#ifdef USING_EIGEN_VERISON
            size_t v1 = std::hash<int>()(edge.m_id[0]);
            size_t v2 = std::hash<int>()(edge.m_id[1]);
#else
            size_t v1 = std::hash<int>()(edge.v1);
            size_t v2 = std::hash<int>()(edge.v2);
#endif
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
        MACRO_EXPANSION_TIME_DEFINE;
        //compute by ibo index count
        std::unordered_map<Edge, int> edgeCount;
        // count every edge
        MACRO_EXPANSION_TIME_START;
        for (const auto& triangle : mesh_ibo) //cost most time
        {
            //std::array<Edge, 3> edges = {
            //    Edge(triangle[0], triangle[1]),
            //    Edge(triangle[1], triangle[2]),
            //    Edge(triangle[2], triangle[0])
            //};
            //for (const auto& edge : edges)
                //edgeCount[edge]++;
            edgeCount[Edge(triangle[0], triangle[1])]++;
            edgeCount[Edge(triangle[1], triangle[2])]++;
            edgeCount[Edge(triangle[2], triangle[0])]++;
        }
        MACRO_EXPANSION_TIME_END("time_create_edgeCount");//cost most time
        // collect
        std::vector<Edge> boundEdges;
        std::unordered_set<int> uniqueEdge;
        MACRO_EXPANSION_TIME_START;
        for (const auto& pair : edgeCount)
        {
            if (pair.second == 1)
            {
                boundEdges.push_back(pair.first);
#ifdef USING_EIGEN_VERISON
                uniqueEdge.insert(pair.first.m_id[0]);
                uniqueEdge.insert(pair.first.m_id[1]);
#else
                uniqueEdge.insert(pair.first.v1);
                uniqueEdge.insert(pair.first.v2);
#endif
            }
        }
        MACRO_EXPANSION_TIME_END("time_create_uniqueEdge");
        //extractboundContour
        std::unordered_map<int, std::vector<int>> adjacencyList;
        MACRO_EXPANSION_TIME_START;
        for (const auto& edge : boundEdges)
        {
#ifdef USING_EIGEN_VERISON
            adjacencyList[edge.m_id[0]].push_back(edge.m_id[1]);
            adjacencyList[edge.m_id[1]].push_back(edge.m_id[0]);
#else
            adjacencyList[edge.v1].push_back(edge.v2);
            adjacencyList[edge.v2].push_back(edge.v1);
#endif
        }
        MACRO_EXPANSION_TIME_END("time_create_adjacencyList");
        std::vector<Eigen::Vector3d> boundContour;
        int startVertex = boundEdges[0].first(); // choose one random index
        int currentVertex = startVertex;
        int previousVertex = -1;
        //MACRO_EXPANSION_TIME_START;
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
        //MACRO_EXPANSION_TIME_END("time_create_boundContour");//little ignore
        //boundContour.push_back(boundContour.front());//closed
        //std::map<std::string, double>& dataTime = test::DataRecordSingleton::getData().m_dataTime;
        return { boundContour,uniqueEdge };
    }

    //AdvancingFront U version
    inline std::vector<std::pair<Eigen::Vector3d, int>> computeBoundaryContourU(
        const std::vector<Eigen::Vector3d>& mesh_vbo, const std::vector<Eigen::Vector3i>& mesh_ibo)
    {
        MACRO_EXPANSION_TIME_DEFINE;
        //compute by ibo index count
        std::unordered_map<Edge, int> edgeCount;
        // count every edge
        for (const auto& triangle : mesh_ibo) //cost most time
        {
            edgeCount[Edge(triangle[0], triangle[1])]++;
            edgeCount[Edge(triangle[1], triangle[2])]++;
            edgeCount[Edge(triangle[2], triangle[0])]++;
        }
        // collect
        std::vector<Edge> boundEdges;
        //std::unordered_set<int> uniqueEdge;
        MACRO_EXPANSION_TIME_START;
        for (const auto& pair : edgeCount)
        {
            if (pair.second == 1)
            {
                boundEdges.push_back(pair.first);
                //uniqueEdge.insert(pair.first.m_id[0]);
                //uniqueEdge.insert(pair.first.m_id[1]);
            }
        }
        MACRO_EXPANSION_TIME_END("time_create_uniqueEdge");
        //extractboundContour
        std::unordered_map<int, std::vector<int>> adjacencyList;
        MACRO_EXPANSION_TIME_START;
        for (const auto& edge : boundEdges)
        {
#ifdef USING_EIGEN_VERISON
            adjacencyList[edge.m_id[0]].push_back(edge.m_id[1]);
            adjacencyList[edge.m_id[1]].push_back(edge.m_id[0]);
#endif
        }
        MACRO_EXPANSION_TIME_END("time_create_adjacencyList");
        std::vector<std::pair<Eigen::Vector3d, int>> boundContour;//std::vector<Eigen::Vector3d>
        int startVertex = boundEdges[0].first(); // choose one random index
        int currentVertex = startVertex;
        int previousVertex = -1;
        do {
            boundContour.push_back({ mesh_vbo[currentVertex], currentVertex });
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
        return boundContour;
    }

    //    inline std::vector<std::pair<Eigen::Vector3d, int>> computeBoundaryContourU(
    //        const std::vector<Eigen::Vector3d>& mesh_vbo, const std::vector<Eigen::Vector3i>& mesh_ibo)
    //    {
    //        MACRO_EXPANSION_TIME_DEFINE;
    //        std::vector<Edge> boundEdges;
    //        // todo 优化获取最外边效率
    //        //extractboundContour
    //        std::unordered_map<int, std::vector<int>> adjacencyList;
    //        MACRO_EXPANSION_TIME_START;
    //        for (const auto& edge : boundEdges)
    //        {
    //#ifdef USING_EIGEN_VERISON
    //            adjacencyList[edge.m_id[0]].push_back(edge.m_id[1]);
    //            adjacencyList[edge.m_id[1]].push_back(edge.m_id[0]);
    //#endif
    //        }
    //        MACRO_EXPANSION_TIME_END("time_create_adjacencyList");
    //        std::vector<std::pair<Eigen::Vector3d, int>> boundContour;//std::vector<Eigen::Vector3d>
    //        int startVertex = boundEdges[0].first(); // choose one random index
    //        int currentVertex = startVertex;
    //        int previousVertex = -1;
    //        do {
    //            boundContour.push_back({ mesh_vbo[currentVertex], currentVertex });
    //            const std::vector<int>& neighbors = adjacencyList[currentVertex];
    //            for (const int neighbor : neighbors)
    //            {
    //                if (neighbor != previousVertex) {
    //                    previousVertex = currentVertex;
    //                    currentVertex = neighbor;
    //                    break;
    //                }
    //            }
    //            if (boundEdges.size() < boundContour.size())
    //                return {};// break;
    //        } while (currentVertex != startVertex);
    //        return boundContour;
    //    }

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
                outMesh.push_back(face);
            else
                innMesh.push_back(face);
        }
        //_drawPolyface(getPolyfaceHandleFromModelMesh(innMesh));
        //_drawPolyface(getPolyfaceHandleFromModelMesh(outMesh), colorRand());
        return isInner ? innMesh : outMesh; //getBandMeshByBoundary
    }

    //down-right-up-left, positive direction //first base on max anlge
    std::array<std::vector<Eigen::Vector3d>, 4> splitContourToEdge(
        const std::vector<Eigen::Vector3d>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints, bool isFirst = false);
    std::array<std::vector<std::pair<Eigen::Vector3d, int>>, 4> splitContourToEdge(
        const std::vector<std::pair<Eigen::Vector3d, int>>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints, bool isFirst = false);

    //AdvancingFront U version
    std::array<std::vector<std::pair<Eigen::Vector3d, int>>, 2> splitContourToEdgeFirst(
        const std::vector<std::pair<Eigen::Vector3d, int>>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints);

#ifdef USING_BVHTREE_INDEX2
    inline std::vector<Eigen::Vector2d> getIntersectPoint(const std::vector<Eigen::Vector2d>& lineA, const std::vector<std::vector<Eigen::Vector2d>>& linesV, const bvh::BVHTree2d& bvhtree)
    {
        std::vector<Eigen::Vector2d> res;
        for (int i = 0; i < (int)lineA.size() - 1; ++i)
        {
            Eigen::AlignedBox2d box;
            box.extend(lineA[i]);
            box.extend(lineA[i + 1]);
            std::vector<std::pair<int, int>> inters = bvhtree.findIntersect2(box);
            std::array<Eigen::Vector2d, 2> segmA = { lineA[i],lineA[i + 1] };
            for (const auto& j : inters)
            {
                std::array<Eigen::Vector2d, 2> segmB = { linesV[j.first][j.second], linesV[j.first][j.second + 1] };
                if (!isTwoSegmentsIntersect(segmA, segmB))
                    continue;
                Eigen::Vector2d point = eigen::getIntersectPointOfTwoLines(segmA, segmB);
                if (res.empty() || !(point - res.back()).isZero()) //using tolerancce
                    res.push_back(point);
            }
        }
        //if (lineA.size() != res.size() + 2)
        //    return {};//error
        return res;
    }
#endif 

}

namespace clash
{

    template<class T>
    inline std::vector<T> linspace(const T& p0, const T& p1, int n) //noEnd
    {
        std::vector<T> res(n);
        T v = 1.0 / n * (p1 - p0);
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
        test::DataRecordSingleton::dataCountAppend("getDepthZ_nan");
        return std::nan("0");
    }

    inline double getDeviationZ(const ModelMesh& mesh, const std::vector<int>& inters, const Eigen::Vector3d& p)
    {
        for (const int& i : inters)
        {
            Triangle2d trigon2d = { //2D
                mesh.vbo2_[mesh.ibo_[i][0]],
                mesh.vbo2_[mesh.ibo_[i][1]],
                mesh.vbo2_[mesh.ibo_[i][2]] };
            if (!isPointInTriangle(eigen::to_vec2(p), trigon2d))
                continue;
            double deno = mesh.fno_[i][2];// Vector3d(0, 0, 1).dot(mesh.fno_[i]);
            if (deno == 0)
            {
                double interz = 0;
                return interz / 2;
            }
            double k = (mesh.vbo_[mesh.ibo_[i][0]] - p).dot(mesh.fno_[i]) / deno;
            return fabs(k);
        }
        return std::nan("0");
    }

    template<class T>
    std::vector<std::vector<T>> matrix_transpose(const std::vector<std::vector<T>>& cross2D)
    {
        int sizeU = (int)cross2D[0].size();
        int sizeV = (int)cross2D.size();
        std::vector<std::vector<T>> transp(sizeU);
        for (int i = 0; i < sizeU; ++i)
        {
            std::vector<T> temp(sizeV);
            for (int j = 0; j < sizeV; ++j)
                temp[j] = cross2D[j][i];
            transp[i] = temp;
        }
        return transp;
    }

    template<class T>
    std::vector<std::vector<T>> getBoundGridAreaByIndex(const std::vector<std::vector<T>>& cross2D, int interU, int interV, int u, int v)
    {
        int sizeU = (int)cross2D[0].size();
        int sizeV = (int)cross2D.size();
        int iLt = std::round((u + 0) * (sizeU - 1) / double(interU + 1));
        int iRt = std::round((u + 1) * (sizeU - 1) / double(interU + 1));
        int iDn = std::round((v + 0) * (sizeV - 1) / double(interV + 1));
        int iUp = std::round((v + 1) * (sizeV - 1) / double(interV + 1));
        std::vector<std::vector<T>> block(iUp - iDn + 1);
        int io = 0;
        for (int i = iDn; i <= iUp; i++)
        {
            std::vector<T> temp(iRt - iLt + 1);
            int jo = 0;
            for (int j = iLt; j <= iRt; j++)
                temp[jo++] = cross2D[i][j];
            block[io++] = temp;
        }
        return block;
    }

    inline std::vector<Eigen::Vector2d> edgeAlignEqual(const std::vector<Eigen::Vector2d>& origin, int num)
    {
        if (num <= origin.size())
            return origin;
        std::vector<Eigen::Vector2d> res;
        if (2 * origin.size() - 1 <= num)
        {
            res.reserve(2 * origin.size() - 1);
            for (int i = 0; i < origin.size() - 1; ++i)
            {
                res.push_back(origin[i]);
                res.push_back(0.5 * (origin[i + 1] + origin[i]));
            }
            res.push_back(origin.back());
        }
        else
        {
            int add = num - origin.size();
            std::vector<std::pair<double, int>> distVct;
            for (int i = 0; i < origin.size() - 1; ++i)
                distVct.push_back({ (origin[i + 1] - origin[i]).squaredNorm(), i });
            std::sort(distVct.begin(), distVct.end(), []
            (const std::pair<double, int>& dist0, const std::pair<double, int>& dist1)
                {return dist0.first > dist1.first; });
            //int count = 0;
            std::map<int, double> distMap;
            for (const auto& iter : distVct)
            {
                distMap.emplace(iter.second, iter.first);
                if (distMap.size() + origin.size() == num)
                    break;
            }
            res = origin;
            for (auto iter = distMap.rbegin(); iter != distMap.rend(); iter++)
            {
                int i = iter->first;
                Eigen::Vector2d pnt = 0.5 * (origin[i + 1] + origin[i]);
                res.insert(res.begin() + 1 + i, pnt);
                if (res.size() == num)
                    break;
            }
        }
        if (res.size() < num)
            res = edgeAlignEqual(res, num);
        return res;
    }

    inline std::vector<Eigen::Vector2d> edgeDistanceEqual(const std::vector<Eigen::Vector2d>& origin, int num)
    {
        if (num <= origin.size())
            return origin;
        double length = 0;
        for (int i = 0; i < origin.size() - 1; ++i)
            length += (origin[i + 1] - origin[i]).norm();
        length = length / (num - 1);
        std::vector<Eigen::Vector2d> res;
        double current = 0;
        for (int i = 0; i < origin.size() - 1; ++i)
        {
            double dist = (origin[i + 1] - origin[i]).norm();
            current += dist;
            int num = round(current / length) - res.size();
            std::vector<Eigen::Vector2d> temp = linspace(origin[i], origin[i + 1], num);
            res.insert(res.end(), temp.begin(), temp.end());
        }
        res.push_back(origin.back());
        return res;
    }
}

#endif// CALCULATE_POLYGON3D_H
