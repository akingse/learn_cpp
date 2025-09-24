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
    inline Eigen::Matrix4d mesh_relative_transform(clash::ModelMesh& mesh)
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

}



#endif// CALCULATE_POLYGON3D_H
