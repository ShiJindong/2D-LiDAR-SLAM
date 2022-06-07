#include "gaussian_newton.h"
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SparseCholesky>          // Eigen稀疏矩阵求解库SparseCholesky
// # include<eigen3/Eigen/Sparse>               // Eigen整个稀疏矩阵求解库

#include <iostream>


//进行角度正则化．
double GN_NormalizationAngle(double angle)
{
    // 使角度界于-pi~pi之间
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

//位姿-->转换矩阵
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
{
    Eigen::Matrix3d trans;
    trans << cos(x(2)),-sin(x(2)),x(0),
             sin(x(2)), cos(x(2)),x(1),
                     0,         0,    1;

    return trans;
}
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0,2);
    pose(1) = trans(1,2);
    pose(2) = atan2(trans(1,0),trans(0,0));

    return pose;
}

//计算整个pose-graph的误差
double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges)
{
    double sumError = 0;
    for(int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z  = PoseToTrans(z);

        Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;

        Eigen::Vector3d ei = TransToPose(Ei);


        sumError += ei.transpose() * infoMatrix * ei;
    }
    return sumError;
}


/**
 * @brief CalcJacobianAndError
 *         计算jacobian矩阵和error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     观测值:xj相对于xi的坐标
 * @param ei    计算的误差求解库
 * @param Ai    相对于xi的Jacobian矩阵
 * @param Bi    相对于xj的Jacobian矩阵
 */
void CalcJacobianAndError(Eigen::Vector3d xi,Eigen::Vector3d xj,Eigen::Vector3d z,
                          Eigen::Vector3d& ei,Eigen::Matrix3d& Ai,Eigen::Matrix3d& Bi)
{
    //TODO--Start

    // Estimated Pose
    Eigen::Matrix2d Ri;
    Ri << cos(xi(2)), -sin(xi(2)),
          sin(xi(2)),  cos(xi(2));
    Eigen::Vector2d tj = Eigen::Vector2d(xj(0), xj(1));
    Eigen::Vector2d ti = Eigen::Vector2d(xi(0), xi(1));
    double theta_j = xj(2);
    double theta_i = xi(2);

    // Measured Pose
    Eigen::Matrix2d Rij;
    Rij << cos(z(2)), -sin(z(2)),
           sin(z(2)),  cos(z(2));
    Eigen::Vector2d tij = Eigen::Vector2d(z(0), z(1));
    double theta_ij = z(2);

    // 计算error
    ei << Rij.transpose() * (Ri.transpose()*(tj - ti) - tij),
            theta_j - theta_i - theta_ij;
    // 注意: 必须进行角度正则化，保证角度在-pi~pi之间，否则优化容易出问题!!!
    // 因为有时候当我们朝着梯度下降的方向去优化角度时，由于没有正则化，角度反而会增大，导致优化产生错乱
    ei(2) = GN_NormalizationAngle(ei(2));

    // 当然也可以使用变换矩阵求解，由于TransToPose函数中使用了atan2,所以角度也会被自动限定在-pi~pi之间，而无需进行角度正则化
    // Eigen::Matrix3d Ti = PoseToTrans(xi), Tj = PoseToTrans(xj);
    // Eigen::Matrix3d Tij = PoseToTrans(z);
    // ei = TransToPose(Tij.inverse() * Ti.inverse() * Tj);

    // Jacobian dei / dxi
    Eigen::Matrix2d dRi_dtheta;
    dRi_dtheta << -sin(xi(2)), -cos(xi(2)),
                   cos(xi(2)), -sin(xi(2));
    Ai << -Rij.transpose() * Ri.transpose(),    Rij.transpose() * dRi_dtheta.transpose() * (tj - ti),
            Eigen::Vector2d(0,0).transpose(),    -1;

    // Jacobian dei / dxj
    Bi << Rij.transpose() * Ri.transpose(),  Eigen::Vector2d(0,0),
            Eigen::Vector2d(0,0).transpose(),  1;

    //TODO--end
}

/**
 * @brief LinearizeAndSolve
 *        高斯牛顿方法的一次迭代．
 * @param Vertexs   图中的所有节点
 * @param Edges     图中的所有边
 * @return          位姿的增量
 */
Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges)
{
    //申请内存
    Eigen::MatrixXd H(Vertexs.size() * 3,Vertexs.size() * 3);
    Eigen::VectorXd b(Vertexs.size() * 3);

    H.setZero();
    b.setZero();

    //固定第一帧
    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0,0,3,3) += I;

    //构造H矩阵　＆ b向量
    for(int i = 0; i < Edges.size();i++)
    {
        //提取信息
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        //计算误差和对应的Jacobian
        Eigen::Vector3d ei;
        Eigen::Matrix3d Ai;
        Eigen::Matrix3d Bi;
        CalcJacobianAndError(xi,xj,z,ei,Ai,Bi);

        int index_i, index_j;
        index_i = tmpEdge.xi;
        index_j = tmpEdge.xj;

        // 这里节点和边是从0开始计数的
        H.block(index_i*3, index_i*3, 3, 3) += Ai.transpose() * infoMatrix * Ai;
        H.block(index_i*3, index_j*3, 3, 3) += Ai.transpose() * infoMatrix * Bi;
        H.block(index_j*3, index_i*3, 3, 3) += Bi.transpose() * infoMatrix * Ai;
        H.block(index_j*3, index_j*3, 3, 3) += Bi.transpose() * infoMatrix * Bi;

        b.segment(index_i*3, 3) += Ai.transpose() * infoMatrix * ei;
        b.segment(index_j*3, 3) += Bi.transpose() * infoMatrix * ei;

    }

    //求解
    Eigen::VectorXd dx;

    // 使用稠密矩阵求解
    // dx = H.ldlt().solve(-b);           // 由于对称矩阵H可以保证为半正定矩阵，所以可以用ldlt求解

    // 使用稀疏矩阵求解
    // 构建稀疏矩阵
    Eigen::SparseMatrix<double> S = H.sparseView();
    // 使用稀疏矩阵求解器
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(S);
    if(solver.info()!=Eigen::Success) {
        std::cerr << "Decomposition H failed";
    }
    dx = solver.solve(-b);
    if(solver.info()!=Eigen::Success) {
        std::cerr << "Solving failed";
    }

    return dx;
}











