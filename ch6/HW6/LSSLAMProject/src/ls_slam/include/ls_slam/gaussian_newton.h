#ifndef GAUSSIAN_NEWTON_H
#define GAUSSIAN_NEWTON_H

#include <vector>
#include <eigen3/Eigen/Core>

// 自定义边的数据结构
typedef struct edge
{
  int xi,xj;           // 边连接的两个顶点的序号
  Eigen::Vector3d measurement;
  Eigen::Matrix3d infoMatrix;
}Edge;


Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges);

double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges);

const double GN_PI = 3.1415926;
double GN_NormalizationAngle(double angle);





#endif
