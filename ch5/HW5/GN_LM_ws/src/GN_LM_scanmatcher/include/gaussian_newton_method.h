#ifndef GAUSSIAN_NEWTON_METHOD_H
#define GAUSSIAN_NEWTON_METHOD_H

#include "basic_transform.h"

//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                              std::vector<Eigen::Vector2d> laser_pts,
                              double resolution);

//用高斯牛顿的方法来进行优化
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts);


//进行角度正则化．
double GN_NormalizationAngle(double angle);
Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec);
//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T);
// 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map, Eigen::Vector2d& coords);
// 计算H*dx = b中的H和b
double ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b);


#endif // GAUSSIAN_NEWTON_METHOD_H
