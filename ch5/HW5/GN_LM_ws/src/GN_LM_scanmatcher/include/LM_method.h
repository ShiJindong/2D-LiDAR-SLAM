#ifndef LM_METHOD_H
#define LM_METHOD_H

#include "basic_transform.h"

// Set the initial value of lambda
double ComputeLambdaInitLM(const Eigen::Matrix3d& H, double& ni);

// add current lambda to Hessian
void AddLambdatoHessianLM(Eigen::Matrix3d& H, const double currentLambda);

// remove current lambda from Hessian
void RemoveLambdaHessianLM(Eigen::Matrix3d& H, const double currentLambda);

Eigen::Vector3d SolveLinearSystem(const Eigen::Matrix3d& H, const Eigen::Vector3d& b);

void UpdateStates(Eigen::Vector3d& now_pose, const Eigen::Vector3d& delta_pose);

// calculate Residual after states update
double ComputeResidual(map_t* map, const Eigen::Vector3d now_pose,
                       const std::vector<Eigen::Vector2d>& laser_pts);

bool IsGoodStepInLM(map_t* map, Eigen::Vector3d& now_pose, Eigen::Vector3d& delta_pose,
                    double& currentLambda, std::vector<Eigen::Vector2d>& laser_pts,
                    Eigen::Vector3d& b, double& currentResidual, double ni);

void RollbackStates(Eigen::Vector3d& now_pose, const Eigen::Vector3d& delta_pose);

// Optimization using LM-Algorithm
void LevenbergMarquardtOptimization(map_t* map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts);





#endif // LM_METHOD_H
