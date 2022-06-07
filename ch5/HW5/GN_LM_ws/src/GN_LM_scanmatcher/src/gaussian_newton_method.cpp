#include "gaussian_newton_method.h"

/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t* map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;                            // 规定最大迭代次数
    Eigen::Vector3d now_pose = init_pose;             // 使用init_pose初始化now_pose

    int iterationTimes = 0;
    double lastResidual = 0;

    for(int i = 0; i < maxIteration;i++)
    {
        iterationTimes = i;
        double currentResidual = 0;

        Eigen::Matrix3d H;
        Eigen::Vector3d b;

        currentResidual = ComputeHessianAndb(map, now_pose, laser_pts, H, b);

        Eigen::Vector3d delta_pose;
	
	    double lambda = 10;     		        //给H加上lambda，保证H具有良好的正定性，减小病态
	    H += lambda * Eigen::Matrix3d::Identity();
        delta_pose = H.ldlt().solve(b);        	// 由于H可以保证为半正定矩阵，所以可以用ldlt求解
	    delta_pose(2) = GN_NormalizationAngle(delta_pose(2));

        if (std::isnan(delta_pose(0)) || std::isnan(delta_pose(1)) || std::isnan(delta_pose(2))) {
            std::cout << "result is nan!" << std::endl;
            break;
        }

        if (i > 0 && currentResidual > lastResidual) {
            // 如果代价函数增长，立刻停止迭代
            break;
        }

        // 如果delta_pose太小，就跳出迭代
        /*
        double delta_dist2 = std::pow(delta_pose(0),2) + std::pow(delta_pose(1),2);
        double delta_angle = std::fabs(delta_pose(2));
        if(delta_dist2 < 1e-5 * 1e-5 && delta_angle < 1e-5 *GN_PI/180)   // 跳出迭代的条件: 平移变化量小于0.001m，角度变化量小于0.01度
            break;
        */

        // 使用李代数更新
        now_pose += delta_pose;
        // 更新代价函数
        lastResidual = currentResidual;
    }
    init_pose = now_pose;
    std::cout << "Iterations: " << iterationTimes << " times." << std::endl;
}



