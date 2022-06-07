#include "LM_method.h"

// 计算LM算法中阻尼因子lambda的初始值．
double ComputeLambdaInitLM(const Eigen::Matrix3d& H, double& ni) {
    double maxDiagonal = 0;
    ni = 2.0;
    int size = H.cols();

    for (int i = 0; i < size; ++i) {
        maxDiagonal = std::max(std::fabs(H(i, i)), maxDiagonal);
    }
    double tau = 1e-5;
    return tau * maxDiagonal;     // return the initial value of lambda
}


void AddLambdatoHessianLM(Eigen::Matrix3d& H, const double currentLambda) {
    int size = H.cols();
    for (int i = 0; i < size; ++i) {
        H(i, i) += currentLambda;
    }
}

void RemoveLambdaHessianLM(Eigen::Matrix3d& H, const double currentLambda) {
    int size = H.cols();
    for (int i = 0; i < size; ++i) {
        H(i, i) -= currentLambda;
    }
}

// Solve Hx = b
Eigen::Vector3d SolveLinearSystem(const Eigen::Matrix3d& H, const Eigen::Vector3d& b) {
    Eigen::Vector3d delta_pose;
    // delta_x_ = H.inverse() * b;
    delta_pose = H.ldlt().solve(b);    // 由于对称矩阵H可以保证为半正定矩阵，所以可以用ldlt求解
    return delta_pose;
}

void UpdateStates(Eigen::Vector3d& now_pose, const Eigen::Vector3d& delta_pose){
    // 使用李代数更新
    now_pose += delta_pose;
}

double ComputeResidual(map_t* map, const Eigen::Vector3d now_pose,
                       const std::vector<Eigen::Vector2d>& laser_pts){
    double theta = now_pose(2);
    Eigen::Matrix3d T_laser = GN_V2T(now_pose);                                 // now_pose对应的变换矩阵:  T_laser_world

    double tempResidual = 0;

    for(int k = 0; k != laser_pts.size(); ++k)
    {
        double x, y;                                                            // 位姿变换前的2D点云
        x = laser_pts[k](0);
        y = laser_pts[k](1);

        Eigen::Matrix<double, 2, 3> d_Si_T;
        d_Si_T << 1, 0, -std::sin(theta) * x - std::cos(theta) * y,
                0, 1,  std::cos(theta) * x - std::sin(theta) * y;

        // 将该激光点云 使用估计的位姿 转换到 世界系坐标系下
        Eigen::Vector2d xy_trans = GN_TransPoint(laser_pts[k], T_laser);        // 位姿变换后的2D点云      inverse needed?

        Eigen::Vector2d coords(xy_trans(0), xy_trans(1));
        Eigen::Vector3d ans = InterpMapValueWithDerivatives(map, coords);

        double M = ans(0);
        double error = 1 - M;

        tempResidual += error * error;     // 计算代价函数增长情况
    }

    return tempResidual;
}

bool IsGoodStepInLM(map_t* map, Eigen::Vector3d& now_pose, Eigen::Vector3d& delta_pose,
                    double& currentLambda, std::vector<Eigen::Vector2d>& laser_pts,
                    Eigen::Vector3d& b, double& currentResidual, double ni) {

    double scale = 0;
    double rho = 0;

    // recompute residuals after update state
    double tempResidual = 0.0;
    tempResidual = ComputeResidual(map, now_pose, laser_pts);

    scale = delta_pose.transpose() * (currentLambda * delta_pose + b);
    scale += 1e-3;    // make sure it's non-zero :)
    rho = (currentResidual - tempResidual) / scale;

    if (rho > 0 && std::isfinite(tempResidual))   // last step was good, 误差在下降
    {
        double alpha = 1. - std::pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda *= scaleFactor;
        currentLambda = std::max(15.0, currentLambda);
        ni = 2;
        return true;
    }
    else {
        currentLambda *= ni;
        ni *= 2;
        return false;
    }
}



void RollbackStates(Eigen::Vector3d& now_pose, const Eigen::Vector3d& delta_pose) {
    now_pose -= delta_pose;
}


void LevenbergMarquardtOptimization(map_t* map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;                            // 规定最大迭代次数
    Eigen::Vector3d now_pose = init_pose;             // 使用init_pose初始化now_pose

    Eigen::Matrix3d H;
    Eigen::Vector3d b;
    double currentResidual = ComputeHessianAndb(map, now_pose, laser_pts, H, b);
    double stopThresholdLM = 1e-6 * currentResidual;          // 迭代条件为 误差下降 1e-6 倍

    // LM 初始化
    double currentLambda = 0;
    double ni = 0;
    currentLambda = ComputeLambdaInitLM(H, ni);

    // LM 算法迭代求解
    bool stop = false;
    int iter = 0;

    while(!stop && (iter < maxIteration))
    {
        bool oneStepSuccess = false;
        int false_cnt = 0;

        while (!oneStepSuccess)  // 不断尝试 Lambda, 直到成功迭代一步
        {
            AddLambdatoHessianLM(H, currentLambda);

            // 解线性方程 H * delta_pose = b
            Eigen::Vector3d delta_pose;
            delta_pose = SolveLinearSystem(H, b);

            RemoveLambdaHessianLM(H, currentLambda);

            // 优化退出条件1： delta_pose 很小 或 连续错误次数过多 则退出
            if (delta_pose.squaredNorm() <= 1e-6 || false_cnt > 10) {
                if(delta_pose.squaredNorm() <= 1e-6){
                    std::cout << "delta_pose is small enough!" << std::endl;
                    std::cout << "The false times = " << false_cnt << std::endl;
                }
                if (false_cnt > 10)
                    std::cout << "The false times exceed 10!" << std::endl;
                stop = true;
                break;
            }

            // 更新状态量
            UpdateStates(now_pose, delta_pose);

            // 判断当前步是否可行以及 LM 的 lambda 怎么更新
            oneStepSuccess = IsGoodStepInLM(map, now_pose, delta_pose,
                                            currentLambda, laser_pts,
                                            b, currentResidual, ni);

            if (oneStepSuccess) {
                // 在新线性化点 构建 hessian
                currentResidual = ComputeHessianAndb(map, now_pose, laser_pts, H, b);
                false_cnt = 0;
            } 
	    else {
                false_cnt++;
                RollbackStates(now_pose, delta_pose);   // 误差没下降，回滚
            }
        }
        iter++;
        // 优化退出条件2： currentResidual 跟第一次的currentResidual相比，下降了 1e6 倍则退出
        if (currentResidual <= stopThresholdLM){
            stop = true;
            std::cout << "currentResidual is small enough!!" << std::endl;
        }

    }

    init_pose = now_pose;
    std::cout << "Iterations: " << iter << " times." << std::endl;

}

