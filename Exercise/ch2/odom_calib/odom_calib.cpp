#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;

string scan_match_file = "./scan_match.txt";
string odom_file = "./odom.txt";

int main(int argc, char** argv)
{
    // 放置激光雷达的时间和匹配值 t_s s_x s_y s_th
    vector<vector<double>> s_data;
    // 放置轮速计的时间和左右轮角速度 t_r w_L w_R
    vector<vector<double>> r_data;

    ifstream fin_s(scan_match_file);
    ifstream fin_r(odom_file);
    if (!fin_s || !fin_r)
    {
        cerr << "请在有scan_match.txt和odom.txt的目录下运行此程序" << endl;
        return 1;
    }

    // 读取激光雷达的匹配值
    while (!fin_s.eof()) {
        double s_t, s_x, s_y, s_th;
        fin_s >> s_t >> s_x >> s_y >> s_th;
        s_data.push_back(vector<double>({s_t, s_x, s_y, s_th}));
    }
    fin_s.close();

    // 读取两个轮子的角速度
    while (!fin_r.eof()) {
        double t_r, w_L, w_R;
        fin_r >> t_r >> w_L >> w_R;
        r_data.push_back(vector<double>({t_r, w_L, w_R}));
    }
    fin_r.close();

    // 第一步：计算中间变量J_21和J_22
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    // 设置数据长度
    A.conservativeResize(5000, 2);
    b.conservativeResize(5000);
    A.setZero();
    b.setZero();

    size_t id_r = 0;
    size_t id_s = 0;
    double last_rt = r_data[0][0];
    double w_Lt = 0;
    double w_Rt = 0;
    while (id_s < 5000)
    {
        // 激光的匹配信息
        const double &s_t = s_data[id_s][0];
        const double &s_th = s_data[id_s][3];
        // 里程计信息
        const double &r_t = r_data[id_r][0];
        const double &w_L = r_data[id_r][1];
        const double &w_R = r_data[id_r][2];
        ++id_r;      // 对于同一个 id_s 可递增多次 id_r, 直到 r_t >= s_t, 才递增一次 id_s
        // 在2帧激光匹配时间内进行里程计角度积分
        if (r_t < s_t)
        {
            // 对于一开始的数据： 对里程计数据进行积分，直到离激光数据的时间戳最近的前一个里程计数据的时间戳
            double dt = r_t - last_rt;
            w_Lt += w_L * dt;
            w_Rt += w_R * dt;
            last_rt = r_t;
        }
        else
        {
            // 对于后续的数据： 将上一个里程计数据的时间戳被设定为上一个激光数据的时间戳，从而计算积分值
            double dt = s_t - last_rt;
            w_Lt += w_L * dt;
            w_Rt += w_R * dt;
            last_rt = s_t;
            // 填充A, b矩阵
            //TODO: (3~5 lines)
            A.row(id_s) = Eigen::Vector2d(w_Lt, w_Rt).transpose();
            b(id_s) = s_th;
            //end of TODO
            w_Lt = 0;
            w_Rt = 0;
            ++id_s;      // 只在该else条件内 递增激光数据的时间戳
        }
    }
    // 进行最小二乘求解
    Eigen::Vector2d J21J22;

    //TODO: (1~2 lines)
    // 直接求逆
    // J21J22 = (A.transpose() * A).inverse() * A.transpose() * b;
    // 使用RQ分解求解
    J21J22 = (A.transpose() * A).colPivHouseholderQr().solve(A.transpose() * b);
    //end of TODO

    const double &J21 = J21J22(0);
    const double &J22 = J21J22(1);
    cout << "J21: " << J21 << endl;
    cout << "J22: " << J22 << endl;

    // 第二步，求解轮间距 b_wheel
    Eigen::VectorXd C;
    Eigen::VectorXd S;
    // 设置数据长度
    C.conservativeResize(10000);
    S.conservativeResize(10000);
    C.setZero();
    S.setZero();

    id_r = 0;
    id_s = 0;
    last_rt = r_data[0][0];
    double th = 0;
    double cx = 0;
    double cy = 0;
    while (id_s < 5000)
    {
        // 激光的匹配信息
        const double &s_t = s_data[id_s][0];
        const double &s_x = s_data[id_s][1];
        const double &s_y = s_data[id_s][2];
        // 里程计信息
        const double &r_t = r_data[id_r][0];
        const double &w_L = r_data[id_r][1];
        const double &w_R = r_data[id_r][2];
        ++id_r;
        // 在2帧激光匹配时间内进行里程计位置积分
        if (r_t < s_t)
        {
            double dt = r_t - last_rt;
            cx += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * cos(th);
            cy += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * sin(th);
            th += (J21 * w_L + J22 * w_R) * dt;
            last_rt = r_t;
        }
        else
        {
            double dt = s_t - last_rt;
            cx += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * cos(th);
            cy += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * sin(th);
            th += (J21 * w_L + J22 * w_R) * dt;
            last_rt = s_t;
            // 填充C, S矩阵
            //TODO: (4~5 lines)
            C.segment(id_s * 2, 2) = Eigen::Vector2d(cx, cy).transpose();
            S.segment(id_s * 2, 2) = Eigen::Vector2d(s_x, s_y).transpose();
            //end of TODO
            cx = 0;
            cy = 0;
            th = 0;
            ++id_s;
        }
    }
    // 进行最小二乘求解，计算b_wheel, r_L, r_R
    double b_wheel;
    Eigen::Matrix<double, 1, 1> b_wheel_matrix;
    double r_L;
    double r_R;
    //TODO: (3~5 lines)
    // 直接求逆
    // b_wheel = (C.transpose() * C).inverse() * C.transpose() * S;
    // 使用QR分解
    b_wheel_matrix = (C.transpose() * C).colPivHouseholderQr().solve(C.transpose() * S);
    b_wheel = b_wheel_matrix(0, 0);

    r_L = -J21 * b_wheel;
    r_R = J22 * b_wheel;
    //end of TODO

    cout << "b_wheel: " << b_wheel << endl;
    cout << "r_L: " << r_L << endl;
    cout << "r_R: " << r_R << endl;

    cout << "参考答案：轮间距b为0.6m左右，两轮半径为0.1m左右" << endl;

    return 0;
}

