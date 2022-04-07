#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据
TODO:
构建最小二乘需要的超定方程组
Ax = b
*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //TODO: 构建超定方程组

        // matrix.block(i,j,p,q); is represented for Block of size (p,q), starting at (i,j)   (alternative: matrix.block<p,q>(i,j);)
        A.block(now_len * 3, 0, 1, 3) = Odom.transpose();
        A.block(now_len * 3 + 1, 3, 1, 3) = Odom.transpose();
        A.block(now_len * 3 + 2, 6, 1, 3) = Odom.transpose();

        // vector.segment(i,n); is represented for Block containing n elements, starting at position i
        b.segment(now_len * 3, 3) = scan;

        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;
    //TODO: 求解线性最小二乘

    // 1. 根据最小二乘公式，直接求逆
    // Eigen::Matrix<double, 9, 1> correct_vector = (A.transpose() * A).inverse() * A.transpose() * b;

    // 2. 改进的Cholesky分解法 ldlt() 要求矩阵半正定: 这里无法使用！
    // Eigen::Matrix<double, 9, 1> correct_vector = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    // 3. QR分解法 colPivHouseholderQr()对矩阵无要求
    Eigen::Matrix<double, 9, 1> correct_vector = (A.transpose() * A).colPivHouseholderQr().solve(A.transpose() * b);

    correct_matrix << correct_vector(0, 0), correct_vector(1, 0), correct_vector(2, 0),
                      correct_vector(3, 0), correct_vector(4, 0), correct_vector(5, 0),
                      correct_vector(6, 0), correct_vector(7, 0), correct_vector(8, 0);
    //end of TODO
    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
