#include "basic_transform.h"

const double GN_PI = 3.1415926;

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

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  << cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
            0,           0,     1;

    return T;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}



//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                                std::vector<Eigen::Vector2d> laser_pts,
                                double resolution)
{
    // 初始化map数据结构
    map_t* map = map_alloc();

    // map原点为激光位姿
    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    /*
     * malloc 功能: 分配固定字节的内存块   使用方式类似于 new/delete
     * 如果分配成功则返回指向被分配内存的指针，否则返回空指针NULL。
     * malloc 必须由我们自己计算要使用的字节数，并且在返回后强行转换为实际需要使用的类型的指针
     * 例如: int* p = (int*) malloc ( sizeof(int) * 100 ); 分配可以放得下100个整数的内存空间
     * 当内存不再使用时，应使用free()函数将内存块释放。  free(p);
     */
    // 根据map的size大小设置cells的内存大小
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 1;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物    // 将这一帧激光点全部设置成障碍物
    for(int i = 0; i < laser_pts.size();i++)
    {
        // 将当前激光点转成世界坐标系下
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);

        // 在世界坐标系下，得到地图坐标系下栅格的位置
        int cell_x,cell_y;
        cell_x = MAP_GXWX(map,tmp_pt(0));
        cell_y = MAP_GYWY(map,tmp_pt(1));

        // 将该位置设置为被激光击中了
        map->cells[MAP_INDEX(map,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．   得到我们最后要用到的势然场
    map_update_cspace(map,0.5);     // 0.5表示障碍物最大作用距离，若障碍物离某点距离大于0.5(这里相当于0.5/0.1 = 5个地图中栅格距离)，就不在视其为该点附近的障碍物

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示市场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map, Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans;

    double x, y;            // world coordinate
    x = coords(0);
    y = coords(1);

    double i, j;            // map coordinate
    int i0, j0, i1, j1;
    i = MAP_GXWX_DOUBLE(map, x);
    j = MAP_GXWX_DOUBLE(map, y);
    i0 = MAP_GXWX(map, x);
    j0 = MAP_GXWX(map, y);
    i1 = i0 + 1;
    j1 = j0 + 1;

    double Z1, Z2, Z3, Z4;
    Z1 = map->cells[MAP_INDEX(map, i0, j0)].score;          // 左上角插值点
    Z2 = map->cells[MAP_INDEX(map, i1, j0)].score;		    // 右上角插值点
    Z3 = map->cells[MAP_INDEX(map, i1, j1)].score;		    // 右下角插值点
    Z4 = map->cells[MAP_INDEX(map, i0, j1)].score;   		// 左下角插值点

    double a, b, c, d;
    a = (j - j0)/(j1 - j0);
    b = (j1 - j)/(j1 - j0);
    c = (i - i0)/(i1 - i0);
    d = (i1 - i)/(i1 - i0);

    double L, dL_i, dL_j;
    L = a * (c * Z3 + d * Z4) + b * (c * Z2 + d * Z1);
    dL_i = a * (Z3 - Z4)/(i1 - i0) + b * (Z2 - Z1)/(i1 - i0);
    dL_j = (c * Z3 + d * Z4)/(j1 - j0) - (c * Z2 + d * Z1)/(j1 - j0);

    // 将导数从map转到world
    dL_i = dL_i / map->resolution;
    dL_j = dL_j / map->resolution;

    ans << L, dL_i, dL_j;



    return ans;
}




/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
double ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    double theta = now_pose(2);
    Eigen::Matrix3d T_laser = GN_V2T(now_pose);                             // now_pose对应的变换矩阵:  T_laser_world
    double currentResidual = 0;                                             // 清空currentResidual

    for(int k = 0; k != laser_pts.size(); ++k)
    {
        double x, y;                                                        // 位姿变换前的2D点云
        x = laser_pts[k](0);
        y = laser_pts[k](1);

        Eigen::Matrix<double, 2, 3> d_Si_T;
        d_Si_T << 1, 0, -std::sin(theta) * x - std::cos(theta) * y,
                  0, 1,  std::cos(theta) * x - std::sin(theta) * y;

        // 将该激光点云 使用估计的位姿 转换到 世界系坐标系下
        Eigen::Vector2d xy_trans = GN_TransPoint(laser_pts[k], T_laser);    // 位姿变换后的2D点云

        Eigen::Vector2d coords(xy_trans(0), xy_trans(1));
        Eigen::Vector3d ans = InterpMapValueWithDerivatives(map, coords);

        double M = ans(0);
        double error = 1 - M;
        Eigen::Matrix<double, 1, 2> dM;
        dM << ans(1), ans(2);
        Eigen::Matrix<double, 1, 3> J = dM * d_Si_T;

        b += J.transpose() * error;
	    H += J.transpose() * J;

        currentResidual += error * error;     // 计算代价函数增长情况
    }

    return currentResidual;
}



