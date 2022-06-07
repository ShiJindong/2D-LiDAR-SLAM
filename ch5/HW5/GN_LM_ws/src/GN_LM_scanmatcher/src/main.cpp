#include "gaussian_newton_method.h"
#include "LM_method.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"

double GN_NormalizationAngle(double angle);
int optimization_method;

class GaussianNewtonDebug
{
public:
    GaussianNewtonDebug()
    {
        m_laserscanSub = m_nh.subscribe("sick_scan",1,&GaussianNewtonDebug::rosLaserScanCallback,this);

        m_odomPub = m_nh.advertise<nav_msgs::Path>("odom_path",1,true);

        m_gaussianNewtonPub = m_nh.advertise<nav_msgs::Path>("gaussian_newton_path",1,true);
    }

    //单纯的数据类型转换，不进行坐标系转换．
    void ConvertChampionLaserScanToEigenPointCloud(const sensor_msgs::LaserScanConstPtr& msg,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size();i++)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double angle = msg->angle_min + msg->angle_increment * i;

            double lx = msg->ranges[i] * std::cos(angle);
            double ly = msg->ranges[i] * std::sin(angle);

            if(std::isnan(lx) || std::isinf(ly)||
               std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx,ly));
        }
    }

    void PublishPath(ros::Publisher& puber,
                     std::vector<Eigen::Vector3d>& path)
    {
        // 注意: 这里将Path定义为了函数内的临时变量 而非 类内的全局变量
        // 也就是说每次处理新的一帧数据得到新的一帧位姿后，就要将所有时刻的位姿重新压入path
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "/odom";

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/odom";
        for(int i = 0; i < path.size();i++)
        {
            Eigen::Vector3d traj_node = path[i];
            pose.pose.position.x = traj_node(0);
            pose.pose.position.y = traj_node(1);
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(traj_node(2));
            path_msg.poses.push_back(pose);
        }

        puber.publish(path_msg);
    }

    void rosLaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {
        static bool isFirstFrame = true;
        Eigen::Vector3d nowPose;
        if(getOdomPose(msg->header.stamp,nowPose) == false)      // 得到时刻t 机器人在里程计坐标"/odom"下的坐标
        {
            std::cout <<"Failed to get Odom Pose"<<std::endl;
            return ;
        }

        if(isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            isFirstFrame = false;

            m_prevLaserPose = nowPose;
            ConvertChampionLaserScanToEigenPointCloud(msg,m_prevPts);

            m_odomPath.push_back(nowPose);
            m_gaussianNewtonPath.push_back(nowPose);

            return ;
        }

        // 机器人上一时刻在里程计坐标系下的位姿
        auto pre_odom_pose = m_odomPath.back();
        // 机器人通过监听轮式里程计信息得到在里程计坐标下的位姿变化
        double delta_dist2 = std::pow(nowPose(0) - pre_odom_pose(0),2) + std::pow(nowPose(1) - pre_odom_pose(1),2);
        double delta_angle = std::fabs(tfNormalizeAngle(nowPose(2) - pre_odom_pose(2)));

        // 如果两帧之间轮式里程计位置变化太小，不进行激光匹配，节省计算量
        if(delta_dist2 < 0.2 * 0.2 &&
           delta_angle < tfRadians(10.0))
        {
            return ;
        }
        // 数据类型转换．
        std::vector<Eigen::Vector2d> nowPts;
        ConvertChampionLaserScanToEigenPointCloud(msg,nowPts);
        // 使用上一帧激光点云数据生成地图 (似然场)
        map_t* map = CreateMapFromLaserPoints(m_prevLaserPose,m_prevPts,0.05);     // map resolution = 0.1，相当于1米是1/0.1=10个栅格

        // 进行优化．
        // 初始解为上一帧激光位姿+运动增量，这里运动增量由轮式里程计增量得来
        Eigen::Vector3d deltaPose = nowPose - m_odomPath.back();    // 轮式里程计测得的位姿变化量
        deltaPose(2) = GN_NormalizationAngle(deltaPose(2));         // 角度正则化

        Eigen::Matrix3d R_laser;            // 将2维旋转矩阵拓展成3维(依然正交)，使其可以直接和3维位姿相乘
        double theta = m_prevLaserPose(2);
        R_laser << cos(theta), -sin(theta), 0, 
                   sin(theta),  cos(theta), 0,
                        0,          0,      1;

        Eigen::Matrix3d R_odom;             // 将2维旋转矩阵拓展成3维(依然正交)，使其可以直接和3维位姿相乘
        theta = m_odomPath.back()(2);
        R_odom << cos(theta), -sin(theta), 0, 
                  sin(theta),  cos(theta), 0,
                       0,          0,      1;
        // 利用 轮式里程计位姿增量 计算 激光里程计坐标系下的位姿增量, 从而估计当前帧激光里程计坐标系下的位姿
        // (我们前面规定了激光里程计的原点为轮式里程计的原点，即定义了相同的世界坐标系原点)
        //     -------->             pose_laser  +  T_laser_world *  T_world_odom  *  pose_odom
        //                                |               |              |               |
        Eigen::Vector3d finalPose = m_prevLaserPose + R_laser * R_odom.transpose() * deltaPose;
        finalPose(2) = GN_NormalizationAngle(finalPose(2));         // 角度正则化

        std::cout << "Init Pose:" << finalPose.transpose() << std::endl;

        if(optimization_method == 1)
            GaussianNewtonOptimization(map,finalPose,nowPts);           // 参数: (前一帧激光数据构成的似然场, 估计位姿作为初始值(亦是返回位姿), 当前帧点云)
        else if(optimization_method == 2)
            LevenbergMarquardtOptimization(map,finalPose,nowPts);
        else
            std::cout << "No optimization method is selected." << std::endl;

        //更新数据．
        m_prevLaserPose = finalPose;
        m_prevPts = nowPts;

        std::cout <<"Final Pose:"<<finalPose.transpose()<<std::endl<< std::endl;


        //释放地图
        map_free(map);

        //保存路径．
        m_odomPath.push_back(nowPose);
        m_gaussianNewtonPath.push_back(finalPose);

        PublishPath(m_odomPub,m_odomPath);                          // 发布轮式里程计轨迹
        PublishPath(m_gaussianNewtonPub,m_gaussianNewtonPath);      // 发布激光里程计轨迹
    }

    bool getOdomPose(ros::Time t,
                     Eigen::Vector3d& pose)
    {
        // Get the robot's pose
        /*
         * tf转换，分为两部分：broadcaster和listener。前者是tf的发布者，后者是接收者。
         * 我们如果要建立一个完整的tf体系，需要自己先生成tf信息用broadcaster发布出去，然后再在需要tf信息的节点使用listener接收tf信息后进行转换。
         * 但是大部分情况下，我们都是在一个已经有完善体系的框架下工作，并不需要自己管理tf。这时我们只需要了解怎样用listener接收tf进行pose转换就可以了
         * 参考 void transformPose使用   https://blog.csdn.net/u012706484/article/details/108070293
         */
	t = ros::Time(0.0);
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                   tf::Vector3(0,0,0)), t, "/base_link");      // 这里让"/base_link"和"/odom"坐标系重合，不存在相对位姿差异
        tf::Stamped<tf::Transform> odom_pose;

        // 一般使用try...catch...异常处理结构来监听tf变换
        try
        {
            // m_tfListener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
            // 得到时刻t 机器人在里程计坐标"/odom"下的坐标
            //                    目标frame_id  源pose   目标pose
            //                            |       |        |
            m_tfListener.transformPose("/odom", ident, odom_pose);
            /*
             * 需要注意的是，从参数上来看，转换时是不需要指定 源pose 的 frame_id的，这是因为它已经包含在了源pose的信息中，
             * 换句话说，就是这个函数一个隐含的使用条件是，源pose中必须指明它属于哪个frame，这里为"/base_link"
             */
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }

        double yaw = tf::getYaw(odom_pose.getRotation());
        pose << odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw;

        return true;
    }

    ros::NodeHandle m_nh;

    Eigen::Vector3d m_prevLaserPose;

    std::vector<Eigen::Vector2d> m_prevPts;

    std::vector<Eigen::Vector3d> m_odomPath;
    std::vector<Eigen::Vector3d> m_gaussianNewtonPath;

    tf::TransformListener m_tfListener;
    ros::Subscriber m_laserscanSub;
    ros::Publisher m_odomPub;
    ros::Publisher m_gaussianNewtonPub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "GaussianNewton_debug");

    std::cout << "Which optimization method for scanmatcher would you like to use?" << std::endl;
    std::cout << "Please enter 1 for Gauss Newton Method, enter 2 for Levenberg Marquardt Method." << std::endl;
    while(std::cin >> optimization_method && !(optimization_method == 1 || optimization_method == 2))
        std::cout << "Please enter 1 or 2." << std::endl;

    std::cout << "Please play your rosbag!" << std::endl;

    GaussianNewtonDebug gn_debug;

    ros::spin();

    return (0);
}

