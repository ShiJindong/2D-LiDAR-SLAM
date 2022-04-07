#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <cmath>

pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator(tf::TransformListener* tf)
    {
        tf_ = tf;
        // 通过句柄创建一个subscriber，订阅名为champion_scan的topic,队列长度为10，监听到后立即调用回调函数ScanCallBack (类成员函数)
        // this?
        scan_sub_ = nh_.subscribe("champion_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
    }


    ~LidarMotionCalibrator()
    {
        if(tf_!=NULL)
            delete tf_;
    }

    // 拿到原始的激光数据来进行处理
    // 使用自定义的消息类型 champion_nav_msgs, 通过传入 指向该消息类型的指针 ChampionNavLaserScanPtr 的引用
    void ScanCallBack(const champion_nav_msgs::ChampionNavLaserScanPtr& scan_msg)
    {
        //转换到矫正需要的数据
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;   // 这里header的时间即为该帧点云的起始时间

        champion_nav_msgs::ChampionNavLaserScan laserScanMsg = *scan_msg;   // 解引用获得该消息类型的对象

        //得到最终点的时间
        int beamNum = laserScanMsg.ranges.size();     // 一帧激光点云的点数
        endTime = startTime + ros::Duration(laserScanMsg.time_increment * (beamNum - 1));

        // 将数据复制出来，存储到容器 angles 和 ranges
        std::vector<double> angles,ranges;
        for(int i = beamNum - 1; i >= 0; --i)
        {   
            double lidar_dist = laserScanMsg.ranges[i];
            double lidar_angle = laserScanMsg.angles[i];

            if(lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
                lidar_dist = 0.0;

            ranges.push_back(lidar_dist);
            angles.push_back(lidar_angle);
        }

        //转换为pcl::pointcloud for visuailization

        tf::Stamped<tf::Pose> visualPose;
        if(!getLaserPose(visualPose, startTime, tf_))
        {

            ROS_WARN("Not visualPose,Can not Calib");
            return ;
        }

        double visualYaw = tf::getYaw(visualPose.getRotation());

        visual_cloud_.clear();
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);

            // 将所有激光点转换到起始位姿坐标系下
            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            // pack r/g/b into rgb
            unsigned char r = 255, g = 0, b = 0;    //red color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }
        std::cout << std::endl;



        //进行矫正
        Lidar_Calibration(ranges,angles,
                          startTime,
                          endTime,
                          tf_);

        //转换为pcl::pointcloud for visuailization
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);


            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            unsigned char r = 0, g = 255, b = 0;    // green color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }

        //进行显示
         g_PointCloudView.showCloud(visual_cloud_.makeShared());
    }


    /**
     * @name getLaserPose()
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param odom_pos  机器人的位姿
     * @param dt        dt时刻
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = "base_laser";
        robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系

        // get the global pose of the robot
        try
        {
            // 通过监听tf树，获得dt时刻下，激光雷达坐标系在里程计坐标系下的位姿，超过0.5s报错
            if(!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * @brief Lidar_MotionCalibration
     *        激光雷达运动畸变去除分段函数;
     *        在此分段函数中，认为机器人是匀速运动；
     * @param frame_base_pose       标定完毕之后的基准坐标系
     * @param frame_start_pose      本分段第一个激光点对应的位姿
     * @param frame_end_pose        本分段最后一个激光点对应的位姿
     * @param ranges                激光数据－－距离
     * @param angles                激光数据－－角度
     * @param startIndex            本分段第一个激光点在激光帧中的下标
     * @param beam_number           本分段的激光点数量
     */
    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number)
    {
        //TODO
        // 方法一: 使用四元数及四元数插值，使用平移插值
        tf::Quaternion q_start = frame_start_pose.getRotation();
        tf::Quaternion q_end = frame_end_pose.getRotation();
        tf::Vector3 xy_start = tf::Vector3(frame_start_pose.getOrigin().getX(), frame_start_pose.getOrigin().getY(), 1);
        tf::Vector3 xy_end = tf::Vector3(frame_end_pose.getOrigin().getX(), frame_end_pose.getOrigin().getY(), 1);

        // 对frame_start_pose和frame_end_pose之间的所有点进行线性插值
        for(int i = 0; i <= beam_number-1; ++i)
        {
            double ratio = i/(beam_number-1);
            // 对平移进行插值: 对 xy_start 和 xy_end 根据 ratio 进行插值
            tf::Vector3 xy_lerp = xy_start.lerp(xy_end, ratio);
            // 对旋转进行插值: 对 q_start 和 q_end 根据 ratio 进行插值
            tf::Quaternion q_slerp = q_start.slerp(q_end, ratio);

            // T_O_lerp 为 当前插值激光点对应的机器人坐标系到里程计坐标系的变换矩阵
            tf::Transform T_O_lerp;
            T_O_lerp.setOrigin(xy_lerp);
            T_O_lerp.setRotation(q_slerp);

            // 从容器 ranges 和 angles 中获得存在运动畸变的位姿
            double x_distorted = ranges[startIndex + i] * cos(angles[startIndex + i]);
            double y_distorted = ranges[startIndex + i] * sin(angles[startIndex + i]);
            double yaw_distorted = angles[startIndex + i];

            // 去畸变:  frame_base_pose.inverse() 相当于 T_O_base.inverse()
            tf::Vector3 xy_undistorted = frame_base_pose.inverse() * T_O_lerp * tf::Vector3(x_distorted, y_distorted, 1);
            // 将去除畸变后的位姿转化为激光数据range和angle
            ranges[startIndex + i] = std::sqrt(xy_undistorted[0] * xy_undistorted[0] + xy_undistorted[1] * xy_undistorted[1]);
            angles[startIndex + i] = std::atan2(xy_undistorted[1], xy_undistorted[0]);
        }


        /*
         // 方法二: 纯粹使用变换矩阵，只使用平移插值
         //  data_lerp:          线性插值后的位姿
         //  T_O_base:           基准坐标系到里程计坐标系的变换矩阵
         //  T_O_lerp:           当前插值激光点对应的机器人坐标系到里程计坐标系的变换矩阵
         //  T_lerp_point:       当前扫描到的激光点到当前插值激光点对应的机器人坐标系的变换矩阵
         //  T_base_point:       当前扫描到的激光点到基准坐标系的变换矩阵  T_base_point = T_O_base.inverse() * T_O_lerp * T_lerp_point
         //  pose_undistorted = T2V(T_base_point):   去除运动畸变的点云位姿

         double yaw_base = tf::getYaw(frame_base_pose.getRotation());
         double x_base = frame_base_pose.getOrigin().getX();
         double y_base = frame_base_pose.getOrigin().getY();

         double yaw_start = tf::getYaw(frame_start_pose.getRotation());
         double x_start = frame_start_pose.getOrigin().getX();
         double y_start = frame_start_pose.getOrigin().getY();
         tf::Vector3 data_start(x_start, y_start, yaw_start);

         double yaw_end = tf::getYaw(frame_end_pose.getRotation());
         double x_end = frame_end_pose.getOrigin().getX();
         double y_end = frame_end_pose.getOrigin().getY();
         tf::Vector3 data_end(x_end, y_end, yaw_end);

         // 对frame_start_pose和frame_end_pose之间的所有点进行线性插值
         for(int i = 0; i <= beam_number-1; ++i)
         {
             // 对 data_start 和 data_end 根据 ratio 进行插值， 结果为 data_lerp = [x_lerp, y_lerp, yaw_lerp]
             // 这里对平移和旋转同时进行插值
             double ratio = i/(beam_number-1);
             tf::Vector3 data_lerp = data_start.lerp(data_end, ratio);
             double x_lerp = data_lerp.x();
             double y_lerp = data_lerp.y();
             double yaw_lerp = data_lerp.z();

             // 从容器 ranges 和 angles 中获得存在运动畸变的位姿
             double x_distorted = ranges[startIndex + i] * cos(angles[startIndex + i]);
             double y_distorted = ranges[startIndex + i] * sin(angles[startIndex + i]);
             double yaw_distorted = angles[startIndex + i];

             // 计算变换矩阵
             Eigen::Matrix3d T_O_base;
             T_O_base << cos(yaw_base), -sin(yaw_base), x_base,
                         sin(yaw_base),  cos(yaw_base), y_base,
                                0,             0,           1;

             Eigen::Matrix3d T_O_lerp;
             T_O_lerp << cos(yaw_lerp), -sin(yaw_lerp), x_lerp,
                         sin(yaw_lerp),  cos(yaw_lerp), y_lerp,
                                0,             0,           1;

             Eigen::Matrix3d T_lerp_point;
             T_lerp_point << cos(yaw_distorted), -sin(yaw_distorted), x_distorted,
                             sin(yaw_distorted),  cos(yaw_distorted), y_distorted,
                                    0,                    0,              1;

             // 去畸变
             Eigen::Matrix3d T_base_point = T_O_base.inverse() * T_O_lerp * T_lerp_point;
             Eigen::Vector3d pose_undistorted = Eigen::Vector3d(T_base_point(0, 2), T_base_point(1, 2),
                                                                std::atan2(T_base_point(1, 0), T_base_point(0, 0)));

             // 将去除畸变后的位姿转化为激光数据range和angle
             ranges[startIndex + i] = std::sqrt(pose_undistorted(0) * pose_undistorted(0) + pose_undistorted(1) * pose_undistorted(1));
             angles[startIndex + i] = std::atan2(pose_undistorted(1), pose_undistorted(0));
          }
          */

        //end of TODO

    }



    //激光雷达数据　分段线性进行插值
    //这里会调用Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief 激光雷达数据　分段线性进行差值　分段的周期为5ms
     * @param ranges 激光束的距离值集合
     * @param angle　激光束的角度值集合
     * @param startTime　第一束激光的时间戳
     * @param endTime　最后一束激光的时间戳
     * @param *tf_
    */
    void Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        //统计激光束的数量
        int beamNumber = ranges.size();
        if(beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return ;
        }

        // 5ms来进行分段
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        //起始时间 us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / (beamNumber - 1); // 每束激光数据的时间间隔

        //当前插值的段的起始坐标
        int start_index = 0;

        //起始点的位姿 这里要得到起始点位置的原因是　起始点就是我们的base_pose
        //所有的激光点的基准位姿都会改成我们的base_pose
        // ROS_INFO("get start pose");

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0),tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        int cnt = 0;
        //基准坐标就是第一个位姿的坐标
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            //分段线性,时间段的大小为interpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);

            // interpolation_time_duration 插值时间段的长短取决于里程计位姿的更新速度，这里取 5ms 左右
            // 由于里程计位姿的更新速度低于激光点云的扫描速度，所以需要对interpolation_time_duration内的点云做位姿插值
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                //得到起点和终点的位姿
                //终点的位姿
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                //对当前的起点和终点进行插值
                //interpolation_time_duration中间有多少个点.
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose,
                                        frame_start_pose,
                                        frame_mid_pose,
                                        ranges,
                                        angles,
                                        start_index,
                                        interp_count);

                //更新时间
                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
    }

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;

    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
};




int main(int argc,char ** argv)
{
    ros::init(argc,argv,"LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));        // tf变换监听器，计算每一个时刻机器人在里程计坐标系下的位姿

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);       // 运动畸变去除器

    ros::spin();
    return 0;
}


