#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"

int mapping_method = 0;

/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * 2D画线算法　来进行计算两个点之间的grid cell
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            pointX = y;
            pointY = x;
        }
        else
        {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
            y += ystep;
            error -= deltaX;
        }

        //不包含最后一个点．
        if (pointX == x1 && pointY == y1)
            continue;

        //保存所有的点
        tmpIndex.SetIndex(pointX, pointY);

        gridIndexVector.push_back(tmpIndex);
    }

    return gridIndexVector;
}

void SetMapParams(void)
{
    mapParams.width = 1000;
    mapParams.height = 1000;            // 地图高和宽各为1000个栅格，对应世界坐标系下 50米x50米
    mapParams.resolution = 0.05;        // 表示1个栅格长宽为0.05米

    //每次被击中的log变化值，覆盖栅格建图算法需要的参数
    mapParams.log_free = -1;            // 逆观测模型
    mapParams.log_occ = 2;

    //每个栅格的最大最小值．
    mapParams.log_max = 100.0;
    mapParams.log_min = 1.0;

    mapParams.origin_x = 0.0;
    mapParams.origin_y = 0.0;

    //地图的原点，在地图的正中间
    mapParams.offset_x = 500;           // 世界坐标系原点位于地图正中间
    mapParams.offset_y = 500;

    pMap = new unsigned char[mapParams.width * mapParams.height];

    //计数建图算法需要的参数
    //每个栅格被激光击中的次数
    pMapHits = new unsigned long[mapParams.width * mapParams.height];
    //每个栅格被激光通过的次数
    pMapMisses = new unsigned long[mapParams.width * mapParams.height];

    //TSDF建图算法需要的参数
    pMapW = new unsigned long[mapParams.width * mapParams.height];
    pMapTSDF = new double[mapParams.width * mapParams.height];

    //初始化
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        pMap[i] = 50;                   // 50: Unknown 
        pMapHits[i] = 0;
        pMapMisses[i] = 0;
        pMapW[i] = 0;
        pMapTSDF[i] = -1;
    }
}

// 从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(double x, double y)
{
    GridIndex index;

    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;       // x 为纵向坐标，y为横向坐标
}

// 栅格坐标转化为世界坐标
void ConvertGridIndex2World(GridIndex index, double &x, double &y)
{
    //ceil返回大于等于该数的整数值，与floor相反
    x = (index.x - mapParams.offset_x) * mapParams.resolution + mapParams.origin_x;
    y = (index.y - mapParams.offset_y) * mapParams.resolution + mapParams.origin_y;
}

//插值函数
double interpolation(double A, double B, double a, double b)
{   
    double value = (b*A - a*B) / (b-a);
    return value = a == b ? A : value;
}

//判断index是否有效
bool isValidGridIndex(GridIndex index)
{
    if (index.x >= 0 && index.x < mapParams.height && index.y >= 0 && index.y < mapParams.width)
        return true;

    return false;
}

void DestoryMap()
{
    if (pMap != NULL)
        delete[] pMap;
}

//
void OccupanyMapping(std::vector<GeneralLaserScan> &scans, std::vector<Eigen::Vector3d> &robot_poses)
{
    std::cout << "开始建图，请稍后..." << std::endl;
    //枚举所有的激光雷达数据
    for (int i = 0; i < scans.size(); i++)
    {
        GeneralLaserScan scan = scans[i];
        Eigen::Vector3d robotPose = robot_poses[i];

        //机器人的下标
        GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));

        for (int id = 0; id < scan.range_readings.size(); id++)
        {
            double dist = scan.range_readings[id];
            double angle = -scan.angle_readings[id]; // 激光雷达逆时针转，角度取反

            if (std::isinf(dist) || std::isnan(dist))
                continue;

            //计算得到该激光点的世界坐标系的坐标
            double theta = -robotPose(2);           // 激光雷达逆时针转，角度取反
            double laser_x = dist * cos(angle);
            double laser_y = dist * sin(angle);

            double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);
            double world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);

            //start of TODO 对对应的map的cell信息进行更新．（1,2,3题内容）
            GridIndex pointIndex = ConvertWorld2GridIndex(world_x, world_y);
            std::vector<GridIndex> trace_line = TraceLine(robotIndex.x, robotIndex.y, pointIndex.x, pointIndex.y);

            // 建图方法一: 使用覆盖栅格建图算法 (不使用代码内给定的函数TraceLine)
            {
                /*           
                GridIndex pointIndex = ConvertWorld2GridIndex(world_x, world_y);

                int u = robotIndex.x;
                while(u != pointIndex.x)
                {
                    double v_double = (pointIndex.y - robotIndex.y)/(pointIndex.x - robotIndex.x) 
                                                                * (u - robotIndex.x) + robotIndex.y;
                    int v_int; 

                    if(robotIndex.x < pointIndex.x)
                    {
                        v_int = std::floor(v_double);
                        ++u;
                    }
                    else
                    {
                        v_int = std::ceil(v_double);
                        --u;
                    }

                    GridIndex gd;
                    gd.SetIndex(u, v_int);
                    int linear_index = GridIndexToLinearIndex(gd);

                    if(!isValidGridIndex(gd))
                        continue;

                    pMap[linear_index] += mapParams.log_free;

                    pMap[linear_index] = (pMap[linear_index] > mapParams.log_min) ? pMap[linear_index] : mapParams.log_min; 
                }

                int v = robotIndex.y;
                while(v != pointIndex.y)
                {
                    double u_double = (pointIndex.x - robotIndex.x)/(pointIndex.y - robotIndex.y) 
                                                            * (v - robotIndex.y) + robotIndex.x;
                    int u_int;

                    if(robotIndex.y < pointIndex.y)
                    {
                        u_int = std::floor(u_double);
                        ++v;
                    }
                    else 
                    {
                        u_int = std::ceil(u_double);
                        --v;
                    }

                    GridIndex gd;
                    gd.SetIndex(u_int, v);
                    int linear_index = GridIndexToLinearIndex(gd);

                    if(!isValidGridIndex(gd))
                        continue;

                    pMap[linear_index] += mapParams.log_free; 
                    pMap[linear_index] = (pMap[linear_index] > mapParams.log_min) ? pMap[linear_index] : mapParams.log_min;
                } 

                GridIndex gd;
                gd.SetIndex(pointIndex.x, pointIndex.y);
                int linear_index = GridIndexToLinearIndex(gd);

                if(!isValidGridIndex(gd))
                        continue;

                pMap[linear_index] += mapParams.log_occ;
                pMap[linear_index] = (pMap[linear_index] < mapParams.log_max) ? pMap[linear_index] : mapParams.log_max; 
                */  
            }              
            
            
            if(mapping_method == 1)
            {
                
                // 建图方法一: 使用覆盖栅格建图算法 (使用代码内给定的函数TraceLine)
                // 被激光穿过的栅格
                for(int k = 0; k < trace_line.size(); ++k)
                {
                    if(isValidGridIndex(trace_line[k]))
                    {
                        int linear_index = GridIndexToLinearIndex(trace_line[k]);
                        pMap[linear_index] += mapParams.log_free;
                        pMap[linear_index] = pMap[linear_index] > mapParams.log_min 
                                            ? pMap[linear_index] : mapParams.log_min;
                    }
                }

                // 被激光击中的栅格
                if(isValidGridIndex(pointIndex))
                {
                    int linear_index = GridIndexToLinearIndex(pointIndex);
                    pMap[linear_index] += mapParams.log_occ;
                    pMap[linear_index] = pMap[linear_index] < mapParams.log_max 
                                        ? pMap[linear_index] : mapParams.log_max;
                }
            }
            else if(mapping_method == 2)
            {
                // 建图方法3: 计数法
                // 被激光穿过的栅格
                for(int k = 0; k < trace_line.size(); ++k)
                {
                    if(isValidGridIndex(trace_line[k]))
                    {
                        int linear_index = GridIndexToLinearIndex(trace_line[k]);
                        pMapMisses[linear_index] += 1;
                    }
                }

                // 被激光击中的栅格
                if(isValidGridIndex(pointIndex))
                {
                    int linear_index = GridIndexToLinearIndex(pointIndex);
                    pMapHits[linear_index] += 1;
                }

            }
            else if(mapping_method == 3)
            {
                // 方法3: TSDF建图
                double TSDF, tsdf;              // TSDF: 融合更新后的值    tsdf: 当前值
                double x, y;                    // 世界坐标系下的坐标
                double delta_x, delta_y;
                double dist_xy;                 // 点(x,y)距离传感器原点的距离
                double t = 0.5;                 // 截断距离
                for(int k = 0; k < trace_line.size(); ++k)
                {
                    if(isValidGridIndex(trace_line[k]))
                    {
                        //栅格点需要先转换为世界坐标，才能接下去计算实际距离
                        ConvertGridIndex2World(trace_line[k], x, y);
                        //计算点(x,y)距离传感器原点的实际距离
                        delta_x = x - robotPose(0);
                        delta_y = y - robotPose(1);
                        dist_xy = std::pow(std::pow(delta_x, 2) + std::pow(delta_y, 2), 0.5);
                        tsdf = std::max(-1.0, std::min(1.0, (dist-dist_xy)/t));
                        //二维栅格的一维序号
                        int pose_num = GridIndexToLinearIndex(trace_line[k]);
                        //TSDFi(x)
                        pMapTSDF[pose_num] = (pMapW[pose_num]*pMapTSDF[pose_num] + tsdf)/(pMapW[pose_num]+1);
                        //Wi(x)
                        ++pMapW[pose_num];
                    }
                } 
            }
            else
            {
                std::cerr << "Please enter a correct mapping method!";
            }

            //end of TODO
        }
    }

    //start of TODO 通过计数建图算法或TSDF算法对栅格进行更新（2,3题内容）
    if(mapping_method == 1)
    {    
        // 方法1: 占用栅格，这里不做处理
    }
    else if(mapping_method == 2)
    {
        // 方法2: 计数建图
        for(int k = 0; k < mapParams.width * mapParams.height; ++k)
        {
            if(pMapHits[k] != 0 || pMapMisses[k] != 0)     // 只处理被击中过或被miss过的栅格位置
            {
                double occu_rate = pMapHits[k]/(pMapHits[k] + pMapMisses[k]);
                pMap[k] = static_cast<u_char>(std::ceil(occu_rate * mapParams.log_max));
            }
        }
    }
    else if(mapping_method == 3)
    {
        // 方法3: TSDF建图
        // 使用插值计算障碍物位置
        for(int m = 0; m < mapParams.height-2; m++)     //上下方向(即x方向)上搜索
        {
            for(int n = 0; n < mapParams.width-2; n++)  //左右方向(即y方向)上搜索 
            {
                GridIndex index;
                index.SetIndex(m, n);
                int linear_index = GridIndexToLinearIndex(index);       // 遍历到的点的一维搜索序号
                int linear_x = linear_index + mapParams.width;          // 沿着x轴方向向下移动一格
                int linear_y = linear_index + 1;                        // 沿着y轴方向向右移动一格
                
                //计算 点A 和 点B，用于插值
                double A_x, A_y;                        // 遍历到的点 A = (A_x, A_y)
                ConvertGridIndex2World(index, A_x, A_y);
                double B_x, B_y;                        // 遍历到的点的右下方的点 B = (B_x, B_y)
                index.SetIndex(m+1, n+1);
                ConvertGridIndex2World(index, B_x, B_y);

                double a, bx, by, x, y;
                a = pMapTSDF[linear_index];  
                bx = pMapTSDF[linear_x];    // 沿着x轴方向向下移动一格的TSDF值
                by = pMapTSDF[linear_y];    // 沿着y轴方向向下移动一格的TSDF值

                if( a * by < 0)             // 判断y方向上的TSDF值是否存在符号变化
                {
                    x = A_x;                                // x方向不插值
                    y = interpolation(A_y, B_y, a, by);     // 对 A_y和B_y 根据 a和by 进行插值，得到y方向上的值
                    pMap[GridIndexToLinearIndex(ConvertWorld2GridIndex(x,y))] = 100;  
                }
                else if( a * bx < 0 )        // 判断x方向上的TSDF值是否存在符号变化
                {
                    x = interpolation(A_x, B_x, a, bx);     // 对 A_x和B_x 根据 a和bx 进行插值，得到y方向上的值
                    y = A_y;                                // y方向不插值
                    pMap[GridIndexToLinearIndex(ConvertWorld2GridIndex(x,y))] = 100;
                }
                   
            }
        }
    }
    else
    {
        std::cerr << "Please enter a correct mapping method!";
    }




    //end of TODO
    std::cout << "建图完毕" << std::endl;
}

//发布地图．
void PublishMap(ros::Publisher &map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    //0~100
    int cnt0, cnt1, cnt2;
    cnt0 = cnt1 = cnt2 = 100;
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        if (pMap[i] == 50)
        {
            rosMap.data[i] = -1.0;      // Unknown Cells
        }
        else
        {

            rosMap.data[i] = pMap[i];
        }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OccupanyMapping");

    ros::NodeHandle nodeHandler;

    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map", 1, true);

    std::vector<Eigen::Vector3d> robotPoses;
    std::vector<GeneralLaserScan> generalLaserScans;

    std::string basePath = "/home/jindong/Lidar_SLAM/Exercise/ch7/HW7/OccupanyMappingProject/src/data";

    std::string posePath = basePath + "/pose.txt";
    std::string anglePath = basePath + "/scanAngles.txt";
    std::string scanPath = basePath + "/ranges.txt";

    //读取数据
    ReadPoseInformation(posePath, robotPoses);

    ReadLaserScanInformation(anglePath,
                             scanPath,
                             generalLaserScans);

    std::cout << "***********************************" << std::endl;   
    std::cout << "Pose number: " << robotPoses.size() << std::endl;
    std::cout << "Angle number at each pose: " << generalLaserScans[1].angle_readings.size() << std::endl;
    std::cout << "Range number at each pose: " << generalLaserScans[1].range_readings.size() << std::endl;
    std::cout << "***********************************" << std::endl;

    std::cout << "Please enter the mapping method you want to use:" << std::endl;
    std::cout << "[1: occupancy grid, 2: count model, 3: TSDF]" << std::endl;
    std::cin >> mapping_method;
    
    //设置地图信息
    SetMapParams();

    OccupanyMapping(generalLaserScans, robotPoses);

    PublishMap(mapPub);

    ros::spin();    // ?

    DestoryMap();

    std::cout << "Release Memory!!" << std::endl;
}
