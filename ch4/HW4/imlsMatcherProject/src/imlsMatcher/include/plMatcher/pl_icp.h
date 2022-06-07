#ifndef PL_ICP_H
#define PL_ICP_H

#include "boost/asio.hpp"   //包含boost库函数
#include "boost/bind.hpp"
#include <iostream>
#include <vector>
#include <set>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/unsupported/Eigen/Polynomials>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sys/time.h>
#include <math.h>

#include <csm/csm_all.h>
#include "champion_nav_msgs/ChampionNavLaserScan.h"

class PLICPMatcher
{
public:
    PLICPMatcher();

    //进行pl-icp的相关函数.
    void SetPLICPParams();
    void LaserScanToLDP(const champion_nav_msgs::ChampionNavLaserScanConstPtr& pScan, LDP& ldp);
    Eigen::Vector3d  PLICPBetweenTwoFrames(LDP& currentLDPScan,
                                           Eigen::Vector3d tmprPose);

    //进行PI-ICP需要的变量
    LDP m_prevLDP;
    sm_params m_PLICPParams;
    sm_result m_OutputResult;

};




















#endif // PL_ICP_H