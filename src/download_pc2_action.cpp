/*
 * Updates made by Anja Sheppard, Fall 2021.
 *
 * Copyright (c) 2017-2019, Andrea Pagani, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 */

#include "download_pc2_action.h"

/**
 * @brief DownloadPC2_Action Constructor.
 * @param name ROS Action Server name.
 * @param eb_ ElasticBridge pointer.
 * @param ROS node handle.
 */
DownloadPC2_Action::DownloadPC2_Action (const std::string name, ElasticBridge* eb_, ros::NodeHandle& nh) :
    m_nh(nh),
    m_as(m_nh, name, boost::bind(&DownloadPC2_Action::executeCB, this, _1), false)
{
    eb = eb_;
    m_as.start();
}

/**
 * @brief Callback for Action Server.
 * @param goal Action data.
 */
void DownloadPC2_Action::executeCB (const elastic_bridge::downloadPointcloud2GoalConstPtr& goal)
{
    if (goal->stop)
    {
        eb->scanFinishCallback(std_msgs::EmptyConstPtr(new std_msgs::Empty()));
    }

    elastic_bridge::downloadPointcloud2Result result;

    sensor_msgs::PointCloud2ConstPtr pointCloud2 = eb->requestDownload(result.guids, result.luids);
    if (!pointCloud2)
    {
      m_as.setAborted();
      return;
    }

    result.pointcloud = *pointCloud2;
    m_as.setSucceeded(result);
}
