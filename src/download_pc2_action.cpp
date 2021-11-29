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
 * @param eb ElasticBridge pointer.
 * @param ROS node handle.
 */
DownloadPC2_Action::DownloadPC2_Action (const std::string name, ElasticBridge* eb, ros::NodeHandle& nh) :
    m_eb(eb),
    m_nh(nh),
    m_as(m_nh, name, boost::bind(&DownloadPC2_Action::executeCB, this, _1), false)
{
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
        ros::ServiceClient client = m_nh.serviceClient<elastic_bridge::Suspend>("suspend");
        elastic_bridge::Suspend srv;
        srv.request.suspend = true;
        client.call(srv);
    }

    elastic_bridge::downloadPointcloud2Result result;

    sensor_msgs::PointCloud2ConstPtr point_cloud2 = m_eb->requestDownload(result.guids, result.luids);
    if (!point_cloud2)
    {
      m_as.setAborted();
      return;
    }

    result.pointcloud = *point_cloud2;
    m_as.setSucceeded(result);
}
