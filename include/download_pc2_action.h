#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <actionlib/server/simple_action_server.h>

#include "elastic_bridge.h"
#include "elastic_bridge/downloadPointcloud2Action.h"

//! @brief DownloadPC2_Action is a class for retrieving point cloud data after running ElasticFusion.
class DownloadPC2_Action
{
protected:
    ros::NodeHandle & m_nh;
    actionlib::SimpleActionServer<elastic_bridge::downloadPointcloud2Action> m_as;
    ElasticBridge * eb;

public:
    DownloadPC2_Action (const std::string name, ElasticBridge * eb_, ros::NodeHandle & nh);
    void executeCB (const elastic_bridge::downloadPointcloud2GoalConstPtr &goal);
};
