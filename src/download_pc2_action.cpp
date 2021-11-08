#include "download_pc2_action.h"

DownloadPC2_Action::DownloadPC2_Action (const std::string name, ElasticBridge * eb_, ros::NodeHandle & nh) :
    m_nh(nh),
    m_as(m_nh, name, boost::bind(&DownloadPC2_Action::executeCB, this, _1), false)
    {
    eb = eb_;
    m_as.start();
}

void DownloadPC2_Action::executeCB (const elastic_bridge::downloadPointcloud2GoalConstPtr &goal)
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
