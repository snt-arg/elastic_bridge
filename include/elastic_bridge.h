/*
 * Updates made by Anja Sheppard, Fall 2021.
 *
 * Copyright (c) 2017-2019, Andrea Pagani, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 */

#pragma once

#include <init_fake_opengl_context/fake_opengl_context.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <elastic_bridge/FrameState.h>

//c++
#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

//ElasticFusion
#include <ElasticFusion.h>
#include "GlobalModel.h"
#include "IndexMap.h"
#include "GPUTexture.h"
#include "pangolin/pangolin.h"

//Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

//Pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


//! @brief ElasticBridge is a class for running a ROS package version of ElasticFusion.
class ElasticBridge {
public:
    typedef uint32_t uint32;
    typedef uint16_t uint16;
    typedef uint8_t uint8;
    typedef uint64_t uint64;
    typedef std::vector<uint64> Uint64Vector;
    typedef std::vector<uint32> Uint32Vector;
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageFilterSubscriber;
    typedef boost::shared_ptr<ImageFilterSubscriber> ImageFilterSubscriberPtr;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ATSyncPolicy;
    typedef message_filters::Synchronizer<ATSyncPolicy> ATSynchronizer;
    typedef boost::shared_ptr<ATSynchronizer> ATSynchronizerPtr;

private:
    int m_frame_count = 0;
    int m_height, m_width;
    float m_center_x,m_center_y,m_focal_x,m_focal_y;
    std::string m_TopicImageColor, m_TopicImageDepth, m_TopicCameraInfo,
      m_topic_periodic_world_pub,
      m_world_frame, m_camera_frame, m_input_world_frame, m_input_camera_frame;
    bool m_cameraInfoOK = false;
    bool m_poseFromTFAlways = false;
    bool m_poseFromTFFirst = false;
    Eigen::Matrix4f first_pose;
    
    ElasticFusion* m_eFusion = NULL;

    boost::mutex m_mutex;
    boost::condition_variable m_cond_var;

    uint32 m_download_request = false;
    uint32 m_download_ready = false;

    sensor_msgs::ImageConstPtr m_image_color; // NULL if not ready
    sensor_msgs::ImageConstPtr m_image_depth; // NULL if not ready
    sensor_msgs::CameraInfoConstPtr m_camera_info; // NULL if not ready
    
    bool m_started = true;

    bool m_black_to_nan = false;

    sensor_msgs::PointCloud2ConstPtr m_point2;
    Uint64Vector m_guids;
    Uint32Vector m_luids;

    ros::NodeHandle m_nh;
    ros::Subscriber cameraInfo_sub, scanReady_sub, scanFinish_sub;
    ros::Publisher m_periodic_cloud_pub, m_image_pub;
    ros::Publisher m_frame_state_stable_pub;
    tf::TransformBroadcaster m_transformBroadcaster;
    tf::TransformListener m_transformListener;
    ImageFilterSubscriberPtr m_imageColor_sub, m_imageDepth_sub;
    ATSynchronizerPtr m_sync_sub;

    std::string m_display_name;

    uint32 m_periodic_world_publication;

    std::vector<uint64> m_guid_assoc;
    uint64 m_guid_counter;

public:
    ElasticBridge (ros::NodeHandle& nh);
    ~ElasticBridge ();

    void sendTF (const Eigen::Matrix4f& pose, std::string from, std::string to);
    Eigen::Matrix4f readTF ();

    void publishFrameState (const std::vector<uint16>& depth_data, const std::vector<uint8>& rgb_data,
                            pangolin::GlTexture* guid_texture,
                            pangolin::GlTexture* image_texture, pangolin::GlTexture* vertex_texture,
                            pangolin::GlTexture* normal_texture,const Eigen::Affine3f& pose,
                            const std::vector<uint32>& disposed_luids,
                            ros::Publisher& pub);

    void imagePublish (const Eigen::Matrix4f& currPose,
                       const std::vector<uint16>& depth_data,const std::vector<uint8>& rgb_data);
    
    sensor_msgs::PointCloud2ConstPtr requestDownload (Uint64Vector& guids, Uint32Vector& luids);

    sensor_msgs::PointCloud2ConstPtr getPC2 (Uint64Vector* guids = NULL, Uint32Vector* luids = NULL);

    void publishWorldExec ();

    void scanReadyCallback (const std_msgs::EmptyConstPtr& msg);
    void scanFinishCallback (const std_msgs::EmptyConstPtr& msg);

    void cameraInfoCallbackWorker (const sensor_msgs::CameraInfoConstPtr& msg);
    void cameraInfoCallback (const sensor_msgs::CameraInfoPtr& msg);

    void ImagesCallbackWorker (const sensor_msgs::ImageConstPtr& imageColor,
                               const sensor_msgs::ImageConstPtr& imageDepth);
    void ImagesCallback (const sensor_msgs::ImageConstPtr& imageColor, const sensor_msgs::ImageConstPtr& imageDepth);

    void InitElasticFusion (int Height, int Width, float fx, float fy, float cx, float cy);

    void run ();
    void init ();
};
