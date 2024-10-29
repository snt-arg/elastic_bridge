/*
 * Updates made by Anja Sheppard, Fall 2021.
 *
 * Copyright (c) 2017-2019, Andrea Pagani, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 */

#pragma once

#include <elastic_bridge/FrameState.h>
#include <init_fake_opengl_context/fake_opengl_context.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <ElasticFusion.h>
#include "GlobalModel.h"
#include "IndexMap.h"
#include "GPUTexture.h"
#include "pangolin/pangolin.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <elastic_bridge/Suspend.h>

//! @brief ElasticBridge is a class for running a ROS package version of ElasticFusion.
class ElasticBridge
{
public:
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageFilterSubscriber;
    typedef boost::shared_ptr<ImageFilterSubscriber> ImageFilterSubscriberPtr;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ATSyncPolicy;
    typedef message_filters::Synchronizer<ATSyncPolicy> ATSynchronizer;
    typedef boost::shared_ptr<ATSynchronizer> ATSynchronizerPtr;

private:
    ElasticFusion* m_eFusion = NULL; //! @brief ElasticFusion pointer.
   
    /* ROS variables */
    ros::NodeHandle m_nh; //! @brief ROS node handle.
    ros::Subscriber m_camera_info_sub;
    ros::Publisher m_periodic_cloud_pub, m_image_pub;
    ros::Publisher m_frame_state_stable_pub, m_camera_pose_pub;
    tf::TransformBroadcaster m_transform_broadcaster;
    tf::TransformListener m_transform_listener;
    ImageFilterSubscriberPtr m_imageColor_sub, m_imageDepth_sub;
    ATSynchronizerPtr m_sync_sub;

    std::string m_topic_image_color;
    std::string m_topic_image_depth;
    std::string m_topic_camera_info;
    std::string m_topic_periodic_world_pub;
    std::string m_world_frame;
    std::string m_camera_frame;
    std::string m_input_world_frame;
    std::string m_input_camera_frame;

    sensor_msgs::ImageConstPtr m_image_color; // NULL if not ready
    sensor_msgs::ImageConstPtr m_image_depth; // NULL if not ready
    sensor_msgs::CameraInfoConstPtr m_camera_info; // NULL if not ready

    /* Camera sensor calibration variables */
    int m_frame_count = 0; //! @brief Number of frames since run start.
    unsigned int m_height; //! @brief Frame height (px).
    unsigned int m_width; //! @brief Frame width (px).
    float m_focal_x; //! @brief Camera focal length x (mm).
    float m_focal_y; //! @brief Camera focal length y (mm).
    float m_center_x; //! @brief Camera principal point x (mm).
    float m_center_y; //! @brief Camera principal point x (mm).
    bool m_camera_info_received = false; //! @brief True once camera calibration info has been received.
    const uint32_t TEXTURE_PIXEL_SIZE = 4;
    const uint32_t OUTPUT_PIXEL_SIZE = 4;
    
    boost::mutex m_mutex;
    boost::condition_variable m_cond_var;

    /* NH parameters */
    bool m_pose_from_tf_always = false; //! @brief When true, sensor pose is read from TF.
    bool m_pose_from_tf_first = false; //! @brief When true, sensor pose is read from TF for the first frame only.
    
    bool m_running = true; //! @brief True when ElasticFusion is running.

    bool m_black_to_nan = false; //! @brief When true, set black pixels to have 0 depth.

    std::string m_display_name; //! @brief Window name for fake openGL display.

    uint32_t m_periodic_world_publication; //! @brief When true, publishes a global model during execution.

    /* GUID variables */
    std::vector<uint64_t> m_guids;
    std::vector<uint32_t> m_luids;
    std::vector<uint64_t> m_guid_assoc;
    uint64_t m_guid_counter;

    /* Download PC variables */
    sensor_msgs::PointCloud2ConstPtr m_point_cloud_for_download; //! @brief PointCloud2 for download.
    uint32_t m_download_request = false;
    uint32_t m_download_ready = false;

public:
    ElasticBridge (ros::NodeHandle& nh);
    ~ElasticBridge ();

    void sendTF (const Eigen::Matrix4f& pose, std::string from, std::string to);
    Eigen::Matrix4f readTF ();

    void publishFrameState (const std::vector<uint16_t>& depth_data, const std::vector<uint8_t>& rgb_data,
                            pangolin::GlTexture* guid_texture,
                            pangolin::GlTexture* image_texture, pangolin::GlTexture* vertex_texture,
                            pangolin::GlTexture* normal_texture,const Eigen::Affine3f& pose,
                            const std::vector<uint32_t>& disposed_luids,
                            ros::Publisher& posePub, ros::Publisher& statePub, const ros::Time& stamp);

    void publishFrame (const Eigen::Matrix4f& curr_pose,
                       const std::vector<uint16_t>& depth_data,const std::vector<uint8_t>& rgb_data,
                       const ros::Time& stamp);
    void imagePublish (const int n_pixel, std::vector<float>& image_vec);
    
    sensor_msgs::PointCloud2ConstPtr requestDownload (std::vector<uint64_t>& guids, std::vector<uint32_t>& luids);

    sensor_msgs::PointCloud2ConstPtr getPC2 (std::vector<uint64_t>* guids = NULL, std::vector<uint32_t>* luids = NULL);

    void publishWorldExec ();

    void cameraInfoCallbackWorker (const sensor_msgs::CameraInfoConstPtr& msg);
    void cameraInfoCallback (const sensor_msgs::CameraInfoPtr& msg);

    void imagesCallbackWorker (const sensor_msgs::ImageConstPtr& image_color,
                               const sensor_msgs::ImageConstPtr& image_depth);
    void imagesCallback (const sensor_msgs::ImageConstPtr& image_color, const sensor_msgs::ImageConstPtr& image_depth);

    bool suspend (elastic_bridge::Suspend::Request &req, elastic_bridge::Suspend::Response &res);

    void initElasticFusion (unsigned int height, unsigned int width, float fx, float fy, float cx, float cy);

    void run ();
    void init ();
};
