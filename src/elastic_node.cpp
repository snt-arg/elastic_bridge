/*
 * Updates made by Anja Sheppard, Fall 2021.
 *
 * Copyright (c) 2017-2019, Andrea Pagani, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 */

#include "download_pc2_action.h"
#include "elastic_bridge.h"

/**
 * @brief Constructor for ElasticBridge instance.
 * @param nh ROS node handle.
 */
ElasticBridge::ElasticBridge (ros::NodeHandle& nh) : m_nh(nh)
{
    init();
}

/**
 * @brief Destructor for ElasticBridge instance.
 */
ElasticBridge::~ElasticBridge ()
{
    if (m_eFusion)
        delete m_eFusion;
}

/**
 * @brief Broadcast transform between two frames to ROS.
 * @param pose Transform between the two frames.
 * @param from Parent coordinate frame.
 * @param to Child coordinate frame.
 */
void ElasticBridge::sendTF (const Eigen::Matrix4f& pose, std::string from, std::string to)
{
    tf::Transform transform;
    Eigen::Affine3d affine;
    affine.matrix() = pose.cast<double>();
    tf::transformEigenToTF(affine, transform);
    const ros::Time& time = ros::Time::now();
    m_transform_broadcaster.sendTransform(tf::StampedTransform(transform, time, from, to));
}

/**
 * @brief Look up transform between the input world frame and the input camera frame.
 */
Eigen::Matrix4f ElasticBridge::readTF ()
{
    tf::StampedTransform transform;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    try
    {
        m_transform_listener.lookupTransform(m_input_world_frame, m_input_camera_frame, ros::Time(0), transform);
        Eigen::Affine3d affine;
        tf::transformTFToEigen(transform, affine);
        pose = affine.matrix().cast<float>();

    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("elastic_bridge: could not read input TF: %s", ex.what());
    }
    return pose;
}

/**
 * @brief Populate FrameState message and publish.
 * @param depth_data Depth data per pixel for the frame stored in a 1D array of length frame height * width.
 * @param rgb_data Color data per pixel for the frame stored in a 1D array of length frame height * width.
 * @param guid_texture Texture object for GUID extension.
 * @param image_texture Image texture.
 * @param vertex_texture Vertex texture.
 * @param normal_texture Normal texture.
 * @param pose Current sensor pose.
 * @param disposed_luids Destroyed luids in this frame.
 * @param pub ROS publisher that the FrameState message is published on.
 */
void ElasticBridge::publishFrameState (const std::vector<uint16_t>& depth_data,
                                       const std::vector<uint8_t>& rgb_data,
                                       pangolin::GlTexture* guid_texture,
                                       pangolin::GlTexture* image_texture,
                                       pangolin::GlTexture* vertex_texture,
                                       pangolin::GlTexture* normal_texture,
                                       const Eigen::Affine3f& pose,
                                       const std::vector<uint32_t>& disposed_luids,
                                       ros::Publisher& pub)
{
    if (pub.getNumSubscribers() == 0)
        return;
    
    const uint64_t size = m_width * m_height;
    
    std::vector<uint32_t> guid_vec(size, 127);
#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
    guid_texture->Download(guid_vec.data(), GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_INT);
#endif
    
    std::vector<float> image_vec(size * 4, 127);
    image_texture->Download(image_vec.data(), GL_BGRA, GL_FLOAT);
    
    std::vector<float> vertex_vec(size * 4, 127);
    vertex_texture->Download(vertex_vec.data(), GL_RGBA, GL_FLOAT);
    
    std::vector<float> normal_vec(size * 4, 127);
    normal_texture->Download(normal_vec.data(), GL_RGBA, GL_FLOAT);
    
    elastic_bridge::FrameState state;
    
    state.width = m_width;
    state.height = m_height;
    state.seq = m_frame_count;
    
    state.focal_x = m_focal_x;
    state.center_x = m_center_x;
    state.focal_y = m_focal_y;
    state.center_y = m_center_y;
    
    state.input_color.resize(size * 3);
    state.input_depth.resize(size);
    state.color.resize(size * 3);
    state.position.resize(size * 3);
    state.depth.resize(size);
    state.luid.resize(size);
    state.guid.resize(size);
    state.normal.resize(size * 3);
    state.radius.resize(size);
    state.luid_removed = disposed_luids;

    for (uint64_t i = 0; i < size; i++)
    {
        state.input_color[i * 3 + 2] = rgb_data[i * 3 + 0];
        state.input_color[i * 3 + 1] = rgb_data[i * 3 + 1];
        state.input_color[i * 3 + 0] = rgb_data[i * 3 + 2];
        
        state.input_depth[i] = depth_data[i];
        
        state.color[i * 3 + 0] = image_vec[i * 4 + 0];
        state.color[i * 3 + 1] = image_vec[i * 4 + 1];
        state.color[i * 3 + 2] = image_vec[i * 4 + 2];
        
        state.position[i * 3 + 0] = vertex_vec[i * 4 + 0];
        state.position[i * 3 + 1] = vertex_vec[i * 4 + 1];
        state.position[i * 3 + 2] = vertex_vec[i * 4 + 2];
        
        state.normal[i * 3 + 0] = normal_vec[i * 4 + 0];
        state.normal[i * 3 + 1] = normal_vec[i * 4 + 1];
        state.normal[i * 3 + 2] = normal_vec[i * 4 + 2];
        state.radius[i] = normal_vec[i * 4 + 3];
        
        state.depth[i] = vertex_vec[i * 4 + 2];
#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
        const uint32_t luid = guid_vec[i];
#else
        const uint32_t luid = 0;
#endif
        state.luid[i] = luid;
        state.guid[i] = luid ? m_guid_assoc[luid - 1] : 0;
    }
    state.max_luid = m_guid_assoc.size();
    
    tf::poseEigenToMsg(pose.cast<double>(), state.pose);
    
    pub.publish(state);
}

void ElasticBridge::publishFrame (const Eigen::Matrix4f& curr_pose,
                   const std::vector<uint16_t>& depth_data,
                   const std::vector<uint8_t>& rgb_data)
{
    const int n_pixel = m_height * m_width;

    pangolin::GlTexture * image_texture = m_eFusion->getIndexMap().imageTex()->texture;

#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
    pangolin::GlTexture * guid_texture = m_eFusion->getIndexMap().guidTex()->texture;
#else
    pangolin::GlTexture * guid_texture = NULL;
#endif

    pangolin::GlTexture * vertex_texture = m_eFusion->getIndexMap().vertexTex()->texture;

    pangolin::GlTexture * normal_texture = m_eFusion->getIndexMap().normalTex()->texture;

    std::vector<float> image_vec(n_pixel * 4, 127);
    image_texture->Download(image_vec.data(), GL_BGRA, GL_FLOAT);

    std::vector<uint32_t> guid_vec(n_pixel, 127);
#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
    guid_texture->Download(guid_vec.data(), GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_INT);
#endif

    std::vector<float> bimage_vec(n_pixel * 4, 127);

#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
    const std::vector<GLuint> disposed_guids = m_eFusion->getGlobalModel().downloadDisposedGuids(true);
#else
    const std::vector<GLuint> disposed_guids;
#endif

    imagePublish(n_pixel, image_vec);

    for (uint64_t i = 0; i < guid_vec.size(); i++)
    {
        const uint32_t internal_guid = guid_vec[i];
        if (!internal_guid)
            continue;

        if (internal_guid > m_guid_assoc.size())
            m_guid_assoc.resize(internal_guid,0);

        if (m_guid_assoc[internal_guid - 1] == 0)
            m_guid_assoc[internal_guid - 1] = ++m_guid_counter;
    }

    for (uint64_t i = 0; i < disposed_guids.size(); i++)
    {
        const uint32_t disposed_guid = disposed_guids[i];
        if (!disposed_guid)
            continue;

        if (disposed_guid > m_guid_assoc.size())
            continue;

        m_guid_assoc[disposed_guid - 1] = 0;
    }

    Eigen::Affine3f pose;
    pose.matrix() = curr_pose;

    publishFrameState(depth_data, rgb_data, guid_texture, image_texture, vertex_texture, normal_texture,
                      pose, disposed_guids, m_frame_state_stable_pub);
}

/**
 * @brief Populate/publish image (frame) and call function to publish FrameState.
 * @param curr_pose Current sensor pose.
 * @param depth_data Depth data per pixel for the frame stored in a 1D array of length frame height * width.
 * @param rgb_data Color data per pixel for the frame stored in a 1D array of length frame height * width.
 */
void ElasticBridge::imagePublish (const int n_pixel, std::vector<float>& image_vec)
{
    sensor_msgs::Image image_msg;
    image_msg.height = m_height;
    image_msg.width = m_width;
    image_msg.encoding = "rgba8";
    image_msg.is_bigendian = false;
    image_msg.step = image_msg.width * OUTPUT_PIXEL_SIZE;
    image_msg.data.resize(n_pixel * OUTPUT_PIXEL_SIZE);
    
    for (unsigned int y = 0; y < m_height; y++)
    {
          for (unsigned int x = 0; x < m_width; x++)
          {
              const int i = y * m_width + x;

              image_msg.data[i * OUTPUT_PIXEL_SIZE + 0] = image_vec[i * TEXTURE_PIXEL_SIZE + 0] * 255;
              image_msg.data[i * OUTPUT_PIXEL_SIZE + 1] = image_vec[i * TEXTURE_PIXEL_SIZE + 1] * 255;
              image_msg.data[i * OUTPUT_PIXEL_SIZE + 2] = image_vec[i * TEXTURE_PIXEL_SIZE + 2] * 255;
              image_msg.data[i * OUTPUT_PIXEL_SIZE + 3] = 255;
          }
    }
    
    m_image_pub.publish(image_msg);
}

/**
 * @brief Send request for point cloud download.
 * @param guids Vector of guids.
 * @param luids Vector of luids.
 */
sensor_msgs::PointCloud2ConstPtr ElasticBridge::requestDownload (std::vector<uint64_t>& guids, std::vector<uint32_t>& luids)
{
    boost::mutex::scoped_lock lock(m_mutex);

    // wait until previous requests are satisfied
    while (m_download_request && !ros::isShuttingDown())
    {
        m_cond_var.wait(lock);
    }

    m_download_request = true;
    m_cond_var.notify_all();

    if (ros::isShuttingDown())
      return m_point_cloud_for_download;

    // wait until this request is satisfied
    while (!m_download_ready && !ros::isShuttingDown())
    {
        m_cond_var.wait(lock);
    }

    m_download_ready = false;
    m_download_request = false;
    sensor_msgs::PointCloud2ConstPtr point2 = m_point_cloud_for_download;
    guids.swap(m_guids);
    luids.swap(m_luids);
    m_guids.clear();
    m_luids.clear();
    m_point_cloud_for_download.reset();
    m_cond_var.notify_all();

    return point2;
}

/**
 * @brief Retrieve point cloud data from ElasticFusion and populate PointCloud2 message.
 * @param guids Vector of guids.
 * @param luids Vector of luids.
 */
sensor_msgs::PointCloud2ConstPtr ElasticBridge::getPC2 (std::vector<uint64_t>* guids, std::vector<uint32_t>* luids)
{
    Eigen::Vector4f* map_data = m_eFusion->getGlobalModel().downloadMap();

    const uint64_t last_count = m_eFusion->getGlobalModel().lastCount();

    pcl::PointCloud<pcl::PointSurfel> pcl_cloud;
    pcl_cloud.is_dense = true;
    pcl_cloud.height = 1;
    pcl_cloud.reserve(last_count);

    if (guids)
    {
        guids->clear();
        guids->reserve(last_count);
    }

    if (luids)
    {
        luids->clear();
        luids->reserve(last_count);
    }

    const float confidence_threshold = m_eFusion->getConfidenceThreshold();

    int count = 0;
    ROS_INFO("elastic_bridge: downloading %d points.", int(last_count));
    for (unsigned int i = 0; i < last_count; i++)
    {
        Eigen::Vector4f pos = map_data[(i * 3) + 0];

        if(pos[3] <= confidence_threshold)
          continue;

        pcl::PointSurfel point_surfel;

        Eigen::Vector4f col = map_data[(i * 3) + 1];
        Eigen::Vector4f nor = map_data[(i * 3) + 2];

        nor[0] *= -1;
        nor[1] *= -1;
        nor[2] *= -1;

        point_surfel.x = pos[0];
        point_surfel.y = pos[1];
        point_surfel.z = pos[2];

        unsigned char b = int(col[0]) >> 16 & 0xFF;
        unsigned char g = int(col[0]) >> 8 & 0xFF;
        unsigned char r = int(col[0]) & 0xFF;

        point_surfel.r = r;
        point_surfel.g = g;
        point_surfel.b = b;
        point_surfel.a = 255;
        point_surfel.normal_x = nor[0];
        point_surfel.normal_y = nor[1];
        point_surfel.normal_z = nor[2];
        point_surfel.radius = nor[3];

        pcl_cloud.points.push_back(point_surfel);

        const uint32_t internal_guid = col[1];
        const uint64_t guid = !internal_guid ? 0 : m_guid_assoc[internal_guid - 1];
        if (guids)
            guids->push_back(guid);
        if (luids)
            luids->push_back(internal_guid);

        count++;
    }

    delete [] map_data;

    pcl_cloud.width = count;

    ROS_INFO("elastic_bridge: sending %d points.", int(count));

    sensor_msgs::PointCloud2Ptr point_cloud2_ptr(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2& point_cloud2 = *point_cloud2_ptr;
    pcl::toROSMsg(pcl_cloud, point_cloud2);
    point_cloud2.header.stamp = ros::Time::now();
    point_cloud2.header.frame_id = m_world_frame;

    return point_cloud2_ptr;
}

/**
 * @brief Publish PointCloud2 message.
 */
void ElasticBridge::publishWorldExec ()
{
    sensor_msgs::PointCloud2ConstPtr point_cloud2 = this->getPC2();
    m_periodic_cloud_pub.publish(point_cloud2);
}

/**
 * @brief Receive camera info and initialize ElasticFusion.
 */
void ElasticBridge::cameraInfoCallbackWorker (const sensor_msgs::CameraInfoConstPtr& msg)
{
    if (m_camera_info_received)
      return; // already initialized

    ROS_INFO("elastic_bridge: first camera info received.");

    float fx, fy, cx, cy;
    const boost::array<double, 9> & K = msg->K;
    fx = K[0];
    fy = K[4];
    cx = K[2];
    cy = K[5];
    m_height = msg->height;
    m_width = msg->width;
    m_focal_x = fx;
    m_focal_y = fy;
    m_center_x = cx;
    m_center_y = cy;

    initElasticFusion(m_height, m_width, fx, fy, cx, cy);
    m_camera_info_received = true;
}

/**
 * @brief Callback for TOPIC_CAMERA_INFO that captures camera info data.
 * @param msg Message received from the topic.
 */
void ElasticBridge::cameraInfoCallback (const sensor_msgs::CameraInfoPtr& msg)
{
    boost::mutex::scoped_lock lock(m_mutex);
    m_camera_info = msg;
    m_cond_var.notify_all();
}

/**
 * @brief Image(frame) publish helper function.
 * @param imageColor Frame color message data.
 * @param imageDepth Frame depth message data.
 */
void ElasticBridge::imagesCallbackWorker (const sensor_msgs::ImageConstPtr& image_color,
                                          const sensor_msgs::ImageConstPtr& image_depth)
{
    if ((m_camera_info_received) && (m_running))
    {
        //ROS_INFO("elastic_bridge: received frame %d", int(m_frame_count));

        const std::string encoding = image_depth->encoding;
        const std::string color_encoding = image_color->encoding;
        const bool bigendian = image_depth->is_bigendian;

        ros::Time header_stamp = image_color->header.stamp;

        int n_pixel = m_height * m_width;

        int64_t time = header_stamp.toNSec();
        std::vector<uint16_t> depth_data(n_pixel);
        std::vector<uint8_t> rgb_data(n_pixel * 3);

        // Depth data decoding
        for (int r = 0; r < n_pixel; r++)
        {
            if (encoding == "32FC1")
            {
                float f;
                std::memcpy(&f, &(image_depth->data[4 * r]), sizeof (f));
                if (std::isfinite(f))
                  depth_data[r] = uint16_t(f * 1000);
                else
                  depth_data[r] = 0;
            }
            else if (encoding == "16UC1")
            {
                if (bigendian)
                    depth_data[r] = (image_depth->data[2 * r] * 256) + (image_depth->data[2 * r + 1]);
                else
                    depth_data[r] = (image_depth->data[2 * r]) + (image_depth->data[2 * r + 1] * 256);
            }
            else
            {
                ROS_ERROR("Unknown encoding: %s", encoding.c_str());
                return;
            }
        }

        // Color data decoding
        for (int i = 0; i < n_pixel; i++){
            if (color_encoding == "rgb8")
            {
                for (int h = 0; h < 3; h++)
                    rgb_data[i * 3 + h] = image_color->data[i * 3 + (2 - h)];
            }
            else if (color_encoding == "bgr8")
            {
                std::memcpy(&(rgb_data[i * 3]), &(image_color->data[i * 3]), 3);
            }
            else if (color_encoding == "bgra8")
            {
                std::memcpy(&(rgb_data[i * 3]), &(image_color->data[i * 4]), 4);
            }
            else
            {
                ROS_ERROR("Unknown color encoding: %s", color_encoding.c_str());
                return;
            }
        }

        if (m_black_to_nan)
        {
            for (int i = 0; i < n_pixel; i++)
            {
                if (rgb_data[i * 3 + 0] == 0 &&
                    rgb_data[i * 3 + 1] == 0 &&
                    rgb_data[i * 3 + 2] == 0)
                    depth_data[i] = 0;
            }
        }

        Eigen::Matrix4f curr_pose;
        if (m_pose_from_tf_first || m_pose_from_tf_always)
        {
            Eigen::Matrix4f pose = readTF();
            m_eFusion->processFrame(rgb_data.data(), depth_data.data(), time, &pose);
            curr_pose = pose;
        }
        else
        {
            m_eFusion->processFrame(rgb_data.data(), depth_data.data(), time);
            curr_pose = m_eFusion->getCurrPose();
        }

        publishFrame(curr_pose, depth_data, rgb_data);

        sendTF(curr_pose, m_world_frame, m_camera_frame);
        
        if (m_periodic_world_publication)
        {
            if ((m_frame_count % m_periodic_world_publication) + 1 == m_periodic_world_publication)
            {
                ROS_INFO("Publishing GlobalModel (during execution)");
                publishWorldExec();
                ROS_INFO("Published GlobalModel (during execution)");
            }
        }

        //ROS_INFO("elastic_bridge: processed frame %d, point count: %d", m_frame_count, m_eFusion->getGlobalModel().lastCount());
        m_frame_count++;
    }
}

/**
 * @brief Callback function for images received from the sensor.
 * @param imageColor Frame color message data.
 * @param imageDepth Frame depth message data.
 */
void ElasticBridge::imagesCallback (const sensor_msgs::ImageConstPtr& image_color, const sensor_msgs::ImageConstPtr& image_depth)
{
    boost::mutex::scoped_lock lock(m_mutex);
    m_image_color = image_color;
    m_image_depth = image_depth;
    m_cond_var.notify_all();
}

/**
 * @brief Callback function for suspend service. Pauses elastic fusion when "true" is sent. Unpauses when "false".
 * @param req Request containing true or false.
 * @param res Response detailing whether suspended or not.
 */
bool ElasticBridge::suspend (elastic_bridge::Suspend::Request &req, elastic_bridge::Suspend::Response &res)
{
    if (req.suspend == true) // pause node
    {
        ROS_INFO("elastic_bridge: stopped.");
        boost::mutex::scoped_lock lock(m_mutex);
        m_running = false;

        res.suspended = true;
    }
    else // un-pause node
    {
        ROS_INFO("elastic_bridge: started.");
        boost::mutex::scoped_lock lock(m_mutex);
        m_running = true;

        res.suspended = false;
    }

    return true;
}

/**
 * @brief Initialize ElasticFusion object for interacting with the ElasticFusion library.
 * @param height Frame height from the sensor.
 * @param width Frame width from the sensor.
 * @param fx Focal length x
 * @param fy Focal length y
 * @param cx Camera principal point x
 * @param cy Camera principal point y
 *
 */
void ElasticBridge::initElasticFusion (unsigned int height, unsigned int width, float fx, float fy, float cx, float cy)
{
    ROS_INFO("elastic_bridge: Initializing Elastic Fusion...");
    ROS_INFO("elastic_bridge: w %d, h %d, fx %f, fy %f, cx %f, cy %f",
             width, height, fx, fy, cx, cy);

    Resolution::getInstance(width, height);
    Intrinsics::getInstance(fx, fy, cx, cy);

    InitFakeOpenGLContext(m_display_name);

    m_guid_counter = 0;

    m_eFusion = new ElasticFusion(200, 35000, 5e-05, 1e-05, !m_pose_from_tf_always);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    ROS_INFO("elastic_bridge: Elastic Fusion initialized.");
}

/**
 * @brief Baseline run loop. Frame color and depth messages are populated in the callback function and then
 *        checked here. If the message has been populated, then the relative callback worker is called.
 */
void ElasticBridge::run ()
{
    boost::mutex::scoped_lock lock(m_mutex);
    while (!ros::isShuttingDown())
    {
        m_cond_var.wait_for(lock, boost::chrono::duration<uint64_t, boost::milli>(100));

        if (m_image_color && m_image_depth)
        {
            imagesCallbackWorker(m_image_color, m_image_depth);
            m_image_color.reset();
            m_image_depth.reset();
        }

        if (m_camera_info)
        {
            cameraInfoCallbackWorker(m_camera_info);
            m_camera_info.reset();
        }

        if (m_download_request)
        {
            m_point_cloud_for_download = this->getPC2(&m_guids, &m_luids);
            m_download_ready = true;
            m_cond_var.notify_all();
        }
    }

    m_cond_var.notify_all(); // upon exiting, wake up everyone
}

/**
 * @brief Initializes elastic bridge from ROS parameters.
 */
void ElasticBridge::init ()
{
    boost::mutex::scoped_lock lock(m_mutex);

    int tmp_param_val = 0;
    std::string tmp_param_str;

    m_nh.param<std::string>("TOPIC_IMAGE_COLOR", m_topic_image_color, "/camera/rgb/image_rect_color");
    m_nh.param<std::string>("TOPIC_IMAGE_DEPTH", m_topic_image_depth, "/camera/depth_registered/sw_registered/image_rect");
    m_nh.param<std::string>("TOPIC_CAMERA_INFO", m_topic_camera_info, "/camera/rgb/camera_info");
    m_nh.param<std::string>("WORLD_FRAME", m_world_frame, "first_frame");
    m_nh.param<std::string>("CAMERA_FRAME", m_camera_frame, "camera_frame");

    m_nh.param<bool>("AUTOSTART", m_running, false);
    m_nh.param<std::string>("DISPLAY_NAME", m_display_name, ""); // empty for auto-detect current
                                                                 // set to Xorg display name otherwise (e.g. ":0")

    m_nh.param<bool>("BLACK_TO_NAN", m_black_to_nan, false);

    m_nh.param<int>("PERIODIC_WORLD_PUBLICATION", tmp_param_val, 0); // 0 to disable
    m_periodic_world_publication = std::max<int>(tmp_param_val, 0);
    m_nh.param<std::string>("TOPIC_PERIODIC_WORLD_PUB", m_topic_periodic_world_pub, "/elastic_world_pub");
    m_periodic_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>(m_topic_periodic_world_pub, 1);

    m_nh.param<std::string>("TOPIC_FRAME_STATE", tmp_param_str, "/elastic_frame_state_stable");
    m_frame_state_stable_pub = m_nh.advertise<elastic_bridge::FrameState>(tmp_param_str, 1);

    m_nh.param<std::string>("TOPIC_CURRENT_VIEW", tmp_param_str, "/elastic_current_view");
    m_image_pub = m_nh.advertise<sensor_msgs::Image>(tmp_param_str, 1);

    m_camera_info_sub = m_nh.subscribe(m_topic_camera_info, 1, &ElasticBridge::cameraInfoCallback, this);

    m_nh.param<bool>("TF_POSE_ALWAYS", m_pose_from_tf_always, false);
    m_nh.param<bool>("TF_POSE_FIRST", m_pose_from_tf_first, false);
    m_nh.param<std::string>("TF_INPUT_WORLD_FRAME", m_input_world_frame, "world");
    m_nh.param<std::string>("TF_INPUT_CAMERA_FRAME", m_input_camera_frame, "robot");

    m_imageColor_sub = ImageFilterSubscriberPtr(new ImageFilterSubscriber(m_nh, m_topic_image_color, 1));
    m_imageDepth_sub = ImageFilterSubscriberPtr(new ImageFilterSubscriber(m_nh, m_topic_image_depth, 1));

    m_sync_sub = ATSynchronizerPtr(new ATSynchronizer(ATSyncPolicy(10), *m_imageColor_sub, *m_imageDepth_sub));
    m_sync_sub->registerCallback(boost::bind(&ElasticBridge::imagesCallback, this, _1, _2));
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "elastic_node");

    ros::NodeHandle nh("~");

    ElasticBridge eb(nh);

    // Service setup
    ros::ServiceServer server = nh.advertiseService("suspend", &ElasticBridge::suspend, &eb);

    std::string param_string;
    nh.param<std::string>("SAVE_PCL_ACTION", param_string, "/save_pcl");
    DownloadPC2_Action action(param_string, &eb, nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    eb.run();

    return 0;
}
