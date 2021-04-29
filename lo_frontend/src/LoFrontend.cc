/*
Authors:
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
*/





#include <lo_frontend/LoFrontend.h>





namespace pu = parameter_utils;
namespace gu = geometry_utils;





// Constructor/destructor -------------------------------------------------------------------------

LoFrontend::LoFrontend()
  : b_add_first_scan_to_key_(true),
    counter_(0),
    b_pcld_received_(false),
    msg_filtered_(new PointCloud()),
    msg_transformed_(new PointCloud()),
    msg_neighbors_(new PointCloud()),
    msg_base_(new PointCloud()),
    msg_fixed_(new PointCloud()),
    mapper_unused_fixed_(new PointCloud()),
    mapper_unused_out_(new PointCloud()),
    b_interpolate_(false),
    b_use_imu_integration_(false),
    b_use_imu_yaw_integration_(false),
    b_use_odometry_integration_(false),
    b_use_pose_stamped_integration_(false),
    imu_number_of_calls_(0),
    odometry_number_of_calls_(0),
    pose_stamped_number_of_calls_(0),
    b_imu_has_been_received_(false),
    b_odometry_has_been_received_(false),
    b_pose_stamped_has_been_received_(false),
    b_imu_frame_is_correct_(false),
    b_is_open_space_(false),
    b_run_with_gt_point_cloud_(false),
    publish_diagnostics_(false),
    tf_buffer_authority_("transform_odometry"),
    scans_dropped_(0), 
    previous_stamp_(0) {}

LoFrontend::~LoFrontend() {}





// Setup spinners/subscribers ---------------------------------------------------------------------

std::vector<ros::AsyncSpinner>
LoFrontend::setAsynchSpinners(ros::NodeHandle& _nh) {
  // Set spinners depending on parameters.
  // If a param does not exist, do not create the spinner.
  std::vector<ros::AsyncSpinner> async_spinners;

  // IMU spinner
  {
    ros::AsyncSpinner spinner_imu(1, &this->imu_queue_);
    async_spinners.push_back(spinner_imu);
    setImuSubscriber(_nh);
    ROS_INFO("[LoFrontend::setAsynchSpinners] : New subscriber for IMU.");
  }

  // Odom spinner
  {
    ros::AsyncSpinner spinner_odom(1, &this->odom_queue_);
    async_spinners.push_back(spinner_odom);
    setOdomSubscriber(_nh);
    ROS_INFO("[LoFrontend::setAsynchSpinners] : New subscriber for odom.");
  }

  // Lidar spinner
  {
    ros::AsyncSpinner spinner_lidar(1, &this->lidar_queue_);
    async_spinners.push_back(spinner_lidar);
    setLidarSubscriber(_nh);
    ROS_INFO("[LoFrontend::setAsynchSpinners] : New subscriber for lidar.");
  }

  return async_spinners;
}

void LoFrontend::setImuSubscriber(ros::NodeHandle& _nh) {
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions opts = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      "IMU_TOPIC",                                              // topic name
      imu_queue_size_,                                          // queue length
      boost::bind(&LoFrontend::ImuCallback, this, _1),          // callback
      ros::VoidPtr(),       // tracked object, we don't need one thus NULL
      &this->imu_queue_     // pointer to callback queue object
  );
  this->imu_sub_ = _nh.subscribe(opts); // subscribe
}

void LoFrontend::setOdomSubscriber(ros::NodeHandle& _nh) {
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions opts =
      ros::SubscribeOptions::create<nav_msgs::Odometry>(
          "ODOMETRY_TOPIC",                                     // topic name
          odom_queue_size_,                                     // queue length
          boost::bind(&LoFrontend::OdometryCallback, this, _1), // callback
          ros::VoidPtr(),    // tracked object, we don't need one thus NULL
          &this->odom_queue_ // pointer to callback queue object
      );
  this->odom_sub_ = _nh.subscribe(opts); // subscribe
}

void LoFrontend::setLidarSubscriber(ros::NodeHandle& _nh) {
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions opts = ros::SubscribeOptions::create<PointCloud>(
      "LIDAR_TOPIC",                                            // topic name
      lidar_queue_size_,                                        // queue length
      boost::bind(&LoFrontend::LidarCallback, this, _1),        // callback
      ros::VoidPtr(),       // tracked object, we don't need one thus NULL
      &this->lidar_queue_   // pointer to callback queue object
  );
  this->lidar_sub_ = _nh.subscribe(opts); // subscribe
}





// Initialize --------------------------------------------------------------------------------------

bool LoFrontend::Initialize(const ros::NodeHandle& n, bool from_log) {
  ROS_INFO("LoFrontend - Initialize");
  name_ = ros::names::append(n.getNamespace(), "lo_frontend");

  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }
  if (!odometry_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud odometry.", name_.c_str());
    return false;
  }
  if (!localization_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize localization.", name_.c_str());
    return false;
  }
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  // IMPORTANT - Initialize mapper after LoadParameters as we need to know the type
  if (!mapper_->Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }
  if (!SetDataIntegrationMode()) {
    ROS_ERROR("Failed to set data integration mode");
    return false;
  }
  if (!RegisterCallbacks(n, from_log)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  if (b_convert_imu_to_base_link_frame_) {
    LoadCalibrationFromTfTree();
  }
  if (b_run_with_gt_point_cloud_) {
    InitWithGTPointCloud(gt_point_cloud_filename_);
  }

  last_refresh_pose_ = localization_.GetIntegratedEstimate();
  latest_odom_stamp_ = ros::Time(0);

  return true;
}

bool LoFrontend::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - LoadParameters");
  if (!pu::Get("b_verbose", 
                b_verbose_)) 
    return false;
  if (!pu::Get("odom_pub_rate", 
                odom_pub_rate_))
    return false;
  if (!pu::Get("transform_wait_duration", 
                transform_wait_duration_))
    return false;
  if (!pu::Get("translation_threshold_kf", 
                translation_threshold_kf_))
    return false;
  if (!pu::Get("rotation_threshold_kf", 
                rotation_threshold_kf_))
    return false;
  if (!pu::Get("number_of_points_open_space", 
                number_of_points_open_space_))
    return false;
  if (!pu::Get("map_publishment/meters", 
                map_publishment_meters_))
    return false;
  if (!pu::Get("map_publishment/b_publish_map", 
                b_publish_map_))
    return false;
  if (!pu::Get("frame_id/fixed", 
                fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/base",   
                base_frame_id_))
    return false;
  if (!pu::Get("frame_id/imu", 
                imu_frame_id_))
    return false;
  if (!pu::Get("frame_id/bd_odometry", 
                bd_odom_frame_id_))
    return false;
  if (!pu::Get("frame_conversions/b_convert_imu_to_base_link_frame",
                b_convert_imu_to_base_link_frame_))
    return false;
  if (!pu::Get("queues/lidar_queue_size", 
                lidar_queue_size_))
    return false;
  if (!pu::Get("queues/imu_queue_size", 
                imu_queue_size_))
    return false;
  if (!pu::Get("queues/odom_queue_size", 
                odom_queue_size_))
    return false;
  if (!pu::Get("queues/pose_queue_size", 
                pose_queue_size_))
    return false;
  if (!pu::Get("buffers/imu_buffer_size_limit", 
                imu_buffer_size_limit_))
    return false;
  if (!pu::Get("buffers/odometry_buffer_size_limit",
               odometry_buffer_size_limit_))
    return false;
  if (!pu::Get("buffers/pose_stamped_buffer_size_limit",
               pose_stamped_buffer_size_limit_))
    return false;
  if (!pu::Get("data_integration/mode", 
                data_integration_mode_))
    return false;
  if (!pu::Get("data_integration/max_number_of_calls", 
                max_number_of_calls_))
    return false;
  if (!pu::Get("b_enable_computation_time_profiling",
                b_enable_computation_time_profiling_))
    return false;
  if (!pu::Get("b_run_with_gt_point_cloud", 
                b_run_with_gt_point_cloud_))
    return false;
  if (!pu::Get("gt_point_cloud_filename", 
                gt_point_cloud_filename_))
    return false;
  if (!pu::Get("publish_diagnostics",   
                publish_diagnostics_))
    return false;
  if (!pu::Get("window_local_mapping_type", 
                window_local_mapping_type_))
    return false;
  if (!pu::Get("b_enable_msw", 
                b_enable_msw_))
    return false;
  if (!pu::Get("box_filter_size", 
                box_filter_size_))
    return false;
  if (!pu::Get("velocity_buffer_size", 
                velocity_buffer_size_))
    return false;
  if (!pu::Get("translation_threshold_msw", 
                translation_threshold_msw_))
    return false;
  if (!pu::Get("rotational_velocity_threshold", 
                rotational_velocity_threshold_))
    return false;
  if (!pu::Get("translational_velocity_threshold", 
                translational_velocity_threshold_))
    return false;
  if (!pu::Get("statistics_time_window", 
                statistics_time_window_))
    return false;
  if (!pu::Get("statistics_verbosity_level", 
                statistics_verbosity_level_))
    return false;
  if (!pu::Get("b_interpolate", 
                b_interpolate_))
    return false;
  if (!pu::Get("b_use_osd", 
                b_use_osd_))
    return false;
  if (!pu::Get("osd_size_threshold", 
                osd_size_threshold_))
    return false;
  if (!pu::Get("b_publish_xy_cross_section", 
                b_publish_xy_cross_section_))
    return false;  
  if (!pu::Get("translation_threshold_closed_space_kf", 
                translation_threshold_closed_space_kf_))
    return false;
  if (!pu::Get("rotation_threshold_closed_space_kf", 
                rotation_threshold_closed_space_kf_))
    return false;
  if (!pu::Get("translation_threshold_open_space_kf", 
                translation_threshold_open_space_kf_))
    return false;
  if (!pu::Get("rotation_threshold_open_space_kf", 
                rotation_threshold_open_space_kf_))
    return false;
  if (!pu::Get("b_debug_transforms", 
                b_debug_transforms_))
    return false;

  if (n.getNamespace().find("spot") != std::string::npos) {
    if ((data_integration_mode_ == 0) || 
        (data_integration_mode_ == 1) || 
        (data_integration_mode_ == 2)) b_interpolate_ = false;
  }  
  ROS_INFO_STREAM("b_interpolate_: " << b_interpolate_);

  mapper_ = mapperFabric(window_local_mapping_type_);
  mapper_->SetBoxFilterSize(box_filter_size_);

  return true;
}

bool LoFrontend::SetDataIntegrationMode() {
  ROS_INFO("LoFrontend - SetDataIntegrationMode");
  switch (data_integration_mode_) {
  case 0:
    ROS_INFO("No integration requested");
    break;
  case 1:
    ROS_INFO("Imu integration requested");
    b_use_imu_integration_ = true;
    odometry_.EnableImuIntegration();
    break;
  case 2:
    ROS_INFO("Imu yaw integration requested");
    b_use_imu_integration_ = true;
    b_use_imu_yaw_integration_ = true;
    odometry_.EnableImuIntegration();
    break;
  case 3:
    ROS_INFO("Odometry integration requested");
    b_use_odometry_integration_ = true;
    odometry_.EnableOdometryIntegration();
    break;
  case 4:
    ROS_ERROR("PoseStamped integration not currently supported");
    return false;
  default:
    ROS_ERROR("Default case to be handled");
    return false;
  }
  return true;
}

bool LoFrontend::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  ROS_INFO("LoFrontend - RegisterCallbacks");
  ros::NodeHandle nl(n);
  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);
}

bool LoFrontend::RegisterLogCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - RegisterLogCallbacks");
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool LoFrontend::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - RegisterOnlineCallbacks");
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());
  nl_ = ros::NodeHandle(n);
  fga_sub_ = nl_.subscribe(
      "FGA_TOPIC", 1, &LoFrontend::FlatGroundAssumptionCallback, this);
  return CreatePublishers(n);
}

bool LoFrontend::CreatePublishers(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - CreatePublishers");
  ros::NodeHandle nl(n);

  odom_pub_timer_ =
      nl_.createTimer(odom_pub_rate_, &LoFrontend::PublishOdomOnTimer, this);  
  odometry_pub_ = 
      nl.advertise<nav_msgs::Odometry>("odometry", 10, false);

  base_frame_pcld_pub_ =
      nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);
  
  lidar_callback_duration_pub_ =
      nl.advertise<std_msgs::Float64>("lidar_callback_duration", 10, false);
  scan_to_scan_duration_pub_ =
      nl.advertise<std_msgs::Float64>("scan_to_scan_duration", 10, false);
  scan_to_submap_duration_pub_ =
      nl.advertise<std_msgs::Float64>("scan_to_submap_duration", 10, false);
  approx_nearest_neighbors_duration_pub_ =
      nl.advertise<std_msgs::Float64>("approx_nearest_neighbors_duration", 10, false);
  
  xy_cross_section_pub_ =  
      nl.advertise<std_msgs::Float64>("xy_cross_section", 10, false);
  
  diagnostics_pub_ =
      nl.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, false);
  
  return true;
}





// Callbacks ---------------------------------------------------------------------------------------

void LoFrontend::ImuCallback(const ImuConstPtr& imu_msg) {
  if (!b_imu_frame_is_correct_) CheckImuFrame(imu_msg);
  if (CheckBufferSize(imu_buffer_) > imu_buffer_size_limit_) {
    imu_buffer_.erase(imu_buffer_.begin());
  }
  if (CheckNans(*imu_msg)) {
    ROS_WARN("LoFrontend - ImuCallback - Throwing message as it contains NANS");
    return;
  }
  if (!InsertMsgInBuffer(imu_msg, imu_buffer_)) {
    ROS_WARN("LoFrontend - ImuCallback - Unable to store message in buffer");
  }
}

void LoFrontend::OdometryCallback(const OdometryConstPtr& odometry_msg) {
  if (!b_interpolate_) {
    if (CheckBufferSize(odometry_buffer_) > odometry_buffer_size_limit_)
      odometry_buffer_.erase(odometry_buffer_.begin());
    if (!InsertMsgInBuffer(odometry_msg, odometry_buffer_))
      ROS_WARN("LoFrontend - OdometryCallback - Unable to store message in buffer");
  } 
  else {
    geometry_msgs::TransformStamped odometry;
    geometry_msgs::Vector3 t;
    t.x = odometry_msg->pose.pose.position.x;
    t.y = odometry_msg->pose.pose.position.y;
    t.z = odometry_msg->pose.pose.position.z;
    odometry.transform.translation = t;
    odometry.transform.rotation = odometry_msg->pose.pose.orientation;
    odometry.header = odometry_msg->header;
    odometry.header.frame_id = odometry_msg->header.frame_id;
    odometry.child_frame_id = odometry_msg->child_frame_id;
    tf2_ros_odometry_buffer_.setTransform(
        odometry, tf_buffer_authority_, false);
    latest_odom_stamp_ = odometry_msg->header.stamp;
  }
}

void LoFrontend::PoseStampedCallback(const PoseStampedConstPtr& pose_stamped_msg) {
  if (CheckBufferSize(pose_stamped_buffer_) > pose_stamped_buffer_size_limit_) {
    pose_stamped_buffer_.erase(pose_stamped_buffer_.begin());
  }
  if (!InsertMsgInBuffer(pose_stamped_msg, pose_stamped_buffer_)) {
    ROS_WARN(
        "LoFrontend - PoseStampedCallback - Unable to store message in buffer");
  }
}

void LoFrontend::LidarCallback(const PointCloud::ConstPtr& msg) {
  // TO TEST Delays
  // ros::Duration(0.2).sleep();
  
  // TODO: move to class members
  ros::Time lidar_callback_start;
  ros::Time scan_to_scan_start;
  ros::Time scan_to_submap_start;
  ros::Time approx_nearest_neighbors_start;

  if (b_enable_computation_time_profiling_) {
    lidar_callback_start = ros::Time::now();
  }

  if (b_use_osd_) {
    pcl::getMinMax3D(*msg, minPoint_, maxPoint_);
    auto size_x = maxPoint_.x - minPoint_.x;
    auto size_y = maxPoint_.y - minPoint_.y;
    if (size_x > osd_size_threshold_ && size_y > osd_size_threshold_) {
      b_is_open_space_ = true;
      translation_threshold_kf_ = translation_threshold_open_space_kf_; 
      rotation_threshold_kf_ = rotation_threshold_open_space_kf_;  
    }
    else {
      b_is_open_space_ = false;
      translation_threshold_kf_ = translation_threshold_closed_space_kf_; 
      rotation_threshold_kf_ = rotation_threshold_closed_space_kf_;  
    }
    if (b_publish_xy_cross_section_) {
      auto xy_cross_section_msg = std_msgs::Float64();
      xy_cross_section_msg.data = size_x * size_y;
      xy_cross_section_pub_.publish(xy_cross_section_msg);
    }
  }
  
  if (!b_pcld_received_) {
    statistics_start_time_ = ros::Time::now();
    pcld_seq_prev_ = msg->header.seq;
    b_pcld_received_ = true;
  } 
  else {
    auto sequence_difference = (int)msg->header.seq - (int)pcld_seq_prev_;
    if (sequence_difference != 1) scans_dropped_ = scans_dropped_ + sequence_difference - 1;    
    if (sequence_difference <= 0) {
      ROS_WARN("--------- sequence_difference <= 0 ---------");
      ROS_INFO_STREAM("Current sequence: " << msg->header.seq);
      ROS_INFO_STREAM("Previous sequence: " << pcld_seq_prev_);
      ROS_WARN("--------------------------------------------");
    } 
    if (statistics_verbosity_level_ == "high") ROS_INFO_STREAM("Dropped " << scans_dropped_ << " scans");    
    if (statistics_verbosity_level_ == "low") {
      if (ros::Time::now().toSec() - statistics_start_time_.toSec() > statistics_time_window_) {
        auto drop_rate = (float)scans_dropped_ / (float)statistics_time_window_; 
        ROS_INFO_STREAM("Dropped " << scans_dropped_ << " scans over " << statistics_time_window_ << 
                        " s ---> drop rate is: " << drop_rate << " scans/s");
      scans_dropped_ = 0; 
      statistics_start_time_ = ros::Time::now();
      }
    }
    pcld_seq_prev_ = msg->header.seq;    
  }

  auto msg_stamp = msg->header.stamp;
  ros::Time stamp = pcl_conversions::fromPCL(msg_stamp);

  if (!b_interpolate_) {

    /*
    b_interpolate_ false for: 
        - Husky with WIO/IMU/NO integration
        - Spot with IMU/NO integration 
    */    
    
    if (b_use_odometry_integration_) {
      Odometry odometry_msg;
      if (!GetMsgAtTime(stamp, odometry_msg, odometry_buffer_)) {
        ROS_WARN("Unable to retrieve odometry_msg from odometry_buffer_ given "
                 "Lidar timestamp");
        odometry_number_of_calls_++;
        if (odometry_number_of_calls_ > max_number_of_calls_) {
          ROS_WARN("Deactivating odometry_integration in LoFrontend as "
                   "odometry_number_of_calls > max_number_of_calls");
          SwitchToImuIntegration();
        }
        return;
      }
      odometry_number_of_calls_ = 0;
      if (!b_odometry_has_been_received_) {
        ROS_INFO("Receiving odometry for the first time");
        tf::poseMsgToTF(odometry_msg.pose.pose, odometry_pose_previous_);
        b_odometry_has_been_received_ = true;
        return;
      }
      tf::Transform odometry_pose;
      tf::poseMsgToTF(odometry_msg.pose.pose, odometry_pose);
      odometry_.SetOdometryDelta(GetOdometryDelta(odometry_pose));
      tf::poseMsgToTF(odometry_msg.pose.pose, odometry_pose_previous_);
    }     
    else if (b_use_imu_integration_) {
      Imu imu_msg;
      if (!GetMsgAtTime(stamp, imu_msg, imu_buffer_)) {
        ROS_WARN("Unable to retrieve imu_msg from imu_buffer_ given Lidar "
                 "timestamp");
        imu_number_of_calls_++;
        if (imu_number_of_calls_ > max_number_of_calls_) {
          ROS_WARN("Deactivating imu_integration in LoFrontend as "
                   "imu_number_of_calls > max_number_of_calls");
          b_use_imu_integration_ = false;
          odometry_.DisableImuIntegration();
        }
        return;
      }
      imu_number_of_calls_ = 0;
      auto imu_quaternion = GetImuQuaternion(imu_msg);
      if (!b_imu_has_been_received_) {
        ROS_INFO("Receiving imu for the first time");
        imu_quaternion_previous_ = imu_quaternion;
        b_imu_has_been_received_ = true;
        return;
      }
      imu_quaternion_change_ =
          imu_quaternion_previous_.inverse() * imu_quaternion;
      if (b_use_imu_yaw_integration_) {
        odometry_.SetImuDelta(GetImuYawDelta());
      } else {
        odometry_.SetImuDelta(GetImuDelta());
      }
      imu_quaternion_previous_ = imu_quaternion;
    }
  }

  else {

    /*
    b_interpolate_ true for: 
        - Spot with VO integration 
    */  

    bool have_odom_transform = false;
    geometry_msgs::TransformStamped t;

    ros::Time stamp_transform_to;
    if (latest_odom_stamp_ < stamp) {
      stamp_transform_to = latest_odom_stamp_;
    } 
    else {
      stamp_transform_to = stamp;
    }

    // Check if we can get an odometry source transform 
    // from the time of the last pointcloud to the latest VO timestamp
    if (tf2_ros_odometry_buffer_.canTransform(base_frame_id_,
                                              previous_stamp_,
                                              base_frame_id_,
                                              stamp_transform_to,
                                              bd_odom_frame_id_)) {
      have_odom_transform = true;
      t = tf2_ros_odometry_buffer_.lookupTransform(base_frame_id_,
                                                   previous_stamp_,
                                                   base_frame_id_,
                                                   stamp_transform_to,  
                                                   bd_odom_frame_id_);
    }

    if (b_debug_transforms_) {
      ROS_INFO("-------------------------------------------------");
      ROS_INFO_STREAM("stamp: " << stamp);
      ROS_INFO_STREAM("latest_odom_stamp_: " << latest_odom_stamp_);
      ROS_INFO_STREAM("stamp_transform_to:" << stamp_transform_to);
      ROS_INFO_STREAM("previous_stamp_:" << previous_stamp_);
      if (!have_odom_transform) ROS_WARN("Don't have odom transform"); 
    }

    tf::Transform tf_transform;

    if (have_odom_transform) {
      // Have the tf, so use it
      tf::Vector3 tf_translation;
      tf::Quaternion tf_quaternion;
      tf::vector3MsgToTF(t.transform.translation, tf_translation);
      tf::quaternionMsgToTF(t.transform.rotation, tf_quaternion);
      tf_transform.setOrigin(tf_translation);
      tf_transform.setRotation(tf_quaternion);
    } 
    else {
      // Don't have a valid tf so do pure LO
      tf::Vector3 tf_translation(0.0, 0.0, 0.0);
      tf::Quaternion tf_quaternion(0.0, 0.0, 0.0, 1.0);
      tf_transform.setOrigin(tf_translation);
      tf_transform.setRotation(tf_quaternion);
    }

    if (!b_odometry_has_been_received_) {
      ROS_INFO("Receiving odometry for the first time");
      b_odometry_has_been_received_ = true;
      return;
    }

    // Have the delta - set this directly in odom
    odometry_.SetOdometryDelta(tf_transform);
  }

  filter_.Filter(msg, msg_filtered_, b_is_open_space_);
  odometry_.SetLidar(*msg_filtered_);

  if (b_enable_computation_time_profiling_) {
    scan_to_scan_start = ros::Time::now();
  }

  if (!odometry_.UpdateEstimate()) {
    b_add_first_scan_to_key_ = true;
  }
  diagnostic_msgs::DiagnosticStatus diagnostics_odometry =
      odometry_.GetDiagnostics();
  if (diagnostics_odometry.level == 0)
    odometry_.PublishAll();

  if (b_enable_computation_time_profiling_) {
    auto scan_to_scan_end = ros::Time::now();
    auto scan_to_scan_duration = scan_to_scan_end - scan_to_scan_start;
    auto scan_to_scan_duration_msg = std_msgs::Float64();
    scan_to_scan_duration_msg.data = float(scan_to_scan_duration.toSec());
    scan_to_scan_duration_pub_.publish(scan_to_scan_duration_msg);
  }

  if (b_add_first_scan_to_key_ && !b_run_with_gt_point_cloud_) {
    localization_.TransformPointsToFixedFrame(*msg, msg_transformed_.get());
    mapper_->InsertPoints(msg_transformed_, mapper_unused_fixed_.get());
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();
    b_add_first_scan_to_key_ = false;
    last_keyframe_pose_ = localization_.GetIntegratedEstimate();
    previous_pose_ = localization_.GetIntegratedEstimate();
    previous_stamp_ = stamp;
    return;
  }

  if (b_enable_computation_time_profiling_) {
    scan_to_submap_start = ros::Time::now();
  }

  localization_.MotionUpdate(odometry_.GetIncrementalEstimate());
  localization_.TransformPointsToFixedFrame(*msg, msg_transformed_.get());

  if (b_enable_computation_time_profiling_) {
    approx_nearest_neighbors_start = ros::Time::now();
  }
  mapper_->ApproxNearestNeighbors(*msg_transformed_, msg_neighbors_.get());
  if (b_enable_computation_time_profiling_) {
    auto approx_nearest_neighbors_end = ros::Time::now();
    auto approx_nearest_neighbors_duration = approx_nearest_neighbors_end - approx_nearest_neighbors_start;
    auto approx_nearest_neighbors_duration_msg = std_msgs::Float64();
    approx_nearest_neighbors_duration_msg.data = float(approx_nearest_neighbors_duration.toSec());
    approx_nearest_neighbors_duration_pub_.publish(approx_nearest_neighbors_duration_msg); 
  }

  localization_.TransformPointsToSensorFrame(*msg_neighbors_,
                                             msg_neighbors_.get());
  localization_.MeasurementUpdate(
      msg_filtered_, msg_neighbors_, msg_base_.get());

  diagnostic_msgs::DiagnosticStatus diagnostics_localization =
      localization_.GetDiagnostics();
  if (diagnostics_localization.level == 0)
    localization_.PublishAll();

  if (b_enable_computation_time_profiling_) {
    auto scan_to_submap_end = ros::Time::now();
    auto scan_to_submap_duration = scan_to_submap_end - scan_to_submap_start;
    auto scan_to_submap_duration_msg = std_msgs::Float64();
    scan_to_submap_duration_msg.data = float(scan_to_submap_duration.toSec());
    scan_to_submap_duration_pub_.publish(scan_to_submap_duration_msg);
  }

  geometry_utils::Transform3 current_pose =
      localization_.GetIntegratedEstimate();

  // Update current pose for publishing
  latest_pose_ = current_pose;  
  previous_stamp_ = stamp;
  latest_pose_stamp_ = stamp;
  b_have_published_odom_ = false;

  // Compute delta
  gtsam::Pose3 delta =
      ToGtsam(geometry_utils::PoseDelta(last_keyframe_pose_, current_pose));

  if (delta.translation().norm() > translation_threshold_kf_ ||
      fabs(2 * acos(delta.rotation().toQuaternion().w())) >
          rotation_threshold_kf_) {
    if (b_verbose_)
      ROS_INFO_STREAM("Adding to map with translation "
                      << delta.translation().norm() << " and rotation "
                      << 2 * acos(delta.rotation().toQuaternion().w()) * 180.0 /
                          M_PI
                      << " deg");
    localization_.MotionUpdate(gu::Transform3::Identity());
    localization_.TransformPointsToFixedFrame(*msg, msg_fixed_.get());
    mapper_->InsertPoints(msg_fixed_, mapper_unused_out_.get());
    if (b_publish_map_) {
      counter_++;
      if (counter_ == map_publishment_meters_) {
        if (b_enable_msw_) mapper_->Refresh(current_pose);       
        mapper_->PublishMap();
        counter_ = 0;
      }
    }
    last_keyframe_pose_ = current_pose;
  }

  if (base_frame_pcld_pub_.getNumSubscribers() != 0) {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }

  if (b_enable_computation_time_profiling_) {
    auto lidar_callback_end = ros::Time::now();
    auto lidar_callback_duration = lidar_callback_end - lidar_callback_start;
    auto lidar_callback_duration_msg = std_msgs::Float64();
    lidar_callback_duration_msg.data = float(lidar_callback_duration.toSec());
    lidar_callback_duration_pub_.publish(lidar_callback_duration_msg);
  }

  // Publish diagnostics
  if (publish_diagnostics_) {
    diagnostic_msgs::DiagnosticArray diagnostic_array;
    diagnostic_array.status.push_back(diagnostics_odometry);
    diagnostic_array.status.push_back(diagnostics_localization);
    diagnostic_array.header.seq++;
    diagnostic_array.header.stamp = ros::Time::now();
    diagnostic_array.header.frame_id = name_;
    diagnostics_pub_.publish(diagnostic_array);
  }
}

void LoFrontend::FlatGroundAssumptionCallback(const std_msgs::Bool& bool_msg) {
  ROS_INFO("LoFrontend - FlatGroundAssumptionCallback");
  std::cout << "Received " << bool_msg.data << std::endl;
  odometry_.SetFlatGroundAssumptionValue(bool_msg.data);
  localization_.SetFlatGroundAssumptionValue(bool_msg.data);
}





// Publish odometry at fixed rate --------------------------------------------------------------------

void LoFrontend::PublishOdomOnTimer(const ros::TimerEvent& ev) {
  // Publishes the latest odometry at a fixed rate
  
  // Currently works for VO - TODO: think about removing b_interpolate
  // TODO - add check to stop if we don't have lidar yet
  
  // Get latest covariance
  Eigen::Matrix<double, 6, 6> covariance =
      localization_.GetLatestDeltaCovariance();

  // Get the timestamp of the latest pose
  ros::Time lidar_stamp = localization_.GetLatestTimestamp(); 
  ros::Time publish_stamp = lidar_stamp;

  // Check if we can get additional transforms from the odom source
  bool have_odom_transform = false;
  geometry_msgs::TransformStamped t;
  if (latest_odom_stamp_ > lidar_stamp && data_integration_mode_ >= 3 &&
      false &&
      tf2_ros_odometry_buffer_.canTransform(base_frame_id_,
                                            latest_pose_stamp_,
                                            base_frame_id_,
                                            latest_odom_stamp_,
                                            bd_odom_frame_id_)) {
    have_odom_transform = true;
    publish_stamp = latest_odom_stamp_;
    // Get transform between latest lidar timestamp and latest odom timestamp
    t = tf2_ros_odometry_buffer_.lookupTransform(base_frame_id_,
                                                 latest_pose_stamp_,
                                                 base_frame_id_,
                                                 latest_odom_stamp_,
                                                 bd_odom_frame_id_);
  } else {
    publish_stamp = latest_pose_stamp_;
    // TODO - don't print this warning if we have not chosen odom as an input
    // TODO - just do stats on this
    // ROS_WARN("Can not get transform from odom source");
  }
  
  if (have_odom_transform) {
    // Convert transform into common format
    geometry_utils::Transform3 odom_delta =
        geometry_utils::ros::FromROS(t.transform);

    latest_pose_ = geometry_utils::PoseUpdate(latest_pose_, odom_delta);
    latest_pose_stamp_ = latest_odom_stamp_;
  } else {
    // latest_pose_ = latest_pose_;
  }

  // Publish as an odometry message
  if (!b_have_published_odom_ || have_odom_transform) {
    // TODO - add to the covariance with the delta from visual odom
    PublishOdometry(latest_pose_, covariance, publish_stamp);
    b_have_published_odom_ = true;
  }

  return;
}

void LoFrontend::PublishOdometry(const geometry_utils::Transform3& odometry,
                                 const Eigen::Matrix<double, 6, 6>& covariance,
                                 const ros::Time stamp) {
  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.stamp = stamp;
  odometry_msg.header.frame_id = fixed_frame_id_;
  odometry_msg.pose.pose.position =
      geometry_utils::ros::ToRosPoint(odometry.translation);
  odometry_msg.pose.pose.orientation = geometry_utils::ros::ToRosQuat(
      geometry_utils::RToQuat(odometry.rotation));
  for (size_t i = 0; i < 36; i++) {
    size_t row = static_cast<size_t>(i / 6);
    size_t col = i % 6;
    odometry_msg.pose.covariance[i] = covariance(row, col);
  }
  odometry_pub_.publish(odometry_msg);

  return;
}





// Utilities ----------------------------------------------------------------------------------------

void LoFrontend::CheckImuFrame(const ImuConstPtr& imu_msg) {
  if (b_convert_imu_to_base_link_frame_) {
    if (imu_msg->header.frame_id.find("vn100") != std::string::npos) {
      ROS_INFO("Received imu_msg is correctly expressed in imu frame");
      b_imu_frame_is_correct_ = true;
    } else {
      ROS_ERROR("Received imu_msg is not expressed in imu frame, but an imu to "
                "base_link conversion is enabled");
      return;
    }
  } else {
    if (imu_msg->header.frame_id.find("base_link") != std::string::npos) {
      ROS_INFO("Received imu_msg is correctly expressed in base_link frame");
      b_imu_frame_is_correct_ = true;
    } else {
      ROS_ERROR("Received imu_msg is not expressed in base_link frame, but an "
                "imu to base_link conversion is disabled");
      return;
    }
  }
}

bool LoFrontend::LoadCalibrationFromTfTree() {
  ROS_INFO("LoFrontend - LoadCalibrationFromTfTree");
  ROS_WARN_DELAYED_THROTTLE(
      2.0,
      "Waiting for \'%s\' and \'%s\' to appear in tf_tree...",
      imu_frame_id_.c_str(),
      base_frame_id_.c_str());
  tf::StampedTransform imu_T_base_transform;
  try {
    imu_T_base_listener_.waitForTransform(
        imu_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(2.0));

    imu_T_base_listener_.lookupTransform(
        imu_frame_id_, base_frame_id_, ros::Time(0), imu_T_base_transform);

    geometry_msgs::TransformStamped imu_T_base_tmp_msg;
    tf::transformStampedTFToMsg(imu_T_base_transform, imu_T_base_tmp_msg);
    tf::transformMsgToEigen(imu_T_base_tmp_msg.transform, I_T_B_);
    B_T_I_ = I_T_B_.inverse();
    ROS_INFO_STREAM("Loaded pose_sensor to imu calibration B_T_L:");
    std::cout << I_T_B_.translation() << std::endl;
    std::cout << I_T_B_.rotation() << std::endl;
    I_T_B_q_ = Eigen::Quaterniond(I_T_B_.rotation());
    ROS_INFO("q: x: %.3f, y: %.3f, z: %.3f, w: %.3f",
             I_T_B_q_.x(),
             I_T_B_q_.y(),
             I_T_B_q_.z(),
             I_T_B_q_.w());
    return true;
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    I_T_B_ = Eigen::Affine3d::Identity();
    B_T_I_ = Eigen::Affine3d::Identity();
    return false;
  }
}

gtsam::Pose3 LoFrontend::ToGtsam(const geometry_utils::Transform3& pose) const {
  gtsam::Vector3 t;
  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);
  gtsam::Rot3 r(pose.rotation(0, 0),
                pose.rotation(0, 1),
                pose.rotation(0, 2),
                pose.rotation(1, 0),
                pose.rotation(1, 1),
                pose.rotation(1, 2),
                pose.rotation(2, 0),
                pose.rotation(2, 1),
                pose.rotation(2, 2));
  return gtsam::Pose3(r, t);
}

bool LoFrontend::CheckNans(const Imu& imu_msg) {
  return (
      std::isnan(imu_msg.orientation.x) || std::isnan(imu_msg.orientation.y) ||
      std::isnan(imu_msg.orientation.z) || std::isnan(imu_msg.orientation.w) ||
      std::isnan(imu_msg.angular_velocity.x) ||
      std::isnan(imu_msg.angular_velocity.y) ||
      std::isnan(imu_msg.angular_velocity.z) ||
      std::isnan(imu_msg.linear_acceleration.x) ||
      std::isnan(imu_msg.linear_acceleration.y) ||
      std::isnan(imu_msg.linear_acceleration.z));
}

void LoFrontend::InitWithGTPointCloud(const std::string filename) {
  ROS_INFO_STREAM("Generating point cloud ground truth using point cloud from "
                  << filename);

  // Read ground truth from file
  pcl::PCDReader pcd_reader;
  PointCloud gt_point_cloud;
  pcd_reader.read(filename, gt_point_cloud);
  PointCloud::Ptr gt_pc_ptr(new PointCloud(gt_point_cloud));

  // Create octree map to select only the parts needed
  PointCloud::Ptr unused(new PointCloud);
  mapper_->InsertPoints(gt_pc_ptr, unused.get());

  ROS_INFO("Completed addition of GT point cloud to map");
}

void LoFrontend::SwitchToImuIntegration() {
  ROS_WARN("LoFrontend - SwitchToImuIntegration");
  b_use_odometry_integration_ = false;
  odometry_.DisableOdometryIntegration();
  b_use_imu_integration_ = true;
  odometry_.EnableImuIntegration();
}





// Getters ------------------------------------------------------------------------------------------

Eigen::Quaterniond LoFrontend::GetImuQuaternion(const Imu& imu_msg) {
  Eigen::Quaterniond imu_quaternion =
      Eigen::Quaterniond(double(imu_msg.orientation.w),
                         double(imu_msg.orientation.x),
                         double(imu_msg.orientation.y),
                         double(imu_msg.orientation.z));
  if (b_convert_imu_to_base_link_frame_) {
    imu_quaternion = I_T_B_q_ * imu_quaternion * I_T_B_q_.inverse();
  }
  return imu_quaternion;
}

tf::Transform LoFrontend::GetOdometryDelta(const tf::Transform& odometry_pose) const {
  return odometry_pose_previous_.inverseTimes(odometry_pose);
}

Eigen::Matrix3d LoFrontend::GetImuDelta() {
  return imu_quaternion_change_.normalized().toRotationMatrix();
}

Eigen::Matrix3d LoFrontend::GetImuYawDelta() {
  Eigen::Matrix3d rot_yaw_mat;
  double roll, pitch, yaw;
  tf::Quaternion q(imu_quaternion_change_.x(),
                   imu_quaternion_change_.y(),
                   imu_quaternion_change_.z(),
                   imu_quaternion_change_.w());
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  ROS_INFO_STREAM("LoFrontend - GetImuYawDelta - Yaw delta from IMU is "
                  << yaw * 180.0 / M_PI << " deg");
  rot_yaw_mat = Eigen::Matrix3d();
  rot_yaw_mat << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  return rot_yaw_mat;
}

double LoFrontend::GetVectorAverage(const std::vector<double>& vector) {
  return vector.empty() ?
      0.0 :
      std::accumulate(vector.begin(), vector.end(), 0.0) / vector.size();
}





// Buffer management --------------------------------------------------------------------------------

template <typename T1, typename T2>
bool LoFrontend::InsertMsgInBuffer(const T1& msg, 
                                   T2& buffer) {
  auto initial_size = buffer.size();
  auto current_time = msg->header.stamp.toSec();
  buffer.insert({current_time, *msg});
  auto final_size = buffer.size();
  if (final_size == (initial_size + 1)) {
    return true;
  } else {
    return false;
  }
}

template <typename T>
int LoFrontend::CheckBufferSize(const T& buffer) const {
  return buffer.size();
}

template <typename T1, typename T2>
bool LoFrontend::GetMsgAtTime(const ros::Time& stamp,
                              T1& msg,
                              T2& buffer) const {
  if (buffer.size() == 0) {
    return false;
  }
  auto itrTime = buffer.lower_bound(stamp.toSec());
  auto time2 = itrTime->first;
  double time_diff;
  if (itrTime == buffer.begin()) {
    msg = itrTime->second;
    time_diff = itrTime->first - stamp.toSec();
    ROS_WARN("itrTime points to buffer begin");
  } else if (itrTime == buffer.end()) {
    itrTime--;
    msg = itrTime->second;
    time_diff = stamp.toSec() - itrTime->first;
    ROS_WARN("itrTime points to buffer end");
  } else {
    double time1 = std::prev(itrTime, 1)->first;
    if (time2 - stamp.toSec() < stamp.toSec() - time1) {
      msg = itrTime->second;
      time_diff = time2 - stamp.toSec();
    } else {
      msg = std::prev(itrTime, 1)->second;
      time_diff = stamp.toSec() - time1;
    }
  }
  if (fabs(time_diff) > 0.1) {
    ROS_WARN("fabs(time_diff) > 0.1 : returing false");
    return false;
  }
  return true;
}