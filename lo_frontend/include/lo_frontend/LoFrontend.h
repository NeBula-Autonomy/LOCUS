#ifndef LO_FRONTEND_LO_FRONTEND_H
#define LO_FRONTEND_LO_FRONTEND_H

#include <chrono>
#include <math.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_odometry/PointCloudOdometry.h>
#include <point_cloud_localization/PointCloudLocalization.h>
#include <point_cloud_mapper/PointCloudMapper.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <core_msgs/PoseAndScan.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <core_msgs/PoseAndScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

class LoFrontend {

  friend class LoFrontendTest;

public:

  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef sensor_msgs::Imu Imu;
  typedef nav_msgs::Odometry Odometry;
  typedef geometry_msgs::PoseStamped PoseStamped; 
  typedef std::map<double, Imu> ImuBuffer;       
  typedef std::map<double, Odometry> OdometryBuffer;         
  typedef std::map<double, PoseStamped> PoseStampedBuffer;   
  typedef Imu::ConstPtr ImuConstPtr;
  typedef Odometry::ConstPtr OdometryConstPtr;
  typedef PoseStamped::ConstPtr PoseStampedConstPtr;

  LoFrontend();
  ~LoFrontend();

  bool Initialize(const ros::NodeHandle& n, bool from_log);

private:

  std::string name_;
  bool b_verbose_;

  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
  bool RegisterLogCallbacks(const ros::NodeHandle& n);
  bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  ros::Subscriber lidar_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;

  ros::Publisher base_frame_pcld_pub_;
  ros::Publisher diagnostics_pub_;

  void LidarCallback(const PointCloud::ConstPtr& msg);
  void ImuCallback(const ImuConstPtr& imu_msg);
  void OdometryCallback(const OdometryConstPtr& odometry_msg);
  void PoseStampedCallback(const PoseStampedConstPtr& pose_stamped_msg);

  int lidar_queue_size_; 
  int imu_queue_size_; 
  int odom_queue_size_; 
  int pose_queue_size_; 

  ImuBuffer imu_buffer_;
  OdometryBuffer odometry_buffer_;
  PoseStampedBuffer pose_stamped_buffer_;
  
  int imu_buffer_size_limit_; 
  int odometry_buffer_size_limit_;
  int pose_stamped_buffer_size_limit_;

  template <typename T1, typename T2>
  bool InsertMsgInBuffer(const T1& msg, T2& buffer);

  template <typename T>
  int CheckBufferSize(const T& buffer) const;

  template <typename T1, typename T2>
  bool GetMsgAtTime(const ros::Time& stamp, T1& msg, T2& buffer) const;         

  double translation_threshold_kf_;
  double rotation_threshold_kf_;
  bool b_add_first_scan_to_key_;

  gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;   
  geometry_utils::Transform3 last_keyframe_pose_;      

  std::string fixed_frame_id_; 
  std::string base_frame_id_; 
  std::string imu_frame_id_;
  
  bool LoadCalibrationFromTfTree();
  tf::TransformListener imu_T_base_listener_;
  Eigen::Affine3d I_T_B_;    
  Eigen::Affine3d B_T_I_; 
  Eigen::Quaterniond I_T_B_q_;   
  
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  PointCloudLocalization localization_;
  PointCloudMapper mapper_;
  bool b_run_rolling_map_buffer_;

  bool b_publish_map_;
  int counter_;
  int map_publishment_meters_;
  
  bool b_pcld_received_;
  int pcld_seq_prev_;

  PointCloud::Ptr msg_filtered_;
  PointCloud::Ptr msg_transformed_;
  PointCloud::Ptr msg_neighbors_;
  PointCloud::Ptr msg_base_;
  PointCloud::Ptr msg_fixed_;
  PointCloud::Ptr mapper_unused_fixed_;
  PointCloud::Ptr mapper_unused_out_;

  /*--------------
  Data integration 
  --------------*/

  bool SetDataIntegrationMode();
  int data_integration_mode_;
  int max_number_of_calls_;

  // Imu  
  void CheckImuFrame(const ImuConstPtr& imu_msg); 
  bool CheckNans(const Imu &msg);
  Eigen::Quaterniond GetImuQuaternion(const Imu& imu_msg);
  bool b_convert_imu_to_base_link_frame_;
  bool b_imu_frame_is_correct_;
  bool b_use_imu_integration_;
  bool b_use_imu_yaw_integration_;
  bool b_imu_has_been_received_;
  int imu_number_of_calls_;
  Eigen::Quaterniond imu_quaternion_previous_;
  Eigen::Quaterniond imu_quaternion_change_;
  Eigen::Matrix3d GetImuDelta();
  Eigen::Matrix3d GetImuYawDelta();

  // Odometry
  bool b_use_odometry_integration_;
  bool b_odometry_has_been_received_;
  int odometry_number_of_calls_;
  tf::Transform odometry_pose_previous_;
  tf::Transform GetOdometryDelta(const Odometry& odometry_msg) const; 

  // PoseStamped 
  bool b_use_pose_stamped_integration_;
  bool b_pose_stamped_has_been_received_;
  int pose_stamped_number_of_calls_;

  /*-----------------
  Open space detector
  ------------------*/
  
  bool b_is_open_space_;
  int number_of_points_open_space_;

  /* ----------------------------------
  Dynamic hierarchical data integration
  ---------------------------------- */
  
  ros::NodeHandle nl_;
  void SwitchToImuIntegration();

  /* -------------------------
  Flat Ground Assumption (FGA)
  ------------------------- */

  ros::Subscriber fga_sub_;
  void FlatGroundAssumptionCallback(const std_msgs::Bool& bool_msg);

  /* -------------------------
  Computation Time Profiling 
  ------------------------- */

  bool b_enable_computation_time_profiling_;
  ros::Publisher lidar_callback_duration_pub_; 
  ros::Publisher scan_to_scan_duration_pub_;  
  ros::Publisher scan_to_submap_duration_pub_;  
  
  /* -------------------------
  Ground Truth 
  ------------------------- */

  std::string gt_point_cloud_filename_;
  bool b_run_with_gt_point_cloud_;
  void InitWithGTPointCloud(const std::string filename);

  /* -------------------------
  Diagnostics
  ------------------------- */
  bool publish_diagnostics_;

  /*--------------------------
  Map Sliding Window 2 
  --------------------------*/
  int msw_box_filter_size_;
  geometry_utils::Transform3 previous_pose_; 
  geometry_utils::Transform3 last_refresh_pose_;
  ros::Time previous_stamp_; 
  std::vector<double> translational_velocity_buffer_; 
  std::vector<double> rotational_velocity_buffer_;
  double GetVectorAverage(const std::vector<double>& vector); 

};

#endif 