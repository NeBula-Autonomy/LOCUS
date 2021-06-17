#include <point_cloud_merger/PointCloudMerger.h>

namespace pu = parameter_utils;

PointCloudMerger::PointCloudMerger()
  : b_use_random_filter_(false), b_use_radius_filter_(false) {}

PointCloudMerger::~PointCloudMerger() {}

bool PointCloudMerger::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudMerger");
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  number_of_active_devices_ = number_of_velodynes_;
  return true;
}

bool PointCloudMerger::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("merging/number_of_velodynes", number_of_velodynes_))
    return false;
  if (!pu::Get("merging/decimate_percentage", decimate_percentage_))
    return false;
  if (!pu::Get("merging/b_use_random_filter", b_use_random_filter_))
    return false;
  if (!pu::Get("merging/b_use_radius_filter", b_use_radius_filter_))
    return false;
  if (!pu::Get("merging/radius", radius_))
    return false;
  if (!pu::Get("merging/radius_knn", radius_knn_))
    return false;
  return true;
}

bool PointCloudMerger::RegisterCallbacks(const ros::NodeHandle& n) {
  nl_ = ros::NodeHandle(n);

  failure_detection_sub_ =
      nl_.subscribe("failure_detection",
                    10,
                    &PointCloudMerger::FailureDetectionCallback,
                    this);
  resurrection_detection_sub_ =
      nl_.subscribe("resurrection_detection",
                    10,
                    &PointCloudMerger::ResurrectionDetectionCallback,
                    this);

  pcld0_sub_ = new message_filters::Subscriber<PointCloudF>(nl_, "pcld0", 10);
  pcld1_sub_ = new message_filters::Subscriber<PointCloudF>(nl_, "pcld1", 10);
  id_to_sub_map_.insert({0, pcld0_sub_});
  id_to_sub_map_.insert({1, pcld1_sub_});
  alive_keys_ = {0, 1};

  if (number_of_velodynes_ == 2) {
    ROS_INFO("PointCloudMerger - 2 VLPs merging requested");
    pcld_synchronizer_2_ =
        std::unique_ptr<TwoPcldSynchronizer>(new TwoPcldSynchronizer(
            TwoPcldSyncPolicy(pcld_queue_size_), *pcld0_sub_, *pcld1_sub_));
    two_sync_connection_ = pcld_synchronizer_2_->registerCallback(
        &PointCloudMerger::TwoPointCloudCallback, this);
  } else if (number_of_velodynes_ == 3) {
    ROS_INFO("PointCloudMerger - 3 VLPs merging requested");
    pcld2_sub_ = new message_filters::Subscriber<PointCloudF>(nl_, "pcld2", 10);
    pcld_synchronizer_3_ = std::unique_ptr<ThreePcldSynchronizer>(
        new ThreePcldSynchronizer(ThreePcldSyncPolicy(pcld_queue_size_),
                                  *pcld0_sub_,
                                  *pcld1_sub_,
                                  *pcld2_sub_));
    three_sync_connection_ = pcld_synchronizer_3_->registerCallback(
        &PointCloudMerger::ThreePointCloudCallback, this);
    alive_keys_.push_back(2);
    id_to_sub_map_.insert({2, pcld2_sub_});
  } else {
    ROS_WARN("PointCloudMerger - number_of_velodynes_ !=2 and !=3");
    return false;
  }

  return true;
}

bool PointCloudMerger::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  merged_pcld_pub_ =
      nl.advertise<PointCloudF>("combined_point_cloud", 10, false);
  return true;
}

// TODO:
// Reduce---------------------------------------------------------------------------------

void PointCloudMerger::OnePointCloudCallback(const PointCloudF::ConstPtr& a) {
  PointCloudF::Ptr cloud(new PointCloudF(*a));

  if (b_use_random_filter_) {
    const int n_points =
        static_cast<int>((1.0 - decimate_percentage_) * cloud->size());
    pcl::RandomSample<PointF> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(cloud);
    random_filter.filter(*cloud);
  }
  if (b_use_radius_filter_) {
    pcl::RadiusOutlierRemoval<PointF> rad;
    rad.setInputCloud(cloud);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*cloud);
  }

  PublishMergedPointCloud(cloud);
}

void PointCloudMerger::TwoPointCloudCallback(const PointCloudF::ConstPtr& a,
                                             const PointCloudF::ConstPtr& b) {
  //  PointCloudF p1, p2;
  //  pcl::fromROSMsg(*a, p1);
  //  pcl::fromROSMsg(*b, p2);

  PointCloudF::Ptr sum(new PointCloudF(PointCloudF(*a) + PointCloudF(*b)));

  if (b_use_random_filter_) {
    const int n_points =
        static_cast<int>((1.0 - decimate_percentage_) * sum->size());
    pcl::RandomSample<PointF> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(sum);
    random_filter.filter(*sum);
  }
  if (b_use_radius_filter_) {
    pcl::RadiusOutlierRemoval<PointF> rad;
    rad.setInputCloud(sum);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*sum);
  }

  PublishMergedPointCloud(sum);
}

void PointCloudMerger::ThreePointCloudCallback(const PointCloudF::ConstPtr& a,
                                               const PointCloudF::ConstPtr& b,
                                               const PointCloudF::ConstPtr& c) {
  //  PointCloudF p1, p2, p3;
  //  pcl::fromROSMsg(*a, p1);
  //  pcl::fromROSMsg(*b, p2);
  //  pcl::fromROSMsg(*c, p3);

  PointCloudF::Ptr sum(
      new PointCloudF(PointCloudF(*a) + (PointCloudF(*b) + PointCloudF(*c))));

  if (b_use_random_filter_) {
    const int n_points =
        static_cast<int>((1.0 - decimate_percentage_) * sum->size());
    pcl::RandomSample<PointF> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(sum);
    random_filter.filter(*sum);
  }
  if (b_use_radius_filter_) {
    pcl::RadiusOutlierRemoval<PointF> rad;
    rad.setInputCloud(sum);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*sum);
  }

  PublishMergedPointCloud(sum);
}

// ---------------------------------------------------------------------------------------------

void PointCloudMerger::PublishMergedPointCloud(
    const PointCloudF::ConstPtr combined_pc) {
  if (merged_pcld_pub_.getNumSubscribers() != 0)
    merged_pcld_pub_.publish(*combined_pc);
}

// TODO: Unify
// ------------------------------------------------------------------------------------------------------

void PointCloudMerger::FailureDetectionCallback(
    const std_msgs::Int8& sensor_id) {
  ROS_INFO("PointCloudMerger - Received failure detection of sensor %d",
           sensor_id.data);

  alive_keys_.erase(
      std::remove(alive_keys_.begin(), alive_keys_.end(), sensor_id.data),
      alive_keys_.end());
  number_of_active_devices_ = alive_keys_.size();

  if (number_of_active_devices_ == 2) {
    three_sync_connection_.disconnect();
    pcld_synchronizer_2_ = std::unique_ptr<TwoPcldSynchronizer>(
        new TwoPcldSynchronizer(TwoPcldSyncPolicy(pcld_queue_size_),
                                *id_to_sub_map_[alive_keys_[0]],
                                *id_to_sub_map_[alive_keys_[1]]));
    two_sync_connection_ = pcld_synchronizer_2_->registerCallback(
        &PointCloudMerger::TwoPointCloudCallback, this);
  } else if (number_of_active_devices_ == 1) {
    two_sync_connection_.disconnect();
    auto topic = "pcld" + std::to_string(alive_keys_[0]);
    standard_pcld_sub_ =
        nl_.subscribe(topic, 1, &PointCloudMerger::OnePointCloudCallback, this);
  } else if (number_of_active_devices_ == 0) {
    ROS_ERROR("PointCloudMerger - No active lidar sensors");
    standard_pcld_sub_.shutdown();
  }
}

void PointCloudMerger::ResurrectionDetectionCallback(
    const std_msgs::Int8& sensor_id) {
  ROS_INFO("PointCloudMerger - Received resurrection detection of sensor %d",
           sensor_id.data);

  alive_keys_.push_back(sensor_id.data);
  number_of_active_devices_ = alive_keys_.size();

  if (number_of_active_devices_ == 3) {
    two_sync_connection_.disconnect();
    pcld_synchronizer_3_ = std::unique_ptr<ThreePcldSynchronizer>(
        new ThreePcldSynchronizer(ThreePcldSyncPolicy(pcld_queue_size_),
                                  *id_to_sub_map_[alive_keys_[0]],
                                  *id_to_sub_map_[alive_keys_[1]],
                                  *id_to_sub_map_[alive_keys_[2]]));
    three_sync_connection_ = pcld_synchronizer_3_->registerCallback(
        &PointCloudMerger::ThreePointCloudCallback, this);
  } else if (number_of_active_devices_ == 2) {
    standard_pcld_sub_.shutdown();
    pcld_synchronizer_2_ = std::unique_ptr<TwoPcldSynchronizer>(
        new TwoPcldSynchronizer(TwoPcldSyncPolicy(pcld_queue_size_),
                                *id_to_sub_map_[alive_keys_[0]],
                                *id_to_sub_map_[alive_keys_[1]]));
    two_sync_connection_ = pcld_synchronizer_2_->registerCallback(
        &PointCloudMerger::TwoPointCloudCallback, this);
  } else if (number_of_active_devices_ == 1) {
    auto topic = "pcld" + std::to_string(alive_keys_[0]);
    standard_pcld_sub_ =
        nl_.subscribe(topic, 1, &PointCloudMerger::OnePointCloudCallback, this);
  }
}

// ------------------------------------------------------------------------------------------------------------------
