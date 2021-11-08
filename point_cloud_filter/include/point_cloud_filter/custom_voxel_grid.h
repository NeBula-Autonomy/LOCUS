#pragma once

// PCL includes
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/filters/filter.h>
#include <ros/ros.h>
// Dynamic reconfigure
#include <point_cloud_filter/CustomVoxelGridConfig.h>

namespace point_cloud_filter {
/** \brief @b VoxelGrid assembles a local 3D grid over a given PointCloud, and
 * downsamples + filters the data. \author Radu Bogdan Rusu
 */
class CustomVoxelGrid : public pcl_ros::Filter {
protected:
  /** \brief Pointer to a dynamic reconfigure service. */
  boost::shared_ptr<dynamic_reconfigure::Server<
      point_cloud_filter::CustomVoxelGridConfig>>
      srv_;

  /** \brief The PCL filter implementation used. */
  pcl::VoxelGrid<pcl::PCLPointCloud2> impl_;

  /** \brief Call the actual filter.
   * \param input the input point cloud dataset
   * \param indices the input set of indices to use from \a input
   * \param output the resultant filtered dataset
   */
  virtual void filter(const PointCloud2::ConstPtr& input,
                      const IndicesPtr& indices,
                      PointCloud2& output);

  void ChangeLeafSizeRostopic(const std_msgs::Float64::ConstPtr leaf_size);

  /** \brief Child initialization routine.
   * \param nh ROS node handle
   * \param has_service set to true if the child has a Dynamic Reconfigure
   * service
   */
  bool child_init(ros::NodeHandle& nh, bool& has_service);

  /** \brief Dynamic reconfigure callback
   * \param config the config object
   * \param level the dynamic reconfigure level
   */
  void config_callback(point_cloud_filter::CustomVoxelGridConfig& config,
                       uint32_t level);

  ros::Subscriber change_leaf_size_sub;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace point_cloud_filter
