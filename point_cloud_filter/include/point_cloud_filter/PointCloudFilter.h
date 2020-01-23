/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

// Point label options
enum PointLabel
{
    CORNER_SHARP = 2,      ///< sharp corner point
    CORNER_LESS_SHARP = 1, ///< less sharp corner point
    SURFACE_LESS_FLAT = 0, ///< less flat surface point
    SURFACE_FLAT = -1      ///< flat surface point
};

class PointCloudFilter {

 public:

  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr pclConstPtr;
  typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pclPtr;
  typedef std::pair<size_t, size_t> IndexRange;

  PointCloudFilter();
  ~PointCloudFilter();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either
  bool Initialize(const ros::NodeHandle& n);

  // Filter an incoming point cloud
  bool Filter(const PointCloud::ConstPtr& points,
              PointCloud::Ptr points_filtered, 
              const bool b_is_open_space);

  // VLP16 characteristics
  const float lowerBound_ = -15;
  const float upperBound_ = 15;
  const uint16_t nScanRings_ = 16;
  // linear interpolation factor
  float factor_ = (nScanRings_ - 1) / (upperBound_ - lowerBound_);       

 private:

  struct Configuration {
    const int curvatureRegion = 5;
    const int nFeatureRegions = 6;
    const int maxCornerSharp = 2; // 2 * 6(feature_regions) * 16(rings) = 192 sharp features
    const int maxCornerLessSharp = 10 * maxCornerSharp; //1920 less sharp features
    const int maxSurfaceFlat = 4;
    const float surfaceCurvatureThreshold = 0.1;
    const float lessFlatFilterSize = 0.2;
  } feature_config_;  

  // Node initialization
  bool LoadParameters(const ros::NodeHandle& n);
  std::string name_;
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans_;
  pcl::PointCloud<pcl::PointXYZI> laserCloud_;            // < full resolution input cloud - smk:contains all points but now ring-wise sorted
  std::vector<IndexRange> scanIndices_;                   // < start and end indices of the individual scans withing the full resolution cloud
  pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp_;     // < sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp_; // < less sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI> surfacePointsFlat_;     // < flat surface points cloud
  pcl::PointCloud<pcl::PointXYZI> surfacePointsLessFlat_; // < less flat surface points cloud
  std::vector<float> regionCurvature_;                    // < point curvature buffer
  std::vector<PointLabel> regionLabel_;                   // < point label buffer
  std::vector<size_t> regionSortIndices_;                 // < sorted region indices based on point curvature
  std::vector<int> scanNeighborPicked_;                   // < flag if neighboring point was already picked

  void arrangePCLInScanLines(const PointCloud &laserCloudIn, float scanPeriod);
  void extractFeatures(const uint16_t &beginIdx = 0);
  void setRegionBuffersFor(const size_t &startIdx, const size_t &endIdx);
  void setScanBuffersFor(const size_t &startIdx, const size_t &endIdx);
  void markAsPicked(const size_t &cloudIdx, const size_t &scanIdx);

  /** \brief Map the specified vertical point angle to its ring ID
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
  int getRingForAngle(const float &angle) const;

  // Calculate the squared difference of the given two points
  template <typename PointT>
  inline float calcSquaredDiff(const PointT &a, const PointT &b)
  {
      float diffX = a.x - b.x;
      float diffY = a.y - b.y;
      float diffZ = a.z - b.z;
      return diffX * diffX + diffY * diffY + diffZ * diffZ;
  }

  //Calculate the squared difference of the given two points with weighting
  template <typename PointT>
  inline float calcSquaredDiff(const PointT &a, const PointT &b, const float &wb)
  {
      float diffX = a.x - b.x * wb;
      float diffY = a.y - b.y * wb;
      float diffZ = a.z - b.z * wb;
      return diffX * diffX + diffY * diffY + diffZ * diffZ;
  }

  // Calculate the absolute distance of the point to the origin
  template <typename PointT>
  inline float calcPointDistance(const PointT &p)
  {
      return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  }

  // Calculate the squared distance of the point to the origin
  template <typename PointT>
  inline float calcSquaredPointDistance(const PointT &p)
  {
      return p.x * p.x + p.y * p.y + p.z * p.z;
  }

  struct Parameters {
    // Apply feature extraction
    bool extract_features;
    // Apply a voxel grid filter.
    bool grid_filter;
    // Resolution of voxel grid filter.
    double grid_res;
    // Apply a random downsampling filter.
    bool random_filter;
    // Percentage of points to discard. Must be between 0.0 and 1.0;
    double decimate_percentage;
    // Percentage of points to discard when in open space. Must be between 0.0 and 1.0;
    double decimate_percentage_open_space;
    // Apply a statistical outlier filter.
    bool outlier_filter;
    // Standard deviation threshold in distance to neighbors for outlier
    // removal.
    double outlier_std;
    // Number of nearest neighbors to use for outlier filter.
    unsigned int outlier_knn;
    // Apply a radius outlier filter.
    bool radius_filter;
    // Size of the radius filter.
    double radius;
    // If this number of neighbors are not found within a radius around each point, remove that point
    unsigned int radius_knn;
  } params_;

};

#endif