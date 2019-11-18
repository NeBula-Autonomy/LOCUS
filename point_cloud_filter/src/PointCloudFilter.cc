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

#include <point_cloud_filter/PointCloudFilter.h>
#include <parameter_utils/ParameterUtils.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace pu = parameter_utils;

PointCloudFilter::PointCloudFilter() {}
PointCloudFilter::~PointCloudFilter() {}

bool PointCloudFilter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudFilter");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudFilter::LoadParameters(const ros::NodeHandle& n) {
  // Load filtering parameters.
  if (!pu::Get("filtering/grid_filter", params_.grid_filter)) return false;
  if (!pu::Get("filtering/grid_res", params_.grid_res)) return false;

  if (!pu::Get("filtering/random_filter", params_.random_filter)) return false;
  if (!pu::Get("filtering/decimate_percentage", params_.decimate_percentage))
    return false;

  if (!pu::Get("filtering/outlier_filter", params_.outlier_filter)) return false;
  if (!pu::Get("filtering/outlier_std", params_.outlier_std)) return false;
  if (!pu::Get("filtering/outlier_knn", params_.outlier_knn)) return false;

  if (!pu::Get("filtering/radius_filter", params_.radius_filter)) return false;
  if (!pu::Get("filtering/radius", params_.radius)) return false;
  if (!pu::Get("filtering/radius_knn", params_.radius_knn)) return false;
  if (!pu::Get("filtering/extract_features", params_.extract_features)) return false;

  // Cap to [0.0, 1.0].
  params_.decimate_percentage =
      std::min(1.0, std::max(0.0, params_.decimate_percentage));

  return true;
}

bool PointCloudFilter::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  return true;
}

bool PointCloudFilter::Filter(const PointCloud::ConstPtr& points,
                              PointCloud::Ptr points_filtered) {
  if (points_filtered == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Copy input points.
  *points_filtered = *points; 
  if (!params_.extract_features){
    // Apply a random downsampling filter to the incoming point cloud.
    if (params_.random_filter) {
      const int n_points = static_cast<int>((1.0 - params_.decimate_percentage) *
                                            points_filtered->size());
      pcl::RandomSample<pcl::PointXYZ> random_filter;
      random_filter.setSample(n_points);
      random_filter.setInputCloud(points_filtered);
      random_filter.filter(*points_filtered);
    }

    // Apply a voxel grid filter to the incoming point cloud.
    if (params_.grid_filter) {
      pcl::VoxelGrid<pcl::PointXYZ> grid;
      grid.setLeafSize(params_.grid_res, params_.grid_res, params_.grid_res);
      grid.setInputCloud(points_filtered);
      grid.filter(*points_filtered);
    }

    // Remove statistical outliers in incoming the point cloud.
    if (params_.outlier_filter) {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(points_filtered);
      sor.setMeanK(params_.outlier_knn);
      sor.setStddevMulThresh(params_.outlier_std);
      sor.filter(*points_filtered);
    }

    // Remove points without a threshold number of neighbors within a specified
    // radius.
    if (params_.radius_filter) {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> rad;
      rad.setInputCloud(points_filtered);
      rad.setRadiusSearch(params_.radius);
      rad.setMinNeighborsInRadius(params_.radius_knn);
      rad.filter(*points_filtered);
    }
  // Downsample point cloud by extracting features  
  } else {
        arrangePCLInScanLines(*points_filtered, 0.1); // Todo: the VLP scan period should be a parameter set by user

        //extract features
        extractFeatures();
  }
  return true;
}

// Arrange the pointcloud in Scanlines
void PointCloudFilter::arrangePCLInScanLines(const PointCloud &laserCloudIn, float scanPeriod)
{
    size_t cloudSize = laserCloudIn.size();

    // determine scan start and end orientations
    float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
    float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y, laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    pcl::PointXYZI point;
    laserCloudScans_.clear();
    laserCloudScans_.resize(nScanRings_);

    // extract valid points from input cloud
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn[i].y;
        point.y = laserCloudIn[i].z;
        point.z = laserCloudIn[i].x;

        // skip NaN and INF valued points
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }

        // skip zero valued points
        if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001)
        {
            continue;
        }

        // calculate vertical point angle and scan ID
        float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
        int scanID = getRingForAngle(angle);
        if (scanID >= nScanRings_ || scanID < 0)
        {
            continue;
        }

        // calculate horizontal point angle
        float ori = -std::atan2(point.x, point.z);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;

            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        // Given the orientation of the current point and knowning the start/end orientations and total scan time 
        // we compute the relative scan time at which this point was scanned when the lidar was doing its sweep.
        float relTime = scanPeriod * (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + relTime;
        laserCloudScans_[scanID].push_back(point);
    }

    // construct sorted full resolution cloud
    laserCloud_.clear();
    scanIndices_.clear();
    size_t cloudSizeScans = 0;
    for (int i = 0; i < laserCloudScans_.size(); i++)
    {
        laserCloud_ += laserCloudScans_[i];

        IndexRange range(cloudSizeScans, 0);
        cloudSizeScans += laserCloudScans_[i].size();
        range.second = cloudSizeScans > 0 ? cloudSizeScans - 1 : 0;
        scanIndices_.push_back(range);
    }
}

//Extract Feature Clouds from Scan Lines
void PointCloudFilter::extractFeatures(const uint16_t &beginIdx)
{
    //Clear out old features
    cornerPointsSharp_.clear();
    cornerPointsLessSharp_.clear();
    surfacePointsFlat_.clear();
    surfacePointsLessFlat_.clear();
    regionCurvature_.clear();
    regionLabel_.clear();
    regionSortIndices_.clear();
    scanNeighborPicked_.clear();

    // extract features from individual scans
    size_t nScans = scanIndices_.size();
    for (size_t i = beginIdx; i < nScans; i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
        size_t scanStartIdx = scanIndices_[i].first;
        size_t scanEndIdx = scanIndices_[i].second;

        // skip empty scans
        if (scanEndIdx <= scanStartIdx + 2 * feature_config_.curvatureRegion)
        {
            continue;
        }

        // reset scan buffers
        setScanBuffersFor(scanStartIdx, scanEndIdx);

        // extract features from equally sized scan regions
        for (int j = 0; j < feature_config_.nFeatureRegions; j++)
        {
            size_t sp = ((scanStartIdx + feature_config_.curvatureRegion) * (feature_config_.nFeatureRegions - j) + (scanEndIdx - feature_config_.curvatureRegion) * j) / feature_config_.nFeatureRegions;
            size_t ep = ((scanStartIdx + feature_config_.curvatureRegion) * (feature_config_.nFeatureRegions - 1 - j) + (scanEndIdx - feature_config_.curvatureRegion) * (j + 1)) / feature_config_.nFeatureRegions - 1;

            // skip empty regions
            if (ep <= sp)
            {
                continue;
            }

            size_t regionSize = ep - sp + 1;

            // reset region buffers 
            setRegionBuffersFor(sp, ep);
            // Calculate the curvature for points in the scans
            // Extract corner features
            int largestPickedNum = 0;
            for (size_t k = regionSize; k > 0 && largestPickedNum < feature_config_.maxCornerLessSharp;)
            {
                size_t idx = regionSortIndices_[--k];
                size_t scanIdx = idx - scanStartIdx;
                size_t regionIdx = idx - sp;

                // Pick sharp points, the sharp and less sharp need to be above "surfaceCurvatureThreshold"
                // Technically there is no difference between the two. Some of them are marked are sharp to reduce data for laser odometry
                if (scanNeighborPicked_[scanIdx] == 0 && regionCurvature_[regionIdx] > feature_config_.surfaceCurvatureThreshold)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= feature_config_.maxCornerSharp)
                    {
                        regionLabel_[regionIdx] = CORNER_SHARP;
                        cornerPointsSharp_.push_back(laserCloud_[idx]);
                    }
                    else
                    {
                        regionLabel_[regionIdx] = CORNER_LESS_SHARP;
                    }
                    cornerPointsLessSharp_.push_back(laserCloud_[idx]);

                    markAsPicked(idx, scanIdx);
                }
            }

            // Extract flat surface features
            // A certain number of points that are less than "surfaceCurvatureThreshold" are marked as Flat
            int smallestPickedNum = 0;
            for (int k = 0; k < regionSize && smallestPickedNum < feature_config_.maxSurfaceFlat; k++)
            {
                size_t idx = regionSortIndices_[k];
                size_t scanIdx = idx - scanStartIdx;
                size_t regionIdx = idx - sp;

                if (scanNeighborPicked_[scanIdx] == 0 && regionCurvature_[regionIdx] < feature_config_.surfaceCurvatureThreshold)
                {
                    smallestPickedNum++;
                    regionLabel_[regionIdx] = SURFACE_FLAT;
                    surfacePointsFlat_.push_back(laserCloud_[idx]);

                    markAsPicked(idx, scanIdx);
                }
            }

            // Extract less flat surface features
            for (int k = 0; k < regionSize; k++)
            {
                if (regionLabel_[k] <= SURFACE_LESS_FLAT)
                {
                    surfPointsLessFlatScan->push_back(laserCloud_[sp + k]);
                }
            }
        }

        // down size less flat surface point cloud of current scan
        pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(feature_config_.lessFlatFilterSize, feature_config_.lessFlatFilterSize, feature_config_.lessFlatFilterSize);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfacePointsLessFlat_ += surfPointsLessFlatScanDS;
    }
}

void PointCloudFilter::setRegionBuffersFor(const size_t &startIdx, const size_t &endIdx)
{
    // resize buffers
    size_t regionSize = endIdx - startIdx + 1;
    regionCurvature_.resize(regionSize);
    regionSortIndices_.resize(regionSize);
    // Initially every point is considered a SURFACE_LESS_FLAT
    regionLabel_.assign(regionSize, SURFACE_LESS_FLAT); 

    // calculate point curvatures and reset sort indices
    // looping through # number of neighbor points(curvatureRegion) and calculate squared difference
    float pointWeight = -2 * feature_config_.curvatureRegion;

    for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++)
    {
        float diffX = pointWeight * laserCloud_[i].x;
        float diffY = pointWeight * laserCloud_[i].y;
        float diffZ = pointWeight * laserCloud_[i].z;

        for (int j = 1; j <= feature_config_.curvatureRegion; j++)
        {
            diffX += laserCloud_[i + j].x + laserCloud_[i - j].x;
            diffY += laserCloud_[i + j].y + laserCloud_[i - j].y;
            diffZ += laserCloud_[i + j].z + laserCloud_[i - j].z;
        }

        regionCurvature_[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        regionSortIndices_[regionIdx] = i;
    }

    // sort point curvatures
    for (size_t i = 1; i < regionSize; i++)
    {
        for (size_t j = i; j >= 1; j--)
        {
            if (regionCurvature_[regionSortIndices_[j] - startIdx] < regionCurvature_[regionSortIndices_[j - 1] - startIdx])
            {
                std::swap(regionSortIndices_[j], regionSortIndices_[j - 1]);
            }
        }
    }
}

void PointCloudFilter::setScanBuffersFor(const size_t &startIdx, const size_t &endIdx)
{
    // resize buffers
    size_t scanSize = endIdx - startIdx + 1;
    scanNeighborPicked_.assign(scanSize, 0);

    // Mark unreliable points as picked 
    // Points that have depth difference greater than 0.1 meters are set as already picked so they can be ignored later
    for (size_t i = startIdx + feature_config_.curvatureRegion; i < endIdx - feature_config_.curvatureRegion; i++)
    {
        const pcl::PointXYZI &previousPoint = (laserCloud_[i - 1]);
        const pcl::PointXYZI &point = (laserCloud_[i]);
        const pcl::PointXYZI &nextPoint = (laserCloud_[i + 1]);

        float diffNext = calcSquaredDiff(nextPoint, point);

        if (diffNext > 0.1)
        {
            float depth1 = calcPointDistance(point);
            float depth2 = calcPointDistance(nextPoint);

            if (depth1 > depth2)
            {
                float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

                if (weighted_distance < 0.1)
                {
                    std::fill_n(&scanNeighborPicked_[i - startIdx - feature_config_.curvatureRegion], feature_config_.curvatureRegion + 1, 1);

                    continue;
                }
            }
            else
            {
                float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

                if (weighted_distance < 0.1)
                {
                    std::fill_n(&scanNeighborPicked_[i - startIdx + 1], feature_config_.curvatureRegion + 1, 1);
                }
            }
        }  

        float diffPrevious = calcSquaredDiff(point, previousPoint);
        float dis = calcSquaredPointDistance(point);

        // This will reject a point if the difference in distance or depth of point 
        // and its neighbors is outside a bound i.e. rejecting very sharp ramp like points
        if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) 
        {
            scanNeighborPicked_[i - startIdx] = 1;
        }
    }
}

void PointCloudFilter::markAsPicked(const size_t &cloudIdx, const size_t &scanIdx)
{
    scanNeighborPicked_[scanIdx] = 1;

    // Not only mark the point as picked but mark all the points 
    // used in the curvatureRegion around it as picked as well
    for (int i = 1; i <= feature_config_.curvatureRegion; i++)
    {
        if (calcSquaredDiff(laserCloud_[cloudIdx + i], laserCloud_[cloudIdx + i - 1]) > 0.05)
        {
            break;
        }

        scanNeighborPicked_[scanIdx + i] = 1;
    }

    for (int i = 1; i <= feature_config_.curvatureRegion; i++)
    {
        if (calcSquaredDiff(laserCloud_[cloudIdx - i], laserCloud_[cloudIdx - i + 1]) > 0.05)
        {
            break;
        }

        scanNeighborPicked_[scanIdx - i] = 1;
    }
}


int PointCloudFilter::getRingForAngle(const float &angle) const
{
    return int(((angle * 180 / M_PI) - lowerBound_) * factor_ + 0.5);
}
