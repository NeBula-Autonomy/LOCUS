/*
point_cloud_localization/utils.h
author: Yun Chang yunchang@mit.edu

Utilitiy functions for point cloud localization and ICP
*/

#ifndef UTILS_H
#define UTILS_H

#include <frontend_utils/CommonStructs.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void addNormal(const PointCloudF& cloud,
               pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals,
               const int k_nearest_neighbours);

void addNormal(const PointCloudF& cloud,
               PointCloudF::Ptr& cloud_with_normals,
               const int k_nearest_neighbours);

// returns a point cloud whose centroid is the origin, and that the mean of the
// distances to the origin is 1
void normalizePCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr pclptr_normalized);
void normalizePCloud(const PointCloudF& cloud,
                     PointCloudF::Ptr pclptr_normalized);

void doEigenDecomp6x6(const Eigen::Matrix<double, 6, 6>& data,
                      Eigen::Matrix<double, 6, 1>& eigenvalues,
                      Eigen::Matrix<double, 6, 6>& eigenvectors);

void doEigenDecomp3x3(const Eigen::Matrix<double, 3, 3>& data,
                      Eigen::Matrix<double, 3, 1>& eigenvalues,
                      Eigen::Matrix<double, 3, 3>& eigenvectors);
#endif
