#pragma once
#include <pcl_ros/point_cloud.h>
#include <vector>

typedef pcl::PointXYZINormal PointF;
typedef pcl::PointCloud<PointF> PointCloudF;
typedef pcl::PointCloud<PointF>::ConstPtr pclConstPtr;
typedef pcl::PointCloud<PointF>::Ptr pclPtr;

typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>
    MatricesVector;
typedef boost::shared_ptr<MatricesVector> MatricesVectorPtr;

typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>
    MatricesVectorf;
typedef boost::shared_ptr<MatricesVectorf> MatricesVectorfPtr;

template <typename T>
using MatricesVectorT =
    std::vector<Eigen::Matrix<T, 3, 3>,
                Eigen::aligned_allocator<Eigen::Matrix<T, 3, 3>>>;

template <typename T>
using MatricesVectorTPtr = boost::shared_ptr<MatricesVectorT<T>>;
