#include <point_cloud_localization/utils.h>

void addNormal(const pcl::PointCloud<pcl::PointXYZI>& cloud,
               pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals,
               const int k_nearest_neighbours) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
      new pcl::PointCloud<pcl::PointXYZ>);
  // Convert point cloud XYZI to XYZ

  for (size_t i = 0; i < cloud.size(); i++) {
    pcl::PointXYZ point;
    point.x = cloud.points[i].x;
    point.y = cloud.points[i].y;
    point.z = cloud.points[i].z;
    cloud_xyz->points.push_back(point);
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud_xyz);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator(
      4); // parallel
  // TODO: if it's commented out => we don't use it => time to remove ?
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  // not
  // parallel

  normalEstimator.setInputCloud(cloud_xyz);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(k_nearest_neighbours);
  normalEstimator.compute(*normals);

  pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
}

// returns a point cloud whose centroid is the origin, and that the mean of
// the distances to the origin is 1
void normalizePCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr pclptr_normalized) {
  Eigen::Vector4f centroid_4d;
  pcl::compute3DCentroid(cloud, centroid_4d);
  Eigen::Vector3f centroid(centroid_4d.x(), centroid_4d.y(), centroid_4d.z());

  float dist = 0;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it =
           cloud.points.begin();
       it != cloud.points.end();
       it++) {
    Eigen::Vector3f a_i(it->x, it->y, it->z);
    dist = dist + (a_i - centroid).norm();
  }
  float factor = cloud.points.size() / dist;
  Eigen::Matrix4f transform;
  transform = Eigen::Matrix4f::Identity();
  transform.block(0, 0, 3, 3) = factor * Eigen::Matrix3f::Identity();
  transform.block(0, 3, 4, 1) = -factor * centroid_4d;
  pcl::transformPointCloud(cloud, *pclptr_normalized, transform);
  pcl::compute3DCentroid(*pclptr_normalized, centroid_4d);

  // checkIfPCloudIsNormalized(pclptr_normalized);  //
}

void doEigenDecomp6x6(const Eigen::Matrix<double, 6, 6>& data,
                      Eigen::Matrix<double, 6, 1>& eigenvalues,
                      Eigen::Matrix<double, 6, 6>& eigenvectors) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(data);
  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors =
        eigensolver.eigenvectors(); // the eigenvectors are the columns
  } else {
    std::cout << "Eigen failed to do the eigendecomp" << std::endl;
  }
}

void doEigenDecomp3x3(const Eigen::Matrix<double, 3, 3>& data,
                      Eigen::Matrix<double, 3, 1>& eigenvalues,
                      Eigen::Matrix<double, 3, 3>& eigenvectors) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver(data);
  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors =
        eigensolver.eigenvectors(); // the eigenvectors are the columns
  } else {
    std::cout << "Eigen failed to do the eigendecomp" << std::endl;
  }
}
