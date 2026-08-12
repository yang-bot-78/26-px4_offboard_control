#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace plan_env
{

// Read-only index retaining raw point coordinates instead of voxel indices.
class RawObstacleDistanceIndex
{
public:
  void rebuild(const pcl::PointCloud<pcl::PointXYZ> & input)
  {
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_->reserve(input.size());
    for (const auto & point : input.points)
      if (pcl::isFinite(point)) cloud_->push_back(point);
    tree_.setInputCloud(cloud_);
  }

  bool nearest(const Eigen::Vector3d & position, Eigen::Vector3d & nearest_point,
               double & distance) const
  {
    distance = std::numeric_limits<double>::infinity();
    nearest_point = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    if (!position.allFinite() || !cloud_ || cloud_->empty()) return false;
    pcl::PointXYZ query;
    query.x = static_cast<float>(position.x());
    query.y = static_cast<float>(position.y());
    query.z = static_cast<float>(position.z());
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    if (tree_.nearestKSearch(query, 1, indices, squared_distances) != 1) return false;
    const auto & point = cloud_->points[indices.front()];
    nearest_point = Eigen::Vector3d(point.x, point.y, point.z);
    distance = std::sqrt(static_cast<double>(squared_distances.front()));
    return true;
  }

  std::size_t size() const { return cloud_ ? cloud_->size() : 0U; }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
};

}  // namespace plan_env
