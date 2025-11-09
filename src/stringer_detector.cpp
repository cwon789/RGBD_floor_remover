#include "floor_removal_rgbd/stringer_detector.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <algorithm>
#include <iostream>

namespace floor_removal_rgbd
{

StringerDetector::StringerDetector(const StringerDetectorParams& params)
  : params_(params)
{
}

void StringerDetector::setParams(const StringerDetectorParams& params)
{
  params_ = params;
}

StringerDetectionResult StringerDetector::detect(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  StringerDetectionResult result;

  if (!cloud || cloud->points.empty()) {
    std::cout << "[StringerDetector] Input cloud is empty" << std::endl;
    return result;
  }

  std::cout << "[StringerDetector] Processing cloud with " << cloud->points.size() << " points" << std::endl;

  // Step 1: Apply downsampling if needed
  auto cloud_for_clustering = downsampleForClustering(cloud);

  // Step 2: Perform clustering
  auto cluster_indices = clusterPointCloud(cloud_for_clustering);
  std::cout << "[StringerDetector] Found " << cluster_indices.size() << " clusters" << std::endl;

  // Step 3: Analyze each cluster
  for (const auto& indices : cluster_indices) {
    if (indices.indices.empty()) {
      continue;
    }

    // Compute bounding box and centroid
    BoundingBox bbox = computeBoundingBox(cloud_for_clustering, indices);

    // Check if dimensions match stringer criteria
    if (matchesStringerCriteria(bbox)) {
      std::cout << "[StringerDetector] ✓ Stringer detected: "
                << "W=" << bbox.width() << "m, "
                << "H=" << bbox.height() << "m, "
                << "D=" << bbox.depth() << "m" << std::endl;

      result.detected_stringers.push_back(bbox);

      // Add center point
      pcl::PointXYZRGB center;
      center.x = bbox.centroid_x;
      center.y = bbox.centroid_y;
      center.z = bbox.centroid_z;
      center.r = 0;
      center.g = 255;
      center.b = 0;
      result.stringer_centers->points.push_back(center);
    }
  }

  result.stringer_centers->width = result.stringer_centers->points.size();
  result.stringer_centers->height = 1;
  result.stringer_centers->is_dense = false;

  std::cout << "[StringerDetector] Total stringers detected: " << result.detected_stringers.size() << std::endl;

  return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StringerDetector::downsampleForClustering(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  if (!params_.use_additional_downsampling ||
      cloud->points.size() <= params_.max_points_for_clustering) {
    return cloud;
  }

  std::cout << "[StringerDetector] Cloud too large (" << cloud->points.size()
            << " points), applying downsampling for clustering" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

  try {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(
      params_.downsampling_voxel_size,
      params_.downsampling_voxel_size,
      params_.downsampling_voxel_size);
    voxel_filter.filter(*cloud_downsampled);

    std::cout << "[StringerDetector] Downsampled to " << cloud_downsampled->points.size() << " points" << std::endl;

    if (cloud_downsampled->points.empty()) {
      return cloud;
    }

    return cloud_downsampled;
  } catch (const std::exception& e) {
    std::cout << "[StringerDetector] Downsampling failed, using original cloud" << std::endl;
    return cloud;
  }
}

std::vector<pcl::PointIndices> StringerDetector::clusterPointCloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  std::vector<pcl::PointIndices> cluster_indices;

  try {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(params_.cluster_tolerance);
    ec.setMinClusterSize(params_.min_cluster_size);
    ec.setMaxClusterSize(params_.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
  } catch (const std::exception& e) {
    std::cout << "[StringerDetector] Clustering failed: " << e.what() << std::endl;
  }

  return cluster_indices;
}

BoundingBox StringerDetector::computeBoundingBox(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const pcl::PointIndices& indices)
{
  BoundingBox bbox;
  bbox.min_x = bbox.min_y = bbox.min_z = 1e6;
  bbox.max_x = bbox.max_y = bbox.max_z = -1e6;

  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  size_t num_points = indices.indices.size();

  for (const auto& idx : indices.indices) {
    const auto& point = cloud->points[idx];

    // Update bounding box
    bbox.min_x = std::min(bbox.min_x, static_cast<double>(point.x));
    bbox.max_x = std::max(bbox.max_x, static_cast<double>(point.x));
    bbox.min_y = std::min(bbox.min_y, static_cast<double>(point.y));
    bbox.max_y = std::max(bbox.max_y, static_cast<double>(point.y));
    bbox.min_z = std::min(bbox.min_z, static_cast<double>(point.z));
    bbox.max_z = std::max(bbox.max_z, static_cast<double>(point.z));

    // Accumulate for centroid
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
  }

  // Compute centroid (average of all points in cluster)
  bbox.centroid_x = sum_x / num_points;
  bbox.centroid_y = sum_y / num_points;
  bbox.centroid_z = sum_z / num_points;

  return bbox;
}

bool StringerDetector::matchesStringerCriteria(const BoundingBox& bbox)
{
  // Camera frame: X=right, Y=down, Z=forward
  double width_x = bbox.width();   // horizontal (left-right)
  double height_y = bbox.height(); // vertical (floor to top)
  double depth_z = bbox.depth();   // horizontal (front-back)

  // Stringer check: height must be in range,
  // and at least one horizontal dimension (X or Z) must be in width range
  bool height_ok = (height_y >= params_.height_min && height_y <= params_.height_max);
  bool width_x_ok = (width_x >= params_.width_min && width_x <= params_.width_max);
  bool width_z_ok = (depth_z >= params_.width_min && depth_z <= params_.width_max);

  if (height_ok && (width_x_ok || width_z_ok)) {
    return true;
  }

  // Debug rejected clusters
  std::cout << "[StringerDetector] ✗ Rejected cluster: "
            << "H=" << height_y << "m (" << params_.height_min << "-" << params_.height_max << "), "
            << "X=" << width_x << "m, "
            << "Z=" << depth_z << "m (" << params_.width_min << "-" << params_.width_max << ")" << std::endl;

  return false;
}

}  // namespace floor_removal_rgbd
