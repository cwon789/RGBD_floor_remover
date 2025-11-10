#include "floor_removal_rgbd/stringer_detector.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <algorithm>
#include <iostream>
#include <cmath>

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

  // Step 3: Analyze each cluster - detect columns based on voxel connectivity
  for (const auto& indices : cluster_indices) {
    if (indices.indices.empty()) {
      continue;
    }

    // Detect columns (linear segments) in this cluster
    // detectColumnsInCluster already filters by criteria
    auto columns = detectColumnsInCluster(cloud_for_clustering, indices);

    // Track horizontal and vertical components in this cluster
    const DetectedColumn* horizontal_col = nullptr;
    const DetectedColumn* vertical_col = nullptr;

    // Add all detected columns (already filtered)
    for (const auto& column : columns) {
      std::cout << "[StringerDetector] ✓ Stringer column detected: "
                << (column.is_horizontal ? "[HORIZONTAL] " : "[VERTICAL] ")
                << "Length=" << column.length << "m, "
                << "Height=" << column.height << "m, "
                << "Points=" << column.num_points << std::endl;

      result.detected_columns.push_back(column);

      // Track which components this cluster has
      if (column.is_horizontal) {
        horizontal_col = &result.detected_columns.back();
      }
      if (column.is_vertical) {
        vertical_col = &result.detected_columns.back();
      }

      // Add center point
      pcl::PointXYZRGB center;
      center.x = column.center_x;
      center.y = column.center_y;
      center.z = column.center_z;
      if (column.is_horizontal) {
        center.r = 0;
        center.g = 255;
        center.b = 0;  // Green for horizontal
      } else {
        center.r = 0;
        center.g = 0;
        center.b = 255;  // Blue for vertical
      }
      result.stringer_centers->points.push_back(center);

      // For backward compatibility, create a bounding box
      BoundingBox bbox;
      bbox.centroid_x = column.center_x;
      bbox.centroid_y = column.center_y;
      bbox.centroid_z = column.center_z;
      result.detected_stringers.push_back(bbox);
    }

    // Step 3.5: Determine intersection point for this cluster
    pcl::PointXYZRGB intersection_point;

    if (horizontal_col && vertical_col) {
      // Both horizontal and vertical detected - use intersection point
      intersection_point.x = (horizontal_col->center_x + vertical_col->center_x) / 2.0;
      intersection_point.y = (horizontal_col->center_y + vertical_col->center_y) / 2.0;
      intersection_point.z = (horizontal_col->center_z + vertical_col->center_z) / 2.0;
      intersection_point.r = 255;
      intersection_point.g = 255;
      intersection_point.b = 0;  // Yellow for intersections

      result.intersection_points->points.push_back(intersection_point);

      std::cout << "[StringerDetector] ✓ Intersection (both H+V) at ("
                << intersection_point.x << ", " << intersection_point.y << ", "
                << intersection_point.z << ")" << std::endl;
    }
    else if (horizontal_col) {
      // Only horizontal detected - use its center
      intersection_point.x = horizontal_col->center_x;
      intersection_point.y = horizontal_col->center_y;
      intersection_point.z = horizontal_col->center_z;
      intersection_point.r = 0;
      intersection_point.g = 255;
      intersection_point.b = 0;  // Green for horizontal-only

      result.intersection_points->points.push_back(intersection_point);

      std::cout << "[StringerDetector] ✓ Center point (H-only) at ("
                << intersection_point.x << ", " << intersection_point.y << ", "
                << intersection_point.z << ")" << std::endl;
    }
    else if (vertical_col) {
      // Only vertical detected - use its center
      intersection_point.x = vertical_col->center_x;
      intersection_point.y = vertical_col->center_y;
      intersection_point.z = vertical_col->center_z;
      intersection_point.r = 0;
      intersection_point.g = 0;
      intersection_point.b = 255;  // Blue for vertical-only

      result.intersection_points->points.push_back(intersection_point);

      std::cout << "[StringerDetector] ✓ Center point (V-only) at ("
                << intersection_point.x << ", " << intersection_point.y << ", "
                << intersection_point.z << ")" << std::endl;
    }
  }

  result.stringer_centers->width = result.stringer_centers->points.size();
  result.stringer_centers->height = 1;
  result.stringer_centers->is_dense = false;

  result.intersection_points->width = result.intersection_points->points.size();
  result.intersection_points->height = 1;
  result.intersection_points->is_dense = false;

  std::cout << "[StringerDetector] Total stringers detected: " << result.detected_stringers.size() << std::endl;
  std::cout << "[StringerDetector] Total intersections found: " << result.intersection_points->points.size() << std::endl;

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
  // Robot frame: X=forward, Y=left, Z=up
  double depth_x = bbox.width();   // horizontal (front-back)
  double width_y = bbox.height();  // horizontal (left-right)
  double height_z = bbox.depth();  // vertical (floor to top)

  // Stringer check: height must be in range,
  // and at least one horizontal dimension (X or Y) must be in width range
  bool height_ok = (height_z >= params_.height_min && height_z <= params_.height_max);
  bool width_x_ok = (depth_x >= params_.width_min && depth_x <= params_.width_max);
  bool width_y_ok = (width_y >= params_.width_min && width_y <= params_.width_max);

  if (height_ok && (width_x_ok || width_y_ok)) {
    return true;
  }

  // Debug rejected clusters
  std::cout << "[StringerDetector] ✗ Rejected cluster: "
            << "H(Z)=" << height_z << "m (" << params_.height_min << "-" << params_.height_max << "), "
            << "W(X)=" << depth_x << "m, "
            << "W(Y)=" << width_y << "m (" << params_.width_min << "-" << params_.width_max << ")" << std::endl;

  return false;
}

std::vector<DetectedPlane> StringerDetector::detectPlanesInCluster(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const pcl::PointIndices& indices)
{
  std::vector<DetectedPlane> planes;

  // Extract cluster points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto& idx : indices.indices) {
    cluster_cloud->points.push_back(cloud->points[idx]);
  }
  cluster_cloud->width = cluster_cloud->points.size();
  cluster_cloud->height = 1;
  cluster_cloud->is_dense = false;

  if (cluster_cloud->points.size() < static_cast<size_t>(params_.min_plane_inliers)) {
    return planes;
  }

  // Run RANSAC to detect a plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params_.ransac_max_iterations);
  seg.setDistanceThreshold(params_.ransac_distance_threshold);
  seg.setInputCloud(cluster_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() < static_cast<size_t>(params_.min_plane_inliers) || coefficients->values.size() != 4) {
    return planes;
  }

  // Extract plane parameters
  DetectedPlane plane;
  plane.nx = coefficients->values[0];
  plane.ny = coefficients->values[1];
  plane.nz = coefficients->values[2];
  plane.d = coefficients->values[3];
  plane.num_inliers = inliers->indices.size();

  // Determine orientation (Robot frame: X=forward, Y=left, Z=up)
  double abs_nx = std::abs(plane.nx);
  double abs_ny = std::abs(plane.ny);
  double abs_nz = std::abs(plane.nz);

  plane.is_vertical = (abs_nx > params_.vertical_normal_threshold ||
                       abs_ny > params_.vertical_normal_threshold);
  plane.is_horizontal = (abs_nz > params_.horizontal_normal_threshold);

  // Compute dimensions and centroid
  computePlaneDimensions(cluster_cloud, inliers->indices, plane);

  planes.push_back(plane);

  return planes;
}

void StringerDetector::computePlaneDimensions(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const std::vector<int>& indices,
  DetectedPlane& plane)
{
  if (indices.empty()) {
    return;
  }

  // Compute centroid
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  for (const auto& idx : indices) {
    const auto& pt = cloud->points[idx];
    sum_x += pt.x;
    sum_y += pt.y;
    sum_z += pt.z;
  }

  plane.center_x = sum_x / indices.size();
  plane.center_y = sum_y / indices.size();
  plane.center_z = sum_z / indices.size();

  // Compute bounding box for dimensions
  double min_x = 1e6, max_x = -1e6;
  double min_y = 1e6, max_y = -1e6;
  double min_z = 1e6, max_z = -1e6;

  for (const auto& idx : indices) {
    const auto& pt = cloud->points[idx];
    min_x = std::min(min_x, static_cast<double>(pt.x));
    max_x = std::max(max_x, static_cast<double>(pt.x));
    min_y = std::min(min_y, static_cast<double>(pt.y));
    max_y = std::max(max_y, static_cast<double>(pt.y));
    min_z = std::min(min_z, static_cast<double>(pt.z));
    max_z = std::max(max_z, static_cast<double>(pt.z));
  }

  // Robot frame: X=forward, Y=left, Z=up
  // Width = horizontal extent, Height = vertical extent
  double extent_x = max_x - min_x;
  double extent_y = max_y - min_y;
  double extent_z = max_z - min_z;  // vertical

  // For stringer detection:
  // - Height = Z extent (vertical)
  // - Width = larger of X or Y extent (horizontal)
  plane.height = extent_z;
  plane.width = std::max(extent_x, extent_y);
}

bool StringerDetector::matchesStringerPlaneCriteria(const DetectedPlane& plane)
{
  // Check vertical dimension (height)
  bool height_ok = (plane.height >= params_.height_min &&
                    plane.height <= params_.height_max);

  // Check horizontal dimension (width)
  bool width_ok = (plane.width >= params_.width_min &&
                   plane.width <= params_.width_max);

  if (height_ok && width_ok) {
    return true;
  }

  std::cout << "[StringerDetector] ✗ Rejected plane: "
            << "Height=" << plane.height << "m (" << params_.height_min << "-" << params_.height_max << "), "
            << "Width=" << plane.width << "m (" << params_.width_min << "-" << params_.width_max << "), "
            << "Vertical=" << plane.is_vertical << ", "
            << "Horizontal=" << plane.is_horizontal << std::endl;

  return false;
}

std::vector<DetectedColumn> StringerDetector::detectColumnsInCluster(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const pcl::PointIndices& indices)
{
  std::vector<DetectedColumn> columns;

  if (indices.indices.empty()) {
    return columns;
  }

  // Compute column properties (checks both horizontal and vertical)
  DetectedColumn column = computeColumnLength(cloud, indices.indices);

  std::cout << "[StringerDetector] Cluster analysis: "
            << "H-Length=" << column.length << "m, "
            << "V-Height=" << column.height << "m" << std::endl;

  // Check if horizontal component matches
  if (column.is_horizontal) {
    std::cout << "[StringerDetector] ✓ Has valid HORIZONTAL component" << std::endl;
  }

  // Check if vertical component matches
  if (column.is_vertical) {
    std::cout << "[StringerDetector] ✓ Has valid VERTICAL component" << std::endl;
  }

  // Add horizontal line if valid
  if (column.is_horizontal) {
    DetectedColumn h_column = column;
    h_column.is_vertical = false;  // Mark as horizontal-only
    columns.push_back(h_column);
  }

  // Add vertical line if valid
  if (column.is_vertical) {
    DetectedColumn v_column = column;
    v_column.is_horizontal = false;  // Mark as vertical-only
    // For vertical, update start/end to be Z-aligned
    v_column.start_x = column.center_x;
    v_column.start_y = column.center_y;
    v_column.start_z = column.center_z - column.height / 2.0;
    v_column.end_x = column.center_x;
    v_column.end_y = column.center_y;
    v_column.end_z = column.center_z + column.height / 2.0;
    v_column.length = column.height;  // For vertical, length = height
    columns.push_back(v_column);
  }

  return columns;
}

DetectedColumn StringerDetector::computeColumnLength(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const std::vector<int>& indices)
{
  DetectedColumn column;
  column.num_points = indices.size();

  if (indices.empty()) {
    column.length = 0.0;
    return column;
  }

  // Extract cluster points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto& idx : indices) {
    cluster_cloud->points.push_back(cloud->points[idx]);
  }
  cluster_cloud->width = cluster_cloud->points.size();
  cluster_cloud->height = 1;
  cluster_cloud->is_dense = false;

  // Compute centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster_cloud, centroid);

  // Perform PCA to find principal axis in HORIZONTAL plane (X-Y only)
  // Create 2D points (X, Y) for horizontal PCA
  Eigen::MatrixXf points_2d(2, cluster_cloud->points.size());
  for (size_t i = 0; i < cluster_cloud->points.size(); ++i) {
    points_2d(0, i) = cluster_cloud->points[i].x;
    points_2d(1, i) = cluster_cloud->points[i].y;
  }

  // Center the points
  Eigen::Vector2f centroid_2d(centroid[0], centroid[1]);
  for (size_t i = 0; i < cluster_cloud->points.size(); ++i) {
    points_2d(0, i) -= centroid_2d[0];
    points_2d(1, i) -= centroid_2d[1];
  }

  // Compute covariance matrix (2x2 for X-Y plane)
  Eigen::Matrix2f cov = (points_2d * points_2d.transpose()) / float(cluster_cloud->points.size());

  // Compute eigenvectors (principal directions)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(cov);
  Eigen::Vector2f principal_axis = eigen_solver.eigenvectors().col(1);  // Largest eigenvalue

  // Snap principal axis to nearest cardinal direction (X or Y axis)
  // Robot frame: X=forward, Y=left
  double abs_x = std::abs(principal_axis[0]);
  double abs_y = std::abs(principal_axis[1]);

  Eigen::Vector2f aligned_axis;
  if (abs_x > abs_y) {
    // Align to X axis (forward/backward)
    aligned_axis = Eigen::Vector2f(principal_axis[0] > 0 ? 1.0f : -1.0f, 0.0f);
  } else {
    // Align to Y axis (left/right)
    aligned_axis = Eigen::Vector2f(0.0f, principal_axis[1] > 0 ? 1.0f : -1.0f);
  }

  // Project all points onto the aligned axis and find extremes
  double min_proj = 1e6;
  double max_proj = -1e6;
  int idx_min = 0;
  int idx_max = 0;

  for (size_t i = 0; i < cluster_cloud->points.size(); ++i) {
    const auto& pt = cluster_cloud->points[i];
    Eigen::Vector2f point_2d(pt.x - centroid_2d[0], pt.y - centroid_2d[1]);

    // Project onto aligned axis
    double projection = point_2d.dot(aligned_axis);

    if (projection < min_proj) {
      min_proj = projection;
      idx_min = i;
    }
    if (projection > max_proj) {
      max_proj = projection;
      idx_max = i;
    }
  }

  // Store start and end points (extremes along aligned axis)
  const auto& start_pt = cluster_cloud->points[idx_min];
  const auto& end_pt = cluster_cloud->points[idx_max];

  // Compute height (Z direction) - vertical extent
  double min_z = 1e6;
  double max_z = -1e6;
  for (const auto& pt : cluster_cloud->points) {
    min_z = std::min(min_z, static_cast<double>(pt.z));
    max_z = std::max(max_z, static_cast<double>(pt.z));
  }
  column.height = max_z - min_z;

  // Determine if this is horizontal or vertical line
  // Horizontal: X or Y axis aligned (length in X-Y plane)
  // Vertical: Z axis aligned (length in Z direction)

  // Compare horizontal extent vs vertical extent
  if (abs_x > abs_y) {
    // X-direction dominates
    column.length = std::abs(end_pt.x - start_pt.x);
  } else {
    // Y-direction dominates
    column.length = std::abs(end_pt.y - start_pt.y);
  }

  // Check both horizontal and vertical components
  // A cluster can have BOTH horizontal extent AND vertical extent
  // We'll use the dominant direction for visualization, but mark both

  // Default: use horizontal alignment for start/end points
  if (abs_x > abs_y) {
    // X-aligned
    column.start_x = start_pt.x;
    column.start_y = centroid[1];
    column.start_z = centroid[2];
    column.end_x = end_pt.x;
    column.end_y = centroid[1];
    column.end_z = centroid[2];
  } else {
    // Y-aligned
    column.start_x = centroid[0];
    column.start_y = start_pt.y;
    column.start_z = centroid[2];
    column.end_x = centroid[0];
    column.end_y = end_pt.y;
    column.end_z = centroid[2];
  }

  // Mark if this cluster has significant horizontal extent
  column.is_horizontal = (column.length >= params_.width_min &&
                          column.length <= params_.width_max);

  // Mark if this cluster has significant vertical extent
  column.is_vertical = (column.height >= params_.height_min &&
                        column.height <= params_.height_max);

  // Compute center (centroid of all points)
  column.center_x = centroid[0];
  column.center_y = centroid[1];
  column.center_z = centroid[2];

  return column;
}

bool StringerDetector::matchesStringerColumnCriteria(const DetectedColumn& column)
{
  // This is the old combined function - deprecated
  // Use matchesHorizontalCriteria or matchesVerticalCriteria instead
  if (column.is_horizontal) {
    return matchesHorizontalCriteria(column);
  } else if (column.is_vertical) {
    return matchesVerticalCriteria(column);
  }
  return false;
}

bool StringerDetector::matchesHorizontalCriteria(const DetectedColumn& column)
{
  // For horizontal lines: check width (length in X-Y plane)
  bool length_ok = (column.length >= params_.width_min &&
                    column.length <= params_.width_max);

  if (length_ok) {
    std::cout << "[StringerDetector] ✓ Horizontal line ACCEPTED: "
              << "Length=" << column.length << "m ("
              << params_.width_min << "-" << params_.width_max << ")"
              << std::endl;
  } else {
    std::cout << "[StringerDetector] ✗ Horizontal line REJECTED: "
              << "Length=" << column.length << "m ("
              << params_.width_min << "-" << params_.width_max << ")"
              << std::endl;
  }

  return length_ok;
}

bool StringerDetector::matchesVerticalCriteria(const DetectedColumn& column)
{
  // For vertical lines: check height (length in Z direction)
  bool height_ok = (column.length >= params_.height_min &&
                    column.length <= params_.height_max);

  if (height_ok) {
    std::cout << "[StringerDetector] ✓ Vertical line ACCEPTED: "
              << "Height=" << column.length << "m ("
              << params_.height_min << "-" << params_.height_max << ")"
              << std::endl;
  } else {
    std::cout << "[StringerDetector] ✗ Vertical line REJECTED: "
              << "Height=" << column.length << "m ("
              << params_.height_min << "-" << params_.height_max << ")"
              << std::endl;
  }

  return height_ok;
}

}  // namespace floor_removal_rgbd
