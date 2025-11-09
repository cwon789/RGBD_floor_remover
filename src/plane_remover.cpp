#include "floor_removal_rgbd/plane_remover.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <algorithm>
#include <cmath>

namespace floor_removal_rgbd
{

PlaneRemover::PlaneRemover(const PlaneRemoverParams& params)
  : params_(params)
{
}

void PlaneRemover::setParams(const PlaneRemoverParams& params)
{
  params_ = params;
}

void PlaneRemover::reset()
{
  // No temporal state to reset
}

PlaneRemovalResult PlaneRemover::process(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_camera)
{
  PlaneRemovalResult result;

  if (!cloud_camera || cloud_camera->points.empty()) {
    return result;
  }

  result.total_points = cloud_camera->points.size();

  // Step 1: Apply voxel grid downsampling if enabled (work in camera frame directly)
  auto cloud_for_processing = cloud_camera;
  if (params_.use_voxel_grid) {
    cloud_for_processing = applyVoxelGrid(cloud_camera);
    result.voxelized_points = cloud_for_processing->points.size();
  } else {
    result.voxelized_points = cloud_camera->points.size();
  }

  // Step 2: Determine floor height based on mode
  double max_y;
  if (params_.auto_floor_detection_mode) {
    // Auto mode: Find maximum Y (camera frame: Y=down, so max Y = floor)
    max_y = findMaxY(cloud_for_processing);
  } else {
    // Fixed mode: Use configured camera height
    max_y = params_.camera_height;
  }

  // Step 3: Extract floor region (points near maximum Y in camera frame)
  auto floor_region = extractFloorRegionCameraFrame(cloud_for_processing, max_y);
  result.floor_region_points = floor_region->points.size();

  if (floor_region->points.empty()) {
    return result;
  }

  // Step 5: Detect floor plane using RANSAC
  double nx, ny, nz, d, inlier_ratio;
  if (!detectFloorPlane(floor_region, nx, ny, nz, d, inlier_ratio)) {
    return result;
  }

  // Step 6: Validate plane
  if (!isPlaneValid(ny)) {
    return result;
  }

  result.plane_found = true;
  result.nx = nx;
  result.ny = ny;
  result.nz = nz;
  result.d = d;

  // Step 4: Classify all points in camera frame cloud (original)
  classifyPoints(cloud_camera, nx, ny, nz, d, result.floor_cloud, result.no_floor_cloud);
  result.floor_points = result.floor_cloud->points.size();

  // Step 5: Classify voxelized cloud
  classifyPoints(cloud_for_processing, nx, ny, nz, d, result.floor_cloud_voxelized, result.no_floor_cloud_voxelized);

  // Step 6: Detect stringers in the no-floor cloud (voxelized for efficiency)
  if (params_.enable_stringer_detection) {
    result.detected_stringers = detectStringers(result.no_floor_cloud_voxelized);

    // Compute center points for each detected stringer (using centroid)
    for (const auto& bbox : result.detected_stringers) {
      pcl::PointXYZRGB center;
      center.x = bbox.centroid_x;
      center.y = bbox.centroid_y;
      center.z = bbox.centroid_z;
      center.r = 0;
      center.g = 255;
      center.b = 0;
      result.stringer_centers->points.push_back(center);
    }
    result.stringer_centers->width = result.stringer_centers->points.size();
    result.stringer_centers->height = 1;
    result.stringer_centers->is_dense = false;
  }

  return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneRemover::transformToStandardFrame(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_camera)
{
  // Transform from camera optical frame to standard robot frame
  // Camera: X=right, Y=down, Z=forward
  // Standard: X=forward, Y=left, Z=up
  //
  // Transformation:
  //   std.x = cam.z (forward = depth)
  //   std.y = -cam.x (left = -right)
  //   std.z = -cam.y (up = -down)

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_standard(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_standard->header = cloud_camera->header;
  cloud_standard->points.reserve(cloud_camera->points.size());

  for (const auto& cam_point : cloud_camera->points) {
    // Skip invalid points
    if (!std::isfinite(cam_point.x) || !std::isfinite(cam_point.y) || !std::isfinite(cam_point.z)) {
      continue;
    }

    pcl::PointXYZRGB std_point;
    std_point.x = cam_point.z;   // forward (depth becomes forward)
    std_point.y = -cam_point.x;  // left (right becomes left, negated)
    std_point.z = -cam_point.y;  // up (down becomes up, negated)
    std_point.rgb = cam_point.rgb;
    cloud_standard->points.push_back(std_point);
  }

  cloud_standard->width = cloud_standard->points.size();
  cloud_standard->height = 1;
  cloud_standard->is_dense = false;

  return cloud_standard;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneRemover::applyVoxelGrid(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

  try {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size, params_.voxel_leaf_size);
    voxel_filter.filter(*cloud_downsampled);

    // If filter succeeded but result is empty or too small, return original
    if (cloud_downsampled->points.empty()) {
      return cloud;
    }
  } catch (const std::exception& e) {
    // If voxel grid fails (e.g., leaf size too small), return original cloud
    return cloud;
  }

  return cloud_downsampled;
}

double PlaneRemover::findMaxY(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // Find maximum Y (camera frame: Y=down, so max Y = lowest point = floor)
  double max_y = -1e6;
  for (const auto& point : cloud->points) {
    max_y = std::max(max_y, static_cast<double>(point.y));
  }

  return max_y;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneRemover::extractFloorRegionCameraFrame(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, double max_y)
{
  // Camera frame: Y=down, so floor is near max Y
  // Use DETECTION thickness (wider) for RANSAC to get enough points
  double floor_y_min = max_y - params_.floor_detection_thickness;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_region(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto& point : cloud->points) {
    if (point.y >= floor_y_min && point.y <= max_y) {
      floor_region->points.push_back(point);
    }
  }

  floor_region->width = floor_region->points.size();
  floor_region->height = 1;
  floor_region->is_dense = false;

  return floor_region;
}

bool PlaneRemover::detectFloorPlane(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_region,
  double& nx, double& ny, double& nz, double& d, double& inlier_ratio)
{
  // Need at least 3 points for RANSAC
  if (floor_region->points.size() < 10) {
    return false;
  }

  // Run RANSAC to detect floor plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params_.ransac_max_iterations);
  seg.setDistanceThreshold(params_.ransac_distance_threshold);
  seg.setInputCloud(floor_region);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty() || coefficients->values.size() != 4) {
    return false;
  }

  // Extract plane coefficients: ax + by + cz + d = 0
  nx = coefficients->values[0];
  ny = coefficients->values[1];
  nz = coefficients->values[2];
  d = coefficients->values[3];

  // Calculate inlier ratio
  inlier_ratio = static_cast<double>(inliers->indices.size()) / floor_region->points.size();

  return true;
}

bool PlaneRemover::isPlaneValid(double ny)
{
  // Normal should point downward in camera frame (Y component should be positive and significant)
  // Camera frame: Y=down, so floor normal should have positive Y
  if (std::abs(ny) < params_.floor_normal_z_threshold) {
    return false;
  }

  return true;
}

void PlaneRemover::classifyPoints(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  double nx, double ny, double nz, double d,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& no_floor_cloud)
{
  // Use REMOVAL thickness (thin) to preserve objects on floor like pallets
  double threshold = params_.floor_removal_thickness / 2.0 + params_.floor_margin;

  for (const auto& point : cloud->points) {
    // Calculate signed distance from point to plane: nx*x + ny*y + nz*z + d
    double signed_distance = nx * point.x + ny * point.y + nz * point.z + d;

    // Only mark points ON or BELOW the floor (within thin layer)
    if (signed_distance >= -threshold && signed_distance <= threshold) {
      floor_cloud->points.push_back(point);
    } else {
      no_floor_cloud->points.push_back(point);
    }
  }

  floor_cloud->width = floor_cloud->points.size();
  floor_cloud->height = 1;
  floor_cloud->is_dense = false;

  no_floor_cloud->width = no_floor_cloud->points.size();
  no_floor_cloud->height = 1;
  no_floor_cloud->is_dense = false;
}

std::vector<BoundingBox> PlaneRemover::detectStringers(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  std::vector<BoundingBox> stringers;

  if (!params_.enable_stringer_detection) {
    return stringers;
  }

  if (cloud->points.empty()) {
    std::cout << "[DEBUG] No-floor cloud is empty, cannot detect stringers" << std::endl;
    return stringers;
  }

  std::cout << "[DEBUG] Detecting stringers in cloud with " << cloud->points.size() << " points" << std::endl;

  // Additional voxel downsampling if cloud is too large
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_for_clustering = cloud;
  if (cloud->points.size() > 5000) {
    std::cout << "[DEBUG] Cloud too large (" << cloud->points.size()
              << " points), applying additional downsampling for clustering" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);  // 5cm voxels for clustering
    voxel_filter.filter(*cloud_downsampled);

    cloud_for_clustering = cloud_downsampled;
    std::cout << "[DEBUG] Downsampled to " << cloud_for_clustering->points.size() << " points" << std::endl;
  }

  // Use Euclidean clustering to segment the cloud into clusters
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_for_clustering);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.08);  // 8cm tolerance (increased for downsampled cloud)
  ec.setMinClusterSize(5);       // minimum 5 points (reduced for heavily downsampled)
  ec.setMaxClusterSize(10000);   // reduced max size
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_for_clustering);
  ec.extract(cluster_indices);

  std::cout << "[DEBUG] Found " << cluster_indices.size() << " clusters" << std::endl;

  // For each cluster, compute bounding box and check if it matches stringer dimensions
  for (const auto& indices : cluster_indices) {
    if (indices.indices.empty()) {
      continue;
    }

    // Compute bounding box and centroid
    BoundingBox bbox;
    bbox.min_x = bbox.min_y = bbox.min_z = 1e6;
    bbox.max_x = bbox.max_y = bbox.max_z = -1e6;

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    size_t num_points = indices.indices.size();

    for (const auto& idx : indices.indices) {
      const auto& point = cloud_for_clustering->points[idx];

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

    // Check if dimensions match stringer criteria
    // Camera frame: X=right, Y=down, Z=forward
    double width_x = bbox.width();   // horizontal (left-right)
    double height_y = bbox.height(); // vertical (floor to top)
    double depth_z = bbox.depth();   // horizontal (front-back)

    std::cout << "[DEBUG] Cluster: "
              << "X=" << width_x << "m, "
              << "Y=" << height_y << "m, "
              << "Z=" << depth_z << "m" << std::endl;

    // Stringer check: height must be in range,
    // and at least one horizontal dimension (X or Z) must be in width range
    bool height_ok = (height_y >= params_.stringer_height_min &&
                      height_y <= params_.stringer_height_max);

    bool width_x_ok = (width_x >= params_.stringer_width_min &&
                       width_x <= params_.stringer_width_max);

    bool width_z_ok = (depth_z >= params_.stringer_width_min &&
                       depth_z <= params_.stringer_width_max);

    if (height_ok && (width_x_ok || width_z_ok)) {
      std::cout << "[DEBUG] ✓ Cluster matches stringer criteria! "
                << "(height=" << height_y << "m, "
                << "X=" << width_x << "m, "
                << "Z=" << depth_z << "m)" << std::endl;
      stringers.push_back(bbox);
    } else {
      std::cout << "[DEBUG] ✗ Rejected: "
                << "height_ok=" << height_ok
                << ", width_x_ok=" << width_x_ok
                << ", width_z_ok=" << width_z_ok
                << " (height range: " << params_.stringer_height_min << "-"
                << params_.stringer_height_max
                << "m, width range: " << params_.stringer_width_min << "-"
                << params_.stringer_width_max << "m)" << std::endl;
    }
  }

  std::cout << "[DEBUG] Total stringers detected: " << stringers.size() << std::endl;

  return stringers;
}

}  // namespace floor_removal_rgbd
