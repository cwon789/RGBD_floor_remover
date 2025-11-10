#include "floor_removal_rgbd/plane_remover.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>

namespace floor_removal_rgbd
{

PlaneRemover::PlaneRemover(const PlaneRemoverParams& params)
  : params_(params)
{
  // Initialize stringer detector with corresponding parameters
  StringerDetectorParams stringer_params;
  stringer_params.width_min = params_.stringer_width_min;
  stringer_params.width_max = params_.stringer_width_max;
  stringer_params.height_min = params_.stringer_height_min;
  stringer_params.height_max = params_.stringer_height_max;
  stringer_detector_ = std::make_unique<StringerDetector>(stringer_params);
}

void PlaneRemover::setParams(const PlaneRemoverParams& params)
{
  params_ = params;

  // Update stringer detector parameters
  StringerDetectorParams stringer_params;
  stringer_params.width_min = params_.stringer_width_min;
  stringer_params.width_max = params_.stringer_width_max;
  stringer_params.height_min = params_.stringer_height_min;
  stringer_params.height_max = params_.stringer_height_max;
  stringer_detector_->setParams(stringer_params);
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

  // Step 1: Transform to robot frame (X=forward, Y=left, Z=up)
  auto cloud_robot = transformToStandardFrame(cloud_camera);
  std::cout << "[DEBUG] Transformed cloud size: " << cloud_robot->points.size() << std::endl;

  // Step 2: Filter by max detection distance (in robot frame, X=forward is depth)
  auto cloud_filtered = filterByDistance(cloud_robot);
  std::cout << "[DEBUG] Filtered cloud size (within " << params_.max_detection_distance
            << "m): " << cloud_filtered->points.size() << std::endl;

  // Step 3: Apply voxel grid downsampling if enabled (work in robot frame)
  auto cloud_voxelized = cloud_filtered;
  if (params_.use_voxel_grid) {
    cloud_voxelized = applyVoxelGrid(cloud_filtered);
    result.voxelized_points = cloud_voxelized->points.size();
    std::cout << "[DEBUG] Voxelized cloud size: " << result.voxelized_points << std::endl;
  } else {
    result.voxelized_points = cloud_filtered->points.size();
  }

  // Step 4: Extract floor region for RANSAC (based on Z height)
  double min_z;
  if (params_.auto_floor_detection_mode) {
    // Auto mode: Find minimum Z (robot frame: Z=up, so min Z = floor)
    min_z = findMinZ(cloud_voxelized);
    std::cout << "[DEBUG] Auto mode - min_z: " << min_z << std::endl;
  } else {
    // Fixed mode: Use configured floor height
    min_z = params_.floor_height;
    std::cout << "[DEBUG] Fixed mode - floor_height: " << min_z << std::endl;
  }

  auto floor_region = extractFloorRegion(cloud_voxelized, min_z);
  result.floor_region_points = floor_region->points.size();
  std::cout << "[DEBUG] Floor region points for RANSAC: " << result.floor_region_points << std::endl;

  if (floor_region->points.empty()) {
    std::cout << "[DEBUG] Floor region is empty - returning transformed cloud without floor removal" << std::endl;
    // Return transformed and voxelized clouds without floor separation
    result.no_floor_cloud = cloud_filtered;
    result.no_floor_cloud_voxelized = cloud_voxelized;
    return result;
  }

  // Step 5: Detect floor plane using RANSAC on floor region
  double nx, ny, nz, d, inlier_ratio;
  if (!detectFloorPlane(floor_region, nx, ny, nz, d, inlier_ratio)) {
    std::cout << "[DEBUG] Failed to detect floor plane - returning transformed cloud without floor removal" << std::endl;
    result.no_floor_cloud = cloud_filtered;
    result.no_floor_cloud_voxelized = cloud_voxelized;
    return result;
  }

  // Step 6: Validate plane (in robot frame, nz should be significant)
  if (!isPlaneValid(nz)) {
    std::cout << "[DEBUG] Plane validation failed (nz=" << nz << ") - returning transformed cloud without floor removal" << std::endl;
    result.no_floor_cloud = cloud_filtered;
    result.no_floor_cloud_voxelized = cloud_voxelized;
    return result;
  }

  result.plane_found = true;
  result.nx = nx;
  result.ny = ny;
  result.nz = nz;
  result.d = d;
  std::cout << "[DEBUG] Floor plane found: nx=" << nx << ", ny=" << ny
            << ", nz=" << nz << ", d=" << d << std::endl;

  // Step 6: Remove points below the plane (floor removal)
  // In robot frame Z=up, so points below plane have negative signed distance
  removePointsBelowPlane(cloud_filtered, nx, ny, nz, d, result.floor_cloud, result.no_floor_cloud);
  result.floor_points = result.floor_cloud->points.size();
  std::cout << "[DEBUG] Floor points: " << result.floor_points
            << ", No-floor points: " << result.no_floor_cloud->points.size() << std::endl;

  // Step 7: Classify voxelized cloud for visualization
  removePointsBelowPlane(cloud_voxelized, nx, ny, nz, d,
                         result.floor_cloud_voxelized, result.no_floor_cloud_voxelized);

  // Step 8: Detect stringers in the no-floor cloud (voxelized for efficiency)
  if (params_.enable_stringer_detection && stringer_detector_) {
    auto stringer_result = stringer_detector_->detect(result.no_floor_cloud_voxelized);
    result.detected_stringers = stringer_result.detected_stringers;
    result.stringer_centers = stringer_result.stringer_centers;
  }

  return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneRemover::transformToStandardFrame(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_camera)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_standard(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_standard->header = cloud_camera->header;
  cloud_standard->points.reserve(cloud_camera->points.size());

  // Step 1: Always apply default optical->base transform first
  // Camera optical: X=right, Y=down, Z=forward
  // Robot base: X=forward, Y=left, Z=up
  for (const auto& cam_point : cloud_camera->points) {
    if (!std::isfinite(cam_point.x) || !std::isfinite(cam_point.y) || !std::isfinite(cam_point.z)) {
      continue;
    }

    pcl::PointXYZRGB base_point;
    base_point.x = cam_point.z;   // forward = depth
    base_point.y = -cam_point.x;  // left = -right
    base_point.z = -cam_point.y;  // up = -down
    base_point.rgb = cam_point.rgb;

    // Step 2: Apply additional extrinsic transform if enabled
    if (!params_.use_default_transform) {
      // Build rotation matrix from Euler angles (ZYX order: Yaw * Pitch * Roll)
      double cr = std::cos(params_.cam_roll);
      double sr = std::sin(params_.cam_roll);
      double cp = std::cos(params_.cam_pitch);
      double sp = std::sin(params_.cam_pitch);
      double cy = std::cos(params_.cam_yaw);
      double sy = std::sin(params_.cam_yaw);

      // Rotation matrix (ZYX Euler)
      double r11 = cy * cp;
      double r12 = cy * sp * sr - sy * cr;
      double r13 = cy * sp * cr + sy * sr;
      double r21 = sy * cp;
      double r22 = sy * sp * sr + cy * cr;
      double r23 = sy * sp * cr - cy * sr;
      double r31 = -sp;
      double r32 = cp * sr;
      double r33 = cp * cr;

      // Apply rotation to base_point
      double x_rot = r11 * base_point.x + r12 * base_point.y + r13 * base_point.z;
      double y_rot = r21 * base_point.x + r22 * base_point.y + r23 * base_point.z;
      double z_rot = r31 * base_point.x + r32 * base_point.y + r33 * base_point.z;

      // Apply translation
      base_point.x = x_rot + params_.cam_tx;
      base_point.y = y_rot + params_.cam_ty;
      base_point.z = z_rot + params_.cam_tz;
    }

    cloud_standard->points.push_back(base_point);
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneRemover::filterByDistance(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // Filter points by distance from camera origin
  // In robot frame: X=forward (depth), so distance = sqrt(x^2 + y^2 + z^2)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_filtered->header = cloud->header;
  cloud_filtered->points.reserve(cloud->points.size());

  double max_dist_sq = params_.max_detection_distance * params_.max_detection_distance;

  for (const auto& point : cloud->points) {
    double dist_sq = point.x * point.x + point.y * point.y + point.z * point.z;
    if (dist_sq <= max_dist_sq) {
      cloud_filtered->points.push_back(point);
    }
  }

  cloud_filtered->width = cloud_filtered->points.size();
  cloud_filtered->height = 1;
  cloud_filtered->is_dense = false;

  return cloud_filtered;
}

double PlaneRemover::findMinZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // Find minimum Z (robot frame: Z=up, so min Z = lowest point = floor)
  double min_z = 1e6;
  for (const auto& point : cloud->points) {
    min_z = std::min(min_z, static_cast<double>(point.z));
  }

  return min_z;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneRemover::extractFloorRegion(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, double min_z)
{
  // Robot frame: Z=up, so floor is near min Z
  // Use DETECTION thickness (wider) for RANSAC to get enough points
  double floor_z_max = min_z + params_.floor_detection_thickness;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_region(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto& point : cloud->points) {
    if (point.z >= min_z && point.z <= floor_z_max) {
      floor_region->points.push_back(point);
    }
  }

  floor_region->width = floor_region->points.size();
  floor_region->height = 1;
  floor_region->is_dense = false;

  return floor_region;
}

bool PlaneRemover::detectFloorPlane(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  double& nx, double& ny, double& nz, double& d, double& inlier_ratio)
{
  // Need at least 10 points for RANSAC
  if (cloud->points.size() < 10) {
    return false;
  }

  // Run RANSAC to detect floor plane on entire voxelized cloud
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params_.ransac_max_iterations);
  seg.setDistanceThreshold(params_.ransac_distance_threshold);
  seg.setInputCloud(cloud);
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
  inlier_ratio = static_cast<double>(inliers->indices.size()) / cloud->points.size();

  return true;
}

bool PlaneRemover::isPlaneValid(double nz)
{
  // Normal should point upward in robot frame (Z component should be positive and significant)
  // Robot frame: Z=up, so floor normal should have positive Z (close to 1.0 for horizontal floor)
  // Check if normal is pointing upward (positive Z) and is significant
  if (nz < params_.floor_normal_z_threshold) {
    return false;
  }

  return true;
}

void PlaneRemover::removePointsBelowPlane(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  double nx, double ny, double nz, double d,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& no_floor_cloud)
{
  // Remove points below the plane (floor removal)
  // In robot frame (Z=up), floor normal points upward (nz > 0)
  // Signed distance = nx*x + ny*y + nz*z + d
  // Negative distance = below plane, Positive distance = above plane

  // Use REMOVAL thickness to define floor region
  double threshold = params_.floor_removal_thickness / 2.0 + params_.floor_margin;

  for (const auto& point : cloud->points) {
    // Calculate signed distance from point to plane
    double signed_distance = nx * point.x + ny * point.y + nz * point.z + d;

    // Points AT or BELOW the plane are considered floor
    // Since normal points up, negative distance means below plane
    if (signed_distance <= threshold) {
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

}  // namespace floor_removal_rgbd
