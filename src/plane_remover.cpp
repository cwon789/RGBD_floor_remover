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

  // Step 2: Find maximum Y (camera frame: Y=down, so max Y = floor)
  double max_y = findMaxY(cloud_for_processing);

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

}  // namespace floor_removal_rgbd
