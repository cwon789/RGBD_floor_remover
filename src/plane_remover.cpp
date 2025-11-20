#include "floor_removal_rgbd/plane_remover.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <iomanip>

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
  // Reset temporal smoothing state
  prev_nx_ = 0.0;
  prev_ny_ = 0.0;
  prev_nz_ = 0.0;
  prev_d_ = 0.0;
  has_prev_plane_ = false;
  prev_plane_found_ = false;
}

PlaneRemovalResult PlaneRemover::process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera)
{
  PlaneRemovalResult result;

  if (!cloud_camera || cloud_camera->points.empty()) {
    return result;
  }

  result.total_points = cloud_camera->points.size();

  // Step 1: Filter by max detection distance (camera frame: Z=forward/depth)
  auto cloud_filtered = filterByDistance(cloud_camera);

  // Step 2: Extract floor detection region based on depth
  auto floor_region = extractFloorDetectionRegion(cloud_filtered);
  result.floor_region_points = floor_region->points.size();

  if (floor_region->points.size() < params_.min_points_for_plane) {
    // State change: plane lost
    if (prev_plane_found_) {
      std::cout << "[WARNING] Floor plane LOST - insufficient points in detection region ("
                << floor_region->points.size() << " < " << params_.min_points_for_plane << ")" << std::endl;
      prev_plane_found_ = false;
    }
    result.no_floor_cloud = cloud_filtered;
    result.no_floor_cloud_voxelized = cloud_filtered;
    return result;
  }

  // Step 3: Apply voxel grid downsampling to detection region if enabled
  auto floor_region_voxelized = floor_region;
  if (params_.use_voxel_grid) {
    floor_region_voxelized = applyVoxelGrid(floor_region);
    result.voxelized_points = floor_region_voxelized->points.size();
  } else {
    result.voxelized_points = floor_region->points.size();
  }

  // Step 4: Detect floor plane using RANSAC on detection region
  double nx, ny, nz, d, inlier_ratio;
  if (!detectFloorPlane(floor_region_voxelized, nx, ny, nz, d, inlier_ratio)) {
    // State change: plane lost
    if (prev_plane_found_) {
      std::cout << "[WARNING] Floor plane LOST - RANSAC detection failed" << std::endl;
      prev_plane_found_ = false;
    }
    has_prev_plane_ = false;  // Reset smoothing state
    result.no_floor_cloud = cloud_filtered;
    result.no_floor_cloud_voxelized = cloud_filtered;
    return result;
  }

  // Step 5: Validate plane (camera frame: ny should be negative and significant)
  if (!isPlaneValid(ny)) {
    // State change: plane lost
    if (prev_plane_found_) {
      std::cout << "[WARNING] Floor plane LOST - validation failed (ny=" << ny << ")" << std::endl;
      prev_plane_found_ = false;
    }
    has_prev_plane_ = false;  // Reset smoothing state
    result.no_floor_cloud = cloud_filtered;
    result.no_floor_cloud_voxelized = cloud_filtered;
    return result;
  }

  // Step 5.5: Apply temporal smoothing to plane coefficients (reduce jitter)
  if (params_.enable_plane_smoothing && has_prev_plane_) {
    double alpha = params_.plane_smoothing_alpha;
    nx = alpha * nx + (1.0 - alpha) * prev_nx_;
    ny = alpha * ny + (1.0 - alpha) * prev_ny_;
    nz = alpha * nz + (1.0 - alpha) * prev_nz_;
    d = alpha * d + (1.0 - alpha) * prev_d_;

    // Re-normalize the plane normal after smoothing
    double norm = std::sqrt(nx*nx + ny*ny + nz*nz);
    nx /= norm;
    ny /= norm;
    nz /= norm;
    d /= norm;
  }

  // Update previous plane for next frame
  prev_nx_ = nx;
  prev_ny_ = ny;
  prev_nz_ = nz;
  prev_d_ = d;
  has_prev_plane_ = true;

  result.plane_found = true;
  result.nx = nx;
  result.ny = ny;
  result.nz = nz;
  result.d = d;

  // State change: plane newly found
  if (!prev_plane_found_) {
    std::cout << "[INFO] Floor plane FOUND: normal=[" << nx << ", " << ny << ", " << nz
              << "], d=" << d << ", inlier_ratio=" << inlier_ratio << std::endl;
    prev_plane_found_ = true;
  }

  // Step 6: Remove points on the plane (floor removal)
  removePointsOnPlane(cloud_filtered, nx, ny, nz, d, result.floor_cloud, result.no_floor_cloud);
  result.floor_points = result.floor_cloud->points.size();

  // Step 7: Apply voxel grid to full cloud for visualization
  auto cloud_voxelized = cloud_filtered;
  if (params_.use_voxel_grid) {
    cloud_voxelized = applyVoxelGrid(cloud_filtered);
  }

  // Step 8: Classify voxelized cloud for visualization
  removePointsOnPlane(cloud_voxelized, nx, ny, nz, d,
                      result.floor_cloud_voxelized, result.no_floor_cloud_voxelized);

  // Step 9: Remove noise from voxelized no-floor cloud
  result.no_floor_cloud_voxelized = removeFloorNoise(result.no_floor_cloud_voxelized, nx, ny, nz, d, result.noise_cloud);
  result.noise_points = result.noise_cloud->points.size();

  // Step 10: Project no_floor_cloud_voxelized to 2D (detected floor plane)
  result.no_floor_cloud_voxelized_2d_projected = projectTo2D(result.no_floor_cloud_voxelized, nx, ny, nz, d);

  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneRemover::filterByDistance(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  // Filter points by distance from camera origin
  // Camera optical frame: X=right, Y=down, Z=forward/depth
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered->header = cloud->header;
  cloud_filtered->points.reserve(cloud->points.size());

  double max_dist_sq = params_.max_detection_distance * params_.max_detection_distance;

  for (const auto& point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

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

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneRemover::extractFloorDetectionRegion(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  // Extract floor detection region based on depth (Z axis in camera frame)
  // Camera optical frame: Z=forward/depth
  // ToF cameras typically capture floor in near depth range
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_region(new pcl::PointCloud<pcl::PointXYZ>);
  floor_region->header = cloud->header;
  floor_region->points.reserve(cloud->points.size());

  for (const auto& point : cloud->points) {
    // Filter by depth range
    if (point.z >= params_.floor_detection_min_depth &&
        point.z <= params_.floor_detection_max_depth) {
      floor_region->points.push_back(point);
    }
  }

  floor_region->width = floor_region->points.size();
  floor_region->height = 1;
  floor_region->is_dense = false;

  return floor_region;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneRemover::applyVoxelGrid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);

  try {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
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

bool PlaneRemover::detectFloorPlane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  double& nx, double& ny, double& nz, double& d, double& inlier_ratio)
{
  // Need at least 10 points for RANSAC
  if (cloud->points.size() < 10) {
    return false;
  }

  // Run RANSAC to detect floor plane on entire voxelized cloud
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
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

bool PlaneRemover::isPlaneValid(double ny)
{
  // Normal should point upward in camera optical frame
  // Camera frame: Y=down, so floor normal should have negative Y (pointing up)
  // Check if abs(Y) component is significant (close to -1.0 for horizontal floor)
  if (std::abs(ny) < params_.floor_normal_y_threshold) {
    return false;
  }

  // Ensure normal points upward (negative Y in camera frame)
  if (ny > 0) {
    return false;
  }

  return true;
}

void PlaneRemover::removePointsOnPlane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  double nx, double ny, double nz, double d,
  pcl::PointCloud<pcl::PointXYZ>::Ptr& floor_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr& no_floor_cloud)
{
  // Remove points on or near the detected plane (floor removal)
  // Camera optical frame: Y=down, floor normal points up (-Y direction, ny < 0)
  // Plane equation: nx*x + ny*y + nz*z + d = 0
  // Signed distance = nx*x + ny*y + nz*z + d
  //
  // Since ny < 0 (normal points up):
  //   - signed_distance > 0: above floor (keep if > threshold)
  //   - signed_distance <= 0: at or below floor (remove ALL - ToF ghost removal)

  double threshold = params_.floor_removal_distance_threshold + params_.floor_margin;

  for (const auto& point : cloud->points) {
    // Calculate signed distance from point to plane
    double signed_distance = nx * point.x + ny * point.y + nz * point.z + d;

    // ToF ghost handling: remove ALL points at or below floor
    // Only apply threshold to points ABOVE floor
    if (signed_distance <= 0.0) {
      // Below or on floor plane -> remove (ToF ghost)
      floor_cloud->points.push_back(point);
    } else if (signed_distance <= threshold) {
      // Above floor but within threshold -> floor
      floor_cloud->points.push_back(point);
    } else {
      // Above floor and beyond threshold -> keep
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneRemover::projectTo2D(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  double nx, double ny, double nz, double d)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_2d->header = cloud->header;
  cloud_2d->points.reserve(cloud->points.size());

  // Normalize plane normal vector
  double norm = std::sqrt(nx*nx + ny*ny + nz*nz);
  if (norm < 1e-6) {
    // If normal vector is zero, return original cloud
    *cloud_2d = *cloud;
    return cloud_2d;
  }
  
  double unit_nx = nx / norm;
  double unit_ny = ny / norm;
  double unit_nz = nz / norm;

  for (const auto& point : cloud->points) {
    // Calculate signed distance from point to plane
    double signed_distance = unit_nx * point.x + unit_ny * point.y + unit_nz * point.z + d / norm;
    
    // Project point onto plane by moving along normal direction
    pcl::PointXYZ point_2d = point;
    point_2d.x -= signed_distance * unit_nx;
    point_2d.y -= signed_distance * unit_ny;
    point_2d.z -= signed_distance * unit_nz;
    
    cloud_2d->points.push_back(point_2d);
  }

  cloud_2d->width = cloud_2d->points.size();
  cloud_2d->height = 1;
  cloud_2d->is_dense = false;

  return cloud_2d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneRemover::removeFloorNoise(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  double nx, double ny, double nz, double d,
  pcl::PointCloud<pcl::PointXYZ>::Ptr& noise_cloud)
{
  if (!params_.enable_noise_removal || cloud->points.empty()) {
    return cloud;
  }

  // Fast voxel-based noise removal (much faster than KD-tree)
  // Use a simple 3D grid hash map to count neighbors
  const double voxel_size = params_.noise_radius_search;
  const double plane_margin = params_.noise_plane_distance_margin;

  // Create voxel grid hash map
  std::unordered_map<size_t, int> voxel_count;
  auto voxel_hash = [&](double x, double y, double z) -> size_t {
    int ix = static_cast<int>(std::floor(x / voxel_size));
    int iy = static_cast<int>(std::floor(y / voxel_size));
    int iz = static_cast<int>(std::floor(z / voxel_size));
    // Simple hash combining
    return ((size_t)ix * 73856093) ^ ((size_t)iy * 19349663) ^ ((size_t)iz * 83492791);
  };

  // First pass: count points in each voxel (only for points near plane)
  for (const auto& point : cloud->points) {
    double dist_to_plane = std::abs(nx * point.x + ny * point.y + nz * point.z + d);
    if (dist_to_plane < plane_margin) {
      size_t hash = voxel_hash(point.x, point.y, point.z);
      voxel_count[hash]++;
    }
  }

  // Second pass: filter based on neighborhood occupancy (3x3x3 voxels)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered->header = cloud->header;
  cloud_filtered->points.reserve(cloud->points.size());
  noise_cloud->header = cloud->header;
  noise_cloud->points.reserve(cloud->points.size());

  size_t noise_removed = 0;

  auto voxel_hash_ijk = [](int ix, int iy, int iz) -> size_t {
    return ((size_t)ix * 73856093) ^ ((size_t)iy * 19349663) ^ ((size_t)iz * 83492791);
  };

  for (const auto& point : cloud->points) {
    // Only filter points near detected plane
    double dist_to_plane = std::abs(nx * point.x + ny * point.y + nz * point.z + d);
    if (dist_to_plane >= plane_margin) {
      cloud_filtered->points.push_back(point);
      continue;
    }

    // Get voxel indices for current point
    int ix = static_cast<int>(std::floor(point.x / voxel_size));
    int iy = static_cast<int>(std::floor(point.y / voxel_size));
    int iz = static_cast<int>(std::floor(point.z / voxel_size));

    // Check 3x3x3 neighborhood (27 voxels including current)
    int neighbor_count = 0;
    int occupied_voxels = 0;  // Count how many neighboring voxels have points

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        for (int dz = -1; dz <= 1; dz++) {
          size_t neighbor_hash = voxel_hash_ijk(ix + dx, iy + dy, iz + dz);
          auto it = voxel_count.find(neighbor_hash);
          if (it != voxel_count.end()) {
            neighbor_count += it->second;
            occupied_voxels++;
          }
        }
      }
    }

    // More strict filtering: require both sufficient points AND occupied voxels
    // This prevents single isolated points from passing through
    bool has_enough_neighbors = (neighbor_count >= params_.noise_min_neighbors) &&
                                 (occupied_voxels >= 3);  // At least 3 voxels occupied

    if (has_enough_neighbors) {
      cloud_filtered->points.push_back(point);
    } else {
      noise_cloud->points.push_back(point);
      noise_removed++;
    }
  }

  cloud_filtered->width = cloud_filtered->points.size();
  cloud_filtered->height = 1;
  cloud_filtered->is_dense = false;

  noise_cloud->width = noise_cloud->points.size();
  noise_cloud->height = 1;
  noise_cloud->is_dense = false;

  // Only log if significant noise was removed (> 10% of near-plane points)
  size_t near_plane_points = cloud->points.size() - cloud_filtered->points.size() + noise_removed;
  if (noise_removed > 0 && near_plane_points > 0) {
    double noise_ratio = 100.0 * noise_removed / near_plane_points;
    if (noise_ratio > 10.0) {
      std::cout << "[INFO] Removed " << noise_removed << " isolated floor noise points ("
                << std::fixed << std::setprecision(1) << noise_ratio << "%)" << std::endl;
    }
  }

  return cloud_filtered;
}

}  // namespace floor_removal_rgbd
