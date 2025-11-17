#include "floor_removal_rgbd/pallet_detection.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <iostream>

namespace floor_removal_rgbd
{

PalletDetection::PalletDetection(const PalletDetectionParams& params)
  : params_(params)
{
}

void PalletDetection::setParams(const PalletDetectionParams& params)
{
  params_ = params;
}

PalletDetectionResult PalletDetection::detect(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const std::string& frame_id)
{
  PalletDetectionResult result;

  if (!cloud || cloud->points.empty()) {
    std::cout << "[PalletDetection] Input cloud is empty" << std::endl;
    return result;
  }

  // Step 1: Perform Euclidean clustering to separate objects
  auto cluster_indices = performClustering(cloud);

  std::cout << "[PalletDetection] Found " << cluster_indices.size() << " clusters" << std::endl;

  // Step 2: For each cluster, detect a YZ plane
  int plane_count = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    // Extract cluster points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : cluster_indices[i].indices) {
      cluster_cloud->points.push_back(cloud->points[idx]);
    }
    cluster_cloud->width = cluster_cloud->points.size();
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = true;

    // Detect plane in this cluster
    DetectedPlane plane;
    if (detectPlaneInCluster(cluster_cloud, plane)) {
      // Extract pallet candidate points within plane thickness
      extractPalletCandidates(cloud, plane, result.pallet_candidates);

      // Create visualization marker
      auto marker = createPlaneMarker(plane, plane_count, frame_id);
      result.marker_array.markers.push_back(marker);

      result.detected_planes.push_back(plane);

      std::cout << "[PalletDetection] Detected wall " << plane_count
                << " (cluster " << i << "): normal=["
                << plane.nx << ", " << plane.ny << ", " << plane.nz
                << "], d=" << plane.d << ", inliers=" << plane.num_inliers << std::endl;

      plane_count++;
    }
  }

  // Finalize pallet candidates cloud
  if (!result.pallet_candidates->points.empty()) {
    result.pallet_candidates->width = result.pallet_candidates->points.size();
    result.pallet_candidates->height = 1;
    result.pallet_candidates->is_dense = true;
  }

  std::cout << "[PalletDetection] Total planes detected: " << plane_count << std::endl;
  std::cout << "[PalletDetection] Pallet candidates: " << result.pallet_candidates->points.size()
            << " points from " << plane_count << " planes" << std::endl;

  return result;
}

std::vector<pcl::PointIndices> PalletDetection::performClustering(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(params_.cluster_tolerance);
  ec.setMinClusterSize(params_.min_cluster_size);
  ec.setMaxClusterSize(params_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  return cluster_indices;
}

bool PalletDetection::detectPlaneInCluster(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster_cloud,
  DetectedPlane& plane)
{
  // RANSAC plane segmentation with axis constraint
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params_.max_iterations);
  seg.setDistanceThreshold(params_.distance_threshold);

  // Set axis to X (looking for planes perpendicular to X axis)
  Eigen::Vector3f axis(1.0, 0.0, 0.0);
  seg.setAxis(axis);
  seg.setEpsAngle(0.5);  // ~30 degrees tolerance

  seg.setInputCloud(cluster_cloud);
  seg.segment(*inliers, *coefficients);

  // Check if enough inliers found
  if (inliers->indices.size() < static_cast<size_t>(params_.min_plane_inliers)) {
    return false;
  }

  // Extract plane normal: ax + by + cz + d = 0
  double nx = coefficients->values[0];
  double ny = coefficients->values[1];
  double nz = coefficients->values[2];
  double d = coefficients->values[3];

  // Normalize the normal vector
  double norm = std::sqrt(nx*nx + ny*ny + nz*nz);
  nx /= norm;
  ny /= norm;
  nz /= norm;
  d /= norm;

  // Check if this is a YZ plane (normal should point in X direction)
  if (std::fabs(nx) < params_.normal_x_threshold) {
    return false;
  }

  // Extract inlier points
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cluster_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*(plane.plane_cloud));

  if (plane.plane_cloud->points.empty()) {
    return false;
  }

  // Store plane coefficients
  plane.nx = nx;
  plane.ny = ny;
  plane.nz = nz;
  plane.d = d;
  plane.num_inliers = inliers->indices.size();

  // Calculate plane bounds and coordinate system
  calculatePlaneBounds(plane.plane_cloud, nx, ny, nz, d, plane);

  return true;
}

void PalletDetection::calculatePlaneBounds(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_cloud,
  double nx, double ny, double nz, double d,
  DetectedPlane& plane)
{
  Eigen::Vector3f normal(nx, ny, nz);

  // Calculate centroid of plane points
  Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
  for (const auto& pt : plane_cloud->points) {
    centroid.x() += pt.x;
    centroid.y() += pt.y;
    centroid.z() += pt.z;
  }
  centroid /= plane_cloud->points.size();

  plane.center_x = centroid.x();
  plane.center_y = centroid.y();
  plane.center_z = centroid.z();

  // Create plane coordinate system with stable orientation
  Eigen::Vector3f y_axis;
  if (std::abs(nx) > 0.9) {
    // Normal is nearly parallel to X axis - YZ plane
    // Force Y-axis to align with world Y
    y_axis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  } else {
    // For other orientations, project normal onto XY plane and get perpendicular
    Eigen::Vector3f normal_xy(nx, ny, 0.0f);
    float norm_xy = normal_xy.norm();
    if (norm_xy > 0.01) {
      y_axis = Eigen::Vector3f(-ny, nx, 0.0f) / norm_xy;
    } else {
      y_axis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    }
  }
  y_axis.normalize();

  // X-axis is cross product of Y and Z (normal)
  Eigen::Vector3f x_axis = y_axis.cross(normal);
  x_axis.normalize();

  // Recompute Y-axis to ensure orthogonality
  y_axis = normal.cross(x_axis);
  y_axis.normalize();

  // Store coordinate system
  plane.axis_x[0] = x_axis.x();
  plane.axis_x[1] = x_axis.y();
  plane.axis_x[2] = x_axis.z();

  plane.axis_y[0] = y_axis.x();
  plane.axis_y[1] = y_axis.y();
  plane.axis_y[2] = y_axis.z();

  plane.normal[0] = normal.x();
  plane.normal[1] = normal.y();
  plane.normal[2] = normal.z();

  // Project all points onto the plane coordinate system to find bounds
  float min_u = std::numeric_limits<float>::max();
  float max_u = std::numeric_limits<float>::lowest();
  float min_v = std::numeric_limits<float>::max();
  float max_v = std::numeric_limits<float>::lowest();

  for (const auto& pt : plane_cloud->points) {
    Eigen::Vector3f point(pt.x, pt.y, pt.z);
    Eigen::Vector3f relative = point - centroid;
    float u = relative.dot(x_axis);
    float v = relative.dot(y_axis);

    min_u = std::min(min_u, u);
    max_u = std::max(max_u, u);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }

  plane.min_u = min_u;
  plane.max_u = max_u;
  plane.min_v = min_v;
  plane.max_v = max_v;
}

void PalletDetection::extractPalletCandidates(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const DetectedPlane& plane,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& candidates)
{
  Eigen::Vector3f normal(plane.normal[0], plane.normal[1], plane.normal[2]);
  Eigen::Vector3f centroid(plane.center_x, plane.center_y, plane.center_z);
  Eigen::Vector3f x_axis(plane.axis_x[0], plane.axis_x[1], plane.axis_x[2]);
  Eigen::Vector3f y_axis(plane.axis_y[0], plane.axis_y[1], plane.axis_y[2]);

  float thickness_offset = static_cast<float>(params_.marker_thickness);

  for (const auto& pt : cloud->points) {
    Eigen::Vector3f point(pt.x, pt.y, pt.z);

    // Calculate signed distance from point to plane
    float signed_distance = normal.dot(point) + plane.d;

    // Check if point is within the thickness bounds
    bool inside = false;
    if (params_.marker_bidirectional) {
      // Bidirectional: within ±(thickness/2) from detected plane
      float half_thickness = std::abs(thickness_offset) / 2.0f;
      inside = (std::abs(signed_distance) <= half_thickness + 0.01f);
    } else {
      // Unidirectional: within one-sided thickness
      if (thickness_offset >= 0) {
        inside = (signed_distance >= -0.01f && signed_distance <= thickness_offset + 0.01f);
      } else {
        inside = (signed_distance >= thickness_offset - 0.01f && signed_distance <= 0.01f);
      }
    }

    if (inside) {
      // Check if point is within the plane's XY extent
      Eigen::Vector3f relative = point - centroid;
      float u = relative.dot(x_axis);
      float v = relative.dot(y_axis);

      float margin = 0.1f;  // 10cm margin
      if (u >= plane.min_u - margin && u <= plane.max_u + margin &&
          v >= plane.min_v - margin && v <= plane.max_v + margin) {
        candidates->points.push_back(pt);
      }
    }
  }
}

visualization_msgs::msg::Marker PalletDetection::createPlaneMarker(
  const DetectedPlane& plane,
  int marker_id,
  const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "yz_plane";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Apply thickness extension along the normal direction
  float thickness_offset = static_cast<float>(params_.marker_thickness);
  Eigen::Vector3f normal(plane.normal[0], plane.normal[1], plane.normal[2]);
  Eigen::Vector3f centroid(plane.center_x, plane.center_y, plane.center_z);

  Eigen::Vector3f marker_center;
  float marker_thickness;

  if (params_.marker_bidirectional) {
    // Bidirectional: extend equally in both normal directions
    marker_center = centroid;
    marker_thickness = 0.02f + std::abs(thickness_offset);
  } else {
    // Unidirectional: extend in one direction only
    marker_center = centroid + normal * (thickness_offset * 0.5f);
    marker_thickness = 0.02f + std::abs(thickness_offset);
  }

  // Position at calculated center
  marker.pose.position.x = marker_center.x();
  marker.pose.position.y = marker_center.y();
  marker.pose.position.z = marker_center.z();

  // Orientation: align marker's Z-axis with plane normal
  Eigen::Vector3f x_axis(plane.axis_x[0], plane.axis_x[1], plane.axis_x[2]);
  Eigen::Vector3f y_axis(plane.axis_y[0], plane.axis_y[1], plane.axis_y[2]);

  Eigen::Matrix3f rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = normal;

  Eigen::Quaternionf quat(rotation);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // Size based on plane extent
  marker.scale.x = std::max(0.1f, plane.max_u - plane.min_u);
  marker.scale.y = std::max(0.1f, plane.max_v - plane.min_v);
  marker.scale.z = marker_thickness;

  // Blue color for YZ planes
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 1.0;
  marker.color.a = 0.5;

  marker.lifetime = rclcpp::Duration::from_seconds(2.0);

  return marker;
}

}  // namespace floor_removal_rgbd
