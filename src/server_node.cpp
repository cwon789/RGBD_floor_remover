#include "floor_removal_rgbd/server_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Core>
#include <cmath>
#include <limits>

namespace floor_removal_rgbd
{

FloorRemovalServerNode::FloorRemovalServerNode()
  : Node("floor_removal_node")
{
  // Declare all parameters
  declareParameters();

  // Load parameters
  loadParameters();

  // Create PlaneRemover with loaded parameters
  PlaneRemoverParams params;
  params.ransac_distance_threshold = this->get_parameter("ransac_distance_threshold").as_double();
  params.ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();
  params.floor_normal_z_threshold = this->get_parameter("floor_normal_z_threshold").as_double();
  params.floor_height = this->get_parameter("floor_height").as_double();
  params.floor_detection_thickness = this->get_parameter("floor_detection_thickness").as_double();
  params.floor_removal_thickness = this->get_parameter("floor_removal_thickness").as_double();
  params.floor_margin = this->get_parameter("floor_margin").as_double();
  params.use_voxel_grid = this->get_parameter("use_voxel_grid").as_bool();
  params.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
  params.max_detection_distance = this->get_parameter("max_detection_distance").as_double();

  // Camera extrinsic parameters
  params.use_default_transform = this->get_parameter("use_default_transform").as_bool();
  params.cam_tx = this->get_parameter("cam_tx").as_double();
  params.cam_ty = this->get_parameter("cam_ty").as_double();
  params.cam_tz = this->get_parameter("cam_tz").as_double();
  params.cam_roll = this->get_parameter("cam_roll").as_double();
  params.cam_pitch = this->get_parameter("cam_pitch").as_double();
  params.cam_yaw = this->get_parameter("cam_yaw").as_double();

  plane_remover_ = std::make_unique<PlaneRemover>(params);

  // Load YZ plane detection parameters
  enable_yz_plane_detection_ = this->get_parameter("enable_yz_plane_detection").as_bool();
  yz_plane_distance_threshold_ = this->get_parameter("yz_plane_distance_threshold").as_double();
  yz_plane_max_iterations_ = this->get_parameter("yz_plane_max_iterations").as_int();
  yz_plane_normal_x_threshold_ = this->get_parameter("yz_plane_normal_x_threshold").as_double();
  yz_plane_marker_thickness_ = this->get_parameter("yz_plane_marker_thickness").as_double();
  yz_plane_marker_bidirectional_ = this->get_parameter("yz_plane_marker_bidirectional").as_bool();

  if (enable_yz_plane_detection_) {
    RCLCPP_INFO(this->get_logger(), "[YZ PLANE] YZ plane detection enabled");
    RCLCPP_INFO(this->get_logger(), "[YZ PLANE]   Distance threshold: %.3f m", yz_plane_distance_threshold_);
    RCLCPP_INFO(this->get_logger(), "[YZ PLANE]   Normal X threshold: %.3f", yz_plane_normal_x_threshold_);
    RCLCPP_INFO(this->get_logger(), "[YZ PLANE]   Marker thickness: %.3f m (%s)",
                yz_plane_marker_thickness_,
                yz_plane_marker_bidirectional_ ? "bidirectional" : "unidirectional");
  }

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&FloorRemovalServerNode::cloudCallback, this, std::placeholders::_1));

  // Publishers
  floor_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_floor_cloud_topic_, 10);

  no_floor_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_no_floor_cloud_topic_, 10);

  floor_cloud_voxelized_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_floor_cloud_voxelized_topic_, 10);

  no_floor_cloud_voxelized_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_no_floor_cloud_voxelized_topic_, 10);

  // Publisher for 2D projected no-floor cloud
  no_floor_cloud_voxelized_2d_projected_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/no_floor_cloud_voxelized_2d_projected", 10);

  stringer_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/stringer_markers", 10);

  stringer_centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/stringer_centers", 10);

  intersection_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/intersection_points", 10);

  // YZ plane marker publisher
  if (enable_yz_plane_detection_) {
    yz_plane_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/yz_plane_markers", 10);
    pallet_candidates_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pallet_candidates", 10);
  }

  RCLCPP_INFO(this->get_logger(), "Floor Removal Server Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output floor: %s", output_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output no-floor: %s", output_no_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output floor (voxelized): %s", output_floor_cloud_voxelized_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output no-floor (voxelized): %s", output_no_floor_cloud_voxelized_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Floor height: %.3f m", params.floor_height);
  RCLCPP_INFO(this->get_logger(), "  Voxel grid: %s (leaf size: %.3f m)",
              params.use_voxel_grid ? "enabled" : "disabled",
              params.voxel_leaf_size);
  RCLCPP_INFO(this->get_logger(), "  Camera transform: %s",
              params.use_default_transform ? "default (optical->base)" : "custom extrinsic");
  if (!params.use_default_transform) {
    RCLCPP_INFO(this->get_logger(), "  Extrinsic T: [%.3f, %.3f, %.3f] m",
                params.cam_tx, params.cam_ty, params.cam_tz);
    RCLCPP_INFO(this->get_logger(), "  Extrinsic R: [%.3f, %.3f, %.3f] rad",
                params.cam_roll, params.cam_pitch, params.cam_yaw);
  }
}

void FloorRemovalServerNode::declareParameters()
{
  // Input/output topics
  this->declare_parameter<std::string>("input_cloud_topic", "/camera/depth/color/points");
  this->declare_parameter<std::string>("output_floor_cloud_topic", "/floor_cloud");
  this->declare_parameter<std::string>("output_no_floor_cloud_topic", "/no_floor_cloud");
  this->declare_parameter<std::string>("output_floor_cloud_voxelized_topic", "/floor_cloud_voxelized");
  this->declare_parameter<std::string>("output_no_floor_cloud_voxelized_topic", "/no_floor_cloud_voxelized");

  // TF frames
  this->declare_parameter<std::string>("camera_frame", "camera_link");
  this->declare_parameter<std::string>("base_frame", "camera_link");

  // RANSAC parameters
  this->declare_parameter<double>("ransac_distance_threshold", 0.02);
  this->declare_parameter<int>("ransac_max_iterations", 100);
  this->declare_parameter<double>("floor_normal_z_threshold", 0.15);

  this->declare_parameter<double>("floor_height", 0.0);

  // Floor region parameters
  this->declare_parameter<double>("floor_detection_thickness", 0.15);
  this->declare_parameter<double>("floor_removal_thickness", 0.03);
  this->declare_parameter<double>("floor_margin", 0.01);

  // Voxel grid parameters
  this->declare_parameter<bool>("use_voxel_grid", true);
  this->declare_parameter<double>("voxel_leaf_size", 0.005);

  // Detection range parameters
  this->declare_parameter<double>("max_detection_distance", 10.0);

  // Stringer detection parameters

  // Camera extrinsic parameters
  this->declare_parameter<bool>("use_default_transform", true);
  this->declare_parameter<double>("cam_tx", 0.0);
  this->declare_parameter<double>("cam_ty", 0.0);
  this->declare_parameter<double>("cam_tz", 0.0);
  this->declare_parameter<double>("cam_roll", 0.0);
  this->declare_parameter<double>("cam_pitch", 0.0);
  this->declare_parameter<double>("cam_yaw", 0.0);

  // Legacy parameters (kept for compatibility)
  this->declare_parameter<double>("floor_height_min", -5.0);
  this->declare_parameter<double>("floor_height_max", -0.3);

  // YZ plane detection parameters
  this->declare_parameter<bool>("enable_yz_plane_detection", true);
  this->declare_parameter<double>("yz_plane_distance_threshold", 0.02);
  this->declare_parameter<int>("yz_plane_max_iterations", 100);
  this->declare_parameter<double>("yz_plane_normal_x_threshold", 0.7);
  this->declare_parameter<double>("yz_plane_marker_thickness", 0.0);  // Default: no offset
  this->declare_parameter<bool>("yz_plane_marker_bidirectional", false);  // Default: unidirectional
}

void FloorRemovalServerNode::loadParameters()
{
  input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
  output_floor_cloud_topic_ = this->get_parameter("output_floor_cloud_topic").as_string();
  output_no_floor_cloud_topic_ = this->get_parameter("output_no_floor_cloud_topic").as_string();
  output_floor_cloud_voxelized_topic_ = this->get_parameter("output_floor_cloud_voxelized_topic").as_string();
  output_no_floor_cloud_voxelized_topic_ = this->get_parameter("output_no_floor_cloud_voxelized_topic").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
}

void FloorRemovalServerNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud_camera);

  if (cloud_camera->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }

  // Process with PlaneRemover
  auto result = plane_remover_->process(cloud_camera);

  // Log results periodically (every 30 frames)
  if (debug_counter_++ % 30 == 0) {
    if (result.plane_found) {
      RCLCPP_INFO(this->get_logger(),
                  "Processed: %zu pts -> voxel: %zu pts, floor region: %zu pts, floor: %zu pts (%.1f%%)",
                  result.total_points,
                  result.voxelized_points,
                  result.floor_region_points,
                  result.floor_points,
                  100.0 * result.floor_points / result.total_points);
      RCLCPP_INFO(this->get_logger(),
                  "  Plane: normal=[%.2f, %.2f, %.2f], d=%.3f",
                  result.nx, result.ny, result.nz, result.d);
    } else {
      RCLCPP_WARN(this->get_logger(), "No floor plane detected");
    }
  }

  // Convert PCL clouds back to ROS messages
  sensor_msgs::msg::PointCloud2 floor_msg, no_floor_msg;
  sensor_msgs::msg::PointCloud2 floor_voxelized_msg, no_floor_voxelized_msg;

  pcl::toROSMsg(*result.floor_cloud, floor_msg);
  pcl::toROSMsg(*result.no_floor_cloud, no_floor_msg);
  pcl::toROSMsg(*result.floor_cloud_voxelized, floor_voxelized_msg);
  pcl::toROSMsg(*result.no_floor_cloud_voxelized, no_floor_voxelized_msg);

  // Set headers
  floor_msg.header = msg->header;
  no_floor_msg.header = msg->header;
  floor_voxelized_msg.header = msg->header;
  no_floor_voxelized_msg.header = msg->header;

  // Publish (even if plane not found, transformed clouds will be published)
  floor_cloud_pub_->publish(floor_msg);
  no_floor_cloud_pub_->publish(no_floor_msg);
  floor_cloud_voxelized_pub_->publish(floor_voxelized_msg);
  no_floor_cloud_voxelized_pub_->publish(no_floor_voxelized_msg);

  // Publish 2D projected no-floor cloud
  if (result.no_floor_cloud_voxelized_2d_projected) {
    sensor_msgs::msg::PointCloud2 no_floor_voxelized_2d_projected_msg;
    pcl::toROSMsg(*result.no_floor_cloud_voxelized_2d_projected, no_floor_voxelized_2d_projected_msg);
    no_floor_voxelized_2d_projected_msg.header = msg->header;
    no_floor_cloud_voxelized_2d_projected_pub_->publish(no_floor_voxelized_2d_projected_msg);
  }

  // YZ plane detection (detect vertical walls) - Cluster-based approach
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "[YZ PLANE DEBUG] enable=%d, no_floor_voxelized size=%zu",
              enable_yz_plane_detection_, result.no_floor_cloud_voxelized->points.size());

  if (enable_yz_plane_detection_ && !result.no_floor_cloud_voxelized->points.empty()) {
    // Create marker array for all planes
    visualization_msgs::msg::MarkerArray marker_array;

    // Accumulate all pallet candidate points across all detected planes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_pallet_candidates(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Step 1: Euclidean Clustering to separate objects
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(result.no_floor_cloud_voxelized);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.10);  // 10cm - 클러스터 간 거리
    ec.setMinClusterSize(50);      // 최소 포인트 개수
    ec.setMaxClusterSize(25000);   // 최대 포인트 개수
    ec.setSearchMethod(tree);
    ec.setInputCloud(result.no_floor_cloud_voxelized);
    ec.extract(cluster_indices);

    if (debug_counter_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[YZ PLANE DEBUG] Found %zu clusters", cluster_indices.size());
    }

    // Step 2: For each cluster, detect a plane
    int plane_count = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      // Extract cluster points
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto& idx : cluster_indices[i].indices) {
        cluster_cloud->points.push_back(result.no_floor_cloud_voxelized->points[idx]);
      }
      cluster_cloud->width = cluster_cloud->points.size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;

      if (debug_counter_ % 30 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "[YZ PLANE DEBUG] Cluster %zu: %zu points", i, cluster_cloud->points.size());
      }

      // RANSAC plane segmentation with axis constraint
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(yz_plane_max_iterations_);
      seg.setDistanceThreshold(yz_plane_distance_threshold_);

      // Set axis to X (looking for planes perpendicular to X axis)
      Eigen::Vector3f axis(1.0, 0.0, 0.0);  // X axis
      seg.setAxis(axis);
      seg.setEpsAngle(0.5);  // ~30 degrees tolerance

      seg.setInputCloud(cluster_cloud);
      seg.segment(*inliers, *coefficients);

      // Check if enough inliers found
      if (inliers->indices.size() < 30) {  // 클러스터별로 더 작은 threshold
        continue;
      }

      // Extract plane normal: ax + by + cz + d = 0
      double nx = coefficients->values[0];
      double ny = coefficients->values[1];
      double nz = coefficients->values[2];
      double d = coefficients->values[3];

      // Normalize the normal vector
      double norm = sqrt(nx*nx + ny*ny + nz*nz);
      nx /= norm;
      ny /= norm;
      nz /= norm;
      d /= norm;

      if (debug_counter_ % 30 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "[YZ PLANE DEBUG] Cluster %zu plane: normal=[%.2f, %.2f, %.2f], inliers=%zu",
                    i, nx, ny, nz, inliers->indices.size());
      }

      // Check if this is a YZ plane (normal should point in X direction)
      if (fabs(nx) > yz_plane_normal_x_threshold_) {
        // Extract inlier points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cluster_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);

        // Calculate plane bounds for visualization using plane-fitted coordinate system
        if (!plane_cloud->points.empty()) {
          // Plane equation: nx*x + ny*y + nz*z + d = 0
          // Normal vector: (nx, ny, nz) - already normalized
          Eigen::Vector3f normal(nx, ny, nz);

          // Calculate centroid of plane points
          Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
          for (const auto& pt : plane_cloud->points) {
            centroid.x() += pt.x;
            centroid.y() += pt.y;
            centroid.z() += pt.z;
          }
          centroid /= plane_cloud->points.size();

          // Create plane coordinate system with stable orientation
          // Z-axis: normal direction
          // For YZ planes (normal in X direction), we want consistent Y and Z axes

          // Force Y-axis to be horizontal (in XY plane, perpendicular to normal's XY component)
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
              // Y-axis perpendicular to normal in XY plane
              y_axis = Eigen::Vector3f(-ny, nx, 0.0f) / norm_xy;
            } else {
              // Normal is nearly vertical
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

          // Project all points onto the plane coordinate system
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

          // Create visualization marker
          visualization_msgs::msg::Marker marker;
          marker.header = msg->header;
          marker.ns = "yz_plane";
          marker.id = plane_count;
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.action = visualization_msgs::msg::Marker::ADD;

          // Apply thickness extension along the normal direction
          float thickness_offset = static_cast<float>(yz_plane_marker_thickness_);
          Eigen::Vector3f marker_center;
          float marker_thickness;

          if (yz_plane_marker_bidirectional_) {
            // Bidirectional: extend equally in both normal directions from detected plane
            // Marker center stays at detected plane centroid
            marker_center = centroid;
            marker_thickness = 0.02f + std::abs(thickness_offset);
          } else {
            // Unidirectional: extend in one direction only
            // Positive: extend in +normal direction
            // Negative: extend in -normal direction
            // Marker center shifts by half to keep one side at the detected plane
            marker_center = centroid + normal * (thickness_offset * 0.5f);
            marker_thickness = 0.02f + std::abs(thickness_offset);
          }

          // Position at calculated center
          marker.pose.position.x = marker_center.x();
          marker.pose.position.y = marker_center.y();
          marker.pose.position.z = marker_center.z();

          // Orientation: align marker's Z-axis with plane normal
          // Create rotation matrix from plane coordinate system
          Eigen::Matrix3f rotation;
          rotation.col(0) = x_axis;
          rotation.col(1) = y_axis;
          rotation.col(2) = normal;

          Eigen::Quaternionf quat(rotation);
          marker.pose.orientation.x = quat.x();
          marker.pose.orientation.y = quat.y();
          marker.pose.orientation.z = quat.z();
          marker.pose.orientation.w = quat.w();

          // Size based on plane extent in plane coordinate system
          // X and Y: extent along the plane
          // Z: thickness based on mode
          marker.scale.x = std::max(0.1f, max_u - min_u);
          marker.scale.y = std::max(0.1f, max_v - min_v);
          marker.scale.z = marker_thickness;

          // Blue color for YZ planes
          marker.color.r = 0.0;
          marker.color.g = 0.5;
          marker.color.b = 1.0;
          marker.color.a = 0.5;

          marker.lifetime = rclcpp::Duration::from_seconds(2.0);  // 2초로 증가

          // Add marker to array instead of publishing immediately
          marker_array.markers.push_back(marker);

          // Filter points that are within the extended plane thickness
          // Use the entire no_floor_cloud_voxelized, not just the cluster
          for (const auto& pt : result.no_floor_cloud_voxelized->points) {
            Eigen::Vector3f point(pt.x, pt.y, pt.z);

            // Calculate signed distance from point to plane
            float signed_distance = normal.dot(point) + d;

            // Check if point is within the thickness bounds
            bool inside = false;
            if (yz_plane_marker_bidirectional_) {
              // Bidirectional: within ±(thickness/2) from detected plane
              float half_thickness = std::abs(thickness_offset) / 2.0f;
              inside = (std::abs(signed_distance) <= half_thickness + 0.01f);  // 0.01 = base thickness/2
            } else {
              // Unidirectional: within one-sided thickness
              if (thickness_offset >= 0) {
                // Extend in +normal direction: plane to plane+thickness
                inside = (signed_distance >= -0.01f && signed_distance <= thickness_offset + 0.01f);
              } else {
                // Extend in -normal direction: plane+thickness to plane
                inside = (signed_distance >= thickness_offset - 0.01f && signed_distance <= 0.01f);
              }
            }

            if (inside) {
              // Also check if point is within the plane's XY extent (optional - to avoid points behind/front)
              Eigen::Vector3f relative = point - centroid;
              float u = relative.dot(x_axis);
              float v = relative.dot(y_axis);

              // Allow some margin beyond the detected plane extent
              float margin = 0.1f;  // 10cm margin
              if (u >= min_u - margin && u <= max_u + margin &&
                  v >= min_v - margin && v <= max_v + margin) {
                all_pallet_candidates->points.push_back(pt);
              }
            }
          }

          if (debug_counter_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "[YZ PLANE] Detected wall %d (cluster %zu): normal=[%.2f, %.2f, %.2f], d=%.3f, inliers=%zu",
                        plane_count, i, nx, ny, nz, d, inliers->indices.size());
          }

          plane_count++;
        }
      }
    }

    // Publish all markers at once
    if (!marker_array.markers.empty()) {
      yz_plane_marker_pub_->publish(marker_array);
    }

    // Publish pallet candidates
    if (!all_pallet_candidates->points.empty()) {
      all_pallet_candidates->width = all_pallet_candidates->points.size();
      all_pallet_candidates->height = 1;
      all_pallet_candidates->is_dense = true;

      sensor_msgs::msg::PointCloud2 pallet_candidates_msg;
      pcl::toROSMsg(*all_pallet_candidates, pallet_candidates_msg);
      pallet_candidates_msg.header = msg->header;
      pallet_candidates_pub_->publish(pallet_candidates_msg);

      if (debug_counter_ % 30 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "[YZ PLANE] Pallet candidates: %zu points from %d planes",
                    all_pallet_candidates->points.size(), plane_count);
      }
    }

    if (debug_counter_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[YZ PLANE DEBUG] Total planes detected: %d", plane_count);
    }
  }

  // Only process stringers if plane was found
  if (!result.plane_found) {
    return;
  }

  // Publish stringer centers
  if (!result.stringer_centers->points.empty()) {
    sensor_msgs::msg::PointCloud2 stringer_centers_msg;
    pcl::toROSMsg(*result.stringer_centers, stringer_centers_msg);
    stringer_centers_msg.header = msg->header;
    stringer_centers_pub_->publish(stringer_centers_msg);
  }

  // Publish intersection points
  if (!result.intersection_points->points.empty()) {
    sensor_msgs::msg::PointCloud2 intersection_points_msg;
    pcl::toROSMsg(*result.intersection_points, intersection_points_msg);
    intersection_points_msg.header = msg->header;
    intersection_points_pub_->publish(intersection_points_msg);
  }

}

}  // namespace floor_removal_rgbd

// Main function
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<floor_removal_rgbd::FloorRemovalServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
