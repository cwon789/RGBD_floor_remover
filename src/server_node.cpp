#include "floor_removal_rgbd/server_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

  // Create PalletDetection with loaded parameters
  enable_pallet_detection_ = this->get_parameter("enable_pallet_detection").as_bool();

  if (enable_pallet_detection_) {
    PalletDetectionParams pallet_params;
    pallet_params.line_distance_threshold = this->get_parameter("pallet_line_distance_threshold").as_double();
    pallet_params.line_min_points = this->get_parameter("pallet_line_min_points").as_int();
    pallet_params.line_max_iterations = this->get_parameter("pallet_line_max_iterations").as_int();
    pallet_params.line_merge_angle_threshold = this->get_parameter("pallet_line_merge_angle_threshold").as_double();
    pallet_params.line_merge_distance_threshold = this->get_parameter("pallet_line_merge_distance_threshold").as_double();
    pallet_params.line_min_length = this->get_parameter("pallet_line_min_length").as_double();
    pallet_params.line_max_length = this->get_parameter("pallet_line_max_length").as_double();
    pallet_params.line_max_length_tolerance = this->get_parameter("pallet_line_max_length_tolerance").as_double();
    pallet_params.marker_thickness = this->get_parameter("pallet_marker_thickness").as_double();
    pallet_params.marker_height = this->get_parameter("pallet_marker_height").as_double();

    // Preprocessing parameters
    pallet_params.dbscan_eps = this->get_parameter("pallet_dbscan_eps").as_double();
    pallet_params.dbscan_min_points = this->get_parameter("pallet_dbscan_min_points").as_int();
    pallet_params.angle_bin_size = this->get_parameter("pallet_angle_bin_size").as_double();

    // Cuboid volume generation parameters
    pallet_params.cuboid_height = this->get_parameter("pallet_cuboid_height").as_double();
    pallet_params.cuboid_thickness = this->get_parameter("pallet_cuboid_thickness").as_double();

    pallet_detection_ = std::make_unique<PalletDetection>(pallet_params);

    RCLCPP_INFO(this->get_logger(), "[PALLET] Pallet detection enabled (2D line extraction)");
    RCLCPP_INFO(this->get_logger(), "[PALLET]   Line distance threshold: %.3f m", pallet_params.line_distance_threshold);
    RCLCPP_INFO(this->get_logger(), "[PALLET]   Min points per line: %d", pallet_params.line_min_points);
    RCLCPP_INFO(this->get_logger(), "[PALLET]   Min line length: %.3f m", pallet_params.line_min_length);
    RCLCPP_INFO(this->get_logger(), "[PALLET]   Marker height: %.3f m", pallet_params.marker_height);
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

  // Pallet detection publishers
  if (enable_pallet_detection_) {
    extracted_lines_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/extracted_lines", 10);
    pallet_candidates_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pallet_candidates", 10);
    pallet_cuboid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pallet_cuboid", 10);
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

  // Pallet detection parameters (line-based)
  this->declare_parameter<bool>("enable_pallet_detection", true);
  this->declare_parameter<double>("pallet_line_distance_threshold", 0.02);
  this->declare_parameter<int>("pallet_line_min_points", 20);
  this->declare_parameter<int>("pallet_line_max_iterations", 100);
  this->declare_parameter<double>("pallet_line_merge_angle_threshold", 5.0);
  this->declare_parameter<double>("pallet_line_merge_distance_threshold", 0.1);
  this->declare_parameter<double>("pallet_line_min_length", 0.3);
  this->declare_parameter<double>("pallet_line_max_length", 2.0);
  this->declare_parameter<double>("pallet_line_max_length_tolerance", 0.2);
  this->declare_parameter<double>("pallet_marker_thickness", 0.02);
  this->declare_parameter<double>("pallet_marker_height", 0.5);

  // Preprocessing parameters
  this->declare_parameter<double>("pallet_dbscan_eps", 0.05);
  this->declare_parameter<int>("pallet_dbscan_min_points", 5);
  this->declare_parameter<double>("pallet_angle_bin_size", 0.5);

  // Cuboid volume generation parameters
  this->declare_parameter<double>("pallet_cuboid_height", 1.0);
  this->declare_parameter<double>("pallet_cuboid_thickness", 0.1);
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

  // Pallet detection (line extraction from 2D projected cloud)
  if (enable_pallet_detection_ && pallet_detection_ &&
      result.no_floor_cloud_voxelized_2d_projected &&
      !result.no_floor_cloud_voxelized_2d_projected->points.empty()) {

    // Detect lines in 2D projected cloud
    auto pallet_result = pallet_detection_->detect(
      result.no_floor_cloud_voxelized_2d_projected,
      msg->header.frame_id);

    // Publish extracted line markers
    if (!pallet_result.line_markers.markers.empty()) {
      extracted_lines_pub_->publish(pallet_result.line_markers);
    }

    // Publish pallet candidates
    if (!pallet_result.pallet_candidates->points.empty()) {
      sensor_msgs::msg::PointCloud2 pallet_candidates_msg;
      pcl::toROSMsg(*pallet_result.pallet_candidates, pallet_candidates_msg);
      pallet_candidates_msg.header = msg->header;
      pallet_candidates_pub_->publish(pallet_candidates_msg);
    }

    // Publish pallet cuboid markers
    if (!pallet_result.cuboid_markers.markers.empty()) {
      pallet_cuboid_pub_->publish(pallet_result.cuboid_markers);
    }
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
