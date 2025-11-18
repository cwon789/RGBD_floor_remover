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
  loosely_coupled_ = this->get_parameter("loosely_coupled").as_bool();
  enable_hole_detection_ = this->get_parameter("enable_hole_detection").as_bool();

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
    RCLCPP_INFO(this->get_logger(), "[PALLET]   Loosely coupled: %s", loosely_coupled_ ? "true" : "false");
  }

  // Create HoleDetector with loaded parameters
  if (enable_hole_detection_) {
    HoleDetectorParams hole_params;
    hole_params.grid_resolution = this->get_parameter("hole_grid_resolution").as_double();
    hole_params.min_points_per_cell = this->get_parameter("hole_min_points_per_cell").as_int();
    hole_params.min_hole_cells = this->get_parameter("hole_min_hole_cells").as_int();
    hole_params.hole_z_min = this->get_parameter("hole_z_min").as_double();
    hole_params.hole_z_max = this->get_parameter("hole_z_max").as_double();
    hole_params.search_distance_from_line = this->get_parameter("hole_search_distance_from_line").as_double();
    hole_params.marker_thickness = this->get_parameter("hole_marker_thickness").as_double();

    hole_detector_ = std::make_unique<HoleDetector>(hole_params);

    RCLCPP_INFO(this->get_logger(), "[HOLE] Hole detection enabled");
    RCLCPP_INFO(this->get_logger(), "[HOLE]   Grid resolution: %.3f m", hole_params.grid_resolution);
    RCLCPP_INFO(this->get_logger(), "[HOLE]   Min points per cell: %d", hole_params.min_points_per_cell);
    RCLCPP_INFO(this->get_logger(), "[HOLE]   Min hole cells: %d", hole_params.min_hole_cells);
    RCLCPP_INFO(this->get_logger(), "[HOLE]   Height range: [%.3f, %.3f] m", hole_params.hole_z_min, hole_params.hole_z_max);
    RCLCPP_INFO(this->get_logger(), "[HOLE]   Search distance: %.3f m", hole_params.search_distance_from_line);
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
    line_candidates_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/line_candidates", 10);
    pallet_candidates_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pallet_candidates", 10);
    pallet_cuboid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pallet_cuboid", 10);

    // PoseStamped publisher for loosely coupled mode
    if (loosely_coupled_) {
      pallet_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pallet_pose", 10);
    }
  }

  // Hole detection publishers
  if (enable_hole_detection_) {
    pallet_holes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pallet_holes", 10);
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
  this->declare_parameter<bool>("loosely_coupled", false);
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
  this->declare_parameter<double>("pallet_angle_bin_size", 0.5);

  // Cuboid volume generation parameters
  this->declare_parameter<double>("pallet_cuboid_height", 1.0);
  this->declare_parameter<double>("pallet_cuboid_thickness", 0.1);

  // Hole detection parameters
  this->declare_parameter<bool>("enable_hole_detection", true);
  this->declare_parameter<double>("hole_grid_resolution", 0.05);
  this->declare_parameter<int>("hole_min_points_per_cell", 3);
  this->declare_parameter<int>("hole_min_hole_cells", 4);
  this->declare_parameter<double>("hole_z_min", 0.05);
  this->declare_parameter<double>("hole_z_max", 0.30);
  this->declare_parameter<double>("hole_search_distance_from_line", 0.20);
  this->declare_parameter<double>("hole_marker_thickness", 0.02);
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

    // Detect lines in 2D projected cloud and filter no_floor points inside cuboids
    auto pallet_result = pallet_detection_->detect(
      result.no_floor_cloud_voxelized_2d_projected,
      result.no_floor_cloud_voxelized,  // Pass no_floor cloud to filter with cuboids
      result.nx, result.ny, result.nz, result.d,  // Floor plane parameters
      msg->header.frame_id);

    // Publish extracted line markers
    if (!pallet_result.line_markers.markers.empty()) {
      extracted_lines_pub_->publish(pallet_result.line_markers);
    }

    // Publish line candidates (2D line points)
    if (!pallet_result.line_candidates->points.empty()) {
      sensor_msgs::msg::PointCloud2 line_candidates_msg;
      pcl::toROSMsg(*pallet_result.line_candidates, line_candidates_msg);
      line_candidates_msg.header = msg->header;
      line_candidates_pub_->publish(line_candidates_msg);
    }

    // Publish pallet candidates (floor points inside cuboids)
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

    // Publish cuboid poses if loosely coupled mode is enabled
    if (loosely_coupled_ && pallet_pose_pub_ && !pallet_result.detected_lines.empty()) {
      // Publish each detected line as a PoseStamped
      for (const auto& line : pallet_result.detected_lines) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;

        // Calculate line center point
        double center_x = (line.start_x + line.end_x) / 2.0;
        double center_y = (line.start_y + line.end_y) / 2.0;
        double center_z = pallet_detection_->getParams().cuboid_height / 2.0;

        pose_msg.pose.position.x = center_x;
        pose_msg.pose.position.y = center_y;
        pose_msg.pose.position.z = center_z;

        // Calculate orientation from line angle (rotation around Z axis)
        // Adjust angle: line.angle is relative to X-axis, add -π/2 to align with desired frame
        // where X=forward(inward), Y=left, Z=up
        double yaw = line.angle - M_PI / 2.0;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
        pose_msg.pose.orientation.w = std::cos(yaw / 2.0);

        pallet_pose_pub_->publish(pose_msg);
      }
    }

    // Hole detection (detect empty spaces in pallet candidates)
    if (enable_hole_detection_ && hole_detector_ &&
        !pallet_result.pallet_candidates->points.empty() &&
        !pallet_result.detected_lines.empty()) {
      auto hole_result = hole_detector_->detect(
        pallet_result.pallet_candidates,
        pallet_result.detected_lines,
        msg->header.frame_id);

      // Publish hole markers
      if (!hole_result.hole_markers.markers.empty()) {
        pallet_holes_pub_->publish(hole_result.hole_markers);
      }
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
