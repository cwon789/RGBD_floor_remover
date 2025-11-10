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
  params.auto_floor_detection_mode = this->get_parameter("auto_floor_detection_mode").as_bool();
  params.floor_height = this->get_parameter("floor_height").as_double();
  params.floor_detection_thickness = this->get_parameter("floor_detection_thickness").as_double();
  params.floor_removal_thickness = this->get_parameter("floor_removal_thickness").as_double();
  params.floor_margin = this->get_parameter("floor_margin").as_double();
  params.use_voxel_grid = this->get_parameter("use_voxel_grid").as_bool();
  params.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
  params.max_detection_distance = this->get_parameter("max_detection_distance").as_double();
  params.enable_stringer_detection = this->get_parameter("enable_stringer_detection").as_bool();
  params.stringer_width_min = this->get_parameter("stringer_width_min").as_double();
  params.stringer_width_max = this->get_parameter("stringer_width_max").as_double();
  params.stringer_height_min = this->get_parameter("stringer_height_min").as_double();
  params.stringer_height_max = this->get_parameter("stringer_height_max").as_double();

  // Camera extrinsic parameters
  params.use_default_transform = this->get_parameter("use_default_transform").as_bool();
  params.cam_tx = this->get_parameter("cam_tx").as_double();
  params.cam_ty = this->get_parameter("cam_ty").as_double();
  params.cam_tz = this->get_parameter("cam_tz").as_double();
  params.cam_roll = this->get_parameter("cam_roll").as_double();
  params.cam_pitch = this->get_parameter("cam_pitch").as_double();
  params.cam_yaw = this->get_parameter("cam_yaw").as_double();

  plane_remover_ = std::make_unique<PlaneRemover>(params);

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

  stringer_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/stringer_markers", 10);

  stringer_centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/stringer_centers", 10);

  RCLCPP_INFO(this->get_logger(), "Floor Removal Server Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output floor: %s", output_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output no-floor: %s", output_no_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output floor (voxelized): %s", output_floor_cloud_voxelized_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output no-floor (voxelized): %s", output_no_floor_cloud_voxelized_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Floor detection mode: %s",
              params.auto_floor_detection_mode ? "auto (z_min)" : "fixed (floor height)");
  if (!params.auto_floor_detection_mode) {
    RCLCPP_INFO(this->get_logger(), "  Floor height: %.3f m", params.floor_height);
  }
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

  // Floor detection mode
  this->declare_parameter<bool>("auto_floor_detection_mode", true);
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
  this->declare_parameter<bool>("enable_stringer_detection", true);
  this->declare_parameter<double>("stringer_width_min", 0.05);
  this->declare_parameter<double>("stringer_width_max", 0.15);
  this->declare_parameter<double>("stringer_height_min", 0.08);
  this->declare_parameter<double>("stringer_height_max", 0.20);

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
      if (!result.detected_stringers.empty()) {
        RCLCPP_INFO(this->get_logger(),
                    "  Detected %zu stringers",
                    result.detected_stringers.size());
      }
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

  // Publish stringer column markers (line segments)
  std::cout << "[DEBUG SERVER] Publishing line markers for " << result.detected_columns.size() << " column stringers" << std::endl;

  if (!result.detected_columns.empty()) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < result.detected_columns.size(); ++i) {
      const auto& column = result.detected_columns[i];

      std::cout << "[DEBUG SERVER] Column marker " << i
                << " from (" << column.start_x << ", " << column.start_y << ", " << column.start_z << ")"
                << " to (" << column.end_x << ", " << column.end_y << ", " << column.end_z << ")"
                << " Length=" << column.length << "m" << std::endl;

      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "stringer_columns";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Line from start to end
      geometry_msgs::msg::Point start_point;
      start_point.x = column.start_x;
      start_point.y = column.start_y;
      start_point.z = column.start_z;

      geometry_msgs::msg::Point end_point;
      end_point.x = column.end_x;
      end_point.y = column.end_y;
      end_point.z = column.end_z;

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);

      // Line width
      marker.scale.x = 0.02;  // 2cm thick line

      // Set color (green for detected stringers)
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      marker_array.markers.push_back(marker);
    }

    stringer_markers_pub_->publish(marker_array);
  } else {
    // Clear previous markers
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = msg->header;
    delete_marker.ns = "stringer_columns";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    stringer_markers_pub_->publish(marker_array);
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
