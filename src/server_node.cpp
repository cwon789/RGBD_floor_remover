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
  params.floor_detection_thickness = this->get_parameter("floor_detection_thickness").as_double();
  params.floor_removal_thickness = this->get_parameter("floor_removal_thickness").as_double();
  params.floor_margin = this->get_parameter("floor_margin").as_double();
  params.min_inlier_ratio = this->get_parameter("min_inlier_ratio").as_double();
  params.max_plane_change = this->get_parameter("max_plane_change").as_double();
  params.max_min_z_change = this->get_parameter("max_min_z_change").as_double();
  params.use_previous_plane = this->get_parameter("use_previous_plane").as_bool();
  params.use_voxel_grid = this->get_parameter("use_voxel_grid").as_bool();
  params.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();

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

  RCLCPP_INFO(this->get_logger(), "Floor Removal Server Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output floor: %s", output_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output no-floor: %s", output_no_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Voxel grid: %s (leaf size: %.3f m)",
              params.use_voxel_grid ? "enabled" : "disabled",
              params.voxel_leaf_size);
}

void FloorRemovalServerNode::declareParameters()
{
  // Input/output topics
  this->declare_parameter<std::string>("input_cloud_topic", "/camera/depth/color/points");
  this->declare_parameter<std::string>("output_floor_cloud_topic", "/floor_cloud");
  this->declare_parameter<std::string>("output_no_floor_cloud_topic", "/no_floor_cloud");

  // TF frames
  this->declare_parameter<std::string>("camera_frame", "camera_link");
  this->declare_parameter<std::string>("base_frame", "camera_link");

  // RANSAC parameters
  this->declare_parameter<double>("ransac_distance_threshold", 0.02);
  this->declare_parameter<int>("ransac_max_iterations", 100);
  this->declare_parameter<double>("floor_normal_z_threshold", 0.15);

  // Floor region parameters
  this->declare_parameter<double>("floor_detection_thickness", 0.15);
  this->declare_parameter<double>("floor_removal_thickness", 0.03);
  this->declare_parameter<double>("floor_margin", 0.01);

  // Floor stability parameters
  this->declare_parameter<double>("min_inlier_ratio", 0.3);
  this->declare_parameter<double>("max_plane_change", 0.2);
  this->declare_parameter<double>("max_min_z_change", 0.05);
  this->declare_parameter<bool>("use_previous_plane", true);

  // Voxel grid parameters
  this->declare_parameter<bool>("use_voxel_grid", true);
  this->declare_parameter<double>("voxel_leaf_size", 0.005);

  // Legacy parameters (kept for compatibility)
  this->declare_parameter<double>("floor_height_min", -5.0);
  this->declare_parameter<double>("floor_height_max", -0.3);
}

void FloorRemovalServerNode::loadParameters()
{
  input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
  output_floor_cloud_topic_ = this->get_parameter("output_floor_cloud_topic").as_string();
  output_no_floor_cloud_topic_ = this->get_parameter("output_no_floor_cloud_topic").as_string();
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
                  "  Plane: normal=[%.2f, %.2f, %.2f], d=%.3f%s",
                  result.nx, result.ny, result.nz, result.d,
                  result.used_previous_plane ? " (using previous)" : "");
    } else {
      RCLCPP_WARN(this->get_logger(), "No floor plane detected");
    }
  }

  if (!result.plane_found) {
    return;
  }

  // Convert PCL clouds back to ROS messages
  sensor_msgs::msg::PointCloud2 floor_msg, no_floor_msg;
  pcl::toROSMsg(*result.floor_cloud, floor_msg);
  pcl::toROSMsg(*result.no_floor_cloud, no_floor_msg);

  // Set headers
  floor_msg.header = msg->header;
  no_floor_msg.header = msg->header;

  // Publish
  floor_cloud_pub_->publish(floor_msg);
  no_floor_cloud_pub_->publish(no_floor_msg);
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
