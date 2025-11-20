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
  params.floor_detection_min_depth = this->get_parameter("floor_detection_min_depth").as_double();
  params.floor_detection_max_depth = this->get_parameter("floor_detection_max_depth").as_double();
  params.floor_normal_y_threshold = this->get_parameter("floor_normal_y_threshold").as_double();
  params.floor_removal_distance_threshold = this->get_parameter("floor_removal_distance_threshold").as_double();
  params.floor_margin = this->get_parameter("floor_margin").as_double();
  params.use_voxel_grid = this->get_parameter("use_voxel_grid").as_bool();
  params.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
  params.enable_noise_removal = this->get_parameter("enable_noise_removal").as_bool();
  params.noise_radius_search = this->get_parameter("noise_radius_search").as_double();
  params.noise_min_neighbors = this->get_parameter("noise_min_neighbors").as_int();
  params.noise_plane_distance_margin = this->get_parameter("noise_plane_distance_margin").as_double();
  params.max_detection_distance = this->get_parameter("max_detection_distance").as_double();
  params.min_points_for_plane = this->get_parameter("min_points_for_plane").as_int();
  params.enable_plane_smoothing = this->get_parameter("enable_plane_smoothing").as_bool();
  params.plane_smoothing_alpha = this->get_parameter("plane_smoothing_alpha").as_double();

  plane_remover_ = std::make_unique<PlaneRemover>(params);

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

  noise_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_noise_cloud_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Floor Removal Server Node initialized (Extrinsic-independent)");
  RCLCPP_INFO(this->get_logger(), "  Input: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output floor: %s", output_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output no-floor: %s", output_no_floor_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output floor (voxelized): %s", output_floor_cloud_voxelized_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output no-floor (voxelized): %s", output_no_floor_cloud_voxelized_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output noise: %s", output_noise_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Floor detection depth range: [%.2f, %.2f] m",
              params.floor_detection_min_depth, params.floor_detection_max_depth);
  RCLCPP_INFO(this->get_logger(), "  Floor normal Y threshold: %.2f (camera frame)",
              params.floor_normal_y_threshold);
  RCLCPP_INFO(this->get_logger(), "  Floor removal distance threshold: %.3f m",
              params.floor_removal_distance_threshold);
  RCLCPP_INFO(this->get_logger(), "  Max detection distance: %.2f m", params.max_detection_distance);
  RCLCPP_INFO(this->get_logger(), "  Voxel grid: %s (leaf size: %.3f m)",
              params.use_voxel_grid ? "enabled" : "disabled",
              params.voxel_leaf_size);
  RCLCPP_INFO(this->get_logger(), "  Noise removal: %s", params.enable_noise_removal ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "  Plane smoothing: %s (alpha: %.2f)",
              params.enable_plane_smoothing ? "enabled" : "disabled",
              params.plane_smoothing_alpha);
}

void FloorRemovalServerNode::declareParameters()
{
  // Input/output topics
  this->declare_parameter<std::string>("input_cloud_topic", "/camera/depth/color/points");
  this->declare_parameter<std::string>("output_floor_cloud_topic", "/floor_cloud");
  this->declare_parameter<std::string>("output_no_floor_cloud_topic", "/no_floor_cloud");
  this->declare_parameter<std::string>("output_floor_cloud_voxelized_topic", "/floor_cloud_voxelized");
  this->declare_parameter<std::string>("output_no_floor_cloud_voxelized_topic", "/no_floor_cloud_voxelized");
  this->declare_parameter<std::string>("output_noise_cloud_topic", "/noise_cloud");

  // RANSAC parameters
  this->declare_parameter<double>("ransac_distance_threshold", 0.02);
  this->declare_parameter<int>("ransac_max_iterations", 100);

  // Floor detection region (depth-based)
  this->declare_parameter<double>("floor_detection_min_depth", 0.3);
  this->declare_parameter<double>("floor_detection_max_depth", 2.0);
  this->declare_parameter<int>("min_points_for_plane", 50);

  // Floor plane validation
  this->declare_parameter<double>("floor_normal_y_threshold", 0.7);

  // Floor removal parameters
  this->declare_parameter<double>("floor_removal_distance_threshold", 0.05);
  this->declare_parameter<double>("floor_margin", 0.01);

  // Voxel grid parameters
  this->declare_parameter<bool>("use_voxel_grid", true);
  this->declare_parameter<double>("voxel_leaf_size", 0.01);

  // Noise removal parameters
  this->declare_parameter<bool>("enable_noise_removal", true);
  this->declare_parameter<double>("noise_radius_search", 0.05);
  this->declare_parameter<int>("noise_min_neighbors", 5);
  this->declare_parameter<double>("noise_plane_distance_margin", 0.15);

  // Detection range parameters
  this->declare_parameter<double>("max_detection_distance", 10.0);

  // Temporal smoothing parameters
  this->declare_parameter<bool>("enable_plane_smoothing", true);
  this->declare_parameter<double>("plane_smoothing_alpha", 0.3);
}

void FloorRemovalServerNode::loadParameters()
{
  input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
  output_floor_cloud_topic_ = this->get_parameter("output_floor_cloud_topic").as_string();
  output_no_floor_cloud_topic_ = this->get_parameter("output_no_floor_cloud_topic").as_string();
  output_floor_cloud_voxelized_topic_ = this->get_parameter("output_floor_cloud_voxelized_topic").as_string();
  output_no_floor_cloud_voxelized_topic_ = this->get_parameter("output_no_floor_cloud_voxelized_topic").as_string();
  output_noise_cloud_topic_ = this->get_parameter("output_noise_cloud_topic").as_string();
}

void FloorRemovalServerNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
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
  sensor_msgs::msg::PointCloud2 noise_msg;

  pcl::toROSMsg(*result.floor_cloud, floor_msg);
  pcl::toROSMsg(*result.no_floor_cloud, no_floor_msg);
  pcl::toROSMsg(*result.floor_cloud_voxelized, floor_voxelized_msg);
  pcl::toROSMsg(*result.no_floor_cloud_voxelized, no_floor_voxelized_msg);
  pcl::toROSMsg(*result.noise_cloud, noise_msg);

  // Set headers
  floor_msg.header = msg->header;
  no_floor_msg.header = msg->header;
  floor_voxelized_msg.header = msg->header;
  no_floor_voxelized_msg.header = msg->header;
  noise_msg.header = msg->header;

  // Publish (even if plane not found, transformed clouds will be published)
  floor_cloud_pub_->publish(floor_msg);
  no_floor_cloud_pub_->publish(no_floor_msg);
  floor_cloud_voxelized_pub_->publish(floor_voxelized_msg);
  no_floor_cloud_voxelized_pub_->publish(no_floor_voxelized_msg);
  noise_cloud_pub_->publish(noise_msg);
}

}  // namespace floor_removal_rgbd

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<floor_removal_rgbd::FloorRemovalServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
