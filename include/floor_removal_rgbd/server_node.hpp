#ifndef FLOOR_REMOVAL_RGBD_SERVER_NODE_HPP
#define FLOOR_REMOVAL_RGBD_SERVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "floor_removal_rgbd/plane_remover.hpp"
#include "floor_removal_rgbd/pallet_detection.hpp"
#include <memory>

namespace floor_removal_rgbd
{

/**
 * @brief ROS2 node for floor plane removal service
 *
 * This node subscribes to point cloud topics, processes them using PlaneRemover,
 * and publishes separated floor and non-floor point clouds.
 */
class FloorRemovalServerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  FloorRemovalServerNode();

  /**
   * @brief Destructor
   */
  ~FloorRemovalServerNode() = default;

private:
  /**
   * @brief Callback for point cloud messages
   * @param msg Point cloud message
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Load parameters from ROS parameter server
   */
  void loadParameters();

  /**
   * @brief Declare all ROS parameters with default values
   */
  void declareParameters();

  // ROS publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr floor_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_floor_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr floor_cloud_voxelized_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_floor_cloud_voxelized_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_floor_cloud_voxelized_2d_projected_pub_;  // 2D projected publisher

  // Pallet detection publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr extracted_lines_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr line_candidates_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pallet_candidates_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pallet_cuboid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pose_pub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Core algorithms
  std::unique_ptr<PlaneRemover> plane_remover_;
  std::unique_ptr<PalletDetection> pallet_detection_;

  // Pallet detection enable flag
  bool enable_pallet_detection_;
  bool loosely_coupled_;

  // Topic names
  std::string input_cloud_topic_;
  std::string output_floor_cloud_topic_;
  std::string output_no_floor_cloud_topic_;
  std::string output_floor_cloud_voxelized_topic_;
  std::string output_no_floor_cloud_voxelized_topic_;

  // Frame IDs
  std::string camera_frame_;
  std::string base_frame_;

  // Debug counter for periodic logging
  int debug_counter_ = 0;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_SERVER_NODE_HPP
