#ifndef FLOOR_REMOVAL_RGBD_PALLET_DETECTION_HPP
#define FLOOR_REMOVAL_RGBD_PALLET_DETECTION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace floor_removal_rgbd
{

/**
 * @brief Parameters for line-based pallet detection
 */
struct PalletDetectionParams
{
  // Line extraction parameters
  double line_distance_threshold = 0.02;   // meters - distance threshold for line fitting
  int line_min_points = 20;                // minimum points to form a line
  int line_max_iterations = 100;           // maximum RANSAC iterations for line fitting
  double line_merge_angle_threshold = 5.0; // degrees - merge lines within this angle
  double line_merge_distance_threshold = 0.1; // meters - merge lines within this distance
  double line_min_length = 0.3;            // meters - minimum line length to keep

  // Visualization marker parameters
  double marker_thickness = 0.02;          // meters - line marker thickness
  double marker_height = 0.5;              // meters - line marker height above floor
};

/**
 * @brief Detected 2D line information
 */
struct DetectedLine
{
  // Line endpoints in 2D (on floor plane)
  double start_x, start_y;
  double end_x, end_y;

  // Line equation: ax + by + c = 0
  double a, b, c;

  // Line length
  double length;

  // Line angle (radians, relative to X-axis)
  double angle;

  // Number of inlier points
  size_t num_inliers;

  // Points belonging to this line
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_cloud;

  DetectedLine()
    : start_x(0), start_y(0), end_x(0), end_y(0)
    , a(0), b(0), c(0)
    , length(0), angle(0)
    , num_inliers(0)
    , line_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
  {}
};

/**
 * @brief Result of pallet detection operation
 */
struct PalletDetectionResult
{
  std::vector<DetectedLine> detected_lines;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_candidates;
  visualization_msgs::msg::MarkerArray line_markers;  // For /extracted_lines topic

  PalletDetectionResult()
    : pallet_candidates(new pcl::PointCloud<pcl::PointXYZRGB>)
  {}
};

/**
 * @brief Detector for pallet walls using 2D line extraction
 *
 * This class detects lines in 2D projected point clouds (floor plane)
 * using incremental RANSAC-based line fitting. It's designed to work
 * with 2D projected point clouds where Z=0 (on floor plane).
 */
class PalletDetection
{
public:
  /**
   * @brief Constructor
   * @param params Detection parameters
   */
  explicit PalletDetection(const PalletDetectionParams& params = PalletDetectionParams());

  /**
   * @brief Detect lines in 2D projected point cloud
   * @param cloud_2d Input 2D point cloud (Z should be 0 or near 0)
   * @param frame_id Frame ID for marker visualization
   * @return Detection result with detected lines and visualization markers
   */
  PalletDetectionResult detect(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2d,
    const std::string& frame_id);

  /**
   * @brief Update detector parameters
   * @param params New parameters
   */
  void setParams(const PalletDetectionParams& params);

  /**
   * @brief Get current parameters
   * @return Current parameters
   */
  PalletDetectionParams getParams() const { return params_; }

private:
  /**
   * @brief Extract lines from 2D point cloud using incremental RANSAC
   * @param cloud_2d Input 2D cloud
   * @return Vector of detected lines
   */
  std::vector<DetectedLine> extractLines(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2d);

  /**
   * @brief Fit a line to a set of 2D points using RANSAC
   * @param cloud_2d Input cloud
   * @param indices Indices of points to consider
   * @param line Output detected line
   * @return True if line detected successfully
   */
  bool fitLineRANSAC(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2d,
    const std::vector<int>& indices,
    DetectedLine& line);

  /**
   * @brief Merge similar lines (nearby and parallel)
   * @param lines Input lines
   * @return Merged lines
   */
  std::vector<DetectedLine> mergeLines(
    const std::vector<DetectedLine>& lines);

  /**
   * @brief Check if two lines should be merged
   * @param line1 First line
   * @param line2 Second line
   * @return True if lines should be merged
   */
  bool shouldMergeLines(
    const DetectedLine& line1,
    const DetectedLine& line2);

  /**
   * @brief Create visualization marker for a detected line
   * @param line Detected line
   * @param marker_id Marker ID
   * @param frame_id Frame ID
   * @return Visualization marker
   */
  visualization_msgs::msg::Marker createLineMarker(
    const DetectedLine& line,
    int marker_id,
    const std::string& frame_id);

  /**
   * @brief Calculate distance from point to line
   * @param px, py Point coordinates
   * @param a, b, c Line equation coefficients
   * @return Distance from point to line
   */
  double pointToLineDistance(double px, double py, double a, double b, double c);

  PalletDetectionParams params_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_PALLET_DETECTION_HPP
