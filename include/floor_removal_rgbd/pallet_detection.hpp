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
 * @brief Parameters for YZ plane (wall) detection
 */
struct PalletDetectionParams
{
  // RANSAC parameters for YZ plane detection
  double distance_threshold = 0.02;    // meters - points within this distance are inliers
  int max_iterations = 100;            // maximum RANSAC iterations
  double normal_x_threshold = 0.7;     // minimum X component of normal (perpendicular to X axis)

  // Clustering parameters
  double cluster_tolerance = 0.10;     // meters - distance between clusters
  int min_cluster_size = 50;           // minimum points in a cluster
  int max_cluster_size = 25000;        // maximum points in a cluster
  int min_plane_inliers = 30;          // minimum inliers to consider a valid plane

  // Visualization marker parameters
  double marker_thickness = 0.0;       // meters - thickness extension amount
  bool marker_bidirectional = false;   // true: extend equally in both directions
                                       // false: extend in one direction only
};

/**
 * @brief Detected YZ plane information
 */
struct DetectedPlane
{
  // Plane equation: nx*x + ny*y + nz*z + d = 0
  double nx, ny, nz, d;

  // Plane bounds (in plane coordinate system)
  float min_u, max_u;  // extent along plane X-axis
  float min_v, max_v;  // extent along plane Y-axis

  // Plane coordinate system
  float center_x, center_y, center_z;
  float axis_x[3], axis_y[3], normal[3];

  // Points belonging to this plane
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud;

  // Number of inliers
  size_t num_inliers;

  DetectedPlane()
    : nx(0), ny(0), nz(0), d(0)
    , min_u(0), max_u(0), min_v(0), max_v(0)
    , center_x(0), center_y(0), center_z(0)
    , plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    , num_inliers(0)
  {
    for (int i = 0; i < 3; ++i) {
      axis_x[i] = 0;
      axis_y[i] = 0;
      normal[i] = 0;
    }
  }
};

/**
 * @brief Result of pallet detection operation
 */
struct PalletDetectionResult
{
  std::vector<DetectedPlane> detected_planes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_candidates;
  visualization_msgs::msg::MarkerArray marker_array;

  PalletDetectionResult()
    : pallet_candidates(new pcl::PointCloud<pcl::PointXYZRGB>)
  {}
};

/**
 * @brief Detector for pallet walls using YZ plane detection
 *
 * This class detects vertical walls (YZ planes) in point clouds using
 * Euclidean clustering and RANSAC plane fitting. It's designed to work
 * with point clouds in robot frame (X=forward, Y=left, Z=up).
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
   * @brief Detect YZ planes (walls) in a point cloud
   * @param cloud Input point cloud (typically no-floor points in robot frame)
   * @param frame_id Frame ID for marker visualization
   * @return Detection result with detected planes and pallet candidates
   */
  PalletDetectionResult detect(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
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
   * @brief Perform Euclidean clustering on point cloud
   * @param cloud Input cloud
   * @return Vector of point indices for each cluster
   */
  std::vector<pcl::PointIndices> performClustering(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Detect YZ plane in a single cluster
   * @param cluster_cloud Cluster point cloud
   * @param plane Output detected plane (if successful)
   * @return True if valid plane detected
   */
  bool detectPlaneInCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster_cloud,
    DetectedPlane& plane);

  /**
   * @brief Calculate plane bounds and coordinate system
   * @param plane_cloud Points belonging to the plane
   * @param nx, ny, nz Plane normal components
   * @param d Plane distance
   * @param plane Output plane with calculated bounds
   */
  void calculatePlaneBounds(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_cloud,
    double nx, double ny, double nz, double d,
    DetectedPlane& plane);

  /**
   * @brief Extract pallet candidate points within plane thickness
   * @param cloud Full point cloud
   * @param plane Detected plane
   * @param candidates Output candidate points
   */
  void extractPalletCandidates(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const DetectedPlane& plane,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& candidates);

  /**
   * @brief Create visualization marker for a detected plane
   * @param plane Detected plane
   * @param marker_id Marker ID
   * @param frame_id Frame ID
   * @return Visualization marker
   */
  visualization_msgs::msg::Marker createPlaneMarker(
    const DetectedPlane& plane,
    int marker_id,
    const std::string& frame_id);

  PalletDetectionParams params_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_PALLET_DETECTION_HPP
