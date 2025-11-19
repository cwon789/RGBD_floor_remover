#ifndef FLOOR_REMOVAL_RGBD_PLANE_REMOVER_HPP
#define FLOOR_REMOVAL_RGBD_PLANE_REMOVER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <cstddef>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace floor_removal_rgbd
{

/**
 * @brief Parameters for the floor plane removal algorithm
 */
struct PlaneRemoverParams
{
  // RANSAC parameters
  double ransac_distance_threshold = 0.02;  // meters - points within this distance are inliers
  int ransac_max_iterations = 100;          // maximum iterations for RANSAC

  // Floor detection region (camera optical frame: X=right, Y=down, Z=forward/depth)
  // ToF cameras typically capture floor in near depth range
  double floor_detection_min_depth = 0.3;   // meters - minimum depth for floor detection region
  double floor_detection_max_depth = 2.0;   // meters - maximum depth for floor detection region

  // Floor plane validation (camera optical frame)
  // Floor normal should point upward, which is negative Y in camera frame (Y=down)
  double floor_normal_y_threshold = 0.7;    // minimum abs(Y) component of normal (0.7 ≈ 45° tolerance)

  // Floor removal parameters
  double floor_removal_distance_threshold = 0.05;  // meters - distance from plane to consider as floor
  double floor_margin = 0.01;                      // meters - additional safety margin

  // Voxel grid parameters
  bool use_voxel_grid = true;               // enable voxel grid downsampling
  double voxel_leaf_size = 0.01;            // voxel size (meters) - applied to detection region

  // Noise removal parameters (post-processing after floor removal)
  bool enable_noise_removal = true;         // enable outlier removal for isolated floor remnants
  double noise_radius_search = 0.05;        // meters - radius to search for neighbors
  int noise_min_neighbors = 5;              // minimum neighbors within radius to keep point
  double noise_plane_distance_margin = 0.15;  // meters - only filter points near detected plane

  // Detection range parameters (camera optical frame)
  double max_detection_distance = 10.0;     // maximum detection distance from camera (meters)
  double min_points_for_plane = 50;         // minimum points in detection region for RANSAC
};

/**
 * @brief Result of plane removal operation
 */
struct PlaneRemovalResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_floor_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud_voxelized;
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_floor_cloud_voxelized;
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_floor_cloud_voxelized_2d_projected;  // 2D projection of no_floor_cloud_voxelized
  pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud;  // Removed noise points

  // Plane coefficients: nx*x + ny*y + nz*z + d = 0
  double nx, ny, nz, d;

  bool plane_found = false;

  size_t total_points = 0;
  size_t floor_points = 0;
  size_t voxelized_points = 0;
  size_t floor_region_points = 0;
  size_t noise_points = 0;


  PlaneRemovalResult()
    : floor_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    , no_floor_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    , floor_cloud_voxelized(new pcl::PointCloud<pcl::PointXYZ>)
    , no_floor_cloud_voxelized(new pcl::PointCloud<pcl::PointXYZ>)
    , no_floor_cloud_voxelized_2d_projected(new pcl::PointCloud<pcl::PointXYZ>)
    , noise_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    , nx(0), ny(0), nz(0), d(0)
  {}
};

/**
 * @brief Core algorithm for floor plane detection and removal
 *
 * This class handles the detection and removal of floor planes from point clouds.
 * It uses RANSAC for plane fitting and operates in the camera optical frame.
 * Extrinsic-independent approach using depth-based floor detection region.
 *
 * Coordinate frame: Camera optical (X=right, Y=down, Z=forward/depth)
 * Input: Camera optical frame point cloud
 * Processing: Camera optical frame (extrinsic-independent)
 * Output: Camera optical frame point cloud with floor removed
 */
class PlaneRemover
{
public:
  /**
   * @brief Constructor
   * @param params Algorithm parameters
   */
  explicit PlaneRemover(const PlaneRemoverParams& params = PlaneRemoverParams());

  /**
   * @brief Process a point cloud and remove floor plane
   * @param cloud_camera Input point cloud in camera optical frame (X=right, Y=down, Z=forward)
   * @return PlaneRemovalResult containing separated floor and non-floor clouds
   */
  PlaneRemovalResult process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera);

  /**
   * @brief Update algorithm parameters
   * @param params New parameters
   */
  void setParams(const PlaneRemoverParams& params);

  /**
   * @brief Get current parameters
   * @return Current parameters
   */
  PlaneRemoverParams getParams() const { return params_; }

  /**
   * @brief Reset temporal state (previous plane, min_z)
   */
  void reset();

private:
  /**
   * @brief Apply voxel grid downsampling
   * @param cloud Input cloud in camera optical frame
   * @return Downsampled cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyVoxelGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /**
   * @brief Filter points by maximum detection distance from camera
   * @param cloud Input cloud in camera optical frame
   * @return Filtered cloud with points within max_detection_distance
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /**
   * @brief Extract floor detection region based on depth (Z axis in camera frame)
   * @param cloud Input cloud in camera optical frame
   * @return Floor detection region point cloud (depth-filtered)
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr extractFloorDetectionRegion(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /**
   * @brief Detect floor plane using RANSAC on detection region
   * @param cloud Floor detection region point cloud (depth-filtered)
   * @param nx Output: plane normal X
   * @param ny Output: plane normal Y (should be negative for floor in camera frame)
   * @param nz Output: plane normal Z
   * @param d Output: plane distance
   * @param inlier_ratio Output: ratio of inliers
   * @return True if plane detected successfully
   */
  bool detectFloorPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double& nx, double& ny, double& nz, double& d, double& inlier_ratio);

  /**
   * @brief Validate detected plane (camera frame: floor normal should point up = -Y)
   * @param ny Plane normal Y component (should be negative with large magnitude)
   * @return True if plane is valid floor plane
   */
  bool isPlaneValid(double ny);

  /**
   * @brief Remove points on or near the detected plane (floor removal)
   * @param cloud Input cloud in camera optical frame
   * @param nx Plane normal X
   * @param ny Plane normal Y
   * @param nz Plane normal Z
   * @param d Plane distance
   * @param floor_cloud Output: floor points (on or near plane)
   * @param no_floor_cloud Output: non-floor points (away from plane)
   */
  void removePointsOnPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double nx, double ny, double nz, double d,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& floor_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& no_floor_cloud);

  /**
   * @brief Project point cloud to 2D plane
   * @param cloud Input cloud in camera optical frame
   * @param nx Plane normal X
   * @param ny Plane normal Y
   * @param nz Plane normal Z
   * @param d Plane distance
   * @return 2D projected cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr projectTo2D(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double nx, double ny, double nz, double d);

  /**
   * @brief Remove isolated noise points near detected floor plane
   * @param cloud Input cloud in camera optical frame
   * @param nx Plane normal X
   * @param ny Plane normal Y
   * @param nz Plane normal Z
   * @param d Plane distance
   * @param noise_cloud Output: removed noise points
   * @return Filtered cloud with noise removed
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeFloorNoise(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double nx, double ny, double nz, double d,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& noise_cloud);

  PlaneRemoverParams params_;

  // State tracking for debug logging
  bool prev_plane_found_ = false;  // Track previous plane detection state
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_PLANE_REMOVER_HPP
