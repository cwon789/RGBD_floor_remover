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
  double ransac_distance_threshold = 0.02;  // meters
  int ransac_max_iterations = 100;
  double floor_normal_z_threshold = 0.15;   // minimum Z component of normal

  // Floor height in robot frame (Z coordinate)
  double floor_height = 0.0;                // meters - floor height in robot frame (Z coordinate)

  // Floor region parameters
  double floor_detection_thickness = 0.15;  // meters - region for RANSAC plane detection
  double floor_removal_thickness = 0.03;    // meters - thickness for actual floor removal
  double floor_margin = 0.01;               // additional margin around detected plane

  // Voxel grid parameters
  bool use_voxel_grid = true;               // enable voxel grid downsampling
  double voxel_leaf_size = 0.005;           // voxel size (meters)

  // Detection range parameters
  double max_detection_distance = 10.0;     // maximum detection distance from camera (meters)
  double max_height = 3.0;                  // maximum height (Z coordinate) in robot frame (meters)

  // Camera extrinsic parameters (camera optical frame to robot base frame)
  // Translation: camera position in robot frame (meters)
  double cam_tx = 0.0;  // X offset (forward)
  double cam_ty = 0.0;  // Y offset (left)
  double cam_tz = 0.0;  // Z offset (up)

  // Rotation: Euler angles (radians) - applied in order: Roll(X) -> Pitch(Y) -> Yaw(Z)
  double cam_roll = 0.0;   // rotation around X axis (radians)
  double cam_pitch = 0.0;  // rotation around Y axis (radians)
  double cam_yaw = 0.0;    // rotation around Z axis (radians)

  // Use default optical frame to base frame transform (ignores above extrinsics)
  // Default: cam_optical (X=right, Y=down, Z=forward) -> base (X=forward, Y=left, Z=up)
  bool use_default_transform = true;
};

/**
 * @brief Result of plane removal operation
 */
struct PlaneRemovalResult
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_floor_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud_voxelized;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_floor_cloud_voxelized;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_floor_cloud_voxelized_2d_projected;  // 2D projection of no_floor_cloud_voxelized

  // Plane coefficients: nx*x + ny*y + nz*z + d = 0
  double nx, ny, nz, d;

  bool plane_found = false;

  size_t total_points = 0;
  size_t floor_points = 0;
  size_t voxelized_points = 0;
  size_t floor_region_points = 0;


  PlaneRemovalResult()
    : floor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    , no_floor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    , floor_cloud_voxelized(new pcl::PointCloud<pcl::PointXYZRGB>)
    , no_floor_cloud_voxelized(new pcl::PointCloud<pcl::PointXYZRGB>)
    , no_floor_cloud_voxelized_2d_projected(new pcl::PointCloud<pcl::PointXYZRGB>)
    , nx(0), ny(0), nz(0), d(0)
  {}
};

/**
 * @brief Core algorithm for floor plane detection and removal
 *
 * This class handles the detection and removal of floor planes from point clouds.
 * It uses RANSAC for plane fitting and operates in the robot coordinate frame.
 *
 * Input: Camera optical frame (X=right, Y=down, Z=forward)
 * Processing: Robot frame (X=forward, Y=left, Z=up)
 * Output: Robot frame (X=forward, Y=left, Z=up)
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
  PlaneRemovalResult process(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_camera);

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
   * @brief Transform point cloud from camera optical frame to standard robot frame
   * @param cloud_camera Input cloud in camera frame
   * @return Transformed cloud in standard frame
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformToStandardFrame(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_camera);

  /**
   * @brief Apply voxel grid downsampling
   * @param cloud Input cloud
   * @return Downsampled cloud
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyVoxelGrid(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Filter points by maximum detection distance
   * @param cloud Input cloud
   * @return Filtered cloud with points within max_detection_distance
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterByDistance(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Find minimum Z (robot frame: Z=up)
   * @param cloud Input cloud
   * @return Minimum Z value
   */
  double findMinZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Extract floor region points for RANSAC (points near minimum Z)
   * @param cloud Input cloud
   * @param min_z Minimum Z value
   * @return Floor region point cloud
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractFloorRegion(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, double min_z);

  /**
   * @brief Detect floor plane using RANSAC on voxelized cloud
   * @param cloud Voxelized point cloud
   * @param nx Output: plane normal X
   * @param ny Output: plane normal Y
   * @param nz Output: plane normal Z
   * @param d Output: plane distance
   * @param inlier_ratio Output: ratio of inliers
   * @return True if plane detected successfully
   */
  bool detectFloorPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    double& nx, double& ny, double& nz, double& d, double& inlier_ratio);

  /**
   * @brief Validate detected plane
   * @param nz Plane normal Z
   * @return True if plane is valid
   */
  bool isPlaneValid(double nz);

  /**
   * @brief Remove points below the detected plane (floor removal)
   * @param cloud Input cloud in robot frame
   * @param nx Plane normal X
   * @param ny Plane normal Y
   * @param nz Plane normal Z
   * @param d Plane distance
   * @param floor_cloud Output: floor points (at or below plane)
   * @param no_floor_cloud Output: non-floor points (above plane)
   */
  void removePointsBelowPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    double nx, double ny, double nz, double d,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& no_floor_cloud);

  /**
   * @brief Project point cloud to 2D plane (Z=0)
   * @param cloud Input cloud
   * @param nx Plane normal X
   * @param ny Plane normal Y
   * @param nz Plane normal Z
   * @param d Plane distance
   * @return 2D projected cloud
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectTo2D(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    double nx, double ny, double nz, double d);

  PlaneRemoverParams params_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_PLANE_REMOVER_HPP
