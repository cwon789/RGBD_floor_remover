#ifndef FLOOR_REMOVAL_RGBD_PLANE_REMOVER_HPP
#define FLOOR_REMOVAL_RGBD_PLANE_REMOVER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

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

  // Floor detection mode
  bool auto_floor_detection_mode = true;    // true: auto-detect using y_max, false: use camera height
  double camera_height = 0.80;              // meters - camera height from floor (camera frame Y)

  // Floor region parameters
  double floor_detection_thickness = 0.15;  // meters - region for RANSAC plane detection
  double floor_removal_thickness = 0.03;    // meters - thickness for actual floor removal
  double floor_margin = 0.01;               // additional margin around detected plane

  // Voxel grid parameters
  bool use_voxel_grid = true;               // enable voxel grid downsampling
  double voxel_leaf_size = 0.005;           // voxel size (meters)

  // Stringer detection parameters
  bool enable_stringer_detection = true;    // enable pallet stringer detection
  double stringer_width_min = 0.05;         // minimum stringer width (meters)
  double stringer_width_max = 0.15;         // maximum stringer width (meters)
  double stringer_height_min = 0.08;        // minimum stringer height (meters)
  double stringer_height_max = 0.20;        // maximum stringer height (meters)
};

/**
 * @brief Bounding box for detected object
 */
struct BoundingBox
{
  double min_x, max_x;
  double min_y, max_y;
  double min_z, max_z;

  // Centroid (average of actual points, not geometric center)
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  double centroid_z = 0.0;

  double width() const { return max_x - min_x; }
  double height() const { return max_y - min_y; }
  double depth() const { return max_z - min_z; }

  double center_x() const { return (min_x + max_x) / 2.0; }
  double center_y() const { return (min_y + max_y) / 2.0; }
  double center_z() const { return (min_z + max_z) / 2.0; }
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stringer_centers;

  // Plane coefficients: nx*x + ny*y + nz*z + d = 0
  double nx, ny, nz, d;

  bool plane_found = false;

  size_t total_points = 0;
  size_t floor_points = 0;
  size_t voxelized_points = 0;
  size_t floor_region_points = 0;

  // Stringer detection results
  std::vector<BoundingBox> detected_stringers;

  PlaneRemovalResult()
    : floor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    , no_floor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    , floor_cloud_voxelized(new pcl::PointCloud<pcl::PointXYZRGB>)
    , no_floor_cloud_voxelized(new pcl::PointCloud<pcl::PointXYZRGB>)
    , stringer_centers(new pcl::PointCloud<pcl::PointXYZRGB>)
    , nx(0), ny(0), nz(0), d(0)
  {}
};

/**
 * @brief Core algorithm for floor plane detection and removal
 *
 * This class handles the detection and removal of floor planes from point clouds.
 * It uses RANSAC for plane fitting and supports temporal stability filtering.
 *
 * Coordinate system: Standard robot frame (X=forward, Y=left, Z=up)
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
   * @brief Find maximum Y (camera frame: Y=down)
   * @param cloud Input cloud
   * @return Maximum Y value
   */
  double findMaxY(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Extract floor region points in camera frame (points near maximum Y)
   * @param cloud Input cloud
   * @param max_y Maximum Y value
   * @return Floor region point cloud
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractFloorRegionCameraFrame(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, double max_y);

  /**
   * @brief Detect floor plane using RANSAC
   * @param floor_region Floor region point cloud
   * @param nx Output: plane normal X
   * @param ny Output: plane normal Y
   * @param nz Output: plane normal Z
   * @param d Output: plane distance
   * @param inlier_ratio Output: ratio of inliers
   * @return True if plane detected successfully
   */
  bool detectFloorPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_region,
    double& nx, double& ny, double& nz, double& d, double& inlier_ratio);

  /**
   * @brief Validate detected plane
   * @param ny Plane normal Y
   * @return True if plane is valid
   */
  bool isPlaneValid(double ny);

  /**
   * @brief Classify points as floor or non-floor based on plane equation
   * @param cloud Input cloud in standard frame
   * @param nx Plane normal X
   * @param ny Plane normal Y
   * @param nz Plane normal Z
   * @param d Plane distance
   * @param floor_cloud Output: floor points
   * @param no_floor_cloud Output: non-floor points
   */
  void classifyPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    double nx, double ny, double nz, double d,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& no_floor_cloud);

  /**
   * @brief Detect pallet stringers in the no-floor cloud
   * @param cloud Input cloud (no-floor points)
   * @return Vector of detected stringer bounding boxes
   */
  std::vector<BoundingBox> detectStringers(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  PlaneRemoverParams params_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_PLANE_REMOVER_HPP
