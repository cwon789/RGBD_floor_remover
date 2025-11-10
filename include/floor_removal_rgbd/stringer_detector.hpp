#ifndef FLOOR_REMOVAL_RGBD_STRINGER_DETECTOR_HPP
#define FLOOR_REMOVAL_RGBD_STRINGER_DETECTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <vector>

namespace floor_removal_rgbd
{

/**
 * @brief Detected line/column (연속된 복셀 그룹)
 */
struct DetectedColumn
{
  // Start and end points of the column
  double start_x, start_y, start_z;
  double end_x, end_y, end_z;

  // Center point (midpoint)
  double center_x, center_y, center_z;

  // Dimensions
  double length;  // Horizontal length (X-Y plane) or vertical length
  double height;  // Vertical height (Z direction)

  // Orientation
  bool is_horizontal;  // true if horizontal line (X or Y direction)
  bool is_vertical;    // true if vertical line (Z direction)

  // Number of points in this column
  size_t num_points;
};

/**
 * @brief Detected plane information
 */
struct DetectedPlane
{
  // Plane equation: nx*x + ny*y + nz*z + d = 0
  double nx, ny, nz, d;

  // Plane center (centroid of inlier points)
  double center_x, center_y, center_z;

  // Plane dimensions
  double width;   // horizontal extent
  double height;  // vertical extent

  // Orientation
  bool is_vertical;   // true if plane is vertical (wall-like)
  bool is_horizontal; // true if plane is horizontal (floor/ceiling-like)

  // Inlier points
  size_t num_inliers;
};

/**
 * @brief Bounding box for detected object (kept for backward compatibility)
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
 * @brief Parameters for stringer detection
 */
struct StringerDetectorParams
{
  // Stringer dimension constraints
  double width_min = 0.05;   // meters - minimum stringer width (horizontal)
  double width_max = 0.15;   // meters - maximum stringer width (horizontal)
  double height_min = 0.08;  // meters - minimum stringer height (vertical)
  double height_max = 0.20;  // meters - maximum stringer height (vertical)

  // Plane detection parameters
  double ransac_distance_threshold = 0.02;  // meters - RANSAC distance threshold
  int ransac_max_iterations = 100;          // RANSAC max iterations
  int min_plane_inliers = 10;               // minimum inliers for valid plane

  // Orientation thresholds (for determining vertical/horizontal)
  double vertical_normal_threshold = 0.8;   // |nx| or |ny| > this → vertical plane
  double horizontal_normal_threshold = 0.8; // |nz| > this → horizontal plane

  // Clustering parameters
  double cluster_tolerance = 0.08;  // meters - Euclidean distance tolerance for clustering
  int min_cluster_size = 10;        // minimum points per cluster
  int max_cluster_size = 10000;     // maximum points per cluster

  // Downsampling parameters
  bool use_additional_downsampling = true;  // Apply extra downsampling for large clouds
  size_t max_points_for_clustering = 5000;  // Trigger downsampling above this point count
  double downsampling_voxel_size = 0.05;    // meters - voxel size for clustering downsampling
};

/**
 * @brief Result of stringer detection operation
 */
struct StringerDetectionResult
{
  std::vector<BoundingBox> detected_stringers;  // For backward compatibility
  std::vector<DetectedPlane> detected_planes;   // Plane-based detection
  std::vector<DetectedColumn> detected_columns; // New: voxel connectivity-based detection
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stringer_centers;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersection_points;  // H-V line intersections

  StringerDetectionResult()
    : stringer_centers(new pcl::PointCloud<pcl::PointXYZRGB>)
    , intersection_points(new pcl::PointCloud<pcl::PointXYZRGB>)
  {}
};

/**
 * @brief Detector for pallet stringers in point clouds
 *
 * This class uses Euclidean clustering and dimensional analysis to detect
 * pallet stringers (wooden support beams). It's designed to work with
 * point clouds in robot frame (X=forward, Y=left, Z=up).
 */
class StringerDetector
{
public:
  /**
   * @brief Constructor
   * @param params Detector parameters
   */
  explicit StringerDetector(const StringerDetectorParams& params = StringerDetectorParams());

  /**
   * @brief Detect stringers in a point cloud
   * @param cloud Input point cloud (typically no-floor points)
   * @return Detection result with bounding boxes and center points
   */
  StringerDetectionResult detect(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Update detector parameters
   * @param params New parameters
   */
  void setParams(const StringerDetectorParams& params);

  /**
   * @brief Get current parameters
   * @return Current parameters
   */
  StringerDetectorParams getParams() const { return params_; }

private:
  /**
   * @brief Apply downsampling to large point clouds for efficient clustering
   * @param cloud Input cloud
   * @return Downsampled cloud (or original if downsampling not needed)
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampleForClustering(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Perform Euclidean clustering on point cloud
   * @param cloud Input cloud
   * @return Vector of cluster point indices
   */
  std::vector<pcl::PointIndices> clusterPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Compute bounding box and centroid for a cluster
   * @param cloud Input cloud
   * @param indices Point indices belonging to the cluster
   * @return Bounding box with centroid
   */
  BoundingBox computeBoundingBox(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::PointIndices& indices);

  /**
   * @brief Check if bounding box matches stringer dimension criteria
   * @param bbox Bounding box to check
   * @return True if dimensions match stringer criteria
   */
  bool matchesStringerCriteria(const BoundingBox& bbox);

  /**
   * @brief Detect planes in a cluster using RANSAC
   * @param cloud Input cloud
   * @param indices Point indices belonging to the cluster
   * @return Detected planes with vertical and horizontal orientation
   */
  std::vector<DetectedPlane> detectPlanesInCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::PointIndices& indices);

  /**
   * @brief Compute plane dimensions and centroid from inlier points
   * @param cloud Input cloud
   * @param indices Inlier point indices
   * @param plane Plane to update with dimensions and centroid
   */
  void computePlaneDimensions(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::vector<int>& indices,
    DetectedPlane& plane);

  /**
   * @brief Check if plane matches stringer criteria
   * @param plane Detected plane
   * @return True if plane matches vertical and horizontal dimension criteria
   */
  bool matchesStringerPlaneCriteria(const DetectedPlane& plane);

  /**
   * @brief Detect columns in a cluster based on voxel connectivity
   * @param cloud Input cloud
   * @param indices Point indices belonging to the cluster
   * @return Detected columns (continuous voxel groups)
   */
  std::vector<DetectedColumn> detectColumnsInCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::PointIndices& indices);

  /**
   * @brief Compute linear length of a point group
   * @param cloud Input cloud
   * @param indices Point indices
   * @return DetectedColumn with start, end, center, and length
   */
  DetectedColumn computeColumnLength(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::vector<int>& indices);

  /**
   * @brief Check if column length matches stringer criteria
   * @param column Detected column
   * @return True if length is within width_min ~ width_max
   */
  bool matchesStringerColumnCriteria(const DetectedColumn& column);

  /**
   * @brief Check if horizontal line matches width criteria
   * @param column Detected column
   * @return True if horizontal length is within width_min ~ width_max
   */
  bool matchesHorizontalCriteria(const DetectedColumn& column);

  /**
   * @brief Check if vertical line matches height criteria
   * @param column Detected column
   * @return True if vertical height is within height_min ~ height_max
   */
  bool matchesVerticalCriteria(const DetectedColumn& column);

  StringerDetectorParams params_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_STRINGER_DETECTOR_HPP
