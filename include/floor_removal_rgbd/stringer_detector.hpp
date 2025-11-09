#ifndef FLOOR_REMOVAL_RGBD_STRINGER_DETECTOR_HPP
#define FLOOR_REMOVAL_RGBD_STRINGER_DETECTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <vector>

namespace floor_removal_rgbd
{

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
 * @brief Parameters for stringer detection
 */
struct StringerDetectorParams
{
  // Stringer dimension constraints
  double width_min = 0.05;   // meters - minimum stringer width
  double width_max = 0.15;   // meters - maximum stringer width
  double height_min = 0.08;  // meters - minimum stringer height
  double height_max = 0.20;  // meters - maximum stringer height

  // Clustering parameters
  double cluster_tolerance = 0.08;  // meters - Euclidean distance tolerance for clustering
  int min_cluster_size = 5;         // minimum points per cluster
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
  std::vector<BoundingBox> detected_stringers;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stringer_centers;

  StringerDetectionResult()
    : stringer_centers(new pcl::PointCloud<pcl::PointXYZRGB>)
  {}
};

/**
 * @brief Detector for pallet stringers in point clouds
 *
 * This class uses Euclidean clustering and dimensional analysis to detect
 * pallet stringers (wooden support beams). It's designed to work with
 * point clouds in camera optical frame (X=right, Y=down, Z=forward).
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

  StringerDetectorParams params_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_STRINGER_DETECTOR_HPP
