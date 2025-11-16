#ifndef FLOOR_REMOVAL_RGBD_PALLET_DETECTOR_HPP
#define FLOOR_REMOVAL_RGBD_PALLET_DETECTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <cstddef>

namespace floor_removal_rgbd
{

/**
 * @brief Detected pallet information
 */
struct DetectedPallet
{
  // Position (center of pallet)
  double x, y, z;
  
  // Orientation (Euler angles in radians)
  double roll, pitch, yaw;
  
  // Confidence score (0.0 - 1.0)
  double confidence;
  
  // Size
  double length, width, height;
};

/**
 * @brief Parameters for pallet detection
 */
struct PalletDetectorParams
{
  // Template file path
  std::string template_file_path = "/tmp/pallet_template.yaml";
  
  // Matching threshold (0.0 - 1.0)
  double matching_threshold = 0.8;
  
  // Maximum distance for considering a match (meters)
  double max_matching_distance = 0.5;
};

/**
 * @brief Result of pallet detection operation
 */
struct PalletDetectionResult
{
  std::vector<DetectedPallet> detected_pallets;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_centers;
  
  PalletDetectionResult()
    : pallet_centers(new pcl::PointCloud<pcl::PointXYZRGB>)
  {}
};

/**
 * @brief Template structure for pallet matching
 */
struct PalletTemplate
{
  // Distances between key points (meters)
  std::vector<double> distances;
  
  // Tolerance for matching (mm)
  double tolerance_mm;
  
  // Base frame for template
  std::string base_frame;
};

/**
 * @brief Detector for pallets in point clouds using template matching
 *
 * This class uses template matching to detect pallets in 2D projected
 * point clouds. It's designed to work with point clouds in robot frame 
 * (X=forward, Y=left, Z=up).
 */
class PalletDetector
{
public:
  /**
   * @brief Constructor
   * @param params Detector parameters
   */
  explicit PalletDetector(const PalletDetectorParams& params = PalletDetectorParams());

  /**
   * @brief Detect pallets in a point cloud
   * @param cloud Input point cloud (typically no-floor points projected to 2D)
   * @return Detection result with pallet positions and orientations
   */
  PalletDetectionResult detect(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Update detector parameters
   * @param params New parameters
   */
  void setParams(const PalletDetectorParams& params);

  /**
   * @brief Get current parameters
   * @return Current parameters
   */
  PalletDetectorParams getParams() const { return params_; }

  /**
   * @brief Save template to file
   * @param template_data Template data to save
   * @param file_path Path to save template file
   * @return True if successful
   */
  bool saveTemplate(const PalletTemplate& template_data, const std::string& file_path);

  /**
   * @brief Load template from file
   * @param file_path Path to template file
   * @return Loaded template data
   */
  PalletTemplate loadTemplate(const std::string& file_path);

private:
  /**
   * @brief Perform template matching on point cloud
   * @param cloud Input cloud
   * @param templ Template to match
   * @return Vector of detected pallets
   */
  std::vector<DetectedPallet> matchTemplate(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const PalletTemplate& templ);

  /**
   * @brief Calculate distance between two points
   * @param p1 First point
   * @param p2 Second point
   * @return Distance between points
   */
  double calculateDistance(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) const;

  /**
   * @brief Check if distances match template within tolerance
   * @param measured Distances measured from point cloud
   * @param template_distances Distances from template
   * @param tolerance_mm Tolerance in millimeters
   * @return True if distances match within tolerance
   */
  bool distancesMatch(
    const std::vector<double>& measured,
    const std::vector<double>& template_distances,
    double tolerance_mm) const;

  /**
   * @brief Find the closest pallet to the robot
   * @param detected_pallets Vector of detected pallets
   * @return Index of closest pallet, or -1 if none
   */
  int findClosestPallet(const std::vector<DetectedPallet>& detected_pallets) const;

  PalletDetectorParams params_;
  PalletTemplate template_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_PALLET_DETECTOR_HPP
