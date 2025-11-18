#ifndef FLOOR_REMOVAL_RGBD_HOLE_DETECTOR_HPP
#define FLOOR_REMOVAL_RGBD_HOLE_DETECTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>
#include "floor_removal_rgbd/pallet_detection.hpp"

namespace floor_removal_rgbd
{

/**
 * @brief Parameters for hole detection in pallet point clouds
 */
struct HoleDetectorParams
{
  // Grid-based detection parameters
  double grid_resolution = 0.05;           // meters - size of each grid cell (5cm x 5cm)
  int min_points_per_cell = 3;             // minimum points for a cell to be considered occupied
  int min_hole_cells = 4;                  // minimum cells for a hole region (about 10cm x 10cm)

  // Height range for hole detection (Z range from floor)
  double hole_z_min = 0.05;                // meters - minimum height from floor
  double hole_z_max = 0.30;                // meters - maximum height from floor

  // Distance from line to search for holes (perpendicular direction)
  double search_distance_from_line = 0.20; // meters - how far from line to look for holes

  // Visualization marker parameters
  double marker_thickness = 0.02;          // meters - thickness of hole plane marker (depth into pallet)
};

/**
 * @brief Detected hole region information
 */
struct DetectedHole
{
  double center_x, center_y, center_z;     // Center position of hole in 3D
  double width, height;                    // Width (along line) and height (vertical) of hole
  int num_cells;                           // Number of empty cells in this hole
  double area;                             // Approximate area in square meters
  double line_angle;                       // Angle of associated pallet line (for orientation)

  DetectedHole()
    : center_x(0), center_y(0), center_z(0)
    , width(0), height(0)
    , num_cells(0), area(0), line_angle(0)
  {}
};

/**
 * @brief Result of hole detection operation
 */
struct HoleDetectionResult
{
  std::vector<DetectedHole> detected_holes;
  visualization_msgs::msg::MarkerArray hole_markers;  // Plane markers for holes

  HoleDetectionResult() = default;
};

/**
 * @brief Detector for holes (empty spaces) in pallet point clouds
 *
 * This class uses a grid-based approach to detect empty regions in the
 * pallet vertical surfaces (YZ plane). For each detected pallet line, it
 * projects points onto a 2D plane perpendicular to the line direction,
 * then identifies clusters of empty cells as holes.
 */
class HoleDetector
{
public:
  /**
   * @brief Constructor
   * @param params Detection parameters
   */
  explicit HoleDetector(const HoleDetectorParams& params = HoleDetectorParams());

  /**
   * @brief Detect holes in pallet point cloud
   * @param pallet_candidates Input point cloud (pallet candidates - floor points inside cuboids)
   * @param line_candidates Input point cloud (line candidates - wall/line points)
   * @param detected_lines Detected pallet lines (walls)
   * @param frame_id Frame ID for marker visualization
   * @return Detection result with detected holes and visualization markers
   */
  HoleDetectionResult detect(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pallet_candidates,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& line_candidates,
    const std::vector<DetectedLine>& detected_lines,
    const std::string& frame_id);

  /**
   * @brief Update detector parameters
   * @param params New parameters
   */
  void setParams(const HoleDetectorParams& params);

  /**
   * @brief Get current parameters
   * @return Current parameters
   */
  HoleDetectorParams getParams() const { return params_; }

private:
  /**
   * @brief Grid cell identifier (using integer coordinates)
   */
  struct GridCell
  {
    int x, y;

    GridCell(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}

    bool operator<(const GridCell& other) const
    {
      if (x != other.x) return x < other.x;
      return y < other.y;
    }

    bool operator==(const GridCell& other) const
    {
      return x == other.x && y == other.y;
    }
  };

  /**
   * @brief Projected point in 2D local coordinate system
   */
  struct ProjectedPoint
  {
    double along_line;  // Coordinate along the pallet line (parallel to line)
    double height;      // Coordinate in vertical direction (Z)
    pcl::PointXYZRGB original_point;  // Original 3D point
  };

  /**
   * @brief Detect holes for a single pallet line
   * @param cloud Input point cloud
   * @param line Detected pallet line
   * @return Vector of detected holes for this line
   */
  std::vector<DetectedHole> detectHolesForLine(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const DetectedLine& line);

  /**
   * @brief Project points onto 2D plane perpendicular to line
   * @param cloud Input point cloud
   * @param line Pallet line
   * @return Vector of projected points in 2D local coordinates
   */
  std::vector<ProjectedPoint> projectPointsToLineFrame(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const DetectedLine& line);

  /**
   * @brief Convert 2D local coordinates to grid cell
   * @param along_line, height Local 2D coordinates
   * @return Grid cell
   */
  GridCell localToGrid(double along_line, double height) const;

  /**
   * @brief Convert grid cell to 2D local coordinates (cell center)
   * @param cell Grid cell
   * @param along_line, height Output local coordinates
   */
  void gridToLocal(const GridCell& cell, double& along_line, double& height) const;

  /**
   * @brief Find connected components of empty cells (hole regions)
   * @param empty_cells Set of empty grid cells
   * @return Vector of hole regions (each is a vector of cells)
   */
  std::vector<std::vector<GridCell>> findHoleRegions(
    const std::set<GridCell>& empty_cells);

  /**
   * @brief Create plane marker for a detected hole
   * @param hole Detected hole
   * @param marker_id Marker ID
   * @param frame_id Frame ID
   * @return Visualization marker (CUBE)
   */
  visualization_msgs::msg::Marker createHoleMarker(
    const DetectedHole& hole,
    int marker_id,
    const std::string& frame_id);

  /**
   * @brief Depth-first search for connected components
   * @param current Current cell
   * @param empty_cells Set of all empty cells
   * @param visited Set of visited cells
   * @param component Current component being built
   */
  void dfs(const GridCell& current,
           const std::set<GridCell>& empty_cells,
           std::set<GridCell>& visited,
           std::vector<GridCell>& component);

  HoleDetectorParams params_;
};

}  // namespace floor_removal_rgbd

#endif  // FLOOR_REMOVAL_RGBD_HOLE_DETECTOR_HPP
