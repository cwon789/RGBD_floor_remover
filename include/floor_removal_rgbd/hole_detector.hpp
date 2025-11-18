#ifndef FLOOR_REMOVAL_RGBD_HOLE_DETECTOR_HPP
#define FLOOR_REMOVAL_RGBD_HOLE_DETECTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>

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

  // Visualization marker parameters
  double marker_height = 0.01;             // meters - thickness of hole plane marker
  double marker_z_offset = 0.005;          // meters - height above floor to display marker
};

/**
 * @brief Detected hole region information
 */
struct DetectedHole
{
  double center_x, center_y;               // Center position of hole
  double min_x, min_y, max_x, max_y;       // Bounding box of hole
  int num_cells;                           // Number of empty cells in this hole
  double area;                             // Approximate area in square meters

  DetectedHole()
    : center_x(0), center_y(0)
    , min_x(0), min_y(0), max_x(0), max_y(0)
    , num_cells(0), area(0)
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
 * pallet point cloud. It divides the XY plane into a grid, counts points
 * in each cell, and identifies clusters of empty cells as holes.
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
   * @param cloud Input point cloud (pallet candidates)
   * @param frame_id Frame ID for marker visualization
   * @return Detection result with detected holes and visualization markers
   */
  HoleDetectionResult detect(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
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
   * @brief Convert world coordinates to grid cell
   * @param wx, wy World coordinates
   * @return Grid cell
   */
  GridCell worldToGrid(double wx, double wy) const;

  /**
   * @brief Convert grid cell to world coordinates (cell center)
   * @param cell Grid cell
   * @param wx, wy Output world coordinates
   */
  void gridToWorld(const GridCell& cell, double& wx, double& wy) const;

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
