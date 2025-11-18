#include "floor_removal_rgbd/hole_detector.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <set>

namespace floor_removal_rgbd
{

HoleDetector::HoleDetector(const HoleDetectorParams& params)
  : params_(params)
{
}

void HoleDetector::setParams(const HoleDetectorParams& params)
{
  params_ = params;
}

HoleDetectionResult HoleDetector::detect(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pallet_candidates,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& line_candidates,
  const std::vector<DetectedLine>& detected_lines,
  const std::string& frame_id)
{
  HoleDetectionResult result;

  if ((!pallet_candidates || pallet_candidates->points.empty()) &&
      (!line_candidates || line_candidates->points.empty())) {
    std::cout << "[HoleDetector] Both input clouds are empty" << std::endl;
    return result;
  }

  if (detected_lines.empty()) {
    std::cout << "[HoleDetector] No detected lines provided" << std::endl;
    return result;
  }

  // Merge pallet_candidates and line_candidates
  // This helps detect holes even when bottom surface is missing
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pallet_candidates && !pallet_candidates->points.empty()) {
    *merged_cloud += *pallet_candidates;
  }

  if (line_candidates && !line_candidates->points.empty()) {
    *merged_cloud += *line_candidates;
  }

  std::cout << "[HoleDetector] Merged cloud: pallet_candidates="
            << (pallet_candidates ? pallet_candidates->points.size() : 0)
            << " + line_candidates="
            << (line_candidates ? line_candidates->points.size() : 0)
            << " = " << merged_cloud->points.size() << " points" << std::endl;
  std::cout << "[HoleDetector] Processing " << detected_lines.size() << " pallet lines" << std::endl;

  // Detect holes for each pallet line
  int marker_id = 0;
  for (const auto& line : detected_lines) {
    std::cout << "[HoleDetector] Processing line at angle "
              << (line.angle * 180.0 / M_PI) << " degrees" << std::endl;

    auto holes_for_line = detectHolesForLine(merged_cloud, line);

    for (auto& hole : holes_for_line) {
      result.detected_holes.push_back(hole);

      // Create visualization marker
      auto marker = createHoleMarker(hole, marker_id++, frame_id);
      result.hole_markers.markers.push_back(marker);

      std::cout << "[HoleDetector] Hole " << result.detected_holes.size()
                << ": center=[" << hole.center_x << ", " << hole.center_y << ", " << hole.center_z
                << "], size=[" << hole.width << " x " << hole.height
                << "], cells=" << hole.num_cells
                << ", area=" << hole.area << " m²" << std::endl;
    }
  }

  std::cout << "[HoleDetector] Detected " << result.detected_holes.size()
            << " holes total" << std::endl;

  return result;
}

std::vector<DetectedHole> HoleDetector::detectHolesForLine(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const DetectedLine& line)
{
  std::vector<DetectedHole> holes;

  // Step 1: Project points to 2D local coordinate system (along line, height)
  auto projected_points = projectPointsToLineFrame(cloud, line);

  if (projected_points.empty()) {
    std::cout << "[HoleDetector]   No points near this line" << std::endl;
    return holes;
  }

  std::cout << "[HoleDetector]   Projected " << projected_points.size() << " points" << std::endl;

  // Step 2: Build occupancy grid in 2D local coordinates
  std::map<GridCell, int> occupancy_grid;
  double min_along = std::numeric_limits<double>::max();
  double max_along = std::numeric_limits<double>::lowest();
  double min_height = std::numeric_limits<double>::max();
  double max_height = std::numeric_limits<double>::lowest();

  for (const auto& pp : projected_points) {
    GridCell cell = localToGrid(pp.along_line, pp.height);
    occupancy_grid[cell]++;

    min_along = std::min(min_along, pp.along_line);
    max_along = std::max(max_along, pp.along_line);
    min_height = std::min(min_height, pp.height);
    max_height = std::max(max_height, pp.height);
  }

  std::cout << "[HoleDetector]   Grid bounds: along=[" << min_along << ", " << max_along
            << "], height=[" << min_height << ", " << max_height << "]" << std::endl;
  std::cout << "[HoleDetector]   Occupied cells: " << occupancy_grid.size() << std::endl;

  // Step 3: Find all cells within the bounding box
  GridCell min_cell = localToGrid(min_along, min_height);
  GridCell max_cell = localToGrid(max_along, max_height);

  // Step 4: Identify empty cells
  std::set<GridCell> empty_cells;
  int total_cells = 0;
  int occupied_cells = 0;

  for (int gx = min_cell.x; gx <= max_cell.x; ++gx) {
    for (int gy = min_cell.y; gy <= max_cell.y; ++gy) {
      total_cells++;
      GridCell cell(gx, gy);

      auto it = occupancy_grid.find(cell);
      int point_count = (it != occupancy_grid.end()) ? it->second : 0;

      if (point_count < params_.min_points_per_cell) {
        empty_cells.insert(cell);
      } else {
        occupied_cells++;
      }
    }
  }

  std::cout << "[HoleDetector]   Total cells: " << total_cells
            << ", occupied: " << occupied_cells
            << ", empty: " << empty_cells.size() << std::endl;

  // Step 5: Find connected components of empty cells
  auto hole_regions = findHoleRegions(empty_cells);

  std::cout << "[HoleDetector]   Found " << hole_regions.size() << " hole regions" << std::endl;

  // Step 6: Create DetectedHole objects and transform back to 3D
  for (const auto& region : hole_regions) {
    if (static_cast<int>(region.size()) < params_.min_hole_cells) {
      continue;
    }

    DetectedHole hole;
    hole.num_cells = region.size();
    hole.line_angle = line.angle;

    // Calculate bounding box in 2D local coordinates
    double min_local_along = std::numeric_limits<double>::max();
    double max_local_along = std::numeric_limits<double>::lowest();
    double min_local_height = std::numeric_limits<double>::max();
    double max_local_height = std::numeric_limits<double>::lowest();

    for (const auto& cell : region) {
      double along, height;
      gridToLocal(cell, along, height);

      min_local_along = std::min(min_local_along, along - params_.grid_resolution / 2.0);
      max_local_along = std::max(max_local_along, along + params_.grid_resolution / 2.0);
      min_local_height = std::min(min_local_height, height - params_.grid_resolution / 2.0);
      max_local_height = std::max(max_local_height, height + params_.grid_resolution / 2.0);
    }

    // Calculate center in local coordinates
    double center_along = (min_local_along + max_local_along) / 2.0;
    double center_height = (min_local_height + max_local_height) / 2.0;

    // Calculate dimensions
    hole.width = max_local_along - min_local_along;
    hole.height = max_local_height - min_local_height;
    hole.area = hole.width * hole.height;

    // Transform center back to 3D world coordinates
    // Line direction vector (normalized)
    double dx = line.end_x - line.start_x;
    double dy = line.end_y - line.start_y;
    double line_len = std::sqrt(dx*dx + dy*dy);
    double dir_x = dx / line_len;
    double dir_y = dy / line_len;

    // Perpendicular vector (pointing inward to pallet)
    double perp_x = -dir_y;
    double perp_y = dir_x;

    // Line center point
    double line_center_x = (line.start_x + line.end_x) / 2.0;
    double line_center_y = (line.start_y + line.end_y) / 2.0;

    // Calculate 3D position of hole center
    // Start from line center, move along line by center_along, up by center_height
    hole.center_x = line_center_x + dir_x * center_along;
    hole.center_y = line_center_y + dir_y * center_along;
    hole.center_z = center_height;

    holes.push_back(hole);
  }

  return holes;
}

std::vector<HoleDetector::ProjectedPoint> HoleDetector::projectPointsToLineFrame(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const DetectedLine& line)
{
  std::vector<ProjectedPoint> projected;

  // Line direction vector (normalized)
  double dx = line.end_x - line.start_x;
  double dy = line.end_y - line.start_y;
  double line_len = std::sqrt(dx*dx + dy*dy);

  if (line_len < 1e-6) {
    return projected;
  }

  double dir_x = dx / line_len;
  double dir_y = dy / line_len;

  // Perpendicular vector (pointing inward to pallet)
  double perp_x = -dir_y;
  double perp_y = dir_x;

  // Line center point (reference point)
  double line_center_x = (line.start_x + line.end_x) / 2.0;
  double line_center_y = (line.start_y + line.end_y) / 2.0;

  // Filter and project points
  for (const auto& pt : cloud->points) {
    // Check if point is within height range
    if (pt.z < params_.hole_z_min || pt.z > params_.hole_z_max) {
      continue;
    }

    // Vector from line center to point
    double to_point_x = pt.x - line_center_x;
    double to_point_y = pt.y - line_center_y;

    // Distance perpendicular to line
    double perp_dist = to_point_x * perp_x + to_point_y * perp_y;

    // Only consider points within search distance from line
    if (std::abs(perp_dist) > params_.search_distance_from_line) {
      continue;
    }

    // Project onto line direction (parallel component)
    double along_line = to_point_x * dir_x + to_point_y * dir_y;

    // Height is just Z coordinate
    double height = pt.z;

    ProjectedPoint pp;
    pp.along_line = along_line;
    pp.height = height;
    pp.original_point = pt;

    projected.push_back(pp);
  }

  return projected;
}

HoleDetector::GridCell HoleDetector::localToGrid(double along_line, double height) const
{
  int gx = static_cast<int>(std::floor(along_line / params_.grid_resolution));
  int gy = static_cast<int>(std::floor(height / params_.grid_resolution));
  return GridCell(gx, gy);
}

void HoleDetector::gridToLocal(const GridCell& cell, double& along_line, double& height) const
{
  // Return center of cell
  along_line = (cell.x + 0.5) * params_.grid_resolution;
  height = (cell.y + 0.5) * params_.grid_resolution;
}

std::vector<std::vector<HoleDetector::GridCell>> HoleDetector::findHoleRegions(
  const std::set<GridCell>& empty_cells)
{
  std::vector<std::vector<GridCell>> regions;
  std::set<GridCell> visited;

  for (const auto& cell : empty_cells) {
    if (visited.find(cell) != visited.end()) {
      continue;
    }

    // Start a new region with DFS
    std::vector<GridCell> component;
    dfs(cell, empty_cells, visited, component);

    if (!component.empty()) {
      regions.push_back(component);
    }
  }

  return regions;
}

void HoleDetector::dfs(const GridCell& current,
                        const std::set<GridCell>& empty_cells,
                        std::set<GridCell>& visited,
                        std::vector<GridCell>& component)
{
  // Check if already visited or not in empty cells
  if (visited.find(current) != visited.end() ||
      empty_cells.find(current) == empty_cells.end()) {
    return;
  }

  // Mark as visited and add to component
  visited.insert(current);
  component.push_back(current);

  // Check 4-connected neighbors (left, right, up, down)
  std::vector<GridCell> neighbors = {
    GridCell(current.x - 1, current.y),
    GridCell(current.x + 1, current.y),
    GridCell(current.x, current.y - 1),
    GridCell(current.x, current.y + 1)
  };

  for (const auto& neighbor : neighbors) {
    dfs(neighbor, empty_cells, visited, component);
  }
}

visualization_msgs::msg::Marker HoleDetector::createHoleMarker(
  const DetectedHole& hole,
  int marker_id,
  const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "pallet_holes";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position at center of hole
  marker.pose.position.x = hole.center_x;
  marker.pose.position.y = hole.center_y;
  marker.pose.position.z = hole.center_z;

  // Orientation based on line angle
  // The hole marker should be perpendicular to the pallet line
  // Rotate around Z axis by line_angle + 90 degrees
  double marker_yaw = hole.line_angle + M_PI / 2.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(marker_yaw / 2.0);
  marker.pose.orientation.w = std::cos(marker_yaw / 2.0);

  // Set scale (dimensions)
  marker.scale.x = params_.marker_thickness;  // Depth into pallet (perpendicular to wall)
  marker.scale.y = hole.width;                // Width along the pallet line
  marker.scale.z = hole.height;               // Height (vertical)

  // Set color (red for holes - indicating insertion points)
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.7;  // Semi-transparent

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  return marker;
}

}  // namespace floor_removal_rgbd
