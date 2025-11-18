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
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const std::string& frame_id)
{
  HoleDetectionResult result;

  if (!cloud || cloud->points.empty()) {
    std::cout << "[HoleDetector] Input cloud is empty" << std::endl;
    return result;
  }

  std::cout << "[HoleDetector] Processing cloud with " << cloud->points.size() << " points" << std::endl;

  // Step 1: Build occupancy grid
  // Map from grid cell to point count
  std::map<GridCell, int> occupancy_grid;

  // Track grid bounds
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& pt : cloud->points) {
    GridCell cell = worldToGrid(pt.x, pt.y);
    occupancy_grid[cell]++;

    // Track bounds
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  std::cout << "[HoleDetector] Grid bounds: X=[" << min_x << ", " << max_x
            << "], Y=[" << min_y << ", " << max_y << "]" << std::endl;
  std::cout << "[HoleDetector] Occupied cells: " << occupancy_grid.size() << std::endl;

  // Step 2: Find all cells within the bounding box
  GridCell min_cell = worldToGrid(min_x, min_y);
  GridCell max_cell = worldToGrid(max_x, max_y);

  // Step 3: Identify empty cells (cells with insufficient points)
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

  std::cout << "[HoleDetector] Total cells in bounding box: " << total_cells << std::endl;
  std::cout << "[HoleDetector] Occupied cells: " << occupied_cells << std::endl;
  std::cout << "[HoleDetector] Empty cells: " << empty_cells.size() << std::endl;

  // Step 4: Find connected components of empty cells (hole regions)
  auto hole_regions = findHoleRegions(empty_cells);

  std::cout << "[HoleDetector] Found " << hole_regions.size() << " hole regions" << std::endl;

  // Step 5: Create DetectedHole objects for regions meeting minimum size
  int marker_id = 0;
  for (const auto& region : hole_regions) {
    if (static_cast<int>(region.size()) < params_.min_hole_cells) {
      continue;  // Skip small regions
    }

    DetectedHole hole;
    hole.num_cells = region.size();

    // Calculate bounding box
    hole.min_x = std::numeric_limits<double>::max();
    hole.max_x = std::numeric_limits<double>::lowest();
    hole.min_y = std::numeric_limits<double>::max();
    hole.max_y = std::numeric_limits<double>::lowest();

    for (const auto& cell : region) {
      double wx, wy;
      gridToWorld(cell, wx, wy);

      hole.min_x = std::min(hole.min_x, wx - params_.grid_resolution / 2.0);
      hole.max_x = std::max(hole.max_x, wx + params_.grid_resolution / 2.0);
      hole.min_y = std::min(hole.min_y, wy - params_.grid_resolution / 2.0);
      hole.max_y = std::max(hole.max_y, wy + params_.grid_resolution / 2.0);
    }

    // Calculate center and area
    hole.center_x = (hole.min_x + hole.max_x) / 2.0;
    hole.center_y = (hole.min_y + hole.max_y) / 2.0;
    hole.area = (hole.max_x - hole.min_x) * (hole.max_y - hole.min_y);

    result.detected_holes.push_back(hole);

    // Create visualization marker
    auto marker = createHoleMarker(hole, marker_id++, frame_id);
    result.hole_markers.markers.push_back(marker);

    std::cout << "[HoleDetector] Hole " << result.detected_holes.size()
              << ": center=[" << hole.center_x << ", " << hole.center_y
              << "], size=[" << (hole.max_x - hole.min_x) << " x " << (hole.max_y - hole.min_y)
              << "], cells=" << hole.num_cells
              << ", area=" << hole.area << " m²" << std::endl;
  }

  std::cout << "[HoleDetector] Detected " << result.detected_holes.size()
            << " holes (after filtering)" << std::endl;

  return result;
}

HoleDetector::GridCell HoleDetector::worldToGrid(double wx, double wy) const
{
  int gx = static_cast<int>(std::floor(wx / params_.grid_resolution));
  int gy = static_cast<int>(std::floor(wy / params_.grid_resolution));
  return GridCell(gx, gy);
}

void HoleDetector::gridToWorld(const GridCell& cell, double& wx, double& wy) const
{
  // Return center of cell
  wx = (cell.x + 0.5) * params_.grid_resolution;
  wy = (cell.y + 0.5) * params_.grid_resolution;
}

std::vector<std::vector<HoleDetector::GridCell>> HoleDetector::findHoleRegions(
  const std::set<GridCell>& empty_cells)
{
  std::vector<std::vector<GridCell>> regions;
  std::set<GridCell> visited;

  for (const auto& cell : empty_cells) {
    if (visited.find(cell) != visited.end()) {
      continue;  // Already visited
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

  // Check 4-connected neighbors (up, down, left, right)
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

  // Position at center of hole, slightly above floor
  marker.pose.position.x = hole.center_x;
  marker.pose.position.y = hole.center_y;
  marker.pose.position.z = params_.marker_z_offset + params_.marker_height / 2.0;

  // No rotation (aligned with XY plane)
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set scale (dimensions of the plane)
  marker.scale.x = hole.max_x - hole.min_x;
  marker.scale.y = hole.max_y - hole.min_y;
  marker.scale.z = params_.marker_height;

  // Set color (red for holes - indicating danger/warning)
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.7;  // Semi-transparent

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  return marker;
}

}  // namespace floor_removal_rgbd
