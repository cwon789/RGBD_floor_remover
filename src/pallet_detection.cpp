#include "floor_removal_rgbd/pallet_detection.hpp"

#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>
#include <random>

namespace floor_removal_rgbd
{

PalletDetection::PalletDetection(const PalletDetectionParams& params)
  : params_(params)
{
}

void PalletDetection::setParams(const PalletDetectionParams& params)
{
  params_ = params;
}

PalletDetectionResult PalletDetection::detect(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2d,
  const std::string& frame_id)
{
  PalletDetectionResult result;

  if (!cloud_2d || cloud_2d->points.empty()) {
    std::cout << "[PalletDetection] Input cloud is empty" << std::endl;
    return result;
  }

  std::cout << "[PalletDetection] Processing 2D cloud with " << cloud_2d->points.size() << " points" << std::endl;

  // Extract lines from 2D point cloud
  result.detected_lines = extractLines(cloud_2d);

  // Merge similar lines
  result.detected_lines = mergeLines(result.detected_lines);

  // Create visualization markers
  for (size_t i = 0; i < result.detected_lines.size(); ++i) {
    auto marker = createLineMarker(result.detected_lines[i], i, frame_id);
    result.line_markers.markers.push_back(marker);
  }

  // Collect all line points as pallet candidates
  for (const auto& line : result.detected_lines) {
    for (const auto& pt : line.line_cloud->points) {
      result.pallet_candidates->points.push_back(pt);
    }
  }

  if (!result.pallet_candidates->points.empty()) {
    result.pallet_candidates->width = result.pallet_candidates->points.size();
    result.pallet_candidates->height = 1;
    result.pallet_candidates->is_dense = false;
  }

  std::cout << "[PalletDetection] Detected " << result.detected_lines.size() << " lines" << std::endl;
  std::cout << "[PalletDetection] Total pallet candidates: " << result.pallet_candidates->points.size() << " points" << std::endl;

  return result;
}

std::vector<DetectedLine> PalletDetection::extractLines(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2d)
{
  std::vector<DetectedLine> lines;

  // Copy indices of all points
  std::vector<int> remaining_indices;
  for (size_t i = 0; i < cloud_2d->points.size(); ++i) {
    remaining_indices.push_back(i);
  }

  // Iteratively extract lines using RANSAC
  while (remaining_indices.size() >= static_cast<size_t>(params_.line_min_points)) {
    DetectedLine line;

    if (fitLineRANSAC(cloud_2d, remaining_indices, line)) {
      // Check minimum length
      if (line.length >= params_.line_min_length) {
        lines.push_back(line);

        std::cout << "[PalletDetection] Line " << lines.size()
                  << ": length=" << line.length << "m, angle=" << (line.angle * 180.0 / M_PI)
                  << "°, inliers=" << line.num_inliers << std::endl;
      }

      // Remove inlier points from remaining indices
      std::vector<int> new_remaining;
      for (int idx : remaining_indices) {
        const auto& pt = cloud_2d->points[idx];
        double dist = pointToLineDistance(pt.x, pt.y, line.a, line.b, line.c);

        if (dist > params_.line_distance_threshold) {
          new_remaining.push_back(idx);
        }
      }
      remaining_indices = new_remaining;
    } else {
      // Failed to find more lines
      break;
    }
  }

  return lines;
}

bool PalletDetection::fitLineRANSAC(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2d,
  const std::vector<int>& indices,
  DetectedLine& line)
{
  if (indices.size() < 2) {
    return false;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, indices.size() - 1);

  int best_inliers = 0;
  std::vector<int> best_inlier_indices;

  // RANSAC iterations
  for (int iter = 0; iter < params_.line_max_iterations; ++iter) {
    // Sample two random points
    int idx1 = dis(gen);
    int idx2 = dis(gen);
    if (idx1 == idx2) continue;

    const auto& p1 = cloud_2d->points[indices[idx1]];
    const auto& p2 = cloud_2d->points[indices[idx2]];

    // Compute line equation: ax + by + c = 0
    // From two points (x1,y1) and (x2,y2):
    // (y2-y1)x - (x2-x1)y + (x2-x1)y1 - (y2-y1)x1 = 0
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
      continue;  // Same point
    }

    double a = dy;
    double b = -dx;
    double c = dx * p1.y - dy * p1.x;

    // Normalize
    double norm = std::sqrt(a*a + b*b);
    a /= norm;
    b /= norm;
    c /= norm;

    // Count inliers
    std::vector<int> inlier_indices;
    for (int idx : indices) {
      const auto& pt = cloud_2d->points[idx];
      double dist = std::abs(a * pt.x + b * pt.y + c);

      if (dist <= params_.line_distance_threshold) {
        inlier_indices.push_back(idx);
      }
    }

    // Update best model
    if (static_cast<int>(inlier_indices.size()) > best_inliers) {
      best_inliers = inlier_indices.size();
      best_inlier_indices = inlier_indices;
    }
  }

  // Check if we found enough inliers
  if (best_inliers < params_.line_min_points) {
    return false;
  }

  // Refine line parameters using all inliers (least squares)
  double sum_x = 0, sum_y = 0;
  double sum_xx = 0, sum_xy = 0, sum_yy = 0;

  for (int idx : best_inlier_indices) {
    const auto& pt = cloud_2d->points[idx];
    sum_x += pt.x;
    sum_y += pt.y;
    sum_xx += pt.x * pt.x;
    sum_xy += pt.x * pt.y;
    sum_yy += pt.y * pt.y;
  }

  int n = best_inlier_indices.size();
  double mean_x = sum_x / n;
  double mean_y = sum_y / n;

  // Compute covariance matrix
  double cov_xx = sum_xx / n - mean_x * mean_x;
  double cov_xy = sum_xy / n - mean_x * mean_y;
  double cov_yy = sum_yy / n - mean_y * mean_y;

  // Compute principal direction (eigenvector of largest eigenvalue)
  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double lambda = trace / 2.0 + std::sqrt(trace * trace / 4.0 - det);

  double vx, vy;
  if (std::abs(cov_xy) > 1e-6) {
    vx = lambda - cov_yy;
    vy = cov_xy;
  } else if (std::abs(cov_xx - lambda) > 1e-6) {
    vx = 1.0;
    vy = 0.0;
  } else {
    vx = 0.0;
    vy = 1.0;
  }

  // Normalize direction vector
  double norm = std::sqrt(vx*vx + vy*vy);
  vx /= norm;
  vy /= norm;

  // Line equation: perpendicular to direction
  line.a = -vy;
  line.b = vx;
  line.c = vy * mean_x - vx * mean_y;

  // Normalize
  norm = std::sqrt(line.a*line.a + line.b*line.b);
  line.a /= norm;
  line.b /= norm;
  line.c /= norm;

  // Find endpoints (min/max projection on line direction)
  double min_proj = std::numeric_limits<double>::max();
  double max_proj = std::numeric_limits<double>::lowest();

  for (int idx : best_inlier_indices) {
    const auto& pt = cloud_2d->points[idx];
    double proj = vx * pt.x + vy * pt.y;
    min_proj = std::min(min_proj, proj);
    max_proj = std::max(max_proj, proj);
  }

  // Calculate endpoints
  double center_proj = (min_proj + max_proj) / 2.0;
  line.start_x = mean_x + vx * (min_proj - center_proj);
  line.start_y = mean_y + vy * (min_proj - center_proj);
  line.end_x = mean_x + vx * (max_proj - center_proj);
  line.end_y = mean_y + vy * (max_proj - center_proj);

  // Calculate line length
  double dx = line.end_x - line.start_x;
  double dy = line.end_y - line.start_y;
  line.length = std::sqrt(dx*dx + dy*dy);

  // Calculate angle (relative to X-axis)
  line.angle = std::atan2(vy, vx);

  // Store inlier count
  line.num_inliers = best_inlier_indices.size();

  // Store inlier points
  for (int idx : best_inlier_indices) {
    line.line_cloud->points.push_back(cloud_2d->points[idx]);
  }
  line.line_cloud->width = line.line_cloud->points.size();
  line.line_cloud->height = 1;
  line.line_cloud->is_dense = false;

  return true;
}

std::vector<DetectedLine> PalletDetection::mergeLines(
  const std::vector<DetectedLine>& lines)
{
  if (lines.empty()) {
    return lines;
  }

  std::vector<DetectedLine> merged_lines;
  std::vector<bool> merged(lines.size(), false);

  for (size_t i = 0; i < lines.size(); ++i) {
    if (merged[i]) continue;

    DetectedLine merged_line = lines[i];
    std::vector<size_t> merge_group = {i};

    // Find all lines that should merge with this one
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (merged[j]) continue;

      if (shouldMergeLines(merged_line, lines[j])) {
        merge_group.push_back(j);
        merged[j] = true;
      }
    }

    // If multiple lines were merged, recompute the merged line
    if (merge_group.size() > 1) {
      // Combine all points from merged lines
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (size_t idx : merge_group) {
        for (const auto& pt : lines[idx].line_cloud->points) {
          combined_cloud->points.push_back(pt);
        }
      }

      // Create indices vector
      std::vector<int> all_indices(combined_cloud->points.size());
      std::iota(all_indices.begin(), all_indices.end(), 0);

      // Refit line to combined points
      if (fitLineRANSAC(combined_cloud, all_indices, merged_line)) {
        merged_lines.push_back(merged_line);
      }
    } else {
      merged_lines.push_back(merged_line);
    }

    merged[i] = true;
  }

  std::cout << "[PalletDetection] Merged " << lines.size() << " lines into " << merged_lines.size() << " lines" << std::endl;

  return merged_lines;
}

bool PalletDetection::shouldMergeLines(
  const DetectedLine& line1,
  const DetectedLine& line2)
{
  // Check angle difference
  double angle_diff = std::abs(line1.angle - line2.angle);
  // Normalize to [0, pi]
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  angle_diff = std::abs(angle_diff);
  if (angle_diff > M_PI / 2) angle_diff = M_PI - angle_diff;

  double angle_threshold_rad = params_.line_merge_angle_threshold * M_PI / 180.0;
  if (angle_diff > angle_threshold_rad) {
    return false;
  }

  // Check distance between lines (shortest distance between line segments)
  // Use midpoints as representative points
  double mid1_x = (line1.start_x + line1.end_x) / 2.0;
  double mid1_y = (line1.start_y + line1.end_y) / 2.0;
  double mid2_x = (line2.start_x + line2.end_x) / 2.0;
  double mid2_y = (line2.start_y + line2.end_y) / 2.0;

  // Distance from mid1 to line2
  double dist1 = pointToLineDistance(mid1_x, mid1_y, line2.a, line2.b, line2.c);
  // Distance from mid2 to line1
  double dist2 = pointToLineDistance(mid2_x, mid2_y, line1.a, line1.b, line1.c);

  double min_dist = std::min(dist1, dist2);

  return min_dist <= params_.line_merge_distance_threshold;
}

visualization_msgs::msg::Marker PalletDetection::createLineMarker(
  const DetectedLine& line,
  int marker_id,
  const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "extracted_lines";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Set line start and end points (with height above floor)
  geometry_msgs::msg::Point p1, p2;
  p1.x = line.start_x;
  p1.y = line.start_y;
  p1.z = params_.marker_height;

  p2.x = line.end_x;
  p2.y = line.end_y;
  p2.z = params_.marker_height;

  marker.points.push_back(p1);
  marker.points.push_back(p2);

  // Set line thickness
  marker.scale.x = params_.marker_thickness;

  // Color: green for detected lines
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  return marker;
}

double PalletDetection::pointToLineDistance(double px, double py, double a, double b, double c)
{
  return std::abs(a * px + b * py + c) / std::sqrt(a*a + b*b);
}

}  // namespace floor_removal_rgbd
