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
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_cloud_3d,
  double nx, double ny, double nz, double d,
  const std::string& frame_id)
{
  PalletDetectionResult result;

  if (!cloud_2d || cloud_2d->points.empty()) {
    std::cout << "[PalletDetection] Input cloud is empty" << std::endl;
    return result;
  }

  std::cout << "[PalletDetection] Processing 2D cloud with " << cloud_2d->points.size() << " points" << std::endl;
  std::cout << "[PalletDetection] Floor plane: nx=" << nx << ", ny=" << ny << ", nz=" << nz << ", d=" << d << std::endl;
  if (floor_cloud_3d && !floor_cloud_3d->points.empty()) {
    std::cout << "[PalletDetection] Floor cloud (3D) has " << floor_cloud_3d->points.size() << " points" << std::endl;
  }

  // Preprocess: sort by angle and remove noise
  auto cloud_preprocessed = preprocessCloud(cloud_2d);

  if (!cloud_preprocessed || cloud_preprocessed->points.empty()) {
    std::cout << "[PalletDetection] Cloud is empty after preprocessing" << std::endl;
    return result;
  }

  std::cout << "[PalletDetection] After preprocessing: " << cloud_preprocessed->points.size() << " points" << std::endl;

  // Extract lines from preprocessed point cloud
  result.detected_lines = extractLines(cloud_preprocessed);

  // Merge similar lines
  result.detected_lines = mergeLines(result.detected_lines);

  // Disambiguate angles using previous frame data for temporal consistency
  std::vector<double> current_angles;
  for (auto& line : result.detected_lines) {
    // Find closest previous angle by position
    double best_prev_angle = 0.0;
    bool found_previous = false;

    if (!previous_angles_.empty()) {
      // Use the first previous angle as reference (simple approach)
      // In production, you'd match by position/distance
      best_prev_angle = previous_angles_[0];
      found_previous = true;
    }

    if (found_previous) {
      // Disambiguate: choose angle or angle+π based on previous
      double disambiguated = disambiguateAngle(line.angle, best_prev_angle);

      // If angle changed, flip direction vector to maintain consistency
      if (std::abs(disambiguated - line.angle) > M_PI / 2.0) {
        // Swap start and end points
        std::swap(line.start_x, line.end_x);
        std::swap(line.start_y, line.end_y);
      }

      line.angle = disambiguated;
    }

    current_angles.push_back(line.angle);
  }

  // Update previous angles for next frame
  previous_angles_ = current_angles;

  // Create visualization markers (lines and cuboids)
  for (size_t i = 0; i < result.detected_lines.size(); ++i) {
    // Line marker
    auto line_marker = createLineMarker(result.detected_lines[i], i, frame_id);
    result.line_markers.markers.push_back(line_marker);

    // Cuboid marker
    auto cuboid_marker = createCuboidMarker(result.detected_lines[i], i, frame_id);
    result.cuboid_markers.markers.push_back(cuboid_marker);
  }

  // Collect all line points as line_candidates
  for (const auto& line : result.detected_lines) {
    for (const auto& pt : line.line_cloud->points) {
      result.line_candidates->points.push_back(pt);
    }
  }

  if (!result.line_candidates->points.empty()) {
    result.line_candidates->width = result.line_candidates->points.size();
    result.line_candidates->height = 1;
    result.line_candidates->is_dense = false;
  }

  // Filter no_floor_cloud_3d points that are inside cuboids -> pallet_candidates
  // no_floor_cloud_3d is already in the same coordinate system as cloud_2d (projected)
  // So we can directly compare XY coordinates, just need to check Z height
  if (floor_cloud_3d && !floor_cloud_3d->points.empty() && !result.detected_lines.empty()) {
    for (const auto& pt : floor_cloud_3d->points) {
      // Check if point is inside any cuboid (direct XY check + Z height check)
      for (const auto& line : result.detected_lines) {
        if (isPointInsideCuboid3D(pt, line)) {
          result.pallet_candidates->points.push_back(pt);
          break;  // Point belongs to this cuboid, no need to check others
        }
      }
    }

    if (!result.pallet_candidates->points.empty()) {
      result.pallet_candidates->width = result.pallet_candidates->points.size();
      result.pallet_candidates->height = 1;
      result.pallet_candidates->is_dense = false;
    }
  }

  std::cout << "[PalletDetection] Detected " << result.detected_lines.size() << " lines" << std::endl;
  std::cout << "[PalletDetection] Total line candidates: " << result.line_candidates->points.size() << " points" << std::endl;
  std::cout << "[PalletDetection] Total pallet candidates (in cuboids): " << result.pallet_candidates->points.size() << " points" << std::endl;
  std::cout << "[PalletDetection] Generated " << result.cuboid_markers.markers.size() << " cuboid markers" << std::endl;

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
      // Check minimum and maximum length
      bool length_ok = (line.length >= params_.line_min_length);

      if (length_ok) {
        // Check if line is too long (might be person + wall merged together)
        if (params_.line_max_length > 0 && line.length > (params_.line_max_length + params_.line_max_length_tolerance)) {
          std::cout << "[PalletDetection] Line too long (" << line.length
                    << "m > " << (params_.line_max_length + params_.line_max_length_tolerance)
                    << "m), attempting to split..." << std::endl;

          // Try to split this line by finding gaps in the inlier points
          auto split_lines = splitLongLine(cloud_2d, line);

          for (const auto& split_line : split_lines) {
            if (split_line.length >= params_.line_min_length &&
                split_line.length <= (params_.line_max_length + params_.line_max_length_tolerance)) {
              lines.push_back(split_line);
              std::cout << "[PalletDetection] Split line " << lines.size()
                        << ": length=" << split_line.length << "m, angle=" << (split_line.angle * 180.0 / M_PI)
                        << "°, inliers=" << split_line.num_inliers << std::endl;
            }
          }
        } else {
          lines.push_back(line);
          std::cout << "[PalletDetection] Line " << lines.size()
                    << ": length=" << line.length << "m, angle=" << (line.angle * 180.0 / M_PI)
                    << "°, inliers=" << line.num_inliers << std::endl;
        }
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

  // Normalize to forward direction [-π/2, π/2] to prevent 180-degree flip ambiguity
  // This ensures consistent orientation across frames
  double angle = std::atan2(vy, vx);
  if (angle > M_PI / 2.0) {
    angle -= M_PI;
    vx = -vx;  // Flip direction vector
    vy = -vy;
  } else if (angle < -M_PI / 2.0) {
    angle += M_PI;
    vx = -vx;  // Flip direction vector
    vy = -vy;
  }

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

  // Find actual points closest to min/max projections to ensure line length matches point cloud extent
  double min_dist_to_min = std::numeric_limits<double>::max();
  double min_dist_to_max = std::numeric_limits<double>::max();
  pcl::PointXYZRGB start_point, end_point;

  for (int idx : best_inlier_indices) {
    const auto& pt = cloud_2d->points[idx];
    double proj = vx * pt.x + vy * pt.y;

    double dist_to_min = std::abs(proj - min_proj);
    if (dist_to_min < min_dist_to_min) {
      min_dist_to_min = dist_to_min;
      start_point = pt;
    }

    double dist_to_max = std::abs(proj - max_proj);
    if (dist_to_max < min_dist_to_max) {
      min_dist_to_max = dist_to_max;
      end_point = pt;
    }
  }

  // Use actual point coordinates as endpoints
  line.start_x = start_point.x;
  line.start_y = start_point.y;
  line.end_x = end_point.x;
  line.end_y = end_point.y;

  // Calculate line length
  double dx = line.end_x - line.start_x;
  double dy = line.end_y - line.start_y;
  line.length = std::sqrt(dx*dx + dy*dy);

  // Store normalized angle
  line.angle = angle;

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

std::vector<DetectedLine> PalletDetection::splitLongLine(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& /* cloud_2d */,
  const DetectedLine& long_line)
{
  std::vector<DetectedLine> split_lines;

  if (long_line.line_cloud->points.empty()) {
    return split_lines;
  }

  // Step 1: Sort line points by projection along the line direction
  struct PointProjection {
    pcl::PointXYZRGB point;
    double projection;
    size_t original_idx;
  };

  // Calculate line direction vector
  double dx = long_line.end_x - long_line.start_x;
  double dy = long_line.end_y - long_line.start_y;
  double line_len = std::sqrt(dx*dx + dy*dy);

  if (line_len < 1e-6) {
    return split_lines;
  }

  double dir_x = dx / line_len;
  double dir_y = dy / line_len;

  std::vector<PointProjection> projections;
  projections.reserve(long_line.line_cloud->points.size());

  for (size_t i = 0; i < long_line.line_cloud->points.size(); ++i) {
    const auto& pt = long_line.line_cloud->points[i];
    PointProjection pp;
    pp.point = pt;
    // Project point onto line direction from start point
    pp.projection = (pt.x - long_line.start_x) * dir_x + (pt.y - long_line.start_y) * dir_y;
    pp.original_idx = i;
    projections.push_back(pp);
  }

  // Sort by projection
  std::sort(projections.begin(), projections.end(),
    [](const PointProjection& a, const PointProjection& b) {
      return a.projection < b.projection;
    });

  // Step 2: Find gaps in the sorted points
  // Gap threshold: if distance between consecutive points is too large, it's a gap
  double gap_threshold = 0.15; // meters - gap threshold for splitting lines

  std::vector<std::vector<size_t>> segments;
  std::vector<size_t> current_segment;
  current_segment.push_back(0);

  for (size_t i = 1; i < projections.size(); ++i) {
    double gap = projections[i].projection - projections[i-1].projection;

    if (gap > gap_threshold) {
      // Found a gap - save current segment and start new one
      if (current_segment.size() >= static_cast<size_t>(params_.line_min_points)) {
        segments.push_back(current_segment);
      }
      current_segment.clear();
    }

    current_segment.push_back(i);
  }

  // Don't forget the last segment
  if (current_segment.size() >= static_cast<size_t>(params_.line_min_points)) {
    segments.push_back(current_segment);
  }

  std::cout << "[SplitLine] Found " << segments.size() << " segments in long line" << std::endl;

  // Step 3: Create a line for each segment
  for (const auto& segment : segments) {
    if (segment.size() < static_cast<size_t>(params_.line_min_points)) {
      continue;
    }

    // Collect points for this segment
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    segment_cloud->points.reserve(segment.size());

    for (size_t idx : segment) {
      segment_cloud->points.push_back(projections[idx].point);
    }

    segment_cloud->width = segment_cloud->points.size();
    segment_cloud->height = 1;
    segment_cloud->is_dense = false;

    // Fit a line to this segment using least squares
    DetectedLine segment_line;
    segment_line.line_cloud = segment_cloud;

    // Compute mean
    double sum_x = 0, sum_y = 0;
    double sum_xx = 0, sum_xy = 0, sum_yy = 0;

    for (const auto& pt : segment_cloud->points) {
      sum_x += pt.x;
      sum_y += pt.y;
      sum_xx += pt.x * pt.x;
      sum_xy += pt.x * pt.y;
      sum_yy += pt.y * pt.y;
    }

    int n = segment_cloud->points.size();
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

    // Normalize to forward direction [-π/2, π/2] to prevent 180-degree flip ambiguity
    // This ensures consistent orientation across frames
    double angle = std::atan2(vy, vx);
    if (angle > M_PI / 2.0) {
      angle -= M_PI;
      vx = -vx;  // Flip direction vector
      vy = -vy;
    } else if (angle < -M_PI / 2.0) {
      angle += M_PI;
      vx = -vx;  // Flip direction vector
      vy = -vy;
    }

    // Line equation: perpendicular to direction
    segment_line.a = -vy;
    segment_line.b = vx;
    segment_line.c = vy * mean_x - vx * mean_y;

    // Normalize
    norm = std::sqrt(segment_line.a*segment_line.a + segment_line.b*segment_line.b);
    segment_line.a /= norm;
    segment_line.b /= norm;
    segment_line.c /= norm;

    // Find endpoints (min/max projection on line direction)
    double min_proj = std::numeric_limits<double>::max();
    double max_proj = std::numeric_limits<double>::lowest();

    for (const auto& pt : segment_cloud->points) {
      double proj = vx * pt.x + vy * pt.y;
      min_proj = std::min(min_proj, proj);
      max_proj = std::max(max_proj, proj);
    }

    // Find actual points closest to min/max projections to ensure line length matches point cloud extent
    double min_dist_to_min = std::numeric_limits<double>::max();
    double min_dist_to_max = std::numeric_limits<double>::max();
    pcl::PointXYZRGB seg_start_point, seg_end_point;

    for (const auto& pt : segment_cloud->points) {
      double proj = vx * pt.x + vy * pt.y;

      double dist_to_min = std::abs(proj - min_proj);
      if (dist_to_min < min_dist_to_min) {
        min_dist_to_min = dist_to_min;
        seg_start_point = pt;
      }

      double dist_to_max = std::abs(proj - max_proj);
      if (dist_to_max < min_dist_to_max) {
        min_dist_to_max = dist_to_max;
        seg_end_point = pt;
      }
    }

    // Use actual point coordinates as endpoints
    segment_line.start_x = seg_start_point.x;
    segment_line.start_y = seg_start_point.y;
    segment_line.end_x = seg_end_point.x;
    segment_line.end_y = seg_end_point.y;

    // Calculate line length
    double seg_dx = segment_line.end_x - segment_line.start_x;
    double seg_dy = segment_line.end_y - segment_line.start_y;
    segment_line.length = std::sqrt(seg_dx*seg_dx + seg_dy*seg_dy);

    // Store normalized angle
    segment_line.angle = angle;

    segment_line.num_inliers = segment_cloud->points.size();

    split_lines.push_back(segment_line);
  }

  return split_lines;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PalletDetection::preprocessCloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2d)
{
  if (!cloud_2d || cloud_2d->points.empty()) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  }

  // Step 1: Sort points by angle (like LaserScan)
  std::cout << "[Preprocessing] Sorting points by angle..." << std::endl;

  struct PointWithAngle {
    pcl::PointXYZRGB point;
    double angle;
    double range;
  };

  std::vector<PointWithAngle> points_with_angles;
  points_with_angles.reserve(cloud_2d->points.size());

  for (const auto& pt : cloud_2d->points) {
    PointWithAngle pwa;
    pwa.point = pt;
    pwa.angle = std::atan2(pt.y, pt.x);
    pwa.range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    points_with_angles.push_back(pwa);
  }

  // Sort by angle
  std::sort(points_with_angles.begin(), points_with_angles.end(),
    [](const PointWithAngle& a, const PointWithAngle& b) {
      return a.angle < b.angle;
    });

  // Step 2: Angular binning - keep only closest point in each angular bin
  std::cout << "[Preprocessing] Angular binning..." << std::endl;

  double angle_bin_rad = params_.angle_bin_size * M_PI / 180.0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sorted(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_sorted->header = cloud_2d->header;

  if (points_with_angles.empty()) {
    return cloud_sorted;
  }

  // Group by angular bins
  double current_bin_start = points_with_angles[0].angle;
  double min_range = points_with_angles[0].range;
  size_t min_range_idx = 0;

  for (size_t i = 0; i < points_with_angles.size(); ++i) {
    const auto& pwa = points_with_angles[i];

    // Check if still in same bin
    if (pwa.angle - current_bin_start < angle_bin_rad) {
      // Update minimum range point in this bin
      if (pwa.range < min_range) {
        min_range = pwa.range;
        min_range_idx = i;
      }
    } else {
      // Save the closest point from previous bin
      cloud_sorted->points.push_back(points_with_angles[min_range_idx].point);

      // Start new bin
      current_bin_start = pwa.angle;
      min_range = pwa.range;
      min_range_idx = i;
    }
  }

  // Add last bin's closest point
  cloud_sorted->points.push_back(points_with_angles[min_range_idx].point);

  cloud_sorted->width = cloud_sorted->points.size();
  cloud_sorted->height = 1;
  cloud_sorted->is_dense = false;

  std::cout << "[Preprocessing] Final point count: " << cloud_sorted->points.size() << std::endl;

  return cloud_sorted;
}

visualization_msgs::msg::Marker PalletDetection::createCuboidMarker(
  const DetectedLine& line,
  int marker_id,
  const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "pallet_cuboids";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Calculate line center point
  double center_x = (line.start_x + line.end_x) / 2.0;
  double center_y = (line.start_y + line.end_y) / 2.0;
  double center_z = params_.cuboid_height / 2.0;  // Center at half height

  marker.pose.position.x = center_x;
  marker.pose.position.y = center_y;
  marker.pose.position.z = center_z;

  // Calculate orientation from line angle
  // Line is in XY plane, so rotation is around Z axis
  double yaw = line.angle;  // Line angle is already in radians

  // Convert yaw to quaternion (rotation around Z axis)
  // q = [cos(yaw/2), 0, 0, sin(yaw/2)]
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(yaw / 2.0);
  marker.pose.orientation.w = std::cos(yaw / 2.0);

  // Set scale (dimensions of the box)
  marker.scale.x = line.length;              // Length along the line
  marker.scale.y = params_.cuboid_thickness;  // Thickness perpendicular to line
  marker.scale.z = params_.cuboid_height;     // Height

  // Set color (blue with less transparency)
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.6;  // Less transparent (higher value = less transparent)

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  return marker;
}

bool PalletDetection::isPointInsideCuboid3D(
  const pcl::PointXYZRGB& point,
  const DetectedLine& line)
{
  // Check Z height first: cuboid extends from floor (Z≈0) up to cuboid_height
  if (point.z < 0.0 || point.z > params_.cuboid_height) {
    return false;
  }

  // Calculate line direction and perpendicular vectors
  double dx = line.end_x - line.start_x;
  double dy = line.end_y - line.start_y;
  double line_len = std::sqrt(dx*dx + dy*dy);

  if (line_len < 1e-6) {
    return false;
  }

  double dir_x = dx / line_len;
  double dir_y = dy / line_len;

  // Perpendicular vector (to the left when facing along the line)
  double perp_x = -dir_y;
  double perp_y = dir_x;

  // Line center
  double center_x = (line.start_x + line.end_x) / 2.0;
  double center_y = (line.start_y + line.end_y) / 2.0;

  // Vector from center to point
  double to_point_x = point.x - center_x;
  double to_point_y = point.y - center_y;

  // Project onto line direction (along the line)
  double proj_along = to_point_x * dir_x + to_point_y * dir_y;

  // Project onto perpendicular direction (thickness)
  double proj_perp = to_point_x * perp_x + to_point_y * perp_y;

  // Check if point is within cuboid bounds
  double half_length = line.length / 2.0;
  double half_thickness = params_.cuboid_thickness / 2.0;

  return (std::abs(proj_along) <= half_length) && (std::abs(proj_perp) <= half_thickness);
}

double PalletDetection::disambiguateAngle(double angle, double prev_angle)
{
  // Normalize both angles to [-π, π]
  auto normalize = [](double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  };

  angle = normalize(angle);
  prev_angle = normalize(prev_angle);

  // Two possible directions: angle and angle+π
  double angle_alt = angle + M_PI;
  angle_alt = normalize(angle_alt);

  // Choose the one closer to previous angle
  double diff1 = std::abs(angle - prev_angle);
  double diff2 = std::abs(angle_alt - prev_angle);

  // Handle angle wrapping (e.g., -π vs +π)
  if (diff1 > M_PI) diff1 = 2.0 * M_PI - diff1;
  if (diff2 > M_PI) diff2 = 2.0 * M_PI - diff2;

  return (diff1 < diff2) ? angle : angle_alt;
}

}  // namespace floor_removal_rgbd
