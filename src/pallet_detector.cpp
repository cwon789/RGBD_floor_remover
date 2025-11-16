#include "floor_removal_rgbd/pallet_detector.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <cstddef>

namespace floor_removal_rgbd
{

PalletDetector::PalletDetector(const PalletDetectorParams& params)
  : params_(params)
{
  // Load template from file if it exists
  try {
    template_ = loadTemplate(params_.template_file_path);
  } catch (const std::exception& e) {
    std::cout << "[PalletDetector] Warning: Could not load template from " 
              << params_.template_file_path << ": " << e.what() << std::endl;
  }
}

void PalletDetector::setParams(const PalletDetectorParams& params)
{
  params_ = params;
}

PalletDetectionResult PalletDetector::detect(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  PalletDetectionResult result;

  if (!cloud || cloud->points.empty()) {
    std::cout << "[PalletDetector] Input cloud is empty" << std::endl;
    return result;
  }

  std::cout << "[PalletDetector] Processing cloud with " << cloud->points.size() << " points" << std::endl;

  // Perform template matching
  result.detected_pallets = matchTemplate(cloud, template_);

  // Find closest pallet
  int closest_idx = findClosestPallet(result.detected_pallets);
  if (closest_idx >= 0) {
    std::cout << "[PalletDetector] Closest pallet at (" 
              << result.detected_pallets[closest_idx].x << ", "
              << result.detected_pallets[closest_idx].y << ", "
              << result.detected_pallets[closest_idx].z << ")" << std::endl;
  }

  // Create point cloud of pallet centers
  for (const auto& pallet : result.detected_pallets) {
    pcl::PointXYZRGB center;
    center.x = pallet.x;
    center.y = pallet.y;
    center.z = pallet.z;
    
    // Color code by confidence
    if (pallet.confidence > 0.9) {
      // High confidence - green
      center.r = 0;
      center.g = 255;
      center.b = 0;
    } else if (pallet.confidence > 0.7) {
      // Medium confidence - yellow
      center.r = 255;
      center.g = 255;
      center.b = 0;
    } else {
      // Low confidence - red
      center.r = 255;
      center.g = 0;
      center.b = 0;
    }
    
    result.pallet_centers->points.push_back(center);
  }

  result.pallet_centers->width = result.pallet_centers->points.size();
  result.pallet_centers->height = 1;
  result.pallet_centers->is_dense = false;

  std::cout << "[PalletDetector] Detected " << result.detected_pallets.size() << " pallets" << std::endl;

  return result;
}

std::vector<DetectedPallet> PalletDetector::matchTemplate(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  const PalletTemplate& templ)
{
  std::vector<DetectedPallet> detected_pallets;

  if (templ.distances.empty() || cloud->points.size() < 3) {
    return detected_pallets;
  }

  // Simple template matching approach:
  // For each point in the cloud, try to match it with the template
  // by finding two other points that match the template distances
  
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto& p1 = cloud->points[i];
    
    // Try to find matching points for each other point in the cloud
    for (size_t j = i + 1; j < cloud->points.size(); ++j) {
      const auto& p2 = cloud->points[j];
      double d12 = calculateDistance(p1, p2);
      
      for (size_t k = j + 1; k < cloud->points.size(); ++k) {
        const auto& p3 = cloud->points[k];
        double d13 = calculateDistance(p1, p3);
        double d23 = calculateDistance(p2, p3);
        
        // Create measured distances vector and sort it
        std::vector<double> measured = {d12, d13, d23};
        std::sort(measured.begin(), measured.end());
        
        // Sort template distances for comparison
        std::vector<double> sorted_template = templ.distances;
        std::sort(sorted_template.begin(), sorted_template.end());
        
        // Check if distances match
        if (distancesMatch(measured, sorted_template, templ.tolerance_mm)) {
          // Found a match, create detected pallet
          DetectedPallet pallet;
          
          // Position is centroid of the three points
          pallet.x = (p1.x + p2.x + p3.x) / 3.0;
          pallet.y = (p1.y + p2.y + p3.y) / 3.0;
          pallet.z = (p1.z + p2.z + p3.z) / 3.0;
          
          // Simple orientation estimation (placeholder)
          pallet.roll = 0.0;
          pallet.pitch = 0.0;
          pallet.yaw = 0.0;
          
          // Confidence based on distance match quality
          pallet.confidence = 1.0; // Simplified
          
          // Size estimation (placeholder)
          pallet.length = 1.2;  // Standard pallet size
          pallet.width = 1.0;
          pallet.height = 0.144; // Standard pallet height
          
          detected_pallets.push_back(pallet);
          
          std::cout << "[PalletDetector] Found pallet at (" 
                    << pallet.x << ", " << pallet.y << ", " << pallet.z << ")" << std::endl;
        }
      }
    }
  }

  return detected_pallets;
}

double PalletDetector::calculateDistance(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) const
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool PalletDetector::distancesMatch(
  const std::vector<double>& measured,
  const std::vector<double>& template_distances,
  double tolerance_mm) const
{
  if (measured.size() != template_distances.size()) {
    return false;
  }
  
  double tolerance_m = tolerance_mm / 1000.0; // Convert mm to meters
  
  for (size_t i = 0; i < measured.size(); ++i) {
    if (std::abs(measured[i] - template_distances[i]) > tolerance_m) {
      return false;
    }
  }
  
  return true;
}

int PalletDetector::findClosestPallet(const std::vector<DetectedPallet>& detected_pallets) const
{
  if (detected_pallets.empty()) {
    return -1;
  }
  
  int closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  // Robot position is assumed to be at origin (0, 0, 0)
  for (size_t i = 0; i < detected_pallets.size(); ++i) {
    const auto& pallet = detected_pallets[i];
    double distance = std::sqrt(pallet.x * pallet.x + pallet.y * pallet.y + pallet.z * pallet.z);
    
    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = static_cast<int>(i);
    }
  }
  
  return closest_idx;
}

bool PalletDetector::saveTemplate(const PalletTemplate& template_data, const std::string& file_path)
{
  try {
    YAML::Node root;
    root["template"]["distances_m"] = template_data.distances;
    root["template"]["tolerance_mm"] = template_data.tolerance_mm;
    root["template"]["base_frame"] = template_data.base_frame;

    std::ofstream fout(file_path);
    fout << root;
    fout.close();
    
    std::cout << "[PalletDetector] Saved template to " << file_path << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[PalletDetector] Failed to save template: " << e.what() << std::endl;
    return false;
  }
}

PalletTemplate PalletDetector::loadTemplate(const std::string& file_path)
{
  PalletTemplate templ;
  
  try {
    YAML::Node root = YAML::LoadFile(file_path);
    
    if (root["template"]) {
      auto temp_node = root["template"];
      
      if (temp_node["distances_m"]) {
        templ.distances = temp_node["distances_m"].as<std::vector<double>>();
      }
      
      if (temp_node["tolerance_mm"]) {
        templ.tolerance_mm = temp_node["tolerance_mm"].as<double>();
      }
      
      if (temp_node["base_frame"]) {
        templ.base_frame = temp_node["base_frame"].as<std::string>();
      }
      
      std::cout << "[PalletDetector] Loaded template from " << file_path << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "[PalletDetector] Failed to load template: " << e.what() << std::endl;
    throw; // Re-throw to let caller handle
  }
  
  return templ;
}

}  // namespace floor_removal_rgbd
