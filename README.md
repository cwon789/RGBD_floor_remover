# Floor Removal RGBD

A ROS2 package for detecting and removing floor planes from RGBD point clouds using RANSAC plane segmentation.

## Overview

This package processes point clouds from RGBD cameras (e.g., Intel RealSense D435i) to detect and separate floor planes from the scene. It uses a two-stage approach:

1. **Detection Stage**: Identifies the floor plane using RANSAC on a wider region
2. **Removal Stage**: Removes only a thin layer around the detected plane to preserve objects on the floor

This design allows the algorithm to:
- Robustly detect floor planes with sufficient sample points
- Preserve low-height objects on the floor (e.g., 10cm pallets)
- Maintain temporal stability to prevent flickering

## Features

- **Dual Detection Modes**: Auto-detect floor using y_max OR use fixed camera height
- **Camera Frame Processing**: Works directly in camera optical frame (no coordinate transformation required)
- **Dual-Thickness Strategy**: Separate parameters for detection and removal
- **Voxel Grid Downsampling**: Reduces computational cost and filters noise
- **Object Preservation**: Configurable removal thickness to preserve objects on floor
- **Multiple Output Topics**: Publishes both original and voxelized point clouds

## Architecture

### File Structure

```
floor_removal_rgbd/
├── include/floor_removal_rgbd/
│   ├── plane_remover.hpp       # Core algorithm class
│   └── server_node.hpp         # ROS2 node interface
├── src/
│   ├── plane_remover.cpp       # Core floor removal implementation
│   └── server_node.cpp         # ROS2 node + main()
├── launch/
│   └── floor_removal.launch.py
├── config/
│   └── params.yaml
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Class Overview

#### `PlaneRemover`
Core algorithm class that handles floor plane detection and removal.

**Key Methods:**
- `process()`: Main processing pipeline
- `applyVoxelGrid()`: Downsamples point cloud
- `findMaxY()`: Finds floor location (auto mode)
- `extractFloorRegionCameraFrame()`: Extracts candidate floor points
- `detectFloorPlane()`: Runs RANSAC to detect floor plane
- `isPlaneValid()`: Validates detected plane
- `classifyPoints()`: Separates floor and non-floor points

#### `FloorRemovalServerNode`
ROS2 node that provides the service interface.

**Responsibilities:**
- Load parameters from ROS2 parameter server
- Subscribe to point cloud topics
- Call PlaneRemover for processing
- Publish separated floor and non-floor clouds

## Algorithm

### Coordinate System

The algorithm works in **camera optical frame**:
- **X axis**: Right (+)
- **Y axis**: Down (+)
- **Z axis**: Forward/Depth (+)

Since Y points downward, the floor has the **maximum Y value** (lowest points).

### Processing Pipeline

1. **Voxel Grid Downsampling** (optional)
   - Reduces ~100k points to ~10k points
   - Filters noise and improves performance
   - Configurable leaf size (default: 1cm)

2. **Determine Floor Height**
   - **Auto Mode** (`auto_floor_detection_mode: true`): Finds maximum Y value in the scene
   - **Fixed Mode** (`auto_floor_detection_mode: false`): Uses configured `camera_height` value

3. **Extract Floor Region (Detection)**
   - Selects points within `floor_detection_thickness` from determined height
   - Uses wider region (15cm) to ensure sufficient points for RANSAC
   - Example: if floor height = 0.8m, selects points with Y ∈ [0.65, 0.8]

4. **RANSAC Plane Fitting**
   - Runs RANSAC on floor region to find plane equation: `nx*x + ny*y + nz*z + d = 0`
   - Validates plane normal (should point downward: Y component > threshold)

5. **Point Classification (Removal)**
   - Calculates signed distance from each point to plane
   - Uses thin removal thickness (3cm) to preserve objects
   - Removes only points within `±(floor_removal_thickness/2 + floor_margin)`
   - Example: 3cm removal = removes points ±1.5cm from plane

### Dual-Thickness Strategy

**Problem**: Using the same thickness for detection and removal creates a trade-off:
- Too thick → includes objects on floor (e.g., pallets) → inaccurate plane
- Too thin → insufficient points for RANSAC → unstable detection

**Solution**: Separate parameters
- **Detection thickness** (15cm): Wide region for robust RANSAC
- **Removal thickness** (3cm): Thin layer for precise floor removal

**Example**:
```
Floor plane at Y = 0.8m

Detection phase:
  Select points: Y ∈ [0.65, 0.8] (15cm range)
  → Enough points for RANSAC
  → Plane equation: ny*y + ... = 0

Removal phase:
  Remove points: distance to plane ∈ [-0.015, +0.015] (3cm range)
  → Only removes thin floor surface
  → 10cm pallet preserved ✓
```

## Parameters

### Input/Output Topics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_cloud_topic` | `/camera/depth/color/points` | Input point cloud topic |
| `output_floor_cloud_topic` | `/floor_cloud` | Output topic for floor points (original resolution) |
| `output_no_floor_cloud_topic` | `/no_floor_cloud` | Output topic for non-floor points (original resolution) |
| `output_floor_cloud_voxelized_topic` | `/floor_cloud_voxelized` | Output topic for floor points (voxelized) |
| `output_no_floor_cloud_voxelized_topic` | `/no_floor_cloud_voxelized` | Output topic for non-floor points (voxelized) |

### Floor Detection Mode

| Parameter | Default | Description |
|-----------|---------|-------------|
| `auto_floor_detection_mode` | `true` | `true`: Auto-detect floor using y_max<br>`false`: Use fixed camera height |
| `camera_height` | `0.80` | Camera height from floor (meters)<br>Used only when `auto_floor_detection_mode: false` |

**Mode Details**:
- **Auto Mode** (`true`): Automatically finds the lowest point (y_max) in the scene. Best for testing and dynamic environments.
- **Fixed Mode** (`false`): Uses the configured `camera_height` value. Best for production environments with fixed camera positions.

### RANSAC Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ransac_distance_threshold` | `0.02` | Distance threshold for RANSAC inliers (meters) |
| `ransac_max_iterations` | `100` | Maximum iterations for RANSAC |
| `floor_normal_z_threshold` | `0.15` | Minimum Y component of normal (camera frame: Y=down) |

The normal threshold ensures the detected plane is approximately horizontal (floor-like, not wall-like).

### Floor Region Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `floor_detection_thickness` | `0.15` | Thickness for RANSAC plane detection (meters)<br>**Wider region** to get sufficient points after voxelization |
| `floor_removal_thickness` | `0.03` | Thickness for actual floor removal (meters)<br>**Thin layer** to preserve objects on floor (e.g., 10cm pallets) |
| `floor_margin` | `0.01` | Additional margin around detected plane (meters) |

**Tuning Guide**:
- If RANSAC fails (too few points): Increase `floor_detection_thickness`
- If objects on floor are removed: Decrease `floor_removal_thickness`
- If floor edges are missed: Increase `floor_margin`

### Voxel Grid Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_voxel_grid` | `true` | Enable voxel grid downsampling |
| `voxel_leaf_size` | `0.01` | Voxel size (meters)<br>1cm reduces ~100k points to ~10-15k points |

**Trade-offs**:
- Smaller leaf size (0.005): More detail, more computation
- Larger leaf size (0.02): Faster, less detail, may miss small features

## Usage

### Installation

```bash
# Install dependencies
sudo apt-get install ros-foxy-pcl-ros ros-foxy-pcl-conversions

# Build the package
cd ~/catkin_rs
colcon build --packages-select floor_removal_rgbd
source install/setup.bash
```

### Launch

```bash
# Launch with default parameters
ros2 launch floor_removal_rgbd floor_removal.launch.py

# Launch with custom parameters
ros2 launch floor_removal_rgbd floor_removal.launch.py params_file:=/path/to/custom_params.yaml
```

### With RealSense Camera

```bash
# Terminal 1: Launch RealSense camera
ros2 launch realsense2_camera rs_launch.py \
  enable_rgbd:=true \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true

# Terminal 2: Launch floor removal
ros2 launch floor_removal_rgbd floor_removal.launch.py
```

### Visualize in RViz

```bash
# Open RViz
rviz2

# Add displays:
# - PointCloud2: /camera/depth/color/points (original)
# - PointCloud2: /floor_cloud (detected floor - original resolution)
# - PointCloud2: /no_floor_cloud (scene without floor - original resolution)
# - PointCloud2: /floor_cloud_voxelized (detected floor - voxelized)
# - PointCloud2: /no_floor_cloud_voxelized (scene without floor - voxelized)

# Set Fixed Frame to: camera_color_optical_frame
```

### Parameter Tuning

Edit `config/params.yaml` and restart the node:

```yaml
floor_removal_node:
  ros__parameters:
    # Switch to fixed camera height mode (production)
    auto_floor_detection_mode: false
    camera_height: 0.75              # Camera is 75cm from floor

    # Adjust detection thickness if RANSAC fails
    floor_detection_thickness: 0.20  # Increase to 20cm

    # Adjust removal thickness based on object height
    floor_removal_thickness: 0.05    # Increase to 5cm for thicker floor layer
```

## Examples

### Example 1: Testing with Auto Detection

**Scenario**: Testing and calibration with dynamic floor detection

**Configuration**:
```yaml
auto_floor_detection_mode: true  # Auto-detect floor
use_voxel_grid: true
voxel_leaf_size: 0.01
```

**Result**: Automatically finds floor in varying environments ✓

### Example 2: Production with Fixed Camera Height

**Scenario**: Production environment with camera mounted at fixed height

**Configuration**:
```yaml
auto_floor_detection_mode: false
camera_height: 0.75              # Camera is 75cm from floor
floor_detection_thickness: 0.15
floor_removal_thickness: 0.03
```

**Result**: Consistent floor removal without dynamic detection overhead ✓

### Example 3: Preserving Pallets

**Scenario**: Detect floor but preserve 10cm pallets

**Configuration**:
```yaml
auto_floor_detection_mode: false
camera_height: 0.80
floor_detection_thickness: 0.15  # Wide enough for RANSAC
floor_removal_thickness: 0.03    # Thin removal (< 10cm)
floor_margin: 0.01
```

**Result**: Floor removed, pallets preserved ✓

### Example 4: Performance Optimization

**Scenario**: 30Hz processing required

**Configuration**:
```yaml
use_voxel_grid: true
voxel_leaf_size: 0.02            # Aggressive downsampling
ransac_max_iterations: 50        # Fewer iterations
```

**Result**: Faster processing with acceptable accuracy ✓

## Troubleshooting

### Issue: "No floor plane detected"

**Possible Causes**:
1. Floor region too thin → insufficient points for RANSAC
2. Floor not visible in frame
3. Floor not horizontal (normal check fails)

**Solutions**:
- Increase `floor_detection_thickness` to 0.2m or 0.3m
- Check camera view (floor should be visible)
- Decrease `floor_normal_z_threshold` if floor is tilted

### Issue: RANSAC errors ("Can not select unique points")

**Cause**: Floor region has too few points after voxelization

**Solutions**:
- Increase `floor_detection_thickness`
- Decrease `voxel_leaf_size` (more points kept)
- Disable voxel grid: `use_voxel_grid: false`

### Issue: Objects on floor are removed

**Cause**: Removal thickness too large

**Solutions**:
- Decrease `floor_removal_thickness` to 0.02m or 0.01m
- Ensure objects are at least 2x removal thickness height
- Check plane accuracy (visualize `/floor_cloud`)

### Issue: Floor edges are missed

**Cause**: Removal thickness or margin too small

**Solutions**:
- Increase `floor_margin` to 0.02m or 0.03m
- Slightly increase `floor_removal_thickness`

### Issue: Voxel grid overflow errors

**Cause**: Leaf size too small for the dataset

**Solutions**:
- Increase `voxel_leaf_size` to 0.015m or 0.02m
- Algorithm automatically falls back to original cloud on error

## Performance

Typical performance on Intel Core i5:
- Input: 120k points
- After voxelization (1cm): 12k points
- Processing time: 25-35ms (~30Hz)

## Dependencies

- ROS2 Foxy or later
- PCL (Point Cloud Library)
- Eigen3
- sensor_msgs
- geometry_msgs

## Published Topics

- `/floor_cloud` (sensor_msgs/PointCloud2): Detected floor points
- `/no_floor_cloud` (sensor_msgs/PointCloud2): Scene with floor removed

## Subscribed Topics

- `/camera/depth/color/points` (sensor_msgs/PointCloud2): Input point cloud

## License

[Specify your license here]

## Authors

[Your name/organization]

## References

- RANSAC: Fischler & Bolles, "Random Sample Consensus", 1981
- PCL: Point Cloud Library - https://pointclouds.org/
