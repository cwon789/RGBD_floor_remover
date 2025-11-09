# Floor Removal RGBD

A ROS2 package for detecting and removing floor planes from RGBD point clouds using RANSAC plane segmentation. Includes pallet stringer detection for warehouse automation.

## Overview

This package processes point clouds from RGBD cameras (e.g., Intel RealSense D435i) to:
1. Detect and separate floor planes from the scene
2. Detect pallet stringers (wooden support beams) in the remaining cloud

The algorithm uses a two-stage approach:
1. **Detection Stage**: Identifies the floor plane using RANSAC on a wider region
2. **Removal Stage**: Removes only a thin layer around the detected plane to preserve objects on the floor

This design allows the algorithm to:
- Robustly detect floor planes with sufficient sample points
- Preserve low-height objects on the floor (e.g., 10cm pallets)
- Detect pallet stringers for automated forklift navigation
- Work in robot coordinate frame for intuitive parameter tuning

## Features

- **Robot Coordinate Frame Processing**: Processes in robot base frame (X=forward, Y=left, Z=up) for intuitive height-based filtering
- **Dual Detection Modes**: Auto-detect floor using z_min OR use fixed floor height
- **Camera Extrinsic Support**: Configurable camera position and orientation
- **Dual-Thickness Strategy**: Separate parameters for detection and removal
- **Voxel Grid Downsampling**: Reduces computational cost and filters noise
- **Pallet Stringer Detection**: Detects wooden support beams using clustering and dimensional analysis
- **Multiple Output Topics**: Publishes both original and voxelized point clouds
- **Visualization Support**: Publishes bounding box markers for detected stringers

## Architecture

### File Structure

```
floor_removal_rgbd/
├── include/floor_removal_rgbd/
│   ├── plane_remover.hpp       # Core floor removal algorithm
│   ├── stringer_detector.hpp   # Pallet stringer detection
│   └── server_node.hpp         # ROS2 node interface
├── src/
│   ├── plane_remover.cpp       # Floor removal implementation
│   ├── stringer_detector.cpp   # Stringer detection implementation
│   └── server_node.cpp         # ROS2 node + main()
├── launch/
│   └── floor_removal.launch.py
├── config/
│   └── params.yaml             # Configuration file
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Class Overview

#### `PlaneRemover`
Core algorithm class that handles floor plane detection and removal.

**Key Methods:**
- `process()`: Main processing pipeline
- `transformToStandardFrame()`: Converts camera optical frame to robot base frame
- `applyVoxelGrid()`: Downsamples point cloud
- `findMinZ()`: Finds floor location in robot frame (auto mode)
- `extractFloorRegionRobotFrame()`: Extracts candidate floor points
- `detectFloorPlane()`: Runs RANSAC to detect floor plane
- `isPlaneValid()`: Validates detected plane
- `classifyPoints()`: Separates floor and non-floor points

#### `StringerDetector`
Detects pallet stringers using Euclidean clustering and dimensional analysis.

**Key Methods:**
- `detect()`: Main detection pipeline
- `clusterPointCloud()`: Performs Euclidean clustering
- `computeBoundingBox()`: Computes bounding box for each cluster
- `matchesStringerCriteria()`: Validates cluster dimensions

#### `FloorRemovalServerNode`
ROS2 node that provides the subscription/publishing interface.

**Responsibilities:**
- Load parameters from ROS2 parameter server
- Subscribe to point cloud topics
- Call PlaneRemover for processing
- Publish separated floor and non-floor clouds
- Publish stringer detection results (markers and centers)

## Algorithm

### Coordinate System

The algorithm works in **robot base frame** after transforming from camera optical frame:

**Camera Optical Frame** (input):
- **X axis**: Right (+)
- **Y axis**: Down (+)
- **Z axis**: Forward/Depth (+)

**Robot Base Frame** (processing):
- **X axis**: Forward (+)
- **Y axis**: Left (+)
- **Z axis**: Up (+)

Since Z points upward, the floor has the **minimum Z value** (lowest points).

### Processing Pipeline

1. **Coordinate Transformation**
   - Transforms camera optical frame to robot base frame
   - Optional: Apply custom extrinsic parameters (camera position/orientation)

2. **Voxel Grid Downsampling** (optional)
   - Reduces ~100k points to ~10k points
   - Filters noise and improves performance
   - Configurable leaf size (default: 3cm)

3. **Determine Floor Height**
   - **Auto Mode** (`auto_floor_detection_mode: true`): Finds minimum Z value in the scene
   - **Fixed Mode** (`auto_floor_detection_mode: false`): Uses configured `floor_height` value

4. **Extract Floor Region (Detection)**
   - Selects points within `floor_detection_thickness` from determined height
   - Uses wider region (15cm) to ensure sufficient points for RANSAC
   - Example: if floor height = 0.0m, selects points with Z ∈ [0.0, 0.15]

5. **RANSAC Plane Fitting**
   - Runs RANSAC on floor region to find plane equation: `nx*x + ny*y + nz*z + d = 0`
   - Validates plane normal (should point upward: Z component > threshold)

6. **Point Classification (Removal)**
   - Calculates signed distance from each point to plane
   - Uses thin removal thickness (3cm) to preserve objects
   - Removes only points within `±(floor_removal_thickness/2 + floor_margin)`
   - Example: 3cm removal = removes points ±1.5cm from plane

7. **Stringer Detection** (optional)
   - Clusters remaining points using Euclidean clustering
   - Analyzes cluster dimensions (width, height, depth)
   - Identifies clusters matching stringer criteria

### Dual-Thickness Strategy

**Problem**: Using the same thickness for detection and removal creates a trade-off:
- Too thick → includes objects on floor (e.g., pallets) → inaccurate plane
- Too thin → insufficient points for RANSAC → unstable detection

**Solution**: Separate parameters
- **Detection thickness** (15cm): Wide region for robust RANSAC
- **Removal thickness** (3cm): Thin layer for precise floor removal

**Example**:
```
Floor plane at Z = 0.0m (robot frame)

Detection phase:
  Select points: Z ∈ [0.0, 0.15] (15cm range)
  → Enough points for RANSAC
  → Plane equation: nz*z + ... = 0

Removal phase:
  Remove points: distance to plane ∈ [-0.015, +0.015] (3cm range)
  → Only removes thin floor surface
  → 10cm pallet preserved ✓
```

## Parameters

### Input/Output Topics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_cloud_topic` | `/camera/depth/color/points` | Input point cloud topic (camera optical frame) |
| `output_floor_cloud_topic` | `/floor_cloud` | Output topic for floor points (robot frame, original resolution) |
| `output_no_floor_cloud_topic` | `/no_floor_cloud` | Output topic for non-floor points (robot frame, original resolution) |
| `output_floor_cloud_voxelized_topic` | `/floor_cloud_voxelized` | Output topic for floor points (voxelized) |
| `output_no_floor_cloud_voxelized_topic` | `/no_floor_cloud_voxelized` | Output topic for non-floor points (voxelized) |

### Floor Detection Mode

| Parameter | Default | Description |
|-----------|---------|-------------|
| `auto_floor_detection_mode` | `true` | `true`: Auto-detect floor using z_min<br>`false`: Use fixed floor height |
| `floor_height` | `0.0` | Floor Z coordinate in robot frame (meters)<br>Used only when `auto_floor_detection_mode: false` |

**Mode Details**:
- **Auto Mode** (`true`): Automatically finds the lowest point (z_min) in the scene. Best for testing and dynamic environments.
- **Fixed Mode** (`false`): Uses the configured `floor_height` value. Best for production environments with known floor positions.

### RANSAC Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ransac_distance_threshold` | `0.02` | Distance threshold for RANSAC inliers (meters) |
| `ransac_max_iterations` | `100` | Maximum iterations for RANSAC |
| `floor_normal_z_threshold` | `0.7` | Minimum Z component of normal (robot frame: Z=up)<br>For horizontal floor, nz should be close to 1.0 |

The normal threshold ensures the detected plane is approximately horizontal (floor-like, not wall-like).

### Floor Region Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `floor_detection_thickness` | `0.15` | Thickness for RANSAC plane detection (meters)<br>**Wider region** to get sufficient points after voxelization |
| `floor_removal_thickness` | `0.03` | Thickness for actual floor removal (meters)<br>**Thin layer** to preserve objects on floor (e.g., 10cm pallets) |
| `floor_margin` | `0.02` | Additional margin around detected plane (meters) |

**Tuning Guide**:
- If RANSAC fails (too few points): Increase `floor_detection_thickness`
- If objects on floor are removed: Decrease `floor_removal_thickness`
- If floor edges are missed: Increase `floor_margin`

### Voxel Grid Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_voxel_grid` | `true` | Enable voxel grid downsampling |
| `voxel_leaf_size` | `0.03` | Voxel size (meters)<br>3cm reduces ~100k points to ~10-15k points |

**Trade-offs**:
- Smaller leaf size (0.01): More detail, more computation
- Larger leaf size (0.05): Faster, less detail, may miss small features

### Camera Extrinsic Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_default_transform` | `true` | `true`: Use only default optical→base transform<br>`false`: Apply additional extrinsic on top |
| `cam_tx` | `0.0` | Camera X offset in robot frame (meters, forward) |
| `cam_ty` | `0.0` | Camera Y offset in robot frame (meters, left) |
| `cam_tz` | `0.0` | Camera Z offset in robot frame (meters, up) |
| `cam_roll` | `0.0` | Rotation around X axis (radians) |
| `cam_pitch` | `0.0` | Rotation around Y axis (radians) |
| `cam_yaw` | `0.0` | Rotation around Z axis (radians) |

**How it works**:
1. **Default transform** (always applied): Camera optical → Robot base
   - Converts axes: (X=right, Y=down, Z=forward) → (X=forward, Y=left, Z=up)
2. **Additional extrinsic** (only if `use_default_transform: false`):
   - Applied on top of default transform
   - Accounts for camera mounting position and orientation

**Example configurations**:

```yaml
# Example 1: Camera at robot origin (default)
use_default_transform: true

# Example 2: Camera 15cm above robot base
use_default_transform: false
cam_tx: 0.0
cam_ty: 0.0
cam_tz: 0.15  # 15cm up

# Example 3: Camera 10cm forward, 15cm up, tilted 10° down
use_default_transform: false
cam_tx: 0.10      # 10cm forward
cam_ty: 0.0
cam_tz: 0.15      # 15cm up
cam_roll: 0.0
cam_pitch: -0.1745  # -10 degrees in radians
cam_yaw: 0.0
```

### Stringer Detection Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_stringer_detection` | `true` | Enable pallet stringer detection |
| `stringer_width_min` | `0.05` | Minimum stringer width (meters, 5cm) |
| `stringer_width_max` | `0.10` | Maximum stringer width (meters, 10cm) |
| `stringer_height_min` | `0.10` | Minimum stringer height (meters, 10cm) |
| `stringer_height_max` | `0.20` | Maximum stringer height (meters, 20cm) |

**How stringer detection works**:
1. After floor removal, remaining points are clustered
2. Each cluster's bounding box is computed
3. Clusters with dimensions matching stringer criteria are identified
4. In robot frame: height = Z dimension, width = X or Y dimension

## Usage

### Installation

```bash
# Install dependencies
sudo apt-get install ros-humble-pcl-ros ros-humble-pcl-conversions

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
ros2 launch floor_removal_rgbd floor_removal.launch.py \
  config_file:=/path/to/custom_params.yaml
```

### With RealSense Camera

```bash
# Terminal 1: Launch RealSense camera
ros2 launch realsense2_camera rs_launch.py \
  enable_rgbd:=true \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  pointcloud.enable:=true

# Terminal 2: Launch floor removal
ros2 launch floor_removal_rgbd floor_removal.launch.py
```

### Visualize in RViz

```bash
# Open RViz
rviz2

# Add displays:
# - PointCloud2: /camera/depth/color/points (original, camera optical frame)
# - PointCloud2: /no_floor_cloud (scene without floor, robot base frame)
# - PointCloud2: /floor_cloud (detected floor, robot base frame)
# - MarkerArray: /stringer_markers (detected stringer bounding boxes)
# - PointCloud2: /stringer_centers (stringer center points)

# Set Fixed Frame to: camera_link or base_link
```

### Parameter Tuning

Edit `config/params.yaml` and restart the node:

```yaml
floor_removal_node:
  ros__parameters:
    # Switch to fixed floor height mode (production)
    auto_floor_detection_mode: false
    floor_height: 0.0              # Floor at Z=0 in robot frame

    # Adjust detection thickness if RANSAC fails
    floor_detection_thickness: 0.20  # Increase to 20cm

    # Adjust removal thickness based on object height
    floor_removal_thickness: 0.05    # Increase to 5cm for thicker floor layer

    # Configure camera extrinsic (if camera not at robot origin)
    use_default_transform: false
    cam_tz: 0.15                   # Camera 15cm above robot base
```

## Examples

### Example 1: Testing with Auto Detection

**Scenario**: Testing and calibration with dynamic floor detection

**Configuration**:
```yaml
auto_floor_detection_mode: true  # Auto-detect floor
use_voxel_grid: true
voxel_leaf_size: 0.03
use_default_transform: true
```

**Result**: Automatically finds floor in varying environments ✓

### Example 2: Production with Fixed Floor Height

**Scenario**: Production environment with known floor position

**Configuration**:
```yaml
auto_floor_detection_mode: false
floor_height: 0.0                # Floor at robot base level
floor_detection_thickness: 0.15
floor_removal_thickness: 0.03
use_default_transform: true
```

**Result**: Consistent floor removal without dynamic detection overhead ✓

### Example 3: Camera Mounted Above Robot

**Scenario**: Camera mounted 15cm above robot base

**Configuration**:
```yaml
use_default_transform: false
cam_tx: 0.0
cam_ty: 0.0
cam_tz: 0.15                     # 15cm above base

# Floor is now 15cm below camera
auto_floor_detection_mode: false
floor_height: -0.15              # Floor 15cm below camera origin
```

**Result**: Correct floor detection with camera offset ✓

### Example 4: Preserving Pallets and Detecting Stringers

**Scenario**: Detect floor but preserve 10cm pallets, detect stringers

**Configuration**:
```yaml
auto_floor_detection_mode: true
floor_detection_thickness: 0.15  # Wide enough for RANSAC
floor_removal_thickness: 0.03    # Thin removal (< 10cm)
floor_margin: 0.02

# Enable stringer detection
enable_stringer_detection: true
stringer_width_min: 0.05
stringer_width_max: 0.10
stringer_height_min: 0.10
stringer_height_max: 0.20
```

**Result**: Floor removed, pallets preserved, stringers detected ✓

### Example 5: Performance Optimization

**Scenario**: 30Hz processing required

**Configuration**:
```yaml
use_voxel_grid: true
voxel_leaf_size: 0.05            # Aggressive downsampling
ransac_max_iterations: 50        # Fewer iterations
enable_stringer_detection: false # Disable for speed
```

**Result**: Faster processing with acceptable accuracy ✓

## Troubleshooting

### Issue: "No floor plane detected"

**Possible Causes**:
1. Floor region too thin → insufficient points for RANSAC
2. Floor not visible in frame
3. Floor not horizontal (normal check fails)
4. Floor normal threshold too high

**Solutions**:
- Increase `floor_detection_thickness` to 0.2m or 0.3m
- Check camera view (floor should be visible)
- Decrease `floor_normal_z_threshold` to 0.5 if floor is tilted
- Enable debug logs to see min_z and floor region point count

### Issue: RANSAC errors or unstable detection

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
- Increase `floor_margin` to 0.03m or 0.04m
- Slightly increase `floor_removal_thickness`

### Issue: Coordinate frame confusion

**Problem**: Point clouds appear in wrong location

**Solutions**:
- Check `use_default_transform` setting
- Verify camera extrinsic parameters (tx, ty, tz)
- Use `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` to check transforms
- Visualize in RViz with correct Fixed Frame

### Issue: Stringers not detected

**Possible Causes**:
1. Stringer dimensions outside configured range
2. Floor removal also removed stringers
3. Clustering parameters too strict

**Solutions**:
- Adjust `stringer_width_min/max` and `stringer_height_min/max`
- Decrease `floor_removal_thickness` to preserve stringers
- Check debug logs for rejected clusters

## Performance

Typical performance on Intel Core i5:
- Input: 120k points (camera optical frame)
- After transformation: 120k points (robot base frame)
- After voxelization (3cm): 10-12k points
- Processing time: 30-40ms (~25-30Hz)
- With stringer detection: +10-15ms

## Published Topics

- `/floor_cloud` (sensor_msgs/PointCloud2): Detected floor points (robot frame)
- `/no_floor_cloud` (sensor_msgs/PointCloud2): Scene with floor removed (robot frame)
- `/floor_cloud_voxelized` (sensor_msgs/PointCloud2): Floor points (voxelized)
- `/no_floor_cloud_voxelized` (sensor_msgs/PointCloud2): Non-floor points (voxelized)
- `/stringer_markers` (visualization_msgs/MarkerArray): Bounding boxes for detected stringers
- `/stringer_centers` (sensor_msgs/PointCloud2): Center points of detected stringers

## Subscribed Topics

- `/camera/depth/color/points` (sensor_msgs/PointCloud2): Input point cloud (camera optical frame)

## Dependencies

- ROS2 Humble or later
- PCL (Point Cloud Library)
- Eigen3
- sensor_msgs
- geometry_msgs
- visualization_msgs

## Technical Notes

### Coordinate Frame Convention

The package follows ROS REP-103 and REP-105:
- **Camera optical frame**: X=right, Y=down, Z=forward (standard camera convention)
- **Robot base frame**: X=forward, Y=left, Z=up (standard robot convention)

### Why Robot Frame Processing?

Processing in robot frame provides:
1. **Intuitive parameters**: "Floor at Z=0" instead of "Floor at Y=0.8"
2. **Easy calibration**: Measure camera height directly
3. **Operator-friendly**: Field technicians can understand Z-height settings
4. **Consistent semantics**: Height = Z coordinate (natural interpretation)

### Extrinsic Transform Details

The transform pipeline:
1. Input: Camera optical frame points
2. Apply default optical→base transform (axis remapping)
3. If `use_default_transform: false`, apply additional extrinsic:
   - Rotation (Euler ZYX: Yaw × Pitch × Roll)
   - Translation
4. Output: Robot base frame points

## Future Improvements

- [ ] Support for non-horizontal floors (inclined surfaces)
- [ ] Multi-plane detection (stairs, ramps)
- [ ] Integration with TF2 for automatic extrinsic calculation
- [ ] Dynamic reconfigure support
- [ ] GPU acceleration for large point clouds

## License

MIT

## Authors

cwon789 (wonyoung chung)

## References

- RANSAC: Fischler & Bolles, "Random Sample Consensus", 1981
- PCL: Point Cloud Library - https://pointclouds.org/
- ROS REP-103: Standard Units of Measure and Coordinate Conventions
- ROS REP-105: Coordinate Frames for Mobile Platforms
