# Floor Removal RGBD

A ROS2 package for detecting and removing floor planes from RGBD point clouds using RANSAC plane segmentation. Includes vertical wall detection and pallet candidate filtering for warehouse automation.

## Overview

This package processes point clouds from RGBD cameras (e.g., Intel RealSense D435i) to:
1. Detect and separate floor planes from the scene
2. Detect vertical walls (YZ planes) perpendicular to the forward direction
3. Filter pallet candidates based on proximity to detected walls
4. Detect pallet stringers (wooden support beams) in the remaining cloud

The algorithm uses a multi-stage approach:
1. **Floor Detection**: Identifies the floor plane using RANSAC with dual-thickness strategy
2. **Floor Removal**: Removes only a thin layer around the detected plane to preserve objects on the floor
3. **Wall Detection**: Detects vertical planes (walls) using cluster-based RANSAC
4. **Pallet Candidate Filtering**: Extracts points within configurable distance from detected walls

This design allows the algorithm to:
- Robustly detect floor planes with sufficient sample points
- Preserve low-height objects on the floor (e.g., 10cm pallets)
- Detect vertical walls at any angle and orientation
- Filter potential pallet locations based on wall proximity
- Detect pallet stringers for automated forklift navigation
- Work in robot coordinate frame for intuitive parameter tuning

## Features

### Core Features
- **Robot Coordinate Frame Processing**: Processes in robot base frame (X=forward, Y=left, Z=up) for intuitive height-based filtering
- **Fixed Floor Height Mode**: Uses fixed floor height for consistent floor removal
- **Camera Extrinsic Support**: Configurable camera position and orientation
- **Dual-Thickness Strategy**: Separate parameters for detection and removal
- **Voxel Grid Downsampling**: Reduces computational cost and filters noise
- **Multiple Output Topics**: Publishes both original and voxelized point clouds

### Wall Detection Features (NEW)
- **YZ Plane Detection**: Detects vertical walls perpendicular to X-axis (forward direction)
- **Cluster-based Segmentation**: Separates objects before plane fitting for robust multi-wall detection
- **Accurate Plane Orientation**: Uses proper coordinate system to avoid projection artifacts
- **Stable Visualization**: Pitch-stabilized plane markers that don't flicker
- **Configurable Thickness**: Adjustable plane thickness visualization (unidirectional or bidirectional)
- **Pallet Candidate Filtering**: Publishes points within plane thickness for downstream processing

### Stringer Detection Features
- **Pallet Stringer Detection**: Detects wooden support beams using clustering and dimensional analysis
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
│   └── server_node.cpp         # ROS2 node + YZ plane detection + main()
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
- `extractFloorRegionRobotFrame()`: Extracts candidate floor points
- `detectFloorPlane()`: Runs RANSAC to detect floor plane
- `isPlaneValid()`: Validates detected plane
- `classifyPoints()`: Separates floor and non-floor points

#### `FloorRemovalServerNode`
ROS2 node that provides the subscription/publishing interface and implements YZ plane detection.

**Responsibilities:**
- Load parameters from ROS2 parameter server
- Subscribe to point cloud topics
- Call PlaneRemover for floor processing
- Perform cluster-based YZ plane detection
- Filter pallet candidates based on wall proximity
- Publish separated floor and non-floor clouds
- Publish wall visualization markers and pallet candidates
- Publish stringer detection results (markers and centers)

#### `StringerDetector`
Detects pallet stringers using Euclidean clustering and dimensional analysis.

**Key Methods:**
- `detect()`: Main detection pipeline
- `clusterPointCloud()`: Performs Euclidean clustering
- `computeBoundingBox()`: Computes bounding box for each cluster
- `matchesStringerCriteria()`: Validates cluster dimensions

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
   - Configurable leaf size (default: 2cm)

3. **Floor Detection and Removal**
   - Uses configured `floor_height` value in robot frame (Z coordinate)
   - Extracts floor region with `floor_detection_thickness` (wider for RANSAC)
   - Runs RANSAC plane fitting with normal validation
   - Removes thin layer with `floor_removal_thickness` (preserves objects)

4. **YZ Plane Detection** (Vertical Walls)
   - Euclidean clustering to separate objects (10cm tolerance, 50-25000 points)
   - For each cluster:
     - RANSAC with perpendicular plane model (axis constraint on X-axis)
     - Normal validation (|nx| > threshold, typically 0.3-0.7)
     - Extract plane inliers and compute bounds
   - Plane coordinate system creation:
     - Forces Y-axis to be horizontal (stable orientation)
     - Z-axis aligned with plane normal
     - Prevents pitch flickering by using consistent reference
   - Visualization marker generation:
     - Position: centroid of detected plane
     - Orientation: quaternion from plane coordinate system
     - Size: actual extent of points projected onto plane axes
     - Thickness: configurable offset in normal direction

5. **Pallet Candidate Filtering**
   - For each detected wall plane:
     - Calculate signed distance from each point to plane
     - Apply thickness filter (bidirectional or unidirectional mode)
     - Apply spatial extent filter (within plane XY bounds + margin)
   - Accumulate candidates from all detected planes
   - Publish combined point cloud

6. **Stringer Detection** (optional)
   - Clusters remaining points using Euclidean clustering
   - Analyzes cluster dimensions (width, height, depth)
   - Identifies clusters matching stringer criteria

### Dual-Thickness Strategy (Floor)

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

### YZ Plane Detection Algorithm

**Goal**: Detect vertical walls at any orientation, not just axis-aligned.

**Key Innovation**: Plane-fitted coordinate system instead of axis projection.

**Problem with Naive Approach**:
```
❌ Old approach: Project all points to YZ plane (X=0)
   - Rotated walls appear as if projected onto X=0
   - Loses actual wall orientation and position
   - Cannot distinguish between rotated wall and straight wall
```

**Solution - Proper 3D Plane Fitting**:
```
✓ New approach:
  1. Cluster points to separate objects (each cluster may be a wall)
  2. For each cluster, fit plane with RANSAC
     - Use SACMODEL_PERPENDICULAR_PLANE with X-axis constraint
     - Allows walls at any angle, not just X=0
  3. Extract plane normal (nx, ny, nz) and centroid
  4. Create plane-local coordinate system:
     - Z-axis = normal direction (perpendicular to wall)
     - Y-axis = horizontal (forced to XY plane for stability)
     - X-axis = Y × Z (completes right-handed system)
  5. Project points onto plane-local axes to get actual extent
  6. Visualize with proper orientation (not axis-aligned)
```

**Thickness Modes**:

1. **Bidirectional Mode** (`yz_plane_marker_bidirectional: true`)
   ```
   Detected plane position (fixed)
         │
    ←───┼───→  Extends equally in both normal directions
   -t/2 │ +t/2
   ```
   - Marker center stays at detected plane centroid
   - Extends ±(thickness/2) from plane
   - Sign of thickness parameter ignored
   - Use case: Symmetric search around detected wall

2. **Unidirectional Mode** (`yz_plane_marker_bidirectional: false`)
   ```
   Positive thickness:              Negative thickness:
   Plane ───→                       ←─── Plane
   │    +t                          -t   │
   │                                     │
   Extends in +normal direction     Extends in -normal direction
   ```
   - Marker center shifts by thickness/2
   - One face of marker stays at detected plane
   - Sign controls direction
   - Use case: Search only on one side of wall (e.g., pallets against wall)

**Pallet Candidate Filtering**:
```
For each point in no_floor_cloud_voxelized:
  1. Calculate signed distance to plane: d = nx*x + ny*y + nz*z + d0
  2. Check thickness bounds:
     - Bidirectional: |d| <= thickness/2 + 0.01
     - Unidirectional (+): -0.01 <= d <= thickness + 0.01
     - Unidirectional (-): thickness - 0.01 <= d <= 0.01
  3. Check spatial extent (within plane XY bounds + 10cm margin)
  4. If both pass, add to pallet_candidates
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

### Floor Detection Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `floor_height` | `0.0` | Floor Z coordinate in robot frame (meters) |
| `ransac_distance_threshold` | `0.02` | Distance threshold for RANSAC inliers (meters) |
| `ransac_max_iterations` | `100` | Maximum iterations for RANSAC |
| `floor_normal_z_threshold` | `0.7` | Minimum Z component of normal (robot frame: Z=up)<br>For horizontal floor, nz should be close to 1.0 |
| `floor_detection_thickness` | `0.15` | Thickness for RANSAC plane detection (meters)<br>Wider region to get sufficient points |
| `floor_removal_thickness` | `0.03` | Thickness for actual floor removal (meters)<br>Thin layer to preserve objects |
| `floor_margin` | `0.02` | Additional margin around detected plane (meters) |

### YZ Plane Detection Parameters (NEW)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_yz_plane_detection` | `true` | Enable vertical wall (YZ plane) detection |
| `yz_plane_distance_threshold` | `0.03` | RANSAC distance threshold for wall detection (meters)<br>Points within this distance are considered inliers |
| `yz_plane_max_iterations` | `200` | Maximum RANSAC iterations for wall detection |
| `yz_plane_normal_x_threshold` | `0.3` | Minimum absolute X component of normal<br>Higher = more perpendicular to X-axis required<br>Range: 0.3 (lenient) to 0.9 (strict) |
| `yz_plane_marker_thickness` | `0.0` | Visualization marker thickness extension (meters)<br>**Bidirectional mode**: Extends ±thickness/2 from plane<br>**Unidirectional mode**:<br>  - Positive: Extends in +normal direction<br>  - Negative: Extends in -normal direction<br>Does NOT affect plane detection, only visualization |
| `yz_plane_marker_bidirectional` | `false` | Thickness extension mode<br>`false`: Unidirectional (sign matters)<br>`true`: Bidirectional (extends both ways, sign ignored) |

**YZ Plane Parameter Tuning**:
- **Missing walls**: Decrease `yz_plane_normal_x_threshold` (e.g., 0.3)
- **Too many false detections**: Increase `yz_plane_normal_x_threshold` (e.g., 0.7)
- **Noisy wall planes**: Increase `yz_plane_distance_threshold` (e.g., 0.05)
- **Want symmetric search**: Set `yz_plane_marker_bidirectional: true`
- **Want one-sided search**: Set `yz_plane_marker_bidirectional: false` and adjust `yz_plane_marker_thickness` sign

### Voxel Grid Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_voxel_grid` | `true` | Enable voxel grid downsampling |
| `voxel_leaf_size` | `0.02` | Voxel size (meters)<br>2cm reduces ~100k points to ~10-15k points |

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

**Example - Camera mounted 35cm above robot base**:
```yaml
use_default_transform: false
cam_tx: 0.0
cam_ty: 0.0
cam_tz: 0.35      # 35cm up
```

## Usage

### Installation

```bash
# Install dependencies
sudo apt-get install ros-humble-pcl-ros ros-humble-pcl-conversions

# Build the package
cd ~/catkin_1014
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
# - MarkerArray: /yz_plane_markers (detected wall planes, blue boxes)
# - PointCloud2: /pallet_candidates (points near walls)
# - MarkerArray: /stringer_markers (detected stringer bounding boxes)
# - PointCloud2: /stringer_centers (stringer center points)

# Set Fixed Frame to: camera_link or base_link
```

## Examples

### Example 1: Warehouse with Pallets Against Walls

**Scenario**: Detect walls and find pallets placed against them

**Configuration**:
```yaml
# Floor detection
floor_height: 0.0
floor_detection_thickness: 0.15
floor_removal_thickness: 0.03

# Camera mounted 35cm above base
use_default_transform: false
cam_tz: 0.35

# Wall detection with one-sided search
enable_yz_plane_detection: true
yz_plane_distance_threshold: 0.03
yz_plane_normal_x_threshold: 0.5
yz_plane_marker_thickness: -0.05     # 5cm toward camera
yz_plane_marker_bidirectional: false # One-sided
```

**Result**:
- Floor removed ✓
- Walls detected at any angle ✓
- Pallet candidates within 5cm of walls ✓

### Example 2: Multi-Wall Detection with Symmetric Search

**Scenario**: Detect multiple walls and search both sides

**Configuration**:
```yaml
# Wall detection with bidirectional search
enable_yz_plane_detection: true
yz_plane_distance_threshold: 0.03
yz_plane_normal_x_threshold: 0.3     # Lenient threshold
yz_plane_marker_thickness: 0.10      # 10cm total (±5cm)
yz_plane_marker_bidirectional: true  # Both sides
```

**Result**:
- Multiple walls detected ✓
- Search ±5cm from each wall ✓
- Works with rotated walls ✓

### Example 3: Performance Optimization

**Scenario**: 30Hz processing required

**Configuration**:
```yaml
use_voxel_grid: true
voxel_leaf_size: 0.05            # Aggressive downsampling
ransac_max_iterations: 50        # Fewer iterations
yz_plane_max_iterations: 100     # Fewer wall RANSAC iterations
```

**Result**: Faster processing with acceptable accuracy ✓

## Published Topics

- `/floor_cloud` (sensor_msgs/PointCloud2): Detected floor points (robot frame)
- `/no_floor_cloud` (sensor_msgs/PointCloud2): Scene with floor removed (robot frame)
- `/floor_cloud_voxelized` (sensor_msgs/PointCloud2): Floor points (voxelized)
- `/no_floor_cloud_voxelized` (sensor_msgs/PointCloud2): Non-floor points (voxelized)
- `/yz_plane_markers` (visualization_msgs/MarkerArray): Detected wall plane markers (blue boxes)
- `/pallet_candidates` (sensor_msgs/PointCloud2): Points within plane thickness (for pallet detection)
- `/stringer_markers` (visualization_msgs/MarkerArray): Bounding boxes for detected stringers
- `/stringer_centers` (sensor_msgs/PointCloud2): Center points of detected stringers

## Subscribed Topics

- `/camera/depth/color/points` (sensor_msgs/PointCloud2): Input point cloud (camera optical frame)

## Troubleshooting

### Issue: "No floor plane detected"

**Possible Causes**:
1. Floor region too thin → insufficient points for RANSAC
2. Floor not visible in frame
3. Incorrect `floor_height` value

**Solutions**:
- Increase `floor_detection_thickness` to 0.2m or 0.3m
- Check camera view (floor should be visible)
- Verify `floor_height` matches your camera extrinsic setup

### Issue: Walls not detected

**Possible Causes**:
1. Wall normal not perpendicular enough to X-axis
2. Clustering separated wall into multiple small clusters
3. Insufficient inlier points

**Solutions**:
- Decrease `yz_plane_normal_x_threshold` to 0.3 (more lenient)
- Increase `yz_plane_distance_threshold` to 0.05 (more tolerance)
- Check cluster sizes in debug logs

### Issue: Wall markers are flickering/rotating

**Cause**: This should not happen with the current implementation (stable Y-axis).

**If it occurs**:
- Check that the code uses horizontal Y-axis stabilization
- Verify plane normal is being computed correctly
- Check debug logs for normal values

### Issue: Pallet candidates include too many points

**Cause**: Thickness too large or spatial extent filter too loose

**Solutions**:
- Decrease `yz_plane_marker_thickness` (e.g., 0.02 instead of 0.05)
- Code uses 10cm margin - can be adjusted in source if needed

### Issue: Objects on floor are removed

**Cause**: Removal thickness too large

**Solutions**:
- Decrease `floor_removal_thickness` to 0.02m or 0.01m
- Ensure objects are at least 2x removal thickness height

## Performance

Typical performance on Intel Core i5:
- Input: 120k points (camera optical frame)
- After voxelization (2cm): 10-12k points
- Floor detection + removal: 20-30ms
- YZ plane detection (3-5 clusters): +15-25ms
- Pallet candidate filtering: +5-10ms
- Total: 40-65ms (~15-25Hz)

## Dependencies

- ROS2 Humble or later
- PCL (Point Cloud Library)
- Eigen3
- sensor_msgs
- geometry_msgs
- visualization_msgs

## Technical Notes

### Why Cluster-Based Wall Detection?

**Alternative approach**: Run RANSAC on entire no_floor_cloud
- ❌ Problem: Finds dominant plane only (misses other walls)
- ❌ Problem: Iterative removal is slow and unreliable

**Our approach**: Cluster first, then detect plane in each cluster
- ✓ Detects multiple walls simultaneously
- ✓ Each cluster likely represents one object/wall
- ✓ More robust to outliers
- ✓ Better for warehouse environments with multiple walls

### Coordinate Frame Convention

The package follows ROS REP-103 and REP-105:
- **Camera optical frame**: X=right, Y=down, Z=forward
- **Robot base frame**: X=forward, Y=left, Z=up

### Why Robot Frame Processing?

Processing in robot frame provides:
1. **Intuitive parameters**: "Floor at Z=0" instead of "Floor at Y=0.8"
2. **Easy calibration**: Measure camera height directly
3. **Operator-friendly**: Field technicians can understand Z-height settings
4. **Consistent semantics**: Height = Z coordinate (natural interpretation)

## License

MIT

## Authors

cwon789 (wonyoung chung)

## References

- RANSAC: Fischler & Bolles, "Random Sample Consensus", 1981
- PCL: Point Cloud Library - https://pointclouds.org/
- ROS REP-103: Standard Units of Measure and Coordinate Conventions
- ROS REP-105: Coordinate Frames for Mobile Platforms
