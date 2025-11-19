# Floor Removal RGBD

A ROS2 package for robust floor plane detection and removal from RGBD/ToF point clouds using RANSAC-based plane segmentation with automatic floor detection.

## Overview

This package processes point clouds from depth cameras (e.g., ToF cameras, Intel RealSense) to detect and remove floor planes from the scene. Designed for warehouse automation and mobile robot applications, particularly for FMR (Forklift Mobile Robot) systems that need to detect pallets and objects while filtering out the floor.

**Key Features:**
- **Automatic Floor Detection**: Uses percentile-based floor height estimation to handle robot pitch/roll
- **Fixed Mode Support**: Option to use fixed floor height for consistent results
- **ToF Shadow Filtering**: Removes physically impossible points (z < 0) caused by ToF sensor artifacts
- **Noise Removal**: Intelligent post-processing to remove isolated floor remnants
- **Real-time Performance**: Optimized O(N) algorithms for fast processing
- **Warehouse-Ready**: Handles uneven floors, robot tilt, and challenging industrial environments

## Architecture

### File Structure

```
floor_removal_rgbd/
├── include/floor_removal_rgbd/
│   ├── plane_remover.hpp       # Core floor removal algorithm
│   └── server_node.hpp         # ROS2 node interface
├── src/
│   ├── plane_remover.cpp       # Floor removal implementation
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
- `applyVoxelGrid()`: Downsamples point cloud for performance
- `filterByValidZ()`: Removes ToF shadow artifacts (z < min_valid_z)
- `findMinZPercentile()`: Auto mode - finds floor using percentiles
- `extractFloorRegion()`: Extracts candidate floor points
- `detectFloorPlane()`: Runs RANSAC to detect floor plane
- `isPlaneValid()`: Validates detected plane normal
- `removePointsBelowPlane()`: Separates floor and non-floor points
- `removeFloorNoise()`: Post-processing to remove isolated floor remnants

#### `FloorRemovalServerNode`
ROS2 node that provides the subscription/publishing interface.

**Responsibilities:**
- Load parameters from ROS2 parameter server
- Subscribe to point cloud topics
- Call PlaneRemover for floor processing
- Publish separated floor and non-floor clouds
- Handle state change logging (plane found/lost)

## Mathematical Algorithm

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

**Transform:**
```
x_robot = z_camera
y_robot = -x_camera
z_robot = -y_camera
```

Since Z points upward, the floor has the **minimum Z value** (lowest points).

### Processing Pipeline

#### 1. Coordinate Transformation

Given a point `p_cam = (x_c, y_c, z_c)` in camera optical frame, transform to robot base frame:

```
p_base = (x_b, y_b, z_b) where:
  x_b = z_c    (forward = depth)
  y_b = -x_c   (left = -right)
  z_b = -y_c   (up = -down)
```

If custom extrinsics are enabled (`use_default_transform: false`), apply additional rotation and translation:

```
R = R_z(yaw) * R_y(pitch) * R_x(roll)
p_final = R * p_base + t

where t = (cam_tx, cam_ty, cam_tz)
```

#### 2. Voxel Grid Downsampling

Reduces point cloud density while preserving geometric structure:

```
For each point p = (x, y, z):
  voxel_index = (⌊x/L⌋, ⌊y/L⌋, ⌊z/L⌋)

  Group all points in same voxel
  Replace group with centroid

where L = voxel_leaf_size (default: 0.02m)
```

**Effect**: Reduces ~300k points to ~5-10k points while filtering noise.

#### 3. ToF Shadow Filtering

ToF cameras can produce erroneous points below the physical floor (z < 0):

```
For each point p = (x, y, z):
  if z < min_valid_z:
    discard point
```

**Default**: `min_valid_z = 0.0` (robot base is at ground level)

#### 4. Automatic Floor Detection (Auto Mode)

**Problem**: Fixed floor height doesn't work when robot has pitch/roll or floor is uneven.

**Solution**: Use percentile-based floor estimation.

Given point cloud P = {p₁, p₂, ..., pₙ} with z-coordinates Z = {z₁, z₂, ..., zₙ}:

```
Step 1: Sort Z in ascending order
  Z_sorted = sort(Z)

Step 2: Find minimum floor height (lowest p%)
  min_z = Z_sorted[⌊n × p_min / 100⌋]

  where p_min = auto_floor_percentile (default: 1.0%)

Step 3: Find maximum floor height (lowest q%)
  max_z = Z_sorted[⌊n × p_max / 100⌋]

  where p_max = auto_floor_max_percentile (default: 10.0%)

Step 4: Clamp to valid range
  min_z = clamp(min_z, min_valid_z, max_floor_z)
  max_z = clamp(max_z, min_z + floor_detection_thickness, max_floor_z + 0.15)
```

**Why percentiles?**
- **Robust to outliers**: Ignores ToF shadows and high objects
- **Adaptive to tilt**: Automatically adjusts to robot pitch/roll
- **Handles uneven floors**: Uses statistical distribution instead of absolute values

**Example:**
```
Robot tilted 5° forward → floor appears at different z-heights
Without percentiles: Fixed z=0.0 fails
With percentiles: Uses lowest 1% of points → robust ✓
```

#### 5. Floor Region Extraction

Extract points likely to be part of the floor plane:

```
For each point p = (x, y, z):
  if min_z ≤ z ≤ max_z:
    add p to floor_region
```

This creates a **thick slice** containing the floor and nearby objects.

#### 6. RANSAC Plane Detection

**Goal**: Find the best-fit plane through floor_region points.

**Plane Equation**:
```
n · p + d = 0

where:
  n = (nx, ny, nz) is the unit normal vector
  d is the signed distance from origin
  p = (x, y, z) is any point on the plane
```

Expanded form:
```
nx·x + ny·y + nz·z + d = 0
```

**RANSAC Algorithm**:
```
best_inliers = 0
best_plane = null

for i = 1 to ransac_max_iterations:
  Step 1: Randomly sample 3 points from floor_region
    p1, p2, p3 ~ uniform(floor_region)

  Step 2: Compute plane from 3 points
    v1 = p2 - p1
    v2 = p3 - p1
    n = normalize(v1 × v2)  // cross product
    d = -n · p1

  Step 3: Count inliers (points close to plane)
    inliers = 0
    for each point p in floor_region:
      distance = |n · p + d| / ||n||
      if distance < ransac_distance_threshold:
        inliers++

  Step 4: Keep best plane
    if inliers > best_inliers:
      best_inliers = inliers
      best_plane = (n, d)

return best_plane
```

**Distance Calculation**:
```
For a point p = (x, y, z) and plane (n, d):

  signed_distance = nx·x + ny·y + nz·z + d

  unsigned_distance = |signed_distance|
```

**Inlier Threshold**:
- `ransac_distance_threshold = 0.03m` means points within ±3cm are considered part of the plane.

#### 7. Plane Validation

Check if detected plane is actually a horizontal floor:

```
In robot frame (Z = up):
  A horizontal floor has n ≈ (0, 0, ±1)

Validation:
  if |nz| < floor_normal_z_threshold:
    reject plane (not horizontal enough)
```

**Default**: `floor_normal_z_threshold = 0.7`
- `nz = 1.0`: Perfectly horizontal
- `nz = 0.7`: ~45° tilt (acceptable for uneven floors)
- `nz = 0.0`: Vertical (reject)

#### 8. Floor Point Separation

Classify each point as floor or non-floor based on signed distance to plane:

```
For each point p = (x, y, z) in original cloud:

  Step 1: Calculate signed distance to plane
    d_signed = nx·x + ny·y + nz·z + d₀

  Step 2: Calculate removal threshold
    In robot frame (Z=up, floor below), points BELOW plane are floor:

    threshold_lower = -(floor_removal_thickness/2 + floor_margin)
    threshold_upper = +(floor_removal_thickness/2 + floor_margin)

  Step 3: Classify point
    if threshold_lower ≤ d_signed ≤ threshold_upper:
      add p to floor_cloud
    else:
      add p to no_floor_cloud
```

**Why signed distance?**
```
d_signed > 0: Point is ABOVE the plane (toward +normal direction)
d_signed < 0: Point is BELOW the plane (toward -normal direction)
d_signed = 0: Point is ON the plane
```

**Example** (floor at z=0, normal n=(0,0,1), d=0):
```
Point p = (1, 2, 0.01):
  d_signed = 0·1 + 0·2 + 1·0.01 + 0 = 0.01 (above floor)

Point p = (1, 2, -0.02):
  d_signed = 0·1 + 0·2 + 1·(-0.02) + 0 = -0.02 (below floor)

With floor_removal_thickness = 0.05, floor_margin = 0.01:
  threshold_lower = -(0.05/2 + 0.01) = -0.035
  threshold_upper = +(0.05/2 + 0.01) = +0.035

  p = (1, 2, 0.01): -0.035 ≤ 0.01 ≤ 0.035 → FLOOR ✓
  p = (1, 2, -0.02): -0.035 ≤ -0.02 ≤ 0.035 → FLOOR ✓
  p = (1, 2, 0.10): 0.10 > 0.035 → NOT FLOOR ✓
```

#### 9. Noise Removal Post-Processing

After floor removal, small isolated floor remnants may remain due to noise.

**Algorithm**: 3x3x3 voxel neighborhood filtering

```
Given voxel size v_size = noise_radius_search (default: 0.05m)

Step 1: Build voxel occupancy map
  For each point p = (x, y, z) near floor (z < floor_z + margin):
    voxel_idx = (⌊x/v_size⌋, ⌊y/v_size⌋, ⌊z/v_size⌋)
    voxel_count[hash(voxel_idx)]++

Step 2: Filter based on neighborhood density
  For each point p near floor:
    voxel_idx = (ix, iy, iz) for point p

    // Count points in 3x3x3 neighborhood (27 voxels)
    neighbor_count = 0
    occupied_voxels = 0

    for dx = -1 to +1:
      for dy = -1 to +1:
        for dz = -1 to +1:
          neighbor_idx = (ix+dx, iy+dy, iz+dz)
          if voxel_count[hash(neighbor_idx)] > 0:
            neighbor_count += voxel_count[hash(neighbor_idx)]
            occupied_voxels++

    // Dual criteria filtering
    if (neighbor_count ≥ noise_min_neighbors) AND (occupied_voxels ≥ 3):
      KEEP point
    else:
      REMOVE point (isolated noise)
```

**Why dual criteria?**
1. **neighbor_count ≥ 5**: Requires sufficient density
2. **occupied_voxels ≥ 3**: Requires spatial distribution

**Example cases**:
```
Case 1: Single isolated point
  neighbor_count = 1 (only itself)
  occupied_voxels = 1
  → REMOVE ✓

Case 2: Cluster of 5 points in same voxel
  neighbor_count = 5
  occupied_voxels = 1 (all in one voxel)
  → REMOVE ✓ (still isolated)

Case 3: Valid sparse region
  neighbor_count = 6
  occupied_voxels = 3
  → KEEP ✓

Case 4: Dense valid region
  neighbor_count = 20
  occupied_voxels = 8
  → KEEP ✓
```

**Computational Complexity**: O(N)
- Hash map lookups: O(1)
- Each point checks 27 voxels: O(27) = O(1)
- Total: O(N) where N = number of points

This is much faster than KD-tree radius search which is O(N log N).

### Performance Optimization

#### Voxel Grid Downsampling
- Input: ~300k points
- Output: ~5-10k points
- Speedup: ~30-60x reduction
- Quality: Negligible loss for plane detection

#### Two-Pass Processing
1. **Voxelized cloud**: Fast processing for plane detection
2. **Original cloud**: High-resolution floor separation

This gives both speed and accuracy.

## Parameters

All parameters are configured in `config/params.yaml`.

### Common Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ransac_distance_threshold` | double | 0.03 | Distance threshold for RANSAC inliers (meters)<br>Increased from 0.02 to handle uneven warehouse floors |
| `ransac_max_iterations` | int | 150 | Maximum RANSAC iterations<br>Increased from 100 for more robust detection |
| `floor_detection_thickness` | double | 0.01 | Thickness of floor region for RANSAC (meters) |
| `floor_removal_thickness` | double | 0.01 | Thickness of floor to remove (meters) |
| `use_voxel_grid` | bool | true | Enable voxel grid downsampling |
| `voxel_leaf_size` | double | 0.02 | Voxel size for downsampling (meters) |
| `floor_normal_z_threshold` | double | 0.7 | Minimum \|nz\| for valid floor plane<br>Range: [0, 1] where 1 = perfectly horizontal |
| `floor_margin` | double | 0.02 | Additional margin for floor removal (meters) |
| `max_detection_distance` | double | 5.0 | Maximum depth for processing (meters) |

### Auto Mode Parameters

Only used when `use_auto_floor_detection: true`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_auto_floor_detection` | bool | true | Enable automatic floor detection using percentiles |
| `auto_floor_percentile` | double | 1.0 | Percentile for minimum floor height (0-100)<br>Uses lowest 1% of points |
| `auto_floor_max_percentile` | double | 10.0 | Percentile for maximum floor height (0-100)<br>Uses lowest 10% of points |
| `min_valid_z` | double | 0.0 | Minimum valid Z coordinate (meters)<br>Filters ToF shadow artifacts below this value |
| `max_floor_z` | double | 0.12 | Maximum expected floor height (meters)<br>Safety clamp for auto detection |

**Auto Mode Example**:
```yaml
use_auto_floor_detection: true
auto_floor_percentile: 1.0      # Minimum floor from lowest 1%
auto_floor_max_percentile: 10.0  # Maximum floor from lowest 10%
min_valid_z: 0.0                 # Discard z < 0 (ToF shadows)
max_floor_z: 0.12                # Floor can't be above 12cm
```

### Fixed Mode Parameters

Only used when `use_auto_floor_detection: false`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `floor_height` | double | 0.0 | Fixed floor Z coordinate in robot frame (meters) |

**Fixed Mode Example**:
```yaml
use_auto_floor_detection: false
floor_height: 0.0  # Floor exactly at z=0
```

### Noise Removal Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_noise_removal` | bool | true | Enable post-processing noise removal |
| `noise_radius_search` | double | 0.05 | Voxel size for neighborhood search (meters) |
| `noise_min_neighbors` | int | 5 | Minimum neighbors to keep point<br>Higher = more aggressive filtering |
| `noise_floor_height_margin` | double | 0.15 | Only filter points within this height above floor (meters) |

**Noise Removal Tuning**:
- **Still seeing isolated points**: Increase `noise_min_neighbors` to 7-10
- **Removing too much**: Decrease `noise_min_neighbors` to 3-4
- **Performance issues**: Increase `noise_radius_search` to 0.08-0.10

### Camera Extrinsic Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_default_transform` | bool | false | true: Use only optical→base transform<br>false: Apply additional extrinsic parameters |
| `cam_tx` | double | 0.0 | Camera X translation in robot frame (forward, meters) |
| `cam_ty` | double | 0.0 | Camera Y translation in robot frame (left, meters) |
| `cam_tz` | double | 0.35 | Camera Z translation in robot frame (up, meters) |
| `cam_roll` | double | 0.0 | Rotation around X axis (radians) |
| `cam_pitch` | double | 0.0 | Rotation around Y axis (radians) |
| `cam_yaw` | double | 0.0 | Rotation around Z axis (radians) |

**Common Setup** (camera mounted 35cm above robot base):
```yaml
use_default_transform: false
cam_tx: 0.0
cam_ty: 0.0
cam_tz: 0.35  # 35cm above base
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

# Launch with custom config
ros2 launch floor_removal_rgbd floor_removal.launch.py \
  config_file:=/path/to/custom_params.yaml
```

### With ToF Camera

```bash
# Terminal 1: Launch your camera node
# (publishes to /camera/depth/color/points)

# Terminal 2: Launch floor removal
ros2 launch floor_removal_rgbd floor_removal.launch.py
```

### Visualize in RViz

```bash
rviz2

# Add these displays:
# - PointCloud2: /camera/depth/color/points (original)
# - PointCloud2: /no_floor_cloud (scene without floor)
# - PointCloud2: /floor_cloud (detected floor)
# - PointCloud2: /no_floor_cloud_voxelized (downsampled scene)

# Set Fixed Frame to: camera_link or base_link
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/floor_cloud` | sensor_msgs/PointCloud2 | Detected floor points (full resolution, robot frame) |
| `/no_floor_cloud` | sensor_msgs/PointCloud2 | Scene with floor removed (full resolution, robot frame) |
| `/floor_cloud_voxelized` | sensor_msgs/PointCloud2 | Floor points (downsampled) |
| `/no_floor_cloud_voxelized` | sensor_msgs/PointCloud2 | Non-floor points (downsampled) |
| `/no_floor_cloud_voxelized_2d_projected` | sensor_msgs/PointCloud2 | Non-floor points projected to 2D plane |

**Note**: All output clouds are in robot base frame (X=forward, Y=left, Z=up).

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/depth/color/points` | sensor_msgs/PointCloud2 | Input point cloud (camera optical frame) |

## Logging Behavior

The node uses state-change logging to minimize console spam:

**Normal operation**: Silent (no logs)

**State changes**:
- `[INFO] Floor plane FOUND: normal=[nx, ny, nz], d=d` - Plane newly detected
- `[WARNING] Floor plane LOST - floor region is empty` - Plane lost
- `[WARNING] Floor plane LOST - RANSAC detection failed` - RANSAC failed
- `[WARNING] Floor plane LOST - validation failed (nz=...)` - Normal too tilted
- `[INFO] Removed N isolated floor noise points (X%)` - Significant noise removed (>10%)

This reduces log spam while alerting you to important events.

## Examples

### Example 1: FMR Warehouse Application

**Scenario**: Mobile robot with ToF camera in warehouse with uneven floors

**Configuration**:
```yaml
# Auto mode for handling robot tilt and uneven floors
use_auto_floor_detection: true
auto_floor_percentile: 1.0
auto_floor_max_percentile: 10.0
min_valid_z: 0.0
max_floor_z: 0.12

# Robust RANSAC for uneven surfaces
ransac_distance_threshold: 0.03  # 3cm tolerance
ransac_max_iterations: 150

# Camera mounted 35cm above base
use_default_transform: false
cam_tz: 0.35

# Noise removal
enable_noise_removal: true
noise_min_neighbors: 5
```

**Result**: Robust floor detection despite robot movement and floor variations ✓

### Example 2: Fixed Indoor Environment

**Scenario**: Stationary camera, perfectly flat floor

**Configuration**:
```yaml
# Fixed mode for consistent performance
use_auto_floor_detection: false
floor_height: 0.0

# Tighter tolerances
ransac_distance_threshold: 0.02
floor_removal_thickness: 0.01

# Noise removal
enable_noise_removal: true
noise_min_neighbors: 5
```

**Result**: Faster processing with consistent results ✓

### Example 3: Performance-Critical Application

**Scenario**: Need 30+ Hz processing

**Configuration**:
```yaml
# Aggressive downsampling
use_voxel_grid: true
voxel_leaf_size: 0.05  # 5cm voxels

# Fewer RANSAC iterations
ransac_max_iterations: 50

# Disable noise removal if acceptable
enable_noise_removal: false
```

**Result**: Higher framerate with acceptable accuracy ✓

## Troubleshooting

### "Floor plane LOST" warnings

**Cause**: Floor not consistently detected

**Solutions**:
1. Increase `floor_detection_thickness` to 0.02-0.03
2. Increase `ransac_distance_threshold` to 0.04-0.05
3. Check that floor is visible in camera view
4. Verify camera extrinsics are correct

### Isolated floor points remaining

**Cause**: Noise removal not aggressive enough

**Solutions**:
1. Increase `noise_min_neighbors` to 7-10
2. Decrease `noise_radius_search` to 0.03-0.04
3. Check that `enable_noise_removal: true`

### Valid objects being removed

**Cause**: Floor removal too thick or noise removal too aggressive

**Solutions**:
1. Decrease `floor_removal_thickness` to 0.005-0.01
2. Decrease `noise_min_neighbors` to 3-4
3. Ensure objects are at least 2-3cm above floor

### ToF shadow artifacts

**Cause**: Points with z < 0 due to sensor errors

**Solution**:
```yaml
min_valid_z: 0.0  # Discard all points below z=0
```

### High CPU usage

**Cause**: Processing too many points

**Solutions**:
1. Increase `voxel_leaf_size` to 0.03-0.05
2. Decrease `max_detection_distance` to 3.0-4.0
3. Reduce `ransac_max_iterations` to 50-100

## Performance

Typical performance on Intel Core i5-8265U:

| Input Size | Voxelized | Floor Detection | Noise Removal | Total | FPS |
|------------|-----------|-----------------|---------------|-------|-----|
| 307k points | 6k points | 8-12 ms | 2-3 ms | 15-20 ms | 50-65 Hz |
| 200k points | 4k points | 5-8 ms | 1-2 ms | 10-12 ms | 80-100 Hz |

**Note**: Performance scales with point count and voxel size.

## Technical Notes

### Why Percentile-Based Detection?

**Problem with fixed height**:
- Robot pitch/roll changes effective floor height
- Uneven warehouse floors vary in height
- Camera calibration errors accumulate

**Percentile solution**:
- **Robust to outliers**: Ignores high objects and sensor noise
- **Adaptive**: Automatically adjusts to robot orientation
- **Statistical**: Uses data distribution, not absolute values
- **Fast**: O(N log N) sorting is acceptable for voxelized cloud

**Example**:
```
Robot tilts 5° forward:
  Front of floor: z = -0.04m
  Back of floor: z = +0.04m

Fixed mode (floor_height=0.0):
  → Misses front, includes back objects ❌

Auto mode (1st percentile):
  → Finds z = -0.04m, adjusts range ✓
```

### Why 3x3x3 Neighborhood Filtering?

**Alternatives considered**:
1. **KD-tree radius search**: O(N log N), too slow
2. **Single voxel check**: O(N), but removes valid sparse regions
3. **3x3x3 neighborhood**: O(N), preserves continuity ✓

**Why it works**:
- Isolated noise: 1 occupied voxel → removed
- Valid sparse region: 3+ occupied voxels → kept
- Dense region: Many occupied voxels → kept

### Coordinate Frame Convention

Follows ROS REP-103 and REP-105:
- **Camera optical frame**: X=right, Y=down, Z=forward (per REP-103)
- **Robot base frame**: X=forward, Y=left, Z=up (per REP-105)

Processing in robot frame provides:
1. Intuitive parameters (height = Z coordinate)
2. Easy calibration (measure camera height directly)
3. Operator-friendly (field technicians understand Z-height)

## Dependencies

- ROS2 Foxy or later
- PCL (Point Cloud Library) 1.10+
- Eigen3
- sensor_msgs

## License

MIT

## Authors

cwon789 (Wonyoung Chung)

## References

- **RANSAC**: Fischler & Bolles, "Random Sample Consensus", Communications of the ACM, 1981
- **PCL**: Point Cloud Library - https://pointclouds.org/
- **ROS REP-103**: Standard Units of Measure and Coordinate Conventions
- **ROS REP-105**: Coordinate Frames for Mobile Platforms
