# WIA Smart Wheelchair SLAM Specification

## Version
- **Version**: 1.0.0
- **Date**: 2025-01-01
- **Status**: Draft

## 1. Overview

본 문서는 WIA Smart Wheelchair의 SLAM(Simultaneous Localization and Mapping) 시스템 표준을 정의합니다.

### 1.1 Supported Algorithms

| Algorithm | Type | Use Case | ROS2 Package |
|-----------|------|----------|--------------|
| GMapping | 2D Laser | 소규모 실내 | slam_gmapping |
| Cartographer | 2D/3D | 대규모 실내/실외 | cartographer_ros |
| RTAB-Map | RGB-D | 시각적 SLAM | rtabmap_ros |
| Hector SLAM | 2D Laser | IMU 없이 사용 | hector_slam |
| AMCL | Localization | 사전 지도 기반 | nav2_amcl |

### 1.2 System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | Quad-core 1.5GHz | Quad-core 2.0GHz+ |
| RAM | 4GB | 8GB+ |
| LiDAR | 5Hz, 360° | 10Hz+, 360° |
| Odometry | Wheel encoders | Wheel + IMU |

## 2. Map Format

### 2.1 Occupancy Grid

```yaml
# OccupancyGrid 메시지 구조
header:
  stamp: <timestamp>
  frame_id: "map"

info:
  resolution: 0.05          # meters/cell
  width: 400                # cells
  height: 400               # cells
  origin:
    position:
      x: -10.0              # meters
      y: -10.0              # meters
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

data: [...]                 # int8 array
# Values:
#   -1 = Unknown
#    0 = Free
#  100 = Occupied
#  1-99 = Probability of occupancy
```

### 2.2 Map File Format

지도는 두 개의 파일로 저장됩니다:

**map.yaml (메타데이터):**
```yaml
image: map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**map.pgm (이미지):**
- Format: PGM (Portable Gray Map)
- 205 (0xCD): Unknown
- 254 (0xFE): Free
- 0 (0x00): Occupied

### 2.3 Coordinate System

```
     +Y (North)
      ^
      |
      |
      +-----> +X (East)

Origin: 지도 좌측 하단
Orientation: 반시계 방향 (+theta)
```

## 3. SLAM Interface

### 3.1 ROS2 Topics

**Subscribed:**
| Topic | Type | Description |
|-------|------|-------------|
| `/wia_wheelchair/scan` | sensor_msgs/LaserScan | LiDAR 스캔 |
| `/wia_wheelchair/odom` | nav_msgs/Odometry | 오도메트리 |
| `/wia_wheelchair/imu` | sensor_msgs/Imu | IMU 데이터 (옵션) |

**Published:**
| Topic | Type | Description |
|-------|------|-------------|
| `/wia_wheelchair/map` | nav_msgs/OccupancyGrid | 점유 격자 지도 |
| `/wia_wheelchair/map_metadata` | nav_msgs/MapMetaData | 지도 메타데이터 |
| `/wia_wheelchair/pose` | geometry_msgs/PoseWithCovarianceStamped | 로봇 위치 |

**TF Transforms:**
| Parent | Child | Description |
|--------|-------|-------------|
| map | odom | 지도-오도메트리 변환 |
| odom | base_link | 오도메트리 변환 |

### 3.2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/wia_wheelchair/slam/start_mapping` | std_srvs/Trigger | 매핑 시작 |
| `/wia_wheelchair/slam/stop_mapping` | std_srvs/Trigger | 매핑 중지 |
| `/wia_wheelchair/slam/save_map` | wia_wheelchair_msgs/SaveMap | 지도 저장 |
| `/wia_wheelchair/slam/load_map` | wia_wheelchair_msgs/LoadMap | 지도 로드 |
| `/wia_wheelchair/slam/relocalize` | wia_wheelchair_msgs/Relocalize | 재위치 추정 |

### 3.3 Actions

| Action | Type | Description |
|--------|------|-------------|
| `/wia_wheelchair/slam/set_initial_pose` | wia_wheelchair_msgs/SetInitialPose | 초기 위치 설정 |

## 4. Configuration

### 4.1 GMapping Parameters

```yaml
slam_gmapping:
  ros__parameters:
    # Laser parameters
    maxRange: 12.0
    maxUrange: 11.5

    # Map parameters
    xmin: -10.0
    ymin: -10.0
    xmax: 10.0
    ymax: 10.0
    delta: 0.05           # resolution

    # Processing
    particles: 30
    iterations: 5

    # Motion model
    srr: 0.1              # odometry error (rotation from rotation)
    srt: 0.2              # odometry error (rotation from translation)
    str: 0.1              # odometry error (translation from rotation)
    stt: 0.2              # odometry error (translation from translation)

    # Update thresholds
    linearUpdate: 0.2     # meters
    angularUpdate: 0.1    # radians
    temporalUpdate: 1.0   # seconds
```

### 4.2 Cartographer Parameters

```yaml
cartographer:
  ros__parameters:
    use_sim_time: false

    # Trajectory builder
    trajectory_builder_2d:
      min_range: 0.3
      max_range: 12.0
      use_imu_data: true

      # Submaps
      submaps:
        resolution: 0.05
        num_range_data: 90

      # Motion filter
      motion_filter:
        max_time_seconds: 5.0
        max_distance_meters: 0.2
        max_angle_radians: 0.1

    # Pose graph
    pose_graph:
      optimize_every_n_nodes: 90
      constraint_builder:
        min_score: 0.55
      optimization_problem:
        huber_scale: 1.0
```

### 4.3 AMCL Parameters (Localization)

```yaml
amcl:
  ros__parameters:
    # Filter parameters
    min_particles: 500
    max_particles: 2000

    # Motion model
    robot_model_type: "differential"
    alpha1: 0.2           # rotation noise from rotation
    alpha2: 0.2           # rotation noise from translation
    alpha3: 0.2           # translation noise from translation
    alpha4: 0.2           # translation noise from rotation

    # Laser model
    laser_model_type: "likelihood_field"
    laser_max_range: 12.0
    laser_min_range: 0.3
    max_beams: 60

    # Update thresholds
    update_min_d: 0.1     # meters
    update_min_a: 0.1     # radians

    # Recovery
    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.1
```

## 5. Operating Modes

### 5.1 Mapping Mode

```
┌─────────────────────────────────────────┐
│              Mapping Mode               │
├─────────────────────────────────────────┤
│  1. Start SLAM node                     │
│  2. Drive wheelchair around environment │
│  3. Build occupancy grid                │
│  4. Save map when complete              │
└─────────────────────────────────────────┘
```

**State Machine:**
```
IDLE → MAPPING → SAVING → IDLE
         │
         └──────→ PAUSED
```

### 5.2 Localization Mode

```
┌─────────────────────────────────────────┐
│           Localization Mode             │
├─────────────────────────────────────────┤
│  1. Load existing map                   │
│  2. Initialize pose (manual or auto)    │
│  3. Track position using AMCL           │
│  4. Publish map→odom transform          │
└─────────────────────────────────────────┘
```

**Relocalization:**
1. Global localization (전역 검색)
2. Pose estimation with covariance
3. Converge to single hypothesis

## 6. Map Management

### 6.1 Map Storage

```
maps/
├── home/
│   ├── floor1/
│   │   ├── map.yaml
│   │   ├── map.pgm
│   │   └── locations.yaml
│   └── floor2/
│       ├── map.yaml
│       └── map.pgm
└── office/
    ├── map.yaml
    └── map.pgm
```

### 6.2 Saved Locations

```yaml
# locations.yaml
locations:
  - name: "거실"
    pose:
      x: 2.5
      y: 3.0
      theta: 0.0

  - name: "침실"
    pose:
      x: 5.0
      y: 1.5
      theta: 1.57

  - name: "화장실"
    pose:
      x: 1.0
      y: 5.5
      theta: 3.14

  - name: "현관"
    pose:
      x: 0.5
      y: 0.5
      theta: 0.0
```

## 7. Quality Metrics

### 7.1 Localization Quality

| Metric | Threshold | Action |
|--------|-----------|--------|
| Covariance (x,y) | < 0.1 m² | Normal |
| Covariance (x,y) | 0.1-0.5 m² | Warning |
| Covariance (x,y) | > 0.5 m² | Relocalize |
| Covariance (theta) | < 0.05 rad² | Normal |
| Particle spread | < 1.0 m | Normal |

### 7.2 Map Quality

| Metric | Good | Poor |
|--------|------|------|
| Coverage | > 95% | < 80% |
| Loop closures | Multiple | None |
| Consistency | No overlap | Ghost walls |

## 8. TypeScript Interface

```typescript
export interface SLAMInterface {
  // Algorithm selection
  algorithm: 'gmapping' | 'cartographer' | 'rtabmap' | 'hector';

  // Map management
  startMapping(): Promise<void>;
  stopMapping(): Promise<void>;
  saveMap(path: string): Promise<void>;
  loadMap(path: string): Promise<void>;

  // Pose estimation
  getCurrentPose(): Pose2D;
  getPoseCovariance(): number[];
  relocalize(initialPose?: Pose2D): Promise<boolean>;

  // Map data
  getMap(): OccupancyGrid;
  getMapMetadata(): MapMetaData;

  // Status
  getStatus(): SLAMStatus;
}

export interface OccupancyGrid {
  header: Header;
  info: MapMetaData;
  data: Int8Array;
}

export interface MapMetaData {
  resolution: number;
  width: number;
  height: number;
  origin: Pose2D;
}

export interface SLAMStatus {
  mode: 'idle' | 'mapping' | 'localizing' | 'saving';
  mapLoaded: boolean;
  localizationQuality: 'good' | 'fair' | 'poor' | 'lost';
  particleCount?: number;
  covarianceTrace?: number;
}
```

## Appendix A: Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Map drift | Poor odometry | Improve odometry, add IMU |
| Ghost walls | Inconsistent scans | Reduce noise, check LiDAR |
| Lost localization | Dynamic environment | Relocalize, update map |
| Slow mapping | High resolution | Reduce resolution, particles |

## Appendix B: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-01-01 | WIA | Initial release |
