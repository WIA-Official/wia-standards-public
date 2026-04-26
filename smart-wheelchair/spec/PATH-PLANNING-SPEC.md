# WIA Smart Wheelchair Path Planning Specification

## Version
- **Version**: 1.0.0
- **Date**: 2025-01-01
- **Status**: Draft

## 1. Overview

본 문서는 WIA Smart Wheelchair의 경로 계획 및 장애물 회피 시스템을 정의합니다.

### 1.1 Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Navigation Stack                           │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────────┐   │
│  │ Goal Manager │──▶│Global Planner│──▶│  Local Planner   │   │
│  └──────────────┘   └──────────────┘   └──────────────────┘   │
│          │                 │                    │               │
│          ▼                 ▼                    ▼               │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────────┐   │
│  │   Costmap    │   │ Global Path  │   │   cmd_vel        │   │
│  │   Server     │   │              │   │                  │   │
│  └──────────────┘   └──────────────┘   └──────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Supported Algorithms

**Global Planners:**
| Algorithm | Description | Best For |
|-----------|-------------|----------|
| Dijkstra | 최단 경로 | 정적 환경 |
| A* | 휴리스틱 기반 | 대규모 지도 |
| RRT | 샘플링 기반 | 복잡한 환경 |
| NavFn | Nav2 기본 | 일반 실내 |
| Theta* | Any-angle | 넓은 공간 |

**Local Planners:**
| Algorithm | Description | Best For |
|-----------|-------------|----------|
| DWA | Dynamic Window | 일반 주행 |
| TEB | Timed Elastic Band | 좁은 공간 |
| MPC | Model Predictive | 정밀 제어 |
| MPPI | Model Predictive Path Integral | 동적 환경 |

## 2. Costmap

### 2.1 Layer Structure

```
┌─────────────────────────────────────┐
│         Costmap 2D                  │
├─────────────────────────────────────┤
│  ┌─────────────────────────────┐   │
│  │     Inflation Layer         │   │  ← 장애물 확장
│  ├─────────────────────────────┤   │
│  │     Obstacle Layer          │   │  ← 동적 장애물
│  ├─────────────────────────────┤   │
│  │     Static Layer            │   │  ← 정적 지도
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
```

### 2.2 Cost Values

| Value | Name | Description |
|-------|------|-------------|
| 0 | FREE_SPACE | 자유 공간 |
| 1-127 | LOW_COST | 낮은 비용 (경고 영역) |
| 128-252 | HIGH_COST | 높은 비용 (위험 영역) |
| 253 | INSCRIBED | 로봇 반경 내 |
| 254 | LETHAL | 충돌 확정 |
| 255 | NO_INFORMATION | 정보 없음 |

### 2.3 Inflation Radius

```
     ┌───────────────────┐
     │    ██████████     │  ← Lethal (254)
     │  ████████████████ │  ← Inscribed (253)
     │████████████████████│  ← Inflation (252-1)
     │  ████████████████ │
     │    ██████████     │
     └───────────────────┘

robot_radius = 0.35m (휠체어 반경)
inflation_radius = 0.55m (안전 거리 포함)
cost_scaling_factor = 3.0
```

### 2.4 Configuration

```yaml
global_costmap:
  ros__parameters:
    update_frequency: 1.0
    publish_frequency: 1.0
    global_frame: map
    robot_base_frame: base_link
    robot_radius: 0.35
    resolution: 0.05
    track_unknown_space: true

    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: true
      observation_sources: scan
      scan:
        topic: /wia_wheelchair/scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        raytrace_max_range: 10.0
        obstacle_max_range: 8.0

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.55
```

```yaml
local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_link
    robot_radius: 0.35
    resolution: 0.05
    rolling_window: true
    width: 5
    height: 5

    plugins: ["obstacle_layer", "inflation_layer"]
```

## 3. Global Planner

### 3.1 Interface

```yaml
# Topics
Input:
  /wia_wheelchair/goal_pose: geometry_msgs/PoseStamped
  /wia_wheelchair/map: nav_msgs/OccupancyGrid

Output:
  /wia_wheelchair/plan: nav_msgs/Path

# Service
/wia_wheelchair/compute_path_to_pose:
  Request:
    goal: geometry_msgs/PoseStamped
    start: geometry_msgs/PoseStamped (optional)
    use_start: bool
  Response:
    path: nav_msgs/Path
    planning_time: float
```

### 3.2 A* Configuration

```yaml
global_planner:
  ros__parameters:
    planner_plugin: "nav2_navfn_planner/NavfnPlanner"

    NavfnPlanner:
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
      use_final_approach_orientation: false
```

### 3.3 Path Format

```typescript
interface Path {
  header: Header;
  poses: PoseStamped[];
}

interface PoseStamped {
  header: Header;
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}
```

## 4. Local Planner

### 4.1 DWA (Dynamic Window Approach)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Velocity limits (휠체어 특성)
      min_vel_x: 0.0
      max_vel_x: 1.0
      min_vel_y: 0.0          # 차동 구동
      max_vel_y: 0.0
      max_vel_theta: 0.8
      min_speed_xy: 0.1
      max_speed_xy: 1.0
      min_speed_theta: 0.1

      # Acceleration limits
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_theta: -1.0

      # Trajectory generation
      vx_samples: 20
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 1.5

      # Goal tolerance
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.1
      trans_stopped_velocity: 0.05
      rot_stopped_velocity: 0.05

      # Critics (scoring)
      critics:
        - "RotateToGoal"
        - "Oscillation"
        - "ObstacleFootprint"
        - "GoalAlign"
        - "PathAlign"
        - "PathDist"
        - "GoalDist"

      PathAlign:
        scale: 32.0
      GoalAlign:
        scale: 24.0
      PathDist:
        scale: 32.0
      GoalDist:
        scale: 24.0
      ObstacleFootprint:
        scale: 0.01
        sum_scores: false
```

### 4.2 TEB (Timed Elastic Band)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "teb_local_planner::TebLocalPlannerROS"

      # Trajectory
      teb_autosize: true
      dt_ref: 0.3
      dt_hysteresis: 0.1
      min_samples: 3
      global_plan_overwrite_orientation: true
      allow_init_with_backwards_motion: false

      # Robot
      max_vel_x: 1.0
      max_vel_x_backwards: 0.3
      max_vel_theta: 0.8
      acc_lim_x: 0.5
      acc_lim_theta: 1.0

      footprint_model:
        type: "circular"
        radius: 0.35

      # Goal tolerance
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.1
      free_goal_vel: false

      # Obstacles
      min_obstacle_dist: 0.3
      inflation_dist: 0.5
      include_costmap_obstacles: true
      costmap_obstacles_behind_robot_dist: 1.0

      # Optimization
      no_inner_iterations: 5
      no_outer_iterations: 4
      penalty_epsilon: 0.1
      weight_max_vel_x: 2.0
      weight_max_vel_theta: 1.0
      weight_acc_lim_x: 1.0
      weight_acc_lim_theta: 1.0
      weight_kinematics_nh: 1000.0
      weight_kinematics_forward_drive: 1.0
      weight_obstacle: 100.0
```

## 5. Goal Manager

### 5.1 Goal Types

| Type | Method | Example |
|------|--------|---------|
| Pose | `setGoalByPose()` | x=2.0, y=3.0, theta=0 |
| Named | `setGoalByName()` | "화장실" |
| Marker | `setGoalByMarker()` | QR code ID |
| Voice | `setGoalByVoice()` | "주방으로 가줘" |

### 5.2 Navigation States

```
┌───────┐    goal    ┌──────────┐   path   ┌────────────┐
│ IDLE  │──────────▶│ PLANNING │────────▶│ NAVIGATING │
└───────┘           └──────────┘          └────────────┘
    ▲                    │                      │
    │                    │ fail                 │ arrived
    │                    ▼                      ▼
    │               ┌────────┐            ┌─────────┐
    └───────────────│ FAILED │            │ ARRIVED │
         cancel     └────────┘            └─────────┘
```

### 5.3 Progress Reporting

```typescript
interface NavigationProgress {
  status: NavigationStatus;
  distanceRemaining: number;      // meters
  estimatedTimeRemaining: number; // seconds
  percentComplete: number;        // 0-100
  currentSpeed: number;           // m/s
  currentWaypoint: number;
  totalWaypoints: number;
}

type NavigationStatus =
  | 'idle'
  | 'planning'
  | 'navigating'
  | 'recovering'
  | 'arrived'
  | 'failed'
  | 'cancelled';
```

## 6. Obstacle Avoidance

### 6.1 Safety Zones

```
              ┌─────────────────────────────────┐
              │                                 │
              │         Caution Zone            │  1.5m
              │     (reduced speed)             │
              │  ┌───────────────────────────┐  │
              │  │                           │  │
              │  │      Warning Zone         │  │  0.8m
              │  │    (prepare to stop)      │  │
              │  │   ┌───────────────────┐   │  │
              │  │   │   Critical Zone   │   │  │  0.3m
              │  │   │   (emergency stop)│   │  │
              │  │   │    ┌───────┐      │   │  │
              │  │   │    │ROBOT│      │   │  │
              │  │   │    └───────┘      │   │  │
              │  │   └───────────────────┘   │  │
              │  └───────────────────────────┘  │
              └─────────────────────────────────┘
```

### 6.2 Avoidance Modes

| Mode | Behavior | Use Case |
|------|----------|----------|
| STOP | 즉시 정지, 사용자 개입 대기 | 수동 모드 |
| SLOW | 감속 후 정지 | 보조 모드 |
| AVOID | 장애물 회피 경로 생성 | 자율 모드 |
| AUTONOMOUS | 완전 자동 회피 | 자율 모드 |

### 6.3 Recovery Behaviors

```yaml
recovery_behaviors:
  - name: "spin"
    plugin: "nav2_behaviors/Spin"
  - name: "backup"
    plugin: "nav2_behaviors/BackUp"
  - name: "wait"
    plugin: "nav2_behaviors/Wait"

behavior_server:
  ros__parameters:
    spin:
      spin_dist: 1.57        # 90 degrees
    backup:
      backup_dist: 0.3
      backup_speed: 0.1
    wait:
      wait_time: 5.0
```

## 7. Indoor Navigation Features

### 7.1 Narrow Passage

```yaml
narrow_passage:
  detection_threshold: 0.8    # meters
  reduced_speed: 0.3          # m/s
  inflation_reduction: 0.1    # meters
```

### 7.2 Elevator Integration

```typescript
interface ElevatorInterface {
  detectElevator(): boolean;
  callElevator(floor: number): Promise<void>;
  getElevatorStatus(): ElevatorStatus;
  enterElevator(): Promise<void>;
  exitElevator(): Promise<void>;
}

interface ElevatorStatus {
  available: boolean;
  currentFloor: number;
  doorOpen: boolean;
  targetFloor: number | null;
}
```

### 7.3 Automatic Door

```typescript
interface AutomaticDoorInterface {
  detectDoor(): DoorInfo | null;
  requestOpen(doorId: string): Promise<void>;
  waitForOpen(timeout: number): Promise<boolean>;
  getDoorStatus(doorId: string): DoorStatus;
}

interface DoorInfo {
  id: string;
  position: Pose2D;
  width: number;
  type: 'sliding' | 'swing' | 'revolving';
}

type DoorStatus = 'closed' | 'opening' | 'open' | 'closing';
```

### 7.4 Ramp Detection

```yaml
ramp_detection:
  max_safe_angle: 8.0         # degrees
  warning_angle: 6.0          # degrees
  speed_reduction_factor: 0.5  # reduce speed by 50% on ramps
```

## 8. ROS2 Interface

### 8.1 Actions

| Action | Type | Description |
|--------|------|-------------|
| `/wia_wheelchair/navigate_to_pose` | NavigateToPose | 목표 위치 이동 |
| `/wia_wheelchair/follow_path` | FollowPath | 경로 추종 |
| `/wia_wheelchair/navigate_through_poses` | NavigateThroughPoses | 다중 웨이포인트 |

### 8.2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/wia_wheelchair/clear_costmap` | ClearCostmap | 코스트맵 초기화 |
| `/wia_wheelchair/get_costmap` | GetCostmap | 코스트맵 조회 |

## 9. TypeScript Interface

```typescript
export interface PathPlanner {
  // Global planning
  planGlobalPath(
    start: Pose2D,
    goal: Pose2D,
    map: OccupancyGrid
  ): Promise<Path>;

  // Local planning
  planLocalPath(
    currentPose: Pose2D,
    globalPath: Path,
    obstacles: Obstacle[]
  ): Path;

  // Configuration
  config: PathPlannerConfig;
}

export interface PathPlannerConfig {
  globalPlanner: 'dijkstra' | 'astar' | 'rrt' | 'navfn';
  localPlanner: 'dwa' | 'teb' | 'mpc';
  inflationRadius: number;
  robotRadius: number;
  costFactors: CostFactors;
}

export interface CostFactors {
  distance: number;
  safety: number;
  smoothness: number;
  energy: number;
}

export interface Path {
  header: Header;
  poses: Pose2D[];
  totalCost: number;
  estimatedTime: number;
  length: number;
}

export interface GoalManager {
  // Goal setting
  setGoalByPose(pose: Pose2D): void;
  setGoalByName(name: string): void;
  setGoalByMarker(markerId: string): void;

  // Saved locations
  savedLocations: Map<string, Pose2D>;
  saveLocation(name: string, pose: Pose2D): void;
  deleteLocation(name: string): void;

  // Status
  getCurrentGoal(): Pose2D | null;
  getProgress(): NavigationProgress;
  cancelNavigation(): void;
}
```

## Appendix A: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-01-01 | WIA | Initial release |
