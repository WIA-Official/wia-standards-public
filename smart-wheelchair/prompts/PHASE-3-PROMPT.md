# WIA Smart Wheelchair - Phase 3: Autonomous Navigation

## 목표
휠체어 자율주행 기능을 표준화합니다.

## 3.1 SLAM (Simultaneous Localization and Mapping)

```typescript
interface SLAMInterface {
  // 알고리즘 선택
  algorithm: 'gmapping' | 'cartographer' | 'rtabmap' | 'hector';

  // 지도 관리
  startMapping(): void;
  stopMapping(): void;
  saveMap(path: string): Promise<void>;
  loadMap(path: string): Promise<void>;

  // 위치 추정
  getCurrentPose(): Pose2D;
  relocalize(initialPose?: Pose2D): Promise<boolean>;

  // 지도 데이터
  getMap(): OccupancyGrid;
}

interface OccupancyGrid {
  header: {
    timestamp: number;
    frameId: string;
  };
  resolution: number;          // meters/cell
  width: number;               // cells
  height: number;              // cells
  origin: Pose2D;              // 지도 원점
  data: Int8Array;             // -1=unknown, 0=free, 100=occupied
}
```

## 3.2 경로 계획

```typescript
interface PathPlanner {
  // 전역 경로 계획
  planGlobalPath(
    start: Pose2D,
    goal: Pose2D,
    map: OccupancyGrid
  ): Path;

  // 지역 경로 계획 (장애물 회피)
  planLocalPath(
    currentPose: Pose2D,
    globalPath: Path,
    obstacles: Obstacle[]
  ): Path;

  // 알고리즘 설정
  config: {
    globalPlanner: 'dijkstra' | 'astar' | 'rrt' | 'prm';
    localPlanner: 'dwa' | 'teb' | 'mpc';
    inflationRadius: number;   // meters
    costFactors: CostFactors;
  };
}

interface Path {
  poses: Pose2D[];
  timestamps?: number[];
  costs?: number[];
  totalCost: number;
  estimatedTime: number;       // seconds
}

interface CostFactors {
  distance: number;
  safety: number;
  smoothness: number;
  energy: number;
}
```

## 3.3 장애물 회피

```typescript
interface ObstacleAvoidance {
  // 회피 모드
  mode: 'stop' | 'slow' | 'avoid' | 'autonomous';

  // 안전 영역 설정
  safetyZones: {
    critical: number;          // meters (즉시 정지)
    warning: number;           // meters (감속)
    caution: number;           // meters (주의)
  };

  // 동적 장애물 예측
  predictTrajectory(obstacle: Obstacle, horizon: number): Pose2D[];

  // 회피 경로 생성
  generateAvoidancePath(obstacle: Obstacle): Path;

  // 긴급 정지
  emergencyStop(): void;
}
```

## 3.4 목적지 설정

```typescript
interface GoalManager {
  // 목적지 설정 방법
  setGoalByPose(pose: Pose2D): void;
  setGoalByName(locationName: string): void;  // "거실", "화장실"
  setGoalByMarker(markerId: string): void;    // QR 코드 등
  setGoalByVoice(command: string): void;      // "화장실 가줘"

  // 저장된 위치
  savedLocations: Map<string, Pose2D>;
  saveLocation(name: string, pose: Pose2D): void;
  deleteLocation(name: string): void;

  // 상태
  getCurrentGoal(): Pose2D | null;
  getProgress(): NavigationProgress;
  cancelNavigation(): void;
}

interface NavigationProgress {
  status: 'idle' | 'planning' | 'navigating' | 'arrived' | 'failed';
  distanceRemaining: number;   // meters
  estimatedTimeRemaining: number; // seconds
  percentComplete: number;
  currentSpeed: number;        // m/s
}
```

## 3.5 실내 내비게이션 특화

```typescript
interface IndoorNavigation {
  // 엘리베이터 연동
  elevator: {
    detectElevator(): boolean;
    callElevator(floor: number): Promise<void>;
    enterElevator(): Promise<void>;
    exitElevator(): Promise<void>;
  };

  // 자동문 연동
  automaticDoor: {
    detectDoor(): boolean;
    requestOpen(): Promise<void>;
    waitForOpen(): Promise<boolean>;
  };

  // 좁은 통로
  narrowPassage: {
    detectNarrowPassage(): boolean;
    planCarefulPath(): Path;
    setReducedSpeed(): void;
  };

  // 경사로
  ramp: {
    detectRamp(): { angle: number; direction: 'up' | 'down' } | null;
    adjustSpeedForRamp(angle: number): void;
  };
}
```

---

## 산출물

```
smart-wheelchair/
├── ros2_ws/src/
│   └── wia_wheelchair_navigation/
│       ├── nodes/
│       │   ├── slam_node.py
│       │   ├── global_planner.py
│       │   ├── local_planner.py
│       │   └── goal_manager.py
│       ├── config/
│       │   ├── nav_params.yaml
│       │   └── costmap_params.yaml
│       └── launch/
│           └── navigation.launch.py
├── maps/
│   └── examples/
├── spec/
│   ├── SLAM-SPEC.md
│   └── PATH-PLANNING-SPEC.md
```

---

## 다음: Phase 4 (보조기기 연동)
