# 5장: 청소 로봇 제어 프로토콜

## 내비게이션, 경로 계획 및 청소 알고리즘

### 5.1 내비게이션 프로토콜 아키텍처

WIA-CLEANING-ROBOT 표준은 SLAM, 경로 계획 및 장애물 회피를 포함한 로봇 내비게이션을 위한 종합 프로토콜을 정의합니다. 이러한 프로토콜은 안전을 유지하면서 효율적인 커버리지를 보장합니다.

```typescript
// 내비게이션 프로토콜 아키텍처
interface NavigationProtocolArchitecture {
  version: '1.0.0';

  layers: {
    perception: {
      description: '센서 데이터 획득 및 처리';
      components: ['LiDAR 처리', '카메라 비전', '센서 융합'];
      frequency: '10-30 Hz';
    };
    localization: {
      description: '로봇 위치 추정';
      components: ['SLAM', '오도메트리', '랜드마크 인식'];
      frequency: '10-20 Hz';
    };
    mapping: {
      description: '환경 지도 구축';
      components: ['점유 그리드', '시맨틱 매핑', '방 분할'];
      updateFrequency: '탐색 중 실시간';
    };
    planning: {
      description: '경로 및 커버리지 계획';
      components: ['글로벌 플래너', '로컬 플래너', '커버리지 플래너'];
      frequency: '1-10 Hz';
    };
    control: {
      description: '모션 실행';
      components: ['속도 컨트롤러', '궤적 추적기'];
      frequency: '50-100 Hz';
    };
  };
}

// SLAM 프로토콜
interface SLAMProtocol {
  type: SlamType;

  configuration: {
    mapResolution: number;        // 셀당 미터
    maxRange: number;             // 미터
    minRange: number;             // 미터
    updateInterval: number;       // 밀리초
    keyframeDistance: number;     // 미터
    keyframeAngle: number;        // 라디안
    loopClosureThreshold: number;
  };

  algorithms: {
    scanMatching: ScanMatchingAlgorithm;
    loopClosure: LoopClosureAlgorithm;
    graphOptimization: GraphOptimizationAlgorithm;
    mapMerging: MapMergingAlgorithm;
  };
}

// SLAM 구현
class SLAMSystem {
  private config: SLAMProtocol;
  private poseGraph: PoseGraph;
  private occupancyMap: OccupancyGrid;
  private scanMatcher: ScanMatcher;
  private loopCloser: LoopClosureDetector;

  constructor(config: SLAMProtocol) {
    this.config = config;
    this.poseGraph = new PoseGraph();
    this.occupancyMap = new OccupancyGrid(config.configuration.mapResolution);
    this.scanMatcher = new ScanMatcher(config.algorithms.scanMatching);
    this.loopCloser = new LoopClosureDetector(config.algorithms.loopClosure);
  }

  async processLidarScan(
    scan: LidarScan,
    odometry: Odometry
  ): Promise<SLAMUpdate> {
    // 오도메트리에서 자세 예측
    const predictedPose = this.predictPose(odometry);

    // 자세 보정을 위한 스캔 매칭
    const matchResult = await this.scanMatcher.match(
      scan,
      this.occupancyMap,
      predictedPose
    );

    if (!matchResult.valid) {
      return { success: false, reason: '스캔 매칭 실패' };
    }

    const correctedPose = matchResult.pose;
    const covariance = matchResult.covariance;

    // 포즈 그래프에 추가
    const node = this.poseGraph.addNode(correctedPose, scan);

    // 루프 클로저 확인
    const loopClosure = await this.loopCloser.detect(scan, correctedPose);
    if (loopClosure) {
      this.poseGraph.addLoopConstraint(loopClosure);
      await this.optimizePoseGraph();
    }

    // 점유 지도 업데이트
    await this.updateMap(scan, correctedPose);

    return {
      success: true,
      pose: correctedPose,
      covariance,
      mapUpdated: true,
      loopClosureDetected: !!loopClosure
    };
  }

  private async updateMap(scan: LidarScan, pose: Pose2D): Promise<void> {
    // 점유 업데이트를 위한 레이 캐스팅
    for (const point of scan.points) {
      const worldPoint = this.transformToWorld(point, pose);
      const rayEnd = { x: worldPoint.x, y: worldPoint.y };
      const rayStart = { x: pose.x, y: pose.y };

      // 레이를 따라 셀을 자유로 표시
      const rayCells = this.rayCast(rayStart, rayEnd);
      for (const cell of rayCells.slice(0, -1)) {
        this.occupancyMap.updateCell(cell, OccupancyValue.FREE);
      }

      // 끝 셀을 점유로 표시 (유효 범위인 경우)
      if (point.range < this.config.configuration.maxRange) {
        this.occupancyMap.updateCell(
          rayCells[rayCells.length - 1],
          OccupancyValue.OCCUPIED
        );
      }
    }
  }
}
```

### 5.2 경로 계획 프로토콜

```typescript
// 경로 계획 프로토콜
interface PathPlanningProtocol {
  globalPlanner: {
    algorithm: GlobalPlannerAlgorithm;
    heuristic: PlannerHeuristic;
    costFactors: CostFactors;
  };

  localPlanner: {
    algorithm: LocalPlannerAlgorithm;
    horizonDistance: number;
    updateFrequency: number;
  };

  coveragePlanner: {
    algorithm: CoveragePlannerAlgorithm;
    cellSize: number;
    overlapRatio: number;
  };
}

type GlobalPlannerAlgorithm = 'ASTAR' | 'DIJKSTRA' | 'RRT' | 'THETA_STAR' | 'JPS';
type LocalPlannerAlgorithm = 'DWA' | 'TEB' | 'MPC' | 'PURE_PURSUIT';
type CoveragePlannerAlgorithm = 'BOUSTROPHEDON' | 'SPIRAL' | 'GRID_BASED' | 'NEURAL';

// 글로벌 경로 플래너
class GlobalPathPlanner {
  private algorithm: GlobalPlannerAlgorithm;
  private heuristic: HeuristicFunction;
  private costMap: CostMap;

  constructor(config: PathPlanningProtocol['globalPlanner']) {
    this.algorithm = config.algorithm;
    this.heuristic = this.createHeuristic(config.heuristic);
    this.costMap = new CostMap();
  }

  async planPath(
    start: Position2D,
    goal: Position2D,
    map: OccupancyGrid
  ): Promise<PlannedPath> {
    // 점유 그리드에서 비용 지도 업데이트
    this.costMap.updateFromOccupancy(map);

    // 알고리즘 적용
    switch (this.algorithm) {
      case 'ASTAR':
        return this.aStarSearch(start, goal);
      case 'THETA_STAR':
        return this.thetaStarSearch(start, goal);
      case 'JPS':
        return this.jumpPointSearch(start, goal);
      case 'RRT':
        return this.rrtSearch(start, goal);
      default:
        return this.aStarSearch(start, goal);
    }
  }

  private async aStarSearch(
    start: Position2D,
    goal: Position2D
  ): Promise<PlannedPath> {
    const openSet = new PriorityQueue<SearchNode>();
    const closedSet = new Set<string>();
    const cameFrom = new Map<string, string>();
    const gScore = new Map<string, number>();
    const fScore = new Map<string, number>();

    const startKey = this.positionKey(start);
    const goalKey = this.positionKey(goal);

    gScore.set(startKey, 0);
    fScore.set(startKey, this.heuristic(start, goal));
    openSet.enqueue({ position: start, priority: fScore.get(startKey)! });

    while (!openSet.isEmpty()) {
      const current = openSet.dequeue()!;
      const currentKey = this.positionKey(current.position);

      if (currentKey === goalKey) {
        return this.reconstructPath(cameFrom, current.position);
      }

      closedSet.add(currentKey);

      for (const neighbor of this.getNeighbors(current.position)) {
        const neighborKey = this.positionKey(neighbor);

        if (closedSet.has(neighborKey)) continue;
        if (!this.isTraversable(neighbor)) continue;

        const tentativeG = gScore.get(currentKey)! +
          this.getMovementCost(current.position, neighbor);

        if (!gScore.has(neighborKey) || tentativeG < gScore.get(neighborKey)!) {
          cameFrom.set(neighborKey, currentKey);
          gScore.set(neighborKey, tentativeG);
          fScore.set(neighborKey, tentativeG + this.heuristic(neighbor, goal));

          if (!openSet.contains(neighborKey)) {
            openSet.enqueue({
              position: neighbor,
              priority: fScore.get(neighborKey)!
            });
          }
        }
      }
    }

    // 경로를 찾지 못함
    return { valid: false, waypoints: [], distance: 0 };
  }
}

// 로컬 경로 플래너 (Dynamic Window Approach)
class DynamicWindowPlanner {
  private config: DWAConfig;
  private costMap: CostMap;

  constructor(config: DWAConfig) {
    this.config = config;
    this.costMap = new CostMap();
  }

  computeVelocity(
    currentState: RobotState,
    globalPath: PlannedPath,
    obstacles: Obstacle[]
  ): VelocityCommand {
    // 현재 속도와 제약 조건에 따라 동적 창 가져오기
    const dynamicWindow = this.getDynamicWindow(currentState.velocity);

    // 동적 창에서 속도 샘플링
    const candidates = this.sampleVelocities(dynamicWindow);

    // 각 후보 평가
    let bestVelocity: VelocityCommand = { linear: 0, angular: 0 };
    let bestScore = -Infinity;

    for (const velocity of candidates) {
      // 궤적 시뮬레이션
      const trajectory = this.simulateTrajectory(
        currentState.pose,
        velocity,
        this.config.simulationTime
      );

      // 충돌 확인
      if (this.checkCollision(trajectory, obstacles)) continue;

      // 점수 계산
      const score = this.evaluateTrajectory(
        trajectory,
        velocity,
        globalPath,
        currentState.pose
      );

      if (score > bestScore) {
        bestScore = score;
        bestVelocity = velocity;
      }
    }

    return bestVelocity;
  }
}
```

### 5.3 커버리지 계획 프로토콜

```typescript
// 청소를 위한 커버리지 계획
interface CoveragePlanningProtocol {
  algorithm: CoveragePlannerAlgorithm;
  cellDecomposition: CellDecompositionMethod;
  sweepDirection: SweepDirectionStrategy;
  overlapWidth: number;           // 미터
  robotWidth: number;             // 미터
}

// 커버리지 플래너 구현
class CoveragePlanner {
  private config: CoveragePlanningProtocol;
  private decomposer: CellDecomposer;

  constructor(config: CoveragePlanningProtocol) {
    this.config = config;
    this.decomposer = new CellDecomposer(config.cellDecomposition);
  }

  async planCoverage(
    area: Polygon,
    obstacles: Polygon[],
    startPosition: Position2D
  ): Promise<CoveragePlan> {
    // 영역을 셀로 분해
    const cells = await this.decomposer.decompose(area, obstacles);

    // 셀 방문 최적 순서 결정
    const orderedCells = this.orderCells(cells, startPosition);

    // 각 셀에 대한 커버리지 경로 생성
    const paths: CellPath[] = [];
    let currentPosition = startPosition;

    for (const cell of orderedCells) {
      // 셀에 대한 스위프 경로 생성
      const cellPath = this.generateCellPath(cell, currentPosition);
      paths.push(cellPath);

      currentPosition = cellPath.endPosition;
    }

    // 셀 간 전환 경로 생성
    const fullPath = this.connectCellPaths(paths);

    return {
      cells: orderedCells,
      paths,
      fullPath,
      totalDistance: this.calculateTotalDistance(fullPath),
      estimatedTime: this.estimateTime(fullPath),
      coverage: this.estimateCoverage(area, fullPath)
    };
  }

  private generateCellPath(
    cell: DecomposedCell,
    entryPoint: Position2D
  ): CellPath {
    // 스위프 방향 결정
    const sweepDirection = this.determineSweepDirection(cell);

    // 패스 수 계산
    const effectiveWidth = this.config.robotWidth - this.config.overlapWidth;
    const cellWidth = this.getCellWidth(cell, sweepDirection);
    const numPasses = Math.ceil(cellWidth / effectiveWidth);

    // Boustrophedon 패턴 생성
    const waypoints: Position2D[] = [];
    const { startY, endY, sweepLines } = this.calculateSweepLines(
      cell,
      sweepDirection,
      numPasses
    );

    for (let i = 0; i < sweepLines.length; i++) {
      const line = sweepLines[i];

      if (i % 2 === 0) {
        // 전방 스위프
        waypoints.push(line.start, line.end);
      } else {
        // 역방향 스위프
        waypoints.push(line.end, line.start);
      }
    }

    // 진입/퇴출 최적화
    const optimizedPath = this.optimizeEntryExit(waypoints, entryPoint);

    return {
      cell,
      waypoints: optimizedPath,
      startPosition: optimizedPath[0],
      endPosition: optimizedPath[optimizedPath.length - 1],
      sweepDirection,
      numPasses
    };
  }
}

interface CoveragePlan {
  cells: DecomposedCell[];
  paths: CellPath[];
  fullPath: Position2D[];
  totalDistance: number;
  estimatedTime: number;
  coverage: number;
}
```

### 5.4 장애물 회피 프로토콜

```typescript
// 장애물 회피 프로토콜
interface ObstacleAvoidanceProtocol {
  detectionRange: number;         // 미터
  safetyMargin: number;           // 미터
  emergencyStopDistance: number;  // 미터

  avoidanceStrategy: AvoidanceStrategy;
  replanningThreshold: number;    // 미터
}

type AvoidanceStrategy = 'REACTIVE' | 'PREDICTIVE' | 'HYBRID';

// 장애물 회피 시스템
class ObstacleAvoidanceSystem {
  private config: ObstacleAvoidanceProtocol;
  private obstacleTracker: ObstacleTracker;
  private velocityObstacles: VelocityObstacleCalculator;

  constructor(config: ObstacleAvoidanceProtocol) {
    this.config = config;
    this.obstacleTracker = new ObstacleTracker();
    this.velocityObstacles = new VelocityObstacleCalculator();
  }

  async processObstacles(
    sensorData: SensorData,
    robotState: RobotState
  ): Promise<ObstacleAvoidanceResult> {
    // 장애물 추적 업데이트
    const obstacles = await this.obstacleTracker.update(sensorData);

    // 비상 정지 확인
    if (this.requiresEmergencyStop(obstacles, robotState)) {
      return {
        action: 'EMERGENCY_STOP',
        velocity: { linear: 0, angular: 0 },
        reason: '비상 정지 거리 내 장애물'
      };
    }

    // 안전한 속도 계산
    const safeVelocities = this.calculateSafeVelocities(
      obstacles,
      robotState
    );

    if (safeVelocities.length === 0) {
      return {
        action: 'STOP',
        velocity: { linear: 0, angular: 0 },
        reason: '사용 가능한 안전한 속도 없음'
      };
    }

    // 최적의 안전 속도 선택
    const bestVelocity = this.selectBestVelocity(
      safeVelocities,
      robotState.desiredVelocity
    );

    return {
      action: 'AVOID',
      velocity: bestVelocity,
      safeVelocities,
      obstacles
    };
  }

  private requiresEmergencyStop(
    obstacles: TrackedObstacle[],
    robotState: RobotState
  ): boolean {
    for (const obstacle of obstacles) {
      const distance = this.distanceToObstacle(robotState.pose, obstacle);
      if (distance < this.config.emergencyStopDistance) {
        return true;
      }
    }
    return false;
  }

  private selectBestVelocity(
    safeVelocities: VelocityCommand[],
    desiredVelocity: VelocityCommand
  ): VelocityCommand {
    let best = safeVelocities[0];
    let minDistance = Infinity;

    for (const velocity of safeVelocities) {
      const distance = Math.hypot(
        velocity.linear - desiredVelocity.linear,
        velocity.angular - desiredVelocity.angular
      );

      if (distance < minDistance) {
        minDistance = distance;
        best = velocity;
      }
    }

    return best;
  }
}
```

### 5.5 청소 모드 프로토콜

```typescript
// 청소 모드 프로토콜
interface CleaningModeProtocol {
  modes: {
    auto: AutoCleaningProtocol;
    spot: SpotCleaningProtocol;
    edge: EdgeCleaningProtocol;
    room: RoomCleaningProtocol;
    zone: ZoneCleaningProtocol;
  };

  transitions: ModeTransition[];
  interruptHandling: InterruptHandling;
}

interface AutoCleaningProtocol {
  coverageStrategy: CoveragePlannerAlgorithm;
  adaptiveCleaning: boolean;
  dirtDetectResponse: DirtDetectResponse;
  carpetBoost: CarpetBoostConfig;
  resumeAfterCharge: boolean;
}

interface SpotCleaningProtocol {
  pattern: 'SPIRAL' | 'ZIGZAG' | 'EXPANDING_SQUARE';
  radius: number;                 // 미터
  passes: number;
  intensity: 'NORMAL' | 'INTENSIVE';
}

interface EdgeCleaningProtocol {
  wallFollowingDistance: number;  // 미터
  cornerBehavior: 'SLOW_TURN' | 'BACKUP_TURN' | 'PIVOT';
  sideBrushSpeed: 'NORMAL' | 'HIGH';
}

// 청소 컨트롤러
class CleaningController {
  private currentMode: CleaningMode;
  private protocol: CleaningModeProtocol;
  private coveragePlanner: CoveragePlanner;
  private pathPlanner: GlobalPathPlanner;

  async startCleaning(
    task: CleaningTask,
    map: RobotMap
  ): Promise<void> {
    this.currentMode = this.determineMode(task);

    switch (this.currentMode) {
      case 'AUTO':
        await this.startAutoCleaning(task, map);
        break;
      case 'SPOT':
        await this.startSpotCleaning(task);
        break;
      case 'EDGE':
        await this.startEdgeCleaning(task, map);
        break;
      case 'ROOM':
        await this.startRoomCleaning(task, map);
        break;
      case 'ZONE':
        await this.startZoneCleaning(task, map);
        break;
    }
  }

  private async startAutoCleaning(
    task: CleaningTask,
    map: RobotMap
  ): Promise<void> {
    // 청소 영역 가져오기
    const cleaningArea = this.extractCleaningArea(map, task.target);

    // 장애물 추출
    const obstacles = this.extractObstacles(map);

    // 커버리지 계획 생성
    const coveragePlan = await this.coveragePlanner.planCoverage(
      cleaningArea,
      obstacles,
      task.startPosition
    );

    // 커버리지 계획 실행
    await this.executeCoveragePlan(coveragePlan, task.settings);
  }

  private async startSpotCleaning(task: CleaningTask): Promise<void> {
    const protocol = this.protocol.modes.spot;
    const center = task.target.spotCenter!;

    // 나선형 경로 생성
    const path = this.generateSpotPath(
      center,
      protocol.radius,
      protocol.pattern
    );

    // 패스 실행
    for (let pass = 0; pass < protocol.passes; pass++) {
      await this.executePath(path, task.settings);
    }
  }

  private generateSpotPath(
    center: Position2D,
    radius: number,
    pattern: string
  ): Position2D[] {
    const path: Position2D[] = [];

    if (pattern === 'SPIRAL') {
      // 아르키메데스 나선 생성
      const spiralSpacing = 0.05;  // 회전 간 5cm
      const totalTurns = radius / spiralSpacing;

      for (let theta = 0; theta < totalTurns * 2 * Math.PI; theta += 0.1) {
        const r = spiralSpacing * theta / (2 * Math.PI);
        path.push({
          x: center.x + r * Math.cos(theta),
          y: center.y + r * Math.sin(theta)
        });
      }

      // 역방향 나선
      const reversePath = [...path].reverse();
      path.push(...reversePath);
    }

    return path;
  }
}
```

---

**WIA-CLEANING-ROBOT 제어 프로토콜**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
