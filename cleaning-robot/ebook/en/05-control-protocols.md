# Chapter 5: Cleaning Robot Control Protocols

## Navigation, Path Planning, and Cleaning Algorithms

### 5.1 Navigation Protocol Architecture

The WIA-CLEANING-ROBOT standard defines comprehensive protocols for robot navigation, including SLAM, path planning, and obstacle avoidance. These protocols ensure efficient coverage while maintaining safety.

```typescript
// Navigation Protocol Architecture
interface NavigationProtocolArchitecture {
  version: '1.0.0';

  layers: {
    perception: {
      description: 'Sensor data acquisition and processing';
      components: ['LiDAR processing', 'Camera vision', 'Sensor fusion'];
      frequency: '10-30 Hz';
    };
    localization: {
      description: 'Robot position estimation';
      components: ['SLAM', 'Odometry', 'Landmark recognition'];
      frequency: '10-20 Hz';
    };
    mapping: {
      description: 'Environment map construction';
      components: ['Occupancy grid', 'Semantic mapping', 'Room segmentation'];
      updateFrequency: 'Real-time during exploration';
    };
    planning: {
      description: 'Path and coverage planning';
      components: ['Global planner', 'Local planner', 'Coverage planner'];
      frequency: '1-10 Hz';
    };
    control: {
      description: 'Motion execution';
      components: ['Velocity controller', 'Trajectory follower'];
      frequency: '50-100 Hz';
    };
  };
}

// SLAM Protocol
interface SLAMProtocol {
  type: SlamType;

  configuration: {
    mapResolution: number;        // meters per cell
    maxRange: number;             // meters
    minRange: number;             // meters
    updateInterval: number;       // milliseconds
    keyframeDistance: number;     // meters
    keyframeAngle: number;        // radians
    loopClosureThreshold: number;
  };

  algorithms: {
    scanMatching: ScanMatchingAlgorithm;
    loopClosure: LoopClosureAlgorithm;
    graphOptimization: GraphOptimizationAlgorithm;
    mapMerging: MapMergingAlgorithm;
  };
}

type ScanMatchingAlgorithm = 'ICP' | 'NDT' | 'CORRELATIVE' | 'HYBRID';
type LoopClosureAlgorithm = 'SCAN_CONTEXT' | 'BAG_OF_WORDS' | 'DNN_BASED';
type GraphOptimizationAlgorithm = 'G2O' | 'GTSAM' | 'CERES';

// SLAM Implementation
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
    // Predict pose from odometry
    const predictedPose = this.predictPose(odometry);

    // Scan matching for pose correction
    const matchResult = await this.scanMatcher.match(
      scan,
      this.occupancyMap,
      predictedPose
    );

    if (!matchResult.valid) {
      return { success: false, reason: 'Scan matching failed' };
    }

    const correctedPose = matchResult.pose;
    const covariance = matchResult.covariance;

    // Add to pose graph
    const node = this.poseGraph.addNode(correctedPose, scan);

    // Check for loop closure
    const loopClosure = await this.loopCloser.detect(scan, correctedPose);
    if (loopClosure) {
      this.poseGraph.addLoopConstraint(loopClosure);
      await this.optimizePoseGraph();
    }

    // Update occupancy map
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
    // Ray casting for occupancy update
    for (const point of scan.points) {
      const worldPoint = this.transformToWorld(point, pose);
      const rayEnd = { x: worldPoint.x, y: worldPoint.y };
      const rayStart = { x: pose.x, y: pose.y };

      // Mark cells along ray as free
      const rayCells = this.rayCast(rayStart, rayEnd);
      for (const cell of rayCells.slice(0, -1)) {
        this.occupancyMap.updateCell(cell, OccupancyValue.FREE);
      }

      // Mark end cell as occupied (if valid range)
      if (point.range < this.config.configuration.maxRange) {
        this.occupancyMap.updateCell(
          rayCells[rayCells.length - 1],
          OccupancyValue.OCCUPIED
        );
      }
    }
  }

  private async optimizePoseGraph(): Promise<void> {
    const optimizer = new GraphOptimizer(this.config.algorithms.graphOptimization);
    const optimizedPoses = await optimizer.optimize(this.poseGraph);

    // Update pose graph with optimized poses
    this.poseGraph.updatePoses(optimizedPoses);

    // Rebuild map from optimized poses
    await this.rebuildMap();
  }

  async getMap(): Promise<OccupancyGrid> {
    return this.occupancyMap.clone();
  }

  getCurrentPose(): Pose2D {
    return this.poseGraph.getCurrentPose();
  }
}

// Supporting Types
interface LidarScan {
  timestamp: number;
  points: LidarPoint[];
  angleMin: number;
  angleMax: number;
  angleIncrement: number;
  rangeMin: number;
  rangeMax: number;
}

interface LidarPoint {
  angle: number;
  range: number;
  intensity?: number;
}

interface Pose2D {
  x: number;
  y: number;
  theta: number;
}

interface Odometry {
  pose: Pose2D;
  twist: Twist2D;
  timestamp: number;
}

interface Twist2D {
  linear: number;
  angular: number;
}
```

### 5.2 Path Planning Protocol

```typescript
// Path Planning Protocol
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
type PlannerHeuristic = 'EUCLIDEAN' | 'MANHATTAN' | 'DIAGONAL' | 'OCTILE';

interface CostFactors {
  distance: number;
  obstacles: number;
  unknownArea: number;
  turnPenalty: number;
  proximityToObstacles: number;
}

// Global Path Planner
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
    // Update cost map from occupancy grid
    this.costMap.updateFromOccupancy(map);

    // Apply algorithm
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

    // No path found
    return { valid: false, waypoints: [], distance: 0 };
  }

  private getMovementCost(from: Position2D, to: Position2D): number {
    const distance = Math.hypot(to.x - from.x, to.y - from.y);
    const obstacleCost = this.costMap.getCost(to);
    return distance * (1 + obstacleCost);
  }

  private reconstructPath(
    cameFrom: Map<string, string>,
    goal: Position2D
  ): PlannedPath {
    const waypoints: Position2D[] = [goal];
    let current = this.positionKey(goal);

    while (cameFrom.has(current)) {
      current = cameFrom.get(current)!;
      waypoints.unshift(this.keyToPosition(current));
    }

    // Smooth path
    const smoothedWaypoints = this.smoothPath(waypoints);

    return {
      valid: true,
      waypoints: smoothedWaypoints,
      distance: this.calculatePathDistance(smoothedWaypoints)
    };
  }

  private smoothPath(waypoints: Position2D[]): Position2D[] {
    if (waypoints.length < 3) return waypoints;

    const smoothed: Position2D[] = [waypoints[0]];
    let i = 0;

    while (i < waypoints.length - 1) {
      let j = waypoints.length - 1;

      while (j > i + 1) {
        if (this.hasLineOfSight(waypoints[i], waypoints[j])) {
          smoothed.push(waypoints[j]);
          i = j;
          break;
        }
        j--;
      }

      if (j === i + 1) {
        smoothed.push(waypoints[i + 1]);
        i++;
      }
    }

    return smoothed;
  }
}

// Local Path Planner (Dynamic Window Approach)
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
    // Get dynamic window based on current velocity and constraints
    const dynamicWindow = this.getDynamicWindow(currentState.velocity);

    // Sample velocities in dynamic window
    const candidates = this.sampleVelocities(dynamicWindow);

    // Evaluate each candidate
    let bestVelocity: VelocityCommand = { linear: 0, angular: 0 };
    let bestScore = -Infinity;

    for (const velocity of candidates) {
      // Simulate trajectory
      const trajectory = this.simulateTrajectory(
        currentState.pose,
        velocity,
        this.config.simulationTime
      );

      // Check for collisions
      if (this.checkCollision(trajectory, obstacles)) continue;

      // Calculate score
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

  private getDynamicWindow(currentVelocity: Twist2D): DynamicWindow {
    return {
      minLinear: Math.max(
        this.config.minLinearVelocity,
        currentVelocity.linear - this.config.maxLinearAcceleration * this.config.dt
      ),
      maxLinear: Math.min(
        this.config.maxLinearVelocity,
        currentVelocity.linear + this.config.maxLinearAcceleration * this.config.dt
      ),
      minAngular: Math.max(
        -this.config.maxAngularVelocity,
        currentVelocity.angular - this.config.maxAngularAcceleration * this.config.dt
      ),
      maxAngular: Math.min(
        this.config.maxAngularVelocity,
        currentVelocity.angular + this.config.maxAngularAcceleration * this.config.dt
      )
    };
  }

  private simulateTrajectory(
    startPose: Pose2D,
    velocity: VelocityCommand,
    duration: number
  ): Trajectory {
    const points: TrajectoryPoint[] = [];
    let pose = { ...startPose };
    const dt = 0.1;

    for (let t = 0; t <= duration; t += dt) {
      points.push({
        pose: { ...pose },
        timestamp: t
      });

      // Simple kinematics
      pose.x += velocity.linear * Math.cos(pose.theta) * dt;
      pose.y += velocity.linear * Math.sin(pose.theta) * dt;
      pose.theta += velocity.angular * dt;
    }

    return { points };
  }

  private evaluateTrajectory(
    trajectory: Trajectory,
    velocity: VelocityCommand,
    globalPath: PlannedPath,
    currentPose: Pose2D
  ): number {
    const endPose = trajectory.points[trajectory.points.length - 1].pose;

    // Heading score - alignment with path
    const headingScore = this.calculateHeadingScore(endPose, globalPath);

    // Distance score - progress along path
    const distanceScore = this.calculateDistanceScore(endPose, globalPath);

    // Velocity score - prefer faster movement
    const velocityScore = velocity.linear / this.config.maxLinearVelocity;

    // Obstacle score - prefer staying away from obstacles
    const obstacleScore = this.calculateObstacleScore(trajectory);

    return (
      this.config.headingWeight * headingScore +
      this.config.distanceWeight * distanceScore +
      this.config.velocityWeight * velocityScore +
      this.config.obstacleWeight * obstacleScore
    );
  }
}

interface DWAConfig {
  maxLinearVelocity: number;
  minLinearVelocity: number;
  maxAngularVelocity: number;
  maxLinearAcceleration: number;
  maxAngularAcceleration: number;
  dt: number;
  simulationTime: number;
  velocityResolution: number;
  angularResolution: number;
  headingWeight: number;
  distanceWeight: number;
  velocityWeight: number;
  obstacleWeight: number;
}
```

### 5.3 Coverage Planning Protocol

```typescript
// Coverage Planning for Cleaning
interface CoveragePlanningProtocol {
  algorithm: CoveragePlannerAlgorithm;
  cellDecomposition: CellDecompositionMethod;
  sweepDirection: SweepDirectionStrategy;
  overlapWidth: number;           // meters
  robotWidth: number;             // meters
}

type CellDecompositionMethod = 'BOUSTROPHEDON' | 'TRAPEZOIDAL' | 'EXACT' | 'GRID';
type SweepDirectionStrategy = 'FIXED' | 'OPTIMAL' | 'WALL_FOLLOWING';

// Coverage Planner Implementation
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
    // Decompose area into cells
    const cells = await this.decomposer.decompose(area, obstacles);

    // Determine optimal order to visit cells
    const orderedCells = this.orderCells(cells, startPosition);

    // Generate coverage path for each cell
    const paths: CellPath[] = [];
    let currentPosition = startPosition;

    for (const cell of orderedCells) {
      // Generate sweep path for cell
      const cellPath = this.generateCellPath(cell, currentPosition);
      paths.push(cellPath);

      currentPosition = cellPath.endPosition;
    }

    // Generate transition paths between cells
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
    // Determine sweep direction
    const sweepDirection = this.determineSweepDirection(cell);

    // Calculate number of passes
    const effectiveWidth = this.config.robotWidth - this.config.overlapWidth;
    const cellWidth = this.getCellWidth(cell, sweepDirection);
    const numPasses = Math.ceil(cellWidth / effectiveWidth);

    // Generate boustrophedon pattern
    const waypoints: Position2D[] = [];
    const { startY, endY, sweepLines } = this.calculateSweepLines(
      cell,
      sweepDirection,
      numPasses
    );

    for (let i = 0; i < sweepLines.length; i++) {
      const line = sweepLines[i];

      if (i % 2 === 0) {
        // Forward sweep
        waypoints.push(line.start, line.end);
      } else {
        // Reverse sweep
        waypoints.push(line.end, line.start);
      }
    }

    // Optimize entry/exit
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

  private calculateSweepLines(
    cell: DecomposedCell,
    direction: number,
    numPasses: number
  ): { startY: number; endY: number; sweepLines: SweepLine[] } {
    // Rotate cell to align with sweep direction
    const rotatedCell = this.rotateCell(cell, -direction);

    // Get bounding box
    const bbox = this.getBoundingBox(rotatedCell);

    const effectiveWidth = this.config.robotWidth - this.config.overlapWidth;
    const startY = bbox.minY + this.config.robotWidth / 2;
    const endY = bbox.maxY - this.config.robotWidth / 2;

    const sweepLines: SweepLine[] = [];

    for (let i = 0; i < numPasses; i++) {
      const y = startY + i * effectiveWidth;

      // Find intersection with cell boundary
      const intersection = this.findCellIntersection(rotatedCell, y);

      if (intersection) {
        // Rotate back to original orientation
        const rotatedLine = this.rotateLine(intersection, direction);
        sweepLines.push(rotatedLine);
      }
    }

    return { startY, endY, sweepLines };
  }

  private orderCells(
    cells: DecomposedCell[],
    startPosition: Position2D
  ): DecomposedCell[] {
    // Use nearest neighbor heuristic with optimization
    const ordered: DecomposedCell[] = [];
    const remaining = new Set(cells.map((_, i) => i));
    let currentPosition = startPosition;

    while (remaining.size > 0) {
      let nearestIndex = -1;
      let nearestDistance = Infinity;

      for (const index of remaining) {
        const cell = cells[index];
        const entryPoint = this.findBestEntryPoint(cell, currentPosition);
        const distance = this.distance(currentPosition, entryPoint);

        if (distance < nearestDistance) {
          nearestDistance = distance;
          nearestIndex = index;
        }
      }

      ordered.push(cells[nearestIndex]);
      currentPosition = this.getCellCentroid(cells[nearestIndex]);
      remaining.delete(nearestIndex);
    }

    // Optional: Apply 2-opt optimization
    return this.twoOptOptimization(ordered);
  }

  private determineSweepDirection(cell: DecomposedCell): number {
    switch (this.config.sweepDirection) {
      case 'FIXED':
        return 0;  // Horizontal

      case 'OPTIMAL':
        // Find direction that minimizes number of turns
        return this.findOptimalSweepDirection(cell);

      case 'WALL_FOLLOWING':
        // Align with longest edge
        return this.findLongestEdgeDirection(cell);

      default:
        return 0;
    }
  }

  private findOptimalSweepDirection(cell: DecomposedCell): number {
    let bestDirection = 0;
    let minWidth = Infinity;

    // Sample directions
    for (let angle = 0; angle < Math.PI; angle += Math.PI / 18) {
      const width = this.getCellWidth(cell, angle);
      if (width < minWidth) {
        minWidth = width;
        bestDirection = angle;
      }
    }

    return bestDirection;
  }
}

// Cell Decomposer
class CellDecomposer {
  private method: CellDecompositionMethod;

  constructor(method: CellDecompositionMethod) {
    this.method = method;
  }

  async decompose(
    area: Polygon,
    obstacles: Polygon[]
  ): Promise<DecomposedCell[]> {
    switch (this.method) {
      case 'BOUSTROPHEDON':
        return this.boustrophedonDecomposition(area, obstacles);
      case 'TRAPEZOIDAL':
        return this.trapezoidalDecomposition(area, obstacles);
      case 'GRID':
        return this.gridDecomposition(area, obstacles);
      default:
        return this.boustrophedonDecomposition(area, obstacles);
    }
  }

  private async boustrophedonDecomposition(
    area: Polygon,
    obstacles: Polygon[]
  ): Promise<DecomposedCell[]> {
    // Create event list (all vertices sorted by x)
    const events = this.createEventList(area, obstacles);

    const cells: DecomposedCell[] = [];
    const activeCells: Map<string, Partial<DecomposedCell>> = new Map();

    for (const event of events) {
      switch (event.type) {
        case 'OPEN':
          // Start new cell
          const newCell = this.startCell(event);
          activeCells.set(newCell.id!, newCell);
          break;

        case 'CLOSE':
          // Close existing cell
          const closedCell = this.closeCell(event, activeCells);
          if (closedCell) {
            cells.push(closedCell as DecomposedCell);
          }
          break;

        case 'SPLIT':
          // Split cell into two
          const splitCells = this.splitCell(event, activeCells);
          for (const cell of splitCells) {
            activeCells.set(cell.id!, cell);
          }
          break;

        case 'MERGE':
          // Merge two cells into one
          const mergedCell = this.mergeCells(event, activeCells);
          activeCells.set(mergedCell.id!, mergedCell);
          break;
      }
    }

    return cells;
  }
}

interface DecomposedCell {
  id: string;
  polygon: Polygon;
  neighbors: string[];
  area: number;
  centroid: Position2D;
}

interface CoveragePlan {
  cells: DecomposedCell[];
  paths: CellPath[];
  fullPath: Position2D[];
  totalDistance: number;
  estimatedTime: number;
  coverage: number;
}

interface CellPath {
  cell: DecomposedCell;
  waypoints: Position2D[];
  startPosition: Position2D;
  endPosition: Position2D;
  sweepDirection: number;
  numPasses: number;
}

interface SweepLine {
  start: Position2D;
  end: Position2D;
}
```

### 5.4 Obstacle Avoidance Protocol

```typescript
// Obstacle Avoidance Protocol
interface ObstacleAvoidanceProtocol {
  detectionRange: number;         // meters
  safetyMargin: number;           // meters
  emergencyStopDistance: number;  // meters

  avoidanceStrategy: AvoidanceStrategy;
  replanningThreshold: number;    // meters
}

type AvoidanceStrategy = 'REACTIVE' | 'PREDICTIVE' | 'HYBRID';

// Obstacle Avoidance System
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
    // Update obstacle tracking
    const obstacles = await this.obstacleTracker.update(sensorData);

    // Check for emergency stop
    if (this.requiresEmergencyStop(obstacles, robotState)) {
      return {
        action: 'EMERGENCY_STOP',
        velocity: { linear: 0, angular: 0 },
        reason: 'Obstacle within emergency stop distance'
      };
    }

    // Calculate safe velocities
    const safeVelocities = this.calculateSafeVelocities(
      obstacles,
      robotState
    );

    if (safeVelocities.length === 0) {
      return {
        action: 'STOP',
        velocity: { linear: 0, angular: 0 },
        reason: 'No safe velocity available'
      };
    }

    // Select best safe velocity
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

  private calculateSafeVelocities(
    obstacles: TrackedObstacle[],
    robotState: RobotState
  ): VelocityCommand[] {
    // Build velocity obstacles
    const vos = this.velocityObstacles.calculate(
      robotState,
      obstacles,
      this.config.safetyMargin
    );

    // Sample velocity space
    const candidates = this.sampleVelocitySpace(robotState);

    // Filter out velocities in velocity obstacles
    return candidates.filter(v => !this.isInVelocityObstacle(v, vos));
  }

  private isInVelocityObstacle(
    velocity: VelocityCommand,
    velocityObstacles: VelocityObstacle[]
  ): boolean {
    const velocityPoint = {
      x: velocity.linear * Math.cos(velocity.angular),
      y: velocity.linear * Math.sin(velocity.angular)
    };

    for (const vo of velocityObstacles) {
      if (this.pointInCone(velocityPoint, vo)) {
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

// Obstacle Tracker
class ObstacleTracker {
  private trackedObstacles: Map<string, TrackedObstacle>;
  private nextId: number = 0;

  async update(sensorData: SensorData): Promise<TrackedObstacle[]> {
    // Extract obstacle detections from sensor data
    const detections = this.extractDetections(sensorData);

    // Associate detections with existing tracks
    const associations = this.associateDetections(detections);

    // Update existing tracks
    for (const [trackId, detection] of associations) {
      const track = this.trackedObstacles.get(trackId)!;
      this.updateTrack(track, detection);
    }

    // Create new tracks for unassociated detections
    for (const detection of detections) {
      if (!associations.has(detection.id)) {
        this.createTrack(detection);
      }
    }

    // Remove stale tracks
    this.removeStaleTrackers();

    return Array.from(this.trackedObstacles.values());
  }

  private associateDetections(
    detections: ObstacleDetection[]
  ): Map<string, ObstacleDetection> {
    const associations = new Map<string, ObstacleDetection>();

    for (const detection of detections) {
      let bestTrack: string | null = null;
      let minDistance = Infinity;

      for (const [trackId, track] of this.trackedObstacles) {
        const predicted = this.predictPosition(track);
        const distance = this.distance(detection.position, predicted);

        if (distance < this.associationThreshold && distance < minDistance) {
          minDistance = distance;
          bestTrack = trackId;
        }
      }

      if (bestTrack) {
        associations.set(bestTrack, detection);
      }
    }

    return associations;
  }

  private updateTrack(
    track: TrackedObstacle,
    detection: ObstacleDetection
  ): void {
    // Kalman filter update
    const kalman = track.kalmanFilter;
    kalman.predict();
    kalman.update(detection.position);

    track.position = kalman.getState();
    track.velocity = kalman.getVelocity();
    track.lastSeen = Date.now();
    track.confidence = Math.min(track.confidence + 0.1, 1.0);
  }

  private createTrack(detection: ObstacleDetection): void {
    const id = `obstacle_${this.nextId++}`;
    const kalman = new KalmanFilter2D(detection.position);

    this.trackedObstacles.set(id, {
      id,
      position: detection.position,
      velocity: { x: 0, y: 0 },
      size: detection.size,
      type: detection.type,
      confidence: 0.5,
      lastSeen: Date.now(),
      kalmanFilter: kalman
    });
  }
}

interface TrackedObstacle {
  id: string;
  position: Position2D;
  velocity: Position2D;
  size: { width: number; height: number };
  type: ObstacleType;
  confidence: number;
  lastSeen: number;
  kalmanFilter: KalmanFilter2D;
}

type ObstacleType = 'STATIC' | 'DYNAMIC' | 'TEMPORARY' | 'UNKNOWN';
```

### 5.5 Cleaning Mode Protocols

```typescript
// Cleaning Mode Protocols
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
  radius: number;                 // meters
  passes: number;
  intensity: 'NORMAL' | 'INTENSIVE';
}

interface EdgeCleaningProtocol {
  wallFollowingDistance: number;  // meters
  cornerBehavior: 'SLOW_TURN' | 'BACKUP_TURN' | 'PIVOT';
  sideBrushSpeed: 'NORMAL' | 'HIGH';
}

// Cleaning Controller
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
    // Get cleaning area
    const cleaningArea = this.extractCleaningArea(map, task.target);

    // Extract obstacles
    const obstacles = this.extractObstacles(map);

    // Generate coverage plan
    const coveragePlan = await this.coveragePlanner.planCoverage(
      cleaningArea,
      obstacles,
      task.startPosition
    );

    // Execute coverage plan
    await this.executeCoveragePlan(coveragePlan, task.settings);
  }

  private async startSpotCleaning(task: CleaningTask): Promise<void> {
    const protocol = this.protocol.modes.spot;
    const center = task.target.spotCenter!;

    // Generate spiral path
    const path = this.generateSpotPath(
      center,
      protocol.radius,
      protocol.pattern
    );

    // Execute with passes
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
      // Generate Archimedean spiral
      const spiralSpacing = 0.05;  // 5cm between turns
      const totalTurns = radius / spiralSpacing;

      for (let theta = 0; theta < totalTurns * 2 * Math.PI; theta += 0.1) {
        const r = spiralSpacing * theta / (2 * Math.PI);
        path.push({
          x: center.x + r * Math.cos(theta),
          y: center.y + r * Math.sin(theta)
        });
      }

      // Reverse spiral back
      const reversePath = [...path].reverse();
      path.push(...reversePath);
    }

    return path;
  }

  private async startEdgeCleaning(
    task: CleaningTask,
    map: RobotMap
  ): Promise<void> {
    const protocol = this.protocol.modes.edge;

    // Extract wall segments
    const walls = this.extractWallSegments(map);

    // Generate edge following path
    const edgePath = this.generateEdgePath(
      walls,
      protocol.wallFollowingDistance,
      task.startPosition
    );

    // Execute edge cleaning
    await this.executePath(edgePath, {
      ...task.settings,
      sideBrushSpeed: protocol.sideBrushSpeed
    });
  }

  private async startRoomCleaning(
    task: CleaningTask,
    map: RobotMap
  ): Promise<void> {
    const roomIds = task.target.rooms!;
    const roomOrder = this.optimizeRoomOrder(roomIds, map, task.startPosition);

    for (const roomId of roomOrder) {
      const room = map.rooms.find(r => r.id === roomId);
      if (!room) continue;

      // Get room-specific settings
      const roomSettings = task.settings.roomSettings?.get(roomId) || task.settings;

      // Generate room coverage plan
      const roomPlan = await this.coveragePlanner.planCoverage(
        room.geometry.boundary,
        this.extractRoomObstacles(map, room),
        task.startPosition
      );

      // Execute room cleaning
      await this.executeCoveragePlan(roomPlan, roomSettings);
    }
  }

  private async executeCoveragePlan(
    plan: CoveragePlan,
    settings: CleaningSettings
  ): Promise<void> {
    // Configure cleaning systems
    await this.configureCleaningSystems(settings);

    for (const cellPath of plan.paths) {
      // Navigate to cell entry point
      await this.navigateTo(cellPath.startPosition);

      // Execute cell cleaning
      for (let i = 0; i < cellPath.waypoints.length - 1; i++) {
        const from = cellPath.waypoints[i];
        const to = cellPath.waypoints[i + 1];

        // Execute cleaning pass
        await this.executeCleaningPass(from, to, settings);

        // Check for dirt detection response
        if (settings.adaptiveSettings.dirtDetectMode) {
          const dirtLevel = await this.getDirtLevel();
          if (dirtLevel > 0.7) {
            // Repeat pass
            await this.executeCleaningPass(from, to, settings);
          }
        }
      }
    }
  }
}
```

---

**WIA-CLEANING-ROBOT Control Protocols**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
