# Chapter 3: Cleaning Robot Data Formats

## Robot State, Mapping, and Operational Data Models

### 3.1 Core Data Model Architecture

The WIA-CLEANING-ROBOT standard defines comprehensive data formats for representing robot state, environmental maps, cleaning tasks, and operational telemetry. These formats ensure interoperability between robots, cloud platforms, and third-party applications.

```typescript
// Core Robot Data Model
interface CleaningRobotDataModel {
  version: '1.0.0';

  dataCategories: {
    robotState: {
      description: 'Current robot status and telemetry';
      updateFrequency: 'Real-time (100ms-1s)';
      storage: 'In-memory with periodic persistence';
    };
    environmentMap: {
      description: 'Spatial representation of cleaning area';
      updateFrequency: 'Per cleaning session or on-demand';
      storage: 'Persistent local + cloud sync';
    };
    cleaningTask: {
      description: 'Task definitions and progress';
      updateFrequency: 'Event-driven';
      storage: 'Persistent with history';
    };
    telemetry: {
      description: 'Operational metrics and sensor data';
      updateFrequency: 'High-frequency (10-100Hz)';
      storage: 'Time-series database';
    };
    maintenance: {
      description: 'Component health and service records';
      updateFrequency: 'Per session + periodic checks';
      storage: 'Persistent with full history';
    };
  };
}

// Robot Identity and Configuration
interface RobotIdentity {
  id: string;                          // UUID v4
  serialNumber: string;                 // Manufacturer serial
  macAddress: string;                   // Primary MAC address
  model: RobotModel;
  manufacturer: string;
  manufactureDate: Date;
  firmwareVersion: SemanticVersion;
  hardwareRevision: string;

  registration: {
    registeredAt: Date;
    registeredBy: string;
    accountId: string;
    homeId: string;
    nickname: string;
  };

  capabilities: RobotCapabilities;
  configuration: RobotConfiguration;
}

interface RobotModel {
  name: string;
  series: string;
  generation: number;
  releaseYear: number;

  specifications: {
    dimensions: Dimensions;
    weight: number;              // grams
    batteryCapacity: number;     // mAh
    dustbinCapacity: number;     // ml
    waterTankCapacity: number;   // ml
    maxSuction: number;          // Pa
    noiseLevel: number;          // dB
  };
}

interface Dimensions {
  diameter: number;     // mm
  height: number;       // mm
  climbHeight: number;  // mm
}

interface SemanticVersion {
  major: number;
  minor: number;
  patch: number;
  build?: string;

  toString(): string;
  compare(other: SemanticVersion): number;
}

interface RobotCapabilities {
  // Cleaning capabilities
  cleaning: {
    vacuuming: boolean;
    mopping: boolean;
    scrubbing: boolean;
    uvDisinfection: boolean;
    steamCleaning: boolean;

    suctionLevels: SuctionLevel[];
    mopWetnessLevels: WetnessLevel[];
    cleaningPatterns: CleaningPattern[];
  };

  // Navigation capabilities
  navigation: {
    slamType: SlamType;
    multiFloorMapping: boolean;
    maxFloors: number;
    realTimeObjectRecognition: boolean;
    recognizedObjects: ObjectType[];
    virtualBoundaries: boolean;
    noGoZones: boolean;
    selectiveRoomCleaning: boolean;
  };

  // Docking capabilities
  docking: {
    autoReturn: boolean;
    autoResume: boolean;
    autoEmptyDust: boolean;
    autoWashMop: boolean;
    autoRefillWater: boolean;
    autoDryMop: boolean;
  };

  // Connectivity
  connectivity: {
    wifi: WifiCapability;
    bluetooth: BluetoothCapability;
    voiceAssistants: VoiceAssistant[];
    smartHomeProtocols: SmartHomeProtocol[];
  };

  // Sensors
  sensors: {
    lidar: boolean;
    camera: CameraCapability[];
    tof: boolean;
    ultrasonic: boolean;
    cliff: boolean;
    bumper: boolean;
    dirtDetection: DirtDetectionType;
    carpetDetection: boolean;
    floorType: boolean;
  };
}

type SlamType = 'LIDAR_SLAM' | 'VISUAL_SLAM' | 'HYBRID_SLAM' | 'BASIC_NAVIGATION';
type DirtDetectionType = 'NONE' | 'ACOUSTIC' | 'OPTICAL' | 'ACOUSTIC_OPTICAL';
type VoiceAssistant = 'ALEXA' | 'GOOGLE_ASSISTANT' | 'SIRI' | 'BIXBY';
type SmartHomeProtocol = 'MATTER' | 'HOMEKIT' | 'ZIGBEE' | 'ZWAVE' | 'THREAD';
```

### 3.2 Robot State Representation

```typescript
// Comprehensive Robot State Model
interface RobotState {
  robotId: string;
  timestamp: Date;

  // Operating state
  operatingState: OperatingState;

  // Position and pose
  pose: RobotPose;

  // Power status
  power: PowerState;

  // Cleaning system status
  cleaningSystem: CleaningSystemState;

  // Sensor status
  sensors: SensorState;

  // Connectivity status
  connectivity: ConnectivityState;

  // Current task
  currentTask: TaskState | null;

  // Error and warnings
  diagnostics: DiagnosticsState;
}

interface OperatingState {
  state: RobotOperatingState;
  mode: OperatingMode;
  subState: string | null;
  stateEnteredAt: Date;
  previousState: RobotOperatingState | null;
}

type RobotOperatingState =
  | 'IDLE'                    // Ready, not cleaning
  | 'CLEANING'                // Actively cleaning
  | 'SPOT_CLEANING'           // Spot cleaning mode
  | 'EDGE_CLEANING'           // Edge cleaning mode
  | 'RETURNING_TO_DOCK'       // Going back to dock
  | 'DOCKING'                 // Docking in progress
  | 'CHARGING'                // On dock, charging
  | 'CHARGED'                 // On dock, fully charged
  | 'PAUSED'                  // Cleaning paused
  | 'STUCK'                   // Robot is stuck
  | 'ERROR'                   // Error state
  | 'MAINTENANCE_REQUIRED'    // Needs maintenance
  | 'EMPTYING_DUSTBIN'        // Auto-emptying dust
  | 'WASHING_MOP'             // Auto-washing mop
  | 'DRYING_MOP'              // Auto-drying mop
  | 'REFILLING_WATER'         // Auto-refilling water
  | 'MAPPING'                 // Creating new map
  | 'LOCATING'                // Trying to locate
  | 'UPDATING'                // Firmware update
  | 'MANUAL_CONTROL';         // Remote control mode

interface RobotPose {
  position: Position2D;
  orientation: number;        // radians, 0 = positive X axis
  mapId: string;
  floor: number;

  confidence: number;         // 0-1
  lastUpdated: Date;
  source: PoseSource;
}

interface Position2D {
  x: number;                  // meters from origin
  y: number;                  // meters from origin
}

type PoseSource = 'SLAM' | 'ODOMETRY' | 'DOCK_LOCALIZATION' | 'MANUAL_PLACEMENT';

interface PowerState {
  batteryLevel: number;           // 0-100 percent
  batteryVoltage: number;         // volts
  batteryCurrent: number;         // amps
  batteryTemperature: number;     // celsius

  isCharging: boolean;
  chargeRate: number;             // watts
  estimatedRuntime: number;       // minutes
  estimatedChargeTime: number;    // minutes

  batteryHealth: BatteryHealth;
  chargeCycles: number;
}

interface BatteryHealth {
  healthPercentage: number;       // 0-100
  degradationRate: number;        // percent per month
  estimatedLifeRemaining: number; // months
  lastCalibrated: Date | null;
}

interface CleaningSystemState {
  vacuum: {
    active: boolean;
    suctionLevel: SuctionLevel;
    actualSuction: number;        // Pa
    motorRPM: number;
    motorTemperature: number;     // celsius
  };

  mainBrush: {
    active: boolean;
    rpm: number;
    motorCurrent: number;         // amps
    estimatedLife: number;        // percent remaining
  };

  sideBrush: {
    active: boolean;
    rpm: number;
    motorCurrent: number;
    estimatedLife: number;
  };

  mop: {
    active: boolean;
    mode: MopMode;
    wetnessLevel: WetnessLevel;
    waterFlowRate: number;        // ml/min
    vibrationFrequency: number;   // Hz (if vibrating mop)
    rotationSpeed: number;        // RPM (if rotating mop)
    mopAttached: boolean;
    mopClean: boolean;
  };

  dustbin: {
    level: number;                // 0-100 percent
    full: boolean;
    installed: boolean;
  };

  waterTank: {
    level: number;                // 0-100 percent
    empty: boolean;
    installed: boolean;
  };

  dirtyWaterTank: {
    level: number;
    full: boolean;
    installed: boolean;
  };
}

type MopMode = 'OFF' | 'STANDARD' | 'DEEP' | 'GENTLE' | 'QUICK_DRY';

interface SensorState {
  lidar: {
    operational: boolean;
    scanRate: number;             // Hz
    pointCount: number;           // points per scan
    errorRate: number;            // percent
  };

  cameras: Array<{
    id: string;
    type: CameraType;
    operational: boolean;
    frameRate: number;
    exposure: number;
  }>;

  cliffSensors: Array<{
    position: string;
    triggered: boolean;
    value: number;
  }>;

  bumperSensors: Array<{
    position: string;
    triggered: boolean;
    force: number;
  }>;

  wheelEncoders: {
    leftWheel: EncoderReading;
    rightWheel: EncoderReading;
  };

  imu: {
    accelerometer: Vector3D;
    gyroscope: Vector3D;
    orientation: Quaternion;
  };

  dirtSensor: {
    active: boolean;
    dirtLevel: number;            // 0-100
    lastDetection: Date | null;
  };

  carpetSensor: {
    onCarpet: boolean;
    carpetHeight: number;         // mm
  };
}

type CameraType = 'RGB' | 'DEPTH' | 'STRUCTURED_LIGHT' | 'INFRARED';

interface Vector3D {
  x: number;
  y: number;
  z: number;
}

interface EncoderReading {
  ticks: number;
  velocity: number;               // rad/s
  distance: number;               // meters traveled
}

interface ConnectivityState {
  wifi: {
    connected: boolean;
    ssid: string | null;
    signalStrength: number;       // dBm
    ipAddress: string | null;
    macAddress: string;
  };

  bluetooth: {
    enabled: boolean;
    connected: boolean;
    connectedDevices: string[];
  };

  cloud: {
    connected: boolean;
    lastSync: Date | null;
    syncStatus: SyncStatus;
  };
}

type SyncStatus = 'SYNCED' | 'SYNCING' | 'PENDING' | 'ERROR';

interface DiagnosticsState {
  errors: ErrorInfo[];
  warnings: WarningInfo[];
  healthScore: number;            // 0-100
  lastDiagnosticRun: Date | null;
}

interface ErrorInfo {
  code: number;
  severity: ErrorSeverity;
  message: string;
  component: string;
  occurredAt: Date;
  resolved: boolean;
  resolvedAt: Date | null;
}

type ErrorSeverity = 'CRITICAL' | 'ERROR' | 'WARNING' | 'INFO';
```

### 3.3 Map Data Formats

```typescript
// Comprehensive Map Data Model
interface RobotMap {
  id: string;
  robotId: string;
  name: string;
  floor: number;

  metadata: MapMetadata;
  grid: OccupancyGrid;
  rooms: Room[];
  zones: Zone[];
  boundaries: VirtualBoundary[];
  landmarks: Landmark[];

  cleaningData: MapCleaningData;
}

interface MapMetadata {
  version: number;
  createdAt: Date;
  updatedAt: Date;
  source: MapSource;

  coordinateSystem: {
    origin: Position2D;
    rotation: number;             // radians
    scale: number;                // meters per unit
  };

  bounds: {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
  };

  statistics: {
    totalArea: number;            // square meters
    cleanableArea: number;
    roomCount: number;
    boundaryCount: number;
  };
}

type MapSource = 'SLAM_GENERATED' | 'USER_MODIFIED' | 'IMPORTED' | 'CLOUD_SYNCED';

interface OccupancyGrid {
  width: number;                  // cells
  height: number;                 // cells
  resolution: number;             // meters per cell

  // Cell types: 0=unknown, 1=free, 2=occupied, 3=virtual_boundary
  data: Uint8Array;

  // Additional layers
  layers: {
    floor: Uint8Array;           // Floor type per cell
    cleaned: Uint8Array;         // Clean status per cell
    probability: Float32Array;    // Occupancy probability
  };
}

interface Room {
  id: string;
  name: string;
  type: RoomType;

  geometry: {
    boundary: Polygon;
    area: number;                 // square meters
    centroid: Position2D;
  };

  cleaningSettings: RoomCleaningSettings;

  statistics: {
    totalCleanings: number;
    lastCleaned: Date | null;
    averageCleaningTime: number;  // minutes
    dirtinessLevel: number;       // 0-100
  };

  furniture: FurnitureItem[];
}

type RoomType =
  | 'LIVING_ROOM'
  | 'BEDROOM'
  | 'KITCHEN'
  | 'BATHROOM'
  | 'DINING_ROOM'
  | 'OFFICE'
  | 'HALLWAY'
  | 'ENTRANCE'
  | 'GARAGE'
  | 'UTILITY'
  | 'OTHER';

interface Polygon {
  points: Position2D[];
  holes: Position2D[][];          // Interior holes/cutouts
}

interface RoomCleaningSettings {
  enabled: boolean;
  vacuumPower: SuctionLevel;
  mopWetness: WetnessLevel;
  passes: number;
  cleaningPattern: CleaningPattern;
  schedule: RoomSchedule | null;
}

interface RoomSchedule {
  enabled: boolean;
  days: DayOfWeek[];
  time: TimeOfDay;
  repeatWeeks: number;
}

type DayOfWeek = 'MON' | 'TUE' | 'WED' | 'THU' | 'FRI' | 'SAT' | 'SUN';

interface TimeOfDay {
  hour: number;                   // 0-23
  minute: number;                 // 0-59
}

interface FurnitureItem {
  id: string;
  type: FurnitureType;
  position: Position2D;
  dimensions: {
    width: number;
    depth: number;
    height: number;
  };
  avoidance: boolean;
  detectedConfidence: number;
}

type FurnitureType =
  | 'SOFA'
  | 'TABLE'
  | 'CHAIR'
  | 'BED'
  | 'DESK'
  | 'CABINET'
  | 'SHELF'
  | 'TV_STAND'
  | 'REFRIGERATOR'
  | 'UNKNOWN';

interface Zone {
  id: string;
  name: string;
  type: ZoneType;
  geometry: Polygon;
  settings: ZoneSettings;
}

type ZoneType =
  | 'CLEANING_ZONE'
  | 'NO_GO_ZONE'
  | 'NO_MOP_ZONE'
  | 'QUIET_ZONE'
  | 'PET_ZONE'
  | 'HIGH_TRAFFIC_ZONE';

interface ZoneSettings {
  vacuumEnabled: boolean;
  mopEnabled: boolean;
  vacuumPower: SuctionLevel | null;
  mopWetness: WetnessLevel | null;
  priority: number;
}

interface VirtualBoundary {
  id: string;
  name: string;
  type: BoundaryType;
  geometry: LineString | Polygon;
  active: boolean;
  schedule: BoundarySchedule | null;
}

type BoundaryType =
  | 'VIRTUAL_WALL'
  | 'NO_GO_ZONE'
  | 'NO_MOP_ZONE'
  | 'THRESHOLD'
  | 'KEEP_OUT';

interface LineString {
  points: Position2D[];
}

interface Landmark {
  id: string;
  type: LandmarkType;
  position: Position2D;
  confidence: number;
  lastSeen: Date;
  features: Map<string, any>;
}

type LandmarkType =
  | 'DOCK'
  | 'DOOR'
  | 'FURNITURE'
  | 'WALL_FEATURE'
  | 'FLOOR_TRANSITION';

interface MapCleaningData {
  heatmap: CleaningHeatmap;
  coverage: CoverageData;
  dirtHistory: DirtHistoryData;
}

interface CleaningHeatmap {
  resolution: number;
  data: Float32Array;             // Cleaning frequency per cell
  lastUpdated: Date;
}

interface CoverageData {
  totalCleanableArea: number;
  averageCoverage: number;
  coverageByRoom: Map<string, number>;
}

interface DirtHistoryData {
  resolution: number;
  data: Float32Array;             // Dirt detection frequency per cell
  lastUpdated: Date;
}
```

### 3.4 Cleaning Task Data Formats

```typescript
// Cleaning Task Data Model
interface CleaningTask {
  id: string;
  robotId: string;

  type: CleaningTaskType;
  status: TaskStatus;
  priority: TaskPriority;

  target: CleaningTarget;
  settings: CleaningSettings;
  constraints: TaskConstraints;

  schedule: TaskSchedule | null;

  execution: TaskExecution;
  result: TaskResult | null;
}

type CleaningTaskType =
  | 'FULL_HOME'
  | 'SELECTED_ROOMS'
  | 'SELECTED_ZONES'
  | 'SPOT_CLEAN'
  | 'EDGE_CLEAN'
  | 'QUICK_CLEAN'
  | 'DEEP_CLEAN'
  | 'MAINTENANCE_CLEAN';

type TaskStatus =
  | 'PENDING'
  | 'SCHEDULED'
  | 'QUEUED'
  | 'STARTING'
  | 'IN_PROGRESS'
  | 'PAUSED'
  | 'RESUMING'
  | 'RETURNING'
  | 'COMPLETED'
  | 'CANCELLED'
  | 'FAILED';

type TaskPriority = 'LOW' | 'NORMAL' | 'HIGH' | 'URGENT';

interface CleaningTarget {
  type: TargetType;
  mapId: string;

  // For full home
  entireMap?: boolean;

  // For selected rooms
  rooms?: string[];

  // For selected zones
  zones?: string[];

  // For spot clean
  spotCenter?: Position2D;
  spotRadius?: number;            // meters
}

type TargetType = 'FULL_MAP' | 'ROOMS' | 'ZONES' | 'SPOT' | 'CUSTOM_AREA';

interface CleaningSettings {
  vacuum: {
    enabled: boolean;
    power: SuctionLevel;
    adaptivePower: boolean;
  };

  mop: {
    enabled: boolean;
    wetness: WetnessLevel;
    adaptiveWetness: boolean;
  };

  pattern: CleaningPattern;
  passes: number;
  edgeFirst: boolean;

  adaptiveSettings: {
    carpetBoost: boolean;
    dirtDetectMode: boolean;
    quietMode: boolean;
    petMode: boolean;
  };
}

interface TaskConstraints {
  timeConstraints: {
    maxDuration: number | null;   // minutes
    mustCompleteBy: Date | null;
    noCleanBefore: TimeOfDay | null;
    noCleanAfter: TimeOfDay | null;
  };

  batteryConstraints: {
    minimumBattery: number;       // percent
    returnThreshold: number;      // percent
    allowResume: boolean;
  };

  areaConstraints: {
    avoidRooms: string[];
    avoidZones: string[];
    priorityOrder: string[];
  };
}

interface TaskSchedule {
  type: ScheduleType;

  // One-time
  scheduledTime?: Date;

  // Recurring
  recurrence?: {
    frequency: RecurrenceFrequency;
    daysOfWeek: DayOfWeek[];
    time: TimeOfDay;
    startDate: Date;
    endDate: Date | null;
  };

  active: boolean;
  nextExecution: Date | null;
}

type ScheduleType = 'IMMEDIATE' | 'ONE_TIME' | 'RECURRING';
type RecurrenceFrequency = 'DAILY' | 'WEEKLY' | 'BIWEEKLY' | 'MONTHLY';

interface TaskExecution {
  startedAt: Date | null;
  completedAt: Date | null;

  progress: TaskProgress;
  path: CleaningPath;
  events: TaskEvent[];
}

interface TaskProgress {
  percentage: number;             // 0-100
  areaCompleted: number;          // square meters
  areaRemaining: number;          // square meters

  roomProgress: Map<string, RoomProgress>;

  estimatedTimeRemaining: number; // minutes
  elapsedTime: number;            // minutes
}

interface RoomProgress {
  roomId: string;
  status: 'PENDING' | 'IN_PROGRESS' | 'COMPLETED' | 'SKIPPED';
  percentage: number;
  startedAt: Date | null;
  completedAt: Date | null;
}

interface CleaningPath {
  points: PathPoint[];
  totalDistance: number;          // meters
  totalTime: number;              // seconds
}

interface PathPoint {
  position: Position2D;
  timestamp: Date;
  activity: PathActivity;
}

type PathActivity =
  | 'CLEANING'
  | 'TRAVELING'
  | 'AVOIDING_OBSTACLE'
  | 'STUCK'
  | 'PAUSED';

interface TaskEvent {
  timestamp: Date;
  type: TaskEventType;
  details: Record<string, any>;
}

type TaskEventType =
  | 'STARTED'
  | 'ROOM_STARTED'
  | 'ROOM_COMPLETED'
  | 'OBSTACLE_DETECTED'
  | 'STUCK_DETECTED'
  | 'STUCK_RESOLVED'
  | 'LOW_BATTERY'
  | 'RETURNING_TO_CHARGE'
  | 'CHARGING_STARTED'
  | 'CHARGING_COMPLETED'
  | 'RESUMING'
  | 'PAUSED'
  | 'CANCELLED'
  | 'ERROR'
  | 'COMPLETED';

interface TaskResult {
  status: TaskCompletionStatus;

  summary: CleaningSummary;
  coverage: CoverageResult;
  performance: PerformanceMetrics;

  issues: CleaningIssue[];
}

type TaskCompletionStatus =
  | 'COMPLETED_SUCCESS'
  | 'COMPLETED_PARTIAL'
  | 'CANCELLED_USER'
  | 'CANCELLED_ERROR'
  | 'FAILED';

interface CleaningSummary {
  totalArea: number;              // square meters
  cleanedArea: number;
  cleaningTime: number;           // minutes
  travelTime: number;
  chargingTime: number;

  roomsCleaned: number;
  roomsTotal: number;

  batteryUsed: number;            // percent
  waterUsed: number;              // ml

  dirtCollected: DirtCollectionStats;
}

interface DirtCollectionStats {
  totalDirtLevel: number;
  highDirtAreas: number;
  dustbinFillLevel: number;
}

interface CoverageResult {
  overallCoverage: number;        // percent
  coverageByRoom: Map<string, number>;
  missedAreas: Position2D[];
  obstaclesCoverage: number;
}

interface PerformanceMetrics {
  efficiency: number;             // square meters per minute
  batteryEfficiency: number;      // square meters per percent
  pathEfficiency: number;         // actual/optimal path ratio
  suctionEffectiveness: number;   // based on dirt detection
}

interface CleaningIssue {
  type: CleaningIssueType;
  severity: IssueSeverity;
  location: Position2D | null;
  timestamp: Date;
  description: string;
  resolved: boolean;
}

type CleaningIssueType =
  | 'STUCK'
  | 'CLIFF_DETECTED'
  | 'BUMPER_STUCK'
  | 'WHEEL_STUCK'
  | 'BRUSH_TANGLED'
  | 'DUSTBIN_FULL'
  | 'WATER_EMPTY'
  | 'MOP_DIRTY'
  | 'LOW_BATTERY'
  | 'NAVIGATION_ERROR'
  | 'SENSOR_ERROR';

type IssueSeverity = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
```

### 3.5 Telemetry Data Formats

```typescript
// High-Frequency Telemetry Data
interface RobotTelemetry {
  robotId: string;
  sessionId: string;

  samples: TelemetrySample[];
}

interface TelemetrySample {
  timestamp: number;              // Unix milliseconds

  pose: {
    x: number;
    y: number;
    theta: number;
    confidence: number;
  };

  velocity: {
    linear: number;               // m/s
    angular: number;              // rad/s
  };

  motors: {
    leftWheel: MotorTelemetry;
    rightWheel: MotorTelemetry;
    mainBrush: MotorTelemetry;
    sideBrush: MotorTelemetry;
    vacuum: MotorTelemetry;
  };

  power: {
    batteryVoltage: number;
    batteryCurrent: number;
    batteryLevel: number;
    isCharging: boolean;
  };

  sensors: {
    cliff: number[];              // Distance readings
    bumper: boolean[];            // Contact states
    wall: number[];               // Wall sensor readings
    dirt: number;                 // Dirt level
  };

  environment: {
    temperature: number;
    humidity: number;
    lightLevel: number;
  };
}

interface MotorTelemetry {
  rpm: number;
  current: number;
  temperature: number;
  pwm: number;
}

// Telemetry Storage Service
class TelemetryStorageService {
  private timeSeriesDb: TimeSeriesDatabase;

  async storeTelemetry(
    telemetry: RobotTelemetry
  ): Promise<void> {
    const points = telemetry.samples.map(sample => ({
      measurement: 'robot_telemetry',
      tags: {
        robot_id: telemetry.robotId,
        session_id: telemetry.sessionId
      },
      fields: this.flattenSample(sample),
      timestamp: sample.timestamp
    }));

    await this.timeSeriesDb.writePoints(points);
  }

  async queryTelemetry(
    robotId: string,
    timeRange: TimeRange,
    fields: string[],
    aggregation?: AggregationOptions
  ): Promise<TelemetryQueryResult> {
    const query = this.buildQuery(robotId, timeRange, fields, aggregation);
    const result = await this.timeSeriesDb.query(query);

    return {
      robotId,
      timeRange,
      fields,
      samples: result.rows,
      statistics: this.calculateStatistics(result.rows, fields)
    };
  }

  private flattenSample(sample: TelemetrySample): Record<string, number | boolean> {
    return {
      'pose_x': sample.pose.x,
      'pose_y': sample.pose.y,
      'pose_theta': sample.pose.theta,
      'pose_confidence': sample.pose.confidence,
      'velocity_linear': sample.velocity.linear,
      'velocity_angular': sample.velocity.angular,
      'motor_left_rpm': sample.motors.leftWheel.rpm,
      'motor_left_current': sample.motors.leftWheel.current,
      'motor_right_rpm': sample.motors.rightWheel.rpm,
      'motor_right_current': sample.motors.rightWheel.current,
      'motor_brush_rpm': sample.motors.mainBrush.rpm,
      'motor_vacuum_rpm': sample.motors.vacuum.rpm,
      'battery_voltage': sample.power.batteryVoltage,
      'battery_current': sample.power.batteryCurrent,
      'battery_level': sample.power.batteryLevel,
      'is_charging': sample.power.isCharging,
      'dirt_level': sample.sensors.dirt,
      'temperature': sample.environment.temperature
    };
  }
}
```

### 3.6 Data Serialization and Exchange

```typescript
// Data Serialization Formats
interface DataSerializationFormats {
  json: {
    description: 'Human-readable format for APIs';
    compression: 'gzip optional';
    useCase: 'API responses, configuration';
  };

  protobuf: {
    description: 'Efficient binary format';
    compression: 'Built-in efficiency';
    useCase: 'Real-time telemetry, large datasets';
  };

  msgpack: {
    description: 'JSON-like binary format';
    compression: 'Compact representation';
    useCase: 'WebSocket communication';
  };
}

// Protocol Buffer Definitions
const robotStateProto = `
syntax = "proto3";

package wia.cleaning_robot;

message RobotState {
  string robot_id = 1;
  int64 timestamp = 2;

  OperatingState operating_state = 3;
  RobotPose pose = 4;
  PowerState power = 5;
  CleaningSystemState cleaning_system = 6;
}

message OperatingState {
  RobotOperatingStateEnum state = 1;
  string mode = 2;
  string sub_state = 3;
}

enum RobotOperatingStateEnum {
  IDLE = 0;
  CLEANING = 1;
  RETURNING_TO_DOCK = 2;
  CHARGING = 3;
  PAUSED = 4;
  ERROR = 5;
}

message RobotPose {
  double x = 1;
  double y = 2;
  double orientation = 3;
  string map_id = 4;
  int32 floor = 5;
  float confidence = 6;
}

message PowerState {
  int32 battery_level = 1;
  float battery_voltage = 2;
  float battery_current = 3;
  bool is_charging = 4;
  int32 estimated_runtime = 5;
}

message CleaningSystemState {
  VacuumState vacuum = 1;
  MopState mop = 2;
  int32 dustbin_level = 3;
  int32 water_tank_level = 4;
}

message VacuumState {
  bool active = 1;
  string suction_level = 2;
  int32 actual_suction_pa = 3;
}

message MopState {
  bool active = 1;
  string wetness_level = 2;
  bool mop_attached = 3;
}
`;

// Data Transformer Service
class DataTransformerService {
  // JSON serialization
  toJSON<T>(data: T): string {
    return JSON.stringify(data, this.jsonReplacer, 2);
  }

  fromJSON<T>(json: string, validator?: (data: any) => data is T): T {
    const parsed = JSON.parse(json, this.jsonReviver);
    if (validator && !validator(parsed)) {
      throw new Error('Invalid data format');
    }
    return parsed;
  }

  // Protocol buffer serialization
  toProtobuf<T>(data: T, messageType: string): Uint8Array {
    const Message = this.getMessageType(messageType);
    const message = Message.create(this.toProtoFormat(data));
    return Message.encode(message).finish();
  }

  fromProtobuf<T>(buffer: Uint8Array, messageType: string): T {
    const Message = this.getMessageType(messageType);
    const decoded = Message.decode(buffer);
    return this.fromProtoFormat(decoded) as T;
  }

  // MessagePack serialization
  toMsgPack<T>(data: T): Uint8Array {
    return msgpack.encode(data);
  }

  fromMsgPack<T>(buffer: Uint8Array): T {
    return msgpack.decode(buffer) as T;
  }

  private jsonReplacer(key: string, value: any): any {
    if (value instanceof Date) {
      return { __type: 'Date', value: value.toISOString() };
    }
    if (value instanceof Map) {
      return { __type: 'Map', value: Array.from(value.entries()) };
    }
    return value;
  }

  private jsonReviver(key: string, value: any): any {
    if (value && typeof value === 'object') {
      if (value.__type === 'Date') {
        return new Date(value.value);
      }
      if (value.__type === 'Map') {
        return new Map(value.value);
      }
    }
    return value;
  }
}
```

---

**WIA-CLEANING-ROBOT Data Formats**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
