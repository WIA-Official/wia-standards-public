# 3장: 청소 로봇 데이터 형식

## 로봇 상태, 매핑 및 운영 데이터 모델

### 3.1 핵심 데이터 모델 아키텍처

WIA-CLEANING-ROBOT 표준은 로봇 상태, 환경 지도, 청소 작업 및 운영 텔레메트리를 나타내기 위한 종합 데이터 형식을 정의합니다. 이러한 형식은 로봇, 클라우드 플랫폼 및 타사 애플리케이션 간의 상호운용성을 보장합니다.

```typescript
// 핵심 로봇 데이터 모델
interface CleaningRobotDataModel {
  version: '1.0.0';

  dataCategories: {
    robotState: {
      description: '현재 로봇 상태 및 텔레메트리';
      updateFrequency: '실시간 (100ms-1s)';
      storage: '주기적 영속화와 함께 인메모리';
    };
    environmentMap: {
      description: '청소 영역의 공간 표현';
      updateFrequency: '청소 세션별 또는 온디맨드';
      storage: '영속 로컬 + 클라우드 동기화';
    };
    cleaningTask: {
      description: '작업 정의 및 진행 상황';
      updateFrequency: '이벤트 기반';
      storage: '히스토리와 함께 영속';
    };
    telemetry: {
      description: '운영 메트릭 및 센서 데이터';
      updateFrequency: '고빈도 (10-100Hz)';
      storage: '시계열 데이터베이스';
    };
    maintenance: {
      description: '구성 요소 상태 및 서비스 기록';
      updateFrequency: '세션별 + 주기적 점검';
      storage: '전체 히스토리와 함께 영속';
    };
  };
}

// 로봇 ID 및 구성
interface RobotIdentity {
  id: string;                          // UUID v4
  serialNumber: string;                 // 제조업체 시리얼
  macAddress: string;                   // 기본 MAC 주소
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
    weight: number;              // 그램
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

interface RobotCapabilities {
  // 청소 기능
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

  // 내비게이션 기능
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

  // 도킹 기능
  docking: {
    autoReturn: boolean;
    autoResume: boolean;
    autoEmptyDust: boolean;
    autoWashMop: boolean;
    autoRefillWater: boolean;
    autoDryMop: boolean;
  };

  // 연결성
  connectivity: {
    wifi: WifiCapability;
    bluetooth: BluetoothCapability;
    voiceAssistants: VoiceAssistant[];
    smartHomeProtocols: SmartHomeProtocol[];
  };

  // 센서
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

### 3.2 로봇 상태 표현

```typescript
// 종합 로봇 상태 모델
interface RobotState {
  robotId: string;
  timestamp: Date;

  // 작동 상태
  operatingState: OperatingState;

  // 위치 및 자세
  pose: RobotPose;

  // 전원 상태
  power: PowerState;

  // 청소 시스템 상태
  cleaningSystem: CleaningSystemState;

  // 센서 상태
  sensors: SensorState;

  // 연결 상태
  connectivity: ConnectivityState;

  // 현재 작업
  currentTask: TaskState | null;

  // 오류 및 경고
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
  | 'IDLE'                    // 준비, 청소 안함
  | 'CLEANING'                // 적극적으로 청소 중
  | 'SPOT_CLEANING'           // 스팟 청소 모드
  | 'EDGE_CLEANING'           // 가장자리 청소 모드
  | 'RETURNING_TO_DOCK'       // 도크로 복귀 중
  | 'DOCKING'                 // 도킹 진행 중
  | 'CHARGING'                // 도크에서 충전 중
  | 'CHARGED'                 // 도크에서 완충
  | 'PAUSED'                  // 청소 일시 중지
  | 'STUCK'                   // 로봇이 막힘
  | 'ERROR'                   // 오류 상태
  | 'MAINTENANCE_REQUIRED'    // 유지 관리 필요
  | 'EMPTYING_DUSTBIN'        // 자동 먼지 비움
  | 'WASHING_MOP'             // 자동 걸레 세척
  | 'DRYING_MOP'              // 자동 걸레 건조
  | 'REFILLING_WATER'         // 자동 물 충전
  | 'MAPPING'                 // 새 지도 생성
  | 'LOCATING'                // 위치 찾기 시도
  | 'UPDATING'                // 펌웨어 업데이트
  | 'MANUAL_CONTROL';         // 원격 제어 모드

interface RobotPose {
  position: Position2D;
  orientation: number;        // 라디안, 0 = 양의 X축
  mapId: string;
  floor: number;

  confidence: number;         // 0-1
  lastUpdated: Date;
  source: PoseSource;
}

interface Position2D {
  x: number;                  // 원점에서 미터
  y: number;                  // 원점에서 미터
}

type PoseSource = 'SLAM' | 'ODOMETRY' | 'DOCK_LOCALIZATION' | 'MANUAL_PLACEMENT';

interface PowerState {
  batteryLevel: number;           // 0-100 퍼센트
  batteryVoltage: number;         // 볼트
  batteryCurrent: number;         // 암페어
  batteryTemperature: number;     // 섭씨

  isCharging: boolean;
  chargeRate: number;             // 와트
  estimatedRuntime: number;       // 분
  estimatedChargeTime: number;    // 분

  batteryHealth: BatteryHealth;
  chargeCycles: number;
}

interface CleaningSystemState {
  vacuum: {
    active: boolean;
    suctionLevel: SuctionLevel;
    actualSuction: number;        // Pa
    motorRPM: number;
    motorTemperature: number;     // 섭씨
  };

  mainBrush: {
    active: boolean;
    rpm: number;
    motorCurrent: number;         // 암페어
    estimatedLife: number;        // 남은 퍼센트
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
    waterFlowRate: number;        // ml/분
    vibrationFrequency: number;   // Hz (진동 걸레인 경우)
    rotationSpeed: number;        // RPM (회전 걸레인 경우)
    mopAttached: boolean;
    mopClean: boolean;
  };

  dustbin: {
    level: number;                // 0-100 퍼센트
    full: boolean;
    installed: boolean;
  };

  waterTank: {
    level: number;                // 0-100 퍼센트
    empty: boolean;
    installed: boolean;
  };
}

type MopMode = 'OFF' | 'STANDARD' | 'DEEP' | 'GENTLE' | 'QUICK_DRY';
```

### 3.3 지도 데이터 형식

```typescript
// 종합 지도 데이터 모델
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
    rotation: number;             // 라디안
    scale: number;                // 단위당 미터
  };

  bounds: {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
  };

  statistics: {
    totalArea: number;            // 제곱미터
    cleanableArea: number;
    roomCount: number;
    boundaryCount: number;
  };
}

type MapSource = 'SLAM_GENERATED' | 'USER_MODIFIED' | 'IMPORTED' | 'CLOUD_SYNCED';

interface OccupancyGrid {
  width: number;                  // 셀
  height: number;                 // 셀
  resolution: number;             // 셀당 미터

  // 셀 유형: 0=알 수 없음, 1=자유, 2=점유, 3=가상_경계
  data: Uint8Array;

  // 추가 레이어
  layers: {
    floor: Uint8Array;           // 셀별 바닥 유형
    cleaned: Uint8Array;         // 셀별 청소 상태
    probability: Float32Array;    // 점유 확률
  };
}

interface Room {
  id: string;
  name: string;
  type: RoomType;

  geometry: {
    boundary: Polygon;
    area: number;                 // 제곱미터
    centroid: Position2D;
  };

  cleaningSettings: RoomCleaningSettings;

  statistics: {
    totalCleanings: number;
    lastCleaned: Date | null;
    averageCleaningTime: number;  // 분
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
  holes: Position2D[][];          // 내부 구멍/컷아웃
}

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
```

### 3.4 청소 작업 데이터 형식

```typescript
// 청소 작업 데이터 모델
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

interface CleaningTarget {
  type: TargetType;
  mapId: string;

  // 전체 가정용
  entireMap?: boolean;

  // 선택한 방용
  rooms?: string[];

  // 선택한 구역용
  zones?: string[];

  // 스팟 청소용
  spotCenter?: Position2D;
  spotRadius?: number;            // 미터
}

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

interface TaskProgress {
  percentage: number;             // 0-100
  areaCompleted: number;          // 제곱미터
  areaRemaining: number;          // 제곱미터

  roomProgress: Map<string, RoomProgress>;

  estimatedTimeRemaining: number; // 분
  elapsedTime: number;            // 분
}

interface TaskResult {
  status: TaskCompletionStatus;

  summary: CleaningSummary;
  coverage: CoverageResult;
  performance: PerformanceMetrics;

  issues: CleaningIssue[];
}

interface CleaningSummary {
  totalArea: number;              // 제곱미터
  cleanedArea: number;
  cleaningTime: number;           // 분
  travelTime: number;
  chargingTime: number;

  roomsCleaned: number;
  roomsTotal: number;

  batteryUsed: number;            // 퍼센트
  waterUsed: number;              // ml

  dirtCollected: DirtCollectionStats;
}
```

### 3.5 텔레메트리 데이터 형식

```typescript
// 고빈도 텔레메트리 데이터
interface RobotTelemetry {
  robotId: string;
  sessionId: string;

  samples: TelemetrySample[];
}

interface TelemetrySample {
  timestamp: number;              // Unix 밀리초

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
    cliff: number[];              // 거리 측정값
    bumper: boolean[];            // 접촉 상태
    wall: number[];               // 벽 센서 측정값
    dirt: number;                 // 먼지 수준
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

// 텔레메트리 저장 서비스
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
}
```

### 3.6 데이터 직렬화 및 교환

```typescript
// 데이터 직렬화 형식
interface DataSerializationFormats {
  json: {
    description: 'API용 사람이 읽을 수 있는 형식';
    compression: 'gzip 선택적';
    useCase: 'API 응답, 구성';
  };

  protobuf: {
    description: '효율적인 바이너리 형식';
    compression: '내장 효율성';
    useCase: '실시간 텔레메트리, 대용량 데이터셋';
  };

  msgpack: {
    description: 'JSON과 유사한 바이너리 형식';
    compression: '컴팩트 표현';
    useCase: 'WebSocket 통신';
  };
}

// 데이터 변환기 서비스
class DataTransformerService {
  // JSON 직렬화
  toJSON<T>(data: T): string {
    return JSON.stringify(data, this.jsonReplacer, 2);
  }

  fromJSON<T>(json: string, validator?: (data: any) => data is T): T {
    const parsed = JSON.parse(json, this.jsonReviver);
    if (validator && !validator(parsed)) {
      throw new Error('잘못된 데이터 형식');
    }
    return parsed;
  }

  // Protocol buffer 직렬화
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

  // MessagePack 직렬화
  toMsgPack<T>(data: T): Uint8Array {
    return msgpack.encode(data);
  }

  fromMsgPack<T>(buffer: Uint8Array): T {
    return msgpack.decode(buffer) as T;
  }
}
```

---

**WIA-CLEANING-ROBOT 데이터 형식**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
