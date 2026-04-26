# 제9장: 커넥티드카 미래 트렌드

## 자율주행차, MaaS 및 신기술

본 장에서는 향후 10년간 자동차 산업을 형성할 신기술 트렌드와 커넥티드 차량 기술의 미래 진화를 탐구합니다.

---

## 자율주행차 데이터 아키텍처

### 센서 퓨전 및 데이터 처리

```typescript
// WIA 커넥티드카 미래 기술
// 자율주행차 데이터 아키텍처

/**
 * 자율주행차 센서 퓨전 프레임워크
 * SAE Level 4+ 차량을 위한 다중 센서 인지 시스템
 */
interface AutonomousVehicleArchitecture {
  perceptionSystem: PerceptionSystem;         // 인지 시스템
  planningSystem: PlanningSystem;             // 계획 시스템
  controlSystem: ControlSystem;               // 제어 시스템
  v2xIntegration: V2XAutonomousIntegration;  // V2X 통합
  dataRecording: DataRecordingSystem;         // 데이터 기록
  cloudIntelligence: CloudAIIntegration;     // 클라우드 AI
}

interface PerceptionSystem {
  sensors: SensorConfiguration[];             // 센서 구성
  fusionEngine: FusionEngine;                 // 퓨전 엔진
  objectDetection: ObjectDetectionPipeline;  // 객체 탐지
  localization: LocalizationSystem;          // 위치 추정
  mapping: MappingSystem;                     // 맵핑
}

interface SensorConfiguration {
  type: SensorType;                 // 센서 유형
  model: string;                    // 모델명
  position: SensorPosition;         // 위치
  specifications: SensorSpecifications;  // 사양
  dataInterface: DataInterface;     // 데이터 인터페이스
}

type SensorType =
  | "CAMERA"          // 카메라
  | "LIDAR"           // 라이다
  | "RADAR"           // 레이더
  | "ULTRASONIC"      // 초음파
  | "IMU"             // 관성 측정 장치
  | "GNSS"            // 위성 항법
  | "WHEEL_ODOMETRY"  // 휠 오도메트리
  | "THERMAL"         // 열화상
  | "EVENT_CAMERA";   // 이벤트 카메라

interface SensorPosition {
  location: { x: number; y: number; z: number };  // 차량 좌표계 위치
  orientation: { roll: number; pitch: number; yaw: number };  // 방향
  mountingPoint: string;  // 장착 위치
}

interface SensorSpecifications {
  // 카메라 사양
  resolution?: { width: number; height: number };  // 해상도
  fov?: { horizontal: number; vertical: number };  // 화각
  frameRate?: number;         // 프레임 레이트
  dynamicRange?: number;      // 다이나믹 레인지

  // 라이다 사양
  channels?: number;          // 채널 수
  pointsPerSecond?: number;   // 초당 포인트
  range?: number;             // 탐지 거리
  angularResolution?: number; // 각 해상도

  // 레이더 사양
  rangeResolution?: number;   // 거리 해상도
  velocityResolution?: number;  // 속도 해상도
  azimuthFov?: number;        // 방위각 화각
}

interface DataInterface {
  protocol: "ETHERNET" | "CAN_FD" | "FLEXRAY" | "GMSL" | "FPD_LINK";
  bandwidth: number;  // Mbps
  latency: number;    // ms
}

/**
 * 센서 퓨전 엔진
 * 인지를 위한 실시간 다중 센서 데이터 퓨전
 */
class SensorFusionEngine {
  private sensorRegistry: Map<string, SensorStream> = new Map();
  private fusionPipeline: FusionPipeline;
  private kalmanFilter: ExtendedKalmanFilter;
  private graphOptimizer: FactorGraphOptimizer;

  constructor(config: FusionConfig) {
    this.fusionPipeline = new FusionPipeline(config.pipeline);
    this.kalmanFilter = new ExtendedKalmanFilter(config.ekf);
    this.graphOptimizer = new FactorGraphOptimizer(config.optimizer);
  }

  /**
   * 동기화된 센서 데이터 처리
   */
  async processSensorData(
    timestamp: bigint,
    sensorData: SensorDataPacket[]
  ): Promise<FusedPerception> {
    // 시간 동기화
    const synchronized = this.synchronizeSensors(timestamp, sensorData);

    // 센서별 처리
    const detections: Detection[] = [];

    for (const data of synchronized) {
      switch (data.sensorType) {
        case "CAMERA":
          detections.push(...await this.processCameraData(data));
          break;
        case "LIDAR":
          detections.push(...await this.processLidarData(data));
          break;
        case "RADAR":
          detections.push(...await this.processRadarData(data));
          break;
      }
    }

    // 트랙 수준 퓨전
    const tracks = await this.fuseDetections(detections);

    // 상태 추정
    const estimatedState = this.kalmanFilter.update(tracks);

    // 위치 추정을 위한 그래프 최적화
    const localizedPose = await this.graphOptimizer.optimize({
      odometry: this.getOdometry(synchronized),
      landmarks: tracks.filter(t => t.type === "LANDMARK"),
      gnss: this.getGNSSData(synchronized),
      mapConstraints: await this.getMapConstraints()
    });

    return {
      timestamp,
      pose: localizedPose,
      objects: tracks.map(t => this.trackToObject(t)),
      freespace: this.computeFreespace(synchronized),
      confidence: this.computeOverallConfidence(tracks)
    };
  }

  /**
   * 카메라 인지 파이프라인
   */
  private async processCameraData(
    data: SensorDataPacket
  ): Promise<Detection[]> {
    const detections: Detection[] = [];

    // 2D 객체 탐지 (YOLO/EfficientDet)
    const objects2D = await this.run2DDetection(data.imageData!);

    // 깊이 추정 (스테레오 또는 모노 깊이)
    const depthMap = await this.estimateDepth(data);

    // 3D 투영
    for (const obj of objects2D) {
      const detection: Detection = {
        id: crypto.randomUUID(),
        sensorId: data.sensorId,
        timestamp: data.timestamp,
        type: obj.class,
        bbox2D: obj.bbox,
        bbox3D: this.project2Dto3D(obj.bbox, depthMap, data.calibration!),
        confidence: obj.confidence,
        attributes: this.extractAttributes(obj, data.imageData!)
      };
      detections.push(detection);
    }

    // 차선 탐지
    const lanes = await this.detectLanes(data.imageData!);
    for (const lane of lanes) {
      detections.push({
        id: crypto.randomUUID(),
        sensorId: data.sensorId,
        timestamp: data.timestamp,
        type: "LANE_MARKING",
        polyline: lane.points,
        laneType: lane.type,
        confidence: lane.confidence
      } as Detection);
    }

    // 교통 표지판/신호등 탐지
    const trafficElements = await this.detectTrafficElements(data.imageData!);
    detections.push(...trafficElements);

    return detections;
  }

  /**
   * 라이다 인지 파이프라인
   */
  private async processLidarData(
    data: SensorDataPacket
  ): Promise<Detection[]> {
    const detections: Detection[] = [];

    // 지면 분할
    const { groundPoints, nonGroundPoints } = this.segmentGround(
      data.pointCloud!
    );

    // 클러스터링 (DBSCAN 또는 유클리디안)
    const clusters = this.clusterPoints(nonGroundPoints);

    // 3D 객체 탐지 (PointPillars/CenterPoint)
    const objects3D = await this.run3DDetection(data.pointCloud!);

    for (const obj of objects3D) {
      detections.push({
        id: crypto.randomUUID(),
        sensorId: data.sensorId,
        timestamp: data.timestamp,
        type: obj.class,
        bbox3D: obj.bbox3D,
        velocity: obj.velocity,
        pointCount: obj.numPoints,
        confidence: obj.confidence
      });
    }

    // 도로 경계 추출
    const roadBoundaries = this.extractRoadBoundaries(groundPoints);
    detections.push(...roadBoundaries);

    return detections;
  }

  /**
   * 레이더 인지 파이프라인
   */
  private async processRadarData(
    data: SensorDataPacket
  ): Promise<Detection[]> {
    const detections: Detection[] = [];

    // 레이더 타겟 처리
    for (const target of data.radarTargets!) {
      detections.push({
        id: crypto.randomUUID(),
        sensorId: data.sensorId,
        timestamp: data.timestamp,
        type: this.classifyRadarTarget(target),
        position: {
          x: target.range * Math.cos(target.azimuth),
          y: target.range * Math.sin(target.azimuth),
          z: 0  // 2D 레이더
        },
        velocity: {
          vx: target.rangeRate * Math.cos(target.azimuth),
          vy: target.rangeRate * Math.sin(target.azimuth),
          vz: 0
        },
        rcs: target.rcs,  // 레이더 단면적
        confidence: target.probability
      });
    }

    return detections;
  }

  /**
   * 다중 센서 트랙 퓨전
   */
  private async fuseDetections(
    detections: Detection[]
  ): Promise<Track[]> {
    // 헝가리안 알고리즘 또는 GNN을 사용한 연관
    const associations = this.associateDetections(detections);

    // 기존 트랙 업데이트
    for (const [trackId, detectionIds] of associations) {
      const track = this.getTrack(trackId);
      const associatedDetections = detectionIds.map(id =>
        detections.find(d => d.id === id)!
      );
      this.updateTrack(track, associatedDetections);
    }

    // 연관되지 않은 탐지에 대해 새 트랙 생성
    const unassociated = detections.filter(d =>
      !Array.from(associations.values()).flat().includes(d.id)
    );

    for (const detection of unassociated) {
      this.createTrack(detection);
    }

    // 오래된 트랙 제거
    this.pruneOldTracks();

    return this.getAllTracks();
  }

  // 스텁 구현 (생략)
}

interface SensorDataPacket {
  sensorId: string;
  sensorType: SensorType;
  timestamp: bigint;
  imageData?: Buffer;
  pointCloud?: PointCloud;
  radarTargets?: RadarTarget[];
  calibration?: CalibrationData;
}

interface PointCloud {
  points: Float32Array;  // x, y, z, 반사강도
  numPoints: number;
}

interface RadarTarget {
  range: number;        // 거리
  azimuth: number;      // 방위각
  elevation?: number;   // 고도각
  rangeRate: number;    // 상대 속도
  rcs: number;          // 레이더 단면적
  probability: number;  // 신뢰도
}

interface Detection {
  id: string;
  sensorId: string;
  timestamp: bigint;
  type: string;
  bbox2D?: any;
  bbox3D?: any;
  position?: { x: number; y: number; z: number };
  velocity?: { vx: number; vy: number; vz: number };
  polyline?: any;
  laneType?: string;
  pointCount?: number;
  rcs?: number;
  confidence: number;
  attributes?: any;
}

interface Track {
  id: string;
  type: string;
  state: TrackState;
  history: Detection[];
  age: number;
  confidence: number;
}

interface TrackState {
  position: { x: number; y: number; z: number };
  velocity: { vx: number; vy: number; vz: number };
  acceleration?: { ax: number; ay: number; az: number };
  dimensions: { length: number; width: number; height: number };
  heading: number;  // 진행 방향
}

interface FusedPerception {
  timestamp: bigint;
  pose: VehiclePose;
  objects: PerceivedObject[];
  freespace: FreespacePolygon;
  confidence: number;
}

interface VehiclePose {
  position: { x: number; y: number; z: number };
  orientation: { roll: number; pitch: number; yaw: number };
  velocity: { vx: number; vy: number; vz: number };
  timestamp: bigint;
}

interface PerceivedObject {
  id: string;
  type: ObjectType;
  position: { x: number; y: number; z: number };
  velocity: { vx: number; vy: number; vz: number };
  dimensions: { length: number; width: number; height: number };
  heading: number;
  confidence: number;
  predictedTrajectory?: TrajectoryPoint[];  // 예측 경로
}

type ObjectType =
  | "VEHICLE"           // 차량
  | "PEDESTRIAN"        // 보행자
  | "CYCLIST"           // 자전거
  | "MOTORCYCLE"        // 오토바이
  | "TRUCK"             // 트럭
  | "BUS"               // 버스
  | "TRAFFIC_SIGN"      // 교통 표지판
  | "TRAFFIC_LIGHT"     // 신호등
  | "CONSTRUCTION_ZONE" // 공사 구역
  | "ANIMAL"            // 동물
  | "UNKNOWN";          // 알 수 없음

interface TrajectoryPoint {
  position: { x: number; y: number };
  timestamp: number;
  probability: number;
}

interface FreespacePolygon {
  boundary: { x: number; y: number }[];
  confidence: number;
}
```

---

## MaaS (Mobility-as-a-Service) 통합

```typescript
/**
 * MaaS 플랫폼 통합
 * 포괄적 MaaS 생태계 연결
 */
interface MaaSPlatform {
  tripPlanning: MultimodalTripPlanner;     // 다중 교통수단 여정 계획
  booking: UnifiedBookingSystem;           // 통합 예약 시스템
  payment: IntegratedPayment;              // 통합 결제
  fleetManagement: FleetOrchestration;     // 차량대 관리
  userExperience: MaaSUserExperience;      // 사용자 경험
}

/**
 * 다중 교통수단 여정 계획 엔진
 */
class MultimodalTripPlanner {
  private routingEngines: Map<TransportMode, RoutingEngine>;
  private transitData: TransitDataProvider;
  private realTimeData: RealTimeDataAggregator;
  private preferences: UserPreferenceEngine;

  constructor(config: TripPlannerConfig) {
    this.routingEngines = this.initializeRoutingEngines(config);
    this.transitData = new TransitDataProvider(config.transitFeeds);
    this.realTimeData = new RealTimeDataAggregator(config.realTimeProviders);
    this.preferences = new UserPreferenceEngine();
  }

  /**
   * 최적의 다중 교통수단 여정 계획
   */
  async planTrip(request: TripRequest): Promise<TripOptions> {
    // 사용자 선호도 조회
    const userPrefs = await this.preferences.getUserPreferences(request.userId);

    // 각 교통수단 조합에 대한 후보 경로 생성
    const modeSequences = this.generateModeSequences(
      request.origin,
      request.destination,
      userPrefs.allowedModes
    );

    const tripOptions: TripOption[] = [];

    for (const sequence of modeSequences) {
      const legs = await this.planModeSequence(request, sequence);
      if (legs) {
        const option = this.assembleTripOption(legs, userPrefs);
        tripOptions.push(option);
      }
    }

    // 사용자 선호도 기반 옵션 순위 지정
    const rankedOptions = this.rankOptions(tripOptions, userPrefs);

    // 실시간 조정 적용
    const adjustedOptions = await this.applyRealTimeAdjustments(rankedOptions);

    return {
      request,
      options: adjustedOptions.slice(0, 5),
      generatedAt: new Date(),
      validUntil: new Date(Date.now() + 15 * 60 * 1000)  // 15분 유효
    };
  }

  /**
   * 가능한 교통수단 조합 생성
   */
  private generateModeSequences(
    origin: Location,
    destination: Location,
    allowedModes: TransportMode[]
  ): TransportMode[][] {
    const sequences: TransportMode[][] = [];

    // 직접 옵션
    for (const mode of allowedModes) {
      sequences.push([mode]);
    }

    // 첫 번째/마지막 마일 조합
    const firstMileModes: TransportMode[] = ["WALK", "BIKE_SHARE", "SCOOTER", "RIDE_HAIL"];
    const mainModes: TransportMode[] = ["TRANSIT", "TRAIN", "CAR_SHARE", "RIDE_HAIL"];
    const lastMileModes: TransportMode[] = ["WALK", "BIKE_SHARE", "SCOOTER", "RIDE_HAIL"];

    for (const first of firstMileModes) {
      for (const main of mainModes) {
        for (const last of lastMileModes) {
          if (allowedModes.includes(first) &&
              allowedModes.includes(main) &&
              allowedModes.includes(last)) {
            sequences.push([first, main, last]);
          }
        }
      }
    }

    // 허용된 교통수단만 필터링
    return sequences.filter(seq =>
      seq.every(mode => allowedModes.includes(mode))
    );
  }

  /**
   * 교통수단 시퀀스 경로 계획
   */
  private async planModeSequence(
    request: TripRequest,
    modes: TransportMode[]
  ): Promise<TripLeg[] | null> {
    const legs: TripLeg[] = [];
    let currentLocation = request.origin;

    for (let i = 0; i < modes.length; i++) {
      const mode = modes[i];
      const isLastLeg = i === modes.length - 1;
      const destination = isLastLeg
        ? request.destination
        : await this.findTransferPoint(currentLocation, request.destination, mode, modes[i + 1]);

      if (!destination) return null;

      const engine = this.routingEngines.get(mode);
      if (!engine) return null;

      const leg = await engine.route(currentLocation, destination, {
        departureTime: legs.length > 0
          ? legs[legs.length - 1].arrivalTime
          : request.departureTime,
        preferences: request.preferences
      });

      if (!leg) return null;

      legs.push(leg);
      currentLocation = destination;
    }

    return legs;
  }

  private assembleTripOption(legs: TripLeg[], prefs: UserPreferences): TripOption {
    const totalDuration = legs.reduce((sum, leg) => sum + leg.duration, 0);
    const totalDistance = legs.reduce((sum, leg) => sum + leg.distance, 0);
    const totalCost = legs.reduce((sum, leg) => sum + (leg.estimatedCost || 0), 0);
    const totalEmissions = legs.reduce((sum, leg) => sum + (leg.emissions || 0), 0);

    return {
      id: crypto.randomUUID(),
      legs,
      summary: {
        totalDuration,     // 총 소요시간
        totalDistance,     // 총 거리
        totalCost,         // 총 비용
        totalEmissions,    // 총 탄소 배출량
        departureTime: legs[0].departureTime,
        arrivalTime: legs[legs.length - 1].arrivalTime,
        modes: legs.map(l => l.mode)
      },
      score: this.calculateScore(legs, prefs)
    };
  }
}

type TransportMode =
  | "WALK"              // 도보
  | "BIKE"              // 자전거
  | "BIKE_SHARE"        // 공유 자전거
  | "SCOOTER"           // 전동 킥보드
  | "TRANSIT"           // 대중교통
  | "BUS"               // 버스
  | "TRAIN"             // 기차
  | "SUBWAY"            // 지하철
  | "TRAM"              // 트램
  | "FERRY"             // 페리
  | "RIDE_HAIL"         // 호출 택시
  | "TAXI"              // 택시
  | "CAR_SHARE"         // 카셰어링
  | "PRIVATE_CAR"       // 자가용
  | "AUTONOMOUS_SHUTTLE"  // 자율주행 셔틀
  | "AIR_TAXI";         // 에어 택시

interface TripRequest {
  userId: string;
  origin: Location;
  destination: Location;
  departureTime?: Date;
  arrivalTime?: Date;
  preferences: TripPreferences;
}

interface Location {
  latitude: number;
  longitude: number;
  address?: string;
  name?: string;
}

interface TripPreferences {
  optimizeFor: "TIME" | "COST" | "COMFORT" | "EMISSIONS" | "BALANCED";
  maxWalkingDistance: number;  // 최대 도보 거리
  maxTransfers: number;        // 최대 환승 횟수
  accessibility: AccessibilityRequirements;
  luggageSize?: "NONE" | "SMALL" | "MEDIUM" | "LARGE";
}

interface AccessibilityRequirements {
  wheelchairAccessible: boolean;  // 휠체어 접근성
  visualAssistance: boolean;      // 시각 보조
  audioAssistance: boolean;       // 청각 보조
}

interface TripOption {
  id: string;
  legs: TripLeg[];
  summary: TripSummary;
  score: number;
}

interface TripLeg {
  mode: TransportMode;
  from: Location;
  to: Location;
  departureTime: Date;
  arrivalTime: Date;
  duration: number;        // 소요시간 (초)
  distance: number;        // 거리 (미터)
  polyline?: string;       // 경로 폴리라인
  provider?: ServiceProvider;
  estimatedCost?: number;  // 예상 비용
  emissions?: number;      // 탄소 배출량 (g CO2)
  instructions?: Instruction[];
  vehicleInfo?: VehicleInfo;
  bookingRequired?: boolean;  // 예약 필요 여부
}

interface TripSummary {
  totalDuration: number;     // 총 소요시간
  totalDistance: number;     // 총 거리
  totalCost: number;         // 총 비용
  totalEmissions: number;    // 총 탄소 배출
  departureTime: Date;
  arrivalTime: Date;
  modes: TransportMode[];
}
```

---

## V2G (Vehicle-to-Grid) 통합

```typescript
/**
 * V2G 에너지 관리
 * 양방향 충전 및 그리드 서비스
 */
interface V2GSystem {
  energyManagement: EnergyManagementSystem;  // 에너지 관리
  gridServices: GridServicesInterface;       // 그리드 서비스
  smartCharging: SmartChargingOptimizer;     // 스마트 충전
  userPreferences: V2GUserPreferences;       // 사용자 설정
}

/**
 * 스마트 충전 및 V2G 최적화기
 */
class V2GOptimizer {
  private gridData: GridDataProvider;
  private vehicleFleet: FleetManager;
  private marketInterface: EnergyMarketInterface;
  private predictor: EnergyPredictor;

  constructor(config: V2GConfig) {
    this.gridData = new GridDataProvider(config.gridConnection);
    this.vehicleFleet = new FleetManager(config.fleet);
    this.marketInterface = new EnergyMarketInterface(config.market);
    this.predictor = new EnergyPredictor(config.prediction);
  }

  /**
   * 단일 차량 충전 스케줄 최적화
   */
  async optimizeChargingSchedule(
    request: ChargingRequest
  ): Promise<ChargingSchedule> {
    // 사용자 제약 조건
    const constraints = {
      requiredSoC: request.targetSoC,
      departureTime: request.departureTime,
      preferences: request.preferences
    };

    // 그리드 신호 조회
    const gridSignals = await this.gridData.getForecast(24);

    // 에너지 가격 조회
    const prices = await this.marketInterface.getPriceForecast(24);

    // 재생에너지 가용성 조회
    const renewables = await this.predictor.getRenewableForecast(24);

    // 선형 프로그래밍 또는 강화학습으로 최적화
    const schedule = this.runOptimization({
      constraints,
      gridSignals,
      prices,
      renewables,
      vehicleCapacity: request.batteryCapacity,
      currentSoC: request.currentSoC,
      maxChargePower: request.maxChargePower,
      maxDischargePower: request.maxDischargePower,
      v2gEnabled: request.v2gEnabled
    });

    return schedule;
  }

  /**
   * 차량대 수준 V2G 최적화
   */
  async optimizeFleetV2G(
    timeHorizon: number  // 시간
  ): Promise<FleetV2GSchedule> {
    // 사용 가능한 모든 차량 조회
    const vehicles = await this.vehicleFleet.getV2GCapableVehicles();

    // 집계된 그리드 요구사항 조회
    const gridNeeds = await this.gridData.getGridNeeds(timeHorizon);

    // 개별 차량 제약 조건 조회
    const vehicleConstraints = await Promise.all(
      vehicles.map(v => this.getVehicleConstraints(v))
    );

    // 가용 용량 집계
    const availableCapacity = this.calculateAvailableCapacity(
      vehicles,
      vehicleConstraints
    );

    // 차량대 배차 최적화
    const dispatch = this.optimizeFleetDispatch({
      vehicles,
      constraints: vehicleConstraints,
      gridNeeds,
      marketOpportunities: await this.marketInterface.getOpportunities(timeHorizon)
    });

    return {
      timeHorizon,
      vehicleSchedules: dispatch.schedules,
      aggregatedPower: dispatch.aggregatedPower,
      estimatedRevenue: dispatch.estimatedRevenue,
      gridServicesProvided: dispatch.gridServices
    };
  }

  /**
   * 최적화 알고리즘 실행
   */
  private runOptimization(params: OptimizationParams): ChargingSchedule {
    const intervals = this.generateTimeIntervals(params);
    const schedule: ChargingInterval[] = [];

    let currentSoC = params.currentSoC;

    for (const interval of intervals) {
      // 이 시간대의 최적 행동 결정
      const action = this.determineAction(
        interval,
        currentSoC,
        params
      );

      schedule.push({
        startTime: interval.start,
        endTime: interval.end,
        action: action.type,
        power: action.power,
        price: interval.price,
        gridCarbonIntensity: interval.carbonIntensity
      });

      // SoC 업데이트
      const energyDelta = action.power * (interval.duration / 60); // kWh
      currentSoC += (energyDelta / params.vehicleCapacity) * 100;
      currentSoC = Math.max(0, Math.min(100, currentSoC));
    }

    return {
      vehicleId: params.vehicleId || "",
      intervals: schedule,
      estimatedCost: this.calculateCost(schedule),
      estimatedRevenue: this.calculateRevenue(schedule),
      finalSoC: currentSoC,
      meetsConstraints: currentSoC >= params.constraints.requiredSoC
    };
  }

  private determineAction(
    interval: TimeInterval,
    currentSoC: number,
    params: OptimizationParams
  ): ChargingAction {
    // 간단한 규칙 기반 로직 (프로덕션에서는 LP/RL 사용)
    const { constraints, prices, gridSignals } = params;

    // SoC가 너무 낮으면 반드시 충전
    if (currentSoC < 20) {
      return { type: "CHARGE", power: params.maxChargePower };
    }

    // 방전 가능 조건:
    // 1. V2G 활성화
    // 2. SoC가 최소값 이상
    // 3. 그리드에 전력 필요 또는 가격이 높음
    if (params.v2gEnabled &&
        currentSoC > constraints.requiredSoC + 10 &&
        (gridSignals.peakDemand || interval.price > prices.peakThreshold)) {
      return { type: "DISCHARGE", power: -params.maxDischargePower };
    }

    // 가격이 낮고 완충 아니면 충전
    if (currentSoC < 95 && interval.price < prices.offPeakThreshold) {
      return { type: "CHARGE", power: params.maxChargePower };
    }

    // 그 외 대기
    return { type: "IDLE", power: 0 };
  }
}

interface ChargingRequest {
  vehicleId: string;
  currentSoC: number;           // 현재 충전 상태
  targetSoC: number;            // 목표 충전 상태
  departureTime: Date;          // 출발 시간
  batteryCapacity: number;      // 배터리 용량 (kWh)
  maxChargePower: number;       // 최대 충전 전력 (kW)
  maxDischargePower: number;    // 최대 방전 전력 (kW)
  v2gEnabled: boolean;          // V2G 활성화
  preferences: V2GUserPreferences;
}

interface V2GUserPreferences {
  minDepartureSoC: number;      // 출발 시 최소 SoC
  preferRenewable: boolean;     // 재생에너지 선호
  allowGridServices: boolean;   // 그리드 서비스 허용
  maxDischargeCycles: number;   // 최대 방전 사이클
}

interface ChargingSchedule {
  vehicleId: string;
  intervals: ChargingInterval[];
  estimatedCost: number;        // 예상 비용
  estimatedRevenue: number;     // 예상 수익
  finalSoC: number;             // 최종 SoC
  meetsConstraints: boolean;    // 제약 충족 여부
}

interface ChargingInterval {
  startTime: Date;
  endTime: Date;
  action: "CHARGE" | "DISCHARGE" | "IDLE";
  power: number;                // kW (양수 = 충전, 음수 = 방전)
  price: number;                // 전력 가격
  gridCarbonIntensity: number;  // 그리드 탄소 집약도
}

interface FleetV2GSchedule {
  timeHorizon: number;
  vehicleSchedules: ChargingSchedule[];
  aggregatedPower: number[];        // 집계된 전력
  estimatedRevenue: number;         // 예상 총 수익
  gridServicesProvided: string[];   // 제공된 그리드 서비스
}
```

---

## SDV (Software-Defined Vehicle) 아키텍처

```typescript
/**
 * SDV 아키텍처
 * OTA 업데이트 가능한 미래 차량 플랫폼
 */
interface SDVArchitecture {
  centralCompute: CentralComputePlatform;   // 중앙 컴퓨팅
  zonalArchitecture: ZonalControllers;      // 존 컨트롤러
  softwarePlatform: VehicleSoftwarePlatform;  // 소프트웨어 플랫폼
  appEcosystem: VehicleAppStore;            // 앱 생태계
  digitalTwin: DigitalTwinPlatform;         // 디지털 트윈
}

/**
 * 중앙 컴퓨팅 플랫폼
 * SDV를 위한 고성능 컴퓨팅 허브
 */
interface CentralComputePlatform {
  hardware: HardwareSpecification;
  operatingSystem: VehicleOS;
  hypervisor: Hypervisor;
  containers: ContainerOrchestration;
  aiAccelerator: AIAccelerator;
}

interface HardwareSpecification {
  cpu: {
    type: string;
    cores: number;               // 코어 수
    frequency: number;           // 주파수
    architecture: "ARM" | "x86";
  };
  gpu: {
    type: string;
    teraflops: number;          // 연산 성능
    memory: number;             // 메모리
  };
  npu: {
    type: string;
    topsInt8: number;           // INT8 연산 성능
  };
  memory: {
    type: string;
    capacity: number;           // 용량
    bandwidth: number;          // 대역폭
  };
  storage: {
    type: string;
    capacity: number;
    speed: number;
  };
  connectivity: {
    ethernet: string[];
    can: string[];
    flexray: boolean;
    pcie: number;
  };
}

/**
 * 차량 운영 체제
 */
interface VehicleOS {
  kernel: "LINUX" | "QNX" | "AUTOSAR_ADAPTIVE" | "PROPRIETARY";
  middleware: MiddlewareStack;
  services: SystemServices;
  apis: PlatformAPIs;
  security: OSSecurityFeatures;
}

interface MiddlewareStack {
  communication: {
    someIP: boolean;    // SOME/IP
    dds: boolean;       // DDS
    mqtt: boolean;      // MQTT
    grpc: boolean;      // gRPC
  };
  diagnostics: {
    uds: boolean;       // UDS
    doip: boolean;      // DoIP
    sota: boolean;      // SOTA
  };
  persistence: {
    database: string;
    keyValue: string;
    fileSystem: string;
  };
}

/**
 * 차량 앱 플랫폼
 */
class VehicleAppPlatform {
  private runtime: AppRuntime;
  private permissions: PermissionManager;
  private lifecycle: AppLifecycleManager;
  private store: AppStoreConnector;

  /**
   * 애플리케이션 설치
   */
  async installApp(
    appPackage: AppPackage
  ): Promise<InstallationResult> {
    // 앱 서명 검증
    const signatureValid = await this.verifyAppSignature(appPackage);
    if (!signatureValid) {
      return {
        success: false,
        error: "유효하지 않은 애플리케이션 서명"
      };
    }

    // 호환성 확인
    const compatibility = await this.checkCompatibility(appPackage);
    if (!compatibility.compatible) {
      return {
        success: false,
        error: `호환 불가: ${compatibility.reason}`
      };
    }

    // 필요 권한 확인
    const permissionApproval = await this.permissions.requestPermissions(
      appPackage.manifest.permissions
    );
    if (!permissionApproval.granted) {
      return {
        success: false,
        error: "필요 권한 미승인"
      };
    }

    // 런타임에 설치
    const installResult = await this.runtime.install(appPackage);

    if (installResult.success) {
      // 라이프사이클 관리자에 등록
      await this.lifecycle.register(appPackage.manifest.appId);

      // 스토어에 설치 보고
      await this.store.reportInstallation(appPackage.manifest.appId);
    }

    return installResult;
  }

  /**
   * 애플리케이션 실행
   */
  async launchApp(
    appId: string,
    context: LaunchContext
  ): Promise<AppInstance> {
    // 현재 차량 상태에서 앱 실행 가능 여부 확인
    const stateCheck = await this.lifecycle.checkVehicleState(appId);
    if (!stateCheck.allowed) {
      throw new Error(`현재 상태에서 앱 실행 불가: ${stateCheck.reason}`);
    }

    // 리소스 할당
    const resources = await this.runtime.allocateResources(appId);

    // 앱 컨테이너/프로세스 시작
    const instance = await this.runtime.start(appId, {
      resources,
      context,
      permissions: await this.permissions.getGrantedPermissions(appId)
    });

    // 실행 인스턴스 등록
    this.lifecycle.registerInstance(instance);

    return instance;
  }
}

interface AppManifest {
  appId: string;
  name: string;
  version: string;
  publisher: string;
  permissions: Permission[];
  capabilities: string[];
  vehicleRequirements: VehicleRequirements;
  uiType: "HMI" | "HEADLESS" | "BOTH";
}

interface Permission {
  type: PermissionType;
  access: "READ" | "WRITE" | "EXECUTE";
  resource: string;
}

type PermissionType =
  | "VEHICLE_DATA"    // 차량 데이터
  | "LOCATION"        // 위치
  | "CAMERA"          // 카메라
  | "MICROPHONE"      // 마이크
  | "NETWORK"         // 네트워크
  | "STORAGE"         // 저장소
  | "ACTUATOR"        // 액추에이터
  | "NOTIFICATION"    // 알림
  | "BACKGROUND";     // 백그라운드

interface AppInstance {
  instanceId: string;
  appId: string;
  state: "STARTING" | "RUNNING" | "PAUSED" | "STOPPED";
  resources: AllocatedResources;
}

interface AllocatedResources {
  cpuShare: number;      // CPU 할당
  memoryLimit: number;   // 메모리 제한
  gpuAccess: boolean;    // GPU 접근
}
```

---

## 요약: 미래 기술 타임라인

| 기술 | 현재 상태 (2025) | 단기 (2027) | 장기 (2030+) |
|------|-----------------|-------------|--------------|
| **자율주행** | L2+ ADAS 보편화 | L3 고속도로 배치 | L4 도심 로보택시 |
| **연결성** | 5G 롤아웃 | 5G-V2X | 6G 통합 |
| **V2G** | 초기 파일럿 | 상업적 배치 | 그리드 통합 |
| **SDV** | OEM 개발 중 | 첫 양산 | 산업 표준 |
| **MaaS** | 도시 수준 시범 | 지역 통합 | 글로벌 원활 연결 |
| **에어 모빌리티** | 테스트 단계 | 제한적 상용화 | 도심 통합 |

---

## 결론

커넥티드카 생태계는 다음과 같은 미래로 빠르게 진화하고 있습니다:

- **자율주행**: 점점 복잡한 환경에서 자율주행 가능
- **연결**: 유비쿼터스 V2X 통신 네트워크의 일부
- **전동화**: 전력망과 양방향 통합
- **소프트웨어 정의**: OTA 업데이트를 통한 지속적 개선
- **공유**: 원활한 MaaS 플랫폼의 일부

WIA 표준은 상호운용성, 보안 및 프라이버시를 유지하면서 이러한 새로운 기능을 지원하기 위해 계속 발전할 것입니다.

---

**WIA 커넥티드카 표준 Ebook 끝**

---

© 2025 World Industry Association (WIA). All rights reserved.
弘益人間 (홍익인간) · Benefit All Humanity
