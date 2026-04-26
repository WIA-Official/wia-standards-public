# 제5장: 커넥티드카 제어 프로토콜

## V2X 통신 및 차량 제어 시스템

이 장에서는 V2X(Vehicle-to-Everything) 통신과 원격 차량 제어를 가능하게 하는 통신 프로토콜을 상세히 설명하며, 지능형 교통 시스템의 기반을 형성합니다.

---

## V2X 프로토콜 스택

### 프로토콜 아키텍처 개요

```typescript
// WIA 커넥티드카 V2X 프로토콜 구현
// 완전한 V2X 통신 스택

/**
 * V2X 프로토콜 스택 아키텍처
 * OSI 모델 적용을 따르는 계층화 접근
 */
interface V2XProtocolStack {
  application: ApplicationLayer;
  facilities: FacilitiesLayer;
  networking: NetworkingLayer;
  access: AccessLayer;
  physical: PhysicalLayer;
  security: SecurityServices;
  management: ManagementServices;
}

/**
 * 애플리케이션 계층 - 안전 및 비안전 애플리케이션
 */
interface ApplicationLayer {
  safetyApplications: SafetyApplication[];
  mobilityApplications: MobilityApplication[];
  informationApplications: InfoApplication[];
  commercialApplications: CommercialApplication[];
}

interface SafetyApplication {
  id: string;
  name: string;
  type: SafetyAppType;
  priority: ApplicationPriority;
  latencyRequirement: number;  // 밀리초
  reliabilityTarget: number;   // 백분율
  messageTypes: string[];
}

type SafetyAppType =
  | "FORWARD_COLLISION_WARNING"      // 전방 충돌 경고
  | "INTERSECTION_MOVEMENT_ASSIST"   // 교차로 이동 지원
  | "BLIND_SPOT_WARNING"             // 사각지대 경고
  | "LANE_CHANGE_WARNING"            // 차선 변경 경고
  | "DO_NOT_PASS_WARNING"            // 추월 금지 경고
  | "EMERGENCY_VEHICLE_ALERT"        // 긴급 차량 알림
  | "QUEUE_WARNING"                  // 정체 경고
  | "REDUCED_SPEED_ZONE_WARNING"     // 감속 구역 경고
  | "CURVE_SPEED_WARNING"            // 커브 속도 경고
  | "ROAD_HAZARD_WARNING";           // 도로 위험 경고

type ApplicationPriority = "CRITICAL" | "HIGH" | "MEDIUM" | "LOW";

interface MobilityApplication {
  id: string;
  name: string;
  type: MobilityAppType;
  requirements: ApplicationRequirements;
}

type MobilityAppType =
  | "TRAFFIC_SIGNAL_PRIORITY"        // 교통 신호 우선권
  | "GREEN_LIGHT_OPTIMAL_SPEED"      // 녹색등 최적 속도
  | "INTELLIGENT_INTERSECTION"       // 지능형 교차로
  | "COOPERATIVE_ADAPTIVE_CRUISE"    // 협력적 적응 순항
  | "PLATOONING"                     // 군집 주행
  | "COOPERATIVE_MERGE"              // 협력적 합류
  | "SPEED_HARMONIZATION";           // 속도 조화

/**
 * 시설 계층 - 메시지 처리 및 프로세싱
 */
interface FacilitiesLayer {
  messageHandler: MessageHandler;
  vehicleDataProvider: VehicleDataProvider;
  positionProvider: PositionProvider;
  timeProvider: TimeProvider;
  localDynamicMap: LocalDynamicMap;
}

interface MessageHandler {
  encoders: MessageEncoder[];
  decoders: MessageDecoder[];
  validators: MessageValidator[];
  routing: MessageRouter;
}

interface MessageEncoder {
  messageType: string;
  encoding: EncodingType;
  encode(message: any): Buffer;
}

interface MessageDecoder {
  messageType: string;
  encoding: EncodingType;
  decode(buffer: Buffer): any;
}

type EncodingType = "UPER" | "XER" | "JER" | "OER";

/**
 * 네트워킹 및 전송 계층
 */
interface NetworkingLayer {
  geoNetworking: GeoNetworking;
  btp: BasicTransportProtocol;
  ipv6: IPv6Adaptation;
}

interface GeoNetworking {
  type: "ETSI_ITS_GN";
  version: string;
  capabilities: GNCapabilities;
  configuration: GNConfiguration;
}

interface GNCapabilities {
  geoUnicast: boolean;
  geoBroadcast: boolean;
  geoAnycast: boolean;
  topoBroadcast: boolean;
  singleHop: boolean;
  locationService: boolean;
}

type StationType =
  | "UNKNOWN" | "PEDESTRIAN" | "CYCLIST" | "MOPED"
  | "MOTORCYCLE" | "PASSENGER_CAR" | "BUS" | "LIGHT_TRUCK"
  | "HEAVY_TRUCK" | "TRAILER" | "SPECIAL_VEHICLE" | "TRAM"
  | "ROAD_SIDE_UNIT";

/**
 * 접속 계층 - DSRC 및 C-V2X
 */
interface AccessLayer {
  dsrc: DSRCAccess;
  cV2X: CV2XAccess;
  multiAccess: MultiAccessCoordination;
}

interface DSRCAccess {
  standard: "IEEE_802_11P" | "ETSI_ITS_G5";
  channels: DSRCChannel[];
  txPower: number;  // dBm
  dataRate: number; // Mbps
  channelSwitching: ChannelSwitchingMode;
}

interface DSRCChannel {
  number: number;
  frequency: number;     // MHz
  bandwidth: number;     // MHz
  usage: ChannelUsage;
  priority: number;
}

type ChannelUsage =
  | "CONTROL" | "SERVICE" | "SAFETY" | "NON_SAFETY";

interface CV2XAccess {
  mode: "PC5" | "UU" | "DUAL";
  release: "REL14" | "REL15" | "REL16" | "REL17";
  configuration: CV2XConfiguration;
}

interface CV2XConfiguration {
  resourceAllocation: ResourceAllocationMode;
  slBandwidth: number;
  txPool: TransmissionPool;
  rxPool: ReceptionPool;
  syncConfig: SyncConfiguration;
}

type ResourceAllocationMode = "MODE_1" | "MODE_2" | "MODE_3" | "MODE_4";

/**
 * V2X 메시지 구현 - SAE J2735
 */
class V2XMessageService {
  private encoder: BSMEncoder;
  private decoder: BSMDecoder;
  private securityService: V2XSecurityService;
  private transmitter: V2XTransmitter;

  constructor(
    private config: V2XServiceConfig,
    private vehicleDataProvider: VehicleDataProvider
  ) {
    this.encoder = new BSMEncoder();
    this.decoder = new BSMDecoder();
    this.securityService = new V2XSecurityService(config.security);
    this.transmitter = new V2XTransmitter(config.transmit);
  }

  /**
   * 기본 안전 메시지 생성 및 전송
   */
  async transmitBSM(): Promise<void> {
    // 차량 데이터 수집
    const vehicleData = await this.vehicleDataProvider.getCurrentState();

    // BSM Part I (핵심 데이터) 구성
    const bsmPart1: BSMCoreData = {
      msgCnt: this.getNextMessageCount(),
      id: this.config.temporaryId,
      secMark: this.getDSecond(),
      lat: this.encodeLat(vehicleData.position.latitude),
      long: this.encodeLong(vehicleData.position.longitude),
      elev: this.encodeElev(vehicleData.position.altitude),
      accuracy: this.encodeAccuracy(vehicleData.position.accuracy),
      transmission: this.encodeTransmission(vehicleData.transmission),
      speed: this.encodeSpeed(vehicleData.speed),
      heading: this.encodeHeading(vehicleData.heading),
      angle: this.encodeSteeringAngle(vehicleData.steeringAngle),
      accelSet: this.encodeAcceleration(vehicleData.acceleration),
      brakes: this.encodeBrakes(vehicleData.brakes),
      size: this.encodeSize(vehicleData.dimensions)
    };

    // BSM Part II (선택적 확장) 구성
    const bsmPart2 = this.buildBSMPart2(vehicleData);

    // UPER로 인코딩
    const encoded = this.encoder.encode({ coreData: bsmPart1, partII: bsmPart2 });

    // 메시지 서명
    const signed = await this.securityService.signMessage(encoded);

    // 전송
    await this.transmitter.broadcast(signed, {
      channel: this.config.transmitChannel,
      priority: "HIGH",
      rate: this.config.transmitRate
    });
  }

  /**
   * 수신된 V2X 메시지 처리
   */
  async processReceivedMessage(
    rawMessage: Buffer,
    metadata: ReceiveMetadata
  ): Promise<ProcessedV2XMessage> {
    // 서명 검증
    const verified = await this.securityService.verifyMessage(rawMessage);
    if (!verified.valid) {
      throw new SecurityError("메시지 서명 검증 실패");
    }

    // 메시지 디코딩
    const decoded = this.decoder.decode(verified.payload);

    // 타당성 검증
    const plausibility = this.validatePlausibility(decoded, metadata);
    if (!plausibility.valid) {
      console.warn("타당성 검사 실패:", plausibility.reasons);
    }

    // 내부 표현으로 변환
    return {
      type: decoded.messageType,
      content: this.convertToInternalFormat(decoded),
      source: {
        id: decoded.coreData?.id,
        certificate: verified.certificate,
        rssi: metadata.rssi,
        timestamp: metadata.timestamp
      },
      plausibility
    };
  }

  private encodeLat(lat: number): number {
    // 1/10 마이크로도로 변환
    return Math.round(lat * 10000000);
  }

  private encodeLong(long: number): number {
    return Math.round(long * 10000000);
  }

  private encodeElev(altitude: number): number {
    // 데시미터로 변환, 4096 오프셋
    return Math.round(altitude * 10) + 4096;
  }

  private encodeSpeed(speedKmh: number): number {
    // km/h를 0.02 m/s 단위로 변환
    const speedMs = speedKmh / 3.6;
    return Math.round(speedMs / 0.02);
  }

  private encodeHeading(heading: number): number {
    // 도를 0.0125도 단위로 변환
    return Math.round(heading / 0.0125);
  }

  private getNextMessageCount(): number {
    this.messageCount = (this.messageCount + 1) % 128;
    return this.messageCount;
  }

  private getDSecond(): number {
    const now = new Date();
    return (now.getSeconds() * 1000) + now.getMilliseconds();
  }

  private messageCount = 0;

  // 인코딩용 스텁 메서드
  private encodeAccuracy(accuracy: any): any { return accuracy; }
  private encodeTransmission(transmission: any): any { return transmission; }
  private encodeSteeringAngle(angle: number): any { return { value: angle }; }
  private encodeAcceleration(accel: any): any { return accel; }
  private encodeBrakes(brakes: any): any { return brakes; }
  private encodeSize(dimensions: any): any { return dimensions; }
  private buildBSMPart2(data: any): any { return null; }
  private validatePlausibility(msg: any, meta: any): any { return { valid: true }; }
  private convertToInternalFormat(msg: any): any { return msg; }
}

interface BSMCoreData {
  msgCnt: number;
  id: Buffer;
  secMark: number;
  lat: number;
  long: number;
  elev: number;
  accuracy: any;
  transmission: any;
  speed: number;
  heading: number;
  angle: any;
  accelSet: any;
  brakes: any;
  size: any;
}

interface V2XServiceConfig {
  temporaryId: Buffer;
  transmitChannel: number;
  transmitRate: number;
  security: SecurityConfig;
  transmit: TransmitConfig;
}

interface ReceiveMetadata {
  rssi: number;
  timestamp: Date;
  channel: number;
  dataRate: number;
}

interface ProcessedV2XMessage {
  type: string;
  content: any;
  source: MessageSource;
  plausibility: PlausibilityResult;
}

class SecurityError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "SecurityError";
  }
}
```

---

## 신호 위상 및 타이밍 (SPaT)

```typescript
/**
 * 교통 신호 통신을 위한 SPaT 메시지 처리
 * SAE J2735 신호 위상 및 타이밍 메시지
 */
interface SPaTMessage {
  timeStamp?: number;  // 연간 분
  name?: string;       // 설명 이름
  intersections: IntersectionState[];
}

interface IntersectionState {
  name?: string;
  id: IntersectionReferenceID;
  revision: number;
  status: IntersectionStatusObject;
  moy?: number;           // 연간 분
  timeStamp?: number;     // DSecond
  enabledLanes?: LaneID[];
  states: MovementState[];
  maneuverAssistList?: ConnectionManeuverAssist[];
}

interface IntersectionReferenceID {
  region?: number;   // 0-65535
  id: number;        // 0-65535
}

interface IntersectionStatusObject {
  manualControlIsEnabled: boolean;        // 수동 제어 활성화
  stopTimeIsActivated: boolean;           // 정지 시간 활성화
  failureFlash: boolean;                  // 고장 깜박임
  preemptIsActive: boolean;               // 선점 활성화
  signalPriorityIsActive: boolean;        // 신호 우선권 활성화
  fixedTimeOperation: boolean;            // 고정 시간 운영
  trafficDependentOperation: boolean;     // 교통 의존 운영
  standbyOperation: boolean;              // 대기 운영
  failureMode: boolean;                   // 고장 모드
  off: boolean;                           // 꺼짐
  recentMAPmessageUpdate: boolean;
  recentChangeInMAPassignedLanesIDsUsed: boolean;
  noValidMAPisAvailableAtThisTime: boolean;
  noValidSPaTisAvailableAtThisTime: boolean;
}

interface MovementState {
  movementName?: string;
  signalGroup: number;    // 0-255
  stateTimeSpeed: MovementEvent[];
  maneuverAssistList?: ConnectionManeuverAssist[];
}

interface MovementEvent {
  eventState: MovementPhaseState;
  timing?: TimeChangeDetails;
  speeds?: AdvisorySpeed[];
}

enum MovementPhaseState {
  unavailable = 0,                    // 사용 불가
  dark = 1,                           // 꺼짐
  stopThenProceed = 2,               // 정지 후 진행
  stopAndRemain = 3,                 // 정지 유지
  preMovement = 4,                   // 이동 준비
  permissiveMovementAllowed = 5,     // 허용 이동 가능
  protectedMovementAllowed = 6,      // 보호 이동 가능
  permissiveClearance = 7,           // 허용 정리
  protectedClearance = 8,            // 보호 정리
  cautionConflictingTraffic = 9      // 충돌 교통 주의
}

interface TimeChangeDetails {
  startTime?: number;       // TimeMark
  minEndTime: number;       // TimeMark
  maxEndTime?: number;      // TimeMark
  likelyTime?: number;      // TimeMark
  confidence?: number;      // 0-15
  nextTime?: number;        // TimeMark
}

interface AdvisorySpeed {
  type: SpeedAdvisoryType;
  speed?: number;           // 0-500 (0.1 m/s 단위)
  confidence?: SpeedConfidence;
  distance?: number;        // 미터
  class?: number;           // 차량 등급
}

enum SpeedAdvisoryType {
  none = 0,
  greenwave = 1,      // 녹색 파도
  ecoDrive = 2,       // 친환경 운전
  transit = 3         // 대중교통
}

type LaneID = number;  // 0-255

/**
 * SPaT 처리 및 GLOSA (녹색등 최적 속도 안내)
 */
class SPaTProcessor {
  private currentSPaT: Map<string, SPaTMessage> = new Map();
  private approachingIntersections: IntersectionApproach[] = [];

  /**
   * 수신된 SPaT 메시지 처리
   */
  processSPaT(spat: SPaTMessage, source: V2XMessageSource): void {
    for (const intersection of spat.intersections) {
      const key = this.getIntersectionKey(intersection.id);
      this.currentSPaT.set(key, spat);

      // 접근 중인 교차로 목록 업데이트
      this.updateApproachingIntersection(intersection);
    }
  }

  /**
   * 녹색등 최적 속도 안내 계산
   */
  calculateGLOSA(
    vehiclePosition: Position,
    vehicleSpeed: number,
    vehicleHeading: number,
    targetIntersection: IntersectionReferenceID,
    targetSignalGroup: number
  ): GLOSAResult {
    const key = this.getIntersectionKey(targetIntersection);
    const spat = this.currentSPaT.get(key);

    if (!spat) {
      return { available: false, reason: "SPaT 데이터 없음" };
    }

    const intersection = spat.intersections.find(
      i => i.id.id === targetIntersection.id
    );

    if (!intersection) {
      return { available: false, reason: "SPaT에서 교차로를 찾을 수 없음" };
    }

    const movement = intersection.states.find(
      s => s.signalGroup === targetSignalGroup
    );

    if (!movement) {
      return { available: false, reason: "신호 그룹을 찾을 수 없음" };
    }

    const currentEvent = movement.stateTimeSpeed[0];
    if (!currentEvent || !currentEvent.timing) {
      return { available: false, reason: "타이밍 정보 없음" };
    }

    // 교차로까지 거리 계산
    const distance = this.calculateDistanceToStopBar(
      vehiclePosition,
      targetIntersection
    );

    // 다양한 속도에서 교차로 도달 시간 계산
    const currentState = currentEvent.eventState;
    const timing = currentEvent.timing;

    // 현재 신호에 남은 시간 계산
    const now = this.getCurrentTimeMark();
    const minTimeRemaining = Math.max(0, timing.minEndTime - now);
    const maxTimeRemaining = timing.maxEndTime
      ? Math.max(0, timing.maxEndTime - now)
      : minTimeRemaining;
    const likelyTimeRemaining = timing.likelyTime
      ? Math.max(0, timing.likelyTime - now)
      : (minTimeRemaining + maxTimeRemaining) / 2;

    // 최적 속도 계산
    return this.computeOptimalSpeed(
      distance,
      vehicleSpeed,
      currentState,
      {
        min: minTimeRemaining / 10,  // 초로 변환
        max: maxTimeRemaining / 10,
        likely: likelyTimeRemaining / 10
      },
      {
        minSpeed: 20,   // km/h
        maxSpeed: 60,   // km/h (제한속도)
        targetAccel: 1.5,  // m/s²
        maxDecel: 3.0      // m/s²
      }
    );
  }

  private computeOptimalSpeed(
    distance: number,
    currentSpeed: number,
    signalState: MovementPhaseState,
    timeRemaining: { min: number; max: number; likely: number },
    constraints: SpeedConstraints
  ): GLOSAResult {
    const isGreen = signalState === MovementPhaseState.permissiveMovementAllowed ||
                    signalState === MovementPhaseState.protectedMovementAllowed;
    const isRed = signalState === MovementPhaseState.stopAndRemain;

    if (isGreen) {
      // 녹색에서 통과할 속도 계산
      const speedToPass = (distance / timeRemaining.min) * 3.6;  // m/s를 km/h로

      if (speedToPass <= constraints.maxSpeed && speedToPass >= constraints.minSpeed) {
        return {
          available: true,
          advisorySpeed: Math.round(speedToPass),
          speedType: "PROCEED_GREEN",
          confidence: this.calculateConfidence(timeRemaining),
          timeToGreen: 0,
          distance
        };
      } else if (speedToPass < constraints.minSpeed) {
        // 너무 느려서 통과 못함, 다음 녹색 속도 계산
        return this.calculateSpeedForNextGreen(
          distance,
          timeRemaining,
          constraints
        );
      } else {
        // 제한속도 초과 필요
        return {
          available: true,
          advisorySpeed: constraints.maxSpeed,
          speedType: "PROCEED_CAUTION",
          confidence: 0.7,
          distance
        };
      }
    } else if (isRed) {
      // 녹색 도착 시간 계산
      return this.calculateSpeedForNextGreen(
        distance,
        timeRemaining,
        constraints
      );
    }

    return { available: false, reason: "알 수 없는 신호 상태" };
  }

  private calculateSpeedForNextGreen(
    distance: number,
    currentPhaseRemaining: { min: number; max: number; likely: number },
    constraints: SpeedConstraints
  ): GLOSAResult {
    // 다음 신호 타이밍 추정 (정확성을 위해 전체 사이클 데이터 필요)
    const estimatedNextGreen = currentPhaseRemaining.likely + 5; // 5초 황색 + 버퍼

    const speedToArriveAtGreen = (distance / estimatedNextGreen) * 3.6;

    if (speedToArriveAtGreen >= constraints.minSpeed &&
        speedToArriveAtGreen <= constraints.maxSpeed) {
      return {
        available: true,
        advisorySpeed: Math.round(speedToArriveAtGreen),
        speedType: "ECO_APPROACH",
        confidence: 0.6,
        timeToGreen: estimatedNextGreen,
        distance
      };
    }

    // 필요한 속도가 제약 범위 밖 - 정지 권고
    return {
      available: true,
      advisorySpeed: 0,
      speedType: "PREPARE_TO_STOP",
      confidence: 0.8,
      timeToGreen: estimatedNextGreen,
      distance
    };
  }

  private calculateDistanceToStopBar(
    position: Position,
    intersection: IntersectionReferenceID
  ): number {
    // 정확한 정지선 위치를 위해 MAP 메시지 데이터 사용
    return 200;  // 미터
  }

  private getCurrentTimeMark(): number {
    const now = new Date();
    return now.getSeconds() * 10 + Math.floor(now.getMilliseconds() / 100);
  }

  private getIntersectionKey(id: IntersectionReferenceID): string {
    return `${id.region || 0}-${id.id}`;
  }

  private calculateConfidence(timing: { min: number; max: number }): number {
    const spread = timing.max - timing.min;
    if (spread < 2) return 0.95;
    if (spread < 5) return 0.85;
    if (spread < 10) return 0.70;
    return 0.50;
  }

  private updateApproachingIntersection(intersection: IntersectionState): void {
    // 접근 중인 교차로 추적 구현
  }
}

interface GLOSAResult {
  available: boolean;
  reason?: string;
  advisorySpeed?: number;
  speedType?: SpeedAdvisoryTypeString;
  confidence?: number;
  timeToGreen?: number;
  distance?: number;
}

type SpeedAdvisoryTypeString =
  | "PROCEED_GREEN"      // 녹색 진행
  | "PROCEED_CAUTION"    // 주의 진행
  | "ECO_APPROACH"       // 친환경 접근
  | "PREPARE_TO_STOP";   // 정지 준비

interface SpeedConstraints {
  minSpeed: number;
  maxSpeed: number;
  targetAccel: number;
  maxDecel: number;
}

interface Position {
  latitude: number;
  longitude: number;
  altitude?: number;
}

interface V2XMessageSource {
  id: string;
  rssi: number;
  channel: number;
}

interface IntersectionApproach {
  intersection: IntersectionReferenceID;
  distance: number;
  eta: number;
  signalGroup: number;
}
```

---

## 원격 차량 제어 프로토콜

```typescript
/**
 * 원격 차량 제어 프로토콜
 * 보안 명령 실행 프레임워크
 */
interface RemoteControlProtocol {
  authentication: AuthenticationLayer;
  authorization: AuthorizationLayer;
  commandExecution: CommandExecutionLayer;
  feedback: FeedbackLayer;
}

/**
 * 명령 유형 및 정의
 */
enum VehicleCommandType {
  // 보안 명령
  LOCK = "LOCK",                    // 잠금
  UNLOCK = "UNLOCK",                // 잠금 해제
  ARM_ALARM = "ARM_ALARM",          // 경보 활성화
  DISARM_ALARM = "DISARM_ALARM",    // 경보 비활성화

  // 파워트레인 명령
  REMOTE_START = "REMOTE_START",    // 원격 시동
  REMOTE_STOP = "REMOTE_STOP",      // 원격 시동 끄기

  // 공조 명령
  CLIMATE_ON = "CLIMATE_ON",        // 공조 켜기
  CLIMATE_OFF = "CLIMATE_OFF",      // 공조 끄기
  SET_TEMPERATURE = "SET_TEMPERATURE",  // 온도 설정
  DEFROST_ON = "DEFROST_ON",        // 서리 제거 켜기
  DEFROST_OFF = "DEFROST_OFF",      // 서리 제거 끄기
  SEAT_HEATER = "SEAT_HEATER",      // 시트 히터
  SEAT_COOLER = "SEAT_COOLER",      // 시트 쿨러
  STEERING_HEATER = "STEERING_HEATER",  // 핸들 히터

  // 충전 명령 (EV)
  START_CHARGING = "START_CHARGING",    // 충전 시작
  STOP_CHARGING = "STOP_CHARGING",      // 충전 중지
  SET_CHARGE_LIMIT = "SET_CHARGE_LIMIT",  // 충전 한도 설정
  SET_CHARGE_SCHEDULE = "SET_CHARGE_SCHEDULE",  // 충전 스케줄 설정
  OPEN_CHARGE_PORT = "OPEN_CHARGE_PORT",  // 충전 포트 열기
  CLOSE_CHARGE_PORT = "CLOSE_CHARGE_PORT",  // 충전 포트 닫기

  // 접근 명령
  OPEN_TRUNK = "OPEN_TRUNK",        // 트렁크 열기
  CLOSE_TRUNK = "CLOSE_TRUNK",      // 트렁크 닫기
  OPEN_FRUNK = "OPEN_FRUNK",        // 프렁크 열기
  OPEN_WINDOWS = "OPEN_WINDOWS",    // 창문 열기
  CLOSE_WINDOWS = "CLOSE_WINDOWS",  // 창문 닫기
  VENT_WINDOWS = "VENT_WINDOWS",    // 창문 환기

  // 위치 명령
  FLASH_LIGHTS = "FLASH_LIGHTS",    // 라이트 점멸
  HONK_HORN = "HONK_HORN",          // 경적 울리기
  LOCATE_VEHICLE = "LOCATE_VEHICLE",  // 차량 위치 찾기

  // 고급 명령
  SUMMON = "SUMMON",                // 소환
  PARK_ASSIST = "PARK_ASSIST",      // 주차 보조
  SOFTWARE_UPDATE = "SOFTWARE_UPDATE"  // 소프트웨어 업데이트
}

interface VehicleCommand {
  id: string;
  type: VehicleCommandType;
  vehicleId: string;
  userId: string;
  timestamp: Date;
  parameters?: CommandParameters;
  timeout: number;
  priority: CommandPriority;
  requiresConfirmation: boolean;
  expiresAt: Date;
}

type CommandParameters = Record<string, any>;

enum CommandPriority {
  CRITICAL = 0,   // 안전 관련 명령
  HIGH = 1,       // 보안 명령
  NORMAL = 2,     // 표준 명령
  LOW = 3         // 비긴급 명령
}

interface CommandResult {
  commandId: string;
  status: CommandStatus;
  startedAt?: Date;
  completedAt?: Date;
  progress?: number;
  result?: any;
  error?: CommandError;
}

enum CommandStatus {
  QUEUED = "QUEUED",              // 대기열
  PENDING = "PENDING",            // 대기 중
  SENT = "SENT",                  // 전송됨
  ACKNOWLEDGED = "ACKNOWLEDGED",  // 확인됨
  IN_PROGRESS = "IN_PROGRESS",    // 진행 중
  COMPLETED = "COMPLETED",        // 완료됨
  FAILED = "FAILED",              // 실패함
  TIMEOUT = "TIMEOUT",            // 시간 초과
  CANCELLED = "CANCELLED",        // 취소됨
  REJECTED = "REJECTED"           // 거부됨
}

interface CommandError {
  code: string;
  message: string;
  retryable: boolean;
  details?: any;
}

/**
 * 원격 제어 서비스 구현
 */
class RemoteControlService {
  private commandQueue: PriorityQueue<VehicleCommand>;
  private activeCommands: Map<string, CommandExecution> = new Map();
  private vehicleConnection: VehicleConnectionManager;

  constructor(
    private config: RemoteControlConfig,
    private authService: AuthenticationService,
    private authzService: AuthorizationService
  ) {
    this.commandQueue = new PriorityQueue((a, b) => a.priority - b.priority);
    this.vehicleConnection = new VehicleConnectionManager(config.connection);
  }

  /**
   * 실행을 위한 명령 제출
   */
  async submitCommand(
    command: Omit<VehicleCommand, "id" | "timestamp" | "expiresAt">
  ): Promise<CommandResult> {
    // 명령 ID 생성
    const commandId = crypto.randomUUID();

    // 사용자 인증 검증
    const authResult = await this.authService.validateSession(command.userId);
    if (!authResult.valid) {
      throw new AuthenticationError("유효하지 않은 세션");
    }

    // 권한 확인
    const authzResult = await this.authzService.checkPermission({
      userId: command.userId,
      vehicleId: command.vehicleId,
      action: command.type,
      context: { parameters: command.parameters }
    });

    if (!authzResult.allowed) {
      throw new AuthorizationError(
        `${command.type}에 대한 권한 없음`,
        authzResult.reason
      );
    }

    // 명령 매개변수 검증
    this.validateCommandParameters(command.type, command.parameters);

    // 차량 연결 확인
    const connectivity = await this.vehicleConnection.checkConnectivity(
      command.vehicleId
    );

    if (!connectivity.online) {
      // 나중에 전달하기 위해 명령 대기
      return this.queueOfflineCommand(command, commandId);
    }

    // 명령 실행
    return this.executeCommand({
      ...command,
      id: commandId,
      timestamp: new Date(),
      expiresAt: new Date(Date.now() + command.timeout * 1000)
    });
  }

  /**
   * 차량에서 명령 실행
   */
  private async executeCommand(command: VehicleCommand): Promise<CommandResult> {
    const execution = new CommandExecution(command);
    this.activeCommands.set(command.id, execution);

    try {
      // 상태를 대기 중으로 업데이트
      execution.updateStatus(CommandStatus.PENDING);

      // 보안 명령 페이로드 구성
      const payload = await this.buildSecurePayload(command);

      // 차량으로 전송
      execution.updateStatus(CommandStatus.SENT);
      const response = await this.vehicleConnection.sendCommand(
        command.vehicleId,
        payload,
        { timeout: command.timeout * 1000 }
      );

      // 응답 처리
      if (response.acknowledged) {
        execution.updateStatus(CommandStatus.ACKNOWLEDGED);
      }

      // 완료 대기
      const result = await this.waitForCompletion(execution, response);

      return result;
    } catch (error) {
      execution.updateStatus(CommandStatus.FAILED, {
        error: this.normalizeError(error)
      });
      throw error;
    } finally {
      this.activeCommands.delete(command.id);
    }
  }

  /**
   * 명령 매개변수 검증
   */
  private validateCommandParameters(
    type: VehicleCommandType,
    parameters?: CommandParameters
  ): void {
    const rules = COMMAND_VALIDATION_RULES[type];
    if (!rules) return;

    for (const rule of rules) {
      if (rule.required && !parameters?.[rule.field]) {
        throw new ValidationError(`필수 매개변수 누락: ${rule.field}`);
      }

      if (parameters?.[rule.field] !== undefined) {
        if (rule.min !== undefined && parameters[rule.field] < rule.min) {
          throw new ValidationError(
            `매개변수 ${rule.field}가 최솟값 미만: ${rule.min}`
          );
        }
        if (rule.max !== undefined && parameters[rule.field] > rule.max) {
          throw new ValidationError(
            `매개변수 ${rule.field}가 최댓값 초과: ${rule.max}`
          );
        }
        if (rule.enum && !rule.enum.includes(parameters[rule.field])) {
          throw new ValidationError(
            `${rule.field}의 유효하지 않은 값: ${parameters[rule.field]}`
          );
        }
      }
    }
  }

  private async buildSecurePayload(command: VehicleCommand): Promise<Buffer> {
    return Buffer.from(JSON.stringify(command));
  }

  private async waitForCompletion(
    execution: CommandExecution,
    response: CommandResponse
  ): Promise<CommandResult> {
    return execution.getResult();
  }

  private async queueOfflineCommand(
    command: any,
    commandId: string
  ): Promise<CommandResult> {
    return { commandId, status: CommandStatus.QUEUED };
  }

  private normalizeError(error: any): CommandError {
    return {
      code: error.code || "UNKNOWN",
      message: error.message,
      retryable: error.retryable ?? false
    };
  }
}

// 명령 매개변수에 대한 검증 규칙
const COMMAND_VALIDATION_RULES: Record<string, ValidationRule[]> = {
  [VehicleCommandType.SET_TEMPERATURE]: [
    { field: "temperature", required: true, min: 15, max: 28 },
    { field: "zone", required: false, enum: ["DRIVER", "PASSENGER", "ALL"] }
  ],
  [VehicleCommandType.SET_CHARGE_LIMIT]: [
    { field: "limit", required: true, min: 50, max: 100 }
  ],
  [VehicleCommandType.SEAT_HEATER]: [
    { field: "seat", required: true, enum: ["DRIVER", "PASSENGER", "REAR_LEFT", "REAR_RIGHT"] },
    { field: "level", required: true, min: 0, max: 3 }
  ],
  [VehicleCommandType.SUMMON]: [
    { field: "direction", required: true, enum: ["FORWARD", "REVERSE"] },
    { field: "distance", required: false, min: 0, max: 60 }
  ]
};

interface ValidationRule {
  field: string;
  required: boolean;
  min?: number;
  max?: number;
  enum?: any[];
}

// 오류 클래스
class AuthenticationError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "AuthenticationError";
  }
}

class AuthorizationError extends Error {
  constructor(message: string, public reason?: string) {
    super(message);
    this.name = "AuthorizationError";
  }
}

class ValidationError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "ValidationError";
  }
}
```

---

## CAN 버스 프로토콜 브리지

```typescript
/**
 * CAN 버스 프로토콜 브리지
 * 클라우드 명령을 차량 CAN 메시지로 변환
 */
interface CANBridge {
  translator: CommandTranslator;
  gateway: CANGateway;
  security: CANSecurity;
}

interface CANMessage {
  id: number;           // CAN ID (11비트 표준 또는 29비트 확장)
  extended: boolean;    // 확장 프레임 플래그
  data: Buffer;         // 최대 8바이트 (CAN 2.0) 또는 64바이트 (CAN FD)
  dlc: number;          // 데이터 길이 코드
  timestamp?: number;   // 메시지 타임스탬프
}

interface CANSignal {
  name: string;
  startBit: number;
  length: number;
  byteOrder: "LITTLE_ENDIAN" | "BIG_ENDIAN";
  valueType: "SIGNED" | "UNSIGNED";
  factor: number;
  offset: number;
  min: number;
  max: number;
  unit: string;
}

/**
 * CAN 데이터베이스 (DBC) 파서 및 핸들러
 */
class CANDatabase {
  private messages: Map<number, CANMessageDefinition> = new Map();
  private signalsByName: Map<string, { messageId: number; signal: CANSignal }> = new Map();

  /**
   * DBC 파일 로드
   */
  loadDBC(dbcContent: string): void {
    const lines = dbcContent.split("\n");
    let currentMessage: CANMessageDefinition | null = null;

    for (const line of lines) {
      const trimmed = line.trim();

      // 메시지 정의 파싱
      const messageMatch = trimmed.match(/^BO_ (\d+) (\w+): (\d+)/);
      if (messageMatch) {
        currentMessage = {
          id: parseInt(messageMatch[1]),
          name: messageMatch[2],
          length: parseInt(messageMatch[3]),
          signals: []
        };
        this.messages.set(currentMessage.id, currentMessage);
        continue;
      }

      // 신호 정의 파싱
      const signalMatch = trimmed.match(
        /^SG_ (\w+) : (\d+)\|(\d+)@([01])([+-]) \(([^,]+),([^)]+)\) \[([^|]+)\|([^\]]+)\] "([^"]*)" (.*)/
      );
      if (signalMatch && currentMessage) {
        const signal: CANSignal = {
          name: signalMatch[1],
          startBit: parseInt(signalMatch[2]),
          length: parseInt(signalMatch[3]),
          byteOrder: signalMatch[4] === "1" ? "LITTLE_ENDIAN" : "BIG_ENDIAN",
          valueType: signalMatch[5] === "+" ? "UNSIGNED" : "SIGNED",
          factor: parseFloat(signalMatch[6]),
          offset: parseFloat(signalMatch[7]),
          min: parseFloat(signalMatch[8]),
          max: parseFloat(signalMatch[9]),
          unit: signalMatch[10]
        };
        currentMessage.signals.push(signal);
        this.signalsByName.set(signal.name, {
          messageId: currentMessage.id,
          signal
        });
      }
    }
  }

  /**
   * CAN 메시지를 물리값으로 디코딩
   */
  decode(message: CANMessage): DecodedCANMessage | null {
    const definition = this.messages.get(message.id);
    if (!definition) return null;

    const signals: Record<string, number> = {};

    for (const signalDef of definition.signals) {
      const rawValue = this.extractSignalValue(message.data, signalDef);
      const physicalValue = rawValue * signalDef.factor + signalDef.offset;
      signals[signalDef.name] = physicalValue;
    }

    return {
      id: message.id,
      name: definition.name,
      signals,
      timestamp: message.timestamp
    };
  }

  /**
   * 물리값을 CAN 메시지로 인코딩
   */
  encode(messageName: string, signals: Record<string, number>): CANMessage {
    const definition = Array.from(this.messages.values()).find(
      m => m.name === messageName
    );

    if (!definition) {
      throw new Error(`데이터베이스에서 메시지 ${messageName}를 찾을 수 없음`);
    }

    const data = Buffer.alloc(definition.length);

    for (const [signalName, physicalValue] of Object.entries(signals)) {
      const signalDef = definition.signals.find(s => s.name === signalName);
      if (!signalDef) continue;

      const rawValue = Math.round((physicalValue - signalDef.offset) / signalDef.factor);
      this.packSignalValue(data, signalDef, rawValue);
    }

    return {
      id: definition.id,
      extended: false,
      data,
      dlc: definition.length
    };
  }

  private extractSignalValue(data: Buffer, signal: CANSignal): number {
    // 비트 추출 로직 구현
    return 0;
  }

  private packSignalValue(data: Buffer, signal: CANSignal, value: number): void {
    // 비트 패킹 로직 구현
  }
}

interface CANMessageDefinition {
  id: number;
  name: string;
  length: number;
  signals: CANSignal[];
}

interface DecodedCANMessage {
  id: number;
  name: string;
  signals: Record<string, number>;
  timestamp?: number;
}

/**
 * 명령-CAN 변환기
 */
class CommandToCANTranslator {
  private canDb: CANDatabase;
  private commandMappings: Map<VehicleCommandType, CommandCANMapping>;

  constructor(canDb: CANDatabase) {
    this.canDb = canDb;
    this.commandMappings = this.initializeMappings();
  }

  /**
   * 클라우드 명령을 CAN 메시지로 변환
   */
  translate(
    command: VehicleCommandType,
    parameters: CommandParameters
  ): CANMessage[] {
    const mapping = this.commandMappings.get(command);
    if (!mapping) {
      throw new Error(`명령에 대한 CAN 매핑 없음: ${command}`);
    }

    return mapping.generator(parameters, this.canDb);
  }

  private initializeMappings(): Map<VehicleCommandType, CommandCANMapping> {
    const mappings = new Map<VehicleCommandType, CommandCANMapping>();

    // 도어 잠금 명령
    mappings.set(VehicleCommandType.LOCK, {
      generator: (params, db) => [
        db.encode("BCM_DoorControl", {
          DoorLockRequest: 1,
          AllDoors: 1,
          RequestSource: 2  // 원격
        })
      ]
    });

    // 도어 잠금 해제 명령
    mappings.set(VehicleCommandType.UNLOCK, {
      generator: (params, db) => [
        db.encode("BCM_DoorControl", {
          DoorLockRequest: 0,
          AllDoors: 1,
          RequestSource: 2
        })
      ]
    });

    // 공조 제어
    mappings.set(VehicleCommandType.CLIMATE_ON, {
      generator: (params, db) => {
        const messages: CANMessage[] = [];

        // HVAC 전원 켜기
        messages.push(db.encode("HVAC_Control", {
          HVACPowerRequest: 1,
          ACRequest: 1,
          RecirculationMode: 0
        }));

        // 온도 설정 (제공된 경우)
        if (params.temperature) {
          messages.push(db.encode("HVAC_TempControl", {
            TargetTemperature: params.temperature,
            Zone: params.zone === "DRIVER" ? 0 : params.zone === "PASSENGER" ? 1 : 2
          }));
        }

        return messages;
      }
    });

    // 충전 제어
    mappings.set(VehicleCommandType.START_CHARGING, {
      generator: (params, db) => [
        db.encode("BMS_ChargeControl", {
          ChargeRequest: 1,
          ChargeMode: params.mode || 0,  // 0=일반, 1=급속
          TargetSOC: params.targetSoC || 80
        })
      ]
    });

    return mappings;
  }
}

interface CommandCANMapping {
  generator: (params: CommandParameters, db: CANDatabase) => CANMessage[];
}
```

---

## 요약

| 프로토콜 | 사용 사례 | 지연시간 | 범위 | 보안 |
|---------|----------|---------|------|------|
| **BSM (SAE J2735)** | V2V 안전 | <100ms | 300m | IEEE 1609.2 |
| **SPaT/MAP** | V2I 교통 | <200ms | 500m | IEEE 1609.2 |
| **C-V2X PC5** | 직접 V2X | <20ms | 500m | 3GPP 보안 |
| **원격 명령** | 차량 제어 | <30s | 전역 | TLS + mTLS |
| **CAN 버스** | 내부 ECU | <10ms | 차량 | SecOC |

---

**다음 장:** [제6장: 통합](./06-integration.md) - OEM 및 에코시스템 통합 패턴.

---

© 2025 World Industry Association (WIA). All rights reserved.
