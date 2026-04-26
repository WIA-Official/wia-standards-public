# WIA 커넥티드카 데이터 교환 표준

## V2X(Vehicle-to-Everything) 통신을 위한 완전한 기술 참조서

### 버전 1.0 | WIA 표준 기구

---

## 문서 정보

| 속성 | 값 |
|------|-----|
| **표준 ID** | WIA-CONNECTED-CAR-2025 |
| **버전** | 1.0.0 |
| **상태** | 발행됨 |
| **카테고리** | OTHER - 자동차 IoT |
| **최종 업데이트** | 2025-01-01 |

---

## 개요 요약

WIA 커넥티드카 데이터 교환 표준은 차량과 인프라, 클라우드 서비스, 모바일 기기 간의 원활한 데이터 교환을 가능하게 하는 V2X(Vehicle-to-Everything) 통신을 위한 포괄적인 프로토콜을 수립합니다. 이 표준은 커넥티드 차량 생태계에서 상호운용성에 대한 증가하는 요구를 해결합니다.

### 산업 과제

커넥티드카 산업은 서로 다른 제조사의 독점 프로토콜로 인한 심각한 분열에 직면해 있습니다. 이러한 표준화 부재는 혁신을 저해하고 제3자 서비스 제공업체에게 장벽을 만듭니다.

### 우리의 솔루션

WIA-CONNECTED-CAR는 다음을 제공하는 통합 프레임워크입니다:

- 차량 데이터 형식 및 통신 프로토콜 **표준화**
- 안전한 무선 업데이트 및 원격 진단 **지원**
- 자율주행 데이터 요구사항 **충족**
- MaaS(Mobility-as-a-Service) 통합 **촉진**

---

## 표준 아키텍처 개요

```typescript
// WIA 커넥티드카 표준 - 핵심 타입 정의
// V2X(Vehicle-to-Everything) 통신 프레임워크

/**
 * WIA 커넥티드카 표준 핵심 인터페이스
 * 커넥티드 차량 시스템의 완전한 구조 정의
 */
interface WIAConnectedCarStandard {
  metadata: StandardMetadata;
  vehicle: VehicleIdentity;
  telematics: TelematicsData;
  v2x: V2XCommunication;
  security: SecurityFramework;
  services: ConnectedServices;
}

interface StandardMetadata {
  standardId: "WIA-CONNECTED-CAR-2025";
  version: SemanticVersion;
  publishDate: ISO8601DateTime;
  complianceLevel: ComplianceLevel;
  certificationAuthority: string;
  validUntil: ISO8601DateTime;
  supersedes?: string;
  amendmentHistory: Amendment[];
}

interface SemanticVersion {
  major: number;
  minor: number;
  patch: number;
  preRelease?: string;
  buildMetadata?: string;
}

type ComplianceLevel = "BASIC" | "STANDARD" | "ADVANCED" | "AUTONOMOUS";

interface Amendment {
  version: string;
  date: ISO8601DateTime;
  description: string;
  author: string;
  approvedBy: string;
}

/**
 * 차량 식별 관리
 * 글로벌 차량 네트워크 전반의 고유 식별
 */
interface VehicleIdentity {
  vin: VehicleIdentificationNumber;
  wiaVehicleId: WIAVehicleIdentifier;
  manufacturer: ManufacturerInfo;
  model: VehicleModel;
  registration: RegistrationInfo;
  connectivity: ConnectivityModule;
}

interface VehicleIdentificationNumber {
  value: string;  // 17자리 VIN
  wmi: string;    // 세계 제조사 식별자 (1-3번 위치)
  vds: string;    // 차량 설명 섹션 (4-9번 위치)
  vis: string;    // 차량 식별 섹션 (10-17번 위치)
  checkDigit: string;
  validated: boolean;
}

interface WIAVehicleIdentifier {
  globalId: string;  // 시간 순서 고유성을 위한 UUID v7
  regionCode: ISO3166Alpha2;
  manufacturerCode: string;
  vehicleSequence: string;
  checksum: string;
  createdAt: ISO8601DateTime;
  expiresAt: ISO8601DateTime;
}

interface ManufacturerInfo {
  code: string;
  name: string;
  country: ISO3166Alpha2;
  brandFamily: string[];
  certificationStatus: CertificationStatus;
  contactInfo: ContactInformation;
}

interface CertificationStatus {
  wiaCompliant: boolean;
  certificationLevel: ComplianceLevel;
  certifiedDate: ISO8601DateTime;
  auditor: string;
  certificateId: string;
  nextAuditDate: ISO8601DateTime;
}

interface VehicleModel {
  name: string;
  year: number;
  generation: string;
  platform: string;
  bodyStyle: BodyStyle;
  driveType: DriveType;
  fuelType: FuelType;
  autonomyLevel: SAEAutonomyLevel;
  electrificationLevel: ElectrificationLevel;
}

type BodyStyle =
  | "SEDAN" | "SUV" | "CROSSOVER" | "HATCHBACK"
  | "COUPE" | "CONVERTIBLE" | "WAGON" | "PICKUP"
  | "VAN" | "MINIVAN" | "TRUCK";

type DriveType = "FWD" | "RWD" | "AWD" | "4WD";

type FuelType =
  | "GASOLINE" | "DIESEL" | "HYBRID"
  | "PLUG_IN_HYBRID" | "BATTERY_ELECTRIC"
  | "FUEL_CELL" | "NATURAL_GAS";

type SAEAutonomyLevel = 0 | 1 | 2 | 3 | 4 | 5;

type ElectrificationLevel =
  | "ICE" | "MHEV" | "HEV" | "PHEV" | "BEV" | "FCEV";

/**
 * 텔레매틱스 데이터 수집
 * 실시간 차량 상태 및 성능 모니터링
 */
interface TelematicsData {
  timestamp: ISO8601DateTime;
  location: VehicleLocation;
  motion: VehicleMotion;
  powertrain: PowertrainStatus;
  battery: BatteryStatus;
  diagnostics: DiagnosticData;
  environment: EnvironmentSensors;
  occupancy: OccupancyData;
}

interface VehicleLocation {
  position: GeoPosition;
  heading: Heading;
  elevation: Elevation;
  accuracy: PositionAccuracy;
  source: PositionSource;
  timestamp: ISO8601DateTime;
}

interface GeoPosition {
  latitude: number;   // WGS84 10진수 도
  longitude: number;  // WGS84 10진수 도
  altitude?: number;  // 해수면 위 미터
}

interface Heading {
  degrees: number;     // 0-360, 진북
  magneticVariation: number;
  source: "GPS" | "COMPASS" | "GYROSCOPE" | "FUSED";
}

interface PositionAccuracy {
  horizontal: number;  // 미터
  vertical?: number;   // 미터
  hdop?: number;       // 수평 정밀도 희석
  pdop?: number;       // 위치 정밀도 희석
  satellites?: number;
}

type PositionSource =
  | "GPS" | "GLONASS" | "GALILEO" | "BEIDOU"
  | "CELLULAR" | "WIFI" | "DEAD_RECKONING" | "FUSED";

interface VehicleMotion {
  speed: Speed;
  acceleration: Acceleration3D;
  rotation: Rotation3D;
  odometer: Odometer;
  tripMeter: TripMeter[];
}

interface Speed {
  value: number;      // km/h
  source: SpeedSource;
  accuracy: number;   // km/h
}

type SpeedSource = "WHEEL" | "GPS" | "RADAR" | "FUSED";

interface Acceleration3D {
  x: number;  // m/s² (종방향: + 전진)
  y: number;  // m/s² (횡방향: + 우측)
  z: number;  // m/s² (수직: + 상향)
  timestamp: ISO8601DateTime;
}

interface Rotation3D {
  pitch: number;  // 도 (-90 ~ 90)
  roll: number;   // 도 (-180 ~ 180)
  yaw: number;    // 도 (0 ~ 360)
  pitchRate: number;  // 도/초
  rollRate: number;
  yawRate: number;
}

interface PowertrainStatus {
  engineState: EngineState;
  rpm?: number;
  throttlePosition: number;     // 0-100%
  brakePressure: number;        // 0-100%
  steeringAngle: number;        // 도
  gearPosition: GearPosition;
  transmissionMode: TransmissionMode;
  fuelLevel?: FuelLevel;
  engineTemperature?: Temperature;
  oilPressure?: Pressure;
  coolantTemperature?: Temperature;
}

type EngineState =
  | "OFF" | "ACCESSORY" | "STARTING"
  | "RUNNING" | "STOPPING" | "ERROR";

type GearPosition =
  | "PARK" | "REVERSE" | "NEUTRAL" | "DRIVE"
  | "SPORT" | "LOW" | "MANUAL" | number;

type TransmissionMode =
  | "ECONOMY" | "NORMAL" | "SPORT" | "MANUAL" | "WINTER";

interface BatteryStatus {
  hvBattery?: HighVoltageBattery;
  lvBattery: LowVoltageBattery;
  chargingStatus?: ChargingStatus;
}

interface HighVoltageBattery {
  stateOfCharge: number;        // 0-100%
  stateOfHealth: number;        // 0-100%
  voltage: number;              // 볼트
  current: number;              // 암페어 (+ 충전, - 방전)
  temperature: Temperature;
  range: EstimatedRange;
  cellVoltages?: number[];
  cellTemperatures?: Temperature[];
  energyCapacity: number;       // kWh
  energyRemaining: number;      // kWh
  power: number;                // kW
  maxChargePower: number;       // kW
  maxDischargePower: number;    // kW
}

interface ChargingStatus {
  isCharging: boolean;
  chargeType?: ChargeType;
  chargeRate: number;           // kW
  timeToFullCharge?: number;    // 분
  targetSoC: number;            // 0-100%
  chargerConnected: boolean;
  chargePortOpen: boolean;
  scheduledChargeStart?: ISO8601DateTime;
}

type ChargeType =
  | "AC_LEVEL_1" | "AC_LEVEL_2" | "DC_FAST"
  | "WIRELESS" | "BATTERY_SWAP";

interface EstimatedRange {
  value: number;       // km
  confidence: number;  // 0-100%
  scenario: RangeScenario;
}

type RangeScenario =
  | "OPTIMISTIC" | "REALISTIC" | "CONSERVATIVE" | "CLIMATE_ADJUSTED";

/**
 * 진단 고장 코드 및 건강 모니터링
 */
interface DiagnosticData {
  dtcList: DiagnosticTroubleCode[];
  systemHealth: SystemHealthReport;
  maintenanceSchedule: MaintenanceItem[];
  recalls: RecallNotice[];
}

interface DiagnosticTroubleCode {
  code: string;           // P0xxx, B0xxx, C0xxx, U0xxx
  type: DTCType;
  severity: DTCSeverity;
  description: string;
  system: VehicleSystem;
  firstOccurrence: ISO8601DateTime;
  occurrenceCount: number;
  freezeFrame?: FreezeFrameData;
  isActive: boolean;
  isPending: boolean;
}

type DTCType =
  | "POWERTRAIN" | "BODY" | "CHASSIS" | "NETWORK";

type DTCSeverity =
  | "INFO" | "WARNING" | "CRITICAL" | "SAFETY";

type VehicleSystem =
  | "ENGINE" | "TRANSMISSION" | "FUEL" | "EMISSION"
  | "ELECTRICAL" | "HVAC" | "BRAKES" | "STEERING"
  | "SUSPENSION" | "AIRBAGS" | "ADAS" | "INFOTAINMENT";

interface FreezeFrameData {
  timestamp: ISO8601DateTime;
  engineRpm: number;
  vehicleSpeed: number;
  engineLoad: number;
  coolantTemp: number;
  fuelTrim: { short: number; long: number };
  throttlePosition: number;
  oxygenSensorReadings: number[];
}

interface SystemHealthReport {
  overall: HealthScore;
  subsystems: SubsystemHealth[];
  lastUpdated: ISO8601DateTime;
  nextScheduledCheck: ISO8601DateTime;
}

interface HealthScore {
  score: number;      // 0-100
  status: HealthStatus;
  trend: HealthTrend;
}

type HealthStatus = "EXCELLENT" | "GOOD" | "FAIR" | "POOR" | "CRITICAL";
type HealthTrend = "IMPROVING" | "STABLE" | "DECLINING";

/**
 * V2X 통신 프레임워크
 * 차량-모든 것 통신 프로토콜
 */
interface V2XCommunication {
  capabilities: V2XCapabilities;
  v2v: VehicleToVehicle;
  v2i: VehicleToInfrastructure;
  v2n: VehicleToNetwork;
  v2p: VehicleToPedestrian;
  v2g: VehicleToGrid;
}

interface V2XCapabilities {
  supportedProtocols: V2XProtocol[];
  dsrcSupport: boolean;
  cV2XSupport: boolean;
  fiveGSupport: boolean;
  wifiSupport: boolean;
  bluetoothSupport: boolean;
  uwbSupport: boolean;
}

type V2XProtocol =
  | "DSRC_WAVE" | "C_V2X_PC5" | "C_V2X_UU"
  | "5G_NR_V2X" | "IEEE_802_11P" | "ETSI_ITS_G5";

interface VehicleToVehicle {
  enabled: boolean;
  basicSafetyMessage: BSMConfiguration;
  collisionWarning: CollisionWarningConfig;
  platooning: PlatooningConfig;
  emergencyVehicleAlert: EVAConfig;
}

interface BSMConfiguration {
  transmitRate: number;        // Hz (일반적으로 10)
  maxRange: number;            // 미터
  channelCongestionControl: boolean;
  contentType: BSMContentType[];
}

type BSMContentType =
  | "POSITION" | "SPEED" | "HEADING" | "ACCELERATION"
  | "BRAKE_STATUS" | "SIZE" | "PATH_HISTORY" | "PATH_PREDICTION";

interface CollisionWarningConfig {
  enabled: boolean;
  forwardCollision: boolean;
  intersectionMovement: boolean;
  blindSpotWarning: boolean;
  laneChangeWarning: boolean;
  doNotPassWarning: boolean;
  warningThreshold: number;    // 충돌까지 초
}

interface VehicleToInfrastructure {
  enabled: boolean;
  signalPhaseAndTiming: SPaTConfig;
  mapData: MapDataConfig;
  tollCollection: TollConfig;
  parkingManagement: ParkingConfig;
  trafficManagement: TrafficManagementConfig;
}

interface SPaTConfig {
  enabled: boolean;
  intersectionIds: string[];
  greenLightOptimal: boolean;
  adaptiveSPaT: boolean;
  preemptionSupport: boolean;
}

interface VehicleToNetwork {
  enabled: boolean;
  cloudConnectivity: CloudConnectConfig;
  otaUpdates: OTAUpdateConfig;
  remoteServices: RemoteServicesConfig;
  fleetManagement: FleetManagementConfig;
}

interface CloudConnectConfig {
  primaryProvider: string;
  endpoints: CloudEndpoint[];
  connectionType: ConnectionType[];
  heartbeatInterval: number;
  dataUploadInterval: number;
  queuedMessageLimit: number;
}

type ConnectionType =
  | "4G_LTE" | "5G_NR" | "WIFI" | "SATELLITE";

interface OTAUpdateConfig {
  enabled: boolean;
  autoDownload: boolean;
  autoInstall: boolean;
  installWindow: TimeWindow;
  updateChannels: UpdateChannel[];
  maxDownloadSize: number;
  requireWifi: boolean;
  requireCharging: boolean;
}

type UpdateChannel =
  | "STABLE" | "BETA" | "CANARY" | "SECURITY";

interface TimeWindow {
  startHour: number;
  endHour: number;
  daysOfWeek: DayOfWeek[];
}

type DayOfWeek =
  | "MONDAY" | "TUESDAY" | "WEDNESDAY" | "THURSDAY"
  | "FRIDAY" | "SATURDAY" | "SUNDAY";

/**
 * 보안 프레임워크
 * 커넥티드 차량을 위한 포괄적인 사이버 보안
 */
interface SecurityFramework {
  authentication: AuthenticationConfig;
  encryption: EncryptionConfig;
  certificates: CertificateManagement;
  intrusionDetection: IDSConfig;
  secureBootChain: SecureBootConfig;
  firmwareSecurity: FirmwareSecurityConfig;
}

interface AuthenticationConfig {
  vehicleIdentity: VehicleIdentityCert;
  userAuthentication: UserAuthConfig;
  serviceAuthentication: ServiceAuthConfig;
  mutualTLS: boolean;
  tokenExpiry: number;
}

interface EncryptionConfig {
  atRestEncryption: EncryptionSpec;
  inTransitEncryption: EncryptionSpec;
  keyManagement: KeyManagementConfig;
  hardwareSecurityModule: HSMConfig;
}

interface EncryptionSpec {
  algorithm: EncryptionAlgorithm;
  keySize: number;
  mode: EncryptionMode;
}

type EncryptionAlgorithm =
  | "AES" | "CHACHA20" | "RSA" | "ECC" | "ECDSA";

type EncryptionMode =
  | "GCM" | "CBC" | "CTR" | "XTS";

interface HSMConfig {
  present: boolean;
  vendor: string;
  model: string;
  firmwareVersion: string;
  fipsCompliance: string;
  keyStorageCapacity: number;
}

/**
 * 구현 예시: 커넥티드카 클라이언트
 */
class WIAConnectedCarClient {
  private vehicleId: string;
  private telematicsBuffer: TelematicsData[] = [];
  private wsConnection: WebSocket | null = null;

  constructor(
    private config: ConnectedCarClientConfig,
    private securityManager: SecurityManager
  ) {
    this.vehicleId = config.vehicleId;
  }

  /**
   * WIA 커넥티드카 플랫폼에 연결 초기화
   */
  async initialize(): Promise<void> {
    // 플랫폼 인증
    const authToken = await this.securityManager.authenticate({
      vehicleId: this.vehicleId,
      certificate: this.config.vehicleCertificate
    });

    // 실시간 데이터용 WebSocket 연결 수립
    this.wsConnection = await this.establishWebSocket(authToken);

    // 텔레메트리 수집 시작
    this.startTelemetryCollection();

    console.log(`차량 커넥티드카 클라이언트 초기화 완료: ${this.vehicleId}`);
  }

  /**
   * 텔레매틱스 데이터 수집 및 전송
   */
  private startTelemetryCollection(): void {
    setInterval(() => {
      const telematics = this.collectTelematicsData();
      this.telematicsBuffer.push(telematics);

      if (this.telematicsBuffer.length >= this.config.batchSize) {
        this.transmitTelematicsData();
      }
    }, this.config.collectionInterval);
  }

  /**
   * 현재 차량 텔레매틱스 수집
   */
  private collectTelematicsData(): TelematicsData {
    return {
      timestamp: new Date().toISOString(),
      location: this.getVehicleLocation(),
      motion: this.getVehicleMotion(),
      powertrain: this.getPowertrainStatus(),
      battery: this.getBatteryStatus(),
      diagnostics: this.getDiagnosticData(),
      environment: this.getEnvironmentData(),
      occupancy: this.getOccupancyData()
    };
  }

  /**
   * V2X 기본 안전 메시지 전송
   */
  transmitBSM(bsm: BasicSafetyMessage): void {
    if (this.config.v2xEnabled) {
      const signedBSM = this.securityManager.signBSM(bsm);
      this.v2xTransmitter.broadcast(signedBSM);
    }
  }

  /**
   * OTA 업데이트 처리
   */
  async processOTAUpdate(update: OTAUpdatePackage): Promise<UpdateResult> {
    // 업데이트 서명 검증
    const isValid = await this.securityManager.verifyUpdateSignature(update);
    if (!isValid) {
      throw new SecurityError("유효하지 않은 업데이트 서명");
    }

    // 호환성 확인
    const compatible = this.checkCompatibility(update);
    if (!compatible.isCompatible) {
      return {
        success: false,
        error: compatible.reason
      };
    }

    // 업데이트 패키지 다운로드
    const downloadPath = await this.downloadUpdate(update);

    // 설치 예약
    return this.scheduleInstallation(downloadPath, update.installationPolicy);
  }
}

// 타입 별칭 및 추가 인터페이스
type ISO8601DateTime = string;
type ISO3166Alpha2 = string;
```

---

## 표준 발전 타임라인

```
┌─────────────────────────────────────────────────────────────────────────┐
│                  WIA 커넥티드카 표준 발전 과정                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  2010-2015: 기반 구축 시대                                               │
│  ├── DSRC/WAVE 초기 배포                                                │
│  ├── 기본 텔레매틱스 (OnStar, mbrace)                                   │
│  └── OEM 독점 프로토콜                                                   │
│                                                                          │
│  2015-2020: 연결성 확장                                                  │
│  ├── 4G LTE 내장 연결                                                   │
│  ├── C-V2X 표준화 (3GPP Release 14)                                     │
│  ├── OTA 업데이트 채택                                                   │
│  └── 커넥티드 서비스 확산                                                │
│                                                                          │
│  2020-2025: 지능 통합                                                    │
│  ├── V2X용 5G 배포                                                      │
│  ├── 엣지 컴퓨팅 통합                                                    │
│  ├── 자율주행 데이터 요구사항                                            │
│  └── WIA 표준 개발                                                       │
│                                                                          │
│  2025-2030: 자율주행 및 통합 시대                                        │
│  ├── WIA-CONNECTED-CAR-2025 발행 ◄── 현재                               │
│  ├── Level 4+ 자율주행 차량 지원                                         │
│  ├── 범용 V2X 상호운용성                                                 │
│  └── MaaS(Mobility-as-a-Service) 통합                                   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 이해관계자 생태계

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    커넥티드카 이해관계자 생태계                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                  │
│  │   차량      │    │  플랫폼     │    │  서비스     │                  │
│  │   OEM       │◄──►│  제공업체   │◄──►│  제공업체   │                  │
│  └─────────────┘    └─────────────┘    └─────────────┘                  │
│        │                  │                  │                          │
│        ▼                  ▼                  ▼                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                  │
│  │   Tier 1    │    │   통신      │    │   보험      │                  │
│  │  공급업체   │    │   사업자    │    │   회사      │                  │
│  └─────────────┘    └─────────────┘    └─────────────┘                  │
│        │                  │                  │                          │
│        ▼                  ▼                  ▼                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                  │
│  │   규제      │    │   스마트    │    │   플릿      │                  │
│  │   기관      │    │   시티      │    │   운영사    │                  │
│  └─────────────┘    └─────────────┘    └─────────────┘                  │
│                                                                          │
│                    ┌─────────────────┐                                  │
│                    │     소비자      │                                  │
│                    │   차량 소유자   │                                  │
│                    │   차량 공유자   │                                  │
│                    │   플릿 드라이버 │                                  │
│                    └─────────────────┘                                  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 문서 탐색

| 챕터 | 제목 | 설명 |
|------|------|------|
| 01 | [표지](./01-cover.md) | 소개 및 개요 (본 문서) |
| 02 | [시장 분석](./02-market-analysis.md) | 산업 현황 및 트렌드 |
| 03 | [데이터 형식](./03-data-formats.md) | 차량 데이터 스키마 및 인코딩 |
| 04 | [API 인터페이스](./04-api-interface.md) | REST, GraphQL, 스트리밍 API |
| 05 | [제어 프로토콜](./05-control-protocols.md) | V2X 및 텔레매틱스 프로토콜 |
| 06 | [통합](./06-integration.md) | OEM 및 생태계 통합 |
| 07 | [보안](./07-security.md) | 사이버보안 및 개인정보 보호 |
| 08 | [구현](./08-implementation.md) | 배포 및 인증 |
| 09 | [미래 트렌드](./09-future-trends.md) | 자율주행 및 MaaS |

---

**문서 관리:**
- **소유자:** WIA 자동차 기술위원회
- **관리자:** WIA 표준 기구
- **검토 주기:** 반년
- **분류:** 공개

---

© 2025 세계산업협회(WIA). 모든 권리 보유.
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
