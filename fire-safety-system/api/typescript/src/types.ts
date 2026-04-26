/**
 * WIA-CITY-013: Fire Safety System Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  lat: number;
  lon: number;
}

/**
 * Location information
 */
export interface Location {
  lat: number;
  lon: number;
  floor?: string;
  zone?: string;
  building?: string;
  address?: string;
  city?: string;
  country?: string;
  postalCode?: string;
}

// ============================================================================
// Fire Detection Sensors
// ============================================================================

/**
 * Sensor types
 */
export enum SensorType {
  SMOKE = 'SMOKE',                    // 연기 감지기
  HEAT = 'HEAT',                      // 열 감지기
  FLAME = 'FLAME',                    // 화염 감지기
  CO = 'CO',                          // 일산화탄소 감지기
  GAS = 'GAS',                        // 가스 감지기
  MULTI = 'MULTI',                    // 복합 감지기
}

/**
 * Sensor status
 */
export enum SensorStatus {
  NORMAL = 'NORMAL',                  // 정상
  ALERT = 'ALERT',                    // 경보
  WARNING = 'WARNING',                // 주의
  FAULT = 'FAULT',                    // 고장
  MAINTENANCE = 'MAINTENANCE',        // 점검 중
  OFFLINE = 'OFFLINE',                // 오프라인
}

/**
 * Smoke detector
 */
export interface SmokeDetector {
  sensorId: string;
  type: SensorType.SMOKE;
  location: Location;
  status: SensorStatus;
  smokeLevel_ppm?: number;            // 연기 농도 (ppm)
  visibility_m?: number;              // 가시거리 (m)
  batteryLevel_percent?: number;
  lastMaintenance?: Timestamp;
  manufacturer?: string;
  model?: string;
  installDate?: Timestamp;
}

/**
 * Heat detector
 */
export interface HeatDetector {
  sensorId: string;
  type: SensorType.HEAT;
  location: Location;
  status: SensorStatus;
  temperature_C: number;              // 현재 온도 (°C)
  threshold_C: number;                // 경보 임계값 (°C)
  rateOfRise_C_per_min?: number;      // 온도 상승률 (°C/min)
  batteryLevel_percent?: number;
  lastMaintenance?: Timestamp;
  manufacturer?: string;
  model?: string;
  installDate?: Timestamp;
}

/**
 * Flame detector
 */
export interface FlameDetector {
  sensorId: string;
  type: SensorType.FLAME;
  location: Location;
  status: SensorStatus;
  uvIntensity?: number;               // UV 강도
  irIntensity?: number;               // 적외선 강도
  flameDetected: boolean;
  detectionRange_m: number;           // 감지 범위 (m)
  batteryLevel_percent?: number;
  lastMaintenance?: Timestamp;
  manufacturer?: string;
  model?: string;
  installDate?: Timestamp;
}

/**
 * CO (Carbon Monoxide) detector
 */
export interface CODetector {
  sensorId: string;
  type: SensorType.CO;
  location: Location;
  status: SensorStatus;
  coLevel_ppm: number;                // CO 농도 (ppm)
  threshold_ppm: number;              // 경보 임계값 (ppm)
  batteryLevel_percent?: number;
  lastMaintenance?: Timestamp;
  manufacturer?: string;
  model?: string;
  installDate?: Timestamp;
}

/**
 * Generic fire sensor
 */
export type FireSensor = SmokeDetector | HeatDetector | FlameDetector | CODetector;

// ============================================================================
// Sprinkler System
// ============================================================================

/**
 * Sprinkler types
 */
export enum SprinklerType {
  WET_PIPE = 'WET_PIPE',              // 습식
  DRY_PIPE = 'DRY_PIPE',              // 건식
  PREACTION = 'PREACTION',            // 준비작동식
  DELUGE = 'DELUGE',                  // 일제살수식
}

/**
 * Sprinkler head status
 */
export enum SprinklerStatus {
  READY = 'READY',                    // 대기
  ACTIVATED = 'ACTIVATED',            // 작동 중
  DISCHARGED = 'DISCHARGED',          // 방수 완료
  FAULT = 'FAULT',                    // 고장
  MAINTENANCE = 'MAINTENANCE',        // 점검 중
}

/**
 * Sprinkler head
 */
export interface SprinklerHead {
  headId: string;
  type: SprinklerType;
  location: Location;
  status: SprinklerStatus;
  activationTemp_C?: number;          // 작동 온도 (°C)
  coverageArea_m2: number;            // 방호 면적 (m²)
  flowRate_lpm?: number;              // 유량 (L/min)
  activatedAt?: Timestamp;
  manufacturer?: string;
  model?: string;
  installDate?: Timestamp;
}

/**
 * Sprinkler system
 */
export interface SprinklerSystem {
  systemId: string;
  name: string;
  type: SprinklerType;
  heads: SprinklerHead[];
  waterPressure_bar: number;          // 수압 (bar)
  waterFlow_lpm?: number;             // 유량 (L/min)
  tankCapacity_L?: number;            // 수조 용량 (L)
  tankLevel_percent?: number;         // 수조 수위 (%)
  pumpStatus: 'ON' | 'OFF' | 'STANDBY' | 'FAULT';
  valveStatus: 'OPEN' | 'CLOSED' | 'PARTIAL';
  lastTest?: Timestamp;
}

// ============================================================================
// Fire Alarm System
// ============================================================================

/**
 * Alarm types
 */
export enum AlarmType {
  AUDIBLE = 'AUDIBLE',                // 음향 경보
  VISUAL = 'VISUAL',                  // 시각 경보
  VOICE = 'VOICE',                    // 음성 경보
  TEXT = 'TEXT',                      // 문자 알림
  COMBINED = 'COMBINED',              // 복합 경보
}

/**
 * Alarm severity
 */
export enum AlarmSeverity {
  INFO = 'INFO',                      // 정보
  WARNING = 'WARNING',                // 주의
  CRITICAL = 'CRITICAL',              // 심각
  EMERGENCY = 'EMERGENCY',            // 긴급
}

/**
 * Fire alarm
 */
export interface FireAlarm {
  alarmId: string;
  type: AlarmType;
  severity: AlarmSeverity;
  location: Location;
  triggeredBy: string;                // 트리거 센서/장치 ID
  triggeredAt: Timestamp;
  message?: string;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
  silenced: boolean;
  silencedAt?: Timestamp;
}

/**
 * Notification channel
 */
export enum NotificationChannel {
  SMS = 'SMS',
  EMAIL = 'EMAIL',
  PUSH = 'PUSH',
  VOICE_CALL = 'VOICE_CALL',
  PA_SYSTEM = 'PA_SYSTEM',            // 공공 방송 시스템
}

/**
 * Notification
 */
export interface Notification {
  notificationId: string;
  channel: NotificationChannel;
  recipient: string;
  message: string;
  sentAt: Timestamp;
  delivered: boolean;
  deliveredAt?: Timestamp;
  read?: boolean;
  readAt?: Timestamp;
}

// ============================================================================
// Evacuation Routes
// ============================================================================

/**
 * Exit types
 */
export enum ExitType {
  MAIN_EXIT = 'MAIN_EXIT',            // 주 출구
  EMERGENCY_EXIT = 'EMERGENCY_EXIT',  // 비상구
  FIRE_ESCAPE = 'FIRE_ESCAPE',        // 화재 대피용 계단
  RESCUE_WINDOW = 'RESCUE_WINDOW',    // 구조용 창
}

/**
 * Exit status
 */
export enum ExitStatus {
  CLEAR = 'CLEAR',                    // 통행 가능
  BLOCKED = 'BLOCKED',                // 차단됨
  HAZARDOUS = 'HAZARDOUS',            // 위험
  DAMAGED = 'DAMAGED',                // 손상됨
}

/**
 * Emergency exit
 */
export interface EmergencyExit {
  exitId: string;
  type: ExitType;
  location: Location;
  status: ExitStatus;
  capacity_persons_per_min: number;   // 수용 인원 (명/분)
  width_m: number;                    // 출구 너비 (m)
  doorStatus?: 'OPEN' | 'CLOSED' | 'LOCKED';
  signage?: {
    illuminated: boolean;
    batteryBackup: boolean;
    lastTest?: Timestamp;
  };
}

/**
 * Evacuation route
 */
export interface EvacuationRoute {
  routeId: string;
  name: string;
  startLocation: Location;
  endLocation: Location;
  exits: EmergencyExit[];
  distance_m: number;                 // 거리 (m)
  estimatedTime_sec: number;          // 예상 소요 시간 (초)
  capacity_persons: number;           // 수용 인원 (명)
  accessible: boolean;                // 장애인 접근 가능 여부
  currentStatus: 'CLEAR' | 'CONGESTED' | 'BLOCKED' | 'HAZARDOUS';
  obstructions?: string[];
}

// ============================================================================
// Fire Extinguishers & Hydrants
// ============================================================================

/**
 * Extinguisher types
 */
export enum ExtinguisherType {
  WATER = 'WATER',                    // 물 소화기
  FOAM = 'FOAM',                      // 포말 소화기
  CO2 = 'CO2',                        // 이산화탄소 소화기
  DRY_POWDER = 'DRY_POWDER',          // 분말 소화기
  HALON = 'HALON',                    // 할론 소화기
  CLEAN_AGENT = 'CLEAN_AGENT',        // 청정 소화약제
}

/**
 * Fire class
 */
export enum FireClass {
  CLASS_A = 'CLASS_A',                // 일반 화재 (목재, 종이, 섬유)
  CLASS_B = 'CLASS_B',                // 유류 화재
  CLASS_C = 'CLASS_C',                // 전기 화재
  CLASS_D = 'CLASS_D',                // 금속 화재
  CLASS_K = 'CLASS_K',                // 주방 화재
}

/**
 * Fire extinguisher
 */
export interface FireExtinguisher {
  extinguisherId: string;
  type: ExtinguisherType;
  location: Location;
  capacity_kg: number;                // 용량 (kg)
  pressure_bar?: number;              // 압력 (bar)
  suitableFor: FireClass[];           // 적용 가능한 화재 등급
  lastInspection: Timestamp;
  nextInspection: Timestamp;
  expiryDate?: Timestamp;
  status: 'READY' | 'USED' | 'EXPIRED' | 'MAINTENANCE';
  manufacturer?: string;
  model?: string;
  installDate?: Timestamp;
}

/**
 * Fire hydrant
 */
export interface FireHydrant {
  hydrantId: string;
  location: Location;
  type: 'INDOOR' | 'OUTDOOR';
  waterPressure_bar: number;          // 수압 (bar)
  flowRate_lpm: number;               // 유량 (L/min)
  hoseLength_m: number;               // 호스 길이 (m)
  nozzleType?: string;
  lastInspection: Timestamp;
  nextInspection: Timestamp;
  status: 'READY' | 'IN_USE' | 'FAULT' | 'MAINTENANCE';
  installDate?: Timestamp;
}

// ============================================================================
// Fire Doors & Fire Walls
// ============================================================================

/**
 * Fire door types
 */
export enum FireDoorType {
  SINGLE_SWING = 'SINGLE_SWING',      // 단일 여닫이문
  DOUBLE_SWING = 'DOUBLE_SWING',      // 이중 여닫이문
  SLIDING = 'SLIDING',                // 미닫이문
  ROLLING = 'ROLLING',                // 롤링 셔터
}

/**
 * Fire resistance rating
 */
export enum FireRating {
  FRR_30 = 'FRR_30',                  // 30분 내화
  FRR_60 = 'FRR_60',                  // 60분 내화
  FRR_90 = 'FRR_90',                  // 90분 내화
  FRR_120 = 'FRR_120',                // 120분 내화
}

/**
 * Fire door
 */
export interface FireDoor {
  doorId: string;
  type: FireDoorType;
  location: Location;
  fireRating: FireRating;
  status: 'OPEN' | 'CLOSED' | 'AUTOMATIC' | 'FAULT';
  autoCloseEnabled: boolean;
  closerType?: 'HYDRAULIC' | 'SPRING' | 'ELECTROMAGNETIC';
  holdOpenDevice?: boolean;
  lastInspection: Timestamp;
  nextInspection: Timestamp;
  manufacturer?: string;
  model?: string;
  installDate?: Timestamp;
}

/**
 * Fire zone
 */
export interface FireZone {
  zoneId: string;
  name: string;
  building: string;
  floor: string;
  area_m2: number;                    // 면적 (m²)
  fireRating: FireRating;
  sensors: FireSensor[];
  sprinklers: SprinklerHead[];
  extinguishers: FireExtinguisher[];
  fireDoors: FireDoor[];
  occupancyLimit: number;             // 최대 수용 인원
  currentOccupancy?: number;          // 현재 인원
  evacuationRoutes: EvacuationRoute[];
}

// ============================================================================
// HVAC Integration
// ============================================================================

/**
 * HVAC control action
 */
export enum HVACAction {
  SHUTDOWN = 'SHUTDOWN',              // 전체 정지
  SMOKE_CONTROL = 'SMOKE_CONTROL',    // 배연 모드
  PRESSURIZE = 'PRESSURIZE',          // 가압 모드
  NORMAL = 'NORMAL',                  // 정상 운전
}

/**
 * HVAC system status
 */
export interface HVACStatus {
  systemId: string;
  zone: string;
  action: HVACAction;
  fanStatus: 'ON' | 'OFF' | 'STANDBY';
  damperStatus: 'OPEN' | 'CLOSED' | 'MODULATING';
  airPressure_Pa?: number;            // 공기 압력 (Pa)
  airflow_m3h?: number;               // 풍량 (m³/h)
  smokeExhaustActive: boolean;
}

// ============================================================================
// Elevator Control
// ============================================================================

/**
 * Elevator status
 */
export interface ElevatorStatus {
  elevatorId: string;
  currentFloor: string;
  status: 'NORMAL' | 'FIRE_MODE' | 'EMERGENCY_STOP' | 'OUT_OF_SERVICE';
  doorsStatus: 'OPEN' | 'CLOSED' | 'OPENING' | 'CLOSING';
  occupancy?: number;
  recalledToFloor?: string;           // 화재 시 이동할 층
}

// ============================================================================
// Fire Incident
// ============================================================================

/**
 * Incident severity
 */
export enum IncidentSeverity {
  MINOR = 'MINOR',                    // 경미
  MODERATE = 'MODERATE',              // 중간
  MAJOR = 'MAJOR',                    // 주요
  CATASTROPHIC = 'CATASTROPHIC',      // 재난
}

/**
 * Incident status
 */
export enum IncidentStatus {
  DETECTED = 'DETECTED',              // 감지됨
  CONFIRMED = 'CONFIRMED',            // 확인됨
  RESPONDING = 'RESPONDING',          // 대응 중
  CONTAINED = 'CONTAINED',            // 진압 중
  EXTINGUISHED = 'EXTINGUISHED',      // 진화 완료
  RESOLVED = 'RESOLVED',              // 해결됨
  FALSE_ALARM = 'FALSE_ALARM',        // 오보
}

/**
 * Fire incident
 */
export interface FireIncident {
  incidentId: string;
  detectedAt: Timestamp;
  location: Location;
  severity: IncidentSeverity;
  status: IncidentStatus;
  fireClass?: FireClass;
  detectedBy: string[];               // 감지 센서 IDs
  affectedZones: string[];            // 영향 받는 구역 IDs
  alarmsTriggered: string[];          // 트리거된 경보 IDs
  sprinklersActivated: string[];      // 작동된 스프링클러 IDs
  evacuationInitiated: boolean;
  evacuatedPersons?: number;
  fireDepartmentNotified: boolean;
  fireDepartmentArrival?: Timestamp;
  estimatedDamage_USD?: number;
  injuries?: number;
  fatalities?: number;
  notes?: string;
  resolvedAt?: Timestamp;
}

// ============================================================================
// System Integration
// ============================================================================

/**
 * Building management system integration
 */
export interface BMSIntegration {
  bmsId: string;
  connectedSystems: {
    hvac?: HVACStatus[];
    elevators?: ElevatorStatus[];
    accessControl?: any[];
    lighting?: any[];
  };
  lastSync: Timestamp;
  status: 'CONNECTED' | 'DISCONNECTED' | 'ERROR';
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Fire detection event
 */
export interface FireDetectionEvent {
  eventId: string;
  timestamp: Timestamp;
  sensorId: string;
  sensorType: SensorType;
  location: Location;
  readings: {
    temperature_C?: number;
    smokeLevel_ppm?: number;
    coLevel_ppm?: number;
    flameDetected?: boolean;
  };
  alarmTriggered: boolean;
}

/**
 * System health check
 */
export interface SystemHealthCheck {
  checkId: string;
  timestamp: Timestamp;
  overall: 'HEALTHY' | 'WARNING' | 'CRITICAL';
  components: {
    sensors: { total: number; healthy: number; faulty: number };
    sprinklers: { total: number; ready: number; fault: number };
    alarms: { total: number; operational: number; fault: number };
    exits: { total: number; clear: number; blocked: number };
  };
  maintenanceRequired: string[];
  recommendations?: string[];
}

/**
 * Evacuation status
 */
export interface EvacuationStatus {
  initiated: boolean;
  initiatedAt?: Timestamp;
  totalOccupancy: number;
  evacuated: number;
  remaining: number;
  routes: {
    routeId: string;
    status: string;
    personsUsing: number;
  }[];
  estimatedCompletion?: Timestamp;
}

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
