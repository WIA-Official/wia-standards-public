/**
 * WIA-CITY-010: HVAC System Standard - TypeScript Type Definitions
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
  address?: string;
  city?: string;
  country?: string;
  postalCode?: string;
  building?: string;
  floor?: number;
  room?: string;
}

// ============================================================================
// HVAC System Types
// ============================================================================

/**
 * HVAC system type
 */
export enum SystemType {
  SPLIT_AC = 'SPLIT_AC',                     // 분리형 에어컨
  MULTI_SPLIT = 'MULTI_SPLIT',               // 멀티 시스템
  VRF = 'VRF',                               // 가변 냉매 유량
  VRV = 'VRV',                               // VRF와 동일 (다이킨 상표명)
  CHILLER = 'CHILLER',                       // 칠러 시스템
  HEAT_PUMP = 'HEAT_PUMP',                   // 히트펌프
  AHU = 'AHU',                               // 공조기
  FCU = 'FCU',                               // 팬코일 유닛
  RADIANT = 'RADIANT',                       // 복사 냉난방
  GEOTHERMAL = 'GEOTHERMAL',                 // 지열
  HYBRID = 'HYBRID',                         // 하이브리드
}

/**
 * Operating mode
 */
export enum OperatingMode {
  OFF = 'OFF',                               // 꺼짐
  COOLING = 'COOLING',                       // 냉방
  HEATING = 'HEATING',                       // 난방
  AUTO = 'AUTO',                             // 자동
  DRY = 'DRY',                               // 제습
  FAN_ONLY = 'FAN_ONLY',                     // 송풍
  VENTILATION = 'VENTILATION',               // 환기
  ECO = 'ECO',                               // 절전
  TURBO = 'TURBO',                           // 급속
}

/**
 * Fan speed
 */
export enum FanSpeed {
  OFF = 'OFF',
  AUTO = 'AUTO',
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  TURBO = 'TURBO',
}

/**
 * System status
 */
export enum SystemStatus {
  RUNNING = 'RUNNING',                       // 가동 중
  STOPPED = 'STOPPED',                       // 정지
  STANDBY = 'STANDBY',                       // 대기
  ERROR = 'ERROR',                           // 오류
  MAINTENANCE = 'MAINTENANCE',               // 정비 중
  OFFLINE = 'OFFLINE',                       // 오프라인
}

// ============================================================================
// Zone Management
// ============================================================================

/**
 * Zone information
 */
export interface Zone {
  zone_id: string;
  name: string;
  building?: string;
  floor?: number;

  // Geometry
  area_m2: number;                           // 면적 (m²)
  volume_m3: number;                         // 체적 (m³)
  ceiling_height_m: number;                  // 천장 높이 (m)

  // Thermal properties
  design_cooling_load_kw?: number;           // 설계 냉방 부하 (kW)
  design_heating_load_kw?: number;           // 설계 난방 부하 (kW)
  thermal_mass?: ThermalMass;

  // Occupancy
  max_occupancy?: number;
  current_occupancy?: number;
  occupancy_sensor?: boolean;

  // Equipment
  hvac_units: string[];                      // HVAC unit IDs
  sensors: string[];                         // Sensor IDs

  // Control
  priority: ZonePriority;
  independent_control: boolean;
}

/**
 * Thermal mass
 */
export enum ThermalMass {
  LIGHT = 'LIGHT',                           // 경량 (목조)
  MEDIUM = 'MEDIUM',                         // 중량 (일반 건물)
  HEAVY = 'HEAVY',                           // 중량 (콘크리트)
}

/**
 * Zone priority
 */
export enum ZonePriority {
  CRITICAL = 'CRITICAL',                     // 핵심 (서버실, 수술실)
  HIGH = 'HIGH',                             // 높음 (회의실, 사무실)
  NORMAL = 'NORMAL',                         // 보통 (복도, 로비)
  LOW = 'LOW',                               // 낮음 (창고, 주차장)
}

/**
 * Zone schedule
 */
export interface ZoneSchedule {
  zone_id: string;
  schedules: SchedulePeriod[];
}

/**
 * Schedule period
 */
export interface SchedulePeriod {
  name: string;
  days: DayOfWeek[];
  periods: TimePeriod[];
}

/**
 * Day of week
 */
export enum DayOfWeek {
  MON = 'MON',
  TUE = 'TUE',
  WED = 'WED',
  THU = 'THU',
  FRI = 'FRI',
  SAT = 'SAT',
  SUN = 'SUN',
}

/**
 * Time period
 */
export interface TimePeriod {
  start: string;                             // HH:MM format
  end: string;                               // HH:MM format
  mode: OccupancyMode;
  temp_setpoint_c?: number;
  humidity_setpoint_percent?: number;
}

/**
 * Occupancy mode
 */
export enum OccupancyMode {
  OCCUPIED = 'OCCUPIED',                     // 재실
  UNOCCUPIED = 'UNOCCUPIED',                 // 부재
  SETBACK = 'SETBACK',                       // 세트백
  PREHEAT = 'PREHEAT',                       // 예열
  PRECOOL = 'PRECOOL',                       // 예냉
}

// ============================================================================
// Temperature & Humidity Control
// ============================================================================

/**
 * Temperature setpoint
 */
export interface TemperatureSetpoint {
  zone_id: string;
  setpoint_c: number;                        // 설정온도 (°C)
  mode: OperatingMode;
  deadband_c?: number;                       // 데드밴드 (°C)
  timestamp: Timestamp;
}

/**
 * Humidity setpoint
 */
export interface HumiditySetpoint {
  zone_id: string;
  setpoint_percent: number;                  // 설정습도 (%)
  deadband_percent?: number;                 // 데드밴드 (%)
  timestamp: Timestamp;
}

/**
 * Thermal comfort
 */
export interface ThermalComfort {
  zone_id: string;
  temperature_c: number;
  humidity_percent: number;
  pmv: number;                               // Predicted Mean Vote (-3 to +3)
  ppd: number;                               // Predicted Percentage of Dissatisfied (%)
  comfort_level: ComfortLevel;
  timestamp: Timestamp;
}

/**
 * Comfort level
 */
export enum ComfortLevel {
  OPTIMAL = 'OPTIMAL',                       // 최적
  COMFORTABLE = 'COMFORTABLE',               // 쾌적
  ACCEPTABLE = 'ACCEPTABLE',                 // 수용 가능
  UNCOMFORTABLE = 'UNCOMFORTABLE',           // 불쾌
  VERY_UNCOMFORTABLE = 'VERY_UNCOMFORTABLE', // 매우 불쾌
}

// ============================================================================
// Air Quality Integration
// ============================================================================

/**
 * Air quality parameters
 */
export interface AirQualityParameters {
  zone_id: string;
  timestamp: Timestamp;

  // Gas concentrations
  co2_ppm?: number;                          // 이산화탄소 (ppm)
  co_ppm?: number;                           // 일산화탄소 (ppm)

  // Particulate matter
  pm25_ugm3?: number;                        // PM2.5 (μg/m³)
  pm10_ugm3?: number;                        // PM10 (μg/m³)

  // VOCs
  tvoc_ppb?: number;                         // 총휘발성유기화합물 (ppb)

  // Environmental
  temperature_c: number;
  humidity_percent: number;
}

/**
 * Ventilation control
 */
export interface VentilationControl {
  system_id: string;
  mode: VentilationMode;

  // Airflow
  outdoor_air_flow_cmh?: number;             // 외기 풍량 (m³/h)
  outdoor_air_percent?: number;              // 외기 비율 (%)
  air_change_rate_ach?: number;              // 환기율 (ACH)

  // CO2-based control
  co2_based_control: boolean;
  co2_setpoint_ppm?: number;

  // Economizer
  economizer_enabled: boolean;
  economizer_active?: boolean;

  timestamp: Timestamp;
}

/**
 * Ventilation mode
 */
export enum VentilationMode {
  MINIMUM = 'MINIMUM',                       // 최소 환기
  DEMAND_CONTROLLED = 'DEMAND_CONTROLLED',   // 요구 제어 환기
  ECONOMIZER = 'ECONOMIZER',                 // 이코노마이저
  MAXIMUM = 'MAXIMUM',                       // 최대 환기
  NIGHT_PURGE = 'NIGHT_PURGE',               // 야간 퍼지
}

/**
 * Filter status
 */
export interface FilterStatus {
  filter_id: string;
  filter_type: string;                       // G4, F7, H13 등
  location: string;

  // Status
  installed_date: Timestamp;
  last_replaced: Timestamp;
  operating_hours: number;

  // Performance
  pressure_drop_pa: number;                  // 압력 강하 (Pa)
  pressure_threshold_pa: number;             // 교체 기준 압력 (Pa)
  efficiency_percent?: number;

  // Maintenance
  needs_replacement: boolean;
  estimated_replacement_date?: Timestamp;
  replacement_reason?: string;
}

// ============================================================================
// Energy Efficiency
// ============================================================================

/**
 * Energy metrics
 */
export interface EnergyMetrics {
  system_id: string;
  timestamp: Timestamp;
  period: TimePeriod;

  // Power
  current_power_kw: number;                  // 현재 전력 (kW)
  peak_power_kw?: number;                    // 피크 전력 (kW)

  // Energy
  energy_kwh: number;                        // 에너지 소비 (kWh)
  cooling_energy_kwh?: number;               // 냉방 에너지
  heating_energy_kwh?: number;               // 난방 에너지
  fan_energy_kwh?: number;                   // 팬 에너지
  pump_energy_kwh?: number;                  // 펌프 에너지

  // Efficiency
  cop?: number;                              // 성능계수
  eer?: number;                              // 에너지효율비
  seer?: number;                             // 계절에너지효율

  // Cost
  energy_cost?: number;
  currency?: string;
}

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  system_id: string;
  timestamp: Timestamp;

  // Efficiency
  cop_cooling?: number;                      // 냉방 COP
  cop_heating?: number;                      // 난방 COP
  eer?: number;                              // EER
  seer?: number;                             // SEER
  iplv?: number;                             // IPLV (통합부분부하효율)

  // Load
  cooling_load_kw?: number;                  // 냉방 부하 (kW)
  heating_load_kw?: number;                  // 난방 부하 (kW)
  part_load_ratio?: number;                  // 부분 부하율 (0-1)

  // Temperatures
  supply_temp_c?: number;                    // 공급 온도
  return_temp_c?: number;                    // 환수 온도
  outdoor_temp_c?: number;                   // 외기 온도

  // Pressures (for chillers)
  evaporator_pressure_bar?: number;
  condenser_pressure_bar?: number;
}

// ============================================================================
// Equipment Types
// ============================================================================

/**
 * Split AC unit
 */
export interface SplitACUnit {
  unit_id: string;
  type: 'SPLIT_AC';

  indoor_unit: {
    model: string;
    capacity_btu: number;
    capacity_kw: number;
  };

  outdoor_unit: {
    model: string;
    compressor_type: CompressorType;
    refrigerant: string;
  };

  control: SplitACControl;
  status: EquipmentStatus;
}

/**
 * VRF system
 */
export interface VRFSystem {
  system_id: string;
  type: 'VRF';

  // Outdoor units
  outdoor_units: VRFOutdoorUnit[];

  // Indoor units
  indoor_units: VRFIndoorUnit[];

  // Features
  features: {
    simultaneous_heating_cooling: boolean;
    heat_recovery: boolean;
    individual_zone_control: boolean;
  };

  status: SystemStatus;
}

/**
 * VRF outdoor unit
 */
export interface VRFOutdoorUnit {
  unit_id: string;
  model: string;
  capacity_kw: number;
  compressor_type: CompressorType;
  refrigerant: string;
  inverter_controlled: boolean;
  status: EquipmentStatus;
}

/**
 * VRF indoor unit
 */
export interface VRFIndoorUnit {
  unit_id: string;
  model: string;
  type: IndoorUnitType;
  zone_id: string;
  capacity_kw: number;
  control: VRFIndoorControl;
  status: EquipmentStatus;
}

/**
 * Indoor unit type
 */
export enum IndoorUnitType {
  WALL_MOUNTED = 'WALL_MOUNTED',             // 벽걸이
  CEILING_CASSETTE = 'CEILING_CASSETTE',     // 천장형 카세트
  CEILING_CONCEALED = 'CEILING_CONCEALED',   // 천장형 덕트
  FLOOR_STANDING = 'FLOOR_STANDING',         // 바닥 설치형
  DUCTED = 'DUCTED',                         // 덕트형
}

/**
 * Chiller system
 */
export interface ChillerSystem {
  system_id: string;
  type: 'CHILLER';

  // Chiller
  chiller: {
    chiller_id: string;
    type: ChillerType;
    compressor_type: CompressorType;
    capacity_ton: number;
    capacity_kw: number;
    refrigerant: string;
    cop_rated: number;
    iplv: number;
  };

  // Cooling tower
  cooling_tower?: {
    tower_id: string;
    type: CoolingTowerType;
    capacity_ton: number;
    fan_control: FanControlType;
  };

  // Pumps
  pumps: Pump[];

  // Distribution
  distribution: {
    primary_loop: WaterLoop;
    secondary_loop?: WaterLoop;
  };

  status: SystemStatus;
}

/**
 * Chiller type
 */
export enum ChillerType {
  AIR_COOLED = 'AIR_COOLED',                 // 공랭식
  WATER_COOLED = 'WATER_COOLED',             // 수냉식
  EVAPORATIVE_COOLED = 'EVAPORATIVE_COOLED', // 증발식
}

/**
 * Cooling tower type
 */
export enum CoolingTowerType {
  OPEN_CIRCUIT = 'OPEN_CIRCUIT',             // 개방형
  CLOSED_CIRCUIT = 'CLOSED_CIRCUIT',         // 밀폐형
  HYBRID = 'HYBRID',                         // 하이브리드
}

/**
 * Compressor type
 */
export enum CompressorType {
  RECIPROCATING = 'RECIPROCATING',           // 왕복동
  SCROLL = 'SCROLL',                         // 스크롤
  SCREW = 'SCREW',                           // 스크류
  CENTRIFUGAL = 'CENTRIFUGAL',               // 원심
  ROTARY = 'ROTARY',                         // 로터리
  INVERTER_SCROLL = 'INVERTER_SCROLL',       // 인버터 스크롤
  INVERTER_ROTARY = 'INVERTER_ROTARY',       // 인버터 로터리
}

/**
 * Pump
 */
export interface Pump {
  pump_id: string;
  type: PumpType;
  flow_rate_lpm: number;                     // 유량 (L/min)
  head_m: number;                            // 양정 (m)
  motor_power_kw: number;
  vfd_enabled: boolean;                      // 인버터 제어
  vfd_speed_percent?: number;
  status: EquipmentStatus;
}

/**
 * Pump type
 */
export enum PumpType {
  CHILLED_WATER = 'CHILLED_WATER',           // 냉수 펌프
  CONDENSER_WATER = 'CONDENSER_WATER',       // 냉각수 펌프
  HOT_WATER = 'HOT_WATER',                   // 온수 펌프
  PRIMARY = 'PRIMARY',                       // 1차 펌프
  SECONDARY = 'SECONDARY',                   // 2차 펌프
}

/**
 * Water loop
 */
export interface WaterLoop {
  supply_temp_c: number;                     // 공급 온도 (°C)
  return_temp_c: number;                     // 환수 온도 (°C)
  delta_t: number;                           // 온도차 (°C)
  flow_rate_lpm?: number;                    // 유량 (L/min)
  pressure_bar?: number;                     // 압력 (bar)
}

/**
 * AHU (Air Handling Unit)
 */
export interface AHU {
  ahu_id: string;
  type: 'AHU';
  capacity_cmh: number;                      // 풍량 (m³/h)
  capacity_cfm: number;                      // 풍량 (CFM)

  // Components
  components: {
    filters: FilterComponent[];
    cooling_coil?: Coil;
    heating_coil?: Coil;
    humidifier?: Humidifier;
    supply_fan: Fan;
    return_fan?: Fan;
  };

  // Control
  control: AHUControl;

  status: EquipmentStatus;
}

/**
 * Filter component
 */
export interface FilterComponent {
  filter_id: string;
  filter_type: string;                       // G4, F7, H13 등
  efficiency_percent: number;
  pressure_drop_pa: number;
}

/**
 * Coil
 */
export interface Coil {
  coil_id: string;
  type: CoilType;
  rows: number;
  capacity_kw: number;
  valve_type?: ValveType;
  valve_position_percent?: number;
}

/**
 * Coil type
 */
export enum CoilType {
  CHILLED_WATER = 'CHILLED_WATER',           // 냉수 코일
  HOT_WATER = 'HOT_WATER',                   // 온수 코일
  STEAM = 'STEAM',                           // 증기 코일
  DX = 'DX',                                 // 직팽 코일
}

/**
 * Valve type
 */
export enum ValveType {
  TWO_WAY = 'TWO_WAY',                       // 2방 밸브
  THREE_WAY = 'THREE_WAY',                   // 3방 밸브
  MODULATING = 'MODULATING',                 // 비례제어 밸브
  ON_OFF = 'ON_OFF',                         // ON/OFF 밸브
}

/**
 * Humidifier
 */
export interface Humidifier {
  humidifier_id: string;
  type: HumidifierType;
  capacity_kg_h: number;                     // 가습량 (kg/h)
  modulation_percent?: number;
}

/**
 * Humidifier type
 */
export enum HumidifierType {
  STEAM = 'STEAM',                           // 증기식
  ULTRASONIC = 'ULTRASONIC',                 // 초음파식
  EVAPORATIVE = 'EVAPORATIVE',               // 증발식
  ATOMIZING = 'ATOMIZING',                   // 분무식
}

/**
 * Fan
 */
export interface Fan {
  fan_id: string;
  type: FanType;
  motor_power_kw: number;
  vfd_enabled: boolean;
  speed_percent?: number;
  airflow_cmh?: number;
}

/**
 * Fan type
 */
export enum FanType {
  CENTRIFUGAL = 'CENTRIFUGAL',               // 원심팬
  AXIAL = 'AXIAL',                           // 축류팬
  MIXED_FLOW = 'MIXED_FLOW',                 // 사류팬
}

/**
 * Fan control type
 */
export enum FanControlType {
  CONSTANT_SPEED = 'CONSTANT_SPEED',         // 정속
  VFD = 'VFD',                               // 인버터
  TWO_SPEED = 'TWO_SPEED',                   // 2단 속도
  EC = 'EC',                                 // EC 모터
}

// ============================================================================
// Control Interfaces
// ============================================================================

/**
 * Split AC control
 */
export interface SplitACControl {
  power: boolean;
  mode: OperatingMode;
  temperature_setpoint_c: number;
  fan_speed: FanSpeed;
  swing_vertical: boolean;
  swing_horizontal: boolean;
}

/**
 * VRF indoor control
 */
export interface VRFIndoorControl {
  power: boolean;
  mode: OperatingMode;
  temperature_setpoint_c: number;
  fan_speed: FanSpeed;
  louver_position?: number;                  // 0-100%
}

/**
 * AHU control
 */
export interface AHUControl {
  control_type: AHUControlType;
  supply_temp_setpoint_c: number;
  supply_temp_reset_enabled: boolean;

  // Ventilation
  outdoor_air_percent: number;
  min_outdoor_air_percent: number;
  economizer_enabled: boolean;

  // VAV/CAV
  vav_enabled: boolean;
  static_pressure_setpoint_pa?: number;
}

/**
 * AHU control type
 */
export enum AHUControlType {
  CAV = 'CAV',                               // 정풍량
  VAV = 'VAV',                               // 가변풍량
  DUAL_DUCT = 'DUAL_DUCT',                   // 이중덕트
}

/**
 * Equipment status
 */
export interface EquipmentStatus {
  status: SystemStatus;
  run_hours: number;                         // 가동 시간 (h)
  start_count: number;                       // 시작 횟수
  last_maintenance?: Timestamp;
  next_maintenance?: Timestamp;
  alarms: Alarm[];
}

/**
 * Alarm
 */
export interface Alarm {
  alarm_id: string;
  severity: AlarmSeverity;
  type: AlarmType;
  message: string;
  timestamp: Timestamp;
  acknowledged: boolean;
  acknowledged_by?: string;
  acknowledged_at?: Timestamp;
  cleared: boolean;
  cleared_at?: Timestamp;
}

/**
 * Alarm severity
 */
export enum AlarmSeverity {
  INFO = 'INFO',
  WARNING = 'WARNING',
  CRITICAL = 'CRITICAL',
  EMERGENCY = 'EMERGENCY',
}

/**
 * Alarm type
 */
export enum AlarmType {
  HIGH_TEMP = 'HIGH_TEMP',
  LOW_TEMP = 'LOW_TEMP',
  HIGH_PRESSURE = 'HIGH_PRESSURE',
  LOW_PRESSURE = 'LOW_PRESSURE',
  FILTER_DIRTY = 'FILTER_DIRTY',
  COMPRESSOR_FAULT = 'COMPRESSOR_FAULT',
  FAN_FAULT = 'FAN_FAULT',
  COMMUNICATION_LOST = 'COMMUNICATION_LOST',
  FREEZE_PROTECTION = 'FREEZE_PROTECTION',
  MAINTENANCE_DUE = 'MAINTENANCE_DUE',
}

// ============================================================================
// Protocols Integration
// ============================================================================

/**
 * BACnet object
 */
export interface BACnetObject {
  object_type: BACnetObjectType;
  object_instance: number;
  object_name: string;
  description?: string;
  present_value?: any;
  units?: string;
  writable: boolean;
}

/**
 * BACnet object type
 */
export enum BACnetObjectType {
  ANALOG_INPUT = 'ANALOG_INPUT',
  ANALOG_OUTPUT = 'ANALOG_OUTPUT',
  ANALOG_VALUE = 'ANALOG_VALUE',
  BINARY_INPUT = 'BINARY_INPUT',
  BINARY_OUTPUT = 'BINARY_OUTPUT',
  BINARY_VALUE = 'BINARY_VALUE',
  MULTI_STATE_INPUT = 'MULTI_STATE_INPUT',
  MULTI_STATE_OUTPUT = 'MULTI_STATE_OUTPUT',
  MULTI_STATE_VALUE = 'MULTI_STATE_VALUE',
}

/**
 * Modbus register
 */
export interface ModbusRegister {
  address: number;
  name: string;
  type: ModbusRegisterType;
  data_type: ModbusDataType;
  unit?: string;
  scale?: number;
  writable: boolean;
}

/**
 * Modbus register type
 */
export enum ModbusRegisterType {
  COIL = 'COIL',                             // 코일 (00001-09999)
  DISCRETE_INPUT = 'DISCRETE_INPUT',         // 이산 입력 (10001-19999)
  INPUT_REGISTER = 'INPUT_REGISTER',         // 입력 레지스터 (30001-39999)
  HOLDING_REGISTER = 'HOLDING_REGISTER',     // 홀딩 레지스터 (40001-49999)
}

/**
 * Modbus data type
 */
export enum ModbusDataType {
  BOOL = 'BOOL',
  INT16 = 'INT16',
  UINT16 = 'UINT16',
  INT32 = 'INT32',
  UINT32 = 'UINT32',
  FLOAT32 = 'FLOAT32',
}

// ============================================================================
// Predictive Maintenance
// ============================================================================

/**
 * Condition monitoring
 */
export interface ConditionMonitoring {
  equipment_id: string;
  timestamp: Timestamp;

  // Vibration
  vibration_mm_s?: number;                   // 진동 (mm/s)
  vibration_threshold_mm_s?: number;

  // Temperature
  bearing_temp_c?: number;                   // 베어링 온도 (°C)
  bearing_temp_threshold_c?: number;

  winding_temp_c?: number;                   // 권선 온도 (°C)
  winding_temp_threshold_c?: number;

  // Pressure
  suction_pressure_bar?: number;             // 흡입 압력 (bar)
  discharge_pressure_bar?: number;           // 토출 압력 (bar)

  // Current
  motor_current_a?: number;                  // 전류 (A)
  motor_current_baseline_a?: number;

  // Oil
  oil_level_percent?: number;                // 오일 레벨 (%)
  oil_temp_c?: number;                       // 오일 온도 (°C)

  // Status
  condition: EquipmentCondition;
}

/**
 * Equipment condition
 */
export enum EquipmentCondition {
  EXCELLENT = 'EXCELLENT',
  GOOD = 'GOOD',
  FAIR = 'FAIR',
  POOR = 'POOR',
  CRITICAL = 'CRITICAL',
}

/**
 * Failure prediction
 */
export interface FailurePrediction {
  equipment_id: string;
  component: string;
  failure_mode: string;

  // Prediction
  probability_percent: number;               // 고장 확률 (%)
  confidence_percent: number;                // 예측 신뢰도 (%)
  time_to_failure_days?: number;             // 고장 예상 일수

  // Indicators
  indicators: string[];

  // Recommendation
  recommended_action: string;
  urgency: MaintenanceUrgency;

  timestamp: Timestamp;
}

/**
 * Maintenance urgency
 */
export enum MaintenanceUrgency {
  IMMEDIATE = 'IMMEDIATE',                   // 즉시
  URGENT = 'URGENT',                         // 긴급 (1주 이내)
  SOON = 'SOON',                             // 조속히 (1개월 이내)
  PLANNED = 'PLANNED',                       // 계획 (3개월 이내)
  ROUTINE = 'ROUTINE',                       // 정기
}

/**
 * Maintenance record
 */
export interface MaintenanceRecord {
  record_id: string;
  equipment_id: string;
  type: MaintenanceType;

  // Schedule
  scheduled_date?: Timestamp;
  performed_date: Timestamp;

  // Details
  description: string;
  tasks: MaintenanceTask[];
  parts_replaced?: PartReplacement[];

  // Personnel
  performed_by: string;
  technician_id?: string;

  // Cost
  labor_hours?: number;
  labor_cost?: number;
  parts_cost?: number;
  total_cost?: number;

  // Notes
  notes?: string;
  photos?: string[];
}

/**
 * Maintenance type
 */
export enum MaintenanceType {
  PREVENTIVE = 'PREVENTIVE',                 // 예방 정비
  PREDICTIVE = 'PREDICTIVE',                 // 예지 정비
  CORRECTIVE = 'CORRECTIVE',                 // 수리 정비
  EMERGENCY = 'EMERGENCY',                   // 긴급 정비
}

/**
 * Maintenance task
 */
export interface MaintenanceTask {
  task: string;
  completed: boolean;
  notes?: string;
}

/**
 * Part replacement
 */
export interface PartReplacement {
  part_name: string;
  part_number?: string;
  quantity: number;
  cost?: number;
}

// ============================================================================
// HVAC System
// ============================================================================

/**
 * Complete HVAC system
 */
export interface HVACSystem {
  // Identification
  system_id: string;
  name: string;
  system_type: SystemType;

  // Location
  building: string;
  location: Location;

  // Capacity
  total_cooling_capacity_kw: number;
  total_heating_capacity_kw: number;

  // Equipment
  equipment: Equipment[];

  // Zones
  zones: Zone[];

  // Status
  status: SystemStatus;

  // Performance
  performance?: PerformanceMetrics;

  // Energy
  energy?: EnergyMetrics;

  // Maintenance
  maintenance?: MaintenanceRecord[];

  // Installation
  installation_date?: Timestamp;
  warranty_expiry?: Timestamp;

  // Metadata
  manufacturer?: string;
  model?: string;
  serial_number?: string;
}

/**
 * Equipment union type
 */
export type Equipment =
  | SplitACUnit
  | VRFSystem
  | ChillerSystem
  | AHU;

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * System registration request
 */
export interface SystemRegistrationRequest {
  system: HVACSystem;
}

/**
 * System registration response
 */
export interface SystemRegistrationResponse {
  system_id: string;
  registration_date: Timestamp;
  dashboard_url: string;
  api_access_token?: string;
}

/**
 * Temperature control request
 */
export interface TemperatureControlRequest {
  zone_id: string;
  temperature_c: number;
  mode?: OperatingMode;
  fan_speed?: FanSpeed;
}

/**
 * Status query response
 */
export interface StatusQueryResponse {
  system_id: string;
  timestamp: Timestamp;
  overall_status: SystemStatus;
  zones: ZoneStatus[];
  energy: EnergyMetrics;
  alarms?: Alarm[];
}

/**
 * Zone status
 */
export interface ZoneStatus {
  zone_id: string;
  name: string;
  temperature_c: number;
  setpoint_c: number;
  humidity_percent?: number;
  mode: OperatingMode;
  comfort_level: ComfortLevel;
  air_quality?: AirQualityParameters;
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
