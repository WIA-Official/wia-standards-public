/**
 * WIA Smart Building Standard - TypeScript Type Definitions
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
  latitude: number;
  longitude: number;
}

/**
 * Building location
 */
export interface BuildingLocation {
  address: string;
  city: string;
  state: string;
  country: string;
  postalCode: string;
  coordinates: Coordinates;
  timezone: string;
}

// ============================================================================
// Building Zone and Space Types
// ============================================================================

/**
 * Zone type classification
 */
export enum ZoneType {
  OFFICE = 'office',
  CONFERENCE_ROOM = 'conference-room',
  LOBBY = 'lobby',
  CORRIDOR = 'corridor',
  RESTROOM = 'restroom',
  COMMON_AREA = 'common-area',
  SERVER_ROOM = 'server-room',
  PARKING = 'parking',
  STORAGE = 'storage',
  MECHANICAL = 'mechanical',
  RETAIL = 'retail',
  RESTAURANT = 'restaurant',
}

/**
 * Zone information
 */
export interface Zone {
  zoneId: string;
  name: string;
  type: ZoneType;
  building: string;
  floor: number;
  area: number;                          // m²
  volume: number;                        // m³
  maxOccupancy: number;
  currentOccupancy: number;
  hvacZoneId?: string;
  lightingZoneId?: string;
  accessControlZoneId?: string;
}

/**
 * Space information
 */
export interface Space {
  spaceId: string;
  zoneId: string;
  name: string;
  area: number;                          // m²
  sensors: string[];                     // sensor IDs
  devices: string[];                     // device IDs
}

// ============================================================================
// HVAC Control Types
// ============================================================================

/**
 * HVAC operating mode
 */
export enum HVACMode {
  OFF = 'off',
  COOLING = 'cooling',
  HEATING = 'heating',
  AUTO = 'auto',
  VENTILATION = 'ventilation',
  ECO = 'eco',
}

/**
 * HVAC status
 */
export interface HVACStatus {
  zoneId: string;
  mode: HVACMode;
  currentTemperature: number;            // °C
  setpointTemperature: number;           // °C
  currentHumidity: number;               // %
  setpointHumidity?: number;             // %
  fanSpeed: number;                      // % (0-100)
  airQuality: AirQuality;
  timestamp: Timestamp;
}

/**
 * HVAC control command
 */
export interface HVACControlCommand {
  zoneId: string;
  mode: HVACMode;
  setpointTemperature: number;           // °C
  setpointHumidity?: number;             // %
  fanSpeed?: number;                     // % (0-100)
  duration?: number;                     // minutes (0 = indefinite)
}

/**
 * HVAC schedule
 */
export interface HVACSchedule {
  scheduleId: string;
  zoneId: string;
  name: string;
  enabled: boolean;
  schedules: ScheduleEntry[];
}

/**
 * Schedule entry
 */
export interface ScheduleEntry {
  dayOfWeek: number[];                   // 0-6 (Sunday-Saturday)
  startTime: string;                     // HH:MM
  endTime: string;                       // HH:MM
  mode: HVACMode;
  setpointTemperature: number;           // °C
  setpointHumidity?: number;             // %
}

// ============================================================================
// Lighting Control Types
// ============================================================================

/**
 * Lighting zone status
 */
export interface LightingStatus {
  zoneId: string;
  lightingGroupId: string;
  state: 'on' | 'off';
  brightness: number;                    // % (0-100)
  colorTemperature?: number;             // Kelvin
  rgbColor?: {
    red: number;                         // 0-255
    green: number;                       // 0-255
    blue: number;                        // 0-255
  };
  powerConsumption: number;              // W
  timestamp: Timestamp;
}

/**
 * Lighting control command
 */
export interface LightingControlCommand {
  zoneId: string;
  lightingGroupId?: string;              // optional, controls all lights if omitted
  action: 'on' | 'off' | 'dim' | 'color';
  brightness?: number;                   // % (0-100)
  colorTemperature?: number;             // Kelvin
  rgbColor?: {
    red: number;
    green: number;
    blue: number;
  };
  fadeTime?: number;                     // seconds
}

/**
 * Lighting schedule
 */
export interface LightingSchedule {
  scheduleId: string;
  zoneId: string;
  name: string;
  enabled: boolean;
  schedules: LightingScheduleEntry[];
}

/**
 * Lighting schedule entry
 */
export interface LightingScheduleEntry {
  dayOfWeek: number[];
  startTime: string;                     // HH:MM
  endTime: string;                       // HH:MM
  action: 'on' | 'off';
  brightness?: number;                   // %
}

/**
 * Daylight harvesting configuration
 */
export interface DaylightHarvesting {
  zoneId: string;
  enabled: boolean;
  targetIlluminance: number;             // lux
  minimumBrightness: number;             // %
  maximumBrightness: number;             // %
}

// ============================================================================
// Access Control and Security Types
// ============================================================================

/**
 * Access control status
 */
export interface AccessControlStatus {
  zoneId: string;
  accessPointId: string;
  locked: boolean;
  lastAccessTime?: Timestamp;
  lastAccessUser?: string;
  alarmStatus: 'armed' | 'disarmed' | 'triggered';
}

/**
 * Access event
 */
export interface AccessEvent {
  eventId: string;
  accessPointId: string;
  zoneId: string;
  userId?: string;
  userName?: string;
  eventType: 'entry' | 'exit' | 'denied' | 'forced';
  credentialType?: 'card' | 'pin' | 'biometric' | 'mobile';
  timestamp: Timestamp;
  success: boolean;
  denialReason?: string;
}

/**
 * Access control command
 */
export interface AccessControlCommand {
  accessPointId: string;
  action: 'lock' | 'unlock' | 'arm' | 'disarm';
  duration?: number;                     // minutes (0 = indefinite)
  userId?: string;
}

/**
 * Security zone
 */
export interface SecurityZone {
  securityZoneId: string;
  name: string;
  zones: string[];                       // zone IDs
  alarmStatus: 'armed' | 'disarmed' | 'triggered';
  sensors: SecuritySensor[];
}

/**
 * Security sensor
 */
export interface SecuritySensor {
  sensorId: string;
  type: 'motion' | 'door' | 'window' | 'glass-break' | 'smoke' | 'flood';
  status: 'normal' | 'triggered' | 'fault' | 'tampered';
  battery?: number;                      // %
  lastTriggered?: Timestamp;
}

// ============================================================================
// Energy Management Types
// ============================================================================

/**
 * Energy consumption
 */
export interface EnergyConsumption {
  zoneId?: string;                       // optional, building-wide if omitted
  buildingId: string;
  timestamp: Timestamp;
  interval: '1min' | '5min' | '15min' | '1hour' | '1day';

  // Power metrics
  activePower: number;                   // kW
  reactivePower?: number;                // kvar
  apparentPower?: number;                // kVA
  powerFactor?: number;                  // 0-1

  // Energy metrics
  totalEnergy: number;                   // kWh
  hvacEnergy?: number;                   // kWh
  lightingEnergy?: number;               // kWh
  plugLoadEnergy?: number;               // kWh
  otherEnergy?: number;                  // kWh

  // Cost
  energyCost?: number;
  currency?: string;
}

/**
 * Energy optimization strategy
 */
export interface EnergyOptimization {
  buildingId: string;
  enabled: boolean;
  strategy: 'peak-shaving' | 'load-shifting' | 'demand-response' | 'eco-mode';
  targetReduction: number;               // % or kW
  constraints: {
    comfortTemperatureMin: number;       // °C
    comfortTemperatureMax: number;       // °C
    minimumLightingLevel: number;        // %
    criticalZones: string[];             // zone IDs
  };
}

/**
 * Demand response event
 */
export interface DemandResponseEvent {
  eventId: string;
  buildingId: string;
  startTime: Timestamp;
  endTime: Timestamp;
  targetReduction: number;               // kW or %
  actualReduction?: number;              // kW
  participation: 'mandatory' | 'voluntary';
  compensation?: number;
}

// ============================================================================
// Occupancy and Sensor Types
// ============================================================================

/**
 * Occupancy status
 */
export interface OccupancyStatus {
  zoneId: string;
  occupied: boolean;
  occupantCount: number;
  maxOccupancy: number;
  lastMotionDetected?: Timestamp;
  occupancyRate: number;                 // % (0-100)
  timestamp: Timestamp;
}

/**
 * Occupancy sensor
 */
export interface OccupancySensor {
  sensorId: string;
  zoneId: string;
  type: 'pir' | 'ultrasonic' | 'camera' | 'co2' | 'wifi';
  status: 'active' | 'inactive' | 'fault';
  occupied: boolean;
  occupantCount?: number;
  confidence?: number;                   // % (0-100)
  lastUpdate: Timestamp;
}

/**
 * Environmental sensor
 */
export interface EnvironmentalSensor {
  sensorId: string;
  zoneId: string;
  type: 'temperature' | 'humidity' | 'co2' | 'voc' | 'pm' | 'light' | 'noise';
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical' | 'fault';
  lastUpdate: Timestamp;
}

/**
 * Air quality
 */
export interface AirQuality {
  zoneId: string;
  co2: number;                           // ppm
  voc?: number;                          // ppb
  pm25?: number;                         // μg/m³
  pm10?: number;                         // μg/m³
  temperature: number;                   // °C
  humidity: number;                      // %
  airQualityIndex: number;               // 0-500 (AQI)
  qualityLevel: 'excellent' | 'good' | 'moderate' | 'poor' | 'hazardous';
  timestamp: Timestamp;
}

// ============================================================================
// Building Automation Protocol Types (BACnet, Modbus)
// ============================================================================

/**
 * BACnet device
 */
export interface BACnetDevice {
  deviceId: number;
  deviceName: string;
  deviceType: string;
  vendorId: number;
  vendorName: string;
  modelName: string;
  firmwareRevision: string;
  networkAddress: string;
  objects: BACnetObject[];
}

/**
 * BACnet object
 */
export interface BACnetObject {
  objectType: BACnetObjectType;
  objectInstance: number;
  objectName: string;
  presentValue?: any;
  units?: string;
  description?: string;
  writable: boolean;
}

/**
 * BACnet object types
 */
export enum BACnetObjectType {
  ANALOG_INPUT = 'analog-input',
  ANALOG_OUTPUT = 'analog-output',
  ANALOG_VALUE = 'analog-value',
  BINARY_INPUT = 'binary-input',
  BINARY_OUTPUT = 'binary-output',
  BINARY_VALUE = 'binary-value',
  MULTI_STATE_INPUT = 'multi-state-input',
  MULTI_STATE_OUTPUT = 'multi-state-output',
  MULTI_STATE_VALUE = 'multi-state-value',
  DEVICE = 'device',
}

/**
 * Modbus device
 */
export interface ModbusDevice {
  deviceId: string;
  slaveAddress: number;
  deviceType: string;
  networkAddress: string;
  registers: ModbusRegister[];
}

/**
 * Modbus register
 */
export interface ModbusRegister {
  address: number;
  registerType: ModbusRegisterType;
  dataType: ModbusDataType;
  name: string;
  unit?: string;
  scale?: number;
  writable: boolean;
  value?: number;
}

/**
 * Modbus register types
 */
export enum ModbusRegisterType {
  COIL = 'coil',                         // 0xxxx
  DISCRETE_INPUT = 'discrete-input',     // 1xxxx
  INPUT_REGISTER = 'input-register',     // 3xxxx
  HOLDING_REGISTER = 'holding-register', // 4xxxx
}

/**
 * Modbus data types
 */
export enum ModbusDataType {
  BOOL = 'bool',
  INT16 = 'int16',
  UINT16 = 'uint16',
  INT32 = 'int32',
  UINT32 = 'uint32',
  FLOAT32 = 'float32',
}

// ============================================================================
// Alarm and Event Management
// ============================================================================

/**
 * Alarm severity
 */
export enum AlarmSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical',
}

/**
 * Alarm type
 */
export enum AlarmType {
  HVAC_FAULT = 'hvac-fault',
  LIGHTING_FAULT = 'lighting-fault',
  ACCESS_VIOLATION = 'access-violation',
  SECURITY_BREACH = 'security-breach',
  FIRE_ALARM = 'fire-alarm',
  FLOOD_DETECTED = 'flood-detected',
  HIGH_ENERGY = 'high-energy',
  POOR_AIR_QUALITY = 'poor-air-quality',
  EQUIPMENT_FAULT = 'equipment-fault',
  COMMUNICATION_LOST = 'communication-lost',
  SENSOR_FAULT = 'sensor-fault',
}

/**
 * Alarm
 */
export interface Alarm {
  alarmId: string;
  buildingId: string;
  zoneId?: string;
  severity: AlarmSeverity;
  type: AlarmType;
  message: string;
  details?: any;
  timestamp: Timestamp;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
  resolved: boolean;
  resolvedAt?: Timestamp;
}

/**
 * Event log
 */
export interface EventLog {
  eventId: string;
  buildingId: string;
  zoneId?: string;
  eventType: string;
  category: 'hvac' | 'lighting' | 'access' | 'security' | 'energy' | 'system';
  description: string;
  data?: any;
  userId?: string;
  timestamp: Timestamp;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

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

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

// ============================================================================
// Building System Types
// ============================================================================

/**
 * Building information
 */
export interface BuildingInfo {
  buildingId: string;
  name: string;
  location: BuildingLocation;
  totalArea: number;                     // m²
  floors: number;
  zones: Zone[];
  systems: {
    hvac: boolean;
    lighting: boolean;
    accessControl: boolean;
    fireAlarm: boolean;
    energyManagement: boolean;
  };
}

/**
 * Building dashboard
 */
export interface BuildingDashboard {
  buildingId: string;
  timestamp: Timestamp;
  occupancy: {
    current: number;
    maximum: number;
    rate: number;                        // %
  };
  energy: {
    currentPower: number;                // kW
    todayConsumption: number;            // kWh
    todayCost?: number;
  };
  comfort: {
    averageTemperature: number;          // °C
    averageHumidity: number;             // %
    airQualityIndex: number;
  };
  alarms: {
    critical: number;
    warning: number;
    total: number;
  };
  systems: {
    hvac: 'online' | 'offline' | 'degraded';
    lighting: 'online' | 'offline' | 'degraded';
    accessControl: 'online' | 'offline' | 'degraded';
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  BuildingLocation,
};
