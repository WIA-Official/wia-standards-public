/**
 * WIA-CITY-009: Smart Lighting Standard - TypeScript Type Definitions
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
  altitude?: number;
}

/**
 * Location information
 */
export interface Location {
  building?: string;
  floor?: string;
  room?: string;
  zone?: string;
  coordinates?: {
    x: number;
    y: number;
    z?: number;
  };
  gps?: Coordinates;
}

// ============================================================================
// Lighting Fixture Types
// ============================================================================

/**
 * Fixture types
 */
export enum FixtureType {
  LED = 'led',
  OLED = 'oled',
  FLUORESCENT = 'fluorescent',
  HALOGEN = 'halogen',
  INCANDESCENT = 'incandescent',
}

/**
 * Socket types
 */
export type SocketType = 'E26' | 'E27' | 'GU10' | 'B22' | 'MR16' | 'PAR38';

/**
 * Main lighting fixture interface
 */
export interface LightingFixture {
  // Identifiers
  fixtureId: string;
  name: string;
  description?: string;

  // Location
  location: Location;

  // Hardware specifications
  hardware: {
    manufacturer: string;
    model: string;
    type: FixtureType;
    socketType?: SocketType;
    wattage: number;                    // W
    lumens: number;                     // lm
    efficacy: number;                   // lm/W
    colorTemperature: number | { min: number; max: number }; // K
    cri: number;                        // 0-100
    beamAngle?: number;                 // degrees
    ipRating?: string;                  // IP65, etc.
    dimming: boolean;
    tunableWhite: boolean;
    rgbColor: boolean;
  };

  // Communication and control
  communication: {
    protocols: ControlProtocol[];
    address?: {
      dali?: number;                    // 0-63
      dmx?: number;                     // 1-512
      zigbee?: string;                  // IEEE address
      ble?: string;                     // UUID
      ip?: string;                      // IP address
    };
    gateway?: string;                   // Gateway ID
  };

  // Current state
  state: FixtureState;

  // Groups and scenes
  groups: string[];                     // Group IDs
  scenes: string[];                     // Scene IDs

  // Installation and maintenance
  installation: {
    installDate: Timestamp;
    warrantyUntil?: Timestamp;
    lastMaintenance?: Timestamp;
    nextMaintenance?: Timestamp;
  };

  // Metadata
  metadata?: {
    tags?: string[];
    customFields?: Record<string, any>;
  };
}

/**
 * Fixture state
 */
export interface FixtureState {
  on: boolean;
  reachable: boolean;                   // Communication available
  brightness: number;                   // % (0-100)
  colorTemperature?: number;            // K
  color?: {
    hue: number;                        // 0-360
    saturation: number;                 // 0-100
  };
  power?: number;                       // W (current power)
  energy?: number;                      // kWh (cumulative)
  lastUpdate: Timestamp;
}

/**
 * Smart bulb
 */
export interface SmartBulb extends LightingFixture {
  capabilities: {
    dimming: boolean;
    tunableWhite: boolean;
    rgbColor: boolean;
    powerMeasurement: boolean;
    firmware_ota: boolean;              // OTA firmware update
  };
}

// ============================================================================
// Control Protocols
// ============================================================================

/**
 * Control protocol types
 */
export type ControlProtocol = 'dali' | 'dmx' | 'zigbee' | 'ble' | 'wifi' | '0-10v' | 'pwm';

/**
 * DALI device
 */
export interface DALIDevice {
  daliAddress: number;                  // 0-63
  groups: number[];                     // 0-15
  scenes: DALIScene[];                  // Max 16

  capabilities: {
    dimming: boolean;
    colorControl: boolean;
    powerMeasurement: boolean;
    deviceType: number;                 // DALI Device Type
  };

  status: {
    lampFailure: boolean;
    lampOn: boolean;
    limitError: boolean;
    fadeRunning: boolean;
    actualLevel: number;                // 0-254
    powerOn: boolean;
  };
}

/**
 * DALI scene
 */
export interface DALIScene {
  sceneNumber: number;                  // 0-15
  level: number;                        // 0-254
  fadeTime: number;                     // ms
}

/**
 * DMX universe
 */
export interface DMXUniverse {
  universeId: number;
  channels: number[];                   // 512 channels (0-255 each)
  updateRate: number;                   // Hz
  fixtures: DMXFixture[];
}

/**
 * DMX fixture
 */
export interface DMXFixture {
  fixtureId: string;
  startChannel: number;                 // 1-512
  channelCount: number;
  channelMap: Record<number, DMXChannelFunction>;
}

/**
 * DMX channel functions
 */
export type DMXChannelFunction =
  | 'red'
  | 'green'
  | 'blue'
  | 'white'
  | 'dimmer'
  | 'strobe'
  | 'pan'
  | 'tilt'
  | 'mode';

/**
 * Zigbee light
 */
export interface ZigbeeLight {
  ieeeAddress: string;                  // 64-bit MAC address
  networkAddress: number;               // 16-bit network address
  endpoint: number;

  clusters: {
    onOff: boolean;
    levelControl: boolean;
    colorControl: boolean;
    scenes: boolean;
    groups: boolean;
  };

  state: {
    on: boolean;
    brightness: number;                 // 0-254
    colorMode: 'hs' | 'xy' | 'ct';
    hue?: number;                       // 0-254
    saturation?: number;                // 0-254
    colorTemp?: number;                 // mireds (1,000,000/K)
    x?: number;                         // CIE x (0-1)
    y?: number;                         // CIE y (0-1)
  };
}

/**
 * BLE Mesh light
 */
export interface BLEMeshLight {
  uuid: string;                         // 128-bit UUID
  unicastAddress: number;               // 16-bit unicast address

  elements: {
    elementIndex: number;
    models: BLEMeshModel[];
  }[];

  subscriptions: number[];              // Group addresses
  publications: {
    address: number;
    period: number;                     // ms
  };
}

/**
 * BLE Mesh models
 */
export type BLEMeshModel =
  | 'generic_onoff'
  | 'generic_level'
  | 'light_lightness'
  | 'light_ctl';

// ============================================================================
// Dimming and Color Control
// ============================================================================

/**
 * Dimming curves
 */
export enum DimmingCurve {
  LINEAR = 'linear',
  LOGARITHMIC = 'logarithmic',
  S_CURVE = 's_curve',
  CUSTOM = 'custom',
}

/**
 * Dimming profile
 */
export interface DimmingProfile {
  curve: DimmingCurve;
  minLevel: number;                     // % (minimum dimming level)
  maxLevel: number;                     // % (maximum dimming level)
  fadeTime: number;                     // ms
  customCurve?: number[];               // Custom curve (0-100, 256 points)
}

/**
 * PWM dimming
 */
export interface PWMDimming {
  frequency: number;                    // Hz (1000-50000 recommended)
  dutyCycle: number;                    // % (0-100)
  resolution: number;                   // bits (8-16)
  brightness: number;                   // % (0-100)
}

/**
 * Tunable white LED
 */
export interface TunableWhiteLED {
  minColorTemperature: number;          // K (e.g., 2700)
  maxColorTemperature: number;          // K (e.g., 6500)
  currentCCT: number;                   // K
  warmLEDPower: number;                 // % (0-100)
  coolLEDPower: number;                 // % (0-100)
  transitionTime: number;               // ms
}

/**
 * Tunable white control
 */
export interface TunableWhiteControl {
  targetCCT: number;                    // K
  warmLEDChannel: number;               // 0-255 (2700K LED)
  coolLEDChannel: number;               // 0-255 (6500K LED)
  brightness: number;                   // % (0-100)
  transitionTime: number;               // ms
}

// ============================================================================
// Circadian Lighting
// ============================================================================

/**
 * Circadian lighting schedule
 */
export interface CircadianLightingSchedule {
  enabled: boolean;
  latitude: number;
  longitude: number;
  timezone: string;

  // Dynamic adjustment
  sunriseSyncEnabled: boolean;
  sunsetSyncEnabled: boolean;

  // User customization
  wakeTime: string;                     // HH:MM
  sleepTime: string;                    // HH:MM

  // Lighting profiles
  profiles: {
    morning: LightingProfile;
    day: LightingProfile;
    evening: LightingProfile;
    night: LightingProfile;
  };
}

/**
 * Lighting profile
 */
export interface LightingProfile {
  brightness: number;                   // %
  colorTemperature: number;             // K
  eml: number;                          // Equivalent Melanopic Lux
  transitionDuration: number;           // minutes
}

/**
 * Circadian phase
 */
export enum CircadianPhase {
  WAKE = 'wake',
  MORNING = 'morning',
  DAY = 'day',
  EVENING = 'evening',
  NIGHT = 'night',
  SLEEP = 'sleep',
}

/**
 * Circadian CCT schedule
 */
export interface CircadianCCTSchedule {
  timePoints: {
    time: string;                       // HH:MM
    cct: number;                        // K
    brightness: number;                 // %
  }[];
}

/**
 * Circadian health metrics
 */
export interface CircadianHealthMetrics {
  userId: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };

  exposure: {
    averageDayEML: number;              // lux
    averageNightEML: number;            // lux
    blueExposureDuration: number;       // minutes
  };

  subjective?: {
    sleepQuality: number;               // 1-10
    daytimeAlertness: number;           // 1-10
    mood: number;                       // 1-10
    eyeStrain: number;                  // 1-10
  };

  recommendations: string[];
}

// ============================================================================
// Sensors
// ============================================================================

/**
 * Sensor types
 */
export type SensorType =
  | 'occupancy'
  | 'light'
  | 'temperature'
  | 'humidity'
  | 'co2';

/**
 * Generic sensor
 */
export interface Sensor {
  sensorId: string;
  name: string;
  type: SensorType;
  location: Location;

  specifications: {
    manufacturer: string;
    model: string;
    range?: { min: number; max: number };
    accuracy?: number;                  // %
    resolution?: number;
    samplingRate?: number;              // Hz
  };

  communication: {
    protocol: 'zigbee' | 'ble' | 'wifi' | 'wired';
    address?: string;
    gateway?: string;
  };

  state: SensorState;

  configuration: {
    updateInterval: number;             // seconds
    threshold?: number;
    calibrationDate?: Timestamp;
  };
}

/**
 * Sensor state
 */
export interface SensorState {
  reachable: boolean;
  batteryLevel?: number;                // %
  lastUpdate: Timestamp;

  // Sensor-specific readings
  occupancy?: boolean;
  illuminance?: number;                 // lux
  temperature?: number;                 // °C
  humidity?: number;                    // %
  co2?: number;                         // ppm
}

/**
 * PIR sensor
 */
export interface PIRSensor extends Sensor {
  specifications: Sensor['specifications'] & {
    detectionRange: number;             // m
    detectionAngle: number;             // degrees
    sensitivity: number;                // 1-10
    timeDelay: number;                  // seconds
  };

  state: SensorState & {
    occupied: boolean;
    lastMotionDetected: Timestamp;
    temperature?: number;               // °C
  };
}

/**
 * Light sensor
 */
export interface LightSensor extends Sensor {
  specifications: Sensor['specifications'] & {
    spectralResponse: 'human_eye' | 'broad_spectrum';
  };

  state: SensorState & {
    illuminance: number;                // lux
  };
}

/**
 * Vision occupancy sensor (camera-based)
 */
export interface VisionOccupancySensor extends Sensor {
  cameraId: string;

  capabilities: {
    peopleCount: boolean;
    positionTracking: boolean;
    activityRecognition: boolean;
    anonymization: boolean;             // Privacy protection
  };

  state: SensorState & {
    occupantCount: number;
    positions?: { x: number; y: number }[];
    activityLevel: 'low' | 'medium' | 'high';
  };
}

/**
 * Multi-sensor zone
 */
export interface MultiSensorZone {
  zoneId: string;
  zoneName: string;
  area: number;                         // m²

  sensors: {
    occupancy: PIRSensor[];
    light: LightSensor[];
    temperature?: Sensor[];
  };

  aggregatedState: {
    occupied: boolean;
    occupantCount?: number;
    averageIlluminance: number;         // lux
    temperature?: number;               // °C
    confidence: number;                 // 0-1
  };

  control: {
    targetIlluminance: number;          // lux
    occupiedBrightness: number;         // %
    vacantBrightness: number;           // %
    timeDelay: number;                  // seconds
  };
}

// ============================================================================
// Daylight Harvesting
// ============================================================================

/**
 * Daylight harvesting control
 */
export interface DaylightHarvestingControl {
  zoneId: string;

  sensors: {
    outdoor: LightSensor;
    indoor: LightSensor[];
  };

  target: {
    illuminance: number;                // lux
    uniformity: number;                 // 0-1
  };

  control: {
    mode: 'open_loop' | 'closed_loop' | 'hybrid';
    algorithm: 'proportional' | 'pid' | 'fuzzy' | 'ml';
    updateInterval: number;             // seconds

    pid?: {
      kp: number;
      ki: number;
      kd: number;
    };
  };

  state: {
    outdoorIlluminance: number;         // lux
    indoorIlluminance: number;          // lux
    artificialContribution: number;     // %
    daylightContribution: number;       // %
    energySaved: number;                // kWh (cumulative)
  };
}

/**
 * Daylight savings report
 */
export interface DaylightSavingsReport {
  period: {
    start: Timestamp;
    end: Timestamp;
  };

  baseline: {
    totalEnergyConsumption: number;     // kWh
    averageDailyConsumption: number;    // kWh/day
  };

  withDaylight: {
    totalEnergyConsumption: number;     // kWh
    averageDailyConsumption: number;    // kWh/day
    daylightContribution: number;       // %
  };

  savings: {
    energySaved: number;                // kWh
    percentageSaved: number;            // %
    costSaved: number;                  // currency
    co2Reduced: number;                 // kg CO2
  };
}

// ============================================================================
// Energy Management
// ============================================================================

/**
 * Power measurement
 */
export interface PowerMeasurement {
  fixtureId: string;
  timestamp: Timestamp;

  instantaneous: {
    voltage: number;                    // V
    current: number;                    // A
    power: number;                      // W (real power)
    apparentPower: number;              // VA
    powerFactor: number;                // 0-1
  };

  cumulative: {
    energyConsumed: number;             // kWh
    operatingHours: number;             // hours
  };
}

/**
 * Power budget
 */
export interface PowerBudget {
  zoneId: string;
  period: 'daily' | 'weekly' | 'monthly';

  budget: {
    target: number;                     // kWh
    current: number;                    // kWh
    remaining: number;                  // kWh
    percentUsed: number;                // %
  };

  forecast: {
    expectedConsumption: number;        // kWh
    exceedanceRisk: number;             // %
  };

  actions: {
    dimmingLevel: number;               // %
    alertsEnabled: boolean;
    autoAdjustEnabled: boolean;
  };
}

/**
 * Task tuning zone
 */
export interface TaskTuningZone {
  zoneId: string;

  areas: {
    task: {
      illuminance: number;              // lux
      fixtures: string[];
    };
    ambient: {
      illuminance: number;              // lux
      fixtures: string[];
    };
  };

  taskDetection: {
    enabled: boolean;
    method: 'occupancy' | 'vision' | 'manual';
    adaptiveAdjustment: boolean;
  };
}

/**
 * Personal lighting control
 */
export interface PersonalLightingControl {
  userId: string;
  fixtureId: string;

  preferences: {
    defaultBrightness: number;          // %
    defaultCCT: number;                 // K
    autoAdjust: boolean;
  };

  overrides: {
    manualOverride: boolean;
    overrideUntil?: Timestamp;
  };

  usage: {
    averageBrightness: number;          // %
    energySaved: number;                // kWh
  };
}

/**
 * Energy dashboard
 */
export interface EnergyDashboard {
  overview: {
    totalPower: number;                 // W
    totalEnergy: number;                // kWh
    averagePowerFactor: number;
    peakDemand: number;                 // W
    peakDemandTime: Timestamp;
  };

  byZone: {
    zoneId: string;
    zoneName: string;
    power: number;                      // W
    energy: number;                     // kWh
    efficiency: number;                 // lm/W
  }[];

  trends: {
    hourly: { hour: number; energy: number }[];
    daily: { date: string; energy: number }[];
    monthly: { month: string; energy: number }[];
  };

  savings: {
    compared_to_baseline: number;      // %
    daylight_harvesting: number;       // kWh
    occupancy_control: number;         // kWh
    dimming: number;                   // kWh
    total: number;                     // kWh
  };
}

/**
 * Energy anomaly detection
 */
export interface EnergyAnomalyDetection {
  fixtureId: string;

  baseline: {
    averagePower: number;               // W
    stdDeviation: number;               // W
  };

  anomalies: {
    timestamp: Timestamp;
    measuredPower: number;              // W
    expectedPower: number;              // W
    deviation: number;                  // %
    severity: 'low' | 'medium' | 'high';
    possibleCauses: string[];
  }[];

  alerts: {
    enabled: boolean;
    threshold: number;                  // %
    notification: 'email' | 'sms' | 'push';
  };
}

// ============================================================================
// Scheduling and Scene Management
// ============================================================================

/**
 * Schedule action
 */
export interface ScheduleAction {
  type: 'on' | 'off' | 'dim' | 'scene' | 'auto';
  brightness?: number;                  // %
  sceneId?: string;
  fadeTime?: number;                    // ms
}

/**
 * Time-based schedule
 */
export interface TimeBasedSchedule {
  scheduleId: string;
  name: string;
  enabled: boolean;

  timeSlots: {
    daysOfWeek: number[];               // 0=Sunday, 1=Monday, ..., 6=Saturday
    startTime: string;                  // HH:MM
    endTime: string;                    // HH:MM
    action: ScheduleAction;
  }[];

  exceptions: {
    date: string;                       // YYYY-MM-DD
    action: ScheduleAction | 'skip';
  }[];
}

/**
 * Astronomical schedule
 */
export interface AstronomicalSchedule {
  scheduleId: string;
  location: {
    latitude: number;
    longitude: number;
    timezone: string;
  };

  events: {
    event: 'sunrise' | 'sunset' | 'dawn' | 'dusk' | 'solar_noon';
    offset: number;                     // minutes
    action: ScheduleAction;
  }[];

  todayEvents: {
    sunrise: Timestamp;
    sunset: Timestamp;
    dawn: Timestamp;
    dusk: Timestamp;
    solarNoon: Timestamp;
  };
}

/**
 * Unoccupied dimming schedule
 */
export interface UnoccupiedDimmingSchedule {
  zoneId: string;

  schedule: {
    occupiedHours: {
      start: string;                    // HH:MM
      end: string;                      // HH:MM
      brightness: number;               // %
    };
    unoccupiedHours: {
      brightness: number;               // %
      offDelay: number;                 // minutes
    };
  };

  exceptions: {
    date: string;                       // YYYY-MM-DD
    occupied: boolean;
  }[];
}

/**
 * Lighting scene
 */
export interface LightingScene {
  sceneId: string;
  name: string;
  description?: string;
  icon?: string;

  fixtureStates: {
    fixtureId: string;
    on: boolean;
    brightness?: number;                // %
    colorTemperature?: number;          // K
    color?: {
      hue: number;                      // 0-360
      saturation: number;               // 0-100
    };
  }[];

  transition: {
    duration: number;                   // ms
    curve: 'linear' | 'ease-in' | 'ease-out' | 'ease-in-out';
  };

  metadata: {
    category: 'work' | 'relax' | 'entertainment' | 'dining' | 'custom';
    createdBy?: string;
    createdAt: Timestamp;
    tags?: string[];
  };
}

/**
 * Dynamic scene
 */
export interface DynamicScene extends LightingScene {
  dynamic: {
    enabled: boolean;
    type: 'fade' | 'pulse' | 'color_cycle' | 'random';
    parameters: {
      speed?: number;                   // 1-10
      colorRange?: {
        minHue: number;
        maxHue: number;
      };
      brightnessRange?: {
        min: number;
        max: number;
      };
    };
  };
}

// ============================================================================
// Automation
// ============================================================================

/**
 * Trigger
 */
export interface Trigger {
  type: 'time' | 'sensor' | 'state' | 'manual';

  time?: {
    at: string;                         // HH:MM
    daysOfWeek?: number[];
  };

  sensor?: {
    sensorId: string;
    parameter: 'occupancy' | 'illuminance' | 'temperature';
    operator: '>' | '<' | '==' | '!=' | '>=' | '<=';
    value: number | boolean;
  };

  state?: {
    fixtureId: string;
    parameter: 'on' | 'brightness' | 'cct';
    operator: '>' | '<' | '==' | '!=' | '>=' | '<=';
    value: number | boolean;
  };
}

/**
 * Condition
 */
export interface Condition {
  type: 'time_range' | 'sensor' | 'state';

  timeRange?: {
    start: string;                      // HH:MM
    end: string;                        // HH:MM
  };

  sensor?: {
    sensorId: string;
    parameter: string;
    operator: string;
    value: number | boolean;
  };

  state?: {
    fixtureId: string;
    parameter: string;
    operator: string;
    value: number | boolean;
  };
}

/**
 * Action
 */
export interface Action {
  type: 'fixture' | 'scene' | 'notification' | 'api_call';

  fixture?: {
    fixtureId: string | 'all' | `group:${string}`;
    command: 'on' | 'off' | 'dim' | 'set_cct' | 'set_color';
    parameters?: {
      brightness?: number;
      colorTemperature?: number;
      color?: { hue: number; saturation: number };
      fadeTime?: number;
    };
  };

  scene?: {
    sceneId: string;
  };

  notification?: {
    message: string;
    channels: ('email' | 'sms' | 'push')[];
  };

  apiCall?: {
    url: string;
    method: 'GET' | 'POST' | 'PUT';
    body?: any;
  };
}

/**
 * Automation rule
 */
export interface AutomationRule {
  ruleId: string;
  name: string;
  enabled: boolean;

  trigger: Trigger;
  conditions: Condition[];
  actions: Action[];

  metadata: {
    createdBy?: string;
    createdAt: Timestamp;
    lastTriggered?: Timestamp;
    triggerCount: number;
  };
}

// ============================================================================
// Zone Management
// ============================================================================

/**
 * Zone
 */
export interface Zone {
  zoneId: string;
  name: string;
  description?: string;

  location: {
    building?: string;
    floor?: string;
    area?: number;                      // m²
    capacity?: number;
  };

  fixtures: string[];
  sensors: string[];

  control: {
    mode: 'manual' | 'auto' | 'schedule' | 'sensor';
    schedule?: string;
    automationRules?: string[];

    defaults: {
      occupiedBrightness: number;       // %
      vacantBrightness: number;         // %
      targetIlluminance?: number;       // lux
      colorTemperature?: number;        // K
      fadeTime: number;                 // ms
    };

    sensorControl?: {
      enabled: boolean;
      occupancyBased: boolean;
      daylightHarvesting: boolean;
      timeDelay: number;                // seconds
    };
  };

  energyBudget?: {
    daily: number;                      // kWh
    monthly: number;                    // kWh
  };
}

// ============================================================================
// Events and Alerts
// ============================================================================

/**
 * Lighting event
 */
export interface LightingEvent {
  eventId: string;
  timestamp: Timestamp;
  type: 'state_change' | 'sensor_trigger' | 'schedule_execute' | 'automation_run' | 'alert' | 'maintenance';

  source: {
    type: 'user' | 'sensor' | 'schedule' | 'automation' | 'system';
    id?: string;
    name?: string;
  };

  target: {
    type: 'fixture' | 'zone' | 'scene';
    id: string;
    name?: string;
  };

  details: {
    action?: string;
    previousState?: any;
    newState?: any;
    reason?: string;
  };

  metadata?: {
    userId?: string;
    sessionId?: string;
    ipAddress?: string;
  };
}

/**
 * Lighting alert
 */
export interface LightingAlert {
  alertId: string;
  timestamp: Timestamp;
  severity: 'info' | 'warning' | 'error' | 'critical';
  type: 'fixture_offline' | 'sensor_offline' | 'energy_exceeded' | 'maintenance_due' | 'anomaly_detected';

  source: {
    type: 'fixture' | 'sensor' | 'zone' | 'system';
    id: string;
    name?: string;
  };

  message: string;
  description?: string;

  status: 'active' | 'acknowledged' | 'resolved' | 'closed';
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
  resolvedAt?: Timestamp;

  actions?: {
    label: string;
    action: string;
  }[];
}

// ============================================================================
// API Types
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
// Security
// ============================================================================

/**
 * User role
 */
export enum Role {
  ADMIN = 'admin',
  MANAGER = 'manager',
  USER = 'user',
  VIEWER = 'viewer',
}

/**
 * Permission
 */
export interface Permission {
  resource: 'fixtures' | 'scenes' | 'schedules' | 'zones' | 'sensors' | 'analytics';
  action: 'read' | 'write' | 'delete' | 'control';
  scope?: string;
}

/**
 * Camera privacy settings
 */
export interface CameraPrivacySettings {
  sensorId: string;

  privacyMode: {
    enabled: boolean;
    anonymization: 'blur_faces' | 'blur_all' | 'silhouette' | 'bounding_box';
    dataRetention: number;              // days
    accessLog: boolean;
  };

  consent: {
    signageDisplayed: boolean;
    notificationSent: boolean;
    optOut: boolean;
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
