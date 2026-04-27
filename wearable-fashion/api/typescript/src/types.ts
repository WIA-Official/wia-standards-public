/**
 * WIA-IND-003: Wearable Fashion Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Fashion Technology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Fashion Types
// ============================================================================

/**
 * Wearable fashion garment types
 */
export type GarmentType =
  | 'jacket'
  | 'dress'
  | 'shirt'
  | 'pants'
  | 'skirt'
  | 'hoodie'
  | 'vest'
  | 'gloves'
  | 'hat'
  | 'shoes'
  | 'bag'
  | 'scarf';

/**
 * Smart jewelry types
 */
export type JewelryType =
  | 'ring'
  | 'bracelet'
  | 'necklace'
  | 'earrings'
  | 'anklet'
  | 'brooch'
  | 'watch_band';

/**
 * Wearable category
 */
export type WearableCategory =
  | 'smart_jewelry'
  | 'led_clothing'
  | 'heated_garment'
  | 'interactive_fashion'
  | 'health_monitoring'
  | 'solar_fashion'
  | 'haptic_clothing';

/**
 * Fashion size standards
 */
export type FashionSize = 'XXS' | 'XS' | 'S' | 'M' | 'L' | 'XL' | 'XXL' | 'XXXL';

// ============================================================================
// LED and Display Types
// ============================================================================

/**
 * LED types supported
 */
export type LEDType =
  | 'WS2812B'
  | 'APA102'
  | 'SK6812'
  | 'LPD8806'
  | 'standard_RGB'
  | 'standard_single'
  | 'fiber_optic'
  | 'EL_wire';

/**
 * LED control protocol
 */
export type LEDProtocol =
  | 'single_wire'  // WS2812B
  | 'SPI'          // APA102
  | 'PWM'          // Standard LEDs
  | 'AC_inverter'; // EL wire

/**
 * LED pattern types
 */
export type LEDPattern =
  | 'solid'
  | 'rainbow'
  | 'chase'
  | 'breathing'
  | 'sparkle'
  | 'fire'
  | 'music_reactive'
  | 'touch_reactive'
  | 'motion_reactive'
  | 'temperature_reactive'
  | 'heartbeat'
  | 'custom';

/**
 * LED animation speed
 */
export type AnimationSpeed = 'slow' | 'medium' | 'fast' | 'very_fast';

/**
 * Color scheme
 */
export type ColorScheme = 'warm' | 'cool' | 'monochrome' | 'rainbow' | 'pastel' | 'vivid' | 'custom';

/**
 * LED configuration
 */
export interface LEDConfiguration {
  /** LED type */
  type: LEDType;

  /** Number of LEDs */
  count: number;

  /** Control protocol */
  protocol: LEDProtocol;

  /** Data pin (GPIO) */
  dataPin?: number;

  /** Clock pin for SPI types */
  clockPin?: number;

  /** Voltage (V) */
  voltage: number;

  /** Current per LED (mA) */
  currentPerLED: number;

  /** Maximum brightness (0-255) */
  maxBrightness: number;

  /** Color correction */
  colorCorrection?: {
    red: number;    // 0-255
    green: number;  // 0-255
    blue: number;   // 0-255
  };

  /** Refresh rate (Hz) */
  refreshRate?: number;
}

/**
 * LED state
 */
export interface LEDState {
  /** Currently active */
  isActive: boolean;

  /** Current pattern */
  pattern: LEDPattern;

  /** Brightness (0-100%) */
  brightness: number;

  /** Animation speed */
  speed: AnimationSpeed;

  /** Color scheme */
  colorScheme: ColorScheme;

  /** Primary color (RGB hex) */
  primaryColor?: string;

  /** Secondary color (RGB hex) */
  secondaryColor?: string;

  /** Individual LED states (if tracked) */
  ledStates?: Array<{
    index: number;
    r: number; // 0-255
    g: number; // 0-255
    b: number; // 0-255
  }>;
}

// ============================================================================
// Power System Types
// ============================================================================

/**
 * Battery chemistry types
 */
export type BatteryChemistry =
  | 'LiPo'
  | 'Li_ion'
  | 'flexible_battery'
  | 'coin_cell_CR2032'
  | 'coin_cell_CR2025'
  | 'thin_film';

/**
 * Charging method
 */
export type ChargingMethod =
  | 'USB_C'
  | 'micro_USB'
  | 'wireless_Qi'
  | 'magnetic_connector'
  | 'solar'
  | 'kinetic';

/**
 * Battery configuration
 */
export interface BatteryConfiguration {
  /** Chemistry type */
  chemistry: BatteryChemistry;

  /** Nominal voltage (V) */
  nominalVoltage: number;

  /** Capacity (mAh) */
  capacity_mAh: number;

  /** Energy capacity (Wh) */
  capacity_Wh: number;

  /** Discharge rate (C) */
  dischargeRate: number;

  /** Charging method */
  chargingMethod: ChargingMethod;

  /** Max charging current (mA) */
  maxChargingCurrent: number;

  /** Charging time (hours) */
  chargingTime: number;

  /** Cycle life (number of charge cycles) */
  cycleLife: number;

  /** Form factor */
  formFactor: 'pouch' | 'cylindrical' | 'coin' | 'prismatic' | 'flexible';

  /** Dimensions (mm) */
  dimensions?: {
    length: number;
    width: number;
    height: number;
  };

  /** Weight (g) */
  weight_g: number;

  /** Protection circuit included */
  hasProtectionCircuit: boolean;
}

/**
 * Battery state
 */
export interface BatteryState {
  /** State of charge (0-100%) */
  soc_percent: number;

  /** State of health (0-100%) */
  soh_percent: number;

  /** Voltage (V) */
  voltage_v: number;

  /** Current (mA, positive = discharge) */
  current_ma: number;

  /** Temperature (°C) */
  temperature_c: number;

  /** Charging status */
  isCharging: boolean;

  /** Time to full charge (minutes, if charging) */
  timeToFull_min?: number;

  /** Remaining time (minutes, if discharging) */
  timeRemaining_min?: number;

  /** Power consumption (W) */
  powerConsumption_w: number;

  /** Cycle count */
  cycleCount: number;
}

/**
 * Power consumption profile
 */
export interface PowerProfile {
  /** LED power (W) */
  ledPower: number;

  /** Sensor power (W) */
  sensorPower: number;

  /** Microcontroller power (W) */
  mcuPower: number;

  /** Wireless power (W) */
  wirelessPower: number;

  /** Heating/cooling power (W) */
  thermalPower: number;

  /** Other components power (W) */
  otherPower: number;

  /** Total power (W) */
  totalPower: number;

  /** Average current (mA) */
  averageCurrent_ma: number;

  /** Estimated battery life (hours) */
  batteryLife_hours: number;
}

/**
 * Energy harvesting configuration
 */
export interface EnergyHarvesting {
  /** Harvesting type */
  type: 'solar' | 'kinetic_piezo' | 'kinetic_electromagnetic' | 'thermoelectric';

  /** Maximum output power (mW) */
  maxPower_mw: number;

  /** Average output power (mW) */
  averagePower_mw: number;

  /** Voltage output (V) */
  voltage_v: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** For solar: panel area (cm²) */
  panelArea_cm2?: number;

  /** For kinetic: placement locations */
  placement?: Array<'shoulder' | 'elbow' | 'knee' | 'hip' | 'chest'>;

  /** For thermoelectric: temperature differential (°C) */
  tempDifferential_c?: number;
}

// ============================================================================
// Smart Materials Types
// ============================================================================

/**
 * Conductive material types
 */
export type ConductiveMaterial =
  | 'silver_thread'
  | 'copper_thread'
  | 'stainless_steel_thread'
  | 'conductive_polymer'
  | 'silver_fabric'
  | 'carbon_fiber'
  | 'graphene_textile';

/**
 * Fabric type
 */
export type FabricType =
  | 'cotton'
  | 'organic_cotton'
  | 'polyester'
  | 'recycled_polyester'
  | 'nylon'
  | 'recycled_nylon'
  | 'silk'
  | 'wool'
  | 'bamboo'
  | 'hemp'
  | 'blend';

/**
 * Conductive thread specification
 */
export interface ConductiveThread {
  /** Material type */
  material: ConductiveMaterial;

  /** Resistance per meter (Ω/m) */
  resistance_ohm_per_m: number;

  /** Current capacity (mA) */
  currentCapacity_ma: number;

  /** Diameter (mm) */
  diameter_mm: number;

  /** Flexibility rating (1-10) */
  flexibility: number;

  /** Washability */
  washability: 'hand_wash' | 'machine_delicate' | 'machine_normal' | 'dry_clean';

  /** Durability (flex cycles) */
  flexCycles: number;

  /** Cost per meter (relative, 1-10) */
  costRating: number;
}

/**
 * Fabric specification
 */
export interface FabricSpecification {
  /** Fabric type */
  type: FabricType;

  /** Weight (g/m²) */
  weight_gsm: number;

  /** Thickness (mm) */
  thickness_mm: number;

  /** Stretch percentage */
  stretch_percent: number;

  /** Breathability (1-10) */
  breathability: number;

  /** Water resistance */
  waterResistance: 'none' | 'water_repellent' | 'waterproof';

  /** Moisture wicking */
  moistureWicking: boolean;

  /** Thermal properties */
  thermal: {
    insulation: number; // 1-10
    conductivity: number; // W/m·K
  };

  /** Certifications */
  certifications?: Array<'OEKO_TEX' | 'GOTS' | 'GRS' | 'Fair_Trade'>;
}

// ============================================================================
// Sensor Types
// ============================================================================

/**
 * Sensor types
 */
export type SensorType =
  | 'accelerometer'
  | 'gyroscope'
  | 'magnetometer'
  | 'IMU'
  | 'heart_rate'
  | 'skin_temperature'
  | 'GSR'
  | 'touch_capacitive'
  | 'touch_resistive'
  | 'pressure'
  | 'stretch'
  | 'ambient_light'
  | 'temperature'
  | 'humidity'
  | 'UV_index';

/**
 * Sensor configuration
 */
export interface SensorConfiguration {
  /** Sensor type */
  type: SensorType;

  /** Sample rate (Hz) */
  sampleRate_hz: number;

  /** Resolution (bits) */
  resolution_bits: number;

  /** Power consumption (mA) */
  power_ma: number;

  /** Communication interface */
  interface: 'I2C' | 'SPI' | 'UART' | 'analog' | 'digital';

  /** I2C address (if applicable) */
  i2cAddress?: number;

  /** Measurement range */
  range?: {
    min: number;
    max: number;
    unit: string;
  };

  /** Accuracy */
  accuracy?: string;

  /** Placement on garment */
  placement: Array<'wrist' | 'chest' | 'back' | 'shoulder' | 'waist' | 'leg' | 'arm' | 'head'>;
}

/**
 * Sensor data
 */
export interface SensorData {
  /** Sensor type */
  type: SensorType;

  /** Timestamp */
  timestamp: number;

  /** Raw value(s) */
  raw: number | number[];

  /** Processed value(s) */
  value: number | number[] | { [key: string]: number };

  /** Unit */
  unit: string;

  /** Quality indicator (0-100%) */
  quality?: number;
}

/**
 * Motion data (IMU)
 */
export interface MotionData {
  /** Timestamp */
  timestamp: number;

  /** Acceleration (g) */
  acceleration: {
    x: number;
    y: number;
    z: number;
  };

  /** Gyroscope (°/s) */
  gyroscope: {
    x: number;
    y: number;
    z: number;
  };

  /** Magnetometer (μT) */
  magnetometer?: {
    x: number;
    y: number;
    z: number;
  };

  /** Quaternion (if computed) */
  quaternion?: {
    w: number;
    x: number;
    y: number;
    z: number;
  };

  /** Euler angles (if computed) */
  euler?: {
    roll: number;
    pitch: number;
    yaw: number;
  };
}

/**
 * Biometric data
 */
export interface BiometricData {
  /** Timestamp */
  timestamp: number;

  /** Heart rate (bpm) */
  heartRate?: number;

  /** Heart rate variability (ms) */
  hrv?: number;

  /** Skin temperature (°C) */
  skinTemperature?: number;

  /** Galvanic skin response (μS) */
  gsr?: number;

  /** Breathing rate (breaths/min) */
  breathingRate?: number;

  /** SpO2 (%) */
  spo2?: number;

  /** Stress level (0-100) */
  stressLevel?: number;
}

// ============================================================================
// Thermal Management Types
// ============================================================================

/**
 * Heating element type
 */
export type HeatingElementType =
  | 'carbon_fiber'
  | 'resistance_wire'
  | 'conductive_fabric'
  | 'PCM';

/**
 * Cooling method
 */
export type CoolingMethod =
  | 'peltier'
  | 'fan'
  | 'heat_dissipating_fabric'
  | 'evaporative'
  | 'phase_change';

/**
 * Thermal zone configuration
 */
export interface ThermalZone {
  /** Zone identifier */
  id: string;

  /** Zone name */
  name: string;

  /** Location on garment */
  location: 'chest' | 'back' | 'neck' | 'hands' | 'feet' | 'arms' | 'legs';

  /** Heating or cooling */
  function: 'heating' | 'cooling';

  /** Element type */
  elementType: HeatingElementType | CoolingMethod;

  /** Maximum power (W) */
  maxPower_w: number;

  /** Target temperature (°C) */
  targetTemp_c: number;

  /** Temperature sensor */
  hasTempSensor: boolean;

  /** Area (cm²) */
  area_cm2: number;

  /** Resistance (Ω, for heating) */
  resistance_ohm?: number;
}

/**
 * Thermal state
 */
export interface ThermalState {
  /** Zone ID */
  zoneId: string;

  /** Active status */
  isActive: boolean;

  /** Current temperature (°C) */
  currentTemp_c: number;

  /** Target temperature (°C) */
  targetTemp_c: number;

  /** Power level (0-100%) */
  powerLevel_percent: number;

  /** Actual power consumption (W) */
  actualPower_w: number;

  /** Time to target (seconds) */
  timeToTarget_s?: number;
}

// ============================================================================
// Haptic Types
// ============================================================================

/**
 * Haptic actuator type
 */
export type HapticType = 'ERM' | 'LRA' | 'solenoid' | 'voice_coil';

/**
 * Haptic pattern
 */
export interface HapticPattern {
  /** Pattern name */
  name: string;

  /** Sequence of vibrations */
  sequence: Array<{
    duration_ms: number;
    intensity: number; // 0-100%
    pause_ms: number;  // After this vibration
  }>;

  /** Total duration (ms) */
  totalDuration_ms: number;
}

/**
 * Haptic configuration
 */
export interface HapticConfiguration {
  /** Actuator type */
  type: HapticType;

  /** Number of actuators */
  count: number;

  /** Voltage (V) */
  voltage: number;

  /** Current (mA) */
  current_ma: number;

  /** Frequency (Hz, for LRA) */
  frequency_hz?: number;

  /** Response time (ms) */
  responseTime_ms: number;

  /** Placement locations */
  placement: Array<'wrist' | 'shoulder' | 'back' | 'chest' | 'waist' | 'other'>;
}

// ============================================================================
// Connectivity Types
// ============================================================================

/**
 * Wireless protocol
 */
export type WirelessProtocol =
  | 'BLE'
  | 'Bluetooth_5'
  | 'WiFi'
  | 'NFC'
  | 'Zigbee'
  | 'ANT_plus';

/**
 * Wireless configuration
 */
export interface WirelessConfiguration {
  /** Protocol */
  protocol: WirelessProtocol;

  /** Device name */
  deviceName: string;

  /** MAC address */
  macAddress?: string;

  /** TX power (dBm) */
  txPower_dbm: number;

  /** Range (m) */
  range_m: number;

  /** Data rate (kbps) */
  dataRate_kbps: number;

  /** Power consumption */
  power: {
    tx_ma: number;
    rx_ma: number;
    idle_ma: number;
    sleep_ua: number;
  };

  /** Services (for BLE) */
  services?: Array<{
    uuid: string;
    name: string;
    characteristics: Array<{
      uuid: string;
      name: string;
      properties: Array<'read' | 'write' | 'notify' | 'indicate'>;
    }>;
  }>;
}

/**
 * Connection state
 */
export interface ConnectionState {
  /** Connected status */
  isConnected: boolean;

  /** Connection quality (0-100%) */
  quality_percent?: number;

  /** RSSI (dBm) */
  rssi_dbm?: number;

  /** Connected device name */
  connectedDevice?: string;

  /** Connection duration (seconds) */
  connectionDuration_s?: number;

  /** Data transferred (bytes) */
  dataTransferred_bytes?: number;
}

// ============================================================================
// Safety and Compliance Types
// ============================================================================

/**
 * IP rating (Ingress Protection)
 */
export type IPRating =
  | 'IP20'  // Indoor, no protection
  | 'IP54'  // Dust protected, splash resistant
  | 'IP65'  // Dust tight, water jets
  | 'IP67'  // Dust tight, immersion 1m
  | 'IP68'; // Dust tight, continuous immersion

/**
 * Washability rating
 */
export type WashabilityRating =
  | 'spot_clean'
  | 'hand_wash'
  | 'machine_delicate'
  | 'machine_normal'
  | 'dry_clean_only'
  | 'not_washable';

/**
 * Safety certifications
 */
export type SafetyCertification =
  | 'FCC'
  | 'CE'
  | 'IC'
  | 'RoHS'
  | 'OEKO_TEX'
  | 'UL'
  | 'IEC_62368';

/**
 * Safety limits
 */
export interface SafetyLimits {
  /** Maximum skin contact temperature (°C) */
  maxSkinTemp_c: number;

  /** Maximum internal temperature (°C) */
  maxInternalTemp_c: number;

  /** Maximum current (A) */
  maxCurrent_a: number;

  /** Maximum voltage (V) */
  maxVoltage_v: number;

  /** Overcurrent protection */
  hasOvercurrentProtection: boolean;

  /** Thermal cutoff temperature (°C) */
  thermalCutoff_c?: number;

  /** EMF/SAR limit compliance */
  emfCompliant: boolean;
}

// ============================================================================
// Wearable Fashion Device Types
// ============================================================================

/**
 * Complete wearable fashion device specification
 */
export interface WearableFashionDevice {
  // Identity
  id: string;
  name: string;
  category: WearableCategory;
  type: GarmentType | JewelryType;
  size: FashionSize | string;

  // Physical properties
  fabric?: FabricSpecification;
  weight_g: number;
  dimensions?: {
    length_cm: number;
    width_cm: number;
    height_cm?: number;
  };

  // Electronic components
  led?: LEDConfiguration;
  battery: BatteryConfiguration;
  sensors?: SensorConfiguration[];
  thermal?: ThermalZone[];
  haptic?: HapticConfiguration;
  wireless?: WirelessConfiguration;

  // Materials
  conductiveMaterials?: ConductiveThread[];

  // Energy
  energyHarvesting?: EnergyHarvesting;
  powerProfile: PowerProfile;

  // Safety and compliance
  ipRating: IPRating;
  washability: WashabilityRating;
  certifications: SafetyCertification[];
  safetyLimits: SafetyLimits;

  // Sustainability
  sustainability?: {
    recycledContent_percent: number;
    recyclability_percent: number;
    carbonFootprint_kg: number;
    repairability_score: number; // 1-10
  };

  // Metadata
  manufacturer?: string;
  modelNumber?: string;
  firmwareVersion?: string;
  manufactureDate?: Date;
}

/**
 * Device state (runtime)
 */
export interface DeviceState {
  /** Device ID */
  deviceId: string;

  /** Power state */
  isPoweredOn: boolean;

  /** Battery state */
  battery: BatteryState;

  /** LED state */
  led?: LEDState;

  /** Thermal state */
  thermal?: ThermalState[];

  /** Sensor data */
  sensors?: SensorData[];

  /** Motion data */
  motion?: MotionData;

  /** Biometric data */
  biometrics?: BiometricData;

  /** Connection state */
  connection?: ConnectionState;

  /** Operating mode */
  mode: string;

  /** Errors/warnings */
  errors?: string[];
  warnings?: string[];

  /** Timestamp */
  timestamp: number;
}

// ============================================================================
// Design and Manufacturing Types
// ============================================================================

/**
 * Manufacturing method
 */
export type ManufacturingMethod =
  | 'hand_sewn'
  | 'machine_sewn'
  | 'embroidered'
  | 'printed'
  | 'woven'
  | 'adhesive'
  | 'ultrasonic_welded'
  | 'hybrid';

/**
 * Circuit integration method
 */
export type CircuitIntegration =
  | 'conductive_thread'
  | 'printed_electronics'
  | 'embroidered_circuit'
  | 'flexible_PCB'
  | 'woven_circuit';

/**
 * Manufacturing specification
 */
export interface ManufacturingSpec {
  /** Primary method */
  method: ManufacturingMethod;

  /** Circuit integration */
  circuitIntegration: CircuitIntegration;

  /** Assembly time (hours) */
  assemblyTime_hours: number;

  /** Skill level required */
  skillLevel: 'beginner' | 'intermediate' | 'advanced' | 'expert';

  /** Special equipment needed */
  equipment?: string[];

  /** Quality control points */
  qcPoints?: Array<{
    stage: string;
    test: string;
    criteria: string;
  }>;

  /** Estimated cost (USD) */
  estimatedCost_usd?: number;
}

// ============================================================================
// User Interaction Types
// ============================================================================

/**
 * Control method
 */
export type ControlMethod =
  | 'button'
  | 'touch'
  | 'gesture'
  | 'voice'
  | 'smartphone_app'
  | 'automatic';

/**
 * User preferences
 */
export interface UserPreferences {
  /** Preferred LED pattern */
  preferredPattern?: LEDPattern;

  /** Preferred brightness (0-100%) */
  preferredBrightness?: number;

  /** Preferred colors */
  preferredColors?: string[];

  /** Thermal comfort temperature (°C) */
  comfortTemp_c?: number;

  /** Auto-brightness enabled */
  autoBrightness?: boolean;

  /** Motion activation enabled */
  motionActivation?: boolean;

  /** Haptic feedback enabled */
  hapticEnabled?: boolean;

  /** Notification settings */
  notifications?: {
    lowBattery: boolean;
    temperatureWarning: boolean;
    connectionLost: boolean;
  };
}

// ============================================================================
// Analytics and Performance Types
// ============================================================================

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Average battery life (hours) */
  avgBatteryLife_hours: number;

  /** Power efficiency (Wh/day) */
  powerEfficiency_wh_per_day: number;

  /** LED brightness uniformity (%) */
  ledUniformity_percent?: number;

  /** Sensor accuracy (%) */
  sensorAccuracy_percent?: number;

  /** Thermal response time (seconds) */
  thermalResponse_s?: number;

  /** Wireless connection reliability (%) */
  connectionReliability_percent?: number;

  /** User comfort rating (1-10) */
  comfortRating?: number;
}

/**
 * Usage statistics
 */
export interface UsageStatistics {
  /** Total wear time (hours) */
  totalWearTime_hours: number;

  /** Total charge cycles */
  totalChargeCycles: number;

  /** Most used LED pattern */
  mostUsedPattern?: LEDPattern;

  /** Average brightness */
  avgBrightness_percent?: number;

  /** Total energy consumed (Wh) */
  totalEnergyConsumed_wh: number;

  /** Energy harvested (Wh) */
  energyHarvested_wh?: number;

  /** Wash count */
  washCount?: number;

  /** Firmware updates */
  firmwareUpdates?: number;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  /** Success status */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error message (if failed) */
  error?: string;

  /** Timestamp */
  timestamp: number;
}

/**
 * Calculation result
 */
export interface CalculationResult {
  /** Result value */
  value: number;

  /** Unit */
  unit: string;

  /** Confidence (0-100%) */
  confidence?: number;

  /** Min/max range */
  range?: {
    min: number;
    max: number;
  };

  /** Assumptions used */
  assumptions?: { [key: string]: any };
}

// ============================================================================
// Export all types
// ============================================================================

export default {
  // Main device types
  WearableFashionDevice,
  DeviceState,

  // Component types
  LEDConfiguration,
  BatteryConfiguration,
  SensorConfiguration,
  ThermalZone,
  HapticConfiguration,
  WirelessConfiguration,

  // State types
  LEDState,
  BatteryState,
  ThermalState,
  ConnectionState,

  // Data types
  SensorData,
  MotionData,
  BiometricData,

  // Enums
  GarmentType,
  JewelryType,
  WearableCategory,
  LEDType,
  LEDPattern,
  BatteryChemistry,
  ChargingMethod,
  SensorType,
  WirelessProtocol,
  IPRating,
  WashabilityRating,
  SafetyCertification,

  // Other types
  PowerProfile,
  EnergyHarvesting,
  UserPreferences,
  PerformanceMetrics,
  UsageStatistics,
  ManufacturingSpec,
};
