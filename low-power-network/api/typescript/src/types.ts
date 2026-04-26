/**
 * WIA-COMM-018: Low-Power Network - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core LPWAN Technologies
// ============================================================================

/**
 * Supported LPWAN technologies
 */
export enum Technology {
  /** LoRaWAN - Long Range Wide Area Network */
  LORAWAN = 'LoRaWAN',

  /** Sigfox - Ultra-narrow band LPWAN */
  SIGFOX = 'Sigfox',

  /** NB-IoT - Narrowband Internet of Things */
  NBIOT = 'NB-IoT',

  /** LTE-M - LTE Cat-M1 */
  LTEM = 'LTE-M'
}

/**
 * LoRaWAN device classes
 */
export enum DeviceClass {
  /** Class A: Lowest power, uplink anytime, downlink after uplink only */
  A = 'A',

  /** Class B: Scheduled receive windows (beacon-based) */
  B = 'B',

  /** Class C: Continuous listening (except when transmitting) */
  C = 'C'
}

/**
 * LoRaWAN regions
 */
export enum Region {
  /** Europe 868 MHz */
  EU868 = 'EU868',

  /** United States 915 MHz */
  US915 = 'US915',

  /** China 779 MHz */
  CN779 = 'CN779',

  /** Europe 433 MHz */
  EU433 = 'EU433',

  /** Australia 915 MHz */
  AU915 = 'AU915',

  /** China 470 MHz */
  CN470 = 'CN470',

  /** Asia 923 MHz */
  AS923 = 'AS923',

  /** Korea 920 MHz */
  KR920 = 'KR920',

  /** India 865 MHz */
  IN865 = 'IN865',

  /** Russia 864 MHz */
  RU864 = 'RU864'
}

/**
 * LoRa spreading factors
 */
export enum SpreadingFactor {
  SF7 = 7,
  SF8 = 8,
  SF9 = 9,
  SF10 = 10,
  SF11 = 11,
  SF12 = 12
}

/**
 * LoRa bandwidth options
 */
export enum Bandwidth {
  BW125 = 125,
  BW250 = 250,
  BW500 = 500
}

/**
 * LoRa coding rates
 */
export enum CodingRate {
  CR_4_5 = '4/5',
  CR_4_6 = '4/6',
  CR_4_7 = '4/7',
  CR_4_8 = '4/8'
}

// ============================================================================
// Device Configuration
// ============================================================================

/**
 * Generic LPWAN device configuration
 */
export interface LPWANConfig {
  /** Technology to use */
  technology: Technology;

  /** Device unique identifier */
  deviceId: string;

  /** Device class (LoRaWAN only) */
  deviceClass?: DeviceClass;

  /** Region (LoRaWAN only) */
  region?: Region;

  /** Power saving enabled */
  powerSaving?: boolean;

  /** Enable adaptive data rate */
  enableADR?: boolean;
}

/**
 * LoRaWAN specific configuration
 */
export interface LoRaWANConfig extends LPWANConfig {
  technology: Technology.LORAWAN;

  /** Device EUI (Extended Unique Identifier) - 8 bytes */
  deviceEUI: string;

  /** Application EUI - 8 bytes */
  appEUI: string;

  /** Application Key - 16 bytes (OTAA) */
  appKey?: string;

  /** Network Session Key - 16 bytes (ABP) */
  nwkSKey?: string;

  /** Application Session Key - 16 bytes (ABP) */
  appSKey?: string;

  /** Device Address - 4 bytes (ABP) */
  devAddr?: string;

  /** Device class */
  deviceClass: DeviceClass;

  /** Region */
  region: Region;

  /** Initial spreading factor */
  spreadingFactor?: SpreadingFactor;

  /** Initial bandwidth */
  bandwidth?: Bandwidth;

  /** Initial coding rate */
  codingRate?: CodingRate;

  /** Initial TX power in dBm */
  txPower?: number;

  /** Use confirmed uplinks */
  confirmed?: boolean;

  /** Retries for confirmed uplinks */
  retries?: number;
}

/**
 * Sigfox specific configuration
 */
export interface SigfoxConfig extends LPWANConfig {
  technology: Technology.SIGFOX;

  /** Device ID (4 bytes) */
  deviceId: string;

  /** Device PAC (Porting Authorization Code) */
  pac: string;

  /** Network Authentication Key (16 bytes) */
  nak?: string;

  /** RC (Radio Configuration) zone */
  rcZone: 'RC1' | 'RC2' | 'RC3' | 'RC4' | 'RC5' | 'RC6' | 'RC7';
}

/**
 * NB-IoT specific configuration
 */
export interface NBIoTConfig extends LPWANConfig {
  technology: Technology.NBIOT;

  /** IMSI (International Mobile Subscriber Identity) */
  imsi: string;

  /** IMEI (International Mobile Equipment Identity) */
  imei: string;

  /** SIM card PIN */
  pin?: string;

  /** APN (Access Point Name) */
  apn: string;

  /** PSM (Power Saving Mode) enabled */
  psmEnabled?: boolean;

  /** eDRX (Extended Discontinuous Reception) enabled */
  edrxEnabled?: boolean;

  /** PSM active timer (T3324) in seconds */
  psmActiveTimer?: number;

  /** PSM periodic TAU timer (T3412) in seconds */
  psmPeriodicTimer?: number;

  /** eDRX cycle in seconds */
  edrxCycle?: number;
}

/**
 * LTE-M specific configuration
 */
export interface LTEMConfig extends LPWANConfig {
  technology: Technology.LTEM;

  /** IMSI (International Mobile Subscriber Identity) */
  imsi: string;

  /** IMEI (International Mobile Equipment Identity) */
  imei: string;

  /** SIM card PIN */
  pin?: string;

  /** APN (Access Point Name) */
  apn: string;

  /** PSM enabled */
  psmEnabled?: boolean;

  /** eDRX enabled */
  edrxEnabled?: boolean;

  /** VoLTE enabled */
  volteEnabled?: boolean;
}

// ============================================================================
// Message Types
// ============================================================================

/**
 * Uplink message (device to network)
 */
export interface UplinkMessage {
  /** Application port */
  port: number;

  /** Payload data */
  payload: Buffer;

  /** Request confirmation (LoRaWAN) */
  confirmed?: boolean;

  /** Timestamp */
  timestamp?: number;

  /** Encoding type */
  encoding?: 'binary' | 'cayennelpp' | 'json';
}

/**
 * Downlink message (network to device)
 */
export interface DownlinkMessage {
  /** Application port */
  port: number;

  /** Payload data */
  payload: Buffer;

  /** Message was confirmed */
  confirmed: boolean;

  /** Frame counter */
  frameCounter: number;

  /** RSSI (Received Signal Strength Indicator) in dBm */
  rssi?: number;

  /** SNR (Signal-to-Noise Ratio) in dB */
  snr?: number;

  /** Timestamp */
  timestamp: number;
}

/**
 * Join request/response
 */
export interface JoinRequest {
  /** Device EUI */
  deviceEUI: string;

  /** Application EUI */
  appEUI: string;

  /** Device nonce */
  devNonce: number;
}

export interface JoinAccept {
  /** Application nonce */
  appNonce: number;

  /** Network ID */
  netID: number;

  /** Device address */
  devAddr: string;

  /** Join successful */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Device Status
// ============================================================================

/**
 * Device connection status
 */
export enum ConnectionStatus {
  /** Not connected */
  DISCONNECTED = 'disconnected',

  /** Connecting to network */
  CONNECTING = 'connecting',

  /** Connected to network */
  CONNECTED = 'connected',

  /** Joining network (LoRaWAN OTAA) */
  JOINING = 'joining',

  /** Error state */
  ERROR = 'error'
}

/**
 * Power mode
 */
export enum PowerMode {
  /** Active mode - can TX/RX anytime */
  ACTIVE = 'active',

  /** Idle mode - low power, can wake quickly */
  IDLE = 'idle',

  /** Deep sleep - ultra-low power, slower wake */
  DEEP_SLEEP = 'deep_sleep',

  /** Hibernation - minimal power, may lose RAM */
  HIBERNATION = 'hibernation'
}

/**
 * Device status information
 */
export interface DeviceStatus {
  /** Connection status */
  connectionStatus: ConnectionStatus;

  /** Current power mode */
  powerMode: PowerMode;

  /** Battery level (0-100%) */
  batteryLevel?: number;

  /** Battery voltage (V) */
  batteryVoltage?: number;

  /** Signal strength (RSSI) in dBm */
  rssi?: number;

  /** Signal quality (SNR) in dB */
  snr?: number;

  /** Frame counter (uplink) */
  frameCounterUp?: number;

  /** Frame counter (downlink) */
  frameCounterDown?: number;

  /** Uptime in seconds */
  uptime?: number;

  /** Last seen timestamp */
  lastSeen?: number;

  /** Data rate (bps) */
  dataRate?: number;

  /** Spreading factor (LoRaWAN) */
  spreadingFactor?: SpreadingFactor;

  /** TX power (dBm) */
  txPower?: number;

  /** Messages sent today */
  messagesSentToday?: number;

  /** Messages received today */
  messagesReceivedToday?: number;

  /** Duty cycle usage (%) */
  dutyCycleUsage?: number;
}

// ============================================================================
// Adaptive Data Rate (ADR)
// ============================================================================

/**
 * ADR configuration
 */
export interface ADRConfig {
  /** Enable ADR */
  enabled: boolean;

  /** Target SNR in dB */
  targetSNR?: number;

  /** Minimum data rate */
  minDataRate?: number;

  /** Maximum data rate */
  maxDataRate?: number;

  /** Minimum TX power in dBm */
  minTxPower?: number;

  /** Maximum TX power in dBm */
  maxTxPower?: number;

  /** Number of uplinks for averaging */
  historySize?: number;

  /** SNR margin in dB */
  snrMargin?: number;
}

/**
 * ADR adjustment
 */
export interface ADRAdjustment {
  /** Adjustment type */
  action: 'INCREASE_SF' | 'DECREASE_SF' | 'INCREASE_POWER' | 'REDUCE_POWER' | 'NO_CHANGE';

  /** New spreading factor */
  newSpreadingFactor?: SpreadingFactor;

  /** New TX power in dBm */
  newTxPower?: number;

  /** Reason for adjustment */
  reason: string;

  /** Average SNR that triggered adjustment */
  averageSNR?: number;
}

// ============================================================================
// Power Management
// ============================================================================

/**
 * Sleep configuration
 */
export interface SleepConfig {
  /** Sleep duration in milliseconds */
  duration: number;

  /** Power mode during sleep */
  mode: PowerMode.DEEP_SLEEP | PowerMode.HIBERNATION;

  /** Wake on downlink message */
  wakeOnDownlink?: boolean;

  /** Wake on external interrupt */
  wakeOnInterrupt?: boolean;

  /** Interrupt pin (if wakeOnInterrupt=true) */
  interruptPin?: number;

  /** Scheduled wake time (timestamp) */
  scheduledWakeTime?: number;
}

/**
 * Battery information
 */
export interface BatteryInfo {
  /** Battery level (0-100%) */
  level: number;

  /** Battery voltage (V) */
  voltage: number;

  /** Battery type */
  type: 'alkaline' | 'lithium' | 'li-ion' | 'li-socl2' | 'rechargeable' | 'unknown';

  /** Charging status */
  charging: boolean;

  /** Estimated days remaining */
  estimatedDaysRemaining?: number;

  /** Average daily consumption (mAh) */
  avgDailyConsumption?: number;

  /** Battery health (0-100%) */
  health?: number;
}

/**
 * Energy harvesting status
 */
export interface EnergyHarvesting {
  /** Harvesting enabled */
  enabled: boolean;

  /** Harvesting type */
  type: 'solar' | 'thermal' | 'vibration' | 'rf' | 'none';

  /** Current power generated (mW) */
  currentPower?: number;

  /** Daily energy harvested (mWh) */
  dailyEnergy?: number;

  /** Harvesting efficiency (%) */
  efficiency?: number;
}

// ============================================================================
// Sensor Data Encoding
// ============================================================================

/**
 * Cayenne LPP data types
 */
export enum CayenneLPPType {
  DIGITAL_INPUT = 0,
  DIGITAL_OUTPUT = 1,
  ANALOG_INPUT = 2,
  ANALOG_OUTPUT = 3,
  ILLUMINANCE = 101,
  PRESENCE = 102,
  TEMPERATURE = 103,
  HUMIDITY = 104,
  ACCELEROMETER = 113,
  BAROMETRIC_PRESSURE = 115,
  GYROMETER = 134,
  GPS = 136
}

/**
 * Cayenne LPP sensor data
 */
export interface CayenneLPPData {
  /** Channel number */
  channel: number;

  /** Data type */
  type: CayenneLPPType;

  /** Value */
  value: number | { x: number; y: number; z: number } | { lat: number; lon: number; alt: number };
}

/**
 * Sensor reading
 */
export interface SensorReading {
  /** Sensor type */
  type: string;

  /** Sensor value */
  value: number | object;

  /** Unit of measurement */
  unit?: string;

  /** Timestamp */
  timestamp?: number;

  /** Accuracy/precision */
  accuracy?: number;
}

// ============================================================================
// Network Information
// ============================================================================

/**
 * Gateway information
 */
export interface GatewayInfo {
  /** Gateway ID */
  gatewayId: string;

  /** Gateway location */
  location?: {
    latitude: number;
    longitude: number;
    altitude?: number;
  };

  /** RSSI in dBm */
  rssi: number;

  /** SNR in dB */
  snr: number;

  /** Channel frequency */
  frequency?: number;

  /** Distance from device (meters) */
  distance?: number;
}

/**
 * Network statistics
 */
export interface NetworkStats {
  /** Total uplinks sent */
  totalUplinks: number;

  /** Total downlinks received */
  totalDownlinks: number;

  /** Failed transmissions */
  failedTransmissions: number;

  /** Average RSSI */
  averageRSSI: number;

  /** Average SNR */
  averageSNR: number;

  /** Packet delivery ratio (%) */
  packetDeliveryRatio: number;

  /** Average latency (ms) */
  averageLatency?: number;

  /** Gateways in range */
  gatewaysInRange?: GatewayInfo[];
}

// ============================================================================
// Coverage and Link Quality
// ============================================================================

/**
 * Link quality metrics
 */
export interface LinkQuality {
  /** RSSI in dBm */
  rssi: number;

  /** SNR in dB */
  snr: number;

  /** Link quality indicator (0-100) */
  lqi: number;

  /** Estimated range (meters) */
  estimatedRange?: number;

  /** Link margin (dB) */
  linkMargin?: number;

  /** Quality category */
  quality: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';
}

/**
 * Coverage survey result
 */
export interface CoverageSurveyResult {
  /** Survey start time */
  startTime: number;

  /** Survey duration (seconds) */
  duration: number;

  /** Test location */
  location?: {
    latitude: number;
    longitude: number;
  };

  /** Number of test messages */
  totalTests: number;

  /** Successful transmissions */
  successfulTests: number;

  /** Success rate (%) */
  successRate: number;

  /** Average RSSI */
  averageRSSI: number;

  /** Average SNR */
  averageSNR: number;

  /** Min/Max RSSI */
  rssiRange: { min: number; max: number };

  /** Min/Max SNR */
  snrRange: { min: number; max: number };

  /** Detected gateways */
  gateways: GatewayInfo[];

  /** Overall coverage quality */
  coverageQuality: 'excellent' | 'good' | 'marginal' | 'poor' | 'no_coverage';
}

// ============================================================================
// Events
// ============================================================================

/**
 * Event types
 */
export enum EventType {
  /** Connected to network */
  CONNECTED = 'connected',

  /** Disconnected from network */
  DISCONNECTED = 'disconnected',

  /** Join successful */
  JOIN_SUCCESS = 'join_success',

  /** Join failed */
  JOIN_FAILED = 'join_failed',

  /** Uplink sent */
  UPLINK_SENT = 'uplink_sent',

  /** Uplink confirmed */
  UPLINK_CONFIRMED = 'uplink_confirmed',

  /** Uplink failed */
  UPLINK_FAILED = 'uplink_failed',

  /** Downlink received */
  DOWNLINK_RECEIVED = 'downlink_received',

  /** ADR adjustment */
  ADR_ADJUSTED = 'adr_adjusted',

  /** Battery low */
  BATTERY_LOW = 'battery_low',

  /** Battery critical */
  BATTERY_CRITICAL = 'battery_critical',

  /** Signal weak */
  SIGNAL_WEAK = 'signal_weak',

  /** Signal lost */
  SIGNAL_LOST = 'signal_lost',

  /** Error occurred */
  ERROR = 'error'
}

/**
 * Event data
 */
export interface EventData {
  /** Event type */
  type: EventType;

  /** Event timestamp */
  timestamp: number;

  /** Event message */
  message?: string;

  /** Additional data */
  data?: any;

  /** Severity */
  severity?: 'info' | 'warning' | 'error' | 'critical';
}

/**
 * Event handler
 */
export type EventHandler = (event: EventData) => void;

// ============================================================================
// Error Types
// ============================================================================

/**
 * LPWAN error codes
 */
export enum LPWANErrorCode {
  /** Generic error */
  GENERIC_ERROR = 'LPWAN001',

  /** Initialization failed */
  INITIALIZATION_FAILED = 'LPWAN002',

  /** Join failed */
  JOIN_FAILED = 'LPWAN003',

  /** Transmission failed */
  TRANSMISSION_FAILED = 'LPWAN004',

  /** Invalid configuration */
  INVALID_CONFIG = 'LPWAN005',

  /** Invalid payload */
  INVALID_PAYLOAD = 'LPWAN006',

  /** Duty cycle exceeded */
  DUTY_CYCLE_EXCEEDED = 'LPWAN007',

  /** No network coverage */
  NO_COVERAGE = 'LPWAN008',

  /** Authentication failed */
  AUTH_FAILED = 'LPWAN009',

  /** Battery too low */
  BATTERY_LOW = 'LPWAN010',

  /** Timeout */
  TIMEOUT = 'LPWAN011',

  /** Not supported */
  NOT_SUPPORTED = 'LPWAN012'
}

/**
 * LPWAN error
 */
export class LPWANError extends Error {
  constructor(
    public code: LPWANErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'LPWANError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * LPWAN communication constants
 */
export const LPWAN_CONSTANTS = {
  /** Maximum payload sizes by technology */
  MAX_PAYLOAD: {
    LORAWAN: 242,
    SIGFOX: 12,
    NBIOT: 1600,
    LTEM: 1000
  },

  /** Typical battery life targets (years) */
  BATTERY_LIFE_TARGET: {
    LORAWAN_CLASS_A: 10,
    LORAWAN_CLASS_B: 7,
    LORAWAN_CLASS_C: 1,
    SIGFOX: 15,
    NBIOT_PSM: 10,
    LTEM_PSM: 8
  },

  /** Typical range (km) */
  TYPICAL_RANGE_URBAN: {
    LORAWAN: 5,
    SIGFOX: 7,
    NBIOT: 5,
    LTEM: 5
  },

  TYPICAL_RANGE_RURAL: {
    LORAWAN: 30,
    SIGFOX: 40,
    NBIOT: 35,
    LTEM: 40
  },

  /** Data rates (bps) */
  DATA_RATE: {
    LORAWAN_MIN: 250,
    LORAWAN_MAX: 50000,
    SIGFOX: 100,
    NBIOT_MIN: 20000,
    NBIOT_MAX: 250000,
    LTEM_MAX: 1000000
  },

  /** Power consumption (mA) */
  POWER_CONSUMPTION: {
    SLEEP_MIN: 0.001,
    SLEEP_MAX: 0.005,
    RX_MIN: 10,
    RX_MAX: 60,
    TX_MIN: 20,
    TX_MAX: 250
  }
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = LPWANError> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = LPWANError> = Promise<Result<T, E>>;

/**
 * Message handler
 */
export type MessageHandler = (message: DownlinkMessage) => void | Promise<void>;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  LPWANConfig,
  LoRaWANConfig,
  SigfoxConfig,
  NBIoTConfig,
  LTEMConfig,
  UplinkMessage,
  DownlinkMessage,
  JoinRequest,
  JoinAccept,
  DeviceStatus,
  ADRConfig,
  ADRAdjustment,
  SleepConfig,
  BatteryInfo,
  EnergyHarvesting,
  CayenneLPPData,
  SensorReading,
  GatewayInfo,
  NetworkStats,
  LinkQuality,
  CoverageSurveyResult,
  EventData,
  EventHandler,
  MessageHandler
};

export {
  Technology,
  DeviceClass,
  Region,
  SpreadingFactor,
  Bandwidth,
  CodingRate,
  ConnectionStatus,
  PowerMode,
  CayenneLPPType,
  EventType,
  LPWANErrorCode,
  LPWANError,
  LPWAN_CONSTANTS
};
