/**
 * WIA-COMM-002: IoT (M2M) - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA IoT and M2M Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core IoT Types
// ============================================================================

/**
 * IoT Protocol types
 */
export type IoTProtocol = 'MQTT' | 'CoAP' | 'AMQP' | 'HTTP' | 'WebSocket' | 'MQTT-SN';

/**
 * LPWAN Technology types
 */
export type LPWANTechnology = 'LoRaWAN' | 'Sigfox' | 'NB-IoT' | 'LTE-M' | 'Thread' | 'Zigbee';

/**
 * Device type categories
 */
export type DeviceType =
  | 'sensor'
  | 'actuator'
  | 'gateway'
  | 'controller'
  | 'hybrid'
  | 'edge-device';

/**
 * Device power mode
 */
export type PowerMode = 'normal' | 'eco' | 'sleep' | 'deep-sleep' | 'ultra-low-power';

/**
 * Device status
 */
export type DeviceStatus = 'online' | 'offline' | 'sleeping' | 'error' | 'maintenance';

// ============================================================================
// Device Configuration
// ============================================================================

/**
 * IoT device configuration
 */
export interface IoTDeviceConfig {
  /** Unique device identifier */
  deviceId: string;

  /** Device type */
  deviceType: DeviceType;

  /** Communication protocol */
  protocol: IoTProtocol;

  /** Connection details */
  connection: ConnectionConfig;

  /** Security credentials */
  credentials?: DeviceCredentials;

  /** Device metadata */
  metadata?: DeviceMetadata;

  /** Power management settings */
  power?: PowerConfig;

  /** Reporting configuration */
  reporting?: ReportingConfig;
}

/**
 * Connection configuration
 */
export interface ConnectionConfig {
  /** Broker/server address */
  broker?: string;

  /** Port number */
  port?: number;

  /** Use TLS/SSL */
  secure?: boolean;

  /** Connection timeout (ms) */
  timeout?: number;

  /** Keep-alive interval (seconds) */
  keepAlive?: number;

  /** Auto-reconnect */
  autoReconnect?: boolean;

  /** Reconnect backoff delays (seconds) */
  reconnectBackoff?: number[];
}

/**
 * Device credentials
 */
export interface DeviceCredentials {
  /** Authentication type */
  type: 'password' | 'certificate' | 'token' | 'psk' | 'none';

  /** Username (for password auth) */
  username?: string;

  /** Password (for password auth) */
  password?: string;

  /** Client certificate (PEM format) */
  clientCert?: string;

  /** Client private key (PEM format) */
  clientKey?: string;

  /** CA certificate (PEM format) */
  caCert?: string;

  /** Authentication token */
  token?: string;

  /** Pre-shared key */
  psk?: string;

  /** Key identifier */
  keyId?: string;
}

/**
 * Device metadata
 */
export interface DeviceMetadata {
  /** Device name */
  name?: string;

  /** Manufacturer */
  manufacturer?: string;

  /** Model number */
  model?: string;

  /** Serial number */
  serialNumber?: string;

  /** Hardware version */
  hardwareVersion?: string;

  /** Firmware version */
  firmwareVersion?: string;

  /** Location */
  location?: GeolocationData;

  /** Tags */
  tags?: string[];

  /** Custom properties */
  properties?: Record<string, unknown>;
}

/**
 * Geolocation data
 */
export interface GeolocationData {
  /** Latitude */
  latitude: number;

  /** Longitude */
  longitude: number;

  /** Altitude (meters) */
  altitude?: number;

  /** Accuracy (meters) */
  accuracy?: number;

  /** Description */
  description?: string;
}

/**
 * Power configuration
 */
export interface PowerConfig {
  /** Power mode */
  mode: PowerMode;

  /** Sleep interval (seconds) */
  sleepInterval?: number;

  /** Wake-up sources */
  wakeupSources?: Array<'timer' | 'interrupt' | 'network' | 'motion'>;

  /** Battery monitoring */
  batteryMonitoring?: boolean;

  /** Low battery threshold (percentage) */
  lowBatteryThreshold?: number;
}

/**
 * Reporting configuration
 */
export interface ReportingConfig {
  /** Reporting interval (seconds) */
  interval: number;

  /** Report on change threshold */
  changeThreshold?: number;

  /** Batch reporting */
  batching?: boolean;

  /** Batch size */
  batchSize?: number;

  /** Data compression */
  compression?: boolean;

  /** Payload format */
  format?: 'json' | 'cbor' | 'protobuf' | 'binary';
}

// ============================================================================
// MQTT Protocol
// ============================================================================

/**
 * MQTT Quality of Service levels
 */
export type MQTTQoS = 0 | 1 | 2;

/**
 * MQTT version
 */
export type MQTTVersion = '3.1.1' | '5.0';

/**
 * MQTT configuration
 */
export interface MQTTConfig extends ConnectionConfig {
  /** MQTT version */
  version?: MQTTVersion;

  /** Client ID */
  clientId?: string;

  /** Clean session */
  cleanSession?: boolean;

  /** Last Will and Testament */
  will?: MQTTWill;

  /** Persistent session */
  persistentSession?: boolean;

  /** Message queue size */
  queueSize?: number;
}

/**
 * MQTT Last Will and Testament
 */
export interface MQTTWill {
  /** Topic */
  topic: string;

  /** Payload */
  payload: string | Buffer;

  /** QoS level */
  qos: MQTTQoS;

  /** Retain flag */
  retain: boolean;
}

/**
 * MQTT publish options
 */
export interface MQTTPublishOptions {
  /** Quality of Service */
  qos?: MQTTQoS;

  /** Retain message */
  retain?: boolean;

  /** Duplicate flag */
  dup?: boolean;

  /** Message properties (MQTT 5.0) */
  properties?: MQTTProperties;
}

/**
 * MQTT 5.0 properties
 */
export interface MQTTProperties {
  /** Message expiry interval (seconds) */
  messageExpiryInterval?: number;

  /** Content type */
  contentType?: string;

  /** Response topic */
  responseTopic?: string;

  /** Correlation data */
  correlationData?: Buffer;

  /** User properties */
  userProperties?: Record<string, string>;

  /** Topic alias */
  topicAlias?: number;
}

// ============================================================================
// CoAP Protocol
// ============================================================================

/**
 * CoAP method types
 */
export type CoAPMethod = 'GET' | 'POST' | 'PUT' | 'DELETE';

/**
 * CoAP message type
 */
export type CoAPMessageType = 'CON' | 'NON' | 'ACK' | 'RST';

/**
 * CoAP configuration
 */
export interface CoAPConfig {
  /** Server host */
  host: string;

  /** Server port */
  port?: number;

  /** Use DTLS */
  secure?: boolean;

  /** Request timeout (ms) */
  timeout?: number;

  /** Maximum retransmissions */
  maxRetransmit?: number;

  /** ACK timeout (ms) */
  ackTimeout?: number;
}

/**
 * CoAP request options
 */
export interface CoAPRequestOptions {
  /** Request method */
  method: CoAPMethod;

  /** Resource path */
  path: string;

  /** Message type */
  type?: CoAPMessageType;

  /** Payload */
  payload?: Buffer | string;

  /** Observe */
  observe?: boolean;

  /** Content format */
  contentFormat?: number;

  /** Accept format */
  accept?: number;

  /** Custom options */
  options?: Record<number, Buffer | string | number>;
}

/**
 * CoAP response
 */
export interface CoAPResponse {
  /** Response code */
  code: number;

  /** Response code class */
  codeClass: number;

  /** Response code detail */
  codeDetail: number;

  /** Payload */
  payload: Buffer;

  /** Options */
  options: Record<number, Buffer | string | number>;

  /** Response time (ms) */
  responseTime: number;
}

// ============================================================================
// Device Telemetry
// ============================================================================

/**
 * Telemetry measurement
 */
export interface TelemetryMeasurement {
  /** Metric name */
  metric: string;

  /** Measured value */
  value: number | string | boolean;

  /** Unit of measurement */
  unit?: string;

  /** Timestamp */
  timestamp?: Date | string;

  /** Quality indicator */
  quality?: number;

  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Telemetry data packet
 */
export interface TelemetryData {
  /** Device ID */
  deviceId: string;

  /** Timestamp */
  timestamp: Date | string;

  /** Measurements */
  measurements: TelemetryMeasurement[];

  /** Location (if mobile device) */
  location?: GeolocationData;

  /** Signal quality */
  signal?: SignalQuality;

  /** Battery level */
  battery?: BatteryStatus;

  /** Sequence number */
  sequence?: number;
}

/**
 * Signal quality metrics
 */
export interface SignalQuality {
  /** Signal strength (dBm) */
  rssi?: number;

  /** Signal-to-noise ratio (dB) */
  snr?: number;

  /** Link quality indicator */
  lqi?: number;

  /** Bit error rate */
  ber?: number;
}

/**
 * Battery status
 */
export interface BatteryStatus {
  /** Voltage (V) */
  voltage?: number;

  /** Percentage (0-100) */
  percentage?: number;

  /** Charging status */
  charging?: boolean;

  /** Time to empty (minutes) */
  timeToEmpty?: number;

  /** Health indicator */
  health?: 'good' | 'fair' | 'poor' | 'critical';
}

// ============================================================================
// Device Commands
// ============================================================================

/**
 * Device command
 */
export interface DeviceCommand {
  /** Command ID */
  id: string;

  /** Device ID */
  deviceId: string;

  /** Command action */
  action: string;

  /** Command parameters */
  parameters?: Record<string, unknown>;

  /** Priority */
  priority?: 'low' | 'normal' | 'high' | 'critical';

  /** Expiration time */
  expiresAt?: Date | string;

  /** Correlation ID (for request-response) */
  correlationId?: string;

  /** Timestamp */
  timestamp: Date | string;
}

/**
 * Command response
 */
export interface CommandResponse {
  /** Command ID */
  commandId: string;

  /** Device ID */
  deviceId: string;

  /** Status */
  status: 'success' | 'failure' | 'pending' | 'timeout';

  /** Result data */
  result?: unknown;

  /** Error message (if failed) */
  error?: string;

  /** Execution time (ms) */
  executionTime?: number;

  /** Timestamp */
  timestamp: Date | string;
}

// ============================================================================
// Device Provisioning
// ============================================================================

/**
 * Device registration request
 */
export interface DeviceRegistration {
  /** Device serial number */
  serialNumber: string;

  /** Device type */
  deviceType: DeviceType;

  /** Manufacturer */
  manufacturer: string;

  /** Model */
  model: string;

  /** Hardware version */
  hardwareVersion: string;

  /** Firmware version */
  firmwareVersion: string;

  /** MAC address */
  macAddress?: string;

  /** IMEI (for cellular devices) */
  imei?: string;

  /** Public key (for certificate-based auth) */
  publicKey?: string;

  /** Attestation data (from TPM) */
  attestation?: AttestationData;
}

/**
 * TPM attestation data
 */
export interface AttestationData {
  /** Endorsement key certificate */
  ekCert: string;

  /** Attestation identity key */
  aikCert: string;

  /** Quote/attestation signature */
  quote: string;

  /** PCR values */
  pcrValues?: Record<number, string>;

  /** Nonce */
  nonce?: string;
}

/**
 * Provisioning response
 */
export interface ProvisioningResponse {
  /** Assigned device ID */
  deviceId: string;

  /** Device credentials */
  credentials: DeviceCredentials;

  /** Connection endpoint */
  endpoint: string;

  /** Topic configuration */
  topics?: TopicConfiguration;

  /** Initial configuration */
  configuration?: Record<string, unknown>;

  /** Provisioning status */
  status: 'success' | 'pending' | 'rejected';

  /** Expiration time (for credentials) */
  expiresAt?: Date | string;
}

/**
 * Topic configuration
 */
export interface TopicConfiguration {
  /** Telemetry publish topic */
  telemetry: string;

  /** Command subscribe topic */
  commands: string;

  /** Status publish topic */
  status: string;

  /** Configuration subscribe topic */
  configuration?: string;

  /** Firmware update topic */
  firmwareUpdate?: string;
}

// ============================================================================
// Firmware Update
// ============================================================================

/**
 * Firmware update information
 */
export interface FirmwareUpdate {
  /** Update ID */
  id: string;

  /** Device ID or device type */
  target: string;

  /** Firmware version */
  version: string;

  /** Download URL */
  url: string;

  /** File size (bytes) */
  size: number;

  /** Checksum (SHA-256) */
  checksum: string;

  /** Signature (for verification) */
  signature?: string;

  /** Update type */
  type: 'full' | 'delta' | 'patch';

  /** Critical update */
  critical: boolean;

  /** Release notes */
  releaseNotes?: string;

  /** Minimum required version */
  minVersion?: string;

  /** Published date */
  publishedAt: Date | string;
}

/**
 * Firmware update status
 */
export interface FirmwareUpdateStatus {
  /** Update ID */
  updateId: string;

  /** Device ID */
  deviceId: string;

  /** Status */
  status:
    | 'pending'
    | 'downloading'
    | 'downloaded'
    | 'verifying'
    | 'installing'
    | 'completed'
    | 'failed'
    | 'rolled-back';

  /** Progress (0-100) */
  progress: number;

  /** Current step */
  currentStep?: string;

  /** Error message (if failed) */
  error?: string;

  /** Started at */
  startedAt?: Date | string;

  /** Completed at */
  completedAt?: Date | string;
}

// ============================================================================
// Digital Twin
// ============================================================================

/**
 * Device shadow/digital twin
 */
export interface DeviceShadow {
  /** Device ID */
  deviceId: string;

  /** Shadow state */
  state: ShadowState;

  /** Metadata */
  metadata: ShadowMetadata;

  /** Version */
  version: number;

  /** Timestamp */
  timestamp: Date | string;
}

/**
 * Shadow state
 */
export interface ShadowState {
  /** Reported state (from device) */
  reported?: Record<string, unknown>;

  /** Desired state (to device) */
  desired?: Record<string, unknown>;
}

/**
 * Shadow metadata
 */
export interface ShadowMetadata {
  /** Reported metadata */
  reported?: Record<string, PropertyMetadata>;

  /** Desired metadata */
  desired?: Record<string, PropertyMetadata>;
}

/**
 * Property metadata
 */
export interface PropertyMetadata {
  /** Last update timestamp */
  timestamp: Date | string;

  /** Update source */
  source?: string;

  /** Data quality */
  quality?: number;
}

/**
 * Shadow delta
 */
export interface ShadowDelta {
  /** Changed properties */
  state: Record<string, unknown>;

  /** Metadata */
  metadata: Record<string, PropertyMetadata>;

  /** Version */
  version: number;

  /** Timestamp */
  timestamp: Date | string;
}

// ============================================================================
// LoRaWAN
// ============================================================================

/**
 * LoRaWAN device class
 */
export type LoRaWANClass = 'A' | 'B' | 'C';

/**
 * LoRaWAN configuration
 */
export interface LoRaWANConfig {
  /** Device EUI */
  devEUI: string;

  /** Application EUI */
  appEUI: string;

  /** Application key */
  appKey: string;

  /** Device class */
  deviceClass: LoRaWANClass;

  /** Activation mode */
  activationMode: 'OTAA' | 'ABP';

  /** Network session key (for ABP) */
  nwkSKey?: string;

  /** Application session key (for ABP) */
  appSKey?: string;

  /** Device address (for ABP) */
  devAddr?: string;

  /** Data rate */
  dataRate?: number;

  /** TX power (dBm) */
  txPower?: number;

  /** Adaptive data rate */
  adr?: boolean;

  /** Confirmed messages */
  confirmed?: boolean;
}

/**
 * LoRaWAN uplink message
 */
export interface LoRaWANUplink {
  /** Device EUI */
  devEUI: string;

  /** Frame counter */
  fCnt: number;

  /** Port */
  fPort: number;

  /** Payload (encrypted) */
  payload: Buffer;

  /** Gateway metadata */
  metadata: LoRaWANMetadata[];

  /** Timestamp */
  timestamp: Date | string;
}

/**
 * LoRaWAN gateway metadata
 */
export interface LoRaWANMetadata {
  /** Gateway ID */
  gatewayId: string;

  /** RSSI (dBm) */
  rssi: number;

  /** SNR (dB) */
  snr: number;

  /** Frequency (Hz) */
  frequency: number;

  /** Data rate */
  dataRate: string;

  /** Location */
  location?: GeolocationData;
}

// ============================================================================
// Edge Computing
// ============================================================================

/**
 * Edge rule/filter
 */
export interface EdgeRule {
  /** Rule ID */
  id: string;

  /** Rule name */
  name: string;

  /** Enabled */
  enabled: boolean;

  /** Condition */
  condition: RuleCondition;

  /** Actions */
  actions: RuleAction[];

  /** Priority */
  priority?: number;
}

/**
 * Rule condition
 */
export interface RuleCondition {
  /** Field to evaluate */
  field: string;

  /** Operator */
  operator: 'eq' | 'ne' | 'gt' | 'gte' | 'lt' | 'lte' | 'in' | 'contains' | 'matches';

  /** Value to compare */
  value: unknown;

  /** Logical operator for multiple conditions */
  logic?: 'and' | 'or';

  /** Nested conditions */
  conditions?: RuleCondition[];
}

/**
 * Rule action
 */
export interface RuleAction {
  /** Action type */
  type: 'publish' | 'store' | 'forward' | 'alert' | 'execute' | 'discard';

  /** Action parameters */
  parameters: Record<string, unknown>;
}

// ============================================================================
// Data Aggregation
// ============================================================================

/**
 * Aggregation configuration
 */
export interface AggregationConfig {
  /** Window size (seconds) */
  windowSize: number;

  /** Aggregation functions */
  functions: AggregationFunction[];

  /** Group by fields */
  groupBy?: string[];

  /** Output topic/destination */
  output: string;
}

/**
 * Aggregation function
 */
export interface AggregationFunction {
  /** Function name */
  name: 'avg' | 'min' | 'max' | 'sum' | 'count' | 'first' | 'last' | 'stddev';

  /** Field to aggregate */
  field: string;

  /** Output field name */
  outputField: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMM-002 error codes
 */
export enum IoTErrorCode {
  CONNECTION_FAILED = 'IOT001',
  AUTHENTICATION_FAILED = 'IOT002',
  PUBLISH_FAILED = 'IOT003',
  SUBSCRIBE_FAILED = 'IOT004',
  TIMEOUT = 'IOT005',
  INVALID_PAYLOAD = 'IOT006',
  DEVICE_NOT_FOUND = 'IOT007',
  PROVISIONING_FAILED = 'IOT008',
  FIRMWARE_UPDATE_FAILED = 'IOT009',
  NETWORK_ERROR = 'IOT010',
  SECURITY_ERROR = 'IOT011',
  CONFIGURATION_ERROR = 'IOT012',
  RESOURCE_NOT_FOUND = 'IOT013',
  METHOD_NOT_ALLOWED = 'IOT014',
  RATE_LIMIT_EXCEEDED = 'IOT015',
}

/**
 * IoT error
 */
export class IoTError extends Error {
  constructor(
    public code: IoTErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'IoTError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Event callback type
 */
export type EventCallback<T = unknown> = (data: T) => void | Promise<void>;

/**
 * Subscription handle
 */
export interface Subscription {
  /** Topic/resource */
  topic: string;

  /** Unsubscribe */
  unsubscribe(): void | Promise<void>;

  /** Active */
  isActive(): boolean;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * IoT protocol default ports
 */
export const IOT_PORTS = {
  MQTT: 1883,
  MQTT_TLS: 8883,
  MQTT_WS: 9001,
  MQTT_WSS: 9443,
  CoAP: 5683,
  CoAP_DTLS: 5684,
  AMQP: 5672,
  AMQP_TLS: 5671,
  HTTP: 80,
  HTTPS: 443,
} as const;

/**
 * CoAP content formats
 */
export const COAP_CONTENT_FORMATS = {
  TEXT_PLAIN: 0,
  APPLICATION_LINK_FORMAT: 40,
  APPLICATION_XML: 41,
  APPLICATION_OCTET_STREAM: 42,
  APPLICATION_EXI: 47,
  APPLICATION_JSON: 50,
  APPLICATION_CBOR: 60,
  APPLICATION_SENML_JSON: 110,
  APPLICATION_SENSML_CBOR: 112,
} as const;

/**
 * MQTT topic wildcards
 */
export const MQTT_WILDCARDS = {
  SINGLE_LEVEL: '+',
  MULTI_LEVEL: '#',
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  IoTProtocol,
  LPWANTechnology,
  DeviceType,
  PowerMode,
  DeviceStatus,
  IoTDeviceConfig,
  ConnectionConfig,
  DeviceCredentials,
  DeviceMetadata,
  GeolocationData,
  PowerConfig,
  ReportingConfig,
  MQTTQoS,
  MQTTVersion,
  MQTTConfig,
  MQTTWill,
  MQTTPublishOptions,
  MQTTProperties,
  CoAPMethod,
  CoAPMessageType,
  CoAPConfig,
  CoAPRequestOptions,
  CoAPResponse,
  TelemetryMeasurement,
  TelemetryData,
  SignalQuality,
  BatteryStatus,
  DeviceCommand,
  CommandResponse,
  DeviceRegistration,
  AttestationData,
  ProvisioningResponse,
  TopicConfiguration,
  FirmwareUpdate,
  FirmwareUpdateStatus,
  DeviceShadow,
  ShadowState,
  ShadowMetadata,
  PropertyMetadata,
  ShadowDelta,
  LoRaWANClass,
  LoRaWANConfig,
  LoRaWANUplink,
  LoRaWANMetadata,
  EdgeRule,
  RuleCondition,
  RuleAction,
  AggregationConfig,
  AggregationFunction,
  EventCallback,
  Subscription,
};

export { IoTErrorCode, IoTError, IOT_PORTS, COAP_CONTENT_FORMATS, MQTT_WILDCARDS };

/**
 * 弘益人間 (Benefit All Humanity)
 */
