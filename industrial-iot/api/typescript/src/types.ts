/**
 * WIA-IND-027: Industrial IoT - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Industrial IoT Types
// ============================================================================

/**
 * IIoT device protocols
 */
export type IIoTProtocol =
  | 'opcua'
  | 'mqtt'
  | 'modbus-rtu'
  | 'modbus-tcp'
  | 'profinet'
  | 'ethercat'
  | 'bacnet'
  | 'http'
  | 'coap';

/**
 * Device types in industrial environments
 */
export type DeviceType =
  | 'plc'
  | 'sensor'
  | 'actuator'
  | 'robot'
  | 'cnc'
  | 'motor-drive'
  | 'agv'
  | 'scada'
  | 'hmi'
  | 'gateway'
  | 'edge-server';

/**
 * Sensor types
 */
export type SensorType =
  | 'temperature'
  | 'pressure'
  | 'vibration'
  | 'humidity'
  | 'flow'
  | 'level'
  | 'proximity'
  | 'current'
  | 'voltage'
  | 'power'
  | 'speed'
  | 'torque'
  | 'position';

/**
 * Alert severity levels
 */
export type AlertSeverity = 'info' | 'warning' | 'error' | 'critical';

/**
 * Device status
 */
export type DeviceStatus =
  | 'online'
  | 'offline'
  | 'running'
  | 'stopped'
  | 'idle'
  | 'error'
  | 'maintenance';

/**
 * Time aggregation functions
 */
export type AggregationFunction =
  | 'mean'
  | 'sum'
  | 'min'
  | 'max'
  | 'count'
  | 'first'
  | 'last'
  | 'stddev'
  | 'median';

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Main SDK configuration
 */
export interface IndustrialIoTConfig {
  /** Factory/site identifier */
  factoryId: string;

  /** Edge gateway identifier */
  edgeGateway?: string;

  /** API authentication key */
  apiKey?: string;

  /** API endpoint URL */
  apiEndpoint?: string;

  /** Enable debug logging */
  debug?: boolean;

  /** Connection timeout in milliseconds */
  timeout?: number;

  /** Retry configuration */
  retry?: {
    maxAttempts: number;
    backoff: 'linear' | 'exponential';
    initialDelay: number;
  };
}

/**
 * OPC-UA connection configuration
 */
export interface OPCUAConfig {
  /** OPC-UA server endpoint URL */
  endpoint: string;

  /** Security mode */
  securityMode?: 'None' | 'Sign' | 'SignAndEncrypt';

  /** Security policy */
  securityPolicy?:
    | 'None'
    | 'Basic128Rsa15'
    | 'Basic256'
    | 'Basic256Sha256'
    | 'Aes128Sha256RsaOaep'
    | 'Aes256Sha256RsaPss';

  /** Authentication credentials */
  credentials?: {
    type: 'anonymous' | 'username' | 'certificate';
    username?: string;
    password?: string;
    certificateFile?: string;
    privateKeyFile?: string;
  };

  /** Client application name */
  applicationName?: string;

  /** Connection timeout */
  timeout?: number;

  /** Keep-alive interval */
  keepAliveInterval?: number;
}

/**
 * MQTT connection configuration
 */
export interface MQTTConfig {
  /** MQTT broker URL */
  broker: string;

  /** Client identifier */
  clientId?: string;

  /** Authentication */
  username?: string;
  password?: string;

  /** Quality of Service level */
  qos?: 0 | 1 | 2;

  /** Retain messages */
  retain?: boolean;

  /** Clean session */
  cleanSession?: boolean;

  /** Keep-alive interval in seconds */
  keepAlive?: number;

  /** Reconnect period in milliseconds */
  reconnectPeriod?: number;

  /** Topics to subscribe */
  topics?: string[];

  /** Last Will and Testament */
  will?: {
    topic: string;
    payload: string;
    qos: 0 | 1 | 2;
    retain: boolean;
  };
}

/**
 * Modbus connection configuration
 */
export interface ModbusConfig {
  /** Connection type */
  type: 'rtu' | 'tcp';

  /** Serial port (for RTU) */
  port?: string;

  /** Baud rate (for RTU) */
  baudRate?: number;

  /** TCP host (for TCP) */
  host?: string;

  /** TCP port (for TCP) */
  tcpPort?: number;

  /** Slave/unit ID */
  slaveId: number;

  /** Timeout in milliseconds */
  timeout?: number;
}

// ============================================================================
// Device & Sensor Types
// ============================================================================

/**
 * Device information
 */
export interface DeviceInfo {
  /** Unique device identifier */
  deviceId: string;

  /** Device name */
  name: string;

  /** Device type */
  type: DeviceType;

  /** Communication protocol */
  protocol: IIoTProtocol;

  /** Manufacturer */
  manufacturer?: string;

  /** Model number */
  model?: string;

  /** Serial number */
  serialNumber?: string;

  /** Firmware version */
  firmwareVersion?: string;

  /** Location/zone */
  location?: string;

  /** Device status */
  status: DeviceStatus;

  /** Last communication timestamp */
  lastSeen?: Date;

  /** Device metadata */
  metadata?: Record<string, any>;
}

/**
 * Sensor configuration
 */
export interface SensorConfig {
  /** Sensor identifier */
  sensorId: string;

  /** Sensor type */
  type: SensorType;

  /** Measurement unit */
  unit: string;

  /** Sampling rate in Hz */
  sampleRate?: number;

  /** Measurement range */
  range?: {
    min: number;
    max: number;
  };

  /** Calibration offset */
  offset?: number;

  /** Calibration scale factor */
  scale?: number;

  /** Data filtering */
  filter?: {
    type: 'none' | 'deadband' | 'moving-average' | 'kalman';
    params?: Record<string, any>;
  };
}

/**
 * Sensor reading
 */
export interface SensorReading {
  /** Sensor identifier */
  sensorId: string;

  /** Measured value */
  value: number;

  /** Measurement unit */
  unit: string;

  /** Reading timestamp */
  timestamp: Date;

  /** Data quality indicator */
  quality?: 'good' | 'uncertain' | 'bad';

  /** Status code */
  statusCode?: string;
}

// ============================================================================
// OPC-UA Types
// ============================================================================

/**
 * OPC-UA node identifier
 */
export interface OPCUANodeId {
  /** Namespace index */
  namespace: number;

  /** Identifier type */
  identifierType: 'string' | 'numeric' | 'guid' | 'opaque';

  /** Identifier value */
  identifier: string | number;
}

/**
 * OPC-UA data value
 */
export interface OPCUADataValue {
  /** Node identifier */
  nodeId: string;

  /** Value */
  value: any;

  /** Data type */
  dataType?: string;

  /** Status code */
  statusCode: string;

  /** Source timestamp */
  sourceTimestamp: Date;

  /** Server timestamp */
  serverTimestamp: Date;
}

/**
 * OPC-UA subscription options
 */
export interface OPCUASubscriptionOptions {
  /** Publishing interval in milliseconds */
  publishingInterval: number;

  /** Lifetime count */
  lifetimeCount?: number;

  /** Max keep-alive count */
  maxKeepAliveCount?: number;

  /** Max notifications per publish */
  maxNotificationsPerPublish?: number;

  /** Priority */
  priority?: number;
}

/**
 * OPC-UA monitored item
 */
export interface OPCUAMonitoredItem {
  /** Node identifier */
  nodeId: string;

  /** Sampling interval in milliseconds */
  samplingInterval: number;

  /** Queue size */
  queueSize?: number;

  /** Discard oldest */
  discardOldest?: boolean;

  /** Data change filter */
  filter?: {
    deadbandType: 'none' | 'absolute' | 'percent';
    deadbandValue?: number;
  };
}

// ============================================================================
// Time-Series Types
// ============================================================================

/**
 * Time-series data point
 */
export interface TimeSeriesPoint {
  /** Measurement name */
  measurement: string;

  /** Tags for indexing */
  tags: Record<string, string>;

  /** Field values */
  fields: Record<string, number | string | boolean>;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Time-series query options
 */
export interface TimeSeriesQueryOptions {
  /** Measurement name */
  measurement: string;

  /** Start time */
  start: Date | string;

  /** End time */
  end: Date | string;

  /** Tag filters */
  filter?: Record<string, string>;

  /** Fields to select */
  fields?: string[];

  /** Aggregation function */
  aggregation?: AggregationFunction;

  /** Aggregation interval */
  interval?: string;

  /** Group by tags */
  groupBy?: string[];

  /** Limit results */
  limit?: number;

  /** Order by */
  orderBy?: 'time' | 'value';

  /** Sort direction */
  order?: 'asc' | 'desc';
}

/**
 * Time-series query result
 */
export interface TimeSeriesResult {
  /** Series name */
  series: string;

  /** Tags */
  tags: Record<string, string>;

  /** Data points */
  values: Array<{
    timestamp: Date;
    [field: string]: any;
  }>;
}

// ============================================================================
// Alert & Notification Types
// ============================================================================

/**
 * Alert rule configuration
 */
export interface AlertRule {
  /** Rule identifier */
  ruleId: string;

  /** Rule name */
  name: string;

  /** Metric to monitor */
  metric: string;

  /** Condition operator */
  operator: '>' | '<' | '>=' | '<=' | '==' | '!=';

  /** Threshold value */
  threshold: number;

  /** Alert severity */
  severity: AlertSeverity;

  /** Duration before triggering (seconds) */
  duration?: number;

  /** Cooldown period (seconds) */
  cooldown?: number;

  /** Notification channels */
  channels?: Array<'sms' | 'email' | 'slack' | 'webhook'>;

  /** Rule enabled status */
  enabled: boolean;

  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Alert event
 */
export interface AlertEvent {
  /** Alert identifier */
  alertId: string;

  /** Rule that triggered */
  ruleId: string;

  /** Alert severity */
  severity: AlertSeverity;

  /** Alert message */
  message: string;

  /** Device/sensor identifier */
  source: string;

  /** Current metric value */
  value: number;

  /** Threshold that was exceeded */
  threshold: number;

  /** Timestamp */
  timestamp: Date;

  /** Alert status */
  status: 'active' | 'acknowledged' | 'resolved';

  /** Acknowledged by */
  acknowledgedBy?: string;

  /** Acknowledged at */
  acknowledgedAt?: Date;

  /** Resolution notes */
  resolutionNotes?: string;
}

/**
 * Notification configuration
 */
export interface NotificationConfig {
  /** Notification channels */
  channels: {
    sms?: {
      enabled: boolean;
      phoneNumbers: string[];
      provider?: 'twilio' | 'aws-sns';
    };
    email?: {
      enabled: boolean;
      recipients: string[];
      smtpConfig?: {
        host: string;
        port: number;
        secure: boolean;
        auth: { user: string; pass: string };
      };
    };
    slack?: {
      enabled: boolean;
      webhookUrl: string;
      channel: string;
    };
    webhook?: {
      enabled: boolean;
      url: string;
      method: 'POST' | 'PUT';
      headers?: Record<string, string>;
    };
  };

  /** Severity-based routing */
  routing?: {
    [severity in AlertSeverity]?: Array<'sms' | 'email' | 'slack' | 'webhook'>;
  };

  /** Rate limiting */
  rateLimit?: {
    maxPerHour: number;
    maxPerDay: number;
  };
}

// ============================================================================
// Manufacturing Metrics Types
// ============================================================================

/**
 * OEE (Overall Equipment Effectiveness) metrics
 */
export interface OEEMetrics {
  /** Availability percentage */
  availability: number;

  /** Performance percentage */
  performance: number;

  /** Quality percentage */
  quality: number;

  /** Overall OEE percentage */
  oee: number;

  /** Planned production time (minutes) */
  plannedTime: number;

  /** Actual operating time (minutes) */
  operatingTime: number;

  /** Downtime (minutes) */
  downtime: number;

  /** Target output (units) */
  targetOutput: number;

  /** Actual output (units) */
  actualOutput: number;

  /** Good units */
  goodUnits: number;

  /** Defective units */
  defectiveUnits: number;

  /** Calculation timestamp */
  timestamp: Date;
}

/**
 * Production metrics
 */
export interface ProductionMetrics {
  /** Production line identifier */
  lineId: string;

  /** Units produced */
  unitsProduced: number;

  /** Production rate (units/hour) */
  productionRate: number;

  /** Cycle time (seconds) */
  cycleTime: number;

  /** Throughput */
  throughput: number;

  /** Scrap rate percentage */
  scrapRate: number;

  /** First pass yield percentage */
  firstPassYield: number;

  /** OEE metrics */
  oee: OEEMetrics;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Equipment health metrics
 */
export interface EquipmentHealth {
  /** Equipment identifier */
  equipmentId: string;

  /** Health score (0-100) */
  healthScore: number;

  /** MTBF (Mean Time Between Failures) in hours */
  mtbf: number;

  /** MTTR (Mean Time To Repair) in hours */
  mttr: number;

  /** Remaining Useful Life in hours */
  remainingUsefulLife?: number;

  /** Failure probability (0-1) */
  failureProbability?: number;

  /** Maintenance status */
  maintenanceStatus: 'ok' | 'warning' | 'critical';

  /** Last maintenance date */
  lastMaintenance?: Date;

  /** Next scheduled maintenance */
  nextMaintenance?: Date;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Digital Twin Types
// ============================================================================

/**
 * Digital twin configuration
 */
export interface DigitalTwinConfig {
  /** Asset identifier */
  assetId: string;

  /** Asset type */
  type: string;

  /** Manufacturer model */
  model?: string;

  /** Physical location */
  location?: string;

  /** Associated sensors */
  sensors: Array<{
    id: string;
    type: SensorType;
    location: string;
  }>;

  /** Simulation model */
  simulationModel?: {
    type: 'physics' | 'statistical' | 'ml';
    parameters: Record<string, any>;
  };

  /** Update frequency in milliseconds */
  updateInterval?: number;

  /** Enable predictive analytics */
  enablePredictive?: boolean;
}

/**
 * Digital twin state
 */
export interface DigitalTwinState {
  /** Asset identifier */
  assetId: string;

  /** Current sensor values */
  sensorData: Record<string, number>;

  /** Calculated properties */
  calculatedProperties: Record<string, number>;

  /** Predicted values */
  predictions?: Record<string, number>;

  /** Health indicators */
  health: {
    overall: number;
    subsystems: Record<string, number>;
  };

  /** Anomalies detected */
  anomalies: Array<{
    sensor: string;
    value: number;
    expected: number;
    deviation: number;
  }>;

  /** Last update timestamp */
  lastUpdate: Date;
}

/**
 * Predictive maintenance result
 */
export interface PredictiveMaintenanceResult {
  /** Asset identifier */
  assetId: string;

  /** Failure mode */
  failureMode: string;

  /** Probability of failure (0-1) */
  probability: number;

  /** Remaining useful life (hours) */
  remainingUsefulLife: number;

  /** Confidence interval */
  confidence: {
    lower: number;
    upper: number;
  };

  /** Recommended actions */
  recommendations: Array<{
    action: string;
    priority: 'low' | 'medium' | 'high';
    estimatedCost?: number;
  }>;

  /** Prediction timestamp */
  timestamp: Date;
}

// ============================================================================
// MES/ERP Integration Types
// ============================================================================

/**
 * Production order
 */
export interface ProductionOrder {
  /** Order identifier */
  orderId: string;

  /** Product code */
  productCode: string;

  /** Quantity to produce */
  quantity: number;

  /** Production line */
  lineId?: string;

  /** Order status */
  status: 'pending' | 'scheduled' | 'in-progress' | 'completed' | 'cancelled';

  /** Priority */
  priority: 'low' | 'normal' | 'high' | 'urgent';

  /** Due date */
  dueDate: Date;

  /** Scheduled start */
  scheduledStart?: Date;

  /** Actual start */
  actualStart?: Date;

  /** Actual completion */
  actualCompletion?: Date;

  /** Units completed */
  unitsCompleted?: number;

  /** Order metadata */
  metadata?: Record<string, any>;
}

/**
 * Material tracking
 */
export interface MaterialTracking {
  /** Batch/lot number */
  batchNumber: string;

  /** Material code */
  materialCode: string;

  /** Quantity */
  quantity: number;

  /** Unit of measure */
  unit: string;

  /** Location */
  location: string;

  /** Status */
  status: 'available' | 'reserved' | 'in-use' | 'consumed' | 'expired';

  /** Expiration date */
  expirationDate?: Date;

  /** Traceability data */
  traceability?: {
    supplier: string;
    receivedDate: Date;
    certificate?: string;
  };
}

// ============================================================================
// Edge Computing Types
// ============================================================================

/**
 * Edge gateway configuration
 */
export interface EdgeGatewayConfig {
  /** Gateway identifier */
  gatewayId: string;

  /** Gateway location */
  location: string;

  /** Connected devices */
  devices: string[];

  /** Local processing rules */
  processingRules?: Array<{
    ruleId: string;
    condition: string;
    action: string;
  }>;

  /** Data buffering */
  buffering?: {
    enabled: boolean;
    maxSize: number; // MB
    flushInterval: number; // seconds
  };

  /** Offline mode */
  offlineMode?: {
    enabled: boolean;
    maxStorageDays: number;
  };
}

/**
 * Edge analytics result
 */
export interface EdgeAnalyticsResult {
  /** Analysis type */
  analysisType: 'anomaly' | 'prediction' | 'aggregation' | 'correlation';

  /** Input data reference */
  dataSource: string;

  /** Analysis result */
  result: Record<string, any>;

  /** Confidence score (0-1) */
  confidence?: number;

  /** Processing time (ms) */
  processingTime: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';

/**
 * 弘益人間 (Benefit All Humanity)
 */
