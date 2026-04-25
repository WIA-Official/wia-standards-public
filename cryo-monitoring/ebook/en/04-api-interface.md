# Chapter 4: API Interface Specifications
## REST, GraphQL, and WebSocket APIs for Cryogenic Monitoring

**弘익人間 (Hongik Ingan)** - Universal APIs for universal access

---

## 1. API Overview

The WIA Cryo Monitoring Standard provides three complementary API interfaces:

- **REST API:** Traditional HTTP-based API for CRUD operations and queries
- **GraphQL API:** Flexible query language for complex data requirements
- **WebSocket API:** Real-time streaming for live monitoring and alerts

All APIs support:
- OAuth 2.0 and API key authentication
- Rate limiting and throttling
- Versioning and backward compatibility
- Comprehensive error handling
- Audit logging

---

## 2. REST API

### 2.1 REST API Architecture

```typescript
/**
 * WIA Cryo Monitoring REST API
 *
 * RESTful HTTP API following OpenAPI 3.0 specification
 */

import express, { Request, Response, NextFunction } from 'express';
import { z } from 'zod';

/**
 * API configuration
 */
export interface RestApiConfig {
  baseUrl: string;
  version: string;
  port: number;
  authentication: {
    type: 'oauth2' | 'api-key' | 'jwt';
    enabled: boolean;
  };
  rateLimit: {
    enabled: boolean;
    windowMs: number;
    maxRequests: number;
  };
  cors: {
    enabled: boolean;
    origins: string[];
  };
}

/**
 * Standard API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata: {
    timestamp: string;
    requestId: string;
    version: string;
  };
  pagination?: {
    page: number;
    pageSize: number;
    totalItems: number;
    totalPages: number;
    hasNext: boolean;
    hasPrevious: boolean;
  };
}

/**
 * Query parameters schema
 */
export const QueryParamsSchema = z.object({
  page: z.coerce.number().int().positive().default(1),
  pageSize: z.coerce.number().int().positive().max(1000).default(100),
  sortBy: z.string().optional(),
  sortOrder: z.enum(['asc', 'desc']).default('desc'),
  filter: z.string().optional()
});

/**
 * REST API Implementation
 */
export class CryoMonitoringRestApi {
  private app: express.Application;
  private config: RestApiConfig;

  constructor(config: RestApiConfig) {
    this.app = express();
    this.config = config;
    this.setupMiddleware();
    this.setupRoutes();
  }

  /**
   * Setup middleware
   */
  private setupMiddleware(): void {
    this.app.use(express.json());
    this.app.use(this.requestLogger);
    this.app.use(this.errorHandler);
  }

  /**
   * Request logging middleware
   */
  private requestLogger(req: Request, res: Response, next: NextFunction): void {
    const requestId = crypto.randomUUID();
    req.headers['x-request-id'] = requestId;

    console.log({
      requestId,
      method: req.method,
      path: req.path,
      query: req.query,
      timestamp: new Date().toISOString()
    });

    next();
  }

  /**
   * Error handling middleware
   */
  private errorHandler(
    err: Error,
    req: Request,
    res: Response,
    next: NextFunction
  ): void {
    console.error('API Error:', err);

    const response: ApiResponse<null> = {
      success: false,
      error: {
        code: 'INTERNAL_ERROR',
        message: err.message
      },
      metadata: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string || 'unknown',
        version: this.config.version
      }
    };

    res.status(500).json(response);
  }

  /**
   * Setup routes
   */
  private setupRoutes(): void {
    const router = express.Router();

    // Sensor readings endpoints
    router.get('/readings', this.getReadings.bind(this));
    router.get('/readings/:readingId', this.getReading.bind(this));
    router.post('/readings', this.createReading.bind(this));
    router.post('/readings/batch', this.createReadingsBatch.bind(this));

    // Equipment endpoints
    router.get('/equipment', this.getEquipment.bind(this));
    router.get('/equipment/:equipmentId', this.getEquipmentById.bind(this));
    router.get('/equipment/:equipmentId/status', this.getEquipmentStatus.bind(this));
    router.post('/equipment', this.createEquipment.bind(this));
    router.put('/equipment/:equipmentId', this.updateEquipment.bind(this));
    router.delete('/equipment/:equipmentId', this.deleteEquipment.bind(this));

    // Sensor endpoints
    router.get('/sensors', this.getSensors.bind(this));
    router.get('/sensors/:sensorId', this.getSensor.bind(this));
    router.post('/sensors', this.createSensor.bind(this));
    router.put('/sensors/:sensorId', this.updateSensor.bind(this));
    router.delete('/sensors/:sensorId', this.deleteSensor.bind(this));
    router.post('/sensors/:sensorId/calibrate', this.calibrateSensor.bind(this));

    // Alert endpoints
    router.get('/alerts', this.getAlerts.bind(this));
    router.get('/alerts/:alertId', this.getAlert.bind(this));
    router.post('/alerts/:alertId/acknowledge', this.acknowledgeAlert.bind(this));
    router.post('/alerts/:alertId/resolve', this.resolveAlert.bind(this));

    // Dashboard endpoints
    router.get('/dashboard/metrics', this.getDashboardMetrics.bind(this));
    router.get('/dashboard/summary', this.getDashboardSummary.bind(this));

    // Compliance endpoints
    router.get('/compliance/audit-log', this.getAuditLog.bind(this));
    router.get('/compliance/reports', this.getComplianceReports.bind(this));
    router.post('/compliance/reports/generate', this.generateComplianceReport.bind(this));

    // Analytics endpoints
    router.get('/analytics/temperature-trends', this.getTemperatureTrends.bind(this));
    router.get('/analytics/alert-statistics', this.getAlertStatistics.bind(this));
    router.get('/analytics/equipment-performance', this.getEquipmentPerformance.bind(this));

    this.app.use(`/api/${this.config.version}`, router);
  }

  /**
   * GET /readings
   * Retrieve sensor readings with filtering and pagination
   */
  private async getReadings(req: Request, res: Response): Promise<void> {
    try {
      const params = QueryParamsSchema.parse(req.query);

      // Filter parameters
      const filters = {
        equipmentId: req.query.equipmentId as string | undefined,
        sensorId: req.query.sensorId as string | undefined,
        parameter: req.query.parameter as string | undefined,
        startDate: req.query.startDate ? new Date(req.query.startDate as string) : undefined,
        endDate: req.query.endDate ? new Date(req.query.endDate as string) : undefined,
        minValue: req.query.minValue ? parseFloat(req.query.minValue as string) : undefined,
        maxValue: req.query.maxValue ? parseFloat(req.query.maxValue as string) : undefined
      };

      // Database query (example)
      const readings = await this.queryReadings(filters, params);
      const total = await this.countReadings(filters);

      const response: ApiResponse<any> = {
        success: true,
        data: readings,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.headers['x-request-id'] as string,
          version: this.config.version
        },
        pagination: {
          page: params.page,
          pageSize: params.pageSize,
          totalItems: total,
          totalPages: Math.ceil(total / params.pageSize),
          hasNext: params.page * params.pageSize < total,
          hasPrevious: params.page > 1
        }
      };

      res.json(response);
    } catch (error) {
      this.handleError(error, req, res);
    }
  }

  /**
   * GET /readings/:readingId
   * Retrieve a specific sensor reading
   */
  private async getReading(req: Request, res: Response): Promise<void> {
    try {
      const { readingId } = req.params;

      // Database query
      const reading = await this.findReadingById(readingId);

      if (!reading) {
        const response: ApiResponse<null> = {
          success: false,
          error: {
            code: 'NOT_FOUND',
            message: `Reading ${readingId} not found`
          },
          metadata: {
            timestamp: new Date().toISOString(),
            requestId: req.headers['x-request-id'] as string,
            version: this.config.version
          }
        };
        res.status(404).json(response);
        return;
      }

      const response: ApiResponse<any> = {
        success: true,
        data: reading,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.headers['x-request-id'] as string,
          version: this.config.version
        }
      };

      res.json(response);
    } catch (error) {
      this.handleError(error, req, res);
    }
  }

  /**
   * POST /readings
   * Create a new sensor reading
   */
  private async createReading(req: Request, res: Response): Promise<void> {
    try {
      // Validate request body
      const reading = SensorReadingSchema.parse(req.body);

      // Save to database
      const saved = await this.saveReading(reading);

      // Check for alert conditions
      await this.checkAlertConditions(saved);

      const response: ApiResponse<any> = {
        success: true,
        data: saved,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.headers['x-request-id'] as string,
          version: this.config.version
        }
      };

      res.status(201).json(response);
    } catch (error) {
      this.handleError(error, req, res);
    }
  }

  /**
   * POST /readings/batch
   * Create multiple sensor readings in batch
   */
  private async createReadingsBatch(req: Request, res: Response): Promise<void> {
    try {
      const batch = SensorReadingBatchSchema.parse(req.body);

      // Validate all readings
      const validationResults = CryoDataValidator.validateBatch(
        batch.readings,
        SensorReadingSchema
      );

      if (validationResults.invalid.length > 0) {
        const response: ApiResponse<any> = {
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Some readings failed validation',
            details: validationResults.invalid
          },
          metadata: {
            timestamp: new Date().toISOString(),
            requestId: req.headers['x-request-id'] as string,
            version: this.config.version
          }
        };
        res.status(400).json(response);
        return;
      }

      // Save all readings
      const saved = await this.saveReadingsBatch(validationResults.valid);

      const response: ApiResponse<any> = {
        success: true,
        data: {
          saved: saved.length,
          readings: saved
        },
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.headers['x-request-id'] as string,
          version: this.config.version
        }
      };

      res.status(201).json(response);
    } catch (error) {
      this.handleError(error, req, res);
    }
  }

  /**
   * GET /equipment/:equipmentId/status
   * Get current status of equipment
   */
  private async getEquipmentStatus(req: Request, res: Response): Promise<void> {
    try {
      const { equipmentId } = req.params;

      // Get latest status
      const status = await this.getLatestEquipmentStatus(equipmentId);

      // Get current measurements
      const measurements = await this.getCurrentMeasurements(equipmentId);

      // Get active alerts
      const alerts = await this.getActiveAlerts(equipmentId);

      const response: ApiResponse<any> = {
        success: true,
        data: {
          status,
          measurements,
          alerts,
          timestamp: new Date().toISOString()
        },
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.headers['x-request-id'] as string,
          version: this.config.version
        }
      };

      res.json(response);
    } catch (error) {
      this.handleError(error, req, res);
    }
  }

  /**
   * POST /alerts/:alertId/acknowledge
   * Acknowledge an alert
   */
  private async acknowledgeAlert(req: Request, res: Response): Promise<void> {
    try {
      const { alertId } = req.params;
      const { userId, comments } = req.body;

      // Update alert
      const alert = await this.updateAlertStatus(alertId, {
        status: 'acknowledged',
        acknowledgedBy: userId,
        acknowledgedAt: new Date(),
        comments
      });

      // Create audit log entry
      await this.createAuditLogEntry({
        action: 'alert-acknowledge',
        userId,
        alertId,
        timestamp: new Date()
      });

      const response: ApiResponse<any> = {
        success: true,
        data: alert,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.headers['x-request-id'] as string,
          version: this.config.version
        }
      };

      res.json(response);
    } catch (error) {
      this.handleError(error, req, res);
    }
  }

  /**
   * GET /analytics/temperature-trends
   * Get temperature trend analytics
   */
  private async getTemperatureTrends(req: Request, res: Response): Promise<void> {
    try {
      const equipmentId = req.query.equipmentId as string;
      const timeRange = req.query.timeRange as string || '24h';
      const aggregation = req.query.aggregation as string || '1hour';

      const trends = await this.calculateTemperatureTrends(
        equipmentId,
        timeRange,
        aggregation
      );

      const response: ApiResponse<any> = {
        success: true,
        data: trends,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.headers['x-request-id'] as string,
          version: this.config.version
        }
      };

      res.json(response);
    } catch (error) {
      this.handleError(error, req, res);
    }
  }

  // Helper methods (placeholder implementations)
  private async queryReadings(filters: any, params: any): Promise<any[]> {
    // Database implementation
    return [];
  }

  private async countReadings(filters: any): Promise<number> {
    return 0;
  }

  private async findReadingById(id: string): Promise<any | null> {
    return null;
  }

  private async saveReading(reading: any): Promise<any> {
    return reading;
  }

  private async saveReadingsBatch(readings: any[]): Promise<any[]> {
    return readings;
  }

  private async checkAlertConditions(reading: any): Promise<void> {
    // Alert checking logic
  }

  private async getLatestEquipmentStatus(equipmentId: string): Promise<any> {
    return {};
  }

  private async getCurrentMeasurements(equipmentId: string): Promise<any> {
    return {};
  }

  private async getActiveAlerts(equipmentId: string): Promise<any[]> {
    return [];
  }

  private async updateAlertStatus(alertId: string, update: any): Promise<any> {
    return {};
  }

  private async createAuditLogEntry(entry: any): Promise<void> {
    // Audit log creation
  }

  private async calculateTemperatureTrends(
    equipmentId: string,
    timeRange: string,
    aggregation: string
  ): Promise<any> {
    return {};
  }

  private handleError(error: any, req: Request, res: Response): void {
    console.error('Request error:', error);

    const statusCode = error.statusCode || 500;
    const response: ApiResponse<null> = {
      success: false,
      error: {
        code: error.code || 'INTERNAL_ERROR',
        message: error.message || 'An unexpected error occurred'
      },
      metadata: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string || 'unknown',
        version: this.config.version
      }
    };

    res.status(statusCode).json(response);
  }

  /**
   * Start the API server
   */
  public start(): void {
    this.app.listen(this.config.port, () => {
      console.log(`Cryo Monitoring REST API running on port ${this.config.port}`);
      console.log(`Base URL: ${this.config.baseUrl}/api/${this.config.version}`);
    });
  }
}

/**
 * Example usage
 */
export const exampleRestApiConfig: RestApiConfig = {
  baseUrl: 'https://api.cryo-monitoring.wia.org',
  version: 'v1',
  port: 3000,
  authentication: {
    type: 'oauth2',
    enabled: true
  },
  rateLimit: {
    enabled: true,
    windowMs: 60000, // 1 minute
    maxRequests: 100
  },
  cors: {
    enabled: true,
    origins: ['https://dashboard.cryo-monitoring.wia.org']
  }
};
```

---

## 3. GraphQL API

### 3.1 GraphQL Schema

```typescript
/**
 * WIA Cryo Monitoring GraphQL API
 *
 * Flexible query interface for complex data requirements
 */

import { GraphQLServer } from 'graphql-yoga';
import { makeExecutableSchema } from '@graphql-tools/schema';

/**
 * GraphQL Type Definitions
 */
export const typeDefs = `
  # Scalar types
  scalar DateTime
  scalar JSON

  # Sensor Reading Type
  type SensorReading {
    readingId: ID!
    sensorId: String!
    equipmentId: String!
    facilityId: String!
    timestamp: DateTime!
    receivedAt: DateTime
    processedAt: DateTime

    measurement: Measurement!
    sensorStatus: SensorStatus
    location: Location
    alert: ReadingAlert
    compliance: ComplianceInfo

    metadata: JSON
    schemaVersion: String!
  }

  type Measurement {
    parameter: MeasurementParameter!
    value: Float!
    unit: String!
    statistics: MeasurementStatistics
    quality: QualityInfo
  }

  enum MeasurementParameter {
    TEMPERATURE
    PRESSURE
    LIQUID_LEVEL
    HUMIDITY
    DOOR_STATUS
    VIBRATION
    POWER_STATUS
    CO2_LEVEL
    O2_LEVEL
  }

  type MeasurementStatistics {
    min: Float
    max: Float
    average: Float
    stddev: Float
    sampleCount: Int
  }

  type QualityInfo {
    isValid: Boolean!
    confidence: Float
    flags: [String!]
    calibrationStatus: CalibrationStatus
  }

  enum CalibrationStatus {
    CALIBRATED
    DUE
    OVERDUE
    UNKNOWN
  }

  type SensorStatus {
    health: SensorHealth!
    batteryLevel: Float
    signalStrength: Float
    lastCalibration: DateTime
    errors: [String!]
  }

  enum SensorHealth {
    HEALTHY
    WARNING
    CRITICAL
    OFFLINE
  }

  type Location {
    compartment: String
    position: String
    coordinates: Coordinates
  }

  type Coordinates {
    x: Float
    y: Float
    z: Float
  }

  type ReadingAlert {
    triggered: Boolean!
    alertIds: [String!]
    severity: AlertSeverity
  }

  enum AlertSeverity {
    CRITICAL
    HIGH
    MEDIUM
    LOW
    INFO
  }

  type ComplianceInfo {
    validated: Boolean!
    validatedBy: String
    validatedAt: DateTime
    signature: String
    witnessSignature: String
    comments: String
  }

  # Alert Type
  type Alert {
    alertId: ID!
    alertRuleId: String!
    correlationId: String

    source: AlertSource!
    classification: AlertClassification!
    timing: AlertTiming!
    condition: AlertCondition!
    state: AlertState!

    notifications: [Notification!]
    actions: [AlertAction!]
    impact: AlertImpact

    metadata: JSON
  }

  type AlertSource {
    sensorId: String!
    equipmentId: String!
    facilityId: String!
    readingId: String
  }

  type AlertClassification {
    type: AlertType!
    severity: AlertSeverity!
    category: AlertCategory!
  }

  enum AlertType {
    THRESHOLD_EXCEEDED
    THRESHOLD_BELOW
    RATE_OF_CHANGE
    ANOMALY_DETECTED
    SENSOR_FAILURE
    COMMUNICATION_LOSS
    CALIBRATION_DUE
    MAINTENANCE_DUE
    BATTERY_LOW
    CUSTOM
  }

  enum AlertCategory {
    TEMPERATURE
    PRESSURE
    LEVEL
    EQUIPMENT
    SENSOR
    COMMUNICATION
    COMPLIANCE
    MAINTENANCE
  }

  type AlertTiming {
    detected: DateTime!
    triggered: DateTime!
    acknowledged: DateTime
    resolved: DateTime
    durationSeconds: Int
  }

  type AlertCondition {
    parameter: String!
    currentValue: Float!
    threshold: Float!
    operator: ComparisonOperator!
    unit: String!
    description: String!
  }

  enum ComparisonOperator {
    GT
    LT
    EQ
    NE
    GTE
    LTE
  }

  type AlertState {
    status: AlertStatus!
    acknowledgedBy: String
    acknowledgedAt: DateTime
    resolvedBy: String
    resolvedAt: DateTime
    resolution: String
    escalationLevel: Int
  }

  enum AlertStatus {
    ACTIVE
    ACKNOWLEDGED
    RESOLVED
    SUPPRESSED
    ESCALATED
    FALSE_POSITIVE
    CANCELLED
  }

  type Notification {
    notificationId: ID!
    channel: NotificationChannel!
    recipient: String!
    sentAt: DateTime!
    deliveredAt: DateTime
    status: NotificationStatus!
    retryCount: Int!
  }

  enum NotificationChannel {
    EMAIL
    SMS
    PHONE
    PUSH
    WEBHOOK
    PAGER
  }

  enum NotificationStatus {
    SENT
    DELIVERED
    FAILED
    BOUNCED
  }

  type AlertAction {
    actionId: ID!
    actionType: String!
    executedAt: DateTime!
    executedBy: String!
    status: ActionStatus!
    result: String
    error: String
  }

  enum ActionStatus {
    PENDING
    SUCCESS
    FAILED
  }

  type AlertImpact {
    affectedEquipment: [String!]
    affectedSamples: Int
    estimatedSampleValue: Float
    riskLevel: RiskLevel
    complianceImpact: Boolean
  }

  enum RiskLevel {
    CRITICAL
    HIGH
    MEDIUM
    LOW
  }

  # Equipment Type
  type Equipment {
    equipmentId: ID!
    equipmentType: EquipmentType!
    manufacturer: String!
    model: String!
    serialNumber: String!

    location: EquipmentLocation!
    capacity: EquipmentCapacity!
    operatingParameters: OperatingParameters!
    maintenance: MaintenanceInfo!
    installation: InstallationInfo!

    # Related data
    sensors: [Sensor!]!
    currentStatus: EquipmentStatus
    recentReadings(limit: Int = 100): [SensorReading!]!
    activeAlerts: [Alert!]!
  }

  enum EquipmentType {
    LIQUID_NITROGEN_TANK
    ULTRA_LOW_FREEZER
    CONTROLLED_RATE_FREEZER
    VAPOR_PHASE_STORAGE
    LIQUID_PHASE_STORAGE
    DRY_ICE_STORAGE
    CRYOGENIC_REFRIGERATOR
  }

  type EquipmentLocation {
    facility: String!
    building: String!
    room: String!
    position: String!
  }

  type EquipmentCapacity {
    volume: Float!
    compartments: Int!
    maxSamples: Int!
  }

  type OperatingParameters {
    targetTemperature: Float!
    temperatureRange: TemperatureRange!
    pressureRange: PressureRange
  }

  type TemperatureRange {
    min: Float!
    max: Float!
    alarm: AlarmThresholds!
  }

  type AlarmThresholds {
    low: Float!
    high: Float!
  }

  type PressureRange {
    min: Float!
    max: Float!
    unit: PressureUnit!
  }

  enum PressureUnit {
    PSI
    BAR
    KPA
  }

  type MaintenanceInfo {
    lastService: DateTime!
    nextService: DateTime!
    serviceInterval: Int!
    maintenanceProvider: String!
  }

  type InstallationInfo {
    installedDate: DateTime!
    warrantyExpiry: DateTime!
    certifications: [String!]!
  }

  type EquipmentStatus {
    statusId: ID!
    equipmentId: String!
    timestamp: DateTime!

    operational: OperationalStatus!
    currentMeasurements: CurrentMeasurements!
    performance: PerformanceMetrics
    health: HealthIndicators!
    alarms: AlarmsSummary
    maintenance: MaintenanceStatus
    environment: EnvironmentalConditions
  }

  type OperationalStatus {
    status: OperationalState!
    uptime: Float!
    lastStartup: DateTime
    lastShutdown: DateTime
    operatingMode: OperatingMode
  }

  enum OperationalState {
    OPERATIONAL
    STANDBY
    MAINTENANCE
    ALARM
    OFFLINE
    ERROR
  }

  enum OperatingMode {
    NORMAL
    BACKUP
    EMERGENCY
    TEST
    CALIBRATION
  }

  type CurrentMeasurements {
    temperature: TemperatureMeasurement
    pressure: PressureMeasurement
    liquidLevel: LiquidLevelMeasurement
    power: PowerMeasurement
  }

  type TemperatureMeasurement {
    value: Float!
    unit: String!
    setpoint: Float
    deviation: Float
  }

  type PressureMeasurement {
    value: Float!
    unit: String!
    setpoint: Float
  }

  type LiquidLevelMeasurement {
    value: Float!
    unit: String!
    capacity: Float
    timeToRefill: Float
  }

  type PowerMeasurement {
    voltage: Float
    current: Float
    consumption: Float
    frequency: Float
  }

  type PerformanceMetrics {
    efficiency: Float
    cycleCount: Int
    averageCycleTime: Float
    temperatureStability: TemperatureStability
    energy: EnergyMetrics
  }

  type TemperatureStability {
    deviation: Float
    uniformity: Float
    recoveryTime: Float
  }

  type EnergyMetrics {
    dailyConsumption: Float
    comparedToBaseline: Float
    trend: EnergyTrend
  }

  enum EnergyTrend {
    INCREASING
    STABLE
    DECREASING
  }

  type HealthIndicators {
    overall: HealthStatus!
    score: Float
    components: [ComponentHealth!]
    predictions: [HealthPrediction!]
  }

  enum HealthStatus {
    EXCELLENT
    GOOD
    FAIR
    POOR
    CRITICAL
  }

  type ComponentHealth {
    component: String!
    status: ComponentStatus!
    lastMaintenance: DateTime
    nextMaintenance: DateTime
    remainingLife: Float
  }

  enum ComponentStatus {
    HEALTHY
    WARNING
    CRITICAL
    FAILED
  }

  type HealthPrediction {
    type: PredictionType!
    component: String!
    confidence: Float!
    timeframe: String!
    recommendation: String!
  }

  enum PredictionType {
    FAILURE
    MAINTENANCE
    PERFORMANCE_DEGRADATION
  }

  type AlarmsSummary {
    active: [ActiveAlarm!]!
    count: AlarmCount!
  }

  type ActiveAlarm {
    alarmId: String!
    type: String!
    severity: AlertSeverity!
    triggeredAt: DateTime!
    message: String!
  }

  type AlarmCount {
    critical: Int!
    high: Int!
    medium: Int!
    low: Int!
  }

  type MaintenanceStatus {
    lastPerformed: DateTime
    nextScheduled: DateTime
    daysUntilDue: Int
    overdue: Boolean!
    history: [MaintenanceRecord!]
  }

  type MaintenanceRecord {
    date: DateTime!
    type: MaintenanceType!
    performedBy: String!
    workOrder: String
    notes: String
  }

  enum MaintenanceType {
    PREVENTIVE
    CORRECTIVE
    CALIBRATION
    INSPECTION
  }

  type EnvironmentalConditions {
    ambientTemperature: Float
    ambientHumidity: Float
    roomConditions: RoomConditionStatus
  }

  enum RoomConditionStatus {
    NORMAL
    WARNING
    ALARM
  }

  # Sensor Type
  type Sensor {
    sensorId: ID!
    sensorType: SensorType!
    equipmentId: String!

    manufacturer: String!
    model: String!
    serialNumber: String!

    measurement: SensorMeasurement!
    sampling: SamplingConfig!
    calibration: CalibrationInfo!
    communication: CommunicationConfig!
    installation: SensorInstallation!

    # Related data
    equipment: Equipment!
    recentReadings(limit: Int = 100): [SensorReading!]!
    currentStatus: SensorStatus
  }

  enum SensorType {
    TEMPERATURE_PROBE
    RTD_SENSOR
    THERMOCOUPLE
    PRESSURE_TRANSDUCER
    LEVEL_SENSOR
    HUMIDITY_SENSOR
    DOOR_SENSOR
    VIBRATION_SENSOR
    POWER_MONITOR
  }

  type SensorMeasurement {
    parameter: MeasurementParameter!
    unit: String!
    precision: Float!
    accuracy: Float!
    range: MeasurementRange!
  }

  type MeasurementRange {
    min: Float!
    max: Float!
  }

  type SamplingConfig {
    interval: Int!
    method: SamplingMethod!
    aggregation: AggregationMethod
  }

  enum SamplingMethod {
    CONTINUOUS
    PERIODIC
    EVENT_DRIVEN
  }

  enum AggregationMethod {
    AVERAGE
    MIN
    MAX
    LAST
  }

  type CalibrationInfo {
    lastCalibrated: DateTime!
    nextCalibration: DateTime!
    calibrationInterval: Int!
    calibrationCertificate: String
  }

  type CommunicationConfig {
    protocol: CommunicationProtocol!
    address: String!
    port: Int
  }

  enum CommunicationProtocol {
    MODBUS
    MQTT
    HTTP
    SERIAL
    I2C
  }

  type SensorInstallation {
    installedDate: DateTime!
    installedBy: String!
    location: String!
    position: String!
  }

  # Dashboard Metrics Type
  type DashboardMetrics {
    metricsId: ID!
    facilityId: String!
    timeRange: TimeRange!
    generatedAt: DateTime!

    equipment: EquipmentSummary!
    sensors: SensorSummary!
    alerts: AlertSummary!
    temperature: TemperatureMetrics
    compliance: ComplianceMetrics
    systemHealth: SystemHealth!
    trends: [Trend!]
  }

  type TimeRange {
    start: DateTime!
    end: DateTime!
    aggregation: String!
  }

  type EquipmentSummary {
    total: Int!
    operational: Int!
    alarm: Int!
    maintenance: Int!
    offline: Int!
    byType: JSON
  }

  type SensorSummary {
    total: Int!
    active: Int!
    warning: Int!
    offline: Int!
    calibrationDue: Int!
  }

  type AlertSummary {
    active: Int!
    acknowledged: Int!
    resolved: Int!
    bySeverity: SeverityBreakdown!
    byCategory: JSON
    responseTime: ResponseTimeMetrics
  }

  type SeverityBreakdown {
    critical: Int!
    high: Int!
    medium: Int!
    low: Int!
  }

  type ResponseTimeMetrics {
    average: Float!
    median: Float
    p95: Float
  }

  type TemperatureMetrics {
    averages: [TemperatureAverage!]!
    excursions: TemperatureExcursions
  }

  type TemperatureAverage {
    equipmentId: String!
    value: Float!
    min: Float!
    max: Float!
    stddev: Float
  }

  type TemperatureExcursions {
    count: Int!
    totalDuration: Float!
    events: [ExcursionEvent!]!
  }

  type ExcursionEvent {
    equipmentId: String!
    start: DateTime!
    end: DateTime
    peak: Float!
    duration: Float!
  }

  type ComplianceMetrics {
    validatedReadings: Int!
    totalReadings: Int!
    validationRate: Float!
    calibration: CalibrationMetrics!
    documentation: DocumentationStatus
  }

  type CalibrationMetrics {
    current: Int!
    due: Int!
    overdue: Int!
  }

  type DocumentationStatus {
    complete: Boolean!
    missingItems: [String!]
  }

  type SystemHealth {
    overall: SystemHealthStatus!
    score: Float!
    dataQuality: DataQuality
    availability: Availability
  }

  enum SystemHealthStatus {
    HEALTHY
    WARNING
    CRITICAL
  }

  type DataQuality {
    completeness: Float!
    accuracy: Float!
    timeliness: Float!
  }

  type Availability {
    uptime: Float!
    dataLoss: Float!
  }

  type Trend {
    metric: String!
    direction: TrendDirection!
    changePercent: Float!
    significance: Significance!
  }

  enum TrendDirection {
    IMPROVING
    STABLE
    DEGRADING
  }

  enum Significance {
    HIGH
    MEDIUM
    LOW
  }

  # Query Root
  type Query {
    # Sensor readings
    readings(
      page: Int
      pageSize: Int
      equipmentId: String
      sensorId: String
      parameter: MeasurementParameter
      startDate: DateTime
      endDate: DateTime
      minValue: Float
      maxValue: Float
    ): ReadingsConnection!

    reading(readingId: ID!): SensorReading

    # Equipment
    equipment(page: Int, pageSize: Int, facilityId: String): EquipmentConnection!
    equipmentById(equipmentId: ID!): Equipment
    equipmentStatus(equipmentId: ID!): EquipmentStatus

    # Sensors
    sensors(page: Int, pageSize: Int, equipmentId: String): SensorsConnection!
    sensor(sensorId: ID!): Sensor

    # Alerts
    alerts(
      page: Int
      pageSize: Int
      status: AlertStatus
      severity: AlertSeverity
      equipmentId: String
      startDate: DateTime
      endDate: DateTime
    ): AlertsConnection!

    alert(alertId: ID!): Alert

    # Dashboard
    dashboardMetrics(
      facilityId: String!
      startDate: DateTime!
      endDate: DateTime!
      aggregation: String
    ): DashboardMetrics!

    # Analytics
    temperatureTrends(
      equipmentId: String!
      timeRange: String!
      aggregation: String
    ): JSON!

    alertStatistics(
      facilityId: String!
      timeRange: String!
    ): JSON!
  }

  # Connection types for pagination
  type ReadingsConnection {
    edges: [ReadingEdge!]!
    pageInfo: PageInfo!
    totalCount: Int!
  }

  type ReadingEdge {
    node: SensorReading!
    cursor: String!
  }

  type EquipmentConnection {
    edges: [EquipmentEdge!]!
    pageInfo: PageInfo!
    totalCount: Int!
  }

  type EquipmentEdge {
    node: Equipment!
    cursor: String!
  }

  type SensorsConnection {
    edges: [SensorEdge!]!
    pageInfo: PageInfo!
    totalCount: Int!
  }

  type SensorEdge {
    node: Sensor!
    cursor: String!
  }

  type AlertsConnection {
    edges: [AlertEdge!]!
    pageInfo: PageInfo!
    totalCount: Int!
  }

  type AlertEdge {
    node: Alert!
    cursor: String!
  }

  type PageInfo {
    hasNextPage: Boolean!
    hasPreviousPage: Boolean!
    startCursor: String
    endCursor: String
  }

  # Mutation Root
  type Mutation {
    # Create readings
    createReading(input: CreateReadingInput!): SensorReading!
    createReadingsBatch(input: CreateReadingsBatchInput!): BatchResult!

    # Equipment management
    createEquipment(input: CreateEquipmentInput!): Equipment!
    updateEquipment(equipmentId: ID!, input: UpdateEquipmentInput!): Equipment!
    deleteEquipment(equipmentId: ID!): Boolean!

    # Sensor management
    createSensor(input: CreateSensorInput!): Sensor!
    updateSensor(sensorId: ID!, input: UpdateSensorInput!): Sensor!
    deleteSensor(sensorId: ID!): Boolean!
    calibrateSensor(sensorId: ID!, input: CalibrationInput!): Sensor!

    # Alert management
    acknowledgeAlert(alertId: ID!, userId: String!, comments: String): Alert!
    resolveAlert(alertId: ID!, userId: String!, resolution: String!): Alert!

    # Compliance
    generateComplianceReport(facilityId: String!, startDate: DateTime!, endDate: DateTime!): JSON!
  }

  # Input types
  input CreateReadingInput {
    sensorId: String!
    equipmentId: String!
    facilityId: String!
    timestamp: DateTime!
    measurement: MeasurementInput!
  }

  input MeasurementInput {
    parameter: MeasurementParameter!
    value: Float!
    unit: String!
  }

  input CreateReadingsBatchInput {
    readings: [CreateReadingInput!]!
  }

  input CreateEquipmentInput {
    equipmentType: EquipmentType!
    manufacturer: String!
    model: String!
    serialNumber: String!
    location: EquipmentLocationInput!
    capacity: EquipmentCapacityInput!
    operatingParameters: OperatingParametersInput!
  }

  input EquipmentLocationInput {
    facility: String!
    building: String!
    room: String!
    position: String!
  }

  input EquipmentCapacityInput {
    volume: Float!
    compartments: Int!
    maxSamples: Int!
  }

  input OperatingParametersInput {
    targetTemperature: Float!
    temperatureRange: TemperatureRangeInput!
  }

  input TemperatureRangeInput {
    min: Float!
    max: Float!
    alarm: AlarmThresholdsInput!
  }

  input AlarmThresholdsInput {
    low: Float!
    high: Float!
  }

  input UpdateEquipmentInput {
    manufacturer: String
    model: String
    location: EquipmentLocationInput
    operatingParameters: OperatingParametersInput
  }

  input CreateSensorInput {
    sensorType: SensorType!
    equipmentId: String!
    manufacturer: String!
    model: String!
    serialNumber: String!
    measurement: SensorMeasurementInput!
    sampling: SamplingConfigInput!
    communication: CommunicationConfigInput!
  }

  input SensorMeasurementInput {
    parameter: MeasurementParameter!
    unit: String!
    precision: Float!
    accuracy: Float!
    range: MeasurementRangeInput!
  }

  input MeasurementRangeInput {
    min: Float!
    max: Float!
  }

  input SamplingConfigInput {
    interval: Int!
    method: SamplingMethod!
    aggregation: AggregationMethod
  }

  input CommunicationConfigInput {
    protocol: CommunicationProtocol!
    address: String!
    port: Int
  }

  input UpdateSensorInput {
    manufacturer: String
    model: String
    sampling: SamplingConfigInput
  }

  input CalibrationInput {
    calibratedBy: String!
    calibrationDate: DateTime!
    certificate: String
    notes: String
  }

  type BatchResult {
    success: Boolean!
    count: Int!
    errors: [String!]
  }

  # Subscription Root
  type Subscription {
    # Real-time sensor readings
    readingCreated(equipmentId: String, sensorId: String): SensorReading!

    # Real-time alerts
    alertTriggered(facilityId: String, severity: AlertSeverity): Alert!
    alertUpdated(alertId: ID!): Alert!

    # Equipment status updates
    equipmentStatusChanged(equipmentId: ID!): EquipmentStatus!

    # System events
    systemEvent(facilityId: String): SystemEvent!
  }

  type SystemEvent {
    eventId: ID!
    timestamp: DateTime!
    type: String!
    severity: String!
    message: String!
    data: JSON
  }
`;

/**
 * GraphQL Resolvers
 */
export const resolvers = {
  Query: {
    readings: async (parent: any, args: any, context: any) => {
      // Implementation
      return {
        edges: [],
        pageInfo: {
          hasNextPage: false,
          hasPreviousPage: false
        },
        totalCount: 0
      };
    },
    equipmentById: async (parent: any, args: any, context: any) => {
      // Implementation
      return null;
    }
    // ... other resolvers
  },

  Mutation: {
    createReading: async (parent: any, args: any, context: any) => {
      // Implementation
      return args.input;
    }
    // ... other resolvers
  },

  Subscription: {
    readingCreated: {
      subscribe: (parent: any, args: any, context: any) => {
        // PubSub implementation
        return context.pubsub.asyncIterator(['READING_CREATED']);
      }
    }
    // ... other resolvers
  }
};

/**
 * Create GraphQL server
 */
export function createGraphQLServer() {
  const schema = makeExecutableSchema({
    typeDefs,
    resolvers
  });

  const server = new GraphQLServer({
    schema,
    context: (request) => ({
      ...request,
      // Add context objects like database, pubsub, etc.
    })
  });

  return server;
}
```

---

## 4. WebSocket API

### 4.1 Real-Time Streaming

```typescript
/**
 * WIA Cryo Monitoring WebSocket API
 *
 * Real-time data streaming for live monitoring
 */

import WebSocket from 'ws';
import { EventEmitter } from 'events';

/**
 * WebSocket message types
 */
export enum WSMessageType {
  // Client -> Server
  SUBSCRIBE = 'subscribe',
  UNSUBSCRIBE = 'unsubscribe',
  PING = 'ping',

  // Server -> Client
  READING = 'reading',
  ALERT = 'alert',
  STATUS_UPDATE = 'status_update',
  PONG = 'pong',
  ERROR = 'error'
}

/**
 * WebSocket message structure
 */
export interface WSMessage {
  type: WSMessageType;
  id?: string;
  timestamp: string;
  data: any;
}

/**
 * Subscription request
 */
export interface SubscriptionRequest {
  channel: 'readings' | 'alerts' | 'status' | 'events';
  filters?: {
    facilityId?: string;
    equipmentId?: string;
    sensorId?: string;
    severity?: string[];
  };
}

/**
 * WebSocket server implementation
 */
export class CryoMonitoringWSServer extends EventEmitter {
  private wss: WebSocket.Server;
  private clients: Map<string, WebSocket>;
  private subscriptions: Map<string, SubscriptionRequest[]>;

  constructor(port: number) {
    super();
    this.wss = new WebSocket.Server({ port });
    this.clients = new Map();
    this.subscriptions = new Map();

    this.setupServer();
  }

  private setupServer(): void {
    this.wss.on('connection', (ws: WebSocket, req: any) => {
      const clientId = crypto.randomUUID();
      this.clients.set(clientId, ws);
      this.subscriptions.set(clientId, []);

      console.log(`Client connected: ${clientId}`);

      ws.on('message', (message: string) => {
        this.handleMessage(clientId, message);
      });

      ws.on('close', () => {
        console.log(`Client disconnected: ${clientId}`);
        this.clients.delete(clientId);
        this.subscriptions.delete(clientId);
      });

      ws.on('error', (error) => {
        console.error(`WebSocket error for client ${clientId}:`, error);
      });

      // Send welcome message
      this.sendMessage(ws, {
        type: WSMessageType.STATUS_UPDATE,
        timestamp: new Date().toISOString(),
        data: {
          message: 'Connected to WIA Cryo Monitoring WebSocket API',
          clientId
        }
      });
    });
  }

  private handleMessage(clientId: string, message: string): void {
    try {
      const parsed: WSMessage = JSON.parse(message);

      switch (parsed.type) {
        case WSMessageType.SUBSCRIBE:
          this.handleSubscribe(clientId, parsed.data);
          break;

        case WSMessageType.UNSUBSCRIBE:
          this.handleUnsubscribe(clientId, parsed.data);
          break;

        case WSMessageType.PING:
          this.handlePing(clientId);
          break;

        default:
          this.sendError(clientId, 'Unknown message type');
      }
    } catch (error) {
      this.sendError(clientId, 'Invalid message format');
    }
  }

  private handleSubscribe(clientId: string, subscription: SubscriptionRequest): void {
    const subs = this.subscriptions.get(clientId) || [];
    subs.push(subscription);
    this.subscriptions.set(clientId, subs);

    const ws = this.clients.get(clientId);
    if (ws) {
      this.sendMessage(ws, {
        type: WSMessageType.STATUS_UPDATE,
        timestamp: new Date().toISOString(),
        data: {
          message: `Subscribed to ${subscription.channel}`,
          subscription
        }
      });
    }
  }

  private handleUnsubscribe(clientId: string, subscription: SubscriptionRequest): void {
    const subs = this.subscriptions.get(clientId) || [];
    const filtered = subs.filter(s => s.channel !== subscription.channel);
    this.subscriptions.set(clientId, filtered);

    const ws = this.clients.get(clientId);
    if (ws) {
      this.sendMessage(ws, {
        type: WSMessageType.STATUS_UPDATE,
        timestamp: new Date().toISOString(),
        data: {
          message: `Unsubscribed from ${subscription.channel}`
        }
      });
    }
  }

  private handlePing(clientId: string): void {
    const ws = this.clients.get(clientId);
    if (ws) {
      this.sendMessage(ws, {
        type: WSMessageType.PONG,
        timestamp: new Date().toISOString(),
        data: {}
      });
    }
  }

  private sendMessage(ws: WebSocket, message: WSMessage): void {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }

  private sendError(clientId: string, error: string): void {
    const ws = this.clients.get(clientId);
    if (ws) {
      this.sendMessage(ws, {
        type: WSMessageType.ERROR,
        timestamp: new Date().toISOString(),
        data: { error }
      });
    }
  }

  /**
   * Broadcast sensor reading to subscribed clients
   */
  public broadcastReading(reading: SensorReading): void {
    this.subscriptions.forEach((subs, clientId) => {
      const matchingSubs = subs.filter(sub => {
        if (sub.channel !== 'readings') return false;

        if (sub.filters) {
          if (sub.filters.equipmentId && sub.filters.equipmentId !== reading.equipmentId) {
            return false;
          }
          if (sub.filters.sensorId && sub.filters.sensorId !== reading.sensorId) {
            return false;
          }
        }

        return true;
      });

      if (matchingSubs.length > 0) {
        const ws = this.clients.get(clientId);
        if (ws) {
          this.sendMessage(ws, {
            type: WSMessageType.READING,
            timestamp: new Date().toISOString(),
            data: reading
          });
        }
      }
    });
  }

  /**
   * Broadcast alert to subscribed clients
   */
  public broadcastAlert(alert: Alert): void {
    this.subscriptions.forEach((subs, clientId) => {
      const matchingSubs = subs.filter(sub => {
        if (sub.channel !== 'alerts') return false;

        if (sub.filters) {
          if (sub.filters.equipmentId &&
              sub.filters.equipmentId !== alert.source.equipmentId) {
            return false;
          }
          if (sub.filters.severity &&
              !sub.filters.severity.includes(alert.classification.severity)) {
            return false;
          }
        }

        return true;
      });

      if (matchingSubs.length > 0) {
        const ws = this.clients.get(clientId);
        if (ws) {
          this.sendMessage(ws, {
            type: WSMessageType.ALERT,
            timestamp: new Date().toISOString(),
            data: alert
          });
        }
      }
    });
  }

  /**
   * Broadcast status update to subscribed clients
   */
  public broadcastStatusUpdate(status: EquipmentStatus): void {
    this.subscriptions.forEach((subs, clientId) => {
      const matchingSubs = subs.filter(sub => {
        if (sub.channel !== 'status') return false;

        if (sub.filters) {
          if (sub.filters.equipmentId &&
              sub.filters.equipmentId !== status.equipmentId) {
            return false;
          }
        }

        return true;
      });

      if (matchingSubs.length > 0) {
        const ws = this.clients.get(clientId);
        if (ws) {
          this.sendMessage(ws, {
            type: WSMessageType.STATUS_UPDATE,
            timestamp: new Date().toISOString(),
            data: status
          });
        }
      }
    });
  }
}

/**
 * Example usage
 */
export function startWebSocketServer(): void {
  const wsServer = new CryoMonitoringWSServer(8080);

  // Example: Broadcast reading
  wsServer.on('reading', (reading: SensorReading) => {
    wsServer.broadcastReading(reading);
  });

  // Example: Broadcast alert
  wsServer.on('alert', (alert: Alert) => {
    wsServer.broadcastAlert(alert);
  });

  console.log('WebSocket server started on port 8080');
}
```

---

## Conclusion

The WIA Cryo Monitoring Standard provides comprehensive API interfaces supporting REST, GraphQL, and WebSocket protocols. This multi-protocol approach ensures maximum flexibility for different integration scenarios while maintaining consistent data models and security standards.

**弘益人間 (Hongik Ingan)** - Universal APIs serving all monitoring needs.

---

© 2026 World Industry Association
Licensed under Apache 2.0
