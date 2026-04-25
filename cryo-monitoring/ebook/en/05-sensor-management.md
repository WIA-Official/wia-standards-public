# Chapter 5: Sensor Management
## Configuration, Calibration, and Health Monitoring

**弘益人間 (Hongik Ingan)** - Precision monitoring for humanity's benefit

---

## 1. Introduction to Sensor Management

Effective sensor management is critical for reliable cryogenic monitoring. This chapter covers comprehensive sensor lifecycle management including deployment, configuration, calibration, health monitoring, and maintenance.

### 1.1 Sensor Lifecycle

```typescript
/**
 * Sensor Lifecycle Management System
 *
 * Complete lifecycle management from procurement to decommissioning
 */

export enum SensorLifecycleStage {
  PROCUREMENT = 'procurement',
  RECEIVING = 'receiving',
  CALIBRATION = 'calibration',
  INSTALLATION = 'installation',
  COMMISSIONING = 'commissioning',
  OPERATIONAL = 'operational',
  MAINTENANCE = 'maintenance',
  RECALIBRATION = 'recalibration',
  REPAIR = 'repair',
  DECOMMISSIONING = 'decommissioning',
  RETIRED = 'retired'
}

export interface SensorLifecycle {
  sensorId: string;
  currentStage: SensorLifecycleStage;
  stageHistory: LifecycleStageHistory[];

  procurement: {
    vendor: string;
    purchaseOrder: string;
    purchaseDate: Date;
    cost: number;
    warrantyPeriod: number; // months
    warrantyExpiry: Date;
  };

  receiving: {
    receivedDate: Date;
    receivedBy: string;
    condition: 'excellent' | 'good' | 'acceptable' | 'damaged';
    inspectionNotes?: string;
    serialNumberVerified: boolean;
  };

  initialCalibration: {
    calibratedDate: Date;
    calibratedBy: string;
    calibrationLab: string;
    certificateNumber: string;
    certificateDocument?: string; // URL or file path
    nextCalibrationDue: Date;
  };

  installation: {
    installedDate: Date;
    installedBy: string;
    equipmentId: string;
    location: {
      compartment: string;
      position: string;
      coordinates?: { x: number; y: number; z: number };
    };
    installationChecklist: ChecklistItem[];
    installationNotes?: string;
  };

  commissioning: {
    commissionedDate: Date;
    commissionedBy: string;
    verificationTests: VerificationTest[];
    performanceBaseline: PerformanceBaseline;
    acceptanceCriteria: AcceptanceCriteria;
    passed: boolean;
  };

  operational: {
    activatedDate: Date;
    totalUptime: number; // seconds
    totalReadings: number;
    lastReading: Date;
    performanceMetrics: OperationalMetrics;
  };

  maintenanceHistory: MaintenanceRecord[];
  calibrationHistory: CalibrationRecord[];
  repairHistory: RepairRecord[];

  decommissioning?: {
    decommissionedDate: Date;
    decommissionedBy: string;
    reason: string;
    finalCalibrationStatus: string;
    disposition: 'returned' | 'salvaged' | 'recycled' | 'disposed';
    notes?: string;
  };
}

export interface LifecycleStageHistory {
  stage: SensorLifecycleStage;
  enteredDate: Date;
  exitedDate?: Date;
  duration?: number; // seconds
  performedBy: string;
  notes?: string;
}

export interface ChecklistItem {
  item: string;
  completed: boolean;
  completedBy?: string;
  completedDate?: Date;
  notes?: string;
}

export interface VerificationTest {
  testId: string;
  testName: string;
  testDate: Date;
  testedBy: string;
  procedure: string;
  expectedResult: any;
  actualResult: any;
  passed: boolean;
  deviationNotes?: string;
}

export interface PerformanceBaseline {
  parameter: string;
  nominalValue: number;
  tolerance: number;
  unit: string;
  measuredValue: number;
  deviation: number;
  acceptable: boolean;
}

export interface AcceptanceCriteria {
  criterion: string;
  requirement: string;
  result: string;
  passed: boolean;
}

export interface OperationalMetrics {
  availability: number; // percentage
  reliability: number; // MTBF in hours
  accuracy: number; // percentage deviation from calibrated value
  driftRate: number; // units per month
  failureRate: number; // failures per year
}

export interface MaintenanceRecord {
  maintenanceId: string;
  date: Date;
  type: 'preventive' | 'corrective' | 'inspection';
  performedBy: string;
  description: string;
  partsReplaced?: string[];
  cost?: number;
  downtime?: number; // minutes
  result: 'success' | 'partial' | 'failed';
  nextMaintenanceDue?: Date;
}

export interface CalibrationRecord {
  calibrationId: string;
  date: Date;
  calibratedBy: string;
  calibrationLab: string;
  method: string;
  referenceStandard: string;
  certificateNumber: string;

  beforeCalibration: {
    readings: CalibrationPoint[];
    drift: number;
    withinTolerance: boolean;
  };

  adjustmentsMade: boolean;
  adjustmentDetails?: string;

  afterCalibration: {
    readings: CalibrationPoint[];
    uncertainty: number;
    withinTolerance: boolean;
  };

  result: 'pass' | 'pass-with-adjustment' | 'fail';
  nextCalibrationDue: Date;
  certificateDocument?: string;
}

export interface CalibrationPoint {
  setpoint: number;
  measured: number;
  deviation: number;
  unit: string;
}

export interface RepairRecord {
  repairId: string;
  date: Date;
  repairedBy: string;
  problem: string;
  diagnosis: string;
  repair: string;
  partsReplaced: string[];
  cost: number;
  downtime: number; // minutes
  testAfterRepair: boolean;
  result: 'success' | 'partial' | 'failed';
}

/**
 * Sensor Lifecycle Manager
 */
export class SensorLifecycleManager {
  /**
   * Initialize new sensor
   */
  public static initializeSensor(
    sensorId: string,
    procurement: SensorLifecycle['procurement']
  ): SensorLifecycle {
    return {
      sensorId,
      currentStage: SensorLifecycleStage.PROCUREMENT,
      stageHistory: [
        {
          stage: SensorLifecycleStage.PROCUREMENT,
          enteredDate: new Date(),
          performedBy: 'system'
        }
      ],
      procurement,
      receiving: {} as any,
      initialCalibration: {} as any,
      installation: {} as any,
      commissioning: {} as any,
      operational: {} as any,
      maintenanceHistory: [],
      calibrationHistory: [],
      repairHistory: []
    };
  }

  /**
   * Advance to next lifecycle stage
   */
  public static advanceStage(
    lifecycle: SensorLifecycle,
    nextStage: SensorLifecycleStage,
    performedBy: string,
    notes?: string
  ): SensorLifecycle {
    // Close current stage
    const currentHistory = lifecycle.stageHistory[lifecycle.stageHistory.length - 1];
    currentHistory.exitedDate = new Date();
    currentHistory.duration = (currentHistory.exitedDate.getTime() -
                              currentHistory.enteredDate.getTime()) / 1000;

    // Add new stage
    lifecycle.stageHistory.push({
      stage: nextStage,
      enteredDate: new Date(),
      performedBy,
      notes
    });

    lifecycle.currentStage = nextStage;
    return lifecycle;
  }

  /**
   * Check if calibration is due
   */
  public static isCalibrationDue(lifecycle: SensorLifecycle): boolean {
    const lastCalibration = lifecycle.calibrationHistory[lifecycle.calibrationHistory.length - 1];
    if (!lastCalibration) return true;

    const dueDate = new Date(lastCalibration.nextCalibrationDue);
    return new Date() >= dueDate;
  }

  /**
   * Calculate sensor health score
   */
  public static calculateHealthScore(lifecycle: SensorLifecycle): number {
    let score = 100;

    // Deduct for calibration overdue
    if (this.isCalibrationDue(lifecycle)) {
      score -= 20;
    }

    // Deduct for recent failures
    const recentFailures = lifecycle.repairHistory.filter(r => {
      const monthsAgo = (Date.now() - r.date.getTime()) / (1000 * 60 * 60 * 24 * 30);
      return monthsAgo <= 6;
    });
    score -= recentFailures.length * 10;

    // Deduct for poor operational metrics
    if (lifecycle.operational?.performanceMetrics) {
      const metrics = lifecycle.operational.performanceMetrics;
      if (metrics.availability < 95) score -= 10;
      if (metrics.accuracy < 95) score -= 10;
      if (metrics.failureRate > 1) score -= 15;
    }

    return Math.max(0, Math.min(100, score));
  }
}
```

---

## 2. Sensor Configuration

### 2.1 Configuration Management

```typescript
/**
 * Sensor Configuration Management
 *
 * Comprehensive configuration system with versioning and validation
 */

export interface SensorConfig {
  configId: string;
  sensorId: string;
  version: string;
  createdDate: Date;
  createdBy: string;
  effectiveDate: Date;
  expiryDate?: Date;
  status: 'draft' | 'active' | 'superseded' | 'archived';

  // Core configuration
  basic: {
    name: string;
    description: string;
    enabled: boolean;
    priority: 'critical' | 'high' | 'normal' | 'low';
  };

  // Measurement configuration
  measurement: {
    parameter: string;
    unit: string;
    precision: number;
    accuracy: number;
    range: {
      min: number;
      max: number;
      safeMin: number;
      safeMax: number;
    };

    // Data validation
    validation: {
      enabled: boolean;
      rules: ValidationRule[];
      onValidationFailure: 'reject' | 'flag' | 'accept';
    };
  };

  // Sampling configuration
  sampling: {
    mode: 'continuous' | 'periodic' | 'event-driven' | 'on-demand';
    interval?: number; // seconds
    burstMode?: {
      enabled: boolean;
      samplesPerBurst: number;
      burstInterval: number; // seconds
    };
    aggregation?: {
      enabled: boolean;
      method: 'average' | 'min' | 'max' | 'median';
      windowSize: number; // seconds
    };
  };

  // Communication configuration
  communication: {
    protocol: 'modbus' | 'mqtt' | 'http' | 'serial' | 'i2c' | 'spi';
    address: string;
    port?: number;
    baudRate?: number; // for serial

    // Connection settings
    timeout: number; // milliseconds
    retries: number;
    retryDelay: number; // milliseconds
    keepAlive?: number; // seconds

    // Authentication
    authentication?: {
      type: 'none' | 'basic' | 'token' | 'certificate';
      username?: string;
      password?: string;
      token?: string;
      certificatePath?: string;
    };

    // TLS/SSL
    tls?: {
      enabled: boolean;
      version: string;
      certificateValidation: boolean;
    };
  };

  // Alert thresholds
  alerting: {
    enabled: boolean;
    thresholds: {
      criticalHigh?: number;
      warningHigh?: number;
      warningLow?: number;
      criticalLow?: number;
    };
    rateOfChange?: {
      enabled: boolean;
      maxRate: number; // units per minute
      timeWindow: number; // seconds
    };
  };

  // Data storage
  storage: {
    local: {
      enabled: boolean;
      path?: string;
      retention: number; // days
      compression: boolean;
    };
    cloud: {
      enabled: boolean;
      provider?: 'aws' | 'azure' | 'gcp';
      bucket?: string;
      retention: number; // days
    };
  };

  // Advanced settings
  advanced: {
    warmupTime?: number; // seconds before readings are valid
    cooldownTime?: number; // seconds to wait after power on
    averagingFilter?: {
      enabled: boolean;
      windowSize: number;
      method: 'simple' | 'weighted' | 'exponential';
    };
    outlierDetection?: {
      enabled: boolean;
      method: 'zscore' | 'iqr' | 'isolation-forest';
      threshold: number;
    };
  };

  // Metadata
  metadata: {
    tags: string[];
    customFields: Record<string, any>;
    notes?: string;
  };
}

export interface ValidationRule {
  ruleId: string;
  name: string;
  description: string;
  enabled: boolean;
  type: 'range' | 'rate-of-change' | 'pattern' | 'custom';

  condition: {
    operator: 'gt' | 'lt' | 'eq' | 'ne' | 'gte' | 'lte' | 'between';
    value?: number;
    min?: number;
    max?: number;
    pattern?: string;
    customFunction?: string;
  };

  action: {
    onSuccess?: string[];
    onFailure: string[];
  };
}

/**
 * Configuration Manager
 */
export class SensorConfigManager {
  private configs: Map<string, SensorConfig[]> = new Map();

  /**
   * Create new configuration
   */
  public createConfig(config: Omit<SensorConfig, 'configId' | 'version' | 'createdDate'>): SensorConfig {
    const newConfig: SensorConfig = {
      ...config,
      configId: crypto.randomUUID(),
      version: this.getNextVersion(config.sensorId),
      createdDate: new Date()
    };

    // Validate configuration
    this.validateConfig(newConfig);

    // Store configuration
    const sensorConfigs = this.configs.get(config.sensorId) || [];
    sensorConfigs.push(newConfig);
    this.configs.set(config.sensorId, sensorConfigs);

    return newConfig;
  }

  /**
   * Get active configuration for sensor
   */
  public getActiveConfig(sensorId: string): SensorConfig | undefined {
    const configs = this.configs.get(sensorId) || [];
    return configs.find(c => c.status === 'active' && this.isEffective(c));
  }

  /**
   * Update configuration
   */
  public updateConfig(
    sensorId: string,
    updates: Partial<SensorConfig>
  ): SensorConfig {
    // Get current active config
    const currentConfig = this.getActiveConfig(sensorId);
    if (!currentConfig) {
      throw new Error(`No active configuration for sensor ${sensorId}`);
    }

    // Create new version
    const newConfig: SensorConfig = {
      ...currentConfig,
      ...updates,
      configId: crypto.randomUUID(),
      version: this.getNextVersion(sensorId),
      createdDate: new Date(),
      status: 'active'
    };

    // Validate
    this.validateConfig(newConfig);

    // Supersede old config
    currentConfig.status = 'superseded';
    currentConfig.expiryDate = new Date();

    // Store new config
    const configs = this.configs.get(sensorId) || [];
    configs.push(newConfig);
    this.configs.set(sensorId, configs);

    return newConfig;
  }

  /**
   * Validate configuration
   */
  private validateConfig(config: SensorConfig): void {
    // Validate measurement range
    if (config.measurement.range.min >= config.measurement.range.max) {
      throw new Error('Invalid measurement range: min must be less than max');
    }

    // Validate safe range within measurement range
    if (config.measurement.range.safeMin < config.measurement.range.min ||
        config.measurement.range.safeMax > config.measurement.range.max) {
      throw new Error('Safe range must be within measurement range');
    }

    // Validate sampling interval
    if (config.sampling.mode === 'periodic' && !config.sampling.interval) {
      throw new Error('Periodic sampling requires interval');
    }

    // Validate alert thresholds
    if (config.alerting.enabled && config.alerting.thresholds.criticalHigh !== undefined) {
      if (config.alerting.thresholds.warningHigh !== undefined &&
          config.alerting.thresholds.criticalHigh <= config.alerting.thresholds.warningHigh) {
        throw new Error('Critical high threshold must be greater than warning high');
      }
    }

    // Additional validation...
  }

  /**
   * Check if configuration is effective
   */
  private isEffective(config: SensorConfig): boolean {
    const now = new Date();
    const effective = config.effectiveDate <= now;
    const notExpired = !config.expiryDate || config.expiryDate > now;
    return effective && notExpired;
  }

  /**
   * Get next version number
   */
  private getNextVersion(sensorId: string): string {
    const configs = this.configs.get(sensorId) || [];
    if (configs.length === 0) return '1.0.0';

    const versions = configs.map(c => c.version);
    const latestVersion = versions.sort().reverse()[0];
    const [major, minor, patch] = latestVersion.split('.').map(Number);

    return `${major}.${minor}.${patch + 1}`;
  }

  /**
   * Apply configuration to sensor
   */
  public async applyConfig(sensorId: string, config: SensorConfig): Promise<void> {
    console.log(`Applying configuration ${config.configId} to sensor ${sensorId}`);

    // Send configuration to sensor hardware
    // This would interface with the actual sensor communication protocol
    await this.sendConfigToSensor(sensorId, config);

    // Verify configuration applied successfully
    const verified = await this.verifyConfig(sensorId, config);
    if (!verified) {
      throw new Error(`Failed to verify configuration for sensor ${sensorId}`);
    }

    console.log(`Configuration ${config.configId} applied successfully`);
  }

  /**
   * Send configuration to sensor (placeholder)
   */
  private async sendConfigToSensor(sensorId: string, config: SensorConfig): Promise<void> {
    // Implementation would depend on sensor communication protocol
    // Example: MQTT, Modbus, HTTP, etc.
  }

  /**
   * Verify configuration applied correctly
   */
  private async verifyConfig(sensorId: string, config: SensorConfig): Promise<boolean> {
    // Read back configuration from sensor and compare
    return true; // Placeholder
  }
}
```

---

## 3. Sensor Calibration

### 3.1 Calibration Management System

```typescript
/**
 * Sensor Calibration Management
 *
 * Complete calibration lifecycle including scheduling, execution,
 * verification, and documentation
 */

export interface CalibrationSchedule {
  scheduleId: string;
  sensorId: string;

  schedule: {
    frequency: number; // days
    nextDue: Date;
    reminderDays: number; // days before due to send reminder
    autoSchedule: boolean;
  };

  requirements: {
    calibrationLab: 'internal' | 'external' | 'vendor';
    preferredProvider?: string;
    method: string;
    referenceStandard: string;
    traceability: 'NIST' | 'ISO' | 'vendor';
    environmentalConditions: {
      temperature: { min: number; max: number };
      humidity: { min: number; max: number };
    };
  };

  notifications: {
    recipients: string[];
    reminderChannels: ('email' | 'sms' | 'dashboard')[];
  };
}

export interface CalibrationProcedure {
  procedureId: string;
  name: string;
  version: string;
  sensorType: string;
  parameter: string;

  equipmentRequired: {
    item: string;
    specification: string;
    traceability?: string;
  }[];

  environmentalConditions: {
    temperature: { min: number; max: number; unit: string };
    humidity: { min: number; max: number; unit: string };
    stabilizationTime: number; // minutes
  };

  calibrationPoints: {
    setpoint: number;
    tolerance: number;
    unit: string;
    dwellTime: number; // seconds
  }[];

  steps: ProcedureStep[];

  acceptanceCriteria: {
    maxDeviation: number;
    maxUncertainty: number;
    repeatability: number;
  };

  documentation: {
    template: string;
    requiredFields: string[];
    attachments: string[];
  };
}

export interface ProcedureStep {
  stepNumber: number;
  instruction: string;
  type: 'setup' | 'measurement' | 'adjustment' | 'verification' | 'documentation';
  requiresData: boolean;
  dataFields?: {
    field: string;
    type: 'number' | 'text' | 'boolean';
    required: boolean;
  }[];
  warning?: string;
  note?: string;
}

export interface CalibrationSession {
  sessionId: string;
  sensorId: string;
  procedureId: string;

  sessionInfo: {
    startTime: Date;
    endTime?: Date;
    technician: string;
    witness?: string;
    location: string;
  };

  environment: {
    temperature: number;
    humidity: number;
    pressure?: number;
    stable: boolean;
  };

  equipment: {
    item: string;
    serialNumber: string;
    calibrationCertificate: string;
    calibrationDate: Date;
  }[];

  measurements: {
    setpoint: number;
    beforeAdjustment: CalibrationMeasurement[];
    afterAdjustment?: CalibrationMeasurement[];
  }[];

  adjustmentsMade: boolean;
  adjustmentDetails?: {
    parameter: string;
    beforeValue: number;
    afterValue: number;
    method: string;
  }[];

  results: {
    passed: boolean;
    deviation: number;
    uncertainty: number;
    repeatability: number;
    comments?: string;
  };

  certificate: {
    certificateNumber: string;
    issuedDate: Date;
    nextCalibrationDue: Date;
    document?: string; // URL to certificate PDF
  };
}

export interface CalibrationMeasurement {
  reading: number;
  timestamp: Date;
  deviation: number;
  unit: string;
}

/**
 * Calibration Manager
 */
export class CalibrationManager {
  private schedules: Map<string, CalibrationSchedule> = new Map();
  private procedures: Map<string, CalibrationProcedure> = new Map();
  private sessions: CalibrationSession[] = [];

  /**
   * Create calibration schedule
   */
  public createSchedule(schedule: CalibrationSchedule): void {
    this.validateSchedule(schedule);
    this.schedules.set(schedule.sensorId, schedule);

    // Set up reminder notifications
    this.scheduleReminders(schedule);
  }

  /**
   * Check for due calibrations
   */
  public getDueCalibrations(): CalibrationSchedule[] {
    const now = new Date();
    const due: CalibrationSchedule[] = [];

    this.schedules.forEach(schedule => {
      if (schedule.schedule.nextDue <= now) {
        due.push(schedule);
      }
    });

    return due;
  }

  /**
   * Get upcoming calibrations
   */
  public getUpcomingCalibrations(daysAhead: number): CalibrationSchedule[] {
    const now = new Date();
    const futureDate = new Date(now.getTime() + daysAhead * 24 * 60 * 60 * 1000);
    const upcoming: CalibrationSchedule[] = [];

    this.schedules.forEach(schedule => {
      if (schedule.schedule.nextDue > now && schedule.schedule.nextDue <= futureDate) {
        upcoming.push(schedule);
      }
    });

    return upcoming;
  }

  /**
   * Start calibration session
   */
  public startCalibrationSession(
    sensorId: string,
    procedureId: string,
    technician: string,
    witness?: string
  ): CalibrationSession {
    const procedure = this.procedures.get(procedureId);
    if (!procedure) {
      throw new Error(`Procedure ${procedureId} not found`);
    }

    const session: CalibrationSession = {
      sessionId: crypto.randomUUID(),
      sensorId,
      procedureId,

      sessionInfo: {
        startTime: new Date(),
        technician,
        witness,
        location: '' // To be filled
      },

      environment: {
        temperature: 0, // To be measured
        humidity: 0,
        stable: false
      },

      equipment: [],
      measurements: [],
      adjustmentsMade: false,

      results: {
        passed: false,
        deviation: 0,
        uncertainty: 0,
        repeatability: 0
      },

      certificate: {
        certificateNumber: this.generateCertificateNumber(),
        issuedDate: new Date(),
        nextCalibrationDue: this.calculateNextDue(sensorId)
      }
    };

    this.sessions.push(session);
    return session;
  }

  /**
   * Record measurement during calibration
   */
  public recordMeasurement(
    sessionId: string,
    setpoint: number,
    reading: number
  ): void {
    const session = this.sessions.find(s => s.sessionId === sessionId);
    if (!session) {
      throw new Error(`Session ${sessionId} not found`);
    }

    const measurement: CalibrationMeasurement = {
      reading,
      timestamp: new Date(),
      deviation: reading - setpoint,
      unit: 'celsius' // Should come from sensor config
    };

    // Find or create measurement set for this setpoint
    let measurementSet = session.measurements.find(m => m.setpoint === setpoint);
    if (!measurementSet) {
      measurementSet = {
        setpoint,
        beforeAdjustment: []
      };
      session.measurements.push(measurementSet);
    }

    measurementSet.beforeAdjustment.push(measurement);
  }

  /**
   * Complete calibration session
   */
  public completeCalibrationSession(
    sessionId: string,
    passed: boolean,
    comments?: string
  ): CalibrationSession {
    const session = this.sessions.find(s => s.sessionId === sessionId);
    if (!session) {
      throw new Error(`Session ${sessionId} not found`);
    }

    session.sessionInfo.endTime = new Date();
    session.results.passed = passed;
    session.results.comments = comments;

    // Calculate statistics
    this.calculateCalibrationStatistics(session);

    // Update sensor calibration record
    this.updateSensorCalibrationRecord(session);

    // Update schedule
    if (passed) {
      this.updateSchedule(session.sensorId, session.certificate.nextCalibrationDue);
    }

    // Generate certificate
    this.generateCertificate(session);

    return session;
  }

  /**
   * Calculate calibration statistics
   */
  private calculateCalibrationStatistics(session: CalibrationSession): void {
    const allMeasurements = session.measurements.flatMap(m => m.beforeAdjustment);

    // Calculate maximum deviation
    const deviations = allMeasurements.map(m => Math.abs(m.deviation));
    session.results.deviation = Math.max(...deviations);

    // Calculate uncertainty (simplified)
    const mean = deviations.reduce((a, b) => a + b, 0) / deviations.length;
    const squaredDiffs = deviations.map(d => Math.pow(d - mean, 2));
    const variance = squaredDiffs.reduce((a, b) => a + b, 0) / squaredDiffs.length;
    session.results.uncertainty = Math.sqrt(variance) * 2; // 95% confidence interval

    // Calculate repeatability
    const setpointMeasurements = session.measurements.map(m => {
      const readings = m.beforeAdjustment.map(r => r.reading);
      const mean = readings.reduce((a, b) => a + b, 0) / readings.length;
      const diffs = readings.map(r => Math.abs(r - mean));
      return Math.max(...diffs);
    });
    session.results.repeatability = Math.max(...setpointMeasurements);
  }

  /**
   * Generate certificate number
   */
  private generateCertificateNumber(): string {
    const date = new Date();
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const random = Math.floor(Math.random() * 10000).toString().padStart(4, '0');
    return `CAL-${year}${month}-${random}`;
  }

  /**
   * Calculate next calibration due date
   */
  private calculateNextDue(sensorId: string): Date {
    const schedule = this.schedules.get(sensorId);
    if (!schedule) {
      // Default to 1 year
      const nextDue = new Date();
      nextDue.setFullYear(nextDue.getFullYear() + 1);
      return nextDue;
    }

    const nextDue = new Date();
    nextDue.setDate(nextDue.getDate() + schedule.schedule.frequency);
    return nextDue;
  }

  /**
   * Update sensor calibration record (placeholder)
   */
  private updateSensorCalibrationRecord(session: CalibrationSession): void {
    // Update sensor lifecycle with calibration record
  }

  /**
   * Update schedule after calibration
   */
  private updateSchedule(sensorId: string, nextDue: Date): void {
    const schedule = this.schedules.get(sensorId);
    if (schedule) {
      schedule.schedule.nextDue = nextDue;
      this.scheduleReminders(schedule);
    }
  }

  /**
   * Generate calibration certificate
   */
  private generateCertificate(session: CalibrationSession): void {
    // Generate PDF certificate with all calibration data
    console.log(`Generated certificate ${session.certificate.certificateNumber}`);
  }

  /**
   * Schedule reminder notifications
   */
  private scheduleReminders(schedule: CalibrationSchedule): void {
    // Set up notifications for upcoming calibration
    const reminderDate = new Date(schedule.schedule.nextDue);
    reminderDate.setDate(reminderDate.getDate() - schedule.schedule.reminderDays);

    console.log(`Scheduled calibration reminder for ${schedule.sensorId} on ${reminderDate}`);
  }

  /**
   * Validate calibration schedule
   */
  private validateSchedule(schedule: CalibrationSchedule): void {
    if (schedule.schedule.frequency <= 0) {
      throw new Error('Calibration frequency must be positive');
    }
    if (schedule.schedule.reminderDays < 0) {
      throw new Error('Reminder days cannot be negative');
    }
  }
}
```

---

## 4. Sensor Health Monitoring

### 4.1 Health Monitoring System

```typescript
/**
 * Sensor Health Monitoring System
 *
 * Real-time health assessment and predictive diagnostics
 */

export interface SensorHealth {
  sensorId: string;
  timestamp: Date;

  overall: {
    status: 'healthy' | 'warning' | 'critical' | 'offline';
    score: number; // 0-100
    trend: 'improving' | 'stable' | 'degrading';
  };

  components: {
    hardware: ComponentHealth;
    communication: ComponentHealth;
    dataQuality: ComponentHealth;
    calibration: ComponentHealth;
    power: ComponentHealth;
  };

  metrics: {
    availability: number; // percentage
    reliability: number; // MTBF in hours
    accuracy: number; // percentage
    responseTime: number; // milliseconds
    dataLoss: number; // percentage
  };

  anomalies: Anomaly[];
  predictions: HealthPrediction[];
  recommendations: Recommendation[];
}

export interface ComponentHealth {
  status: 'healthy' | 'warning' | 'critical' | 'failed';
  score: number; // 0-100
  lastCheck: Date;
  indicators: HealthIndicator[];
  issues: Issue[];
}

export interface HealthIndicator {
  name: string;
  value: number;
  unit: string;
  threshold: {
    warning: number;
    critical: number;
  };
  status: 'ok' | 'warning' | 'critical';
}

export interface Issue {
  issueId: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  description: string;
  detectedAt: Date;
  impact: string;
  recommendedAction: string;
  acknowledged: boolean;
}

export interface Anomaly {
  anomalyId: string;
  type: 'statistical' | 'pattern' | 'rate-of-change' | 'correlation';
  detected: Date;
  description: string;
  confidence: number; // 0-100
  severity: 'low' | 'medium' | 'high';
  affectedMetrics: string[];
  possibleCauses: string[];
}

export interface HealthPrediction {
  predictionId: string;
  type: 'failure' | 'degradation' | 'calibration-drift';
  predictedDate: Date;
  confidence: number; // 0-100
  component: string;
  description: string;
  recommendedAction: string;
  preventiveMeasures: string[];
}

export interface Recommendation {
  recommendationId: string;
  priority: 'immediate' | 'high' | 'medium' | 'low';
  category: 'maintenance' | 'calibration' | 'configuration' | 'replacement';
  description: string;
  benefit: string;
  effort: 'low' | 'medium' | 'high';
  estimatedCost?: number;
}

/**
 * Health Monitoring Engine
 */
export class SensorHealthMonitor {
  /**
   * Assess sensor health
   */
  public assessHealth(sensorId: string, readings: SensorReading[]): SensorHealth {
    const health: SensorHealth = {
      sensorId,
      timestamp: new Date(),

      overall: {
        status: 'healthy',
        score: 100,
        trend: 'stable'
      },

      components: {
        hardware: this.assessHardware(sensorId, readings),
        communication: this.assessCommunication(sensorId, readings),
        dataQuality: this.assessDataQuality(readings),
        calibration: this.assessCalibration(sensorId),
        power: this.assessPower(readings)
      },

      metrics: this.calculateMetrics(readings),
      anomalies: this.detectAnomalies(readings),
      predictions: this.generatePredictions(sensorId, readings),
      recommendations: []
    };

    // Calculate overall score
    health.overall.score = this.calculateOverallScore(health.components);
    health.overall.status = this.determineStatus(health.overall.score);
    health.overall.trend = this.determineTrend(sensorId);

    // Generate recommendations
    health.recommendations = this.generateRecommendations(health);

    return health;
  }

  /**
   * Assess hardware component health
   */
  private assessHardware(sensorId: string, readings: SensorReading[]): ComponentHealth {
    const indicators: HealthIndicator[] = [];
    const issues: Issue[] = [];

    // Check reading consistency
    const recentReadings = readings.slice(-100);
    if (recentReadings.length > 0) {
      const values = recentReadings.map(r => r.measurement.value);
      const stddev = this.calculateStdDev(values);

      indicators.push({
        name: 'Reading Stability',
        value: stddev,
        unit: 'stddev',
        threshold: {
          warning: 1.0,
          critical: 2.0
        },
        status: stddev > 2.0 ? 'critical' : stddev > 1.0 ? 'warning' : 'ok'
      });

      if (stddev > 2.0) {
        issues.push({
          issueId: crypto.randomUUID(),
          severity: 'high',
          description: 'High reading variability detected',
          detectedAt: new Date(),
          impact: 'Unreliable measurements',
          recommendedAction: 'Check sensor mounting and environmental stability',
          acknowledged: false
        });
      }
    }

    return {
      status: issues.some(i => i.severity === 'critical') ? 'critical' :
              issues.some(i => i.severity === 'high') ? 'warning' : 'healthy',
      score: 100 - (issues.length * 10),
      lastCheck: new Date(),
      indicators,
      issues
    };
  }

  /**
   * Assess communication health
   */
  private assessCommunication(sensorId: string, readings: SensorReading[]): ComponentHealth {
    const indicators: HealthIndicator[] = [];
    const issues: Issue[] = [];

    // Check for communication gaps
    const timeGaps = this.findTimeGaps(readings);
    if (timeGaps.length > 0) {
      const maxGap = Math.max(...timeGaps);

      indicators.push({
        name: 'Communication Reliability',
        value: maxGap,
        unit: 'seconds',
        threshold: {
          warning: 300,
          critical: 600
        },
        status: maxGap > 600 ? 'critical' : maxGap > 300 ? 'warning' : 'ok'
      });

      if (maxGap > 600) {
        issues.push({
          issueId: crypto.randomUUID(),
          severity: 'critical',
          description: `Communication gap detected: ${maxGap} seconds`,
          detectedAt: new Date(),
          impact: 'Missing data',
          recommendedAction: 'Check network connection and sensor power',
          acknowledged: false
        });
      }
    }

    return {
      status: issues.length === 0 ? 'healthy' : 'warning',
      score: 100 - (timeGaps.length * 5),
      lastCheck: new Date(),
      indicators,
      issues
    };
  }

  /**
   * Assess data quality
   */
  private assessDataQuality(readings: SensorReading[]): ComponentHealth {
    const indicators: HealthIndicator[] = [];
    const issues: Issue[] = [];

    // Check data validity
    const invalidReadings = readings.filter(r =>
      r.measurement.quality && !r.measurement.quality.isValid
    );

    const invalidPercentage = (invalidReadings.length / readings.length) * 100;

    indicators.push({
      name: 'Data Validity',
      value: 100 - invalidPercentage,
      unit: '%',
      threshold: {
        warning: 95,
        critical: 90
      },
      status: invalidPercentage > 10 ? 'critical' : invalidPercentage > 5 ? 'warning' : 'ok'
    });

    if (invalidPercentage > 10) {
      issues.push({
        issueId: crypto.randomUUID(),
        severity: 'high',
        description: `High invalid reading rate: ${invalidPercentage.toFixed(1)}%`,
        detectedAt: new Date(),
        impact: 'Unreliable data for monitoring and alerts',
        recommendedAction: 'Calibrate sensor or check measurement conditions',
        acknowledged: false
      });
    }

    return {
      status: invalidPercentage > 10 ? 'critical' :
              invalidPercentage > 5 ? 'warning' : 'healthy',
      score: 100 - invalidPercentage,
      lastCheck: new Date(),
      indicators,
      issues
    };
  }

  /**
   * Assess calibration status
   */
  private assessCalibration(sensorId: string): ComponentHealth {
    // Check calibration due date
    // This would integrate with CalibrationManager

    return {
      status: 'healthy',
      score: 100,
      lastCheck: new Date(),
      indicators: [],
      issues: []
    };
  }

  /**
   * Assess power status
   */
  private assessPower(readings: SensorReading[]): ComponentHealth {
    const indicators: HealthIndicator[] = [];
    const issues: Issue[] = [];

    // Check battery levels if applicable
    const recentReadings = readings.slice(-10);
    const batteryLevels = recentReadings
      .map(r => r.sensorStatus?.batteryLevel)
      .filter(b => b !== undefined) as number[];

    if (batteryLevels.length > 0) {
      const avgBattery = batteryLevels.reduce((a, b) => a + b, 0) / batteryLevels.length;

      indicators.push({
        name: 'Battery Level',
        value: avgBattery,
        unit: '%',
        threshold: {
          warning: 20,
          critical: 10
        },
        status: avgBattery < 10 ? 'critical' : avgBattery < 20 ? 'warning' : 'ok'
      });

      if (avgBattery < 10) {
        issues.push({
          issueId: crypto.randomUUID(),
          severity: 'critical',
          description: 'Battery level critically low',
          detectedAt: new Date(),
          impact: 'Sensor may go offline soon',
          recommendedAction: 'Replace battery immediately',
          acknowledged: false
        });
      }
    }

    return {
      status: batteryLevels.length > 0 && batteryLevels[0] < 10 ? 'critical' :
              batteryLevels.length > 0 && batteryLevels[0] < 20 ? 'warning' : 'healthy',
      score: batteryLevels.length > 0 ? batteryLevels[0] : 100,
      lastCheck: new Date(),
      indicators,
      issues
    };
  }

  /**
   * Calculate metrics
   */
  private calculateMetrics(readings: SensorReading[]): SensorHealth['metrics'] {
    return {
      availability: 99.5, // Placeholder
      reliability: 8760, // Placeholder - 1 year MTBF
      accuracy: 99.9,
      responseTime: 100,
      dataLoss: 0.1
    };
  }

  /**
   * Detect anomalies
   */
  private detectAnomalies(readings: SensorReading[]): Anomaly[] {
    const anomalies: Anomaly[] = [];

    // Statistical anomaly detection
    const values = readings.map(r => r.measurement.value);
    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    const stddev = this.calculateStdDev(values);

    readings.forEach(reading => {
      const zScore = Math.abs((reading.measurement.value - mean) / stddev);
      if (zScore > 3) {
        anomalies.push({
          anomalyId: crypto.randomUUID(),
          type: 'statistical',
          detected: reading.timestamp,
          description: `Reading ${reading.measurement.value} is ${zScore.toFixed(2)} standard deviations from mean`,
          confidence: Math.min(95, zScore * 20),
          severity: zScore > 4 ? 'high' : 'medium',
          affectedMetrics: ['temperature'],
          possibleCauses: [
            'Sensor malfunction',
            'Environmental disturbance',
            'Calibration drift'
          ]
        });
      }
    });

    return anomalies;
  }

  /**
   * Generate predictions
   */
  private generatePredictions(sensorId: string, readings: SensorReading[]): HealthPrediction[] {
    // Machine learning-based predictions would go here
    return [];
  }

  /**
   * Generate recommendations
   */
  private generateRecommendations(health: SensorHealth): Recommendation[] {
    const recommendations: Recommendation[] = [];

    // Check for critical issues
    Object.values(health.components).forEach(component => {
      component.issues.forEach(issue => {
        if (issue.severity === 'critical') {
          recommendations.push({
            recommendationId: crypto.randomUUID(),
            priority: 'immediate',
            category: 'maintenance',
            description: issue.recommendedAction,
            benefit: 'Prevent sensor failure',
            effort: 'medium'
          });
        }
      });
    });

    return recommendations;
  }

  /**
   * Calculate overall score
   */
  private calculateOverallScore(components: SensorHealth['components']): number {
    const scores = Object.values(components).map(c => c.score);
    return scores.reduce((a, b) => a + b, 0) / scores.length;
  }

  /**
   * Determine status from score
   */
  private determineStatus(score: number): SensorHealth['overall']['status'] {
    if (score >= 90) return 'healthy';
    if (score >= 70) return 'warning';
    return 'critical';
  }

  /**
   * Determine trend (placeholder)
   */
  private determineTrend(sensorId: string): SensorHealth['overall']['trend'] {
    return 'stable';
  }

  /**
   * Find time gaps in readings
   */
  private findTimeGaps(readings: SensorReading[]): number[] {
    const gaps: number[] = [];

    for (let i = 1; i < readings.length; i++) {
      const gap = (readings[i].timestamp.getTime() - readings[i-1].timestamp.getTime()) / 1000;
      if (gap > 120) { // More than 2 minutes
        gaps.push(gap);
      }
    }

    return gaps;
  }

  /**
   * Calculate standard deviation
   */
  private calculateStdDev(values: number[]): number {
    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    const squaredDiffs = values.map(v => Math.pow(v - mean, 2));
    const variance = squaredDiffs.reduce((a, b) => a + b, 0) / squaredDiffs.length;
    return Math.sqrt(variance);
  }
}
```

---

## Conclusion

Comprehensive sensor management is essential for reliable cryogenic monitoring. The WIA standard provides complete lifecycle management, configuration control, calibration tracking, and health monitoring to ensure optimal sensor performance and data integrity.

**弘益人間 (Hongik Ingan)** - Through excellence in sensor management, we protect what matters most.

---

© 2026 World Industry Association
Licensed under Apache 2.0
