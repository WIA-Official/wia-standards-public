# Chapter 3: Data Formats and Schemas
## Comprehensive Data Structures for Cryogenic Monitoring

**弘益人間 (Hongik Ingan)** - Standardized data for universal benefit

---

## 1. Introduction to WIA Cryo Data Formats

The WIA Cryo Monitoring Standard defines comprehensive data formats using TypeScript and Zod schemas for runtime validation. These formats ensure interoperability, data integrity, and compliance across all cryogenic monitoring systems.

### 1.1 Schema Design Principles

- **Type Safety:** Full TypeScript type definitions with compile-time checking
- **Runtime Validation:** Zod schemas for runtime data validation
- **Extensibility:** Support for custom fields and vendor-specific extensions
- **Backward Compatibility:** Versioned schemas with migration paths
- **Compliance:** Built-in support for regulatory requirements

---

## 2. Core Data Schemas

### 2.1 Sensor Reading Schema

```typescript
import { z } from 'zod';

/**
 * Sensor reading data structure
 *
 * Represents a single measurement from a cryogenic monitoring sensor.
 * Includes full metadata for traceability, compliance, and analytics.
 */

// Base sensor reading schema
export const SensorReadingSchema = z.object({
  // Identification
  readingId: z.string().uuid().describe('Unique identifier for this reading'),
  sensorId: z.string().describe('Sensor that generated this reading'),
  equipmentId: z.string().describe('Equipment being monitored'),
  facilityId: z.string().describe('Facility location'),

  // Timestamp information
  timestamp: z.coerce.date().describe('When the reading was taken'),
  receivedAt: z.coerce.date().optional().describe('When the reading was received by the system'),
  processedAt: z.coerce.date().optional().describe('When the reading was processed'),

  // Measurement data
  measurement: z.object({
    parameter: z.enum([
      'temperature',
      'pressure',
      'liquid-level',
      'humidity',
      'door-status',
      'vibration',
      'power-status',
      'co2-level',
      'o2-level',
      'flow-rate',
      'vacuum-pressure'
    ]).describe('Type of parameter being measured'),

    value: z.number().describe('Measured value'),
    unit: z.string().describe('Unit of measurement (e.g., celsius, psi, %)'),

    // Statistical information (for aggregated readings)
    statistics: z.object({
      min: z.number().optional(),
      max: z.number().optional(),
      average: z.number().optional(),
      stddev: z.number().optional(),
      sampleCount: z.number().int().positive().optional()
    }).optional(),

    // Quality indicators
    quality: z.object({
      isValid: z.boolean().describe('Whether reading passed validation'),
      confidence: z.number().min(0).max(100).optional().describe('Confidence score (0-100)'),
      flags: z.array(z.string()).optional().describe('Quality flags'),
      calibrationStatus: z.enum(['calibrated', 'due', 'overdue', 'unknown']).optional()
    }).optional()
  }),

  // Sensor status
  sensorStatus: z.object({
    health: z.enum(['healthy', 'warning', 'critical', 'offline']).describe('Sensor health status'),
    batteryLevel: z.number().min(0).max(100).optional().describe('Battery percentage'),
    signalStrength: z.number().min(-100).max(0).optional().describe('Signal strength (dBm)'),
    lastCalibration: z.coerce.date().optional(),
    errors: z.array(z.string()).optional()
  }).optional(),

  // Location information
  location: z.object({
    compartment: z.string().optional(),
    position: z.string().optional(),
    coordinates: z.object({
      x: z.number().optional(),
      y: z.number().optional(),
      z: z.number().optional()
    }).optional()
  }).optional(),

  // Alert information
  alert: z.object({
    triggered: z.boolean().describe('Whether this reading triggered an alert'),
    alertIds: z.array(z.string()).optional().describe('IDs of triggered alerts'),
    severity: z.enum(['critical', 'high', 'medium', 'low', 'info']).optional()
  }).optional(),

  // Compliance and audit
  compliance: z.object({
    validated: z.boolean().describe('Whether reading has been validated'),
    validatedBy: z.string().optional(),
    validatedAt: z.coerce.date().optional(),
    signature: z.string().optional().describe('Digital signature for compliance'),
    witnessSignature: z.string().optional(),
    comments: z.string().optional()
  }).optional(),

  // Metadata
  metadata: z.record(z.any()).optional().describe('Additional custom metadata'),

  // Version control
  schemaVersion: z.string().default('1.0.0')
});

export type SensorReading = z.infer<typeof SensorReadingSchema>;

/**
 * Batch of sensor readings
 */
export const SensorReadingBatchSchema = z.object({
  batchId: z.string().uuid(),
  readings: z.array(SensorReadingSchema),
  collectionStart: z.coerce.date(),
  collectionEnd: z.coerce.date(),
  totalReadings: z.number().int().positive(),
  metadata: z.record(z.any()).optional()
});

export type SensorReadingBatch = z.infer<typeof SensorReadingBatchSchema>;

/**
 * Example sensor reading
 */
export const exampleSensorReading: SensorReading = {
  readingId: '550e8400-e29b-41d4-a716-446655440000',
  sensorId: 'sensor-temp-001',
  equipmentId: 'ln2-tank-001',
  facilityId: 'facility-main-lab',

  timestamp: new Date('2026-01-11T10:30:00Z'),
  receivedAt: new Date('2026-01-11T10:30:01Z'),
  processedAt: new Date('2026-01-11T10:30:02Z'),

  measurement: {
    parameter: 'temperature',
    value: -195.8,
    unit: 'celsius',

    quality: {
      isValid: true,
      confidence: 98.5,
      flags: [],
      calibrationStatus: 'calibrated'
    }
  },

  sensorStatus: {
    health: 'healthy',
    batteryLevel: 87,
    signalStrength: -45,
    lastCalibration: new Date('2025-11-01'),
    errors: []
  },

  location: {
    compartment: 'compartment-1',
    position: 'top-center',
    coordinates: {
      x: 0,
      y: 0,
      z: 50
    }
  },

  alert: {
    triggered: false
  },

  compliance: {
    validated: true,
    validatedBy: 'system-auto-validator',
    validatedAt: new Date('2026-01-11T10:30:02Z')
  },

  schemaVersion: '1.0.0'
};
```

### 2.2 Alert Schema

```typescript
/**
 * Alert data structure
 *
 * Represents an alert generated by the monitoring system when
 * sensor readings exceed thresholds or anomalies are detected.
 */

export const AlertSchema = z.object({
  // Identification
  alertId: z.string().uuid(),
  alertRuleId: z.string().describe('Rule that triggered this alert'),
  correlationId: z.string().uuid().optional().describe('For grouping related alerts'),

  // Source information
  source: z.object({
    sensorId: z.string(),
    equipmentId: z.string(),
    facilityId: z.string(),
    readingId: z.string().optional().describe('Reading that triggered the alert')
  }),

  // Alert classification
  classification: z.object({
    type: z.enum([
      'threshold-exceeded',
      'threshold-below',
      'rate-of-change',
      'anomaly-detected',
      'sensor-failure',
      'communication-loss',
      'calibration-due',
      'maintenance-due',
      'battery-low',
      'custom'
    ]),
    severity: z.enum(['critical', 'high', 'medium', 'low', 'info']),
    category: z.enum([
      'temperature',
      'pressure',
      'level',
      'equipment',
      'sensor',
      'communication',
      'compliance',
      'maintenance'
    ])
  }),

  // Timing information
  timing: z.object({
    detected: z.coerce.date().describe('When the alert condition was detected'),
    triggered: z.coerce.date().describe('When the alert was triggered'),
    acknowledged: z.coerce.date().optional(),
    resolved: z.coerce.date().optional(),
    durationSeconds: z.number().optional().describe('How long condition persisted before trigger')
  }),

  // Condition details
  condition: z.object({
    parameter: z.string(),
    currentValue: z.number(),
    threshold: z.number(),
    operator: z.enum(['gt', 'lt', 'eq', 'ne', 'gte', 'lte']),
    unit: z.string(),
    description: z.string()
  }),

  // Alert state
  state: z.object({
    status: z.enum([
      'active',
      'acknowledged',
      'resolved',
      'suppressed',
      'escalated',
      'false-positive',
      'cancelled'
    ]),
    acknowledgedBy: z.string().optional(),
    acknowledgedAt: z.coerce.date().optional(),
    resolvedBy: z.string().optional(),
    resolvedAt: z.coerce.date().optional(),
    resolution: z.string().optional().describe('How the alert was resolved'),
    escalationLevel: z.number().int().min(0).optional()
  }),

  // Notification tracking
  notifications: z.array(z.object({
    notificationId: z.string().uuid(),
    channel: z.enum(['email', 'sms', 'phone', 'push', 'webhook', 'pager']),
    recipient: z.string(),
    sentAt: z.coerce.date(),
    deliveredAt: z.coerce.date().optional(),
    status: z.enum(['sent', 'delivered', 'failed', 'bounced']),
    retryCount: z.number().int().default(0)
  })).optional(),

  // Actions taken
  actions: z.array(z.object({
    actionId: z.string().uuid(),
    actionType: z.string(),
    executedAt: z.coerce.date(),
    executedBy: z.string(),
    status: z.enum(['pending', 'success', 'failed']),
    result: z.string().optional(),
    error: z.string().optional()
  })).optional(),

  // Impact assessment
  impact: z.object({
    affectedEquipment: z.array(z.string()).optional(),
    affectedSamples: z.number().int().optional(),
    estimatedSampleValue: z.number().optional().describe('USD value of affected samples'),
    riskLevel: z.enum(['critical', 'high', 'medium', 'low']).optional(),
    complianceImpact: z.boolean().optional()
  }).optional(),

  // Metadata
  metadata: z.record(z.any()).optional(),
  schemaVersion: z.string().default('1.0.0')
});

export type Alert = z.infer<typeof AlertSchema>;

/**
 * Example alert
 */
export const exampleCriticalAlert: Alert = {
  alertId: '660e8400-e29b-41d4-a716-446655440000',
  alertRuleId: 'rule-temp-high-critical',

  source: {
    sensorId: 'sensor-temp-001',
    equipmentId: 'ln2-tank-001',
    facilityId: 'facility-main-lab',
    readingId: '550e8400-e29b-41d4-a716-446655440000'
  },

  classification: {
    type: 'threshold-exceeded',
    severity: 'critical',
    category: 'temperature'
  },

  timing: {
    detected: new Date('2026-01-11T10:30:00Z'),
    triggered: new Date('2026-01-11T10:35:00Z'),
    durationSeconds: 300
  },

  condition: {
    parameter: 'temperature',
    currentValue: -138.5,
    threshold: -140,
    operator: 'gt',
    unit: 'celsius',
    description: 'Temperature exceeded critical upper limit'
  },

  state: {
    status: 'active',
    escalationLevel: 1
  },

  notifications: [
    {
      notificationId: '770e8400-e29b-41d4-a716-446655440000',
      channel: 'email',
      recipient: 'lab-manager@facility.com',
      sentAt: new Date('2026-01-11T10:35:01Z'),
      deliveredAt: new Date('2026-01-11T10:35:02Z'),
      status: 'delivered',
      retryCount: 0
    },
    {
      notificationId: '880e8400-e29b-41d4-a716-446655440000',
      channel: 'sms',
      recipient: '+1-555-0123',
      sentAt: new Date('2026-01-11T10:35:01Z'),
      deliveredAt: new Date('2026-01-11T10:35:03Z'),
      status: 'delivered',
      retryCount: 0
    }
  ],

  impact: {
    affectedEquipment: ['ln2-tank-001'],
    affectedSamples: 27000,
    estimatedSampleValue: 2500000,
    riskLevel: 'critical',
    complianceImpact: true
  },

  schemaVersion: '1.0.0'
};
```

### 2.3 Equipment Status Schema

```typescript
/**
 * Equipment status data structure
 *
 * Comprehensive status information for cryogenic equipment including
 * operational parameters, health metrics, and maintenance tracking.
 */

export const EquipmentStatusSchema = z.object({
  // Identification
  statusId: z.string().uuid(),
  equipmentId: z.string(),
  timestamp: z.coerce.date(),

  // Operational status
  operational: z.object({
    status: z.enum([
      'operational',
      'standby',
      'maintenance',
      'alarm',
      'offline',
      'error'
    ]),
    uptime: z.number().describe('Uptime in seconds'),
    lastStartup: z.coerce.date().optional(),
    lastShutdown: z.coerce.date().optional(),
    operatingMode: z.enum([
      'normal',
      'backup',
      'emergency',
      'test',
      'calibration'
    ]).optional()
  }),

  // Current measurements
  currentMeasurements: z.object({
    temperature: z.object({
      value: z.number(),
      unit: z.string(),
      setpoint: z.number().optional(),
      deviation: z.number().optional()
    }).optional(),

    pressure: z.object({
      value: z.number(),
      unit: z.string(),
      setpoint: z.number().optional()
    }).optional(),

    liquidLevel: z.object({
      value: z.number(),
      unit: z.enum(['percent', 'liters', 'inches']),
      capacity: z.number().optional(),
      timeToRefill: z.number().optional().describe('Estimated hours until refill needed')
    }).optional(),

    power: z.object({
      voltage: z.number().optional(),
      current: z.number().optional(),
      consumption: z.number().optional().describe('Watts'),
      frequency: z.number().optional()
    }).optional()
  }),

  // Performance metrics
  performance: z.object({
    efficiency: z.number().min(0).max(100).optional().describe('Operating efficiency %'),
    cycleCount: z.number().int().optional().describe('Number of cooling cycles'),
    averageCycleTime: z.number().optional().describe('Average cycle time in seconds'),

    // Temperature performance
    temperatureStability: z.object({
      deviation: z.number().optional().describe('Temperature deviation from setpoint'),
      uniformity: z.number().optional().describe('Temperature uniformity across zones'),
      recoveryTime: z.number().optional().describe('Time to recover from door opening (seconds)')
    }).optional(),

    // Energy metrics
    energy: z.object({
      dailyConsumption: z.number().optional().describe('kWh per day'),
      comparedToBaseline: z.number().optional().describe('Percentage vs baseline'),
      trend: z.enum(['increasing', 'stable', 'decreasing']).optional()
    }).optional()
  }).optional(),

  // Health indicators
  health: z.object({
    overall: z.enum(['excellent', 'good', 'fair', 'poor', 'critical']),
    score: z.number().min(0).max(100).optional().describe('Overall health score'),

    // Component health
    components: z.array(z.object({
      component: z.string(),
      status: z.enum(['healthy', 'warning', 'critical', 'failed']),
      lastMaintenance: z.coerce.date().optional(),
      nextMaintenance: z.coerce.date().optional(),
      remainingLife: z.number().optional().describe('Estimated remaining life percentage')
    })).optional(),

    // Predictive indicators
    predictions: z.array(z.object({
      type: z.enum(['failure', 'maintenance', 'performance-degradation']),
      component: z.string(),
      confidence: z.number().min(0).max(100),
      timeframe: z.string().describe('e.g., "within 30 days"'),
      recommendation: z.string()
    })).optional()
  }),

  // Alarms and warnings
  alarms: z.object({
    active: z.array(z.object({
      alarmId: z.string(),
      type: z.string(),
      severity: z.enum(['critical', 'high', 'medium', 'low']),
      triggeredAt: z.coerce.date(),
      message: z.string()
    })),
    count: z.object({
      critical: z.number().int(),
      high: z.number().int(),
      medium: z.number().int(),
      low: z.number().int()
    })
  }).optional(),

  // Maintenance tracking
  maintenance: z.object({
    lastPerformed: z.coerce.date().optional(),
    nextScheduled: z.coerce.date().optional(),
    daysUntilDue: z.number().int().optional(),
    overdue: z.boolean(),

    history: z.array(z.object({
      date: z.coerce.date(),
      type: z.enum(['preventive', 'corrective', 'calibration', 'inspection']),
      performedBy: z.string(),
      workOrder: z.string().optional(),
      notes: z.string().optional()
    })).optional()
  }).optional(),

  // Environmental conditions
  environment: z.object({
    ambientTemperature: z.number().optional(),
    ambientHumidity: z.number().optional(),
    roomConditions: z.enum(['normal', 'warning', 'alarm']).optional()
  }).optional(),

  // Metadata
  metadata: z.record(z.any()).optional(),
  schemaVersion: z.string().default('1.0.0')
});

export type EquipmentStatus = z.infer<typeof EquipmentStatusSchema>;
```

### 2.4 Dashboard Metrics Schema

```typescript
/**
 * Dashboard metrics data structure
 *
 * Aggregated metrics and KPIs for dashboard visualization
 */

export const DashboardMetricsSchema = z.object({
  // Identification
  metricsId: z.string().uuid(),
  facilityId: z.string(),
  timeRange: z.object({
    start: z.coerce.date(),
    end: z.coerce.date(),
    aggregation: z.enum(['1min', '5min', '15min', '1hour', '1day', '1week', '1month'])
  }),
  generatedAt: z.coerce.date(),

  // Equipment summary
  equipment: z.object({
    total: z.number().int(),
    operational: z.number().int(),
    alarm: z.number().int(),
    maintenance: z.number().int(),
    offline: z.number().int(),

    byType: z.record(z.object({
      count: z.number().int(),
      operational: z.number().int()
    })).optional()
  }),

  // Sensor summary
  sensors: z.object({
    total: z.number().int(),
    active: z.number().int(),
    warning: z.number().int(),
    offline: z.number().int(),
    calibrationDue: z.number().int()
  }),

  // Alert summary
  alerts: z.object({
    active: z.number().int(),
    acknowledged: z.number().int(),
    resolved: z.number().int(),

    bySeverity: z.object({
      critical: z.number().int(),
      high: z.number().int(),
      medium: z.number().int(),
      low: z.number().int()
    }),

    byCategory: z.record(z.number().int()).optional(),

    responseTime: z.object({
      average: z.number().describe('Average response time in seconds'),
      median: z.number().optional(),
      p95: z.number().optional()
    }).optional()
  }),

  // Temperature metrics
  temperature: z.object({
    averages: z.array(z.object({
      equipmentId: z.string(),
      value: z.number(),
      min: z.number(),
      max: z.number(),
      stddev: z.number().optional()
    })),

    excursions: z.object({
      count: z.number().int(),
      totalDuration: z.number().describe('Total excursion time in seconds'),
      events: z.array(z.object({
        equipmentId: z.string(),
        start: z.coerce.date(),
        end: z.coerce.date().optional(),
        peak: z.number(),
        duration: z.number()
      }))
    }).optional()
  }).optional(),

  // Compliance metrics
  compliance: z.object({
    validatedReadings: z.number().int(),
    totalReadings: z.number().int(),
    validationRate: z.number().min(0).max(100),

    calibration: z.object({
      current: z.number().int(),
      due: z.number().int(),
      overdue: z.number().int()
    }),

    documentation: z.object({
      complete: z.boolean(),
      missingItems: z.array(z.string()).optional()
    }).optional()
  }).optional(),

  // System health
  systemHealth: z.object({
    overall: z.enum(['healthy', 'warning', 'critical']),
    score: z.number().min(0).max(100),

    dataQuality: z.object({
      completeness: z.number().min(0).max(100),
      accuracy: z.number().min(0).max(100),
      timeliness: z.number().min(0).max(100)
    }).optional(),

    availability: z.object({
      uptime: z.number().min(0).max(100).describe('System uptime percentage'),
      dataLoss: z.number().min(0).max(100).describe('Percentage of data loss')
    }).optional()
  }),

  // Trends
  trends: z.array(z.object({
    metric: z.string(),
    direction: z.enum(['improving', 'stable', 'degrading']),
    changePercent: z.number(),
    significance: z.enum(['high', 'medium', 'low'])
  })).optional(),

  // Metadata
  metadata: z.record(z.any()).optional(),
  schemaVersion: z.string().default('1.0.0')
});

export type DashboardMetrics = z.infer<typeof DashboardMetricsSchema>;
```

### 2.5 Audit Log Schema

```typescript
/**
 * Audit log entry schema
 *
 * Complete audit trail for compliance and forensic analysis
 */

export const AuditLogEntrySchema = z.object({
  // Identification
  entryId: z.string().uuid(),
  timestamp: z.coerce.date(),
  sequenceNumber: z.number().int().positive().describe('Monotonically increasing sequence'),

  // Actor information
  actor: z.object({
    userId: z.string(),
    userName: z.string().optional(),
    userRole: z.string().optional(),
    ipAddress: z.string().optional(),
    userAgent: z.string().optional(),
    sessionId: z.string().optional()
  }),

  // Action details
  action: z.object({
    type: z.enum([
      'create',
      'read',
      'update',
      'delete',
      'login',
      'logout',
      'alert-acknowledge',
      'alert-resolve',
      'calibration',
      'maintenance',
      'configuration-change',
      'data-export',
      'report-generation',
      'system-start',
      'system-stop'
    ]),
    category: z.enum([
      'authentication',
      'data-access',
      'data-modification',
      'system-configuration',
      'alert-management',
      'equipment-management',
      'compliance'
    ]),
    description: z.string(),
    result: z.enum(['success', 'failure', 'partial']),
    errorMessage: z.string().optional()
  }),

  // Target information
  target: z.object({
    resourceType: z.string().describe('e.g., equipment, sensor, alert, user'),
    resourceId: z.string(),
    resourceName: z.string().optional(),
    previousState: z.record(z.any()).optional(),
    newState: z.record(z.any()).optional()
  }).optional(),

  // Compliance tracking
  compliance: z.object({
    framework: z.string().optional().describe('e.g., FDA 21 CFR Part 11'),
    requiresSignature: z.boolean().default(false),
    signature: z.string().optional(),
    witnessSignature: z.string().optional(),
    reason: z.string().optional().describe('Reason for change')
  }).optional(),

  // Additional context
  context: z.object({
    facilityId: z.string().optional(),
    equipmentId: z.string().optional(),
    correlationId: z.string().optional().describe('Link related audit entries'),
    tags: z.array(z.string()).optional()
  }).optional(),

  // Metadata
  metadata: z.record(z.any()).optional(),
  schemaVersion: z.string().default('1.0.0')
});

export type AuditLogEntry = z.infer<typeof AuditLogEntrySchema>;
```

---

## 3. Data Validation and Processing

```typescript
/**
 * Data validation utilities
 */

export class CryoDataValidator {
  /**
   * Validate sensor reading
   */
  static validateSensorReading(data: unknown): SensorReading {
    return SensorReadingSchema.parse(data);
  }

  /**
   * Validate sensor reading with detailed error reporting
   */
  static validateSensorReadingSafe(data: unknown): {
    success: boolean;
    data?: SensorReading;
    errors?: z.ZodError;
  } {
    const result = SensorReadingSchema.safeParse(data);

    if (result.success) {
      return { success: true, data: result.data };
    } else {
      return { success: false, errors: result.error };
    }
  }

  /**
   * Validate alert
   */
  static validateAlert(data: unknown): Alert {
    return AlertSchema.parse(data);
  }

  /**
   * Validate equipment status
   */
  static validateEquipmentStatus(data: unknown): EquipmentStatus {
    return EquipmentStatusSchema.parse(data);
  }

  /**
   * Validate dashboard metrics
   */
  static validateDashboardMetrics(data: unknown): DashboardMetrics {
    return DashboardMetricsSchema.parse(data);
  }

  /**
   * Validate audit log entry
   */
  static validateAuditLogEntry(data: unknown): AuditLogEntry {
    return AuditLogEntrySchema.parse(data);
  }

  /**
   * Batch validation
   */
  static validateBatch<T>(
    data: unknown[],
    schema: z.ZodSchema<T>
  ): {
    valid: T[];
    invalid: Array<{ index: number; data: unknown; errors: z.ZodError }>;
  } {
    const valid: T[] = [];
    const invalid: Array<{ index: number; data: unknown; errors: z.ZodError }> = [];

    data.forEach((item, index) => {
      const result = schema.safeParse(item);
      if (result.success) {
        valid.push(result.data);
      } else {
        invalid.push({ index, data: item, errors: result.error });
      }
    });

    return { valid, invalid };
  }
}

/**
 * Data transformation utilities
 */
export class CryoDataTransformer {
  /**
   * Convert Celsius to Fahrenheit
   */
  static celsiusToFahrenheit(celsius: number): number {
    return (celsius * 9/5) + 32;
  }

  /**
   * Convert Fahrenheit to Celsius
   */
  static fahrenheitToCelsius(fahrenheit: number): number {
    return (fahrenheit - 32) * 5/9;
  }

  /**
   * Convert PSI to bar
   */
  static psiToBar(psi: number): number {
    return psi * 0.0689476;
  }

  /**
   * Convert bar to PSI
   */
  static barToPsi(bar: number): number {
    return bar * 14.5038;
  }

  /**
   * Normalize sensor reading to standard units
   */
  static normalizeSensorReading(reading: SensorReading): SensorReading {
    const normalized = { ...reading };

    // Convert temperature to Celsius if needed
    if (reading.measurement.parameter === 'temperature') {
      if (reading.measurement.unit === 'fahrenheit') {
        normalized.measurement.value = this.fahrenheitToCelsius(reading.measurement.value);
        normalized.measurement.unit = 'celsius';
      } else if (reading.measurement.unit === 'kelvin') {
        normalized.measurement.value = reading.measurement.value - 273.15;
        normalized.measurement.unit = 'celsius';
      }
    }

    // Convert pressure to bar if needed
    if (reading.measurement.parameter === 'pressure') {
      if (reading.measurement.unit === 'psi') {
        normalized.measurement.value = this.psiToBar(reading.measurement.value);
        normalized.measurement.unit = 'bar';
      }
    }

    return normalized;
  }

  /**
   * Aggregate sensor readings
   */
  static aggregateReadings(
    readings: SensorReading[],
    aggregationType: 'average' | 'min' | 'max' | 'last'
  ): number | undefined {
    if (readings.length === 0) return undefined;

    const values = readings.map(r => r.measurement.value);

    switch (aggregationType) {
      case 'average':
        return values.reduce((a, b) => a + b, 0) / values.length;
      case 'min':
        return Math.min(...values);
      case 'max':
        return Math.max(...values);
      case 'last':
        return values[values.length - 1];
    }
  }

  /**
   * Calculate statistics for readings
   */
  static calculateStatistics(readings: SensorReading[]): {
    min: number;
    max: number;
    average: number;
    stddev: number;
    count: number;
  } | undefined {
    if (readings.length === 0) return undefined;

    const values = readings.map(r => r.measurement.value);
    const min = Math.min(...values);
    const max = Math.max(...values);
    const average = values.reduce((a, b) => a + b, 0) / values.length;

    // Calculate standard deviation
    const squareDiffs = values.map(value => Math.pow(value - average, 2));
    const avgSquareDiff = squareDiffs.reduce((a, b) => a + b, 0) / squareDiffs.length;
    const stddev = Math.sqrt(avgSquareDiff);

    return {
      min,
      max,
      average,
      stddev,
      count: readings.length
    };
  }
}
```

---

## Conclusion

The WIA Cryo Monitoring Standard provides comprehensive, type-safe data schemas with runtime validation using Zod. These schemas ensure data integrity, interoperability, and compliance across all cryogenic monitoring implementations.

**弘益人間 (Hongik Ingan)** - Standardized data structures that serve all of humanity's monitoring needs.

---

© 2026 World Industry Association
Licensed under Apache 2.0
