# WIA Exoskeleton Safety Checklist Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14
**Safety Classification:** ISO 13849 PLd / IEC 62304 Class B

## 1. Overview

이 문서는 재활 외골격의 안전한 운용을 위한 체크리스트 시스템을 정의합니다.
운용 전, 중, 후 점검 항목과 자동/수동 검사 절차를 포함합니다.

## 2. Checklist Categories

### 2.1 Category Overview

| Category | Timing | Responsibility | Required |
|----------|--------|----------------|----------|
| Pre-Operation | Before each session | Operator + System | Mandatory |
| During Operation | Continuous | System | Automatic |
| Post-Operation | After each session | Operator + System | Mandatory |
| Periodic Maintenance | Scheduled | Technician | Mandatory |
| Emergency | As needed | Anyone | Mandatory |

### 2.2 Check Types

| Type | Method | Frequency | Pass Criteria |
|------|--------|-----------|---------------|
| Automatic | System self-test | Every startup | All tests pass |
| Visual | Operator inspection | Every session | No defects |
| Functional | Manual test | Every session | Functions correctly |
| Documented | Review records | Weekly/Monthly | Up to date |

## 3. Pre-Operation Checklist

### 3.1 Hardware Checks

```typescript
interface HardwareChecklist {
  // Power system
  battery: {
    level: () => CheckResult;        // > 20%
    voltage: () => CheckResult;      // Within range
    temperature: () => CheckResult;  // < 45°C
    connections: () => CheckResult;  // Secure
  };

  // Motors
  motors: {
    connections: () => CheckResult;  // All connected
    resistance: () => CheckResult;   // Within spec
    encoders: () => CheckResult;     // Responding
    temperature: () => CheckResult;  // < 40°C
  };

  // Sensors
  sensors: {
    imu: () => CheckResult;          // Calibrated
    forceSensors: () => CheckResult; // Zeroed
    encoders: () => CheckResult;     // Reading correctly
    temperature: () => CheckResult;  // All responding
  };

  // Mechanical
  mechanical: {
    fasteners: () => CheckResult;    // Torqued
    joints: () => CheckResult;       // No binding
    cables: () => CheckResult;       // No damage
    structure: () => CheckResult;    // No cracks
  };

  // Safety systems
  safety: {
    eStopButton: () => CheckResult;  // Functional
    limitSwitches: () => CheckResult;// Functional
    watchdog: () => CheckResult;     // Active
    alarms: () => CheckResult;       // Functional
  };
}
```

### 3.2 Hardware Check Details

| Check | Method | Pass Criteria | Fail Action |
|-------|--------|---------------|-------------|
| Battery Level | Auto | > 20% SOC | Charge battery |
| Battery Voltage | Auto | 22-56V | Check battery |
| Battery Temperature | Auto | < 45°C | Cool down |
| Motor Connections | Auto | All responding | Check wiring |
| Motor Resistance | Auto | Within ±10% | Replace motor |
| Encoder Response | Auto | Valid data | Check encoder |
| IMU Calibration | Auto | Drift < 1°/min | Recalibrate |
| E-Stop Function | Manual | Triggers stop | Repair |
| Limit Switches | Manual | Triggers at limit | Repair |
| Fastener Torque | Manual | Per spec | Retorque |
| Cable Condition | Visual | No damage | Replace |

### 3.3 User Checks

```typescript
interface UserChecklist {
  // Fitting
  fitting: {
    sizeCorrect: () => CheckResult;      // Correct size selected
    strapsTight: () => CheckResult;      // Secure but comfortable
    alignment: () => CheckResult;        // Joint alignment correct
    clearance: () => CheckResult;        // No pinch points
  };

  // Comfort
  comfort: {
    pressurePoints: () => CheckResult;   // No excessive pressure
    skinCondition: () => CheckResult;    // No irritation
    temperature: () => CheckResult;      // Not too hot/cold
    mobility: () => CheckResult;         // Can move safely
  };

  // Range of motion
  rom: {
    hipFlexion: () => CheckResult;       // Within limits
    hipExtension: () => CheckResult;     // Within limits
    kneeFlexion: () => CheckResult;      // Within limits
    ankleRange: () => CheckResult;       // Within limits
  };

  // User readiness
  readiness: {
    understoodInstructions: () => CheckResult;
    knowsEmergencyProcedure: () => CheckResult;
    physicallyReady: () => CheckResult;
    noContraindications: () => CheckResult;
  };
}
```

### 3.4 User Check Details

| Check | Method | Pass Criteria | Fail Action |
|-------|--------|---------------|-------------|
| Size Selection | Manual | Matches prescription | Adjust size |
| Strap Tension | Manual | Snug, not painful | Readjust |
| Joint Alignment | Manual | < 5° offset | Realign |
| Pressure Points | Manual | No pain | Add padding |
| Skin Condition | Visual | No redness | Adjust fit |
| ROM Test | Functional | Full prescribed ROM | Adjust limits |
| Emergency Knowledge | Verbal | Can describe procedure | Reteach |

## 4. During Operation Monitoring

### 4.1 Continuous Monitoring

```typescript
interface ContinuousMonitoring {
  // System health
  system: {
    processsorLoad: MonitoredValue;    // < 80%
    memoryUsage: MonitoredValue;       // < 90%
    communicationLatency: MonitoredValue; // < 10ms
    watchdogStatus: MonitoredValue;    // Active
  };

  // Motor health
  motors: {
    current: MonitoredValue[];         // Within limits
    temperature: MonitoredValue[];     // < 80°C
    velocity: MonitoredValue[];        // Within limits
    position: MonitoredValue[];        // Within limits
  };

  // User safety
  user: {
    gaitSymmetry: MonitoredValue;      // > 80%
    fatigue: MonitoredValue;           // < 70%
    heartRate: MonitoredValue;         // Within target
    bloodPressure: MonitoredValue;     // Optional
  };

  // Environmental
  environment: {
    ambientTemp: MonitoredValue;       // 15-35°C
    humidity: MonitoredValue;          // 20-80%
    surfaceType: MonitoredValue;       // Detected
  };
}

interface MonitoredValue {
  current: number;
  min: number;
  max: number;
  average: number;
  trend: 'stable' | 'increasing' | 'decreasing';
  status: 'ok' | 'warning' | 'critical';
  lastUpdate: number;
}
```

### 4.2 Alert Thresholds

| Parameter | Warning | Alert | Critical |
|-----------|---------|-------|----------|
| Processor Load | 70% | 80% | 90% |
| Motor Temperature | 60°C | 75°C | 85°C |
| Motor Current | 80% rated | 100% rated | 150% rated |
| Fatigue Level | 50% | 70% | 85% |
| Gait Symmetry | < 80% | < 70% | < 60% |
| Communication Latency | 5 ms | 10 ms | 20 ms |

### 4.3 Automated Responses

| Condition | Response | Priority |
|-----------|----------|----------|
| Warning threshold | Log + Display | Low |
| Alert threshold | Audio alert + Reduce power | Medium |
| Critical threshold | Stop operation | High |
| Multiple warnings | Operator notification | Medium |
| Rapid parameter change | Immediate review | High |

## 5. Post-Operation Checklist

### 5.1 System Shutdown

```typescript
interface PostOperationChecklist {
  // Controlled shutdown
  shutdown: {
    returnToHome: () => CheckResult;   // Joints at safe position
    disengageMotors: () => CheckResult;// Motors off
    saveSessionData: () => CheckResult;// Data saved
    generateReport: () => CheckResult; // Report created
  };

  // System state
  systemState: {
    errorLogs: () => CheckResult;      // No critical errors
    warnings: () => CheckResult;       // All acknowledged
    temperatures: () => CheckResult;   // Cooling down
    batteryLevel: () => CheckResult;   // Record level
  };

  // User removal
  userRemoval: {
    safePosition: () => CheckResult;   // User can exit safely
    strapsReleased: () => CheckResult; // All straps undone
    skinCheck: () => CheckResult;      // No injuries
    userFeedback: () => CheckResult;   // Comfort survey
  };

  // Equipment care
  equipment: {
    cleaning: () => CheckResult;       // Surfaces cleaned
    disinfection: () => CheckResult;   // Pads disinfected
    storage: () => CheckResult;        // Proper storage
    charging: () => CheckResult;       // Battery charging
  };
}
```

### 5.2 Session Documentation

```typescript
interface SessionReport {
  // Session info
  sessionId: string;
  userId: string;
  operatorId: string;
  startTime: Date;
  endTime: Date;
  duration: number;

  // Pre-operation results
  preOpChecklist: ChecklistResult;

  // Session summary
  summary: {
    distance: number;
    steps: number;
    averageSpeed: number;
    assistanceUsed: number;
    peakTorques: JointTorques;
  };

  // Safety events
  safetyEvents: SafetyEvent[];
  warnings: Warning[];
  interventions: Intervention[];

  // Equipment status
  equipmentStatus: {
    batteryStartLevel: number;
    batteryEndLevel: number;
    maxMotorTemperatures: number[];
    errorCodes: string[];
  };

  // User feedback
  userFeedback: {
    comfortRating: number;         // 1-10
    fatigueRating: number;         // 1-10
    painReported: boolean;
    painLocation?: string;
    comments?: string;
  };

  // Post-operation results
  postOpChecklist: ChecklistResult;
}
```

## 6. Periodic Maintenance Checklist

### 6.1 Maintenance Schedule

| Interval | Check Type | Duration | Technician Required |
|----------|------------|----------|---------------------|
| Daily | Visual inspection | 5 min | No |
| Weekly | Functional test | 15 min | No |
| Monthly | Calibration check | 30 min | Yes |
| Quarterly | Full inspection | 2 hours | Yes |
| Annually | Complete overhaul | 8 hours | Yes |

### 6.2 Monthly Maintenance

```typescript
interface MonthlyMaintenance {
  // Calibration
  calibration: {
    encoderCalibration: () => CheckResult;
    forceCalibration: () => CheckResult;
    imuCalibration: () => CheckResult;
    currentSensorCalibration: () => CheckResult;
  };

  // Mechanical
  mechanical: {
    bearingInspection: () => CheckResult;
    gearboxInspection: () => CheckResult;
    beltTension: () => CheckResult;
    lubricant: () => CheckResult;
  };

  // Electrical
  electrical: {
    connectionResistance: () => CheckResult;
    insulationTest: () => CheckResult;
    groundContinuity: () => CheckResult;
    cableWear: () => CheckResult;
  };

  // Software
  software: {
    firmwareVersion: () => CheckResult;
    logAnalysis: () => CheckResult;
    parameterBackup: () => CheckResult;
    securityUpdate: () => CheckResult;
  };
}
```

### 6.3 Maintenance Records

```typescript
interface MaintenanceRecord {
  id: string;
  date: Date;
  type: 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annual';
  technician: string;
  deviceId: string;
  hoursAtService: number;

  checklistResults: ChecklistResult;

  replacedParts: {
    partNumber: string;
    partName: string;
    reason: string;
    oldSerialNumber: string;
    newSerialNumber: string;
  }[];

  adjustments: {
    parameter: string;
    oldValue: number;
    newValue: number;
    reason: string;
  }[];

  notes: string;
  nextServiceDue: Date;
}
```

## 7. Emergency Checklist

### 7.1 Emergency Procedures

```typescript
interface EmergencyProcedures {
  // E-Stop triggered
  eStop: {
    verifyStop: () => void;           // Confirm system stopped
    assessSituation: () => void;      // Check for injuries
    secureUser: () => void;           // Ensure user safety
    contactEmergency: () => void;     // Call for help if needed
    documentIncident: () => void;     // Record what happened
  };

  // Fall detection
  fall: {
    assessUser: () => void;           // Check for injuries
    doNotMove: () => void;            // Don't move if suspected injury
    callForHelp: () => void;          // Get assistance
    monitorVitals: () => void;        // Watch breathing, pulse
    documentIncident: () => void;     // Record details
  };

  // Power failure
  powerFailure: {
    engageBrakes: () => void;         // Mechanical brakes engage
    supportUser: () => void;          // Prevent falls
    manualRelease: () => void;        // Release user from device
    reportIssue: () => void;          // Document failure
  };

  // Fire/evacuation
  evacuation: {
    initiateEStop: () => void;        // Stop operation
    releaseUser: () => void;          // Quick release
    evacuate: () => void;             // Follow evacuation plan
    accountForAll: () => void;        // Verify all safe
  };
}
```

### 7.2 Emergency Contact Information

```typescript
interface EmergencyContacts {
  internal: {
    safetyOfficer: ContactInfo;
    technicalSupport: ContactInfo;
    medicalStaff: ContactInfo;
    supervisor: ContactInfo;
  };

  external: {
    emergencyServices: string;        // 119 / 911
    poisonControl: string;
    manufacturer: ContactInfo;
    serviceCenter: ContactInfo;
  };
}

interface ContactInfo {
  name: string;
  phone: string;
  email: string;
  available: string;                  // e.g., "24/7" or "9-5"
}
```

## 8. Checklist Interface

### 8.1 API Interface

```typescript
interface IChecklistSystem {
  // Run checklists
  runPreOperationChecklist(): Promise<ChecklistResult>;
  runPostOperationChecklist(): Promise<ChecklistResult>;
  runMaintenanceChecklist(type: MaintenanceType): Promise<ChecklistResult>;

  // Individual checks
  runCheck(checkId: string): Promise<CheckResult>;
  runCategory(category: CheckCategory): Promise<ChecklistResult>;

  // Manual confirmation
  confirmManualCheck(checkId: string, result: boolean, notes?: string): void;

  // Status
  getChecklistStatus(): ChecklistStatus;
  getRequiredChecks(): CheckItem[];
  getOverdueChecks(): CheckItem[];

  // Configuration
  configure(config: ChecklistConfig): void;
  addCustomCheck(check: CustomCheck): void;
  removeCustomCheck(checkId: string): void;

  // History
  getCheckHistory(options: HistoryOptions): ChecklistResult[];
  exportReport(sessionId: string): SessionReport;
}
```

### 8.2 Check Result Structure

```typescript
interface CheckResult {
  checkId: string;
  name: string;
  category: CheckCategory;
  timestamp: number;
  passed: boolean;
  value?: number | string | boolean;
  expectedValue?: number | string | boolean;
  method: 'automatic' | 'manual' | 'visual';
  operatorId?: string;
  notes?: string;
  failureReason?: string;
  retryCount?: number;
}

interface ChecklistResult {
  id: string;
  type: ChecklistType;
  timestamp: number;
  operatorId: string;
  deviceId: string;

  overall: {
    passed: boolean;
    totalChecks: number;
    passedChecks: number;
    failedChecks: number;
    skippedChecks: number;
  };

  categories: {
    [category: string]: {
      passed: boolean;
      checks: CheckResult[];
    };
  };

  criticalFailures: CheckResult[];
  warnings: CheckResult[];

  duration: number;
  signoff?: {
    operatorId: string;
    signature: string;
    timestamp: number;
  };
}
```

## 9. Checklist Configuration

```typescript
interface ChecklistConfig {
  // Required checks
  requiredCategories: CheckCategory[];
  criticalChecks: string[];

  // Timing
  preOpTimeout: number;           // Max time to complete
  checkRetryLimit: number;        // Max retries per check
  autoCheckInterval: number;      // During operation (ms)

  // Overrides
  allowSkip: boolean;             // Allow skipping non-critical
  requireSignoff: boolean;        // Require operator signature
  requireSupervisorApproval: boolean;

  // Notifications
  notifyOnFailure: boolean;
  notifyOnOverdue: boolean;
  escalationContacts: string[];
}
```

## 10. Related Specifications

- [EMERGENCY-STOP.md](./EMERGENCY-STOP.md) - 비상 정지 명세
- [JOINT-LIMITS.md](./JOINT-LIMITS.md) - 관절 가동 범위 제한
- [OVERLOAD-DETECTION.md](./OVERLOAD-DETECTION.md) - 과부하 감지

## 11. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
