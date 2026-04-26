# Chapter 8: Implementation Guide

## Overview

This chapter provides comprehensive guidance for implementing WIA-IND-012 compliant fitness tracking systems, covering sensor integration, accuracy standards, battery optimization, privacy compliance, and certification requirements.

---

## 8.1 Sensor Integration Requirements

### 8.1.1 Accelerometer

**Sampling Rate:**
```
Minimum: 50 Hz
Recommended: 100 Hz
Maximum: 200 Hz (diminishing returns beyond this)
```

**Purpose:**
- Step detection
- Activity classification
- Movement patterns
- Fall detection

**Configuration:**
```typescript
interface AccelerometerConfig {
  samplingRate: number;        // Hz
  sensitivity: 'low' | 'medium' | 'high';
  filterType: 'low_pass' | 'high_pass' | 'band_pass';
  cutoffFrequency: number;     // Hz
}

const config: AccelerometerConfig = {
  samplingRate: 100,
  sensitivity: 'medium',
  filterType: 'high_pass',
  cutoffFrequency: 0.5
};
```

**Step Detection Algorithm:**
```typescript
class StepDetector {
  private threshold: number = 1.2;  // g-force threshold
  private minPeakDistance: number = 200;  // ms
  private lastStepTime: number = 0;
  private stepCount: number = 0;

  processAccelData(x: number, y: number, z: number, timestamp: number): boolean {
    // Calculate magnitude
    const magnitude = Math.sqrt(x * x + y * y + z * z);

    // Check for step
    if (magnitude > this.threshold &&
        timestamp - this.lastStepTime > this.minPeakDistance) {

      this.stepCount++;
      this.lastStepTime = timestamp;
      return true;  // Step detected
    }

    return false;
  }

  getStepCount(): number {
    return this.stepCount;
  }

  reset(): void {
    this.stepCount = 0;
    this.lastStepTime = 0;
  }
}
```

### 8.1.2 GPS

**Sampling Rate:**
```
Activity Type       Rate        Reason
Walking/Hiking:     1 Hz        Slow movement, battery efficient
Running:            1-5 Hz      Balance accuracy and battery
Cycling:            5 Hz        Higher speed requires more points
Racing:             10 Hz       Maximum accuracy
```

**Accuracy Requirements:**
```typescript
interface GPSAccuracy {
  horizontal: number;      // meters (±)
  vertical: number;        // meters (±)
  confidence: number;      // 0-100%
}

const minimumAccuracy: GPSAccuracy = {
  horizontal: 10,
  vertical: 15,
  confidence: 95
};
```

**GPS Point Validation:**
```typescript
function isValidGPSPoint(
  point: GPSPoint,
  previousPoint: GPSPoint | null
): boolean {
  // Check basic validity
  if (point.latitude < -90 || point.latitude > 90) return false;
  if (point.longitude < -180 || point.longitude > 180) return false;
  if (point.accuracy && point.accuracy > 50) return false;  // Poor accuracy

  // Check for unrealistic speed (if previous point exists)
  if (previousPoint) {
    const timeDiff = (point.timestamp - previousPoint.timestamp) / 1000;  // seconds
    const distance = calculateDistance(previousPoint, point);  // meters
    const speed = distance / timeDiff;  // m/s

    // Reject if speed > 15 m/s (54 km/h) for running/walking
    if (speed > 15) return false;
  }

  return true;
}
```

### 8.1.3 Heart Rate Monitor

**Sampling Rate:**
```
Minimum: 1 Hz (1 reading per second)
Recommended: 5 Hz (5 readings per second)
HRV Measurement: 250-500 Hz (for R-R interval detection)
```

**Optical Heart Rate (PPG):**
```typescript
interface PPGConfig {
  ledIntensity: number;        // 0-100%
  samplingRate: number;        // Hz
  motionCompensation: boolean;
  skinTone: 'light' | 'medium' | 'dark';
}

class OpticalHRMonitor {
  private ppgSignal: number[] = [];
  private heartRates: number[] = [];

  processSignal(ppgValue: number, timestamp: number): void {
    this.ppgSignal.push(ppgValue);

    // Keep 10-second window
    if (this.ppgSignal.length > 50) {  // 5 Hz * 10 seconds
      this.ppgSignal.shift();

      // Calculate heart rate from peaks
      const hr = this.detectHeartRate();
      if (hr) {
        this.heartRates.push(hr);
      }
    }
  }

  private detectHeartRate(): number | null {
    // Find peaks in PPG signal
    const peaks = this.findPeaks(this.ppgSignal);

    if (peaks.length < 2) return null;

    // Calculate average interval between peaks
    const intervals: number[] = [];
    for (let i = 1; i < peaks.length; i++) {
      intervals.push(peaks[i] - peaks[i - 1]);
    }

    const avgInterval = intervals.reduce((a, b) => a + b) / intervals.length;

    // Convert to BPM (assuming 5 Hz sampling)
    const samplesPerSecond = 5;
    const secondsPerBeat = avgInterval / samplesPerSecond;
    const bpm = 60 / secondsPerBeat;

    return Math.round(bpm);
  }

  private findPeaks(signal: number[]): number[] {
    const peaks: number[] = [];
    const threshold = this.calculateThreshold(signal);

    for (let i = 1; i < signal.length - 1; i++) {
      if (signal[i] > signal[i - 1] &&
          signal[i] > signal[i + 1] &&
          signal[i] > threshold) {
        peaks.push(i);
      }
    }

    return peaks;
  }

  private calculateThreshold(signal: number[]): number {
    const mean = signal.reduce((a, b) => a + b) / signal.length;
    const stdDev = Math.sqrt(
      signal.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / signal.length
    );

    return mean + (0.5 * stdDev);
  }
}
```

### 8.1.4 Barometer

**Purpose:**
- Elevation tracking
- Stair climbing detection
- Altitude-based calorie adjustment

**Sampling Rate:**
```
Minimum: 1 Hz
Recommended: 2-5 Hz
```

**Elevation Calculation:**
```typescript
function calculateElevation(pressure: number, seaLevelPressure: number = 1013.25): number {
  // Barometric formula
  // h = (1 - (P/P0)^0.190284) * 44307.69
  const elevation = (1 - Math.pow(pressure / seaLevelPressure, 0.190284)) * 44307.69;
  return Math.round(elevation);  // meters
}
```

---

## 8.2 Accuracy Standards

### 8.2.1 Required Accuracy Thresholds

```typescript
interface AccuracyStandards {
  stepCount: {
    error: number;             // ±5%
    minimumCount: number;      // 100 steps for validation
  };

  distance: {
    gpsError: number;          // ±2% or 50m, whichever greater
    estimatedError: number;    // ±10%
  };

  heartRate: {
    error: number;             // ±5 BPM or ±5%, whichever greater
    restingRange: [number, number];   // 40-100 BPM valid
    activeRange: [number, number];    // 60-220 BPM valid
  };

  calories: {
    error: number;             // ±15%
  };

  elevation: {
    error: number;             // ±10m
  };
}

const accuracyStandards: AccuracyStandards = {
  stepCount: {
    error: 0.05,
    minimumCount: 100
  },
  distance: {
    gpsError: 0.02,
    estimatedError: 0.10
  },
  heartRate: {
    error: 5,
    restingRange: [40, 100],
    activeRange: [60, 220]
  },
  calories: {
    error: 0.15
  },
  elevation: {
    error: 10
  }
};
```

### 8.2.2 Validation Testing

**Step Count Validation:**
```typescript
interface ValidationTest {
  actualValue: number;
  measuredValue: number;
  passes: boolean;
  error: number;
}

function validateStepCount(actual: number, measured: number): ValidationTest {
  const error = Math.abs(actual - measured) / actual;
  const passes = error <= 0.05;

  return {
    actualValue: actual,
    measuredValue: measured,
    passes,
    error: error * 100
  };
}

// Example test
const test = validateStepCount(1000, 985);
console.log(`Error: ${test.error.toFixed(2)}%`);
console.log(`Passes: ${test.passes}`);  // true if error ≤ 5%
```

**Heart Rate Validation:**
```typescript
function validateHeartRate(reference: number, measured: number): boolean {
  const absoluteError = Math.abs(reference - measured);
  const percentError = absoluteError / reference;

  // Passes if ±5 BPM or ±5%, whichever is greater
  return absoluteError <= 5 || percentError <= 0.05;
}
```

---

## 8.3 Battery Optimization

### 8.3.1 Adaptive Sampling Strategy

```typescript
enum ActivityState {
  STATIONARY,
  WALKING,
  RUNNING,
  CYCLING,
  HIGH_INTENSITY
}

interface SamplingConfig {
  gpsRate: number;          // Hz
  accelerometerRate: number; // Hz
  heartRateRate: number;    // Hz
}

function getOptimalSamplingConfig(state: ActivityState): SamplingConfig {
  switch (state) {
    case ActivityState.STATIONARY:
      return {
        gpsRate: 0,           // GPS off
        accelerometerRate: 10, // Low rate for motion detection
        heartRateRate: 0.2    // Every 5 seconds
      };

    case ActivityState.WALKING:
      return {
        gpsRate: 1,
        accelerometerRate: 50,
        heartRateRate: 1
      };

    case ActivityState.RUNNING:
      return {
        gpsRate: 1,
        accelerometerRate: 100,
        heartRateRate: 1
      };

    case ActivityState.CYCLING:
      return {
        gpsRate: 5,            // Higher for speed
        accelerometerRate: 50,
        heartRateRate: 1
      };

    case ActivityState.HIGH_INTENSITY:
      return {
        gpsRate: 1,
        accelerometerRate: 100,
        heartRateRate: 5       // More frequent for HIIT
      };
  }
}
```

### 8.3.2 Batch Processing

**Instead of real-time sync, batch uploads:**
```typescript
class DataBatcher {
  private queue: any[] = [];
  private batchSize: number = 100;
  private batchInterval: number = 300000;  // 5 minutes

  addData(data: any): void {
    this.queue.push(data);

    if (this.queue.length >= this.batchSize) {
      this.flush();
    }
  }

  async flush(): Promise<void> {
    if (this.queue.length === 0) return;

    const batch = this.queue.splice(0, this.batchSize);

    try {
      await this.uploadBatch(batch);
      console.log(`Uploaded ${batch.length} records`);
    } catch (error) {
      // Re-queue failed items
      this.queue.unshift(...batch);
      console.error('Upload failed, will retry');
    }
  }

  private async uploadBatch(batch: any[]): Promise<void> {
    await fetch('/api/v1/data/batch', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ data: batch })
    });
  }
}
```

### 8.3.3 Power Consumption Estimates

```
Component           Power Draw      Battery Impact (1 hour)
GPS (continuous):   50-150 mW       ~5-10% battery
GPS (1 Hz):         20-50 mW        ~2-4% battery
Accelerometer:      0.5-2 mW        ~0.1% battery
Optical HR:         5-15 mW         ~0.5-1% battery
Bluetooth:          10-30 mW        ~1-2% battery
Screen (active):    200-500 mW      ~15-25% battery

Total (active tracking with screen off):
~30-80 mW = 3-6 hours of continuous tracking
```

---

## 8.4 Privacy Compliance

### 8.4.1 GDPR Compliance Checklist

**Data Collection:**
```typescript
interface ConsentRecord {
  userId: string;
  purpose: string;
  granted: boolean;
  timestamp: Date;
  ipAddress?: string;
  userAgent?: string;
}

class GDPRCompliance {
  // Obtain explicit consent
  async requestConsent(userId: string, purpose: string): Promise<boolean> {
    const consent = await this.showConsentDialog(purpose);

    const record: ConsentRecord = {
      userId,
      purpose,
      granted: consent,
      timestamp: new Date(),
      ipAddress: await this.getIPAddress(),
      userAgent: navigator.userAgent
    };

    await this.storeConsentRecord(record);
    return consent;
  }

  // Right to access
  async exportUserData(userId: string): Promise<Blob> {
    const data = await this.collectAllUserData(userId);
    const json = JSON.stringify(data, null, 2);
    return new Blob([json], { type: 'application/json' });
  }

  // Right to deletion (right to be forgotten)
  async deleteUserData(userId: string): Promise<void> {
    await this.deleteActivities(userId);
    await this.deleteWorkouts(userId);
    await this.deleteHealthMetrics(userId);
    await this.deleteGoals(userId);
    await this.anonymizeAnalytics(userId);

    // Log deletion
    await this.logDataDeletion(userId, new Date());
  }

  // Data minimization
  validateDataCollection(data: any): boolean {
    // Only collect necessary fields
    const allowedFields = [
      'userId', 'type', 'startTime', 'endTime',
      'duration', 'distance', 'calories'
    ];

    return Object.keys(data).every(key => allowedFields.includes(key));
  }
}
```

### 8.4.2 HIPAA Compliance (for healthcare integration)

**Requirements:**
```typescript
interface HIPAACompliance {
  // Encryption
  encryptionAtRest: 'AES-256';
  encryptionInTransit: 'TLS 1.3';

  // Access controls
  authentication: 'MFA required';
  authorization: 'Role-based (RBAC)';
  sessionTimeout: number;  // minutes

  // Audit logs
  logAllAccess: boolean;
  logRetention: number;    // days

  // Business Associate Agreement
  baaRequired: boolean;
  baaSigned: boolean;
}

class HIPAASecureStorage {
  async storeData(data: any, userId: string): Promise<void> {
    // Encrypt data
    const encrypted = await this.encrypt(data);

    // Store with access log
    await this.saveEncrypted(encrypted, userId);
    await this.logAccess(userId, 'write', new Date());
  }

  async retrieveData(userId: string, accessorId: string): Promise<any> {
    // Verify authorization
    if (!await this.isAuthorized(accessorId, userId)) {
      throw new Error('Unauthorized access attempt');
    }

    // Log access
    await this.logAccess(userId, 'read', new Date(), accessorId);

    // Retrieve and decrypt
    const encrypted = await this.loadEncrypted(userId);
    return await this.decrypt(encrypted);
  }

  private async logAccess(
    userId: string,
    action: 'read' | 'write' | 'delete',
    timestamp: Date,
    accessor?: string
  ): Promise<void> {
    await this.auditLog.insert({
      userId,
      action,
      timestamp,
      accessor: accessor || userId,
      ipAddress: await this.getIPAddress()
    });
  }
}
```

### 8.4.3 Data Retention Policy

```typescript
interface RetentionPolicy {
  dataType: string;
  retentionPeriod: number;  // days
  action: 'delete' | 'anonymize' | 'archive';
}

const retentionPolicies: RetentionPolicy[] = [
  {
    dataType: 'activity',
    retentionPeriod: 1095,  // 3 years
    action: 'delete'
  },
  {
    dataType: 'health_metrics',
    retentionPeriod: 730,   // 2 years
    action: 'anonymize'
  },
  {
    dataType: 'audit_logs',
    retentionPeriod: 2555,  // 7 years (HIPAA requirement)
    action: 'archive'
  }
];

async function enforceRetentionPolicy(): Promise<void> {
  for (const policy of retentionPolicies) {
    const cutoffDate = new Date();
    cutoffDate.setDate(cutoffDate.getDate() - policy.retentionPeriod);

    const oldData = await queryOldData(policy.dataType, cutoffDate);

    for (const record of oldData) {
      switch (policy.action) {
        case 'delete':
          await deleteRecord(record.id);
          break;
        case 'anonymize':
          await anonymizeRecord(record.id);
          break;
        case 'archive':
          await archiveRecord(record);
          break;
      }
    }
  }
}
```

---

## 8.5 Certification Requirements

### 8.5.1 Certification Levels

**Level 1: Basic**
- ✓ Step counting (±5% accuracy)
- ✓ Basic activity logging
- ✓ Manual calorie entry
- ✓ Simple goal tracking
- ✓ Data export (JSON)

**Testing:** 100 steps walking test, basic functionality check

**Level 2: Standard**
- ✓ All Level 1 features
- ✓ GPS tracking (±2% distance accuracy)
- ✓ Heart rate monitoring (±5 BPM accuracy)
- ✓ Automatic calorie calculation (MET-based)
- ✓ Workout analysis

**Testing:** GPS accuracy test (measured course), HR accuracy test (chest strap reference)

**Level 3: Advanced**
- ✓ All Level 2 features
- ✓ Multi-sport support (10+ activity types)
- ✓ HRV measurement (RMSSD, SDNN)
- ✓ VO2 Max estimation
- ✓ Training load calculation (TRIMP/TSS)
- ✓ Cross-platform sync

**Testing:** Multi-device sync test, training load validation, HRV accuracy

**Level 4: Professional**
- ✓ All Level 3 features
- ✓ Medical-grade accuracy (±3%)
- ✓ FHIR healthcare integration
- ✓ Research-grade data export
- ✓ HIPAA/GDPR compliance
- ✓ Audit logging

**Testing:** Clinical validation study (n≥30), security audit, compliance review

### 8.5.2 Certification Process

**Step 1: Self-Assessment**
```typescript
interface CertificationChecklist {
  level: 1 | 2 | 3 | 4;
  requirements: {
    name: string;
    implemented: boolean;
    tested: boolean;
    documentationComplete: boolean;
  }[];
}

const checklist: CertificationChecklist = {
  level: 2,
  requirements: [
    {
      name: 'Step counting',
      implemented: true,
      tested: true,
      documentationComplete: true
    },
    {
      name: 'GPS tracking',
      implemented: true,
      tested: false,
      documentationComplete: true
    }
    // ... more requirements
  ]
};
```

**Step 2: Submit Application**
- Complete online application
- Submit documentation
- Pay certification fee
- Schedule testing session

**Step 3: Testing**
- Accuracy testing (controlled environment)
- Interoperability testing (data exchange)
- Security testing (penetration test)
- Usability testing (user experience)

**Step 4: Review and Certification**
- Review test results
- Address any gaps
- Receive certification
- Display certification badge

### 8.5.3 Maintenance and Renewal

```
Annual Requirements:
- Submit annual compliance report
- Re-test if major updates
- Maintain accuracy standards
- Keep documentation current
- Pay annual renewal fee

Certification valid for: 2 years
Renewal process: Simplified re-testing
```

---

## 8.6 Reference Implementation

### 8.6.1 Complete Example

**TypeScript SDK:**
```typescript
import { WIAFitnessTracker } from '@wia/fitness-tracking';

// Initialize
const tracker = new WIAFitnessTracker({
  apiKey: 'your_api_key',
  userId: 'user_12345',
  certificationLevel: 2
});

// Start tracking
await tracker.startActivity({
  type: 'running',
  enableGPS: true,
  enableHeartRate: true
});

// Real-time updates
tracker.on('heartRate', (hr: number) => {
  console.log(`Heart Rate: ${hr} BPM`);
});

tracker.on('distance', (distance: number) => {
  console.log(`Distance: ${(distance / 1000).toFixed(2)} km`);
});

// Stop and save
const activity = await tracker.stopActivity();
console.log(`Activity saved: ${activity.id}`);
console.log(`Calories: ${activity.calories} kcal`);
console.log(`Distance: ${(activity.distance / 1000).toFixed(2)} km`);
```

---

## Key Takeaways

✓ Accelerometer at 50-100 Hz for accurate step detection

✓ GPS sampling rate varies by activity type (1-10 Hz)

✓ Heart rate monitoring requires 1-5 Hz sampling

✓ Accuracy standards: ±5% steps, ±2% GPS distance, ±5 BPM heart rate

✓ Battery optimization through adaptive sampling and batch processing

✓ GDPR compliance requires explicit consent and data portability

✓ HIPAA compliance requires encryption, audit logs, and BAA

✓ Four certification levels from Basic to Professional

✓ Certification process includes accuracy testing and security audit

✓ Reference implementations available for rapid development

---

## Conclusion

WIA-IND-012 provides a comprehensive framework for building accurate, interoperable, and privacy-respecting fitness tracking systems. By following the guidelines in this ebook, developers can create certified implementations that work seamlessly across the ecosystem while maintaining user trust and regulatory compliance.

The future of fitness tracking is open, standardized, and user-centric. Join us in building it.

**弘益人間 (Benefit All Humanity)**

---

**Previous:** [← Chapter 7: System Integration](07-system-integration.md)
**Start:** [← Back to Index](00-index.md)

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
