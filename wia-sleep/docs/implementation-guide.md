# WIA-SLEEP Implementation Guide

## Version 1.0.0

---

## 1. Introduction

This guide provides step-by-step instructions for implementing the WIA-SLEEP standard in your application or system.

### 1.1 Prerequisites

- Understanding of sleep science fundamentals
- Familiarity with REST APIs
- JSON/TypeScript knowledge
- Access to sleep data sources (devices, questionnaires, or clinical systems)

### 1.2 Implementation Goals

1. Capture sleep data in WIA-SLEEP format
2. Assess chronotype accurately
3. Generate personalized recommendations
4. Integrate with existing healthcare systems

---

## 2. Getting Started

### 2.1 Install the SDK

```bash
# npm
npm install @wia/sleep-sdk

# yarn
yarn add @wia/sleep-sdk

# pnpm
pnpm add @wia/sleep-sdk
```

### 2.2 Basic Setup

```typescript
import {
  ChronotypeService,
  SleepAnalyzer,
  CircadianCalculator,
  OptimizationEngine
} from '@wia/sleep-sdk';

// Initialize services
const sleepAnalyzer = new SleepAnalyzer();
```

---

## 3. Data Collection

### 3.1 Sleep Record Structure

```typescript
import { SleepRecord, SleepDataSource } from '@wia/sleep-sdk';

const sleepRecord: SleepRecord = {
  recordId: crypto.randomUUID(),
  subjectId: 'user-123',
  timestamp: new Date().toISOString(),
  dataSource: SleepDataSource.CONSUMER_WEARABLE,
  sleepArchitecture: {
    summary: {
      totalSleepTime_min: 420,
      sleepEfficiency_pct: 88.5,
      sleepOnsetLatency_min: 15,
      wakeAfterSleepOnset_min: 25
    },
    stagePercentages: {
      n1_pct: 5,
      n2_pct: 50,
      n3_sws_pct: 20,
      rem_pct: 25
    }
  }
};
```

### 3.2 Device Data Transformation

```typescript
// Example: Transforming Oura Ring data
function transformOuraData(ouraData: OuraSleepData): SleepRecord {
  const totalSleepSec =
    ouraData.light_sleep_duration +
    ouraData.deep_sleep_duration +
    ouraData.rem_sleep_duration;

  return {
    recordId: crypto.randomUUID(),
    subjectId: ouraData.user_id,
    timestamp: ouraData.bedtime_end,
    dataSource: SleepDataSource.CONSUMER_WEARABLE,
    sleepArchitecture: {
      summary: {
        totalSleepTime_min: totalSleepSec / 60,
        sleepEfficiency_pct: ouraData.efficiency,
        sleepOnsetLatency_min: ouraData.latency / 60,
        wakeAfterSleepOnset_min: ouraData.awake_time / 60
      },
      stagePercentages: {
        n1_pct: 0, // Oura combines N1+N2
        n2_pct: (ouraData.light_sleep_duration / totalSleepSec) * 100,
        n3_sws_pct: (ouraData.deep_sleep_duration / totalSleepSec) * 100,
        rem_pct: (ouraData.rem_sleep_duration / totalSleepSec) * 100
      }
    }
  };
}
```

---

## 4. Chronotype Assessment

### 4.1 Collecting MCTQ Data

Create a questionnaire that collects:

```typescript
interface MCTQForm {
  // Workdays
  workday_bedtime: string;        // "23:30"
  workday_time_to_sleep: number;  // minutes
  workday_wake_time: string;      // "06:45"
  workday_get_up_time: string;    // "07:00"

  // Free days
  freeday_bedtime: string;
  freeday_time_to_sleep: number;
  freeday_wake_time: string;
  freeday_get_up_time: string;

  // General
  work_days_per_week: number;     // 5
  use_alarm_workdays: boolean;
  use_alarm_freedays: boolean;
}
```

### 4.2 Processing Assessment

```typescript
import { ChronotypeService, MCTQResponses } from '@wia/sleep-sdk';

function assessChronotype(form: MCTQForm): ChronotypeResult {
  const mctqResponses: MCTQResponses = {
    workdays: {
      bedtime: form.workday_bedtime + ':00',
      sleepLatency_min: form.workday_time_to_sleep,
      wakeTime: form.workday_wake_time + ':00',
      sleepDuration_hours: calculateDuration(
        form.workday_bedtime,
        form.workday_wake_time
      )
    },
    freeDays: {
      bedtime: form.freeday_bedtime + ':00',
      sleepLatency_min: form.freeday_time_to_sleep,
      wakeTime: form.freeday_wake_time + ':00',
      sleepDuration_hours: calculateDuration(
        form.freeday_bedtime,
        form.freeday_wake_time
      )
    },
    workDaysPerWeek: form.work_days_per_week,
    alarmUsage: form.use_alarm_workdays,
    alarmUsageFreeDays: form.use_alarm_freedays
  };

  return ChronotypeService.assessChronotype({
    subjectId: 'user-123',
    mctqResponses
  });
}
```

### 4.3 Displaying Results

```typescript
function displayChronotypeResults(result: ChronotypeResult) {
  const interpretation = ChronotypeService.interpret(result);

  return {
    summary: `Your chronotype is ${result.chronotype.replace('_', ' ')}`,
    msfsc: result.msfsc.toFixed(2),
    socialJetlag: result.socialJetlag_hours.toFixed(1) + ' hours',
    optimalBedtime: formatTime(result.optimalSleepWindow.bedtime),
    optimalWakeTime: formatTime(result.optimalSleepWindow.wakeTime),
    insights: interpretation.insights,
    recommendations: interpretation.recommendations
  };
}
```

---

## 5. Circadian Phase Tracking

### 5.1 Estimating DLMO

```typescript
import { CircadianCalculator, ChronotypeProfile } from '@wia/sleep-sdk';

// From chronotype profile
const dlmo = CircadianCalculator.estimateDLMOFromChronotype(chronotypeProfile);

// From habitual bedtime
import { estimateDLMOFromBedtime } from '@wia/sleep-sdk';
const estimatedDLMO = estimateDLMOFromBedtime('23:30', 0);
```

### 5.2 Phase Estimation

```typescript
const phaseEstimate = CircadianCalculator.generatePhaseEstimate(
  'user-123',
  chronotypeProfile,
  new Date()
);

console.log(`Current circadian phase: ${phaseEstimate.currentPhase}`);
console.log(`Entrainment: ${phaseEstimate.entrainmentStatus}`);
console.log('Alertness windows:', phaseEstimate.optimalAlertness);
```

### 5.3 Alertness Prediction

```typescript
const alertnessWindows = CircadianCalculator.generateAlertnessWindows(dlmo);

// Use for scheduling
const bestMeetingTime = alertnessWindows.find(w => w.level === 'peak');
console.log(`Schedule important meetings between ${bestMeetingTime.startTime} and ${bestMeetingTime.endTime}`);
```

---

## 6. Sleep Quality Analysis

### 6.1 Analyzing Sleep Records

```typescript
import { SleepAnalyzer } from '@wia/sleep-sdk';

const analyzer = new SleepAnalyzer();

// Single record analysis
const qualityAssessment = analyzer.assessQuality(sleepRecord);

console.log(`Quality Level: ${qualityAssessment.qualityLevel}`);
console.log('Strengths:', qualityAssessment.strengths);
console.log('Concerns:', qualityAssessment.concerns);
```

### 6.2 Pattern Analysis

```typescript
// Analyze patterns over time
const records: SleepRecord[] = await fetchWeeklyRecords(userId);
const patterns = analyzer.analyzePatterns(records, 8); // 8 hour sleep need

console.log('Averages:', patterns.averages);
console.log('Sleep debt:', patterns.sleepDebt);
console.log('Trend:', patterns.trends);
```

---

## 7. Generating Recommendations

### 7.1 Creating Optimization Plans

```typescript
import { OptimizationEngine, OptimizationGoal } from '@wia/sleep-sdk';

const plan = OptimizationEngine.generatePlan(
  {
    subjectId: 'user-123',
    goals: [
      OptimizationGoal.PHASE_ADVANCE,
      OptimizationGoal.INCREASE_EFFICIENCY
    ],
    constraints: {
      workSchedule: {
        wakeTimeRequired: '07:00:00',
        flexibility: 'fixed'
      }
    }
  },
  chronotypeProfile,
  currentSleepState
);
```

### 7.2 Light Therapy Prescriptions

```typescript
const lightRx = OptimizationEngine.generateLightPrescription(
  {
    subjectId: 'user-123',
    goal: 'phase_advance',
    targetShift_hours: 1.0,
    constraints: {
      earliestExposure: '06:30:00',
      maxDuration_min: 30
    }
  },
  chronotypeProfile
);

console.log('Light therapy sessions:', lightRx.sessions);
console.log('Avoid light after:', lightRx.avoidanceWindow?.start);
```

---

## 8. Healthcare Integration

### 8.1 FHIR Export

```typescript
import { convertToFHIR } from '@wia/sleep-sdk/fhir';

const fhirObservation = convertToFHIR(sleepRecord);

// Send to EHR
await fetch('https://ehr.example.com/fhir/Observation', {
  method: 'POST',
  headers: { 'Content-Type': 'application/fhir+json' },
  body: JSON.stringify(fhirObservation)
});
```

### 8.2 HL7 v2.x Export

```typescript
import { convertToHL7v2 } from '@wia/sleep-sdk/hl7';

const hl7Message = convertToHL7v2(sleepRecord);
// Returns ORU^R01 message string
```

---

## 9. Error Handling

### 9.1 Validation Errors

```typescript
import { validateSleepRecord } from '@wia/sleep-sdk';

try {
  const validation = validateSleepRecord(record);
  if (!validation.valid) {
    console.error('Validation errors:', validation.errors);
    console.warn('Warnings:', validation.warnings);
  }
} catch (error) {
  console.error('Invalid record format:', error);
}
```

### 9.2 Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Invalid sleep duration | Duration > 24h | Check time calculations |
| Stage percentages don't sum | Missing stage data | Validate input or estimate |
| MCTQ calculation error | Invalid times | Validate 24-hour format |

---

## 10. Best Practices

### 10.1 Data Quality

1. **Validate incoming data** - Use schema validation
2. **Handle missing fields** - Provide sensible defaults or mark as unavailable
3. **Document data source** - Track device/method used
4. **Store raw and processed** - Keep original for reprocessing

### 10.2 User Experience

1. **Progressive disclosure** - Start with simple results
2. **Actionable recommendations** - Specific, timed actions
3. **Track progress** - Show improvement over time
4. **Respect privacy** - Encrypt sensitive sleep data

### 10.3 Clinical Integration

1. **Maintain audit trail** - Log all data transformations
2. **Support HIPAA** - Implement required safeguards
3. **Enable professional review** - Flag concerning patterns
4. **Interoperate** - Use standard formats (FHIR, HL7)

---

## 11. Troubleshooting

### Common Problems

**Q: Chronotype classification seems wrong**
A: Verify MCTQ times are in 24-hour format and sleep durations are calculated correctly.

**Q: Phase shift calculations are off**
A: Check that CBT nadir is correctly estimated (typically ~7h after DLMO).

**Q: Device data doesn't map correctly**
A: Review device mapping files and ensure all required fields are present.

---

## 12. Further Resources

- [WIA-SLEEP API Reference](../api/openapi.yaml)
- [Algorithm Specifications](../algorithms/)
- [Device Mappings](../mappings/device-mappings/)
- [Conformance Criteria](../validation/conformance-criteria.md)

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
