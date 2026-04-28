# WIA-IND-014 — Phase 3: Protocol

> Performance-analytics and safety-and-monitoring protocol layer. Every protocol exchange is wire-level with consent envelopes for personal-fitness data.

## 8. Performance Analytics

### 8.1 Progress Tracking Metrics

#### 8.1.1 Strength Progress

**One-Rep Max Calculations:**
```
Epley Formula:
  1RM = Weight × (1 + Reps / 30)

Brzycki Formula:
  1RM = Weight × (36 / (37 - Reps))

Lombardi Formula:
  1RM = Weight × Reps^0.1

Velocity-Based (if VBT available):
  1RM = Weight / (1.15 × Velocity - 0.05)
```

**Strength Standards (Relative to Bodyweight):**
```typescript
const strengthStandards = {
  squat: {
    beginner: { male: 0.75, female: 0.50 },
    intermediate: { male: 1.25, female: 0.90 },
    advanced: { male: 1.75, female: 1.25 },
    elite: { male: 2.25, female: 1.75 }
  },
  deadlift: {
    beginner: { male: 1.00, female: 0.65 },
    intermediate: { male: 1.50, female: 1.05 },
    advanced: { male: 2.00, female: 1.50 },
    elite: { male: 2.75, female: 2.00 }
  },
  benchPress: {
    beginner: { male: 0.50, female: 0.30 },
    intermediate: { male: 0.85, female: 0.50 },
    advanced: { male: 1.25, female: 0.75 },
    elite: { male: 1.75, female: 1.10 }
  }
};
```

#### 8.1.2 Cardiovascular Progress

**VO2 Max Estimation:**
```
Rockport Walking Test:
  VO2max = 132.853 - (0.0769 × Weight_lbs) - (0.3877 × Age)
           + (6.315 × Gender) - (3.2649 × Time_min)
           - (0.1565 × HeartRate)
  where Gender: male = 1, female = 0

Cooper 12-Minute Run Test:
  VO2max = (Distance_meters - 505) / 45

1.5 Mile Run Test:
  VO2max = 483 / Time_minutes + 3.5
```

**Cardiovascular Fitness Levels:**
```typescript
const vo2maxStandards = {
  age20_29: {
    male: { poor: '<38', fair: '38-43', average: '44-51', good: '52-56', excellent: '>56' },
    female: { poor: '<28', fair: '28-32', average: '33-38', good: '39-43', excellent: '>43' }
  },
  age30_39: {
    male: { poor: '<35', fair: '35-39', average: '40-47', good: '48-51', excellent: '>51' },
    female: { poor: '<27', fair: '27-31', average: '32-36', good: '37-41', excellent: '>41' }
  },
  // ... more age groups
};
```

### 8.2 Body Composition Tracking

#### 8.2.1 Measurement Methods

**Bioelectrical Impedance Analysis (BIA):**
```typescript
interface BIAMeasurement {
  timestamp: Date;
  memberId: string;

  impedance: {
    wholebody: number; // ohms
    rightArm: number;
    leftArm: number;
    trunk: number;
    rightLeg: number;
    leftLeg: number;
  };

  results: {
    weight: number; // kg
    bodyFatPercentage: number;
    fatMass: number; // kg
    leanBodyMass: number; // kg
    muscleMass: number; // kg
    boneMass: number; // kg
    totalBodyWater: number; // kg
    visceralFat: number; // level 1-59
    bmi: number;
    bmr: number; // kcal/day
    metabolicAge: number; // years
  };

  segmentalAnalysis: {
    rightArm: { fatPercent: number; muscleMass: number };
    leftArm: { fatPercent: number; muscleMass: number };
    trunk: { fatPercent: number; muscleMass: number };
    rightLeg: { fatPercent: number; muscleMass: number };
    leftLeg: { fatPercent: number; muscleMass: number };
  };
}
```

**3D Body Scanning:**
```typescript
interface BodyScan3D {
  timestamp: Date;
  memberId: string;

  circumferences: {
    neck: number; // cm
    shoulders: number;
    chest: number;
    waist: number;
    hips: number;
    rightThigh: number;
    leftThigh: number;
    rightCalf: number;
    leftCalf: number;
    rightBicep: number;
    leftBicep: number;
    rightForearm: number;
    leftForearm: number;
  };

  volumes: {
    total: number; // liters
    trunk: number;
    rightArm: number;
    leftArm: number;
    rightLeg: number;
    leftLeg: number;
  };

  posture: {
    shoulderAsymmetry: number; // degrees
    hipAsymmetry: number;
    spinalCurvature: number;
    headTilt: number;
    forwardHeadPosture: number; // cm
  };

  comparison: {
    previousScan: Date;
    totalVolumeChange: number; // liters
    waistChange: number; // cm
    muscleGain: number; // estimated kg
  };
}
```

---


## 9. Safety and Monitoring

### 9.1 Injury Risk Assessment

**Machine Learning Risk Model:**
```typescript
interface InjuryRiskAssessment {
  memberId: string;
  timestamp: Date;

  inputFactors: {
    trainingLoad: {
      acuteLoad: number; // 7-day average
      chronicLoad: number; // 28-day average
      acuteChronicRatio: number; // ACWR
    };

    recoverMetrics: {
      sleepQuality: number; // 1-10
      sleepDuration: number; // hours
      muscularSoreness: number; // 1-10
      hrvScore: number;
      restingHeartRate: number;
    };

    biomechanics: {
      movementAsymmetry: number; // percentage
      formScoreAvg: number; // 1-10
      rangeOfMotionDeficits: string[];
    };

    history: {
      previousInjuries: Array<{
        type: string;
        severity: string;
        daysAgo: number;
        fullyRecovered: boolean;
      }>;
      ageFactorRisk: number; // based on age
    };
  };

  riskScores: {
    overall: number; // 0-1 (0 = low risk, 1 = high risk)
    areas: {
      lowerBack: number;
      shoulders: number;
      knees: number;
      elbows: number;
      ankles: number;
      wrists: number;
    };
    timeframe: '24hours' | '3days' | '1week';
  };

  recommendations: {
    action: 'train-normally' | 'modify-workout' | 'active-recovery' | 'full-rest';
    modifications: string[];
    focusRecovery: string[];
    medicalConsult: boolean;
  };
}
```

### 9.2 Equipment Maintenance Prediction

**Predictive Maintenance:**
```typescript
interface EquipmentMaintenance {
  equipmentId: string;
  type: string;

  usageMetrics: {
    totalHours: number;
    totalSessions: number;
    avgLoadPerSession: number;
    peakLoad: number;
  };

  sensorData: {
    vibration: number; // accelerometer
    temperature: number; // bearing temp
    soundLevel: number; // unusual noise
    calibrationDrift: number; // sensor accuracy
  };

  healthScore: number; // 0-100

  prediction: {
    estimatedRemainingLife: number; // hours
    failureRisk: {
      next7Days: number; // probability
      next30Days: number;
      next90Days: number;
    };
    recommendedAction: {
      urgency: 'immediate' | 'soon' | 'scheduled' | 'monitor';
      task: 'repair' | 'replace-part' | 'full-service' | 'calibration';
      estimatedDowntime: number; // hours
      costEstimate: number;
    };
  };

  maintenanceHistory: Array<{
    date: Date;
    type: 'routine' | 'repair' | 'replacement';
    description: string;
    cost: number;
    performedBy: string;
  }>;
}
```

---



## A.1 Performance-analytics protocol

Performance analytics aggregate workout-session telemetry into
per-member trends, per-cohort comparisons, and per-program
effectiveness measures. The protocol envelopes carry the aggregation
window, the cohort definition, and the consent envelope chain so
analysts can verify which data the aggregate is allowed to include.

## A.2 Safety-and-monitoring protocol

Safety monitoring runs continuously: heart-rate envelope above
documented thresholds triggers a safety-event envelope to the
member's app, the trainer's display, and (with consent) emergency
contacts. The protocol exchanges are wire-level with replay defence
and audit log applied uniformly.

## A.3 Cross-facility federation

When members visit multiple facilities under a chain or alliance,
the federation envelope reuses WIA-SOCIAL Phase 3 §5 receipt shape.
Trust lists name the peer facilities; the member's consent envelope
authorises which facilities can read which fields of their profile.

## A.4 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce
cache. Audit envelopes are written to an append-only log replicated
across at least two storage backends.


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-gym/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-gym-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-gym-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/smart-gym.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.


## A.5 Privacy-preserving aggregate analytics

Aggregate analytics (cohort comparisons, program-effectiveness
measures) require multiple members data to compute. The protocol
forbids aggregates without documented consent envelopes for every
included member; the aggregator publishes the consent-envelope
hash alongside the aggregate so a regulator can verify which
members were included.

## A.6 Real-time form feedback

Real-time form feedback (during virtual training or self-led
workouts with form sensors) emits per-rep envelopes signed by the
exercise-prescription co-signer (trainer or licensed program
provider). The latency budget is documented; envelopes exceeding
the budget emit a warning to operations.

## A.7 Closing protocol note

The smart-gym protocol layer balances real-time responsiveness
(form feedback, safety monitoring) against audit rigour (signed
envelopes, replay defence). The default discipline favours rigour;
operators can opt out of specific defaults in jurisdictions where
the trade-off favours responsiveness, and the opt-out itself is
recorded in the audit chain.
