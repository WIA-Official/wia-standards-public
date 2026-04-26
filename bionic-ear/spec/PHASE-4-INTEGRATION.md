# WIA-AUG-009 PHASE 4 — Integration Specification

**Standard:** WIA-AUG-009
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

sensitivity: 12,
      noiseReduction: 'off',
      directionality: 'omnidirectional'
    },
    {
      name: 'Noise',
      strategy: 'HDCIS',
      sensitivity: 15,
      noiseReduction: 'high',
      directionality: 'narrow'
    },
    {
      name: 'Music',
      strategy: 'FSP',
      sensitivity: 10,
      noiseReduction: 'off',
      directionality: 'omnidirectional'
    }
  ];

  return {
    patientId: patientProfile.id,
    deviceId: '', // Set during fitting
    createdDate: new Date(),
    audiologist: '', // Set during fitting
    processingStrategy: strategy,
    stimulationRate: 900, // Hz, typical
    pulseWidth: 25, // μs, typical
    electrodes,
    sensitivity: 12,
    volume: 128, // Mid-range (0-255)
    compression: 3.0,
    noiseReduction: 'medium',
    directionality: 'adaptive',
    frequencyMap: generateFrequencyMap(electrodes.length),
    programs,
    activeProgram: 0
  };
}
```

### 14.4 Fine-Tuning Process

```typescript
interface FineTuning {
  session: number;
  date: Date;
  adjustments: Adjustment[];
  outcomeMetrics: OutcomeMetric[];
  patientFeedback: string;
}

interface Adjustment {
  type: 'volume' | 'sensitivity' | 't_level' | 'c_level' | 'frequency_shift' | 'strategy';
  electrode?: number; // For electrode-specific adjustments
  previousValue: number;
  newValue: number;
  reason: string;
}

function fineTuneMAP(
  currentMAP: DeviceMAP,
  patientFeedback: PatientFeedback,
  objectiveResults: ObjectiveTest[]
): DeviceMAP {
  const updatedMAP = { ...currentMAP };
  const adjustments: Adjustment[] = [];

  // Address patient feedback
  if (patientFeedback.tooLoud.length > 0) {
    // Reduce C-levels for electrodes perceived as too loud
    patientFeedback.tooLoud.forEach(electrode => {
      const setting = updatedMAP.electrodes.find(e => e.electrode === electrode);
      if (setting) {
        adjustments.push({
          type: 'c_level',
          electrode,
          previousValue: setting.cLevel,
          newValue: setting.cLevel - 10,
          reason: 'Patient reported too loud'
        });
        setting.cLevel -= 10;
      }
    });
  }

  if (patientFeedback.tooSoft.length > 0) {
    // Increase C-levels for electrodes perceived as too soft
    patientFeedback.tooSoft.forEach(electrode => {
      const setting = updatedMAP.electrodes.find(e => e.electrode === electrode);
      if (setting) {
        adjustments.push({
          type: 'c_level',
          electrode,
          previousValue: setting.cLevel,
          newValue: setting.cLevel + 10,
          reason: 'Patient reported too soft'
        });
        setting.cLevel += 10;
      }
    });
  }

  // Address objective test results
  if (objectiveResults.some(r => r.type === 'speech_in_quiet' && r.score < 70)) {
    // Poor speech recognition -> increase sensitivity or adjust strategy
    adjustments.push({
      type: 'sensitivity',
      previousValue: updatedMAP.sensitivity,
      newValue: updatedMAP.sensitivity + 3,
      reason: 'Improve speech recognition in quiet'
    });
    updatedMAP.sensitivity += 3;
  }

  return updatedMAP;
}
```

### 14.5 Objective Verification

```typescript
interface ObjectiveTest {
  type: 'impedance' | 'nrt' | 'ecap' | 'speech_audiometry' | 'aided_thresholds';
  electrode?: number;
  result: number | TestResult;
  interpretation: 'normal' | 'borderline' | 'abnormal';
  recommendation?: string;
}

function performObjectiveTesting(deviceMAP: DeviceMAP): ObjectiveTest[] {
  const tests: ObjectiveTest[] = [];

  // Impedance testing (all electrodes)
  deviceMAP.electrodes.forEach(e => {
    const impedance = measureImpedance(e.electrode);
    tests.push({
      type: 'impedance',
      electrode: e.electrode,
      result: impedance,
      interpretation: classifyImpedance(impedance)
    });
  });

  // Neural Response Telemetry (sample electrodes)
  [1, 6, 11, 16, 22].forEach(electrode => {
    const nrt = measureNRT(electrode);
    tests.push({
      type: 'nrt',
      electrode,
      result: nrt,
      interpretation: classifyNRT(nrt)
    });
  });

  // Aided thresholds (pure tone average)
  const aidedThresholds = measureAidedThresholds(deviceMAP);
  tests.push({
    type: 'aided_thresholds',
    result: aidedThresholds,
    interpretation: classifyAidedThresholds(aidedThresholds)
  });

  return tests;
}

function classifyImpedance(impedance: number): 'normal' | 'borderline' | 'abnormal' {
  if (impedance < 2 || impedance > 25) return 'abnormal';
  if (impedance < 4 || impedance > 20) return 'borderline';
  return 'normal';
}
```

---

## 15. Performance Evaluation

### 15.1 Speech Recognition Testing

```typescript
interface SpeechTest {
  testType: 'CNC' | 'HINT' | 'BKB' | 'AzBio' | 'CUNY';
  condition: 'quiet' | 'noise_fixed' | 'noise_adaptive';
  snr?: number; // dB (if noise condition)
  presentation: 'live_voice' | 'recorded' | 'monitored_live';
  score: number; // % correct
  confidence: { lower: number; upper: number }; // 95% CI
}

const speechTestBattery: SpeechTest[] = [
  {
    testType: 'CNC',
    condition: 'quiet',
    presentation: 'recorded',
    score: 0, // To be measured
    confidence: { lower: 0, upper: 0 }
  },
  {
    testType: 'AzBio',
    condition: 'quiet',
    presentation: 'recorded',
    score: 0,
    confidence: { lower: 0, upper: 0 }
  },
  {
    testType: 'AzBio',
    condition: 'noise_fixed',
    snr: 10,
    presentation: 'recorded',
    score: 0,
    confidence: { lower: 0, upper: 0 }
  },
  {
    testType: 'AzBio',
    condition: 'noise_fixed',
    snr: 5,
    presentation: 'recorded',
    score: 0,
    confidence: { lower: 0, upper: 0 }
  },
  {
    testType: 'HINT',
    condition: 'noise_adaptive',
    presentation: 'recorded',
    score: 0, // SNR-50 (dB for 50% intelligibility)
    confidence: { lower: 0, upper: 0 }
  }
];

function evaluateSpeechPerformance(tests: SpeechTest[]): PerformanceRating {
  // CNC words in quiet (primary metric)
  const cncQuiet = tests.find(t => t.testType === 'CNC' && t.condition === 'quiet');

  let rating: PerformanceRating;
  if (cncQuiet && cncQuiet.score >= 80) {
    rating = 'excellent';
  } else if (cncQuiet && cncQuiet.score >= 60) {
    rating = 'good';
  } else if (cncQuiet && cncQuiet.score >= 40) {
    rating = 'fair';
  } else {
    rating = 'needs_improvement';
  }

  return rating;
}
```

### 15.2 Quality of Life Assessment

```typescript
interface QualityOfLife {
  // Standardized questionnaires
  NCIQ: NCIQScore; // Nijmegen Cochlear Implant Questionnaire
  APHAB: APHABScore; // Abbreviated Profile of Hearing Aid Benefit
  SSQ: SSQScore; // Speech, Spatial, Qualities questionnaire

  // General domains
  overallSatisfaction: number; // 1-10
  communicationAbility: number; // 1-10
  environmentalAwareness: number; // 1-10
  musicEnjoyment: number; // 1-10
  qualityOfLife: number; // 1-10 composite
}

interface NCIQScore {
  physicalDomain: number; // 0-100
  psychologicalDomain: number; // 0-100
  socialDomain: number; // 0-100
  activityLimitationDomain: number; // 0-100
  overallScore: number; // 0-100
}

function assessQualityOfLife(
  preImplant: QualityOfLife,
  postImplant: QualityOfLife
): QOLImprovement {
  return {
    overallImprovement:
      postImplant.overallSatisfaction - preImplant.overallSatisfaction,
    communicationImprovement:
      postImplant.communicationAbility - preImplant.communicationAbility,
    environmentalAwarenessImprovement:
      postImplant.environmentalAwareness - preImplant.environmentalAwareness,
    musicEnjoymentImprovement:
      postImplant.musicEnjoyment - preImplant.musicEnjoyment,

    NCIQImprovement:
      postImplant.NCIQ.overallScore - preImplant.NCIQ.overallScore,

    clinicallySignificant:
      (postImplant.NCIQ.overallScore - preImplant.NCIQ.overallScore) >= 10
  };
}
```

### 15.3 Functional Auditory Performance

```typescript
interface FunctionalPerformance {
  // Categories of Auditory Performance (CAP)
  CAP: CAPLevel;

  // Meaningful Auditory Integration Scale (MAIS) / IT-MAIS
  MAIS: MAISScore;

  // Functional listening situations
  situations: FunctionalSituation[];
}

type CAPLevel =
  | 0 // No awareness of environmental sounds
  | 1 // Awareness of environmental sounds
  | 2 // Responds to speech sounds
  | 3 // Identifies environmental sounds
  | 4 // Discriminates speech sounds without lip-reading
  | 5 // Understands common phrases without lip-reading
  | 6 // Understands conversation without lip-reading
  | 7; // Uses telephone with known speaker

interface FunctionalSituation {
  situation: string;
  performanceRating: number; // 1-5
  importance: number; // 1-5
  satisfaction: number; // 1-5
}

const functionalSituations: FunctionalSituation[] = [
  { situation: 'One-on-one conversation in quiet', performanceRating: 0, importance: 5, satisfaction: 0 },
  { situation: 'Group conversation', performanceRating: 0, importance: 5, satisfaction: 0 },
  { situation: 'Conversation in noise (restaurant)', performanceRating: 0, importance: 4, satisfaction: 0 },
  { situation: 'Telephone conversation', performanceRating: 0, importance: 4, satisfaction: 0 },
  { situation: 'Watching TV/movies', performanceRating: 0, importance: 3, satisfaction: 0 },
  { situation: 'Listening to music', performanceRating: 0, importance: 3, satisfaction: 0 },
  { situation: 'Environmental awareness (traffic, alarms)', performanceRating: 0, importance: 5, satisfaction: 0 },
  { situation: 'Video calls/conferencing', performanceRating: 0, importance: 4, satisfaction: 0 }
];
```

### 15.4 Long-term Monitoring

```typescript
interface LongTermMonitoring {
  implantDate: Date;
  followUpSchedule: FollowUp[];
  deviceReliability: ReliabilityMetrics;
  performanceTrend: PerformanceTrend[];
  adverseEvents: AdverseEvent[];
}

interface FollowUp {
  date: Date;
  type: 'routine' | 'troubleshooting' | 'upgrade';
  activities: string[];
  outcomes: TestResult[];
  MAPadjustments: Adjustment[];
  nextAppointment: Date;
}

interface ReliabilityMetrics {
  deviceFailures: number;
  softFailures: number; // Resolved without replacement
  hardFailures: number; // Required replacement
  meanTimeBetweenFailures: number; // days
  componentReliability: Record<string, number>; // % functional
}

interface PerformanceTrend {
  date: Date;
  speechRecognition: number; // % (CNC in quiet)
  qualityOfLife: number; // NCIQ score
  usageHours: number; // Hours per day
  satisfaction: number; // 1-10
}

function monitorLongTerm(
  patientId: string,
  baseline: PerformanceTrend
): MonitoringPlan {
  return {
    schedule: [
      { interval: 'activation', week: 1 },
      { interval: 'initial_tuning', weeks: [4, 8, 12] },
      { interval: 'stabilization', months: [6, 9, 12] },
      { interval: 'maintenance', frequency: 'annual', years: [2, 3, 4, 5] }
    ],
    metrics: [
      'speech_recognition',
      'quality_of_life',
      'device_integrity',
      'usage_patterns',
      'patient_satisfaction'
    ],
    alerts: {
      performanceDecline: 'Alert if speech recognition drops >10%',
      deviceFailure: 'Immediate escalation',
      usageDecline: 'Alert if daily usage <8 hours',
      satisfactionDrop: 'Alert if rating drops >2 points'
    }
  };
}
```

---

## 16. Implementation Guidelines

### 16.1 System Architecture

```typescript
interface BionicEarSystem {
  hardware: {
    implant: ImplantHardware;
    externalProcessor: ProcessorHardware;
    accessories: Accessory[];
  };
  software: {
    firmware: FirmwareVersion;
    soundProcessing: ProcessingAlgorithm;
    fittingSoftware: FittingSoftware;
    patientApp: PatientApplication;
  };
  connectivity: {
    wireless: WirelessProtocol[];
    streaming: StreamingCapability[];
    remote: RemoteCapability[];
  };
}

interface ImplantHardware {
  receiverStimulator: {
    model: string;
    channels: number;
    stimulationMode: 'monopolar' | 'bipolar' | 'tripolar';
    telemetryEnabled: boolean;
  };
  electrodeArray: {
    type: ElectrodeConfig;
    contacts: number;
    length: number; // mm
    material: string;
  };
  magnet: {
    strength: number; // Gauss
    removable: boolean;
    rotatable: boolean;
  };
}

interface ProcessorHardware {
  model: string;
  microphones: number;
  processor: string; // CPU model
  memory: number; // MB
  battery: BatterySpecification;
  connectivity: string[]; // ['bluetooth', 'telecoil', 'dai']
  waterResistance: string; // IP rating
}
```

### 16.2 Integration Requirements

**Hardware Integration:**
```
□ All components use standard connectors (where applicable)
□ Modular design for component replacement
□ IP68 water/dust resistance for external components
□ EMC compliance (IEC 60601-1-2, FCC Part 15)
□ Biocompatible materials (ISO 10993)
□ MRI conditional safety (labeled field strength)
```

**Software Integration:**
```
□ Standard API for fitting software integration
□ Data format: DICOM, HL7, or proprietary with documented format
□ Firmware updatable (secure OTA or wired)
□ Telemetry data logging and export
□ Compatibility with audiological equipment
□ Patient app (iOS/Android) available
□ Cloud backup of MAPs and settings
```

**Safety Integration:**
```
□ Automated safety limit enforcement
□ Real-time impedance monitoring
□ Temperature monitoring
□ Battery monitoring with alerts
□ Emergency shutoff capability
□ Fault logging and reporting
```

### 16.3 Data Standards

```json
{
  "deviceData": {
    "header": {
      "standardVersion": "WIA-AUG-009-v1.0",
      "deviceId": "CI-2025-001",
      "patientId": "ANON-12345",
      "timestamp": "2025-12-27T10:00:00Z"
    },
    "classification": {
      "type": "COCHLEAR_IMPLANT",
      "electrodeConfig": "PERIMODIOLAR",
      "electrodeCount": 22,
      "processingStrategy": "ACE",
      "category": "Premium"
    },
    "currentMAP": {
      "version": "v3.2",
      "created": "2025-11-15",
      "audiologist": "AUD-789",
      "processingStrategy": "ACE",
      "stimulationRate": 900,
      "pulseWidth": 25,
      "electrodes": [
        {
          "number": 1,
          "active": true,
          "tLevel": 120,
          "cLevel": 180,
          "frequencyRange": { "low": 5938, "high": 7938 }
        }
        // ... more electrodes
      ]
    },
    "performance": {
      "speechRecognitionQuiet": 85,
      "speechRecognitionNoise10dB": 72,
      "qualityOfLife": 87,
      "usageHoursPerDay": 14.5,
      "satisfaction": 8.7
    },
    "status": {
      "implantIntegrity": "normal",
      "allImpedancesNormal": true,
      "batteryHealth": "good",
      "lastCheckup": "2025-12-15"
    }
  }
}
```

### 16.4 Testing Requirements

```typescript
interface TestSuite {
  functional: FunctionalTests;
  safety: SafetyTests;
  performance: PerformanceTests;
  reliability: ReliabilityTests;
  compatibility: CompatibilityTests;
}

const requiredTests: TestSuite = {
  functional: {
    tests: [
      'All electrodes functional',
      'Sound processing accurate',
      'Telemetry operational',
      'Wireless connectivity stable'
    ],
    passCriteria: 'All tests pass'
  },
  safety: {
    tests: [
      'Current limits enforced',
      'Charge balance verified',
      'Temperature limits enforced',
      'MRI safety confirmed',
      'Biocompatibility validated'
    ],
    passCriteria: '100% compliance'
  },
  performance: {
    tests: [
      'Frequency response 125-8000 Hz',
      'Dynamic range ≥40 dB',
      'Processing latency <10 ms',
      'Battery life ≥12 hours',
      'Impedance stability <20% variation'
    ],
    passCriteria: 'Meet or exceed specifications'
  },
  reliability: {
    tests: [
      'Thermal cycling (-20 to +60°C)',
      'Humidity resistance (95% RH)',
      'Drop test (external processor)',
      'Vibration test (IEC 60068-2-6)',
      'Accelerated aging (3 years equivalent)'
    ],
    passCriteria: 'No functional degradation'
  },
  compatibility: {
    tests: [
      'EMC compliance (IEC 60601-1-2)',
      'MRI compatibility verified',
      'Bluetooth interoperability',
      'Telecoil loop system compatibility',
      'Cross-manufacturer accessory compatibility'
    ],
    passCriteria: 'All compatibility verified'
  }
};
```

### 16.5 Certification Requirements

**WIA-AUG-009 Certification Requirements:**

1. **Design Documentation**
   - Complete technical specifications
   - Risk analysis (ISO 14971)
   - User manual and fitting guide
   - Clinical validation protocol

2. **Testing Evidence**
   - Functional test results
   - Safety test results (electrical, biocompatibility)
   - Performance benchmarks
   - Reliability test results
   - Clinical trial data (speech recognition, QOL)

3. **Manufacturing**
   - Quality management system (ISO 13485)
   - Cleanroom classification (ISO 14644)
   - Sterilization validation
   - Traceability system
   - Supplier qualification

4. **Safety Compliance**
   - WIA-AUG-013 (Augmentation Safety)
   - ISO 14708-7 (Cochlear implant systems)
   - IEC 60601-1 (Medical electrical equipment)
   - ISO 10993 (Biocompatibility)
   - Local regulatory compliance (FDA, CE, etc.)

5. **Performance Verification**
   - Speech recognition ≥70% in quiet (post-training)
   - Dynamic range ≥40 dB
   - Battery life ≥12 hours
   - Processing latency <10 ms
   - Impedance stability

6. **Clinical Evidence**
   - Multi-center clinical trials
   - Long-term follow-up data (≥5 years)
   - Adverse event reporting
   - Patient outcomes database
   - Quality of life improvements

---

## 17. References

### 17.1 International Standards

1. ISO 14708-7:2019 - Implants for surgery — Active implantable medical devices — Part 7: Cochlear implant systems
2. IEC 60601-1:2012 - Medical electrical equipment - General requirements for basic safety
3. IEC 60601-2-66 - Particular requirements for hearing instruments and hearing instrument systems
4. ISO 10993 - Biological evaluation of medical devices
5. ISO 13485 - Medical devices — Quality management systems
6. IEC 60118-4 - Hearing aids — Part 4: Induction-loop systems for hearing aid purposes
7. ISO 14971 - Medical devices — Application of risk management

### 17.2 WIA Standards

- WIA-AUG-001: Human Augmentation General
- WIA-AUG-013: Augmentation Safety
- WIA-AUG-014: Human-Machine Interface
- WIA-BCI: Brain-Computer Interface
- WIA-MED: Medical Device Standards
- WIA-DATA: Healthcare Data Standards
- WIA-WIRELESS: Wireless Communication Standards

### 17.3 Research References

- Wilson BS, Dorman MF (2008). "Cochlear implants: A remarkable past and a brilliant future." Hearing Research.
- Zeng FG (2004). "Trends in cochlear implants." Trends in Amplification.
- Loizou PC (1999). "Introduction to cochlear implants." IEEE Engineering in Medicine and Biology.
- Shannon RV (1983). "Multichannel electrical stimulation of the auditory nerve." Journal of the Acoustical Society of America.
- Başkent D, Gaudrain E, Tamati TN, Wagner A (2016). "Perception and enjoyment of music with cochlear implants." Current Opinion in Otolaryngology.

---

## Appendix A: Fitting Checklist

```
Pre-Activation (Day of Surgery to Activation):
□ Confirm surgical success and healing
□ Verify electrode insertion depth
□ Review imaging (X-ray/CT) for array position
□ Schedule activation appointment (1-4 weeks post-op)

Activation Day:
□ Device impedance testing
□ Neural response telemetry (if available)
□ T-level determination (all electrodes)
□ C-level determination (all electrodes)
□ Initial MAP creation
□ Program loading
□ Sound processor orientation
□ Battery instruction
□ Patient education (care, troubleshooting)
□ Schedule follow-up (1-2 weeks)

Initial Fine-Tuning (Weeks 1-12):
□ Patient feedback review
□ MAP adjustments based on loudness perception
□ Speech testing (informal)
□ Environmental sound exposure
□ Program customization
□ Bilateral synchronization (if applicable)

Optimization Phase (Months 3-12):
□ Formal speech testing (CNC, AzBio, HINT)
□ Music program optimization (if desired)
□ Tinnitus management (if needed)
□ Bilateral fine-tuning (if applicable)
□ Quality of life assessment
□ Advanced feature training
```

## Appendix B: Troubleshooting Guide

```typescript
interface TroubleshootingEntry {
  symptom: string;
  possibleCauses: string[];
  diagnosticSteps: string[];
  solutions: string[];
  severity: 'low' | 'medium' | 'high' | 'critical';
}

const troubleshootingDatabase: TroubleshootingEntry[] = [
  {
    symptom: 'No sound',
    possibleCauses: [
      'Dead battery',
      'Processor not on',
      'Coil not positioned correctly',
      'Internal device failure'
    ],
    diagnosticSteps: [
      'Check battery charge',
      'Verify processor power',
      'Reposition coil',
      'Test with known working processor',
      'Check impedances'
    ],
    solutions: [
      'Replace/recharge battery',
      'Turn on processor',
      'Adjust coil position',
      'If internal failure suspected, contact clinic immediately'
    ],
    severity: 'critical'
  },
  {
    symptom: 'Intermittent sound',
    possibleCauses: [
      'Weak battery',
      'Loose coil connection',
      'Moisture in processor',
      'Cable damage',
      'Intermittent electrode failure'
    ],
    diagnosticSteps: [
      'Check battery level',
      'Inspect cable and connections',
      'Dry processor in dehumidifier',
      'Test with spare cable',
      'Check impedances remotely'
    ],
    solutions: [
      'Replace battery',
      'Secure connections',
      'Dry equipment overnight',
      'Replace damaged cable',
      'Adjust MAP to deactivate faulty electrode'
    ],
    severity: 'high'
  },
  {
    symptom: 'Distorted/unnatural sound',
    possibleCauses: [
      'MAP needs adjustment',
      'Electrode migration',
      'Processor malfunction',
      'Wrong program selected'
    ],
    diagnosticSteps: [
      'Verify active program',
      'Test different programs',
      'Review recent MAP changes',
      'Check impedances for changes'
    ],
    solutions: [
      'Switch to different program',
      'Schedule MAP adjustment appointment',
      'If persistent, may need new MAP',
      'Processor replacement if hardware fault'
    ],
    severity: 'medium'
  },
  {
    symptom: 'High-pitched noise or feedback',
    possibleCauses: [
      'Coil not seated properly',
      'Moisture in coil',
      'Electromagnetic interference',
      'Processor malfunction'
    ],
    diagnosticSteps: [
      'Reseat coil',
      'Check for moisture',
      'Move away from electronic devices',
      'Test with spare coil'
    ],
    solutions: [
      'Adjust coil position',
      'Dry coil',
      'Avoid interference sources',
      'Replace coil if faulty'
    ],
    severity: 'medium'
  },
  {
    symptom: 'Sudden loss of performance',
    possibleCauses: [
      'Ear infection/middle ear fluid',
      'Internal device failure',
      'MAP corruption',
      'Electrode failure'
    ],
    diagnosticSteps: [
      'Medical examination (otoscopy)',
      'Impedance testing',
      'Reload previous MAP',
      'Neural response testing'
    ],
    solutions: [
      'Treat infection if present',
      'Reload known good MAP',
      'If device failure, contact manufacturer',
      'May require revision surgery'
    ],
    severity: 'critical'
  }
];
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUG-009 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
