# WIA-AUG-009 PHASE 3 — Protocol Specification

**Standard:** WIA-AUG-009
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

signal: AudioSignal,
  environment: EnvironmentMetrics,
  config: EnvironmentalAdaptation
): AudioSignal {
  let adapted = signal;

  // AGC
  if (config.agc.enabled) {
    adapted = applyAGC(adapted, config.agc);
  }

  // Wind suppression
  if (config.windSuppression.enabled && environment.windLevel > config.windSuppression.threshold) {
    adapted = suppressWind(adapted, config.windSuppression.reductionStrength);
  }

  // Transient suppression
  if (config.transientSuppression.enabled) {
    adapted = suppressTransients(adapted, config.transientSuppression);
  }

  // Reverb compensation
  if (config.reverbCompensation.enabled && environment.reverbTime > 500) {
    adapted = compensateReverb(adapted, environment.reverbTime, config.reverbCompensation);
  }

  return adapted;
}
```

---

## 11. Wireless Connectivity

### 11.1 Connectivity Standards

#### 11.1.1 Bluetooth

**Specifications:**
```
Protocol: Bluetooth 5.0+ (LE Audio)
Codecs: SBC, AAC, LC3 (Low Complexity Communications Codec)
Latency: <50ms (LE Audio)
Range: 10-30 meters
Power: Low Energy mode
Profiles: A2DP, HFP, HSP
```

```typescript
interface BluetoothConfig {
  version: '5.0' | '5.1' | '5.2' | '5.3';
  leAudio: boolean;
  codec: 'SBC' | 'AAC' | 'LC3';
  latency: number; // ms (target)
  multipoint: boolean; // Connect to multiple devices
  pairing: 'standard' | 'secure' | 'nfc';
  audioSharing: boolean; // Broadcast to multiple hearing aids
}

function configureBluetooth(config: BluetoothConfig): BluetoothConnection {
  // LE Audio for ultra-low latency
  if (config.leAudio && config.version >= '5.2') {
    return {
      protocol: 'LE_Audio',
      codec: 'LC3',
      latency: 25, // ms (typical)
      power: 'ultra_low',
      features: ['audioSharing', 'multipleStreams']
    };
  }

  // Classic Bluetooth
  return {
    protocol: 'Classic',
    codec: config.codec,
    latency: 100, // ms (typical for classic)
    power: 'low',
    features: ['A2DP', 'HFP']
  };
}
```

#### 11.1.2 Telecoil (T-Coil)

**Specifications:**
```
Frequency Response: 100-5000 Hz
Sensitivity: ≥31.6 mA/m (100 mV/Pa equivalent)
Standard: IEC 60118-4
Applications: Loop systems, telephone, public venues
Interference Rejection: >30 dB
```

```typescript
interface TelecoilConfig {
  enabled: boolean;
  sensitivity: number; // mA/m
  frequencyResponse: { low: number; high: number }; // Hz
  mixWithMicrophone: boolean;
  microphoneMixRatio: number; // 0-1 (0=telecoil only, 1=mic only)
  automaticTelecoilActivation: boolean;
}

function activateTelecoil(
  inductiveSignal: InductiveSignal,
  config: TelecoilConfig
): AudioSignal {
  if (!config.enabled) {
    return emptySignal();
  }

  // Convert magnetic field to audio signal
  let audio = magneticToAudio(inductiveSignal, config.sensitivity);

  // Filter to telecoil frequency response
  audio = bandpassFilter(audio, config.frequencyResponse);

  // Mix with microphone if configured
  if (config.mixWithMicrophone) {
    const micSignal = getMicrophoneSignal();
    audio = mixSignals(
      audio,
      micSignal,
      config.microphoneMixRatio
    );
  }

  return audio;
}
```

#### 11.1.3 Direct Audio Input (DAI)

**Specifications:**
```
Connector: 3.5mm jack or proprietary
Input Impedance: 10-100 kΩ
Input Level: 50-500 mV RMS
Frequency Response: 100-8000 Hz
THD: <1%
```

```typescript
interface DirectAudioInput {
  enabled: boolean;
  inputType: 'analog' | 'digital';
  connector: '3.5mm' | 'proprietary' | 'USB-C';
  inputGain: number; // dB
  automaticLevelControl: boolean;
  mixWithMicrophone: boolean;
}
```

### 11.2 Streaming Quality

```typescript
interface StreamingQuality {
  sampleRate: number; // Hz (44100, 48000)
  bitDepth: number; // bits (16, 24)
  codec: string;
  bitrate: number; // kbps
  latency: number; // ms
  jitterBuffer: number; // ms
  packetLossConcealment: boolean;
}

const streamingPresets: Record<string, StreamingQuality> = {
  music_high_quality: {
    sampleRate: 48000,
    bitDepth: 24,
    codec: 'LC3',
    bitrate: 320,
    latency: 25,
    jitterBuffer: 20,
    packetLossConcealment: true
  },
  speech_low_latency: {
    sampleRate: 16000,
    bitDepth: 16,
    codec: 'LC3',
    bitrate: 64,
    latency: 15,
    jitterBuffer: 10,
    packetLossConcealment: true
  },
  video_synchronized: {
    sampleRate: 48000,
    bitDepth: 16,
    codec: 'LC3',
    bitrate: 128,
    latency: 20,
    jitterBuffer: 15,
    packetLossConcealment: true
  }
};
```

### 11.3 Remote Programming

```typescript
interface RemoteProgramming {
  enabled: boolean;
  connection: 'bluetooth' | 'wifi' | 'cellular';
  security: 'encrypted' | 'secure_tunnel' | 'vpn';
  capabilities: RemoteCapability[];
  requiresAudiologist: boolean;
}

type RemoteCapability =
  | 'map_adjustment'
  | 'volume_change'
  | 'program_switch'
  | 'firmware_update'
  | 'diagnostics'
  | 'hearing_test';

function remoteAdjustMAP(
  deviceId: string,
  adjustments: MAPAdjustment[],
  security: SecurityConfig
): RemoteAdjustmentResult {
  // Authenticate
  const authenticated = authenticateSession(security);
  if (!authenticated) {
    return { success: false, error: 'Authentication failed' };
  }

  // Validate adjustments
  const validated = validateAdjustments(adjustments);
  if (!validated.valid) {
    return { success: false, error: validated.errors };
  }

  // Apply adjustments
  const result = applyRemoteAdjustments(deviceId, adjustments);

  // Verify
  const verification = verifyAdjustments(deviceId);

  return {
    success: true,
    applied: result.applied,
    verified: verification.success,
    newMAP: result.newMAP
  };
}
```

---

## 12. Power and Battery Management

### 12.1 Battery Requirements

```typescript
interface BatterySpecification {
  type: 'disposable_zinc_air' | 'rechargeable_lithium' | 'rechargeable_silver_zinc';
  capacity: number; // mAh
  voltage: number; // V
  runtime: number; // hours
  chargingTime?: number; // hours (rechargeable only)
  cycles?: number; // charge cycles (rechargeable only)
  size: BatterySize;
}

type BatterySize = '10' | '13' | '312' | '675'; // Hearing aid battery sizes

const batterySpecs: Record<BatterySize, BatterySpecification> = {
  '10': {
    type: 'disposable_zinc_air',
    capacity: 100,
    voltage: 1.4,
    runtime: 80, // hours (typical CI usage)
    size: '10'
  },
  '13': {
    type: 'disposable_zinc_air',
    capacity: 310,
    voltage: 1.4,
    runtime: 240,
    size: '13'
  },
  '312': {
    type: 'disposable_zinc_air',
    capacity: 180,
    voltage: 1.4,
    runtime: 140,
    size: '312'
  },
  '675': {
    type: 'disposable_zinc_air',
    capacity: 650,
    voltage: 1.4,
    runtime: 500,
    size: '675'
  }
};

const rechargeableSpec: BatterySpecification = {
  type: 'rechargeable_lithium',
  capacity: 85,
  voltage: 3.7,
  runtime: 16,
  chargingTime: 3,
  cycles: 500,
  size: '13' // Equivalent
};
```

### 12.2 Power Consumption

| Component | Idle (mW) | Active (mW) | Peak (mW) | % of Total |
|-----------|-----------|-------------|-----------|------------|
| Sound Processor | 5 | 40 | 80 | 60% |
| RF Transmitter | 2 | 15 | 30 | 22% |
| Microphones (2) | 1 | 5 | 10 | 8% |
| Wireless (BT) | 0 | 4 | 8 | 6% |
| Display/UI | 0.5 | 2 | 5 | 3% |
| Other | 0.5 | 1 | 2 | 1% |
| **Total** | **9** | **67** | **135** | **100%** |

```typescript
interface PowerConsumption {
  processor: number; // mW
  transmitter: number; // mW
  microphones: number; // mW
  wireless: number; // mW
  display: number; // mW
  total: number; // mW
}

function calculateBatteryLife(
  battery: BatterySpecification,
  consumption: PowerConsumption,
  usagePattern: UsagePattern
): BatteryLifeEstimate {
  // Calculate weighted average consumption
  const avgConsumption =
    consumption.total * usagePattern.activeTime +
    consumption.processor * 0.1 * usagePattern.idleTime;

  // Battery life in hours
  const batteryLife = (battery.capacity * battery.voltage) / avgConsumption;

  return {
    estimatedHours: batteryLife,
    estimatedDays: batteryLife / usagePattern.hoursPerDay,
    confidence: 0.85 // ±15% variation expected
  };
}
```

### 12.3 Power Management Strategies

```typescript
interface PowerManagement {
  mode: 'max_performance' | 'balanced' | 'economy' | 'custom';
  batteryLevel: number; // 0-100%
  lowPowerThreshold: number; // % to trigger low power mode
  criticalThreshold: number; // % to trigger warnings
  powerSavingFeatures: PowerSavingFeature[];
}

type PowerSavingFeature =
  | 'reduce_sampling_rate'
  | 'reduce_channels'
  | 'disable_wireless'
  | 'reduce_display_brightness'
  | 'limit_peak_current';

function managePower(
  battery: BatteryState,
  mode: PowerManagement['mode']
): PowerConfig {
  const config: PowerConfig = {
    processingStrategy: 'ACE',
    samplingRate: 16000,
    activeChannels: 22,
    wirelessEnabled: true,
    displayBrightness: 100
  };

  if (battery.level < 20 || mode === 'economy') {
    // Low power mode
    config.samplingRate = 8000; // Reduce by 50%
    config.activeChannels = 12; // Reduce to essentials
    config.wirelessEnabled = false;
    config.displayBrightness = 50;
  } else if (mode === 'balanced') {
    // Balanced mode
    config.samplingRate = 12000;
    config.activeChannels = 18;
    config.wirelessEnabled = true;
    config.displayBrightness = 75;
  }
  // Max performance uses defaults

  return config;
}
```

### 12.4 Charging Protocol

```typescript
interface ChargingProtocol {
  method: 'contact' | 'inductive' | 'usb-c';
  voltage: number; // V
  current: number; // mA
  stages: ChargingStage[];
  safetyFeatures: string[];
  temperature: { min: number; max: number }; // Celsius
}

interface ChargingStage {
  name: 'pre-charge' | 'constant-current' | 'constant-voltage' | 'trickle';
  targetCurrent?: number; // mA
  targetVoltage?: number; // V
  duration: number; // minutes
  terminationCondition: string;
}

const chargingProtocolStandard: ChargingProtocol = {
  method: 'inductive',
  voltage: 5.0,
  current: 500,
  stages: [
    {
      name: 'pre-charge',
      targetCurrent: 50,
      duration: 10,
      terminationCondition: 'Voltage > 3.0V'
    },
    {
      name: 'constant-current',
      targetCurrent: 500,
      duration: 120,
      terminationCondition: 'Voltage = 4.2V'
    },
    {
      name: 'constant-voltage',
      targetVoltage: 4.2,
      duration: 60,
      terminationCondition: 'Current < 50mA'
    },
    {
      name: 'trickle',
      targetCurrent: 10,
      duration: 30,
      terminationCondition: 'Full charge'
    }
  ],
  safetyFeatures: [
    'temperature_monitoring',
    'overcharge_protection',
    'short_circuit_protection',
    'reverse_polarity_protection'
  ],
  temperature: { min: 10, max: 40 }
};
```

---

## 13. Safety and Biocompatibility

### 13.1 Electrical Safety Standards

**Current Limits:**
```
Single Electrode:
- Maximum Current: 1.75 mA (per electrode)
- Maximum Charge: 50 nC per phase
- Pulse Width: 10-400 μs
- Stimulation Rate: <50,000 pps (total across all electrodes)

Total Device:
- Maximum Total Current: 10 mA
- Charge Balanced: ±5% balance required
- DC Component: <1 μA
```

```typescript
interface ElectricalSafety {
  // Current limits
  maxCurrentPerElectrode: number; // mA
  maxTotalCurrent: number; // mA
  maxChargePerPhase: number; // nC

  // Pulse characteristics
  pulseWidth: { min: number; max: number }; // μs
  interphaseGap: number; // μs (charge recovery)
  chargeBalance: number; // % (biphasic balance)

  // Safety features
  shortCircuitDetection: boolean;
  openCircuitDetection: boolean;
  overCurrentShutdown: boolean;
  electrodeImpedanceMonitoring: boolean;
}

function verifySafeLimits(
  stimulation: StimulationPattern,
  limits: ElectricalSafety
): SafetyCheck {
  const checks: SafetyCheck = {
    passed: true,
    violations: []
  };

  // Check per-electrode current
  stimulation.electrodes.forEach(e => {
    if (e.current > limits.maxCurrentPerElectrode) {
      checks.passed = false;
      checks.violations.push({
        type: 'CURRENT_EXCEEDED',
        electrode: e.number,
        value: e.current,
        limit: limits.maxCurrentPerElectrode
      });
    }
  });

  // Check charge balance
  stimulation.electrodes.forEach(e => {
    const balance = calculateChargeBalance(e.waveform);
    if (Math.abs(balance) > limits.chargeBalance / 100) {
      checks.passed = false;
      checks.violations.push({
        type: 'CHARGE_IMBALANCE',
        electrode: e.number,
        balance: balance * 100,
        limit: limits.chargeBalance
      });
    }
  });

  return checks;
}
```

### 13.2 Biocompatibility Requirements

**Standards Compliance:**
- ISO 10993: Biological evaluation of medical devices
- ISO 14708-7: Implants for surgery - Cochlear implant systems
- IEC 60601-1: Medical electrical equipment safety

**Materials:**
```typescript
interface BiocompatibleMaterial {
  name: string;
  application: 'electrode' | 'housing' | 'lead' | 'coating';
  standard: string; // ISO standard
  cytotoxicity: 'pass' | 'fail';
  sensitization: 'pass' | 'fail';
  irritation: 'pass' | 'fail';
  implantDuration: 'limited' | 'prolonged' | 'permanent';
}

const approvedMaterials: BiocompatibleMaterial[] = [
  {
    name: 'Platinum-Iridium (90/10)',
    application: 'electrode',
    standard: 'ISO 10993-1',
    cytotoxicity: 'pass',
    sensitization: 'pass',
    irritation: 'pass',
    implantDuration: 'permanent'
  },
  {
    name: 'Medical Grade Silicone',
    application: 'housing',
    standard: 'ISO 10993-1',
    cytotoxicity: 'pass',
    sensitization: 'pass',
    irritation: 'pass',
    implantDuration: 'permanent'
  },
  {
    name: 'Titanium (Grade 4)',
    application: 'housing',
    standard: 'ISO 10993-1',
    cytotoxicity: 'pass',
    sensitization: 'pass',
    irritation: 'pass',
    implantDuration: 'permanent'
  },
  {
    name: 'Parylene-C',
    application: 'coating',
    standard: 'ISO 10993-1',
    cytotoxicity: 'pass',
    sensitization: 'pass',
    irritation: 'pass',
    implantDuration: 'permanent'
  }
];
```

### 13.3 MRI Safety

**MRI Compatibility:**
```typescript
interface MRISafety {
  conditionalSafe: boolean;
  maxFieldStrength: number; // Tesla
  requiresRemoval: ComponentRemoval[];
  restrictions: MRIRestriction[];
  artifacts: ArtifactZone[];
}

interface ComponentRemoval {
  component: 'external_processor' | 'magnet' | 'coil';
  required: boolean;
  procedure: string;
}

const mriSafetyProfile: MRISafety = {
  conditionalSafe: true,
  maxFieldStrength: 1.5, // Some newer models: 3.0T
  requiresRemoval: [
    {
      component: 'external_processor',
      required: true,
      procedure: 'Remove processor and headpiece before MRI'
    },
    {
      component: 'magnet',
      required: false, // Newer models with rotatable magnets
      procedure: 'Align magnet with field direction or remove if necessary'
    }
  ],
  restrictions: [
    'Head-only scans preferred',
    'Specific Absorption Rate (SAR) limits apply',
    'Magnet alignment required',
    'Head bandage to secure position'
  ],
  artifacts: [
    {
      location: 'Internal device',
      radius: 30, // mm
      severity: 'significant'
    },
    {
      location: 'Electrode array',
      radius: 15, // mm
      severity: 'moderate'
    }
  ]
};
```

### 13.4 Sterilization and Packaging

```typescript
interface SterilizationProtocol {
  method: 'ethylene_oxide' | 'gamma_radiation' | 'steam' | 'plasma';
  temperature: number; // Celsius
  duration: number; // hours
  validation: 'ISO 11135' | 'ISO 11137' | 'ISO 17665';
  sterileShelfLife: number; // years
}

const sterilizationStandard: SterilizationProtocol = {
  method: 'ethylene_oxide',
  temperature: 55,
  duration: 12,
  validation: 'ISO 11135',
  sterileShelfLife: 5
};
```

---

## 14. Calibration and Fitting Procedures

### 14.1 Initial Fitting Protocol

**Timeline:**
```
Activation (1-4 weeks post-surgery):
├── Week 0-1: Healing period, no stimulation
├── Week 1-4: First activation appointment
│   ├── Impedance testing
│   ├── Neural Response Telemetry (NRT)
│   ├── T-level determination
│   ├── C-level determination
│   ├── Initial MAP creation
│   └── Basic auditory perception testing
├── Week 4-8: Fine-tuning (2-3 appointments)
└── Month 3-12: Optimization (quarterly)
```

### 14.2 Threshold Determination

```typescript
interface ThresholdMeasurement {
  electrode: number;
  tLevel: number; // Threshold (CL, current level)
  cLevel: number; // Comfortable loudness (CL)
  dynamicRange: number; // C-level - T-level
  measurementMethod: 'behavioral' | 'objective' | 'hybrid';
  confidence: number; // 0-1
}

function measureThresholds(
  electrodeCount: number,
  patient: PatientInfo,
  method: 'behavioral' | 'objective'
): ThresholdMeasurement[] {
  const measurements: ThresholdMeasurement[] = [];

  for (let electrode = 1; electrode <= electrodeCount; electrode++) {
    if (method === 'behavioral') {
      // Behavioral psychophysics
      const tLevel = behavioralTLevel(electrode, patient);
      const cLevel = behavioralCLevel(electrode, patient, tLevel);

      measurements.push({
        electrode,
        tLevel,
        cLevel,
        dynamicRange: cLevel - tLevel,
        measurementMethod: 'behavioral',
        confidence: 0.9
      });
    } else {
      // Objective: NRT (Neural Response Telemetry) or ECAP
      const tLevel = objectiveTLevel(electrode);
      const cLevel = estimateCLevel(tLevel); // Typically T + 10-20 CL

      measurements.push({
        electrode,
        tLevel,
        cLevel,
        dynamicRange: cLevel - tLevel,
        measurementMethod: 'objective',
        confidence: 0.7
      });
    }
  }

  return measurements;
}

// Behavioral T-level measurement (ascending method)
function behavioralTLevel(electrode: number, patient: PatientInfo): number {
  let currentLevel = 0;
  let detected = false;

  while (!detected && currentLevel < 255) {
    currentLevel += 5; // Small increments
    const response = presentStimulus(electrode, currentLevel);
    detected = patient.responseDetected(response);
  }

  // Confirm with descending method
  const descendingT = descendingTLevel(electrode, currentLevel);

  // Average for final T-level
  return Math.round((currentLevel + descendingT) / 2);
}

// Behavioral C-level measurement (bracketing method)
function behavioralCLevel(
  electrode: number,
  patient: PatientInfo,
  tLevel: number
): number {
  let low = tLevel;
  let high = 255;
  let comfortableLevel = low;

  // Bracketing to find comfortable loudness
  while (high - low > 2) {
    const mid = Math.round((low + high) / 2);
    const loudness = patient.rateLoudness(electrode, mid);

    if (loudness === 'too_soft') {
      low = mid;
    } else if (loudness === 'too_loud') {
      high = mid;
    } else if (loudness === 'comfortable') {
      comfortableLevel = mid;
      break;
    }
  }

  return comfortableLevel;
}
```

### 14.3 MAP Creation

```typescript
interface DeviceMAP {
  patientId: string;
  deviceId: string;
  createdDate: Date;
  audiologist: string;

  // Processing
  processingStrategy: ProcessingStrategy;
  stimulationRate: number; // Hz
  pulseWidth: number; // μs

  // Electrode settings
  electrodes: ElectrodeSettings[];

  // Global settings
  sensitivity: number; // Microphone gain
  volume: number; // Overall loudness
  compression: number; // Dynamic range compression ratio

  // Advanced features
  noiseReduction: NoiseReductionMode;
  directionality: DirectionalMode;
  frequencyMap: FrequencyMap[];

  // Programs
  programs: Program[];
  activeProgram: number;
}

interface ElectrodeSettings {
  electrode: number;
  active: boolean;
  tLevel: number; // Threshold
  cLevel: number; // Comfortable
  gain: number; // Individual electrode gain
  frequencyAllocation: { low: number; high: number }; // Hz
}

function createInitialMAP(
  thresholds: ThresholdMeasurement[],
  deviceType: DeviceType,
  patientProfile: PatientInfo
): DeviceMAP {
  // Select appropriate processing strategy
  const strategy = selectStrategy(deviceType, patientProfile);

  // Create electrode settings
  const electrodes: ElectrodeSettings[] = thresholds.map(t => ({
    electrode: t.electrode,
    active: t.dynamicRange > 5, // Deactivate if poor dynamic range
    tLevel: t.tLevel,
    cLevel: t.cLevel,
    gain: 0, // Start neutral
    frequencyAllocation: getFrequencyForElectrode(t.electrode, thresholds.length)
  }));

  // Create default programs
  const programs: Program[] = [
    {
      name: 'Everyday',
      strategy,
      sensitivity: 12, // Typical
      noiseReduction: 'medium',
      directionality: 'adaptive'
    },
    {
      name: 'Quiet',
      strategy,


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
