# WIA-AUG-007: Bionic Limb Specification v1.0

> **Standard ID:** WIA-AUG-007
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Human Augmentation Bionics Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Limb Classification System](#2-limb-classification-system)
3. [Control Methods](#3-control-methods)
4. [Sensory Feedback Systems](#4-sensory-feedback-systems)
5. [Socket Interface Standards](#5-socket-interface-standards)
6. [Grip Patterns and Dexterity](#6-grip-patterns-and-dexterity)
7. [Gait Analysis (Lower Limbs)](#7-gait-analysis-lower-limbs)
8. [Power and Battery Management](#8-power-and-battery-management)
9. [Maintenance and Calibration](#9-maintenance-and-calibration)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for bionic limb prosthetics, including upper and lower limb replacements. The standard ensures interoperability, safety, and optimal functionality across different manufacturers and control systems.

### 1.2 Scope

The standard covers:
- Classification of bionic limb types
- Control method specifications
- Sensory feedback requirements
- Socket interface standards
- Grip pattern definitions
- Gait analysis for lower limbs
- Power management protocols
- Maintenance and calibration procedures

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Bionic limb technologies should restore not just function, but dignity and independence to amputees. This specification ensures that prosthetic systems are standardized, accessible, and continuously improving to serve all who need them.

### 1.4 Terminology

- **Residual Limb**: The remaining biological limb after amputation
- **Socket**: The interface between residual limb and prosthetic device
- **DOF (Degrees of Freedom)**: Number of independent movements
- **EMG**: Electromyography - electrical signals from muscles
- **Proprioception**: Sense of limb position and movement
- **Myoelectric**: Control via muscle electrical signals
- **TMR**: Targeted Muscle Reinnervation
- **Osseointegration**: Direct skeletal attachment

---

## 2. Limb Classification System

### 2.1 Upper Limb Categories

| Category | Amputation Level | DOF Range | Typical Components |
|----------|------------------|-----------|-------------------|
| FINGER | Partial hand | 1-3 | Individual digit prosthetics |
| HAND | Wrist disarticulation | 3-6 | Hand only, passive wrist |
| FOREARM | Below elbow | 6-12 | Hand + powered wrist |
| UPPER_ARM | Above elbow | 12-18 | Hand + wrist + powered elbow |

### 2.2 Lower Limb Categories

| Category | Amputation Level | DOF Range | Typical Components |
|----------|------------------|-----------|-------------------|
| FOOT | Partial foot/ankle | 2-4 | Forefoot or ankle unit |
| LOWER_LEG | Below knee | 4-8 | Foot + powered ankle |
| THIGH | Above knee | 8-12 | Foot + ankle + microprocessor knee |

### 2.3 Classification Algorithm

```typescript
interface LimbClassification {
  type: LimbType;
  amputationLevel: string;
  residualLength: number; // cm
  dof: number;
  category: 'Minimal' | 'Basic' | 'Moderate' | 'Advanced';
}

function classifyLimb(input: {
  type: LimbType;
  dof: number;
  controlComplexity: number;
}): LimbClassification {
  const score = input.dof * 0.6 + input.controlComplexity * 0.4;

  let category: string;
  if (score <= 5) category = 'Minimal';
  else if (score <= 10) category = 'Basic';
  else if (score <= 15) category = 'Moderate';
  else category = 'Advanced';

  return { ...input, category };
}
```

### 2.4 Complexity Score

```
Complexity = (DOF × 0.4) + (Sensors × 0.3) + (Control Methods × 0.3)
```

Where:
- `DOF` = Degrees of freedom (1-20)
- `Sensors` = Number of sensor types (0-10)
- `Control Methods` = Number of available control methods (1-5)

---

## 3. Control Methods

### 3.1 Control Method Types

#### 3.1.1 Myoelectric Control

Surface EMG signals from residual limb muscles control prosthetic movement.

**Specifications:**
```
Signal Frequency: 20-450 Hz
Sampling Rate: ≥1000 Hz
Electrode Count: 2-16 channels
Processing Delay: <200ms
Classification Accuracy: ≥85%
```

**Algorithm:**
```typescript
interface MyoelectricControl {
  electrodes: ElectrodeConfig[];
  samplingRate: number; // Hz
  filterBandpass: { low: number; high: number };
  threshold: number; // μV
  processingDelay: number; // ms
}

function processEMG(signal: number[], config: MyoelectricControl): ControlSignal {
  // 1. Band-pass filter
  const filtered = bandpassFilter(signal, config.filterBandpass);

  // 2. Feature extraction (RMS, MAV, WL, etc.)
  const features = extractFeatures(filtered);

  // 3. Classification
  const intent = classifyIntent(features);

  // 4. Generate control signal
  return generateControl(intent);
}
```

#### 3.1.2 Neural Direct Control

Direct interface with peripheral or central nervous system.

**Specifications:**
```
Interface Type: Implanted electrodes
Channel Count: 16-128 channels
Impedance: 10-500 kΩ
Signal-to-Noise Ratio: ≥40 dB
Response Time: <100ms
Biocompatibility: Full ISO 10993
```

#### 3.1.3 Pattern Recognition

Machine learning-based gesture classification.

**Specifications:**
```
Training Time: 15-30 minutes initial
Retraining: Weekly recommended
Gesture Library: 10-50 gestures
Accuracy: ≥90% for trained gestures
Cross-session Stability: ≥80%
```

**ML Pipeline:**
```typescript
interface PatternRecognitionConfig {
  algorithm: 'LDA' | 'SVM' | 'CNN' | 'LSTM';
  features: FeatureType[];
  trainingEpochs: number;
  validationSplit: number;
}

function trainPatternRecognition(
  trainingData: EMGData[],
  labels: GestureLabel[],
  config: PatternRecognitionConfig
): TrainedModel {
  // Feature extraction
  const features = trainingData.map(d => extractFeatures(d, config.features));

  // Train model
  const model = trainModel(features, labels, config);

  // Validate
  const accuracy = validateModel(model, validationData);

  return { model, accuracy, config };
}
```

#### 3.1.4 Hybrid Control

Combination of multiple control methods.

**Common Combinations:**
- Myoelectric + Pattern Recognition
- Neural + Myoelectric
- Body-Powered + Myoelectric

### 3.2 Control Performance Metrics

| Metric | Minimum | Target | Measurement Method |
|--------|---------|--------|-------------------|
| Accuracy | 85% | 95% | Gesture classification |
| Response Time | <300ms | <150ms | Intent to motion |
| Learning Time | <60min | <20min | Initial calibration |
| Adaptation | 80% | 95% | Cross-session accuracy |
| Robustness | 75% | 90% | Performance under load |

### 3.3 Control Modes

```typescript
enum ControlMode {
  DIRECT = 'direct',           // One-to-one muscle mapping
  SEQUENTIAL = 'sequential',   // Mode switching
  PROPORTIONAL = 'proportional', // Speed/force control
  SIMULTANEOUS = 'simultaneous', // Multi-joint control
  ADAPTIVE = 'adaptive'        // Learning-based adaptation
}
```

---

## 4. Sensory Feedback Systems

### 4.1 Feedback Modalities

#### 4.1.1 Pressure Feedback

**Requirements:**
```
Dynamic Range: 0-200 N
Resolution: 1 N
Response Time: <50ms
Feedback Method: Vibration, electrical stimulation
Spatial Resolution: ≥5 locations
```

**Implementation:**
```typescript
interface PressureFeedback {
  sensors: PressureSensor[];
  feedbackType: 'vibration' | 'electrical' | 'mechanical';
  intensity: number; // 0-1
  location: SensorLocation;
  calibration: CalibrationData;
}

function providePressureFeedback(
  force: number,
  config: PressureFeedback
): FeedbackSignal {
  // Map force to feedback intensity
  const intensity = mapForceToIntensity(force, config.calibration);

  // Generate feedback signal
  const signal = generateFeedback(intensity, config.feedbackType);

  // Deliver to user
  return deliverFeedback(signal, config.location);
}
```

#### 4.1.2 Temperature Feedback

**Requirements:**
```
Range: 15-45°C
Accuracy: ±1°C
Response Time: <500ms
Safety Cutoff: >45°C or <15°C
```

#### 4.1.3 Position Feedback (Proprioception)

**Requirements:**
```
Joint Angle Accuracy: ±2°
Update Rate: ≥50 Hz
Latency: <100ms
Method: Vibration patterns, sensory substitution
```

#### 4.1.4 Slip Detection

**Requirements:**
```
Detection Threshold: 0.5 mm movement
Response Time: <30ms
False Positive Rate: <5%
Action: Automatic grip adjustment
```

### 4.2 Feedback Encoding Schemes

```typescript
interface FeedbackEncoding {
  modality: FeedbackType;
  encoding: 'amplitude' | 'frequency' | 'spatial' | 'temporal';
  intensityLevels: number;
  mappingFunction: (input: number) => FeedbackSignal;
}

// Example: Pressure mapped to vibration frequency
const pressureToVibration: FeedbackEncoding = {
  modality: 'PRESSURE',
  encoding: 'frequency',
  intensityLevels: 10,
  mappingFunction: (force: number) => ({
    frequency: 50 + (force / 200) * 200, // 50-250 Hz
    amplitude: 0.7,
    duration: 0 // continuous
  })
};
```

### 4.3 Multi-modal Feedback Integration

```typescript
interface MultiModalFeedback {
  pressure: PressureFeedback;
  temperature: TemperatureFeedback;
  position: PositionFeedback;
  priority: FeedbackPriority[];
  fusionAlgorithm: FusionMethod;
}

function integrateFeedback(
  inputs: SensorData[],
  config: MultiModalFeedback
): IntegratedFeedback {
  // Priority-based fusion
  const prioritized = prioritizeFeedback(inputs, config.priority);

  // Combine modalities
  const fused = fuseFeedback(prioritized, config.fusionAlgorithm);

  // Deliver to user
  return deliverIntegratedFeedback(fused);
}
```

---

## 5. Socket Interface Standards

### 5.1 Socket Design Requirements

#### 5.1.1 Fit and Comfort

**Specifications:**
```
Pressure Distribution: <50 kPa max at any point
Contact Area: ≥70% of residual limb surface
Material: Medical-grade silicone or thermoplastic
Liner Thickness: 3-6 mm
Donning Time: <2 minutes
```

#### 5.1.2 Suspension Methods

| Method | Retention Force | Comfort | Adjustability |
|--------|----------------|---------|---------------|
| Suction | High | Excellent | Low |
| Pin/Lock | Very High | Good | Medium |
| Sleeve | Medium | Excellent | High |
| Lanyard | Low | Fair | Very High |
| Osseointegration | Permanent | Variable | None |

#### 5.1.3 Socket Materials

```typescript
interface SocketMaterial {
  name: string;
  hardness: number; // Shore A
  biocompatibility: boolean;
  thermalConductivity: number; // W/m·K
  breathability: 'none' | 'low' | 'medium' | 'high';
  durability: number; // months typical life
}

const approvedMaterials: SocketMaterial[] = [
  {
    name: 'Medical Silicone',
    hardness: 20,
    biocompatibility: true,
    thermalConductivity: 0.2,
    breathability: 'low',
    durability: 12
  },
  {
    name: 'Thermoplastic Elastomer (TPE)',
    hardness: 40,
    biocompatibility: true,
    thermalConductivity: 0.15,
    breathability: 'medium',
    durability: 18
  },
  // ... more materials
];
```

### 5.2 Pressure Mapping

```typescript
interface PressureMap {
  grid: number[][]; // kPa values
  resolution: { rows: number; cols: number };
  timestamp: Date;
  maxPressure: number;
  meanPressure: number;
  hotspots: Location[];
}

function analyzePressureMap(map: PressureMap): SocketFitAssessment {
  const hotspots = map.grid.flatMap((row, i) =>
    row.map((p, j) => p > 50 ? { row: i, col: j, pressure: p } : null)
  ).filter(Boolean);

  const quality = hotspots.length === 0 ? 'Excellent' :
                  hotspots.length <= 3 ? 'Good' :
                  'Needs Adjustment';

  return {
    quality,
    hotspots,
    recommendations: generateFitRecommendations(hotspots)
  };
}
```

### 5.3 Socket Fitting Protocol

```
1. Residual Limb Assessment
   - Measure circumference at 5cm intervals
   - Identify bony prominences
   - Assess tissue quality
   - Document sensitive areas

2. Socket Fabrication
   - 3D scan or cast
   - Digital modification
   - Test socket fabrication
   - Pressure mapping verification

3. Fit Evaluation
   - Static alignment check
   - Pressure distribution analysis
   - Range of motion testing
   - User comfort assessment

4. Adjustment Cycle
   - Identify pressure points
   - Modify socket
   - Re-test
   - Iterate until optimal fit

5. Final Verification
   - Full functional testing
   - Prolonged wear trial (4-8 hours)
   - User satisfaction survey
   - Documentation
```

---

## 6. Grip Patterns and Dexterity

### 6.1 Standard Grip Patterns

#### 6.1.1 Power Grips

```typescript
enum PowerGrip {
  CYLINDRICAL = 'cylindrical',  // Holding tools, bottles
  SPHERICAL = 'spherical',      // Holding balls, fruit
  HOOK = 'hook',                // Carrying bags, heavy items
  LATERAL = 'lateral'           // Key grip
}

interface GripConfig {
  pattern: PowerGrip;
  force: number; // Newtons (0-200)
  speed: number; // 0-1 (percentage of max)
  precision: boolean;
}
```

#### 6.1.2 Precision Grips

```typescript
enum PrecisionGrip {
  TRIPOD_PINCH = 'tripod_pinch',    // Writing, eating
  LATERAL_PINCH = 'lateral_pinch',  // Turning keys
  TIP_PINCH = 'tip_pinch',          // Small objects
  PRECISION_GRIP = 'precision_grip' // Fine manipulation
}
```

### 6.2 Grip Force Control

**Requirements:**
```
Minimum Force: 5 N
Maximum Force: 200 N (adult), 100 N (child)
Force Resolution: 1 N
Force Stability: ±5% during hold
Proportional Control: 10 levels minimum
```

**Algorithm:**
```typescript
interface ForceController {
  currentForce: number;
  targetForce: number;
  maxForce: number;
  kp: number; // Proportional gain
  ki: number; // Integral gain
  kd: number; // Derivative gain
}

function pidForceControl(
  controller: ForceController,
  sensorForce: number,
  dt: number
): number {
  const error = controller.targetForce - sensorForce;
  const integral = controller.integral + error * dt;
  const derivative = (error - controller.prevError) / dt;

  const output =
    controller.kp * error +
    controller.ki * integral +
    controller.kd * derivative;

  return clamp(output, 0, controller.maxForce);
}
```

### 6.3 Grip Pattern Library

```json
{
  "gripPatterns": [
    {
      "id": "power_grip_01",
      "name": "Cylindrical Power Grip",
      "fingerPositions": {
        "thumb": { "flexion": 60, "abduction": 30 },
        "index": { "flexion": 90, "abduction": 0 },
        "middle": { "flexion": 95, "abduction": 0 },
        "ring": { "flexion": 95, "abduction": 0 },
        "pinky": { "flexion": 90, "abduction": 0 }
      },
      "force": { "min": 20, "max": 150 },
      "applications": ["tool_holding", "bottle_grip", "handle_grip"]
    },
    {
      "id": "precision_grip_01",
      "name": "Tripod Pinch",
      "fingerPositions": {
        "thumb": { "flexion": 30, "abduction": 40 },
        "index": { "flexion": 45, "abduction": 20 },
        "middle": { "flexion": 50, "abduction": 15 }
      },
      "force": { "min": 2, "max": 30 },
      "applications": ["writing", "eating", "precision_work"]
    }
  ]
}
```

### 6.4 Dexterity Metrics

```typescript
interface DexterityAssessment {
  gripPatterns: number;        // Number of available grips
  transitionTime: number;      // ms between grips
  forceResolution: number;     // Minimum force increment (N)
  independentFingers: number;  // DOF count
  manipulationScore: number;   // 0-100 composite score
}

function assessDexterity(limb: BionicLimb): DexterityAssessment {
  const score =
    (limb.gripPatterns.length * 5) +
    (100 / limb.averageTransitionTime) +
    (limb.dof * 3) +
    (limb.forceControl.resolution * 2);

  return {
    gripPatterns: limb.gripPatterns.length,
    transitionTime: limb.averageTransitionTime,
    forceResolution: limb.forceControl.resolution,
    independentFingers: limb.dof,
    manipulationScore: Math.min(100, score)
  };
}
```

---

## 7. Gait Analysis (Lower Limbs)

### 7.1 Gait Cycle Phases

```
Stance Phase (60%):
├── Initial Contact (0-2%)
├── Loading Response (2-12%)
├── Mid Stance (12-31%)
├── Terminal Stance (31-50%)
└── Pre-Swing (50-62%)

Swing Phase (40%):
├── Initial Swing (62-75%)
├── Mid Swing (75-87%)
└── Terminal Swing (87-100%)
```

### 7.2 Gait Parameters

```typescript
interface GaitParameters {
  spatiotemporal: {
    strideLength: number;      // cm
    stepLength: number;        // cm
    stepWidth: number;         // cm
    cadence: number;           // steps/min
    velocity: number;          // m/s
    stanceTime: number;        // % gait cycle
    swingTime: number;         // % gait cycle
    doubleSupport: number;     // % gait cycle
  };
  kinematics: {
    hipFlexion: AngleRange;
    kneeFlexion: AngleRange;
    ankleDorsiflexion: AngleRange;
    pelvicTilt: AngleRange;
  };
  kinetics: {
    groundReactionForce: ForceVector[];
    kneeMoment: number[];
    anklePower: number[];
  };
}
```

### 7.3 Normal Gait Ranges

| Parameter | Normal Range | Prosthetic Target |
|-----------|--------------|-------------------|
| Stride Length | 130-150 cm | 120-145 cm |
| Cadence | 110-120 steps/min | 100-115 steps/min |
| Stance/Swing | 60:40 | 58:42 - 62:38 |
| Knee Flexion (swing) | 60-70° | 55-65° |
| Ankle Dorsiflexion | 10-15° | 8-12° |
| Walking Speed | 1.2-1.4 m/s | 1.0-1.3 m/s |

### 7.4 Gait Analysis Algorithm

```typescript
interface GaitAnalysis {
  leftStep: GaitCycle;
  rightStep: GaitCycle;
  symmetry: SymmetryMetrics;
  efficiency: number;
  stability: number;
}

function analyzeGait(
  sensorData: IMUData[],
  duration: number
): GaitAnalysis {
  // Detect gait events
  const heelStrikes = detectHeelStrikes(sensorData);
  const toeOffs = detectToeOffs(sensorData);

  // Segment gait cycles
  const cycles = segmentGaitCycles(heelStrikes, toeOffs);

  // Calculate parameters for each cycle
  const parameters = cycles.map(c => calculateGaitParameters(c));

  // Assess symmetry
  const symmetry = assessSymmetry(parameters);

  // Calculate efficiency and stability
  const efficiency = calculateEfficiency(parameters);
  const stability = calculateStability(sensorData);

  return {
    leftStep: parameters.filter(p => p.side === 'left')[0],
    rightStep: parameters.filter(p => p.side === 'right')[0],
    symmetry,
    efficiency,
    stability
  };
}
```

### 7.5 Symmetry Assessment

```typescript
interface SymmetryMetrics {
  spatialSymmetry: number;    // 0-1 (1 = perfect)
  temporalSymmetry: number;   // 0-1
  forceSymmetry: number;      // 0-1
  overallSymmetry: number;    // 0-1
}

function assessSymmetry(
  left: GaitParameters,
  right: GaitParameters
): SymmetryMetrics {
  const spatial = 1 - Math.abs(
    (left.spatiotemporal.strideLength - right.spatiotemporal.strideLength) /
    ((left.spatiotemporal.strideLength + right.spatiotemporal.strideLength) / 2)
  );

  const temporal = 1 - Math.abs(
    (left.spatiotemporal.stanceTime - right.spatiotemporal.stanceTime) /
    ((left.spatiotemporal.stanceTime + right.spatiotemporal.stanceTime) / 2)
  );

  // Force symmetry calculation...
  const force = 0.9; // placeholder

  const overall = (spatial + temporal + force) / 3;

  return { spatial, temporal, force, overall };
}
```

### 7.6 Microprocessor Knee Control

```typescript
interface KneeControl {
  mode: 'stance' | 'swing' | 'transition';
  resistance: number;     // 0-1
  flexionAngle: number;   // degrees
  extensionStop: number;  // degrees
  swingFlexion: number;   // degrees
  terrain: TerrainType;
}

function controlKnee(
  sensors: SensorData,
  phase: GaitPhase,
  config: KneeControl
): KneeCommand {
  switch (phase) {
    case 'stance':
      // High resistance during weight bearing
      return {
        resistance: 0.9,
        targetAngle: sensors.kneeAngle,
        mode: 'lock'
      };

    case 'swing':
      // Low resistance, natural flexion
      return {
        resistance: 0.1,
        targetAngle: config.swingFlexion,
        mode: 'free'
      };

    case 'pre_swing':
      // Prepare for swing
      return {
        resistance: 0.3,
        targetAngle: 0,
        mode: 'transition'
      };
  }
}
```

---

## 8. Power and Battery Management

### 8.1 Battery Requirements

```typescript
interface BatterySpecification {
  type: 'LiPo' | 'Li-ion' | 'LiFePO4';
  capacity: number;        // mAh
  voltage: number;         // V
  weight: number;          // grams
  cycles: number;          // charge cycles
  runtime: number;         // hours (typical use)
  chargingTime: number;    // hours
  safetyFeatures: string[];
}

const standardBattery: BatterySpecification = {
  type: 'Li-ion',
  capacity: 2000,
  voltage: 7.4,
  weight: 60,
  cycles: 500,
  runtime: 8,
  chargingTime: 2,
  safetyFeatures: [
    'overcharge_protection',
    'short_circuit_protection',
    'thermal_cutoff'
  ]
};
```

### 8.2 Power Consumption

| Component | Idle (mW) | Active (mW) | Peak (mW) |
|-----------|-----------|-------------|-----------|
| Control System | 50 | 200 | 500 |
| Sensors | 20 | 100 | 150 |
| Motors (per joint) | 0 | 1000 | 5000 |
| Feedback System | 10 | 50 | 100 |
| Communication | 5 | 30 | 60 |

### 8.3 Power Management Strategy

```typescript
interface PowerManagement {
  mode: 'performance' | 'balanced' | 'economy';
  batteryLevel: number; // 0-100%
  estimatedRuntime: number; // minutes
  powerSaving: boolean;
}

function managePower(
  battery: BatteryState,
  activity: ActivityLevel
): PowerManagement {
  // Calculate mode based on battery and activity
  let mode: PowerManagement['mode'];

  if (battery.level > 50 && activity === 'high') {
    mode = 'performance';
  } else if (battery.level > 20) {
    mode = 'balanced';
  } else {
    mode = 'economy';
  }

  // Estimate runtime
  const consumption = estimateConsumption(activity, mode);
  const runtime = (battery.level / 100) * (battery.capacity / consumption) * 60;

  // Enable power saving if needed
  const powerSaving = battery.level < 20;

  return {
    mode,
    batteryLevel: battery.level,
    estimatedRuntime: runtime,
    powerSaving
  };
}
```

### 8.4 Charging Protocol

```
1. Pre-charge Check
   - Verify battery health
   - Check temperature (10-40°C)
   - Inspect connections

2. Charging Stages
   - Stage 1: Constant Current (CC) - 1C rate
   - Stage 2: Constant Voltage (CV) - 4.2V per cell
   - Stage 3: Trickle charge until full

3. Safety Monitoring
   - Temperature monitoring
   - Voltage monitoring
   - Current monitoring
   - Auto-cutoff at full charge

4. Post-charge
   - Balance cells (multi-cell)
   - Record charge cycle
   - Update battery health metrics
```

### 8.5 Battery Health Monitoring

```typescript
interface BatteryHealth {
  cycleCount: number;
  capacity: number;        // % of original
  impedance: number;       // mΩ
  health: 'excellent' | 'good' | 'fair' | 'replace';
  estimatedLife: number;   // days
}

function assessBatteryHealth(
  battery: BatteryState,
  history: ChargeHistory[]
): BatteryHealth {
  const degradation = calculateDegradation(history);
  const currentCapacity = 100 - degradation;

  let health: BatteryHealth['health'];
  if (currentCapacity > 80) health = 'excellent';
  else if (currentCapacity > 60) health = 'good';
  else if (currentCapacity > 40) health = 'fair';
  else health = 'replace';

  const estimatedLife = estimateRemainingLife(
    battery.cycleCount,
    currentCapacity
  );

  return {
    cycleCount: battery.cycleCount,
    capacity: currentCapacity,
    impedance: battery.impedance,
    health,
    estimatedLife
  };
}
```

---

## 9. Maintenance and Calibration

### 9.1 Maintenance Schedule

#### 9.1.1 Daily Maintenance

```
User Tasks:
□ Visual inspection for damage
□ Clean socket and liner
□ Check battery level
□ Verify proper fit
□ Test basic functions
□ Charge battery overnight
```

#### 9.1.2 Weekly Maintenance

```
User Tasks:
□ Deep clean all components
□ Inspect cables and connections
□ Check for unusual sounds
□ Verify control responsiveness
□ Test all grip patterns
□ Check electrode contact (EMG systems)

Technician Tasks (if needed):
□ Pressure mapping check
□ Software updates
□ Calibration verification
```

#### 9.1.3 Monthly Maintenance

```
Technician Tasks:
□ Full system diagnostic
□ Recalibrate control system
□ Inspect mechanical components
□ Test safety systems
□ Update firmware
□ Document performance metrics
□ Adjust socket fit if needed
```

#### 9.1.4 Annual Maintenance

```
Comprehensive Service:
□ Complete disassembly and inspection
□ Replace wear components
□ Full recalibration
□ Battery health assessment
□ Structural integrity test
□ User training refresh
□ Performance benchmarking
□ Documentation update
```

### 9.2 Calibration Procedures

#### 9.2.1 Control System Calibration

```typescript
interface CalibrationProcedure {
  type: 'myoelectric' | 'neural' | 'pattern';
  duration: number;      // minutes
  exercises: Exercise[];
  validation: ValidationTest[];
  targetAccuracy: number; // 0-1
}

async function calibrateControl(
  limbId: string,
  userId: string,
  config: CalibrationProcedure
): Promise<CalibrationResult> {
  // Step 1: Baseline recording
  const baseline = await recordBaseline(limbId, 30);

  // Step 2: Training exercises
  const trainingData = [];
  for (const exercise of config.exercises) {
    const data = await recordExercise(exercise, userId);
    trainingData.push(data);
  }

  // Step 3: Model training
  const model = await trainModel(trainingData);

  // Step 4: Validation
  const accuracy = await validateModel(model, config.validation);

  // Step 5: Deploy if accurate enough
  if (accuracy >= config.targetAccuracy) {
    await deployModel(limbId, model);
    return {
      success: true,
      accuracy,
      timestamp: new Date()
    };
  } else {
    return {
      success: false,
      accuracy,
      message: 'Accuracy below threshold, recalibration needed'
    };
  }
}
```

#### 9.2.2 Force Calibration

```
Procedure:
1. Zero calibration (no load)
2. Span calibration (known loads)
   - 10N reference
   - 50N reference
   - 100N reference
   - 200N reference
3. Linearity verification
4. Hysteresis test
5. Repeatability test (10 cycles)
6. Generate calibration curve
7. Apply calibration to system
8. Verification test
```

#### 9.2.3 Sensor Calibration

```typescript
interface SensorCalibration {
  sensorId: string;
  type: SensorType;
  calibrationPoints: CalibrationPoint[];
  calibrationCurve: (input: number) => number;
  lastCalibrated: Date;
  nextCalibration: Date;
}

function calibrateSensor(
  sensor: Sensor,
  referenceInputs: number[],
  referenceOutputs: number[]
): SensorCalibration {
  // Fit calibration curve
  const curve = fitPolynomial(referenceInputs, referenceOutputs, 2);

  // Generate calibration function
  const calibrationCurve = (raw: number) => evaluatePolynomial(curve, raw);

  // Schedule next calibration
  const nextCalibration = new Date();
  nextCalibration.setMonth(nextCalibration.getMonth() + 3);

  return {
    sensorId: sensor.id,
    type: sensor.type,
    calibrationPoints: referenceInputs.map((input, i) => ({
      reference: input,
      measured: referenceOutputs[i]
    })),
    calibrationCurve,
    lastCalibrated: new Date(),
    nextCalibration
  };
}
```

### 9.3 Diagnostic Tests

```typescript
interface DiagnosticTest {
  testId: string;
  name: string;
  category: 'mechanical' | 'electrical' | 'control' | 'safety';
  procedure: TestProcedure;
  passCriteria: PassCriteria;
}

const diagnosticTests: DiagnosticTest[] = [
  {
    testId: 'MECH-001',
    name: 'Joint Range of Motion',
    category: 'mechanical',
    procedure: {
      steps: [
        'Position limb in starting position',
        'Move each joint through full range',
        'Record min and max angles',
        'Compare to specifications'
      ]
    },
    passCriteria: {
      minAcceptable: 'Within 5% of design spec',
      preferred: 'Within 2% of design spec'
    }
  },
  {
    testId: 'ELEC-001',
    name: 'Battery Health Check',
    category: 'electrical',
    procedure: {
      steps: [
        'Measure open-circuit voltage',
        'Perform load test',
        'Check internal impedance',
        'Verify charge time'
      ]
    },
    passCriteria: {
      minAcceptable: 'Capacity > 60% of original',
      preferred: 'Capacity > 80% of original'
    }
  },
  {
    testId: 'CTRL-001',
    name: 'Control Accuracy Test',
    category: 'control',
    procedure: {
      steps: [
        'Perform 20 predefined gestures',
        'Record classification results',
        'Calculate accuracy',
        'Document errors'
      ]
    },
    passCriteria: {
      minAcceptable: 'Accuracy ≥ 85%',
      preferred: 'Accuracy ≥ 95%'
    }
  }
];
```

### 9.4 Troubleshooting Guide

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
    symptom: 'Unresponsive control',
    possibleCauses: [
      'Low battery',
      'Electrode displacement',
      'Control system fault',
      'Signal interference'
    ],
    diagnosticSteps: [
      'Check battery level',
      'Verify electrode contact',
      'Test with known good signal',
      'Check for error codes'
    ],
    solutions: [
      'Recharge battery',
      'Reposition electrodes',
      'Recalibrate control system',
      'Contact technician if persistent'
    ],
    severity: 'high'
  },
  {
    symptom: 'Weak grip force',
    possibleCauses: [
      'Motor wear',
      'Low battery',
      'Mechanical binding',
      'Calibration drift'
    ],
    diagnosticSteps: [
      'Check battery level',
      'Test motor without load',
      'Inspect mechanical components',
      'Review force calibration'
    ],
    solutions: [
      'Recharge or replace battery',
      'Lubricate joints',
      'Recalibrate force control',
      'Service motor if needed'
    ],
    severity: 'medium'
  }
];
```

---

## 10. Implementation Guidelines

### 10.1 System Architecture

```typescript
interface BionicLimbSystem {
  hardware: {
    mechanics: MechanicalSystem;
    electronics: ElectronicSystem;
    sensors: SensorArray;
    actuators: ActuatorArray;
    power: PowerSystem;
  };
  software: {
    firmware: FirmwareVersion;
    control: ControlAlgorithm;
    calibration: CalibrationData;
    diagnostics: DiagnosticSuite;
  };
  interface: {
    socket: SocketDesign;
    feedback: FeedbackSystem;
    user: UserInterface;
  };
}
```

### 10.2 Integration Requirements

```
Hardware Integration:
□ All components use standard connectors
□ Modular design for easy replacement
□ IP54 minimum water/dust protection
□ EMC compliance (IEC 60601-1-2)
□ Biocompatible materials (ISO 10993)

Software Integration:
□ Standard API (RESTful or similar)
□ Data format: JSON or Protocol Buffers
□ Firmware updatable via OTA
□ Logging and telemetry capability
□ Open-source control algorithms

Safety Integration:
□ Emergency stop function
□ Fail-safe modes
□ Battery monitoring
□ Temperature monitoring
□ User alert system
```

### 10.3 Data Standards

```json
{
  "limbData": {
    "header": {
      "standardVersion": "WIA-AUG-007-v1.0",
      "limbId": "BL-2025-001",
      "timestamp": "2025-12-26T10:00:00Z"
    },
    "classification": {
      "type": "FOREARM",
      "dof": 8,
      "controlMethod": "PATTERN_RECOGNITION",
      "category": "Moderate"
    },
    "status": {
      "battery": 85,
      "functionalityScore": 95,
      "calibrationDate": "2025-12-20",
      "maintenanceDue": "2026-01-26"
    },
    "performance": {
      "controlAccuracy": 0.93,
      "gripForce": 120,
      "responseTime": 180
    }
  }
}
```

### 10.4 Testing Requirements

```typescript
interface TestSuite {
  functional: FunctionalTests;
  safety: SafetyTests;
  performance: PerformanceTests;
  usability: UsabilityTests;
  durability: DurabilityTests;
}

const requiredTests: TestSuite = {
  functional: {
    tests: [
      'All grip patterns functional',
      'Control system responsive',
      'Sensory feedback working',
      'Battery charging properly'
    ],
    passCriteria: 'All tests pass'
  },
  safety: {
    tests: [
      'Emergency stop functional',
      'Maximum force limits enforced',
      'Temperature limits enforced',
      'Fail-safe modes activate'
    ],
    passCriteria: '100% compliance'
  },
  performance: {
    tests: [
      'Control accuracy ≥85%',
      'Response time <300ms',
      'Battery life ≥8 hours',
      'Grip force 5-200N'
    ],
    passCriteria: 'Meet or exceed specs'
  },
  usability: {
    tests: [
      'User satisfaction survey',
      'Task completion time',
      'Learning curve assessment',
      'Comfort evaluation'
    ],
    passCriteria: '≥80% user satisfaction'
  },
  durability: {
    tests: [
      'Cycle testing (100k cycles)',
      'Drop test (1m height)',
      'Water resistance (IP54)',
      'Temperature cycling'
    ],
    passCriteria: 'No functional degradation'
  }
};
```

### 10.5 Certification Requirements

```
WIA-AUG-007 Certification Requirements:

1. Design Documentation
   □ Complete technical specifications
   □ Risk analysis (FMEA)
   □ User manual
   □ Maintenance guide

2. Testing Evidence
   □ Functional test results
   □ Safety test results
   □ Performance benchmarks
   □ Durability test results
   □ Clinical trial data (if applicable)

3. Manufacturing
   □ Quality management system (ISO 13485)
   □ Traceability system
   □ Supplier qualification
   □ Process validation

4. Safety Compliance
   □ WIA-AUG-013 (Augmentation Safety)
   □ ISO 10993 (Biocompatibility)
   □ IEC 60601 (Medical electrical equipment)
   □ Local regulatory compliance

5. Performance Verification
   □ Control accuracy ≥85%
   □ Battery life ≥8 hours
   □ Grip force range: 5-200N
   □ Response time <300ms
   □ User satisfaction ≥80%

6. Documentation
   □ Installation records
   □ Calibration records
   □ Maintenance logs
   □ Adverse event reporting system
```

---

## 11. References

### 11.1 International Standards

1. ISO 22523 - External limb prostheses and external orthoses
2. ISO 13405-1 - Prosthetics and orthotics vocabulary
3. ISO 10328 - Structural testing of lower-limb prostheses
4. ISO 22675 - Prosthetic ankle-foot devices and foot units
5. ISO 13485 - Medical devices — Quality management systems
6. IEC 60601-1 - Medical electrical equipment
7. ISO 10993 - Biological evaluation of medical devices

### 11.2 WIA Standards

- WIA-AUG-001: Human Augmentation General
- WIA-AUG-013: Augmentation Safety
- WIA-AUG-014: Human-Machine Interface
- WIA-BCI: Brain-Computer Interface
- WIA-MED: Medical Device Standards
- WIA-DATA: Healthcare Data Standards

### 11.3 Research References

- Kuiken 선행 연구. "Targeted muscle reinnervation for real-time myoelectric control of multifunction artificial arms." JAMA.
- Sensinger JW, Weir RF (2008). "Improved torque fidelity in harmonic drive sensors through the union of two existing strategies." IEEE/ASME Trans Mechatronics.
- 선행 연구. "Active upper limb prosthetics: A review on current state and upcoming breakthroughs." Progress in Biomedical Engineering.

---

## Appendix A: Limb Assessment Form

```
Patient Information:
Name: _______________  ID: _______________  Date: _______________
Amputation Level: _______________  Cause: _______________
Time since amputation: _______________

Residual Limb Assessment:
Length: ___ cm  Circumference: ___ cm (at ___ cm from end)
Tissue quality: □ Excellent  □ Good  □ Fair  □ Poor
Scarring: □ None  □ Minimal  □ Moderate  □ Extensive
Bony prominences: □ None  □ Mild  □ Moderate  □ Severe
Range of motion: _______________
Muscle strength: □ Excellent  □ Good  □ Fair  □ Poor

Functional Requirements:
Primary activities: _______________
Occupation: _______________
Hobbies: _______________
Environmental factors: _______________

Recommended System:
Limb type: _______________
Control method: _______________
DOF: _______________
Sensory feedback: □ Yes  □ No
Special features: _______________
```

## Appendix B: Maintenance Log Template

```
Limb ID: _______________
User ID: _______________

Date    | Type      | Performed By | Notes                | Next Due
--------|-----------|--------------|----------------------|----------
        | Daily     |              |                      |
        | Weekly    |              |                      |
        | Monthly   |              |                      |
        | Annual    |              |                      |

Issues Reported:
Date    | Issue            | Action Taken         | Resolved
--------|------------------|----------------------|----------
        |                  |                      | □ Yes □ No
        |                  |                      | □ Yes □ No

Calibration History:
Date    | Type             | Accuracy Before | Accuracy After
--------|------------------|-----------------|----------------
        | Control System   |                 |
        | Force Sensors    |                 |
        | Position Sensors |                 |
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUG-007 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
