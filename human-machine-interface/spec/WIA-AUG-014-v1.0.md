# WIA-AUG-014: Human-Machine Interface Specification v1.0

> **Standard ID:** WIA-AUG-014
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Human Augmentation Interface Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Signal Types and Encoding](#2-signal-types-and-encoding)
3. [Communication Protocol Stack](#3-communication-protocol-stack)
4. [Latency Requirements](#4-latency-requirements)
5. [Bidirectional Communication](#5-bidirectional-communication)
6. [Feedback Systems](#6-feedback-systems)
7. [Calibration Standards](#7-calibration-standards)
8. [Interoperability](#8-interoperability)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the protocols, data formats, and communication standards for Human-Machine Interfaces (HMI), enabling seamless bidirectional communication between human biological systems and augmentation devices.

### 1.2 Scope

The standard covers:
- Neural and biological signal encoding
- Communication protocol stack
- Latency requirements and optimization
- Feedback modalities and patterns
- Device calibration procedures
- Cross-device interoperability

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - By standardizing human-machine interfaces, we enable a future where augmentation devices work together seamlessly, giving users freedom of choice and ensuring technological compatibility across manufacturers.

### 1.4 Terminology

- **HMI**: Human-Machine Interface
- **BCI**: Brain-Computer Interface
- **EMG**: Electromyography (muscle signals)
- **EEG**: Electroencephalography (brain signals)
- **ECoG**: Electrocorticography (cortical signals)
- **SNR**: Signal-to-Noise Ratio
- **Latency**: Time delay between signal generation and response

---

## 2. Signal Types and Encoding

### 2.1 Signal Classification

```
Type A: Neural Signals
├── A1: EEG (Scalp, non-invasive)
├── A2: ECoG (Cortical, invasive)
├── A3: LFP (Local Field Potential)
└── A4: Single-unit (Neuron level)

Type B: Muscular Signals
├── B1: Surface EMG
├── B2: Intramuscular EMG
└── B3: MMG (Mechanomyography)

Type C: Biometric Signals
├── C1: ECG (Heart)
├── C2: GSR (Skin conductance)
├── C3: Respiration
└── C4: Temperature

Type D: Motion Signals
├── D1: Accelerometer
├── D2: Gyroscope
├── D3: Magnetometer
└── D4: Pressure/Force

Type E: Feedback Signals
├── E1: Haptic (vibration, force)
├── E2: Thermal
├── E3: Electrical stimulation
└── E4: Visual/Auditory
```

### 2.2 Signal Parameters

| Signal Type | Sample Rate | Resolution | Bandwidth | Channels |
|-------------|-------------|------------|-----------|----------|
| EEG | 250-1000 Hz | 16-24 bit | 0.1-100 Hz | 8-256 |
| ECoG | 1-30 kHz | 16-24 bit | 0.1-500 Hz | 16-128 |
| EMG | 1-10 kHz | 16-24 bit | 10-500 Hz | 1-16 |
| Motion | 50-1000 Hz | 16 bit | 0-200 Hz | 3-9 |

### 2.3 Signal Encoding Format

```typescript
interface SignalPacket {
  header: {
    version: number;           // Protocol version (1-255)
    signalType: SignalType;    // A1-E4
    timestamp: bigint;         // Microseconds since epoch
    channelCount: number;      // Number of channels
    sampleCount: number;       // Samples per channel
    resolution: number;        // Bits per sample
  };
  metadata: {
    deviceId: string;          // Source device UUID
    sessionId: string;         // Current session UUID
    sequenceNumber: number;    // Packet sequence
    quality: number;           // Signal quality 0-100
  };
  payload: {
    samples: Int32Array;       // Interleaved channel data
    markers: EventMarker[];    // Synchronization markers
    checksum: number;          // CRC-32 checksum
  };
}
```

### 2.4 Compression Methods

```
Method 1: Delta Encoding
- For slowly varying signals
- Compression ratio: 2-4x
- Latency overhead: < 1ms

Method 2: Wavelet Compression
- For complex neural signals
- Compression ratio: 5-10x
- Latency overhead: 2-5ms

Method 3: Lossless (LZ4)
- For high-fidelity requirements
- Compression ratio: 1.5-2x
- Latency overhead: < 0.5ms
```

---

## 3. Communication Protocol Stack

### 3.1 Protocol Layers

```
┌─────────────────────────────────────┐
│ Layer 5: Application Layer          │
│ - Intent interpretation             │
│ - Command generation                │
│ - User state management             │
├─────────────────────────────────────┤
│ Layer 4: Session Layer              │
│ - Connection management             │
│ - Calibration state                 │
│ - Session synchronization           │
├─────────────────────────────────────┤
│ Layer 3: Transport Layer            │
│ - Reliable delivery (optional)      │
│ - Flow control                      │
│ - Multiplexing                      │
├─────────────────────────────────────┤
│ Layer 2: Data Link Layer            │
│ - Framing                           │
│ - Error detection/correction        │
│ - Addressing                        │
├─────────────────────────────────────┤
│ Layer 1: Physical Layer             │
│ - Wired: USB, SPI, I2C              │
│ - Wireless: BLE, WiFi, proprietary  │
└─────────────────────────────────────┘
```

### 3.2 Physical Layer Options

| Medium | Bandwidth | Range | Power | Latency | Use Case |
|--------|-----------|-------|-------|---------|----------|
| USB 3.0 | 5 Gbps | 3m | Low | <1ms | External devices |
| SPI | 100 Mbps | 0.3m | Very Low | <0.1ms | Implant-to-hub |
| BLE 5.0 | 2 Mbps | 50m | Very Low | 7.5ms | Wearables |
| WiFi 6 | 9.6 Gbps | 100m | High | 2ms | High-bandwidth |
| Neural Link | 10 Mbps | 0.01m | Ultra Low | <0.5ms | Implant internal |

### 3.3 Data Link Frame Format

```
┌────────┬────────┬──────────┬─────────┬──────────┬────────┐
│ Sync   │ Header │ Address  │ Length  │ Payload  │ CRC    │
│ 2 bytes│ 1 byte │ 4 bytes  │ 2 bytes │ Variable │ 4 bytes│
└────────┴────────┴──────────┴─────────┴──────────┴────────┘
```

### 3.4 Transport Layer Protocol

```typescript
interface TransportPacket {
  sequenceNumber: number;     // 32-bit sequence
  acknowledgmentNumber: number;
  flags: {
    SYN: boolean;             // Connection initiation
    ACK: boolean;             // Acknowledgment
    FIN: boolean;             // Connection termination
    RST: boolean;             // Reset
    PRI: boolean;             // Priority
    REL: boolean;             // Reliable delivery required
  };
  window: number;             // Flow control window
  checksum: number;           // 16-bit checksum
  urgentPointer: number;      // Priority data pointer
  payload: Uint8Array;
}
```

---

## 4. Latency Requirements

### 4.1 Latency Tiers

```
Tier 1: Ultra-Low Latency (< 10ms)
- Motor control applications
- Balance and postural control
- Reflex-like responses

Tier 2: Real-Time (< 50ms)
- Sensory feedback
- Grip force modulation
- Interactive control

Tier 3: Interactive (< 100ms)
- User interface interactions
- Menu navigation
- Non-critical adjustments

Tier 4: Background (< 500ms)
- Status updates
- Analytics data
- Configuration changes
```

### 4.2 Latency Budget Allocation

For Tier 1 (10ms total):
```
Signal Acquisition:     2ms
Preprocessing:          1ms
Encoding/Compression:   1ms
Transmission:           2ms
Decoding:               1ms
Motor Execution:        3ms
────────────────────────────
Total:                 10ms
```

### 4.3 Latency Measurement Protocol

```typescript
interface LatencyMeasurement {
  measurementId: string;
  tier: 1 | 2 | 3 | 4;
  components: {
    acquisition: number;      // ms
    processing: number;       // ms
    encoding: number;         // ms
    transmission: number;     // ms
    decoding: number;         // ms
    execution: number;        // ms
  };
  total: number;              // ms
  jitter: number;             // ms (standard deviation)
  compliance: boolean;        // Met tier requirement?
  timestamp: Date;
}

function measureLatency(tier: number): LatencyMeasurement {
  const startMark = performance.now();
  // ... measurement implementation
  return measurement;
}
```

### 4.4 Latency Optimization Techniques

```
1. Signal Path Optimization
   - Minimize processing stages
   - Use hardware acceleration
   - Parallel processing pipelines

2. Communication Optimization
   - Priority queuing
   - Predictive transmission
   - Zero-copy data paths

3. Algorithm Optimization
   - Fixed-point arithmetic
   - Lookup tables
   - Real-time OS scheduling
```

---

## 5. Bidirectional Communication

### 5.1 Communication Channels

```
Forward Path (Human → Machine):
┌─────────┐    ┌────────────┐    ┌────────────┐    ┌──────────┐
│ Neural  │ → │ Signal     │ → │ Intent     │ → │ Device   │
│ Signal  │    │ Processing │    │ Decoding   │    │ Control  │
└─────────┘    └────────────┘    └────────────┘    └──────────┘

Feedback Path (Machine → Human):
┌──────────┐    ┌────────────┐    ┌────────────┐    ┌─────────┐
│ Device   │ → │ State      │ → │ Feedback   │ → │ Sensory │
│ State    │    │ Encoding   │    │ Generation │    │ Input   │
└──────────┘    └────────────┘    └────────────┘    └─────────┘
```

### 5.2 Intent Decoding

```typescript
interface Intent {
  category: 'motor' | 'sensory' | 'cognitive' | 'communication';
  action: string;             // e.g., 'grip_close', 'walk_forward'
  parameters: {
    intensity: number;        // 0-1
    speed: number;            // 0-1
    precision: number;        // 0-1
    duration?: number;        // ms
    target?: Vector3;         // Spatial target
  };
  confidence: number;         // 0-1
  timestamp: bigint;
  source: string;             // Signal source ID
}

interface IntentDecoder {
  decode(signals: SignalPacket[]): Intent;
  train(examples: TrainingExample[]): void;
  calibrate(session: CalibrationSession): void;
  getConfidenceThreshold(): number;
  setConfidenceThreshold(value: number): void;
}
```

### 5.3 Command Generation

```typescript
interface MotorCommand {
  deviceId: string;
  commandType: 'position' | 'velocity' | 'force' | 'impedance';
  joints: JointCommand[];
  synchronization: {
    timestamp: bigint;
    deadline: bigint;
    priority: number;
  };
}

interface JointCommand {
  jointId: number;
  target: number;             // Position/velocity/force
  limits: {
    min: number;
    max: number;
    maxVelocity: number;
    maxAcceleration: number;
  };
  compliance: number;         // 0 = rigid, 1 = compliant
}
```

### 5.4 State Synchronization

```typescript
interface DeviceState {
  deviceId: string;
  timestamp: bigint;
  power: {
    batteryLevel: number;     // 0-100%
    charging: boolean;
    estimatedRuntime: number; // minutes
  };
  position: {
    joints: number[];         // Current joint angles
    endEffector: Vector3;     // End effector position
  };
  sensors: {
    force: number[];          // Force sensors
    temperature: number[];    // Thermal sensors
    contact: boolean[];       // Contact sensors
  };
  status: 'ready' | 'active' | 'error' | 'calibrating' | 'sleeping';
}
```

---

## 6. Feedback Systems

### 6.1 Feedback Modalities

#### 6.1.1 Haptic Feedback

```typescript
interface HapticFeedback {
  type: 'vibration' | 'force' | 'electrotactile' | 'thermal';
  pattern: HapticPattern;
  location: BodyLocation[];
  intensity: number;          // 0-1
  duration: number;           // ms
  frequency?: number;         // Hz (for vibration)
  waveform?: number[];        // Custom waveform
}

interface HapticPattern {
  name: string;
  segments: {
    intensity: number;
    duration: number;
    rampUp?: number;
    rampDown?: number;
  }[];
  repeatCount: number;
}
```

Standard Haptic Patterns:
```
| Pattern Name     | Duration | Description          |
|-----------------|----------|----------------------|
| SINGLE_PULSE    | 50ms     | Quick notification   |
| DOUBLE_PULSE    | 150ms    | Confirmation         |
| LONG_PULSE      | 200ms    | Warning              |
| BUZZ            | 500ms    | Alert                |
| HEARTBEAT       | 800ms    | Status active        |
| GRIP_RAMP       | 100ms    | Grip force feedback  |
| CONTACT         | 30ms     | Surface contact      |
| SLIP_WARNING    | 200ms    | Object slipping      |
```

#### 6.1.2 Sensory Substitution

```typescript
interface SensorySubstitution {
  inputModality: 'pressure' | 'temperature' | 'proprioception' | 'texture';
  outputModality: 'vibration' | 'electrotactile' | 'visual' | 'audio';
  mapping: {
    inputRange: [number, number];
    outputRange: [number, number];
    curve: 'linear' | 'logarithmic' | 'exponential';
  };
  channels: number;
  resolution: number;
}
```

#### 6.1.3 Neural Feedback

```typescript
interface NeuralFeedback {
  type: 'cortical' | 'peripheral' | 'spinal';
  target: {
    location: string;         // Brain region or nerve
    channels: number[];       // Electrode channels
  };
  stimulation: {
    amplitude: number;        // μA
    pulseWidth: number;       // μs
    frequency: number;        // Hz
    duration: number;         // ms
    waveform: 'biphasic' | 'monophasic' | 'asymmetric';
  };
  safety: {
    maxCharge: number;        // μC per phase
    maxFrequency: number;     // Hz
    dutyCycle: number;        // 0-1
  };
}
```

### 6.2 Feedback Timing

```
Perception Thresholds:
- Haptic detection: 2-5ms
- Haptic discrimination: 10-20ms
- Proprioceptive: 30-50ms
- Thermal: 100-500ms

Recommended Feedback Delays:
- Force feedback: < 30ms
- Position feedback: < 50ms
- Contact notification: < 20ms
- Status updates: < 200ms
```

---

## 7. Calibration Standards

### 7.1 Calibration Types

```
1. Initial Calibration (First use)
   - Baseline signal acquisition
   - Movement pattern learning
   - Feedback sensitivity tuning
   Duration: 30-60 minutes

2. Session Calibration (Each use)
   - Signal quality verification
   - Baseline adjustment
   - Quick pattern confirmation
   Duration: 2-5 minutes

3. Adaptive Calibration (Continuous)
   - Background optimization
   - Drift compensation
   - Performance tracking
   Duration: Continuous

4. Recalibration (Periodic)
   - Full system verification
   - Algorithm retraining
   - Hardware check
   Duration: 15-30 minutes (monthly)
```

### 7.2 Calibration Protocol

```typescript
interface CalibrationSession {
  sessionId: string;
  type: 'initial' | 'session' | 'adaptive' | 'recalibration';
  startTime: Date;
  endTime?: Date;
  status: 'in_progress' | 'completed' | 'failed' | 'aborted';
  tasks: CalibrationTask[];
  results: CalibrationResult;
}

interface CalibrationTask {
  taskId: string;
  name: string;
  description: string;
  duration: number;           // Expected duration in seconds
  instructions: string[];
  requiredSignals: SignalType[];
  repetitions: number;
  restPeriod: number;         // Seconds between repetitions
}

interface CalibrationResult {
  success: boolean;
  accuracy: number;           // 0-100%
  signalQuality: {
    snr: number;              // dB
    stability: number;        // 0-1
    channels: ChannelQuality[];
  };
  model: CalibrationModel;
  recommendations: string[];
}
```

### 7.3 Standard Calibration Tasks

```
Motor Calibration:
1. Rest state (30 seconds)
2. Maximum voluntary contraction
3. Graded force levels (25%, 50%, 75%, 100%)
4. Movement patterns (flexion, extension, rotation)
5. Fine motor tasks (pinch, grasp, point)

Sensory Calibration:
1. Detection threshold
2. Intensity discrimination
3. Localization accuracy
4. Pattern recognition
5. Response timing
```

### 7.4 Quality Metrics

```typescript
interface QualityMetrics {
  signalToNoise: number;      // dB, target > 40
  channelCorrelation: number; // 0-1, target < 0.3
  baseline Stability: number; // Variance, target < 10%
  artifactRate: number;       // Per minute, target < 5
  decodingAccuracy: number;   // 0-100%, target > 90%
  latencyCompliance: number;  // 0-100%, target > 95%
  feedbackAccuracy: number;   // 0-100%, target > 85%
}
```

---

## 8. Interoperability

### 8.1 Device Discovery Protocol

```typescript
interface DeviceAdvertisement {
  deviceId: string;           // UUID
  deviceType: string;         // e.g., 'prosthetic_arm'
  manufacturer: string;
  model: string;
  version: string;
  capabilities: {
    signalTypes: SignalType[];
    feedbackTypes: FeedbackType[];
    latencyTier: 1 | 2 | 3 | 4;
    channels: number;
    sampleRates: number[];
  };
  protocols: {
    hmi: string;              // e.g., 'WIA-AUG-014/1.0'
    security: string;         // e.g., 'WIA-SEC/1.0'
    safety: string;           // e.g., 'WIA-AUG-013/1.0'
  };
  status: 'available' | 'connected' | 'busy' | 'error';
}
```

### 8.2 Connection Handshake

```
Client                          Device
  │                               │
  ├──── DISCOVER ────────────────►│
  │◄─── ADVERTISE ────────────────┤
  │                               │
  ├──── CONNECT_REQUEST ─────────►│
  │◄─── CONNECT_ACCEPT ───────────┤
  │                               │
  ├──── CAPABILITY_QUERY ────────►│
  │◄─── CAPABILITY_RESPONSE ──────┤
  │                               │
  ├──── CALIBRATION_REQUEST ─────►│
  │◄─── CALIBRATION_DATA ─────────┤
  │                               │
  ├──── SESSION_START ───────────►│
  │◄─── SESSION_ACK ──────────────┤
  │                               │
  │         (Active Session)       │
  │                               │
  ├──── SESSION_END ─────────────►│
  │◄─── DISCONNECT_ACK ───────────┤
```

### 8.3 Cross-Device Coordination

```typescript
interface DeviceCoordination {
  coordinatorId: string;      // Master device
  participants: string[];     // Device IDs
  synchronization: {
    method: 'timestamp' | 'trigger' | 'sequence';
    precision: number;        // Microseconds
    maxSkew: number;          // Maximum allowed skew
  };
  commands: {
    broadcast: boolean;       // Send to all
    sequential: boolean;      // Ordered execution
    timeout: number;          // ms
  };
}
```

### 8.4 Profile Exchange

```typescript
interface UserProfile {
  profileId: string;
  userId: string;
  created: Date;
  updated: Date;
  calibration: {
    signalBaselines: Map<string, number[]>;
    decoderModels: Map<string, DecoderModel>;
    feedbackPreferences: FeedbackPreferences;
  };
  preferences: {
    latencyPriority: 'low' | 'balanced' | 'quality';
    feedbackIntensity: number;  // 0-1
    adaptiveMode: boolean;
  };
  history: {
    devices: string[];
    sessions: number;
    totalUsageHours: number;
  };
}
```

---

## 9. Implementation Guidelines

### 9.1 Certification Requirements

To achieve WIA-AUG-014 certification:

```
1. Protocol Compliance
   - Implement full protocol stack
   - Pass conformance tests
   - Interoperability verification

2. Latency Compliance
   - Meet tier requirements
   - Jitter within specifications
   - Document latency budget

3. Signal Quality
   - SNR requirements met
   - Artifact handling
   - Error correction

4. Feedback System
   - Supported modalities
   - Timing requirements
   - Safety limits

5. Calibration
   - Standard tasks implemented
   - Quality metrics tracked
   - Profile portability

6. Security
   - Encryption support
   - Authentication
   - Secure pairing
```

### 9.2 Minimum Implementation

```typescript
interface MinimumImplementation {
  required: {
    protocolVersion: string;
    signalTypes: SignalType[];     // At least one
    latencyTier: number;           // Tier 3 minimum
    calibration: 'session';
    encryption: boolean;           // Required
  };
  optional: {
    feedbackTypes: FeedbackType[];
    adaptiveCalibration: boolean;
    profilePortability: boolean;
    multiDeviceCoordination: boolean;
  };
}
```

### 9.3 API Interface

```typescript
// Core SDK Interface
interface HumanMachineInterfaceSDK {
  // Connection
  discover(): Promise<DeviceAdvertisement[]>;
  connect(deviceId: string): Promise<Connection>;
  disconnect(): Promise<void>;

  // Signals
  startAcquisition(config: AcquisitionConfig): void;
  stopAcquisition(): void;
  onSignal(callback: (signal: SignalPacket) => void): void;

  // Commands
  sendCommand(command: MotorCommand): Promise<CommandResult>;
  sendFeedback(feedback: HapticFeedback): Promise<void>;

  // Calibration
  startCalibration(type: CalibrationType): Promise<CalibrationSession>;
  loadProfile(profileId: string): Promise<void>;
  saveProfile(): Promise<string>;

  // State
  getDeviceState(): DeviceState;
  getMetrics(): QualityMetrics;
}
```

---

## 10. References

### 10.1 Scientific Literature

1. 선행 연구. Brain-Computer Interfaces: Principles and Practice
2. 선행 연구. Restoring Natural Sensory Feedback in Real-Time Bidirectional Hand Prostheses
3. Bensmaia, S.J. & Miller, L.E. (2014). Restoring Sensorimotor Function Through Intracortical Interfaces

### 10.2 Standards

- IEEE 2413-2019: Standard for an Architectural Framework for the IoT
- ISO 11073: Health informatics — Medical / health device communication
- IEC 62443: Industrial communication networks — IT security

### 10.3 WIA Standards

- WIA-AUG-013: Augmentation Safety
- WIA-AUG-007: Bionic Limb Standards
- WIA-BCI: Brain-Computer Interface
- WIA-HAPTIC: Haptic Feedback Standards
- WIA-SEC: Security Standards

---

## Appendix A: Signal Type Definitions

```typescript
enum SignalType {
  // Neural (Type A)
  EEG = 'A1',
  ECoG = 'A2',
  LFP = 'A3',
  SINGLE_UNIT = 'A4',

  // Muscular (Type B)
  SURFACE_EMG = 'B1',
  INTRAMUSCULAR_EMG = 'B2',
  MMG = 'B3',

  // Biometric (Type C)
  ECG = 'C1',
  GSR = 'C2',
  RESPIRATION = 'C3',
  TEMPERATURE = 'C4',

  // Motion (Type D)
  ACCELEROMETER = 'D1',
  GYROSCOPE = 'D2',
  MAGNETOMETER = 'D3',
  FORCE = 'D4',

  // Feedback (Type E)
  HAPTIC = 'E1',
  THERMAL = 'E2',
  ELECTRICAL_STIM = 'E3',
  VISUAL_AUDIO = 'E4'
}
```

## Appendix B: Error Codes

```
| Code | Category | Description |
|------|----------|-------------|
| H001 | Connection | Device not found |
| H002 | Connection | Connection timeout |
| H003 | Connection | Authentication failed |
| H004 | Signal | Low signal quality |
| H005 | Signal | Channel failure |
| H006 | Signal | Buffer overflow |
| H007 | Latency | Tier requirement not met |
| H008 | Latency | Excessive jitter |
| H009 | Calibration | Calibration required |
| H010 | Calibration | Calibration failed |
| H011 | Command | Invalid command |
| H012 | Command | Execution timeout |
| H013 | Feedback | Feedback delivery failed |
| H014 | Safety | Safety limit exceeded |
| H015 | Battery | Low battery |
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUG-014 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
