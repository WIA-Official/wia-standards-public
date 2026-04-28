# WIA-AUTO-009: Vehicle Semiconductor Specification v1.0

> **Standard ID:** WIA-AUTO-009
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Semiconductor Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Automotive Grade Classifications](#2-automotive-grade-classifications)
3. [MCU and SoC Architecture](#3-mcu-and-soc-architecture)
4. [Power Management ICs](#4-power-management-ics)
5. [Sensor ICs](#5-sensor-ics)
6. [AI/ML Accelerators](#6-aiml-accelerators)
7. [Functional Safety (ISO 26262)](#7-functional-safety-iso-26262)
8. [Qualification and Testing](#8-qualification-and-testing)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the requirements, classifications, and testing procedures for automotive-grade semiconductor components. It ensures that all semiconductors used in vehicles meet the stringent safety, reliability, and performance standards required for automotive applications.

### 1.2 Scope

The standard covers:
- Automotive grade qualification (AEC-Q100/Q101/Q200)
- Functional safety requirements (ISO 26262)
- Temperature and voltage specifications
- Reliability and lifetime requirements
- Testing and validation procedures
- Integration with vehicle systems

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard ensures the highest quality semiconductor components in vehicles, protecting human life and enabling the advancement of safer, cleaner, and more intelligent transportation systems.

### 1.4 Terminology

- **AEC**: Automotive Electronics Council
- **ASIL**: Automotive Safety Integrity Level (QM, A, B, C, D)
- **FIT**: Failures In Time (failures per billion device hours)
- **MTBF**: Mean Time Between Failures
- **Junction Temperature (Tj)**: Maximum operating temperature of semiconductor die
- **ESD**: Electrostatic Discharge
- **EMC**: Electromagnetic Compatibility

---

## 2. Automotive Grade Classifications

### 2.1 AEC-Q100: Failure Mechanism Based Stress Test Qualification

AEC-Q100 is the automotive qualification standard for integrated circuits.

#### 2.1.1 Temperature Grades

| Grade | Temperature Range | Typical Application |
|-------|------------------|---------------------|
| Grade 0 | -40°C to +150°C | Engine compartment, exhaust systems |
| Grade 1 | -40°C to +125°C | Under hood, near engine |
| Grade 2 | -40°C to +105°C | Passenger cabin, dashboard |
| Grade 3 | -40°C to +85°C | Mild environment, trunk |

#### 2.1.2 Required Tests

1. **Preconditioning** (if applicable)
   - Moisture Sensitivity Level (MSL)
   - Baking and storage conditions

2. **Accelerated Environmental Stress Tests**
   - Temperature Cycling (TC): -55°C to +150°C
   - High Temperature Operating Life (HTOL): 1000 hours @ Tj max
   - High Temperature Storage Life (HTSL): 1000 hours
   - Temperature Humidity Bias (THB): 85°C/85%RH

3. **Accelerated Lifetime Simulation**
   - Power Temperature Cycling (PTC)
   - Highly Accelerated Stress Test (HAST)
   - Autoclave or Unbiased HAST (uHAST)

4. **Package Assembly Integrity**
   - Wire Bond Shear
   - Die Shear
   - Solderability
   - Physical Dimensions

5. **Electrical Verification**
   - Electrical Distribution (ESD)
   - Latchup
   - Electrostatic Discharge (ESD): ±2kV HBM minimum

### 2.2 AEC-Q101: Discrete Semiconductor Devices

Qualification for power transistors, diodes, and other discrete components.

**Key Requirements**:
- High-voltage capability (up to 1200V for EVs)
- Low on-resistance for efficiency
- Fast switching for reduced losses
- Avalanche and surge current ratings

### 2.3 AEC-Q200: Passive Components

Standards for resistors, capacitors, and inductors.

**Requirements**:
- High temperature stability
- Low ESR/ESL for power applications
- Vibration and mechanical stress resistance

### 2.4 Reliability Requirements

```
Target FIT Rate: <10 FIT @ 55°C ambient
Qualification Lifetime: 15-20 years
Operating Hours: >200,000 hours
Failure Rate: <0.001% per year
```

---

## 3. MCU and SoC Architecture

### 3.1 Microcontroller Units (MCU)

#### 3.1.1 Core Architecture

```
Processor Cores:
- ARM Cortex-M (M4, M7) for real-time control
- ARM Cortex-R for safety-critical applications
- Lockstep cores for ASIL-D safety

Clock Frequencies:
- Up to 400 MHz for high-performance applications
- Multiple clock domains for power efficiency
```

#### 3.1.2 Memory Configuration

```
Flash Memory: 512KB to 8MB
- ECC protection for ASIL-C/D
- Dual-bank for read-while-write
- Secure boot storage

RAM: 64KB to 1MB
- SRAM with ECC
- Data retention down to -40°C
- Low-power retention modes

EEPROM/FRAM: 4KB to 64KB
- Non-volatile data storage
- 1M+ write cycles
- Data retention: 20+ years @ 85°C
```

#### 3.1.3 Communication Interfaces

```
CAN/CAN-FD:
- Classic CAN: up to 1 Mbps
- CAN-FD: up to 8 Mbps data phase
- Multiple CAN channels (2-8)

LIN (Local Interconnect Network):
- 20 kbps max
- Cost-effective for body electronics

FlexRay:
- 10 Mbps
- Time-triggered, deterministic
- Used in safety-critical systems

Ethernet:
- 100Base-T1 / 1000Base-T1
- AVB/TSN for real-time
- SOME/IP protocol support
```

#### 3.1.4 Safety Features

```
- Lockstep CPU cores (ASIL-D)
- Memory ECC (single/double error correction)
- CRC units for data integrity
- Watchdog timers (independent and window)
- Clock monitoring (frequency and integrity)
- Voltage monitoring (POR, LVD, HVD)
- Temperature sensors
- Safe state management
```

### 3.2 System on Chip (SoC)

#### 3.2.1 High-Performance Computing

```
Application Processors:
- ARM Cortex-A53/A72/A78
- Quad to Octa-core configurations
- Up to 2.5 GHz operating frequency
- NEON SIMD for signal processing

GPU (Graphics Processing Unit):
- OpenGL ES 3.2, Vulkan support
- Multiple display outputs
- 3D graphics for HMI

NPU (Neural Processing Unit):
- AI inference: 10-100 TOPS
- CNN, RNN, transformer support
- INT8/INT16 quantization
- On-chip memory for model caching
```

#### 3.2.2 Advanced Connectivity

```
High-Speed Interfaces:
- PCIe Gen 3/4: for external GPUs, storage
- USB 3.1/3.2: host and device mode
- MIPI CSI-2: camera inputs (4-16 lanes)
- MIPI DSI: display outputs
- Gigabit Ethernet: 2.5/5/10 Gbps

Wireless:
- WiFi 6/6E integration
- Bluetooth 5.2/5.3
- 5G modem (V2X communication)
```

#### 3.2.3 Security Features

```
Cryptographic Accelerators:
- AES-128/256, RSA-2048/4096
- SHA-256/384/512
- ECC (Elliptic Curve Cryptography)
- True Random Number Generator (TRNG)

Security Modules:
- Secure boot with root of trust
- TrustZone for ARM processors
- Hardware Security Module (HSM)
- Key storage and management
- Tamper detection
```

---

## 4. Power Management ICs

### 4.1 Voltage Regulators

#### 4.1.1 Buck Converters (Step-Down)

```
Input Voltage: 4.5V to 40V
Output Voltage: 0.6V to 16V
Output Current: 1A to 20A
Efficiency: >95% @ nominal load
Switching Frequency: 200kHz to 2MHz
```

**Features**:
- Synchronous rectification
- Current mode control
- Spread spectrum for EMI reduction
- Power good indicator
- Enable control

#### 4.1.2 Boost Converters (Step-Up)

```
Input Voltage: 2.5V to 16V
Output Voltage: 5V to 40V
Output Current: 0.5A to 5A
Applications: LED drivers, USB charging
```

#### 4.1.3 LDO (Low Dropout Regulators)

```
Dropout Voltage: 100mV to 500mV
Output Current: 50mA to 3A
PSRR: >60dB @ 1kHz
Quiescent Current: <50µA
Applications: Noise-sensitive circuits
```

### 4.2 Battery Management Systems (BMS)

#### 4.2.1 Cell Monitoring

```
Voltage Measurement:
- Accuracy: ±2mV
- Range: 1.5V to 5.0V per cell
- Channels: 6 to 18 cells

Current Measurement:
- Shunt-based: ±100A range
- Accuracy: ±0.5%
- Coulomb counting for SoC

Temperature Monitoring:
- NTC thermistor inputs
- 4 to 16 channels
- -40°C to +125°C range
```

#### 4.2.2 Protection Features

```
Overvoltage Protection (OVP)
Undervoltage Protection (UVP)
Overcurrent Protection (OCP)
Short Circuit Protection (SCP)
Overtemperature Protection (OTP)
Cell Balancing (passive/active)
```

### 4.3 Gate Drivers

For MOSFET/IGBT switching in power converters:

```
Output Current: ±2A to ±10A
Rise/Fall Time: <20ns
Propagation Delay: <50ns
Dead-time Control: 50ns to 1µs
Isolation: 2.5kV to 5kV (for high-voltage)
```

---

## 5. Sensor ICs

### 5.1 Current Sense Amplifiers

```
Measurement Range: -200A to +200A
Accuracy: ±0.5% to ±1%
Bandwidth: 100kHz to 1MHz
Common-Mode Voltage: -16V to +80V
Applications: Motor control, battery monitoring
```

### 5.2 Temperature Sensors

```
Types:
- Analog output (10mV/°C)
- Digital output (I²C, SPI)

Accuracy: ±0.5°C to ±2°C
Range: -40°C to +150°C
Resolution: 0.0625°C to 0.25°C
Response Time: <5 seconds
```

### 5.3 Pressure Sensors

```
MEMS-based:
- Manifold Absolute Pressure (MAP)
- Tire Pressure Monitoring (TPMS)
- Brake Pressure

Range: 20kPa to 10MPa
Accuracy: ±1% to ±2.5% FSS
Output: Analog or digital
Temperature Compensation: -40°C to +125°C
```

### 5.4 Position Sensors

```
Hall Effect Sensors:
- Linear and switch types
- Magnetic field sensitivity: 50-200 Gauss
- Applications: Throttle, pedal position

Inductive Sensors:
- Non-contact measurement
- Crankshaft/camshaft position
- Wheel speed sensing

Resolver-to-Digital Converters:
- 10-16 bit resolution
- Accuracy: ±5 to ±15 arcmin
- Motor control in EVs
```

### 5.5 IMU (Inertial Measurement Units)

```
Accelerometer:
- Range: ±2g to ±16g
- Resolution: 16-bit
- Noise: <100µg/√Hz

Gyroscope:
- Range: ±125°/s to ±2000°/s
- Resolution: 16-bit
- Bias stability: <10°/h

Applications:
- Electronic Stability Control (ESC)
- Rollover detection
- Autonomous navigation
```

---

## 6. AI/ML Accelerators

### 6.1 Neural Processing Units (NPU)

#### 6.1.1 Performance Specifications

```
Compute Performance:
- INT8: 10 to 100 TOPS
- INT16/FP16: 5 to 50 TOPS
- Batch processing support

Architecture:
- Systolic array
- SIMD vector units
- Dedicated MAC (Multiply-Accumulate) units

Memory:
- On-chip SRAM: 1MB to 10MB
- DMA for efficient data transfer
- Model compression support
```

#### 6.1.2 Supported Operations

```
Convolutional Neural Networks (CNN):
- 2D/3D convolutions
- Depthwise separable convolutions
- Pooling (max, average)

Activation Functions:
- ReLU, Leaky ReLU, PReLU
- Sigmoid, Tanh
- Softmax

Other Operations:
- Fully connected layers
- Normalization (batch, layer)
- Attention mechanisms
```

#### 6.1.3 Model Support

```
Frameworks:
- TensorFlow Lite
- ONNX
- PyTorch Mobile
- TensorRT

Quantization:
- INT8 symmetric/asymmetric
- Mixed precision (INT8/INT16)
- Dynamic range optimization

Model Size:
- Up to 100M parameters
- Compressed models: 10-50MB
```

### 6.2 Vision Processing

```
Image Signal Processor (ISP):
- 12MP to 20MP camera support
- HDR, WDR processing
- Debayering, denoise
- Lens distortion correction

Computer Vision:
- Object detection (YOLO, SSD)
- Lane detection
- Semantic segmentation
- Optical flow
```

### 6.3 Automotive AI Applications

1. **Autonomous Driving**:
   - Sensor fusion (camera, radar, lidar)
   - Path planning and prediction
   - Decision making

2. **ADAS (Advanced Driver Assistance)**:
   - Adaptive Cruise Control (ACC)
   - Lane Keeping Assist (LKA)
   - Automatic Emergency Braking (AEB)
   - Blind Spot Detection (BSD)

3. **Driver Monitoring**:
   - Drowsiness detection
   - Distraction warning
   - Gaze tracking

4. **Predictive Maintenance**:
   - Anomaly detection
   - Component lifetime prediction

---

## 7. Functional Safety (ISO 26262)

### 7.1 ASIL Classification

ISO 26262 defines Automotive Safety Integrity Levels:

```
ASIL D: Highest risk of injury or death
- Applications: Steering, braking, airbag
- Failure rate target: <10 FIT

ASIL C: High risk
- Applications: Antilock braking, traction control
- Failure rate target: <100 FIT

ASIL B: Medium risk
- Applications: Headlights, wipers
- Failure rate target: <1000 FIT

ASIL A: Low risk
- Applications: Rear lights, horn
- Failure rate target: <10000 FIT

QM: Quality Management (non-safety)
- Applications: Infotainment, comfort features
```

### 7.2 Safety Mechanisms

#### 7.2.1 Hardware Safety Mechanisms

```
Redundancy:
- Dual CPU cores in lockstep
- Dual power supplies
- Redundant sensors

Error Detection:
- Memory ECC (single/double bit)
- CRC on data buses
- Parity checking
- Watchdog timers

Fault Handling:
- Graceful degradation
- Safe state transition
- Error logging and reporting
```

#### 7.2.2 Diagnostic Coverage

```
Diagnostic Coverage (DC):
- ASIL D: DC ≥ 99%
- ASIL C: DC ≥ 97%
- ASIL B: DC ≥ 90%
- ASIL A: DC ≥ 60%

Diagnostic Test Interval:
- Power-on self-test (POST)
- Periodic background tests
- On-demand diagnostics
```

### 7.3 Safety Metrics

#### 7.3.1 PMHF (Probabilistic Metric for Hardware Failures)

```
Target:
- ASIL D: <10 FIT
- ASIL C: <100 FIT
- ASIL B: <100 FIT

Calculation includes:
- Single-point faults
- Residual faults
- Dual-point faults
```

#### 7.3.2 LFM (Latent Fault Metric)

```
Target:
- ASIL D: <1%
- ASIL C: <10%

Detected by:
- Periodic self-tests
- External monitoring
```

### 7.4 Safety Development Process

```
1. Hazard Analysis and Risk Assessment (HARA)
2. Safety Goals definition
3. Functional Safety Concept
4. Technical Safety Concept
5. Hardware/Software Safety Requirements
6. Design and Implementation
7. Verification and Validation
8. Functional Safety Assessment
9. Production Release
10. Operation and Maintenance
```

---

## 8. Qualification and Testing

### 8.1 Environmental Testing

#### 8.1.1 Temperature Testing

```
Temperature Cycling (TC):
- Condition A: -55°C to +150°C
- Condition B: -40°C to +125°C
- Dwell time: 10-15 minutes
- Cycles: 1000
- Transfer time: <1 minute
```

#### 8.1.2 High Temperature Operating Life (HTOL)

```
Duration: 1000 hours minimum
Temperature: Tj max (125°C or 150°C)
Voltage: VDD max
Sample Size: 231 units (77 per lot)
Acceptance: 0 failures
```

#### 8.1.3 Temperature Humidity Bias (THB)

```
Temperature: 85°C
Humidity: 85% RH
Voltage: VDD max
Duration: 1000 hours
Purpose: Detect moisture-related failures
```

### 8.2 Electrical Testing

#### 8.2.1 Electrostatic Discharge (ESD)

```
Human Body Model (HBM):
- Minimum: ±2kV
- Typical: ±4kV to ±8kV
- Critical pins: ±8kV

Charged Device Model (CDM):
- Minimum: ±500V
- Typical: ±1000V to ±2000V

Machine Model (MM):
- Minimum: ±200V
```

#### 8.2.2 Latchup Testing

```
Current: ±100mA minimum
Temperature: 125°C
Overvoltage: VDD + 2V
Purpose: Ensure no SCR triggering
```

#### 8.2.3 Electrical Overstress (EOS)

```
Overvoltage on supply pins
Reverse polarity protection
Load dump (ISO 7637-2): +100V pulse
Cold crank: down to 4.5V
```

### 8.3 Mechanical Testing

#### 8.3.1 Vibration Testing

```
Frequency Range: 10Hz to 2000Hz
Acceleration: 20g to 50g peak
Duration: 12 hours per axis
Standard: IEC 60068-2-64
```

#### 8.3.2 Mechanical Shock

```
Half-sine pulse: 1500g, 0.5ms
Multiple orientations
Standard: IEC 60068-2-27
```

### 8.4 Reliability Metrics

#### 8.4.1 FIT Rate Calculation

```
FIT = (failures / device-hours) × 10⁹

Acceleration Factor (AF):
AF = exp[Ea/k × (1/T₁ - 1/T₂)]

Where:
- Ea: Activation energy (0.7 eV typical)
- k: Boltzmann constant (8.617×10⁻⁵ eV/K)
- T₁: Use temperature (K)
- T₂: Test temperature (K)
```

#### 8.4.2 MTBF Calculation

```
MTBF = 10⁹ / FIT

Example:
FIT = 10 @ 55°C
MTBF = 10⁹ / 10 = 100 million hours
MTBF = 11,415 years
```

---

## 9. Data Formats

### 9.1 Component Specification

```json
{
  "partNumber": "WIA-MCU-ASILD-001",
  "type": "MCU",
  "manufacturer": "WIA Semiconductor",
  "description": "Automotive MCU with ASIL-D safety",
  "specifications": {
    "core": "ARM Cortex-M7",
    "frequency": 400,
    "flash": 4096,
    "ram": 512,
    "package": "LQFP176",
    "temperatureGrade": 1,
    "temperatureRange": {
      "min": -40,
      "max": 125,
      "unit": "celsius"
    },
    "voltage": {
      "nominal": 5.0,
      "min": 4.5,
      "max": 5.5,
      "unit": "V"
    },
    "asilLevel": "ASIL-D",
    "aecQualification": "AEC-Q100 Grade 1"
  },
  "interfaces": [
    "CAN-FD",
    "LIN",
    "FlexRay",
    "SPI",
    "I2C",
    "UART"
  ],
  "safetyFeatures": [
    "Lockstep CPU cores",
    "Memory ECC",
    "Watchdog timer",
    "Voltage monitoring",
    "Temperature monitoring"
  ],
  "reliability": {
    "fitRate": 8,
    "mtbf": 125000000,
    "expectedLifetime": 20,
    "unit": "years"
  }
}
```

### 9.2 Test Results

```json
{
  "componentId": "WIA-MCU-ASILD-001",
  "testDate": "2025-12-26",
  "lotNumber": "LOT2025001",
  "testResults": {
    "temperatureCycling": {
      "status": "PASS",
      "cycles": 1000,
      "failures": 0
    },
    "htol": {
      "status": "PASS",
      "duration": 1000,
      "temperature": 150,
      "failures": 0
    },
    "thb": {
      "status": "PASS",
      "duration": 1000,
      "failures": 0
    },
    "esd": {
      "status": "PASS",
      "hbm": 8000,
      "cdm": 2000
    },
    "latchup": {
      "status": "PASS",
      "temperature": 125
    },
    "vibration": {
      "status": "PASS",
      "duration": 12,
      "acceleration": 30
    }
  },
  "certification": {
    "aecQ100": true,
    "iso26262": "ASIL-D",
    "certificationDate": "2025-12-26",
    "expiryDate": "2030-12-26"
  }
}
```

---

## 10. API Interface

### 10.1 Component Classification

```typescript
interface ComponentClassification {
  partNumber: string;
  type: SemiconductorType;
  temperatureGrade: 0 | 1 | 2 | 3;
  voltageClass: string;
  asilLevel: 'QM' | 'ASIL-A' | 'ASIL-B' | 'ASIL-C' | 'ASIL-D';
  application: string;
}

interface ClassificationResult {
  isAutomotiveGrade: boolean;
  aecQualification: string;
  recommendedApplications: string[];
  safetyRating: string;
  reliabilityMetrics: ReliabilityMetrics;
}
```

### 10.2 AEC-Q100 Validation

```typescript
interface AECQ100Validation {
  componentId: string;
  testResults: {
    thermalCycling: 'PASS' | 'FAIL' | 'PENDING';
    htol: 'PASS' | 'FAIL' | 'PENDING';
    htsl: 'PASS' | 'FAIL' | 'PENDING';
    thb: 'PASS' | 'FAIL' | 'PENDING';
    eMSL: 'PASS' | 'FAIL' | 'PENDING';
    electricalDisturbance: 'PASS' | 'FAIL' | 'PENDING';
    esd: 'PASS' | 'FAIL' | 'PENDING';
    latchup: 'PASS' | 'FAIL' | 'PENDING';
  };
}

interface ValidationResult {
  isCompliant: boolean;
  certificationLevel: string;
  failedTests: string[];
  recommendations: string[];
  expiryDate: Date;
}
```

### 10.3 Reliability Calculation

```typescript
interface ReliabilityParams {
  fitRate: number;           // Failures in time @ reference temp
  operatingTemp: number;      // Celsius
  referenceTemp: number;      // Celsius (typically 55°C)
  activationEnergy: number;   // eV (typically 0.7)
}

interface ReliabilityMetrics {
  fitRate: number;            // Adjusted for operating temp
  mtbf: number;               // Hours
  failureRate: number;        // Per year
  expectedLifetime: number;   // Years
  confidence: number;         // Percentage
}
```

### 10.4 Safety Assessment

```typescript
interface SafetyAssessment {
  asilRequired: 'QM' | 'ASIL-A' | 'ASIL-B' | 'ASIL-C' | 'ASIL-D';
  application: string;
  componentSpecs: ComponentSpecification;
}

interface SafetyResult {
  isCompliant: boolean;
  asilAchieved: string;
  safetyMechanisms: string[];
  diagnosticCoverage: number;  // Percentage
  pmhf: number;                // FIT
  lfm: number;                 // Percentage
  recommendations: string[];
}
```

---

## 11. References

### 11.1 Standards and Specifications

1. **AEC-Q100**: Failure Mechanism Based Stress Test Qualification for Integrated Circuits
2. **AEC-Q101**: Failure Mechanism Based Stress Test Qualification for Discrete Semiconductors
3. **AEC-Q200**: Stress Test Qualification for Passive Components
4. **ISO 26262**: Road Vehicles - Functional Safety
5. **IEC 61000-4-2**: EMC - Electrostatic Discharge Immunity Test
6. **CISPR 25**: Vehicles, Boats and Internal Combustion Engines - Radio Disturbance Characteristics
7. **ISO 7637-2**: Road Vehicles - Electrical Disturbances by Conduction and Coupling
8. **SAE J1211**: Handbook for Robustness Validation of Automotive Electrical/Electronic Modules

### 11.2 Semiconductor Types

| Type | Description | Common Applications |
|------|-------------|-------------------|
| MCU | Microcontroller Unit | Body control, powertrain |
| SoC | System on Chip | Infotainment, ADAS |
| PMIC | Power Management IC | Voltage regulation |
| MOSFET | Power Transistor | Motor control, DC-DC |
| IGBT | Insulated Gate Bipolar Transistor | EV inverters, high voltage |
| CAN Transceiver | Communication IC | Vehicle networks |
| Gate Driver | MOSFET/IGBT Driver | Power switching |
| Sensor IC | Sensor Interface | Current, temperature, pressure |
| OpAmp | Operational Amplifier | Signal conditioning |
| ADC/DAC | Analog-Digital Converter | Data acquisition |

### 11.3 WIA Standards

- **WIA-INTENT**: Intent-based vehicle control interfaces
- **WIA-OMNI-API**: Universal automotive API gateway
- **WIA-EV**: Electric vehicle standards
- **WIA-SOCIAL**: Connected vehicle communication

---

## Appendix A: Temperature Grade Selection Guide

### A.1 Decision Matrix

| Location | Ambient Temp | Heat Source | Recommended Grade |
|----------|--------------|-------------|-------------------|
| Engine bay | -40 to +125°C | Engine heat | Grade 0 or 1 |
| Under hood | -40 to +105°C | Moderate | Grade 1 |
| Cabin | -40 to +85°C | HVAC controlled | Grade 2 or 3 |
| Trunk | -40 to +70°C | Minimal | Grade 3 |
| Battery pack | -40 to +60°C | Cell heating | Grade 2 or 3 |

### A.2 Thermal Management

```
Junction Temperature Calculation:
Tj = Ta + (Pd × θja)

Where:
- Tj: Junction temperature (°C)
- Ta: Ambient temperature (°C)
- Pd: Power dissipation (W)
- θja: Thermal resistance junction-to-ambient (°C/W)

Example:
Ta = 105°C (under hood)
Pd = 2W
θja = 40°C/W

Tj = 105 + (2 × 40) = 185°C

If Tj max = 150°C, cooling required!
```

---

## Appendix B: ASIL Determination Example

### B.1 Hazard Analysis

```
Function: Electronic Brake System
Hazard: Loss of braking capability
Severity (S): S3 (Life-threatening)
Exposure (E): E4 (High probability)
Controllability (C): C3 (Difficult to control)

ASIL = f(S, E, C) = ASIL D

Therefore:
- MCU must be ASIL-D qualified
- Dual-redundant architecture required
- Diagnostic coverage ≥99%
- PMHF <10 FIT
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-009 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
