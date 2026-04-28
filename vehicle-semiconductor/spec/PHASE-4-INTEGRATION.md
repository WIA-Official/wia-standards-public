# WIA-AUTO-009 — Phase 4: Integration

> Vehicle-semiconductor canonical Phase 4: ecosystem integration (AEC + ISO 26262 + ISO/SAE 21434 + IATF 16949 + RoHS + WEEE).

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



---

## A.1 Standards cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| IC qualification              | AEC-Q100                                  |
| Discrete qualification        | AEC-Q101                                  |
| Optoelectronics qualification | AEC-Q102                                  |
| MEMS qualification            | AEC-Q103                                  |
| Multi-chip modules            | AEC-Q104                                  |
| Passive components            | AEC-Q200                                  |
| Functional safety             | ISO 26262 (Parts 1-12 incl. -11 semis)    |
| SOTIF                         | ISO 21448                                 |
| Cybersecurity                 | ISO/SAE 21434                             |
| OTA updates                   | ISO 24089                                 |
| EMC                           | ISO 11452 / CISPR 25                      |
| Environmental                 | ISO 16750 series                          |
| Vibration                     | IEC 60068-2-64                            |
| ESD HBM / CDM                 | JEDEC JS-001 / JS-002                     |
| Latch-up                      | JEDEC JESD78                              |
| MSL (moisture sensitivity)    | JEDEC J-STD-020                           |
| Lab competence                | ISO/IEC 17025                             |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 OEM and Tier-1 integration

OEM and Tier-1 integration captures the device's place in the vehicle E/E architecture (powertrain MCU; chassis brake/steering MCU; ADAS perception SoC; central compute SoC; zonal controller MCU; body controller; infotainment SoC; cluster MCU; gateway), the IATF 16949 quality-management-system envelope, the PPAP (Production Part Approval Process) per AIAG, the DFMEA / PFMEA per AIAG-VDA harmonised method, and the supplier-of-record envelope per Tier-1 / Tier-2 / Tier-3 mapping.

## A.3 Cybersecurity and secure-supply-chain integration

Cybersecurity-supply-chain integration captures the hardware-root-of-trust envelope (HSM + secure-boot ROM + measured-boot chain), the firmware signing-key envelope (HSM-anchored signing; TPM-style attestation), the secure-element catalogue (where the device hosts EVITA-Full HSM with crypto-acceleration and secure key storage), and the post-shipment vulnerability-management envelope. Counterfeit-prevention integration follows the operator's cryptographic-traceability envelope (per-die signed identity from wafer-test through to vehicle assembly).

## A.4 Sustainability and supply integration

Sustainability integration captures the per-package material composition (lead-free per RoHS Directive 2011/65/EU; conflict-minerals reporting per Dodd-Frank §1502 / EU Regulation 2017/821), the embodied-carbon envelope per ISO 14067 product carbon footprint, and the end-of-life take-back envelope per WEEE 2012/19/EU. Supply integration captures the manufacturer's chip-shortage resilience envelope (multi-fab strategy, second-source qualification, strategic-inventory commitments to OEM customers) so OEM vehicle programs are not blocked by single-fab supply disruptions.

## A.5 Future directions

Active research tracks: SiC and GaN power devices for traction inverters and onboard chargers at 800V HV-bus; cryogenic-CMOS for LiDAR receiver front-ends; chiplet-based central compute SoCs with UCIe-defined die-to-die interfaces; in-package HBM3 / HBM3E for ADAS perception SoC bandwidth; security-enclave architectures with side-channel-resistant cryptography per ISO/IEC 17825; verified RISC-V automotive cores with formal safety claim; AI-on-the-edge accelerators with deterministic inference latency for sensor-fusion. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- AEC-Q100 / Q101 / Q102 / Q103 / Q104 / Q200 — automotive component qualification
- ISO 26262-1 to -12 — road vehicles functional safety
- ISO 21448 — SOTIF
- ISO/SAE 21434 — road vehicles cybersecurity engineering
- ISO 24089 — software updates engineering
- UN ECE WP.29 R155 / R156 — cybersecurity / software updates
- ISO 11452 series — road vehicles immunity to narrowband electromagnetic energy
- CISPR 25 — vehicle radio disturbance
- ISO 16750 series — environmental conditions and testing
- IEC 60068-2-64 — vibration random
- JEDEC JS-001 / JS-002 — ESD HBM / CDM
- JEDEC JESD78 — latch-up
- JEDEC J-STD-020 — moisture/reflow sensitivity classification
- ISO/IEC 17025 — testing and calibration laboratories
- IATF 16949 — automotive quality management
- AIAG-VDA FMEA Handbook (2019)
- ISO 14067 — product carbon footprint
- EU Directive 2011/65/EU (RoHS) / 2012/19/EU (WEEE) / Regulation 2017/821 (conflict minerals)


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/vehicle-semiconductor/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-vehicle-semiconductor-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/vehicle-semiconductor-host:1.0.0` ships every vehicle-semiconductor envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/vehicle-semiconductor.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Vehicle-semiconductor deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
