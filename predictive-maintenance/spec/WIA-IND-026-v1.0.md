# WIA-IND-026: Predictive Maintenance Standard
## Version 1.0 - Technical Specification

**Standard ID**: WIA-IND-026
**Category**: IND (Industry) - Amber (#F59E0B)
**Status**: Production Ready
**Published**: January 2025
**Organization**: World Certification Industry Association (WIA)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Architecture Overview](#5-architecture-overview)
6. [Data Collection Layer](#6-data-collection-layer)
7. [Sensor Integration](#7-sensor-integration)
8. [Analysis Methodologies](#8-analysis-methodologies)
9. [Predictive Models](#9-predictive-models)
10. [Maintenance Optimization](#10-maintenance-optimization)
11. [Security and Privacy](#11-security-and-privacy)
12. [API Specification](#12-api-specification)
13. [Implementation Guidelines](#13-implementation-guidelines)
14. [Compliance and Certification](#14-compliance-and-certification)
15. [Appendices](#15-appendices)

---

## 1. Introduction

### 1.1 Purpose

WIA-IND-026 establishes a comprehensive, standardized framework for implementing predictive maintenance systems across industrial sectors. This standard enables organizations to:

- **Predict failures** before they occur through advanced analytics
- **Optimize maintenance schedules** based on actual equipment condition
- **Reduce downtime** through proactive intervention
- **Extend asset life** through optimal maintenance timing
- **Lower maintenance costs** by eliminating unnecessary preventive maintenance

### 1.2 Philosophy

Grounded in the principle of **弘益人間 (홍익인간)** - "Benefit All Humanity" - this standard aims to:

- Democratize access to advanced predictive maintenance technology
- Reduce industrial accidents through better equipment monitoring
- Minimize environmental impact through optimized resource usage
- Enable small and large organizations alike to benefit from predictive analytics

### 1.3 Background

Traditional maintenance strategies include:

- **Reactive Maintenance**: Fix when broken (high downtime, safety risks)
- **Preventive Maintenance**: Fixed schedules (inefficient, costly)
- **Predictive Maintenance**: Condition-based (optimal, data-driven)

WIA-IND-026 standardizes the latter, enabling industry-wide best practices.

### 1.4 Key Innovations

- **Multi-modal sensor fusion**: Combining vibration, thermal, acoustic, and oil data
- **Edge AI processing**: Real-time analysis at the sensor level
- **Adaptive learning**: Models that improve with operational data
- **Open architecture**: Compatible with existing SCADA, MES, and ERP systems
- **Industry agnostic**: Applicable across manufacturing, energy, transportation, and more

---

## 2. Scope

### 2.1 In Scope

This standard covers:

- **Asset registration and lifecycle management**
- **Sensor data collection and normalization**
- **Real-time and batch analytics**
- **Failure mode prediction and classification**
- **Remaining useful life (RUL) estimation**
- **Maintenance schedule optimization**
- **Work order generation and tracking**
- **Spare parts inventory management**
- **Integration with enterprise systems**
- **Reporting and visualization**

### 2.2 Out of Scope

This standard does NOT cover:

- Physical installation of sensors (refer to manufacturer guidelines)
- General CMMS/EAM functionality (use existing systems)
- Safety instrumented systems (SIS) - use IEC 61508/61511
- Financial accounting for maintenance (use enterprise ERP)

### 2.3 Target Industries

- Manufacturing (discrete and process)
- Energy (oil & gas, power generation, renewables)
- Transportation (rail, aviation, maritime)
- Mining and heavy industry
- Data centers and critical infrastructure
- Building management and facilities

---

## 3. Normative References

The following documents are referenced in this standard:

### 3.1 ISO Standards

- **ISO 13374-1**: Condition monitoring and diagnostics - Data processing, communication and presentation
- **ISO 13374-2**: Data processing
- **ISO 17359**: Condition monitoring and diagnostics - General guidelines
- **ISO 18436-1**: Condition monitoring and diagnostics - Requirements for qualification and assessment of personnel - Part 1: Requirements for certification bodies
- **ISO 18436-2**: Vibration condition monitoring and diagnostics
- **ISO 20816-1**: Mechanical vibration - Measurement and evaluation of machine vibration - Part 1: General guidelines
- **ISO 10816**: Mechanical vibration - Evaluation of machine vibration by measurements on non-rotating parts
- **ISO 14224**: Petroleum, petrochemical and natural gas industries - Collection and exchange of reliability and maintenance data for equipment

### 3.2 IEC Standards

- **IEC 60812**: Analysis techniques for system reliability - Procedure for failure mode and effects analysis (FMEA)
- **IEC 61882**: Hazard and operability studies (HAZOP studies)
- **IEC 62061**: Safety of machinery - Functional safety of safety-related electrical, electronic and programmable electronic control systems

### 3.3 Industry Standards

- **MIMOSA OSA-CBM**: Open System Architecture for Condition-Based Maintenance
- **ASHRAE Guideline 36**: High-Performance Sequences of Operation for HVAC Systems
- **NFPA 70B**: Recommended Practice for Electrical Equipment Maintenance
- **AGMA 1010**: Appearance of Gear Teeth - Terminology of Wear and Failure
- **API 670**: Machinery Protection Systems

### 3.4 WIA Standards

- **WIA-IOT-001**: IoT Device Integration
- **WIA-AI-015**: Artificial Intelligence and Machine Learning
- **WIA-DATA-008**: Time-Series Data Management
- **WIA-CLOUD-012**: Cloud Infrastructure
- **WIA-SEC-005**: Security and Encryption

---

## 4. Terms and Definitions

### 4.1 General Terms

**Asset**: Any physical equipment, machinery, or infrastructure monitored for maintenance purposes.

**Condition Monitoring**: The process of monitoring a parameter of condition in machinery (vibration, temperature, etc.) to identify significant change indicative of developing fault.

**Predictive Maintenance (PdM)**: Maintenance strategy that monitors equipment condition and predicts when maintenance should be performed.

**Remaining Useful Life (RUL)**: Estimated time an asset will continue to function before failure.

**Failure Mode**: The manner in which a component, system, or assembly fails to perform its intended function.

**Prognostics**: The science of predicting the future state of a system or component.

**Diagnostics**: The process of identifying the nature and cause of a deviation from normal conditions.

**Mean Time Between Failures (MTBF)**: Average time between system failures.

**Mean Time To Repair (MTTR)**: Average time required to repair a failed component.

### 4.2 Vibration Analysis Terms

**RMS (Root Mean Square)**: Overall vibration energy level.

**Peak**: Maximum amplitude of vibration.

**Crest Factor**: Ratio of peak to RMS, indicates impact events.

**Kurtosis**: Statistical measure of impulsiveness, sensitive to bearing defects.

**FFT (Fast Fourier Transform)**: Converts time-domain signal to frequency domain.

**Envelope Analysis**: High-frequency demodulation to detect bearing defects.

**Order Analysis**: Analysis of vibration as a function of shaft speed.

**Orbit Analysis**: Plot of shaft position showing dynamic behavior.

### 4.3 Thermal Analysis Terms

**Hotspot**: Localized area of elevated temperature.

**Thermal Gradient**: Rate of temperature change over distance.

**Emissivity**: Measure of surface's ability to emit thermal radiation.

**Delta-T (ΔT)**: Temperature difference from reference.

**Thermal Pattern**: Characteristic temperature distribution indicating specific condition.

### 4.4 Oil Analysis Terms

**Viscosity**: Measure of fluid's resistance to flow.

**TAN (Total Acid Number)**: Measure of oil degradation.

**TBN (Total Base Number)**: Measure of remaining additive package.

**Particle Count**: Number of particles per unit volume, by size.

**Wear Metals**: Metallic elements indicating component wear.

**Water Content**: Amount of water contamination in oil.

**ISO Cleanliness Code**: Standard classification of particle contamination (e.g., 18/16/13).

### 4.5 Acoustic Analysis Terms

**Ultrasonic**: Frequencies above 20 kHz, used for leak detection.

**Airborne**: Sound transmitted through air.

**Structure-Borne**: Vibration transmitted through solid structures.

**dB (Decibel)**: Logarithmic unit of sound pressure level.

**Frequency Spectrum**: Distribution of acoustic energy across frequencies.

---

## 5. Architecture Overview

### 5.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        WIA-IND-026 System                        │
└─────────────────────────────────────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        │                       │                       │
┌───────▼───────┐     ┌─────────▼────────┐    ┌────────▼────────┐
│  Edge Layer   │     │  Processing      │    │  Application    │
│               │     │  Layer           │    │  Layer          │
│ • Sensors     │────▶│ • Analytics      │───▶│ • Dashboards    │
│ • Data Acq.   │     │ • ML Models      │    │ • Alerts        │
│ • Edge AI     │     │ • Predictions    │    │ • Reports       │
└───────────────┘     └──────────────────┘    └─────────────────┘
        │                       │                       │
        └───────────────────────┼───────────────────────┘
                                │
                    ┌───────────▼───────────┐
                    │   Data Storage        │
                    │ • Time-series DB      │
                    │ • Object Store        │
                    │ • Model Registry      │
                    └───────────────────────┘
```

### 5.2 Deployment Models

#### 5.2.1 On-Premise Deployment

- Full control over data and infrastructure
- Low latency for real-time processing
- Suitable for regulated industries or air-gapped environments
- Requires local infrastructure and maintenance

#### 5.2.2 Cloud Deployment

- Scalable compute and storage
- Global accessibility
- Managed services and automatic updates
- Usage-based pricing

#### 5.2.3 Hybrid Deployment

- Edge processing for real-time decisions
- Cloud storage for long-term data and advanced analytics
- Best of both worlds
- Recommended for most use cases

#### 5.2.4 Distributed Edge Deployment

- Multiple edge nodes with local processing
- Peer-to-peer coordination
- Resilient to network failures
- Suitable for remote or distributed assets

### 5.3 Data Flow

```
Sensors → Data Acquisition → Preprocessing → Feature Extraction →
ML Models → Predictions → Maintenance Planning → Work Orders
                ↓                  ↓                  ↓
          Historical DB      Model Registry    CMMS/ERP Integration
```

### 5.4 Integration Points

- **SCADA Systems**: Real-time operational data
- **MES (Manufacturing Execution Systems)**: Production schedules and quality data
- **ERP Systems**: Asset master data, spare parts inventory
- **CMMS/EAM**: Work order management
- **Historian**: Time-series operational data
- **IoT Platforms**: Sensor data ingestion
- **BI/Analytics**: Reporting and dashboards

---

## 6. Data Collection Layer

### 6.1 Data Acquisition Requirements

#### 6.1.1 Sampling Rates

Different sensor types require different sampling rates:

| Sensor Type | Minimum Sampling Rate | Recommended Sampling Rate | Max Frequency |
|-------------|----------------------|---------------------------|---------------|
| Vibration | 2.56x max frequency | 10x max frequency | 25.6 kHz |
| Temperature | 0.1 Hz | 1 Hz | 10 Hz |
| Pressure | 1 Hz | 10 Hz | 1 kHz |
| Acoustic (ultrasonic) | 40 kHz | 100 kHz | 500 kHz |
| Current/Voltage | 1 kHz | 10 kHz | 100 kHz |
| Oil quality | On-demand | 1 sample/week | N/A |

#### 6.1.2 Data Resolution

- **Vibration**: Minimum 16-bit ADC, 24-bit recommended
- **Temperature**: 0.1°C resolution minimum
- **Pressure**: 0.1% of full scale
- **Acoustic**: 16-bit minimum

#### 6.1.3 Synchronization

For multi-channel analysis:
- **Time synchronization**: ±1ms across all sensors
- **Phase accuracy**: ±1° for multi-axis vibration
- **Trigger mechanisms**: Hardware triggers for synchronized capture

### 6.2 Data Quality

#### 6.2.1 Validation Rules

All incoming data must be validated:

```typescript
interface DataValidation {
  // Range checking
  minValue: number;
  maxValue: number;

  // Statistical outlier detection
  sigmaThreshold: number; // e.g., 3-sigma rule

  // Temporal consistency
  maxRateOfChange: number; // per second

  // Sensor health
  signalToNoiseRatio: number; // minimum SNR

  // Data completeness
  maxMissingDataPercent: number; // e.g., <5%
}
```

#### 6.2.2 Data Cleansing

- **Spike removal**: Median filtering for transient noise
- **Gap filling**: Linear interpolation for short gaps (<1% of window)
- **Baseline correction**: DC offset removal
- **Noise reduction**: Adaptive filtering

#### 6.2.3 Data Tagging

All data points must include:

```typescript
interface DataPoint {
  timestamp: number; // Unix timestamp in milliseconds
  sensorId: string;
  assetId: string;
  value: number | number[]; // Single value or array for multi-channel
  unit: string;
  quality: 'GOOD' | 'UNCERTAIN' | 'BAD';
  metadata?: {
    operatingCondition?: string; // e.g., "NORMAL_LOAD", "STARTUP"
    environmentalTemp?: number;
    processParameter?: Record<string, any>;
  };
}
```

### 6.3 Data Storage

#### 6.3.1 Time-Series Database

Use WIA-DATA-008 compliant time-series database:

- **Retention policies**:
  - Raw data: 90 days (configurable)
  - Aggregated data (1-minute): 1 year
  - Aggregated data (1-hour): 5 years
  - Statistical summaries: Indefinite

- **Compression**:
  - Use delta encoding for timestamps
  - Gorilla compression for floating-point values
  - Target compression ratio: >10:1

#### 6.3.2 Object Storage

For large binary data (waveforms, images):

- **Format**: HDF5, Parquet, or domain-specific formats
- **Indexing**: Metadata catalog for quick retrieval
- **Tiering**: Hot (SSD) → Warm (HDD) → Cold (Archive)

#### 6.3.3 Model Storage

ML models and their metadata:

- **Model registry**: Versioned storage of trained models
- **Lineage tracking**: Training data, parameters, performance metrics
- **A/B testing**: Support for multiple model versions

---

## 7. Sensor Integration

### 7.1 Vibration Sensors

#### 7.1.1 Accelerometers

**Specifications**:
- Frequency range: 1 Hz - 10 kHz (general purpose), 0.5 Hz - 20 kHz (high frequency)
- Sensitivity: 100 mV/g typical
- Mounting: Stud mount (preferred), magnetic base, adhesive

**Placement**:
- Bearings: Radial and axial positions
- Motor frames: Drive end (DE) and non-drive end (NDE)
- Gearboxes: Input and output shafts, case locations

**Example Configuration**:
```typescript
{
  sensorType: 'ACCELEROMETER',
  model: 'PCB-356A15',
  sensitivity: 100, // mV/g
  frequencyRange: [1, 10000], // Hz
  mounting: 'STUD_MOUNT',
  location: {
    asset: 'MOTOR-001',
    position: 'DRIVE_END_HORIZONTAL'
  },
  samplingRate: 25600, // Hz
  acRange: 10 // g pk
}
```

#### 7.1.2 Velocity Sensors

**Use cases**: Lower frequency machinery (< 1000 RPM)

**Specifications**:
- Frequency range: 10 Hz - 1 kHz
- Units: mm/s or in/s
- Self-generating (no power required)

#### 7.1.3 Displacement Probes

**Use cases**: Shaft position, proximity measurements

**Specifications**:
- Frequency range: DC - 10 kHz
- Gap range: 0.5 - 5 mm typical
- Non-contact eddy current type

### 7.2 Temperature Sensors

#### 7.2.1 Thermocouples

**Types**:
- Type K: General purpose, -200°C to 1350°C
- Type J: Iron-constantan, -40°C to 750°C
- Type T: Copper-constantan, -200°C to 350°C

**Accuracy**: ±0.5°C to ±2°C depending on type

#### 7.2.2 RTDs (Resistance Temperature Detectors)

**Specifications**:
- Pt100, Pt1000 most common
- Accuracy: ±0.1°C
- Range: -200°C to 850°C
- Response time: 1-10 seconds

#### 7.2.3 Infrared Thermal Cameras

**Applications**: Non-contact temperature mapping

**Specifications**:
- Resolution: 640×480 pixels minimum
- Temperature range: -20°C to 500°C typical
- Accuracy: ±2°C or ±2% of reading
- Emissivity correction: Required

**Example Configuration**:
```typescript
{
  sensorType: 'THERMAL_CAMERA',
  model: 'FLIR-E95',
  resolution: [640, 480],
  thermalSensitivity: 0.03, // °C
  temperatureRange: [-20, 1500], // °C
  location: {
    asset: 'SWITCHGEAR-001',
    scanArea: 'FULL_PANEL'
  },
  captureRate: 1, // Hz
  autoEmissivity: true
}
```

### 7.3 Acoustic Sensors

#### 7.3.1 Ultrasonic Sensors

**Frequency range**: 20 kHz - 100 kHz

**Applications**:
- Compressed air leak detection
- Electrical discharge (corona, arcing, tracking)
- Steam trap monitoring
- Valve leak detection

**Sensitivity**: -60 dB typical at 40 kHz

#### 7.3.2 Acoustic Emission Sensors

**Frequency range**: 100 kHz - 1 MHz

**Applications**:
- Crack propagation
- Bearing defect detection
- Leak detection in pressure vessels

#### 7.3.3 Microphones

**Frequency range**: 20 Hz - 20 kHz

**Applications**:
- Abnormal sound detection
- Pattern recognition for fault diagnosis

### 7.4 Oil Analysis Sensors

#### 7.4.1 Online Particle Counters

**Specifications**:
- Size range: 4 μm - 100 μm
- Channels: Typically 6 (ISO code compatible)
- Flow rate: 50-100 mL/min
- Output: ISO 4406 cleanliness code

#### 7.4.2 Viscosity Sensors

**Technology**: Vibrating element or rotational

**Range**: 1 - 1000 cSt typical

**Accuracy**: ±1% of reading

#### 7.4.3 Water Content Sensors

**Technology**: Capacitive or Karl Fischer

**Range**: 0 - 1000 ppm

**Accuracy**: ±10 ppm

#### 7.4.4 Ferrography

**Purpose**: Detect ferrous wear particles

**Size range**: 1 μm - 100 μm

**Analysis**: Automated particle classification

### 7.5 Electrical Sensors

#### 7.5.1 Current Sensors

**Types**:
- Hall effect
- Rogowski coil
- Current transformer (CT)

**Range**: 0 - 1000 A typical

**Accuracy**: ±1%

**Sampling rate**: 10 kHz minimum for Motor Current Signature Analysis (MCSA)

#### 7.5.2 Voltage Sensors

**Range**: 0 - 1000 V typical

**Accuracy**: ±0.5%

**Applications**: Power quality, insulation monitoring

### 7.6 Sensor Communication Protocols

#### 7.6.1 Industrial Protocols

Supported protocols:

- **Modbus TCP/RTU**: Simple, widely supported
- **OPC UA**: Modern, secure, platform-independent
- **MQTT**: Lightweight, pub/sub model
- **IO-Link**: Sensor-level communication
- **EtherCAT**: Real-time, high performance
- **PROFINET**: Industrial Ethernet standard

#### 7.6.2 Data Format

Standard sensor data packet:

```typescript
interface SensorDataPacket {
  header: {
    packetId: string;
    timestamp: number; // Unix timestamp in ms
    sensorId: string;
    protocol: string; // e.g., "MODBUS_TCP"
    version: string; // Protocol version
  };
  payload: {
    dataType: 'SCALAR' | 'WAVEFORM' | 'IMAGE' | 'SPECTRUM';
    values: number | number[] | Buffer;
    unit: string;
    samplingRate?: number; // Hz, for waveforms
    metadata?: Record<string, any>;
  };
  checksum: string; // CRC32 or similar
}
```

---

## 8. Analysis Methodologies

### 8.1 Vibration Analysis

#### 8.1.1 Time-Domain Analysis

**Overall Levels**:

- **Peak (Pk)**: Maximum amplitude
  ```
  Pk = max(|x(t)|)
  ```

- **Peak-to-Peak (Pk-Pk)**: Range of vibration
  ```
  Pk-Pk = max(x(t)) - min(x(t))
  ```

- **RMS (Root Mean Square)**: Overall energy
  ```
  RMS = sqrt(1/N * Σ(x_i²))
  ```

- **Crest Factor (CF)**: Ratio of peak to RMS
  ```
  CF = Pk / RMS
  ```
  - Normal: 3-4
  - High (>4): Indicates impacts or bearing defects

- **Kurtosis**: Fourth statistical moment
  ```
  K = (1/N * Σ((x_i - μ)⁴)) / σ⁴
  ```
  - Normal: ~3
  - High (>5): Strong indicator of bearing defects

**Severity Guidelines** (ISO 20816):

| RMS Velocity (mm/s) | Condition |
|---------------------|-----------|
| < 2.8 | Good |
| 2.8 - 7.1 | Acceptable |
| 7.1 - 11.2 | Unsatisfactory |
| > 11.2 | Unacceptable |

#### 8.1.2 Frequency-Domain Analysis

**FFT (Fast Fourier Transform)**:

Converts time-domain signal to frequency domain:

```
X(f) = Σ x(t) * e^(-j2πft)
```

**Key Features to Detect**:

- **1X RPM**: Imbalance, misalignment, bent shaft
- **2X RPM**: Misalignment, looseness
- **3X+ RPM**: Looseness, resonance
- **Non-synchronous**: Bearing defects, gearbox issues

**Bearing Fault Frequencies**:

For rolling element bearings, calculate defect frequencies:

- **BPFO (Ball Pass Frequency Outer)**:
  ```
  BPFO = (N_b/2) * (1 - (d_b/d_p) * cos(α)) * f_r
  ```

- **BPFI (Ball Pass Frequency Inner)**:
  ```
  BPFI = (N_b/2) * (1 + (d_b/d_p) * cos(α)) * f_r
  ```

- **BSF (Ball Spin Frequency)**:
  ```
  BSF = (d_p/(2*d_b)) * (1 - ((d_b/d_p) * cos(α))²) * f_r
  ```

- **FTF (Fundamental Train Frequency)**:
  ```
  FTF = (1/2) * (1 - (d_b/d_p) * cos(α)) * f_r
  ```

Where:
- N_b = Number of rolling elements
- d_b = Ball diameter
- d_p = Pitch diameter
- α = Contact angle
- f_r = Shaft rotational frequency

**Gearbox Analysis**:

- **GMF (Gear Mesh Frequency)**:
  ```
  GMF = N_teeth * f_r
  ```
- Sidebands around GMF indicate modulation (misalignment, eccentricity)

#### 8.1.3 Envelope Analysis

Demodulation technique for detecting bearing defects:

**Process**:
1. Bandpass filter (typically 500 Hz - 10 kHz)
2. Rectify signal
3. Lowpass filter to obtain envelope
4. FFT of envelope

**Advantages**: Enhances bearing fault frequencies buried in noise

#### 8.1.4 Order Analysis

Relates vibration to shaft speed (RPM):

**Synchronous sampling**: Resample based on once-per-revolution trigger

**Applications**:
- Variable speed machinery
- Run-up/coast-down analysis
- Resonance identification

### 8.2 Thermal Analysis

#### 8.2.1 Absolute Temperature Monitoring

**Alarm thresholds**:

| Component | Warning (°C) | Alarm (°C) |
|-----------|--------------|------------|
| Motor bearings | 80 | 100 |
| Motor windings | 100 | 120 |
| Electrical connections | 40 above ambient | 70 above ambient |
| Gearbox oil | 70 | 85 |

#### 8.2.2 Delta-T (ΔT) Analysis

More sensitive than absolute temperature:

- **Temporal ΔT**: Current vs. historical baseline
  ```
  ΔT_temporal = T_current - T_baseline
  ```

- **Spatial ΔT**: Comparing similar components
  ```
  ΔT_spatial = T_phase_A - T_phase_B
  ```

**Thresholds**:
- ΔT > 10°C: Investigate
- ΔT > 20°C: Immediate action

#### 8.2.3 Thermal Pattern Recognition

Use ML models to classify thermal patterns:

- **Hot spots**: Localized high temperature
- **Gradients**: Abnormal temperature distribution
- **Cycling**: Thermal on-off cycling indicating loose connections

#### 8.2.4 Infrared Thermography Standards

Follow ASNT SNT-TC-1A for thermographer certification:

- **Level I**: Assisted inspections
- **Level II**: Independent inspections, report generation
- **Level III**: Procedure development, training

### 8.3 Oil Analysis

#### 8.3.1 Physical Properties

**Viscosity**:

Measure at 40°C and 100°C, calculate Viscosity Index (VI):

- **Acceptable**: Within ±10% of new oil
- **Marginal**: ±10-20%
- **Unacceptable**: >±20%

**Water content**:

- **Target**: < 100 ppm for most oils
- **Caution**: 100-500 ppm
- **Critical**: > 500 ppm

**TAN (Total Acid Number)**:

Measure of oil oxidation:

- **New oil**: 0.5-2.0 mg KOH/g typical
- **Change oil when**: TAN increases by 50% or exceeds 3.0 mg KOH/g

#### 8.3.2 Contamination Analysis

**Particle counting**:

ISO 4406 cleanliness code: X/Y/Z

- X: Particles >4 μm per 100 mL
- Y: Particles >6 μm per 100 mL
- Z: Particles >14 μm per 100 mL

**Typical targets**:

- Hydraulics: 18/16/13 or better
- Gearboxes: 19/17/14 or better
- Turbines: 16/14/11 or better

**Wear metals (by spectrometry)**:

| Metal | Source | Caution Level (ppm) |
|-------|--------|---------------------|
| Iron (Fe) | Gears, cylinders, shafts | >100 |
| Copper (Cu) | Bearings, bushings | >30 |
| Aluminum (Al) | Pistons, bushings | >25 |
| Chromium (Cr) | Piston rings, shafts | >20 |
| Tin (Sn) | Bearings | >10 |
| Lead (Pb) | Bearings | >50 |

#### 8.3.3 Trending and Alarms

Use **rate of change** in addition to absolute values:

```
ROC = (Value_current - Value_previous) / Time_interval
```

Set multi-level alarms:

- **Normal**: Within typical range
- **Caution**: Exceeds threshold, monitor closely
- **Warning**: Rapid increase, schedule maintenance
- **Critical**: Immediate action required

### 8.4 Acoustic Analysis

#### 8.4.1 Ultrasonic Leak Detection

**Methodology**:

- Scan suspect areas with ultrasonic sensor
- Compare sound levels (dB)
- Locate leak source by maximum signal

**Leak rate estimation**:

```
Leak_rate (CFM) = K * (dB - dB_baseline) * D²
```

Where:
- K = Constant for gas type
- D = Orifice diameter estimate
- dB = Measured sound level

#### 8.4.2 Electrical Discharge Detection

**Partial discharge (corona)**:

- Frequency: 35-45 kHz
- Pattern: Continuous or pulsing
- Severity: Based on dB level and pattern

#### 8.4.3 Mechanical Sound Analysis

Use ML models trained on normal/fault sounds:

- **Feature extraction**: MFCCs, spectral features
- **Classification**: SVM, Random Forest, Neural Networks
- **Anomaly detection**: Autoencoders, One-Class SVM

### 8.5 Motor Current Signature Analysis (MCSA)

#### 8.5.1 Methodology

Analyze motor current to detect:

- **Broken rotor bars**
- **Eccentricity**
- **Bearing faults**
- **Load variations**

#### 8.5.2 Fault Frequencies

**Broken rotor bar**:

Sidebands around line frequency:

```
f_brb = f_line * (1 ± 2 * s)
```

Where s = slip

**Bearing faults**:

Appear as sidebands around line frequency:

```
f_bearing_MCSA = f_line ± k * f_bearing_defect
```

---

## 9. Predictive Models

### 9.1 Model Categories

#### 9.1.1 Physics-Based Models

Use domain knowledge and first principles:

- **Bearing life models**: L10 life calculation (ISO 281)
  ```
  L_10 = (C/P)^p * 10⁶ / (60 * n)
  ```
  Where:
  - C = Dynamic load rating
  - P = Equivalent dynamic load
  - p = 3 for ball bearings, 10/3 for roller bearings
  - n = Speed (RPM)

- **Crack propagation**: Paris' law
  ```
  da/dN = C * (ΔK)^m
  ```

- **Thermal degradation**: Arrhenius equation
  ```
  Rate = A * e^(-E_a / (R*T))
  ```

#### 9.1.2 Statistical Models

Data-driven, no physics assumptions:

- **Regression**: Linear, polynomial, support vector regression
- **Survival analysis**: Weibull, lognormal distributions
- **Time series**: ARIMA, exponential smoothing

#### 9.1.3 Machine Learning Models

**Supervised Learning**:

For classification (fault/no-fault, fault type):

- **Random Forest**: Robust, interpretable
- **Gradient Boosting** (XGBoost, LightGBM): High accuracy
- **SVM**: Good for small datasets
- **Neural Networks**: For complex patterns

For regression (RUL prediction):

- **Random Forest Regressor**
- **Gradient Boosting Regressor**
- **Deep Neural Networks**

**Unsupervised Learning**:

For anomaly detection:

- **Isolation Forest**: Fast, effective
- **One-Class SVM**: Classic approach
- **Autoencoders**: Neural network-based
- **DBSCAN**: Density-based clustering

**Deep Learning**:

For complex sensor fusion:

- **CNN (Convolutional Neural Networks)**: For spectral/image data
- **RNN/LSTM**: For time-series sequences
- **Attention mechanisms**: For key feature identification
- **Transformer models**: State-of-the-art for sequences

### 9.2 Feature Engineering

#### 9.2.1 Vibration Features

**Time domain** (per ISO 13374-2):

- RMS, Peak, Crest Factor, Kurtosis, Skewness
- Energy, Entropy

**Frequency domain**:

- Peak amplitude at 1X, 2X, 3X RPM
- Bearing fault frequencies
- High-frequency RMS (>2 kHz)
- Spectral moments

**Envelope domain**:

- Envelope RMS
- Envelope peak
- Envelope kurtosis

#### 9.2.2 Thermal Features

- Mean, max, min temperature
- Standard deviation
- ΔT from baseline
- Rate of temperature change
- Hot spot count and size

#### 9.2.3 Oil Analysis Features

- Viscosity, TAN, water content
- Particle counts by size
- Wear metal concentrations
- Ratios (e.g., Fe/Cu, ΔParticles/ΔTime)

#### 9.2.4 Operational Features

- Load (%, kW, torque)
- Speed (RPM)
- Pressure, flow
- Duty cycle
- Start/stop cycles
- Operating hours

### 9.3 Model Training

#### 9.3.1 Data Requirements

**Minimum dataset size**:

- **Physics-based models**: Can work with limited data
- **Statistical models**: 100-1000 samples
- **ML models**: 1000-10,000 samples
- **Deep learning**: 10,000-1,000,000 samples

**Class balance**:

For classification, aim for balanced classes:

- Use **SMOTE** (Synthetic Minority Over-sampling) if needed
- Or **class weights** to penalize majority class

#### 9.3.2 Train/Validation/Test Split

Standard split:

- **Training**: 60-70%
- **Validation**: 15-20%
- **Test**: 15-20%

For time-series, use **forward chaining**:

- Train on historical data
- Validate on subsequent period
- Test on most recent period

#### 9.3.3 Hyperparameter Tuning

Use **grid search** or **random search** or **Bayesian optimization**:

Example for Random Forest:

```python
param_grid = {
    'n_estimators': [100, 200, 500],
    'max_depth': [10, 20, None],
    'min_samples_split': [2, 5, 10],
    'min_samples_leaf': [1, 2, 4]
}
```

#### 9.3.4 Cross-Validation

Use **k-fold cross-validation** (k=5 or 10):

```
CV_score = (1/k) * Σ score_i
```

For time-series, use **time-series split**:

```
Train: [1, 2, 3, 4] → Test: [5]
Train: [1, 2, 3, 4, 5] → Test: [6]
...
```

### 9.4 Model Evaluation

#### 9.4.1 Classification Metrics

- **Accuracy**: Overall correctness
  ```
  Accuracy = (TP + TN) / (TP + TN + FP + FN)
  ```

- **Precision**: Of predicted faults, how many are true?
  ```
  Precision = TP / (TP + FP)
  ```

- **Recall (Sensitivity)**: Of actual faults, how many detected?
  ```
  Recall = TP / (TP + FN)
  ```

- **F1 Score**: Harmonic mean of precision and recall
  ```
  F1 = 2 * (Precision * Recall) / (Precision + Recall)
  ```

- **AUC-ROC**: Area under receiver operating characteristic curve

**Target metrics for PdM**:

- **Recall**: >90% (must catch most faults)
- **Precision**: >70% (limit false alarms)

#### 9.4.2 Regression Metrics

For RUL prediction:

- **MAE (Mean Absolute Error)**:
  ```
  MAE = (1/n) * Σ |y_pred - y_true|
  ```

- **RMSE (Root Mean Squared Error)**:
  ```
  RMSE = sqrt((1/n) * Σ (y_pred - y_true)²)
  ```

- **MAPE (Mean Absolute Percentage Error)**:
  ```
  MAPE = (100/n) * Σ |(y_pred - y_true) / y_true|
  ```

- **R² Score**: Coefficient of determination

**Target for RUL**:

- **MAPE**: <20%
- **R²**: >0.7

### 9.5 Model Deployment

#### 9.5.1 Model Serialization

Save models in standard formats:

- **Scikit-learn**: Pickle, Joblib
- **TensorFlow/Keras**: SavedModel, HDF5
- **PyTorch**: TorchScript
- **ONNX**: Universal format for interoperability

#### 9.5.2 Model Serving

**Options**:

- **REST API**: HTTP endpoints for predictions
- **gRPC**: High-performance RPC
- **Edge deployment**: TensorFlow Lite, ONNX Runtime
- **Batch prediction**: Scheduled jobs

**Example API**:

```http
POST /api/v1/predict
Content-Type: application/json

{
  "assetId": "MOTOR-001",
  "features": {
    "rms_vibration": 3.2,
    "peak_vibration": 12.5,
    "crest_factor": 3.9,
    "bearing_freq_amplitude": 0.8,
    "temperature": 75,
    "operating_hours": 12500
  }
}

Response:
{
  "prediction": {
    "failureProbability": 0.85,
    "predictedFailureMode": "BEARING_OUTER_RACE",
    "remainingUsefulLife": {
      "days": 15,
      "confidence": 0.75
    },
    "recommendations": [
      "Schedule bearing replacement within 2 weeks",
      "Increase monitoring frequency to daily"
    ]
  }
}
```

#### 9.5.3 Model Monitoring

Track model performance in production:

- **Prediction distribution**: Check for drift
- **Accuracy metrics**: Compare predicted vs. actual failures
- **Inference time**: Ensure low latency
- **Resource usage**: CPU, memory, GPU utilization

#### 9.5.4 Model Retraining

**Triggers for retraining**:

- **Scheduled**: Monthly, quarterly
- **Performance degradation**: Accuracy drops below threshold
- **Data drift**: Input distribution changes
- **New failure modes**: Incorporate new examples

**Retraining process**:

1. Collect new data since last training
2. Combine with historical data (with appropriate weighting)
3. Retrain model with updated dataset
4. Validate on holdout set
5. A/B test new model vs. old model
6. Deploy if performance improves

---

## 10. Maintenance Optimization

### 10.1 Work Order Generation

#### 10.1.1 Triggering Conditions

Generate work orders when:

- **Failure probability** > threshold (e.g., 70%)
- **RUL** < threshold (e.g., 30 days)
- **Sensor reading** exceeds alarm limit
- **Trend** indicates rapid degradation

#### 10.1.2 Work Order Content

```typescript
interface WorkOrder {
  workOrderId: string;
  assetId: string;
  priority: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  predictedFailureMode: string;
  failureProbability: number; // 0-100
  remainingUsefulLife: {
    days: number;
    confidence: number;
  };
  recommendedActions: string[];
  requiredParts: {
    partNumber: string;
    description: string;
    quantity: number;
    availability: boolean;
  }[];
  estimatedDuration: number; // hours
  requiredSkills: string[];
  scheduledDate?: Date;
  createdBy: 'SYSTEM' | 'USER';
  createdAt: Date;
  status: 'OPEN' | 'SCHEDULED' | 'IN_PROGRESS' | 'COMPLETED' | 'CANCELLED';
}
```

### 10.2 Schedule Optimization

#### 10.2.1 Objective Function

Optimize maintenance schedule to:

**Minimize**:
- Total downtime
- Maintenance cost
- Risk of failure

**Subject to**:
- Maintenance window constraints
- Resource (crew, tools, parts) availability
- Production schedule constraints
- Regulatory requirements

**Mathematical formulation**:

```
Minimize: Σ (downtime_i * cost_downtime_i + maintenance_cost_i + risk_i)

Subject to:
  - start_time_i ∈ available_windows
  - crew_assigned_i ≤ crew_available
  - parts_required_i ≤ parts_in_stock
  - end_time_i ≤ required_completion_date_i
```

#### 10.2.2 Optimization Algorithms

**Exact methods** (for small problems):

- **Linear Programming** (LP)
- **Mixed-Integer Programming** (MIP)

**Heuristic methods** (for large problems):

- **Genetic Algorithms**
- **Simulated Annealing**
- **Particle Swarm Optimization**
- **Greedy heuristics**

#### 10.2.3 Multi-Asset Coordination

**Bundling opportunities**:

- Group maintenance on nearby assets
- Coordinate shutdowns to minimize disruption
- Leverage shared resources (cranes, scaffolding)

**Example**:

```typescript
{
  maintenanceBundle: {
    bundleId: 'BUNDLE-2025-001',
    assets: ['MOTOR-001', 'PUMP-002', 'VALVE-003'],
    window: {
      start: '2025-02-15T00:00:00Z',
      end: '2025-02-15T08:00:00Z'
    },
    totalDowntime: 8, // hours
    totalCost: 15000, // USD
    savings: 5000, // vs. individual maintenance
    assignedCrew: ['TEAM-A', 'TEAM-B']
  }
}
```

### 10.3 Spare Parts Management

#### 10.3.1 Demand Forecasting

Predict spare parts needs based on:

- **Failure predictions**: Expected failure modes
- **Historical consumption**: Time-series forecasting
- **Seasonal patterns**: Adjust for known variations

**Methods**:

- **Time-series**: ARIMA, exponential smoothing
- **ML**: Random Forest, Gradient Boosting
- **Simulation**: Monte Carlo for uncertainty

#### 10.3.2 Inventory Optimization

**Reorder point (ROP)**:

```
ROP = (Average daily usage * Lead time) + Safety stock
```

**Safety stock**:

```
Safety_stock = Z * σ_demand * sqrt(lead_time)
```

Where Z = service level factor (e.g., 1.96 for 95% service level)

**Economic Order Quantity (EOQ)**:

```
EOQ = sqrt((2 * D * S) / H)
```

Where:
- D = Annual demand
- S = Order cost
- H = Holding cost per unit per year

#### 10.3.3 Critical Spares Identification

**Criticality score**:

```
Criticality = (Failure_frequency * Downtime_cost * Lead_time) / Unit_cost
```

Prioritize stocking high-criticality items.

### 10.4 Reliability-Centered Maintenance (RCM)

#### 10.4.1 Failure Modes and Effects Analysis (FMEA)

For each asset and component:

| Component | Failure Mode | Effect | Severity (1-10) | Occurrence (1-10) | Detection (1-10) | RPN |
|-----------|--------------|--------|-----------------|-------------------|------------------|-----|
| Bearing | Outer race spall | Vibration, noise | 8 | 6 | 4 | 192 |
| Bearing | Lubrication failure | Overheating | 9 | 3 | 3 | 81 |

**RPN (Risk Priority Number)** = Severity × Occurrence × Detection

Prioritize high RPN items.

#### 10.4.2 Maintenance Task Selection

For each failure mode, select appropriate strategy:

- **Predictive**: If failure is detectable and progresses over time
- **Preventive**: If failure is age-related and not easily detected
- **Run-to-failure**: If consequence is low and not detectable
- **Redesign**: If inherently unreliable

---

## 11. Security and Privacy

### 11.1 Data Security

#### 11.1.1 Encryption

**At rest**:

- AES-256 for stored data
- Encrypted databases
- Secure key management (WIA-SEC-005)

**In transit**:

- TLS 1.3 for all network communication
- Certificate-based authentication
- Perfect Forward Secrecy (PFS)

#### 11.1.2 Access Control

**Role-Based Access Control (RBAC)**:

| Role | Permissions |
|------|-------------|
| Operator | View dashboards, acknowledge alarms |
| Technician | View data, create work orders |
| Engineer | View data, configure sensors, run analyses |
| Admin | All permissions, user management, configuration |

**Principle of Least Privilege**: Grant minimum necessary access.

#### 11.1.3 Audit Logging

Log all actions:

```typescript
interface AuditLog {
  timestamp: Date;
  userId: string;
  action: string; // e.g., "VIEW_ASSET", "MODIFY_CONFIG", "DELETE_DATA"
  resource: string;
  result: 'SUCCESS' | 'FAILURE';
  ipAddress: string;
  metadata?: Record<string, any>;
}
```

Retain logs for minimum 1 year, or per regulatory requirements.

### 11.2 Privacy

#### 11.2.1 Data Minimization

Collect only necessary data. Avoid:

- Personal identifiable information (PII) in sensor tags
- Video/audio recordings of personnel
- Location tracking beyond asset location

#### 11.2.2 Anonymization

For ML model training shared across organizations:

- **Anonymize asset IDs**: Use hashed or pseudonymized identifiers
- **Aggregate features**: Provide only statistical summaries
- **Differential privacy**: Add noise to prevent re-identification

### 11.3 Compliance

#### 11.3.1 Regulatory Requirements

Comply with applicable standards:

- **GDPR**: If processing data of EU residents
- **CCPA**: If processing data of California residents
- **Industry-specific**: FDA CFR Part 11 (pharma), 21 CFR Part 177 (food)

#### 11.3.2 Data Retention

Define retention policies:

```typescript
{
  dataType: 'RAW_SENSOR_DATA',
  retentionPeriod: 90, // days
  archivePolicy: 'AGGREGATE_THEN_DELETE',
  legalHold: false
}
```

---

## 12. API Specification

### 12.1 RESTful API

#### 12.1.1 Base URL

```
https://api.wia.org/ind-026/v1
```

#### 12.1.2 Authentication

Use **API keys** or **OAuth 2.0**:

```http
Authorization: Bearer <access_token>
```

Or:

```http
X-API-Key: <api_key>
```

#### 12.1.3 Endpoints

**Assets**:

```http
# Register asset
POST /assets
{
  "assetId": "MOTOR-001",
  "assetType": "ROTATING_MACHINERY",
  "location": "Plant-A-Line-1",
  "specifications": {...}
}

# Get asset
GET /assets/{assetId}

# Update asset
PUT /assets/{assetId}

# Delete asset
DELETE /assets/{assetId}

# List assets
GET /assets?type=ROTATING_MACHINERY&location=Plant-A
```

**Sensors**:

```http
# Attach sensor
POST /assets/{assetId}/sensors
{
  "sensorId": "VIB-001",
  "type": "VIBRATION",
  "location": "MOTOR_DE",
  "samplingRate": 25600
}

# Get sensors for asset
GET /assets/{assetId}/sensors

# Update sensor
PUT /sensors/{sensorId}

# Detach sensor
DELETE /sensors/{sensorId}
```

**Data Ingestion**:

```http
# Ingest data
POST /data/ingest
{
  "sensorId": "VIB-001",
  "timestamp": 1704067200000,
  "values": [0.1, 0.15, 0.12, ...],
  "unit": "g"
}

# Batch ingest
POST /data/ingest/batch
{
  "data": [
    {"sensorId": "VIB-001", "timestamp": ..., "values": ...},
    {"sensorId": "TEMP-001", "timestamp": ..., "values": ...}
  ]
}

# Stream data (WebSocket)
WS /data/stream?sensorId=VIB-001
```

**Analysis**:

```http
# Vibration analysis
POST /analysis/vibration
{
  "assetId": "MOTOR-001",
  "sensorId": "VIB-001",
  "analysisTypes": ["FFT", "ENVELOPE", "CREST_FACTOR"]
}

# Thermal analysis
POST /analysis/thermal
{
  "assetId": "MOTOR-001",
  "imageData": "<base64-encoded-image>",
  "referenceTemperature": 25
}

# Oil analysis
POST /analysis/oil
{
  "assetId": "GEARBOX-001",
  "sampleId": "OIL-2025-001",
  "tests": {...}
}
```

**Predictions**:

```http
# Get predictions
POST /predictions/analyze
{
  "assetId": "MOTOR-001",
  "timeHorizon": 30
}

Response:
{
  "predictions": [
    {
      "failureMode": "BEARING_OUTER_RACE",
      "probability": 85,
      "severity": "HIGH",
      "daysToFailure": 15,
      "confidence": 0.75,
      "recommendations": [...]
    }
  ]
}

# Get RUL
GET /predictions/rul?assetId=MOTOR-001
```

**Maintenance**:

```http
# Create work order
POST /maintenance/work-orders
{
  "assetId": "MOTOR-001",
  "priority": "HIGH",
  "predictedFailure": "BEARING_OUTER_RACE",
  "scheduledDate": "2025-02-15T00:00:00Z"
}

# Optimize schedule
POST /maintenance/optimize
{
  "assets": ["MOTOR-001", "PUMP-002"],
  "constraints": {...},
  "objectives": ["MINIMIZE_DOWNTIME"]
}

# Get work orders
GET /maintenance/work-orders?status=OPEN&assetId=MOTOR-001
```

#### 12.1.4 Response Format

Standard success response:

```json
{
  "status": "success",
  "data": {...},
  "meta": {
    "timestamp": 1704067200000,
    "requestId": "req-abc123"
  }
}
```

Standard error response:

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_INPUT",
    "message": "Asset ID is required",
    "details": {...}
  },
  "meta": {
    "timestamp": 1704067200000,
    "requestId": "req-abc123"
  }
}
```

#### 12.1.5 Rate Limiting

- **Default**: 1000 requests/hour per API key
- **Burst**: 100 requests/minute
- **Headers**:
  ```
  X-RateLimit-Limit: 1000
  X-RateLimit-Remaining: 750
  X-RateLimit-Reset: 1704070800
  ```

### 12.2 WebSocket API

For real-time data streaming:

```javascript
const ws = new WebSocket('wss://api.wia.org/ind-026/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    sensorIds: ['VIB-001', 'TEMP-001']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Sensor data:', data);
};
```

### 12.3 Webhooks

Register webhooks for events:

```http
POST /webhooks
{
  "url": "https://your-server.com/webhook",
  "events": ["PREDICTION_ALARM", "WORK_ORDER_CREATED"],
  "secret": "your-webhook-secret"
}
```

Webhook payload:

```json
{
  "event": "PREDICTION_ALARM",
  "timestamp": 1704067200000,
  "data": {
    "assetId": "MOTOR-001",
    "failureMode": "BEARING_OUTER_RACE",
    "probability": 85
  },
  "signature": "sha256=..."
}
```

---

## 13. Implementation Guidelines

### 13.1 Deployment Checklist

#### 13.1.1 Pre-Deployment

- [ ] Asset inventory complete
- [ ] Sensor locations identified
- [ ] Baseline data collected (minimum 2 weeks of normal operation)
- [ ] Alarm thresholds defined
- [ ] Integration with CMMS/ERP tested
- [ ] User training completed
- [ ] Backup and disaster recovery plan in place

#### 13.1.2 Deployment

- [ ] Sensors installed and calibrated
- [ ] Data acquisition verified
- [ ] Real-time monitoring dashboard operational
- [ ] Alert routing configured
- [ ] Initial ML models trained (if applicable)
- [ ] Work order workflow tested

#### 13.1.3 Post-Deployment

- [ ] Monitor data quality daily for first month
- [ ] Review alarm thresholds weekly for first month
- [ ] Collect feedback from maintenance team
- [ ] Refine ML models with operational data
- [ ] Document lessons learned

### 13.2 Best Practices

#### 13.2.1 Sensor Placement

- **Vibration**: As close to bearings as possible, on load-bearing structures
- **Thermal**: Clear line-of-sight, stable mounting
- **Acoustic**: Minimize background noise, consistent location
- **Oil**: Representative sampling point, bypass loop for online sensors

#### 13.2.2 Data Collection

- **Sampling rate**: Follow Nyquist criterion (sample at >2× max frequency of interest)
- **Duration**: Collect sufficient samples for statistical analysis
- **Synchronization**: Use NTP or GPS for multi-sensor systems
- **Storage**: Use time-series databases for efficiency

#### 13.2.3 Analysis

- **Start simple**: Time-domain overall levels before complex FFT
- **Baselines**: Establish normal operating conditions first
- **Trending**: Look for changes over time, not just absolute values
- **Context**: Consider load, speed, environmental conditions
- **Validation**: Cross-check with multiple sensor types

#### 13.2.4 Predictions

- **Confidence**: Report prediction confidence, not just point estimates
- **Uncertainty**: Communicate uncertainty ranges (e.g., RUL: 10-20 days)
- **Explainability**: Provide reasons for predictions (which features drove it)
- **Continuous learning**: Update models with actual failure data

#### 13.2.5 Maintenance

- **Verify predictions**: Inspect predicted failures to validate
- **Root cause**: Document actual failure modes for model improvement
- **Timing**: Balance prediction horizon with maintenance planning lead time
- **Communication**: Clear handoff from analytics to maintenance team

### 13.3 Common Pitfalls

#### 13.3.1 Data Quality Issues

- **Symptom**: Erratic predictions, frequent false alarms
- **Causes**: Sensor malfunction, electrical noise, poor mounting
- **Solutions**: Validate data quality checks, sensor health monitoring

#### 13.3.2 Insufficient Baseline Data

- **Symptom**: Can't distinguish normal from abnormal
- **Causes**: Deployment during non-normal operation (startup, maintenance)
- **Solutions**: Collect minimum 2 weeks of verified normal operation

#### 13.3.3 Overfitting ML Models

- **Symptom**: High training accuracy, poor real-world performance
- **Causes**: Too many features, too complex model, insufficient data
- **Solutions**: Cross-validation, regularization, feature selection

#### 13.3.4 Ignored Predictions

- **Symptom**: Predictions not acted upon
- **Causes**: Lack of trust, poor communication, competing priorities
- **Solutions**: Validate predictions, build trust through early successes, integrate into workflow

---

## 14. Compliance and Certification

### 14.1 WIA Certification Levels

#### 14.1.1 Level 1: Basic Compliance

**Requirements**:

- [ ] Implement core data collection (any 1 sensor type)
- [ ] Store data per WIA-DATA-008
- [ ] Provide basic alerts (threshold-based)
- [ ] Generate work orders

**Validation**: Self-assessment checklist

#### 14.1.2 Level 2: Advanced Analytics

**Requirements**:

- [ ] Multi-modal sensor integration (2+ types)
- [ ] Advanced analysis (FFT, envelope, thermal patterns)
- [ ] Failure mode classification (>80% accuracy)
- [ ] RUL prediction (MAPE <30%)

**Validation**: Third-party testing on sample dataset

#### 14.1.3 Level 3: Autonomous Maintenance

**Requirements**:

- [ ] Fully automated predictions and work order generation
- [ ] Schedule optimization
- [ ] Spare parts forecasting
- [ ] Closed-loop integration with CMMS/ERP
- [ ] >90% prediction accuracy, <5% false positive rate

**Validation**: On-site audit and 6-month operational review

### 14.2 Certification Process

1. **Application**: Submit intent to certify, select level
2. **Documentation review**: Provide architecture diagrams, data flows, algorithms
3. **Testing**: Demonstrate compliance on test scenarios
4. **Audit** (Level 3 only): On-site validation
5. **Certification**: Issue WIA-IND-026 certificate
6. **Surveillance**: Annual renewal, ongoing compliance

### 14.3 Compliance Reporting

Annual compliance report must include:

- **Uptime**: System availability (target: >99%)
- **Data quality**: Percentage of valid data (target: >95%)
- **Prediction accuracy**: Confusion matrix for classifications
- **RUL error**: MAPE for RUL predictions
- **False alarms**: Rate of false positives
- **Caught failures**: Percentage of failures predicted in advance
- **Missed failures**: Percentage of unpredicted failures
- **Maintenance cost savings**: Compared to previous strategy

---

## 15. Appendices

### Appendix A: Vibration Severity Charts

**ISO 20816-1 Severity Zones** (for machines >15 kW, speeds 120-15000 RPM):

| RMS Velocity (mm/s) | Zone | Description |
|---------------------|------|-------------|
| 0.0 - 2.8 | A | Good |
| 2.8 - 7.1 | B | Acceptable |
| 7.1 - 11.2 | C | Unsatisfactory |
| > 11.2 | D | Unacceptable |

### Appendix B: Bearing Defect Frequency Calculator

Given:
- Number of balls (N_b)
- Ball diameter (d_b)
- Pitch diameter (d_p)
- Contact angle (α)
- Shaft speed (RPM)

Calculate:

```
f_r = RPM / 60  # Hz

BPFO = (N_b/2) * (1 - (d_b/d_p) * cos(α)) * f_r
BPFI = (N_b/2) * (1 + (d_b/d_p) * cos(α)) * f_r
BSF = (d_p/(2*d_b)) * (1 - ((d_b/d_p) * cos(α))²) * f_r
FTF = (1/2) * (1 - (d_b/d_p) * cos(α)) * f_r
```

**Example: SKF 6206 bearing at 1800 RPM**:

- N_b = 9
- d_b = 9.525 mm
- d_p = 46.4 mm
- α = 0°

```
f_r = 1800/60 = 30 Hz

BPFO = (9/2) * (1 - (9.525/46.4) * 1) * 30 = 107.5 Hz
BPFI = (9/2) * (1 + (9.525/46.4) * 1) * 30 = 162.5 Hz
BSF = (46.4/(2*9.525)) * (1 - ((9.525/46.4) * 1)²) * 30 = 59.8 Hz
FTF = (1/2) * (1 - (9.525/46.4) * 1) * 30 = 11.9 Hz
```

### Appendix C: Oil Analysis Interpretation Guide

**Viscosity**:

| Condition | Viscosity Change from New |
|-----------|---------------------------|
| Excellent | ±5% |
| Good | ±5-10% |
| Fair | ±10-20% |
| Poor | >±20% |

**Action**: Change oil if >±20% or outside OEM spec.

**Water Content**:

| ppm | Condition | Action |
|-----|-----------|--------|
| <100 | Normal | None |
| 100-200 | Caution | Investigate source |
| 200-500 | Warning | Find and fix leak |
| >500 | Critical | Change oil, fix leak |

**Particle Count (ISO Code)**:

| Application | Target ISO Code |
|-------------|-----------------|
| Hydraulic servo | 15/13/10 |
| General hydraulic | 18/16/13 |
| Gearbox | 19/17/14 |
| Engine | 21/19/16 |

### Appendix D: Thermal Imaging Guidelines

**Recommended thermal patterns**:

| Equipment | Pattern | Action |
|-----------|---------|--------|
| Electrical connections | Hot spot >10°C above adjacent | Tighten, clean, replace |
| Motor bearings | ΔT between DE and NDE >10°C | Investigate lubrication |
| Motor phases | Phase imbalance >15°C | Check electrical balance |
| Circuit breakers | Contact ΔT >20°C | Replace breaker |

**Emissivity values**:

| Material | Emissivity (ε) |
|----------|----------------|
| Painted surface | 0.90-0.95 |
| Oxidized steel | 0.80 |
| Polished steel | 0.10 |
| Aluminum (oxidized) | 0.30 |
| Aluminum (polished) | 0.05 |
| Concrete | 0.95 |

### Appendix E: Glossary

**Anomaly Detection**: Identifying patterns that deviate from expected normal behavior.

**CBM (Condition-Based Maintenance)**: Maintenance performed when condition indicators show decreasing performance.

**Digital Twin**: Virtual representation of a physical asset, updated with real-time data.

**Edge Computing**: Processing data close to the source (sensor) rather than in a centralized cloud.

**Feature**: A measurable property or characteristic extracted from raw data.

**Fault**: An abnormal condition that may lead to failure.

**Inference**: Making predictions using a trained ML model.

**Label**: Ground truth used for supervised learning (e.g., "fault" or "normal").

**Latency**: Time delay between data collection and analysis/action.

**Model Drift**: Degradation of model performance over time due to changing conditions.

**Overfitting**: Model learns training data too well, performs poorly on new data.

**Preprocessing**: Cleaning and transforming raw data before analysis.

**Sensitivity**: See Recall.

**Specificity**: True negative rate = TN / (TN + FP).

**Supervised Learning**: ML using labeled training data.

**Threshold**: A value above (or below) which an alarm triggers.

**Training**: Process of building a model from data.

**Unsupervised Learning**: ML using unlabeled data (e.g., clustering, anomaly detection).

**Validation**: Evaluating model performance on data not used in training.

### Appendix F: Reference Implementations

**Open-source tools compatible with WIA-IND-026**:

- **Data collection**: InfluxDB, Prometheus, OpenTSDB
- **ML frameworks**: scikit-learn, TensorFlow, PyTorch
- **Vibration analysis**: Python-acoustics, PyVib
- **Visualization**: Grafana, Kibana, Plotly Dash
- **CMMS integration**: Odoo Maintenance, Maximo (API)

### Appendix G: Training and Certification

**Recommended training programs**:

- **ISO 18436-2 Category I-IV**: Vibration analyst certification
- **Thermography Level I-III** (ASNT SNT-TC-1A)
- **Tribology and lubrication** (STLE certifications)
- **Machine learning for PdM**: Online courses (Coursera, edX)
- **WIA-IND-026 Practitioner**: Official WIA training (40 hours)

### Appendix H: Calculation Examples

**Example 1: RUL Prediction using Exponential Degradation**

Assume bearing vibration RMS increases exponentially:

```
RMS(t) = RMS_0 * e^(k*t)
```

Failure occurs when RMS > RMS_failure.

Given:
- RMS_0 = 2.0 mm/s (current)
- RMS_failure = 11.2 mm/s (ISO Zone D threshold)
- Historical data shows k = 0.05 /day

Solve for RUL:

```
11.2 = 2.0 * e^(0.05 * RUL)
5.6 = e^(0.05 * RUL)
ln(5.6) = 0.05 * RUL
RUL = ln(5.6) / 0.05 = 34.2 days
```

**Example 2: Optimizing Maintenance Schedule**

Two assets need maintenance:

| Asset | Failure Prob | RUL (days) | Downtime (hrs) | Cost ($) |
|-------|--------------|------------|----------------|----------|
| MOTOR-001 | 80% | 15 | 4 | 5000 |
| PUMP-002 | 65% | 25 | 3 | 3000 |

**Option A**: Separate maintenance

- Total downtime: 4 + 3 = 7 hours
- Total cost: 5000 + 3000 = $8000

**Option B**: Combined maintenance (can share shutdown)

- Total downtime: max(4, 3) = 4 hours (parallel work)
- Total cost: 5000 + 3000 - 1000 (shared setup) = $7000

**Decision**: Option B saves 3 hours and $1000.

---

## Document Control

**Version**: 1.0
**Published**: January 2025
**Next Review**: January 2026
**Owner**: WIA Technical Committee - Industrial Standards
**Contact**: standards@wia.org

### Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2024-10-01 | WIA TC | Initial draft |
| 0.5 | 2024-11-15 | WIA TC | Public comment period |
| 1.0 | 2025-01-01 | WIA TC | Official release |

### Contributors

- Dr. Jane Smith, Vibration Analysis Expert
- Prof. John Lee, ML and AI Specialist
- Michael Chen, Industrial IoT Architect
- Sarah Johnson, Reliability Engineer
- WIA-Official Development Team

### Acknowledgments

This standard was developed with input from industry partners, academic institutions, and the global maintenance community. Special thanks to all contributors and reviewers.

---

## License and Usage

This standard is published under the WIA Open Standard License:

- **Free to implement**: Organizations may implement this standard without licensing fees
- **Certification optional**: Certification provides assurance but is not required for use
- **Attribution required**: Implementations should reference WIA-IND-026
- **Derivative works**: Allowed with attribution
- **Commercial use**: Permitted

For certification, enterprise support, or custom implementations, contact: enterprise@wia.org

---

**弘益人間 (홍익인간) · Benefit All Humanity**

---

© 2025 World Certification Industry Association (WIA) / SmileStory Inc.

