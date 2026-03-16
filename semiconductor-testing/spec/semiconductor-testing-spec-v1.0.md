# WIA-SEMI-002: Semiconductor Testing Standard
## Technical Specification v1.0

**Status:** Draft
**Version:** 1.0.0
**Date:** January 2025
**Authors:** WIA Technical Committee on Semiconductor Testing

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Data Format Specification](#5-data-format-specification)
6. [API Interface Specification](#6-api-interface-specification)
7. [Test Protocol Specification](#7-test-protocol-specification)
8. [Quality Metrics](#8-quality-metrics)
9. [Compliance Requirements](#9-compliance-requirements)
10. [Security and Privacy](#10-security-and-privacy)

---

## 1. Introduction

### 1.1 Purpose

The WIA-SEMI-002 standard defines a comprehensive framework for semiconductor testing operations, encompassing wafer probe testing, package final testing, and quality assurance processes. This standard ensures interoperability, data consistency, and quality across the global semiconductor manufacturing ecosystem.

### 1.2 Background

Modern semiconductor manufacturing involves complex supply chains spanning multiple countries and companies. Standardized testing protocols, data formats, and quality metrics are essential to ensure device quality and enable efficient manufacturing operations.

### 1.3 Design Principles

- **Vendor Neutrality:** Compatible with all major ATE platforms
- **Data Integrity:** Comprehensive traceability from wafer to package
- **Scalability:** Support from legacy to cutting-edge technology nodes
- **Extensibility:** Framework for future technology integration
- **Security:** Protection of sensitive manufacturing data

---

## 2. Scope

### 2.1 Inclusions

This standard covers:

- Wafer probe testing data formats and protocols
- Package final testing methodologies
- ATE equipment control interfaces
- Test data management and archival
- Quality metrics and yield analysis
- Integration with Manufacturing Execution Systems (MES)

### 2.2 Exclusions

This standard does not cover:

- Wafer fabrication processes
- Package assembly operations
- Specific test program development
- Equipment maintenance procedures

---

## 3. Normative References

The following standards are referenced in this document:

- **SEMI E5:** Standard Test Data Format (STDF) Specification
- **JEDEC Standard 51:** Methodology for Thermal Measurement
- **IPC/JEDEC J-STD-020:** Moisture/Reflow Sensitivity Classification
- **SEMI E10:** Standard for Definition and Measurement of Equipment Reliability
- **ISO 9001:** Quality Management Systems

---

## 4. Terms and Definitions

### 4.1 Test-Related Terms

**ATE (Automatic Test Equipment):** Computer-controlled equipment used to test semiconductor devices.

**Wafer Probe:** Testing process performed on silicon wafers before packaging.

**Final Test:** Testing performed on packaged devices.

**Bin:** Classification category for devices based on test results.

**Parametric Test:** Measurement of electrical characteristics such as voltage, current, and timing.

**Functional Test:** Verification that a device performs its intended logical operations.

**Burn-In:** Elevated temperature/voltage stress test to accelerate early failures.

### 4.2 Data Terms

**STDF (Standard Test Data Format):** Industry-standard binary format for test data.

**Wafer Map:** Graphical representation of die test results on a wafer.

**Yield:** Percentage of devices passing test criteria.

**DPM (Defects Per Million):** Quality metric measuring defect density.

---

## 5. Data Format Specification

### 5.1 Standard Test Data Format (STDF)

All test data SHALL be stored in STDF v4 format with the following extensions:

#### 5.1.1 Required Records

- **FAR (File Attributes Record):** File version and CPU type
- **MIR (Master Information Record):** Test session metadata
- **WIR (Wafer Information Record):** Wafer identification
- **WRR (Wafer Results Record):** Wafer-level summary
- **PIR (Part Information Record):** Device identification
- **PTR (Parametric Test Record):** Parametric measurements
- **FTR (Functional Test Record):** Functional test results
- **PRR (Part Results Record):** Overall device results
- **MRR (Master Results Record):** File completion

#### 5.1.2 WIA Extensions

Additional record types for modern requirements:

```
WIA_TR (Traceability Record)
  - Equipment serial number
  - Operator ID
  - Test program version
  - Environmental conditions

WIA_AR (Analytics Record)
  - Statistical summaries
  - Yield predictions
  - Defect patterns
```

### 5.2 Wafer Map Format

Wafer maps SHALL use the following structure:

```typescript
interface WaferMap {
  waferId: string;
  lotId: string;
  waferSize: number; // diameter in mm
  orientation: 'flat' | 'notch';
  dieSize: { width: number; height: number }; // in mm
  dies: Die[];
}

interface Die {
  x: number; // die coordinate X
  y: number; // die coordinate Y
  binCode: number;
  testTime: number; // in milliseconds
  parametricResults?: ParametricResult[];
}
```

### 5.3 Real-Time Data Streaming

For real-time applications, data SHALL be streamed using:

- **Protocol:** WebSocket or gRPC
- **Format:** JSON or Protocol Buffers
- **Latency:** < 100ms for critical data
- **Buffering:** Maximum 1000 records or 1 second

---

## 6. API Interface Specification

### 6.1 ATE Control API

#### 6.1.1 Equipment Connection

```typescript
interface ATEConnection {
  connect(config: ATEConfig): Promise<ATESession>;
  disconnect(): Promise<void>;
  getStatus(): Promise<ATEStatus>;
}

interface ATEConfig {
  equipmentId: string;
  model: string;
  ipAddress: string;
  port: number;
  authentication: AuthCredentials;
}
```

#### 6.1.2 Test Execution

```typescript
interface TestExecution {
  loadProgram(programId: string): Promise<void>;
  startTest(config: TestConfig): Promise<TestSession>;
  pauseTest(): Promise<void>;
  resumeTest(): Promise<void>;
  stopTest(): Promise<TestResults>;
}

interface TestConfig {
  deviceType: string;
  testMode: 'parametric' | 'functional' | 'full';
  sites: number[];
  temperature?: number;
  voltage?: number;
}
```

#### 6.1.3 Data Collection

```typescript
interface DataCollection {
  streamResults(callback: (result: TestResult) => void): void;
  getResults(filter: ResultFilter): Promise<TestResult[]>;
  exportResults(format: 'STDF' | 'CSV' | 'JSON'): Promise<Blob>;
}
```

### 6.2 MES Integration API

```typescript
interface MESIntegration {
  reportWaferStart(waferInfo: WaferInfo): Promise<void>;
  reportWaferComplete(results: WaferResults): Promise<void>;
  updateLotStatus(lotId: string, status: LotStatus): Promise<void>;
  querySpecifications(deviceType: string): Promise<TestSpecs>;
}
```

---

## 7. Test Protocol Specification

### 7.1 Wafer Probe Testing

#### 7.1.1 Parametric Tests

Required parametric measurements:

1. **Threshold Voltage (Vth)**
   - Test condition: VDS = 0.1V, VGS sweep
   - Specification: ±10% of nominal
   - Measurement accuracy: ±1mV

2. **Leakage Current (Ileak)**
   - Test condition: VGS = 0V, VDS = nominal
   - Specification: < 100nA per device
   - Measurement accuracy: ±1nA

3. **Drive Current (Idrive)**
   - Test condition: VGS = VDS = nominal
   - Specification: ±15% of nominal
   - Measurement accuracy: ±0.1%

#### 7.1.2 Functional Tests

Functional test requirements:

- **Pattern Coverage:** Minimum 95% stuck-at fault coverage
- **At-Speed Testing:** ≥ 90% of nominal frequency
- **Scan Chain Testing:** Full scan chain integrity verification
- **Memory Testing:** March C- or equivalent algorithm

### 7.2 Package Testing

#### 7.2.1 Continuity Testing

All package pins SHALL be tested for:
- Pin-to-die connectivity
- Inter-pin isolation (> 1MΩ)
- Pin resistance (< 10Ω)

#### 7.2.2 Environmental Testing

Tests SHALL be performed at:
- **Cold:** -40°C or device minimum
- **Nominal:** 25°C
- **Hot:** 125°C or device maximum

Voltage corners:
- **Min:** VDD - 10%
- **Nominal:** VDD
- **Max:** VDD + 10%

#### 7.2.3 Burn-In Protocol

Standard burn-in conditions:
- **Temperature:** 125°C
- **Voltage:** VDD + 10%
- **Duration:** 48-168 hours
- **Activity:** Dynamic pattern (50% toggle rate)
- **Monitoring:** Continuous with < 1 hour intervals

### 7.3 Binning Specification

Standard bin codes:

| Bin | Description | Disposition |
|-----|-------------|-------------|
| 0 | Fail | Scrap |
| 1 | Pass - Full spec | Ship |
| 2 | Pass - Speed bin 2 | Ship |
| 3 | Pass - Speed bin 3 | Ship |
| 4-8 | Pass - Reduced spec | Ship with restrictions |
| 9 | Retest required | Retest |

---

## 8. Quality Metrics

### 8.1 Yield Metrics

**Wafer Probe Yield:**
```
Yield = (Good Dies / Total Dies Tested) × 100%
```

**Final Test Yield:**
```
Yield = (Passing Devices / Total Devices Tested) × 100%
```

**Cumulative Yield:**
```
Cumulative Yield = Probe Yield × Final Yield
```

### 8.2 Defect Density

**Defects Per Million (DPM):**
```
DPM = (Failed Units / Total Units) × 1,000,000
```

Target: < 100 DPM for production

### 8.3 Equipment Metrics

**Equipment Uptime:**
```
Uptime = (Available Time - Downtime) / Available Time × 100%
```

Target: > 95%

**Mean Time Between Failures (MTBF):**
```
MTBF = Total Operating Time / Number of Failures
```

Target: > 720 hours

---

## 9. Compliance Requirements

### 9.1 Implementation Levels

**Level 1 (Basic):**
- STDF v4 support
- Basic parametric testing
- Single-site testing
- Manual data export

**Level 2 (Standard):**
- Real-time data streaming
- Multi-site testing
- Automated binning
- MES integration

**Level 3 (Advanced):**
- AI-powered analytics
- Predictive maintenance
- Adaptive test optimization
- Full automation

### 9.2 Certification Process

Organizations claiming WIA-SEMI-002 compliance SHALL:

1. Submit implementation documentation
2. Pass certification test suite
3. Demonstrate interoperability
4. Maintain compliance through annual audits

---

## 10. Security and Privacy

### 10.1 Data Protection

Test data SHALL be protected through:

- **Encryption:** AES-256 for data at rest
- **Transport Security:** TLS 1.3 for data in transit
- **Access Control:** Role-based authentication
- **Audit Logging:** Comprehensive activity tracking

### 10.2 Intellectual Property Protection

Sensitive device characteristics SHALL be:
- Stored in secure enclaves
- Accessed only by authorized personnel
- Never transmitted in clear text
- Subject to data retention policies

---

## Appendix A: Reference Implementation

See TypeScript SDK in `/api/typescript/` for reference implementation.

## Appendix B: Test Programs

Sample test programs available at: https://github.com/WIA-Official/wia-semi-002-samples

## Appendix C: Compliance Checklist

Detailed compliance checklist available separately.

---

## Document History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-01 | Initial release | WIA Technical Committee |

---

## 弘益人間 (홍익인간) · Benefit All Humanity

*Ensuring quality and reliability in every semiconductor device through standardized testing protocols.*

---

© 2025 WIA (World Certification Industry Association) · SmileStory Inc.
