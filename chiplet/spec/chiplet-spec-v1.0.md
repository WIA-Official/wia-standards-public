# WIA-SEMI-004: Chiplet Standard - Technical Specification v1.0

> **弘益人間 · Benefit All Humanity**

**Document Version**: 1.0.0
**Status**: Production Ready
**Published**: December 27, 2025
**Organization**: World Certification Industry Association (WIA)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope and Objectives](#2-scope-and-objectives)
3. [Terminology and Definitions](#3-terminology-and-definitions)
4. [Architecture Overview](#4-architecture-overview)
5. [Data Format Specification](#5-data-format-specification)
6. [API Interface Specification](#6-api-interface-specification)
7. [Design Protocol Specification](#7-design-protocol-specification)
8. [Integration Specification](#8-integration-specification)
9. [Compliance and Conformance](#9-compliance-and-conformance)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

The WIA Chiplet Standard (WIA-SEMI-004) defines a comprehensive framework for the design, integration, and deployment of modular semiconductor chiplet architectures. This specification enables heterogeneous integration of dies from multiple vendors and process technologies while ensuring interoperability, performance, and reliability.

### 1.2 Background

As semiconductor manufacturing approaches physical and economic limits, the industry requires innovative approaches to continue performance scaling. Chiplet architectures, where multiple smaller dies are integrated in a single package, offer:

- Improved yield through smaller die sizes
- Heterogeneous integration of different process technologies
- Reduced time-to-market through IP reuse
- Cost optimization through modular design
- Scalable performance through multi-die configurations

### 1.3 Design Philosophy

This standard embodies the principle of **弘益人間 (Hongik Ingan)** - "Benefit All Humanity" - by:

- Providing open, vendor-neutral specifications
- Enabling broad industry participation
- Reducing barriers to advanced semiconductor technology
- Fostering innovation through standardization
- Supporting sustainable semiconductor manufacturing

---

## 2. Scope and Objectives

### 2.1 Scope

This specification covers:

- **Data Formats**: JSON schemas for chiplet metadata and design exchange
- **API Interfaces**: Programming interfaces for chiplet management
- **Design Protocols**: UCIe-compliant electrical and protocol specifications
- **Integration Guidelines**: Multi-die package integration methodologies

### 2.2 Objectives

1. Establish interoperable chiplet interfaces across vendors
2. Define standardized data exchange formats
3. Specify performance and reliability requirements
4. Enable ecosystem development through open APIs
5. Provide verification and validation methodologies

### 2.3 Out of Scope

The following are explicitly out of scope:

- Specific foundry manufacturing processes
- Proprietary IP core designs
- End-application software
- Business models and licensing terms

---

## 3. Terminology and Definitions

### 3.1 Core Terms

**Chiplet**: A modular integrated circuit die designed for integration with other chiplets in a single package.

**UCIe (Universal Chiplet Interconnect Express)**: Industry-standard die-to-die interconnect specification.

**Die-to-Die (D2D)**: Communication interface between chiplets within a package.

**2.5D Integration**: Package technology using an interposer layer between chiplets and substrate.

**3D Integration**: Vertical stacking of dies using through-silicon vias (TSVs).

**Known Good Die (KGD)**: Tested die verified to meet specifications before package integration.

**Interposer**: Intermediate layer providing electrical routing between chiplets and package substrate.

### 3.2 Acronyms

| Acronym | Definition |
|---------|------------|
| AI | Artificial Intelligence |
| BIST | Built-In Self-Test |
| CXL | Compute Express Link |
| DFT | Design-For-Test |
| DTM | Dynamic Thermal Management |
| DVFS | Dynamic Voltage and Frequency Scaling |
| EDA | Electronic Design Automation |
| HBM | High Bandwidth Memory |
| KGD | Known Good Die |
| OSAT | Outsourced Semiconductor Assembly and Test |
| PCIe | Peripheral Component Interconnect Express |
| PDN | Power Delivery Network |
| PHY | Physical Layer |
| QoS | Quality of Service |
| SoC | System-on-Chip |
| TSV | Through-Silicon Via |
| UCIe | Universal Chiplet Interconnect Express |

---

## 4. Architecture Overview

### 4.1 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Application Layer                          │
├─────────────────────────────────────────────────────────────┤
│                Chiplet Abstraction Layer (CAL)               │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │  Chiplet 1   │  │  Chiplet 2   │  │  Chiplet N   │     │
│  │              │  │              │  │              │     │
│  │  [UCIe PHY]  │  │  [UCIe PHY]  │  │  [UCIe PHY]  │     │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘     │
│         │                  │                  │             │
│         └──────────────────┴──────────────────┘             │
│                    Interposer Layer                         │
├─────────────────────────────────────────────────────────────┤
│                   Package Substrate                          │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Four-Phase Framework

#### Phase 1: Data Format
Standardized JSON schemas for chiplet specifications, configurations, and design data exchange.

#### Phase 2: API Interface
TypeScript/JavaScript SDK providing programmatic access to chiplet functionality.

#### Phase 3: Design Protocol
UCIe-compliant electrical specifications and communication protocols.

#### Phase 4: Integration
Multi-die package integration guidelines and methodologies.

---

## 5. Data Format Specification

### 5.1 Chiplet Metadata Schema

```json
{
  "$schema": "https://wia.org/schemas/chiplet-v1.0.json",
  "type": "object",
  "required": ["id", "type", "version", "vendor", "technology", "interfaces"],
  "properties": {
    "id": {
      "type": "string",
      "description": "Unique chiplet identifier"
    },
    "type": {
      "type": "string",
      "enum": ["processor", "memory", "io", "accelerator", "custom"]
    },
    "version": {
      "type": "string",
      "pattern": "^[0-9]+\\.[0-9]+\\.[0-9]+$"
    },
    "vendor": {
      "type": "string",
      "description": "Chiplet manufacturer"
    },
    "technology": {
      "type": "object",
      "properties": {
        "processNode": {
          "type": "string",
          "description": "Manufacturing process node (e.g., '3nm', '5nm')"
        },
        "dieSize": {
          "type": "object",
          "properties": {
            "width": { "type": "number", "minimum": 0 },
            "height": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "enum": ["mm", "um"] }
          }
        }
      }
    },
    "interfaces": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/interface"
      }
    },
    "power": {
      "type": "object",
      "properties": {
        "tdp": {
          "type": "number",
          "description": "Thermal Design Power in Watts"
        },
        "idle": {
          "type": "number",
          "description": "Idle power in Watts"
        },
        "voltageRanges": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "domain": { "type": "string" },
              "min": { "type": "number" },
              "max": { "type": "number" }
            }
          }
        }
      }
    },
    "thermal": {
      "type": "object",
      "properties": {
        "tjMax": {
          "type": "number",
          "description": "Maximum junction temperature in Celsius"
        },
        "thermalResistance": {
          "type": "number",
          "description": "Junction-to-case thermal resistance in °C/W"
        }
      }
    }
  },
  "definitions": {
    "interface": {
      "type": "object",
      "required": ["type", "version", "lanes"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["UCIe", "proprietary"]
        },
        "version": { "type": "string" },
        "lanes": { "type": "integer", "minimum": 1 },
        "protocol": {
          "type": "string",
          "enum": ["raw", "streaming", "pcie", "cxl"]
        },
        "bandwidth": {
          "type": "number",
          "description": "Maximum bandwidth in GB/s"
        }
      }
    }
  }
}
```

### 5.2 Configuration Schema

```json
{
  "configuration": {
    "system": {
      "name": "ExampleChipletSystem",
      "version": "1.0.0"
    },
    "chiplets": [
      {
        "id": "compute-001",
        "position": { "x": 0, "y": 0 },
        "powerMode": "balanced",
        "frequencyMHz": 3500
      },
      {
        "id": "memory-001",
        "position": { "x": 10, "y": 0 },
        "powerMode": "performance",
        "capacity": "32GB"
      }
    ],
    "connections": [
      {
        "source": "compute-001",
        "destination": "memory-001",
        "interface": "UCIe",
        "lanes": 32
      }
    ]
  }
}
```

### 5.3 Data Exchange Format

All chiplet data exchange SHALL use UTF-8 encoded JSON with the following requirements:

- **Validation**: All JSON documents MUST validate against published schemas
- **Versioning**: Schema version MUST be specified in `$schema` field
- **Extensions**: Vendor-specific extensions MUST use namespaced property names
- **Compression**: Optional gzip compression for large documents

---

## 6. API Interface Specification

### 6.1 Chiplet Management API

#### 6.1.1 Chiplet Discovery

```typescript
interface ChipletManager {
  /**
   * Discover all chiplets in the system
   * @returns Array of discovered chiplets
   */
  discoverChiplets(): Promise<Chiplet[]>;

  /**
   * Get chiplet by ID
   * @param id Chiplet identifier
   */
  getChiplet(id: string): Promise<Chiplet | null>;

  /**
   * Enumerate chiplets by type
   * @param type Chiplet type filter
   */
  getChipletsByType(type: ChipletType): Promise<Chiplet[]>;
}
```

#### 6.1.2 Configuration API

```typescript
interface ChipletConfiguration {
  /**
   * Configure chiplet parameters
   * @param id Chiplet identifier
   * @param config Configuration object
   */
  configure(id: string, config: ChipletConfig): Promise<void>;

  /**
   * Get current configuration
   * @param id Chiplet identifier
   */
  getConfiguration(id: string): Promise<ChipletConfig>;

  /**
   * Validate configuration
   * @param config Configuration to validate
   */
  validateConfiguration(config: ChipletConfig): Promise<ValidationResult>;
}
```

#### 6.1.3 Performance Monitoring API

```typescript
interface PerformanceMonitor {
  /**
   * Get real-time performance metrics
   * @param id Chiplet identifier
   */
  getMetrics(id: string): Promise<PerformanceMetrics>;

  /**
   * Subscribe to performance events
   * @param id Chiplet identifier
   * @param callback Event handler
   */
  subscribe(id: string, callback: (metrics: PerformanceMetrics) => void): void;
}
```

### 6.2 Error Handling

All API methods SHALL implement consistent error handling:

```typescript
enum ChipletErrorCode {
  NOT_FOUND = 'CHIPLET_NOT_FOUND',
  INVALID_CONFIG = 'INVALID_CONFIGURATION',
  COMMUNICATION_ERROR = 'COMMUNICATION_ERROR',
  POWER_ERROR = 'POWER_ERROR',
  THERMAL_ERROR = 'THERMAL_ERROR'
}

interface ChipletError extends Error {
  code: ChipletErrorCode;
  chipletId?: string;
  details?: any;
}
```

---

## 7. Design Protocol Specification

### 7.1 UCIe Compliance

This standard requires full compliance with UCIe Specification v1.1 or later, including:

#### 7.1.1 Physical Layer

**Standard Package**:
- Bump pitch: 55 μm
- Voltage range: 0.4V - 1.0V
- Maximum reach: 25mm on silicon interposer

**Advanced Package**:
- Bump pitch: 25 μm
- Voltage range: 0.4V - 0.8V
- Maximum reach: 15mm on silicon interposer

#### 7.1.2 Die-to-Die Adapter

**Required Components**:
- UCIe PHY (Physical Layer)
- UCIe Controller (Link Layer)
- Protocol Adapter (Protocol Layer)

**Lane Configurations**:
- Minimum: 16 lanes
- Standard: 32 lanes
- High-bandwidth: 64 lanes

#### 7.1.3 Protocol Layer

**Supported Protocols**:
1. **Raw Mode**: Direct memory-mapped access
2. **Streaming Protocol**: High-throughput data transfer
3. **PCIe Mapping**: PCIe 5.0 compatibility
4. **CXL Mapping**: CXL 2.0/3.0 cache coherence

### 7.2 Electrical Specifications

#### 7.2.1 Signal Integrity

| Parameter | Minimum | Typical | Maximum | Unit |
|-----------|---------|---------|---------|------|
| Eye Height | 60 | 75 | - | mV |
| Eye Width | 0.6 | 0.7 | - | UI |
| Jitter (RJ) | - | 2 | 5 | ps |
| Skew | - | 5 | 10 | ps |

#### 7.2.2 Power Delivery

| Parameter | Specification |
|-----------|--------------|
| Supply Voltage | 0.75V ± 5% |
| Current per Lane | ≤ 50mA |
| Decoupling | Per UCIe spec |
| PDN Impedance | < 5mΩ @ DC |

### 7.3 Performance Requirements

#### 7.3.1 Bandwidth

| Lane Count | Minimum Bandwidth | Typical Bandwidth |
|------------|------------------|-------------------|
| 16 lanes | 128 GB/s | 192 GB/s |
| 32 lanes | 256 GB/s | 384 GB/s |
| 64 lanes | 512 GB/s | 768 GB/s |

#### 7.3.2 Latency

| Metric | Target | Maximum |
|--------|--------|---------|
| D2D Latency | < 4ns | 10ns |
| Protocol Overhead | < 2ns | 5ns |
| Total Link Latency | < 6ns | 15ns |

#### 7.3.3 Energy Efficiency

| Metric | Target |
|--------|--------|
| Energy per Bit | < 0.5 pJ/bit |
| Idle Power per Lane | < 10 mW |
| Active Power per Lane | < 50 mW |

---

## 8. Integration Specification

### 8.1 Package Technologies

#### 8.1.1 2.5D Integration

**Interposer Requirements**:
- Material: Silicon, glass, or organic
- Metal layers: Minimum 4
- Routing pitch: ≤ 2 μm (silicon), ≤ 10 μm (organic)
- TSV pitch: ≥ 10 μm (if applicable)

**Assembly Process**:
1. Chiplet die attach to interposer
2. Micro-bump bonding (C4 or copper pillar)
3. Underfill application
4. Interposer attach to substrate
5. Package encapsulation

#### 8.1.2 3D Integration

**Stacking Requirements**:
- TSV diameter: 5-10 μm
- TSV pitch: ≥ 20 μm
- Maximum stack height: 4 dies
- Bonding: Hybrid bonding or micro-bumps

### 8.2 Thermal Management

#### 8.2.1 Thermal Requirements

| Parameter | Specification |
|-----------|--------------|
| Maximum Junction Temp | 100°C |
| Operating Range | 0°C - 85°C |
| Thermal Resistance | < 0.3 °C/W (junction-to-case) |

#### 8.2.2 Thermal Monitoring

**Required Capabilities**:
- Junction temperature sensing (±3°C accuracy)
- Dynamic thermal management (DTM)
- Thermal throttling at 95°C
- Emergency shutdown at 105°C

### 8.3 Testing and Validation

#### 8.3.1 Known Good Die (KGD) Testing

**Mandatory Tests**:
1. Functional test coverage ≥ 95%
2. Parametric testing (voltage, frequency)
3. Interface compliance testing
4. Burn-in testing (optional, recommended)

#### 8.3.2 System-Level Testing

**Integration Tests**:
1. Multi-chiplet functional testing
2. Interface interoperability testing
3. Performance benchmarking
4. Reliability testing (HTOL, TC, HAST)

---

## 9. Compliance and Conformance

### 9.1 Compliance Levels

#### Level 1: Basic Compliance
- Data format conformance
- Minimum API implementation
- UCIe physical layer compliance

#### Level 2: Standard Compliance
- Full API implementation
- UCIe protocol layer compliance
- Performance requirements met

#### Level 3: Advanced Compliance
- Extended API features
- Advanced package technologies
- Industry-leading performance

### 9.2 Certification Process

1. **Self-Certification**: Vendor testing against specification
2. **Documentation**: Compliance report submission
3. **Verification**: WIA review and verification
4. **Certification**: Certificate issuance
5. **Listing**: Publication in WIA chiplet directory

### 9.3 Conformance Testing

**Required Test Suites**:
- Data format validation suite
- API compliance test suite
- UCIe interoperability tests
- Performance benchmark suite

---

## 10. References

### 10.1 Normative References

1. UCIe Consortium, "UCIe Specification Version 1.1," 2023
2. PCI-SIG, "PCIe Base Specification Revision 5.0," 2019
3. CXL Consortium, "Compute Express Link Specification Revision 3.0," 2022
4. JEDEC, "High Bandwidth Memory (HBM3) JESD238," 2023

### 10.2 Informative References

1. IEEE 1838-2019, "IEEE Standard for Test Access Architecture for Three-Dimensional Stacked Integrated Circuits"
2. SEMI MS1, "Specification for Polished Monocrystalline Silicon Wafers"
3. IPC-2581, "Generic Requirements for Printed Board Assembly Products Manufacturing Description Data"

### 10.3 WIA Standards

- WIA-OMNI-API: Universal API Framework
- WIA-INTENT: Intent Expression Standard
- WIA-AIR-POWER: Distributed Computing Standard
- WIA-AIR-SHIELD: Security Framework Standard

---

## Appendix A: JSON Schema Examples

See Section 5 for complete JSON schema definitions.

## Appendix B: Code Examples

See TypeScript SDK documentation for implementation examples.

## Appendix C: Compliance Checklist

Available at: https://wia.org/chiplet/compliance-checklist

---

**Document Control**

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-27 | Initial release | WIA Standards Committee |

**License**: Creative Commons Attribution 4.0 International (CC BY 4.0)

**Copyright**: © 2025 SmileStory Inc. / World Certification Industry Association

---

弘益人間 · Benefit All Humanity
