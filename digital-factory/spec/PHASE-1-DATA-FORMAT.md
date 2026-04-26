# WIA-IND-028 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-028
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-028: Digital Factory Standard - Technical Specification v1.0

**Standard ID:** WIA-IND-028
**Version:** 1.0.0
**Status:** Active
**Release Date:** 2025-12-27
**Category:** Industry (IND)
**Emoji:** 🏭
**Color:** Amber (#F59E0B)

**Authors:**
WIA Industry Research Group
WIA Digital Transformation Committee

**License:** MIT

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Digital Twin Framework](#5-digital-twin-framework)
6. [Virtual Commissioning](#6-virtual-commissioning)
7. [Production Simulation](#7-production-simulation)
8. [Factory Layout Optimization](#8-factory-layout-optimization)
9. [Energy Management](#9-energy-management)
10. [Worker Safety Monitoring](#10-worker-safety-monitoring)
11. [AR/VR Training Systems](#11-arvr-training-systems)
12. [Real-time KPI Dashboards](#12-real-time-kpi-dashboards)
13. [Connected Worker Platform](#13-connected-worker-platform)
14. [Factory-as-a-Service](#14-factory-as-a-service)
15. [Data Models](#15-data-models)
16. [API Specifications](#16-api-specifications)
17. [Security](#17-security)
18. [Interoperability](#18-interoperability)
19. [Performance Requirements](#19-performance-requirements)
20. [Conformance](#20-conformance)

---

## 1. Introduction

### 1.1 Purpose

This standard defines a comprehensive framework for digital factory systems that enable manufacturers to create virtual representations of their physical facilities, optimize operations, train workers, and deliver manufacturing capabilities as a service.

### 1.2 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize digital transformation in manufacturing, enabling factories of all sizes to compete in the Industry 4.0 era.

### 1.3 Key Benefits

- **Reduced Time-to-Market**: Virtual commissioning reduces physical setup time by 30-50%
- **Operational Excellence**: Real-time optimization improves OEE by 10-25%
- **Energy Efficiency**: AI-driven energy management reduces costs by 15-30%
- **Worker Safety**: Predictive safety monitoring reduces incidents by 40-60%
- **Training Efficiency**: AR/VR training reduces training time by 50-70%
- **Cost Savings**: Overall operational cost reduction of 20-40%

---

## 2. Scope

### 2.1 Covered Systems

This standard covers:

1. **Digital Twin Systems**: Virtual factory representations
2. **Virtual Commissioning**: Pre-deployment validation
3. **Production Simulation**: What-if analysis and optimization
4. **Layout Optimization**: Spatial planning and workflow design
5. **Energy Management**: Real-time monitoring and optimization
6. **Safety Monitoring**: AI-powered worker safety tracking
7. **AR/VR Training**: Immersive learning environments
8. **KPI Dashboards**: Real-time operational metrics
9. **Connected Workers**: Mobile and wearable platforms
10. **Factory-as-a-Service**: Cloud-based manufacturing

### 2.2 Out of Scope

- Product design (covered by WIA-CAD standards)
- Supply chain management (covered by WIA-SCM standards)
- Quality management systems (covered by WIA-QMS standards)

---

## 3. Normative References

### 3.1 ISO Standards

- **ISO 23247-1:2021** - Automation systems and integration - Digital twin framework for manufacturing - Part 1: Overview and general principles
- **ISO 23247-2:2021** - Part 2: Reference architecture
- **ISO 23247-3:2021** - Part 3: Digital representation of manufacturing elements
- **ISO 50001:2018** - Energy management systems
- **ISO 45001:2018** - Occupational health and safety management systems

### 3.2 IEC Standards

- **IEC 62443** - Industrial communication networks - Network and system security
- **IEC 61499** - Function blocks for industrial-process measurement and control systems
- **IEC 62264** - Enterprise-control system integration

### 3.3 WIA Standards

- **WIA-IOT** - Industrial Internet of Things
- **WIA-AI** - Artificial Intelligence for Industry
- **WIA-AR** - Augmented Reality
- **WIA-VR** - Virtual Reality
- **WIA-CLOUD** - Cloud Computing Infrastructure
- **WIA-EDGE** - Edge Computing

---

## 4. Terms and Definitions

### 4.1 Digital Twin

A virtual representation of a physical factory that mirrors its behavior in real-time, enabling simulation, analysis, and optimization.

**Components:**
- **Physical Entity**: The real factory (machines, sensors, workers)
- **Virtual Entity**: The digital model (geometry, logic, data)
- **Connection**: Data exchange mechanism (IoT, APIs, databases)
- **Data**: Information flowing between physical and virtual

### 4.2 Virtual Commissioning

The process of testing and validating production systems in a virtual environment before physical implementation.

### 4.3 Production Simulation

Computer-based modeling of manufacturing processes to predict behavior, identify bottlenecks, and optimize performance.

### 4.4 Connected Worker

An employee equipped with digital tools (mobile apps, wearables, AR glasses) that enhance productivity and safety.

### 4.5 Factory-as-a-Service (FaaS)

A cloud-based business model where manufacturing capabilities are offered as on-demand services.

---

## 5. Digital Twin Framework

### 5.1 Architecture

The digital twin architecture consists of six layers:

```
┌─────────────────────────────────────┐
│    Application Layer                │
│    (Dashboards, Apps, Services)     │
├─────────────────────────────────────┤
│    Digital Twin Layer               │
│    (Virtual Factory Model)          │
├─────────────────────────────────────┤
│    Analytics Layer                  │
│    (AI/ML, Optimization)            │
├─────────────────────────────────────┤
│    Data Layer                       │
│    (Storage, Time-series DB)        │
├─────────────────────────────────────┤
│    Connectivity Layer               │
│    (IoT, Edge, Protocols)           │
├─────────────────────────────────────┤
│    Physical Layer                   │
│    (Factory, Machines, Sensors)     │
└─────────────────────────────────────┘
```

### 5.2 Digital Twin Types

#### 5.2.1 Component Twin
Represents individual equipment (e.g., robot arm, CNC machine)

**Attributes:**
- Geometry (3D model)
- Kinematic model
- Performance parameters
- Maintenance history
- Real-time sensor data

#### 5.2.2 Process Twin
Represents production processes (e.g., assembly, welding)

**Attributes:**
- Process flow
- Cycle times
- Quality parameters
- Resource requirements
- Historical performance

#### 5.2.3 System Twin
Represents complete production systems (e.g., assembly line)

**Attributes:**
- System topology
- Material flow
- Energy consumption
- OEE metrics
- Bottleneck analysis

#### 5.2.4 Factory Twin
Represents the entire facility

**Attributes:**
- Factory layout
- All equipment and systems
- Worker locations
- Energy systems
- Building infrastructure

### 5.3 Synchronization Methods

#### 5.3.1 Real-time Sync
- **Update Frequency**: 1-1000 Hz
- **Latency**: < 100 ms
- **Use Case**: Live monitoring, control

#### 5.3.2 Near-real-time Sync
- **Update Frequency**: 1-60 seconds
- **Latency**: < 5 seconds
- **Use Case**: Dashboards, analytics

#### 5.3.3 Batch Sync
- **Update Frequency**: Minutes to hours
- **Latency**: Minutes
- **Use Case**: Historical analysis, reporting

### 5.4 Data Sources

Digital twins aggregate data from:

1. **Sensors**: Temperature, vibration, pressure, position
2. **PLCs**: Machine states, production counts, alarms
3. **SCADA**: Process variables, control parameters
4. **MES**: Work orders, quality data, traceability
5. **ERP**: Inventory, schedules, costs
6. **Vision Systems**: Quality inspection, tracking
7. **Manual Input**: Operator observations, maintenance logs

### 5.5 Fidelity Levels

| Level | Name | Geometry | Behavior | Data | Use Case |
|-------|------|----------|----------|------|----------|
| L0 | Concept | Schematic | None | None | Planning |
| L1 | Basic | Simplified 3D | Static | Historical | Visualization |
| L2 | Functional | Detailed 3D | Kinematic | Real-time | Monitoring |
| L3 | High-fidelity | Accurate 3D | Dynamic | Real-time + AI | Optimization |
| L4 | Physics-based | Photo-real | Physics sim | Predictive | Virtual commissioning |

### 5.6 Predictive Capabilities

Digital twins shall support:

1. **Anomaly Detection**: Identify deviations from normal operation
2. **Predictive Maintenance**: Forecast equipment failures
3. **Quality Prediction**: Predict defects before they occur
4. **Energy Forecasting**: Predict energy consumption
5. **Throughput Prediction**: Forecast production output

**Accuracy Requirements:**
- Anomaly detection: > 95% true positive rate
- Predictive maintenance: > 90% accuracy, 7-30 days lead time
- Quality prediction: > 92% accuracy
- Energy forecasting: < 5% MAPE (Mean Absolute Percentage Error)

---

## 6. Virtual Commissioning

### 6.1 Purpose

Virtual commissioning enables testing of production systems in a risk-free virtual environment before physical deployment, reducing commissioning time and costs.

### 6.2 Workflow

```
1. Requirements → Define production requirements
2. Design → Create virtual factory model
3. Simulation → Build control logic and automation
4. Testing → Execute test scenarios
5. Validation → Verify against requirements
6. Optimization → Tune parameters
7. Documentation → Generate commissioning docs
8. Physical Build → Construct real system
9. Deployment → Transfer validated programs
10. Monitoring → Compare virtual vs. actual
```

### 6.3 Model Requirements

#### 6.3.1 Geometric Model
- **Format**: STEP, IGES, STL, glTF
- **Accuracy**: ±1 mm for critical dimensions
- **LOD**: Multiple levels of detail for performance

#### 6.3.2 Kinematic Model
- **Joint types**: Revolute, prismatic, cylindrical, spherical
- **Constraints**: Position limits, velocity limits, collision detection
- **Accuracy**: ±0.1° for angular positions, ±0.1 mm for linear

#### 6.3.3 Control Logic
- **Languages**: IEC 61131-3 (Ladder, ST, FBD), Python, C++
- **PLC Emulation**: Support for major PLC brands (Siemens, Allen-Bradley, etc.)
- **Cycle time**: Match real PLC scan times

#### 6.3.4 I/O Simulation
- **Digital I/O**: Binary sensors and actuators
- **Analog I/O**: Continuous sensors (4-20mA, 0-10V)
- **Network I/O**: Profinet, EtherNet/IP, Modbus TCP

### 6.4 Test Scenarios

Virtual commissioning shall include:

1. **Startup Test**: Initial power-on and homing
2. **Normal Operation**: Standard production cycle
3. **Edge Cases**: Boundary conditions, tolerances
4. **Fault Injection**: Sensor failures, component jams
5. **Emergency Stop**: E-stop response and recovery
6. **Changeover**: Product model changes
7. **Maintenance Mode**: Service and repair operations
8. **Performance Test**: Maximum throughput, cycle time

### 6.5 Validation Criteria

| Criterion | Requirement |
|-----------|-------------|
| Cycle time accuracy | ±5% of physical system |
| Motion path accuracy | ±2 mm deviation |
| Energy consumption | ±10% of actual |
| Throughput | ±5% of physical system |
| Safety response | 100% correct E-stop behavior |

### 6.6 Deliverables

1. **Virtual Model**: Complete digital twin
2. **Control Programs**: PLC/robot programs
3. **Test Results**: All test scenarios documented
4. **Performance Report**: Metrics and KPIs
5. **Commissioning Plan**: Physical deployment guide
6. **Training Materials**: Operator and maintenance guides

---

## 7. Production Simulation

### 7.1 Simulation Types

#### 7.1.1 Discrete Event Simulation (DES)
Models production as a sequence of discrete events.

**Applications:**
- Assembly line throughput
- Buffer sizing
- Scheduling optimization
- Queuing analysis

**Tools:** Arena, Simio, AnyLogic, FlexSim

#### 7.1.2 Agent-Based Simulation (ABS)
Models individual entities (workers, AGVs) with autonomous behavior.

**Applications:**
- Worker movements
- AGV routing
- Collaborative assembly
- Emergency evacuation

#### 7.1.3 System Dynamics (SD)
Models high-level system behavior using stocks and flows.

**Applications:**
- Production planning
- Inventory management
- Supply chain dynamics

#### 7.1.4 Physics Simulation
Models physical interactions with realistic physics.

**Applications:**
- Material handling
- Collision detection
- Robot path planning
- Part orientation

### 7.2 Simulation Inputs

1. **Factory Layout**: 2D/3D geometry
2. **Equipment**: Machines, robots, conveyors
3. **Process Times**: Cycle times, setup times
4. **Product Mix**: Demand forecasts
5. **Resources**: Workers, materials, energy
6. **Schedules**: Shifts, maintenance windows
7. **Stochastic Parameters**: Failure rates, quality yields

### 7.3 Simulation Outputs

1. **Throughput**: Units per hour/shift/day
2. **Utilization**: Equipment, worker, space utilization
3. **WIP**: Work-in-process inventory levels
4. **Cycle Time**: Production lead time
5. **Bottlenecks**: Constraint identification
6. **Queue Lengths**: Buffer occupancy
7. **Energy Consumption**: kWh per unit
8. **Cost**: Production cost breakdown

### 7.4 What-If Analysis

Simulations shall support scenario comparison:

```
Baseline Scenario:
- Current layout
- Existing equipment
- Current demand

Alternative Scenarios:
1. Add 2 robots → +15% throughput
2. Redesign layout → -20% material handling
3. Add shift → +40% capacity
4. Upgrade machine → +10% quality
```

### 7.5 Optimization

#### 7.5.1 Optimization Objectives
- Maximize throughput
- Minimize cycle time
- Minimize WIP
- Minimize energy consumption
- Maximize equipment utilization
- Minimize cost

#### 7.5.2 Optimization Methods
- Genetic algorithms
- Simulated annealing
- Particle swarm optimization
- Gradient descent
- Reinforcement learning

#### 7.5.3 Multi-objective Optimization
Support Pareto-optimal solutions balancing multiple objectives.

### 7.6 Validation

Simulation models shall be validated against:

1. **Historical Data**: Compare to actual production data
2. **Expert Judgment**: Review by subject matter experts
3. **Sensitivity Analysis**: Test parameter variations
4. **Statistical Tests**: Hypothesis testing, confidence intervals

**Acceptance Criteria:**
- Throughput within ±5% of actual
- Cycle time within ±10% of actual
- Utilization within ±8% of actual

---


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
