# WIA-IND-028 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-028
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 8. Factory Layout Optimization

### 8.1 Layout Objectives

1. **Minimize Material Handling**: Reduce distance traveled
2. **Maximize Workflow**: Optimize process flow
3. **Minimize Footprint**: Reduce required floor space
4. **Improve Safety**: Separate hazardous areas
5. **Enable Flexibility**: Support future changes
6. **Optimize Energy**: Cluster energy-intensive equipment

### 8.2 Layout Types

#### 8.2.1 Process Layout
Group similar equipment together.

**Advantages:**
- Flexibility for varied products
- High equipment utilization

**Disadvantages:**
- High material handling
- Complex routing

#### 8.2.2 Product Layout
Arrange equipment in production sequence.

**Advantages:**
- Low material handling
- Simple workflow

**Disadvantages:**
- Low flexibility
- Line balancing challenges

#### 8.2.3 Cellular Layout
Group equipment for product families.

**Advantages:**
- Balanced flexibility and efficiency
- Reduced WIP

**Disadvantages:**
- Limited to similar products

#### 8.2.4 Hybrid Layout
Combination of above types.

### 8.3 Layout Constraints

1. **Space Constraints**: Building dimensions, columns, walls
2. **Safety Constraints**: Clearances, emergency exits, fire codes
3. **Utility Constraints**: Power, compressed air, water access
4. **Ergonomic Constraints**: Worker reach, sight lines, accessibility
5. **Environmental Constraints**: Noise, vibration, temperature

### 8.4 Layout Optimization Methods

#### 8.4.1 Systematic Layout Planning (SLP)
Six-step methodology:

1. Material flow analysis
2. Activity relationship diagram
3. Space requirements
4. Space relationship diagram
5. Space alternatives
6. Layout evaluation

#### 8.4.2 Mathematical Optimization

**Quadratic Assignment Problem (QAP):**
Minimize total material handling cost.

```
Minimize: Σ Σ f[i,j] * d[p,q]

Where:
  f[i,j] = material flow between departments i and j
  d[p,q] = distance between locations p and q
```

#### 8.4.3 AI-Based Optimization

Use genetic algorithms, simulated annealing, or deep reinforcement learning.

**Algorithm:**
1. Generate initial population of layouts
2. Evaluate fitness (objectives)
3. Select best layouts
4. Apply genetic operators (crossover, mutation)
5. Repeat until convergence

### 8.5 Layout Evaluation Metrics

| Metric | Formula | Target |
|--------|---------|--------|
| Material Handling Distance | Σ (flow × distance) | Minimize |
| Workflow Score | Adjacency score | Maximize |
| Space Utilization | Used area / Total area | > 70% |
| Safety Score | Hazard separation | > 90 |
| Flexibility Index | Reconfiguration cost | Minimize |

### 8.6 3D Layout Visualization

Support for:
- Interactive 3D viewing
- VR walkthroughs
- AR overlays on real space
- Animation of material flow
- Collision detection
- Line-of-sight analysis

---

## 9. Energy Management

### 9.1 Energy Monitoring

#### 9.1.1 Measurement Points
- **Main Meter**: Total facility power
- **Sub-meters**: By production line, equipment, department
- **Utility Meters**: Compressed air, HVAC, lighting
- **Renewable Sources**: Solar, wind generation

#### 9.1.2 Measurement Frequency
- **Real-time**: 1-second intervals
- **Aggregated**: 15-minute intervals (standard utility billing)
- **Historical**: Long-term storage (5+ years)

#### 9.1.3 Measurement Accuracy
- **Revenue-grade meters**: ±0.2% accuracy (IEC 62053)
- **Sub-meters**: ±1% accuracy
- **Transducers**: ±2% accuracy

### 9.2 Energy Metrics

#### 9.2.1 Total Energy Consumption
```
Total Energy (kWh) = ∫ Power(t) dt
```

#### 9.2.2 Energy Intensity
```
Energy Intensity = Total Energy / Units Produced
```

**Benchmark:** < 5 kWh per unit (varies by industry)

#### 9.2.3 Peak Demand
```
Peak Demand (kW) = max(Power(t))
```

**Importance:** Utilities charge for peak demand (demand charges)

#### 9.2.4 Power Factor
```
Power Factor = Real Power / Apparent Power
```

**Target:** > 0.95 (avoid utility penalties)

#### 9.2.5 Specific Energy Consumption (SEC)
Energy per unit of production output.

**Formula:**
```
SEC = Energy Consumed / Production Volume
```

### 9.3 Energy Optimization Strategies

#### 9.3.1 Load Shifting
Shift energy-intensive operations to off-peak hours.

**Savings:** 20-30% on electricity costs

#### 9.3.2 Demand Response
Reduce load during peak demand events.

**Incentives:** Utility rebates, reduced demand charges

#### 9.3.3 Equipment Scheduling
Optimize start/stop times of equipment.

**Example:**
- Avoid simultaneous startup (reduce peak)
- Schedule maintenance during low-demand periods

#### 9.3.4 Energy-Efficient Equipment
Replace with high-efficiency motors, drives, compressors.

**ROI:** Typically 2-4 years

#### 9.3.5 Renewable Integration
Integrate solar, wind, battery storage.

**Benefits:**
- Reduced grid dependence
- Lower carbon footprint
- Energy cost stability

#### 9.3.6 Waste Heat Recovery
Capture and reuse waste heat.

**Applications:** Preheating, space heating, hot water

### 9.4 AI-Powered Energy Optimization

#### 9.4.1 Predictive Models
Machine learning models to forecast energy consumption.

**Inputs:**
- Historical energy data
- Production schedule
- Weather forecast
- Equipment status

**Accuracy Target:** < 5% MAPE

#### 9.4.2 Prescriptive Analytics
Recommend actions to reduce energy consumption.

**Actions:**
- Adjust HVAC setpoints
- Reschedule production
- Start/stop equipment
- Engage battery storage

#### 9.4.3 Reinforcement Learning
Continuously learn optimal control strategies.

**Reward Function:**
```
Reward = -Cost - α × Emissions - β × Comfort_Penalty
```

### 9.5 Energy Dashboards

Real-time visualization of:

1. **Current Consumption**: Live power usage
2. **Cost**: Real-time energy cost
3. **Trends**: Historical consumption patterns
4. **Benchmarks**: Comparison to targets
5. **Alerts**: Anomalies, threshold violations
6. **Recommendations**: AI-generated savings opportunities

### 9.6 Compliance and Reporting

Support for:

- **ISO 50001**: Energy management systems
- **Carbon Reporting**: GHG Protocol, CDP
- **Utility Reporting**: Demand response events
- **Sustainability Reporting**: ESG metrics

---

## 10. Worker Safety Monitoring

### 10.1 Safety Monitoring Technologies

#### 10.1.1 Computer Vision
- **Cameras**: RGB, thermal, depth cameras
- **AI Models**: Object detection, pose estimation, activity recognition
- **Use Cases**: PPE detection, hazard zones, near-miss events

#### 10.1.2 Wearable Sensors
- **Smart Helmets**: Impact detection, proximity warnings
- **Smart Vests**: Vital signs, fatigue monitoring, location tracking
- **Smart Glasses**: AR guidance, hazard alerts
- **Wristbands**: Environmental exposure, ergonomic strain

#### 10.1.3 Environmental Sensors
- **Gas Sensors**: Detect toxic gases (CO, H2S, VOCs)
- **Noise Sensors**: Monitor sound levels (OSHA compliance)
- **Temperature/Humidity**: Heat stress monitoring
- **Dust/Particulate**: Air quality monitoring

#### 10.1.4 Machine Sensors
- **Force/Torque**: Detect excessive forces
- **Vibration**: Identify equipment malfunctions
- **Speed**: Monitor safe operating speeds
- **Proximity**: Detect workers near hazards

### 10.2 Safety Zones

#### 10.2.1 Zone Types
1. **Restricted Zones**: No entry without authorization
2. **Hazardous Zones**: PPE required, limited occupancy
3. **Collaborative Zones**: Human-robot collaboration
4. **Safe Zones**: General access

#### 10.2.2 Zone Monitoring
- **Geofencing**: Virtual boundaries
- **Occupancy Tracking**: Number of workers in zone
- **Dwell Time**: Time spent in hazardous zones
- **Entry/Exit Logging**: Audit trail

### 10.3 Hazard Detection

#### 10.3.1 PPE Compliance
AI vision detects missing PPE:
- Hard hat
- Safety glasses
- High-visibility vest
- Gloves
- Steel-toed boots

**Action:** Alert worker and supervisor

#### 10.3.2 Unsafe Behavior
Detect risky actions:
- Running in factory
- Using mobile phone near equipment
- Improper lifting technique
- Bypassing safety guards

**Action:** Real-time warning, incident logging

#### 10.3.3 Near-Miss Events
Identify close calls:
- Worker near moving equipment
- Objects falling near workers
- Unexpected equipment motion

**Action:** Alert, investigate root cause

#### 10.3.4 Ergonomic Risks
Monitor worker posture and movements:
- Repetitive motion
- Awkward postures
- Excessive force
- Prolonged standing

**Action:** Job rotation, ergonomic improvements

### 10.4 Incident Response

#### 10.4.1 Alert Levels

| Level | Severity | Response Time | Action |
|-------|----------|---------------|--------|
| Info | Low | None | Log event |
| Warning | Medium | 1 minute | Notify supervisor |
| Alert | High | 10 seconds | Notify supervisor + safety officer |
| Critical | Immediate | < 1 second | Emergency stop + alarm |

#### 10.4.2 Emergency Protocols
- **Automatic E-stop**: Stop equipment in affected zone
- **Alarm Activation**: Visual and audible alarms
- **Emergency Services**: Automated call to first responders
- **Evacuation**: Guide workers to exits via AR/mobile apps
- **Lockout/Tagout**: Automatic equipment isolation

### 10.5 Safety Analytics

#### 10.5.1 Leading Indicators
- Near-miss frequency
- PPE compliance rate
- Safety training completion
- Hazard reports submitted
- Safety observations

#### 10.5.2 Lagging Indicators
- Injury rate (OSHA TRIR)
- Lost time injury frequency (LTIF)
- Severity rate
- Workers' compensation costs

#### 10.5.3 Predictive Safety
AI models predict high-risk situations:

**Inputs:**
- Historical incidents
- Environmental conditions
- Production schedule
- Worker fatigue levels

**Output:** Risk score (0-100)

**Threshold:** > 80 triggers preventive action

### 10.6 Compliance

Standards supported:
- **OSHA**: Occupational Safety and Health Administration (US)
- **ISO 45001**: Occupational health and safety management
- **ANSI Z10**: Occupational health and safety management systems
- **IEC 61508**: Functional safety of electrical systems

---

## 11. AR/VR Training Systems

### 11.1 Training Modalities

#### 11.1.1 Virtual Reality (VR)
Fully immersive 3D environment.

**Hardware:**
- VR Headsets: Meta Quest 3, HTC Vive, Valve Index
- Controllers: 6DOF tracking
- Haptic feedback: Force feedback gloves

**Use Cases:**
- Equipment operation
- Maintenance procedures
- Emergency response
- Hazardous environment training

#### 11.1.2 Augmented Reality (AR)
Digital information overlaid on real world.

**Hardware:**
- AR Glasses: HoloLens 2, Magic Leap 2, RealWear
- Tablets/Phones: ARKit (iOS), ARCore (Android)

**Use Cases:**
- Step-by-step instructions
- Remote assistance
- Quality inspection guidance
- Equipment information overlay

#### 11.1.3 Mixed Reality (MR)
Blend of real and virtual objects with interaction.

**Use Cases:**
- Virtual equipment in real factory
- Holographic work instructions
- Collaborative design review

### 11.2 Training Content

#### 11.2.1 Equipment Operation
- **Robot Programming**: Teach pendant operation, trajectory planning
- **CNC Machining**: Setup, programming, operation
- **Welding**: Technique practice with virtual welding
- **Forklift**: Safe driving in virtual warehouse

#### 11.2.2 Maintenance Procedures
- **Preventive Maintenance**: Step-by-step PM tasks
- **Troubleshooting**: Diagnostic procedures with guided fault finding
- **Repair**: Disassembly/assembly with interactive 3D models
- **Calibration**: Sensor and instrument calibration

#### 11.2.3 Safety Training
- **Lockout/Tagout**: Energy isolation procedures
- **Confined Space**: Entry procedures and hazard awareness
- **Emergency Response**: Fire, spill, injury response
- **Hazard Recognition**: Identify unsafe conditions

#### 11.2.4 Quality Procedures
- **Inspection**: Visual inspection techniques
- **Measurement**: Use of measurement tools (caliper, micrometer)
- **Testing**: Product testing procedures
- **Documentation**: Quality record completion

### 11.3 Training Features

#### 11.3.1 Interactive Scenarios
- Multi-step procedures with branching paths
- Realistic physics and object interaction
- Error feedback and correction
- Time limits and performance scoring

#### 11.3.2 Guided Mode
- Voice narration
- Visual highlights and arrows
- Animated demonstrations
- Contextual help

#### 11.3.3 Assessment Mode
- Timed challenges
- Performance metrics (accuracy, speed, errors)
- Pass/fail criteria
- Certification upon completion

#### 11.3.4 Multiplayer
- Collaborative training (2-10 users)
- Role-playing scenarios
- Instructor-led sessions
- Peer learning

### 11.4 Performance Metrics

| Metric | Measurement | Target |
|--------|-------------|--------|
| Training Time | Hours to competency | 50% reduction vs. traditional |
| Knowledge Retention | Post-training test scores | > 85% after 30 days |
| Skill Transfer | Performance on real equipment | > 90% success rate |
| Engagement | Session completion rate | > 95% |
| Satisfaction | Trainee feedback score | > 4.5 / 5.0 |

### 11.5 Remote Assistance

#### 11.5.1 AR Remote Support


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
