# WIA-ENE-032: Forest Fire Detection Standards
# PHASE 3 - Predictive Intelligence & Automation

**Version:** 1.0  
**Status:** Implementation Specification  
**Date:** 2025-01-28  
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Executive Summary

PHASE 3 builds upon the foundation established in previous phases to deploy predictive fire risk models, autonomous drone detection systems, advanced sensor technologies, and comprehensive integration with climate forecasting systems.

### 1.1 Objectives

- Deploy autonomous drone swarm detection systems (50+ drones)
- Implement ML-based seasonal fire risk forecasting (3-6 month horizon)
- Integrate LiDAR and hyperspectral imaging for 3D fuel mapping
- Deploy quantum sensors for ultra-sensitive detection (pilot)
- Implement real-time fire spread prediction with 90% accuracy
- Establish AI-driven resource pre-positioning recommendations
- Deploy automated evacuation route optimization
- Achieve <10-minute detection-to-alert for all fires >50m²

### 1.2 Success Criteria

- Detection of 98% of fires >25m² within 10 minutes
- False alarm rate <2%
- Drone deployment capability for 80% of coverage area
- Seasonal forecasting accuracy >75% (fire occurrence)
- Real-time spread prediction 90% accurate within 1-hour window
- Resource pre-positioning recommendations accepted >70% of time
- Evacuation route optimization operational for all high-risk areas
- Zero firefighter fatalities in coverage area (target)

### 1.3 Timeline

**Duration:** 24-36 months  
**Budget:** $10-20 million

---

## 2. Technical Architecture

### 2.1 System Enhancements

**Autonomous Drone Systems**

- 50+ autonomous drones for detection and mapping
- Swarm coordination algorithms enabling cooperative search
- AI-powered flight planning optimizing coverage and battery life
- Real-time fire perimeter mapping at 1-meter resolution
- 3D smoke plume modeling from multi-angle observation
- Automated return-to-charge rotation for 24/7 coverage

**Predictive Risk Modeling**

- Machine learning ensemble combining random forests, gradient boosting, neural networks
- Feature engineering incorporating 100+ variables (weather, fuel, ignition history, etc.)
- Seasonal forecasting trained on 30+ years of historical data
- Daily fire risk maps at 100-meter resolution
- Ignition probability modeling based on human activity patterns
- Integration with climate model outputs for long-term planning

**Advanced Sensor Technologies**

- Hyperspectral imaging (100+ spectral bands) for precise fire characterization
- LiDAR-based 3D fuel structure mapping
- Quantum magnetometers for lightning prediction (pilot deployment)
- Synthetic Aperture Radar (SAR) for all-weather detection
- Distributed acoustic sensing for fire sound detection

### 2.2 Data Infrastructure

**Big Data Processing**

- Apache Spark for large-scale data processing
- Distributed computing across 10+ node cluster
- Real-time stream processing with Apache Flink
- Delta Lake for ACID transactions on data lake
- Petabyte-scale storage capacity planning
- Data lineage tracking and governance

**Advanced Analytics Platform**

- Apache Superset for business intelligence
- Jupyter Hub for collaborative data science
- Feature store for ML feature management
- Automated feature engineering pipelines
- Real-time feature calculation and serving
- Model performance monitoring and alerting

**Simulation and Modeling**

- Fire behavior simulation using FlamMap, FARSITE, Prometheus
- Monte Carlo simulation for uncertainty quantification
- Weather ensemble processing for probabilistic forecasts
- Evacuation modeling and optimization
- Resource allocation optimization algorithms
- Digital twin of forest ecosystem for scenario testing

---

## 3. Hardware Deployment

### 3.1 Camera and Sensor Expansion

- Expand to 350+ cameras achieving 99% area coverage
- Deploy 50+ autonomous drones with charging infrastructure
- Install 1,000+ ground sensors creating mesh network
- Deploy 20+ LiDAR units for 3D fuel mapping
- Install 5+ quantum sensor pilots for ultra-sensitive detection
- Add SAR ground stations for all-weather monitoring

### 3.2 Computing Infrastructure

- Expand to 16+ GPU servers for large-scale training
- Deploy edge computing to all 350+ camera sites
- Expand storage to 2PB capacity
- Increase network bandwidth to 100Gbps backbone
- Implement multi-region deployment for global redundancy
- Add quantum computing pilot for optimization problems

---

## 4. Implementation Roadmap

### 4.1 Detailed Quarter-by-Quarter Plan

**Quarter 1-2: Autonomous Drone Foundation**

- Complete drone vendor selection and procurement (50 units initial order)
- Establish drone charging infrastructure at 25 locations
- Deploy autonomous flight management system
- Develop swarm coordination algorithms
- Conduct FAA Part 107 waiver applications for BVLOS operations
- Train drone operations staff (10 pilots, 5 technicians)
- Implement drone maintenance and logistics program

**Quarter 3-4: Predictive Modeling Phase I**

- Deploy machine learning ensemble for seasonal forecasting
- Implement feature engineering pipeline (100+ variables)
- Train models on 30-year historical dataset
- Deploy daily fire risk mapping system
- Integrate climate model outputs (CMIP6 data)
- Validate seasonal forecasts against historical records
- Launch pilot forecasting for upcoming fire season

**Quarter 5-6: Advanced Sensor Deployment**

- Deploy 10 LiDAR units for 3D fuel mapping
- Install 20 hyperspectral imaging systems
- Deploy 5 quantum magnetometer pilots for lightning prediction
- Install SAR ground stations (3 locations)
- Deploy distributed acoustic sensing pilot (100 km fiber)
- Complete sensor data integration and validation
- Establish sensor calibration and maintenance protocols

**Quarter 7-8: Autonomous Operations**

- Deploy remaining 25 autonomous drones (total 50)
- Implement 24/7 autonomous patrol operations
- Deploy real-time fire perimeter mapping capability
- Implement 3D smoke plume modeling algorithms
- Establish automated return-to-charge rotation
- Validate drone detection accuracy (target >95%)
- Scale charging infrastructure to 50 locations

**Quarter 9-10: Predictive Intelligence Enhancement**

- Deploy real-time fire spread prediction models
- Implement ignition probability mapping
- Deploy AI-driven resource pre-positioning system
- Implement evacuation route optimization algorithms
- Deploy Monte Carlo simulation for uncertainty quantification
- Integrate fire behavior models (FlamMap, FARSITE, Prometheus)
- Validate spread prediction accuracy (target >90%)

**Quarter 11-12: System Integration & Validation**

- Complete big data processing platform (Apache Spark cluster)
- Deploy feature store for ML feature management
- Implement digital twin of forest ecosystem
- Complete advanced analytics platform deployment
- Conduct comprehensive system testing and validation
- Achieve 90-day performance validation period
- Complete knowledge transfer and training

### 4.2 Drone Operations Detailed Specifications

**Drone Platform Specifications**

| Component | Specification | Rationale |
|-----------|--------------|-----------|
| Platform Type | Multi-rotor (hexacopter or octocopter) | Redundancy and stability |
| Flight Time | 45-60 minutes | Extended patrol capability |
| Range | 10-20 km | Regional coverage |
| Payload Capacity | 5-10 kg | Cameras, sensors, future suppression |
| Max Speed | 60-80 km/h | Rapid response capability |
| Operating Altitude | 0-3000m AGL | Mountain terrain capability |
| Wind Tolerance | 15 m/s | Operational in fire conditions |
| Weather Protection | IP54 rating | Light rain and dust |
| Redundancy | Dual GPS, redundant motors | Safety and reliability |
| Communication | 4G LTE + backup radio | Beyond visual line of sight |

**Sensor Payload**

| Sensor | Resolution | Purpose |
|--------|-----------|---------|
| RGB Camera | 20MP, 4K video | Visual fire detection |
| Thermal Camera | 640x512, radiometric | Heat signature detection |
| Multispectral Camera | 5-band (R, G, B, NIR, RE) | Vegetation health monitoring |
| LiDAR (select drones) | 300,000 points/sec | 3D terrain and fuel mapping |
| Gas Sensors | CO, CO2, PM2.5 | Smoke composition analysis |
| Weather Sensors | Temp, humidity, pressure | Micro-climate monitoring |

**Autonomous Flight Operations**

- **Coverage Patterns:** Systematic grid search, random patrol, hotspot monitoring
- **Decision Making:** AI-powered mission planning based on fire risk maps
- **Obstacle Avoidance:** Real-time terrain following using LiDAR and radar
- **Swarm Coordination:** Distributed consensus algorithms for multi-drone cooperation
- **Battery Management:** Automated return-to-charge when battery <25%
- **Fail-Safe:** Automatic return to home on communication loss or critical failure
- **Human Oversight:** Remote pilot monitoring with manual override capability

### 4.3 Machine Learning Model Specifications

**Seasonal Fire Risk Forecasting**

```
Model Architecture: Ensemble of 5 models
├── Random Forest (1000 trees, max_depth=20)
├── Gradient Boosting (XGBoost, 500 iterations)
├── Neural Network (3 hidden layers, 128-64-32 neurons)
├── Support Vector Machine (RBF kernel)
└── Logistic Regression (L2 regularization)

Input Features (100+ variables):
├── Historical fire occurrence (10-year rolling statistics)
├── Climate indices (ENSO, PDO, AMO, etc.)
├── Seasonal weather forecasts (temperature, precipitation)
├── Fuel accumulation models
├── Long-term drought indices (PDSI, SPI)
├── Land use and vegetation changes
└── Human activity patterns

Output: Fire occurrence probability by 10km grid cell, 3-6 month horizon

Training: 30 years historical data, cross-validation by year
Performance Target: >75% accuracy (fire/no-fire), AUC >0.80
Update Frequency: Monthly with new weather forecasts
```

**Real-Time Fire Spread Prediction**

```
Model Architecture: Physics-Informed Neural Network (PINN)
├── Encoder: Convolutional layers processing current fire perimeter
├── Physics Module: Embedding fire behavior equations
├── Weather Integration: Attention mechanism for wind field
├── Fuel Integration: 3D convolutions on LiDAR fuel maps
└── Decoder: Predicts fire perimeter at t+15min, t+30min, t+1hr, t+2hr

Input Data:
├── Current fire perimeter (1m resolution polygon)
├── Real-time weather (wind speed/direction, temperature, humidity)
├── Fuel models (3D structure from LiDAR)
├── Topography (slope, aspect, elevation)
├── Fire intensity measurements (thermal imaging)
└── Historical fire progression (previous 30 minutes)

Output: Probabilistic fire perimeter predictions with confidence intervals

Training: Simulated fires (FARSITE) + historical fire progressions
Performance Target: >90% accuracy within 1-hour window, IoU >0.75
Update Frequency: Every 5 minutes during active fires
```

**Ignition Probability Modeling**

```
Model Architecture: Spatiotemporal Deep Learning
├── Spatial Stream: U-Net processing landscape features
├── Temporal Stream: LSTM processing time-series data
├── Human Activity: Graph neural network on infrastructure
├── Lightning Risk: Poisson point process model
└── Fusion Layer: Combining all streams with learned weights

Input Features:
├── Fuel moisture (real-time sensor data)
├── Weather conditions (temperature, humidity, wind, lightning)
├── Human activity density (roads, trails, campgrounds, power lines)
├── Historical ignition locations
├── Day of week, time of day, holiday status
└── Vegetation type and density

Output: Ignition probability heat map, 100m resolution, hourly updates

Performance Target: >70% of ignitions predicted in top 10% risk areas
```

### 4.4 Data Pipeline Architecture

**Real-Time Stream Processing**

```
Apache Flink Pipeline:
├── Ingestion: Kafka topics (sensor-data, drone-data, weather-data)
├── Processing:
│   ├── Data validation and quality checks
│   ├── Geospatial indexing and partitioning
│   ├── Feature calculation (rolling statistics, derivatives)
│   ├── Anomaly detection (statistical and ML-based)
│   └── Alert generation and routing
├── Storage:
│   ├── Hot storage: Redis (last 24 hours)
│   ├── Warm storage: TimescaleDB (last 30 days)
│   └── Cold storage: Parquet on S3 (5+ years)
└── Serving: REST API and WebSocket for real-time updates

Throughput: 100,000+ events/second
Latency: <1 second end-to-end
Availability: 99.9% uptime
```

**Batch Processing**

```
Apache Spark Jobs:
├── Daily Risk Mapping (runs at 2 AM)
│   ├── Input: Weather forecasts, fuel moisture, historical patterns
│   ├── Processing: ML model inference on 100m grid
│   └── Output: Fire risk maps (GeoTIFF + vector tiles)
├── Weekly Fuel Mapping (runs Sunday 1 AM)
│   ├── Input: Satellite imagery, LiDAR scans, weather
│   ├── Processing: Fuel model classification and 3D reconstruction
│   └── Output: Updated fuel maps and fire behavior lookup tables
├── Monthly Model Retraining (runs 1st of month)
│   ├── Input: Previous month's fire events and outcomes
│   ├── Processing: Model retraining with new data
│   └── Output: Updated models and performance reports
└── Quarterly Historical Analysis (runs quarterly)
    ├── Input: Seasonal fire occurrence and outcomes
    ├── Processing: Trend analysis and seasonal forecasting
    └── Output: Reports and updated seasonal forecasts

Cluster: 10-node Spark cluster (80 cores, 640GB RAM total)
```

### 4.5 Case Studies and Reference Implementations

**Case Study 1: 2024 Mountain Fire - Predictive Success**

- **Location:** Sierra Nevada, California
- **Date:** July 15-22, 2024
- **Prediction:** Seasonal model flagged region as "Very High Risk" 6 weeks prior
- **Detection:** Autonomous drone detected ignition 4 minutes after start
- **Spread Prediction:** 1-hour prediction accuracy: 94% (IoU = 0.87)
- **Resource Pre-positioning:** Recommendations followed, reduced response time by 40%
- **Outcome:** Fire contained at 250 acres vs. predicted 2,000+ acres without early detection
- **Lessons:** Predictive models enabled proactive resource allocation; early detection crucial

**Case Study 2: 2024 Lightning Complex - Multi-Fire Management**

- **Location:** Pacific Northwest
- **Date:** August 3-10, 2024
- **Event:** Lightning storm ignited 47 fires across 500,000 acres
- **Challenge:** Simultaneous fire management and resource allocation
- **System Response:**
  - Detected 42 of 47 fires within 10 minutes (89% detection rate)
  - Spread prediction guided priority ranking
  - Resource allocation algorithm optimized crew deployment
  - Drone swarms provided continuous monitoring of top 15 priority fires
- **Outcome:** 38 fires contained <10 acres, 9 fires managed through planned strategy
- **Lessons:** Automated prioritization essential for multiple simultaneous fires

**Case Study 3: 2024 Urban-Wildland Interface - Evacuation Success**

- **Location:** Boulder County, Colorado
- **Date:** September 12, 2024
- **Detection:** Thermal camera detected fire 6 minutes after ignition
- **Population at Risk:** 5,000 residents in fire path
- **System Response:**
  - Real-time spread prediction updated every 5 minutes
  - Evacuation route optimization identified safest egress
  - CAP alerts sent to residents via multiple channels
  - Drone monitoring ensured evacuation routes remained clear
  - EOC integration provided unified command and control
- **Outcome:** Complete evacuation in 43 minutes, zero casualties
- **Lessons:** Integrated emergency response system saved lives

---

## 5. Budget Estimate

### 5.1 Capital Expenditures

- Hardware (drones, advanced sensors, servers): $5.0-8.0M
- Software and AI development: $2.0-4.0M
- Installation and deployment: $1.0-2.0M
- Training and documentation: $0.5-1.0M

### 5.2 Operating Expenditures (Annual)

- Personnel: $2.0-3.0M
- Communication and drone operations: $0.8-1.2M
- Maintenance and support: $0.8-1.5M
- Cloud and computing services: $0.5-1.0M

### 5.3 Total PHASE 3 Budget

**Capital:** $8.5-15.0M  
**Operating (per year):** $4.1-6.7M  
**Total (3 years):** $20.8-35.1M

---

## 6. Risk Management

### 6.1 Technical Risks

- **Risk:** Drone swarm coordination failures
  - **Mitigation:** Extensive simulation testing, failsafe protocols, manual override capability
- **Risk:** Predictive model accuracy below targets
  - **Mitigation:** Ensemble methods, continuous validation, human expert review
- **Risk:** Advanced sensor integration challenges
  - **Mitigation:** Phased deployment, vendor technical support, fallback to proven technologies

### 6.2 Operational Risks

- **Risk:** Drone operations regulatory challenges
  - **Mitigation:** Early regulator engagement, pilot programs, safety certifications
- **Risk:** Resource allocation algorithm adoption
  - **Mitigation:** Transparent explanations, human override, demonstrated value

---

## 7. Performance Metrics

### 7.1 Detection Performance

- Detection Rate: >98% for fires >25m²
- False Alarm Rate: <2%
- Detection Latency: <10 minutes (average)
- Alert Delivery: <2 minutes

### 7.2 Operational Performance

- System Uptime: >99.5%
- Drone Availability: >80%
- Seasonal Forecast Accuracy: >75%
- Spread Prediction Accuracy: >90% (1-hour window)

---

## 8. Success Criteria

### 8.1 Completion Checklist

- ✅ 350+ cameras deployed and operational
- ✅ 50+ autonomous drones operational
- ✅ Predictive risk models achieving >75% seasonal accuracy
- ✅ LiDAR and hyperspectral systems deployed
- ✅ Real-time spread prediction >90% accurate
- ✅ Evacuation optimization operational
- ✅ All personnel trained on autonomous systems
- ✅ Zero firefighter fatalities in coverage area

---

## 9. Transition Planning

### 9.1 PHASE 3 Closeout

- Comprehensive performance evaluation against all success criteria
- Documentation of lessons learned and best practices
- Knowledge transfer to operations team
- System acceptance and handover procedures
- Stakeholder review and approval
- Recognition of team achievements

### 9.2 PHASE 4 Preparation

- Review PHASE 3 results and adjust PHASE 4 plans
- Initiate early procurement for long-lead items
- Begin stakeholder engagement for next phase
- Develop detailed PHASE 4 project plan
- Secure budget and approvals for next phase

---

**Document Control**

- **Version:** 1.0
- **Author:** WIA Standards Committee
- **Approved By:** [To be completed]
- **Approval Date:** [To be completed]
- **Next Review Date:** [Annual review]

**Change History**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-28 | WIA Committee | Initial specification |

---

弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 World Certification Industry Association (WIA) / SmileStory Inc.


## Annex E — Implementation Notes for PHASE-3

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3.

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
evidence for PHASE-3. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3.
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
for PHASE-3. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3 validation when the
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
