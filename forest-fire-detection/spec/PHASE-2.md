# WIA-ENE-032: Forest Fire Detection Standards
# PHASE 2 - Advanced Detection & Integration

**Version:** 1.0  
**Status:** Implementation Specification  
**Date:** 2025-01-28  
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Executive Summary

PHASE 2 builds upon the foundation established in previous phases to expand detection coverage, enhance accuracy through advanced AI, integrate with emergency management systems, and implement automated response capabilities.

### 1.1 Objectives

- Expand camera coverage to 200+ units achieving >95% area coverage
- Deploy advanced multi-modal sensor networks with hyperspectral capabilities
- Implement deep learning ensemble models achieving >95% detection, <3% false alarms
- Integrate with Emergency Operations Centers (EOC) and Computer-Aided Dispatch (CAD)
- Deploy edge computing for sub-60-second detection latency
- Implement automated fire behavior prediction models
- Establish international data sharing partnerships
- Achieve <20-minute average alert-to-response time

### 1.2 Success Criteria

- Detection of 95% of fires >50m² within 15 minutes
- False alarm rate <3%
- System uptime >99%
- Full integration with EOC and CAD systems
- Edge detection latency <60 seconds
- Automated fire behavior predictions with 85% accuracy
- International data sharing operational
- Alert-to-response time <20 minutes (average)

### 1.3 Timeline

**Duration:** 18-24 months  
**Budget:** $5-10 million

---

## 2. Technical Architecture

### 2.1 System Enhancements

**Advanced AI Detection Pipeline**

- Multi-modal fusion networks combining optical, thermal, smoke, and weather data
- Temporal convolutional networks for sequence analysis
- Ensemble methods combining multiple model architectures
- Uncertainty quantification using Bayesian deep learning
- Transfer learning for rapid adaptation to new regions
- Automated model retraining pipeline with A/B testing

**Edge Computing Architecture**

- Edge servers deployed at camera locations (NVIDIA Jetson AGX Xavier or equivalent)
- Local inference for <2-second detection latency
- Distributed processing reducing bandwidth requirements by 90%
- Failover to cloud processing when edge unavailable
- Edge model synchronization with central registry

**Emergency System Integration**

- CAD system integration via NIEM (National Information Exchange Model)
- EOC integration through WebEOC API and EDXL messaging
- GIS integration with ArcGIS Enterprise and QGIS
- Common Alerting Protocol (CAP) for standardized public alerts
- Bidirectional data flow with incident status updates

### 2.2 Data Infrastructure

**Time-Series Data Platform**

- InfluxDB or TimescaleDB for high-frequency sensor data
- Grafana dashboards for real-time visualization
- Automated anomaly detection using statistical process control
- Long-term trend analysis and reporting
- Data retention policies (hot: 30 days, warm: 1 year, cold: 5 years)

**Geospatial Data Management**

- PostGIS database with spatial indexing
- GeoServer for WMS/WFS services
- QGIS Server for advanced processing
- Vector tile generation for responsive web mapping
- Raster data optimization and COG (Cloud-Optimized GeoTIFF) format

**Machine Learning Operations (MLOps)**

- MLflow for experiment tracking and model registry
- Kubeflow or Azure ML for model training pipelines
- TensorBoard for training visualization
- A/B testing framework for model comparison
- Automated retraining triggers based on performance degradation
- Model versioning and rollback capabilities

---

## 3. Hardware Deployment

### 3.1 Camera and Sensor Expansion

- Expand to 200+ cameras achieving >95% area coverage
- Deploy 500+ smoke detectors creating dense monitoring network
- Install 50+ fuel moisture sensors for direct FWI validation
- Add 30+ additional weather stations (total 50+)
- Deploy 10+ hyperspectral cameras for advanced fire characterization
- Install lightning detection network (20+ sensors)

### 3.2 Computing Infrastructure

- Add 4x GPU servers for AI training (NVIDIA A100 or H100)
- Deploy edge computing to 50+ camera sites
- Expand storage to 500TB capacity
- Increase network bandwidth to 10Gbps backbone
- Implement high-availability cluster (99.9% uptime target)
- Add disaster recovery site with automated failover

### 3.3 Detailed Sensor Specifications

**Optical Cameras**

| Specification | Value | Notes |
|--------------|-------|-------|
| Resolution | 4K (3840x2160) minimum | Higher resolution for distant fire detection |
| Focal Length | 100-400mm (variable) | Pan-tilt-zoom capability |
| Field of View | 360° panoramic or 120° directional | Depends on deployment scenario |
| Frame Rate | 10 FPS minimum | Higher for smoke movement analysis |
| Low-Light Performance | 0.001 lux | Essential for nighttime detection |
| Spectral Range | 400-700nm (visible) | Standard RGB sensors |
| Interface | IP camera (ONVIF compliant) | Standardized network protocol |
| Power | PoE+ (25W) or solar | Depends on location |
| Environmental Rating | IP67 minimum | Weatherproof and dustproof |
| Operating Temperature | -40°C to +60°C | Extreme environment capability |

**Thermal Cameras**

| Specification | Value | Notes |
|--------------|-------|-------|
| Resolution | 640x512 minimum | Higher resolution improves detection range |
| Thermal Sensitivity | <50mK NETD | Detects small temperature differences |
| Spectral Range | 8-14 μm (LWIR) | Long-wave infrared optimal for fire |
| Temperature Range | -40°C to +550°C | Covers ambient to active fire |
| Frame Rate | 30 FPS | Real-time monitoring |
| Lens Options | 13-100mm variable | Adjustable for different ranges |
| Accuracy | ±2°C or ±2% | Calibrated temperature measurement |
| Interface | GigE Vision or USB3 | High-bandwidth data transfer |
| Radiometric | Yes | Enables temperature mapping |
| Integration | Fusion with optical camera | Combined RGB-thermal output |

**Hyperspectral Cameras**

| Specification | Value | Notes |
|--------------|-------|-------|
| Spectral Range | 400-2500nm | VNIR + SWIR coverage |
| Spectral Bands | 100-200 bands | High spectral resolution |
| Spatial Resolution | 1280x1024 minimum | Sufficient spatial detail |
| Spectral Resolution | 5-10nm | Distinguishes fire signatures |
| Frame Rate | 1-5 FPS | Slower due to data volume |
| Data Rate | 100-500 MB/s | Requires high-speed storage |
| Calibration | Automated dark/white reference | Ensures data quality |
| Applications | Fire chemistry, fuel mapping | Advanced analysis |
| Processing | Edge preprocessing required | Reduces data transmission |

**Smoke Detectors**

| Specification | Value | Notes |
|--------------|-------|-------|
| Detection Method | Optical scatter + ionization | Dual-mode for reliability |
| Sensitivity | 0.5-3.0% obscuration/ft | Adjustable for environment |
| Response Time | <30 seconds | Rapid alert capability |
| Coverage Area | 30m radius | Optimal spacing |
| Communication | LoRaWAN or cellular | Long-range low-power |
| Battery Life | 5+ years | Low maintenance |
| Environmental Rating | IP65 | Outdoor rated |
| False Alarm Reduction | Multi-criteria algorithm | Weather compensation |
| Self-Testing | Daily diagnostic | Ensures reliability |

**Weather Stations**

| Specification | Value | Notes |
|--------------|-------|-------|
| Wind Speed | 0-75 m/s range, ±0.3 m/s accuracy | Critical for fire behavior |
| Wind Direction | 0-360°, ±3° accuracy | Determines fire spread |
| Temperature | -40°C to +60°C, ±0.3°C | Ambient conditions |
| Humidity | 0-100% RH, ±2% | Fuel moisture proxy |
| Barometric Pressure | 500-1100 hPa, ±0.5 hPa | Weather pattern tracking |
| Precipitation | 0-500 mm/hr, ±5% | Rain events |
| Solar Radiation | 0-1500 W/m², ±5% | Fire weather component |
| Data Logging | 1-minute intervals | High temporal resolution |
| Communication | Cellular + satellite backup | Redundant connectivity |
| Power | Solar + battery backup | Autonomous operation |

**Fuel Moisture Sensors**

| Specification | Value | Notes |
|--------------|-------|-------|
| Measurement Range | 5-40% moisture content | Typical fuel moisture range |
| Accuracy | ±1% | Precise measurement |
| Fuel Types | 10-hour, 100-hour, 1000-hour | Different time-lag fuels |
| Depth | 0-30cm soil moisture | Root zone conditions |
| Temperature Range | -20°C to +50°C | Operating conditions |
| Response Time | <10 minutes (10-hour fuels) | Time-lag appropriate |
| Calibration | Fuel-type specific | Improves accuracy |
| Data Logging | 15-minute intervals | Trend analysis |
| Communication | LoRaWAN mesh network | Low power |
| Deployment | 1 sensor per 100 km² | Regional coverage |

### 3.4 Integration Protocols

**CAD System Integration**

- Protocol: NIEM (National Information Exchange Model) v5.0
- Message Format: XML with GML geometry
- Update Frequency: Real-time (<5 second latency)
- Data Fields: Location, confidence, size estimate, smoke characteristics, weather
- Authentication: OAuth 2.0 with mutual TLS
- Reliability: Message queuing with guaranteed delivery
- Testing: Comprehensive integration testing with fire department CAD

**EOC Integration**

- Protocol: WebEOC API v8.0 + EDXL Distribution Element
- Message Types: Alert, Update, Cancel, Acknowledgment
- Mapping Integration: KML/KMZ export for situation awareness
- Status Updates: Bidirectional incident status synchronization
- Authentication: API keys with IP whitelisting
- Incident Management: Automatic incident creation and linking
- Reporting: Automated situation reports every 15 minutes during active fires

**GIS Integration**

- Platform: ArcGIS Enterprise 11.0 + QGIS Server 3.30
- Services: WMS, WFS, WCS for standard OGC compliance
- Real-time Layers: Fire locations, sensor status, weather conditions
- Static Layers: Fuel models, topography, infrastructure, property boundaries
- Analysis Tools: Viewshed analysis, fire perimeter prediction, evacuation modeling
- Mobile Access: ArcGIS Field Maps for field personnel
- Update Frequency: 1-minute refresh for dynamic layers

**Common Alerting Protocol (CAP)**

- Version: CAP v1.2 (OASIS standard)
- Distribution: IPAWS integration for public alerts
- Message Types: Alert, Update, Cancel, Acknowledgment, Error
- Alert Zones: Geocoded polygons for affected areas
- Languages: Multi-language support for diverse communities
- Delivery Channels: SMS, email, broadcast, sirens, social media
- Testing: Regular test messages and validation

---

## 4. Implementation Roadmap

### 4.1 Detailed Quarter-by-Quarter Plan

**Quarter 1: Foundation & Planning**

- Complete detailed system architecture design
- Finalize hardware procurement specifications
- Establish vendor partnerships and procurement contracts
- Deploy initial edge computing pilot (10 sites)
- Begin AI model enhancement development
- Hire additional technical staff (5 positions)
- Establish project governance and oversight committees

**Quarter 2: Initial Deployment**

- Deploy 50 additional cameras (total 100)
- Complete edge computing infrastructure deployment
- Deploy 200 additional smoke detectors
- Install 10 fuel moisture sensors
- Establish MLOps pipeline and experiment tracking
- Begin CAD/EOC integration development
- Complete initial AI model training for ensemble approach

**Quarter 3: Advanced AI Development**

- Deploy deep learning ensemble models to production
- Implement uncertainty quantification framework
- Complete edge AI optimization and deployment
- Deploy 50 more cameras (total 150)
- Install 15 additional weather stations
- Begin hyperspectral camera pilot (5 units)
- Implement A/B testing framework

**Quarter 4: Integration Phase I**

- Complete CAD system integration
- Implement EOC integration with WebEOC
- Deploy GIS integration with ArcGIS Enterprise
- Install 50 more cameras (total 200)
- Deploy 300 additional smoke detectors (total 500)
- Complete lightning detection network installation
- Launch time-series data platform

**Quarter 5: Advanced Sensors**

- Deploy all 10 hyperspectral cameras
- Complete fuel moisture sensor network (50 sensors)
- Install remaining weather stations (50 total)
- Implement fire behavior prediction models
- Deploy automated anomaly detection system
- Enhance edge computing with failover capabilities
- Begin international data sharing partnership negotiations

**Quarter 6: Integration Phase II**

- Complete Common Alerting Protocol (CAP) integration
- Implement bidirectional data flow with incident management
- Deploy Grafana dashboards for real-time monitoring
- Enhance geospatial data management with vector tiles
- Implement automated retraining pipelines
- Complete disaster recovery site deployment
- Launch international data sharing pilot

**Quarter 7: Optimization & Testing**

- Conduct comprehensive system performance testing
- Optimize AI models based on real-world performance
- Fine-tune fire behavior prediction algorithms
- Complete high-availability cluster deployment
- Implement model versioning and rollback capabilities
- Conduct emergency system integration stress testing
- Expand international partnerships (5+ agencies)

**Quarter 8: Validation & Handover**

- Conduct 90-day performance validation period
- Complete all personnel training programs
- Finalize documentation and standard operating procedures
- Conduct stakeholder review and approval process
- Achieve all success criteria targets
- Complete knowledge transfer to operations team
- Plan PHASE 3 initiation activities

### 4.2 Critical Path Activities

**High-Priority Dependencies:**

1. Edge computing infrastructure must be deployed before AI model optimization
2. CAD/EOC integration requires extensive testing before production deployment
3. Hyperspectral camera deployment depends on vendor delivery timelines
4. International partnerships require legal agreements and data sharing protocols
5. Fire behavior models require comprehensive historical data preparation

### 4.3 Deployment Scenarios

**Scenario 1: Dense Urban-Forest Interface**

- High camera density (1 camera per 5 km²)
- Multiple smoke detector layers for early warning
- Real-time integration with municipal emergency services
- Focus on rapid detection (<5 minutes) due to population density
- Automated evacuation alerts through CAP integration
- Example locations: Boulder, Colorado; Paradise, California

**Scenario 2: Remote Wilderness Areas**

- Lower camera density (1 camera per 50 km²)
- Satellite communication for connectivity
- Extended battery backup and solar power systems
- Focus on fire behavior prediction due to limited suppression access
- Wildlife camera integration for ignition detection
- Example locations: National forests, wilderness preserves

**Scenario 3: Agricultural Fire Management**

- Controlled burn monitoring and validation
- Smoke dispersion modeling integration
- Coordination with agricultural agencies
- Fuel moisture monitoring for burn windows
- Air quality impact assessment
- Example locations: Florida, Southeast pine forests

**Scenario 4: International Deployment**

- Adaptation to local climate conditions
- Integration with local emergency systems
- Language and cultural customization
- Technology transfer and training programs
- Local staff capacity building
- Example locations: Mediterranean region, Australia

---

## 5. Budget Estimate

### 5.1 Capital Expenditures

- Hardware (cameras, sensors, servers): $2.5-4.0M
- Software licenses and development: $0.8-1.5M
- Installation and deployment: $0.5-1.0M
- Training and documentation: $0.2-0.5M

### 5.2 Operating Expenditures (Annual)

- Personnel: $1.2-1.8M
- Communication services: $0.3-0.5M
- Maintenance and support: $0.4-0.7M
- Cloud services: $0.2-0.4M

### 5.3 Total PHASE 2 Budget

**Capital:** $4.0-7.0M  
**Operating (per year):** $2.1-3.4M  
**Total (2 years):** $8.2-13.8M

---

## 6. Risk Management

### 6.1 Technical Risks

- **Risk:** AI model performance degradation in edge deployment
  - **Mitigation:** Model compression techniques, continuous validation, cloud failover
- **Risk:** Emergency system integration failures
  - **Mitigation:** Extensive testing, standardized protocols, vendor partnerships
- **Risk:** Edge computing hardware failures
  - **Mitigation:** Redundant units, remote management, rapid replacement SLAs

### 6.2 Operational Risks

- **Risk:** Increased system complexity reducing reliability
  - **Mitigation:** Comprehensive monitoring, automated diagnostics, extensive training
- **Risk:** Integration with legacy emergency systems
  - **Mitigation:** API abstraction layers, custom adapters, vendor consultation

---

## 7. Performance Metrics

### 7.1 Detection Performance

- Detection Rate: >95% for fires >50m²
- False Alarm Rate: <3%
- Detection Latency: <15 minutes (average)
- Alert Delivery: <3 minutes

### 7.2 Operational Performance

- System Uptime: >99%
- Edge Processing Success: >95%
- EOC/CAD Integration Reliability: >98%
- Fire Behavior Prediction Accuracy: >85%

---

## 8. Success Criteria

### 8.1 Completion Checklist

- ✅ 200+ cameras deployed and operational
- ✅ Edge computing deployed to 50+ sites
- ✅ AI ensemble models achieving >95% detection, <3% false alarms
- ✅ EOC and CAD integration operational
- ✅ Fire behavior predictions validated >85% accuracy
- ✅ International data sharing partnerships established
- ✅ All personnel trained on enhanced systems
- ✅ Performance metrics exceeding targets for 90 days

---

## 9. Transition Planning

### 9.1 PHASE 2 Closeout

- Comprehensive performance evaluation against all success criteria
- Documentation of lessons learned and best practices
- Knowledge transfer to operations team
- System acceptance and handover procedures
- Stakeholder review and approval
- Recognition of team achievements

### 9.2 PHASE 3 Preparation

- Review PHASE 2 results and adjust PHASE 3 plans
- Initiate early procurement for long-lead items
- Begin stakeholder engagement for next phase
- Develop detailed PHASE 3 project plan
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


## Annex E — Implementation Notes for PHASE-2

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2.

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
evidence for PHASE-2. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2.
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
for PHASE-2. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2 validation when the
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
