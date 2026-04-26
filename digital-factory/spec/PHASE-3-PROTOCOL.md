# WIA-IND-028 PHASE 3 — Protocol Specification

**Standard:** WIA-IND-028
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

Expert provides real-time guidance to on-site technician.

**Features:**
- Live video feed from AR glasses
- Annotation tools (arrows, circles, text)
- Screen sharing
- Spatial anchors (persistent 3D markers)
- Session recording for documentation

**Benefits:**
- Reduce downtime by 40-60%
- Eliminate travel costs
- Access global expertise

#### 11.5.2 Use Cases
- **Troubleshooting**: Expert diagnoses issue remotely
- **Repair Guidance**: Step-by-step repair instructions
- **Installation**: Complex equipment installation support
- **Inspection**: Remote quality inspection with AI assistance

### 11.6 Content Creation

#### 11.6.1 3D Modeling
- Import CAD models (STEP, IGES)
- Photogrammetry (scan real equipment)
- 3D modeling software (Blender, Maya, 3ds Max)

#### 11.6.2 Scenario Design
- Visual scripting (Unreal Blueprints, Unity Visual Scripting)
- Behavior trees for AI entities
- Event triggers and logic

#### 11.6.3 Authoring Tools
- No-code platforms for SME-created content
- Template library for common procedures
- Content versioning and updates

---

## 12. Real-time KPI Dashboards

### 12.1 Dashboard Types

#### 12.1.1 Executive Dashboard
High-level factory overview for management.

**KPIs:**
- Overall OEE
- Production volume
- Quality rate
- Safety incidents
- Energy consumption
- Cost per unit

**Update Frequency:** 1 minute

#### 12.1.2 Operations Dashboard
Real-time production monitoring for plant managers.

**KPIs:**
- Line status (running/stopped/alarmed)
- Current vs. target output
- Cycle time
- Downtime reasons
- WIP levels
- Shift performance

**Update Frequency:** 5 seconds

#### 12.1.3 Maintenance Dashboard
Equipment health and maintenance tracking.

**KPIs:**
- Equipment status
- Vibration/temperature trends
- Predicted failures
- PM schedule compliance
- MTBF, MTTR
- Spare parts inventory

**Update Frequency:** 1 minute

#### 12.1.4 Quality Dashboard
Quality metrics and defect tracking.

**KPIs:**
- First pass yield
- Defect rate by type
- Inspection results
- Customer complaints
- Scrap/rework costs
- Six Sigma metrics

**Update Frequency:** Per part/batch

#### 12.1.5 Energy Dashboard
Energy consumption and optimization.

**KPIs:**
- Real-time power (kW)
- Energy consumption (kWh)
- Cost ($)
- Energy intensity (kWh/unit)
- Peak demand
- Renewable %

**Update Frequency:** 1 second

### 12.2 Visualization Types

#### 12.2.1 Time-series Charts
Line charts for trends over time.

**Use Cases:**
- Production volume over time
- Energy consumption patterns
- Temperature trends

#### 12.2.2 Gauges and Meters
Circular or linear gauges for single values.

**Use Cases:**
- Current OEE
- Equipment speed
- Temperature

#### 12.2.3 Bar Charts
Compare values across categories.

**Use Cases:**
- Downtime by reason
- Production by line
- Defects by type

#### 12.2.4 Pie Charts
Show proportions of a whole.

**Use Cases:**
- Downtime breakdown
- Product mix
- Energy by category

#### 12.2.5 3D Factory View
Interactive 3D visualization of factory.

**Features:**
- Equipment status (color-coded)
- Real-time animation (robots, conveyors)
- Clickable for details
- AR/VR support

#### 12.2.6 Heatmaps
Color-coded visualization of spatial data.

**Use Cases:**
- Worker density
- Temperature distribution
- Energy consumption by zone
- Quality issues by location

### 12.3 Dashboard Features

#### 12.3.1 Real-time Updates
- WebSocket connections for live data
- Push notifications for alerts
- Automatic refresh

#### 12.3.2 Drill-down
- Click on chart to see details
- Multi-level hierarchy (factory → line → station → equipment)

#### 12.3.3 Time Range Selection
- Last hour, shift, day, week, month, year
- Custom date ranges
- Compare periods (this week vs. last week)

#### 12.3.4 Filtering
- By line, product, shift, operator
- Multi-select filters
- Save filter sets

#### 12.3.5 Alerts and Notifications
- Threshold-based alerts
- Anomaly detection alerts
- Email, SMS, mobile push
- Alert history and acknowledgment

#### 12.3.6 Export
- PDF reports
- Excel data export
- API access for custom integrations

### 12.4 Dashboard Platform

#### 12.4.1 Web-based
Accessible via browser (desktop, tablet, mobile).

**Technologies:**
- React, Angular, Vue.js
- D3.js, Chart.js for visualizations
- WebGL for 3D

#### 12.4.2 Mobile Apps
Native apps for iOS and Android.

**Features:**
- Push notifications
- Offline access
- Camera integration (AR)

#### 12.4.3 Large Displays
Factory floor displays (55-85 inch screens).

**Use Cases:**
- Andon boards
- Team performance
- Safety metrics

#### 12.4.4 Wearables
Smartwatches, AR glasses.

**Features:**
- Glanceable KPIs
- Alerts
- Voice commands

---

## 13. Connected Worker Platform

### 13.1 Platform Components

#### 13.1.1 Mobile App
Smartphone app for all workers.

**Features:**
- Digital work instructions
- Quality checklists
- Incident reporting
- Time tracking
- Messaging
- Training access

#### 13.1.2 Wearables
Smart devices worn by workers.

**Types:**
- Smart watches (Apple Watch, Samsung Galaxy Watch)
- Smart glasses (HoloLens, RealWear)
- Smart helmets (DAQRI, RealWear)
- Fitness bands (Fitbit, Garmin)

**Features:**
- Hands-free information
- Voice commands
- Biometric monitoring
- Location tracking
- AR overlays

#### 13.1.3 Tablets
Ruggedized tablets for shop floor use.

**Use Cases:**
- Maintenance work orders
- Quality inspection forms
- Equipment manuals
- Production dashboards

### 13.2 Worker Capabilities

#### 13.2.1 Digital Work Instructions
Step-by-step procedures with:
- Text instructions
- Photos and diagrams
- Videos
- 3D models (AR)
- Interactive checklists

**Benefits:**
- Reduce errors by 70%
- Faster training
- Version control

#### 13.2.2 Real-time Communication
- Team messaging
- Video calls
- Group channels by department
- File sharing
- Translation (50+ languages)

#### 13.2.3 Task Management
- Assign tasks to workers
- Track progress
- Set priorities
- Notifications
- Time tracking

#### 13.2.4 Quality Data Collection
- Photo capture
- Barcode/QR scanning
- Measurement entry
- Defect categorization
- Root cause analysis

#### 13.2.5 Incident Reporting
- Quick incident entry
- Photo/video evidence
- Location tagging
- Severity classification
- Automatic notifications

### 13.3 Worker Analytics

#### 13.3.1 Productivity Metrics
- Tasks completed per shift
- Time per task
- Efficiency vs. standard
- Utilization %

#### 13.3.2 Quality Metrics
- Defect rate by worker
- First pass yield
- Rework %

#### 13.3.3 Safety Metrics
- Near-miss reports
- PPE compliance
- Safety training status
- Fatigue indicators

#### 13.3.4 Skills Matrix
Track worker skills and certifications:
- Training completed
- Certifications earned
- Skill proficiency levels
- Cross-training opportunities

### 13.4 Worker Privacy

#### 13.4.1 Data Collection Transparency
- Clear notice of what data is collected
- Opt-in for non-essential data
- Data retention policies

#### 13.4.2 Data Anonymization
- Aggregate data for analytics
- De-identify personal information
- Differential privacy techniques

#### 13.4.3 Access Control
- Workers can view their own data
- Managers see aggregated data only
- Privacy settings

#### 13.4.4 Compliance
- GDPR (EU)
- CCPA (California)
- PIPEDA (Canada)
- Labor laws and union agreements

---

## 14. Factory-as-a-Service

### 14.1 Service Model

Factory-as-a-Service (FaaS) delivers manufacturing capabilities as on-demand cloud services.

#### 14.1.1 Consumption-based Pricing
- Pay per unit produced
- Pay per machine hour
- Pay per gigabyte of data
- Subscription tiers

#### 14.1.2 Elastic Capacity
- Scale production up/down on demand
- Access specialized equipment
- Overflow capacity during peak demand

#### 14.1.3 Shared Infrastructure
- Multi-tenant cloud platform
- Shared simulation and AI services
- Collaborative ecosystem

### 14.2 Service Offerings

#### 14.2.1 Digital Twin as a Service (DTaaS)
Cloud-hosted digital twin platform.

**Features:**
- Virtual factory creation
- Real-time synchronization
- Simulation and optimization
- Predictive analytics

**Pricing:** $5,000 - $50,000/month based on complexity

#### 14.2.2 Simulation as a Service (SimaaS)
On-demand production simulation.

**Features:**
- DES, ABS, physics simulation
- What-if analysis
- Optimization
- Scenario comparison

**Pricing:** $500 - $5,000 per simulation

#### 14.2.3 AI as a Service (AIaaS)
Pre-trained AI models for manufacturing.

**Models:**
- Quality prediction
- Predictive maintenance
- Energy optimization
- Demand forecasting

**Pricing:** $1,000 - $10,000/month per model

#### 14.2.4 Analytics as a Service (AaaS)
Cloud-based analytics and reporting.

**Features:**
- Real-time dashboards
- Custom reports
- Anomaly detection
- Benchmarking

**Pricing:** $2,000 - $20,000/month based on data volume

#### 14.2.5 Training as a Service (TaaS)
Cloud-hosted AR/VR training platform.

**Features:**
- Content library
- Custom content creation tools
- Multi-user sessions
- Progress tracking

**Pricing:** $100/user/month

### 14.3 Platform Architecture

```
┌───────────────────────────────────────────┐
│         Web/Mobile/API Clients            │
└───────────────────────────────────────────┘
                    ↕
┌───────────────────────────────────────────┐
│          API Gateway / Load Balancer       │
└───────────────────────────────────────────┘
                    ↕
┌─────────────────────────────────────────────────┐
│               Microservices Layer               │
│  ┌──────────┬──────────┬──────────┬──────────┐ │
│  │ Digital  │ Simulation│ Analytics│ Training │ │
│  │ Twin Svc │ Service   │ Service  │ Service  │ │
│  └──────────┴──────────┴──────────┴──────────┘ │
└─────────────────────────────────────────────────┘
                    ↕
┌─────────────────────────────────────────────────┐
│               Data Layer                        │
│  ┌──────────┬──────────┬──────────┬──────────┐ │
│  │ Time-    │ Object   │ Graph    │ Cache    │ │
│  │ series DB│ Storage  │ DB       │ (Redis)  │ │
│  └──────────┴──────────┴──────────┴──────────┘ │
└─────────────────────────────────────────────────┘
                    ↕
┌─────────────────────────────────────────────────┐
│            Edge Layer (Factory)                 │
│  - Data collection and preprocessing            │
│  - Local analytics and control                  │
│  - Secure connectivity to cloud                 │
└─────────────────────────────────────────────────┘
```

### 14.4 Service Level Agreements (SLAs)

| Service | Availability | Latency | Support |
|---------|--------------|---------|---------|
| DTaaS | 99.9% | < 100 ms | 24/7 |


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
