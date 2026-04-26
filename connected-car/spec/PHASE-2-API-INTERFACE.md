# WIA-AUTO-008 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-008
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

#### 4.3.3 Integrity Verification

- **Algorithm**: SHA-256 or SHA-3
- **Scope**: Entire update package
- **Timing**: Before and after download
- **Failure Action**: Reject and report

### 4.4 Update Strategies

#### 4.4.1 Phased Rollout

```
Phase 1: Internal Testing (1-2 days)
  └─> Deploy to test fleet
  └─> Monitor for issues

Phase 2: Limited Release (3-7 days)
  └─> Deploy to 1-5% of fleet
  └─> Monitor telemetry

Phase 3: Gradual Expansion (1-2 weeks)
  └─> Deploy to 10%, 25%, 50% of fleet
  └─> Continue monitoring

Phase 4: Full Deployment (1-4 weeks)
  └─> Deploy to entire fleet
  └─> Final validation
```

#### 4.4.2 Rollback Mechanisms

1. **Automatic Rollback**
   - Boot failure detection
   - System health check failures
   - Rollback within 3 boot attempts

2. **Manual Rollback**
   - User-initiated via UI
   - Remote-initiated via cloud
   - Service center rollback

3. **Dual-Bank Flashing**
   - Active bank (current version)
   - Inactive bank (new version)
   - Atomic switch on success

---

## 5. Remote Diagnostics

### 5.1 Diagnostic Levels

#### 5.1.1 Basic Diagnostics

**Frequency**: Continuous or on-demand

```typescript
interface BasicDiagnostics {
  battery: {
    voltage: number;           // Volts
    current: number;           // Amperes
    stateOfCharge: number;     // Percentage
    health: number;            // Percentage
  };
  fluids: {
    engineOil: 'ok' | 'low' | 'critical';
    coolant: 'ok' | 'low' | 'critical';
    washerFluid: 'ok' | 'low' | 'empty';
    brakeFluid: 'ok' | 'low' | 'critical';
  };
  tires: {
    frontLeft: { pressure: number; temperature: number };
    frontRight: { pressure: number; temperature: number };
    rearLeft: { pressure: number; temperature: number };
    rearRight: { pressure: number; temperature: number };
  };
  warningLights: string[];     // Active warning codes
}
```

#### 5.1.2 Standard Diagnostics

**Frequency**: Daily or on-demand

```typescript
interface StandardDiagnostics extends BasicDiagnostics {
  dtcs: DiagnosticTroubleCode[];
  sensors: {
    maf: number;               // Mass Air Flow (g/s)
    o2: number;                // Oxygen sensor (λ)
    map: number;               // Manifold pressure (kPa)
    iat: number;               // Intake air temp (°C)
    ect: number;               // Engine coolant temp (°C)
  };
  performance: {
    fuelConsumption: number;   // L/100km or MPG
    range: number;             // km remaining
    efficiency: number;        // Percentage
  };
}
```

#### 5.1.3 Comprehensive Diagnostics

**Frequency**: Weekly or on-demand

```typescript
interface ComprehensiveDiagnostics extends StandardDiagnostics {
  predictiveMaintenance: {
    component: string;
    currentCondition: number;  // Percentage
    estimatedLife: number;     // Days until service
    severity: 'low' | 'medium' | 'high' | 'critical';
    recommendation: string;
  }[];
  componentWear: {
    brakePads: number;         // mm remaining
    tireDepth: number;         // mm remaining
    batteryHealth: number;     // Percentage
    filterLife: number;        // Days remaining
  };
  systemHealth: {
    engine: number;            // Health score 0-100
    transmission: number;
    brakes: number;
    suspension: number;
    electrical: number;
  };
}
```

### 5.2 Diagnostic Trouble Codes (DTCs)

#### 5.2.1 DTC Format (OBD-II Standard)

```
Format: [System][Type][Component][Specific Code]

Example: P0420
  P = Powertrain
  0 = Generic (SAE standard)
  4 = Emission control
  20 = Catalyst system efficiency below threshold

Systems:
  P = Powertrain (engine, transmission)
  C = Chassis (ABS, suspension)
  B = Body (airbags, climate)
  U = Network (communication)

Type:
  0 = Generic (SAE)
  1 = Manufacturer-specific
  2 = Generic (SAE)
  3 = Manufacturer-specific
```

#### 5.2.2 DTC Severity Classification

```
Critical (Priority 1):
  - Engine failure
  - Brake system failure
  - Airbag malfunction
  - Loss of vehicle control
  Action: Immediate service required

High (Priority 2):
  - Emission system failure
  - Transmission issues
  - Significant power loss
  - Safety system degradation
  Action: Service within 48 hours

Medium (Priority 3):
  - Minor sensor failures
  - Non-critical system degradation
  - Efficiency reduction
  Action: Service within 1 week

Low (Priority 4):
  - Informational codes
  - Intermittent issues
  - Preventive warnings
  Action: Service at next maintenance
```

### 5.3 Predictive Maintenance

#### 5.3.1 Machine Learning Models

**Brake Pad Wear Prediction**

```python
# Input Features
features = [
  'current_thickness_mm',
  'vehicle_weight_kg',
  'avg_speed_kmh',
  'braking_events_per_day',
  'harsh_braking_percentage',
  'distance_driven_km',
  'terrain_type'  # urban, highway, mountain
]

# Output
prediction = {
  'days_until_replacement': 45,
  'confidence': 0.87,
  'recommendation': 'Schedule service in 30 days'
}
```

**Battery Health Prediction**

```python
# Input Features
features = [
  'state_of_health_percentage',
  'charge_cycles',
  'avg_temperature_celsius',
  'depth_of_discharge_avg',
  'age_months',
  'fast_charge_percentage'
]

# Output
prediction = {
  'remaining_useful_life_months': 18,
  'degradation_rate': 0.02,  # per month
  'confidence': 0.92
}
```

#### 5.3.2 Anomaly Detection

**Real-Time Anomaly Detection**

```
Algorithm: Isolation Forest / LSTM Autoencoder

Monitored Parameters:
  - Engine temperature anomalies
  - Vibration pattern changes
  - Fuel consumption spikes
  - Battery voltage fluctuations
  - Sensor reading outliers

Alert Threshold: 3 standard deviations
Response Time: < 1 second
False Positive Rate: < 5%
```

---

## 6. Data Collection and Privacy

### 6.1 Data Categories

#### 6.1.1 Personal Data (PII)

```
Category: Identifiable Information
Data Points:
  - Vehicle Identification Number (VIN)
  - Owner name and contact
  - Driver profile
  - Biometric data (if applicable)

Privacy Level: High
Consent: Explicit required
Retention: User-controlled
Anonymization: Required for analytics
```

#### 6.1.2 Location Data

```
Category: Geospatial Information
Data Points:
  - GPS coordinates
  - Trip history
  - Frequent destinations
  - Speed and direction

Privacy Level: High
Consent: Explicit required
Retention: 30-90 days (configurable)
Anonymization: Required after retention period
```

#### 6.1.3 Vehicle Telemetry

```
Category: Technical Data
Data Points:
  - Speed, acceleration, braking
  - Fuel/battery consumption
  - Engine parameters
  - System diagnostics

Privacy Level: Medium
Consent: Opt-out allowed
Retention: 1-3 years
Anonymization: Recommended
```

#### 6.1.4 Usage Patterns

```
Category: Behavioral Data
Data Points:
  - Driving behavior
  - Feature usage
  - Infotainment preferences
  - Charging/refueling patterns

Privacy Level: Medium
Consent: Opt-out allowed
Retention: 1 year
Anonymization: Required
```

### 6.2 Privacy Compliance

#### 6.2.1 GDPR (General Data Protection Regulation)

**Principles**:
1. **Lawfulness, Fairness, Transparency**: Clear consent and disclosure
2. **Purpose Limitation**: Data used only for stated purposes
3. **Data Minimization**: Collect only necessary data
4. **Accuracy**: Keep data accurate and up-to-date
5. **Storage Limitation**: Delete data when no longer needed
6. **Integrity and Confidentiality**: Secure data protection
7. **Accountability**: Demonstrate compliance

**User Rights**:
- Right to access
- Right to rectification
- Right to erasure ("right to be forgotten")
- Right to data portability
- Right to object
- Right to restrict processing

#### 6.2.2 CCPA (California Consumer Privacy Act)

**Requirements**:
- Disclose data collection practices
- Allow opt-out of data sales
- Provide data access upon request
- Enable data deletion
- Non-discrimination for privacy choices

#### 6.2.3 Data Anonymization Techniques

**K-Anonymity**:
```
Technique: Generalization and Suppression
K Value: ≥ 3 (each record indistinguishable from at least 2 others)
Application: Trip data, driver behavior
```

**Differential Privacy**:
```
Technique: Add statistical noise
ε (epsilon): 0.1 - 1.0 (privacy budget)
Application: Aggregate analytics
```

**Pseudonymization**:
```
Technique: Replace identifiers with pseudonyms
Reversibility: Possible with key (kept separate)
Application: VIN hashing for analytics
```

### 6.3 Consent Management

#### 6.3.1 Consent Levels

```typescript
enum ConsentLevel {
  REQUIRED = 'required',          // Essential functions
  OPTIONAL_SERVICE = 'optional',  // Enhanced features
  ANALYTICS = 'analytics',        // Usage analytics
  MARKETING = 'marketing',        // Promotional content
  THIRD_PARTY = 'third_party'     // Partner services
}

interface ConsentPreferences {
  vehicleId: string;
  userId: string;


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
