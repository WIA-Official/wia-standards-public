# WIA-AUG-010 PHASE 3 — Protocol Specification

**Standard:** WIA-AUG-010
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

const pidOutput = this.pidController(error);

    // Feedforward based on physiological demand
    const demandCompensation = this.estimateDemand(params.physiologicalDemand);

    // Efficiency optimization
    const optimalOperatingPoint = this.findOptimalEfficiency(
      params.currentOutput,
      params.energyInput
    );

    // Combine control signals
    const controlSignal = this.combineSignals(
      pidOutput,
      demandCompensation,
      optimalOperatingPoint
    );

    // Apply constraints and safety limits
    return this.applyConstraints(controlSignal, params.constraints);
  }
}
```

### 7.2 Efficiency Optimization

```
Efficiency = (Useful_Output / Total_Energy_Input) × 100%

For Mechanical Heart:
Useful_Output = Cardiac Work = Pressure × Flow × Conversion_Factor
Total_Energy_Input = Electrical Power Consumption

Target: >75% efficiency

Optimization Strategies:
1. Variable Speed Control: Match RPM to demand
2. Pulsatile vs. Continuous: Select based on physiology
3. Power Management: Sleep modes during low demand
4. Thermal Management: Minimize heat generation
5. Wear Optimization: Reduce friction, extend life
```

### 7.3 Predictive Maintenance

```typescript
interface MaintenancePredictor {
  operatingHours: number;
  cycleCount: number;
  performanceHistory: PerformanceData[];
  wearIndicators: WearMetrics;

  predictNextService(): ServicePrediction {
    const hoursToService = this.calculateHoursToService();
    const cyclesToService = this.calculateCyclesToService();
    const performanceDegradation = this.trendAnalysis();
    const wearLevel = this.assessWear();

    return {
      estimatedServiceDate: new Date(Date.now() + hoursToService * 3600000),
      confidenceLevel: this.calculateConfidence(),
      recommendedActions: this.generateRecommendations(),
      urgency: this.determineUrgency(wearLevel, performanceDegradation)
    };
  }
}
```

---

## 8. Power and Energy Systems

### 8.1 Power System Types

| Type | Technology | Capacity | Duration | Recharge | Applications |
|------|-----------|----------|----------|----------|--------------|
| Battery | Li-ion, LiFePO₄ | 10-100 Wh | 8-24h | External, wireless | Hearts, pumps |
| TET | Transcutaneous Energy Transfer | Continuous | Unlimited | Wireless coil | Implanted devices |
| Biofuel | Glucose fuel cell | 1-10 mW | Continuous | Biological | Bioartificial |
| Hybrid | Battery + TET | 50-200 Wh | >24h | Multiple modes | Advanced systems |
| External | Direct connection | Unlimited | Continuous | N/A | Dialysis, ECMO |

### 8.2 Battery Requirements

```
Primary Battery:
- Capacity: 8-12 hours typical use
- Charge Time: <4 hours to 80%, <6 hours to 100%
- Cycle Life: >1000 full cycles
- Safety: Overcharge, overdischarge, temperature protection
- Indicators: LED, app, audible alerts

Backup Battery:
- Capacity: 30-60 minutes minimum
- Auto-Switchover: <1 second
- Alert: Immediate notification
- Hot-Swappable: Allow primary battery change

Power Management:
- Low Power Mode: Reduce non-critical functions
- Sleep Mode: Minimal power during rest
- Adaptive: Match power to physiological demand
```

### 8.3 Wireless Power Transfer

```
TET System Specifications:
- Frequency: 100-300 kHz typical
- Efficiency: >70% coil-to-coil
- Distance: 5-20 mm tissue penetration
- Power Delivery: 5-50 Watts
- Safety: Temperature monitoring, SAR limits
- Alignment: Magnetic coupling feedback

Design Requirements:
- External Coil: Wearable, comfortable
- Internal Coil: Biocompatible encapsulation
- Positioning: Stable anatomical location
- Interference: EMI shielding, filtering
```

### 8.4 Biofuel Cells

```
Glucose Fuel Cell:
- Mechanism: Glucose oxidation for power
- Power Output: 1-10 mW continuous
- Efficiency: 20-40% glucose to electrical
- Location: Bloodstream, interstitial fluid
- Electrodes: Biocompatible carbon, platinum
- Applications: Low-power sensors, pacing

Advantages:
- Continuous fuel source (glucose)
- No external charging
- Fully implantable
- Long-term stability

Challenges:
- Limited power output
- Biofouling of electrodes
- Glucose variability
- Safety (biocompatibility)
```

---

## 9. Maintenance and Service Scheduling

### 9.1 Maintenance Framework

```
Maintenance Types:
1. Preventive: Scheduled based on time/cycles
2. Predictive: Based on performance trends and wear
3. Corrective: In response to faults or degradation
4. Emergency: Immediate intervention for critical issues

Frequency:
- Daily: Self-check, battery status
- Weekly: Performance review, alert check
- Monthly: Comprehensive function test
- Quarterly: Clinical assessment, biomarkers
- Annually: Major inspection, component replacement if needed
```

### 9.2 Service Scheduling Algorithm

```typescript
interface ServiceSchedule {
  lastServiceDate: Date;
  operatingHours: number;
  cycleCount: number;
  performanceMetrics: PerformanceHistory;
  wearIndicators: WearAssessment;

  calculateNextService(): ServiceRecommendation {
    // Time-based component
    const timeSinceService = Date.now() - this.lastServiceDate.getTime();
    const timeScore = timeSinceService / (365 * 24 * 3600 * 1000); // Years

    // Usage-based component
    const hoursScore = this.operatingHours / this.expectedLifeHours;
    const cyclesScore = this.cycleCount / this.expectedLifeCycles;

    // Performance-based component
    const performanceScore = this.assessPerformanceDegradation();

    // Wear-based component
    const wearScore = this.assessWearLevel();

    // Combined score
    const serviceScore = Math.max(
      timeScore * 0.20,
      hoursScore * 0.25,
      cyclesScore * 0.25,
      performanceScore * 0.20,
      wearScore * 0.10
    );

    return {
      urgency: this.scoreToUrgency(serviceScore),
      recommendedDate: this.calculateDate(serviceScore),
      reason: this.determineReason(serviceScore),
      estimatedDowntime: this.estimateDowntime()
    };
  }
}
```

### 9.3 Component Replacement Guidelines

```
Mechanical Heart:
- Bearings: Every 3-5 years or 50M cycles
- Valves: Every 5-7 years
- Battery: Every 2-3 years or 1000 cycles
- Controller: Every 5-10 years or on failure
- External Equipment: Every 3-5 years

Bioartificial Kidney:
- Membrane Filters: Every 6-12 months
- Cell Cartridges: Every 1-2 years (if applicable)
- Tubing: Every 3-6 months
- Pumps: Every 2-3 years
- Sensors: Every 1-2 years

3D Bioprinted Skin:
- Replacement: Only if rejection or failure
- Monitoring: Ongoing integration assessment
- Debridement: As needed for wound healing
- Secondary Procedures: Scar revision, etc.
```

---

## 10. Failsafe and Emergency Protocols

### 10.1 Failsafe System Architecture

```
Redundancy Levels:
1. Primary System: Main organ function
2. Backup System: Immediate takeover on failure
3. Emergency Mode: Minimal life-sustaining function
4. Alert System: Notification to patient and medical team
5. External Support: Manual intervention, temporary support

Failsafe Triggers:
- System Failure: Hardware or software malfunction
- Power Loss: Battery depletion, connection loss
- Performance Degradation: Output <50% of target
- Rejection: High rejection risk score
- Patient Command: Manual activation
```

### 10.2 Emergency Response Protocol

```typescript
class EmergencyProtocol {
  activateFailsafe(trigger: FailsafeTrigger): EmergencyResponse {
    // 1. Immediate actions
    this.switchToBackupSystem();
    this.activateAlerts();
    this.logEvent(trigger);

    // 2. Assess situation
    const severity = this.assessSeverity(trigger);
    const timeToFailure = this.estimateTimeToFailure();

    // 3. Determine response
    if (severity === 'CRITICAL') {
      return this.criticalResponse();
    } else if (severity === 'HIGH') {
      return this.highPriorityResponse();
    } else {
      return this.standardResponse();
    }
  }

  criticalResponse(): EmergencyResponse {
    return {
      action: 'IMMEDIATE_MEDICAL_INTERVENTION',
      notification: ['PATIENT', 'EMERGENCY_CONTACT', 'MEDICAL_TEAM', 'EMERGENCY_SERVICES'],
      deviceMode: 'EMERGENCY_MODE',
      instructions: 'Proceed to nearest hospital immediately',
      backupDuration: this.calculateBackupDuration(),
      alternativeSupport: this.identifyAlternatives()
    };
  }
}
```

### 10.3 Backup Power Protocol

```
Power Loss Detection:
- Primary Battery: <10% capacity
- Charging Failure: No charge increase in 30 minutes
- TET Disconnect: Coil misalignment detected
- Power Interruption: Voltage drop below threshold

Automatic Actions:
1. Switch to backup battery (<1 second)
2. Activate low-power mode
3. Alert patient (vibration, sound)
4. Notify medical team
5. Display battery status
6. Disable non-essential functions

Patient Actions:
1. Connect to external power if available
2. Replace/recharge primary battery
3. Realign TET coil if applicable
4. Contact medical team if unable to resolve
5. Proceed to medical facility if critical


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
