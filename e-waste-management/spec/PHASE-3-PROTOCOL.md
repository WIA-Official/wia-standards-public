# WIA E-Waste Management Standard
# Phase 3: Protocol Specification v1.0

## Document Information
- **Standard**: WIA E-Waste Management
- **Phase**: 3 - Protocol
- **Version**: 1.0.0
- **Status**: Published
- **Date**: 2025-01-15

## 1. Overview

Phase 3 specifies physical handling protocols for e-waste collection, sorting, dismantling, processing, and material recovery. These protocols ensure worker safety, environmental protection, and efficient resource recovery.

## 2. Collection Protocols

### 2.1 Intake Procedure
1. Visual inspection and damage assessment
2. Device type confirmation against WEEE category
3. Hazard identification (batteries, mercury, lead)
4. Functionality testing (if applicable)
5. Data security assessment
6. Documentation and tracking label application

### 2.2 Initial Sorting Categories
- Reuse-potential devices
- Hazardous-priority items (mercury lamps, damaged batteries, CRTs)
- High-value electronics (smartphones, tablets, computers)
- Large appliances
- General small electronics

## 3. Dismantling Protocols

### 3.1 Universal Sequence (Hazard-First Priority)
1. **Battery Removal** (CRITICAL FIRST STEP)
   - Power off device
   - Disconnect battery cable before physical removal
   - Inspect for swelling/damage
   - Isolate damaged batteries immediately

2. **Hazardous Component Extraction**
   - Mercury switches and relays
   - CFL/fluorescent backlights
   - Capacitors (older equipment)
   - Lead-containing components

3. **Precious Metal Recovery**
   - Circuit boards
   - Connectors
   - Gold-plated components

4. **Material Separation**
   - Ferrous metals (steel chassis, brackets)
   - Non-ferrous metals (copper wire, aluminum heat sinks)
   - Plastics (by type when practical)

### 3.2 Device-Specific Protocols

**Smartphone Dismantling:**
1. Remove SIM/storage cards
2. Heat adhesive (80-90°C) to separate screen
3. Disconnect battery cable, remove battery
4. Extract main logic board
5. Remove camera modules (rare earths)
6. Separate frame materials (aluminum/steel/plastic)

**CRT Monitor Dismantling:**
1. Discharge vacuum tube
2. Separate lead glass from panel glass
3. Remove electron gun assembly
4. Extract copper deflection coils
5. Segregate leaded components

## 4. Processing Methods

### 4.1 Mechanical Processing
- **Shredding**: Reduce to 5-50mm particles
- **Magnetic Separation**: Extract ferrous metals
- **Eddy Current Separation**: Recover non-ferrous metals
- **Density Separation**: Separate plastics from metals
- **Optical Sorting**: Identify plastic types via NIR

### 4.2 Metallurgical Processing
| Process | Materials | Method | Recovery Rate |
|---------|-----------|--------|---------------|
| Hydrometallurgy | Gold, Silver, Copper | Chemical leaching | 95-98% |
| Pyrometallurgy | Copper, Gold, Silver | High-temp smelting | 90-95% |
| Electrorefining | Copper, Gold | Electrolytic | 99%+ |

## 5. Safety Requirements

### 5.1 Personal Protective Equipment
| Task | Required PPE |
|------|--------------|
| Manual Dismantling | Cut-resistant gloves, Safety glasses, Closed-toe shoes |
| Battery Handling | Chemical gloves, Face shield, Apron |
| Shredding | Hearing protection, Full-face shield, Steel-toe boots |
| Chemical Processing | Chemical suit, Respirator, Goggles |

### 5.2 Environmental Controls
- HEPA filtration for particulates
- Activated carbon for VOCs
- Acid fume scrubbers
- Impermeable floors
- Spill containment systems
- Wastewater treatment

## 6. Basel Convention Compliance

### 6.1 Prior Informed Consent Procedure
1. Waste characterization
2. Notification to import country
3. Detailed manifest preparation
4. Written consent from import country
5. Transit country notifications
6. Shipment with proper labeling
7. Receipt confirmation

### 6.2 Documentation Requirements
- Waste description and classification
- Quantities and composition
- Export/import facility details
- Transportation route
- Processing methods
- Emergency procedures

## 7. Extended Producer Responsibility

### 7.1 EPR Fee Calculation
```
EPR Fee = Base Fee × Weight Factor × Hazard Premium × Recyclability Modifier - Material Value Credit

Where:
- Base Fee: Jurisdiction-specific rate
- Weight Factor: Device weight in kg
- Hazard Premium: 1.0 (no hazard) to 2.0 (high hazard)
- Recyclability Modifier: 0.8 (easy) to 1.5 (difficult)
- Material Value Credit: Precious metal content value
```

### 7.2 Collection Targets
EU WEEE Directive: 65% collection rate by weight for devices sold  
California SB 20: 100% recovery of covered electronic waste  
Japan HARL: Manufacturer-specific take-back quotas

## 8. Quality Assurance

### 8.1 Recovery Quality Standards
- Copper: ≥94% purity
- Aluminum: ≥95% purity
- Gold: ≥99% purity (after refining)
- Plastics: Single-polymer streams, <2% contamination

### 8.2 Process Auditing
- Daily operations log
- Weekly quality checks
- Monthly environmental monitoring
- Annual third-party audit

## 9. Refurbishment Standards

### 9.1 Reuse Qualification
1. Functionality testing (100% operational)
2. Component replacement (batteries, screens if needed)
3. Data wiping (NIST 800-88 standards)
4. Cosmetic restoration
5. Software updates
6. Final QA testing
7. Minimum 90-day warranty
8. Refurbishment certificate

## 10. Emergency Procedures

### 10.1 Battery Fire Response
1. Evacuate immediate area
2. Use Class D fire extinguisher (metal fires)
3. Do NOT use water
4. Contain thermal runaway with sand/vermiculite
5. Ventilate area after fire control

### 10.2 Mercury Spill Response
1. Evacuate area
2. Ventilate without creating drafts
3. Use mercury spill kit
4. Collect all visible mercury droplets
5. Seal contaminated materials
6. Professional cleanup if >1 gram

---
© 2025 SmileStory Inc. / WIA · 弘益人間


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
