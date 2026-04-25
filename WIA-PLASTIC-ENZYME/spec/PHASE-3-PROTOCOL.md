# WIA-PLASTIC-ENZYME Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 of the WIA-PLASTIC-ENZYME standard defines the operational protocols that ensure consistent, safe, and efficient enzymatic plastic degradation. These protocols cover the entire process from waste collection to monomer certification.

### 1.1 Scope

This specification covers:

- Pre-treatment protocols
- Enzymatic reaction protocols
- Monomer recovery protocols
- Quality control protocols
- Safety and environmental protocols

### 1.2 Process Flow

```
Collection → Sorting → Pre-treatment → Reaction → Recovery → QC → Certification
```

---

## 2. Pre-treatment Protocol

### 2.1 Cleaning and Decontamination

**Objective:** Remove labels, adhesives, and contaminants that could inhibit enzyme activity.

| Step | Process | Parameters | Duration |
|------|---------|------------|----------|
| 1 | Hot water wash | 80°C | 10 min |
| 2 | Alkaline treatment | 1% NaOH | 5 min |
| 3 | Rinse | Deionized water | 5 min |
| 4 | Drying | 60°C hot air | Until <0.5% moisture |

**Quality Checkpoints:**
- [ ] Visual inspection for remaining labels
- [ ] pH verification (6.5-7.5)
- [ ] Moisture content measurement (<0.5%)
- [ ] Contamination assessment

### 2.2 Size Reduction

**Objective:** Increase surface area for better enzyme access.

| Target Size | Method | Surface Area Increase | Rate Improvement |
|-------------|--------|----------------------|------------------|
| <10mm flakes | Primary grinding | Baseline | 1.0x |
| <2mm particles | Secondary milling | 5x | 2.5x |
| <0.5mm powder | Micronization | 20x | 4x |

**Equipment Requirements:**
- Shredder: Min. 50 kW capacity
- Granulator: Adjustable screen size 1-10mm
- Mill: Cryogenic or ambient, depending on plastic type

### 2.3 Crystallinity Optimization

**Objective:** Reduce crystallinity to increase amorphous regions accessible to enzymes.

**Protocol:**
1. Heat plastic above Tg (>75°C for PET)
2. Maintain for 5-10 minutes
3. Rapid cooling to ambient temperature
4. Verify crystallinity reduction via DSC or FTIR

**Target:** Reduce crystallinity from 30-40% to 15-25%

---

## 3. Enzymatic Reaction Protocol

### 3.1 Reactor Preparation

**Buffer System:**
- 50mM Tris-HCl or phosphate buffer
- Target pH: 8.0 ± 0.2
- Ionic strength: 100-200 mM

**Pre-heat Checklist:**
- [ ] Buffer prepared and pH verified
- [ ] Reactor cleaned and sterilized
- [ ] Temperature controller calibrated
- [ ] Agitation system functional

### 3.2 Standard Operating Conditions

| Parameter | Value | Tolerance |
|-----------|-------|-----------|
| Temperature | 50-55°C | ±2°C |
| pH | 8.0 | ±0.3 |
| Substrate loading | 10-15% (w/v) | - |
| Agitation | 100-200 RPM | - |
| PETase concentration | 2-3 mg/g substrate | - |
| MHETase concentration | 1-1.5 mg/g substrate | - |

### 3.3 Enzyme Addition Protocol

1. **Pre-mixing:** Dissolve enzymes in buffer at room temperature
2. **Gradual addition:** Add enzyme solution over 30 minutes with mixing
3. **Homogenization:** Continue mixing for 15 minutes
4. **Reaction start:** Begin temperature and time logging

### 3.4 Reaction Monitoring

**Monitoring Schedule:**

| Time (h) | Actions |
|----------|---------|
| 0 | Record initial conditions, start logging |
| 6 | First sample, check pH |
| 12 | Sample, TPA/MHET analysis |
| 24 | Sample, assess progress |
| 36 | Sample, pH adjustment if needed |
| 48 | End-point determination |

**Analytical Methods:**
- HPLC-UV for TPA, MHET, BHET quantification
- Gravimetric analysis for residual plastic
- pH and temperature continuous logging

### 3.5 Completion Criteria

| Criterion | Target |
|-----------|--------|
| Degradation | ≥95% |
| TPA concentration plateau | 4+ hours stable |
| Residual plastic | <5% of input |

---

## 4. Monomer Recovery Protocol

### 4.1 Solid-Liquid Separation

1. **Enzyme inactivation:** Heat to 80°C for 15 minutes
2. **Coarse filtration:** 200 mesh filter to remove undegraded material
3. **Centrifugation:** 10,000g for 20 minutes
4. **Clarification:** 0.45µm membrane filtration

### 4.2 TPA Crystallization

| Step | Process | Parameters |
|------|---------|------------|
| 1 | Acidification | Add HCl to pH 2.5 |
| 2 | Cooling | Reduce to 4°C over 2 hours |
| 3 | Crystallization | Hold at 4°C for 4 hours |
| 4 | Filtration | Vacuum filter crystals |
| 5 | Washing | Cold DI water, 2x volume |
| 6 | Drying | Vacuum dry at 60°C |

**Expected Yield:** 85-95% of theoretical

### 4.3 Ethylene Glycol Recovery

| Step | Process | Parameters |
|------|---------|------------|
| 1 | Neutralization | Adjust to pH 7 |
| 2 | Concentration | Evaporate to 50% volume |
| 3 | Distillation | Vacuum, 80°C |
| 4 | Dehydration | Molecular sieves |

**Expected Purity:** ≥98.5%

---

## 5. Quality Control Protocol

### 5.1 TPA Quality Specifications

| Parameter | Method | Specification |
|-----------|--------|---------------|
| Purity | HPLC | ≥99.0% |
| Color (Hazen) | Spectrophotometry | ≤20 |
| Ash content | Gravimetric | ≤0.01% |
| Metal ions (each) | ICP-MS | ≤1 ppm |
| Moisture | Karl Fischer | ≤0.1% |
| MHET impurity | HPLC | ≤500 ppm |
| BHET impurity | HPLC | ≤200 ppm |

### 5.2 EG Quality Specifications

| Parameter | Method | Specification |
|-----------|--------|---------------|
| Purity | GC-FID | ≥99.5% |
| Water content | Karl Fischer | ≤0.1% |
| Acidity | Titration | ≤0.01 mg KOH/g |
| Color (APHA) | Spectrophotometry | ≤10 |
| DEG content | GC | ≤0.1% |

### 5.3 Food Contact Certification Requirements

For food-contact grade monomers, additional testing:

| Test | Requirement |
|------|-------------|
| Heavy metals | Meet EC 1935/2004 |
| Microbial count | <100 CFU/g |
| Endotoxin | <0.5 EU/mL |
| Residual enzyme activity | Undetectable |
| Volatile organics | Meet FDA 21 CFR 177.1630 |

---

## 6. Safety Protocol

### 6.1 Personal Protective Equipment

| Operation | Required PPE |
|-----------|-------------|
| Chemical handling | Lab coat, safety glasses, chemical gloves |
| Powder handling | Respirator (N95+), safety glasses |
| High temperature | Heat-resistant gloves, face shield |
| Sampling | Disposable gloves, lab coat |

### 6.2 Chemical Hazards

| Chemical | Hazard | Mitigation |
|----------|--------|------------|
| NaOH (1%) | Corrosive | Neutralization station, eyewash |
| HCl (concentrated) | Corrosive, fumes | Fume hood, neutralization |
| Enzymes | Potential allergen | Avoid aerosol generation |

### 6.3 Emergency Procedures

- **Enzyme spill:** Inactivate with 70% ethanol, clean with detergent
- **Acid/base spill:** Neutralize, absorb, dispose as hazardous waste
- **High temperature exposure:** Cool with water, seek medical attention

---

## 7. Environmental Protocol

### 7.1 Wastewater Management

| Stream | Treatment | Discharge Limit |
|--------|-----------|-----------------|
| Wash water | Neutralization, settling | pH 6-9, TSS <50 mg/L |
| Reaction liquor | Enzyme inactivation | No active enzymes |
| Acid/base waste | Neutralization | pH 6-9 |

### 7.2 Solid Waste Management

| Waste | Classification | Disposal |
|-------|---------------|----------|
| Undegraded plastic residue | Non-hazardous | Energy recovery or landfill |
| Filter media | Non-hazardous | Industrial waste |
| Contaminated PPE | Non-hazardous | Incineration |

### 7.3 Energy Efficiency

**Target:** <50 kWh per ton of plastic processed

**Energy optimization:**
- Heat recovery from reactor cooling
- Optimized agitation speed
- Insulated reactors and piping
- Off-peak electricity usage

---

## 8. Documentation Requirements

### 8.1 Batch Records

Each batch must include:

- [ ] Batch number and date
- [ ] Input material characterization
- [ ] Enzyme lot numbers and concentrations
- [ ] Time-temperature-pH log
- [ ] Sampling records and results
- [ ] Output yields and quality results
- [ ] Deviation reports (if any)
- [ ] Operator signatures

### 8.2 Retention Period

| Document Type | Retention |
|---------------|-----------|
| Batch records | 5 years |
| Quality certificates | 10 years |
| Training records | Duration of employment + 3 years |
| Equipment logs | 3 years |

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-PLASTIC-ENZYME is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-PLASTIC-ENZYME/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-PLASTIC-ENZYME/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-PLASTIC-ENZYME/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


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

