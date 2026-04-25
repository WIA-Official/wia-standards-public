# WIA-MED-011: Smart Pill
## Phase 3 — Safety, Materials & Regulatory Integration

**Standard ID:** WIA-MED-011
**Category:** Medical — Ingestible Sensors
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 3 governs the safety properties of a smart pill: the materials that touch tissue and digesta, the integrity of the housing across the GI environment, the expulsion guarantee, and the regulatory framing into which a deployment must fit. The standard is medical-device adjacent: it does not displace a jurisdiction's regulatory regime, but it specifies the structural and operational guarantees a conformant platform must deliver into that regime.

## 2. Biocompatibility Envelope

### 2.1 Material Stack

A platform declares its complete material stack — every component that touches tissue or digesta — in the conformance manifest. The declaration identifies each material by an established identifier (chemical name, polymer designation, alloy code) so that a regulatory reviewer can trace the material to a known-safety body of evidence.

A platform whose material stack contains an undeclared component is non-conformant.

### 2.2 Body-Contact Categories

The standard recognizes three body-contact categories, each with its own evidence burden:

| Category | Examples | Evidence |
|---------|---------|----------|
| Surface contact only | external housing | biocompatibility evidence for surface contact in the GI tract |
| Limited diffusion | porous capsule for sampling | additional evidence on extractables and leachables |
| Active release | controlled drug release | full pharmaceutical-grade evidence on the released agent |

A deployment must place each component into a category and produce the corresponding evidence package. An undeclared shift in category — repurposing a surface-contact housing for limited-diffusion behavior — is itself a non-conformance.

### 2.3 Lot Traceability

Every commercially deployed platform carries a lot identifier traceable to the material-supply provenance. A deployment that cannot produce the lot history of an in-service platform is non-conformant in traceability. This is a recall-readiness requirement: a finding on one lot must be actionable across deployments.

## 3. Housing Integrity

### 3.1 Mechanical Envelope

The housing must withstand the mechanical envelope of normal GI transit: peristaltic pressure, mastication-adjacent forces if the platform is chewable-adjacent, and the hydraulic pressure of pyloric and ileocecal transit.

A housing breach in vivo is the most serious failure mode the standard covers. The platform must be designed and tested so that under nominal use, breach probability is below the declared bound. A deployment whose breach rate exceeds the declared bound is in recall-trigger territory and the deployment is non-conformant pending remediation.

### 3.2 Chemical Envelope

The housing must withstand the chemical envelope of the GI tract: pH range from gastric (low) to colonic (mildly basic), bile, pancreatic enzymes, and the patient's diet within reasonable variability. A housing whose integrity is conditional on patient diet is non-conformant unless the deployment's prescribing protocol enforces the conditioning diet and the patient acknowledges it.

### 3.3 Energy Containment

The energy source — battery, capacitor, or harvested — is contained within the housing such that even on housing breach, the energy source does not leak chemically or thermally beyond bounds the gut tissue can tolerate. The standard's posture: a housing breach is a recall-trigger event, but it must not be a clinical emergency.

## 4. Expulsion Guarantee

### 4.1 Transit Envelope

The platform declares a nominal transit envelope (P95 transit time) in the manifest. The envelope is supported by the prescribing protocol: a platform deployed in a patient population with known motility variation must declare an envelope appropriate for that population and may not assume the population mean.

### 4.2 Retention

A platform that has not been expelled within the declared envelope is *retained*. Retention is a clinical event:

- the receiver detects retention from the cessation of telemetry plus the absence of a confirmed expulsion event,
- the prescribing clinician is alerted within the declared latency,
- a retention pathway documented in the deployment's clinical workflow takes over,
- the retention itself is an audit event.

A deployment without a documented retention pathway is non-conformant.

### 4.3 Imaging-Localizable

For platforms in the Imaging or Sampling classes, the platform must be radio-opaque or otherwise imaging-localizable in the event of retention. A platform that becomes invisible to standard imaging modalities post-retention is non-conformant.

## 5. Adverse Event Reporting

### 5.1 Recognized Adverse Events

The standard names a recognized set of adverse events for which the deployment must have an explicit pathway:

| Event | Pathway |
|-------|---------|
| Housing breach | recall-trigger; clinical evaluation |
| Retention | retention-pathway protocol |
| Allergic reaction | clinical evaluation; root-cause to material or release |
| Misidentification | platform-to-patient binding error; immediate clinical review |
| Telemetry-derived clinical alarm | clinical workflow per the prescribing protocol |

### 5.2 Reporting Cadence

Each adverse-event class has a declared reporting cadence: immediate (housing breach, misidentification), within-shift (retention, allergic), within-day (telemetry-derived alarm). The cadences are part of the deployment's manifest and are auditable.

### 5.3 Aggregation Reporting

The deployment publishes a periodic safety summary covering the population of platforms deployed in the period: ingestion count, transit summary, retention count, breach count, allergic reactions, and any deaths or hospitalizations to which the platform may have contributed. The report is reviewed by the prescribing-clinician panel and is part of the lifecycle audit.

## 6. Regulatory Framing

### 6.1 Jurisdictional Integration

WIA-MED-011 does not displace a jurisdiction's medical-device regulation. A deployment must operate under the appropriate regulatory authorization in its jurisdiction (e.g., FDA, EMA, MFDS, PMDA). The standard's role is to give a single technical framework that maps cleanly onto the evidence packages those regimes expect.

### 6.2 Evidence Mapping

The deployment's manifest declares the regulatory authorization under which each component operates. The manifest is the integration point: a regulator reviewing the manifest finds, for each material and each functional class, the authorization under which it is being deployed.

### 6.3 Off-Label Use

A platform deployed outside the regulatory authorization (off-label) is a deployment-level decision the standard does not make. The standard requires the deployment to record the off-label decision as a manifest amendment with the prescribing-clinician's authorization and with explicit patient consent. Off-label without amendment is non-conformant in the standard's posture, regardless of the underlying clinical merit.

## 7. Patient Consent and Information

### 7.1 Consent Surface

A patient receiving a smart pill must have been presented with consent material covering:

- what the platform measures,
- what the data will be used for,
- where the data will be stored,
- how the patient can access their own data,
- the recognized adverse events and their frequencies,
- the expulsion-and-retention pathway,
- the off-label status if any.

The consent surface is part of the deployment's required artifacts. A deployment whose consent surface is missing or incomplete is non-conformant.

### 7.2 Clinically Significant Findings

The platform's data may surface findings that are clinically significant. The deployment must have a documented pathway for explaining such findings to the patient at a clinical-literacy level appropriate to the population. A finding surfaced to a clinician but not communicated to the patient (where clinically indicated) is a deployment-level non-conformance.

## 8. Conformance Tests for Phase 3

The following tests must be reproduced at commissioning and at periodic re-certification:

- biocompatibility test reports for each declared material in its declared category,
- mechanical-envelope test under simulated GI conditions,
- chemical-envelope test under simulated pH and enzymatic conditions,
- energy-containment test under simulated breach,
- imaging-localizability test for Imaging and Sampling classes,
- consent-surface review with at least three patients in the deployment population.

Test reports are signed by the commissioner and attached to the conformance manifest.

## 9. Anti-Patterns

The following are explicitly non-conformant:

- a material in the manifest whose biocompatibility evidence is for a different body-contact category,
- a transit envelope declared at the population mean rather than at a high percentile appropriate for safety,
- a deployment without a documented retention pathway,
- a consent surface that hides the off-label status of any component,
- an adverse-event aggregation report whose denominator excludes platforms that did not transmit (selection bias).

## 10. Worked Example — Retention Event

A diagnostic-class platform is administered. Telemetry is normal for 18 hours, then ceases. The receiver detects cessation and waits the configured minimum gap. Cessation persists past the gap and the receiver transitions to `gap` state. After the platform's transit envelope plus the configured margin, the receiver's session does not transition to `complete`; instead, the prescribing-clinician alert fires.

The retention pathway documented in the deployment's manifest takes over: imaging localization is performed, the clinician reviews, the patient is informed, and the documented disposition is recorded. The session record at end is `end_reason == retained` and the lifecycle audit remains open until the platform is recovered or the case is closed.

If the platform is recovered, the recovery is itself an audit event and the housing is examined; any breach observed in recovery is fed into the deployment's recall analysis.

## 11. Recall Posture

The standard's posture on recall is preventive and rapid:

- a finding on a single platform is investigated to determine whether it is a one-off or a lot-level failure,
- a lot-level finding triggers a notice across all deployments that have administered platforms from the lot,
- the notice carries the deployment's recall-pathway including outreach to in-service patients (where transit may be ongoing) and to patients whose sessions are recently complete,
- the recall is itself a manifest-level event and is reflected in the lifecycle audit.

A deployment that suppresses or delays a lot-level finding for commercial reasons is, in the standard's posture, in serious non-conformance.

## 12. Glossary

- **Biocompatibility envelope.** The declared set of body-contact categories and material identifiers, with corresponding evidence packages.
- **Body-contact category.** Surface only, limited diffusion, or active release; determines evidence burden.
- **Expulsion guarantee.** The L1-level commitment to GI transit within the declared envelope.
- **Housing breach.** Loss of integrity of the platform housing in vivo; the most serious failure mode covered.
- **Lot traceability.** The provenance trail from raw material through manufactured platform to administered platform.
- **Off-label use.** Deployment outside the regulatory authorization for the platform; permitted only under amended manifest with prescribing-clinician and patient authorization.
- **Recall trigger.** A class of finding that obligates the deployment to identify and notify all in-service platforms of an affected lot.
- **Retention pathway.** The documented clinical workflow for handling a retained platform.
- **Transit envelope.** The declared P95 transit duration, supported by the prescribing protocol's assumptions about the patient population.

## 13. Document Conventions

**Must** carries normative force. **Should** indicates strong recommendation; deviations must be documented and justified in the manifest. **May** indicates a permitted option without preference.
