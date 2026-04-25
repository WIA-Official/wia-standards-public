# WIA-MED-011: Smart Pill
## Phase 1 — Core Architecture

**Standard ID:** WIA-MED-011
**Category:** Medical — Ingestible Sensors
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Scope

A *smart pill* under WIA-MED-011 is an ingestible electronic platform whose operational duty is to perform sensing, signaling, or controlled drug release inside the gastrointestinal tract and to transmit data to an external receiver. The standard normalizes the architectural layers, data envelopes, and safety guarantees a conformant platform must provide.

Out of scope: pure inert capsules without electronic components; implantable devices that remain in the body beyond gastrointestinal transit; ingestible devices intended to perform endoscopic surgery or tissue ablation (those fall under separate medical-device standards).

Two characteristics distinguish a WIA-MED-011 platform from a generic ingestible: it carries an electronic measurement or actuation function, and it transmits information out of the body during its transit window.

## 2. Layered Architecture

A conformant platform exposes four logical layers:

```
+----------------------------------------------------+
|  L4 — Clinical Integration                         |
|       (EHR ingest, clinical workflow, audit)       |
+----------------------------------------------------+
|  L3 — External Receiver                            |
|       (skin patch / handheld, decoding, buffering) |
+----------------------------------------------------+
|  L2 — In-Body Telemetry                            |
|       (sensor sampling, modulation, link)          |
+----------------------------------------------------+
|  L1 — Capsule & Safety                             |
|       (housing, energy, expulsion guarantee)       |
+----------------------------------------------------+
```

Each layer has a normative interface that must be present and correctly typed for conformance, and an advisory interface vendors may expose for richer integration.

### 2.1 L1 — Capsule & Safety

L1 is the physical platform: housing, sensor stack, energy source, and the safety properties that govern transit and expulsion. It owns:

- biocompatible housing materials within the declared envelope,
- the energy budget that must allow declared transit duration,
- the integrity of the housing against mechanical and chemical insults of the GI environment,
- the expulsion guarantee — the platform must transit the GI tract along the same path as ordinary digesta within the declared window absent a documented retention pathway authorized by the prescribing clinician,
- a fail-safe state on energy exhaustion: the platform must not become an active hazard when its energy is depleted; it must remain biocompatible and follow the expulsion path.

The expulsion guarantee is normative. A platform whose retention rate exceeds the declared bound is non-conformant. Retention is a clinical event and is itself an audit event for the deployment.

### 2.2 L2 — In-Body Telemetry

L2 performs sensing, encodes the result, and modulates a signal that the external receiver can pick up. It owns:

- the sampling discipline (rate, gating, calibration),
- the channel modulation declared in the manifest,
- the per-sample integrity attached to outgoing telemetry,
- the duty cycle and energy planning that respects the L1 energy budget.

L2 must respect the energy envelope of L1. A platform whose telemetry exceeds its declared duty cycle and runs the battery down faster than its declared transit duration is non-conformant.

### 2.3 L3 — External Receiver

L3 is the wearable or bedside receiver: it captures the signal, decodes it, buffers it locally, and forwards it to L4. The receiver must:

- decode and timestamp every received sample,
- detect and report missed samples,
- buffer locally during loss-of-connectivity to L4,
- expose its battery and signal-quality state to the user,
- cryptographically sign forwarded packets to L4 with a hardware-rooted identity.

A receiver that silently drops samples without surfacing the loss to the user is non-conformant. The clinical reading of a smart-pill record must include the gaps as well as the data.

### 2.4 L4 — Clinical Integration

L4 is the system that ingests the data into the patient record, presents it to the clinician, and audits the lifecycle of the platform. L4 is normatively integrated with WIA-SEC-017 for tamper-evident clinical logging.

## 3. Service Classes

A platform declares one or more service classes in its conformance manifest. The class determines which behavior modules the platform may host.

| Class | Examples |
|-------|---------|
| Adherence | medication-ingestion confirmation |
| Sensing | pH, pressure, temperature, gas composition |
| Imaging | endoscopic capsule (where covered by joint standard) |
| Sampling | localized sampling at a triggered or scheduled site |
| Delivery | controlled-release at a triggered site |

A platform may host a behavior module only for a class it has declared, and the modules are bound to the manifest. Cross-class deployment requires re-commissioning.

## 4. Conformance Manifest

Every conformant deployment publishes a manifest containing:

- platform identity (vendor, model, lot, firmware hash),
- declared service classes,
- declared materials and biocompatibility certifications,
- declared energy source and transit-time envelope,
- declared external-receiver model and decoding profile,
- the WIA-SEC-017 audit endpoint and its public key,
- the clinical workflow into which the platform integrates,
- the prescribing-physician authorization model.

The manifest is signed at commissioning. Field changes require a new signature.

## 5. Identity, Roles, and Privacy

The standard defines five normative roles. A conformant deployment maps every actor in its administration model to exactly one role.

| Role | Authority |
|------|----------|
| `patient` | the recipient of the platform; subject of the data |
| `prescribing_clinician` | authorizes administration; receives clinical results |
| `operator` | runs the deployment day-to-day; assists patients and clinicians |
| `commissioner` | signs the conformance manifest |
| `auditor` | read-only access to the lifecycle audit |

The patient is a first-class role in this standard, not merely a data subject. The patient must be able to view their own platform-derived data, request correction, and receive an explanation of clinically significant findings without going through the deployment as an intermediary.

## 6. Threat Model

The standard's threat surface is governed by three concerns:

| Concern | Examples | Mitigation |
|---------|---------|-----------|
| Privacy | unauthorized access to ingested-platform data | per-platform keys; end-to-end encryption to L4 |
| Integrity | data tampering between L3 and L4 | signed packets; chained audit at L4 |
| Safety | retention beyond envelope, housing breach | expulsion guarantee, biocompatibility certification |

Each concern is mitigated at the layer best able to address it: privacy at L2/L3 (encryption and keying), integrity at L3/L4 (signatures and chaining), safety at L1 (housing and material).

## 7. Time and Identity Discipline

Every telemetry sample carries a monotonic timestamp from L2 plus a wall-clock time from L3 at receipt. Disagreement beyond the declared envelope raises an alert; an out-of-sync platform is, for clinical purposes, an unreliable platform.

Platform identity is bound to a hardware-rooted key burned at manufacture. Software-only identities are non-conformant.

## 8. Versioning

The standard uses semantic versioning. The version identifier is part of every audit record so a record can be interpreted under its version's rules even if the deployment has since been upgraded.

## 9. Document Conventions

**Must** carries normative force. **Should** indicates a strong recommendation; deviations must be documented and justified in the manifest. **May** indicates a permitted option without preference.

## 10. Phase Roadmap

- Phase 2 covers in-body telemetry and the wireless link.
- Phase 3 covers safety: biocompatibility, expulsion, regulatory framing.
- Phase 4 covers data envelope, audit, and clinical integration.

## 11. Conformance Tiers

The standard recognizes three deployment tiers, declared at commissioning:

| Tier | Authorized Settings | Requirements |
|------|--------------------|----------------|
| Adherence-only | medication ingestion confirmation in outpatient or self-administered programs | minimum sensor envelope; receiver-only telemetry |
| Diagnostic | clinical sensing under prescribing-clinician supervision | full Phase 2 telemetry; biocompatibility evidence per Phase 3; clinical workflow integration |
| Therapeutic | controlled drug release or active sampling under clinical supervision | additional regulatory authorization for the released agent or the sampling apparatus; signed actuation chain |

A platform commissioned for one tier may not be deployed at a higher-authority tier without re-commissioning. Redeployment to a lower tier is permitted without re-commissioning since the lower tier's envelope is a subset, but the redeployment is itself an audit event.

## 12. Layer Boundary Enforcement

Cross-layer calls are gated. L2 must reject any L1 fault that would compromise telemetry integrity. L3 must reject any L4 command that would direct a triggered actuation outside the platform's authorized class. Rejection is itself an audit event so that an operator reviewing the log can see when a higher layer attempted an unauthorized action.

The boundary uses a small declarative interface: every cross-layer message is typed against a published schema, every message carries a request ID that propagates to the audit record, and every reject path produces a structured rejection rather than a silent drop.

## 13. Glossary

- **Adherence platform.** A class focused on confirming medication ingestion through detection of digestive activation.
- **Body-contact category.** Surface, limited diffusion, or active release; determines biocompatibility evidence burden.
- **Conformance manifest.** The signed document declaring platform identity, material stack, declared classes, and authorizations.
- **Expulsion guarantee.** The L1-level commitment that the platform transits the GI tract along the same path as ordinary digesta within the declared envelope.
- **External receiver.** The L3 device — typically a skin patch or handheld — that decodes the in-body link and forwards data to L4.
- **Hardware-rooted identity.** The platform's identity bound to a non-extractable key burned at manufacture.
- **In-body telemetry.** The L2 function of sampling, encoding, and transmitting from inside the GI tract.
- **Justification packet.** The bundled evidence accompanying any triggered event.
- **Retention.** A platform not expelled within its declared envelope; a clinical event with documented pathway.
- **Session.** The lifecycle of one administered platform from arming through expulsion or retention.
- **Telemetry gap.** A discontinuity in received samples; recorded as a first-class entity in the session record.

## 14. Anti-Patterns

- A platform whose hardware-rooted identity is stored in software-replaceable form. The identity must survive firmware update by construction.
- A manifest that declares a service class the platform's L2 cannot deliver. A platform without a triggered-actuation surface may not declare itself for the Delivery class.
- A deployment that treats the patient as a data subject only, with no patient-facing affordances. The patient is a first-class role under §5.
- A deployment that conflates platform-derived clinical findings with diagnostic claims. The platform measures; the clinician diagnoses. A platform that markets itself as a diagnostic in deployment material is overreaching its scope under this standard.
