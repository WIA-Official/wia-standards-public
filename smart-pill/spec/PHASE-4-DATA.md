# WIA-MED-011: Smart Pill
## Phase 4 — Data Envelope, Audit & Clinical Integration

**Standard ID:** WIA-MED-011
**Category:** Medical — Ingestible Sensors
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 4 governs the data and the system around the platform: how telemetry becomes a clinical record, how the record is audited, how a clinician interacts with it, and how the platform's lifecycle is managed from commissioning through decommissioning. Phases 1–3 specify the platform; Phase 4 specifies the system the platform lives in.

## 2. Clinical Data Envelope

### 2.1 Per-Session Record

A *session* is the lifecycle of one administered platform from arming through expulsion or retention. The session record is the canonical clinical artifact.

```
SessionRecord := {
    session_id:           UUID,
    patient_id:           clinical-system identifier (linked under deployment policy),
    platform_id:          hardware-rooted identity,
    manifest_hash:        the manifest under which the platform was commissioned,
    armed_at, started_at, ended_at,
    end_reason:           Enum(expelled, retained, energy_exhausted, fault, withdrawn),
    samples:              List<Sample>,
    gaps:                 List<Gap>,
    triggered_events:     List<TriggeredEvent>,
    receiver_quality:     time-series of link-quality readings,
    audit_anchor:         hash of session in the WIA-SEC-017 chain
}
```

The record is signed at session end. A session with `end_reason == retained` does not, by itself, end the audit obligation: retention extends the lifecycle audit until the platform is recovered or until the deployment closes the case under its retention pathway.

### 2.2 Linkage to Patient Record

The deployment's policy declares how `patient_id` links to the institution's patient master record. The link must be auditable: the audit log shows when the link was created, by whom, with what authority. A platform whose data is linked to a patient record without an authorized linkage step is non-conformant in record provenance.

### 2.3 De-Identification for Research

A deployment may export de-identified session records for research. De-identification removes `patient_id` and any timestamps that could permit re-identification by linkage; aggregate temporal features (transit duration, sample-rate summary) are retained. The de-identification is applied at the data-egress boundary, not at L2 where it would compromise clinical utility.

A deployment that exports identifiable data under a de-identified label is non-conformant.

## 3. Audit Envelope

### 3.1 Record Format

Each non-routine event is logged with:

```
AuditRecord := {
    record_id, session_id, sequence, prev_hash,
    timestamp_local, timestamp_wall,
    layer, event_type, payload, justification,
    signature
}
```

The chain is verifiable end-to-end. A break is itself an incident.

### 3.2 Retention

The deployment's manifest declares retention horizons separately for routine session data and non-routine audit records. Routine session data is governed by the jurisdiction's clinical-records retention; non-routine audit records (housing breaches, retentions, off-label authorizations) are retained at the longer of the routine horizon and the standard's recommended floor for safety reviewability.

### 3.3 Cryptographic Destruction

Records past retention are destroyed by the key-destruction model: the keys used to encrypt the storage bucket are destroyed, rendering the encrypted records unreadable. Plain-text deletion alone is not conformant where the storage medium has known persistence properties.

## 4. Clinician Surface

### 4.1 Required Views

A conformant deployment provides clinician-facing views including, at minimum:

| View | Function |
|------|---------|
| Session overview | manifest summary, end reason, key derived findings |
| Sample explorer | per-channel time series with link-quality overlay |
| Triggered events | actuation or sampling events with justification packets |
| Adverse-event log | the deployment's adverse-event entries for this session |
| Patient-facing extract | the view the patient sees, available to the clinician |

The clinician's view must include the gaps (§2.1). A clinician must not be misled into reading inferred values as measured values; gaps are first-class.

### 4.2 Decision Support

Where the deployment includes decision support (a pattern-matching alert on the time series), the clinician's view must label the support's role as advisory. The decision belongs to the clinician. A view that obscures the boundary between measurement and inference is non-conformant.

### 4.3 Clinician Authorization

The clinician's authority to view a session is governed by the institutional access model. The view records the access in the audit log: who viewed what, when. A view that bypasses the institutional access model is, by construction, an unauthorized access.

## 5. Patient Surface

### 5.1 Required Affordances

The patient must be able to:

- view their own session record in plain language,
- request a clarification of any clinically-significant finding,
- export their session data in a portable format,
- request correction of any data entry that the patient believes is in error.

A deployment whose patient surface is hidden behind clinical-staff intermediation is non-conformant.

### 5.2 Communication Discipline

Communication of significant findings to the patient follows the prescribing-clinician's protocol. The deployment must provide a path for that communication; it must not preempt the clinician by surfacing an alarming label to the patient before the clinician has reviewed the record.

## 6. Multi-Site Deployments

### 6.1 Site Boundaries

A deployment that spans multiple clinical sites assigns each session to its originating site. The audit chain preserves the site context. Cross-site queries — population-level analyses across sites — are governed by an explicit data-sharing agreement reflected in the manifest.

### 6.2 Cross-Site Audit

A cross-site finding (a manufacturing-lot issue, a software defect surfacing the same way at multiple sites) is escalated to the deployment's program-level audit, which has read access to per-site audit chains for the purpose of constructing the cross-site picture without otherwise integrating the sites' clinical data.

## 7. Commissioning

The minimum steps:

1. Verify firmware hash and material lot certifications.
2. Load and sign the conformance manifest, including biocompatibility certifications, transit envelope, retention pathway, regulatory authorization, and consent surface.
3. Run the conformance test battery (Phase 2 §7 and Phase 3 §8); capture and sign results.
4. Register the platform's hardware-rooted public key with the WIA-SEC-017 endpoint.
5. Bind clinician, operator, and patient credentials/identities under the institutional access model.
6. Issue a commissioning audit record.

A platform not commissioned must not be administered. Test units used for benchtop work are administered to test phantoms or in a simulator; the standard's posture is that a platform reaching a patient has been commissioned.

## 8. Field Change Control

### 8.1 Re-commissioning Triggers

Re-commissioning is required for:

- firmware update,
- material-lot change,
- transit-envelope change,
- retention-pathway change,
- regulatory authorization change,
- consent-surface change.

A change made without re-commissioning is, by construction, an incident.

### 8.2 Temporary Authorizations

Off-label or temporary deviations are signed amendments with declared expiry, audit-logged. They must not silently extend.

## 9. Decommissioning

A deployment leaving the WIA-MED-011 program is decommissioned by:

1. closing in-flight sessions per the closure rules in §2.1,
2. publishing a notice to patients with active or recent sessions,
3. signing the final lifecycle audit and forwarding to the WIA registry,
4. revoking platform credentials from the audit endpoint,
5. ensuring all material-lot histories are retained for the recall window.

A silent exit — leaving public-facing claims active without backing operations — converts prior conformance into a misrepresentation.

## 10. Lifecycle Audit

The conformance manifest, biocompatibility test reports, mechanical and chemical envelope tests, retention pathway documentation, all session records and audit chains, periodic safety summaries, and all manifest amendments together constitute the lifecycle audit. A deployment must, on lawful request, produce the full lifecycle audit. A deployment unable to produce its lifecycle audit is non-conformant by omission.

## 11. Disaster Recovery

A deployment must rehearse, at minimum annually:

1. **Audit endpoint outage.** Demonstrate that receivers continue to operate, buffer audit records, and flow them once the endpoint returns.
2. **Recall.** Demonstrate the procedure for identifying all in-service platforms of an affected lot and notifying the prescribing clinicians.
3. **Patient access.** Demonstrate that a patient can retrieve their session record with the deployment's standard access pathway, with the result reaching the patient within the declared latency.
4. **Misidentification.** Demonstrate the procedure for correcting a platform-to-patient binding error.

Rehearsal reports are signed by the commissioner and retained as part of the lifecycle audit.

## 12. Inter-Standard Dependencies

This standard depends on:

- WIA-SEC-017 (Security Audit) for non-routine audit transport and storage.
- WIA-MED-001 (Medical baseline) for common medical-device interfaces where applicable.
- The deployment's jurisdictional medical-device regulation, which the standard integrates with rather than displaces.

A break in any dependency invalidates the WIA-MED-011 conformance claim until repaired.

## 13. Glossary

- **Session.** The lifecycle of one administered platform from arming through expulsion or retention.
- **Retention.** A platform that has not been expelled within its declared envelope; a clinical event with a documented pathway.
- **Justification packet.** The bundled evidence accompanying a triggered actuation or sampling event.
- **Body-contact category.** One of surface, limited diffusion, active release; determines the evidence burden for a material.
- **Manifest amendment.** A signed change to the conformance manifest with declared expiry, audit-logged.
- **Lifecycle audit.** The complete record of a deployment, retrievable on lawful request.

## 14. Document Conventions

**Must** carries normative force. **Should** indicates strong recommendation; deviations must be documented and justified in the manifest. **May** indicates a permitted option without preference.

## 15. Worked Example — Patient Subject Access

A patient who completed a diagnostic session three months prior submits a request for their own data. The deployment's patient surface returns:

- the session overview (when administered, when started, end reason),
- a plain-language summary of the clinically reviewed findings,
- a portable export of the time-series data in the standard's portable format,
- a statement of where the data is stored, how long it will be retained, and how the patient can request deletion within the retention rules.

The request, the response, and the access path are all audit-logged. A future audit can show that the patient received their data within the declared latency and through a documented pathway.

The patient surface is reachable without going through the prescribing clinician. It does, however, route any clinician-judgment questions to the clinician for a follow-up conversation; the surface is data-access, not clinical judgment substitution.

## 16. Inter-Operability with Health Records

A deployment that integrates with a national or institutional electronic health record (EHR) system maps the session record onto the EHR's vocabulary at the boundary. The mapping is signed and is part of the manifest. A finding from the smart-pill session, once mapped into the EHR, retains its provenance: the EHR record carries the session ID and the audit anchor so that a later reader can trace the EHR finding back to the underlying session.

A deployment that pushes derived findings to the EHR without preserving the audit anchor is non-conformant in record provenance: the EHR finding becomes a free-standing claim with no traceable basis.

## 17. Inter-Operability with Audit Standards

The standard's audit requirements integrate with WIA-SEC-017 specifically because the smart-pill data envelope crosses the boundary between an in-body platform and a clinical record. The standard prefers integration with WIA-SEC-017 over the deployment's bespoke logging because the integration ensures the same chain-of-custody guarantees apply across both routine clinical records and the higher-stakes events the smart-pill platform produces (triggered actuation, retention, off-label authorization).

A deployment that operates a parallel logging stack alongside WIA-SEC-017 must document the relationship between the two and ensure the audit anchor in the session record references the canonical chain. A deployment whose canonical audit chain is not integrated with WIA-SEC-017 is non-conformant in audit governance.
