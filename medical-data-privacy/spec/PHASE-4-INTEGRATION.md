# WIA-medical-data-privacy PHASE 4 — Integration Specification

**Standard:** WIA-medical-data-privacy
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a real deployment integrates the privacy
artefacts, API surface, and protocols defined in PHASEs 1–3 with
existing healthcare infrastructure: EHR systems, research data
warehouses, public-health reporting, regulator gateways, and
breach-response workflows. It is non-prescriptive about specific
vendor products; it specifies the integration *contracts* a deployment
must satisfy.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Bulk Data Access (R5/bulkdata.html)
- HL7 SMART App Launch 2.2
- IHE ATNA — audit trail and node authentication
- IHE PIX/PDQ — patient identifier cross-referencing (informational)
- ISO/IEC 27002:2022 — controls implementation guidance
- ISO/IEC 27018:2019 — controller-to-processor PII flows
- NIST SP 800-66r2 — HIPAA Security Rule implementation guidance
- DICOM PS3.15 (Security and System Management Profiles) — for imaging integrations

---

## §1 EHR integration

The most common integration target is an existing EHR. The privacy
boundary sits *in front of* the EHR's FHIR API, not inside the EHR.
The EHR continues to hold the raw clinical record; the privacy
boundary brokers every read and write through the consent gate.

Integration responsibilities split:

| Component        | Responsibility                                              |
|------------------|-------------------------------------------------------------|
| EHR              | clinical workflow, raw record storage, MRN management       |
| Privacy boundary | consent gate, purpose enforcement, AuditEvent emission      |
| Identity broker  | MRN ↔ subjectRef mapping, linkage auth records              |
| Audit store      | hash-chained AuditEvents, daily root signing                |
| KMS / HSM        | controller signing key, JWKS publication                    |

The EHR exposes its FHIR API on the controller's internal network.
The privacy boundary is the only client. External callers reach the
boundary, never the EHR directly.

## §2 Research warehouse integration

Research access flows through the Bulk Data Access (FHIR R5
$export) operation defined in PHASE 2 §7. The warehouse provides:

- the cohort selector (FHIR search expression)
- the consent bundle (consent receipt URIs naming each subject)
- the de-identification job specification (PHASE 1 §7)

The boundary executes the export against the cohort intersected
with the consent bundle (i.e., subjects without an active research
consent are excluded silently — their absence from the export is
not signalled, only their presence in the consent bundle is). The
warehouse receives an NDJSON archive plus the de-identification
job record plus the manifest signature.

Re-identification of a research extract requires a linkage
authorisation per PHASE 1 §5. A research warehouse that performs
re-identification without invoking the broker has, by construction,
falsified its own audit trail — auditing this falsification is the
sponsoring institution's IRB responsibility.

## §3 Retention monitoring

The retention engine described in PHASE 3 §7 needs operational
inputs from the EHR (resource types and creation timestamps) and
from the consent gate (consent expiry dates). The integration
contract is:

- The EHR exposes a daily retention candidate feed listing
  resources whose age exceeds the per-jurisdiction retention
  thresholds.
- The boundary intersects the candidate feed with active
  consents (a resource still under active consent for any
  purpose is *not* tombstoned — consent extends retention as
  long as the consent itself is valid).
- The boundary writes tombstones into the EHR via FHIR PUT.

A tombstone is *not* a deletion. The resource is preserved with
`status: tombstoned` and a `meta.security` tag indicating the
retention rule that applied. Real deletion is a manual two-person
operation guarded by §6.

## §4 Public-health reporting

Reporting to public-health authorities follows the jurisdiction's
notifiable-disease workflow (CDC NEDSS, KCDA 감염병 신고, etc.).
The integration contract:

- Notifiable conditions emit a one-shot disclosure with purpose
  `PUBHLTH` to the public-health authority's intake endpoint.
- The disclosure is recorded in the AuditEvent chain with the
  authority's controller URN as the recipient.
- Public-health-purpose disclosures bypass the per-subject consent
  check (because the legal basis is statutory, not consent), but
  still record an AuditEvent and emit a disclosure manifest.

The deployment policy enumerates which conditions are notifiable
under the controlling jurisdiction so that the boundary can route
notifications without per-call clinician judgement.

## §5 Regulator interface

Regulators (HIPAA: HHS OCR; GDPR: the supervisory authority of the
controller's jurisdiction; K-PIPA: 개인정보보호위원회) request
audit windows during investigations. The integration contract:

- The regulator presents a signed request naming the date range,
  the subject scope, and the investigation ID.
- The boundary returns the AuditEvents intersecting the request,
  the daily roots covering the window, and the inclusion proofs
  binding events to roots.
- The boundary emits an AuditEvent recording the regulator's
  request (so that a regulator-initiated audit is itself audited).

The deployment SHOULD support online proof verification by the
regulator (the regulator can replay the chain reconstruction
without touching the controller's primary store).

## §6 Two-person integrity for destructive operations

The following operations require two distinct authenticated
principals:

- Hard deletion of a tombstoned resource (§3)
- Issuance of a "global linkage" authorisation crossing
  controller boundaries
- Rotation of the controller's signing key (PHASE 3 §3) under
  emergency circumstances (planned rotations are single-principal)

Each two-person operation emits an AuditEvent referencing both
principals. The boundary's policy engine rejects any single-principal
attempt with an RFC 9457 problem of
`urn:wia:mdp:problem:two-person-required`.

## §7 Breach-response workflow

A suspected breach surfaces from one of several detectors:

- AuditEvent anomaly detection (unusual read patterns, off-hours
  access, scope creep)
- Self-service report from a subject ("I see records I shouldn't")
- External notification (vendor alert, third-party security
  research disclosure)

On detection, the workflow:

1. Creates a breach investigation record (PHASE 1 §8).
2. Freezes the implicated principal's session token (revocation
   list updated, JWT cache invalidated).
3. Clones the relevant AuditEvent window into a forensic store.
4. Times the regulator-notification clock (HIPAA ≤60 days; GDPR
   ≤72 hours; K-PIPA 사실 인지 후 지체 없이).
5. Times the subject-notification clock per jurisdiction.
6. Reviews and signs off the breach record at +72h, +30d, and
   on regulator notification.

The workflow does not promise that every breach is recoverable;
it promises that every breach is *visible* and *clocked* so that
the controller's response is provably timely.

## §8 Telemetry and observability

The boundary exposes operational metrics consistent with the
clinical workflow's tolerance:

- Consent-check latency p50 / p95 / p99 — should not perceptibly
  affect read latency in the clinician's UI
- AuditEvent write success rate — failures here block reads (§9
  of PHASE 3)
- Tombstone backlog — resources past retention but not yet
  tombstoned (target: 0 over rolling 7-day window)
- Daily-root publication latency — root signed by 23:59:59.999Z,
  published within 5 minutes
- Break-glass review backlog — events past 72h without recorded
  review (target: 0)

These metrics are themselves PHI-free; they describe the system's
behaviour, not its records.

## §9 Acceptance criteria for a deployment

A deployment claims conformance with this standard when:

1. Every read or disclosure in the past quarter has a matching
   AuditEvent in the chain.
2. Every AuditEvent is bound to a daily root with a verifiable
   inclusion proof.
3. Every consent receipt has a verifiable JWS signature against
   the controller's published JWKS.
4. The retention backlog is zero over the prior 30 days.
5. The break-glass review backlog is zero over the prior 30 days.
6. The deployment's jurisdiction declaration matches the
   regulator on file.
7. A spot-check of 1% of disclosures from the prior quarter
   reproduces the consent gate decision deterministically when
   replayed.

A deployment failing any of these is in non-conformance and
SHOULD report the gap in its compliance package (PHASE 3 §4
audit chain) rather than concealing it.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Deployment checklist (informative)

A new deployment claims conformance after a checklist sign-off:

- [ ] Jurisdiction declared in deployment policy and matches the
      regulator on file
- [ ] FHIR base URL has a published `/metadata` with the WIA capability
      extensions (PHASE 2 Annex A)
- [ ] JWKS published at `/.well-known/jwks.json` with current signing key
- [ ] HSM holding signing key is FIPS 140-3 Level 2 (or KCMVP for KR)
- [ ] Identity broker reachable only on internal network
- [ ] AuditEvent chain initialised with a signed genesis root
- [ ] Retention engine running daily, confirmed against the
      jurisdiction-specific table
- [ ] Break-glass review queue populated and reviewed in dry-run
- [ ] Two-person integrity policy applied to deletion endpoints
- [ ] Disaster-recovery test completed with audit-chain reconstruction
- [ ] Initial spot-check of 1% of disclosures replays deterministically

## Annex B — Coexistence with legacy interfaces (informative)

Many existing healthcare systems predate FHIR and this standard. The
deployment can bridge:

- HL7 v2 ADT messages — converted at the boundary into Consent and
  AuditEvent resources; the ADT message itself is preserved in the
  audit store as evidence.
- DICOM imaging — DICOM PS3.15 audit profile is mapped onto the
  AuditEvent chain so that imaging reads share the same chain as
  clinical reads. ATNA conformance is achieved by tunnelling ATNA
  syslog into the boundary's audit ingest.
- HL7 CDA documents — XML Signature signatures (W3C XMLDSig) are
  preserved alongside the JWS receipt; auditors can verify either.

The bridge is *one-way at the boundary*: legacy data flowing into
the boundary acquires WIA privacy artefacts; legacy data flowing out
is gated by the same artefacts. A legacy interface that bypasses the
boundary is non-conformant by construction.

## Annex C — Vendor-neutral migration note (informative)

A deployment migrating from a vendor's proprietary audit format
into this PHASE follows a one-time backfill:

1. The vendor's historic audit events are exported in their native
   format, preserving original timestamps and original signatures
   (where any).
2. Each event is wrapped in a FHIR AuditEvent envelope. The
   wrapper carries the original payload as `entity.detail`, so
   the migration is non-lossy — auditors can still read the
   pre-migration record exactly as it was.
3. Wrapped events are inserted into the chain in original timestamp
   order, with each wrapper signed at insertion time. The chain
   genesis root for the post-migration era references the last
   wrapped event's hash.
4. A migration manifest documents the boundary between pre-migration
   wrapped events and native post-migration events, signed by both
   the outgoing and incoming custodians.

This preserves audit continuity across vendor changes without
inventing fictitious signatures on historic data. The deployment
policy SHOULD describe migration rules so that auditors can
distinguish original signatures from migration wrappers.

## Annex D — Decommissioning

When a deployment is decommissioned, the controller signs a final
audit-chain root, exports the signed chain to the receiving
controller (or to a long-term archive controlled by the regulator
for jurisdictions that require it), and revokes all session tokens.
The decommissioning manifest is itself an AuditEvent in the
final chain root and is preserved by both parties for the
audit-trail retention period.
