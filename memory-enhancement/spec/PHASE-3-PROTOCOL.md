# WIA-memory-enhancement PHASE 3 — Protocol Specification

**Standard:** WIA-memory-enhancement
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols that bind
data records (PHASE 1) and API resources (PHASE 2) into
auditable longitudinal sequences: protocol-version
lifecycle, ethics-approval lifecycle, informed-consent
lifecycle, session execution, outcome capture cadence,
adverse-event escalation with regulator-clock honoured
windows, device-calibration cadence, and the audit-
event chain that every state change must emit. The
protocols are framed so a sponsor inspection can
reconstruct a subject's full trajectory from the event
log alone.

References (CITATION-POLICY ALLOW only):
- ICH E6 (R3) Good Clinical Practice — investigator brochure, source data, source documents
- ICH E2A — Clinical Safety Data Management — definitions and standards for expedited reporting
- ICH E2B (R3) — Electronic transmission of ICSR
- ICH E9 (R1) — Statistical Principles for Clinical Trials, addendum on estimands
- ISO 14155:2020 — Clinical investigation of medical devices
- ISO 14971:2019 — Risk management lifecycle
- ISO/IEC 27037 — guidelines for the identification, collection, acquisition and preservation of digital evidence
- IEC 62304 — Medical device software lifecycle processes (for SaMD updates)
- 21 CFR Part 11 — electronic records and signatures
- 21 CFR §312.32 (IND safety reporting) and §812.150 (IDE reporting)
- EU CTR (Regulation 536/2014) — clinical trials in the European Union
- EU MDR (2017/745) — medical devices regulation, vigilance procedure
- IETF RFC 5424 (Syslog), RFC 8941 (Structured Field Values), RFC 7515 (JWS)

---

## §1 Protocol-version lifecycle

A protocol-version is the immutable artefact under which
sessions execute. Versions are created by the sponsor,
reviewed by the IRB / IEC, and only become operational
after both ethics approval and a sponsor-side activation.

```
draft  →  submitted  →  approved  →  active  →  superseded
                                       │
                                       └──→ suspended → resumed | terminated
```

State transitions emit audit events with the actor, the
timestamp, and the IRB / sponsor decision reference.

`active` is a single-version invariant per protocol — at
any moment a protocol has at most one active version.
Amendments produce a new version that progresses
through draft → approved → active, at which point the
preceding version becomes `superseded`. Sessions in
flight when a version supersedes complete on the prior
version; new sessions open on the active version only.

## §2 Ethics-approval lifecycle

Approval bodies operate independently of the sponsor.
The protocol consumer (the operating site or the
sponsor's clinical operations) submits the approval
record after the IRB / IEC has decided.

```
pending  →  approved  →  expired  →  re-approved
            │           │
            └──→ suspended ──→ withdrawn | resumed
```

`expired` is a hard gate: the API returns `409 Conflict`
with type `ethics-expired` for every session-open
attempt, and the sponsor must re-approve before
re-opening sessions.

## §3 Informed-consent lifecycle

Consent is per subject and per protocol-version. A
subject who has consented to version 1.0 has not
implicitly consented to version 1.1; re-consent is
required for any amendment that changes the risk
profile, the procedures, or the data-retention policy.

```
prepared  →  signed  →  active  →  withdrawn
                        │
                        └──→ re-consent-pending  →  re-consented
```

Withdrawal stops further data collection; data already
de-identified for prior analyses are retained per the
IRB-approved retention plan and the regulator
requirements (FDA, EMA, MFDS, PMDA all accept retention
of trial records for a fixed minimum after study close).

## §4 Session execution

A session executes in five steps:

```
pre-flight  →  prepare  →  deliver  →  observe  →  close
```

1. **pre-flight** — verify active consent, active ethics
   approval, valid device calibration, eligible subject
   status, and rater independence (for arms requiring
   blinded assessment).
2. **prepare** — randomisation envelope opens (for
   blinded studies via Interactive Response Technology
   — IRT — kept independent of the operator); subject
   identity verified by site SOP; vitals captured.
3. **deliver** — intervention executes; for tDCS / tACS
   the device measures actual current vs. set-point
   per second and the deviation is recorded; for TMS
   pulse counter is logged; for drugs the actual dose
   and timing are recorded.
4. **observe** — outcome instruments administered by an
   independent rater; raw answers and standardised
   scores written to the outcome record.
5. **close** — session is sealed and the audit hash
   chain extended with the session payload signature.

A session that closes with an open AE classified as
serious cannot be sealed until the AE record is
filed. Attempting to close returns `409 serious-ae-
unreported`.

## §5 Outcome capture cadence

Outcome instruments are administered:

- at screening (baseline)
- per session if the protocol calls for per-session
  measurement (typical for digital cognitive training)
- at scheduled milestones (4 weeks, 12 weeks, 24 weeks,
  end-of-study) per the protocol's schedule of
  assessments
- at unscheduled visits triggered by AE follow-up

Standardised instruments must be administered in the
language version the subject consented to and by raters
trained on the protocol's rater-certification SOP.

## §6 Adverse-event escalation

| Severity / kind                 | Sponsor clock | Regulator notification    |
|---------------------------------|--------------:|---------------------------|
| Serious unexpected, fatal /     |        7 days | FDA 3500A; EMA EVDAS;      |
| life-threatening, related       |               | PMDA J-AER; MFDS K-PV     |
| Serious unexpected, related     |       15 days | (same)                    |
| Serious expected, related       |     periodic  | DSUR / PSUR cycle         |
| Non-serious                     |     periodic  | DSUR / PSUR cycle         |
| Device-related, public-health   |    immediate  | FDA MedWatch UDI; EU EUDAMED |

The escalation timer starts at sponsor awareness of the
event meeting reportability criteria (per ICH E2A); the
API logs awareness time and notification time so the
clock is auditable.

## §7 Device-calibration cadence

| Intervention | Calibration interval                                        |
|--------------|-------------------------------------------------------------|
| tDCS / tACS  | 90 days or 200 sessions, whichever first                    |
| TMS          | 30 days; coil-temperature check per session per IEC 60601-2 |
| DBS          | annual programmer verification; per-visit IPG telemetry     |
| BCI          | per-protocol; software update under IEC 62304 lifecycle     |
| EEG          | per IEEE 11073-10406 reference-tone; daily impedance check  |

Calibration records are immutable and link to the
operator-credential record so an inspector can verify
the calibration was performed by a qualified party.

## §8 Audit event chain

Every API state change emits one audit event with the
following minimum fields:

| Field          | Meaning                                                   |
|----------------|-----------------------------------------------------------|
| `eventId`      | UUID                                                      |
| `eventTime`    | ISO 8601 with timezone                                    |
| `actor`        | identity (clinician / operator / sponsor / regulator)     |
| `subjectRef`   | the affected subject record (if applicable)               |
| `resourceRef`  | the resource that changed (URI of session / outcome /...) |
| `action`       | created / updated / closed / withdrawn / superseded       |
| `priorHash`    | SHA-256 of the prior event payload                        |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)  |

The chain is per-protocol and is exportable in syslog
RFC 5424 envelopes for SIEM ingestion.

## §9 Protocol-deviation handling

Deviations are recorded as their own resource (referenced
from the session). Severity tiers per ICH E6 (R3):

- minor — no impact on subject rights / safety / data integrity
- major — potential impact on data integrity but not on safety
- critical — impact on rights / safety / scientific value

Critical deviations trigger an immediate sponsor / IRB
notification and pause new sessions on the affected arm
until reviewed.

## §10 Re-consent triggers

Re-consent is required when:

- protocol changes alter risk profile, procedures, or
  retention plan
- a new intervention arm is added
- a relevant new safety finding emerges
- the subject's legal representative changes (paediatric
  age-up, return of capacity)

The subject's prior data remain usable per the original
consent; new data require fresh consent.

## §11 Source-data integrity

All source data is captured at the point of generation
and stored in tamper-evident form (hash-chained event
log). Reconstruction by inspection of the event log
must be byte-equivalent to the API exposure of the
record.

## Annex A — Worked AE escalation example (informative)

A subject in a tDCS arm experiences a tonic-clonic
seizure during session 12. Operator pauses session,
triggers emergency response, and within the protocol's
2-hour window the operator files an AE through `/v1/
adverse-events`. The AE classifies as `serious=true,
expected=false, severity=life-threatening, causality=
probable`, opening the 7-day clock. Sponsor pharmaco-
vigilance prepares and transmits an E2B(R3) ICSR to FDA
within 5 days. The IRB receives the safety report and
issues a temporary halt of new enrolments pending
investigation. The protocol's data-monitoring committee
convenes on day 6 and issues a recommendation to amend
the inclusion / exclusion criteria.

## Annex B — Conformance disclosure

Implementations declare the version of the audit-chain
schema, the JWS algorithm registry they support, and
the time-source they use (e.g. NIST, KASI, or NTP
stratum-1).
