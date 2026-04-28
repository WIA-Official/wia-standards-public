# WIA-physical-enhancement PHASE 3 — Protocol Specification

**Standard:** WIA-physical-enhancement
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
data records and API surface into auditable longitudinal
sequences: protocol-version lifecycle, ethics-approval
lifecycle, informed-consent lifecycle, sport-governance
binding lifecycle (including TUE workflow), session
execution, performance capture cadence, adverse-event
escalation, device-calibration cadence, anti-doping
testing intersection, and the audit-event chain.
Protocols are framed so an inspection by a sport-
governance body, a regulator, or an IRB can reconstruct
the subject's full timeline from the event log.

References (CITATION-POLICY ALLOW only):
- World Anti-Doping Code (WADA) — current edition
- WADA International Standard for Therapeutic Use Exemptions (ISTUE)
- WADA International Standard for Laboratories (ISL)
- WADA International Standard for Testing and Investigations (ISTI)
- IPC Athlete Classification Code
- ICH E6 (R3) Good Clinical Practice
- ICH E2A — clinical safety reporting
- ICH E2B (R3) — ICSR transmission
- ISO 14155:2020 — clinical investigation of medical devices
- ISO 14971:2019 — risk-management lifecycle
- ISO 13482 — personal-care robots safety
- IEC 62304 — medical-device software lifecycle
- 21 CFR Part 11 — electronic records / signatures (US)
- EU CTR (Regulation 536/2014); EU MDR (2017/745)
- ISO/IEC 27037 — digital evidence preservation
- IETF RFC 5424 (Syslog), RFC 7515 (JWS)

---

## §1 Protocol-version lifecycle

```
draft → submitted → approved → active → superseded
                              │
                              └→ suspended → resumed | terminated
```

Active is single-version invariant; sessions in flight
when a version supersedes complete on the prior version.

## §2 Ethics-approval lifecycle

```
pending → approved → expired → re-approved
          │
          └→ suspended → withdrawn | resumed
```

Expiry is a hard gate; sessions cannot open under an
expired approval.

## §3 Informed-consent lifecycle

```
prepared → signed → active → withdrawn
                    │
                    └→ re-consent-pending → re-consented
```

Re-consent is required when a protocol amendment
changes the risk profile, procedure set, or data-
retention plan; sport-context consents additionally
re-issue when WADA Code or IF rule changes affect the
intervention's classification.

## §4 Sport-governance binding lifecycle (TUE)

```
prepared → submitted → reviewed → granted → active → expired
                                  │
                                  └→ rejected → appealed → upheld | overturned
```

A subject in a sport context running a prohibited
intervention requires a granted TUE before the session
opens. The TUE record links to the WADA ADAMS submission
and to the supporting clinical evidence (PHASE 1 §7).

## §5 Session execution

A session executes in five steps:

```
pre-flight → prepare → deliver → observe → close
```

1. **pre-flight** — verify active consent, active ethics
   approval (clinical / occupational), active TUE
   (sport-context prohibited interventions), valid
   device calibration, eligible subject status, and
   rater independence (where blinded measurement is
   protocol-required).
2. **prepare** — randomisation envelope opens (clinical
   trials); subject identity verified; vital signs and
   readiness checks captured.
3. **deliver** — intervention executes; powered-device
   sessions stream telemetry per second; pharmacological
   sessions record administered dose with method and
   route per ATC and WHOCC routes.
4. **observe** — performance instruments administered
   by an independent rater; raw and standardised scores
   recorded.
5. **close** — session sealed; audit hash chain extends
   with the session payload signature.

Sessions cannot be sealed while an open serious AE is
pending the AE record (§7).

## §6 Performance capture cadence

| Context        | Capture cadence                                         |
|----------------|---------------------------------------------------------|
| Clinical       | screening, weekly, milestone (4 / 12 / 24 weeks), end-  |
|                | of-study, unscheduled AE follow-up                      |
| Occupational   | pre-shift baseline, post-shift, milestone (monthly),    |
|                | post-incident                                           |
| Sport          | per training block (mesocycle), pre-competition,        |
|                | post-competition, mandatory sample-collection windows   |

Standardised instruments must be administered in the
language version the subject consented to and by raters
trained on the protocol's certification SOP.

## §7 Adverse-event escalation

| Severity / kind                | Sponsor clock | Regulator notification    |
|--------------------------------|---------------|---------------------------|
| Serious unexpected, fatal /    |        7 days | FDA 3500A / EMA EVDAS /   |
| life-threatening, related      |               | PMDA / MFDS / Health Canada|
| Serious unexpected, related    |       15 days | (same)                    |
| Serious expected, related      |     periodic  | DSUR / PSUR cycle         |
| Device, public-health          |    immediate  | EU EUDAMED / FDA MedWatch  |
| Sport-context AE that may      |    immediate  | IF medical commission /    |
| affect classification or TUE   |               | NADO                       |

The clock starts at sponsor awareness; the API logs
both awareness time and notification time.

## §8 Device-calibration cadence

| Intervention            | Calibration interval                          |
|-------------------------|-----------------------------------------------|
| Powered exoskeleton     | 90 days or 200 sessions                       |
| Orthosis (passive)      | per IEC 60601-2-78 fitting protocol            |
| Prosthesis-augment      | per IEEE 11073-10406 reference impedance      |
| Force-plate / dynamometer| ISO 17025 traceable annual                    |
| Gait-analysis camera    | per-session marker calibration                 |

Calibration records are immutable and link to the
operator credential.

## §9 Anti-doping testing intersection

When a subject is in a registered testing pool (RTP)
or an out-of-competition testing programme, the
protocol records:

- whereabouts compliance events (filed, missed, late)
- doping-control sample collection events (in-
  competition / out-of-competition)
- ADAMS submission references
- chain-of-custody for samples submitted to a WADA-
  accredited laboratory under the ISL

A session that is concurrent with a doping-control
event records the event reference so an inspector can
correlate intervention timing to sample collection.

## §10 Audit event chain

| Field          | Meaning                                                   |
|----------------|-----------------------------------------------------------|
| `eventId`      | UUID                                                      |
| `eventTime`    | ISO 8601 with timezone                                    |
| `actor`        | identity (clinician / operator / coach / sport-body)      |
| `subjectRef`   | the affected subject record (where applicable)            |
| `resourceRef`  | URI of the resource that changed                          |
| `action`       | created / updated / closed / withdrawn / superseded       |
| `priorHash`    | SHA-256 of the prior event payload                        |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)  |

The chain is per-protocol and per-sport-binding.

## §11 Protocol-deviation handling

Severity tiers per ICH E6 (R3): minor, major, critical.
Critical deviations trigger sponsor / IRB / IF
notification and pause new sessions on the affected arm
until reviewed.

## §12 Re-consent triggers

Re-consent is required when:

- protocol amendment changes risk, procedures, or
  retention
- new intervention arm added
- new safety finding emerges
- subject's legal representative changes
- WADA Code or IF rule update changes the intervention's
  permitted / restricted / prohibited classification
- adaptive-class re-evaluation affects participation

## §13 Source-data integrity

All source data is captured at point of generation and
stored in tamper-evident form. The audit-chain export
is byte-equivalent to the API exposure of the record.

## Annex A — Worked AE escalation example (informative)

A subject in an exoskeleton trial trips and falls
during an over-ground session. Operator pauses session,
escalates to local emergency response, and within the
2-hour protocol window files an AE through `/v1/
adverse-events`. The AE classifies as `serious=true,
expected=false (severity=severe), causality=possible`.
The 15-day clock opens; sponsor pharmacovigilance
prepares the ICSR for FDA ESG. The EU MDR vigilance
notification (EU EUDAMED) is filed in parallel. The IRB
issues a temporary halt on over-ground sessions pending
device-design review.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema version,
the JWS algorithm registry, the time-source authority,
and the WADA / IF protocol bindings supported.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Operator-credential binding

| Credential                  | Source                                |
|-----------------------------|---------------------------------------|
| Physical / occupational therapy | national board (e.g. APTA, KOPTA) |
| Sports medicine             | FMS / IOC sports medicine diploma     |
| Strength and conditioning   | NSCA / KSCA / equivalent national     |
| Anti-doping educator        | WADA-recognised certification         |
| Adaptive-sport classifier   | IPC trained classifier                |

A session cannot release results signed by an operator
without an active credential.

## Annex E — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, PTB). Sport contexts
additionally record the in-competition local time per
the IF rules (some federations use stadium clock as
the authoritative reference for in-competition events).

## Annex F — Adaptive-class re-evaluation cadence

| Trigger                              | Action                          |
|--------------------------------------|---------------------------------|
| Initial international classification | record class + minimum disability |
|                                      | criteria, schedule review        |
| Performance-based observation        | classifier review at next-event   |
| Medical condition change             | mandatory re-evaluation          |
| Class protest                        | classifier panel review          |
| Year-cycle review                    | per IF rule (typically 4-year    |
|                                      | Paralympic cycle)                |

Re-evaluation events bind to the sport-governance
record so an inspector can verify the active class is
the one the subject competes under.

## Annex G — Whereabouts compliance recording

For RTP / NRTP athletes the protocol records:

| Event                      | Recorded fields                              |
|----------------------------|----------------------------------------------|
| Quarterly filing           | filing date, period covered, content hash    |
| 60-minute slot declaration | per-day slot, location, alternate slot       |
| Filing failure             | type (no filing, late filing, incomplete),    |
|                            | NADO disposition                              |
| Missed test                | DCO (Doping Control Officer) report digest   |
| Filing amendment           | submitted-time, reason                       |

Three filing failures or missed tests within 12 months
result in an Anti-Doping Rule Violation under the WADA
Code; the protocol's whereabouts events are admissible
evidence in a results-management proceeding.

## Annex H — Inspector replay payload

For an inspection (regulator, IF, or IRB) the protocol
exposes a replay payload covering:

- consent records
- ethics-approval records
- TUE records (sport context)
- session records with telemetry digests
- performance records with rater identity
- AE records with regulator-clock evidence
- audit-chain export

The payload signs with the sponsor's audit-chain JWS
key so the inspector can verify integrity end-to-end.
