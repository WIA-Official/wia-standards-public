# WIA-ozone-layer-protection PHASE 3 — Protocol Specification

**Standard:** WIA-ozone-layer-protection
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2): authentication of
NOU officers, customs officers, GAW station operators,
certified technicians, and Ozone Secretariat staff;
substance-roster snapshotting; phase-out compliance scoring
discipline; atmospheric-observation calibration linkage;
illegal-trade information exchange via iPIC; Article 7
submission workflow; audit-chain construction; time discipline.

References (CITATION-POLICY ALLOW only):
- Vienna Convention + Montreal Protocol — protocol references
- iPIC (informal Prior Informed Consent) Network operating procedures
- BIPM JCGM 100:2008 GUM — measurement uncertainty
- ISO/IEC 17025:2017 — laboratory accreditation
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162 (CT 2.0)
- WIA-pq-crypto PHASE 3 — for ML-KEM/ML-DSA migration

---

## §1 Authentication

NOU officers, customs officers, GAW station operators,
certified technicians, and Ozone Secretariat staff
authenticate using JWS-signed JWTs issued by the deployment's
identity authority. Token claims:

| Claim         | Source                                                 |
|---------------|--------------------------------------------------------|
| `iss`         | identity-provider URL                                  |
| `aud`         | the boundary URL                                       |
| `sub`         | principal URN                                          |
| `iat` / `exp` | per RFC 7519                                           |
| `wia.role`    | `nou-officer`, `customs-officer`, `gaw-station-operator`, `certified-technician`, `secretariat-officer`, `auditor` |
| `wia.partyRef` | for party-bound roles                                  |
| `wia.scope`   | operation-class scopes                                 |
| `wia.certificationRef` | for certified-technician roles                  |

Secretariat tokens require mTLS binding to the UNEP-issued
certificate hierarchy.

## §2 Substance-roster snapshotting

The deployment's substance roster is versioned:

- `rosterId` — URN
- `effectiveFrom` — RFC 3339
- `montrealAmendmentRef` — most recent amendment incorporated
  (Vienna 1985, Montreal 1987, London 1990, Copenhagen 1992,
  Montreal 1997, Beijing 1999, Kigali 2016, future amendments)
- `signature` — JWS by the deployment's roster curator

Each transaction record references the roster ID active at
publication time. Substance reclassification (e.g., a
substance moved between annexes) is rare but tracked in
the roster's chain.

## §3 Phase-out compliance scoring

Compliance scoring runs against the schedule active at the
reporting year:

1. Aggregate per-substance × party × year consumption
2. Look up the schedule for substance × party-status × year
3. Compute headroom or excess
4. Emit per-substance compliance verdict:
   - `compliant` (consumption ≤ target)
   - `over-target-with-exemption` (excess covered by
     authorised exemption)
   - `over-target-without-exemption` (excess not covered;
     non-compliance flag raised)
   - `report-incomplete` (insufficient transaction data)

Compliance scoring is recomputable; corrections to underlying
transactions trigger re-scoring with audit-chain entries
documenting the recomputation.

## §4 Atmospheric-observation calibration linkage

Each observation record's `qualityFlag` reflects calibration
state:

- `nominal` — instrument operating within calibration; data
  emitted in real-time
- `provisional` — recently corrected for known bias; may
  change post final processing
- `processed` — Level-2 product computed
- `validated` — cross-validated against secondary instrument
  or co-located comparison
- `corrected` — re-released after post-publication correction
- `withdrawn` — superseded by a corrected version

For Dobson / Brewer instruments, the calibration linkage
references the World Meteorological Organisation's Calibration
Centre (WCC-Brewer at Toronto, WCC-Dobson at Hohenpeissenberg)
and the most recent intercomparison. For ozonesondes, the
SHADOZ network operating procedure is referenced.

## §5 Illegal-trade information exchange (iPIC)

iPIC consultations follow a documented sequence:

1. Discovering party's NOU posts the case via PHASE 2 §7
2. Boundary forwards the case to the importing/exporting
   party's NOU (per case context) via the iPIC exchange
   channel
3. Receiving NOU acknowledges receipt
4. Receiving NOU posts response (`legal-import`,
   `unauthorised-export`, `under-investigation`,
   `no-record-found`)
5. Both parties update the case state with the response
   and any subsequent enforcement actions
6. UNEP Ozone Secretariat notified for aggregate iPIC
   reporting

Each step emits an audit-chain entry signed by the
responsible NOU.

## §6 Article 7 submission workflow

Annual Article 7 submission follows:

1. Reporting deadline: 30 September of the year following
   the reporting year (per Article 7 paragraph 3)
2. NOU compiles transactions per substance per party per
   year
3. Submission package signed by the NOU and forwarded to
   UNEP via PHASE 2 §9
4. UNEP Ozone Secretariat acknowledges receipt
5. Secretariat aggregation publishes summary tables
6. Implementation Committee reviews any flagged
   non-compliance

Late submissions are accepted but flagged; the
Implementation Committee under the Procedure for
Non-Compliance reviews persistent late filers.

## §7 Audit chain

Every state transition emits an AuditEvent appended to a
Merkle audit log:

`kind` enum:
- `substance-roster-published` / `substance-amended`
- `transaction-published` / `transaction-corrected`
- `consumption-computed` / `consumption-recomputed`
- `exemption-published` / `exemption-state-changed`
- `observation-published` / `observation-validated` /
  `observation-corrected` / `observation-withdrawn`
- `eesc-trajectory-updated`
- `illegal-trade-case-opened` / `case-state-changed` /
  `ipic-consultation-sent` / `ipic-response-received`
- `service-event-published`
- `a7-submission-compiled` / `a7-submission-sent` /
  `a7-acknowledged`
- `compliance-flag-raised` / `compliance-flag-resolved`

Anchored deployments (national NOUs serving as authoritative
reporting boundaries) mirror the audit chain to UNEP Ozone
Secretariat as a witness on the agreed cadence.

## §8 Time discipline

Boundary clock: NTPv4 stratum-2. Atmospheric observations
carry their own time-tagging discipline (GPS-disciplined
for ground stations, satellite ephemeris for orbit-based).
Transaction timestamps reference customs-clearance
timestamps where applicable; the boundary records both
the customs-claimed and the boundary-receipt timestamps
for cross-validation.

## §9 Cryptographic-suite registry

| Concern                  | Default                            | Notes                                |
|--------------------------|------------------------------------|--------------------------------------|
| Token signing            | ES256                              | mTLS-bound for Secretariat           |
| TLS                      | 1.3 (RFC 8446)                     | hybrid groups via WIA-pq-crypto      |
| Audit-chain hash         | SHA-256                            |                                      |
| NOU submission signature | ES256 by NOU                       |                                      |
| Roster signature         | ES384 by roster curator            |                                      |
| Observation file integrity | SHA-256 + JWS                    | for NetCDF / TXT files               |

Post-quantum migration follows WIA-pq-crypto PHASE 3 phase
declarations.

## §10 Failure modes

| Failure                                  | Behaviour                                      |
|------------------------------------------|------------------------------------------------|
| Identity-provider JWKS unreachable        | Cached keys honoured                           |
| Secretariat exchange channel unreachable | Submission queued + retry per backoff          |
| iPIC exchange channel unreachable        | Local case-state advance; cross-party share queued |
| Audit-chain write failure                 | Operation rejected (consistency)               |
| Atmospheric-observation upstream feed lost | Boundary surfaces freshness warning            |
| Roster source unreachable                 | Continue serving last-good roster              |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked authentication sequence (informative)

For NOU officers:

1. Officer authenticates via national-government IdP
   (per regional authentication discipline, e.g., KR
   행정전자서명 + GPKI for Korean officers)
2. IdP issues access token with `wia.role=nou-officer`
   and `wia.partyRef=urn:wia:ozone:party:KOR`
3. Officer calls boundary; boundary validates token + role
4. Operations scoped to officer's party only

For Secretariat staff:

1. Secretariat staff authenticates via UNEP IdP
2. IdP issues mTLS-bound access token with
   `wia.role=secretariat-officer`
3. Operations span all party submissions; per-Annex viewing
   privileges per the Secretariat's internal RBAC

## Annex B — Worked iPIC consultation (informative)

```
1. Korea customs seizes 200 kg of suspected unauthorised
   HCFC-22 at Busan port
2. KR NOU posts illegal-trade case (PHASE 2 §7) with
   substance + quantity + seizure context
3. Boundary forwards to JP NOU (declared origin party)
   via iPIC exchange channel
4. JP NOU acknowledges within 48h
5. JP NOU investigates: confirms exporter is unlicensed
6. JP NOU responds `unauthorised-export`
7. Both parties record the response; KR proceeds with
   destruction; JP initiates investigation against exporter
8. UNEP Ozone Secretariat notified for aggregate iPIC
   tracking
```

The audit chain captures every step.

## Annex C — Compliance scoring worked example (informative)

Korea HCFC reporting 2025 (Article 5 party):

```
Schedule (Annex C-I, Article-5):
  baseline (avg 2009-2010): 100,000 ODP-tonnes (illustrative)
  2013 freeze:               100,000 (100% of baseline)
  2015:                       65,000 (-35%)
  2020:                       32,500 (-67.5%)
  2025:                        2,500 (-97.5%)

Reported 2025 transactions:
  production:            0 (HCFC production phased out in KR earlier)
  imports:           1,800 ODP-tonnes
  exports:               0
  feedstock-use:         0

Consumption = 1,800 ODP-tonnes
2025 target: 2,500 ODP-tonnes
Verdict: compliant
```

## Annex D — Conformance levels

| Level     | Scope                                                                  |
|-----------|------------------------------------------------------------------------|
| Surface   | structural conformance to PHASEs 1–3                                  |
| Verified  | annual third-party audit + UNEP MOP scrutiny                          |
| Anchored  | continuous evidence package + Implementation Committee witness        |

## Annex E — Roster snapshot manifest (informative)

```json
{
  "rosterId": "urn:wia:ozone:roster:kr-nou:r-2026-01",
  "effectiveFrom": "2026-01-01T00:00:00+09:00",
  "montrealAmendmentRef": "Kigali Amendment (2016) entered into force 2019-01-01; locally adopted 2017",
  "substanceCount": 96,
  "scheduleCount": 12,
  "signature": "<jws-detached>"
}
```

## Annex F — Cross-coalition data sharing protocol

For atmospheric observations shared with the World Ozone
and UV Radiation Data Centre (WOUDC) and NDACC:

1. Station operator generates the observation per PHASE 1 §6
2. Boundary forwards to WOUDC submission queue
3. WOUDC validates format; acknowledges or refuses
4. Validated data published to the WOUDC archive
5. Cross-references stored on both sides for replay

Sharing follows the GO FAIR initiative's FAIR principles
(findable, accessible, interoperable, reusable).

## Annex G — Calibration-traceability worked example

A Dobson spectrophotometer at GAW station Boulder:

```json
{
  "stationRef": "urn:wia:ozone:station:gaw:BLD-Dobson",
  "instrumentSerial": "Dobson-83",
  "lastCalibration": "2025-09-15T00:00:00+00:00",
  "calibrationCentre": "WCC-Dobson Hohenpeissenberg",
  "intercomparisonResult": "within ±1.0% of WCC reference (2025 IODC campaign)",
  "nextDueAt": "2027-09-15"
}
```

The boundary marks observations with `qualityFlag=nominal`
while calibration is current; observations after the due
date are flagged for review.

## Annex H — Late-filer escalation

Per Procedure for Non-Compliance:

1. Submission deadline missed
2. Secretariat sends reminder
3. Implementation Committee considers the case
4. Recommendations to MOP per Annex IV of the Procedure

The deployment's audit chain captures the escalation
timeline so non-compliance histories are reconstructable.

## Annex I — Refrigerant-bank tracking interlinkage

Per Article 4B of the Montreal Protocol, parties licence
import / export of controlled substances. Refrigerant-bank
data (PHASE 1 §10) feeds the licence-system check: an
import licence applicant's annual quota is decremented at
each customs-cleared import.

The licence-system integration is per-party; the boundary
emits licence-quota events to the licence system's webhook.
