# WIA-SPACE-014 — Phase 4: Integration Specification

**Standard**: WIA-SPACE-014 (Space Tourism)
**Phase**: 4 of 4 — Integration with Existing Systems
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 4 specifies how WIA-SPACE-014 interoperates with three classes of
existing system:

1. **Civil aviation regulators** (national CAAs, ICAO).
2. **Space operations stack** (vehicle telemetry buses, range safety,
   mission control).
3. **Other WIA family standards** (WIA-OMNI-API for credentials,
   WIA-AIR-SHIELD for transport hardening, WIA-INTENT for booking,
   WIA-ACCESSIBILITY for passenger accommodations).

The aim is to let an operator publish one WIA-SPACE-014 record set and
have it answered by every external system with no per-system retyping.

---

## 2. Civil Aviation Regulator Bridges

### 2.1 Regulator Inbound

A regulator subscribes to a per-operator feed:

```
GET /wst/regulator/feed
Accept: text/event-stream
Authorization: WIA-Sig <regulator key>
```

The feed delivers state-machine transitions (Phase 3 §4), incident-grade
safety events, and licence-affecting changes. The regulator's storage
becomes the operator's shadow index for audit purposes.

### 2.2 Outbound Licence Check

Before transitioning a mission to FROZEN, the operator MUST verify the
regulator's licence freshness:

```
GET https://regulator.example/wia-space-tourism/licence/{operator_id}
```

Returns `200` with the licence record or `403` if suspended. Operators
MUST cache the result no longer than the regulator's `Cache-Control` says.

### 2.3 ICAO Mapping

Operators that fall under ICAO oversight MUST publish a per-flight ICAO
flight plan record using the ICAO Doc 4444 form, derived deterministically
from the WIA mission profile. The mapping is documented in
`spec/profiles/icao.md`.

---

## 3. Space Operations Stack

### 3.1 Telemetry Bus

The on-board telemetry bus (CCSDS Space Packet Protocol) emits frames at
1–10 Hz. The operator's WIA-SPACE-014 telemetry endpoint translates each
frame into the SSE format defined in Phase 2 §9. The translation MUST
preserve frame ordering; out-of-order frames are dropped rather than
re-sorted, since downstream tools rely on monotonic cursors.

### 3.2 Range Safety

Range-safety officers receive a real-time stream of mission state, vehicle
position, and abort signals. The operator's range-safety bridge maps
WIA mission state to the range's expected protocol (typically the
manufacturer-specific range network). Bridges MUST emit a heartbeat at
1 Hz; a missed heartbeat for > 5 s triggers a mission abort hold.

### 3.3 Mission Control Console

A mission control console receives WIA telemetry, decoded crew biometrics
(per WIA-ACCESSIBILITY consent), ECLSS gauges, and event alerts. The
console SHOULD render alerts in colour-blind-safe palettes and provide
caption transcripts for voice loops, per WIA-ACCESSIBILITY §5.

---

## 4. WIA Family Integration

### 4.1 WIA-OMNI-API

Passenger and crew credentials (medical certificates, training records,
identity proofs) are stored in WIA-OMNI-API. Operators fetch them by DID
rather than holding raw documents.

### 4.2 WIA-AIR-SHIELD

Transport hardening (TLS configuration, peer reputation) is delegated to
WIA-AIR-SHIELD. Operators MAY refuse handshakes from peers whose AIR-SHIELD
score is below an operator-set threshold.

### 4.3 WIA-INTENT

Passenger-facing booking flows compose WIA-INTENT verbs. A passenger's
"book a sub-orbital flight in spring" intent is lowered to:

1. Operator search call (`GET /wst/mission?class=Sub-orbital&window=…`).
2. Manifest reservation (`POST /wst/manifest/reserve`).
3. Insurance quote (`GET /wst/insurance/quote?…`).
4. Consent and waiver capture (`PUT /wst/passenger/{id}/consent`).
5. Final manifest confirmation (`PATCH /wst/manifest/{id}`).

### 4.4 WIA-ACCESSIBILITY

Passenger accessibility profiles drive seat selection, suit interface
configuration, on-board communication channel, and post-flight
debriefing format. Operators MUST refuse to manifest a passenger whose
accessibility requirements cannot be met by the assigned vehicle.

### 4.5 WIA-SOCIAL

Mission summaries that the operator publishes for marketing flow through
WIA-SOCIAL bridges so a single operator post fans out to every supported
network with appropriate `audience` and `alt_text`.

---

## 5. Insurance Underwriter Adapters

Underwriters may run independently of operators. The adapter:

* Subscribes to the operator's insurance feed.
* Fetches passenger medical attestations via WIA-OMNI-API on policy issue.
* Receives claim packets (passenger or operator submitted).
* Issues `claim_status` envelopes that flow back to the operator.

Adapters MUST emit a delivery receipt for every claim packet received,
matching the at-least-once contract used by WIA-SOCIAL bridges.

---

## 6. Search and Rescue Adapters

SAR authorities receive:

* Pre-launch flight plan with predicted landing footprint.
* Real-time telemetry stream (filtered to position and vehicle state).
* Contingency notices on abort, off-nominal trajectory, or comm loss.
* Recovery confirmation envelope at mission end.

Adapters MUST be tested through tabletop exercises at least quarterly;
test envelopes MUST carry `test=true` and MUST NOT enter the production
SAR queue.

---

## 7. Migration Paths

### 7.1 From a Legacy Operator Stack

An operator currently using bespoke spreadsheets and email migrates by:

1. Deploying a WIA-SPACE-014 service (reference container available on
   the WIA registry).
2. Importing historical missions as `archived` profiles.
3. Importing live passenger records under a frozen v0 namespace.
4. Cutting over the booking front-end to call the WIA API.

### 7.2 Between Operators

A passenger transferring booking from operator A to operator B uses the
delegation envelope (Phase 3 §6) plus a `move` envelope:

```json
{ "type":"move", "passenger_id":"…", "from_operator":"…", "to_operator":"…", "signature":"…" }
```

Operator A returns `301 Moved Permanently` for the passenger's record for
at least 12 months after the move.

---

## 8. Observability

Adapters and bridges SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_space_tourism_missions_total{state}` | counter | Mission state transitions |
| `wia_space_tourism_safety_events_total{severity}` | counter | Per-severity event counts |
| `wia_space_tourism_telemetry_frames_total{mission_id}` | counter | Frames emitted |
| `wia_space_tourism_handshakes_total{role,outcome}` | counter | Federation handshakes |

Labels MUST NOT include passenger identifiers.

---

## 9. Conformance Profiles

| Level | Required integrations |
|-------|-----------------------|
| **Minimal** | Regulator inbound feed, WIA-OMNI-API |
| **Core**    | Plus WIA-AIR-SHIELD, telemetry bus translation |
| **Full**    | Plus WIA-INTENT booking, WIA-ACCESSIBILITY enforcement, WIA-SOCIAL outbound |

Operators publish their level in the discovery document under
`integration_profile`.

---

## 10. Worked Example — End-to-End Flight

```
Operator: did:wia:operator:lev
Passenger: did:wia:passenger:01HXY…
Mission:  msn_2026-04-27-LEV03-01
```

1. Passenger books via WIA-INTENT lowering to `POST /wst/manifest/reserve`.
2. Operator fetches medical attestation via WIA-OMNI-API.
3. Operator issues insurance quote, passenger accepts.
4. Manifest transitions DRAFT → REVIEWED → FROZEN.
5. SAR adapter subscribes to telemetry stream.
6. Mission LAUNCHED — telemetry SSE flows.
7. Operator emits `safety_event` (anomaly) at T+33.
8. Mission RECOVERED, safety event published to public after 30 days.
9. Passenger files claim via delegation; underwriter settles.
10. Mission ARCHIVED at retention horizon.

---

## 11. Security Considerations

* Bridges hold range-safety material that, if compromised, could disable
  abort capability. Operators MUST run range-safety bridges in HSM-backed
  environments and MUST rotate credentials at every mission boundary.
* Telemetry streams may carry crew biometrics; bridges MUST honour the
  passenger's WIA-ACCESSIBILITY consent regarding what is published.
* SAR adapters carry life-safety information; operators MUST treat SAR
  endpoints as the highest priority during mission timing windows and
  MUST page on-call when an SAR adapter heartbeat fails.

---

## 12. References

* ICAO Doc 4444 — Air Traffic Management
* ICAO Annex 13 — Aircraft accident reporting
* CCSDS 133.0-B-2 — Space Packet Protocol
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* WIA-OMNI-API standard
* WIA-AIR-SHIELD standard
* WIA-INTENT standard
* WIA-ACCESSIBILITY standard
* WIA-SOCIAL standard

---

## Appendix A — Bridge Configuration File

A reference range-safety bridge configuration uses TOML:

```toml
[bridge]
network = "range-safety"
profile = "Full"
discovery_url = "https://range.example/.well-known/wia-space-tourism"

[credentials]
provider = "wia-omni-api"
omni_endpoint = "https://omni.example"
hsm_slot = 3

[delivery]
heartbeat_hz = 1
abort_hold_after_missed_heartbeats = 5

[telemetry]
prometheus_port = 9091
```

Operators MAY embed vendor-specific sections; conformant bridges MUST
ignore unknown sections rather than refuse to start.

## Appendix B — Worked Migration: Legacy Spreadsheet Operator

A small operator currently running passenger records in spreadsheets and
manifests in PDFs migrates as follows:

1. Deploy a WIA-SPACE-014 reference container.
2. Run `wst-import --csv passengers.csv --map spreadsheet-v0.yaml`. The
   importer assigns each row a fresh DID and records the source CSV row
   id in a `provenance` field for audit.
3. Mark every imported passenger record as `medical.legacy=true`. Future
   medical exams overwrite this flag once they pass through a WIA medical
   examiner.
4. Re-issue all OAuth tokens used by the spreadsheet's email integrations
   through WIA-OMNI-API. Old tokens are revoked at cutover.
5. Repoint the booking front-end to the WIA API base URL.
6. Run a 7-day shadow period during which both stacks receive writes and
   the operator compares outputs daily. After zero discrepancies for
   three consecutive days, retire the legacy stack.

## Appendix C — Worked Cross-Operator Move

Bob holds a Class-3 Orbital booking on operator A but moves it to operator
B after a vehicle reassignment.

1. Bob (or his agent under delegation) signs a `move` envelope citing the
   prior manifest's `vmfst_id`.
2. Operator A returns `301 Moved Permanently` for the prior manifest URL
   for at least 12 months.
3. Operator B issues a fresh manifest entry with `previous_signature`
   chained to A's prior entry.
4. Operator B sends handshakes to the regulator, SAR, and underwriter
   that previously served Bob's booking, presenting the chain.
5. Each peer rebinds its local edges to the new manifest id.
6. The underwriter's policy continues without re-issue; only the
   `mission_id` reference inside the policy is appended.

## Appendix D — Adapter State Machine: Range Safety

```
  IDLE ─── operator publishes mission FROZEN ─►  ARMED
  ARMED ── 1 Hz heartbeat OK ────────────────►  LIVE
  LIVE  ── missed heartbeat ≥5 s ────────────►  HOLD
  HOLD  ── operator clears, heartbeat returns ►  LIVE
  HOLD  ── timeout 60 s ─────────────────────►  ABORT
  LIVE  ── mission RECOVERED ─────────────────►  IDLE
```

The `ABORT` transition issues a signed `range_abort` envelope that the
vehicle's flight-termination system MUST honour. Operators MUST log every
state change of the bridge in append-only storage; auditors and
post-flight reviewers depend on the trace for incident reconstruction.

## Appendix E — Conformance Test Coverage by Profile

| Capability | Minimal | Core | Full |
|------------|---------|------|------|
| Regulator inbound feed | ✓ | ✓ | ✓ |
| Licence freshness check | ✓ | ✓ | ✓ |
| WIA-OMNI-API credential fetch | ✓ | ✓ | ✓ |
| Telemetry bus translation | — | ✓ | ✓ |
| Range-safety bridge | — | ✓ | ✓ |
| WIA-INTENT booking lowering | — | — | ✓ |
| WIA-ACCESSIBILITY enforcement | — | — | ✓ |
| WIA-SOCIAL outbound | — | — | ✓ |
| ICAO Doc 4444 mapping | optional | optional | ✓ |
| SAR adapter quarterly drill | optional | ✓ | ✓ |

A conformant operator publishing `integration_profile=Full` MUST pass
every line in the Full column.

弘益人間 — Benefit All Humanity.
