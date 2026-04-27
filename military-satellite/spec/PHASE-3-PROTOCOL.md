# WIA-military-satellite PHASE 3 — Protocol Specification

**Standard:** WIA-military-satellite
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2) and to the on-orbit
command/telemetry binary plane: authentication of assets and
ground systems, the CCSDS uplink/downlink encoding,
cryptographic command authentication, audit-chain
construction, time discipline (GPS-week, leap-second, TAI-UTC),
release-authority handshake, and post-quantum migration.

References (CITATION-POLICY ALLOW only):
- CCSDS 133.0-B-2 — Space Packet Protocol
- CCSDS 232.0-B-3 — TC Space Data Link Protocol
- CCSDS 132.0-B-3 — TM Space Data Link Protocol
- CCSDS 355.0-B-2 — Space Data Link Security Protocol
- CCSDS 357.0-B-2 — Authenticated CRC for Telecommand
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK),
  RFC 9162 (Certificate Transparency 2.0)
- IERS Bulletin C — leap-second authority
- WIA-pq-crypto PHASE 3 — for ML-KEM/ML-DSA migration profiles

---

## §1 Authentication

Tasking authorities, ground stations, processing elements,
and coalition-release boards authenticate to the boundary
using JWS-signed JWTs issued by the deployment's command
authority. Token claims include:

- `iss` — issuing authority URN
- `sub` — operator/system URN
- `aud` — boundary URN
- `iat`, `exp` — RFC 3339 with offset
- `wia.role` — one of `command-authority`, `ground-station`,
  `processing-element`, `release-board`, `coalition-witness`
- `wia.scope[]` — operation-class scopes, e.g.,
  `task:image`, `task:burn`, `posture:write`

Tokens are short-lived (typically 5 minutes) and refreshed
via the deployment's credentials service. Long-lived
credentials are forbidden for command-issuing roles.

## §2 Cryptographic command authentication

On-orbit command frames are authenticated using CCSDS
Authenticated CRC for Telecommand (CCSDS 357.0) or, where
the asset bus supports it, the full Space Data Link Security
Protocol (CCSDS 355.0). The deployment policy declares which
profile applies to which asset class.

Command authentication keys:

- `keyRef` — URN
- `algorithm` — declared per-asset (typically AES-256-CMAC for
  CCSDS 357.0, AES-256-GCM for CCSDS 355.0 link security)
- `rolloverPolicy` — calendar-based with crypto-period not
  exceeding the asset's design crypto-period
- `escrow` — `none` for tactical operations; release-board-
  witnessed escrow for coalition-shared assets

Key updates are themselves uplinked under the prior key plus
a counter-signature from the release board for assets that
require it. A failed command authentication is logged at the
asset and reported in the next telemetry contact; the asset
does not execute the unauthenticated command under any
circumstance.

## §3 CCSDS uplink/downlink encoding

The boundary translates between PHASE-2 JSON tasking orders
and the on-orbit binary plane:

- Uplink commands use CCSDS Telecommand frames with the
  Space Packet Protocol payload structure
- Downlink telemetry uses CCSDS Telemetry frames with VC IDs
  partitioned per subsystem (TT&C, payload, file-management,
  store-and-forward)
- File-management transfers (e.g., product delivery to ground)
  use CCSDS File Delivery Protocol where supported, with
  segmentation appropriate to the link's bit-error-rate
  profile

The mapping table from PHASE-2 task types to CCSDS application
process IDs (APIDs) is published in the deployment's
capability document. APIDs are stable across the asset's
mission lifetime; introducing a new APID requires a new
capability document version.

## §4 Audit chain

Every boundary state transition is appended to a Merkle audit
log:

- `entryId` — URN
- `parent` — prior `entryId` SHA-256
- `at` — RFC 3339 with offset
- `actor` — authority URN of the entity making the transition
- `kind` — closed enum: `task-received`, `task-validated`,
  `task-uplinked`, `product-published`, `product-released`,
  `posture-changed`, `key-rolled`, `release-decision`,
  `coalition-mirror-confirm`
- `payloadHash` — SHA-256 of the canonical JSON payload
- `signature` — JWS by the actor

Coalition-witness mirroring publishes selected entry classes
(`product-released`, `release-decision`, `key-rolled` for
coalition-shared assets) to the partner's audit witness with
a counter-signature. A missing counter-signature past the
deployment's witness-latency budget escalates to the
release-board's monitoring console.

## §5 Release-authority handshake

Coalition release of a product follows a three-message
exchange:

1. Originating deployment posts a release request (PHASE 2 §7)
2. Release board reviews and decides; decision is signed
3. Boundary updates product record's release list and emits
   `release-decision` audit-chain entry

The decision message carries:

- `decision` — `release`, `reject`, `defer`
- `decisionAuthorityRef` — URN of the deciding board
- `decidedAt` — RFC 3339 with offset
- `validUntil` — RFC 3339 (for `release` decisions; bounded
  by the asset's `classificationCeiling` review cadence)
- `coalitionWitnessAck` — confirmation URN from the partner

A decision without a coalition-witness acknowledgement is
held in pending state and not applied to the product record.

## §6 Time discipline

All record timestamps use RFC 3339 with explicit offset.
On-orbit telemetry is time-stamped by the asset's onboard
clock disciplined to GPS time (or coalition equivalent
PNT augmentation); downlink processing converts to UTC at
boundary. Records that fail clock discipline (drift outside
declared bound, lost lock to PNT source) are tagged
`provisional` until a recovery time-tag from a trusted source
arrives. Leap-second handling follows IERS Bulletin C;
GPS-week rollover is handled by always carrying full epoch
year-month-day to avoid ambiguity.

## §7 Transport security

Ground-segment HTTP(S) endpoints require TLS 1.3 (RFC 8446)
with a deployment-declared cipher-suite list. Mutual TLS is
required for command-authority and processing-element
endpoints. Coalition exchanges use a coalition-issued
certificate hierarchy with revocation surfaces published
through the WIA-network-security PHASE 3 governance.

## §8 Post-quantum migration

The standard supports a phased migration to PQC:

- Phase A — classical-only (current default)
- Phase B — hybrid (classical KEM + ML-KEM, classical signature
  + ML-DSA)
- Phase C — PQ-only

Asset-class migration timelines are declared in the
deployment policy and aligned with WIA-pq-crypto PHASE 3
profiles. On-orbit assets unable to receive PQ key uploads
remain on Phase A under a documented exception until end of
mission.

## §9 Replay protection

Telecommand frames carry monotonic counters; the asset
rejects frames with counters older than the high-water mark
plus a small backwards window for retransmission. Uplink-
window contention (multiple ground stations attempting to
reach the same asset) is resolved by the deployment's
ground-network coordinator; only one station holds the
"primary" role per pass.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cipher-suite floors

Boundary endpoints accept only TLS 1.3 cipher suites with
forward secrecy. The minimum cipher floor and PQ-hybrid
profile is published in the deployment's capability
document; partners verify compatibility before exchange
begins.

## Annex B — Audit-chain replication topology

Coalition deployments mirror selected audit-chain entries
to a partner-witness. The replication topology is
deployment-policy and may be one of:

- bilateral (one-to-one)
- star (originator to coalition cell)
- chain (originator → coalition cell → onward partners)

Each topology has documented latency and witnessing
characteristics; the topology in use is declared in the
capability document.

## Annex C — On-orbit anomaly handling

Asset-detected anomalies (out-of-tolerance subsystem state,
authentication failure, payload fault) are reported in the
next telemetry contact with elevated priority. The boundary
escalates anomaly events to the operations cell via PHASE 4
§5 anomaly workflow within a deployment-declared latency
budget. Anomaly state remains until cleared by an operator
acknowledgement that is itself signed and audit-logged.

## Annex D — Range-safety carve-out

For assets with on-orbit propulsion, range-safety
considerations during early-orbit operations may temporarily
restrict tasking to safe-mode commands only. The
deployment's command authority signals this restriction by
emitting a `range-safety-active` capability flag; the
boundary refuses non-safe-mode tasking until the flag clears.

## Annex E — Worked command authentication

A worked example for an `image` tasking-order uplink under
CCSDS 357.0 Authenticated CRC:

1. Boundary receives the validated tasking order (PHASE 2 §1)
2. Boundary projects the order into one or more Telecommand
   frames per the deployment's mapping table
3. Each frame's command-authentication tag is computed
   over the frame body using the asset's current command-
   authentication key
4. Frames are submitted to the ground-station scheduler with
   the asset's pass window
5. Asset receives, verifies the tag, and either executes
   (positive verification) or discards (negative)
6. Verification result is reported in the next telemetry
   contact and bound back to the tasking order via the
   `taskAck` URN

A failed verification is itself audit-chained on the boundary
side as `kind=task-rejected-by-asset` with the asset's
verification failure code.

## Annex F — Coalition-witness mirroring detail

Audit-chain entries selected for coalition mirroring are
batched and pushed to the partner's witness with declared
batch latency. The mirroring envelope:

```json
{
  "batchId": "urn:wia:msat:audit-batch:rok-acomd:b-2026-04-27-001",
  "originator": "urn:wia:auth:rok-acomd",
  "witness": "urn:wia:auth:us-coalition-cell-witness",
  "entries": [/* signed entries */],
  "merkleRoot": "<sha-256>",
  "originatorSignature": "<jws-detached>"
}
```

Witness acknowledgement is a counter-signed receipt published
to both the originator and witness audit chains. Missing
acknowledgement past the latency budget escalates per the
PHASE 4 §8 coalition-exchange contract.

## Annex G — PNT broadcast integrity

For PNT-augmentation assets, broadcast signals carry a
signal-authentication-message (SAM) appended to the navigation
data. The SAM is itself signed by the constellation's signing
key (rolled per the same crypto-period rules as TT&C keys).
Receivers verify the SAM before applying the navigation data;
unverified data is reported in the receiver's posture record
(PHASE 1 §8) for downstream spectrum-coordination evaluation.

## Annex H — Algorithm registry

The deployment maintains an algorithm registry naming the
specific cipher and signature algorithms in use per asset
and per role (TT&C cipher, command-auth MAC, link-security
KEM, audit-chain signature). The registry is published in
the capability document and tracked across PQ migration
phases for partner verification.

## Annex I — Boundary-clock health

The boundary itself maintains a clock-health record published
in the capability document:

- `clockSourceRef` — primary time source (typically GPS or a
  national-laboratory time reference)
- `secondaryClockSourceRef` — fallback time source
- `lastSyncAt` — most recent successful sync to primary
- `currentDriftEstimateNs` — estimated drift in nanoseconds
- `leapSecondsApplied` — count of leap seconds applied since
  the deployment's epoch (per IERS Bulletin C)

Coalition partners verify the boundary clock health before
accepting time-sensitive products; a boundary whose drift
exceeds its declared bound is treated as a degraded
exchange peer until recovered.

## Annex J — Negative-test vectors for protocol layer

| Stimulus                                              | Expected outcome                              |
|-------------------------------------------------------|-----------------------------------------------|
| TT&C frame with bad authentication tag                | asset discards; logs in next telemetry        |
| Command-key rollover before partner counter-signature | refused at boundary; held in pending state    |
| Audit-chain entry with broken parent hash             | rejected at append; boundary alerts           |
| Coalition-witness batch missing acknowledgement       | escalation per Annex F latency budget         |
| Time-tag drift outside declared bound                 | record marked `provisional` until backfilled  |
