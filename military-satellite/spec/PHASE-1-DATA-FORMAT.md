# WIA-military-satellite PHASE 1 — Data Format Specification

**Standard:** WIA-military-satellite
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for military
space-segment operations: orbital-asset registry records, payload
tasking orders, downlink-segment state, mission-data products
(ISR, SIGINT, MASINT classification per coalition rules),
TT&C (telemetry, tracking, command) telemetry, anti-jam
posture observations, and the cross-references that bind the
space picture to the broader operational fire-control and
intelligence-fusion chain. The shape is interoperable with
allied space-domain-awareness exchanges and CCSDS-aligned
ground systems so coalition operations do not require parallel
data models.

References (CITATION-POLICY ALLOW only):
- CCSDS 133.0-B-2 — Space Packet Protocol
- CCSDS 232.0-B-3 — TC Space Data Link Protocol
- CCSDS 132.0-B-3 — TM Space Data Link Protocol
- CCSDS 502.0-B-2 — Orbit Data Messages (OPM/OMM/OEM)
- CCSDS 503.0-B-1 — Tracking Data Message (TDM)
- ITU Radio Regulations (RR) — frequency-band coordination
- STANAG 4609 — Motion-Imagery Standards Profile (NATO MISP)
- MISB ST 0102 — Security Metadata Universal Set
- MISB ST 0601 — UAS Datalink Local Set (re-used for satellite ISR metadata)
- WGS-84 — geodetic reference frame
- IETF RFC 7515 (JWS), RFC 8259 (JSON), RFC 9162 (Certificate Transparency 2.0)
- ISO 19115-1 — Geographic information metadata

---

## §1 Scope

This PHASE applies to systems that catalog, task, command,
receive, or report on military space assets: imaging-ISR
satellites (electro-optical, SAR, hyperspectral), SIGINT
collection platforms, MASINT signatures, MILSATCOM relays,
PNT augmentations, and ground-segment elements operating
under coalition release authority. It addresses the *shape*
of operational records; transport protocols are in PHASE 3
and integration with C2 and intelligence-fusion is in PHASE 4.

The standard is classification-aware: every record carries a
CCEB or NATO security-classification marking and a
release-caveat list that downstream consumers honour before
re-disclosure. Records that have not been marked are refused
at boundary intake.

In scope: orbital-asset registry, payload tasking, downlink
state, mission-data product metadata, TT&C telemetry,
anti-jam posture, conjunction-assessment observations. Out of
scope: launch-vehicle integration (governed by national-range
standards), satellite-bus-vendor proprietary fault codes
(carried opaquely as vendor extensions), and weapon
deployment from space (excluded by treaty and out of WIA scope).

## §2 Orbital-asset registry record

Every fielded asset carries a stable registry record:

| Field             | Source / Binding                                       |
|-------------------|--------------------------------------------------------|
| `assetRef`        | URN of form `urn:wia:msat:asset:<authority>:<id>`      |
| `internationalDesignator` | YYYY-NNNAAA per standard space-object designation |
| `noradCatalogNumber` | catalog number where coalition-disclosed             |
| `mission`         | `imagery-eo`, `imagery-sar`, `sigint`, `milsatcom`, `pnt-aug`, `early-warning`, `meteorology`, `multi-mission` |
| `orbitClass`      | `leo`, `meo`, `geo`, `heo`, `molniya`, `tundra`, `polar-leo` |
| `referenceFrame`  | `j2000`, `icrf`, `tod`; positions exchanged in J2000 by default |
| `releaseAuthority`| URN of the controlling release authority               |
| `classificationCeiling` | maximum classification any product from this asset may carry |
| `tleSourcePolicy` | `coalition-shared`, `national-only`, `restricted`     |

Asset registry changes are signed and audit-logged. A
cross-coalition disclosure of a previously national-only asset
requires a release-authority counter-signature in the audit
chain.

## §3 Orbital-state record

Asset state is exchanged using the CCSDS Orbit Data Messages
shape (OPM for instantaneous state, OEM for ephemeris,
OMM for mean-elements):

- `stateId` — URN
- `assetRef` — URN
- `epoch` — RFC 3339 with offset (UTC by default; coalition
  exchanges declare their reference)
- `referenceFrame` — defaults to J2000
- `position` — x, y, z metres
- `velocity` — vx, vy, vz m/s
- `covariance` — 6×6 covariance matrix (position+velocity)
- `propagationModel` — `sgp4`, `sdp4`, `numerical-cowell`, `gtds`
- `dataSource` — `national-tracking`, `coalition-shared`,
  `commercial-augmentation`, `self-reported-gnss`

Self-reported state from the asset's onboard GNSS is signed
by the asset's signing key; coalition-shared external state
is signed by the contributing partner's release authority.
Mixing untrusted sources without provenance is rejected.

## §4 Payload tasking order

Mission planners issue tasking orders to assets:

- `taskOrderId` — URN of form `urn:wia:msat:task:<authority>:<seq>`
- `assetRef` — target asset
- `taskType` — `image`, `intercept`, `relay`, `ranging`, `safe-mode`,
  `delta-v-burn`, `attitude-slew`, `payload-config`, `key-update`
- `windowStart`, `windowEnd` — RFC 3339 with offset
- `priorityClass` — `routine`, `priority`, `flash`, `flash-override`
- `targetGeometry` — for image/intercept: geometric description
  (point, polygon, area-of-interest in WGS-84) or signal-of-interest
  parameters (frequency band, polarization, dwell)
- `expectedProductType` — links to the product registry
- `coalitionReleaseList` — list of partners cleared to receive
  the resulting product
- `commandAuthorityRef` — the issuing authority URN

Tasking orders are signed by the issuing authority. Orders
that exceed the asset's `classificationCeiling` or whose
release list contradicts the asset's release-authority policy
are refused at boundary intake with an audit event.

## §5 TT&C telemetry record

Health-and-status telemetry from the asset is recorded:

- `telemetryId` — URN
- `assetRef` — URN
- `epoch` — RFC 3339 with microsecond precision where available
- `subsystemSnapshots` — per-subsystem state objects:
  - `power` — solar-array current, battery state-of-charge,
    bus voltage, anomaly flags
  - `thermal` — per-zone temperature readings, thermal-margin
  - `attitude` — quaternion, body-rate vector, sun/Earth/star
    sensor health
  - `propulsion` — tank pressure, valve states, residual delta-v
  - `payload` — per-payload state (idle, configuring,
    collecting, dumping, fault)
  - `communications` — uplink lock status, downlink data-rate
    state, anti-jam posture (PHASE 1 §8)
- `anomalyFlags[]` — per-subsystem anomaly bitmask with
  vendor-extension allowances
- `signature` — JWS signature by the asset's signing key

Sample cadence is mission-dependent (typically 1 Hz routine,
10 Hz during burns, 100 Hz during contingency). Boundary
deduplicates on `(assetRef, epoch)`.

## §6 Mission-data product record

Products generated from collection (imagery, SIGINT cuts,
hyperspectral cubes, TDOA tuples) carry a product record:

- `productId` — URN of form `urn:wia:msat:product:<assetRef>:<seq>`
- `assetRef` — collecting asset
- `taskOrderRef` — the originating tasking order
- `productType` — `eo-frame`, `sar-frame`, `hyperspectral-cube`,
  `sigint-cut`, `bistatic-tdoa`, `relay-segment`
- `collectionWindow` — start/end RFC 3339
- `targetGeometryActual` — what the asset actually collected
- `qualityMetrics` — per-product modality (NIIRS-like for EO,
  GRD for SAR, SNR for SIGINT) — vendor-quantified, coalition-
  agreed scale where available
- `securityMetadata` — MISB ST 0102 set carrying classification,
  caveats, originator, declassification date
- `motionImageryMeta` — for MI products, MISB ST 0601 local
  set per STANAG 4609
- `geoMetadata` — ISO 19115-1 metadata for spatial coverage
- `signature` — JWS signature

Products without complete security metadata are refused; the
metadata is part of the canonical record, not an optional
annotation.

## §7 Conjunction-assessment record

For collision-avoidance with operational and debris objects:

- `caId` — URN
- `assetRef` — primary asset
- `secondaryRef` — URN of the secondary object (asset, debris,
  unknown-tracked-object)
- `tca` — time of closest approach RFC 3339
- `missDistance` — metres at TCA
- `pc` — probability of collision (computed per a declared model)
- `screeningPolicy` — declared screening configuration ID
- `recommendedAction` — `monitor`, `plan-burn`, `execute-burn`,
  `de-orbit`, `safe-mode`
- `coordinationRequired` — boolean (true when secondary is
  another nation's asset and coordination is required)

CA records drive PHASE 4 §6 collision-avoidance workflows.

## §8 Anti-jam / anti-spoof posture record

For asset uplink/downlink and PNT-aug payloads:

- `postureId` — URN
- `assetRef` — URN
- `linkScope` — `uplink`, `downlink`, `pnt-broadcast`
- `frequencyBand` — declared band per ITU RR
- `interferenceState` — `nominal`, `degraded`, `denied`,
  `deceptive-suspected`
- `mitigationActive[]` — closed enum: `frequency-hop`,
  `nulling-antenna`, `power-step`, `crypto-rolled`,
  `geo-fence-confirm`, `civil-handoff`
- `evidenceRefs[]` — links to TT&C / ground-station logs
  supporting the posture

A `denied` or `deceptive-suspected` posture triggers PHASE 4
§7 spectrum-coordination and PHASE 4 §8 cross-coalition
notification flows.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                     | Use site                                                 |
|-------------------------------|----------------------------------------------------------|
| WIA-military-communication    | downlink relay over coalition transport                  |
| WIA-missile-defense           | early-warning IR products feed engagement-decision intake|
| WIA-network-security          | TT&C cipher-suite governance                             |
| WIA-pq-crypto                 | PQ migration schedule for command authentication         |

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their
published capability document. A deployment that is `partial`
or `excluded` on §2 (Asset registry), §4 (Tasking order), §5
(TT&C telemetry), or §6 (Product record) is non-conformant
overall. Section §7 (CA) and §8 (Anti-jam posture) are
mandatory for assets in their scope.

## Annex C — Versioning and deprecation

Versioning follows Semantic Versioning 2.0.0. Coalition-shared
records carry the originating deployment's version so cross-
coalition replay across version transitions is well-defined.
Deprecation enters a 12-month sunset window with the migration
notes recorded in the audit chain.

## Annex D — Worked tasking-order example (informative)

An imaging-EO tasking order with two-coalition release:

```json
{
  "taskOrderId": "urn:wia:msat:task:rok-acomd:t-2026-04-27-2210",
  "assetRef": "urn:wia:msat:asset:rok-acomd:eo-1",
  "taskType": "image",
  "windowStart": "2026-04-28T02:14:00+09:00",
  "windowEnd": "2026-04-28T02:18:30+09:00",
  "priorityClass": "priority",
  "targetGeometry": {
    "type": "polygon",
    "coordinates": [[[127.10,37.55],[127.20,37.55],[127.20,37.45],[127.10,37.45],[127.10,37.55]]]
  },
  "expectedProductType": "eo-frame",
  "coalitionReleaseList": ["urn:wia:auth:rok", "urn:wia:auth:us-coalition-cell"],
  "commandAuthorityRef": "urn:wia:auth:rok-acomd-tasking-cell"
}
```

The boundary chains the resulting product record to this
order so traceability from operational requirement through
collection to disseminated product is preserved.

## Annex E — Vendor extensions

Asset bus and payload vendors may extend telemetry and
product records with `x-vendor-*` fields. Extensions MUST NOT
contradict the canonical fields and MUST NOT be required for
core conformance. A coalition partner unable to interpret a
vendor extension still satisfies conformance by ignoring it.

## Annex F — Time discipline cross-reference

Time fields use the discipline of PHASE 3 §6 (TAI/UTC, leap-
second handling, GPS-week rollover safe encodings). Records
that fail clock-discipline check are tagged `provisional`
until backfill from a trusted time source.

## Annex G — Conformance level

Implementations declare conformance level (Surface / Verified
/ Anchored) per the deployment policy. Anchored requires
continuous TT&C-signed evidence packages plus quarterly
coalition-disclosure replay.
