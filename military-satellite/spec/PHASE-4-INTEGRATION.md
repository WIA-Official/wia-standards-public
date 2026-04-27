# WIA-military-satellite PHASE 4 — Integration Specification

**Standard:** WIA-military-satellite
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a military-satellite ground segment
integrates the data, APIs, and protocols from PHASEs 1–3
with the broader operational picture: orbital-asset fleet
management, ground-network operations, intelligence-fusion
hand-off, missile-defence early-warning hand-off, anomaly
response, conjunction-avoidance, spectrum coordination, and
coalition exchange. It is non-prescriptive about specific
vendors; it specifies the integration *contracts* a deployment
must satisfy.

References (CITATION-POLICY ALLOW only):
- CCSDS 502.0-B-2 / 503.0-B-1 — orbit and tracking data exchange
- CCSDS 132.0/232.0 — TM/TC over the link
- ITU Radio Regulations — spectrum-coordination
- WIA-missile-defense (PHASE 1–4) — early-warning intake
- WIA-military-communication (PHASE 1–4) — coalition transport
- WIA-network-security (PHASE 1–4) — TT&C cipher governance
- WIA-pq-crypto (PHASE 1–4) — PQ migration cadence

---

## §1 Orbital-asset fleet registry

The deployment maintains a registry of every fielded asset
(PHASE 1 §2). Mandatory integration contracts:

- Registry mutations are signed by the command authority
- Registry mirror to coalition partners runs on a declared
  cadence (typically daily) with a cryptographic snapshot
- Asset retirement (de-orbit, end-of-life) closes outstanding
  tasking and emits a final `asset-retired` audit-chain entry

A deployment whose registry is not signed-mirrored does not
satisfy `Verified` conformance.

## §2 Ground-network operations

Ground-segment integration covers TT&C station selection,
contact scheduling, and uplink/downlink coordination:

- Ground-station registry — URN per station with location,
  bands, antenna characteristics
- Pass-prediction service — per asset, computes pass windows
  from current ephemeris (PHASE 1 §3); inputs come from the
  asset itself plus coalition-shared tracking
- Contact-schedule contract — at most one primary station per
  pass per asset; coalition partner stations receive secondary
  role per a deployment-declared protocol
- Anomaly contact — on `anomalyFlags` set, the schedule is
  rebuilt to prioritise the affected asset within a
  deployment-declared latency budget

Ground-network operations expose the schedule via the
boundary's capability document so coalition partners can plan
tasking against expected uplink windows.

## §3 Intelligence-fusion hand-off

Mission-data products (PHASE 1 §6) feed the intelligence-
fusion pipeline:

- EO and SAR frames feed image-intelligence analysis
- SIGINT cuts feed signals analysis
- Hyperspectral cubes feed measurement-and-signature analysis
- Bistatic TDOA tuples feed geolocation analysis

Integration contracts:

- Every product carries MISB ST 0102 security metadata
  (PHASE 1 §6); fusion systems honour caveats before
  re-disclosure
- Fusion-derived products that re-use space inputs cite the
  originating productId in their derived-product manifest;
  this is required for downstream provenance
- Coalition-only fusion outputs that rely on a national-only
  source require a release-board decision on the source
  before publication

A fusion product that omits source citation is an integration
violation; the boundary refuses re-publication until the
citation is added.

## §4 Missile-defence early-warning hand-off

Early-warning IR products feed WIA-missile-defense PHASE 1 §3
detection records:

- Boost-phase IR detections are flagged with `priority=flash`
  and routed to the missile-defense boundary within a
  deployment-declared latency budget (typically a few seconds)
- The same productId is referenced in the missile-defense
  detection record so cross-domain replay reconstructs the
  full picture
- Throughout an active engagement, the deployment elevates
  the affected asset's pass priority to `flash-override`

A space-segment that cannot meet the latency budget is
declared as a non-warning provider for the duration of the
condition, and missile-defense routes around it.

## §5 Anomaly response

Per Annex C of PHASE 3, asset anomalies escalate to operations:

- Anomaly intake is via the boundary's anomaly endpoint
- Severity classification follows the asset's mission rules:
  - `S1` — mission-impacting, immediate response
  - `S2` — degraded but operable, planned response
  - `S3` — informational, scheduled response
- Each anomaly opens an audit-chain anomaly thread; closure
  requires a root-cause note signed by the operations lead

For coalition-shared assets, S1 anomalies are mirrored to
the partner's anomaly board; the partner may elect to send
a liaison observer to the originator's response cell.

## §6 Conjunction-avoidance workflow

CA records (PHASE 1 §7) drive collision-avoidance:

1. CA record arrives (own screening or partner-shared)
2. Pc is compared against the deployment's `pcThreshold`
3. If exceeded:
   a. Operations cell evaluates burn options
   b. A `delta-v-burn` tasking order is generated and
      submitted (PHASE 2 §1)
   c. The maneuver is coordinated with the secondary's
      operator if disclosure is permitted
4. Post-maneuver, a follow-up CA record is published to
   close the thread
5. The audit chain records every step including the burn
   parameters

Non-maneuvered passes through the threshold (e.g., maneuver
infeasible due to propellant) are explicitly declared and
flagged for command-authority review.

## §7 Spectrum coordination

Anti-jam posture (PHASE 1 §8) feeds spectrum coordination:

- A posture transition to `denied` or `deceptive-suspected`
  triggers a notification to the deployment's spectrum-
  coordination cell within a deployment-declared latency
  budget (typically < 30 s)
- The cell evaluates ITU RR-aligned mitigations: frequency
  shift, civil-handoff, geographic routing
- For coalition-shared bands, the partner's spectrum cell is
  notified per a bilateral protocol; ITU RR coordination
  procedures govern resolution if the interference source
  is a registered civil emitter

A deployment without a spectrum-coordination integration is
non-conformant for assets in `pnt-aug` or `milsatcom` mission
classes.

## §8 Coalition exchange

Coalition exchange combines several integration contracts:

- Capability document mirroring — partners hold each other's
  capability documents and verify on every exchange
- Audit-chain witnessing — selected entry classes are
  mirrored to a partner audit witness (PHASE 3 §4)
- Release-board decisions — coalition-released products
  require partner counter-signature (PHASE 2 §7)
- Joint exercises — periodic replay of historical tasking
  and product flows to verify continuing compatibility

A deployment may participate in multiple coalitions
simultaneously; the release lists carry the per-coalition
authority URNs and the boundary enforces the union of all
active release rules.

## §9 Lifecycle integration

Asset lifecycle events integrate with several systems:

- Launch — the asset enters the registry only after
  separation telemetry and orbit-confirmation
- Commissioning — payload activation transitions the asset
  from `commissioning` to `operational` only after a
  documented commissioning-test pass
- Anomaly hold — major anomalies place the asset in
  `safe-mode-hold` and refuse non-safe-mode tasking
- End-of-life — de-orbit or graveyard maneuver closes the
  registry record with the final `asset-retired` audit entry
- Re-tasking transitions — major mission-redirection (e.g.,
  re-purposing a multi-mission asset) re-validates the
  asset's release-authority policy

## §10 Conformance evidence package

Anchored conformance requires a continuous evidence package:

- Signed capability document, refreshed per the deployment-
  declared cadence
- Signed audit-chain snapshot, replicated to coalition
  witness, with witness-acknowledgement
- TT&C-signed telemetry exemplar batches showing live key
  rotation and authentication
- Coalition-release decision archive
- Anomaly-thread closure archive with root-cause notes

The evidence package is published to the deployment's
public attestation surface (or partner-mirrored attestation
if classified). Anchored auditors review the package on the
declared cadence.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain integration matrix

| Cross domain                  | Hand-off                                         |
|-------------------------------|--------------------------------------------------|
| WIA-missile-defense           | early-warning IR products → engagement decisions |
| WIA-military-communication    | downlink relay over coalition transport          |
| WIA-network-security          | TT&C cipher floors and revocation surfaces       |
| WIA-pq-crypto                 | PQ migration cadence per asset class             |
| WIA-nbc-defense               | civil-emergency notification overlap (ground-coverage) |

## Annex B — Coalition integration tiers

| Tier         | Scope                                                |
|--------------|------------------------------------------------------|
| Bilateral    | one-to-one exchange with a single coalition partner  |
| Multilateral | exchange with a registered coalition cell            |
| Open-coalition | exchange with any partner whose authority is in the  |
|              | deployment's registered release-authority allowlist  |

## Annex C — Test-range integration

Test-range integration (e.g., on-orbit demonstration of a
new payload mode) is governed by a separate test-plan
contract. The boundary admits test-range tasking only for
assets and during windows declared in the test plan, and
post-test products are tagged `test-only` until released.

## Annex D — Operator console contract

Operations-cell consoles integrate with the boundary via:

- Live-telemetry view fed by the SSE stream of PHASE 2 §3
- Tasking-order builder that issues against PHASE 2 §1
- Anomaly-thread board fed by the anomaly endpoint
- Conjunction-assessment timeline fed by PHASE 2 §5
- Coalition-release queue for outgoing release requests
  (PHASE 2 §7) and counter-signature inflows

Console actions that mutate boundary state always carry the
operator's authority URN; consoles are themselves
authenticated as `operator-console` role tokens (PHASE 3 §1).

## Annex E — Public attestation surface

Anchored conformance publishes an attestation surface
containing the deployment's:

- Capability document (signed)
- Audit-chain snapshot Merkle roots over a rolling window
- Conformance-evidence package (signed)
- Cipher-suite floor and PQ migration phase
- Coalition-release-board roster and decision archive

For deployments where full publication is not permitted by
classification, the attestation surface is partner-mirrored
to the trusted coalition witness with the same shape; an
out-of-band anchor receipt is then published to a
public-classified summary.

## Annex F — Civil-emergency overlap

Where space assets contribute to civil-emergency response
(e.g., disaster imaging, emergency communications relay) the
deployment maintains a civil-tasking lane with reduced
classification ceiling and broader release-list authority.
Civil tasks pass through the same PHASE 2 §1 endpoint with
a declared `civilOverride: true` flag and the civil
authority's URN as the `commandAuthorityRef`.

## Annex G — Conformance level summary

| Level     | Scope                                                |
|-----------|------------------------------------------------------|
| Surface   | structural conformance to PHASEs 1–3                 |
| Verified  | annual third-party audit of integration contracts    |
| Anchored  | continuous evidence package per Annex E plus quarterly coalition-disclosure replay |
