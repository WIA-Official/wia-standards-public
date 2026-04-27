# WIA-missile-defense PHASE 4 — Integration Specification

**Standard:** WIA-missile-defense
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a deployment integrates the data, APIs, and
protocols from PHASEs 1–3 with the operational picture: sensor-fleet
management, fusion-engine deployment, command-and-control consoles,
weapon systems, civil-airspace overlap, casualty-coordination, and
coalition exchange. It is non-prescriptive about specific vendor
products; it specifies the integration *contracts* a deployment
must satisfy.

References (CITATION-POLICY ALLOW only):
- STANAG 5516 — Tactical Data Link 16
- STANAG 5522 — Link 22
- ICAO Doc 4444 — air-traffic management procedures
- WIA-military-communication — for cross-domain transport
- WIA-medical-data-privacy — for casualty integration
- WIA-medical-imaging — for imaging-driven medical-evacuation coordination

---

## §1 Operational SLAs

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Sensor observation ingest p95 added latency      | ≤ 50 ms                  |
| Track update propagation to fusion               | ≤ 100 ms                 |
| Threat assessment to engagement-authority queue  | ≤ 200 ms                 |
| Engagement decision to weapon system             | ≤ 250 ms                 |
| Audit chain entry available after operation      | ≤ 1 s                    |
| Federation manifest expiry alert lead time       | ≥ 30 days                |

These SLAs are tight because fire-control timelines for hypersonic
or short-range tactical threats compress engagement windows to
seconds. Tighter SLAs are negotiable per deployment; loosening them
requires operational-command sign-off.

## §2 Sensor fleet registry

The deployment maintains a registry of every fielded sensor:

- `sensorRef` — URN
- vendor, model, serial number
- modality (radar, IR/EO, space-based, distributed)
- firmware version, last firmware update
- calibration status — date, calibrating laboratory
- TLS client certificate fingerprint and expiry
- operating-area policy (where the sensor is permitted to ingest)

A sensor not in the registry, or with expired calibration or
expired certificate, is refused at PHASE 2 §1 ingest.

## §3 Fusion-engine deployment

Fusion engines correlate observations across the sensor fleet.
The integration contract:

- Each fusion engine has a unique authority URN; authority
  collisions are rejected at registration
- Engines subscribe to observation streams from sensors in their
  operating area
- Engines publish fused tracks; track-correlation events
  (PHASE 2 §7) link tracks across engines
- Multi-engine deployments use a federated authority for tracks
  correlated across engines, named explicitly in the federation
  manifest

A fusion engine that fails to keep up with observation arrival
rate degrades gracefully (drops oldest pending observations,
records dropped count) rather than crashing.

## §4 Weapon-system integration

Weapon systems are addressable endpoints with their own signing
keys. The integration contract:

- Each weapon system registers its readiness state and operating-
  area scope; engagement decisions refusing weapons outside scope
  are emitted automatically
- Weapon systems acknowledge engagement decisions before status
  reporting begins; an unacknowledged decision past the deployment's
  acknowledgement SLA escalates to operational command
- Weapon systems persist their own status records locally so that
  audit recovery is possible after network outage

A weapon system that loses connectivity continues persisting
status locally and reconciles with the boundary on reconnection.

## §5 Civil-airspace overlap

Many missile-defence operating areas overlap civil airspace. The
integration contract:

- Civil flight plans (ICAO Doc 4444 format) are correlated against
  tracks at fusion time; identification is biased toward `friend`
  for tracks within a registered flight plan
- Engagement decisions on tracks correlated to a civil flight plan
  require an explicit override authorisation flagged in the
  decision record
- Coordination with civil air-traffic services is recorded as a
  cross-system reference; the WIA-military-communication boundary
  carries the coordination message

The deployment's standing operating procedure documents the
civil-correlation policy so that engagement decisions can be
audited against doctrine.

## §6 Casualty coordination

Successful intercepts can produce debris fields with civilian
casualty risk; failed intercepts produce impact damage assessments
that drive casualty response. The integration contract:

- Outcome records (PHASE 1 §8) reference any debris-field model
  output and any predicted casualty area
- Casualty-response coordination flows through WIA-medical-data-
  privacy and WIA-medical-imaging via the WIA-military-communication
  boundary; identifiers preserve pseudonymity across the medical
  boundary

This standard does not duplicate the medical standards' clinical
records; it references them so the missile-defence audit chain
ties to the medical audit chain across the incident.

## §7 Coalition exchange

Multi-national coalition deployments exchange tracks, threat
assessments, and (where authorised) engagement decisions
bilaterally under a federation manifest:

- Each pair of nations signs a federation manifest enumerating
  which records flow which direction, which weapon systems each
  party may task on the other's tracks, and the release authorities
- The boundary honours the manifest at every cross-national
  release request
- Manifest expiry suspends cross-national flows; manifest renewal
  is signed and recorded as an AuditEvent

Ad-hoc cross-coalition tasking outside the manifest requires both
nations' release officers to sign individually; the boundary
refuses ad-hoc release without dual signatures.

## §8 Quarterly compliance report

The boundary emits a quarterly compliance report covering:

- Total observations ingested by sensor and modality
- Fusion-engine track creation rate, federation correlation rate
- Threat assessments by level and confidence band
- Engagement decisions by type and weapon system
- Weapon-system state-transition completeness
- Outcomes by category and confidence
- Cross-coalition exchanges by federation peer
- Federation manifest health (active, expiring, expired)
- Sensor calibration health (current, expiring, expired)
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit chain
so that report tampering would surface in the chain.

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every fielded sensor and weapon system is in its respective
   registry with current calibration and current certificate
2. Every observation, decision, and outcome in the past quarter
   has a matching audit chain entry with verifiable inclusion proof
3. Every cross-coalition release in the past quarter has both
   release-authority signatures on file
4. Civil-flight-plan correlation is in effect for all tracks in
   civil-airspace-overlap operating areas
5. Engagement-authority scopes are current and match the
   doctrine on file
6. Quarterly compliance report has no integrity-check failures

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Common pitfalls (informative)

- **Track-ID drift after fusion-engine restart** — engines that
  reset `seq` to 0 after restart create UID collisions; deployments
  SHOULD persist the last-issued `seq` across restarts
- **Civil-flight-plan staleness** — flight plans change in real
  time; a stale flight-plan cache can mis-identify a friendly
  airliner as `unknown`. The civil-correlation cache MUST refresh
  at least every 5 minutes during active operations
- **Federation manifest renewal lag** — coalition exercises often
  reveal expired manifests at the worst moment; deployments SHOULD
  schedule renewal a calendar quarter before operational need
- **Engagement-authority scope drift** — operating areas shift over
  time; engagement-authority scope claims SHOULD be reviewed at
  every doctrine update

## Annex B — Decommissioning (informative)

When a deployment is decommissioned, the final daily root is
sealed, outstanding decisions are either resolved or cancelled,
weapon systems are returned to safed state, and the chain is
exported to the receiving custodian. The decommissioning manifest
is itself an audit event in the final chain root, signed by both
outgoing and incoming custodians so coalition partners can trace
continuity across the handover.

## Annex C — Federation manifest worked example (informative)

```yaml
federation:
  parties:
    - {orgRef: "urn:wia:org:rok-army.adcc", kid: "ka-rok-2026"}
    - {orgRef: "urn:wia:org:us-army.380adabde", kid: "ka-us-2026"}
  flows:
    - {from: "rok", to: "us", purposes: ["operational"], records: ["tracks", "assessments"]}
    - {from: "us", to: "rok", purposes: ["operational"], records: ["tracks", "assessments", "decisions-on-rok-airspace"]}
  trackCorrelation:
    federatedAuthority: "urn:wia:md:fusion:rok-us-fed-01"
    correlationPolicy: "kinematic-co-track + iff-mode-5 cross-validate"
  weaponTaskingScope:
    - {fromParty: "us", toWeaponSystem: "urn:wia:md:ws:rok-thaad-bty-1", scope: "tactical-ballistic in joint-aor"}
  expiry: "2027-04-27"
  signatures: [<JWS-by-rok>, <JWS-by-us>]
```

A manifest with one signature missing is rejected by both peers;
the manifest is the live agreement, not a draft.

## Annex D — Operational lessons-learned register (informative)

Recurring operational lessons-learned across exercises:

- **Identification latency** — NCTR + IFF correlation take time;
  short-range threats may transition friend → hostile inside the
  engagement decision window. The deployment SHOULD run periodic
  drills with hostile-only simulators to keep latency tight
- **Weapon-system cross-tasking** — coalition partners with
  different weapon mixes often have different optimal target
  classes; the federation manifest's weapon-tasking scope SHOULD
  be reviewed quarterly against the threat-mix forecast
- **Civil flight plan failure mode** — when civil ATC data feeds
  go offline, default identification falls back to `unknown`;
  engagement decisions on `unknown` tracks SHOULD require an
  additional review step in this state
- **Audit-chain reconciliation drift** — weapon systems' local
  buffers occasionally accumulate during long-duration outages;
  exercises SHOULD include reconciliation-after-outage drills

## Annex E — Decommissioning (informative)

When a deployment is decommissioned:

1. Outstanding engagement decisions are either resolved (success,
   miss, aborted) or cancelled with structured reasons recorded
2. Weapon systems are returned to safed state; their final state
   transition is recorded as a decommissioning event
3. Active spectrum coordinations are released back to the
   coordinating authorities
4. Final daily root is sealed and the chain is exported to the
   receiving custodian
5. Coalition partners are notified via the WIA-military-communication
   boundary so the federation manifest can be retired or
   re-pointed at a successor deployment

The decommissioning manifest is itself an audit event in the final
chain root, signed by both outgoing and incoming custodians so
coalition partners can trace continuity across the handover.

## Annex F — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. Coalition operations
prefer staged migration: a major-version bump rolls out at one
deployment per quarter so partners adapt incrementally. Deprecated
versions enter a 12-month sunset window during which the registry
marks them deprecated and surfaces migration notes in the audit
chain. Patch-level errata are issued without a deprecation window
because they do not change normative behaviour.
