# WIA-nbc-defense PHASE 4 — Integration Specification

**Standard:** WIA-nbc-defense
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a deployment integrates the data, APIs,
and protocols defined in PHASEs 1–3 with the operational picture:
sensor fleet management, command-and-control (C2) systems, plume-
modelling pipelines, decontamination logistics, casualty evacuation
chains, public-health bridges, and coalition exchange. It is non-
prescriptive about specific vendor products; it specifies the
integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- STANAG 4694 — NATO Common Data Exchange (CBRN-specific subset)
- STANAG 4587 — Symbology for tactical maps (to render NBC overlays)
- STANAG 2103 / AArtyP-1 — NBC report formats
- ATP-3.8.1 / AJP-3.8 — Comprehensive CBRN Defence doctrine
- ISO 19115 — geographic information metadata
- WIA-medical-data-privacy (PHASE 1–4) — for the medical-bridge
- WIA-medical-imaging (PHASE 1–4) — for imaging-supported triage
- WIA-public-safety — for civilian-emergency integration
- WHO IHR (2005) — for public-health notification flows

---

## §1 Sensor fleet registry

The deployment maintains a registry of every fielded sensor:

- `sensorId` — URN
- vendor, model, serial number
- modality (chemical, biological, radiological, nuclear, multi-mode)
- detection technology (IMS, FPA, GCMS-portable, gamma scintillator,
  thermal-neutron, etc.)
- firmware version, last firmware update
- calibration status — date of last calibration, next due, calibrating laboratory
- TLS client certificate fingerprint and expiry
- operator (the unit / agency owning the sensor)
- operating-area policy (where the sensor is permitted to operate)

A sensor not in the registry, or with expired calibration or expired
certificate, is refused at PHASE 2 §1 ingest. The registry is itself
auditable; every change to a registry entry emits an AuditEvent.

## §2 Field-unit conversion

Sensors emit in their native engineering units, which may not be
in the closed unit list of PHASE 1 §3. The integration boundary:

1. Maps the sensor's native unit (per the registry's vendor table)
   to a permitted unit
2. Records both values in the canonical record (`concentration.value`
   in the canonical unit, plus `concentration.nativeValue` and
   `concentration.nativeUnit` for forensic replay)
3. Refuses ingest if the native unit cannot be cleanly converted
   (analyst review required)

The conversion table is itself versioned and the per-event record
references the conversion-table version so that a future replay
reproduces the same canonical value.

## §3 C2 integration

The C2 system consumes the operational picture by subscribing to
the boundary's event stream:

- New `presumptive`/`confirmed`/`validated` events surface as map
  symbology per STANAG 4587 (CBRN agent-class symbol with
  confidence-band fill)
- Plume forecast contours appear as time-stamped overlay polygons
- Decontamination work-orders surface as task-board entries linked
  to the implicated events
- Triage records surface as casualty-tracker entries linked to the
  relevant medical-evacuation chain

The C2 system does not write back to the boundary; new records flow
in through the operator's PHASE 2 endpoints, not through C2's own
edit path. This preserves a single canonical record per event.

## §4 Plume-modelling pipeline

Plume runs are submitted via PHASE 2 §4. The integration contract:

- The deployment registers each model implementation (HPAC, ALOHA,
  RIMPUFF, JEM, etc.) with its input schema and output product
  shapes
- Inputs are validated against the registered schema before
  queueing
- Outputs are stored at signed URIs referenced by the run's status
  resource
- Output retention follows PHASE 4 §5 below

A run that fails its input validation produces a structured error
record so that the operator can correct the inputs and re-run.

## §5 Output retention

Different output types have different retention policies:

| Output                            | Default retention                    |
|-----------------------------------|--------------------------------------|
| Sensor events (`presumptive`)     | 90 days operational, then archived   |
| Sensor events (`confirmed`)       | 5 years                              |
| Sensor events (`validated`)       | indefinite (legal-evidence horizon)  |
| Plume run inputs and outputs      | 1 year operational, then archived    |
| Decontamination work-orders       | 7 years                              |
| Triage records                    | per the medical-system retention     |
| Sample chain-of-custody logs      | indefinite                           |

Retention exceptions (legal hold, public-inquiry hold) extend the
applicable retention with a recorded justification. Hard deletion
follows the two-person integrity rule from WIA-medical-data-privacy
PHASE 4 §6.

## §6 Decontamination logistics

Work-orders integrate with the deployment's logistics system:

- Required materials (decontaminant solutions, PPE consumables)
  are referenced by NSN (NATO Stock Number) or national equivalent
- Personnel rosters are scoped by unit; the deployment's force-
  protection identifier maps personnel into the work-order without
  exposing personally identifying details to the broader audit
  surface
- Verification events (post-decontamination sensor sweeps) close
  the work-order; an open work-order without a verification event
  past the deployment's SLA escalates to operational command

## §7 Casualty evacuation

Casualty triage records flow into the medical-evacuation chain:

- Triage records project into FHIR Observation + DiagnosticReport
  consumed by the receiving medical facility
- The pseudonymous subject identifier (`subjectRef` per WIA-medical-
  data-privacy PHASE 1 §2) is preserved across handoffs
- Decontamination status is propagated so the receiving facility
  knows whether the casualty is still contaminated, partially
  decontaminated, or decontamination-complete
- The medical-imaging deployment's gate honours the NBC-defence
  triage record as a TREAT-purpose grant for imaging during the
  active treatment window

## §8 Public-health bridge

When NBC events have public-health implications (TIC release in a
populated area, biological-agent presumptive at a transport hub),
the boundary submits an IHR-compatible notification per PHASE 2 §8.
The integration contract:

- The receiver is the national IHR Focal Point
- The notification carries the deployment's release authority's
  signature plus the originating organisation's signature
- Subsequent updates (refinements, all-clear, validated agent ID)
  flow as additional notifications referencing the prior

The bridge is *one-way*: public-health authorities consume but do
not write back; their internal investigations are out of scope of
this PHASE.

## §9 Coalition exchange

Multi-national coalition operations exchange NBC records bilaterally
under a federation manifest:

- Each pair of nations signs a federation manifest enumerating which
  records may flow which direction, under which release authority,
  and for which purposes
- The boundary honours the manifest at every cross-national release
  request
- Manifest expiry suspends cross-national flows; manifest renewal
  is signed and recorded as an AuditEvent

A coalition operation that needs ad-hoc release outside the manifest
requires both nations' release officers to sign each release request
individually; the boundary refuses ad-hoc release without dual
signatures.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Acceptance criteria (informative)

A deployment claims conformance when:

1. Every fielded sensor is in the registry (PHASE 4 §1) with
   current calibration and current TLS certificate.
2. Every event in the past quarter has a matching audit chain
   entry with verifiable inclusion proof.
3. Every cross-national release in the past quarter has both
   release-authority signatures on file.
4. The chain-of-custody log for samples submitted to laboratories
   is unbroken; broken-custody samples were demoted as required.
5. Plume runs invoked in the past quarter validate against the
   registered model schema; failed runs have structured error
   records for follow-up.
6. Triage records flowed into the receiving medical system without
   identifier loss across the boundary.

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

## Annex B — Operational decommissioning (informative)

When a deployment is decommissioned:

1. The signing key is rotated to a final key, the final daily root
   is sealed, and the chain is exported to the receiving custodian
2. Outstanding work-orders are closed (completed, cancelled, or
   transferred) with structured reasons recorded
3. Sample chain-of-custody logs continue under the receiving
   custodian's signing key with a recorded handover event
4. The fleet registry is exported with calibration history so that
   the receiving custodian can continue operating sensors without
   re-calibration

The decommissioning manifest is itself an audit event in the final
chain root, signed by both outgoing and incoming custodians.

## Annex C — Operational SLAs (informative)

| Concern                                          | Default SLA                |
|--------------------------------------------------|----------------------------|
| Sensor event ingest p95 added latency            | ≤ 100 ms                   |
| Tactical link round-trip on STANAG 5066          | ≤ 30 s                     |
| Plume run for ≤ 1 km² area, 4 h horizon          | ≤ 60 s                     |
| Allied report rendering for ≤ 10 events          | ≤ 5 s                      |
| Audit chain entry available after operation      | ≤ 10 s (operational)       |
|                                                  | ≤ 60 s (tactical air-gap)  |
| Federation manifest expiry alert lead time       | ≥ 30 days                  |
| Calibration expiry alert lead time               | ≥ 14 days                  |

Tighter SLAs are negotiable per deployment; loosening them requires
operational-command sign-off.

## Annex D — Quarterly compliance report (informative)

The boundary emits a quarterly compliance report covering:

- Total events ingested by confidence band and domain
- Promotions issued (presumptive→confirmed→validated counts)
- Sample submissions and laboratory turnaround time per laboratory
- Plume runs invoked and average run time per model
- Decontamination work-orders issued, completed, escalated
- Triage records emitted and the receiving medical systems
- Cross-organisation releases with their release-authority signatures
- Federation manifest health (peers active, expiring, expired)
- Sensor fleet calibration health (current, expiring, expired)
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit chain so
that report tampering would surface in the chain.
