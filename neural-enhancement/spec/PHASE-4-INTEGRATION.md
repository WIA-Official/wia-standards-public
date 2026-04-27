# WIA-neural-enhancement PHASE 4 — Integration Specification

**Standard:** WIA-neural-enhancement
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a neural-enhancement deployment
integrates the data, APIs, and protocols from PHASEs 1–3
with broader operational systems: clinical electronic
health records (EHR), research electronic data capture
(EDC), participant communication portals, device-vendor
post-market surveillance, regulatory-reporting authorities,
ethics-committee oversight, effector control planes, and
multi-site research networks.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — for EHR exchange resources
- IEC 62304 — Medical device software lifecycle processes
- IEC 62366-1 — Medical device usability engineering
- ISO 13485:2016 — Quality management for medical devices
- ISO 14971:2019 — Risk management
- WIA-privacy (PHASE 1–4) — for participant-pseudonym lifecycle
- WIA-network-security (PHASE 1–4) — for cipher-suite governance
- WIA-supply-chain (PHASE 1–4) — for firmware-update provenance

---

## §1 EHR integration

For clinical-care deployments, the boundary integrates with
the participant's EHR:

- Recording sessions, stimulation sessions, AE records, and
  clinical-context notes are projected as HL7 FHIR R5
  resources for the EHR
- The participant's EHR encounter references the WIA URN
  for cross-system traceability
- The pseudonym-to-MRN binding is held in the participant
  vault (PHASE 3 §5)

Integration contracts:

- FHIR projections are signed and verifiable
- Bidirectional updates: AE state changes flow from EHR
  to boundary and vice versa
- EHR access to non-AE neural data follows the consent's
  scope; the boundary refuses out-of-scope projections

A clinical-care deployment without an EHR integration does
not satisfy Verified conformance.

## §2 Research EDC integration

For research deployments, the boundary integrates with the
study's EDC:

- Recording sessions and decoded-intent records are
  projected for analytics
- Adverse-event reports are mirrored to the study's safety
  desk
- Protocol-deviation events (e.g., out-of-window sessions)
  are surfaced
- Multi-site data harmonisation respects per-site
  pseudonym scope

Integration contracts:

- EDC consumes a FHIR-aligned or BIDS-aligned projection
  per the study's preference
- Per-site pseudonyms are not unified across sites; cross-
  site analyses use per-site aggregates joined at the
  participant-cohort level by independent statisticians

## §3 Participant communication portal

A participant-facing portal provides:

- Self-service consent management (PHASE 2 §7)
- Session calendar view
- Adverse-event self-reporting
- Data-access request handling per the deployment's privacy
  policy

Integration contracts:

- Portal authenticates participants with self-service
  tokens (PHASE 3 §1)
- Portal actions are audit-chained at `kind=participant-action`
- Withdrawal initiated from the portal triggers consent
  withdrawal across all active scopes

## §4 Device-vendor post-market surveillance

Device vendors consume post-market data via webhook
subscription:

- Anonymised AE records (with `tlpClearance` enforcement)
- Anonymised firmware-version performance reports
- Calibration trends across the deployment's fielded fleet

Integration contracts:

- Vendor receives only anonymised data; raw recordings
  remain inside the deployment
- Vendor's post-market obligations under IEC 62366-1 and
  ISO 13485 are supported by the deployment's audit chain

## §5 Clinical-review workflow

Decoded-intent and stimulation-session records feed clinical
review:

- The responsible clinician reviews session outcomes against
  protocol expectations
- Persistent confidence drift in decoder output triggers a
  re-calibration recommendation
- Adverse-effector outputs (e.g., misclassified intents
  causing unwanted device behaviour) trigger a safety-
  envelope review

Integration contracts:

- Review tasks are queued in the clinical team's task
  manager via webhook
- Review outcomes are recorded in the audit chain
- Material outcomes (e.g., calibration mandate) are mirrored
  to the device's registry

## §6 Regulatory-reporting workflow

Adverse events with regulatory-reporting obligations:

1. AE record is published (PHASE 2 §6)
2. Boundary determines reporting obligations from
   `reportingObligationsRef` and the AE severity
3. Reporting templates are populated from the AE record
4. The regulatory officer reviews and submits to the
   relevant authority (FDA, EMA, KMFDS, etc.)
5. Submission acknowledgements are recorded in the AE record's
   `reportedAt` list
6. Closure of the AE follows the regulatory authority's
   resolution

Integration contracts:

- Regulatory report templates are versioned and audit-chained
- Submission failures (e.g., authority unreachable) trigger
  retry on the deployment's declared schedule

A deployment without a regulatory-reporting integration does
not satisfy Verified conformance for clinical-care
deployments.

## §7 Ethics-committee oversight

Research deployments integrate ethics-committee oversight:

- Protocol amendments are submitted to the ethics committee
  with cross-references to existing consent records
- The ethics committee's approval is recorded as a signed
  artifact attached to the protocol record
- AE records of severity `severe` or above are notified to
  the ethics committee within the declared latency

Integration contracts:

- Ethics-committee approvals are versioned and audit-chained
- AE notifications respect TLP markings on attached
  artifacts

## §8 Effector control-plane integration

For BCI applications with connected effectors (cursor
control, prosthetic actuation, communication output):

- Effector control planes are independent systems with their
  own safety envelopes
- The boundary dispatches decoded-intent records to the
  effector control plane via the safety-envelope-cleared
  output
- Effector ack records are returned and audit-chained

Integration contracts:

- Effector control planes verify the safety envelope hash
  before executing
- Cross-effector switching (e.g., from prosthetic arm to
  speech output) requires explicit consent-scope
  authorisation

## §9 Multi-site research-network integration

For multi-site studies, the boundary integrates with the
study coordinating centre:

- Protocol harmonisation through the centre's protocol
  registry
- Aggregate metrics published per site, not raw data
- Cross-site safety signals (e.g., AE clusters) trigger a
  coordinator-led review
- Site-level conformance is reported to the coordinator's
  conformance dashboard

## §10 Conformance evidence package

Anchored conformance requires a continuous evidence package:

- Capability document signed
- Audit-chain Merkle roots regulator-witnessed
- ISO 13485 / IEC 62304 audit reports (most recent)
- ISO 14971 risk-management file (latest version)
- AE-closure archive
- Calibration-trend report
- Consent-framework version history
- Ethics-committee approval archive (where applicable)

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain integration matrix

| Cross domain                  | Hand-off                                         |
|-------------------------------|--------------------------------------------------|
| WIA-privacy                   | participant-pseudonym lifecycle                  |
| WIA-network-security          | cipher-suite floor and revocation                |
| WIA-supply-chain              | firmware-update provenance                       |

## Annex B — Conformance level summary

| Level     | Scope                                                |
|-----------|------------------------------------------------------|
| Surface   | structural conformance to PHASEs 1–3                 |
| Verified  | annual third-party ISO 13485 / IEC 62304 audit       |
| Anchored  | continuous evidence package per §10                  |

## Annex C — Operator console contract

Clinical consoles integrate with the boundary via:

- Live-recording view fed by PHASE 3 §2 binary projection
- Stimulation control panel via PHASE 2 §3
- Decode dashboard via PHASE 2 §4
- AE workspace via PHASE 2 §6
- Calibration management via PHASE 2 §5

Console actions that mutate boundary state always carry the
clinician's authority URN.

## Annex D — Public attestation surface

Anchored conformance publishes an attestation surface
containing the items in §10 plus the deployment's
algorithm registry, partner-roster snapshot, and consent-
framework version history. Partners and regulators consume
the surface as the canonical conformance signal.

## Annex E — Long-term implant lifecycle

Long-term implants (intracortical arrays with multi-year
expected service) integrate lifecycle events:

- Mid-life service events (impedance recovery, channel
  remapping) emit calibration records
- End-of-service events (explant, palliative shutdown) emit
  device-retirement events
- Cross-implant data continuity (where a participant's
  device is replaced) is handled by linking the new device's
  registry record to the prior device via a documented
  succession URN

## Annex F — Cross-pseudonym data continuity

Where a participant's pseudonym is re-issued (e.g., after a
long absence and re-engagement), data continuity is preserved
by linking the new pseudonym to the prior pseudonym via a
succession-binding artifact in the participant vault. The
binding is access-controlled per PHASE 3 §5 and audit-chained
on access.

## Annex G — Decoder-model lifecycle

Decoder model versions integrate with the clinical-review
workflow:

- Each decoded-intent record references a `decoderModelRef`
  URN
- Decoder model mutations (re-training, calibration set
  updates) emit a versioned record with a model-card style
  description
- Performance metrics per session (precision, recall,
  latency) feed model-card updates over time
- Material model changes require responsible-clinician
  sign-off and a re-run of representative recordings

## Annex H — Cybersecurity posture

The deployment's network-security posture (per WIA-network-
security) is integrated:

- Device-link cipher-suite floors are aligned with WIA-
  network-security PHASE 3
- Firmware-update artifacts are verified through WIA-supply-
  chain provenance before installation
- AE records that reflect a suspected cybersecurity event
  are mirrored to the deployment's SOC for parallel
  investigation
- Breach notifications follow both the medical-device
  reporting obligations and the deployment's privacy/
  cybersecurity reporting obligations

## Annex I — Privacy-officer integration

The deployment's privacy officer integrates with the boundary
through:

- Re-identification request review (PHASE 2 Annex J)
- Data-access request handling (PHASE 2 Annex M)
- Disclosure-event review (PHASE 3 §8)
- Consent-framework version review

Privacy-officer actions are themselves audit-chained at
`kind=privacy-officer-action`. Quarterly privacy reviews are
attached to the conformance evidence package; persistent
disclosure refusals are surfaced for policy review.

## Annex J — Tabletop and scenario exercises

Periodic tabletop exercises validate the integration
contracts under realistic scenarios:

- Mass AE scenario: multiple S2-S3 events from a single
  device class across multiple participants
- Suspected cybersecurity event affecting device-link
  authentication
- Multi-site safety signal driving coordinator review
- Participant withdrawal during in-progress stimulation

Findings are recorded in a tabletop report attached to the
conformance evidence package; Anchored conformance requires
at least one tabletop exercise per declared cadence
(typically annual).
