# WIA-network-security PHASE 4 — Integration Specification

**Standard:** WIA-network-security
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a network-security deployment
integrates the data, APIs, and protocols from PHASEs 1–3
with broader operational systems: SOC operations, SIEM
correlation pipelines, vulnerability-management programmes,
incident-response platforms, threat-intelligence platforms,
identity-and-access-management (IAM), endpoint detection,
network detection, and partner intelligence sharing. It is
non-prescriptive about specific vendors; it specifies the
integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 27001:2022, ISO/IEC 27035 — information-security
  and incident-management frameworks
- OASIS STIX 2.1 / TAXII 2.1
- OASIS OpenC2
- MITRE ATT&CK / D3FEND
- IETF RFC 6545 — Real-time Inter-network Defense (RID)
- IETF RFC 9356 — Vulnerability Reporting Format
- WIA-pq-crypto (PHASE 1–4) — for cipher-suite governance
- WIA-privacy (PHASE 1–4) — for PPI-class controls
- WIA-supply-chain (PHASE 1–4) — for software-inventory join

---

## §1 SOC operations integration

The deployment's SOC consumes the boundary as the canonical
event/alert/incident surface:

- Tier-1 analysts triage alerts via PHASE 2 §7
- Tier-2 analysts manage incidents via PHASE 2 §6
- SOC lead handles cross-shift hand-off and escalations
- IR playbooks reference the action-types in PHASE 1 §9 and
  use the OpenC2 alignment to drive control-plane execution

Integration contracts:

- Analyst tokens are scoped to assets and severity bands per
  the SOC's tiering policy
- Action-blast-radius limits per role are published in the
  capability document
- The SOC's runbooks are published as references that
  alerts and incidents may link to

A SOC without a documented runbook surface does not satisfy
Verified conformance.

## §2 SIEM correlation pipeline

The deployment integrates a SIEM correlation pipeline:

- Events from PHASE 2 §2 feed correlation-rule evaluation
- Sigma-aligned rules are stored with their `correlationRuleRef`
  URN; rule mutations are versioned and audit-chained
- Alerts emitted by correlation reference the rule URN

Integration contracts:

- Rule library published in the capability document with a
  Merkle root for partner verification
- Rule changes routed through the deployment's change-
  approval workflow with a sign-off record
- False-positive feedback (PHASE 2 §7 `false-positive`
  state) feeds rule-tuning queue

## §3 Threat-intelligence platform (TIP) integration

The TAXII surface (PHASE 2 §4) is the canonical exchange
point for STIX-aligned intelligence:

- Inbound TIP feeds publish into TAXII collections
- Outbound TIP shares submit to partner TAXII surfaces
- TIO records are joined with internal events to identify
  active campaigns

Integration contracts:

- Inbound feeds are vetted before promotion to production
  collections; vetting outcomes are audit-chained
- Outbound shares respect TLP markings; markings are
  enforced at the boundary regardless of the consuming
  partner's discipline
- Partner identities are recorded in the capability
  document and verified per request

## §4 Vulnerability management

CSAF 2.0 documents and the vulnerability records (PHASE 1 §6):

- Asset-software-inventory join resolves
  `affectedAssetCount`
- KEV-flagged vulnerabilities trigger elevated remediation
  SLAs per the deployment's policy
- Patch-management systems consume `pending` and
  `in-progress` vulns via webhook subscription
- Risk-acceptance is recorded with a signed record from the
  asset owner; risk-acceptance reviews are scheduled per the
  policy

Integration contracts:

- Software-inventory join must complete within the
  deployment's `vuln-resolution-latency-budget`
- KEV propagation from CISA-style published lists is part of
  the deployment's intelligence-feed integration

## §5 Incident-response platform (IRP) integration

The IRP consumes the boundary as the system of record for
incidents:

- IRP cases reference incident URNs (not the other way
  around); the boundary remains canonical
- State changes flow bi-directionally via webhook + IRP
  outbound calls to PHASE 2 §6
- Analyst notes in the IRP attach as referenced documents,
  not as opaque blobs in the canonical record
- Lessons-learned posts are written back to the boundary at
  closure

Integration contracts:

- IRP outbound calls authenticate as `analyst` role tokens
- IRP must respect TLP markings when surfacing incident
  context to integrating tools

## §6 Identity and access management (IAM)

Response actions targeting identities (`disable-account`,
`revoke-token`, `force-mfa`) integrate with the IAM:

- IAM publishes account URNs into the asset inventory
- Action execution against an account requires the IAM's
  control-plane token
- Account state changes flow back via webhook

For privileged accounts, blast-radius limits restrict actions
to a reviewable subset; full account disable for privileged
accounts requires SOC-lead approval.

## §7 Endpoint detection and response (EDR)

EDR vendors are detector sources (PHASE 1 §3 `detectorRef`)
and action targets (PHASE 1 §9 `quarantine-host`,
`kill-process`):

- EDR onboarding uses the detector handshake (PHASE 3
  Annex G)
- Action latency is monitored per EDR vendor; persistent
  slow execution triggers a re-onboarding review
- EDR vendor extensions are accepted but MUST NOT contradict
  the canonical fields

## §8 Network detection and response (NDR)

NDR sensors feed events with packet-capture references in
the `rawEventRef` URI. Integration contracts:

- Packet captures referenced by URI must remain accessible
  for the deployment's retention window
- High-severity events with packet-capture references are
  preserved beyond the routine retention window pending
  incident closure
- NDR vendor extensions follow the same rules as EDR

## §9 Partner intelligence sharing

Cross-organisation sharing combines TAXII and RID:

- TAXII for ongoing intelligence-object exchange
- RID for in-incident notifications (PHASE 3 §7)
- Coalition-trust roster published in the capability
  document; partner identities are verified before each
  exchange

Integration contracts:

- TLP markings flow with shared records
- Receipt acknowledgements are audit-chained
- Persistent partner-side delivery failure triggers
  partner-engagement escalation

## §10 Conformance evidence package

Anchored conformance requires a continuous evidence package:

- Capability document signed by the deployment
- Audit-chain Merkle roots regulator-witnessed
- Rule library Merkle root
- Partner-roster signed snapshot
- ISO/IEC 27001 audit reports (most recent)
- Incident-closure archive with lessons-learned references

The evidence package is published to the deployment's
attestation surface; partners and regulators consume it as
the canonical conformance signal.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain integration matrix

| Cross domain                  | Hand-off                                         |
|-------------------------------|--------------------------------------------------|
| WIA-pq-crypto                 | cipher-suite floor and PQ migration              |
| WIA-privacy                   | PPI-class enforcement on event re-disclosure     |
| WIA-supply-chain              | software-inventory join for vulnerability resolve|

## Annex B — Conformance level summary

| Level     | Scope                                                |
|-----------|------------------------------------------------------|
| Surface   | structural conformance to PHASEs 1–3                 |
| Verified  | annual third-party ISO/IEC 27001 audit               |
| Anchored  | continuous evidence package per §10                  |

## Annex C — Operator console contract

SOC consoles integrate with the boundary via:

- Live-event view fed by PHASE 2 §2
- Alert triage view fed by PHASE 2 §7
- Incident workspace fed by PHASE 2 §6
- Action audit view fed by §1 of this PHASE
- Vulnerability dashboard fed by PHASE 2 §5

Console actions that mutate boundary state always carry the
analyst's authority URN; consoles authenticate as
`analyst-console` role tokens.

## Annex D — Public attestation surface

Anchored conformance publishes an attestation surface
containing the items in §10 plus the deployment's
cryptographic algorithm registry, partner-roster snapshot,
and rule-library Merkle root. Partners and regulators
consume the surface as the canonical conformance signal.

## Annex E — Tabletop exercise integration

Periodic tabletop exercises validate integration contracts:

- A scripted scenario is executed against the boundary in a
  sandbox lane
- Detector mocks emit pre-recorded events
- Analysts run the scenario via PHASE 2 endpoints
- IRP, IAM, and EDR integrations are exercised end-to-end
- Findings are recorded in a tabletop report attached to the
  conformance evidence package

Anchored conformance requires at least one tabletop exercise
per declared cadence (typically annual). The tabletop report
is a public-attestation artifact (sanitised of operational
specifics) so partners can verify the deployment's
operational maturity.

## Annex F — Cross-vendor detector registry

For deployments operating heterogeneous detector fleets, the
detector registry publishes per-detector capability cards:

- `detectorRef` — URN
- `vendorRef` — vendor URN
- `detectorClass` — `edr`, `ndr`, `siem-rule`, `cloud-native`,
  `application-layer`
- `eventSchemasSupported[]`
- `confidenceCalibrationDate`
- `onboardingDate`
- `decommissionPolicy`

The registry is the canonical join for understanding the
deployment's detection coverage. Coverage gaps (e.g., no
detector emitting MITRE T1059 events on a tier-1 asset
class) are surfaced to the SOC's coverage-review programme.

## Annex G — Cross-organisation incident sharing

For incidents requiring cross-organisation coordination
(supply-chain compromise, sector-wide campaigns), the
deployment integrates with sectoral ISACs/ISAOs:

- ISAC membership is recorded in the partner roster
- Eligible incidents are shared via TAXII or RID per the
  partner agreement
- Sharing decisions respect TLP markings on the underlying
  artifacts
- Sharing-decision audit-chain entries reference the
  partner agreement URN

A deployment without a documented sharing-decision workflow
does not satisfy Anchored conformance for sectors where
ISAC participation is industry-standard.

## Annex H — Cloud-native detector integration

For cloud-native deployments (CSP-managed services), detector
integration adds:

- CSP-native log routes (e.g., audit logs, network flow logs,
  service-specific logs) feed events via PHASE 2 §2
- IAM tokens for cloud-native detectors are CSP-issued
  workload identities, signed and verified per CSP policy
- Cloud-native action targets use CSP-native action APIs
  with the boundary's OpenC2 verbs mapped per a documented
  translation layer

The translation layer is published in the capability document
for partner verification. Cloud-native detector calibration
follows the same handshake as on-premises detectors (PHASE 3
Annex K).

## Annex I — Operational metrics export

The deployment exports operational metrics for SOC-leadership
visibility:

- Mean time to detect (MTTD) per severity band
- Mean time to respond (MTTR) per severity band
- False-positive rate per detector
- Coverage by MITRE ATT&CK technique
- Incident count by sector / asset class

Metrics export is gated; SOC-leadership consumers receive a
curated dashboard, while operators see cycle-level metrics
filtered to their assets.
