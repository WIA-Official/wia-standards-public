# WIA-MED-026 — Phase 2: API Interface

> Rehabilitation-device canonical Phase 2: API surface (devices + sessions + streams + adverse-events + telemetry + audit).


## A.1 Endpoint reference

```http
POST /rehabilitation-device/v1/devices                  # register device
GET  /rehabilitation-device/v1/devices/{id}             # fetch device record
POST /rehabilitation-device/v1/sessions                 # start session
GET  /rehabilitation-device/v1/sessions/{id}            # fetch session
POST /rehabilitation-device/v1/sessions/{id}/streams    # ingest sensor stream
POST /rehabilitation-device/v1/adverse-events           # report adverse event
GET  /rehabilitation-device/v1/clinical-evidence/{id}   # fetch evidence package
WS   /rehabilitation-device/v1/state/stream             # real-time stream
GET  /rehabilitation-device/v1/audit/{id}               # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-rehabilitation-device`. Device-mutation endpoints
require the operator's clinical-administrator credential + per-
device IFU acknowledgement envelope. Patient-data endpoints
additionally require the per-clinician licence envelope per the
per-jurisdiction professional-licensing authority (per US state
medical board; per UK GMC + HCPC; per-EU member-state professional-
register; per-jurisdiction-equivalent).

## A.2 Device-registration API

`POST /devices` accepts the Phase 1 §A.1 envelope. The endpoint
validates the per-device UDI per FDA UDI Database / EUDAMED, links
to the per-device clinical-evidence envelope per §A.5, and emits
the registration event. Devices transition through `provisioning`,
`installed`, `commissioned`, `in-service`, `decommissioned`,
`retired`; transitions emit audit events.

## A.3 Session API

`POST /sessions` accepts the Phase 1 §A.2 envelope. The endpoint
verifies that the patient's prescription envelope is current (per-
prescription expiry envelope per the operator's clinical policy),
verifies device-class ↔ patient-class compatibility, allocates a
per-session record, and emits the session-start event. Subsequent
state transitions (`paused`, `resumed`, `completed`, `aborted-by-
clinician`, `aborted-by-safety`, `error`) emit per-state audit
records with the operator-credential trace.

## A.4 Sensor-stream API

`POST /sessions/{id}/streams` accepts the Phase 1 §A.3 envelope.
The endpoint normalises per-channel sampling-rate, applies per-
channel filter envelope where the operator's policy mandates server-
side filtering, computes per-channel anomaly events (per-channel
threshold-crossing + per-channel out-of-range + per-channel signal-
quality-degraded), and forwards the per-stream event to the
operator's analytics envelope for downstream outcome computation.

## A.5 Adverse-event API

`POST /adverse-events` accepts an adverse-event envelope: per-event
severity per CTCAE; per-event relationship-to-device classification
per FDA + EU MDR + Health Canada criteria; per-event per-jurisdiction
reportability envelope (FDA MAUDE per 21 CFR 803; EU MDR EUDAMED
incident-report per Article 87; per-jurisdiction-equivalent); per-
event corrective-action envelope. The endpoint routes the per-
event report through the operator's regulatory-reporting workflow
per the per-jurisdiction reporting deadline (typical 30-day +
expedited 5-day for serious-injury / death events).

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-session events: per-
sensor-channel anomaly events; per-device safety-threshold-crossing
events; per-session pause / resume events; per-session adverse-
event events; per-device fault-detection events; per-device
calibration-due events. Subscribers can filter by device-id,
session-id, severity-class. Rate limits: 5000 req/h authenticated;
50000 req/h trusted-partner. WebSocket subscriptions are bounded
at 100 simultaneous per credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: device
registration, every session start + end + state-transition event,
every adverse-event report, every clinical-evidence-publish event,
every credential change, every regulatory-report submission. The
audit-trail integrity is anchored into a Merkle tree per-clinical-
site; the root is committed to the operator's archival record per
21 CFR 820.180 + EU MDR Article 10(8) + per-jurisdiction-equivalent
(typical 10 years for medical devices; 15 years for implantable
devices per MDR Article 10(8)).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/rehabilitation-device/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-rehabilitation-device-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/rehabilitation-device-host:1.0.0` ships every rehabilitation-device envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/rehabilitation-device.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Rehabilitation-device deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-rehabilitation-device-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

## Z.9 Disaster recovery and continuity-of-operations envelope

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-
exit envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity. The DR
envelope composes with WIA Secure Enclave for sealed-backup
envelopes and with WIA-AIR-SHIELD for runtime trust-list re-
hydration on the failover instance.

## Z.10 Supply-chain and software-bill-of-materials envelope

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per the operator's chosen specification: SPDX 2.3 / 3.0 per
ISO/IEC 5962 + Linux Foundation SPDX, or CycloneDX 1.6 per OWASP
Foundation. The SBOM enumerates every direct + transitive dependency
with the per-component name + version + licence + supplier + per-
component hash + per-component PURL (Package URL per package-url
spec) + per-component CPE (Common Platform Enumeration per NIST).
The host MUST publish per-release SBOM updates and MUST flag
breaking dependency-version migrations so downstream consumers
can plan ahead. Supply-chain attestation follows in-toto per
CNCF in-toto + SLSA (Supply-chain Levels for Software Artifacts)
per OpenSSF SLSA Framework — typically targeting SLSA Level 3 for
hosted production deployments.

弘益人間 — Benefit All Humanity.
