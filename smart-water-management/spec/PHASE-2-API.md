# WIA-CITY-020 — Phase 2: API Interface

> Smart-water canonical Phase 2: API surface (assets + telemetry + meters + quality + leak-events + telemetry + audit).

# WIA-CITY-020: Smart Water Management Standard v1.0

## Executive Summary

The WIA-CITY-020 standard defines requirements for intelligent lighting systems in smart city environments. This standard enables 60-80% energy savings through LED technology, occupancy sensing, daylight harvesting, and AI-driven optimization.

**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity


## 4. Sensors

### 4.1 Occupancy Sensors (Bronze+)
- Detection rate: >95%
- False positive rate: <2%
- Coverage: PIR, ultrasonic, or microwave

### 4.2 Daylight Sensors (Silver+)
- Range: 0-2000 lux
- Accuracy: ±10%
- Response time: <1 second



## 5. Communication Protocols

### 5.1 Required Support (Silver+)
- Zigbee, Bluetooth Mesh, or Thread
- DALI-2 for fixture control
- RESTful API for cloud connectivity




---

## A.1 Endpoint reference

```http
POST /smart-water-management/v1/assets             # register asset
GET  /smart-water-management/v1/assets/{id}        # fetch asset record
POST /smart-water-management/v1/telemetry          # ingest sensor reading
GET  /smart-water-management/v1/telemetry/{id}     # fetch reading
POST /smart-water-management/v1/meters             # register meter
GET  /smart-water-management/v1/meters/{id}/usage  # meter usage envelope
POST /smart-water-management/v1/quality-results    # publish quality result
POST /smart-water-management/v1/leak-events        # publish leak event
WS   /smart-water-management/v1/state/stream       # state stream
GET  /smart-water-management/v1/audit/{id}         # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-smart-water-management`. Asset-mutation endpoints
require the requester's utility-administrator credential plus the
per-utility regulatory-scope envelope. Customer-meter-data endpoints
require the per-customer consent envelope per the operator's per-
jurisdiction privacy policy.

## A.2 Asset-registration API

`POST /assets` accepts the Phase 1 §A.2 envelope. The endpoint
validates the asset-class envelope against the per-utility GIS
catalogue, links to the per-utility CMMS (Computerised Maintenance
Management System) envelope per ISO 55000 + ISO 55001 + ISO 55002
asset-management policy, and emits the registration event. Asset
state transitions through `provisioning`, `in-service`, `degraded`,
`out-of-service`, `retired`; transitions emit audit events.

## A.3 Telemetry ingest API

`POST /telemetry` accepts the Phase 1 §A.3 envelope. The endpoint
deduplicates by per-sensor + per-timestamp, validates the per-sensor
calibration-state envelope (a reading from an out-of-calibration
sensor is accepted with a non-fatal warning + `pending-revalidation`
flag), normalises the per-sensor unit envelope (per ISO 80000),
applies the per-sensor anomaly-detection envelope (per-sensor
trailing-window z-score + per-sensor rate-of-change envelope),
and emits the per-sensor anomaly event when the reading exceeds
the per-sensor threshold.

## A.4 Customer-meter API

`POST /meters` registers a customer meter per Phase 1 §A.4. `GET
/meters/{id}/usage` returns the per-meter usage envelope: per-
billing-period totalised consumption; per-15-minute consumption
profile per AMI envelope; per-meter leak-detection signature (per-
meter minimum-flow during low-demand period above the per-meter
expected-leak threshold per AWWA M36); per-meter reverse-flow event
envelope; per-meter tamper event envelope. Per-customer data access
follows the per-jurisdiction privacy envelope per Phase 4 §Z.8.

## A.5 Water-quality API

`POST /quality-results` accepts the Phase 1 §A.5 envelope. The
endpoint validates the per-result analytical-method envelope against
the per-utility laboratory's ISO/IEC 17025 accreditation scope,
checks the per-result against the per-jurisdiction MCL envelope,
emits a per-MCL-exceedance event when the per-result triggers the
per-jurisdiction action-level (per US EPA Safe Drinking Water Act
+ EU Directive (EU) 2020/2184 + UK Drinking Water Quality
Regulations + per-jurisdiction-equivalent), and routes the
exceedance to the per-utility regulatory-reporting envelope per
the per-jurisdiction reporting deadline.

## A.6 Leak-event API

`POST /leak-events` accepts a leak-event envelope: detection-method
envelope (acoustic-correlator per AWWA Manual M36 §3.4; pressure-
transient analysis per IWA Water Loss Specialist Group; minimum-
night-flow per IWA WLSG; satellite-imagery anomaly per the operator's
satellite vendor; ML-based pattern-detection per the operator's
data-science envelope); per-event location envelope (per-DMA + per-
pipe-section per the per-utility GIS); per-event severity envelope
(burst / major / minor per AWWA M36 classification); per-event
estimated-loss envelope (volume + duration); per-event repair
envelope (per-event response-time + per-event repair-completion
event); per-event audit envelope linking back to the per-utility
work-order envelope.

## A.7 Telemetry WebSocket

The state-stream WebSocket multiplexes per-utility events: per-
sensor anomaly events; per-sensor threshold-crossing events; per-
DMA pressure + flow events; per-meter leak-signature events; per-
meter tamper events; per-quality-result MCL-exceedance events;
per-leak-event detection + repair events; per-asset condition-
class change events. Subscribers can filter by asset-class, DMA-
id, event-class, severity-class. Rate limits: 5000 req/h authenticated;
50000 req/h trusted-partner. WebSocket subscriptions are bounded
at 100 simultaneous per credential.

## A.8 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: asset
registration, every telemetry-ingest event, every meter-registration
event, every quality-result-publish event with the per-MCL outcome
envelope, every leak-event publish + repair event, every credential
change, every regulatory-report submission event with the per-
report submission-confirmation envelope. The audit-trail integrity
is anchored into a Merkle tree per-utility; the root is committed
to the operator's regulatory-record system per the per-jurisdiction
retention envelope (typical 10 years for SDWA records per 40 CFR
141.33; per-jurisdiction-equivalent).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-water-management/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-water-management-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-water-management-host:1.0.0` ships every smart-water-management envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-water-management.sh` ships sample envelope generators with no
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
ecosystem. Smart-water-management deployments that follow this layering
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
`/.well-known/wia-smart-water-management-capabilities` that enumerates which
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
