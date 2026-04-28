# WIA-COMP-008 — Phase 2: API Interface

> Serverless canonical Phase 2: API surface (functions + versions + invoke + event-sources + aliases + telemetry + audit).

# WIA-COMP-008: Serverless Architecture Specification v1.0

> **Standard ID:** WIA-COMP-008  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active

---


## 4. Security

- Authentication and authorization
- Data encryption
- Network security
- Audit logging

---



## 5. References

- Cloud Native Computing Foundation (CNCF)
- Industry best practices
- WIA related standards

---

**弘익인간 (Benefit All Humanity)**  
*© 2025 SmileStory Inc. / WIA - MIT License*



---

## A.1 Endpoint reference

```http
POST /serverless-architecture/v1/functions               # register function
GET  /serverless-architecture/v1/functions/{id}          # fetch function record
POST /serverless-architecture/v1/functions/{id}/versions # publish new version
POST /serverless-architecture/v1/functions/{id}/invoke   # synchronous invoke
POST /serverless-architecture/v1/functions/{id}/event    # async event invoke
POST /serverless-architecture/v1/event-sources           # bind event source
GET  /serverless-architecture/v1/event-sources/{id}      # fetch binding
POST /serverless-architecture/v1/aliases                 # publish alias
WS   /serverless-architecture/v1/state/stream            # state stream
GET  /serverless-architecture/v1/audit/{id}              # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-serverless-architecture`. Function-mutation
endpoints require the requester's tenant-administrator credential
plus the per-project deployment-quota envelope. Cross-tenant
function visibility is gated by per-tenant explicit grant per the
operator's RBAC envelope.

## A.2 Function-registration API

`POST /functions` accepts the Phase 1 §A.1 envelope. The endpoint
validates the runtime envelope against the platform's supported-
runtime list (a runtime past its end-of-life date is accepted with
a non-fatal warning + a per-function deprecation event), performs
SHA256-based deduplication of the deployment-bundle to avoid
re-uploading content the platform already retains, computes the
per-function expected-cold-start envelope per §A.4, and emits the
registration event. Function records transition through `provisioning`,
`active`, `degraded`, `quarantined`, `retired`; transitions emit
audit events.

## A.3 Versioning API

`POST /functions/{id}/versions` accepts a version-publish envelope
per §A.6. The endpoint records the per-version SHA256, allocates
the platform-side cold-pool warm-up budget (per the operator's
provisioned-concurrency envelope), and advances the alias `latest`
to the newly-published version. Subsequent traffic-shifting events
move a configurable percentage of `live` traffic to the new version
per the operator's progressive-delivery policy. Rollback emits a
per-rollback audit event with the trigger envelope (operator-
manual / automatic-on-SLO-breach / automatic-on-canary-error).

## A.4 Invocation API

`POST /functions/{id}/invoke` (synchronous) and `POST
/functions/{id}/event` (asynchronous) accept invocation envelopes:
event payload (CloudEvents 1.0 envelope per §A.3); per-invocation
trace-context envelope (`traceparent` per W3C Trace Context); per-
invocation idempotency-key per the operator's idempotency policy
per Stripe + per-platform-equivalent. The endpoint enforces the
per-function rate-limit envelope, dispatches the invocation to a
warm execution environment (or starts a cold environment per §A.4),
and returns the synchronous response or the asynchronous correlation
identifier. Errors are bucketed: `4xx` invoker-side; `5xx` platform-
side; `429` rate-limit; `408` timeout per the per-function timeout
envelope.

## A.5 Event-source binding API

`POST /event-sources` accepts an event-source binding envelope:
source-class (HTTP / queue / storage / database / scheduled per
§A.2); per-source filter-expression envelope (per AWS Lambda Event
Filtering + per-platform-equivalent so the function is only invoked
on events matching the filter); per-source batching envelope
(batch-size + batch-window-time per the operator's throughput
policy); per-source dead-letter envelope (failed-event ingest into
DLQ per AWS SQS + Azure Service Bus DLQ + GCP Pub/Sub Dead Letter
Topic); and the per-source retry envelope (per-attempt back-off
per the operator's resiliency policy).

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-function events: per-
invocation start + end events with the per-invocation cold-start
flag + per-invocation duration; per-function error events with the
exception-class breakdown; per-function rate-limit events; per-
function concurrency-saturation events; per-version traffic-shifting
events; per-event-source binding-state events. Subscribers can
filter by function-id, version-id, alias-id, error-class. Rate
limits: 5000 req/h authenticated; 50000 req/h trusted-partner.
WebSocket subscriptions are bounded at 200 simultaneous per
credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: function
registration, every version-publish event, every invocation summary
event (the per-invocation full record may be archived per the
operator's retention envelope), every event-source binding event,
every alias-publish event, every credential change, every quota-
breach event with the operator-acknowledgement envelope, and every
function retirement event. The audit-trail integrity is anchored
into a Merkle tree per-tenant; the root is committed to the
operator's archival record per the per-tenant retention envelope.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/serverless-architecture/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-serverless-architecture-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/serverless-architecture-host:1.0.0` ships every serverless-architecture envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/serverless-architecture.sh` ships sample envelope generators with no
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
ecosystem. Serverless-architecture deployments that follow this layering
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
`/.well-known/wia-serverless-architecture-capabilities` that enumerates which
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
