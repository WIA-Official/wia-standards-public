# WIA-COMP-008 — Phase 3: Protocol

> Serverless canonical Phase 3: protocols (cold-start + concurrency + eventing-fan-out + trace + reliability-retry + auth-tenancy).

# WIA-COMP-008: Serverless Architecture Specification v1.0

> **Standard ID:** WIA-COMP-008  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active

---



---

## A.1 Cold-start mitigation protocol

Cold-start mitigation protocols cover: provisioned-concurrency
protocol (per AWS Lambda provisioned-concurrency envelope; per
Azure Functions Premium plan; per GCP Cloud Run min-instances
envelope) — pre-warmed environments held at platform-side budget,
incurring cost while idle; snapshot-restore protocol (per AWS
Lambda SnapStart for Java; per Firecracker microVM snapshot
restore per Firecracker doc; per CRIU checkpoint-restore per Linux
CRIU project) — per-function cold-start replaced by sub-100-ms
state-restore from a per-function snapshot; lazy-connection
protocol — per-function database / cache connection deferred to
first-invocation reuse so the cold-start path reaches handler
quickly with deferred cost on first invocation; per-function
init-code minimisation protocol (operator-side: minimise top-level
imports + defer heavyweight initialisation to first-handler-call).

## A.2 Concurrency-control protocol

Concurrency-control protocols cover: per-function reserved-
concurrency envelope (AWS Lambda reserved + provisioned per AWS
Lambda doc; per-platform-equivalent) so a per-function spike
cannot consume the tenant-wide concurrency budget; per-tenant
account-level concurrency envelope per the operator's quota policy
per AWS Service Quotas + per-platform-equivalent; per-function
concurrency-spike rate-limit envelope (the per-platform default of
typically 1000 invocations/sec per region per AWS Lambda + per-
platform-equivalent); per-function cold-start vs warm-start budget
envelope so the per-tenant maximum-cold-start-rate is bounded; and
the per-function back-off envelope on platform-side resource
exhaustion (HTTP 429 + Retry-After per RFC 9110 §10.5.6).

## A.3 Eventing-and-fan-out protocol

Eventing-and-fan-out protocols cover: per-event ingest envelope
(per CloudEvents 1.0 binary mode for HTTP; structured mode for
message queues; batched mode for batch-friendly event sources);
per-event fan-out envelope (per-platform fan-out from a single
producer to N subscribers per AWS SNS + Azure Event Grid + GCP Pub/
Sub fan-out subscription envelope); per-event delivery-guarantee
envelope (at-least-once default per most platforms; effectively-
once via idempotency-key per the consumer; exactly-once per
specific patterns per Kafka EOS + Pulsar Functions + Flink
exactly-once-sink envelope); per-event ordering envelope (FIFO per
AWS SQS FIFO + Azure Service Bus sessions + GCP Pub/Sub ordering
key); per-event dead-letter handling envelope (DLQ binding per
the source binding envelope §A.5).

## A.4 Trace-and-tail-latency protocol

Trace-and-tail-latency protocols cover: per-invocation trace
context per W3C Trace Context (`traceparent` + `tracestate` headers)
propagated end-to-end across function-to-function calls + function-
to-database calls + function-to-external-service calls; per-
invocation OpenTelemetry semantic-convention envelope per OTel
Specification 1.40 (function-name + function-version + invocation-
id + cold-start flag); per-platform tail-latency analysis (P50 +
P95 + P99 + P999 per the operator's SLO budget); per-platform
adaptive-timeout envelope (per-call deadline derived from the
upstream caller's remaining-budget per the operator's deadline
propagation policy); per-platform circuit-breaker envelope per
the per-call dependency-health-state envelope.

## A.5 Reliability-and-retry protocol

Reliability-and-retry protocols cover: per-invocation idempotency
envelope (per-invocation idempotency-key passed by the caller +
per-function dedup-store TTL per the operator's idempotency-window
budget); per-invocation retry envelope (synchronous: caller-side
retry per RFC 9110 §15.7.4 + Retry-After; asynchronous: platform-
side retry per AWS Lambda async retry + per-platform-equivalent;
exponential back-off per Google SRE Workbook §22 + jitter per
AWS Architecture Blog "Exponential Backoff and Jitter"); per-
invocation timeout envelope (per-function default + per-invocation
override within the per-function maximum); per-invocation dead-
letter envelope on retry-exhaustion per §A.3.

## A.6 Authorisation-and-tenancy protocol

Authorisation-and-tenancy protocols cover: per-function execution-
role envelope (AWS Lambda IAM execution role + Azure Functions
managed identity + GCP Cloud Run service account + per-platform-
equivalent) tying the function to the per-account permission set
per the principle of least privilege per NIST SP 800-53 AC-6;
per-function caller-identity envelope (caller's JWT per RFC 7519
+ caller's mTLS certificate per RFC 8705 OAuth 2.0 Mutual-TLS
Client Authentication + caller's signed-request per AWS SigV4 +
per-platform-equivalent); per-tenant resource-isolation envelope
(per-tenant VPC + per-tenant subnet + per-tenant KMS key envelope);
per-tenant quota-enforcement protocol (per-tenant concurrency cap
+ per-tenant memory-budget cap + per-tenant rate-limit per the
operator's quota envelope).


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
