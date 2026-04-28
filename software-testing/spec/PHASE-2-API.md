# WIA-COMP-013 — Phase 2: API Interface

> Software-testing canonical Phase 2: API surface (test-runs + test-results + coverage + flake-reports + telemetry + audit).

# WIA-COMP-013: Software Testing Specification v1.0

> **Standard ID:** WIA-COMP-013  
> **Version:** 1.0.0  
> **Published:** 2025-12-27



---

## A.1 Test-result ingest API surface

```http
POST /software-testing/v1/test-runs                # publish test run
GET  /software-testing/v1/test-runs/{id}           # fetch test run
POST /software-testing/v1/test-results             # publish per-test result
GET  /software-testing/v1/test-results/{id}        # fetch per-test result
POST /software-testing/v1/coverage                 # publish coverage data
GET  /software-testing/v1/coverage/{id}            # fetch coverage data
POST /software-testing/v1/flake-reports            # publish flake report
WS   /software-testing/v1/state/stream             # state stream
GET  /software-testing/v1/audit/{id}               # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-software-testing`. Test-run-ingest endpoints
require the requester's CI / per-tenant test-bot credential plus
the per-project deposit-quota envelope. Cross-project test-result
visibility is gated by the per-project explicit-grant envelope.

## A.2 Test-run ingest API

`POST /test-runs` accepts a test-run envelope: project + build
reference, per-run timestamp, per-run trigger-class envelope (per-
PR / per-commit / per-nightly / per-release-candidate / per-on-
demand); per-run platform envelope (per-platform Linux + macOS +
Windows per the operator's matrix); per-run overall outcome envelope.
The endpoint deduplicates by content-hash over the per-run summary
+ per-test-result-list, links to the per-build artefact envelope,
and emits the per-run-published event.

## A.3 Per-test-result ingest API

`POST /test-results` accepts the §A.2 envelope. The endpoint
performs per-test idempotent upsert (per-test identifier + per-run
identifier as the deduplication key), associates the per-test
result with the per-test history (last 100 per-test runs per the
operator's retention envelope), classifies per-test outcome trend
(STABLE / FLAKING / DEGRADING / IMPROVING per the per-test rolling
window), and emits the per-test trend event when the trend
classification changes.

## A.4 Coverage ingest API

`POST /coverage` accepts a coverage envelope per Phase 1 §A.3:
per-build per-file per-line coverage payload; per-build per-file
per-branch coverage payload; per-build per-file per-condition
coverage payload; per-build aggregate envelope (overall line +
branch + condition + MC/DC percentages). The endpoint computes
per-PR coverage-delta vs the per-target-branch baseline, enforces
the per-project coverage-threshold envelope (per-PR coverage
gate), and emits the per-PR coverage-delta event.

## A.5 Flake-report API

`POST /flake-reports` accepts a flake-report envelope: per-test
identifier; per-flake observation count over the rolling window;
per-flake reproducibility-class envelope (NEVER_REPRODUCED /
REPRODUCED_LOCALLY / REPRODUCED_IN_CI / FIXED per the per-test
investigator's classification); per-flake quarantine state (per
the operator's flake-quarantine policy: tests with flake-rate
>= threshold are auto-quarantined per Google Testing Blog "Flaky
Tests at Google"). The endpoint links the per-flake report to
the per-test history and emits the per-flake event downstream.

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-project events: per-
test-run start + completion events with the per-run summary
envelope; per-test outcome events; per-test trend-classification
events; per-build coverage events; per-build flake events; per-
release quality-gate events. Subscribers can filter by project-id,
build-id, test-id, severity-class. Rate limits: 5000 req/h
authenticated; 50000 req/h trusted-partner. WebSocket subscriptions
are bounded at 100 simultaneous per credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: project
registration, every test-run publish event, every coverage
publish event, every flake-quarantine event with the operator-
acknowledgement envelope, every per-test trend event, every
credential change, every retention-purge event. The audit-trail
integrity is anchored into a Merkle tree per-project; the root
is committed to the operator's archival record per the operator's
quality-record retention policy (typical 7 years for software per
ISO 27001 + per-jurisdiction policy).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/software-testing/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-software-testing-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/software-testing-host:1.0.0` ships every software-testing envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/software-testing.sh` ships sample envelope generators with no
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
ecosystem. Software-testing deployments that follow this layering
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
`/.well-known/wia-software-testing-capabilities` that enumerates which
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
