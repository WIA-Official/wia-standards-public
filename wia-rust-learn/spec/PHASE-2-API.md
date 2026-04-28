# WIA-RUST-LEARN — Phase 2: API Interface

> Rust-learn canonical Phase 2: API surface (curricula + enrolments + exercise-submissions + assessments + telemetry + audit) per per-platform LRS + xAPI + cmi5.


## A.1 Endpoint reference

```http
POST /wia-rust-learn/v1/curricula                  # register curriculum
GET  /wia-rust-learn/v1/curricula/{id}             # fetch curriculum
POST /wia-rust-learn/v1/enrolments                 # enrol learner
GET  /wia-rust-learn/v1/enrolments/{id}/progress   # fetch progress
POST /wia-rust-learn/v1/exercise-submissions       # submit attempt
GET  /wia-rust-learn/v1/exercise-submissions/{id}  # fetch submission
POST /wia-rust-learn/v1/assessments                # publish assessment
GET  /wia-rust-learn/v1/assessments/{id}           # fetch assessment
WS   /wia-rust-learn/v1/state/stream               # learner stream
GET  /wia-rust-learn/v1/audit/{id}                 # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-rust-learn`. Curriculum-mutation endpoints require
the requester's curriculum-administrator credential plus the per-
delivery-host scope. Per-learner endpoints additionally require
the per-learner consent envelope per the per-jurisdiction privacy
envelope (FERPA per US 20 U.S.C. §1232g + GDPR + per-jurisdiction-
equivalent for student records).

## A.2 Curriculum-registration API

`POST /curricula` accepts the Phase 1 §A.1 envelope. The endpoint
validates the level-record chain per §A.2 (every Level 0..9 must
exist and pass the per-level prerequisite-chain integrity check),
validates the per-curriculum reference-material catalogue per §A.6
against the upstream-material existence (linkchecker per linkchecker
+ lychee per the per-platform link-validity envelope), and emits
the registration event. Curricula transition through `provisioning`,
`active`, `archived`, `removed`; transitions emit audit events.

## A.3 Enrolment API

`POST /enrolments` accepts an enrolment envelope: curriculum
reference, learner pseudonymous identifier, per-learner intake
profile (per-learner programming background per Phase 1 §A.1
target-audience envelope, per-learner declared learning-pace),
per-learner consent envelope (per-jurisdiction student-data consent
per the per-platform consent record). The endpoint validates the
per-curriculum capacity envelope (cohort-based: per-cohort capacity
cap; instructor-led: per-instructor capacity cap; self-paced:
unlimited subject to per-platform quota), allocates a per-learner
progress record per Phase 1 §A.4, and emits the enrolment event.

## A.4 Exercise-submission API

`POST /exercise-submissions` accepts an attempt envelope: enrolment
reference, exercise reference per Phase 1 §A.3, attempt timestamp
per RFC 3339, attempt outcome (per the rustc + cargo test result
envelope), per-attempt code-bundle (per-attempt content-addressed
SHA256 per FIPS 180-4 over the submission diff), per-attempt
hint-consumption envelope. The endpoint deduplicates by content-
hash, performs the per-exercise grader run (compile-check,
output-match, test-suite execution, or benchmark — per the per-
exercise class), and emits the per-attempt outcome event with the
per-attempt mastery-state update per Phase 1 §A.4.

## A.5 Assessment API

`POST /assessments` accepts an assessment-publish envelope: per-
assessment evaluator-credential envelope (per-evaluator instructor
identifier + per-evaluator certification scope per the per-host
credentialing policy); per-assessment rubric reference per Phase 1
§A.5; per-assessment evidence envelope (per-evidence git-URL +
per-evidence commit-hash + per-evidence artifact-list); per-
assessment outcome envelope (per-criterion score + per-criterion
narrative + per-assessment overall mastery flag); per-assessment
certification-eligibility envelope. The endpoint validates the
evaluator-credential envelope, links the assessment to the per-
learner Phase 1 §A.4 mastery-state, and emits the per-assessment
event.

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-curriculum events: per-
learner attempt events; per-learner mastery-state-change events;
per-learner abandonment-risk events (per-learner inactivity beyond
the per-curriculum trigger envelope); per-learner help-request
events; per-cohort cohort-progress events; per-curriculum exercise-
difficulty-trend events (per-exercise per-cohort attempt-count +
per-exercise per-cohort hint-consumption + per-exercise per-cohort
time-to-completion). Subscribers can filter by curriculum-id, level-
id, learner-id, severity-class. Rate limits: 5000 req/h authenticated;
50000 req/h trusted-partner. WebSocket subscriptions are bounded
at 100 simultaneous per credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: curriculum
registration, every enrolment event, every exercise-submission event
(the per-attempt full payload may be archived per the per-platform
retention envelope), every assessment-publish event, every
credential change, every consent-revocation event with the per-
revocation data-handling response envelope per the per-jurisdiction
privacy regulation. The audit-trail integrity is anchored into a
Merkle tree per-host; the root is committed to the per-host
archival record per the per-jurisdiction student-record retention
policy (FERPA + per-state retention; GDPR + per-jurisdiction
educational-records retention; typical 5-7 years for in-progress
records, longer for credentialing transcripts).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/wia-rust-learn/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-wia-rust-learn-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/wia-rust-learn-host:1.0.0` ships every wia-rust-learn envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/wia-rust-learn.sh` ships sample envelope generators with no
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
ecosystem. Wia-rust-learn deployments that follow this layering
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
`/.well-known/wia-wia-rust-learn-capabilities` that enumerates which
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
