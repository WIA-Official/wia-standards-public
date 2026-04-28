# WIA-COMP-013 — Phase 1: Data Format

> Software-testing canonical Phase 1: test-record + test-result + coverage + test-suite + test-data + test-environment envelopes.

# WIA-COMP-013: Software Testing Specification v1.0

> **Standard ID:** WIA-COMP-013  
> **Version:** 1.0.0  
> **Published:** 2025-12-27


## 1. Introduction

This specification defines comprehensive software testing methodologies and best practices.

**弘익人間 (Benefit All Humanity)**



## 2. Testing Types

### 2.1 Unit Testing
- Test individual components
- Mock dependencies
- Fast execution
- High coverage

### 2.2 Integration Testing
- Test component interactions
- Database integration
- API testing
- Service integration

### 2.3 End-to-End Testing
- Full user workflows
- UI automation
- Cross-browser testing
- Performance validation



## 3. Test Automation

### 3.1 Test Frameworks
- Jest, Mocha (JavaScript)
- pytest (Python)
- JUnit (Java)

### 3.2 Coverage Metrics
- Line coverage
- Branch coverage
- Function coverage

**弘익人間 (Benefit All Humanity)**  
*© 2025 SmileStory Inc. / WIA - MIT License*



---

## A.1 Test-record envelope

The Phase 1 envelope groups tests by level (per ISTQB CTFL syllabus
2023 + ISO/IEC/IEEE 29119-1:2022): unit — testing per-component
in isolation; integration — testing component interactions; system
— testing the assembled system end-to-end; acceptance — testing
against business requirements per the operator's user-acceptance
envelope. Each test record carries: test identifier (UUID v7 per
RFC 9562), test-class envelope (functional / non-functional —
performance / load / stress / security / accessibility / usability /
reliability / compatibility per ISO/IEC 25010), preconditions
envelope, test-step envelope (action + expected-result pairs),
post-condition envelope, traceability envelope (linking back to
the per-requirement envelope per the operator's requirements-
management tool — JIRA / Azure DevOps / Polarion / DOORS / ReqIF),
and the per-test audit envelope.

## A.2 Test-result envelope

Test-result envelopes follow JUnit-XML per Maven Surefire +
Ant-JUnitReport schema as the de-facto wire format with extensions:
test identifier (linking back to §A.1); per-result outcome (PASSED
/ FAILED / SKIPPED / ERRORED / FLAKED — per the operator's flake-
classification policy per Google Testing Blog "Flaky Tests at Google");
per-result duration; per-result execution-environment envelope
(per-platform Linux + macOS + Windows + per-OS-version); per-
result assertion-trace envelope (per-failed-assertion file + line
+ message envelope); per-result attached-artefact envelope (screen-
shots per Selenium / Playwright; HAR archives per W3C HAR; HTTP
trace per the operator's recorder); the per-result reproducibility
envelope (per-result seed + per-result environment-snapshot for
deterministic reproduction).

## A.3 Coverage-record envelope

Coverage-record envelopes carry per-build coverage data: per-line
coverage per gcov / lcov / Istanbul / JaCoCo per per-language
profiler; per-branch coverage; per-condition coverage; per-MC/DC
(Modified Condition / Decision Coverage) per RTCA DO-178C §A.6
Table A-7 for safety-critical code; per-mutation coverage per
PIT (Pitest) per pitest.org + Stryker per stryker-mutator.io for
mutation-testing-effectiveness measurement; per-coverage threshold
envelope (operator-policy — typical 80% line + 70% branch for non-
critical code; 100% MC/DC for DAL-A avionics + ASIL D automotive
software); per-coverage diff envelope (per-PR delta vs base-branch);
per-coverage report-format envelope (Cobertura XML + JaCoCo XML +
LCOV + JSON per the operator's reporting envelope).

## A.4 Test-suite-record envelope

Test-suite-record envelopes carry per-suite metadata: suite-class
envelope (smoke / sanity / regression / nightly / weekly / release-
candidate); per-suite execution-time budget envelope; per-suite
parallelism envelope (per-test-class + per-test-instance + per-
machine sharding policy); per-suite flake-budget envelope (per-
suite maximum-flake-rate threshold per the operator's quality
policy — typical 1% for nightly suites; 0.1% for blocking suites);
per-suite ordering envelope (random per Hyrum Wright + Junit @Order
+ pytest-randomly + per-platform-equivalent for inter-test
independence verification); per-suite hermeticity envelope (per-
suite isolation policy: shared-database vs per-test-database per
TestContainers per testcontainers.org + per-platform-equivalent).

## A.5 Test-data-record envelope

Test-data-record envelopes catalogue per-test data: synthetic-data
envelope (per-Faker per stympy/faker + per-language Faker +
per-platform Synthetic Data Vault per sdv.dev for tabular data +
per-platform GAN-based synthesis per CTGAN per sdv.dev); fixture-
based envelope (per-test fixtures per pytest fixtures + per-Jest
fixtures + per-RSpec let + per-platform-equivalent); production-
mirror-snapshot envelope (per-snapshot anonymisation per Mostly
AI + Tonic.ai + per-platform-equivalent; per-snapshot scrubbing
envelope per the operator's data-classification policy per HIPAA
Safe Harbor + GDPR per Article 26 pseudonymisation); per-test
data-provenance envelope (per-fixture content-hash + per-fixture
update-trail envelope per the operator's data-versioning policy
per DVC + LakeFS + per-platform-equivalent).

## A.6 Test-environment envelope

Test-environment envelopes carry per-environment specification:
per-environment platform envelope (Linux per kernel-version per
the operator's per-environment kernel-image envelope; macOS per
Xcode-version per Apple Developer; Windows per Server / 11 per
Microsoft per per-edition envelope; per-platform mobile-device
envelope per per-OS-version + per-device-class); per-environment
runtime envelope (per-language interpreter + per-language compiler
version + per-language standard-library version); per-environment
external-service envelope (per-database version + per-cache version
+ per-message-broker version per the operator's environment-
matrix); per-environment containerisation envelope (per-platform
OCI image per OCI Image Spec for reproducible per-environment
provisioning); per-environment ephemeral envelope (per-test-run
provision + per-test-run teardown per the operator's CI policy).


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
