# WIA-COMP-013 — Phase 3: Protocol

> Software-testing canonical Phase 3: protocols (execution + mutation + property/fuzz + performance + security + accessibility).

# WIA-COMP-013: Software Testing Specification v1.0

> **Standard ID:** WIA-COMP-013  
> **Version:** 1.0.0  
> **Published:** 2025-12-27



---

## A.1 Test-execution protocol

Test-execution protocols cover: per-suite execution-environment
provisioning envelope (per-OCI image hydration + per-environment
seed-data load + per-environment external-service connection
envelope); per-suite ordering envelope (random per pytest-randomly
+ Junit @Order + Mocha + per-platform-equivalent so inter-test
state-leak is detected); per-suite parallelism envelope (per-
worker isolation per pytest-xdist + per-Junit-fork + per-platform-
equivalent; per-test database-per-worker isolation per
TestContainers + per-platform-equivalent); per-suite retry envelope
(per-test retry-budget per the operator's flake-tolerance policy
— typical 0 retries for new tests; 1 retry for legacy flaky tests
under quarantine investigation); per-suite teardown envelope (per-
environment artefact-collection + per-environment teardown per
the operator's CI policy).

## A.2 Mutation-testing protocol

Mutation-testing protocols per DeMillo + Lipton + Sayward 1978
+ Jia + Harman 2011 cover: per-mutator catalogue (arithmetic-
operator + relational-operator + boundary-condition + return-value
+ negation-conditional + conditional-deletion + per-language
extension catalogue per Pitest mutators + Stryker mutators); per-
mutant generation envelope (per-source-file + per-statement +
per-AST-node mutation per the per-tool envelope); per-mutant
execution envelope (per-test-suite execution against the per-
mutant binary; first-failure stop per the operator's policy);
per-mutant verdict (KILLED / SURVIVED / NO_COVERAGE / TIMED_OUT
per Pitest); per-mutant aggregate (per-package mutation-score +
per-class mutation-score per the operator's coverage policy).

## A.3 Property-and-fuzz-testing protocol

Property-and-fuzz-testing protocols cover: property-based protocol
per Claessen + Hughes 2000 (QuickCheck) + Hypothesis per David
MacIver + per-language-equivalent — operator-author writes
properties as universally-quantified statements + the test
framework explores the input space looking for counter-examples;
shrinking envelope (per-counter-example minimisation to the
smallest reproducer per the per-tool shrink algorithm per Hypothesis
shrinking + QuickCheck shrinking); fuzz-testing protocol per
AFL (American Fuzzy Lop) per Zalewski + libFuzzer per LLVM +
honggfuzz per Robert Swiecki + Atheris per Google for Python +
go-fuzz per Vyukov for Go; per-fuzz coverage-guided exploration
per coverage-instrumentation per per-tool envelope; per-fuzz
corpus envelope (seed-corpus + minimised-corpus per per-tool
minimisation); per-fuzz crash-triage envelope per the operator's
security-response policy.

## A.4 Performance-testing protocol

Performance-testing protocols cover: load-test envelope per
JMeter per Apache + Gatling per gatling.io + k6 per Grafana +
Locust per locust.io + per-platform-equivalent (per-test virtual-
user count + per-test ramp-up profile + per-test sustained duration
+ per-test target throughput); stress-test envelope (per-test
load above expected production capacity to identify breaking
points); soak-test envelope (per-test sustained moderate load
over 24-72 hours to identify memory leaks + resource exhaustion);
spike-test envelope (per-test sudden traffic burst to validate
auto-scaling response); per-test SLO envelope (per-platform p50 +
p95 + p99 + p99.9 latency budget + per-test error-rate budget +
per-test throughput budget per the operator's SLI/SLO policy per
Google SRE Workbook §2-§4).

## A.5 Security-testing protocol

Security-testing protocols cover: SAST envelope per OWASP SAMM
per OWASP + per-tool catalogue (Semgrep per Semgrep + SonarQube
per Sonar + CodeQL per GitHub + Checkmarx + Veracode + Fortify
per OpenText + per-platform-equivalent); DAST envelope per OWASP
ZAP per OWASP + Burp Suite per PortSwigger + per-platform-
equivalent for runtime vulnerability discovery; IAST envelope per
Contrast Security + Hdiv per the operator's IAST envelope; SCA
(Software Composition Analysis) envelope per OWASP Dependency-Check
per OWASP + Snyk + Mend per Mend.io + per-platform-equivalent for
per-dependency CVE catalogue exposure; per-test container-image
scanning envelope per Trivy per Aqua Security + Grype per Anchore
+ per-platform-equivalent; per-test secret-detection envelope per
gitleaks per Zachary Rice + truffleHog per Truffle Security +
per-platform-equivalent.

## A.6 Accessibility-testing protocol

Accessibility-testing protocols cover: per-page automated WCAG
2.2 A/AA validation per axe-core per Deque Systems + Pa11y per
pa11y.org + Lighthouse per Google + per-platform-equivalent;
per-page manual screen-reader validation per JAWS + NVDA +
VoiceOver + TalkBack per the per-platform screen-reader catalogue;
per-page keyboard-only navigation envelope; per-page colour-
contrast envelope per WCAG 2.2 SC 1.4.3 (4.5:1 normal text + 3:1
large text); per-page focus-management envelope per WCAG 2.2 SC
2.4.3 + 2.4.7 + 2.4.11; per-page captioning + audio-description
envelope per WCAG 2.2 SC 1.2.2 + 1.2.5; per-page EN 301 549
conformance envelope (which incorporates WCAG 2.2 + adds ICT-
specific requirements).


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
