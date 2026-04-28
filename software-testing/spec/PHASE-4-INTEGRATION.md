# WIA-COMP-013 — Phase 4: Integration

> Software-testing canonical Phase 4: ecosystem integration (ISO/IEC/IEEE 29119 + ISTQB + DO-178C/ISO 26262/IEC 61508/62304 + OWASP + WCAG + CI/CD).

# WIA-COMP-013: Software Testing Specification v1.0

> **Standard ID:** WIA-COMP-013  
> **Version:** 1.0.0  
> **Published:** 2025-12-27



---

## A.1 Standards cross-walk

| Concern                              | Standard                                     |
|--------------------------------------|----------------------------------------------|
| Software testing process             | ISO/IEC/IEEE 29119-1..5:2022                  |
| Test certifications                  | ISTQB CTFL + CTAL syllabi                     |
| Software quality model               | ISO/IEC 25010 + ISO/IEC 25023                 |
| Avionics test (DAL A/B/C/D)          | RTCA DO-178C + DO-330 + DO-331                |
| Automotive test (ASIL A/B/C/D)       | ISO 26262-6 + ISO 26262-8                     |
| Functional safety test               | IEC 61508-1 + IEC 61508-3 §7.4-7.9            |
| Medical-device software test         | IEC 62304 + ISO 14971                         |
| Application security verification    | OWASP ASVS 4.0 + OWASP MASVS                  |
| Web accessibility                    | WCAG 2.2 + EN 301 549 + ADA Title III         |
| Web app security                     | OWASP Top 10 + OWASP Testing Guide v4.2        |
| Mobile app security                  | OWASP Mobile Top 10 + OWASP MSTG               |
| API security                         | OWASP API Security Top 10                     |
| TLS testing                          | RFC 8446 + RFC 9325 + Mozilla SSL Config       |
| Test report exchange                 | JUnit XML + xUnit + TAP + Allure              |
| Coverage report exchange             | Cobertura XML + JaCoCo XML + LCOV              |
| Mutation testing                     | Pitest + Stryker + per-language-equivalent     |
| Fuzz testing                         | AFL + libFuzzer + honggfuzz + Atheris + go-fuzz|

## A.2 ISO/IEC/IEEE 29119 integration envelope

ISO/IEC/IEEE 29119 integration covers: 29119-1 (concepts +
definitions); 29119-2 (test processes — organisational + management
+ dynamic test); 29119-3 (test documentation — test plan + test
specification + test procedure + test report); 29119-4 (test
techniques — equivalence partitioning + boundary value + decision
table + state transition + use case + scenario per §5; structure-
based — statement + branch + condition + MC/DC + path coverage
per §6; experience-based — error guessing + checklist + exploratory
per §7); 29119-5 (keyword-driven test); the operator's per-product
compliance envelope mapping the per-document per-process per-
project to the per-stakeholder reporting envelope.

## A.3 Safety-critical-test integration envelope

Safety-critical-test integration covers: avionics DO-178C objectives
table per Annex A — per-DAL test-coverage objective (DAL A: 71
objectives + structural-coverage to MC/DC; DAL B: 69 objectives
+ structural-coverage to decision; DAL C: 62 objectives + statement
coverage; DAL D: 26 objectives); automotive ISO 26262-6 software-
unit verification + integration test + safety analysis at the per-
ASIL envelope; medical-device IEC 62304 Class A/B/C software
verification + validation envelope; industrial IEC 61508-3 §7.4.4
+ §7.4.7 verification techniques per the per-SIL envelope (Table
A.5 functional-and-black-box test + Table B.2 dynamic analysis +
Table B.3 functional + Table B.7 modelling + Table B.8 performance);
the per-domain per-objective traceability envelope feeding into
the safety case.

## A.4 Continuous-integration / continuous-delivery integration envelope

CI/CD integration covers: per-platform CI envelope (GitHub Actions
per github.com/features/actions + GitLab CI per gitlab.com + Jenkins
per jenkins.io + CircleCI per circleci.com + Buildkite per
buildkite.com + Travis CI legacy + Azure DevOps Pipelines + AWS
CodePipeline + Argo Workflows per CNCF Argo + Tekton per CNCF
Tekton); per-platform pipeline-as-code envelope (per-YAML per
GitHub Actions / GitLab CI / Azure DevOps; per-Jenkinsfile per
Jenkins; per-Argo-Workflow / Tekton-Pipeline per CNCF projects);
per-platform per-stage gating envelope (test-pass + coverage-
threshold + security-clearance + compliance-clearance per the
operator's quality gate); per-platform per-environment promotion
envelope (dev → staging → canary → prod per the operator's
release-management policy); per-platform per-rollback envelope
(per-rollback automated trigger + per-rollback manual trigger per
the operator's incident-response policy).

## A.5 Quality-records integration envelope

Quality-records integration covers: per-product test-summary report
envelope (per-release verification + validation report per the
operator's release-management policy); per-product traceability
envelope (per-requirement → per-test → per-result trace per the
operator's requirements-traceability tool — JIRA + DOORS + Polarion
+ ReqIF + per-platform-equivalent); per-product retention envelope
(typical 10 years for safety-critical + 7 years for general
software per ISO 27001 + per-jurisdiction retention policy); per-
product audit-readiness envelope (per-product audit-trail surfacing
per the operator's audit policy per ISO 19011); per-product
compliance-attestation envelope (per-product SOC 2 + ISO 27001 +
per-jurisdiction certification per the operator's compliance
roadmap).

## A.6 References

- ISO/IEC/IEEE 29119-1..5:2022: Software testing
- ISO/IEC 25010: Quality model
- ISO/IEC 25023: Quality measurement
- ISTQB CTFL Syllabus 2023 + CTAL Syllabi (Test Manager, Test Analyst, Technical Test Analyst)
- RTCA DO-178C: Software Considerations in Airborne Systems
- ISO 26262-6: Product development at the software level
- ISO 26262-8: Supporting processes
- IEC 61508-1 + IEC 61508-3: Functional safety + software
- IEC 62304: Medical device software
- ISO 14971: Risk management for medical devices
- OWASP ASVS 4.0: Application Security Verification Standard
- OWASP Testing Guide v4.2: Web Security Testing Guide
- OWASP MSTG: Mobile Security Testing Guide
- OWASP Top 10 (2021)
- OWASP API Security Top 10 (2023)
- WCAG 2.2: Web Content Accessibility Guidelines
- EN 301 549: Accessibility requirements for ICT products
- RFC 8446: TLS 1.3
- RFC 9325: Recommendations for Secure Use of TLS + DTLS
- DeMillo + Lipton + Sayward 1978: Hints on Test Data Selection
- Jia + Harman 2011: An Analysis and Survey of the Development of Mutation Testing
- Claessen + Hughes 2000: QuickCheck — A Lightweight Tool for Random Testing
- Hyrum's Law: With a sufficient number of users of an API, all observable behaviours will be depended on
- Google SRE Workbook + Site Reliability Engineering Book


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
