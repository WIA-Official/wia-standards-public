# WIA-COMP-017 — Phase 2: API Interface

> Software-documentation canonical Phase 2: API surface (projects + builds + api-specs + translations + telemetry + audit).

# WIA-COMP-017: Software Documentation Specification v1.0

> **Standard ID:** WIA-COMP-017  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active


## 4. Implementation Guidelines

### 4.1 Tools
- Documentation generators: JSDoc, TypeDoc, Sphinx
- Site generators: MkDocs, Docusaurus, GitBook
- Diagram tools: Mermaid, PlantUML

### 4.2 Versioning
- Documentation versioned with code
- Maintain docs for all supported versions
- Clear migration guides between versions

---

**弘益인간 (Benefit All Humanity)**  
*© 2025 SmileStory Inc. / WIA - MIT License*



---

## A.1 Documentation-build API surface

```http
POST /software-documentation/v1/projects                # register project
GET  /software-documentation/v1/projects/{id}           # fetch project
POST /software-documentation/v1/builds                  # trigger build
GET  /software-documentation/v1/builds/{id}             # fetch build state
POST /software-documentation/v1/api-specs               # publish API spec
GET  /software-documentation/v1/api-specs/{id}          # fetch API spec
POST /software-documentation/v1/translations            # publish translation
GET  /software-documentation/v1/translations/{id}       # fetch translation
WS   /software-documentation/v1/state/stream            # build stream
GET  /software-documentation/v1/audit/{id}              # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-software-documentation`. Project-mutation
endpoints require the requester's project-administrator credential
plus the per-project authorship envelope. Translation-publish
endpoints additionally require the per-locale translator-credential
or the per-locale operator-approval envelope.

## A.2 Project-registration API

`POST /projects` accepts the Phase 1 §A.1 envelope. The endpoint
validates the markup-format envelope (a project declaring an
unsupported format is accepted with a non-fatal warning), validates
the audience-class envelope, allocates the per-project static-site
output URL, and emits the registration event. Project state
transitions through `provisioning`, `active`, `archived`, `removed`;
transitions emit audit events that survive project deletion.

## A.3 Build API

`POST /builds` accepts a build-trigger envelope: project reference,
source-revision (Git SHA per Git per the project's source-control
envelope), build-mode envelope (full / incremental / per-locale-only
/ api-spec-only); per-build configuration override envelope. The
endpoint dispatches the build to the per-platform builder (per
MkDocs / Sphinx / Docusaurus / Astro per the project's choice),
validates the produced output (per-page link-rot detection per
linkchecker + lychee + per-platform-equivalent), and emits the
build-success or build-failure event with the per-failure structured
diagnostics envelope.

## A.4 API-spec API

`POST /api-specs` accepts an API-spec envelope: spec-format (OpenAPI
3.1 / AsyncAPI 3.0 / gRPC / GraphQL); spec-content payload; per-
spec example-validation envelope (per-OpenAPI per Spectral per
Stoplight + per-OpenAPI lint per Redocly per Redocly OpenAPI
toolset + per-OpenAPI breaking-change detection per oasdiff per
Tufin); per-spec change-classification per the operator's API-
versioning policy. The endpoint stores the per-spec history with
per-change classification and emits the change-event downstream so
API-consumer-side notification per the operator's communication
policy fires.

## A.5 Translation API

`POST /translations` accepts a per-locale translation-publish
envelope: source-document reference, target-locale envelope (BCP
47 per RFC 5646), translation-payload, translation-state envelope
per §A.6, per-segment translator-attribution envelope, per-segment
review-state envelope. The endpoint validates the per-locale
glossary envelope, validates the per-locale RTL/bidi envelope per
Unicode UAX 9, computes the per-locale translation-coverage
percentage, and emits the translation-publish event.

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-project events: per-
build start + progress + completion events with per-stage timing
envelope; per-build link-rot events; per-build accessibility-rule
violation events per WCAG 2.2 + EN 301 549; per-build broken-anchor
events; per-locale translation-coverage events; per-API-spec
breaking-change events; per-API-spec example-validation events.
Subscribers can filter by project-id, build-id, locale, severity-
class. Rate limits: 5000 req/h authenticated; 50000 req/h trusted-
partner. WebSocket subscriptions are bounded at 100 simultaneous
per credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: project
registration, every build event, every API-spec publish event with
per-change classification, every translation-publish event with
per-locale state, every credential change, every author-attribution
change, every retirement event. The audit-trail integrity is
anchored into a Merkle tree per-project; the root is committed
to the operator's archival record per the operator's documentation-
retention policy (typically 10 years for regulated software per
ISO 27001 + per-jurisdiction policy).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/software-documentation/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-software-documentation-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/software-documentation-host:1.0.0` ships every software-documentation envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/software-documentation.sh` ships sample envelope generators with no
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
ecosystem. Software-documentation deployments that follow this layering
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
`/.well-known/wia-software-documentation-capabilities` that enumerates which
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
