# WIA-COMP-017 — Phase 1: Data Format

> Software-documentation canonical Phase 1: document + markup + API-doc + architecture + docs-as-code + localisation envelopes.

# WIA-COMP-017: Software Documentation Specification v1.0

> **Standard ID:** WIA-COMP-017  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active


## 1. Introduction

This specification defines standards for software documentation including API docs, user guides, technical specifications, and automated documentation generation.

**弘益인간 (Benefit All Humanity)** - Good documentation enables knowledge sharing and accelerates software development.



## 2. Documentation Types

### 2.1 API Documentation
- Auto-generated from code comments
- JSDoc, TSDoc, Javadoc, XMLDoc
- Interactive API explorers (Swagger/OpenAPI)

### 2.2 User Documentation
- Installation guides
- Usage tutorials
- FAQs and troubleshooting

### 2.3 Technical Documentation
- Architecture diagrams
- System design documents
- Database schemas



## 3. Documentation Standards

### 3.1 README Structure
```markdown
# Project Name
## Overview
## Installation
## Usage
## API Reference
## Contributing
## License
```

### 3.2 Code Comments
- All public APIs must be documented
- Use standard documentation formats
- Include examples where appropriate




---

## A.1 Document-record envelope

The Phase 1 envelope groups documentation artefacts by purpose
(per the Diátaxis framework per Procida 2024: tutorial — learning-
oriented; how-to — task-oriented; reference — information-oriented;
explanation — understanding-oriented) with the per-artefact
canonical fields: document identifier (UUID v7 per RFC 9562 with
content-hash extension), product + version reference, audience-class
envelope (developer / operator / end-user / regulator / auditor),
authorship envelope (per-author identifier + per-author affiliation
+ per-author contribution-class per CRediT taxonomy per CASRAI
contributor-roles), language-class envelope (BCP 47 language tag
per RFC 5646), and the per-artefact licence envelope (CC BY-SA /
CC BY / proprietary per the operator's IP policy).

## A.2 Markup-format catalogue

Markup-format envelopes catalogue the per-document format: Markdown
per CommonMark 0.31.2 + GitHub-Flavored Markdown per GFM Spec +
MyST-Markdown per Executable Books Project; reStructuredText per
Docutils; AsciiDoc per AsciiDoc Language Specification per OASIS
+ Eclipse AsciiDoc.org; DocBook per OASIS DocBook 5.2; DITA per
OASIS DITA 2.0 (Darwin Information Typing Architecture) — topic-
oriented + map-oriented + reusable conrefs; LaTeX per LaTeX
Project per the IEEE / ACM / Elsevier camera-ready templates;
XML / TEI per Text Encoding Initiative for scholarly + heritage
encoding; per-format extension catalogue (admonitions per MyST;
table syntax per CommonMark + GFM + AsciiDoc; cross-reference
syntax per AsciiDoc + DITA + RST).

## A.3 API-documentation envelope

API-documentation envelopes carry: source-comment-extraction
envelope (per JSDoc per Use JSDoc + TSDoc per microsoft/tsdoc;
Javadoc per Oracle Javadoc tool; XMLDoc per Microsoft .NET XML
Documentation Comments; Doxygen per Doxygen project + Doxygen
Awesome theme; Sphinx-autodoc per Sphinx + Python docstring per
PEP 257 + PEP 287 reST + Google + NumPyDoc styles; rustdoc per
Rust Reference; godoc per Go); API-spec envelope (OpenAPI 3.1.0
per OpenAPI Initiative for HTTP APIs; AsyncAPI 3.0 per AsyncAPI
Initiative for event-driven APIs; gRPC + protobuf per Google
+ proto3 spec; GraphQL Schema per GraphQL Specification per
GraphQL Foundation); per-spec example-validation envelope;
per-spec change-classification envelope (breaking / non-breaking /
internal-only per the operator's API-versioning policy).

## A.4 Architecture-document envelope

Architecture-document envelopes follow the C4 model per Brown 2018
+ ISO/IEC/IEEE 42010 + IEEE Std 1471: per-system context diagram
(actors + systems + external dependencies); container diagram
(per-deployment-unit logical structure); component diagram (per-
container internal components); code diagram (per-component class
+ module structure); arc42 sections per arc42 template per Hruschka
+ Starke (introduction + constraints + context + solution-strategy
+ building-blocks + runtime + deployment + concepts + decisions +
quality + risks + glossary); ADR (Architecture Decision Record)
per Nygard 2011 + adr-tools per Henderson + Cabrera with the per-
ADR status envelope (Proposed / Accepted / Deprecated / Superseded).

## A.5 Documentation-as-code envelope

Documentation-as-code envelopes carry: source-of-record placement
(per-repo `docs/` directory co-located with code; per-monorepo
`packages/*/docs/` per Lerna + Nx; per-multi-repo central docs-repo
with per-product subtree); per-build static-site-generator envelope
(MkDocs per MkDocs Project + Material for MkDocs per Squidfunk;
Sphinx per Sphinx Doc; Docusaurus per Meta Open Source; Astro
Starlight per Astro project; Hugo per Hugo project; Jekyll per
Jekyll project; Antora per Antora project for AsciiDoc; Vitepress
per Vue project); per-document review envelope (per-PR docs review
required if `/docs` paths changed per CODEOWNERS); per-document
versioning envelope (per-release docs branch + tag per the
operator's release policy).

## A.6 Localisation-record envelope

Localisation-record envelopes carry per-document language variants:
per-language source-file mapping (per-locale subdirectory per
gettext convention; per-locale prefix per i18next convention);
per-language translation-state envelope (machine-translated +
human-reviewed + community-translated + author-translated per the
operator's quality-policy); per-language translation-coverage
envelope (per-segment translation-status: NEW / FUZZY / TRANSLATED
+ APPROVED per Pootle + Weblate + Crowdin + Transifex + per-
platform-equivalent); per-language glossary envelope (term-
consistency database per locale per the operator's terminology
policy); per-language right-to-left + bi-directional envelope per
Unicode UAX 9 for languages such as Arabic + Hebrew + Persian +
Yiddish.


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
