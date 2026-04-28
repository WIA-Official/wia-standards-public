# WIA-COMP-017 — Phase 4: Integration

> Software-documentation canonical Phase 4: ecosystem integration (ISO 26500 series + developer-portal + i18n/a11y + compliance + references).

# WIA-COMP-017: Software Documentation Specification v1.0

> **Standard ID:** WIA-COMP-017  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active



---

## A.1 Standards cross-walk

| Concern                                | Standard                                     |
|----------------------------------------|----------------------------------------------|
| Architecture description               | ISO/IEC/IEEE 42010 + IEEE Std 1471            |
| Software user documentation            | ISO/IEC/IEEE 26511 + 26512 + 26513 + 26514    |
| Software documentation management      | ISO/IEC/IEEE 26515 (Agile)                    |
| Markdown                               | CommonMark 0.31.2 + GitHub-Flavored Markdown  |
| AsciiDoc                               | AsciiDoc Language Spec (OASIS draft)          |
| DocBook                                | OASIS DocBook 5.2                             |
| DITA                                   | OASIS DITA 2.0                                |
| OpenAPI                                | OpenAPI 3.1.0                                 |
| AsyncAPI                               | AsyncAPI 3.0                                  |
| gRPC + protobuf                        | gRPC + protobuf v3                            |
| GraphQL                                | GraphQL Specification (GraphQL Foundation)    |
| JSDoc / TSDoc / Javadoc / XMLDoc       | per-language tool docs                        |
| Sphinx + Python docstring (PEP 257/287)| python.org PEPs + Sphinx doc                  |
| Diátaxis framework                     | Procida 2024 (CC BY-SA)                       |
| C4 model                               | Brown 2018 (CC BY-SA)                         |
| arc42                                  | Hruschka + Starke arc42 template              |
| ADR (Architecture Decision Record)     | Nygard 2011 + adr-tools                       |
| Accessibility                          | WCAG 2.2 + EN 301 549 + ADA Title III         |
| Translation file formats               | GNU gettext PO + ICU MessageFormat 2.0        |
| Localisation Unicode bidi              | Unicode UAX 9                                 |
| Sitemaps                               | Sitemaps XML 0.9 + RFC 9309 (robots.txt)      |
| Open Graph + structured data           | Open Graph Protocol + Schema.org              |

## A.2 ISO/IEC/IEEE-26500-series integration envelope

ISO/IEC/IEEE 26500-series integration covers: ISO/IEC/IEEE 26511
(processes for managing user documentation); ISO/IEC/IEEE 26512
(requirements for acquirers + suppliers of user documentation);
ISO/IEC/IEEE 26513 (requirements for testers + reviewers of user
documentation); ISO/IEC/IEEE 26514 (designing + developing user
documentation); ISO/IEC/IEEE 26515 (developing user documentation
in an agile environment); ISO/IEC 25010 quality model for the per-
document quality envelope (functional suitability + performance
efficiency + compatibility + usability + reliability + security +
maintainability + portability per ISO/IEC 25010 §4); the operator's
per-product compliance envelope.

## A.3 Developer-portal integration envelope

Developer-portal integration covers: per-portal platform envelope
(Backstage TechDocs per CNCF Backstage; Spotify-flavoured TechDocs
+ TechRadar; ReadMe per ReadMe.io; Mintlify per Mintlify; per-
platform docs-portal envelope); per-portal API-catalogue envelope
(per-team + per-product + per-API listing with the operator's
per-API ownership envelope); per-portal versioning envelope (per-
API + per-product version dropdown + per-version sunset envelope);
per-portal change-feed envelope (per-product change-history +
per-product release-notes per the operator's release-management
policy); per-portal search envelope (per-platform-equivalent of
§A.5 of Phase 3).

## A.4 Internationalisation-and-accessibility integration envelope

Internationalisation-and-accessibility integration covers: per-
locale BCP 47 + ISO 639-3 + ISO 15924 envelope per the operator's
per-locale roadmap; per-locale Unicode CLDR per CLDR Release per
Unicode Consortium for date / number / currency / unit formatting;
per-locale RTL/bidi rendering per Unicode UAX 9 + per-platform
CSS logical-property envelope per CSS Logical Properties Level 1
per W3C; per-locale accessibility envelope (per-locale screen-
reader compatibility per JAWS + NVDA + VoiceOver + TalkBack +
ChromeVox); per-document WCAG 2.2 A/AA conformance envelope per
WCAG 2.2 + ARIA 1.3 + per-locale HTML5 lang-attribute correctness;
per-locale plain-language envelope per the per-jurisdiction plain-
language regulation (US Plain Writing Act 2010; EU Plain Language
guidance; per-locale-equivalent).

## A.5 Compliance-and-records integration envelope

Compliance-and-records integration covers: per-document content-
classification envelope (public / internal / confidential / per-
regulatory per the operator's classification policy); per-document
retention-policy envelope per the operator's records-management
schedule (typically 10 years for software design records per
ISO 27001 + the operator's per-jurisdiction retention envelope);
per-document e-signature envelope per the operator's QES (Qualified
Electronic Signature per eIDAS for EU) / advanced-electronic-
signature envelope; per-document watermark + provenance envelope
per the operator's IP policy; per-document export-control envelope
per ITAR + EAR + EU Dual-Use 2021/821 + Wassenaar Arrangement +
per-jurisdiction-equivalent.

## A.6 References

- ISO/IEC/IEEE 42010: Architecture description
- ISO/IEC/IEEE 26511 / 26512 / 26513 / 26514 / 26515: User documentation
- ISO/IEC 25010: Quality model
- ISO/IEC 25023: Quality measurement
- CommonMark 0.31.2: Markdown specification
- GitHub Flavored Markdown Specification
- AsciiDoc Language Specification: OASIS draft
- OASIS DocBook 5.2 + OASIS DITA 2.0
- OpenAPI 3.1.0: OpenAPI Initiative
- AsyncAPI 3.0: AsyncAPI Initiative
- gRPC + protobuf v3: Google + protobuf project
- GraphQL Specification: GraphQL Foundation
- WCAG 2.2: Web Content Accessibility Guidelines
- EN 301 549: Accessibility requirements for ICT products
- ARIA 1.3: Accessible Rich Internet Applications
- Schema.org: Structured-data vocabulary
- Open Graph Protocol: Open Graph Project
- Unicode UAX 9: Bidirectional Algorithm
- Unicode CLDR: Common Locale Data Repository
- BCP 47 (RFC 5646): Tags for Identifying Languages
- ISO 639-3 + ISO 15924: Language + script codes
- RFC 9309: Robots Exclusion Protocol
- Diátaxis framework (Procida 2024)
- C4 model (Brown 2018)
- arc42 (Hruschka + Starke)
- ADR — Architecture Decision Records (Nygard 2011)
- US Plain Writing Act 2010 (Public Law 111-274)
- eIDAS Regulation (EU) No 910/2014: Trust services + electronic signatures


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
