# WIA-COMP-017 — Phase 3: Protocol

> Software-documentation canonical Phase 3: protocols (publishing + api-doc-generation + localisation + versioning + search + quality-gate).

# WIA-COMP-017: Software Documentation Specification v1.0

> **Standard ID:** WIA-COMP-017  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active



---

## A.1 Documentation-publishing protocol

Documentation-publishing protocols cover: per-project source-
revision integration per Git per Git Reference per Git Project;
per-revision build envelope (per-platform static-site builder per
§A.3); per-build artefact-integrity envelope (per-output SHA256
per FIPS 180-4 over the per-build static-output bundle); per-build
link-rot detection per linkchecker / lychee / per-platform-
equivalent (HTTP HEAD/GET per RFC 9110 with the operator's per-
target rate-limit envelope); per-build accessibility-validation
envelope per axe-core + per-platform-equivalent against WCAG 2.2
A/AA + EN 301 549; per-build SEO-validation envelope (per-page
title + meta-description + canonical-link + structured-data per
Schema.org); per-build deploy envelope (per-target CDN deploy per
the operator's CDN envelope: CloudFlare + Fastly + Akamai + AWS
CloudFront + Azure Front Door + GCP Cloud CDN); per-build cache-
invalidation envelope per the operator's CDN policy.

## A.2 API-documentation generation protocol

API-documentation generation protocols cover: per-source extraction
envelope (per-language tooling per §A.3 of Phase 1); per-spec
materialisation envelope (OpenAPI 3.1 per OpenAPI Initiative spec
+ per-tool generator per swagger-codegen + openapi-generator +
oapi-codegen per the per-language stack); per-spec UI rendering
envelope (Swagger UI per SmartBear + Redoc per Redocly + Stoplight
Elements per Stoplight + Scalar per Scalar.com + per-platform-
equivalent); per-spec test-harness envelope (Schemathesis per
Schemathesis + Dredd per Apiary + per-platform-equivalent for
contract-testing per Phase 1 §A.3 of WIA Software Testing); per-
spec breaking-change detection envelope per oasdiff per Tufin +
openapi-diff + per-platform-equivalent; per-spec example-validation
envelope (per-example schema-conformance + per-example end-to-end
execution where the operator permits).

## A.3 Localisation-workflow protocol

Localisation-workflow protocols cover: per-source extraction
envelope (per-format extractor: gettext for `.po` files per GNU
gettext; Crowdin / Lokalise / Phrase / Transifex / Weblate
file-format support; per-i18next JSON / per-Polyglot YAML / per-
ICU MessageFormat 2.0 envelope); per-locale translation-job
envelope (per-vendor + per-internal-translator queue per the
operator's translation-vendor envelope); per-locale machine-
translation envelope per the operator's MT vendor (DeepL / Google
+ Microsoft / Amazon Translate + per-platform LLM envelope per
the operator's quality + privacy policy); per-locale review
envelope (per-segment translator + reviewer + approver chain);
per-locale terminology-consistency envelope per the per-locale
glossary; per-locale RTL/bidi rendering verification per Unicode
UAX 9 + Bidirectional Algorithm.

## A.4 Versioning-and-deprecation protocol

Versioning-and-deprecation protocols cover: per-document version
envelope (per-product version with per-document tracked-against
the product's release envelope; per-document independent-version
envelope where the documentation evolves at a different cadence
than the product); per-document deprecation envelope (per-document
sunset-date per RFC 8594 Sunset header equivalent; per-document
replacement-pointer envelope where the deprecated document points
to the successor; per-document banner envelope so a reader sees the
deprecation status before consuming the content); per-API-spec
breaking-change envelope (per-major-version branch + per-major-
version sunset window per the operator's API-versioning policy);
per-document archive envelope (per-archived-document content-
addressable URL per the operator's permanent-URL policy).

## A.5 Search-and-discovery protocol

Search-and-discovery protocols cover: per-site search-engine
envelope (Algolia DocSearch per Algolia + per-platform Lunr.js +
per-platform-equivalent client-side index; per-platform Elasticsearch
+ Meilisearch + Typesense backend index); per-page Schema.org
structured-data envelope (TechArticle / Article / FAQPage /
HowTo / SoftwareApplication per Schema.org); per-page sitemap
envelope per Sitemaps XML 0.9; per-page robots-directive envelope
(per-page indexing policy; per-page canonical URL; per-page meta-
description + open-graph per the operator's SEO policy); per-
search-result quality-feedback envelope (per-result click-tracking
+ per-result search-quality scoring per the operator's analytics
envelope subject to the privacy envelope per Phase 4 §Z.8).

## A.6 Quality-gate protocol

Quality-gate protocols cover: per-PR documentation-review envelope
(per-PR CODEOWNERS-trigger per GitHub CODEOWNERS + per-platform-
equivalent so docs changes route to documentation reviewers); per-
PR style-consistency envelope per Vale per Errata + per-tool-
linter (markdownlint + RST-lint + AsciiDoc-lint); per-PR spell-
check envelope per cspell + hunspell + per-tool-equivalent; per-
PR link-validity envelope; per-PR build-success envelope; per-PR
preview-deploy envelope (per-PR Netlify Preview + Vercel Preview +
GitHub Pages Preview + per-platform-equivalent); per-release
documentation-completeness envelope (per-API endpoint coverage
required + per-CLI command coverage required per the operator's
quality-policy).


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
