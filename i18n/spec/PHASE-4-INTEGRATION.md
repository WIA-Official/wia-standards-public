# WIA-i18n PHASE 4 — INTEGRATION Specification

**Standard:** WIA-i18n
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited i18n / l10n programme
integrates with the systems that surround it: software-publisher
build pipelines (CI/CD); language-service-vendor management
systems; CAT tooling that translators use; translation-memory and
terminology repositories; quality-assurance services; long-term
archives that hold released build bundles; and the certifying
bodies that audit i18n programmes.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 17100:2015 (translation services — requirements)
- ISO 8601 (date and time)
- BCP 47 (language tags)
- W3C XLIFF 2.x (interchange envelope reference)
- Unicode CLDR

---

## §1 Build Pipeline Integration

Software-publisher build pipelines consume approved translations
through pull-based polling or streaming subscriptions (PHASE-2
§13). The pipeline pins a build-bundle manifest digest at each
release so that the deployed binaries reference a verifiable
translation set.

Pipelines that fail bundle integrity verification halt the build
with a Problem-Details (RFC 9457) response of type
`urn:wia:i18n:bundle-integrity-failure`.

## §2 LSV Management System Integration

LSV management systems integrate via dispatch and acceptance
endpoints. The integration carries the LSV's accreditation reference,
the project scope, the per-language-pair pricing terms, and the
SLA window for delivery. LSV-side translator records are held in
the LSV's HR system; the API never carries individual translator
identifiers.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  project.json                 — project record
  locale-data/                 — pinned CLDR revisions per locale
  segments/                    — source segment summaries (full
                                  bodies referenced by content-
                                  address)
  translations/                — translation summaries per locale
  glossary/                    — current glossary state
  tm/                          — TM index summary (full TM held in
                                  the TM server)
  quality-reviews/             — review records
  build-bundles/               — bundle records and manifest digests
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by the
operating programme and counter-signed by the LSV when the package
is consumed for SLA verification.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:i18n:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-i18n` that links to the API root, the public
LSV accreditation references, the published quality dossier, the
CLDR pinning currently in force, and the catalogue of released
build bundles.

## §6 CAT Tooling Integration

Computer-assisted translation tools (CAT systems used by individual
translators) integrate via the LSV's proxy, not directly with the
API. The LSV's proxy translates between the CAT tool's native
format (typically XLIFF 2.x or vendor-specific equivalents) and
the API's segment / translation records. Direct CAT-tool access
is permitted only when the CAT tool's operator carries an LSV
accreditation.

## §7 Quality-Assurance Service Integration

Independent QA services consume completed translations and emit
review records (PHASE-2 §8). The QA service's client certificate
is bound to the QA service's accreditation reference; reviews from
non-accredited services are accepted into the project but flagged
as `non-accredited-review` so that downstream consumers can weigh
them appropriately.

## §8 TM and Terminology Repository Integration

Cross-project TM sharing (within or across operators) integrates
through repository-level federation. The integration carries the
participating operators' identifiers, the data-sharing agreement
content-address, and the per-segment access scope. Federation is
mutually authenticated; TM cross-flow without an active agreement
returns `403 Forbidden` with type
`urn:wia:i18n:tm-federation-not-authorised`.

## §9 Long-Term Archive Integration

Programmes designate a long-term archive that holds released build
bundles beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on programme wind-down, remaining bundles
transfer to the archive with content-addresses preserved.

## §10 Citation and Pinning

Externally cited translations (a release, a journal-translated
abstract, a regulatory-translated dossier) are referenced by the
build-bundle manifest digest. Programmes MUST keep build bundles
addressable by their pinned digests for at least seven years from
the citation event.

## §11 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose translation attestations (quality
review pass, ISO 17100 conformance) to consumers of W3C Verifiable
Credentials MAY re-issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance is optional;
the canonical record remains the JSON evidence-package manifest.

## §12 Worked Example: Continuous-Localisation Round-Trip

1. Build pipeline POSTs new source segments at extract time.
2. Streaming subscription notifies the LSV; LSV dispatches to a
   qualified translator via its CAT tool integration.
3. Translator completes translations; LSV reviewer adds quality
   review; operating programme appends approval.
4. Build pipeline pulls approved translations and publishes a new
   build bundle; the bundle's manifest digest is pinned in the
   release notes.
5. Translation memory federation propagates the new TM entries to
   sister projects under the same operator.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (publication,
clinical-document translation, regulatory-document translation)
emit cross-standard linkage records.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual
language-coverage maps, glossary-evolution timelines, quality-
review distributions) alongside the canonical evidence package.
Reader tools are non-normative.

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full CLDR release cycle.

## §16 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue emit an Atom or JSON
Feed listing released build bundles with their evidence-package
manifest digests, the project, the release date, and the locales
covered. The feed does not carry translator identifiers; it is
intended for engineering discovery and audit.

## §17 Migration from Pre-Standard Records

Programmes that operated before WIA-i18n reached version 1.0 MAY
migrate historical projects by emitting synthetic project records
with a `legacyImport` flag. Synthetic projects are accepted by the
public catalogue but are not eligible for evidence-package
generation without contemporaneous re-validation under PHASE-3
§3.

## §18 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (publication
workflows, clinical-document translation, regulatory-document
translation) emit cross-standard linkage records that name the
consuming standard and the version under which the linkage is
claimed. Linkages are not transitive; consumers verify each
adjacent standard's evidence directly.

## §19 LSV-Side Audit-Trail Export

LSVs that are subject to audit (annual ISO 17100 surveillance,
operator-side QA review) consume audit-trail exports through
dedicated client certificates. The export carries the dispatch
records, the per-segment translation timeline, the quality-review
chain, and the build-bundle integration events relevant to the
LSV. Audit access is scoped to the LSV's own contributions; the
operating programme does not expose other LSVs' detail through
this integration.

## §20 Format Adapter Vendor Integration

Format-adapter implementations are typically open-source community
projects (Mozilla L20n / Fluent, Apple's stringsdict tooling,
Android's resource compiler) or commercial CAT-vendor adapters.
The operating programme integrates the adapter as a content-
addressed module and pins a particular module revision per
project. Module revisions are consumed through standard package-
manager channels; the integration record carries the channel and
the pinned revision.

## §21 CLDR Mirror and Refresh Integration

The operating programme mirrors the active CLDR revisions it
consumes so that locale-data lookups remain available even when
the upstream CLDR distribution is unreachable. Refresh integration
runs at the cadence the operator declares (typically per CLDR
release); refresh events emit cross-references when locale-data
changes affect already-released bundles.

## §22 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients: build-pipeline adapters, LSV management adapters,
and CAT-tooling proxies continue to function across minor
revisions. Major revisions go through a deprecation window of at
least one full CLDR release cycle so that downstream pipelines
have time to migrate without losing localised builds in flight.

## §23 Operator-Side Audit-Trail Export

External auditors retained by the operating organisation consume
audit-trail exports through the integration layer. The export
includes API audit logs, dispatch records, build-pipeline
integration events, and the certificate-rotation history; auditor
client certificates are issued by the certifying body with scope
limited to the audit window the auditor was commissioned for.

## §24 Verifiable-Credential Re-Issuance for Project Attestations

Programmes that wish to expose project-level attestations (LSV
ISO 17100 conformance, build-bundle integrity, glossary curation
status) to consumers of W3C Verifiable Credentials MAY re-issue
the attestations as Verifiable Credentials under the Data Model
2.0 specification. Re-issuance is optional; the canonical record
remains the JSON evidence-package manifest.

## §25 Reader Tooling for Translation Reviewers

Manuscript reviewers, in-house language QA leads, and accessibility
advisors benefit from visualisation tools that surface translation
diff views, glossary deviation reports, and locale-coverage maps
across releases. Programmes MAY publish such reader tools alongside
the canonical evidence package; the tools are non-normative.

## §26 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one build pipeline, at least one LSV management
system, the relevant CAT tooling proxy, at least one quality-
assurance service, and at least one long-term archive, and has
published at least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-i18n
- **Last Updated:** 2026-04-27
