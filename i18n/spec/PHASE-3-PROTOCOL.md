# WIA-i18n PHASE 3 — PROTOCOL Specification

**Standard:** WIA-i18n
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
i18n / l10n programme: language-service-vendor (LSV) accreditation,
translator qualification, locale-data adoption, source-extraction
discipline, translation memory governance, terminology management,
quality-review governance, build-bundle reproducibility, records
retention, privacy of translator personal data, programme
wind-down, and inter-vendor portability of TM / glossary assets.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17043:2010 (proficiency testing)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 9001:2015 (quality management systems)
- ISO 17100:2015 (translation services — requirements; cited
  normatively for translator qualification levels)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- BCP 47 / RFC 5646 (language tags)
- Unicode CLDR (locale data)
- W3C XLIFF 2.x (interchange envelope reference)

---

## §1 LSV Accreditation

A language-service vendor MAY claim conformance to WIA-i18n only
after a recognised accreditation body has issued a valid certificate
against ISO 17100:2015 covering the language pairs the LSV
exercises. Each language pair is a distinct accreditation line item;
a vendor that subcontracts to another LSV ensures that the
subcontractor's accreditation covers the relevant pair.

The accreditation register is exposed to the API as a read-only
resource and MUST be re-fetched at least once per calendar quarter.

## §2 Translator Qualification

Individual translators contribute through LSV proxies; their
qualification level (per ISO 17100 categories: translator, reviser,
reviewer, terminology manager, project manager) is recorded in the
LSV's HR system and is referenced from the translation record's
`authorRef`. Programmes that publish externally cited translations
SHOULD require ISO 17100-aligned qualifications for human
translators contributing to the project.

## §3 Locale-Data Adoption

Programmes pin the CLDR revision per project and per locale. CLDR
upgrades that change plural-rule behaviour, digit grouping, or
quotation conventions emit cross-reference records so that
downstream consumers can navigate the change.

A project that targets a new locale not in its current pinning set
records the new locale's pinning at the time of activation; the
historic pinning of existing locales is unchanged.

## §4 Source-Extraction Discipline

Source segments (PHASE-1 §4) are immutable beyond
`source-extracted`. Source-side corrections (typo fixes, copy
revisions) emit new segments; the prior segment is deprecated but
its translations remain accessible so that downstream rebuilds of
historical releases reproduce the released text.

Continuous-localisation pipelines that re-extract sources at every
build verify that re-extracted text matches the prior pinning;
mismatches trigger an alert to the localisation programme manager
and the source segments are dispositioned (accept new, restore
prior, or branch into parallel revisions).

## §5 Translation Memory Governance

TM is governed under operator policy: which segments enter the TM,
which are quarantined, the leverage threshold (default 75 fuzzy
floor), and the cleanup policy for stale or low-quality entries.
TM entries are append-only; deprecated entries remain in the index
but are excluded from leverage suggestions.

Cross-project TM sharing within an operating organisation is
permitted; cross-organisation sharing requires a written agreement
that names the data scope, the retention bound, and the exit
clause for asset return.

## §6 Terminology Management

Glossary entries (PHASE-1 §6) are managed by terminology managers
who are accredited under ISO 17100 §6.1.4 (terminology management
competence). Glossary changes follow change-control: proposed
change → review by terminology manager and domain expert → approval
by operating programme → publication. Programmes maintain an audit
trail of every glossary change so that downstream consumers can
trace term-choice evolution.

## §7 Quality Review Governance

Quality reviews follow the project's chosen quality model
(MQM-aligned, LISA QA, operator-internal). Reviewers are
qualified per ISO 17100 §6.1.5. Critical errors trigger
re-translation; major errors trigger edit; minor and preferential
errors are recorded but do not block release.

A project that publishes externally cited translations
participates in inter-vendor calibration exercises so that
reviewer scoring across vendors is comparable. Calibration
exercises follow ISO/IEC 17043 expectations.

## §8 Build-Bundle Reproducibility

Build bundles are reproducible: given the same source segments,
the same approved translations, the same locale-data pinning, and
the same build pipeline version, a re-build produces a byte-
identical bundle (or an equivalent bundle under the documented
equivalence relation when format-specific metadata such as build
timestamps differ).

Programmes record the build pipeline's version, the runtime
configuration, and the dependency manifest in the build-bundle
record so that retrospective audits can reproduce the bundle.

## §9 Records Retention

Programme records — every record defined in PHASE-1, the API
audit logs, build bundles, quality reviews, glossary changes, and
TM entries — retain for a minimum of seven calendar years from
the last access of the project. Externally cited translations
retain indefinitely.

## §10 Time Synchronisation

Programme clocks synchronise per RFC 5905 (NTPv4) so that
extraction, translation, review, and build events can be ordered
unambiguously across vendors and pipelines.

## §11 Translator Privacy

Individual translator PII (name, contact, payment information) is
held in the LSV's HR / accounts-payable system and is never carried
in records exposed by this PHASE. The `authorRef` field is an
opaque reference to the LSV's internal record. Subject-access
requests from translators are mediated by the LSV under ISO/IEC
27701:2019 expectations.

## §12 Inter-Vendor Portability

When a project changes LSV, the operating programme exports the
project's TM and glossary as a portability bundle (XLIFF 2.x +
TBX-style glossary in a content-addressed archive) and the new
LSV ingests the bundle through the API. Portability is bidirectional
and round-trip-safe; the operating programme verifies bundle
integrity by re-ingesting the bundle into a side-by-side
comparison harness.

## §13 Cybersecurity

Translation deliveries flow over mutually-authenticated TLS 1.3
(RFC 8446). Build pipelines that consume translation bundles
verify the bundle's content-address before integration; failed
verification halts the build.

## §14 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies translators
and LSVs of record retention obligations, and publishes a sunset
timeline for in-flight projects.

## §15 Bidirectional Text and Complex-Script Considerations

Locales that write right-to-left (Arabic, Hebrew) or that depend on
complex-script shaping (Arabic, Indic, Thai, Khmer, etc.) follow
operator-defined layout-test playbooks: bidi-isolation around
ICU MessageFormat placeholders, complex-script font availability
verification, line-break rule validation per Unicode UAX 14, and
shaping-engine version pinning.

The operating programme records the playbook in its quality
dossier and exercises it through pseudo-locale builds (PHASE-1
§10) before any release that targets a complex-script locale.

## §16 Inclusivity and Accessibility

Translations destined for accessibility surfaces (screen readers,
captioning) follow the operator's inclusive-language guidance:
gender-neutral phrasing where the source language permits,
plain-language readability targets for caption tracks, and
accessibility-oriented terminology choices recorded in the
glossary. The guidance is reviewed at least annually by an
accessibility advisor named in the programme's quality dossier.

## §17 Cross-Border Programme Operation

Programmes that operate across borders maintain a primary
jurisdiction of registration and operating MoUs with partner
jurisdictions. Cross-jurisdictional data transfers honour the
source-jurisdiction's data-protection law for translator personal
data; only opaque references flow through the API.

## §18 Citation Hygiene and Successor Relationships

A programme is responsible for the integrity of citations to its
released build bundles. When an external publication cites a
release that the programme has subsequently superseded (a
hot-fix translation, a copy-revision rebuild), the programme keeps
the originally cited bundle addressable at its content-addressed
URL and publishes a successor relationship through the discovery
document so that downstream readers can navigate from the cited
bundle to its current revision.

## §19 Embargo of Pre-Release Translations

Translations destined for a coordinated launch (a press
announcement, a regulator-required simultaneous publication) are
held under embargo until the release time. Embargo-lift events are
recorded against the build bundle and published through the
streaming endpoint so that downstream consumers can synchronise
their consumption with the embargo schedule.

## §20 Format Adapter Versioning

Format adapters (PHASE-1 §13) are versioned independently of the
project. An adapter revision that changes extraction behaviour
(e.g. introduces a new placeholder syntax interpretation) emits a
new adapter record; the project's pinning is updated explicitly so
that re-extraction at the next build is intentional rather than
silent.

Adapter revisions that affect already-released bundles trigger a
re-build verification: the operating programme re-extracts the
prior release using the new adapter and compares the resulting
segment set to the historical segment set; mismatches are
investigated before the new adapter is accepted.

## §21 Confidentiality of Project Content

Some project content is confidential to the operating organisation
or to its end customers (pre-release UI strings, regulatory
filings, financial disclosures). The operating programme records
the confidentiality classification per project and applies access
controls consistent with the classification. LSV access is bound
to the project's confidentiality clause in the LSV agreement;
violations trigger an incident under §22.

## §22 Confidentiality Incidents

Confidentiality incidents (leaked translations, unauthorised
redistribution) are recorded against the operating programme and
investigated under the operator's security incident-response
process. The incident record is preserved indefinitely as part of
the audit trail.

## §23 Inter-LSV Calibration Exercises

Operators that work with multiple LSVs in parallel calibrate
review scoring across vendors at least once per calendar year.
The calibration exercise distributes a shared sample translation
to every participating LSV, collects the per-LSV review scores,
and identifies systematic disagreements that warrant a moderator
discussion. Calibration outcomes are recorded against the
operator's quality dossier and inform per-vendor weighting
decisions on subsequent dispatches.

## §24 Quality-Dossier Annual Review

The programme's quality dossier (PHASE-3 §15 of analogous
standards) is reviewed at least annually by the operator's quality
manager and is read during the annual ISO 17100 surveillance audit
that the operator's primary LSV undergoes. The review's outcomes
are recorded as content-addressed minutes that the public catalogue
references.

## §25 Conformance and Auditing

A programme conformant with WIA-i18n publishes its LSV
accreditation references, its programme code registration, its
quality dossier, and the catalogue of build bundles it has
released, and answers an annual self-assessment that maps each
clause of this PHASE to the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-i18n
- **Last Updated:** 2026-04-27
