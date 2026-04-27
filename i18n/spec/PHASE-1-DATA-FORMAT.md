# WIA-i18n PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-i18n
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-i18n.
The standard covers exchange of internationalisation (i18n) and
localisation (l10n) artefacts — translation memory, glossary,
locale data, message catalogues, and translator workflow records —
among software publishers, language-service vendors, individual
translators, and the platform tooling (CAT systems, build pipelines,
review workflows) that processes the artefacts.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 639-1 / 639-2 / 639-3 (language codes)
- ISO 15924 (script codes)
- ISO 3166-1 / 3166-2 (country and subdivision codes)
- ISO/IEC 17025:2017 (testing and calibration laboratories — used here
  for translation-quality calibration where the laboratory model
  applies)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- IETF RFC 4122 (UUID URN)
- IETF RFC 5646 / BCP 47 (Tags for Identifying Languages)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- W3C XML Localisation Interchange File Format (XLIFF) 2.x as a
  community-maintained interchange envelope; the WIA standard wraps
  rather than redefines XLIFF 2.x
- Unicode CLDR (Common Locale Data Repository) — cited normatively
  for canonical locale data definitions

---

## §1 Scope

This PHASE document defines persistent shapes for translation
memory, glossary, message catalogues, and the workflow records that
accompany them across the i18n / l10n pipeline. Implementations
covered include:

- Translation memory (TM) servers that ingest and serve segment
  alignments.
- Terminology / glossary servers that govern domain-specific
  translation choices.
- Continuous-localisation pipelines that ingest source-language
  artefacts, dispatch them to translators, and re-integrate
  translated artefacts.
- Quality-assurance services that review translations against
  agreed quality models (LISA QA, MQM-aligned models).
- Publication systems that emit localised builds and runtime
  message bundles.

Machine-translation system internals (training data, model weights)
and font-rendering pipelines are out of scope.

## §2 Project Identifier

```
projectId         : string (uuidv7)
projectCreatedAt  : string (ISO 8601 / RFC 3339)
projectOperator   : string (institutional identifier of the
                       publisher operating the project)
sourceLocale      : string (BCP 47 language tag for the source
                       language; e.g. "en-US")
targetLocales     : array of string (BCP 47 tags; e.g.
                       ["ko-KR", "ja-JP", "zh-Hans-CN", "ar-SA"])
qualityModelRef   : string (content-addressed URI of the project's
                       quality model definition)
projectStatus     : enum ("draft" | "active" | "freeze" |
                       "archived" | "deprecated")
```

Projects whose `targetLocales` change after `active` emit a new
project-version record; prior records remain addressable as the
historical state.

## §3 Locale Data Reference

```
localeData:
  localeTag       : string (BCP 47)
  cldrRevision    : string (CLDR release identifier)
  fallbackChain   : array of string (BCP 47 tags in fallback order)
  features:
    plural        : enum ("zero-one-other" | "one-few-other" |
                       "one-other" | "other-only" | "user-defined")
    bidi          : enum ("ltr" | "rtl" | "context")
    digitGrouping : string (CLDR-style group/separator definition)
    quotationMarks : array of string
    listSeparator : string
    timeZoneAware : boolean
```

A project may pin a CLDR revision per locale or globally; revisions
emit new locale-data records and prior records remain addressable.

## §4 Source Segment Record

```
sourceSegment:
  segmentId       : string (uuidv7)
  projectId       : string (uuidv7)
  sourceLocale    : string (BCP 47)
  sourceText      : string (UTF-8; preserves source-side markup)
  context         : string (free text; UI screenshot URI, code
                       comment, calling site identifier, etc.)
  contextRef      : string (content-addressed URI of richer context
                       artefact when applicable)
  maxLengthCodepoints : integer (constraint that affects
                       translation; e.g. UI button caption)
  domain          : string (domain code, e.g. "ui-button",
                       "legal-notice", "marketing-headline")
  status          : enum ("source-extracted" | "ready-for-translation"
                       | "frozen-in-build" | "deprecated")
```

Source segments are immutable once status moves beyond
`source-extracted`; corrections emit a new source segment and emit
a deprecation record against the prior segment.

## §5 Translation Record

```
translation:
  translationId   : string (uuidv7)
  segmentId       : string (uuidv7, references §4)
  targetLocale    : string (BCP 47)
  targetText      : string (UTF-8)
  authorRef       : string (institutional or freelancer identifier;
                       PII held in operator HR / vendor system)
  authorKind      : enum ("human-translator" | "in-house-team" |
                       "machine-translation-with-pe" |
                       "post-edited-mt" | "fuzzy-match" |
                       "exact-match")
  matchScore      : number (0-100; for fuzzy or MT matches, the
                       similarity to the TM hit; absent for full
                       human translations)
  status          : enum ("draft" | "in-review" | "approved" |
                       "rejected" | "deprecated")
  approvalChain   : array of ApprovalEntry
```

Approved translations populate the project's published build; in-
review translations are excluded from the runtime bundle.

## §6 Glossary and Terminology Record

```
glossaryTerm:
  termId          : string (uuidv7)
  projectId       : string (uuidv7)
  domain          : string (matches §4 segment domain when
                       applicable)
  sourceTerm      : string
  sourceLocale    : string (BCP 47)
  preferredTranslations : array of LocaleTranslation
  forbiddenTranslations : array of LocaleTranslation
  notes           : string (translator guidance; redacted on
                       export when contains client-confidential
                       detail)

LocaleTranslation:
  targetLocale    : string (BCP 47)
  text            : string
  rationale       : string
```

Glossary entries are referenced by translation records (§5) so that
QA services can flag deviations from preferred terminology.

## §7 Translation Memory Record

```
tmEntry:
  entryId         : string (uuidv7)
  projectId       : string (uuidv7)
  sourceSegmentId : string (uuidv7)
  translationId   : string (uuidv7)
  bidiText:
    sourceText    : string
    targetText    : string
  contextHash     : string (SHA-256 of the segment's context
                       artefact for ICE-match detection)
  createdAt       : string (ISO 8601)
  lastUsedAt      : string (ISO 8601 / RFC 3339; updated on each
                       leverage)
  usageCount      : integer
```

TM entries are append-only; deprecation of a source segment emits
a TM-entry deprecation record but does not delete the entry, so
that historical builds remain reproducible.

## §8 Quality Review Record

```
qualityReview:
  reviewId        : string (uuidv7)
  translationId   : string (uuidv7)
  reviewerRef     : string (institutional identifier)
  performedAt     : string (ISO 8601)
  qualityModel    : string (e.g. "MQM-2.0", "LISA-QA-3.1",
                       "operator-internal")
  errorCategory   : array of enum ("accuracy" | "fluency" |
                       "terminology" | "style" | "locale-convention"
                       | "design-fit" | "verity" | "user-defined")
  errorSeverity   : enum ("critical" | "major" | "minor" |
                       "preferential")
  recommendedAction : enum ("accept" | "edit" | "retranslate" |
                       "escalate")
  reviewNotes     : string
```

## §9 Build Bundle Record

```
buildBundle:
  bundleId        : string (uuidv7)
  projectId       : string (uuidv7)
  builtAt         : string (ISO 8601)
  bundleArtefactRef : string (content-addressed URI of the
                       built localisation bundle; XLIFF 2.x,
                       gettext .mo, JSON i18n, ICU MessageFormat)
  includedTranslations : array of string (translation IDs)
  emptyKeyHandling : enum ("source-fallback" | "empty-string" |
                       "untranslated-marker")
```

## §10 Pseudo-Locale and Test-Locale Records

Build pipelines exercise i18n correctness through pseudo-locales
(e.g. `en-XA` accent-mangling, `en-XB` length-stretching) that surface
hard-coded strings, length-budget violations, bidirectional layout
breaks, and missing format placeholders.

```
pseudoLocale:
  pseudoLocaleId  : string (uuidv7)
  projectId       : string (uuidv7)
  pseudoTag       : string (BCP 47 private-use tag)
  generationRule  : enum ("accent-mangling" | "length-stretch" |
                       "bidi-injection" | "format-strict" |
                       "user-defined")
  parameters      : object (rule-specific configuration)
```

Pseudo-locale builds are gated as `non-shippable` so that they are
never accidentally promoted to release; the API enforces the gate
at the build-bundle endpoint by refusing to accept pseudo-locale
records as a `targetLocale`.

## §11 Plural / Gender / Selection Catalogue

Some target locales require message catalogues to express plural
or gender selection beyond the simple key→value pair. The catalogue
record carries the selection rules and the per-rule translations.

```
selectionCatalogue:
  catalogueId     : string (uuidv7)
  segmentId       : string (uuidv7)
  targetLocale    : string (BCP 47)
  selectorKind    : enum ("plural" | "gender" | "case" |
                       "user-defined")
  rules           : array of SelectionRule

SelectionRule:
  match           : string (CLDR plural keyword or selector value)
  text            : string (UTF-8)
```

The selector follows CLDR plural-keyword conventions; submissions
whose keywords do not match the locale's plural set return `422`
with type `urn:wia:i18n:plural-rule-mismatch` from the API.

## §12 Translator Workload and Vendor Dispatch Record

The dispatch record tracks the assignment of source segments (or
groups of segments) to LSV vendors and to individual translators
through the LSV proxy. The record carries the dispatch time, the
agreed delivery window, the rate / fee class for the pairing, and
the audit trail of redispatches when the original target cannot
deliver.

```
dispatch:
  dispatchId      : string (uuidv7)
  projectId       : string (uuidv7)
  segmentBatchRef : string (URI of the dispatched segment batch)
  vendorRef       : string (LSV identifier)
  translatorRef   : string (opaque LSV-internal identifier)
  dispatchedAt    : string (ISO 8601)
  dueAt           : string (ISO 8601)
  rateClassRef    : string (opaque pricing-tier identifier;
                       commercial detail held in vendor agreement)
  reassignmentLog : array of ReassignmentEntry
```

Translator identity (legal name, contact, payment) is held in the
LSV's HR / accounts-payable system and is never carried in the
API. The `translatorRef` is the LSV's internal opaque token that
maps to the translator's HR record.

## §13 Format Adapter Record

Source artefacts arrive in many formats — Java `.properties`, iOS
`.strings` / `.stringsdict`, Android `strings.xml`, gettext PO,
ICU MessageFormat JSON, YAML, FluentJS, web frameworks' bespoke
formats. The format-adapter record describes the operator's
extraction recipe per source format so that downstream pipelines
can reconstruct the segment set from the source artefacts.

```
formatAdapter:
  adapterId       : string (uuidv7)
  projectId       : string (uuidv7)
  sourceFormat    : enum ("java-properties" | "apple-strings" |
                       "apple-stringsdict" | "android-xml" |
                       "gettext-po" | "icu-messageformat-json" |
                       "yaml-i18n" | "fluent-js" | "xliff-2" |
                       "user-defined")
  extractionRulesRef : string (content-addressed URI of the
                       extraction rules)
  reintegrationRulesRef : string (content-addressed URI of the
                       reintegration rules used at build time)
```

A project may bind multiple format adapters when its source
artefacts span multiple formats (e.g. iOS + Android + web).

## §14 BCP 47 Tag Validity Notes

All language-tag fields in the records described above are valid
BCP 47 tags. Tags that include private-use subtags (`-x-...`) are
permitted only for pseudo-locales (PHASE-1 §10) and for operator-
internal experimental locales that never reach a release build.
Implementations validate tags against the in-force IANA Language
Subtag Registry on submission.

## §15 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every supported project and honour the
content-addressing rules in §3-§9.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-i18n
- **Last Updated:** 2026-04-27
