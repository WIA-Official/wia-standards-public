# WIA-a11y-wiabooks PHASE 3 — PROTOCOL Specification

**Standard:** WIA-a11y-wiabooks
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a wiabooks
accessibility programme: WCAG 2.0/2.1/2.2 conformance
discipline, EPUB Accessibility 1.1 certification workflow,
DAISY production discipline, refreshable-Braille production
discipline, assistive-technology compatibility test cadence,
accessibility-statement publication discipline, end-of-life
withdrawal, and statutory accessibility-law compliance
across operating jurisdictions.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 40500:2012 (WCAG 2.0)
- ISO/IEC 23761 (DAISY 3 distribution format)
- ANSI/NISO Z39.86-2005 (Specifications for Digital Talking
  Book)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- W3C EPUB 3.3 (community-managed)
- W3C EPUB Accessibility 1.1 + Techniques 1.1
- W3C MathML 3.0 / SSML 1.1 / PLS 1.0 / SMIL 3.0
- W3C ARIA 1.2 / DPub-ARIA 1.1
- US ADA + Section 508 of the Rehabilitation Act + ICT
  Refresh Final Rule
- EU Web Accessibility Directive 2016/2102 + EU
  Accessibility Act (Directive 2019/882) effective from
  2025
- KR Disability Discrimination Act and Remedy Act +
  KS X 6308 (national accessibility standards)
- UNCRPD Article 9 and Article 21 (international
  accessibility convention reference)
- UEB (Unified English Braille) 2023 + national Braille
  authorities (KR Braille standards, JP Braille standards,
  equivalent national Braille codes)

---

## §1 WCAG Conformance Discipline

EPUB content documents (XHTML, CSS, JavaScript, MathML,
SVG) are governed by W3C WCAG. The operator's WCAG
conformance baseline:

- WCAG 2.0 Level AA is the minimum baseline (corresponding
  to ISO/IEC 40500:2012);
- WCAG 2.1 Level AA adds mobile and low-vision success
  criteria; the operator records its WCAG-version target
  per publishing stream;
- WCAG 2.2 Level AA adds user-input and authentication
  success criteria; the operator records the target where
  publications include interactive content;
- AAA criteria are aspirational and are recorded per-
  publication where the publisher claims them.

WCAG-conformance evidence flows through the operator's
accessibility-certification workflow and into the
accessibility statement (PHASE-1 §4
`conformanceProfile`).

## §2 EPUB Accessibility 1.1 Certification

Per the W3C EPUB Accessibility 1.1 specification, the
operator's accessibility-certification body conducts:

- conformance check against the operator's adopted
  Optimised Discoverability profile (mandatory metadata
  fields per the conformance level claimed);
- automated checks against the EPUB Accessibility
  Techniques 1.1 (ace by DAISY-equivalent automated
  validators);
- manual review for techniques the automated validators
  cannot evaluate (alternative-text quality, reading-
  order intent, language-tag correctness on text fragments
  that switch language, MathML semantic correctness);
- per-publication test report that the certification
  body signs and that the publisher attaches as the
  `certifiedBy` reference.

Certifications expire per the operator's revalidation
cadence (typically at each major content revision; minor
revisions inherit the prior certification when the
revision affects only non-content metadata).

## §3 DAISY Production Discipline

Navigable audio for blind and print-disabled readers
follows ISO/IEC 23761 / ANSI/NISO Z39.86 DAISY 3
distribution format or the EPUB 3 successor format with
Media Overlays (PHASE-1 §6). Production discipline:

- per-title narrator selection (human narrator vs TTS;
  hybrid acceptable for technical content with TTS for
  formulae and human narrator for prose);
- per-title pronunciation lexicon authoring for proper
  nouns, terms, foreign-language passages;
- per-title navigable structure (chapter / section /
  page boundaries cited from the source EPUB);
- per-title audio quality conforming to the operator's
  audio-production SOP (sample rate, bit depth, dynamic
  range, ambient noise floor).

DAISY artefacts are content-addressed and ingested
through the alternative-format endpoint of PHASE-2 §6.

## §4 Refreshable-Braille Production Discipline

Refreshable-Braille production produces BRF (Braille Ready
Format) and PEF (Portable Embosser Format) artefacts that
refreshable-display devices and embossers consume.
Discipline:

- per-title Braille code selection per primary language
  (UEB-2023 for English, KR Braille standards for Korean,
  JP Braille for Japanese, equivalent national codes
  elsewhere);
- per-title contracted-Braille vs uncontracted-Braille
  decision per the title's audience (technical / academic
  publications often emit both contracted and
  uncontracted versions);
- per-title math notation handling (Nemeth Code for US
  English math; Korean Braille math notation for Korean;
  equivalent national notations);
- per-title two-pass quality review (one Braille reader
  + one sighted Braille-trained reviewer).

## §5 Assistive-Technology Compatibility Test Cadence

Per-publication AT compatibility tests (PHASE-1 §7) are
performed at least once before distribution and re-
performed on each major content revision. The operator's
test matrix covers:

- screen readers: NVDA (Windows), JAWS (Windows),
  VoiceOver (macOS, iOS), TalkBack (Android), Orca
  (Linux);
- screen magnifiers: ZoomText, OS-built-in magnification;
- voice control: Dragon NaturallySpeaking, OS-built-in
  voice control;
- switch access: Switch Control (iOS), Switch Access
  (Android);
- across reading systems: Apple Books, Google Play Books,
  Kobo, Thorium, Readium-2, Voice Dream, Adobe Digital
  Editions.

Defects discovered during testing are filed against the
party responsible (publisher for content defects, AT
vendor for AT defects, reading-system vendor for reading-
system defects) and the publication's accessibility
statement is updated to record the known limitation
until the defect is resolved.

## §6 Accessibility Statement Publication Discipline

Per W3C EPUB Accessibility 1.1, the per-publication
accessibility statement is published in two forms:

- machine-readable Schema.org `Accessibility*` properties
  in the EPUB OPF metadata + the operator's catalogue
  feed;
- human-readable accessibility statement in the EPUB
  package (a designated XHTML document in the
  publication) and in the operator's catalogue page.

Both forms are localised to the publication's primary
language and to the languages the catalogue serves;
translations beyond the primary language follow the
operator's translation-quality SOP.

## §7 End-of-Life Withdrawal

Withdrawn publications (PHASE-1 §3 `publicationStatus
= "withdrawn"`) remain addressable through their
content-addressed primary format URI so that already-
distributed copies remain accessible to readers; new
acquisitions are blocked through the catalogue's
withdrawal flag.

## §8 Statutory Accessibility-Law Compliance

The operator's per-jurisdiction compliance discipline:

- US ADA + Section 508: applicable to titles distributed
  to US Federal entities and to publishers that publish
  electronically with public-accommodation reach; the
  operator records its Section 508 conformance binding
  per applicable title;
- EU EAA 2025: applicable to e-book products distributed
  to EU consumers from June 2025; the operator records
  the EAA conformity-assessment route applied per title;
- KR Disability Discrimination Act + KS X 6308: the
  operator records its national-standard binding;
- UNCRPD Article 9 / 21: the operator records its
  alignment as a soft-norm reference.

Statutory non-compliance triggers the operator's
remediation workflow: known-limitation disclosure,
remediation plan, and (where the jurisdiction provides
a remedy mechanism) regulator notification.

## §9 Records Retention

Programme records — every publication / accessibility
statement / alternative format / media overlay / AT
compatibility test / API audit log — retain for at least
the duration of the publication's distribution plus the
jurisdiction's records-retention period (typically 7
years post-distribution for trade publications,
indefinitely for educational and government
publications where the jurisdiction requires).

## §10 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so that
publication-event timestamps and audit logs are
consistent across the operator's production fleet and
the catalogue-aggregator integrations.

## §11 Privacy

Reader preferences (PHASE-1 §8) carry only opaque reader
tokens; the operator's CRM mediates to the reader's
identity per the operating jurisdiction's data-protection
law (GDPR, K-PIPA, CCPA, equivalent rules elsewhere).
Reader preferences are revealed only to the reader's own
authorised reading clients; analytics aggregations
follow the operator's privacy-preserving aggregation
policy.

## §12 Quality Dossier

The operator's quality dossier records the WCAG-
conformance-baseline framework, the EPUB-Accessibility
certification body, the DAISY production SOP, the
Braille production SOP, the AT compatibility test
matrix, the per-jurisdiction statutory framework
mapping, and the operator's incident history. The
dossier is reviewed at least annually by the operator's
accessibility-quality manager.

## §13 Cross-Jurisdictional Operation

Multi-jurisdiction publishers honour each jurisdiction's
statutory framework per published title; per-title
governing-jurisdiction tagging supports downstream
catalogue filtering and regulator-specific reporting.

## §14 Image Description and MathML Discipline

Images and mathematical content are accessible only when
the publisher attaches semantically-correct alternative
descriptions and machine-readable mathematical markup.

Image discipline:

- per-image short alternative text (`alt`) describing the
  image's role within the surrounding text;
- per-image long description (`aria-describedby` pointing
  to a `<details>` or sidebar element) for images conveying
  complex information (charts, schematics, diagrams);
- decorative images marked with `role="presentation"` or
  empty `alt=""` so that screen readers skip them;
- per-image complexity classification recorded in the
  operator's image-asset register so that downstream
  re-publication maintains the description quality.

MathML discipline:

- all mathematical content emitted as W3C MathML 3.0 in
  the EPUB content document (no rasterised math images
  except as a fallback alongside MathML);
- per-equation alternative-text rendering for screen
  readers that consume MathSpeak or ClearSpeak; the
  publisher's MathSpeak / ClearSpeak preference is
  declared in the publication's `tts-rate` and
  `text-to-speech-pronunciation` metadata.

## §15 Reading-Order and Structural Navigation Discipline

EPUB content emits a single canonical reading order through
the OPF spine; auxiliary content (sidebars, footnotes,
endnotes) carries DPub-ARIA roles so that screen readers
can navigate the structural hierarchy without losing the
publisher's intended reading flow. The operator's per-
publication review covers the spine order, the per-section
heading hierarchy (h1 → h6 monotonic descent), the table-
of-contents alignment with the spine, and the page-list
metadata where the publication tracks print-equivalent
pagination.

## §16 Conformance and Auditing

A programme conformant with WIA-a11y-wiabooks publishes
its accessibility-framework enrolment, the operator's
per-publication accessibility statements, the catalogue
of alternative formats per title, the AT compatibility
matrix, and the per-jurisdiction statutory framework
binding, and answers an annual self-assessment that
maps each clause of this PHASE to the operator's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-a11y-wiabooks
- **Last Updated:** 2026-04-28
