# WIA-a11y-wiabooks PHASE 4 — INTEGRATION Specification

**Standard:** WIA-a11y-wiabooks
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a wiabooks accessibility programme
integrates with the systems that surround it: accessibility-
metadata aggregators (Bookshare, Vital Source, Accessible
Books Consortium / ABC, Library of Congress NLS BARD,
National Library service for the Blind in the operator's
jurisdiction); EPUB reading-system vendors; assistive-
technology vendors; library and education-procurement
platforms; the W3C EPUB Working Group's accessibility-
techniques registry; statutory regulators (US Access Board,
EU EAA conformity-assessment bodies, KR National Information
Society Agency); and long-term archives that preserve
accessible publications for the long-term reading record.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C EPUB Accessibility 1.1
- W3C Verifiable Credentials Data Model 2.0 (optional)
- Schema.org Accessibility properties
- Marrakesh Treaty (cross-border exchange of accessible
  copies for blind and print-disabled persons)

---

## §1 Accessibility Metadata Aggregator Integration

Aggregators (Bookshare, ABC, Vital Source, NLS BARD,
operator-specific national libraries for the blind)
ingest publisher accessibility statements and alternative-
format catalogues so that disabled readers can discover
accessible titles through the aggregator's interface.
Integration carries each aggregator's identifier, the per-
title metadata-deposit schedule, and the aggregator's
conformance criteria for ingest acceptance.

## §2 EPUB Reading System Vendor Integration

Reading systems (Apple Books, Google Play Books, Kobo,
Thorium, Readium-2, Voice Dream, Adobe Digital Editions,
Calibre Reader) consume publisher accessibility statements
to surface accessibility metadata in the reader UI.
Integration carries each vendor's identifier, the vendor's
EPUB Accessibility 1.1 implementation profile, and the
operator's per-vendor compatibility test results.

## §3 Assistive-Technology Vendor Integration

AT vendors (NV Access for NVDA, Vispero for JAWS, Apple
for VoiceOver, Google for TalkBack, Acapela / ReadSpeaker
for TTS engines, refreshable-Braille hardware vendors
including HumanWare, Freedom Scientific, BraillePen)
consume per-publication AT compatibility test results so
that AT vendor support and roadmap planning can prioritise
defects affecting publisher catalogues. Integration carries
each vendor's identifier, the per-AT-version support window,
and the AT-side defect-intake reference.

## §4 Library and Education-Procurement Platform Integration

Library platforms (OverDrive, Hoopla, RBdigital, Scribd
education tier, OPAC integrations) and education-
procurement platforms (university library digital
collections, K-12 textbook adoption systems) filter
catalogue by accessibility conformance level. Integration
carries each platform's identifier, the per-platform
filter taxonomy, and the operator's catalogue-feed
update cadence.

## §5 W3C EPUB Working Group Integration

The W3C EPUB Working Group maintains the EPUB
Accessibility 1.1 + Techniques 1.1 specifications.
Integration is one-way (operator consumes the
specifications); the operator records the spec revision
in force per published title so that downstream consumers
can resolve the conformance-claim to the spec version
the publisher targeted at publication time.

## §6 Statutory Regulator Integration

Statutory regulators (US Access Board for Section 508,
EU national EAA conformity-assessment bodies, KR
National Information Society Agency for KS X 6308
conformance, equivalent national authorities) consume
per-title compliance evidence. Integration carries the
regulator's identifier, the per-jurisdiction submission
template, and the regulator's complaint-intake endpoint
through which disabled readers may file accessibility
complaints.

## §7 Marrakesh Treaty Cross-Border Exchange Integration

The Marrakesh Treaty enables cross-border exchange of
accessible copies for blind and print-disabled persons
through authorised entities. Operators that participate
as an authorised entity (or that supply authorised
entities) integrate with the Marrakesh Treaty exchange
network. Integration carries the operator's authorised-
entity status reference, the per-title eligibility
declaration, and the per-cross-border-recipient
acknowledgement record.

## §8 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  publications/                — per-publication records
  accessibility-statements/    — per-publication conformance
                                  statements
  alternative-formats/         — per-publication alternative
                                  format references (artefact
                                  bodies content-addressed)
  media-overlays/              — per-publication overlay
                                  metadata
  at-compatibility-tests/      — per-publication AT test
                                  results
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed
by the operator's HTTP-message-signature key (RFC 9421)
and counter-signed by the accessibility-certification body
when the package supports a regulator submission or a
Marrakesh-Treaty cross-border declaration.

## §9 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:a11y-wiabooks:evidence-mismatch`.

## §10 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-a11y-wiabooks` that links to the API
root, the operator's accessibility-framework enrolments,
the published quality dossier, the per-jurisdiction
statutory framework binding, and the catalogue of
released publications with their accessibility-conformance
profiles.

## §11 Long-Term Archive Integration

Operators designate a long-term archive that holds
publications and their alternative formats beyond the
operator's primary distribution horizon (commercial
deletion is not equivalent to accessibility withdrawal —
already-distributed accessible copies remain available
to readers). Quarterly deposits round-trip content-
addresses; on programme wind-down, remaining records
transfer to the archive with content-addresses preserved.

## §12 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (EPUB
Accessibility 1.1 conformance, accessibility-
certification body credentials, ISO/IEC 27001
certification) to consumers of W3C Verifiable Credentials
MAY re-issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance is
optional; the canonical record remains the JSON evidence-
package manifest.

## §13 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long publication-batch ingest or AT-test reporting
windows resume from the last seen event identifier without
losing visibility of priority-1 events (statement
revisions affecting distribution, regulator-notified
complaints, AT-compatibility regressions).

## §14 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full W3C EPUB
Accessibility specification revision cycle so that
aggregator and reading-system integrations have time to
migrate.

## §15 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-
i18n for translation-pipeline integration, WIA-images
for accessibility-aware image alt-text production, WIA-
braille for refreshable-Braille rendering, WIA-tts for
text-to-speech voice management) emit cross-standard
linkage records that name the consuming standard and
the version under which the linkage is claimed.

## §16 Public Catalogue Feed

Operators publish a public catalogue feed (Atom or JSON
Feed) listing distributed publications with their
accessibility-statement digests, alternative-format
catalogue, and AT compatibility summary. The feed enables
discovery by disabled readers using assistive
technologies; aggregators ingest the feed at the
operator's published cadence.

## §17 Reader Tooling for Accessibility Reviewers

Accessibility reviewers (publisher-internal QA, external
certification bodies, regulator inspectors) benefit from
visualisation tools that surface accessibility statement
diff views, AT compatibility matrices, and known-
limitation evolution timelines. Operators MAY publish
reader tools alongside the canonical evidence package;
the tools are non-normative.

## §18 Migration from Pre-Standard Records

Operators of pre-standard accessible publishing
programmes MAY migrate historical records by emitting
synthetic publication records with a `legacyImport`
flag. Synthetic publications are accepted by the
public catalogue but require contemporaneous EPUB
Accessibility 1.1 re-validation before the publication
can advance past `ready-for-distribution`.

## §19 TTS Voice Vendor Integration

Operators that bind TTS narration to per-vendor voices
(ReadSpeaker, Acapela, Polly, Google Cloud TTS, Azure
Speech) integrate with each TTS vendor's voice
inventory. Integration carries the vendor's identifier,
the per-voice licensing terms, and the voice-revision
notification endpoint that triggers regeneration of
TTS-narrated alternative formats when a voice is
revised or retired.

## §20 Accessibility-Complaint Intake Integration

The operator's accessibility-complaint intake (through
the operator's website, the operating jurisdiction's
regulator, and Marrakesh-Treaty-aligned authorised
entities) ingests reader complaints and routes them to
the operator's remediation workflow. Integration carries
the intake-channel identifier, the per-complaint
classification (defect, missing format, statement
inaccuracy, AT incompatibility), and the response SLA
the operator commits to.

## §21 Authoring-Tool Plug-in Integration

Operators integrate accessibility-checking plug-ins into
the publisher's authoring workflow (InDesign accessibility
plug-ins, Word accessibility checker integrations, custom
markdown-to-EPUB pipelines with built-in WCAG validation).
Integration carries the plug-in's identifier, the per-
revision check capability matrix, and the per-issue
severity classification that flows back into the
publisher's editorial review.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with at least one accessibility-metadata
aggregator, at least one EPUB reading-system vendor,
the relevant statutory regulator (where the operating
jurisdiction has one), and at least one long-term
archive, and has published at least one externally
citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-a11y-wiabooks
- **Last Updated:** 2026-04-28
