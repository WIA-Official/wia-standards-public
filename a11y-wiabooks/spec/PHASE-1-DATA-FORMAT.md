# WIA-a11y-wiabooks PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-a11y-wiabooks
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-a11y-wiabooks. The standard covers persistent record
shapes for digital book accessibility — EPUB 3 Accessibility
conformance, alternative formats (DAISY-aligned navigable
audio, refreshable Braille, large-print, simplified-language
companions), reading-system capability negotiation, screen-
reader and assistive-technology compatibility test results,
and the per-publication accessibility statements that
publishers issue to readers and to the regulators that
oversee accessibility law (US ADA / Section 508, EU EAA
2025, KR Disability Discrimination Act). The format is
consumed by accessibility metadata aggregators, screen-reader
vendors, refreshable-Braille device manufacturers, EPUB
reading systems, library and education-procurement
platforms, and the operators of the wiabooks publishing
infrastructure.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 40500:2012 (Information technology — W3C Web
  Content Accessibility Guidelines (WCAG) 2.0; WCAG 2.1 and
  2.2 referenced as community-managed updates)
- ISO/IEC 23761 (Information technology — Document
  description and processing languages — DAISY 3
  distribution format; the standard's WIA records
  reference the DAISY ANSI/NISO Z39.86-2005 base spec)
- ANSI/NISO Z39.86-2005 (Specifications for the Digital
  Talking Book; cited as the canonical DAISY reference
  alongside the EPUB 3 successor spec)
- W3C EPUB 3.3 (Open Container Format, Content Documents,
  Packages, Reading Systems) — community-managed
- W3C EPUB Accessibility 1.1 (conformance and discoverability)
- W3C EPUB Accessibility Techniques 1.1
- W3C MathML 3.0 (cited normatively for accessible math
  rendering in EPUB 3 reading systems)
- W3C SSML 1.1 (Speech Synthesis Markup Language; cited as
  the per-passage pronunciation override format that
  publishers may attach for screen-reader rendering)
- W3C PronunciationLexicon (PLS) 1.0 (per-publication
  pronunciation lexicons for screen-reader engines)
- W3C ARIA 1.2 / DPub-ARIA 1.1 (accessible-rich-internet-
  applications roles for digital publications)
- W3C SMIL 3.0 (Synchronized Multimedia Integration
  Language; cited as the EPUB Media Overlays format that
  binds text to audio for synchronised navigable
  reading)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 5646 / BCP 47 (language tags)
- ISO 15924 (script codes; relevant for Braille script
  selection across regions)
- ISO/IEC 27001:2022 (information security management)
- Schema.org `Accessibility*` properties (a11yMode,
  a11yFeature, a11yHazard, a11yAPI, a11ySummary; cited
  as the canonical metadata vocabulary for catalogue
  exchange)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts a
wiabooks accessibility programme manages. Implementations
covered include:

- EPUB 3 production toolchains that emit Accessibility-1.1-
  conformant EPUBs.
- DAISY-aligned navigable-audio production for blind and
  print-disabled readers.
- Refreshable-Braille production using the operating
  jurisdiction's Braille code (UEB / Korean Braille / kana
  Braille / equivalent).
- Reading-system capability registries (Apple Books,
  Google Play Books, Kobo, Thorium, Readium-based readers,
  Voice Dream, Adobe Digital Editions, Calibre's reader
  pane).
- Assistive-technology compatibility test platforms (NVDA,
  JAWS, VoiceOver, TalkBack, Orca, ZoomText, Dragon
  NaturallySpeaking, Switch Access).
- Library and education-procurement platforms that filter
  catalogue by accessibility conformance.

Print-format accessibility (large-print PDF rasterisation
beyond what reflowable EPUB 3 produces) is out of scope; the
wiabooks programme focuses on digital reading systems.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of
                         the publishing operator)
programmeRegistered  : string (ISO 8601 / RFC 3339)
publishingStreams    : array of enum ("trade-fiction" |
                         "trade-non-fiction" |
                         "academic-textbook" |
                         "professional-reference" |
                         "k-12-education" |
                         "higher-education" |
                         "religious-publication" |
                         "government-publication" |
                         "research-monograph" |
                         "user-defined")
jurisdictionScope    : array of string (ISO 3166-1; per
                         jurisdiction the operator's
                         accessibility-statement profile
                         applies)
accessibilityFramework : array of enum ("EPUB-Accessibility-1.1"
                         | "WCAG-2.0-AA" | "WCAG-2.1-AA" |
                         "WCAG-2.2-AA" | "DAISY-3" |
                         "EAA-2025" | "ADA-Section-508" |
                         "KR-Disability-Discrimination-Act"
                         | "user-defined")
programmeStatus      : enum ("draft" | "operating" |
                         "frozen" | "archived")
```

## §3 Publication Identifier

```
publication:
  publicationId      : string (uuidv7)
  programmeId        : string (uuidv7)
  isbn               : string (13-digit ISBN; absent for
                         publications below the ISBN
                         registration threshold)
  workTitle          : string (UTF-8)
  primaryLanguage    : string (BCP 47 tag — e.g. "ko-KR",
                         "en-US", "ja-JP")
  authoredAt         : string (ISO 8601 date)
  publicationStatus  : enum ("draft" | "in-production" |
                         "ready-for-distribution" |
                         "distributed" | "withdrawn")
  primaryFormatRef   : string (URI of the primary EPUB 3
                         distribution package)
```

## §4 EPUB Accessibility Statement Record

The EPUB Accessibility Statement is the publisher's per-
publication declaration of conformance per W3C EPUB
Accessibility 1.1 + Schema.org `Accessibility*` vocabulary.

```
accessibilityStatement:
  statementId        : string (uuidv7)
  publicationId      : string (uuidv7)
  conformanceProfile : enum ("EPUB-A-1.1-A" |
                         "EPUB-A-1.1-AA" |
                         "EPUB-A-1.1-AAA" |
                         "EPUB-A-1.1-Optimised-Discovery-only"
                         | "non-conformant-known-limitations")
  a11yMode           : array of string (Schema.org accessMode
                         vocabulary: "textual", "visual",
                         "auditory", "tactile",
                         "chartOnVisual", "mathOnVisual",
                         "chemOnVisual", "diagramOnVisual",
                         "musicOnVisual", "textOnVisual")
  a11yFeature        : array of string (Schema.org
                         accessibilityFeature: "alternativeText",
                         "captions", "describedMath",
                         "displayTransformability",
                         "longDescription", "MathML",
                         "readingOrder", "structuralNavigation",
                         "tableOfContents", "taggedPDF",
                         "transcript", "ttsMarkup",
                         "synchronizedAudioText",
                         "highContrastDisplay",
                         "largePrint", "braille",
                         "audioDescription",
                         "signLanguage")
  a11yHazard         : array of string (Schema.org
                         accessibilityHazard: "flashing",
                         "noFlashingHazard", "motionSimulation",
                         "noMotionSimulationHazard", "sound",
                         "noSoundHazard", "unknown")
  a11yAPI            : array of string (Schema.org
                         accessibilityAPI: "ARIA")
  a11ySummary        : string (UTF-8; human-readable
                         accessibility statement; localised
                         per primary language)
  knownLimitations   : array of string (per-limitation
                         description for cases where the
                         publication does not fully meet the
                         declared conformance profile)
  certifiedBy        : string (operator-internal certifier
                         identifier; for AA / AAA conformance,
                         the operator's accessibility-
                         certification body)
  certifiedAt        : string (ISO 8601)
```

## §5 Alternative Format Record

```
alternativeFormat:
  formatId           : string (uuidv7)
  publicationId      : string (uuidv7)
  formatKind         : enum ("epub-3-reflowable" |
                         "epub-3-fixed-layout" |
                         "epub-3-with-media-overlays" |
                         "daisy-3-navigable-audio-only" |
                         "daisy-3-full-text-and-audio" |
                         "refreshable-braille-brf" |
                         "refreshable-braille-pef" |
                         "embossed-braille-pdf" |
                         "large-print-pdf" |
                         "simplified-language-companion" |
                         "sign-language-video-companion")
  artefactRef        : string (content-addressed URI of the
                         format artefact)
  artefactDigest     : string (SHA-256)
  brailleCodeRef     : string (URI of the Braille code in
                         use; e.g. "UEB-2023", "KR-Braille-
                         Standard", "Japanese-Braille")
  audioVoiceRef      : string (TTS / human narrator
                         identifier for audio formats)
  derivationToolRef  : string (URI of the derivation toolchain
                         that produced the alternative format
                         from the primary EPUB 3)
```

## §6 Media Overlay Record (EPUB Synchronised Audio-Text)

EPUB 3 Media Overlays use W3C SMIL 3.0 to bind text fragments
to audio time intervals for synchronised navigable reading.

```
mediaOverlay:
  overlayId          : string (uuidv7)
  publicationId      : string (uuidv7)
  smilArtefactRef    : string (content-addressed URI of the
                         SMIL document)
  audioArtefactRef   : string (content-addressed URI of the
                         narration audio)
  textFragmentMap    : object (per-fragment ID with start /
                         end times in the audio; the SMIL
                         document is canonical, this object
                         is an indexable summary)
  narrationKind      : enum ("human-narrator" | "tts-engine"
                         | "hybrid")
  ttsLexiconRef      : string (URI of the W3C PLS
                         pronunciation lexicon when narration
                         is TTS)
  ssmlOverridesRef   : string (URI of W3C SSML 1.1
                         per-passage pronunciation overrides
                         where the lexicon is insufficient)
```

## §7 Assistive-Technology Compatibility Test Record

```
atCompatibilityTest:
  testId             : string (uuidv7)
  publicationId      : string (uuidv7)
  testedAt           : string (ISO 8601)
  assistiveTechRef   : enum ("nvda-windows" |
                         "jaws-windows" | "voiceover-macos"
                         | "voiceover-ios" | "talkback-android"
                         | "orca-linux" | "zoomtext-windows"
                         | "dragon-naturallyspeaking" |
                         "switch-access-android" |
                         "switch-control-ios" |
                         "user-defined")
  assistiveTechVersion : string
  readingSystemRef   : enum ("apple-books" |
                         "google-play-books" | "kobo" |
                         "thorium" | "readium-2" |
                         "voice-dream" |
                         "adobe-digital-editions" |
                         "calibre-reader" | "user-defined")
  readingSystemVersion : string
  testResultsArtefactRef : string (URI of the test results
                         archive — screenshots, captured
                         screen-reader output, navigation
                         trace)
  outcome            : enum ("nominal" |
                         "partial-functionality" |
                         "blocked-defect-found" |
                         "incompatible")
  defectsFiledRefs   : array of string (URIs of defect
                         tickets filed against publisher,
                         AT vendor, or reading-system vendor)
```

## §8 Per-Reader Reading Preference Record (Optional)

```
readingPreference:
  preferenceId       : string (uuidv7)
  publicationId      : string (uuidv7)
  readerTokenRef     : string (opaque token for the reader;
                         clinical reader identity in the
                         operator's CRM, never on this API)
  preferredFormat    : enum (matches §5 formatKind)
  preferredFontSize  : integer (px; absent for non-visual
                         readers)
  preferredHighContrast : boolean
  preferredReadingOrder : enum ("publisher-default" |
                         "linear-reading-order" |
                         "structural-reading-order")
  preferredTtsVoiceRef : string (TTS voice identifier where
                         the reader uses TTS)
  preferredBrailleCodeRef : string (Braille code identifier
                         where the reader uses Braille)
```

## §9 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every published title and
honour the EPUB Accessibility 1.1 conformance profile in §4.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-a11y-wiabooks
- **Last Updated:** 2026-04-28
