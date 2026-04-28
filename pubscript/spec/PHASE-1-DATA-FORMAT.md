# WIA-pubscript PHASE 1 — Data Format Specification

**Standard:** WIA-pubscript
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-pubscript, the multi-sensory publishing
interoperability standard. The records bind every
publication, sensory representation, accessibility
declaration, narration timing, and cross-format
manifest to documented publishing standards so
that visual, auditory, tactile, spatial, and
gestural representations are produced and
consumed as equal first-class citizens.

The standard's design principle: every reader
deserves an authored representation in the modality
they prefer; no modality is the "default".

References (CITATION-POLICY ALLOW only):
- W3C EPUB 3.3, EPUB Accessibility 1.1 (WCAG 2.2 AA)
- W3C Web Publications, W3C Audiobooks Manifest
- W3C Synchronized Multimedia Integration Language (SMIL) 3.0
- W3C Media Overlays (EPUB Media Overlays 3.3)
- W3C ARIA in EPUB 1.1, W3C Web Accessibility Guidelines 2.2
- W3C Pronunciation Lexicon Specification (PLS) 1.0
- W3C SSML 1.1 (Speech Synthesis Markup Language)
- W3C TTML2, WebVTT 1.0
- IDPF EPUB Open Container Format (OCF) 3.3
- ISO 14289-1:2014 (PDF/UA-1, accessible PDF)
- ISO/IEC 21778:2017 (JSON), IETF RFC 8259, RFC 8785 (JCS)
- ISO/IEC 14496-12 (ISO Base Media File Format)
- ONIX for Books 3.1 (book metadata)
- Schema.org `Book`, `Audiobook`
- Dublin Core ISO 15836-1:2017
- BCP 47 / RFC 5646, ISO 639-3, ISO 15924
- Unicode 15.1, Unicode Braille Patterns block (U+2800..U+28FF)
- ISO 17025 (signage / Braille production calibration, informative)
- IEEE 1857.9 (immersive media coding, informative)
- IETF RFC 4122 (UUID), RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)

---

## §1 Scope

This PHASE applies to records that describe a
publication and its sensory representations. The
publication may be a literary work, a textbook, a
news article, a musical score, a research dataset
narrative, a scriptural reading, or a sovereign-
issued public document.

In scope: publication record, representation
record (one per modality), narration record,
accessibility declaration, cross-format manifest,
language version record, contributor record, and
the cross-references binding each record to its
publisher identity, ONIX metadata, and signed
provenance.

Out of scope: editorial workflows in publishing
houses (governed by the publisher's CMS); rights
clearance and royalty accounting (governed by the
publisher's contract systems).

## §2 Publication record

Every publication carries:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `publicationRef`     | UUID (RFC 4122) opaque                          |
| `isbn`               | ISO 2108 ISBN-13 when applicable                |
| `doi`                | DOI when issued                                 |
| `title`              | localised title (BCP 47 keys)                   |
| `subtitle`           | localised subtitle                              |
| `creator[]`          | contributor records (this PHASE §7)             |
| `publisher`          | publisher identity URI                          |
| `publishedAt`        | ISO 8601                                        |
| `languages[]`        | BCP 47 tags of available language versions      |
| `genre`              | ONIX List 91 / sovereign-equivalent             |
| `audience`           | ONIX List 28                                    |
| `licenseRef`         | SPDX or sovereign equivalent                    |
| `signingKeyRef`      | JWKS URL                                        |

`publicationRef` is the only invariant identifier
across edition revisions; ISBN may change between
print and ebook editions.

## §3 Representation record

A representation is a complete sensory rendering of
the publication.

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `representationRef`  | URI                                             |
| `publicationRef`     | this PHASE §2                                   |
| `modality`           | `visual`, `auditory`, `tactile`, `spatial`,     |
|                      | `gestural`                                       |
| `format`             | per-modality format (see §3.1)                  |
| `language`           | BCP 47 tag                                      |
| `accessibilityRef`   | this PHASE §5                                   |
| `digestRef`          | SHA-512 of the canonical artefact                |
| `signature`          | RFC 7515 detached JWS                           |
| `runtime`            | runtime requirements (e.g., reader, app, device) |

### 3.1 Per-modality formats

| Modality   | Canonical formats                               |
|------------|-------------------------------------------------|
| Visual     | EPUB 3.3, PDF/UA-1, HTML5, Web Publication      |
| Auditory   | Audiobook (W3C), MP3, AAC, FLAC, Opus, SSML 1.1 |
| Tactile    | BRF (Braille Ready Format), Unified English      |
|            | Braille (UEB), Korean Braille (KS X 1026), 3-D  |
|            | tactile graphics (BANA tactile graphic guidelines)|
| Spatial    | glTF 2.0, USDZ, OpenXR scene graph, IEEE 1857.9 |
| Gestural   | sign-language video (ISO 639-3 sign codes),     |
|            | SignWriting per Sutton, motion-capture BVH      |

Modalities are equally authoritative; the
publisher's intent is that consumers may choose
any representation without diminished content
fidelity.

## §4 Narration record

Narration binds visual content to its auditory or
gestural rendering.

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `narrationRef`       | UUID                                            |
| `representationRef`  | this PHASE §3                                   |
| `mediaOverlayRef`    | EPUB Media Overlays 3.3 reference (SMIL)        |
| `voice`              | voice-talent identity or synthesiser model      |
| `pronunciationRef`   | W3C PLS 1.0 reference                           |
| `ssmlRef`            | SSML 1.1 reference for synthetic narration      |
| `signLanguageRef`    | sign-language video reference                   |
| `synchronisation`    | per-paragraph or per-sentence granularity       |

## §5 Accessibility declaration

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `accessibilityRef`   | URI                                             |
| `representationRef`  | this PHASE §3                                   |
| `epubAccessibility`  | EPUB Accessibility 1.1 conformance level         |
|                      | (`a`, `aa`, `aaa`)                              |
| `wcagConformance`    | WCAG 2.2 conformance level                      |
| `pdfUaConformance`   | ISO 14289-1 conformance flag                    |
| `ace`                | Ace by DAISY conformance evidence URI            |
| `mathML`             | boolean — MathML 4 used                         |
| `imageDescriptions`  | `none`, `partial`, `complete`                   |
| `colourContrast`     | `WCAG-AA`, `WCAG-AAA`                            |
| `dyslexiaFriendly`   | optional declared dyslexia-friendly font and    |
|                      | spacing settings                                 |

## §6 Cross-format manifest

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `manifestRef`        | URI                                             |
| `publicationRef`     | this PHASE §2                                   |
| `representations[]`  | per-modality representationRef list             |
| `defaultModality`    | `none`; the publisher MUST NOT impose a default |
| `equivalenceClaim`   | `verified-equivalent`, `partial-equivalent`,    |
|                      | `derivative`                                     |

The default modality is intentionally `none`: this
standard does not designate a "fallback"; every
modality is authored as primary.

## §7 Contributor record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `contributorRef`     | UUID                                            |
| `name`               | localised name (BCP 47 keys)                    |
| `role`               | ONIX List 17 contributor role                   |
| `did`                | optional DID per W3C DID 1.0                    |
| `attribution`        | display attribution string                      |

## §8 Language version record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `languageVersionRef` | URI                                             |
| `publicationRef`     | this PHASE §2                                   |
| `language`           | BCP 47 tag                                      |
| `script`             | ISO 15924 four-letter code                      |
| `region`             | ISO 3166-1 alpha-2                              |
| `translatorRef[]`    | contributor references                          |

Language versions share `publicationRef`; each
version carries its own representation set.

## §9 Cross-domain references (informative)

- WIA-language-bridge — translation provenance
- WIA-prompts — prompt-mediated narration
- WIA-multiverse-interface — XR/spatial reading
- WIA-plugins — reading-app plugin extensions

## Annex A — Conformance disclosure

Implementations declare the schema versions they
support, the canonicalisation form (RFC 8785), and
the JWS key set used to sign publication and
representation records.

## Annex B — Worked publication record (informative)

```json
{
  "publicationRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "isbn": "979-11-90000-12-3",
  "title": {"ko": "삼국지 365", "en": "Three Kingdoms 365"},
  "publisher": "https://wiabook.com",
  "publishedAt": "2026-04-27"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with EPUB / ONIX major revisions.

## Annex D — Conformance level

Conformance is "Core" (publication + at least two
modality representations + accessibility
declaration) or "Full" (adds narration,
cross-format manifest with verified equivalence,
and language version records).

## Annex E — Privacy

Consumer reading analytics are out of scope; the
publisher's reading-app records analytics under
the app's own privacy regime.

## Annex F — Equal-representation principle

The standard refuses to designate a "default"
modality. Reading apps MAY surface the modality
the user previously chose, but no modality is
authoritatively privileged over another. This
principle is enforced by the
`defaultModality: none` constraint in §6.

## Annex G — Music notation interoperability

Music publications carry a music notation
representation (MusicXML 4.0, MNX-Common, or
MEI 5.0) alongside visual / auditory / tactile
modalities. Music notation is treated as a
specialised visual modality with auxiliary
auditory rendering via SSML 1.1 or MIDI.

## Annex H — Mathematical content

Mathematical expressions use MathML 4 in EPUB
representations and may be rendered to:

- LaTeX-source for fallback;
- Nemeth Braille (en-US) or sovereign-equivalent
  mathematical Braille for tactile;
- speech-rule-based audio narration for auditory.

Equation accessibility evidence references the
authoring tool (e.g., MathJax accessibility,
Pearson MathML reader).

## Annex I — Image description authoring

Images carry alt text per WCAG 2.2 AA. Long-form
descriptions use the `aria-describedby`
mechanism in EPUB and the `<Figure>` /
`<Caption>` / `<P>` flow in PDF/UA. Tactile
graphics are produced per BANA tactile graphic
guidelines.

## Annex J — Reading order metadata

Reading order is recorded in the EPUB Package
Document `<spine>`. Web Publication manifests
declare the reading order in the
`readingOrder` array. Tactile and spatial
representations may diverge from the visual
reading order; the divergence is recorded with
the equivalence claim.

## Annex K — Voice characteristics

Synthetic narration declares voice
characteristics:

| Field          | Source / Binding                              |
|----------------|-----------------------------------------------|
| `voiceFamily`  | language-and-region voice family               |
| `gender`       | `male`, `female`, `neutral`, `unspecified`    |
| `ageRange`     | broad age category                            |
| `speakingRate` | words per minute                              |
| `pitch`        | semitone offset relative to neutral           |

Voice characteristics support per-character
narration consistency across chapters.

弘益人間 (Hongik Ingan) — Benefit All Humanity
