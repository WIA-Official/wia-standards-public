# WIA-pubscript PHASE 4 — Integration Specification

**Standard:** WIA-pubscript
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-pubscript integrates
with adjacent ecosystems (publishers, distribution
platforms, reading apps, accessibility auditors,
sovereign legal-deposit registries, and downstream
WIA standards), how conformance evidence is
produced, and how deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO/IEC 27001:2022, ISO/IEC 27701:2019
- ISO 14289-1:2014 (PDF/UA-1), ISO 2108 (ISBN), ISO 15836-1
- ISO 639-3, ISO 15924, BCP 47 / RFC 5646
- W3C EPUB 3.3, EPUB Accessibility 1.1
- W3C Web Publications, W3C Audiobooks Manifest
- W3C SMIL 3.0, EPUB Media Overlays 3.3
- W3C TTML2, WebVTT 1.0, SSML 1.1, PLS 1.0
- W3C WCAG 2.2, W3C VC 2.0
- ONIX for Books 3.1, Schema.org `Book`
- DAISY Consortium Ace, Korea Braille KS X 1026

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-pubscript consumes
upstream specifications (EPUB, PDF/UA, ONIX, BCP
47), how downstream WIA standards reference
publications, and how conformance is assessed.

## §2 Upstream integration

### 2.1 W3C / IDPF

EPUB 3.3, Audiobooks Manifest, Web Publications,
Media Overlays 3.3, ARIA in EPUB 1.1 are consumed
at their published versions.

### 2.2 ISO

PDF/UA-1 (ISO 14289-1) is the canonical accessible
PDF profile. ISBN (ISO 2108) is consumed
unchanged.

### 2.3 ONIX / Schema.org

ONIX for Books 3.1 is the canonical metadata
exchange format. Schema.org provides the
discoverability layer for web search.

### 2.4 DAISY Consortium

The Ace by DAISY accessibility checker provides
the standard evidence package referenced from
the §5 accessibility declaration.

## §3 Reading-app integration

Reading apps fetch the publication's representation
set and surface a modality picker to the user.
The app does not designate a default modality;
it surfaces the user's last choice as the
returning state and lets the user switch freely.

## §4 Distribution platform integration

Distribution platforms (Apple Books, Google Play
Books, Kobo, Storytel, Audible, sovereign-
equivalent stores) consume the publication record
plus the modality representations. Per-platform
DRM is layered on top of the canonical artefacts
without altering their signatures.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared EPUB / PDF / Audiobook / Braille / 3D /
  sign-video versions;
- declared accessibility evidence (Ace by DAISY,
  WCAG checker output);
- declared cross-format equivalence claim;
- the test-vector matrix per Annex G of each
  PHASE;
- the JWS signing key set used.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. EPUB packages validate against EPUB 3.3
   schema.
2. PDF/UA artefacts validate against ISO 14289-1.
3. Accessibility declarations match the
   authoritative checker output.
4. Equivalence claim is reproducible across
   modalities.
5. Sign-language video covers the same content
   as the visual representation.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                  | Integration point                       |
|---------------------------|-----------------------------------------|
| WIA-language-bridge       | translation provenance                  |
| WIA-prompts               | prompt-mediated narration               |
| WIA-multiverse-interface  | XR / spatial reading environments       |
| WIA-plugins               | reading-app plugin extensions           |
| WIA-language-learning     | textbook-style proficiency content      |

## §7 Privacy

Personal data appearing in publications (author
attribution, dedications, contributor identities)
is processed under the publisher's privacy
regime. Reader-app analytics are out of scope.

## §8 Security

Publisher signing key compromise triggers
immediate JWKS rotation and a public revocation
list update. Affected publications are
republished with new signatures.

## §9 Sovereign legal-deposit

Sovereign legal-deposit registries (KORLI for KR,
BNF for FR, BL for UK, LoC for US, NDL for JP)
mirror conformant publications via PHASE-2 Annex
L. The legal-deposit copy preserves the
publisher's signatures verbatim.

## §10 Localisation

Publications support multiple language versions
sharing a `publicationRef`. Translators are
catalogued as contributors with role per ONIX
List 17.

## §11 Accessibility

WCAG 2.2 AA is the floor; AAA is encouraged.
The standard's equal-representation principle
extends accessibility beyond legacy expectations:
sign-language and tactile modalities are
authored as primary, not as accommodations.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence
URL from the README.

## Annex B — Worked manifest (informative)

```json
{
  "manifestRef": "https://reg.example.org/manifests/foo-1.0",
  "publicationRef": "https://reg.example.org/pubs/foo",
  "representations": [
    "https://reg.example.org/repr/foo-epub",
    "https://reg.example.org/repr/foo-audiobook",
    "https://reg.example.org/repr/foo-braille",
    "https://reg.example.org/repr/foo-spatial",
    "https://reg.example.org/repr/foo-sign-en"
  ],
  "defaultModality": "none",
  "equivalenceClaim": "verified-equivalent"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with EPUB / PDF/UA / ONIX major
revisions.

## Annex D — Open governance

Issues at
`github.com/WIA-Official/wia-standards/issues`
with the `pubscript` label.

## Annex E — Withdrawal procedure

Tombstone the evidence package; tombstones are
immutable.

## Annex F — Reproducibility

Evidence is reproducible from publicly available
inputs: EPUB / PDF / audiobook / Braille /
spatial / sign-video artefacts; accessibility
checker output; signing key set.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative
vector under `tests/phase-vectors/`.

## Annex H — Sustainability

Distribution platforms SHOULD declare their
estimated per-download energy estimate. Estimates
feed into sovereign-equivalent green publishing
reports.

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Modality lock-in                      | `defaultModality: none`   |
| Publisher key compromise              | Immediate JWKS rotation   |
| Accessibility regression              | WCAG checker required     |
| Equivalence claim drift               | Annex K audit             |
| ISBN collision                        | Registry uniqueness check |
| Sign-language coverage gap            | Audit at conformance      |
| Braille code drift                    | Code-version pin          |

## Annex J — Industry binding catalogue

| Segment       | Bound profile                              |
|---------------|--------------------------------------------|
| Trade books   | EPUB 3.3 + Audiobook (W3C)                 |
| Education     | EPUB 3.3 + cmi5 link to LMS                |
| Government    | PDF/UA-1 + sovereign legal-deposit         |
| Children      | EPUB 3.3 + sign-language video             |
| Religion      | EPUB 3.3 + multilingual narration          |

## Annex K — Equivalence audit procedure

The cross-format equivalence claim is verified by
running the auditor's content-comparison harness
across modalities: visual word count vs. audio
duration vs. Braille volume vs. sign-language
running time. Acceptable variance ranges are
defined per genre.

## Annex L — Continuous improvement programme

Each registry publishes an annual improvement
plan addressing modality coverage gaps,
accessibility regressions, and sovereign legal-
deposit compliance.

## Annex M — Reference implementation

A reference implementation is published under
Apache-2.0 at the WIA Standards GitHub umbrella
under `wia-pubscript-reference`, covering the
full PHASE contract including the cross-format
equivalence harness.

## Annex N — Vendor neutrality

WIA-pubscript does not endorse a particular
publishing house, distribution platform, or
reading app. The conformance programme is open
to all implementations on identical terms.

## Annex O — Sovereign deposit authority list

| Region | Authority                            | Standards used         |
|--------|--------------------------------------|------------------------|
| KR     | KORLI (국립중앙도서관 납본)         | EPUB 3.3 + ONIX 3.1    |
| FR     | BNF Dépôt légal                      | EPUB 3.3 + ONIX 3.1    |
| UK     | British Library                      | EPUB 3.3 + ONIX 3.1    |
| US     | Library of Congress                  | EPUB 3.3 + ONIX 3.1    |
| JP     | NDL (国立国会図書館)                | EPUB 3.3 + ONIX 3.1    |
| EU     | EU national libraries (BNL, ÖNB...)  | EPUB 3.3 + ONIX 3.1    |

## Annex P — Annual ecosystem report

The registry publishes an annual ecosystem
report summarising publication counts by
modality, accessibility evidence coverage, sign-
language coverage, sovereign legal-deposit
mirroring rate, and ISBN coverage. The report
is informative.

## Annex Q — Disaster recovery

Registry deployments declare RPO ≤ 24h and
RTO ≤ 8h. DR drills run annually with results
in the audit feed.

## Annex R — Marrakesh Treaty integration

Publishers that opt into the Marrakesh Treaty
exception declare the eligibility flag in the
publication record so that authorised entities
can produce accessible-format copies for
visually impaired readers across borders without
seeking individual permissions.

## Annex S — Multilingual ONIX feed

Publishers expose ONIX 3.1 feeds per language
version of a publication. The feeds align with
distribution platform ingestion conventions and
preserve the sovereign legal-deposit metadata.

## Annex T — Open-source SDK catalogue

Reference SDKs are published per language at
`/v1/registry/sdks` (Python, TypeScript, Java)
under permissive open-source licenses
(Apache-2.0).

## Annex U — Continuous fairness audit

Publications that incorporate AI-assisted
narration (synthetic voices, automated
description generation) declare the AI model
card and an annual fairness audit URL. Audits
cover voice diversity, accent coverage, and
description-quality regressions across content
genres.

## Annex V — Reader privacy commitment

Reading apps that consume the registry's
publications carry a declared privacy
commitment: which interactions are logged,
which leave the device, retention windows, and
opt-out paths. Commitments are catalogued at
`/v1/registry/reader-apps/{appRef}/privacy`.

## Annex W — Equal-representation governance

The standard's equal-representation principle is
governed by the WIA Standards working group.
Vendors that quietly downgrade a non-visual
modality to "fallback" status are reported
through the audit feed; persistent violations
result in tier downgrade.

## Annex X — Annual modality coverage report

The registry publishes an annual modality
coverage report by genre and language: percent
of publications that ship visual / auditory /
tactile / spatial / gestural representations.
The report is informative and is intended to
surface coverage gaps for funder attention.

## Annex Y — Cross-platform DRM neutrality

Distribution platforms apply their own DRM
without altering the publisher's signatures.
Auditors verify the publisher signature on the
unwrapped artefact; the DRM layer is a transport
concern, not a content concern.

弘益人間 (Hongik Ingan) — Benefit All Humanity
