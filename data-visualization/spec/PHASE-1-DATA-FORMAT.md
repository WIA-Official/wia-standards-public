# WIA-data-visualization PHASE 1 — Data Format Specification

**Standard:** WIA-data-visualization
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-data-visualization. The standard
covers the persistent record shapes that a
data-visualization operator (an enterprise
business-intelligence platform vendor, a
public-sector open-data dashboard publisher, a
news-media interactive-visualization operator,
a scientific-visualization platform operator
publishing per-experiment visual artefacts, an
inclusive-visualization platform delivering
W3C WAI-ARIA-compliant accessible charts, an
industrial-information-presentation platform
under IEC 61506, an SVG-rendering library
publisher) maintains when registering a
visualization specification, declaring the
per-chart graphical-encoding choices, anchoring
the per-chart accessibility profile under W3C
WAI-ARIA and ISO 9241-303, recording the per-
chart colour-space declaration under ICC and
CIE-LAB references, and tracking the per-chart
chain-of-custody. Records are consumed by the
downstream presentation surface (a web
browser, a mobile application, a public-
display kiosk, a print artefact), by the
inclusive-content team verifying the per-chart
WCAG 2.2 AA compliance, by the editorial-
quality team verifying the per-chart graphical-
encoding faithfulness, and by the per-domain
audit committee approving the per-period
publication.

References (CITATION-POLICY ALLOW only):

- W3C SVG 2 (Scalable Vector Graphics 2 — W3C
  Candidate Recommendation), W3C SVG 1.1
  Second Edition (W3C Recommendation 2011-08-
  16)
- W3C WAI-ARIA 1.2 (Accessible Rich Internet
  Applications), W3C WCAG 2.2 (Web Content
  Accessibility Guidelines)
- W3C HTML Living Standard, W3C CSS Color
  Module Level 4
- ISO 9241-303:2011 (ergonomics of human-
  system interaction — requirements for
  electronic visual displays), ISO 9241-307:
  2008 (analysis and compliance test methods
  for electronic visual displays), ISO 9241-
  306:2018 (field assessment methods for
  electronic visual displays), ISO 9241-110:
  2020 (interaction principles), ISO 9241-210:
  2019 (human-centred design for interactive
  systems)
- ISO 14915-1:2002 / 14915-2:2003 / 14915-3:
  2002 (software ergonomics for multimedia
  user interfaces — design principles, multi-
  media navigation, media selection)
- IEC 61506:1997 (industrial-process
  measurement and control — documentation of
  application software)
- ICC.1:2010 (image technology colour
  management — architecture, profile format,
  and data structure), ICC.2:2019
  (specification — iccMAX)
- CIE-LAB / CIE-XYZ colour-space references
  (CIE 15:2018 — Colorimetry, 4th edition)
- Vega and Vega-Lite (the academically-
  reviewed JSON-encoded grammar of graphics
  reference; the operator may declare a Vega-
  Lite-anchored encoding choice)
- W3C JSON-LD 1.1, W3C SHACL (the per-
  visualization metadata-validation envelope)
- IEEE Std 11073-10101 (medical-device
  nomenclature — used for the per-medical-
  visualization metric binding where
  applicable)
- IETF RFC 8259 (JSON), RFC 4122 (UUID),
  ISO 8601 (date-time)
- ISO/IEC 27001:2022 (information security
  management)
- EN 301 549 v3.2.1 (accessibility
  requirements for ICT products and services
  — applied as the EU public-procurement
  accessibility envelope)
- US Section 508 of the Rehabilitation Act
  (29 U.S.C. § 794d) — applied as the US
  federal-agency accessibility envelope
- KR 한국형 웹 콘텐츠 접근성 지침 (Korean Web
  Content Accessibility Guidelines), KR 정보
  접근성 보장에 관한 법률, KR 장애인차별금지
  및 권리구제 등에 관한 법률

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a data-visualization
operator registers a visualization
specification, declares the per-chart graphical-
encoding choices anchored to a per-chart
grammar-of-graphics envelope, anchors the per-
chart accessibility profile under W3C WAI-ARIA
1.2 and ISO 9241-303, declares the per-chart
colour-space envelope under ICC and CIE-LAB,
and tracks the per-chart chain of custody.
Implementations covered include:

- An enterprise business-intelligence platform
  (a per-tenant dashboard product) publishing
  the per-tenant per-dashboard visualization
  set with the per-chart accessibility
  envelope declared.
- A public-sector open-data dashboard publisher
  publishing the per-period statistics
  dashboard with the per-jurisdiction
  accessibility envelope (the EN 301 549
  envelope for the EU, the Section 508
  envelope for the US, the KR 한국형 웹
  콘텐츠 접근성 지침 envelope for KR).
- A news-media interactive-visualization
  operator publishing the per-story
  interactive chart with the per-chart
  graphical-encoding-faithfulness declaration
  and the per-chart citation envelope.
- A scientific-visualization platform operator
  publishing the per-experiment visual
  artefact (a per-experiment microscopy image,
  a per-experiment graph, a per-experiment 3D
  rendering) with the per-artefact
  measurement-uncertainty envelope.
- An inclusive-visualization platform
  delivering accessible charts to learners
  with visual impairments, hearing
  impairments, motor-coordination differences,
  or cognitive differences per the per-
  jurisdiction accessibility framework.
- An industrial-information-presentation
  platform under IEC 61506:1997 documenting
  the per-process measurement-and-control
  visualization envelope.
- An SVG-rendering library publisher
  publishing the per-library W3C SVG 2
  conformance envelope.

The W3C SVG envelope, the W3C WAI-ARIA 1.2
accessibility envelope, the ICC colour-space
envelope, and the per-domain accessibility-
framework envelope receive distinct encodings
in this PHASE; the additional safeguards
required by each domain are encoded in PHASE-
3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — BI platform
                       vendor, open-data dashboard
                       publisher, news-media
                       operator, scientific-
                       visualization operator,
                       inclusive-visualization
                       provider, industrial-
                       presentation operator,
                       SVG-rendering library
                       publisher)
operatorRole         : enum ("bi-platform" |
                       "open-data-dashboard" |
                       "news-media" |
                       "scientific-visualization"
                       | "inclusive-visualization"
                       | "industrial-presentation"
                       | "svg-library" | "user-
                       defined")
governingFrameworks  : array of enum ("W3C-SVG-2"
                       | "W3C-SVG-1.1" |
                       "W3C-WAI-ARIA-1.2" |
                       "W3C-WCAG-2.2" |
                       "W3C-HTML-Living" |
                       "W3C-CSS-Color-4" |
                       "ISO-9241-303" |
                       "ISO-9241-307" |
                       "ISO-9241-306" |
                       "ISO-9241-110" |
                       "ISO-9241-210" |
                       "ISO-14915-1" |
                       "ISO-14915-2" |
                       "ISO-14915-3" |
                       "IEC-61506" |
                       "ICC-1-2010" |
                       "ICC-2-2019" |
                       "CIE-15-2018" |
                       "Vega-Lite" |
                       "EN-301-549-3.2.1" |
                       "US-Section-508" |
                       "KR-WCAG-Korean" |
                       "user-defined")
accreditationStatus  : object (the ISO/IEC 27001
                       certification reference,
                       the per-jurisdiction
                       accessibility-conformance
                       reference, the per-rendering-
                       library W3C SVG conformance
                       reference)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Visualization-Specification Record

```
vizSpec:
  vizSpecId          : string (uuidv7)
  programmeRef       : string (PHASE-1 §2 record
                       reference)
  vizKind            : enum ("bar-chart" | "line-
                       chart" | "scatter-plot" |
                       "histogram" | "box-plot" |
                       "violin-plot" | "heatmap"
                       | "treemap" | "sankey" |
                       "choropleth" | "network-
                       graph" | "3d-rendering" |
                       "animated-sequence" |
                       "small-multiples" | "user-
                       defined")
  vegaLiteSpecRef    : string (the per-chart
                       Vega-Lite specification
                       reference, where the
                       operator declares a Vega-
                       Lite-anchored encoding)
  svgRef             : string (the per-chart W3C
                       SVG 2 document reference,
                       where the operator
                       publishes a per-chart SVG
                       artefact)
  encodingChannels   : array of object (per-
                       channel encoding —
                       position-x, position-y,
                       size, colour, shape,
                       opacity, texture, motion;
                       each carrying the per-
                       channel data-binding and
                       the per-channel scale
                       declaration)
  dataSourceRef      : string (the per-source
                       reference per WIA-data-
                       integration PHASE-1 §3 or
                       an equivalent per-source
                       declaration)
```

## §4 Accessibility Profile Record

```
accessibilityProfile:
  accessibilityProfileId : string (uuidv7)
  vizSpecRef         : string (PHASE-1 §3 record
                       reference)
  wcagConformance    : enum ("WCAG-2.2-A" |
                       "WCAG-2.2-AA" |
                       "WCAG-2.2-AAA")
  ariaPattern        : enum (per W3C WAI-ARIA
                       1.2 — "img" | "graphics-
                       document" | "graphics-
                       symbol" | "graphics-
                       object" | "user-defined")
  altText            : string (the per-chart
                       descriptive alternative
                       text, used by screen
                       readers)
  longDescription    : string (the per-chart
                       extended description
                       carrying the chart's
                       narrative summary)
  dataTableFallback  : object (the per-chart
                       tabular fallback
                       representation per WCAG
                       2.2 SC 1.3.1 Info and
                       Relationships)
  colourBlindSafe    : boolean (whether the
                       per-chart colour palette
                       is colour-blind safe per
                       the operator's CIE-LAB
                       distance discipline)
  contrastRatio      : number (the per-chart
                       text-and-background
                       contrast ratio per WCAG
                       2.2 SC 1.4.3 / 1.4.6)
```

## §5 Colour-Space Record

```
colourSpaceRecord:
  colourSpaceId      : string (uuidv7)
  vizSpecRef         : string (PHASE-1 §3 record
                       reference)
  iccProfileRef      : string (the per-chart ICC
                       profile reference per
                       ICC.1:2010, where the
                       chart targets a per-
                       device colour-managed
                       presentation surface)
  workingSpace       : enum ("sRGB" | "Display-
                       P3" | "Rec.2020" |
                       "Adobe-RGB-1998" | "ProPhoto-
                       RGB" | "user-defined")
  colorimetricIntent : enum ("perceptual" |
                       "relative-colorimetric"
                       | "saturation" |
                       "absolute-colorimetric")
  cieLABReference    : object (the per-palette
                       CIE-LAB coordinates per
                       CIE 15:2018 for the per-
                       chart colour stops)
```

## §6 Display-Calibration Record

```
displayCalibration:
  displayCalibrationId : string (uuidv7)
  vizSpecRef         : string (PHASE-1 §3 record
                       reference)
  iso9241303Profile  : object (per ISO 9241-303,
                       the per-display luminance,
                       chromaticity, contrast,
                       gamma, resolution, viewing-
                       angle envelope)
  measurementMethod  : enum (per ISO 9241-307,
                       the per-measurement
                       method declaration —
                       "luminance-measurement" |
                       "chromaticity-measurement"
                       | "contrast-measurement"
                       | "user-defined")
  fieldAssessmentRef : string (per ISO 9241-306,
                       the per-deployment field-
                       assessment reference)
```

## §7 Per-Chart Provenance Record

```
provenanceRecord:
  provenanceId       : string (uuidv7)
  vizSpecRef         : string (PHASE-1 §3 record
                       reference)
  dataProvenance     : object (the per-chart
                       data-provenance envelope
                       — the per-source upstream
                       data, the per-source
                       refresh timestamp, the
                       per-source license
                       envelope)
  authorshipRecord   : object (the per-chart
                       authorship — the per-
                       chart author identifier,
                       the per-chart editorial
                       review reference, the per-
                       chart citation set)
  uncertaintyRecord  : object (the per-chart
                       measurement-uncertainty
                       envelope — the per-chart
                       confidence interval
                       declaration, the per-chart
                       error-bar declaration,
                       the per-chart sample-
                       size declaration)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the visualization-
                       spec, accessibility-
                       profile, colour-space,
                       display-calibration, or
                       provenance identifier)
  custodyEvent       : enum ("vizspec-published"
                       | "accessibility-attested"
                       | "colour-space-bound" |
                       "display-calibrated" |
                       "provenance-recorded" |
                       "rendering-published" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "data-
visualization"), `version`, `implementation`,
the operator's `accreditationStatus`, and the
`profile` declaration that selects which of the
optional records (Vega-Lite, SVG, accessibility,
colour-space, display-calibration, provenance)
the implementation supports. The manifest is
signed using a key whose public part is
published on the operator's
`.well-known/wia/data-visualization/`
discovery endpoint declared in PHASE-2.

弘益人間 (Hongik Ingan) — Benefit All Humanity
