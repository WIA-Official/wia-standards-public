# WIA-data-visualization PHASE 3 — Protocol Specification

**Standard:** WIA-data-visualization
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a data-visualization operator across the
specification-to-rendering-to-accessibility-to-
archival value chain: the W3C SVG 2 rendering
discipline that anchors every per-chart
canonical artefact, the W3C WAI-ARIA 1.2
accessibility discipline that gates every per-
chart accessibility profile, the WCAG 2.2
conformance discipline that scores every per-
chart accessibility outcome, the ISO 9241-303
electronic-visual-display discipline that
anchors the per-deployment display envelope,
the ICC.1:2010 colour-management discipline,
the CIE 15:2018 colorimetric discipline, the
graphical-encoding-faithfulness discipline that
prevents distortion, the per-chart provenance
discipline, the chain-of-custody anchoring
discipline, and the per-jurisdiction
accessibility-framework discipline.

References (CITATION-POLICY ALLOW only):

- W3C SVG 2, W3C SVG 1.1 Second Edition, W3C
  WAI-ARIA 1.2, W3C WCAG 2.2, W3C HTML Living
  Standard, W3C CSS Color Module Level 4, W3C
  JSON-LD 1.1, W3C SHACL
- ISO 9241-303:2011, ISO 9241-307:2008, ISO
  9241-306:2018, ISO 9241-110:2020, ISO 9241-
  210:2019, ISO 14915 series, IEC 61506:1997
- ICC.1:2010, ICC.2:2019, CIE 15:2018
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- IETF RFC 9110, RFC 9421, RFC 9457, RFC
  6234, RFC 8615, RFC 6962
- W3C Trace Context
- EN 301 549 v3.2.1, US Section 508, KR 한국
  형 웹 콘텐츠 접근성 지침, KR 정보접근성 보장에
  관한 법률, KR 장애인차별금지 및 권리구제 등에
  관한 법률

---

## §1 W3C SVG 2 Rendering Discipline

### §1.1 Per-chart canonical artefact

Every per-chart canonical artefact is published
as a W3C SVG 2 document. The operator's API
validates the per-chart SVG document against
the W3C SVG 2 grammar; an invalid document is
rejected.

### §1.2 Per-chart presentation-attribute discipline

The operator's API enforces the per-chart
presentation-attribute discipline — the per-
element `aria-label`, `aria-labelledby`, and
`role` attributes per W3C WAI-ARIA 1.2; the
per-element CSS-encoded styling per W3C CSS
Color Module Level 4.

### §1.3 Per-chart text-element discipline

A per-chart axis label, a per-chart legend
entry, and a per-chart annotation MUST be
encoded as a `<text>` element so that an
assistive-technology consumer can read the
text envelope.

## §2 W3C WAI-ARIA Accessibility Discipline

### §2.1 Per-chart ARIA pattern

Every per-chart ARIA pattern is bound to one
of the W3C WAI-ARIA Graphics Module patterns
— `graphics-document`, `graphics-symbol`,
`graphics-object`, `img`. The operator's API
records the per-chart ARIA pattern.

### §2.2 Per-chart alternative-text discipline

The per-chart `altText` envelope is non-empty
and carries the per-chart descriptive
alternative text. The per-chart
`longDescription` envelope carries the per-
chart extended description used by assistive-
technology consumers requiring more context
than the alternative text.

### §2.3 Per-chart data-table-fallback discipline

The per-chart `dataTableFallback` envelope
is published as an HTML `<table>` per WCAG
2.2 SC 1.3.1 Info and Relationships so that
the per-chart underlying data is accessible
through tabular reading.

## §3 WCAG 2.2 Conformance Discipline

### §3.1 Per-chart contrast threshold

The per-chart text-and-background contrast
ratio meets or exceeds the WCAG 2.2 SC 1.4.3
threshold (4.5:1 for normal-size text, 3:1
for large-size text per the standard's
definitions). A per-chart `WCAG-2.2-AAA`
declaration meets the SC 1.4.6 threshold
(7:1 / 4.5:1).

### §3.2 Per-chart non-text-contrast threshold

The per-chart non-text-contrast (per WCAG
2.2 SC 1.4.11) meets or exceeds the 3:1
threshold for graphical objects necessary to
understand the chart.

### §3.3 Per-chart focus-visible discipline

A per-chart interactive element (a per-chart
tooltip trigger, a per-chart selection
control) carries a per-element focus-visible
indicator per WCAG 2.2 SC 2.4.7 and SC
2.4.13.

## §4 ISO 9241-303 Electronic-Visual-Display Discipline

### §4.1 Per-deployment display envelope

The operator's API records the per-deployment
display envelope per ISO 9241-303:2011 — the
per-display luminance, chromaticity, contrast,
gamma, resolution, and viewing-angle envelope.

### §4.2 Per-deployment field-assessment

The operator's per-deployment field assessment
is published per ISO 9241-306:2018 to verify
the per-display envelope under the per-
deployment ambient-light conditions.

## §5 ICC.1:2010 Colour-Management Discipline

### §5.1 Per-chart colour profile

Every per-chart colour profile is bound to an
ICC.1:2010 profile (an sRGB profile, a Display-
P3 profile, a Rec.2020 profile, a per-deployment
custom profile). The operator's API records
the per-chart profile reference.

### §5.2 Per-chart colorimetric intent

The per-chart colorimetric intent is one of
the ICC.1:2010 enumerated intents —
perceptual, relative-colorimetric, saturation,
absolute-colorimetric. The operator's API
records the per-chart intent declaration.

## §6 CIE 15:2018 Colorimetric Discipline

A per-chart palette declares the per-stop
CIE-LAB coordinates per CIE 15:2018. The
operator's API computes the per-stop CIE-LAB
distance to verify the per-palette
discriminability for colour-blind consumers
(per the operator's protanopia, deuteranopia,
and tritanopia simulation discipline).

## §7 Graphical-Encoding-Faithfulness Discipline

### §7.1 Per-channel scale faithfulness

Every per-channel encoding's scale declaration
is faithful to the underlying data — a per-
channel `position` encoding uses a linear
scale unless the per-channel data declaration
explicitly justifies a non-linear scale; a
per-channel `area` encoding is bound to a
square-root scale so that the visual area is
proportional to the underlying value.

### §7.2 Per-chart axis-truncation discipline

A per-chart axis-truncation is permitted only
where the per-chart documented editorial-
discipline justifies the truncation. The
operator's API records the per-chart
truncation envelope so that a downstream
audit can review the per-chart truncation
decision.

### §7.3 Per-chart legend-and-annotation discipline

The per-chart legend declares the per-channel
encoding mapping; the per-chart annotation
declares the per-data-point context. Both are
encoded as text elements so that the per-chart
narrative is accessible.

## §8 Per-Chart Provenance Discipline

Every per-chart provenance record carries the
per-source data-provenance envelope, the per-
chart authorship envelope, and the per-chart
measurement-uncertainty envelope. The
operator's API publishes the per-chart
provenance record alongside the per-chart
canonical artefact.

## §9 Chain-of-Custody Anchoring Discipline

### §9.1 Per-event transparency log

Every chain-of-custody event carried by
PHASE-1 §8 is appended to a per-operator
transparency log modelled on the IETF RFC 6962
Certificate Transparency append-only-log
structure.

### §9.2 Mutation prevention

A custody event cannot be retroactively
edited; an amendment is recorded as a new
event with `previousEventRef` pointing at the
event being amended.

## §10 Per-Jurisdiction Accessibility Framework Discipline

### §10.1 EU EN 301 549 binding

A EU-procurement-context operator binds the
per-chart accessibility profile to the EN
301 549 v3.2.1 envelope per the EU Web
Accessibility Directive (Directive (EU)
2016/2102) and the EU European Accessibility
Act (Directive (EU) 2019/882).

### §10.2 US Section 508 binding

A US-federal-agency operator binds the per-
chart accessibility profile to the US
Section 508 envelope per the US Access Board's
ICT Final Rule.

### §10.3 KR accessibility binding

A KR-jurisdiction operator binds the per-
chart accessibility profile to the KR 한국형
웹 콘텐츠 접근성 지침 (Korean Web Content
Accessibility Guidelines) and the KR 정보
접근성 보장에 관한 법률 (Information
Accessibility Guarantee Act).

## §11 Per-Chart Internationalisation Discipline

The per-chart text envelope is declared with
the per-locale language tag per BCP 47. The
operator's API publishes the per-chart per-
locale rendering so that a downstream consumer
in a different locale receives the per-locale
narrative.

## §12 Per-Chart Lifecycle Discipline

### §12.1 Per-chart deprecation

A per-chart deprecation is signalled by a per-
chart deprecation envelope with a per-chart
sunset window. The per-chart consumer is
expected to migrate to the successor per-chart
within the sunset window.

### §12.2 Per-chart archival

A per-chart archived state preserves the per-
chart canonical artefact, the per-chart
provenance record, and the per-chart
accessibility profile so that a downstream
archival system can retrieve the per-chart
envelope after the per-chart deprecation.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §13 Per-Chart Animation Discipline

A per-chart animated sequence (a per-chart
transition, a per-chart timeline scrubber, a
per-chart auto-playing animation) is bound to
WCAG 2.2 SC 2.2.2 (Pause, Stop, Hide) and SC
2.3.1 (Three Flashes or Below Threshold). The
operator's API enforces a per-animation
controls envelope so that a consumer can
pause, stop, or hide the animation; a per-
animation flash-frequency exceeding the SC
2.3.1 threshold is rejected.

## §14 Per-Chart Interactive-Element Discipline

A per-chart interactive element (a per-chart
tooltip trigger, a per-chart selection
control, a per-chart drill-down link) is
bound to WCAG 2.2 SC 2.1.1 (Keyboard) so that
the element is operable via keyboard alone. A
per-chart pointer-only interaction is rejected.

## §15 Per-Chart Reduced-Motion Discipline

A per-chart deployment honours the consumer's
`prefers-reduced-motion` CSS media query per
W3C Media Queries Level 5; a consumer with
the preference set receives a per-chart
static rendering rather than an animated
rendering.

## §16 Per-Chart Internationalised Number-and-Date Formatting

A per-chart numeric or date-time encoding is
formatted per the per-locale CLDR (Unicode
Common Locale Data Repository) rules. The
operator's API records the per-chart per-
locale formatting envelope so that a downstream
consumer in a different locale receives the
per-locale-correct formatting.

## §17 Per-Chart Right-to-Left Layout Discipline

A per-chart layout for a right-to-left locale
(Arabic, Hebrew, Persian, Urdu) honours the
per-locale layout discipline — the per-chart
axis order, the per-chart legend order, the
per-chart annotation alignment — per W3C HTML
Living Standard `dir="rtl"` and W3C CSS
Logical Properties.

## §18 Per-Chart High-Contrast Mode Discipline

A per-chart deployment honours the consumer's
operating-system high-contrast mode (the
Windows High Contrast theme, the macOS
Increase Contrast preference, the Linux high-
contrast theme) per W3C CSS Media Queries
Level 5 `forced-colors` query. A consumer
with the preference set receives a per-chart
rendering using the operating-system colour
palette rather than the per-chart custom
palette.
