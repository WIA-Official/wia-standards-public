# WIA-data-visualization PHASE 4 — Integration Specification

**Standard:** WIA-data-visualization
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This document defines how a data-visualization
operator integrates with the systems that
surround the specification-to-rendering value
chain: the W3C SVG / WAI-ARIA / WCAG ecosystem
operating the canonical web-rendering layer;
the ISO 9241 ergonomics committee maintaining
the electronic-visual-display reference; the
ICC and CIE colour-management ecosystem; the
EU Web Accessibility Directive supervisory
authority; the US Access Board enforcing
Section 508; the KR 한국정보화진흥원 (NIA)
operating the per-jurisdiction accessibility
audit; the upstream WIA-data-integration
operator providing the per-chart data; the
upstream WIA-content-ai operator providing
the per-chart AI-assisted authoring envelope;
the downstream WIA-lms operator embedding
the per-chart in learning content; and the
inclusive-design certification body.

References (CITATION-POLICY ALLOW only):

- W3C SVG 2, W3C SVG 1.1 Second Edition, W3C
  WAI-ARIA 1.2, W3C WCAG 2.2, W3C HTML Living
  Standard, W3C CSS Color Module Level 4, W3C
  JSON-LD 1.1, W3C SHACL, W3C Verifiable
  Credentials Data Model v2.0
- ISO 9241-303:2011, ISO 9241-307:2008, ISO
  9241-306:2018, ISO 9241-110:2020, ISO 9241-
  210:2019, ISO 14915 series, IEC 61506:1997
- ICC.1:2010, ICC.2:2019, CIE 15:2018
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- EN 301 549 v3.2.1, US Section 508
- EU Web Accessibility Directive (Directive
  (EU) 2016/2102), EU European Accessibility
  Act (Directive (EU) 2019/882)
- KR 한국형 웹 콘텐츠 접근성 지침, KR 정보
  접근성 보장에 관한 법률, KR 장애인차별금지
  및 권리구제 등에 관한 법률
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421
- W3C Trace Context

---

## §1 W3C SVG / WAI-ARIA / WCAG Ecosystem Integration

The operator subscribes to the W3C SVG 2
publishing endpoint, the W3C WAI-ARIA 1.2
publishing endpoint, and the W3C WCAG 2.2
publishing endpoint. A new W3C publication
triggers an internal review cycle in the
operator's quality-management discipline before
the new revision is bound into the operator's
per-chart canonical envelope.

## §2 ISO 9241 Ergonomics Committee Integration

The operator subscribes to the ISO 9241 series
publishing endpoint. A new ISO 9241 amendment
(a per-display ergonomics revision, a per-
interaction principle revision) triggers an
internal review cycle in the operator's
display-deployment discipline.

## §3 ICC / CIE Colour-Management Ecosystem Integration

The operator queries the ICC.1:2010 profile
register for per-display profiles; the per-
display profile is signed by the per-display
manufacturer's signing-key set so that a
downstream consumer can verify the per-display
envelope. The operator queries the CIE 15:2018
colorimetric envelope for the per-palette
discriminability analysis.

## §4 EU Web Accessibility Directive Integration

A EU-public-sector operator publishes the per-
period accessibility-monitoring report per the
EU Web Accessibility Directive (Directive (EU)
2016/2102) Article 8. The report carries the
per-period accessibility-statement envelope,
the per-period feedback-mechanism envelope,
and the per-period non-accessible-content
declaration.

## §5 US Section 508 Access Board Integration

A US-federal-agency operator publishes the
per-period Section 508 conformance report to
the US Access Board. The report carries the
per-period accessibility-conformance summary,
the per-period exception-and-undue-burden
declaration, and the per-period plan for
remediation.

## §6 KR NIA Accessibility-Audit Integration

A KR-jurisdiction operator binds the per-chart
accessibility profile to the KR 한국정보화진흥원
(NIA) operated KR 정보접근성 인증 (Web
Accessibility Mark) audit. The operator's API
publishes the per-chart accessibility-conformance
evidence to NIA's accessibility-audit endpoint.

## §7 WIA-data-integration Operator Integration

The operator's per-chart data source is
referenced via the upstream WIA-data-integration
operator's per-source endpoint. The per-source
schema descriptor (per ISO/IEC 11179) is bound
to the per-chart's encoding-channel binding so
that the per-chart's encoding remains aligned
with the per-source schema.

## §8 WIA-content-ai Operator Integration

Where the operator uses an AI-assisted authoring
envelope to generate the per-chart specification,
the operator binds the per-chart to the
upstream WIA-content-ai operator's per-output
attestation envelope. The per-chart authorship
envelope records the per-chart AI-assisted
declaration.

## §9 WIA-lms Operator Integration

A learning-management-system operator embedding
the per-chart in a per-course content envelope
binds the per-chart's accessibility profile to
the LMS's per-course accessibility-conformance
envelope. The operator's API publishes the per-
chart embeddable HTML envelope (per PHASE-2
§11) for the LMS to ingest.

## §10 Inclusive-Design Certification-Body Integration

An inclusive-design certification body
(operating under ISO/IEC 17065:2012
accreditation) issues the per-chart inclusive-
design certificate. The certification body
queries the operator's API for the per-chart
accessibility profile, the per-chart
provenance record, and the per-chart audit
envelope.

## §11 Audit and Conformity-Assessment Integration

### §11.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system is
audited under ISO/IEC 17021-1 by an accredited
certification body. The audit result is stored
in the operator's audit envelope.

### §11.2 ISO/IEC 17065 product certification

A per-chart whose route to market includes a
product-certification mark (the EU Accessibility
mark, the KR 정보접근성 인증 mark, the US
Trusted Tester certification) is bound to the
certification body's ISO/IEC 17065:2012
accreditation.

## §12 Public Retrieval and Re-Issuance

### §12.1 Public chart-summary publication

The operator publishes per-period
visualization-statistics on the public-portal
endpoint without per-record identifiers — total
charts published, per-chart-kind distribution,
per-accessibility-conformance distribution.

### §12.2 Verifiable-credentials re-issuance

A per-chart accessibility-conformance
attestation is re-issuable as a W3C Verifiable
Credential signed by the certification body's
signing-key set so that a downstream consumer
can verify the attestation without contacting
the certification body directly.

## §13 KR-Jurisdiction Integration

### §13.1 KR Open Data Portal binding

A KR-jurisdiction public-sector dashboard
publisher binds the per-dashboard envelope
to the KR Open Data Portal's accessibility
declaration.

### §13.2 KR 장애인차별금지법 binding

A KR-jurisdiction operator binds the per-
chart accessibility profile to the KR 장애인
차별금지 및 권리구제 등에 관한 법률 (Anti-
Discrimination Act for Persons with
Disabilities) §21 information-accessibility
obligation.

### §13.3 KR 정보접근성 보장에 관한 법률 binding

A KR-jurisdiction operator binds the per-
chart accessibility profile to the KR 정보
접근성 보장에 관한 법률 (Information
Accessibility Guarantee Act).

## §14 Per-Chart Embed Ecosystem Integration

A host-page consumer (a content-management
system, a news-media platform, a public-
sector portal) embeds the per-chart per the
operator's per-chart embed endpoint (PHASE-2
§11). The host page's accessibility envelope
is consistent with the per-chart accessibility
profile so that the host page's per-page
WCAG 2.2 AA conformance is preserved.

## §15 Per-Chart Print-and-Export Integration

A print-and-export consumer (a per-period
publication, a per-event report, a per-
research per-experiment artefact) renders the
per-chart per the operator's per-chart static-
snapshot endpoint (PHASE-2 §12). The print-
and-export envelope preserves the per-chart
colour space (per ISO 12647:2019 print-
production reference, where applicable) and
the per-chart accessibility envelope.

## §16 Continuous Improvement Programme

Each operator publishes an annual improvement
plan addressing per-chart accessibility-
conformance trend, per-chart graphical-
encoding-faithfulness audit findings, per-
deployment display-calibration drift, and per-
period inclusive-design certification renewal.
The programme is open and the annual report
is published on the operator's public-portal
endpoint per PHASE-2 §10.

## §17 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
data-visualization standard. Implementations
cite the W3C / ISO / IEC / ICC / CIE / IETF
/ EU / US / KR references by their issuing
organisation and the publication year so that
a downstream consumer can locate the
authoritative text. Updates to a cited
standard (for example, a new W3C SVG 2
amendment, a new W3C WCAG release, a new ICC
profile-format release) trigger an internal
review cycle in the operator's quality-
management discipline declared in PHASE-3 §10
before the new revision is bound into the
operator's enumeration set.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §18 Per-Chart Open-Source Tool Integration

The operator's per-chart authoring may use
open-source tooling — D3.js for SVG-based
visualization authoring, Vega-Lite for
declarative chart specification, the
Observable Plot library for grammar-of-
graphics visualization, the Apache ECharts
library for cross-platform rendering. The
operator publishes the per-chart tool-version
envelope so that a downstream auditor can
reproduce the per-chart authoring evidence.

## §19 Per-Chart Print-Production Integration

A print-production consumer rendering the
per-chart at a per-publication resolution (a
news-media print supplement, a per-research
publication artefact, a per-corporate per-
period printed report) integrates with the
operator's per-chart static-snapshot endpoint
(per PHASE-2 §12) and applies the ISO
12647:2019 print-production reference so that
the per-chart colour rendering is preserved
across the digital-to-print transition.

## §20 Per-Chart Accessibility Audit Tooling Integration

The operator's per-chart accessibility audit
runs against multiple per-tool envelopes — the
W3C-recommended axe-core open-source rule
engine, the Pa11y open-source automated audit
tool, the WAVE accessibility evaluator
operated by WebAIM. The operator publishes
the per-chart per-tool audit outcome so that
the per-chart accessibility audit evidence is
multi-source.

## §21 Per-Chart Performance Budget Integration

A per-chart deployment under a performance-
budget envelope (a per-page Web Vitals
envelope, a per-page Lighthouse audit
envelope) is bound to the per-page performance
budget. The operator's API publishes the per-
chart resource envelope (the per-chart
rendered SVG byte-count, the per-chart per-
load network round-trip count, the per-chart
per-load CPU time) so that the host page can
budget the per-chart resource consumption.

## §22 Per-Chart Search-Indexing Integration

A per-chart embedded in a host page is
indexed by a search engine through the per-
chart `<title>` element, the per-chart `<desc>`
element, and the per-chart Schema.org JSON-LD
metadata. The operator's API publishes the
per-chart Schema.org `Dataset` envelope so
that a search engine can deterministically
interpret the per-chart subject matter.
