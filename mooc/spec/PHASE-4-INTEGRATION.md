# WIA-mooc PHASE 4 — Integration Specification

**Standard:** WIA-mooc
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-mooc integrates with
adjacent learning, identity, credentialing, content-
authoring, content-delivery, accessibility, research,
and regulatory systems: enterprise LMS via LTI 1.3 and
Common Cartridge, learning-record stores (Caliper
endpoint / xAPI LRS), corporate skills frameworks
(ESCO / O*NET-SOC), micro-credential issuers, video-
streaming and CDN providers, captioning / sign-language
services, identity-verification providers, proctoring
services, regulator and accreditation registries, and
analytics consumers (Caliper Aggregator pattern, data
warehouse, lakehouse).

References (CITATION-POLICY ALLOW only):
- 1EdTech LTI 1.3 / LTI Advantage; Common Cartridge 1.3; QTI 3.0
- 1EdTech Caliper Analytics 1.2; ADL xAPI 2.0
- W3C Verifiable Credentials Data Model 2.0; 1EdTech Open Badges 3.0
- Europass Credentials specification; European Learning Model (ELM)
- ESCO classification; O*NET-SOC; ISCO-08
- ENQA ESG 2015; ISO 21001; ISO/IEC 19796-1
- W3C WCAG 2.2; EN 301 549; Section 508 (US)
- W3C Web Annotation Protocol; W3C Activity Streams 2.0
- HLS / DASH; SCTE 35 (where ad-insertion); SCTE-224 (where ad-policy)
- WebVTT (W3C); SMI; SRT
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)
- ISO 8601, ISO 3166, BCP 47

---

## §1 Enterprise LMS integration via LTI

| Pattern                 | Profile                                          |
|-------------------------|--------------------------------------------------|
| LTI Resource Link       | LTI 1.3 launch into MOOC course / module          |
| LTI Names and Roles     | NRPS roster sync from LMS to MOOC                 |
| LTI Assignment & Grade  | AGS grade returns from MOOC to LMS                |
| LTI Deep Linking 2.0    | tool-catalogue selection of MOOC content          |
| Common Cartridge        | course content packaged for LMS import            |

LMS-side configuration records the MOOC platform's
issuer, deployment-id, and JWKS; tool-platform-config
records the LMS public keys for inbound launches.

## §2 Learning-record store (LRS) integration

| LRS profile             | Use                                              |
|-------------------------|--------------------------------------------------|
| Caliper Aggregator      | sponsor-internal Caliper consumer                |
| xAPI LRS                | research / corporate LRS consumer                |
| Cross-platform analytics | enterprise lakehouse                             |

Events emit to one or more LRS in a fan-out pattern;
the implementation tracks per-LRS delivery
acknowledgement.

## §3 Skills-framework integration

| Framework              | Binding                                          |
|------------------------|--------------------------------------------------|
| ESCO                   | course / lesson outcomes map to ESCO skills /     |
|                        | competences                                      |
| O*NET-SOC              | US occupation alignment                           |
| ISCO-08                | ILO occupation classification                     |
| EQF / NQF              | per-country qualification level                   |
| CASE 1.1               | competency-framework exchange                     |

Mappings are versioned; framework changes (ESCO
release, EQF revision) trigger re-mapping events.

## §4 Micro-credential issuer integration

Course / cohort completion binds to WIA-micro-
credential issuance:

```
completion-validated → credential-class-resolved →
  recipient-DID-resolved → OID4VCI-issuance →
  status-list-entry-published
```

The issuance event emits Caliper / xAPI `Issued`
verbs so downstream analytics see the credential
event.

## §5 Content-delivery and CDN integration

| Component               | Integration                                      |
|-------------------------|--------------------------------------------------|
| Origin storage          | content-addressed object store                    |
| CDN                     | per-implementation (Akamai, Cloudflare, AWS      |
|                         | CloudFront, Fastly, Naver Cloud, etc.)            |
| Adaptive streaming      | HLS + DASH dual manifest                          |
| DRM                     | Widevine / FairPlay / PlayReady (where bound)    |

CDN cache-control headers honour the manifest's
content digest so a manifest update propagates
within the configured TTL.

## §6 Accessibility-services integration

| Service                 | Profile                                          |
|-------------------------|--------------------------------------------------|
| Captioning              | WebVTT / SRT — automatic + human-reviewed        |
| Sign-language            | KSL / ASL / BSL / etc. video track                |
| Audio description       | descriptive-audio track                          |
| Real-time translation   | UN ESCAP / W3C live-region (where bound)          |
| Screen-reader testing   | NVDA / JAWS / VoiceOver / TalkBack matrix         |

The accessibility-assertion record cites the services
used and the auditor's signature.

## §7 Identity-verification and proctoring

| Service                 | Profile                                          |
|-------------------------|--------------------------------------------------|
| Identity verification   | ID document + selfie liveness                     |
| Live human proctoring   | webcam + audio + screen-share                     |
| AI-assisted proctoring  | anomaly detection on webcam + screen              |
| In-person centre        | testing-centre infrastructure                     |

Proctoring artefacts carry their own provenance hash;
the assessment-attempt record references the hash so
recognising parties can request the artefact under
learner consent.

## §8 Regulator / accreditation registries

| Registry                | Use                                              |
|-------------------------|--------------------------------------------------|
| National higher-ed      | accreditation status binding                     |
| accreditation body      |                                                  |
| EU EBSI                 | European trust framework for credentials          |
| Industry recognising    | per-industry / sector accreditation               |
| body                    |                                                  |
| Sponsor-internal        | corporate L&D recognition                         |

Accreditation events bind to courses so the
recognition status is visible to prospective learners.

## §9 Cross-domain WIA bindings

| Companion standard          | Binding purpose                                |
|-----------------------------|------------------------------------------------|
| WIA-micro-credential        | completion-credential issuance                 |
| WIA-virtual-classroom       | live-session integration                       |
| WIA-learning-analytics      | cohort-level analyses                          |
| WIA-content-ai              | AI-generated assessment items                  |
| WIA-data-portability        | learner export                                  |
| WIA-content                 | open-content licence binding                    |

Each binding identifies the consumed PHASE.

## §10 Long-term archival

| Authority / context         | Retention                                |
|-----------------------------|------------------------------------------|
| Issuer (regulated)          | per national rules; typically ≥ 25 years |
| Issuer (vocational)         | ≥ 10 years                                |
| LRS                         | sponsor-defined; typically ≥ 5 years      |
| Accessibility-audit records | per regulator                             |
| Proctoring artefacts        | per learner consent + regulator           |

## §11 Conformance test suite

The reference test suite covers:

- LTI 1.3 launch round-trip with NRPS / AGS
- Caliper Sensor → Aggregator delivery
- xAPI v2.0 statement PUT round-trip
- QTI 3.0 item authoring + attempt scoring
- WCAG 2.2 Level AA on a representative course
- HLS + DASH manifest verification with content digest
- proctored-attempt requires active proctoring
- peer-assessment minimum-grade enforcement
- credential issuance through OID4VCI on completion
- learner data-portability export of events + attempts

## §12 Internationalisation

Course titles, module / lesson labels, item prompts,
narrative feedback, and accessibility tracks carry
BCP 47 language tags. Country-specific accreditation
paths are resolved by the issuer's primary
jurisdiction (ISO 3166-1 alpha-3).

## §13 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for LTI Platform
  ↔ Tool exchanges
- Authentication: OAuth 2 with PKCE for learners; JWT
  Bearer assertions for LTI 1.3
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-cohort key wrapping
- Audit: tamper-evident chain (PHASE 3 §11) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: learner identifiers opaque; PII held in
  identity vault; learner rights honoured per PHASE 3
  §13
- Adaptive-learning consent: explicit and revocable

## §14 Operational metrics

Sponsors report (informationally) on the WIA registry:

- enrolments / completions per cohort
- pass rate per cohort
- accessibility-audit conformance level
- inter-rater reliability per peer-assessment cohort
- cohort outcome reproducibility tier
- adaptive-learning opt-out rate

## §15 Recovery and continuity

- API outage — learners' app caches lessons offline;
  events queue and submit on reconnect
- LRS outage — events buffer locally; replay on
  recovery; idempotent on `eventId`
- CDN outage — origin failover; manifest re-fetch
- proctoring service outage — attempt rejected if
  proctoring is required; retried on recovery

## Annex A — Worked end-to-end example (informative)

A higher-education provider hosts a 6-week
"Introduction to Probability" MOOC. The course is
LTI 1.3-launchable from partner LMS deployments. The
content is delivered as HLS / DASH dual manifest with
WebVTT captions and a KSL sign-language track. Items
are authored in QTI 3.0; calibration uses cohort-1
data. Cohort-2 enrolment includes 12,300 learners
across 27 countries; LTI Resource Links carry per-
institution roster sync. Cohort-2 completion validates
under the pass policy; 4,180 learners receive a
completion credential issued via OID4VCI bound to the
learner's DID. Caliper events stream to a sponsor-
internal Aggregator and to a partner LMS-side LRS.

## Annex B — Conformance disclosure

Implementations declare the LTI 1.3 services supported
(NRPS / AGS / Deep Linking), the Caliper / xAPI
versions served, the QTI revision, the credential-
issuance profile (OID4VCI), the WCAG / EN 301 549
audit results, and the proctoring services integrated.
Disclosure is machine-readable at `/.well-known/wia-
mooc-conformance.json`.

## Annex C — Versioning

Adding a new content-delivery integration is minor;
changing the LTI 1.3 service binding is major.

## Annex D — Open-content licence binding

Course content carries an SPDX licence identifier on
each content-asset record. Common bindings:

| Licence                | Use case                                       |
|------------------------|------------------------------------------------|
| CC0-1.0                | public-domain dedication                       |
| CC-BY-4.0              | attribution required                           |
| CC-BY-SA-4.0           | share-alike                                    |
| CC-BY-NC-4.0           | non-commercial                                 |
| All-Rights-Reserved    | proprietary; explicit licensing required       |

Open Education Resource (OER) consumers honour the
licence at re-use; the platform exposes a licence
filter so OER aggregators discover compatible content.

## Annex E — Research / data-anonymisation export

Research-grade data exports apply k-anonymity (k ≥ 5)
on quasi-identifier fields and l-diversity on
sensitive learning-outcome attributes before release.
Data-use agreements record the IRB approval, the data-
recipient identity, and the retention window.

## Annex F — Cross-platform learner mobility

Where a learner moves between MOOC platforms (e.g.
edX → Coursera → FutureLearn) the standard supports
xAPI / Caliper export of the learner's full event
history bound to the learner's DID. The receiving
platform consumes the export and continues without
loss of progress where the courses align.
