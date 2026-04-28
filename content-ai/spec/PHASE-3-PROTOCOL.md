# WIA-content-ai PHASE 3 — PROTOCOL Specification

**Standard:** WIA-content-ai
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
AI-assisted content-platform operator: the EU AI Act
Article 50 transparency discipline (interaction-with-
AI · generative-output watermarking · emotion-
recognition · deepfake disclosure); the C2PA Content
Credentials issuance and verification discipline; the
EU DSA notice-and-action and statement-of-reasons
discipline; the DSA appeals and out-of-court dispute-
settlement discipline; the DSA risk-assessment and
mitigation discipline for VLOP / VLOSE operators
(Articles 34 + 35); the DSA dark-pattern prohibition
(Article 25); the DSA recommender-system transparency
(Article 27); the DSA online-protection-of-minors
discipline (Article 28); the COPPA / UK AADC / KR
청소년보호법 discipline; the WCAG 2.2 + EU EAA + ADA
accessibility discipline; the recommender-system
fairness and harmful-content classifier discipline;
the model-evaluation, red-team, and incident
discipline; the rights-holder copyright discipline;
and the supervisory and law-enforcement cooperation
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 22989:2022, 23053:2022, 42001:2023
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- C2PA Content Credentials specification v1.4
- ETSI TS 104 224
- IPTC Photo Metadata 2024
- EU AI Act Articles 5, 50, 51 to 55
- EU DSA (Regulation (EU) 2022/2065) Articles 14,
  16, 17, 20, 21, 22, 23, 24, 25, 26, 27, 28, 33 to
  43, 70 to 88
- EU Empowering Consumers (EU) 2024/825
- US Section 230 (47 USC 230)
- US COPPA (15 USC 6501-6506; 16 CFR Part 312)
- US California SB-1001 (Bot Disclosure)
- UK AADC (ICO Children's Code)
- UK Online Safety Act 2023
- KR PIPA + KR 청소년보호법 + KR 정보통신망법 + KR
  방송통신심의위원회 + KR KCC
- W3C WCAG 2.2 + ATAG 2.0 + UAAG 2.0
- EU EAA (EU) 2019/882 + EN 301 549
- US ADA Title III + US Section 508
- NIST AI 600-1 GenAI Profile
- UNESCO Recommendation on the Ethics of AI (2021)

---

## §1 EU AI Act Article 50 Transparency Discipline

The Article 50 discipline is layered:

- Article 50(1) — operators of AI systems that
  interact with natural persons inform the user that
  they are interacting with AI, where this is not
  obvious from the context. The operator's user-
  interface emits the in-context disclosure at the
  first interaction.
- Article 50(2) — providers of generative-AI systems
  mark output as artificially generated or manipulated
  in a machine-readable format. The C2PA Content
  Credentials manifest is the canonical machine-
  readable mark; the operator's editor / publisher
  workflow attaches the manifest to every AI-
  generated output before publication.
- Article 50(3) — deployers of emotion-recognition
  or biometric-categorisation systems inform the
  affected persons.
- Article 50(4) — deployers of deepfake systems
  disclose that the content has been artificially
  generated or manipulated. The disclosure
  presentation must be clear and visible to the
  affected user.

## §2 C2PA Content Credentials Discipline

The Content Credentials discipline:

- Issuance — every AI-generated output produced on
  the operator's platform receives a C2PA v1.4
  manifest at generation time. The manifest is
  signed using the operator's signing key registered
  with the C2PA trust list.
- Embedding — for image / video / audio assets the
  manifest is embedded in JUMBF / XMP per the
  format-specific specification; for text assets the
  manifest is delivered through the side-car URL
  convention plus an HTTP header.
- Provenance preservation — when a downstream
  publisher re-uses operator-produced content the
  parent manifest reference is preserved in the
  child manifest.
- Verification — the operator's verification surface
  (and the public verification widget the operator
  may host) consults the C2PA trust list to validate
  third-party-supplied manifests.

## §3 DSA Notice-and-Action and Statement-of-Reasons
       Discipline

For EU-jurisdiction platforms in DSA scope:

- Article 14 — clear and unambiguous terms-and-
  conditions including content-moderation policy.
- Article 16 — notice-and-action mechanism that
  allows users and trusted flaggers to notify the
  operator of illegal content; the operator's
  technical implementation matches the DSA Article
  16(2) notice schema.
- Article 17 — statement-of-reasons delivered to
  affected users when content is removed, restricted,
  demonetised, or the user's account is restricted.
  The format follows the DSA Transparency Database
  (DSA-TDB) schema.

## §4 DSA Appeals and Out-of-Court Dispute Settlement
       Discipline

- Article 20 — internal complaint-handling system
  available for free for at least six months after
  the original decision.
- Article 21 — out-of-court dispute settlement —
  the operator submits to the resolution of disputes
  by certified out-of-court dispute settlement bodies
  selected by the recipient, where the user has
  exhausted the internal complaint-handling.
- Article 23 — suspension of users that frequently
  provide manifestly illegal content.

## §5 DSA Risk-Assessment and Mitigation Discipline
       (VLOP / VLOSE)

For VLOP / VLOSE designated operators (DSA Articles
33 to 43):

- Article 34 — annual risk assessment covering
  systemic risks (illegal content; rights violations;
  fundamental-rights effects; civic-discourse and
  electoral-process effects; gender-based violence;
  protection of minors; serious negative
  consequences to physical or mental wellbeing).
- Article 35 — mitigation measures proportionate to
  the identified risks.
- Article 36 — crisis-response mechanism activated
  by the Commission.
- Article 37 — independent audit of compliance with
  Chapter III obligations.
- Article 38 — recommender-system transparency
  including the option for the user to switch to a
  recommender system that is not based on profiling.
- Article 39 — additional online advertising
  transparency.
- Article 40 — data access for vetted researchers.

## §6 DSA Dark-Pattern, Recommender-System, and
       Minor-Protection Discipline

- Article 25 — prohibition of dark patterns at the
  online-interface design level. The operator's UX
  reviews surface design patterns that materially
  distort or impair the user's ability to make free
  and informed decisions.
- Article 26 — advertising transparency including
  the parameters used to determine the recipient,
  the natural / legal person on whose behalf the ad
  is presented, and the natural / legal person who
  paid for the ad.
- Article 27 — recommender-system parameter
  transparency in the operator's terms and conditions
  and at the point of personalisation.
- Article 28 — online protection of minors —
  appropriate measures to ensure a high level of
  privacy, safety, and security of minors; advertising
  based on profiling using personal data of recipients
  the operator is reasonably aware are minors is
  prohibited.

## §7 Child-Safety and Age-Appropriate Discipline

The age-appropriate discipline:

- COPPA 16 CFR Part 312 — verifiable parental
  consent before collecting personal information
  from children under 13.
- UK AADC — the ICO's 15 standards for online
  services likely to be accessed by children
  (Best Interests of the Child; data-minimisation;
  default high privacy settings; clear privacy
  information; and so on).
- KR 청소년보호법 — youth-protection programme,
  age-rating discipline for content, and the
  operator's protection-of-minors operating
  procedure.
- KR 정보통신망법 — the operator's notice and the
  removal-of-prohibited-information cooperation
  with KCC / 방송통신심의위원회.

## §8 Accessibility Discipline (WCAG + EAA + ADA)

The accessibility discipline:

- WCAG 2.2 Level AA — the operator's content and
  user-interface conform to perceivable, operable,
  understandable, and robust principles.
- ATAG 2.0 — the operator's authoring tools support
  accessible-content production.
- UAAG 2.0 — the operator's user-agent surfaces
  support assistive-technology integration.
- EU EAA (Directive (EU) 2019/882) — for in-scope
  consumer products and services the operator's
  accessibility-compliance discipline applies.
- US ADA Title III — for public accommodations the
  operator's web-and-mobile-app accessibility is
  compliant.
- KR 장애인차별금지법 + KCC WCAG — for KR-
  jurisdiction the equivalent compliance.

## §9 Classifier and Red-Team Discipline

The operator's classifier-and-red-team discipline:

- Periodic classifier evaluation — precision /
  recall / F1 / disparate-impact metrics across
  protected categories.
- Red-team rotation — internal and external red-
  team engagements that probe for jailbreak,
  manipulation, and evasion.
- Trusted-flagger feedback — the operator
  incorporates trusted-flagger error reports into
  classifier-retraining.
- Adversarial-content monitoring — coordinated
  inauthentic-behaviour, foreign-influence
  operations, and CSAM-detection (NCMEC reporting
  for US-jurisdiction operators).

## §10 Rights-Holder Copyright Discipline

For copyright-bearing content (text, image, video,
audio, music):

- US DMCA 17 USC 512 safe-harbour notice-and-
  takedown for US-jurisdiction operators.
- EU DSM Directive (Directive (EU) 2019/790)
  Article 17 best-effort licensing and the
  proportionate-to-the-rights-holder filter for
  in-scope content-sharing platforms.
- KR 저작권법 — the KR-jurisdiction copyright-
  notice and ISP-cooperation regime.
- AI-training-data opt-out — for AI-generated
  content the upstream model's training-data opt-
  out compliance is preserved in the C2PA manifest.

## §11 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every content
publication, transcript record, content-credentials
issuance, moderation decision, notice receipt, appeal
lodgement, transparency-report publication, child-
safety record, accessibility update, and rights-
holder correspondence.

## §12 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
exercise the EU AI Act Article 50 transparency
obligations, exercise the C2PA Content Credentials
issuance for AI-generated content, exercise the DSA
Article 14-28 obligations where in scope, and
satisfy the WCAG 2.2 / EAA / ADA / KR-equivalent
accessibility-conformance baseline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-content-ai
- **Last Updated:** 2026-04-28
