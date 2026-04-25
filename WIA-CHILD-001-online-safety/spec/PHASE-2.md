# WIA-CHILD-001 PHASE 2: Intelligence

**弘益人間** - Benefit All Humanity

## Phase 2 Overview: AI Threat Detection & Behavioural Analysis

### Objective

Phase 2 introduces machine-learning–assisted threat detection, behavioural-pattern analysis, and predictive risk modelling to identify grooming behaviour, cyberbullying, and emerging threats prior to harm. The Phase 2 design treats child safety as a privacy-preserving problem first and a detection problem second: every detection capability is accompanied by a defined data-minimisation, retention, and oversight regime.

---

## 1. Behavioural Threat Detection Engine

### 1.1 Detection Capabilities

The detection engine is a layered ensemble. Each layer below is independent and contributes a confidence signal that the orchestrator combines:

1. **Lexical layer** — Keyword and phrase matching against a curated lexicon of harmful content categories. The lexicon is multilingual, version-controlled, and reviewed by child-safety experts on a published schedule.
2. **Sequence layer** — Conversation-level features capturing turn count, asymmetry of message length between participants, escalation in personal-information requests, and invitations to move to off-platform channels.
3. **Semantic layer** — Transformer-based language models scored on grooming, sextortion, and bullying taxonomies. Models are evaluated on benchmark sets curated by independent researchers and child-safety organisations.
4. **Cross-conversation layer** — Aggregation across multiple conversations between the same accounts to detect distributed grooming and pattern repetition.

### 1.2 Operating Points

The orchestrator emits one of three classes:

- **GREEN** — Low risk; no intervention.
- **YELLOW** — Heightened risk; trigger soft interventions (educational prompts, friction, increased moderator review queue priority).
- **RED** — High confidence of harm in progress; trigger hard interventions (account suspension review, mandatory reporting where required, immediate notification to designated guardian where applicable).

Operating points are policy decisions made by the deploying platform. The reference policy errs on the side of recall for RED-class outcomes given the irreversibility of harm to children, while keeping YELLOW interventions soft so that false positives result in pedagogically valuable nudges rather than punitive action.

### 1.3 Privacy-Preserving Inference

The detection engine MUST be deployable in three privacy regimes:

- **Server-side inference** — Content is processed by the platform under existing platform terms, with documented retention.
- **Edge inference** — Detection runs on the user's device; only aggregate signals leave the device.
- **Federated inference** — Detection runs across a fleet of devices with cohort-level aggregation; no individual content leaves the device.

The choice of regime is a deployment decision; the standard requires that the deployment publish which regime is in force and why.

---

## 2. Predictive Risk Modelling

### 2.1 Risk Indicators

Predictive risk modelling combines longitudinal indicators across:

- Account-creation patterns (rapid creation of multiple accounts, recently created adult accounts contacting child accounts).
- Network features (degree, clustering, contact diversity).
- Content-pattern features (lexical, sequence, semantic from §1).
- Off-platform indicators when the deploying platform provides them (link-out frequency to specific domains, chat-platform handoffs).

Indicators are computed under the privacy-preservation regime declared in §1.3.

### 2.2 Calibration and Bias

Predictive scores undergo regular calibration audits using:

- Reliability diagrams across demographic strata (age band, language, region) to detect systematically over- or under-predicted segments.
- Equal-opportunity and disparate-impact analyses per the IEEE 7003-2024 algorithmic-bias standard.
- Independent red-teaming against adversarial inputs by external child-safety researchers.

Audit results are published in the deployment's transparency report and feed back into the editorial process for the lexicon and the model training data.

### 2.3 Human-in-the-Loop

Risk-model outputs are not the final word on any intervention against an account. Material consequences (account suspension, mandatory report, guardian notification) require:

- Human review by trained moderators with documented qualifications.
- Audit-trail preservation of reviewer identity and reasoning.
- Right of appeal accessible to the account holder or their legal guardian.

This human-in-the-loop discipline is mandatory under several regulatory regimes (GDPR Article 22, the EU Digital Services Act for systemic platforms) and is mandatory under WIA-CHILD-001 regardless of jurisdiction.

---

## 3. Natural-Language Understanding

### 3.1 Multilingual Support

Phase 2 introduces multilingual sentiment and intent classification across the WIA-CHILD-001 priority language set. Cultural-context handling is supported by region-specific model variants and locale-aware lexicon overrides.

Locale handling follows BCP 47 (RFC 5646) language-tag identification, with CLDR (Unicode Common Locale Data Repository) data driving any locale-specific text processing.

### 3.2 Implicit Threat Recognition

Many grooming and bullying threats are conveyed through implicit language: irony, indirect requests, coded language. The semantic layer is fine-tuned on examples curated for these patterns, and the orchestrator weights implicit-threat signals more heavily when the conversation involves an asymmetric age signal (adult-coded ↔ minor-coded participants).

### 3.3 Intent Detection

Intent classification draws from established conversational-AI taxonomies, with extensions for child-safety-specific intents: information solicitation, grooming-typical reciprocity tests, escalation invitations, and coercion. Each intent class has documented training data, evaluation benchmarks, and operational thresholds.

---

## 4. Computer-Vision Capabilities

### 4.1 Image Classification

Image classification supports:

- Detection of CSAM (child sexual abuse material) by hash matching against the established hash registries operated by NCMEC (National Center for Missing & Exploited Children) and INHOPE for international hotlines, plus perceptual-hash matching for known variants.
- Detection of self-harm imagery using models curated and reviewed under the deploying platform's clinical-safety procedure.
- Detection of nudity and graphic violence within the platform's content-policy framework.

Hash matching against CSAM registries is operated under the reporting and recordkeeping rules of the deploying jurisdiction (18 U.S.C. §2258A in the US; equivalent laws elsewhere).

### 4.2 Video and Live-Stream Considerations

Video and live-stream monitoring follows the same privacy regime as text. Sample-frame analysis is the default; full-stream analysis is reserved for content under elevated risk per §1.

---

## 5. Reference Standards Alignment

| Concern | Reference | Role |
|---------|-----------|------|
| Child rights | UN Convention on the Rights of the Child (UNCRC) | Foundational rights framework |
| Online child privacy (US) | 15 U.S.C. §6501–6506 (COPPA), 16 CFR Part 312 | Mandatory rules for under-13 services |
| Child-specific consent (EU) | Regulation (EU) 2016/679 (GDPR) Article 8 | Age-of-consent rules for online services |
| Educational records (US) | 20 U.S.C. §1232g (FERPA) | Where the platform serves educational contexts |
| Online safety (UK) | UK Online Safety Act 2023 | Children's safety duties |
| Platform regulation (EU) | Regulation (EU) 2022/2065 (DSA) | Risk assessment for systemic platforms |
| CSAM reporting (US) | 18 U.S.C. §2258A | Mandatory reporting of CSAM |
| Privacy framework | ISO/IEC 29100:2011 | Privacy concepts and design |
| Privacy by design | ISO/IEC 29151:2017 | PII protection code of practice |
| Information security | ISO/IEC 27001:2022 | Operating organisation security baseline |
| Privacy management | ISO/IEC 27701:2019 | PIMS |
| Cloud privacy | ISO/IEC 27018:2019 | PII in public clouds |
| Algorithmic bias | IEEE 7003-2024 | Algorithmic bias considerations |
| Ethical AI | IEEE 7000-2021 | Model process for addressing ethical concerns |
| Locale | BCP 47 (RFC 5646) | Language tagging |
| Locale data | Unicode CLDR | Locale-specific data |
| AI risk management | NIST AI Risk Management Framework | Governance, mapping, measurement, management |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

---

## 6. Conformance

A Phase 2 implementation is conformant when:

1. The behavioural detection engine is layered and emits at least the GREEN/YELLOW/RED operating-point classes in §1.2.
2. The privacy-preservation regime is declared per §1.3.
3. Predictive-risk-model calibration and bias audits are documented and public.
4. Human-in-the-loop review is required for material consequences against an account.
5. Multilingual support follows BCP 47 + CLDR.
6. CSAM detection uses an established hash registry and the deploying jurisdiction's reporting procedure.
7. Information-security and privacy controls map to a published statement of applicability against the §5 references.

---

## 7. Implementation Appendix

### 7.1 Lexicon Editorial Process

The lexicon underpinning the lexical layer is treated as a living, version-controlled artefact. The reference editorial process has four stages:

1. **Submission** — New entries proposed by operators, researchers, or partner organisations are filed with a justification, examples, and a category assignment.
2. **Editorial review** — A standing editorial committee with child-safety expertise reviews submissions on a published cadence.
3. **Linguistic review** — Native-speaker reviewers evaluate the proposed entry for cultural appropriateness, false-positive risk, and compatibility with existing entries.
4. **Versioned release** — Approved entries enter the next minor release of the lexicon. Removed or modified entries follow the same process in reverse.

Lexicon changes are tracked in a public changelog with date, motivation, and editorial committee members involved. Operators and partners can pin to a specific lexicon version in their deployment configuration if reproducibility is required.

### 7.2 Model Training Data Provenance

Model training data sources are documented for every production model. The reference programme captures:

- **Source** — Origin of the data (curated open-source benchmark, partner-contributed dataset, platform-internal labelling, etc.).
- **Licence** — Terms under which the data may be used for training.
- **Demographic stratification** — Statistics on language, age band, and region.
- **Sensitive content handling** — Specific safeguards for any sensitive content (CSAM hashes are never used as training data; only documented hashes from authorised registries are used for matching).
- **Retention** — How long the data is retained and under what conditions.

Provenance documentation is preserved alongside the model artefact for the model's full lifecycle.

### 7.3 Calibration Reporting

Calibration reports are published per major model release. The reference report includes:

- Per-class reliability diagrams.
- Brier score and expected calibration error (ECE).
- Per-stratum calibration analysis.
- Operator-set thresholds and their false-positive / false-negative implications.
- Trend analysis against prior releases.

Calibration is a published commitment, not an internal metric.

### 7.4 Adversarial Testing

Adversarial testing is conducted on a published cadence by independent researchers and child-safety organisations. The reference programme covers:

- **Lexicon-evasion attacks** — Misspellings, homoglyphs, code-switching across scripts.
- **Semantic-evasion attacks** — Indirect language, irony, coded references.
- **Vision-evasion attacks** — Image perturbations bounded by perceptual thresholds.
- **Cross-platform attacks** — Coordinated attacks spanning multiple integrated platforms.

Findings are addressed through documented remediation plans with timelines and reviewer signatures.

### 7.5 Operator Tooling

Operators interact with the platform through a console that surfaces detection outputs, intervention queues, calibration metrics, and incident-response workflows. Console actions are audited with reviewer identity, action, and reason.

The console conforms to W3C WCAG 2.2 Level AA so that operators with disabilities can perform the work without external accommodation. Operator training records are maintained per the operating organisation's HR procedure.

---

© 2025 SmileStory Inc. / WIA | 弘益人間
