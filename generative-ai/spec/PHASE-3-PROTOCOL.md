# WIA-generative-ai PHASE 3 — PROTOCOL Specification

**Standard:** WIA-generative-ai
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
generative-AI operator: the model-development
discipline (training-data discipline, fine-tuning
discipline, alignment discipline, evaluation
discipline) aligned with NIST AI RMF and ISO/IEC
42001; the EU AI Act technical-documentation
discipline (Articles 11 and 53, Annexes IV and XI);
the input-output safety discipline at deployment
time; the content-provenance discipline that produces
the C2PA Content Credentials per AI Act Article 50(2);
the user-rights discipline (GDPR Articles 15, 17, 20,
22, and AI Act Article 50 transparency obligations);
the post-market monitoring discipline (AI Act Article
72) for high-risk and GPAI systems; the serious-
incident reporting discipline (AI Act Article 73);
and the systemic-risk discipline (Article 55) that
applies to GPAI models classified as having systemic
risk.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 22989:2022, 23053:2022, 23894:2023, 24029
  -2:2023
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- NIST AI Risk Management Framework 1.0 + NIST AI
  600-1 GenAI Profile
- C2PA Content Credentials specification v1.4
- EU AI Act (Regulation (EU) 2024/1689) Articles
  6, 8 to 17 (high-risk system obligations
  including risk-management Art 9, data-and-data-
  governance Art 10, technical-documentation Art
  11, record-keeping Art 12, transparency-and-
  information-to-deployers Art 13, human-oversight
  Art 14, accuracy-robustness-cybersecurity Art
  15, quality-management Art 17), 25 (provider
  obligations), 26 (deployer obligations), 27
  (FRIA), 50 (transparency to users), 51 to 55
  (GPAI obligations), Article 56 (Code of Practice),
  Article 71 (database for high-risk systems),
  Article 72 (post-market monitoring), Article 73
  (serious-incident reporting), Annexes IV and XI
- EU Code of Practice for General-Purpose AI Models
- EU GDPR Articles 5, 6, 9, 12 to 22, 22(3), 24,
  25, 32, 35
- EU Copyright Directive 2019/790 Article 4 TDM
  opt-out
- US EO 14110, US OMB M-24-10
- KR AI 산업진흥법, KR PIPA Article 28
- OWASP Top 10 for Large Language Model Applications

---

## §1 Training-Data Discipline

The training-data discipline aligns with EU AI Act
Article 10 (data and data governance) and Article
53(1)(c) (sufficiently detailed summary about the
content used for training of GPAI models):

- Source-and-licence inventory — every data source
  is recorded with its provider, the operator's
  licence-to-use evidence, and the date the data
  was acquired or scraped.
- Copyright TDM opt-out compliance — for sources
  scraped under EU Copyright Directive 2019/790
  Article 4 the operator filters out content
  carrying machine-readable opt-out signals
  (robots.txt opt-outs, the IETF AI-prefs draft,
  inline metadata).
- Personal-data filtering — sources are filtered
  for GDPR-protected personal data and Article 9
  special-category data; the residual personal-data
  exposure is documented per Article 35 DPIA when
  the system is high-risk.
- Content-quality filtering — adult, harmful, and
  illegal content (CSAM, terrorism, violent
  extremism) is filtered against the operator's
  classifier set.
- Bias-and-representation review — the dataset's
  demographic-and-language representation is
  documented for the AI-Act Article 10(2)(f)
  examination of biases that may affect the system's
  intended purpose.
- Training-compute estimation — the cumulative
  training compute in FLOP is calculated per the
  operator's hardware estimator; the GPAI Annex XI
  disclosure is updated.

## §2 Fine-Tune, Alignment, and Adapter Discipline

Fine-tune / alignment runs are governed by:

- The fine-tune objective — capability uplift,
  fairness improvement, safety improvement, domain
  adaptation, RLHF reward calibration.
- The fine-tune data manifest — per the §1 source-
  and-licence discipline.
- The evaluation against pre-fine-tune baselines —
  the operator confirms that the fine-tune does not
  regress any safety or fairness metric below the
  baseline.
- The alignment technique — RLHF, DPO, constitutional
  AI, or instruction-tune; the operator records the
  technique, hyperparameters, and reviewer-feedback
  pipeline.
- The model-risk-committee approval — the fine-tuned
  model is approved before it enters production.

## §3 EU AI Act Technical-Documentation Discipline

For high-risk systems and GPAI the operator maintains
the technical-documentation set per AI Act Article 11
(Annex IV for high-risk) and Article 53(1)(a) /
Article 53(1)(b) (Annex XI for GPAI):

- General description of the system / model.
- Detailed information on the system's elements and
  the development process.
- Detailed information on data and data governance.
- Detailed information on monitoring, functioning,
  and control of the system.
- Detailed information on the risk-management
  system (Article 9) and the changes made through
  its lifecycle.
- For GPAI — the cumulative training compute, the
  energy-consumption estimate, the training-data
  summary, the policies for respecting Union
  copyright (Article 53(1)(c) and (d)), and — for
  systemic-risk GPAI — the model-evaluation results,
  the systemic-risk assessment, and the cybersecurity
  protections (Article 55).

## §4 Input-Output Safety Discipline

Deployment-time safety operates as a layered
discipline:

- Input filter — prompt-injection detection, PII /
  secret detection, abuse-pattern detection. Inputs
  flagged are refused or redacted before reaching
  the model.
- Output filter — unsafe-content detection, IP-
  leakage detection, factual-claim flagging, PII
  leakage detection. Outputs flagged are blocked,
  redacted, or accompanied by a warning.
- Tool-use guardrail — the tool catalogue's
  authorised endpoints constrain function calling;
  the operator's policy-decision point evaluates
  every tool invocation before execution.
- Retrieval guardrail — RAG indices are scoped to
  the user's authorised reading scope; cross-tenant
  index leakage is prevented by index-partitioning.

## §5 Content-Provenance Discipline (C2PA)

The content-provenance discipline produces the C2PA
Content Credentials manifest for every AI-generated
output where AI Act Article 50(2) applies:

- The manifest carries the producer claim (the
  operator), the AI-generated declaration, the model
  reference, the system reference, and the
  generation timestamp.
- The manifest is signed using the operator's
  signing-key registered with the C2PA trust list.
- For text outputs the manifest reference is
  delivered alongside the response (header or
  metadata field); for image / video / audio outputs
  the manifest is embedded in the file or delivered
  through the side-car convention.
- For invisible watermarking (where applied) the
  watermark detail is recorded in the manifest;
  cryptographic watermarking schemes follow the
  operator's published algorithm reference.

## §6 Post-Market Monitoring and Re-Evaluation Discipline

For high-risk systems and GPAI the operator's post-
market monitoring discipline aligns with AI Act
Article 72:

- Capability drift — periodic re-running of the
  capability benchmark suite to detect regressions
  or unintended uplifts.
- Fairness drift — periodic re-running of the
  fairness suite across protected classes.
- Safety drift — adversarial red-team rotation;
  jailbreak-and-refusal evaluation; abuse-pattern
  surveillance.
- Use-pattern drift — telemetry-aggregated use
  patterns are reviewed for alignment with the
  declared intended purpose.

Material drift triggers the operator's re-evaluation
workflow; for GPAI systemic-risk models AI Act
Article 55 imposes additional model-evaluation,
adversarial-testing, and mitigation obligations.

## §7 User-Rights and Transparency Discipline

User-facing rights exercised by the discipline:

- AI Act Article 50(1) — operators of AI systems
  intended to interact with natural persons inform
  the user that they are interacting with AI, where
  this is not obvious from the context.
- AI Act Article 50(2) — providers of generative-AI
  systems mark output as artificially generated or
  manipulated using machine-readable formats; the
  C2PA manifest serves this purpose.
- AI Act Article 50(3) — deployers of emotion-
  recognition or biometric-categorisation systems
  inform the affected persons.
- AI Act Article 50(4) — deployers of deepfake-
  generating systems disclose that the content has
  been artificially generated or manipulated.
- GDPR Article 15 — the user's right of access to
  the personal data the operator processes through
  the inference path.
- GDPR Article 17 — the user's right to erasure of
  their transcripts.
- GDPR Article 22 — the user's right not to be
  subject to a decision based solely on automated
  processing where it produces legal or similarly
  significant effects, with the Article 22(3)
  human-review channel.
- KR PIPA Article 28 — the equivalent KR-statutory
  automated-decision-making rights.

## §8 GPAI Systemic-Risk Discipline (Article 55)

For GPAI models classified as having systemic risk
the operator's discipline adds:

- Model evaluation — the operator performs and
  documents model evaluation including adversarial
  testing to identify systemic risks.
- Systemic-risk assessment and mitigation — the
  operator assesses and mitigates systemic risks
  arising from the model's development, placing on
  the market, or use, including risks to health,
  safety, fundamental rights, or democratic processes.
- Tracking serious incidents — the operator tracks
  and documents serious incidents and reports them
  to the AI Office and to the affected national
  competent authorities.
- Cybersecurity — the operator ensures an adequate
  level of cybersecurity for the GPAI model and its
  physical infrastructure.
- Code of Practice adherence — until the harmonised
  standards are published the operator demonstrates
  compliance through the EU Code of Practice for
  General-Purpose AI Models (AI Act Article 56).

## §9 Serious-Incident Reporting Discipline (Article 73)

For high-risk systems serious incidents (death,
serious harm to health, serious harm to property or
to the environment, infringement of fundamental
rights, serious cybersecurity breach) are reported
to the relevant market-surveillance authority within
the AI Act Article 73 timeframes; the operator
preserves the incident record and the corrective-
action record under the Article 12 record-keeping
discipline.

## §10 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
maintain the technical-documentation set per
Articles 11 and 53, exercise the C2PA Content
Credentials discipline at output time, exercise the
post-market monitoring discipline on the operating
cadence, and report serious incidents within the
AI Act Article 73 timeframes.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-generative-ai
- **Last Updated:** 2026-04-28
