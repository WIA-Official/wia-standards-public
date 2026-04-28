# WIA-ai-survival-2026 PHASE 3 — PROTOCOL Specification

**Standard:** WIA-ai-survival-2026
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
AI-survival operator: the ISO/IEC 42001:2023 AIMS
discipline; the EU AI Act technical-and-organisational
discipline (Articles 8 to 17 high-risk + 51-55 GPAI);
the NIST AI RMF Govern / Map / Measure / Manage
discipline; the responsible-scaling and frontier-
preparedness discipline (UK / US AI Safety Institute
voluntary commitments + AI Action Summit Paris 2025
+ Seoul AI Summit 2024); the workforce-transition
and human-oversight discipline; the safety-and-
security testing discipline (red-team rotation,
capability evaluations, robustness tests); the
incident-reporting discipline (AI Act Art 73 + US
EO 14110 + KR AI 기본법); the supply-chain-AI-
integrity discipline; the dual-use and export-
control discipline; the international-coordination
discipline (UN AI Advisory Body + GPAI + OECD +
UNESCO); and the supervisory cooperation
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 + 38507:2022
- ISO/IEC 42001:2023 + 22989 + 23053 + 23894 +
  24029-1/-2 + TR 24027 + TR 24028 + TS 4213
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- NIST AI Risk Management Framework 1.0 + AI 600-1
  GenAI Profile + IR 8332 + IR 8259
- US EO 14110 + US OMB M-24-10 + US AISI
  Consortium voluntary commitments + US AISI
  Pre-Deployment Testing Memorandum of Understanding
- EU AI Act (Regulation (EU) 2024/1689) Articles
  5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
  25, 26, 27, 49, 50, 51, 52, 53, 54, 55, 56, 71,
  72, 73, 74, 86 + Annex III + Annex IV + Annex
  XI + Annex XIII
- EU Code of Practice for General-Purpose AI Models
  (under AI Act Article 56)
- UN AI Advisory Body Final Report 2024 + UN GA
  AI Resolution 2024 + UN GDC (Global Digital
  Compact) AI provisions
- International AI Safety Report 2025 (chaired by
  Yoshua Bengio under the AI Action Summit)
- OECD AI Principles 2019 + 2024 update
- UNESCO Recommendation on the Ethics of AI 2021
- ETSI ISG SAI GR/GS series
- IEC 62443 series (industrial cyber for AI in OT)
- KR 인공지능 발전 및 신뢰 기반 조성 등에 관한 기본법
  (KR AI Basic Act) + KR AI 안전연구소 (KAISI)
  guidance + KR PIPC AI guidance

---

## §1 ISO/IEC 42001:2023 AIMS Discipline

The AIMS discipline operationalises the management-
system framework:

- Clause 4 Context — internal and external issues,
  interested parties, AIMS scope.
- Clause 5 Leadership — top-management commitment,
  AI policy, organisational roles.
- Clause 6 Planning — risks-and-opportunities
  per Clause 6.1 + Annex B (objectives at
  organisational and AI-system levels) + AI-system
  impact-assessment per Clause 6.1.4 + Annex C.
- Clause 7 Support — resources, competence,
  awareness, communication, documented information.
- Clause 8 Operation — AI-system lifecycle, data
  provenance, AI-system technical documentation.
- Clause 9 Performance evaluation — monitoring,
  internal audit, management review.
- Clause 10 Improvement — non-conformity and
  corrective action.

The AIMS scope is documented and reviewed on the
operator's published cadence (typically annual).

## §2 EU AI Act Technical-and-Organisational
       Discipline

For high-risk AI systems (Art 8-17):

- Article 9 — risk-management system across the
  entire lifecycle.
- Article 10 — data and data-governance covering
  training, validation, testing data sets.
- Article 11 — technical documentation per Annex
  IV.
- Article 12 — record-keeping (logs).
- Article 13 — transparency and information to
  deployers.
- Article 14 — human oversight.
- Article 15 — accuracy, robustness, cybersecurity.
- Article 16 — providers' obligations.
- Article 17 — quality management system.
- Article 27 — Fundamental Rights Impact Assessment
  for deployers in scope.

For GPAI models (Art 51-55):

- Article 51 — classification rules + thresholds
  (Annex XIII compute threshold currently 10²⁵
  FLOPs).
- Article 53 — providers' obligations (technical
  documentation Annex XI, content-summary,
  copyright respect, deployer-information).
- Article 55 — additional obligations for GPAI
  models with systemic risk (model-evaluation,
  systemic-risk assessment and mitigation, serious
  incidents tracking, cybersecurity).
- Article 56 — Code of Practice (the published code
  is the conformance route until harmonised
  standards are available).

## §3 NIST AI RMF Discipline (Govern / Map / Measure
       / Manage)

The NIST AI RMF four-function discipline:

- Govern — organisational AI risk-management
  policies, accountability structures, and culture.
- Map — context establishment, system
  categorisation, risk identification.
- Measure — AI risk metrics, performance evaluation
  per ISO/IEC TS 4213, fairness metrics per ISO/IEC
  TR 24027.
- Manage — risk-treatment planning, monitoring,
  documentation, incident response.

The NIST AI 600-1 GenAI Profile applies the four
functions to generative-AI risks; the NIST IR 8332
provides implementation guidance.

## §4 Responsible-Scaling and Frontier-Preparedness
       Discipline

For frontier-AI providers and operators of GPAI
systems with systemic risk:

- Responsible-scaling policy — a published policy
  declaring the operator's deployment guardrails
  by capability threshold.
- Capability elicitation evaluation — the operator
  exercises capability elicitation across the
  declared risk dimensions (cyber-offense, CBRN
  uplift, autonomy, persuasion, financial-crime).
- Pre-deployment review — pre-deployment evaluation
  + safety case + override / pause provisions.
- Voluntary commitments — Seoul AI Summit 2024 /
  AI Action Summit Paris 2025 voluntary
  commitments to AI Safety Institute pre-
  deployment testing.
- AI Safety Institute MOUs — US AISI / UK AISI /
  EU AI Office / KAISI memoranda of understanding
  for pre-deployment red-teaming where in place.

## §5 Workforce-Transition Discipline

The workforce-transition discipline:

- Affected-headcount assessment through the
  operator's HR analytics.
- Reskilling and upskilling programme delivery
  (formal training + apprenticeship + work-
  integrated learning).
- Redeployment internal — preferential transfer
  pipelines for incumbent employees to AI-augmented
  roles.
- Severance and exit support per the operating
  jurisdiction's labour law.
- Sectoral coordination — sectoral training funds,
  trade-union engagement, public-employment-agency
  coordination.

## §6 Safety-and-Security Testing Discipline

The testing discipline:

- Capability benchmarks — public benchmarks (MMLU,
  GPQA, SWE-Bench, HumanEval) + operator-published
  internal benchmarks.
- Robustness tests per ISO/IEC 24029-1/-2.
- Fairness tests per ISO/IEC TR 24027.
- Trustworthiness tests per ISO/IEC TR 24028.
- Adversarial red-team rotation by an independent
  internal team and external red-team firms.
- Cyber tabletop exercises against the operator's
  AI infrastructure.
- CBRN uplift evaluation for frontier models.
- Autonomy and persuasion evaluations.
- Supply-chain-attack simulation per IEC 62443.

## §7 Incident-Reporting Discipline

The incident discipline:

- EU AI Act Article 73 serious-incident reporting
  to the Member-State competent authority — death,
  serious harm to health, serious harm to property
  or environment, fundamental-rights infringement,
  serious cybersecurity breach.
- US AISI notification under the AISI Consortium
  voluntary commitments.
- KR AI 기본법 incident reporting (the KR AI
  Basic Act incident-reporting discipline once in
  force on 22 January 2026).
- Internal incident review with root-cause analysis
  and corrective-action planning.

## §8 Supply-Chain-AI-Integrity Discipline

The supply-chain-AI-integrity discipline:

- CycloneDX v1.6 ML profile SBOMs for foundation
  models, fine-tune adapters, training datasets,
  evaluation benchmarks, inference runtimes, ML
  frameworks, hardware accelerators.
- Provenance attestation through in-toto +
  Sigstore for the model-and-component supply
  chain.
- Model card per HuggingFace Model Card convention
  + datasheet for datasets per the Datasheets for
  Datasets convention.
- Vendor-and-supplier risk assessment for upstream
  AI components.

## §9 Dual-Use and Export-Control Discipline

The dual-use discipline:

- EU Dual-Use Reg (EU) 2021/821 — for AI software
  with cyber-surveillance / dual-use potential.
- US EAR 15 CFR Part 742 — for export-controlled
  AI training compute and model weights.
- US ITAR 22 CFR Parts 120-130 — for AI software
  on the US Munitions List.
- Wassenaar Arrangement cyber-tools controls.
- KR 대외무역법 + 전략물자관리원 controls.
- Per-shipment / per-licence destination-country
  sanctions screening (US OFAC + EU + UN + KR).

## §10 International-Coordination Discipline

The international-coordination discipline:

- UN AI Advisory Body — operator engagement with
  the Advisory Body's recommendations from the
  Final Report 2024 (Governing AI for Humanity).
- GPAI (Global Partnership on AI) — operator
  participation in the GPAI working groups.
- OECD AI Network of Experts — operator
  participation in the OECD AI policy observatory.
- UNESCO — operator alignment with the UNESCO
  Recommendation 2021 ethics principles.
- AI Safety Institute Network — bilateral and
  multilateral evaluation cooperation (US AISI +
  UK AISI + EU AI Office + KAISI + Singapore +
  Japan AISI).

## §11 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every AI-
system deployment / retirement, AIMS update, safety-
test completion, incident report, frontier-policy
amendment, supply-chain-attestation update, dual-
use-licence-acquisition, and supervisory
correspondence.

## §12 Cybersecurity-and-Adversarial-Robustness
        Discipline

For AI-system cybersecurity:

- IEC 62443 zone-and-conduit applied where AI is
  embedded in OT.
- ETSI ISG SAI mitigation strategies covering data
  poisoning, model extraction, evasion attacks,
  membership inference, model inversion.
- Adversarial robustness evaluation per ISO/IEC
  24029 + the ETSI SAI threat-ontology.
- Prompt-injection and jailbreak mitigation for
  generative-AI deployments per OWASP LLM Top 10.
- Supply-chain attack mitigation via SBOM verification
  + Sigstore attestation chain.

## §13 Privacy-Enhancing Technologies (PETs)
        Discipline

For AI training and inference handling personal
data:

- Differential privacy (DP) — operator's published
  ε / δ parameters for DP-trained models.
- Federated learning — operator's federated-learning
  protocol with secure aggregation.
- Confidential computing — TEE-backed inference
  (Intel SGX / TDX, AMD SEV-SNP, NVIDIA
  Confidential Computing).
- Synthetic data — operator's synthetic-data
  generation discipline + membership-inference
  testing.
- Per ISO/IEC 27559 (privacy-enhancing data
  de-identification) where applicable.

## §14 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
satisfy the ISO/IEC 42001:2023 AIMS + EU AI Act
+ NIST AI RMF baseline, exercise the responsible-
scaling and frontier-preparedness discipline for
frontier-AI providers, exercise the workforce-
transition discipline where the operator's
deployment displaces incumbent labour, satisfy the
incident-reporting discipline within the operating
jurisdiction's window, and exercise the supply-
chain-integrity + dual-use disciplines.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-ai-survival-2026
- **Last Updated:** 2026-04-29
