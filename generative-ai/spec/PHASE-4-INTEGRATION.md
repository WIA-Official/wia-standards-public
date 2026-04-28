# WIA-generative-ai PHASE 4 — INTEGRATION Specification

**Standard:** WIA-generative-ai
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a generative-AI operator
integrates with the systems that surround the model
and deployment lifecycle: the model registry and
artefact storage (MLflow + ONNX); the model card and
datasheet publication channel (HuggingFace and
equivalent registries); the C2PA Content Credentials
trust list and provenance verification ecosystem; the
EU AI Office, Member-State market-surveillance
authority, and the EU database for high-risk AI
systems (AI Act Article 71); the US sector regulator,
US OMB AI use-case inventory channel, and the
National Institute of Standards and Technology AISI
red-team programme; the KR PIPC, NIA, and FSC
oversight surfaces; the deployer's downstream
integration channel; the external auditor and the
ISO/IEC 42001 + ISO/IEC 27001 certification body;
the deployer's customer channels; and the long-term
archive that preserves the technical-documentation
set past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 17021-1:2015 (audit and certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0
- C2PA Content Credentials specification v1.4
- HuggingFace Model Card conventions
- MLflow registry conventions
- ONNX (open neural-network exchange)
- NIST AI RMF + NIST AI 600-1 GenAI Profile
- US EO 14110, US OMB M-24-10
- EU AI Act Articles 25, 26, 49 (registration), 71
  (database), 72, 73, 74 (market surveillance), 86
  (right to lodge a complaint)
- EU Code of Practice for GPAI Models
- EU GDPR Articles 9, 15, 17, 20, 22, 33, 46
- KR AI 산업진흥법, KR PIPA Article 28
- OWASP Top 10 for LLM Applications

---

## §1 Model Registry and Artefact Storage Integration

The operator integrates with:

- The model registry (MLflow registry or the
  operator's equivalent) — every promoted model
  carries the registry record (PHASE-1 §3), the
  evaluation report references, and the approval
  decisions.
- The artefact storage — the model weights, the
  tokenizer, the configuration, and (where the
  release licence permits) the ONNX export are
  preserved in the artefact store; the SHA-256
  weights digest is recorded so that the integrity
  of the deployed model can be verified at load
  time.
- The training-data manifest — the dataset's
  manifest is preserved alongside the model in the
  long-term archive so that AI Act Article 18
  ten-year retention applies.

## §2 Model Card and Datasheet Publication Integration

The operator publishes the model card (HuggingFace
format or equivalent) and the datasheet for datasets
on the registry channel registered with the deployer:

- The model card carries the model's intended use,
  out-of-scope use, training data summary,
  evaluation results, ethical considerations, and
  caveats.
- The datasheet carries the dataset's motivation,
  composition, collection process, preprocessing,
  uses, distribution, and maintenance.
- For GPAI models the AI Act Article 53(1)(a) and
  53(1)(c) public-disclosure obligations are
  satisfied through the model card / datasheet
  channel and through the operator's training-
  content summary.

## §3 C2PA Content Credentials Trust-List Integration

The operator integrates with the C2PA trust list:

- The operator's signing key is registered with the
  C2PA trust list so that downstream verifiers can
  validate the manifest signatures.
- Provenance verification endpoints (the operator's
  PHASE-2 §6 verify endpoint) consult the trust
  list at verification time.
- The C2PA Content Credentials manifests issued by
  the operator follow the C2PA v1.4 schema,
  carrying the required claim generators, claims,
  and assertions.

## §4 EU AI Office and Member-State Authority Integration

For EU-regulated operators:

- GPAI providers integrate with the EU AI Office
  for the Article 53 / 55 obligations and for the
  Code of Practice signatory channel; serious
  incidents under Article 55(1)(c) are reported.
- High-risk-system providers register the system in
  the EU database under AI Act Article 49 / 71
  before placing on the market; the database carries
  the system's identification, the provider's
  identification, the high-risk classification, and
  the conformity-assessment-procedure references.
- High-risk-system deployers register their use of
  the system in the EU database under Article 49(1)
  for Annex III §1 / §6 use cases.
- Serious incidents under Article 73 are reported
  to the relevant national market-surveillance
  authority and (for cross-border serious risks) to
  the AI Office.
- The Article 86 right to lodge a complaint surface
  is exposed to natural persons who consider that
  the AI Act has been infringed.

## §5 US Sector Regulator and AISI Integration

For US-regulated operators:

- US sector regulators (FDA for medical AI, EEOC
  for hiring-AI, CFPB for credit-AI, FTC for
  consumer-AI, NHTSA for automotive-AI, etc.)
  exercise their sector-specific authority over the
  generative-AI system's deployed surface.
- The US AI Safety Institute (AISI), under the
  EO 14110 framework and the operator's voluntary
  participation, performs red-team evaluations on
  frontier-class models; the operator's integration
  channel handles the AISI's evaluation requests.
- The OMB M-24-10 use-case inventory channel is the
  integration boundary for federal-agency
  deployments.

## §6 KR Authority Integration

For KR-regulated operators:

- KR PIPC (Personal Information Protection
  Commission) is the integration counterparty for
  the GDPR-equivalent Article 28 automated-
  decision-making rights and for personal-data
  processing.
- KR NIA (National Information society Agency)
  oversees the KR AI 산업진흥법 framework.
- KR FSC oversees AI deployments in financial
  services.
- KR FTC (공정거래위원회) oversees AI deployments
  with consumer-protection implications.

## §7 Downstream-Deployer Integration

For model providers integrating with downstream
deployers:

- The provider's instruction-for-use under AI Act
  Article 13 / 25 carries the intended purpose,
  the limitations, the human-oversight requirements,
  and the cybersecurity guidance; downstream
  deployers integrate this guidance into their
  own AI Act Article 26 deployer obligations.
- The provider's transparency obligations under
  Article 25 to deployers are exercised through a
  documented integration channel — typically a
  versioned model card distribution and a
  provider-deployer contract.
- The provider's serious-incident reporting under
  Article 73 covers incidents occurring at any
  deployer; the deployer's notification channel
  back to the provider is documented.

## §8 External Audit and ISMS / AIMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022; the AIMS is certified against ISO/IEC
42001:2023. Certification scope explicitly extends
to the inference, registry, and content-credentials
endpoints. The certification body operates under
ISO/IEC 17021-1; the conformity-assessment body for
WIA-generative-ai operates under ISO/IEC 17065.

## §9 Customer-Channel Integration

The operator's customer channels include:

- The chat-style end-user channel (web, mobile, API
  console) — the user-facing surface where
  inference is exercised.
- The enterprise API — the programmatic surface
  where downstream products integrate the model.
- The plugin / agent ecosystem — third-party tools
  registered with the system's tool catalogue
  through the operator's plugin-review surface.
- The educational and creative channels — the
  operator's published guides, tutorials, and
  prompt libraries.

Each channel exercises the AI Act Article 50
disclosure discipline appropriately and routes
user-rights requests through the PHASE-2 §7 user-
rights endpoint.

## §10 Long-Term Archival Integration

Records governed by the operator's retention
horizons (EU AI Act Article 18 ten-year retention
for high-risk-system technical documentation and
quality-management records; GDPR Article 5(1)(e)
no-longer-than-necessary for personal data; KR
PIPA retention discipline) are migrated to the
long-term archive. The archive preserves the
technical documentation, the training-data manifest,
the model weights digest, the evaluation reports,
the serious-incident records, the corrective-action
records, the C2PA manifests issued, and the audit-
event trail.

## §11 Open-Source and FOSS-GPAI Discipline

For models released as free and open-source
software the EU AI Act Article 53(2) exempts the
provider from certain GPAI obligations (Article
53(1)(a) and (b)) — provided the model is not
classified as having systemic risk and the parameter
weights, model architecture, and information on the
use of the model are made publicly available.
Operators relying on this exemption document the
licence, the public-availability channel, and the
absence of a systemic-risk designation; the
operator's discipline still covers Article 53(1)(c)
copyright respect and Article 53(1)(d) training
content summary unless the further exemption applies.

## §12 Conformance

Implementations claiming PHASE-4 conformance maintain
the model registry and artefact storage integration,
publish the model card and datasheet through the
public channel, register the system in the EU database
where AI Act Article 49 / 71 applies, integrate with
the operating jurisdiction's regulator-examination
channels, hold the ISO/IEC 42001 + ISO/IEC 27001
certifications, and operate the long-term archival
integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-generative-ai
- **Last Updated:** 2026-04-28
