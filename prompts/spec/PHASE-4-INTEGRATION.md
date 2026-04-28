# WIA-prompts PHASE 4 — Integration Specification

**Standard:** WIA-prompts
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-prompts integrates
with adjacent ecosystems (model vendors,
evaluation labs, deployment platforms, sovereign
AI regulators, and downstream WIA standards), how
conformance evidence is produced, and how
deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO/IEC 23053:2022, ISO/IEC 23894:2023, ISO/IEC 42001:2023
- ISO/IEC 27001:2022, ISO/IEC 27701:2019
- IETF RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)
- NIST AI Risk Management Framework 1.0
- EU AI Act (Regulation (EU) 2024/1689)
- W3C VC 2.0, W3C SHACL
- OWASP LLM Top 10 (informative)
- MITRE ATLAS (informative)

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-prompts consumes upstream
specifications (NIST AI RMF, EU AI Act, ISO/IEC AI
series), how downstream WIA standards reference
prompt artefacts, and how conformance is assessed.

## §2 Upstream integration

### 2.1 ISO

ISO/IEC 23053 (ML framework), 23894 (AI risk),
and 42001 (AI management system) are consumed at
their published versions. Records align with the
ISO/IEC 42001 governance lifecycle (Plan-Do-
Check-Act).

### 2.2 NIST

The NIST AI Risk Management Framework 1.0
provides the four functions (`govern`, `map`,
`measure`, `manage`) that safety-policy records
reference.

### 2.3 EU AI Act

Prompts deployed in EU territory under high-risk
classifications (Annex III) carry a conformity
assessment per Article 43. The model card record
links to the assessment.

### 2.4 OWASP / MITRE

OWASP LLM Top 10 and MITRE ATLAS catalogue
adversarial patterns that safety policies
reference.

## §3 Model vendor integration

Model vendors (OpenAI, Anthropic, Google,
sovereign-equivalent providers) integrate via the
model-card endpoint. Vendors publish version tags
that pin to specific deployed model snapshots so
that evaluation results are reproducible.

## §4 Deployment platform integration

Deployment platforms consume prompts and model
cards via the conformance endpoint. The platform
records the model and prompt versions used per
conversation so that auditors can reproduce
behaviour.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared model card with version pin;
- declared evaluation suite with reference
  results;
- declared safety policies with mitigation
  evidence;
- the test-vector matrix per Annex G of each
  PHASE;
- the JWS signing key set used.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. Model cards link to the published safety
   evaluation and fairness audit.
2. Safety policies cover prompt-injection,
   PII redaction, and topic restriction at
   minimum.
3. Evaluation results are reproducible via the
   declared reproducer URL.
4. Conversation-state records carry the model
   version pin used.
5. EU AI Act conformity assessment is current
   where applicable.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                  | Integration point                       |
|---------------------------|-----------------------------------------|
| WIA-language-bridge       | multilingual prompt translations        |
| WIA-learning-analytics    | prompt-mediated tutoring analytics      |
| WIA-plugins               | plugin-host prompt artefacts            |
| WIA-multiverse-interface  | research prompts                        |
| WIA-lms                   | LTI-tool LLM tutors                     |

## §7 Privacy

Personal data flowing through prompts is
processed under the deployment's privacy regime.
Pseudonymisation is the default; PII fields are
redacted at logging time. Subject access requests
follow GDPR Article 15 / K-PIPA Article 35.

## §8 Security

Prompt-injection mitigation is mandatory for
prompts whose template's `safeRendering: false`
or whose user-supplied parameters lack a
sanitiser. The registry surfaces such prompts in
the audit feed for risk-officer review.

## §9 EU AI Act conformity

High-risk prompt deployments publish a conformity
assessment per Article 43 referencing:

- the technical documentation per Annex IV;
- risk management system per Article 9;
- data and data governance per Article 10;
- transparency to users per Article 13;
- human oversight per Article 14;
- accuracy and robustness per Article 15;
- post-market monitoring per Article 72.

## §10 Localisation

Prompts and model cards are localised in BCP 47
form. Multilingual prompts share a `promptRef`
across language versions; each language version
carries its own signature.

## §11 Accessibility

Prompt-generated outputs MUST honour the
operator's accessibility commitments (alt text on
generated images, captions on generated audio,
plain-language explanations on technical content).
Accessibility evidence URLs are recorded on the
model card.

## §12 Open governance

Issues at
`github.com/WIA-Official/wia-standards/issues`
with the `prompts` label.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence
URL from the README.

## Annex B — Worked safety policy (informative)

```json
{
  "policyRef": "https://reg.example.org/safety/pi-mit-1.0",
  "kind": "prompt-injection-mitigation",
  "appliesTo": "https://reg.example.org/templates/qa-1.0",
  "referenceFramework": "manage",
  "evidence": "https://reg.example.org/safety/pi-mit-1.0/evidence.json"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with ISO/IEC AI series and EU AI Act
revisions.

## Annex D — Open governance

Decision logs are published alongside release
notes for the registry.

## Annex E — Withdrawal procedure

Tombstone the evidence package; tombstones are
immutable.

## Annex F — Reproducibility

Evidence is reproducible from publicly available
inputs: prompt versions, model card pins,
evaluation suites, and the registry's signing
key set.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative
vector under `tests/phase-vectors/`.

## Annex H — Sustainability

Operators SHOULD declare their per-conversation
energy estimate using the methodology of ISO
14064 or the Green Software Foundation SCI
specification. Estimates feed into the
operator's sustainability dashboard.

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Prompt injection                      | Mandatory mitigation     |
| PII leak in conversation              | Redaction policy         |
| Model card / version drift            | Version pin enforced     |
| Hallucinated tool calls               | Output-contract validate |
| Adversarial fairness drift            | Quarterly fairness audit |
| Evaluation result tampering           | Signature verification   |

## Annex J — Industry binding catalogue

| Industry segment   | Bound profile                         |
|--------------------|---------------------------------------|
| Healthcare         | HIPAA-aware redaction, FHIR mapping   |
| Legal              | LegalRuleML rule citation in prompts  |
| Education          | LTI 1.3 tutor prompts                 |
| Finance            | sovereign FRAP / EBA guideline        |
| Government         | sovereign AI procurement profile      |

## Annex K — Continuous improvement programme

Each deployment publishes an annual improvement
plan addressing prompt-injection findings,
fairness drift, and EU AI Act post-market
monitoring outputs.

## Annex L — Reference implementation

A reference implementation is published under
Apache-2.0 at the WIA Standards GitHub umbrella
under `wia-prompts-reference`, covering the full
PHASE contract including the prompt-injection
scanner and the SSE conversation streamer.

## Annex M — Vendor neutrality

WIA-prompts does not endorse a particular model
vendor. The conformance programme is open to
commercial, open-source, and sovereign-developed
implementations on identical terms. Multi-vendor
deployments record the model used per turn so
that vendor switches preserve the audit trail.

## Annex N — Open-source SDK catalogue

Reference SDKs are published per language at
`/v1/registry/sdks` (Python, TypeScript, Go, Rust)
under permissive open-source licenses.

## Annex O — Researcher access programme

Operators may make anonymised conversation
extracts available to academic AI-safety
researchers under a documented data-use
agreement. Agreements are catalogued at
`/v1/registry/researcher-access`.

## Annex P — Annual ecosystem report

The registry publishes an annual ecosystem
report summarising prompt counts by language,
model coverage, evaluation suite adoption,
safety-policy coverage, and EU AI Act
conformity-assessment freshness.

## Annex Q — Disaster recovery

Registry deployments declare RPO ≤ 24h and RTO ≤
8h. DR drills run annually with results in the
audit feed.

## Annex R — Continuous fairness audit

Models in production undergo quarterly fairness
audits. Audit reports are signed by the auditor
and published at the model card's
`fairnessAudit` URL. Audits cover unequal
performance across protected categories per the
deployment's privacy regime.

## Annex S — Vendor certification programme

Vendors apply to the certification programme by
submitting their evidence package. Certified
vendors are catalogued at
`/v1/registry/certified-vendors`.

## Annex T — Annual governance review

The registry hosts an annual governance review
where publishers, vendors, and auditors discuss
emerging needs (new mitigation patterns, evolving
regulatory expectations, model-card schema
extensions). Review minutes are published
alongside release notes.

## Annex U — Industry binding catalogue

| Segment       | Bound profile                              |
|---------------|--------------------------------------------|
| Healthcare    | HIPAA-aware redaction, FHIR mapping        |
| Legal         | LegalRuleML in compliance prompts          |
| Education     | LTI 1.3 tutor prompts                      |
| Government    | sovereign AI procurement profile           |
| Finance       | sovereign FRAP / EBA guidelines             |

## Annex V — Open governance forum

A quarterly open governance forum is held online.
Forum minutes are published alongside release
notes for the registry.

## Annex W — Sovereign AI procurement

Sovereign AI procurement frameworks (UK GDS, US
NSA-AISI, KR AI procurement guideline) reference
the WIA-prompts contract for vendor lock-in
prevention. Sovereign procurement officers verify
the deployment's evidence package as part of the
RFP review.

## Annex X — Multilingual evaluation

Evaluation suites operating across BCP 47 tags
publish per-locale metric breakdowns so that
multilingual deployments do not mask single-
locale regressions in aggregate scores.

## Annex Y — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Prompt injection                      | Mandatory mitigation     |
| PII leak in logs                      | Redaction at log time    |
| Tool-call injection                   | Output-contract validate |
| Model version drift                   | Version pin enforced     |
| Adversarial fairness drift            | Quarterly fairness audit |
| Conversation replay                   | Idempotency + nonces     |
| Webhook replay                        | HMAC + sequence number   |

弘益人間 (Hongik Ingan) — Benefit All Humanity
