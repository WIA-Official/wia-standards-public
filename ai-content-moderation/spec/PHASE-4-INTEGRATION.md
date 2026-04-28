# WIA-ai-content-moderation PHASE 4 — INTEGRATION Specification

**Standard:** WIA-ai-content-moderation
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an AI-content-moderation
operator integrates with the systems that surround it:
EU Digital Services Coordinators (DSCs); the European
Commission's DSA enforcement office; the EU AI Office
for AI Act enforcement; NCMEC CyberTipline; UK Ofcom
under the Online Safety Act; KR KCC and KISA; trusted-
flagger civil-society organisations; out-of-court
dispute settlement bodies; C2PA Content Credentials
authentication services; and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- C2PA Content Credentials specification
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 EU Digital Services Coordinator Integration

EU DSCs (per Member State) coordinate DSA enforcement
within their jurisdiction. Integration carries the DSC's
identifier, the per-jurisdiction DSC contact reference,
the DSC's data-access request endpoint per Article 40,
and the operator's response SLA. The European Commission
acts as the DSC for VLOPs.

## §2 EU AI Office Integration

The EU AI Office enforces the EU AI Act 2024. For
moderation systems classified as high-risk under
Annex III, integration carries the Office's identifier,
the per-system conformity-assessment report, the post-
market monitoring report cadence, and the incident-
notification intake per Article 73.

## §3 NCMEC CyberTipline Integration

NCMEC operates the CyberTipline as the US statutory
recipient for CSAM reports per 18 U.S.C. § 2258A.
Integration carries NCMEC's identifier, the operator's
electronic-service-provider registration, the per-report
NCMEC report identifier, the per-content-class submission
template, and the per-report acknowledgement workflow.

## §4 UK Ofcom Integration

For UK operations, Ofcom enforces the Online Safety Act
2023. Integration carries Ofcom's identifier, the per-
service-class duty-of-care attestation, and Ofcom's
information-notice intake under the operator's
regulatory-cooperation discipline.

## §5 KR KCC and KISA Integration

For Korea operations, the Korea Communications Commission
(KCC) and the Korea Internet & Security Agency (KISA)
oversee content-moderation under the Information and
Communications Network Act and the Telecommunications
Business Act intermediary provisions. Integration
carries the regulator's identifier, the per-removal-order
acknowledgement, and the per-quarter compliance reporting
cadence.

## §6 Trusted-Flagger Civil-Society Integration

Trusted flaggers awarded status by EU DSCs (per DSA
Article 22) integrate with the operator through a
priority-queue endpoint. Integration carries the
flagger's DSC-issued identifier, the per-flagger flag
submission template, the per-flagger feedback loop, and
the per-quarter aggregate-flag-statistics report.

## §7 Out-of-Court Dispute Body Integration

Out-of-court dispute settlement bodies (DSC-certified per
DSA Article 21) consume escalated appeals. Integration
carries each body's identifier, the per-body procedure
reference, the per-case binding decision intake, and the
operator's compliance workflow when a body's decision
requires action.

## §8 C2PA Content Credentials Integration

The C2PA framework provides content-provenance manifests
that platforms can verify to attribute content to its
source and capture chain. Integration carries the C2PA
verification service's identifier, the per-content
manifest validation endpoint, and the per-trust-list
refresh cadence.

## §9 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  policy-versions/             — policy revisions in force
  content-items/               — per-content metadata
                                  (artefacts content-
                                  addressed and gated by
                                  per-jurisdiction
                                  privacy policy)
  classifier-outputs/          — per-content classifier
                                  outputs
  reviewer-decisions/          — per-content human reviewer
                                  decisions
  appeals/                     — appeal records and
                                  resolutions
  statutory-escalations/       — escalation records
                                  (CSAM-specific records
                                  gated by NCMEC discipline)
  transparency-reports/        — published transparency
                                  reports
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the operator's HTTP-message-signature key
(RFC 9421) and counter-signed by the operator's trust-
and-safety officer when the package supports a regulator
submission.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to
the manifest, and reject the package on mismatch with
type `urn:wia:ai-content-moderation:evidence-mismatch`.

## §11 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-ai-content-moderation` that links to
the API root, the operator's governing-framework
enrolments, the published policy-version register, the
transparency-report archive, and the per-jurisdiction
regulator binding.

## §12 Long-Term Archive Integration

Operators designate a long-term archive that holds
policy versions, transparency reports, and statutory-
escalation records beyond the operator's primary
retention horizon. Quarterly deposits round-trip
content-addresses; on programme wind-down, remaining
records transfer to the archive with content-addresses
preserved.

## §13 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (DSA
compliance, EU AI Act conformity-assessment outcome,
ISO/IEC 42001 certification, ISO/IEC 27001 certification)
to consumers of W3C Verifiable Credentials MAY re-issue
the attestations as Verifiable Credentials under the
Data Model 2.0 specification. Re-issuance is optional;
the canonical record remains the JSON evidence-package
manifest.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers that
disconnect during long content-classification windows
resume from the last seen event identifier without
losing visibility of priority-1 events (statutory
escalations, regulator notices, trusted-flagger
priority queue refresh).

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through a
deprecation window of at least one full DSA
transparency-report cycle so that regulator and
trusted-flagger integrations have time to migrate.

## §16 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-ai-
assistant for assistant-output moderation, WIA-
generative-ai for generative-AI output safety, WIA-
images for content-provenance manifest handling, WIA-
intent-lang for declarative moderation intent) emit
cross-standard linkage records.

## §17 Reader Tooling

Operators MAY publish supplementary reader tools (per-
programme transparency-report dashboards, per-policy
appeal-rate trend charts, per-classifier accuracy
dashboards, per-reviewer-cohort wellness summaries)
alongside the canonical evidence package; the tools are
non-normative.

## §18 Public Catalogue Feed

Operators publish a public catalogue feed listing the
in-force policy version, the transparency-report
archive, and the per-quarter aggregate moderation
statistics. The feed enables researcher and civil-
society discovery of the operator's moderation
behaviour over time.

## §19 Subject-Access and Erasure Workflow Integration

Per GDPR Article 15-22 and equivalent rules, users may
access their content-moderation records and (in some
cases) request erasure. The operator's CRM mediates
data-subject rights requests into the WIA platform per
the operator's response SLA.

## §20 Crisis-Response Coordination Integration

For events of widespread coordinated harm (election-
period coordinated inauthentic behaviour, mass-violence
event coverage, public-health emergency mis-information),
the operator participates in industry-wide coordination
through bodies like the Global Internet Forum to Counter
Terrorism (GIFCT) hash-sharing infrastructure and per-
event coordination calls. Integration carries the
coordination body's identifier, the per-event
participation record, and the operator's coordination-
output adoption decision.

## §21 Hash-Sharing Database Integration

Hash-sharing databases (GIFCT terrorism-content hash
database, NCMEC PhotoDNA hash database, StopNCII intimate-
imagery hash database, equivalent industry-shared
infrastructures) consume operator submissions and
distribute hashes to participating platforms. Integration
carries each database's identifier, the per-content-
class submission template, the per-database membership
agreement, and the operator's hash-ingest pipeline that
applies received hashes to subsequent moderation
decisions.

## §22 Audit-Reviewer Workflow Integration

External audit reviewers (DSA-mandated independent
auditors per Article 37 for VLOPs, ISO/IEC 42001
surveillance auditors, civil-society audits per the
operator's voluntary commitments) consume audit-trail
exports through dedicated client certificates. The
export carries the API audit logs for the audit window,
the policy-version revisions, the classifier-output
sample, the reviewer-decision sample, and the appeal
disposition statistics.

## §23 Public Catalogue Aggregator Integration

Civil-society researchers, academic-research consortia,
and journalist organisations (Crowdtangle-equivalent
open-research feeds, social-platform research datasets
under DSA Article 40 access regimes) consume aggregate
moderation statistics for independent analysis.
Integration carries the consumer's identifier, the per-
research-purpose data-access agreement, and the operator's
publication of consumer-attribution in any derivative
research output.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the operator's primary jurisdiction's
content-moderation regulator (EU DSC for EU operators,
Ofcom for UK operators, KCC for Korean operators,
equivalent), the relevant CSAM authority (NCMEC or
national equivalent), at least one trusted-flagger
organisation, the C2PA verification service (where the
operator binds C2PA), and at least one long-term
archive, and has published at least one externally
citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-ai-content-moderation
- **Last Updated:** 2026-04-28
