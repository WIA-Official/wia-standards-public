# WIA-gdpr-compliance PHASE 4 — INTEGRATION Specification

**Standard:** WIA-gdpr-compliance
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a controller (or processor)
integrates with the systems that surround GDPR
compliance: the lead supervisory authority and other
supervisory authorities under the one-stop-shop
mechanism; the European Data Protection Board (EDPB);
the European Data Protection Supervisor (EDPS) for EU
institutions; the controller's joint controllers and
processors; the controller's external auditors and
ISO/IEC 27701 certification body; the data subjects
through Article 15 access exports and Article 20
portability exports; long-term archives; and the
controller's industry sectoral codes-of-conduct and
certification body where adopted.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Lead Supervisory Authority Integration

The lead supervisory authority (Article 56) is the DPA
of the controller's main establishment for cross-border
processing. Integration carries the lead DPA's
identifier, the per-jurisdiction notification-channel
endpoint (each DPA publishes its breach-notification
form), the Article 60 cooperation correspondence
endpoint, and the controller's response SLA. UK-only
controllers integrate with the Information Commissioner's
Office (ICO).

## §2 Concerned Supervisory Authority Integration

Concerned supervisory authorities (Article 4(22)) — the
DPAs of EU jurisdictions where data subjects are
substantially affected by the processing — integrate
through the lead DPA per Article 60. The controller's
concerned-DPA integration carries each concerned DPA's
identifier, the per-jurisdiction transparency-material
language, and the per-correspondence cooperation
record.

## §3 European Data Protection Board (EDPB) Integration

The EDPB issues authoritative interpretive guidance
(consent, transparency, Article 6(1)(f) legitimate
interest, the international-transfer toolkit, breach
notification, data subject rights). Integration carries
the EDPB guideline registry references against which
the controller's discipline is calibrated, and the
controller's adoption-update cadence when new EDPB
guidelines are published.

## §4 European Data Protection Supervisor (EDPS) Integration

For EU-institution controllers (operating under
Regulation (EU) 2018/1725 rather than GDPR) integration
carries the EDPS identifier and the per-correspondence
record; for non-EU-institution controllers, the EDPS is
not the supervisory authority but EDPS guidance is
referenced where it informs sector practice.

## §5 Joint-Controller and Processor Integration

Joint controllers (Article 26) and processors (Article
28) integrate through executed arrangements (PHASE-1
§11). Integration carries each party's legal
identifier, the executed contract artefact reference,
the per-arrangement responsibility-matrix essence
publication URI (for joint controllers), the per-
processor sub-processor approval workflow endpoint,
and the per-arrangement audit cadence. Sub-processor
changes follow the operator's processor DPA's general-
or-specific approval clause.

## §6 Data Subject Integration (Articles 15 and 20)

Data subjects integrate through the controller's
identity-verified access channel:

- Article 15 access export packages the data subject's
  records across the activities they are involved in,
  delivered in a structured commonly-used machine-
  readable format (JSON with a portability profile, per
  the EDPB Guidelines on portability);
- Article 20 portability export uses the same format
  with an additional inter-controller-transfer
  envelope;
- Article 16 rectification and Article 17 erasure
  flows are mediated through the controller's product-
  surface request channel and the API of PHASE-2 §5.

The data subject's request channel is as easy to use
as the corresponding consent capture channel, mirroring
Article 7(3) for consent withdrawal.

## §7 ISO/IEC 27701 Certification Integration

Where the controller adopts ISO/IEC 27701:2019 PIMS
certification, integration carries the certification
body's identifier, the per-cycle certification audit
report reference, the per-finding remediation tracker
URI, and the per-cycle scope statement that defines the
PIMS scope against the controller's processing
inventory. ISO/IEC 27701 builds on ISO/IEC 27001;
controllers typically maintain both certifications in
parallel.

## §8 External Auditor Integration

External auditors (ISO/IEC 27701 surveillance auditors,
EU-approved certification body auditors, sectoral code-
of-conduct monitoring bodies, sectoral DPA-mandated
auditors) consume audit-trail exports through dedicated
client certificates. The export carries the API audit
logs for the audit window, the records of processing
activities, the breach history, the DPIA library, the
sa correspondence, and the consent and data-subject-
request samples.

## §9 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  processing-activities/       — Article 30 records
                                  (current and superseded)
  data-subject-requests/       — request and response
                                  records (per data
                                  subject; aggregate
                                  statistics for
                                  transparency reports)
  consent-records/             — consent records (per
                                  data subject)
  international-transfers/     — Chapter V transfer
                                  records
  dpia-records/                — DPIA library (current
                                  and superseded)
  breach-notifications/        — breach records (with
                                  data-subject details
                                  redacted in shareable
                                  bundle)
  sa-correspondence/           — supervisory-authority
                                  correspondence
                                  (decisions and
                                  outcomes)
  controller-arrangements/     — joint-controller and
                                  processor records
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the controller's HTTP-message-signature key
(RFC 9421) and counter-signed by the DPO when the
package supports a regulator submission.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to
the manifest, and reject the package on mismatch with
type `urn:wia:gdpr-compliance:evidence-mismatch`. Data-
subject-redaction integrity is checked separately so
that public-shareable bundles do not carry per-data-
subject content the controller has not authorised for
disclosure.

## §11 well-known URI Discovery

A conformant controller exposes a discovery document at
`/.well-known/wia-gdpr-compliance` that links to the
API root, the DPO contact, the public privacy notice
(Articles 13 and 14), the lead supervisory authority
identifier, and the data-subject-request channel.

## §12 Long-Term Archive Integration

Controllers designate a long-term archive that holds
records of processing activities, breach histories, sa
correspondence, and DPIA libraries beyond the
controller's primary retention horizon. Quarterly
deposits round-trip content-addresses; on programme
wind-down, remaining records transfer to the archive
with content-addresses preserved subject to the
operating jurisdiction's records-retention rules.

## §13 Verifiable-Credential Re-Issuance (optional)

Controllers that wish to expose attestations (ISO/IEC
27701 certification, sectoral certification under
Article 42, BCR approval under Article 47, code-of-
conduct adherence under Article 40) to consumers of
W3C Verifiable Credentials MAY re-issue the
attestations as Verifiable Credentials under the Data
Model 2.0 specification. Re-issuance is optional; the
canonical record remains the JSON evidence-package
manifest.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers that
disconnect during sa-correspondence windows resume from
the last seen event identifier without losing
visibility of priority-1 events (breach declared, sa
inquiry opened, transfer mechanism suspended).

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through a
deprecation window of at least one full audit cycle so
that DPA, joint-controller, processor, and auditor
integrations have time to migrate.

## §16 Cross-Standard Linkage

Controllers that consume adjacent WIA standards (WIA-
ai-content-moderation for transparency-report
discipline that intersects with DSA Article 15, WIA-ai-
assistant for assistant-output processing that may
involve personal data, WIA-credit-scoring for credit-
decision processing that triggers Article 22, WIA-
healthcare-integration for health-data processing that
triggers Article 9) emit cross-standard linkage records.

## §17 Reader Tooling

Controllers MAY publish supplementary reader tools
(Article 30 register browser, public privacy-notice
search, breach-disclosure archive, DPIA library
browser, sa-correspondence outcome catalogue)
alongside the canonical evidence package; the tools are
non-normative.

## §18 Public Catalogue Feed

Controllers publish a public catalogue feed listing the
in-force public privacy notice, the DPO contact, the
representative under Article 27 (where applicable), the
lead supervisory authority designation, and the
sectoral codes-of-conduct or certification mechanisms
to which the controller adheres. The feed enables
data-subject and civil-society discovery of the
controller's GDPR posture.

## §19 Sectoral Code-of-Conduct Integration

Where the controller adheres to a sectoral code of
conduct under Article 40 (e.g. cloud computing,
pharmaceuticals, marketing, consumer credit), the code-
of-conduct monitoring body consumes the per-controller
adherence statement and the per-cycle compliance
attestation. Integration carries the monitoring body's
identifier, the per-code adherence cycle, and the per-
cycle compliance review outcome.

## §20 EU Cross-Border Cooperation (Article 60 and 65)

EU one-stop-shop cooperation across DPAs is mediated
via the lead-DPA's IMI (Internal Market Information)
system or equivalent EDPB-coordinated channel. Article
65 dispute-resolution outcomes (binding decisions
issued by the EDPB) are recorded in the controller's
sa correspondence with the per-decision compliance
deadline. Integration carries the EDPB-decision
identifier, the per-decision controller compliance
plan, and the per-decision implementation evidence.

## §21 Public Catalogue Aggregator Integration

Civil-society researchers and academic-research
consortia consume aggregate sa-correspondence outcomes
and aggregate breach statistics for independent
analysis. Integration carries the consumer's
identifier, the per-research-purpose data-access
agreement (Article 89 research-purpose discipline
where applicable), and the controller's publication of
consumer-attribution in any derivative research output.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the lead supervisory authority of the
controller's main establishment, with each joint
controller and processor under executed arrangements,
with at least one external auditor (ISO/IEC 27701 or
sectoral), with a long-term archive, and has published
at least one externally citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-gdpr-compliance
- **Last Updated:** 2026-04-28
