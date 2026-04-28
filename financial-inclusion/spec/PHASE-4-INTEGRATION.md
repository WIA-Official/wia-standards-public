# WIA-financial-inclusion PHASE 4 — INTEGRATION Specification

**Standard:** WIA-financial-inclusion
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a financial-inclusion
programme integrates with the systems that surround
it: the operating jurisdiction's financial-inclusion
strategy coordinator; the operating jurisdiction's
central bank or financial-inclusion authority; the
operating jurisdiction's consumer-protection
regulator (US CFPB at federal level, EU Member State
consumer authorities, KR FSC's consumer-protection
arm); the operating jurisdiction's AML/CFT supervisor
and FIU (cross-walked with WIA-anti-money-laundering);
the operating jurisdiction's alternative dispute
resolution body; the operating jurisdiction's credit-
bureau network (where the programme reports credit
data); civil-society partner organisations; the
World Bank Findex collection cycle; the Alliance for
Financial Inclusion (AFI) data-collection cycle (where
the operating jurisdiction is an AFI member); and
long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO 8601 (date and time)
- ISO 20022 (financial-services messaging)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Financial-Inclusion Strategy Coordinator Integration

The operating jurisdiction's financial-inclusion
strategy coordinator (the agency the operating
jurisdiction designates to lead the National
Financial Inclusion Strategy — typically the central
bank, the finance ministry, or a multi-agency
coordinating body) consumes the programme's per-
period strategy-aligned reports. Integration carries
the coordinator's identifier, the per-period reporting
endpoint, the per-cycle review cooperation workflow,
and the coordinator's response SLA.

## §2 Central Bank / Financial-Inclusion Authority Integration

The operating jurisdiction's central bank or
financial-inclusion authority is the prudential and
financial-inclusion regulator. Integration carries
the authority's identifier, the per-licensing
cooperation workflow (for licensed bank, PSP, MFI,
and mobile-money-operator programmes), the per-
inspection cooperation workflow, and the per-period
prudential reporting cadence.

## §3 Consumer-Protection Regulator Integration

The operating jurisdiction's consumer-protection
regulator integrates through:

- US: CFPB at federal level for federally-supervised
  consumer-finance, plus state consumer-protection
  authorities for state-licensed entities;
- EU: each Member State's consumer-protection
  authority plus the European Commission's DG
  JUST consumer-policy unit for cross-Member-State
  matters;
- KR: FSC's consumer-protection arm and the
  operating jurisdiction's general-purpose consumer-
  protection authority (the Fair Trade Commission's
  consumer-policy bureau).

Integration carries the regulator's identifier, the
per-inquiry cooperation workflow, the per-enforcement-
action response workflow, and the per-cycle voluntary
disclosure channel.

## §4 AML/CFT Supervisor and FIU Integration

The operating jurisdiction's AML/CFT supervisor and
FIU integrate through the WIA-anti-money-laundering
standard's API surface. Cross-walked entries between
WIA-financial-inclusion and WIA-anti-money-laundering
flow through the cross-standard linkage in §16. The
FATF risk-based-approach simplified-CDD relaxations
are recorded against both standards' records to
preserve a single regulator-facing view of the
operator's AML/CFT discipline.

## §5 Alternative Dispute Resolution Body Integration

For programmes that consumers can escalate to,
integration carries the ADR body's identifier:

- UK: the Financial Ombudsman Service (FOS) for
  retail-financial-services disputes;
- AU: the Australian Financial Complaints Authority
  (AFCA);
- EU: the FIN-NET network of national ADR bodies;
- US: the CFPB Consumer Response system for
  federally-supervised entities, plus state-level
  ADR for state-licensed entities;
- KR: the Financial Dispute Settlement Committee
  (FDSC) under the Financial Supervisory Service.

Integration carries the per-case escalation intake
endpoint, the per-case cooperation workflow, the
per-case decision intake (binding decisions are
honoured per the ADR body's certified rules), and
the per-cycle aggregate-statistics submission.

## §6 Credit-Bureau Network Integration

For programmes reporting credit data to the operating
jurisdiction's credit-bureau network:

- per-bureau identifier (e.g. US TransUnion / Experian
  / Equifax bureau identifiers, EU Member State
  national credit bureau identifiers, KR's KCB and
  NICE);
- per-bureau data-quality discipline (the operating
  jurisdiction's credit-reporting law's accuracy
  obligation; e.g. US Fair Credit Reporting Act
  obligations on furnishers, EU Member State data-
  protection alignment);
- per-bureau dispute-resolution flow when a customer
  disputes data the operator furnished.

## §7 Civil-Society Partner Organisation Integration

Civil-society partner organisations (NGO partners,
academic-research consortia, financial-literacy
networks) integrate through programme-delivery
agreements. Integration carries each partner's
identifier, the per-cohort responsibility-matrix
reference, the per-cohort cooperation record, and
the per-cycle review cadence.

## §8 World Bank Findex Collection Integration

For operating jurisdictions where the World Bank
Findex Database collection cycle (every three years)
gathers comparable cross-country data:

- per-cycle Findex methodology adoption;
- per-cycle programme-level outcome submission to
  the operating jurisdiction's Findex coordinator;
- per-cycle programme-level publication of Findex-
  comparable indicators in the programme's outcome
  reports.

## §9 AFI Data-Collection Cycle Integration

For operating jurisdictions that are AFI members,
integration carries the AFI Member Data Tracking
Service identifier, the per-cycle Maya-Declaration
commitment progress submission, and the per-AFI-
working-group participation record.

## §10 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  basic-account-access/        — basic-account-access
                                  records (gated to
                                  AML/CFT supervisor
                                  and authorised
                                  consumers)
  consumer-disclosures/        — consumer-protection
                                  disclosure records
  fee-schedules/               — fee schedules (current
                                  and superseded)
  dispute-resolutions/         — dispute-resolution
                                  records
  credit-decisions/            — credit-decision
                                  records (gated to
                                  fair-lending auditor
                                  and authorised
                                  consumers)
  programme-outcomes/          — aggregate outcome
                                  reports
  education-activities/        — financial-literacy
                                  education records
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the programme operator's HTTP-message-
signature key (RFC 9421) and counter-signed by the
programme's compliance officer when the package
supports a regulator submission.

## §11 Manifest and Signatures

Verification tools recompute file digests, compare
to the manifest, and reject the package on mismatch
with type
`urn:wia:financial-inclusion:evidence-mismatch`. The
basic-account-access bundle is gated to AML/CFT
supervisor and fair-lending-auditor consumers;
public-shareable bundles carry only aggregate
counts.

## §12 well-known URI Discovery

A conformant programme exposes a discovery document
at `/.well-known/wia-financial-inclusion` that links
to the API root, the programme's public privacy
notice, the operating jurisdiction's financial-
inclusion strategy coordinator binding, the operating
jurisdiction's consumer-protection regulator
binding, the operating jurisdiction's ADR body
binding, the in-force fee schedule, and the in-force
fee-glossary publication.

## §13 Long-Term Archive Integration

Programmes designate a long-term archive that holds
the basic-account-access records (anonymised in
public-archive bundles), the fee-schedule history,
the dispute-resolution outcomes (aggregate), and the
programme-outcome reports beyond the programme's
primary retention horizon. Quarterly deposits round-
trip content-addresses; on programme wind-down,
remaining records transfer to the archive with
content-addresses preserved subject to the operating
jurisdiction's records-retention rules.

## §14 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations
(operating jurisdiction's National Financial
Inclusion Strategy alignment, ISO/IEC 27001
certification, AFI Maya Declaration commitment
progress, World Bank UFA-2020 framework alignment)
to consumers of W3C Verifiable Credentials MAY re-
issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance
is optional; the canonical record remains the JSON
evidence-package manifest.

## §15 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers
that disconnect during fee-schedule revision windows
or dispute-resolution windows resume from the last
seen event identifier without losing visibility of
priority-1 events (fee-schedule revision, dispute
escalated to ADR body, fair-lending pattern flagged,
strategy-coordinator inquiry opened).

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through
a deprecation window of at least one full reporting
cycle so that strategy-coordinator, regulator, AML/
CFT supervisor, ADR body, credit-bureau, civil-
society-partner, and external-evaluator integrations
have time to migrate.

## §17 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (WIA-
anti-money-laundering for the AML/CFT discipline
that intersects with simplified CDD, WIA-credit-
scoring for the credit-decision processing overlay,
WIA-cross-border-payment for cross-border remittance
inclusion overlay, WIA-digital-citizenship for the
financial-literacy education overlay, WIA-gdpr-
compliance for the privacy overlay across customer
records) emit cross-standard linkage records.

## §18 Reader Tooling

Programmes MAY publish supplementary reader tools
(per-cohort onboarding-funnel dashboards, per-period
Findex-aligned indicator dashboards, per-fee-schedule
revision-history viewers, per-dispute-resolution-
outcome trend charts) alongside the canonical
evidence package; the tools are non-normative.

## §19 Public Catalogue Feed

Programmes publish a public catalogue feed listing
the in-force fee schedule, the in-force public
privacy notice, the operating jurisdiction's
financial-inclusion strategy coordinator binding,
the operating jurisdiction's consumer-protection
regulator binding, and the aggregate new-account-
opening and active-account counts. The feed enables
peer-programme and civil-society discovery of the
programme's posture.

## §20 G20 GPFI / OECD INFE Integration

Where the operating jurisdiction participates in the
G20 Global Partnership for Financial Inclusion
(GPFI) work programme or the OECD International
Network on Financial Education (INFE), integration
carries the per-cycle indicator submission to the
operating jurisdiction's GPFI / INFE coordinator and
the per-cycle programme-experience submission.

## §21 Public Catalogue Aggregator Integration

Civil-society researchers, academic-research
consortia, and financial-development research
organisations consume aggregate financial-inclusion
statistics for independent analysis. Integration
carries the consumer's identifier, the per-research-
purpose data-access agreement, and the programme's
publication of consumer-attribution in any
derivative research output. Per-customer records are
NOT shared through this channel; only aggregate
statistics are.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the operating jurisdiction's
financial-inclusion strategy coordinator, the
operating jurisdiction's central bank or financial-
inclusion authority, the operating jurisdiction's
consumer-protection regulator, the operating
jurisdiction's AML/CFT supervisor (cross-walked with
WIA-anti-money-laundering), the operating
jurisdiction's ADR body (where applicable), at least
one credit-bureau network (for credit-extending
programmes), and at least one long-term archive,
and has published at least one externally citable
evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-financial-inclusion
- **Last Updated:** 2026-04-28
