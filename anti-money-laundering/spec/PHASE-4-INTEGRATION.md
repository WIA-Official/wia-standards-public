# WIA-anti-money-laundering PHASE 4 — INTEGRATION Specification

**Standard:** WIA-anti-money-laundering
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an obliged entity integrates
with the systems that surround AML/CFT compliance: the
operating jurisdiction's financial-intelligence unit
(FinCEN, KoFIU, the operating Member State's FIU,
NCA-UKFIU); the operating jurisdiction's AML/CFT
supervisor (FinCEN + prudential regulator for US, AMLA
for EU once operational, FSC + FSS for KR, FCA / HMRC
/ Gambling Commission for UK per the obliged entity's
class); the SWIFT network and ISO 20022 payment
clearing infrastructure; sanctions-list providers (UN,
EU, OFAC, UK OFSI, KR MOFA, the obliged entity's
contracted screening service); the operating
jurisdiction's beneficial-ownership register; the
Egmont Group (international FIU cooperation); the
Wolfsberg Group (cross-border banking community); the
obliged entity's external auditors; and long-term
archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO 8601 (date and time)
- ISO 20022 (financial-services messaging)
- ISO 17442 (LEI)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Financial-Intelligence Unit Integration

The operating jurisdiction's FIU is the destination
for STR / SAR filings:

- US: FinCEN BSA E-Filing System for SARs and CTRs;
- KR: KoFIU electronic STR / CTR reporting channel;
- EU: each Member State's FIU portal (FIU.NL for
  Netherlands, TRACFIN for France, etc.);
- UK: National Crime Agency UK FIU portal.

Integration carries the FIU's identifier, the per-
filing acknowledgement workflow, the per-FIU response
SLA, and the per-FIU feedback consumption channel.
FIUs share information with peer FIUs through the
Egmont Group's secure web channel; the obliged entity
does not interact with Egmont directly but its filings
flow through the Egmont channel by way of the
operating jurisdiction's FIU.

## §2 AML/CFT Supervisor Integration

Per-jurisdiction supervisor integration:

- US: FinCEN as the BSA administrator plus the obliged
  entity's primary prudential regulator (Federal
  Reserve, OCC, FDIC, NCUA, SEC, CFTC, state
  regulator) per the obliged entity's federal /
  state charter;
- EU: AMLA (Authority for Anti-Money Laundering and
  Countering the Financing of Terrorism) for direct
  supervision of cross-border high-risk obliged
  entities once AMLA is operational; the operating
  Member State's national AML/CFT supervisor for all
  other obliged entities;
- KR: FSC (Financial Services Commission) and FSS
  (Financial Supervisory Service) per the obliged
  entity's class;
- UK: FCA (Financial Conduct Authority) for FCA-
  supervised firms; HMRC for trust-and-company-
  service providers, accountancy service providers,
  art-market participants, and money-service
  businesses; the Gambling Commission for casinos.

Integration carries the supervisor's identifier, the
per-examination cooperation workflow, the per-
enforcement-action response workflow, and the per-
periodic-reporting cadence (where the supervisor
requires periodic AML reporting beyond inquiry-driven
reporting).

## §3 SWIFT and ISO 20022 Payment Network Integration

Cross-border payment messaging integrates with the
SWIFT network for MT and ISO 20022 messages:

- MT 103 single customer credit transfer (originator
  and beneficiary fields 50 and 59);
- MT 202 / MT 202 COV financial-institution transfer
  / cover payment (with the underlying customer
  credit transfer's fields 50 and 59 in the COV
  envelope);
- ISO 20022 pacs.008 / pacs.009 / camt.* under the
  SWIFT MT-to-ISO 20022 migration timetable; the
  obliged entity's parallel-running discipline is
  recorded in the per-corridor migration tracker.

Integration carries the SWIFT BIC of the obliged
entity, the per-corridor migration status, and the
per-message Travel-Rule field validation gate.

## §4 Sanctions-List Provider Integration

Sanctions-list integration consumes:

- UN Security Council Consolidated Sanctions List (the
  baseline list referenced by Recommendations 6
  and 7);
- EU consolidated sanctions list;
- US OFAC SDN and consolidated sanctions list (cited
  authoritatively for US sanctions);
- UK OFSI consolidated list (cited for UK sanctions);
- KR MOFA sanctions list (cited for KR sanctions);
- the obliged entity's internal watchlist of customers
  exited under prior STR / SAR filings;
- the obliged entity's contracted PEP screening
  service.

Integration carries each list's identifier, the per-
list refresh cadence (UN and OFAC publish updates
without notice; EU publishes via the OJEU; the
obliged entity's discipline is "without delay"
ingest), and the per-list false-positive-rate
monitoring.

## §5 Beneficial-Ownership Register Integration

Per-jurisdiction beneficial-ownership register
integration:

- US: FinCEN Beneficial Ownership Information (BOI)
  reporting under the Corporate Transparency Act
  beneficial-ownership reporting regime;
- EU: each Member State's central beneficial-
  ownership register under EU AML Regulation
  2024/1624 (some Member States' registers are public,
  some are restricted to obliged entities and
  competent authorities, per the post-2022-CJEU-
  judgment evolution);
- KR: 실질소유자 신고 under KR Specific Financial
  Information Act and the Commercial Act register;
- UK: Companies House Register of People with
  Significant Control (PSC).

Integration carries the register's identifier, the
per-register lookup endpoint, and the per-register
discrepancy-reporting workflow (the obliged entity
reports discrepancies between register content and
its own CDD finding back to the register).

## §6 Egmont Group International Cooperation

The Egmont Group is the international association of
FIUs. The obliged entity does not interact with
Egmont directly, but its STR / SAR filings flow into
the Egmont information-sharing channel through the
operating jurisdiction's FIU when the FIU determines
international cooperation is warranted. Integration
carries the FIU's identifier as the proxy and the
per-filing flow indicator.

## §7 Wolfsberg Group Community Integration

The Wolfsberg Group publishes principles, frequently-
asked questions, and the Correspondent Banking Due
Diligence Questionnaire (CBDDQ) that obliged entities
use as community-recognised baselines. Integration
carries the obliged entity's adoption statement of
the Wolfsberg Principles (where adopted), the per-
respondent CBDDQ archive, and the per-cycle CBDDQ
refresh.

## §8 External Auditor Integration

External auditors (the obliged entity's independent
auditor under Recommendation 18, the supervisor-
mandated external auditor under enforcement action,
the FATF mutual-evaluation review) consume audit-
trail exports through dedicated client certificates.
The export carries the API audit logs for the audit
window, the EWRA, the CDD population sample, the
transaction-monitoring rule library, the STR / SAR
inventory, the sanctions-screening sample, and the
correspondent-banking review sample.

## §9 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  ewra/                        — enterprise-wide risk
                                  assessment
  cdd-records/                 — CDD records (gated by
                                  the operating
                                  jurisdiction's
                                  privacy regime)
  sanctions-screenings/        — sanctions screening
                                  evidence
  transactions/                — transaction-monitoring
                                  evidence (sample;
                                  full set under
                                  supervisor request)
  suspicious-reports/          — STR / SAR filings
                                  (gated to compliance
                                  officer / MLRO and
                                  external auditors)
  ctr-records/                 — CTR filings
  correspondent-banking/       — correspondent-banking
                                  relationship
                                  evidence
  investigation-cases/         — case histories
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the obliged entity's HTTP-message-signature
key (RFC 9421) and counter-signed by the MLRO when
the package supports a regulator submission.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to
the manifest, and reject the package on mismatch with
type `urn:wia:anti-money-laundering:evidence-mismatch`.
STR / SAR records are gated; bundles for supervisor
submission carry the records, bundles for civil-
society research carry only aggregate counts.

## §11 well-known URI Discovery

A conformant obliged entity exposes a discovery
document at `/.well-known/wia-anti-money-laundering`
that links to the API root, the MLRO contact, the
operating-jurisdiction supervisor binding, and the
operating-jurisdiction FIU binding. End-customer
disclosure (e.g. Recommendation 16 originator
information disclosure to the beneficiary) flows
through the obliged entity's product-surface, not
through the discovery document.

## §12 Long-Term Archive Integration

Obliged entities designate a long-term archive that
holds CDD records, transaction-monitoring evidence,
STR / SAR filings, sanctions screenings, and
supervisor correspondence beyond the obliged entity's
primary retention horizon. Quarterly deposits round-
trip content-addresses; on programme wind-down,
remaining records transfer to the archive with
content-addresses preserved subject to the operating
jurisdiction's records-retention rules.

## §13 Verifiable-Credential Re-Issuance (optional)

Obliged entities that wish to expose attestations
(Wolfsberg Principles adoption, ISO/IEC 27001
certification, supervisor on-site examination
clearance) to consumers of W3C Verifiable
Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0
specification. Re-issuance is optional; the
canonical record remains the JSON evidence-package
manifest.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers that
disconnect during sanctions-list update windows or
transaction-monitoring alert windows resume from the
last seen event identifier without losing visibility
of priority-1 events (sanctions match confirmed,
freezing applied, supervisor inquiry opened, FIU
filing acknowledged).

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through
a deprecation window of at least one full supervisor
examination cycle so that supervisor, FIU, prudential-
regulator, and external-auditor integrations have
time to migrate.

## §16 Cross-Standard Linkage

Obliged entities that consume adjacent WIA standards
(WIA-credit-scoring for credit-decision processing
that intersects with AML risk rating, WIA-cross-border-
payment for cross-border payment messaging, WIA-
financial-data-exchange for shared customer data
governance, WIA-gdpr-compliance for the privacy
overlay across CDD records) emit cross-standard
linkage records.

## §17 Reader Tooling

Obliged entities MAY publish supplementary reader
tools (per-corridor sanctions exposure dashboards,
per-rule transaction-monitoring tuning consoles, per-
PEP-list refresh trackers, per-respondent
correspondent-banking review consoles) alongside the
canonical evidence package; the tools are non-
normative.

## §18 Public Catalogue Feed

Obliged entities publish a public catalogue feed
listing the in-force MLRO contact, the operating-
jurisdiction supervisor binding, the operating-
jurisdiction FIU binding, and the obliged entity's
adopted community baselines (Wolfsberg Principles,
sectoral codes of conduct). The feed enables peer
obliged entities and supervisor discovery of the
obliged entity's AML/CFT posture.

## §19 FATF Mutual-Evaluation Integration

FATF and FATF-style regional bodies (MONEYVAL for
Council of Europe, APG for Asia-Pacific, GAFILAT for
Latin America, MENAFATF for Middle East and North
Africa, etc.) conduct mutual evaluations of the
operating jurisdiction's AML/CFT regime. Obliged
entities support the mutual evaluation by providing
case studies and aggregate statistics through the
operating jurisdiction's coordinator. Integration
carries the per-evaluation cycle's coordinator
identifier and the per-evaluation case-study and
aggregate-statistics submission.

## §20 Public Catalogue Aggregator Integration

Civil-society researchers and academic-research
consortia (anti-corruption observatories, academic
financial-crime research consortia) consume aggregate
AML/CFT statistics for independent analysis.
Integration carries the consumer's identifier, the
per-research-purpose data-access agreement, and the
obliged entity's publication of consumer-attribution
in any derivative research output. Per-customer or
per-transaction records are NOT shared through this
channel; only aggregate statistics are.

## §21 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the operating jurisdiction's FIU,
the operating jurisdiction's AML/CFT supervisor, at
least one sanctions-list provider for each major
list (UN, the operating jurisdiction's national list,
and the obliged entity's coalition lists where
applicable), the operating jurisdiction's beneficial-
ownership register, at least one external auditor,
and at least one long-term archive, and has published
at least one externally citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-anti-money-laundering
- **Last Updated:** 2026-04-28
