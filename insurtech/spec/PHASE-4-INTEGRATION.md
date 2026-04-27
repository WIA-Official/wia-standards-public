# WIA-insurtech PHASE 4 — INTEGRATION Specification

**Standard:** WIA-insurtech
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited insurtech operator
integrates with the systems that surround it: the prudential
regulator and the market-conduct regulator; tax authorities
collecting insurance-premium tax and policyholder protection
levies; KYC and sanctions-list providers; reinsurer cession
ledgers; broker and MGA distribution platforms; embedded-
insurance partners; payment processors; healthcare networks
(for health and disability lines); telematics providers (for
auto and freight); fraud bureaux; complaint and ombudsman
bodies; and long-term archives that hold records beyond the
operator's wind-down horizon. It also defines the evidence-
package format that bundles a programme's record set for
external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- ACORD Reference Architecture (insurance industry data model)
- IFRS 17 (Insurance Contracts)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Prudential Regulator Integration

The prudential regulator (PRA in the UK, BaFin in Germany, FSC
in Korea, NAIC-coordinated state regulators in the US, FSC in
Japan, CIRC in China, and equivalent authorities elsewhere)
consumes solvency QRTs, capital reports, and ad-hoc supervisory
data requests. Integration carries the regulator's identifier,
the per-template submission cadence, the digital-signature
requirements the regulator applies, and the supervisory
contact-of-record.

## §2 Market-Conduct Regulator Integration

Market-conduct regulators (FCA in the UK, state insurance
departments in the US, FSA in Japan, KFSC in Korea, CONSAR in
Mexico, equivalent bodies elsewhere) consume product filings,
complaint statistics, and TCF-aligned outcome reports. The
integration record carries the per-report cadence and the
prescribed templates that the operator emits.

## §3 Tax Authority Integration

Insurance-premium tax (IPT), state-level surcharges, and
policyholder protection levies are remitted to tax authorities
on a periodic basis. The integration carries the authority's
identifier, the remittance cadence, and the per-line-of-business
rate schedule that the operator applies at premium-receipt time
(PHASE-1 §11).

## §4 KYC and Sanctions Provider Integration

KYC vendors (Onfido, Trulioo, Jumio, Refinitiv, equivalent
providers) emit signed identity-verification outcomes that the
operator binds to the party record's KYC state (PHASE-1 §3 /
PHASE-2 §3). Sanctions-list providers emit periodic list
refreshes; the operator's sanctions screening sweep runs at the
cadence the operator declares and emits screening events that
update the party record's `sanctionsScreenAt`.

## §5 Reinsurer Ledger Integration

Reinsurer ledgers consume bordereaux at the agreed reporting
cadence (typically monthly for proportional treaties, quarterly
for non-proportional treaties, per-event for catastrophe
treaties). The integration carries the reinsurer's identifier,
the treaty-side ledger account, and the bordereau dispute-
resolution path when the reinsurer's reconciliation flags
discrepancies.

## §6 Distribution Partner Integration (Broker, MGA,
   Embedded-Insurance, Affinity)

Distribution partners produce business through the operator's
quote-and-bind APIs. The integration carries the partner's
identifier, the per-product binding authority delegated to the
partner, the commission schedule, and the per-partner consumer-
disclosure obligations.

Embedded-insurance partners (e.g. retailers selling extended-
warranty cover at point of sale, mobility platforms selling
per-trip cover) follow the same integration pattern; the
embedded context is recorded against each policy so that
consumer disclosures meet the embedded regulator's enhanced
expectations where they apply.

## §7 Payment Processor Integration

Premium and claim-payment processors (card processors, bank-
direct-debit operators, payment-rails operators) integrate
through the operator's treasury system. Integration carries the
processor's identifier, the per-channel settlement cadence, and
the chargeback / dispute workflow that the processor honours.

## §8 Healthcare Network Integration

Health and disability lines integrate with healthcare networks
(provider directories, claim-adjudication services, EHR
gateways). The integration follows the local healthcare-data-
exchange standard (HL7 FHIR R4 / R5 in most jurisdictions) and
honours the local healthcare-data-protection regime (HIPAA in
the US, the Personal Health Information Act equivalents
elsewhere).

## §9 Telematics Provider Integration

Telematics providers (auto-telematics for usage-based insurance,
freight telematics for cargo cover, IoT-sensor telematics for
property cover) emit observation feeds that the operator's
underwriting and claims systems consume. The integration record
carries the provider's identifier, the per-policy consent the
consumer has granted, and the operator's data-minimisation
filter that strips raw observations to the underwriting-relevant
features.

## §10 Fraud Bureau Integration

Fraud bureaux (IFB in the UK, NICB and CLUE in the US,
equivalent bodies elsewhere) consume confirmed-fraud
notifications and emit fraud-intelligence feeds that the
operator's SIU consumes. The integration record carries the
bureau's identifier, the per-record-class submission template,
and the operator's appeal workflow when a consumer disputes a
bureau-recorded entry.

## §11 Ombudsman and Dispute Resolution Body Integration

External dispute resolution bodies (Financial Ombudsman Service
in the UK, equivalent bodies in other jurisdictions) consume
escalated complaints and emit determinations. The integration
carries the body's identifier, the per-complaint case reference,
and the operator's compliance workflow when a determination
requires remediation.

## §12 Evidence Package Format

```
evidence/
  manifest.json                 — package manifest (signed)
  programme.json                — programme record
  policies/                     — per-policy record sets,
                                   including endorsement chains
  underwriting-decisions/       — decision records and
                                   rationales
  claims/                       — per-claim record sets,
                                   including reserves and
                                   payment history
  cessions/                     — per-treaty cession records
                                   and bordereaux
  consumer-disclosures/         — per-policy disclosure
                                   artefacts
  premium-receipts/             — per-policy receipts and
                                   IFRS 17 contract-group
                                   allocations
  sanctions-screens/            — sanctions-screening sweep
                                   summaries (no list contents
                                   reproduced)
  complaints/                   — complaint records and
                                   resolutions
  audit/                        — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the operator's chief actuary when the
package supports a regulator submission.

## §13 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:insurtech:evidence-mismatch`.

## §14 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-insurtech` that links to the API root, the
public licensing summary, the published quality dossier, the
product register, the consumer claim-service standards, and
the complaint statistics.

## §15 Long-Term Archive Integration

Operators designate a long-term archive that holds records
beyond run-off completion. Quarterly deposits round-trip
content-addresses; on wind-down completion, remaining records
transfer to the archive with content-addresses preserved.

## §16 Cross-Standard Linkage

Operators that consume adjacent WIA standards (clinical-genomics
for life-underwriting genomic disclosures, intelligent-
transportation for telematics-driven auto cover, climate-risk
for catastrophe-bond exposure mapping) emit cross-standard
linkage records that name the consuming standard and the
version under which the linkage is claimed.

## §17 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (regulator licence
status, ISO 9001 conformance, ISO 22301 business-continuity
attestation, ISO/IEC 27001 certification) to consumers of W3C
Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0 specification.
Re-issuance is optional; the canonical record remains the JSON
evidence-package manifest.

## §18 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long claim-investigation windows resume from the last
seen event identifier without losing visibility of priority-1
events.

## §19 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full annual reporting cycle so that reinsurer,
regulator, and broker integrations have time to migrate without
disrupting in-flight policies.

## §20 Reader Tooling

Operators MAY publish supplementary reader tools (claim-
timeline views, reserve-development triangles for actuarial
consumers, complaint-theme dashboards for TCF consumers)
alongside the canonical evidence package; the tools are
non-normative.

## §21 Run-Off Operator Successor Integration

When a portfolio transfer moves policies to a successor
operator under the regulator's portfolio-transfer process
(PHASE-3 §14), the integration carries the successor's
identifier, the per-policy transfer audit chain, and the
recordkeeping continuity arrangement that preserves
content-addresses across the operator boundary.

## §22 Open-Banking and Aggregator Integration

Operators that consume open-banking aggregator feeds for
income-verification and direct-debit-mandate integrations
follow the operating jurisdiction's open-banking specification
(PSD2 / UK CMA Open Banking / equivalent). The integration
record carries the aggregator's identifier, the per-policyholder
consent reference, and the operator's data-minimisation filter
that retains only the underwriting-relevant signals.

## §23 Identity-Provider Integration for Consumer Self-Service

Consumer-facing portals authenticate consumers through an
identity provider (the operator's own IdP, or a federated IdP
under the operator's broker relationship). The integration
carries the IdP's identifier, the supported authentication
strengths (per NIST SP 800-63B AAL tiers), and the per-action
authorisation matrix that maps consumer self-service actions
to the required AAL.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with the prudential regulator, the market-conduct regulator,
the relevant tax authority, at least one KYC and one sanctions
provider, at least one reinsurer ledger, at least one
distribution partner, at least one payment processor, the
relevant fraud bureau, the relevant ombudsman body, and at
least one long-term archive, and has published at least one
externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-insurtech
- **Last Updated:** 2026-04-28
