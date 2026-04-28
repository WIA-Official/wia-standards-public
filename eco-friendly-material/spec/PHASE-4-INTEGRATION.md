# WIA-eco-friendly-material PHASE 4 — INTEGRATION Specification

**Standard:** WIA-eco-friendly-material
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an eco-friendly-material
operator integrates with the systems that
surround LCA, EPD publication, Type II self-
declared claims, Type I ecolabel licensing, and
GHG reporting: the accreditation body issuing the
ISO 14065:2020 verification-body certificate; the
EPD programme operator under ISO 14025 §7; the
Product Category Rule editor; the Type I scheme
operator; the public-procurement authority
running an environmentally-preferable-purchasing
programme; the construction-project building-
level assessment tool consuming the EN 15804+A2
EPD into an EN 15978 calculation; the regulator
auditing the supply-chain-due-diligence dossier;
the customs authority enforcing the EU CBAM
reporting under Regulation (EU) 2023/956; and —
where the operator publishes a Type II self-
declared claim — the consumer-protection
authority auditing the marketing-claim against
ISO 14021:2016.

References (CITATION-POLICY ALLOW only):

- ISO 14025:2006, ISO 14021:2016, ISO 14024:2018
- ISO 14040:2006/Amd 1:2020, ISO 14044:2006/Amd 1:
  2017/Amd 2:2020
- ISO 14067:2018
- ISO 14064-1:2018, ISO 14064-2:2019, ISO 14064-3:
  2019, ISO 14065:2020
- EN 15804:2012+A2:2019, EN 15978:2011, ISO 21930:
  2017
- ISO 9001:2015 (quality management systems)
- ISO/IEC 17000:2020, ISO/IEC 17021-1:2015, ISO/IEC
  17065:2012
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421, RFC
  6962
- W3C Verifiable Credentials Data Model 2.0
  (optional, used for the re-issuance of an
  accreditation reference)
- EU Construction Products Regulation (EU)
  305/2011
- EU Carbon Border Adjustment Mechanism
  Regulation (EU) 2023/956
- EU Green Claims Directive proposal (cited where
  the operator's Type II self-declared claim is
  in scope of the directive's verifying
  obligations)
- KS I ISO 14025 (Korean adoption of ISO 14025)
  and KS I ISO 14021 (Korean adoption of ISO
  14021)
- KR Eco-Label scheme operated under the KR
  Environmental Technology and Industry Support
  Act (환경기술 및 환경산업 지원법)

---

## §1 Accreditation-Body Integration

### §1.1 ISO 14065 register

The operator's API binds every EPD-verification
and GHG-verification record to the ISO 14065:2020
accreditation reference declared in the
verifier's profile. The accreditation body's
published register is queried at verification-
publication time; the response is cached for the
TTL declared in the accreditation body's HTTP
caching headers. A withdrawn or suspended
reference returns `410 Gone` for the operator's
verification-publication endpoint and `200 OK`
with a `withdrawn: true` flag for the public
retrieval endpoint.

### §1.2 ISO/IEC 17065 register for Type I schemes

Every Type I licence carried by the operator is
verified against the issuing scheme operator's
ISO/IEC 17065:2012 accreditation. The operator's
API queries the scheme's published register on
each retrieval after the caching TTL.

## §2 EPD Programme Operator Integration

The operator binds every EPD record to a
programme operator's published instructions
under ISO 14025 §7. The programme operator's
endpoint is published at the operator's
discovery document and is queried at EPD
publication and at retrieval. The PCR editor's
endpoint, where separate from the programme
operator, is queried for the PCR's review-status
flag declared in PHASE-3 §2.2.

## §3 Construction-Project Integration

### §3.1 EN 15978 building-level summary ingestion

An EPD whose `materialFamily` is a construction-
product family carries an EN 15978-compatible
machine-readable summary in the response envelope.
The summary is consumed by a building-level
assessment tool to compute the per-building
environmental performance per EN 15978 §6. The
summary is signed using the operator's public-key
set so that the tool can validate the binding
without re-fetching the EPD.

### §3.2 EU Construction Products Regulation linking

A construction product placed on the EU market
under EU Regulation (EU) 305/2011 carries a CE
marking with a Declaration of Performance. The
operator's API carries the link from the
material record's `identifierBindings` to the
manufacturer's published Declaration of
Performance so that a downstream consumer can
verify the CE-marking compliance of the EPD-
covered product.

## §4 Public-Procurement Integration

A public-procurement authority operating an
environmentally-preferable-purchasing programme
queries the operator's API for the materials
satisfying the programme's environmental
criteria. The query envelope carries the
programme's PCR reference, the required
indicator thresholds, and the validity-period
filter; the response is an RFC 8288 `Link`-
paginated collection of material records that
satisfy the criteria.

## §5 Carbon Border Adjustment Mechanism Integration

### §5.1 CBAM customs-declaration linking

An organisation in scope of EU CBAM Regulation
(EU) 2023/956 publishes a per-product embedded-
emissions declaration that links the carbon
footprint computed under ISO 14067:2018 to the
CBAM declaration filed with the customs
authority. The operator's API carries the per-
product CBAM report identifier and the linked
carbon footprint record so that the customs
authority can verify the embedded-emissions
value against the carbon-footprint computation.

### §5.2 Verified embedded-emissions discipline

The CBAM declaration's verifier reference is bound
to the operator's verifier-of-record under ISO
14064-3:2019. A change of verifier is recorded as
a new chain-of-custody event under PHASE-3 §6 so
that the customs authority's audit trail is
preserved across the verifier change.

## §6 Consumer-Protection Authority Integration

A Type II self-declared claim is reviewable by
the relevant consumer-protection authority under
the operator's jurisdiction. The operator's API
publishes the claim's verbatim text, the
evaluation method, and the evidence artefacts so
that the authority can validate the claim
against ISO 14021 §7 and the jurisdiction's
green-claims regulation. Where the operator is
in scope of the EU Green Claims Directive (when
it enters force), the operator publishes the
ex-ante substantiation file referenced by the
directive.

## §7 KR-Jurisdiction Integration

### §7.1 KS I ISO 14025 / KS I ISO 14021 adoption

A KR-jurisdiction operator declares the KS I ISO
14025 and KS I ISO 14021 adoption references in
the programme record's `governingFrameworks`.
The KR Environmental Industry and Technology
Institute (KEITI) operates the KR Eco-Label scheme
register; the operator's API queries the KEITI
register on Type I licence publication and on each
retrieval after the caching TTL.

### §7.2 KR Carbon-Disclosure linkage

An organisation participating in the KR Greenhouse
Gas Inventory and Reporting System carries the
inventory's per-period reference in the GHG
record's `reportingPeriod`. The operator's API
does not gate publication on the inventory's
review status — the publication is informational
and is reviewed by the operator's regulatory-
affairs team under the operator's quality-
management discipline declared in PHASE-3 §9.

## §8 Public Retrieval and Re-Issuance

### §8.1 Public EPD retrieval

A public consumer (a buyer evaluating a bid, a
designer integrating the EPD into a building-
level assessment, a researcher referencing the
LCA in a publication) retrieves the EPD record
at `/v1/epd-records/{epdId}` without
authentication; the response carries the public
fields and the EN 15978 summary.

### §8.2 Verifiable-credentials re-issuance

A verification reference can be re-issued into
a downstream system as a W3C Verifiable
Credential. The credential carries the
accreditation body's issuer identifier, the
accreditation reference, the scope of the
accreditation, the certificate's expiry, and the
issuing date; the credential is signed using the
accreditation body's public-key set so that a
downstream verifier can validate the credential
without contacting the accreditation body
directly.

## §9 Audit and Conformity-Assessment Integration

### §9.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system declared
in PHASE-3 §9 is audited under ISO/IEC 17021-1
by an accredited certification body. The audit
result is stored in the operator's audit envelope
and is referenced from the programme record's
`accreditationStatus`. The certification body's
public-key set is published at the certification
body's endpoint; the operator's API verifies the
audit certificate's signature against the
certification body's public key on retrieval.

### §9.2 ISO/IEC 17065 product certification

Where a Type I ecolabel is itself a product-
certification deliverable, the certification body
issuing the label operates under ISO/IEC 17065:
2012. The certification body's marking, the scope
of certification, and the certified product's
identifier are published in the licence envelope
so that a downstream consumer can verify the
certification scope before relying on the label.

## §10 References (consolidated)

The references list across PHASE-1 to PHASE-4 is
the canonical citation set for the WIA-eco-
friendly-material standard. Implementations cite
the standards by their issuing organisation (ISO,
IEC, IETF, W3C, EU regulatory text, KS) and the
publication year so that a downstream consumer
can locate the authoritative text. Updates to a
cited standard (for example, a new amendment to
EN 15804) trigger an internal review cycle in the
operator's quality-management discipline declared
in PHASE-3 §9 before the new revision is bound
into the operator's enumeration set.

## §11 LCI Database Aggregation Integration

The operator's LCI dataset references published
in PHASE-1 §4 may bind to one or more LCI
database publishers operating under an industry-
recognised peer-review scheme. The operator
publishes the binding from the LCA record's
`inventoryDataset` to the LCI database's
publication endpoint so that a downstream
consumer can verify the dataset's provenance
without re-running the LCA. Where the operator
runs its own LCI dataset (a sectoral cooperative,
an industry association running a shared
inventory), the operator's publication endpoint
mirrors the binding contract.

## §12 Cross-Programme EPD Federation

### §12.1 Federation discovery

An operator participating in a federation of
programme operators publishes the federation
membership list in the discovery document. The
list carries the federation's name, the
federation's published instructions (the meta-
PCR if any), and the per-peer mutual-recognition
agreement reference. The federation's discovery
document is signed by the federation's secretariat
so that a downstream consumer can validate the
federation membership independently.

### §12.2 Federated query envelope

A federated query is forwarded to every peer in
the federation; each peer responds with the
subset of its EPD register satisfying the query.
The aggregator deduplicates the result by the
EPD's content hash and presents the union to the
querying consumer. The federation discipline
preserves each peer's accreditation reference
so that the consumer can trace each EPD back to
the originating programme operator.

## §13 Verifier-Pool Marketplace Integration

A verifier listed under ISO 14065:2020 publishes
its verification offer (scope, accreditation
reference, fee schedule, geographic coverage) to
a verifier-pool marketplace. An operator seeking
verification queries the marketplace by the
required scope (organisation-level GHG, project-
level GHG, EPD verification, Type I licence
verification) and receives a list of eligible
verifiers. The marketplace does not decide
selection; it carries the offer set and the
operator selects per its quality-management
procurement policy declared under ISO 9001 §8.4.

## §14 KR Green-Procurement Integration

The KR Public Procurement Service operates a
Green Public Procurement scheme that gives
preference to materials carrying a KR Eco-Label
or an equivalent foreign Type I licence. The
operator's API carries the binding from the
KR-jurisdiction material record to the KR Eco-
Label register so that a public buyer querying
the procurement system can verify the eco-label
status without leaving the procurement workflow.
