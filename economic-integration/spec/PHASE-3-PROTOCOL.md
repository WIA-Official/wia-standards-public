# WIA-economic-integration PHASE 3 — PROTOCOL Specification

**Standard:** WIA-economic-integration
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
an economic-integration operator across the
trader-to-customs-to-bank-to-statistics value
chain: the WTO TFA Article 1-12 trade-
facilitation discipline that gates the customs
declaration's processing, the WCO SAFE
Framework AEO discipline that drives mutual
recognition, the WCO Data Model alignment
discipline that anchors every declaration to
the v3 reference, the UN/EDIFACT message-
syntax discipline under ISO 9735-1, the ISO
20022 message-validation discipline that gates
the cross-border payment, the ICC UCP 600
discrepancy-management discipline, the
preferential-origin verification discipline
under WTO Rules of Origin, the export-control
discipline that gates controlled-goods
shipments, the chain-of-custody anchoring
discipline that prevents silent mutation of
the customs file, the dispute-and-appeal
discipline that handles a contested customs
decision, and the post-clearance audit
discipline.

References (CITATION-POLICY ALLOW only):

- WTO GATT 1994, WTO TFA, WTO GPA, WTO TRIPS,
  WTO Customs Valuation Agreement
- WCO SAFE Framework 2018, WCO Data Model v3,
  WCO HS-2022, WCO AEO Programme, WCO MRA
  Framework
- UN/EDIFACT (ISO 9735-1 to -10), UN/CEFACT
  Recommendations 1, 16, 21, 33, 36
- UN COMTRADE submission specification
- ISO 20022 message family (and the SWIFT
  GPI discipline)
- ISO 4217, ISO 3166-1, ISO 9362 (BIC), ISO
  13616 (IBAN), ISO 17442 (LEI)
- ICC Incoterms 2020, ICC UCP 600, ICC ISBP
  745, ICC URDG 758, ICC URBPO 750
- ISO 9001:2015 (QMS) and ISO/IEC 27001:2022
  (ISMS)
- ISO/IEC 17021-1:2015
- IETF RFC 9110, RFC 9421, RFC 9457, RFC
  6234, RFC 8615, RFC 6962
- W3C Trace Context, W3C ODRL 2.2, W3C VC
  v2.0
- EU Union Customs Code Regulation (EU)
  952/2013 and its Implementing Regulation
  (EU) 2015/2447 and Delegated Regulation
  (EU) 2015/2446
- KR 관세법, KR 대외무역법, KR 외국환거래법, KR
  자유무역협정 이행을 위한 관세법의 특례에 관한
  법률

---

## §1 WTO TFA Trade-Facilitation Discipline

### §1.1 TFA Article 1 publication

The operator publishes its trade-facilitation
information per WTO TFA Article 1 — the per-
procedure description, the documentation
requirements, the per-procedure fee schedule,
the appeal procedure — through the operator's
public-portal endpoint.

### §1.2 TFA Article 7 advance ruling

The operator's API supports the advance-
ruling submission per WTO TFA Article 3. A
trader requesting an advance ruling on
classification, valuation, or origin receives
a binding ruling that the operator's customs
admin honours during the validity period
declared in the ruling.

### §1.3 TFA Article 7 release of goods

The operator's API supports the goods-release
discipline per WTO TFA Article 7 — the pre-
arrival processing, the electronic payment of
duties and taxes, the separation of release
from final determination of duties and taxes
(allowing the trader to obtain release upon
provision of a guarantee), and the post-
clearance audit.

## §2 WCO SAFE AEO Discipline

### §2.1 AEO certification

An AEO certificate carried by the operator's
API binds the trader to the AEO Programme
benefits — reduced data-set requirements,
reduced inspection probability, prioritised
processing, mutual-recognition with partner
authorities. The AEO certificate's expiry
triggers a re-certification cycle.

### §2.2 AEO Mutual Recognition Arrangement

The operator's API queries the partner
authority's AEO register on each cross-
border declaration to verify the partner's
AEO status under the operator's MRA
participation set.

### §2.3 AEO Annual Audit

The operator runs an annual audit of the AEO
certificate-holder's continuing compliance
with the AEO Programme criteria — security,
solvency, compliance history, customs-
authority access. The audit outcome is
recorded in the operator's audit envelope.

## §3 WCO Data Model Alignment Discipline

Every declaration record carried by the
operator's API is aligned with the WCO Data
Model v3 reference. The operator's API
publishes the per-element mapping table from
the operator's national field set to the WCO
Data Model element identifier so that a
downstream peer customs admin can
deterministically interpret the declaration.

## §4 UN/EDIFACT Message-Syntax Discipline

### §4.1 Per-directory schema enforcement

Every EDIFACT message is validated against the
declared `edifactDirectory` directory release
and the per-message segment table. The
operator's API rejects a message whose UNH
segment carries a directory release that is
not in the operator's accepted set with `400
Bad Request` and an RFC 9457 problem document.

### §4.2 Acknowledgement protocol

The operator's API generates a CONTRL
acknowledgement message per UN/EDIFACT
Acknowledgement Service for each received
message. The CONTRL message reports the per-
segment validation outcome.

## §5 ISO 20022 Message Validation Discipline

### §5.1 Per-message schema enforcement

Every ISO 20022 message is validated against
the per-message ISO 20022 schema. A message
whose schema validation fails is rejected
with `422 Unprocessable Entity` at
`/problems/iso20022-schema-validation-failure`.

### §5.2 Sanctions screening

The operator's API runs a sanctions-screening
layer at the ISO 20022 message intake. The
screening uses the operator's documented
sanctions-list set (the UN Security Council
Consolidated List, the EU Consolidated List
under Council Regulation (EC) 881/2002, the
US OFAC SDN List, the operator's national
sanctions list). A flagged message is held
for the operator's compliance officer's
review.

### §5.3 SWIFT GPI UETR continuity

The UETR carried in the cross-border
payment is preserved across the correspondent-
banking chain so that the per-leg status is
trackable.

## §6 ICC UCP 600 Discrepancy-Management Discipline

### §6.1 Per-document examination

Every documentary-credit document presented
under PHASE-2 §6.2 is examined per UCP 600
Articles 14-16 by the issuing bank or the
nominated bank. The operator's API records
the per-document compliance state (compliant,
discrepant) and the per-discrepancy reason.

### §6.2 Five-banking-day discipline

The examination is completed within the
five-banking-day window per UCP 600 Article
14(b). The operator's API records the
examination start and end timestamps in the
audit envelope.

### §6.3 Notice of refusal

A notice of refusal under UCP 600 Article 16
is published as a structured envelope listing
each discrepancy. The presenting bank may
correct the discrepancy and re-present within
the credit's expiry.

## §7 Preferential-Origin Verification Discipline

### §7.1 Per-agreement rules of origin

A certificate of origin issued under PHASE-2
§7 is bound to the per-agreement rules of
origin (the EU GSP, USMCA, CPTPP, RCEP,
ASEAN). The issuing chamber verifies the
goods's qualification under the per-agreement
rule (wholly obtained, change in tariff
classification, regional value content,
specific process).

### §7.2 Verification request from importing
       customs

A destination-customs admin requesting
verification of the certificate's authenticity
queries the issuing chamber's verification
endpoint. The chamber's response carries the
per-certificate authenticity attestation.

## §8 Export-Control Discipline

### §8.1 Controlled-goods category binding

A declaration whose `goodsDescription` includes
a controlled-goods category (a dual-use item
under the EU Dual-Use Regulation (EU)
2021/821, a military-list item under the EU
Common Military List, a strategic item under
KR 전략물자 수출입고시) is gated on the
operator's export-control authority's
licensing decision.

### §8.2 Per-licence binding

The operator's API binds the declaration to
the per-shipment export licence reference and
verifies the licence's continuing validity at
clearance time.

## §9 Chain-of-Custody Anchoring Discipline

### §9.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency
log modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure.

### §9.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event
being amended.

## §10 Dispute-and-Appeal Discipline

### §10.1 Per-decision review right

A trader contesting a customs decision (a
classification ruling, a valuation
determination, an origin determination) lodges
an administrative review under the operator's
national customs law and the WTO TFA Article
4 review-and-appeal procedure.

### §10.2 Per-decision binding under review

The contested decision remains binding pending
the review outcome, except where the
operator's national law provides for a stay.

## §11 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the declaration-
processing, EDIFACT-message-handling, ISO
20022 payment-handling, documentary-credit-
handling, certificate-of-origin issuance, UN
COMTRADE submission, and chain-of-custody
processes. Internal audits run on a frequency
declared in the quality manual; the
nonconformity register is reviewed in the
ISO 9001 §9.3 management-review cycle. The
operator's information-security management
system declared under ISO/IEC 27001:2022
covers the per-message confidentiality and
the operator's signing keys.

## §12 Post-Clearance Audit Discipline

### §12.1 Per-audit selection

The operator runs a post-clearance audit
programme per WTO TFA Article 7.5 and the
WCO SAFE Framework. The audit selection
criteria are risk-based — the operator's
documented risk-profile algorithm targets
high-risk consignees, high-risk consignors,
high-risk goods categories, and audit
findings from earlier cycles.

### §12.2 Per-audit outcome recording

The audit outcome (no-discrepancy,
discrepancy-with-corrective-action,
discrepancy-with-penalty) is recorded in the
operator's audit envelope and bound to the
underlying declaration through the chain-of-
custody record.

## §13 KR-Jurisdiction Discipline

### §13.1 KR 관세법 binding

A KR-jurisdiction operator binds the
declaration to the relevant article of KR
관세법 (Customs Act) and the KR 자유무역협정
이행을 위한 관세법의 특례에 관한 법률 (Special
Act on the Implementation of FTA-Based
Customs).

### §13.2 KR 대외무역법 binding

The operator declares the KR 대외무역법
(Foreign Trade Act) reference for the
strategic-items export-control envelope.

### §13.3 KR 외국환거래법 binding

The operator binds the cross-border payment
record to KR 외국환거래법 (Foreign Exchange
Transactions Act) for the per-payment
declaration to the KR Bank of Korea where
the payment exceeds the declared threshold.
