# WIA-economic-integration PHASE 4 — INTEGRATION Specification

**Standard:** WIA-economic-integration
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an economic-
integration operator integrates with the
systems that surround cross-border trade: the
World Trade Organization secretariat operating
the TFA Notifications and Trade Policy
Reviews; the World Customs Organization
operating the WCO SAFE MRA register and the
HS Committee revision cycle; the UN/CEFACT
secretariat publishing the Recommendations
and the Single Window Repository; the UN
COMTRADE secretariat operating the cross-
country trade-statistics database; the SWIFT
network operating the cross-border payments
correspondent-banking layer; the ICC Banking
Commission operating the UCP / ISBP / URDG /
URBPO; the EU Customs Single Window operating
the EU's centralised submission layer; the
ASEAN Single Window operating the ASEAN's
network; and the supervisory data-protection
authority overseeing per-message processing
under GDPR and equivalent frameworks.

References (CITATION-POLICY ALLOW only):

- WTO TFA, GATT 1994, GPA, TRIPS, Customs
  Valuation Agreement
- WCO SAFE 2018, WCO Data Model v3, WCO
  HS-2022, WCO AEO Programme, WCO MRA
- UN/EDIFACT, UN/CEFACT Recommendations 1, 16,
  21, 33, 36, ISO 9735-1
- UN COMTRADE submission specification
- ISO 20022 message family + SWIFT GPI
- ISO 4217, ISO 3166-1, ISO 9362 (BIC), ISO
  13616 (IBAN), ISO 17442 (LEI)
- ICC Incoterms 2020, ICC UCP 600, ICC ISBP
  745, ICC URDG 758, ICC URBPO 750
- ISO 9001:2015, ISO/IEC 27001:2022, ISO/IEC
  17021-1:2015, ISO/IEC 17065:2012
- IETF RFC 8259, RFC 9457, RFC 8615, RFC
  9421, RFC 6234, RFC 6962
- W3C Trace Context, W3C ODRL 2.2, W3C VC
  v2.0
- EU UCC Regulation (EU) 952/2013 + EU
  Implementing Regulation 2015/2447 +
  Delegated Regulation 2015/2446
- EU Dual-Use Regulation (EU) 2021/821, EU
  Common Military List
- KR 관세법, KR 대외무역법, KR 외국환거래법, KR
  자유무역협정 이행을 위한 관세법의 특례에 관한
  법률
- ASEAN Economic Community Blueprint, USMCA,
  CPTPP, RCEP

---

## §1 WTO Secretariat Integration

### §1.1 TFA Notification Submission

The operator's national customs authority
submits the operator's TFA notification
schedule (the per-Article notification
deadlines for Categories A, B, C) through
the WTO TFA Facility's online submission
endpoint. The operator's API publishes the
per-notification status (committed,
implemented, suspended) for transparency.

### §1.2 Trade Policy Review

The operator's national authorities
contribute to the WTO Trade Policy Review
cycle. The per-cycle submission carries the
operator's per-period trade-facilitation
performance indicators.

## §2 WCO Secretariat Integration

### §2.1 WCO SAFE MRA register

The operator queries the WCO SAFE MRA
register on each cross-border declaration
to verify the partner's AEO status. The
register is queried with the partner's AEO
identifier and the operator's MRA
participation reference.

### §2.2 HS Committee revision integration

The WCO HS Committee revises the Harmonised
System every five years; the operator
subscribes to the HS Committee's revision-
publishing endpoint and triggers an internal
review cycle when a new edition is
published.

### §2.3 WCO Data Model maintenance

The operator subscribes to the WCO Data Model
maintenance endpoint and keeps the per-
element mapping table current with the
published revisions.

## §3 UN/CEFACT Secretariat Integration

The UN/CEFACT secretariat publishes the
Recommendations and the Single Window
Repository. The operator subscribes to the
secretariat's publishing endpoint and binds
the operator's per-Recommendation
implementation to the published reference.

## §4 UN COMTRADE Integration

The operator's national statistical office
publishes the per-period trade-statistics
dataset to UN COMTRADE through the COMTRADE
submission endpoint. The submission carries
the per-row commodity code (HS-2022 + the
operator's national-extension digits), the
partner-country code (ISO 3166-1), the trade-
flow direction (import, export, re-import,
re-export), and the per-row quantity-and-
value envelope.

## §5 SWIFT Network Integration

A financial-institution operator participates
in the SWIFT network through its BIC
identifier. The SWIFT GPI tracking layer
publishes the per-payment leg status; the
operator's API ingests the GPI tracking
events and binds them to the per-payment
record.

## §6 ICC Banking Commission Integration

The ICC Banking Commission publishes the
UCP, ISBP, URDG, and URBPO with periodic
revisions. The operator subscribes to the
Banking Commission's publishing endpoint and
adjusts the operator's documentary-credit
discipline when a revision is published.

## §7 EU Customs Single Window Integration

A EU-Member-State operator binds the
operator's declaration register to the EU
Customs Single Window for Certificates
Exchange (EU CSW-CERTEX) where the
declaration involves a non-customs
certificate (a sanitary certificate, a
phytosanitary certificate, a CITES
certificate). The CSW-CERTEX endpoint
verifies the certificate's currency against
the issuing authority's register.

## §8 ASEAN Single Window Integration

An ASEAN-Member-State operator binds the
operator's declaration register to the
ASEAN Single Window. The ASW endpoint
exchanges the per-declaration message
between the ten ASEAN member states' national
single windows.

## §9 Supervisory Data-Protection Authority Integration

The supervisory data-protection authority
overseeing the operator's per-message
processing audits the operator's records of
processing activities on demand. Where the
operator processes personal data (the
trader's natural-person LEI alternative, the
beneficiary individual's account holder
data), the operator's API publishes the
records to the authority's endpoint.

## §10 Public Retrieval and Re-Issuance

### §10.1 Public declaration summary

The operator publishes per-period trade-
volume statistics on the public-portal
endpoint without per-declaration
identifiers. The aggregated dataset is
consumed by researchers, journalists, and
policy analysts.

### §10.2 Verifiable-credentials re-issuance

A certificate of origin is re-issuable as a
W3C Verifiable Credential signed by the
issuing chamber's public-key set so that a
downstream destination-customs admin can
verify the certificate without contacting
the chamber directly.

An AEO certificate is similarly re-issuable
as a Verifiable Credential signed by the
issuing customs authority's public-key set.

## §11 KR-Jurisdiction Integration

### §11.1 KR 관세청 register integration

A KR-jurisdiction operator binds the
declaration to the KR Customs Service's
UNI-PASS register. The KR Customs Service
operates the register; the operator's API
queries the register on each retrieval
after the caching TTL.

### §11.2 KR 한국무역협회 (KITA) trade-data
       integration

A KR-jurisdiction operator participating in
the KITA trade-data exchange binds the
declaration to the KITA platform.

### %11.3 KR 한국무역정보통신 (KTNET)
       integration

A KR-jurisdiction operator using the KTNET
single-window-of-trade exchanges UN/EDIFACT
messages through KTNET's electronic-trade
platform.

### §11.4 KR 자유무역협정 portal integration

The KR FTA portal published by the KR
Ministry of Trade, Industry and Energy
provides the per-FTA preferential-origin
verification endpoint that an importing
foreign customs admin can query.

## §12 Audit and Conformity-Assessment
       Integration

### §12.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system
declared in PHASE-3 §11 is audited under
ISO/IEC 17021-1 by an accredited
certification body.

### §12.2 ISO/IEC 17065 product certification

Where the operator's per-AEO certification or
per-origin certification is issued under a
product-certification scheme, the
certification body's ISO/IEC 17065:2012
accreditation is bound to the certification
reference.

## §13 Regional Economic-Union Integration

### §13.1 ASEAN Economic Community

An ASEAN-Member-State operator publishes the
per-period market-integration dataset to the
ASEAN Economic Community Blueprint
monitoring endpoint.

### §13.2 USMCA / CPTPP / RCEP committees

A signatory operator publishes the per-
agreement implementation report to the
agreement's joint committee endpoint where
the operator is empowered to do so.

## §14 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
economic-integration standard. Implementations
cite the multilateral and intergovernmental
agreements (WTO, WCO, UN/CEFACT, UN COMTRADE,
ISO 20022, ICC, ASEAN, USMCA, CPTPP, RCEP)
and the ISO / IEC / IETF / W3C / EU / KR
references by their issuing organisation and
the publication year so that a downstream
consumer can locate the authoritative text.
Updates to a cited standard (for example,
the next HS revision, a new ISO 20022
message family release, a new ICC UCP
edition) trigger an internal review cycle in
the operator's quality-management discipline
declared in PHASE-3 §11 before the new
revision is bound into the operator's
enumeration set.
