# WIA-fire-resistant-material PHASE 4 — INTEGRATION Specification

**Standard:** WIA-fire-resistant-material
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a fire-resistant-
material operator integrates with the systems
that surround a fire-rated construction product:
the EU notified body designated under EU CPR
Article 39; the ISO/IEC 17025 fire-testing
laboratory's accreditation body; the ISO/IEC
17065 product-certification body; the building-
information-modelling tool ingesting the per-
material reaction-to-fire and resistance-to-fire
classification; the building-code authority
auditing the construction-permit submission;
the fire marshal's pre-occupancy inspection
service; the insurance underwriter pricing the
building's risk profile; the public-procurement
authority running a building-safety programme;
and the customs authority enforcing the import
classification on the commercial invoice.

References (CITATION-POLICY ALLOW only):

- EN 13501-1, EN 13501-2, EN 13823, EN ISO
  11925-2 and the per-element-test EN 1363 / EN
  1364 / EN 1365 / EN 1366 / EN 1634 series
- ISO 1182, ISO 1716, ISO 5660-1, ISO 5660-2,
  ISO 9239-1, ISO 13943
- ASTM E84, UL 723, ASTM E119, ASTM E136, ASTM
  E2257
- NFPA 251 / 252 / 257 / 259 / 268
- KS F 2271, KS F 2257-1
- ISO 9001:2015, ISO/IEC 17000, ISO/IEC 17021-1,
  ISO/IEC 17025, ISO/IEC 17065
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421,
  RFC 6962
- W3C Verifiable Credentials Data Model 2.0
- EU Construction Products Regulation (EU)
  305/2011 (especially Articles 4, 6, 7, 11–14,
  39, 56)
- EU NANDO database (the EU notified-body
  register)
- EU Decision 2000/147/EC and EU Decision
  2000/367/EC

---

## §1 Notified-Body Integration

### §1.1 EU NANDO database query

The operator's API binds every DoP record to a
notified-body designation reference. The NANDO
database is queried on each publication request
and on each retrieval after the caching TTL.
A withdrawn or restricted designation returns
`410 Gone` for the publication endpoint and
annotates the public retrieval endpoint with the
designation status.

### §1.2 Annex IX surveillance audit

The notified body conducts continuing
surveillance audits on the operator's QMS and
factory-production-control system per EU CPR
Article 39 and Annex V. The operator's audit
envelope records the audit dates, findings,
corrective actions, and closure dates so that
the notified body's audit trail is preserved.

## §2 Fire-Testing Laboratory Integration

### §2.1 Laboratory accreditation register

Every test record's signature is verified against
the issuing accreditation body's ISO/IEC 17025
register. The accreditation scope MUST cover
the declared test method; a scope mismatch
returns `403 Forbidden`. Accreditation registers
operated by the ILAC-recognised members are
trusted under the ILAC Mutual Recognition
Arrangement.

### §2.2 Laboratory proficiency-testing register

A laboratory participating in proficiency
testing publishes the round's outcome (the
laboratory's z-score, the round's identifier,
and the date of the round's publication). The
operator's API carries the laboratory's PT
profile as a non-blocking annotation on the
test record so that a downstream consumer can
take the proficiency-testing record into
account.

## §3 Product-Certification Body Integration

### §3.1 ISO/IEC 17065 register

A certification body issuing a product
certification (UL Listed, FM Approved, BS Mark)
is bound to its issuing accreditation body's
ISO/IEC 17065:2012 register. The operator's API
queries the register on each retrieval after the
caching TTL and refuses to publish a product
certification reference whose certification
body's accreditation is suspended.

### §3.2 Marking-and-scope discipline

The certification body's marking, scope of
certification, and the certified product's
identifier are published in the certificate
envelope so that a downstream consumer can
verify the certification scope before relying on
the marking.

## §4 Building-Information-Modelling Integration

A BIM tool consuming the operator's machine-
readable summary (PHASE-2 §12) parameterises
the building-level fire-protection design. The
summary carries:

- The reaction-to-fire Euroclass and the smoke /
  droplet sub-class.
- The resistance-to-fire class (REI, EI, R) and
  the time-to-failure value rounded to the
  declaration interval.
- The product's intended-use scope (interior
  wall, exterior wall, ceiling, floor, roof,
  structural beam-column, compartment
  separation).

A signature over the summary is verified by the
BIM tool against the operator's public-key set
so that the binding is preserved across BIM
authoring and downstream consumer review.

## §5 Building-Code Authority Integration

### §5.1 Construction-permit linkage

A building-code authority reviewing a
construction-permit submission queries the
operator's API for the materials referenced in
the submission. The query envelope carries the
permit reference, the per-material identifier,
and the required performance class; the response
returns the per-material DoP reference and the
underlying test record summary.

### §5.2 Pre-occupancy inspection linkage

The fire marshal's pre-occupancy inspection
queries the operator's API for the as-built
material list and the per-material DoP. The
inspection outcome is recorded as an AHJ-audit
record (PHASE-1 §7) so that the inspection
trail is preserved.

## §6 Insurance Underwriter Integration

An insurance underwriter pricing a building's
risk profile queries the operator's API for the
per-material DoP and the per-material AHJ-audit
outcome. The query envelope carries the building
reference, the underwriter's identifier, and the
underwriting purpose; the response returns the
material list, the per-material classification,
and the underlying notified-body certificate
identifier.

## §7 Public-Procurement Integration

A public-procurement authority running a
building-safety programme integrates with the
operator's API by querying the materials
satisfying the programme's safety criteria. The
query envelope carries the programme's
performance threshold (for example, EN 13501-1
A2 minimum for high-rise residential buildings)
and the response is an RFC 8288 `Link`-paginated
collection of materials satisfying the
threshold.

## §8 Customs Authority Integration

A customs authority enforcing the import
classification queries the operator's API for
the commercial-invoice material's DoP reference.
The customs authority's query carries the
declared HS code and the declared performance
class; the operator's API returns the per-
material DoP and the per-material classification
so that the customs authority can verify the
declaration on the commercial invoice.

## §9 KR-Jurisdiction Integration

### §9.1 KR 건축자재 품질인정 register

A KR-jurisdiction operator declares the KR
건축자재 품질인정 (Quality Recognition System
for Building Materials) reference in the
material record. The KR Ministry of Land,
Infrastructure and Transport (국토교통부)
operates the recognition register; the
operator's API queries the register on each
retrieval after the caching TTL.

### §9.2 KR 건축물의 피난·방화구조 binding

A material installed in a KR-jurisdiction
building is bound to the KR 건축물의 피난·방화
구조 등의 기준에 관한 규칙 article reference
that authorises the use. The KR 시·도 건축위원회
publishes the audit trail; the operator's API
references the audit envelope so that a
downstream consumer can verify the regulatory
basis for the installation.

### §9.3 KR 한국방재시험연구원 (KFRI) integration

A KS F 2271 or KS F 2257-1 test conducted at
the Korea Fire Institute (한국방재시험연구원) is
bound to the KFRI test-report reference. The
operator's API publishes the link to the KFRI
test-report endpoint so that a KR AHJ can
verify the test report directly.

## §10 Public Retrieval and Re-Issuance

### §10.1 Public DoP retrieval

A public consumer (a building designer
integrating the DoP into a fire-protection
design, an AHJ auditing the construction permit,
an insurance underwriter pricing risk) retrieves
the DoP record at `/v1/dop-records/{dopId}`
without authentication; the response carries the
public fields and the underlying test summary.

### §10.2 Verifiable-credentials re-issuance

A material's notified-body certificate of
constancy of performance is re-issuable as a
W3C Verifiable Credential. The credential
carries the notified body's issuer identifier,
the certificate reference, the scope of the
certification, the certificate's expiry, and
the issuing date; the credential is signed
using the notified body's public-key set so
that a downstream consumer can validate the
credential without contacting the notified body
directly.

## §11 Audit and Conformity-Assessment Integration

### §11.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system declared
in PHASE-3 §8 is audited under ISO/IEC 17021-1
by an accredited certification body. The audit
result is stored in the operator's audit
envelope and is referenced from the programme
record's `accreditationStatus`.

### §11.2 ISO/IEC 17065 product certification

A material whose route to market includes a
product-certification mark (UL Listed, FM
Approved, BS Mark, the KR 건축자재 품질인정
mark) is bound to the certification body's
ISO/IEC 17065:2012 accreditation. The
certification body's marking, scope of
certification, and the certified product's
identifier are published so that a downstream
consumer can verify the certification before
relying on it.

## §12 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
fire-resistant-material standard. Implementations
cite the standards by their issuing organisation
(EN, ISO, IEC, ASTM International, UL, NFPA,
KS, EU regulatory text) and the publication year
so that a downstream consumer can locate the
authoritative text. Updates to a cited standard
(for example, an amendment to EN 13501-1)
trigger an internal review cycle in the
operator's quality-management discipline declared
in PHASE-3 §8 before the new revision is bound
into the operator's enumeration set.

## §13 Cross-Border Fire-Class Mapping

A material classified under one regional
classification system (EU Euroclass under EN
13501-1, US Class A/B/C under ASTM E84, UK BS
476 Part 7 surface-spread-of-flame, KR 난연 1급
/ 2급 / 3급) may be marketed in another region
under that region's classification. The operator's
API publishes the mapping table that ties each
classification to the applicable evidence set
under the destination's regulator. The mapping
is annotated with an "approximate equivalence"
flag where the destination's regulator does not
formally recognise the source classification —
the destination's regulator's audit trail is
preserved through the AHJ-audit record set.

## §14 Insurance-Industry Risk-Score Integration

An insurance underwriter integrates the
operator's per-material classification with the
underwriter's risk-scoring model. The
underwriter's query envelope carries the
building's geometry, the per-zone material
list, and the per-zone occupancy class; the
response carries the per-zone fire-load
declaration, the per-zone fire-resistance
rating, and the per-material smoke / toxicity
sub-class so that the underwriter's risk model
can produce a per-zone underwriting score.

The operator's API does not publish the
underwriter's risk model (the model is the
underwriter's intellectual property); the API
publishes only the inputs the model consumes,
under the underwriter's data-use agreement.
