# WIA-fire-resistant-material PHASE 3 — PROTOCOL Specification

**Standard:** WIA-fire-resistant-material
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a fire-resistant-material operator across the
manufacturer-to-laboratory-to-notified-body-to-
AHJ value chain: the EN 13501-1 reaction-to-fire
classification discipline that ties the material
to its Euroclass declaration, the EN 13501-2
resistance-to-fire classification discipline
that binds the load-bearing-and-separating
performance to the time-to-failure declaration,
the EU CPR Annex V conformity-assessment-system
discipline that gates the issuance of a
Declaration of Performance, the laboratory-
accreditation discipline that anchors every test
result to an ISO/IEC 17025 accredited test
report, the manufacturing factory-production-
control discipline that ties continuing
production to the test-of-record conditions, the
chain-of-custody anchoring discipline that
prevents silent mutation of the test result, the
post-installation Authority Having Jurisdiction
audit discipline, and the recall-and-corrective-
action discipline that handles a discovered non-
conformance.

References (CITATION-POLICY ALLOW only):

- EN 13501-1:2018+A1:2019, EN 13501-2:2023
- EN 13823:2020+A1:2022, EN ISO 11925-2:2020
- ISO 1182:2020, ISO 1716:2018, ISO 5660-1:2015,
  ISO 5660-2:2002, ISO 9239-1:2010, ISO
  13943:2017
- ASTM E84-23, UL 723:2018, ASTM E119-23a,
  ASTM E136-22, ASTM E2257-22
- NFPA 251, NFPA 252, NFPA 257, NFPA 259, NFPA
  268
- EN 1363-1:2020, EN 1364 series, EN 1365 series,
  EN 1366 series, EN 1634-1
- KS F 2271:2016, KS F 2257-1:2019
- ISO 9001:2015 (quality management systems)
- ISO/IEC 17000:2020, ISO/IEC 17021-1:2015,
  ISO/IEC 17025:2017, ISO/IEC 17065:2012
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 8615,
  RFC 6962
- W3C Trace Context
- EU Construction Products Regulation (EU)
  305/2011 (especially Annex V conformity-
  assessment systems and Article 39 notified-
  body designation)
- EU Decision 2000/147/EC (reaction-to-fire
  classes)
- EU Decision 2000/367/EC (resistance-to-fire
  classes)

---

## §1 EN 13501-1 Reaction-to-Fire Discipline

### §1.1 Test-set completeness

Every reaction-to-fire test record carries the
test-set completeness check declared in PHASE-2
§4.1. The operator's API rejects a record whose
declared `classification.primaryClass` is not
supported by the underlying test set; the
rejection lists the missing test method per the
EN 13501-1 Annex B classification table.

### §1.2 Smoke-class and droplet-class binding

The smoke class (s1, s2, s3) is derived from
the EN 13823 smoke-growth rate index (SMOGRA)
and the total-smoke-production-in-600 s
(TSP-600s). The droplet class (d0, d1, d2) is
derived from the EN 13823 flaming-droplets-or-
particles observation. The operator's API
recomputes the two classes from the underlying
measurement set and rejects a record whose
declared classes do not match the recomputation.

### §1.3 Floor-finish classification

A flooring product (`materialFamily: floor`)
declares the floor-finish class per EN 13501-1
Annex C — A1FL, A2FL, BFL, CFL, DFL, EFL, FFL —
derived from ISO 1182, ISO 1716, ISO 9239-1
critical-radiant-flux, and EN ISO 11925-2.
The smoke class for floor finishes is s1 or s2;
the droplet class is not declared for floors.

## §2 EN 13501-2 Resistance-to-Fire Discipline

### §2.1 R / E / I criterion enforcement

Every resistance-to-fire test record is
classified per EN 13501-2 §7 against the
combined R (load-bearing capacity), E
(integrity), and I (insulation) criteria. The
operator's API enforces the criterion matching:
a record claiming a load-bearing-only
classification (R 60) is publishable only when
the test specimen's load configuration satisfies
EN 1363-1 §6 mechanical loading.

### §2.2 Heating-curve enforcement

The heating curve declared in the test record
binds the result's applicability scope. A
hydrocarbon heating curve test result cannot be
used to claim a standard cellulosic ISO 834
heating curve resistance class; the operator's
API rejects a classification that does not match
the heating curve.

## §3 EU CPR Conformity-Assessment-System Discipline

### §3.1 System 1+ for fire-performance products

A fire-performance product (a product whose
declared performance includes a reaction-to-fire
or resistance-to-fire essential characteristic)
falls in scope of EU CPR Annex V system 1+. The
notified body issues the certificate of
constancy of performance, performs the initial
type test (ITT), assesses the factory-production-
control system, and conducts continuing
surveillance.

### §3.2 ITT-and-FPC discipline

The initial type test (ITT) is the basis of the
declared performance under EU CPR Article 4. The
factory-production-control (FPC) system per EU
CPR Article 12 ensures that ongoing production
satisfies the ITT-of-record performance. The
operator's API records the per-batch FPC
measurement and binds the batch's DoP to the
FPC log.

## §4 Laboratory-Accreditation Discipline

### §4.1 ISO/IEC 17025 scope binding

Every test record is uploaded under an HTTP
Message Signature (RFC 9421) issued under the
testing laboratory's ISO/IEC 17025:2017
accreditation certificate. The operator's API
verifies the signature, the certificate's
currency, and the scope of the accreditation
against the declared test method. A scope
mismatch returns `403 Forbidden`.

### §4.2 Inter-laboratory comparison

A laboratory whose accreditation scope includes
a fire test method participates in an inter-
laboratory comparison run by the EU EFR (the
European Group of Notified Bodies for the CPR)
or an equivalent peer-review group. The
comparison's outcome (the laboratory's z-score
relative to the consensus statistic) is recorded
in the laboratory's profile so that a downstream
consumer can take the proficiency-testing record
into account.

## §5 Manufacturing Factory-Production-Control Discipline

### §5.1 Per-batch FPC measurement

The operator's API records the per-batch FPC
measurement (the production-line surrogate test
that correlates with the ITT — for example, the
gypsum-board specific gravity, the mineral-wool
fibre length distribution, the intumescent-
coating dry-film thickness). The FPC measurement
is run at the cadence declared in the FPC plan
(typically per shift, per truck, or per batch).

### §5.2 Out-of-control corrective action

Where an FPC measurement falls outside the
declared control limit, the operator's API
flags the affected batch and triggers an ISO
9001 §10.2 nonconformity-and-corrective-action
record. The corrective action's closure is
gated on a confirmatory ITT-equivalent test
that confirms the batch's classification
remains valid.

## §6 Chain-of-Custody Anchoring Discipline

### §6.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency
log modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure. The log
publishes a signed tree-head every signed-tree-
head period (default 24 h, configurable per
programme).

### §6.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended.

## §7 AHJ Audit Discipline

### §7.1 As-built verification

The Authority Having Jurisdiction inspecting the
installed material verifies the as-built
condition against the DoP declared performance.
The verification carries the building-permit
reference, the date of inspection, and the
audit outcome. A non-conforming outcome
triggers the AHJ's enforcement workflow under
the local building code.

### §7.2 Retroactive recall linkage

Where a DoP is later withdrawn (PHASE-2 §6) due
to a discovered nonconformance, the operator's
API publishes the withdrawal notice on the
public retrieval endpoint and notifies every
AHJ that has issued an audit referencing the
withdrawn DoP so that the AHJ can re-audit the
affected installation.

## §8 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the material design,
testing, classification, DoP publication, factory
production control, and chain-of-custody
processes. Internal audits run on a frequency
declared in the quality manual; the
nonconformity register is reviewed in the ISO
9001 §9.3 management-review cycle. Notified
bodies operating under EU CPR Article 39 audit
the operator's QMS as part of the continuing
surveillance discipline declared in §3.

## §9 Recall and Corrective-Action Discipline

### §9.1 EU CPR Article 56 corrective measures

Where a DoP is found to misrepresent the
declared performance, the manufacturer publishes
a corrective measure under EU CPR Article 56
(withdraw, recall, or correct). The notified
body issuing the certificate of constancy of
performance is informed and may suspend or
withdraw the certificate.

### §9.2 Public-retrieval annotation

The operator's API annotates the public DoP
retrieval response with the corrective-measure
status flag and the date of the corrective
measure so that a downstream building designer
or AHJ can detect the measure without bilateral
notification.

## §10 KR-Jurisdiction Discipline

### §10.1 KS F 2271 / KS F 2257-1 binding

A KR-jurisdiction operator declares the KS F
2271 surface-burning test and the KS F 2257-1
fire-resistance test in the relevant test
records. The KR Korean Building Code (건축물의
피난·방화구조 등의 기준에 관한 규칙) cites
these KS standards as the conformance basis for
domestic-market building products.

### §10.2 KR 건축자재 품질인정 discipline

A material in scope of the KR Quality
Recognition System for Building Materials is
bound to the KR 건축자재 품질인정 인증 번호
declared in the material record's
`identifierBindings`. The operator's API queries
the KR quality-recognition register on each
publication request.

## §11 Smoke and Toxicity Discipline

### §11.1 Smoke-density binding

For materials intended for egress-route finishes
(`intendedUse: egress-route-finish`), the
operator's API records the smoke-density
measurement per ASTM E662 (smoke generated by
solid materials) where the AHJ's local code
references the test, in addition to the EN
13501-1 smoke class.

### §11.2 Toxic-gas declaration

Where the operator publishes a toxic-gas
emission profile (CO, CO₂, HCl, HBr, HCN, SO₂,
NOₓ) per the relevant national code (for
example, the BS 6853:1999 railway code), the
emission profile is bound to the underlying
test report and is published on the public
retrieval endpoint.

## §12 ASTM E84 / UL 723 Steiner-Tunnel Discipline

A material declaring an ASTM E84 / UL 723
classification (Class A, B, or C per the FSI
and SDI thresholds — Class A: FSI ≤ 25 and
SDI ≤ 450; Class B: FSI 26–75 and SDI ≤ 450;
Class C: FSI 76–200 and SDI ≤ 450) carries the
underlying tunnel test report from a UL-
registered laboratory or an ASTM-recognised
laboratory operating under ISO/IEC 17025. The
operator's API verifies that the declared FSI
and SDI ranges match the classification.

The Steiner-tunnel test is run for ten minutes
under a controlled flame source and the smoke-
optical-density-versus-time curve is integrated
to derive the SDI per ASTM E84 §11. The test
specimen's mounting orientation (horizontal,
vertical) is declared in the test record and
checked against the material's intended-use
declaration.

## §13 ASTM E119 Time-Temperature Discipline

A material declaring an ASTM E119 hourly fire-
resistance rating (1-hour, 2-hour, 3-hour,
4-hour) carries the underlying ASTM E119 test
report from a UL-registered or NFPA-recognised
laboratory. The operator's API verifies the
ASTM E119 standard time-temperature curve was
applied (the curve is published in ASTM E119
§7), and that the test specimen's load
configuration satisfies the load-bearing claim.
