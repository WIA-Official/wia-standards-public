# WIA-flexible-display PHASE 4 — INTEGRATION Specification

**Standard:** WIA-flexible-display
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a flexible-display
operator integrates with the systems that
surround a flexible-display product: the EU
notified body designated under EU LVD or EU EMC
Directive; the ISO/IEC 17025 metrology
laboratory's accreditation body; the ISO/IEC
17065 product-certification body; the SID
International Display Metrology Committee
publishing the IDMS measurement procedures; the
JEITA secretariat publishing the JP-jurisdiction
industry-standard reference; the device's
upstream display-manufacturing line operator;
the device's downstream system integrator (the
phone manufacturer, the television
manufacturer, the automotive cluster
integrator); the consumer-protection authority
auditing a warranty-claim dispute; and the
customs authority verifying the import
classification on the commercial invoice.

References (CITATION-POLICY ALLOW only):

- IEC 62977 series, IEC 62715 series, IEC
  62341 series, IEC 61747 series, IEC TS
  62687, ISO 9241-302/-303/-306/-307
- SID IDMS 1.03, JEITA RC-9131, KS C IEC
  62715-1-1, KS C IEC 62977-1
- IEC 60068 series, IEC 61000 series, IEC
  62368-1
- ISO 9001:2015, ISO/IEC 17000, ISO/IEC
  17021-1, ISO/IEC 17025, ISO/IEC 17065
- IETF RFC 8259, RFC 9457, RFC 8615, RFC
  9421, RFC 6962
- W3C Verifiable Credentials Data Model 2.0
- EU LVD 2014/35/EU, EU EMC 2014/30/EU, EU
  RoHS 2011/65/EU, EU REACH (EC) 1907/2006
- US 47 CFR Part 15 (FCC RF emission limits),
  US OSHA workplace radiation safety
- KR 전파법 + 전기용품 및 생활용품 안전관리법
- JP 電気用品安全法 (PSE) and JP 電波法 (Radio
  Law) and JP 工業標準調査会 (JIS) — cited where
  the device is sold in the JP market

---

## §1 Notified-Body Integration

### §1.1 EU LVD / EMC notified body

A device in scope of EU LVD 2014/35/EU or EU
EMC Directive 2014/30/EU is conformity-
assessed under the directive's assessment
modules (typically self-declaration plus EU-
type examination by a notified body for
class-I devices). The operator's API binds the
device record to the notified body's assessment
identifier and the certificate's expiry date.

### §1.2 Surveillance audit envelope

The notified body conducts surveillance audits
on the operator's QMS at a frequency declared
in the conformity-assessment module. The
operator's audit envelope records the audit
findings and the closure dates.

## §2 Metrology-Laboratory Integration

### §2.1 ISO/IEC 17025 register query

Every optical-record signature is verified
against the issuing accreditation body's
ISO/IEC 17025 register. The accreditation scope
MUST cover the declared test method (IEC 62977,
IEC 62715, IEC 62341, ISO 9241-307, SID IDMS).

### §2.2 Calibration-traceability chain

The metrology laboratory's instrument
calibration is traceable to a national
metrology institute (NMI). The calibration-
record reference declared in the optical record
binds the instrument calibration to the NMI
chain so that the optical measurement is
traceable to the SI primary standard.

## §3 SID International Display Metrology Committee Integration

The SID IDMC publishes the IDMS measurement
procedure manual on a multi-year cycle. The
operator subscribes to the IDMC publishing
endpoint and triggers an internal review cycle
when an IDMS revision is published. A measurement
procedure that is deprecated by IDMC is marked
deprecated in the operator's enumeration set
but is not removed for the duration of the
operator's record-retention period.

## §4 Display-Manufacturing-Line Integration

### §4.1 Per-batch line-acceptance test

The display-manufacturing-line operator runs a
per-batch line-acceptance test that samples the
batch against the SID IDMS optical procedures
and the IEC 62715 mechanical test-of-record.
The line-acceptance test result is bound to the
batch identifier in the operator's chain-of-
custody record so that a downstream system
integrator can verify the batch's qualification
status.

### §4.2 Yield-and-defect feedback

The line operator's yield-and-defect feedback
loop returns to the upstream display-design
team. The operator's API records the feedback
loop's per-batch defect-mode count so that the
design team can adjust the design-of-record
where the defect-mode population indicates a
process-window drift.

## §5 System-Integrator Integration

### §5.1 Module-to-product binding

A phone, television, automotive cluster, or
wearable manufacturer (the system integrator)
integrates the flexible-display module into a
system-level product. The integrator binds the
module's identifier (PHASE-1 §3 record) to the
system-level identifier (the phone model
number, the TV model number, the automotive
cluster part number) so that a downstream
warranty-claim or recall workflow can trace
the system-level event to the underlying
module.

### §5.2 System-level certification cascading

A system-level CE marking, FCC certification,
or KR KC marking is partially substantiated by
the module-level qualification. The system
integrator's certification record links to the
module's qualification record under the
operator's API so that the substantiation chain
is preserved.

## §6 Consumer-Protection Authority Integration

A consumer-protection authority auditing a
warranty-claim dispute queries the operator's
API for the device's warranty-claim history,
the device's `declaredFoldCycles` /
`declaredBendRadiusMm`, and the warranty
decline outcome (where applicable). The
authority's audit trail is preserved through
the per-event chain-of-custody record.

## §7 Customs Authority Integration

A customs authority verifying the import
classification on the commercial invoice
queries the operator's API for the per-device
declaration of conformity. The customs
authority's query carries the declared HS code
and the declared classification; the operator's
API returns the per-device CE marking, FCC
certification, or KR KC marking reference.

## §8 KR-Jurisdiction Integration

### §8.1 KR 전파법 register

The KR 전파법 적합성평가 register is operated
by the KR Ministry of Science and ICT. The
operator's API queries the register on each
device-registration request and on each
retrieval after the caching TTL.

### §8.2 KR 전기용품 KC marking register

The KR KC marking register is operated by the
KR National Institute of Technology and
Standards. The operator's API queries the
register on each retrieval after the caching
TTL.

### §8.3 KR 전자제품 등의 자원순환에 관한 법률
       integration

A device in scope of the KR Resource
Recirculation Act for Electronic Equipment is
bound to the act's recovery-and-recycling
declaration. The operator publishes the per-
device recovery-and-recycling envelope so that
the KR Ministry of Environment can audit the
declaration.

## §9 JP-Jurisdiction Integration

### §9.1 JP PSE marking

A device sold in the JP market is bound to the
JP 電気用品安全法 PSE marking. The operator's
API publishes the PSE certificate reference.

### §9.2 JP 電波法 marking

A device with a wireless interface is bound to
the JP 電波法 marking under the JP Ministry of
Internal Affairs and Communications register.
The operator's API publishes the radio-law
certificate reference.

### §9.3 JEITA RC-9131 reference

A JP-jurisdiction operator declares the JEITA
RC-9131 industry-standard reference in the
programme record's `governingFrameworks` set.

## §10 US-Jurisdiction Integration

### §10.1 US FCC Part 15 marking

A device with a wireless interface is bound to
the US FCC Part 15 certification. The operator's
API publishes the FCC ID and the supplier's
declaration of conformity reference.

### §10.2 US OSHA workplace radiation safety

Where the device is intended for workplace use
(an industrial heads-up display, a manufacturing-
floor display panel), the operator publishes
the per-device OSHA-compliance attestation under
the workplace-safety regime.

## §11 Public Retrieval and Re-Issuance

### §11.1 Public DoC retrieval

A public consumer (a buyer evaluating the
product, a system integrator integrating the
module, a regulator running a market-
surveillance audit) retrieves the device record
at `/v1/displays/{displayId}` without
authentication; the response carries the public
fields and the underlying test summary.

### §11.2 Verifiable-credentials re-issuance

A device's CE marking, FCC ID, KR KC marking,
or JP PSE certificate is re-issuable as a W3C
Verifiable Credential signed by the operator's
public-key set so that a downstream system
integrator can validate the certification
without contacting the operator directly.

## §12 Audit and Conformity-Assessment Integration

### §12.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system declared
in PHASE-3 §9 is audited under ISO/IEC 17021-1
by an accredited certification body.

### §12.2 ISO/IEC 17065 product certification

A device whose route to market includes a
product-certification mark (UL Recognised
Component, TUV mark, BS Mark, KR KC marking)
is bound to the certification body's ISO/IEC
17065:2012 accreditation. The marking, scope
of certification, and the certified product's
identifier are published so that a downstream
consumer can verify the certification before
relying on it.

## §13 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
flexible-display standard. Implementations cite
the standards by their issuing organisation
(IEC, ISO, IEEE, SID, JEITA, KS, EU regulatory
text, US FCC regulatory text, JP regulatory
text) and the publication year so that a
downstream consumer can locate the authoritative
text. Updates to a cited standard (for example,
an amendment to IEC 62715-6-1) trigger an
internal review cycle in the operator's quality-
management discipline declared in PHASE-3 §9
before the new revision is bound into the
operator's enumeration set.

## §14 Repair-Right and Lifetime Discipline

### §14.1 EU Right-to-Repair binding

A device sold in the EU market is bound to the
EU Right-to-Repair directive obligations where
applicable. The operator's API publishes the
per-device repair-information envelope (the
spare-parts-availability declaration, the
repair-instruction reference, the per-repair
cost ceiling) so that an authorised repair
service can substantiate the repair claim.

### §14.2 Display-lifetime declaration

The IEC 62341-2-2 L70 half-life declaration is
bound to the device record. A downstream system
integrator parameterises the system-level
lifetime declaration from the per-module L70
value and publishes the system-level lifetime
declaration on the system-level public retrieval
endpoint.

## §15 Cross-Border Customs-Tariff Mapping

A flexible-display device classified under the
World Customs Organisation Harmonised System
carries the per-region tariff classification
(for example, HS code 8529.90 or 8537.10
depending on the device's intended use). The
operator's API publishes the per-region
classification table so that a customs broker
can verify the tariff line on the commercial
invoice without bilateral coordination with the
manufacturer.
