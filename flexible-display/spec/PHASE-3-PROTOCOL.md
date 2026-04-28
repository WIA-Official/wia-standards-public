# WIA-flexible-display PHASE 3 — PROTOCOL Specification

**Standard:** WIA-flexible-display
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a flexible-display operator across the
manufacturer-to-laboratory-to-notified-body-to-
buyer value chain: the IEC 62977 / SID IDMS
optical-measurement discipline that anchors the
optical-performance declaration, the IEC 62715
mechanical-flexibility discipline that gates
the foldability and rollability declarations,
the IEC 62341 OLED-specific discipline that
binds the emissive-pixel parameters to the
optical record, the ISO 9241-307 ergonomic
compliance discipline that ties the device to
its intended-use viewing distance, the IEC
60068 environmental-reliability discipline that
qualifies the device against the operating-
environment envelope, the IEC 61000 EMC
discipline and the IEC 62368-1 safety discipline
that gate CE marking, the laboratory-
accreditation discipline that anchors every
measurement to an ISO/IEC 17025 accredited test
report, the chain-of-custody anchoring discipline
that prevents silent mutation of the test
result, and the recall-and-corrective-action
discipline that handles a discovered non-
conformance.

References (CITATION-POLICY ALLOW only):

- IEC 62977 series (multimedia display
  measurement methods)
- IEC 62715 series (flexible display devices)
- IEC 62341 series (OLED display)
- IEC 61747 series (LCD module)
- IEC TS 62687:2014
- ISO 9241 series (-302, -303, -306, -307)
- SID IDMS 1.03
- JEITA RC-9131
- KS C IEC 62715-1-1, KS C IEC 62977-1
- IEC 60068 series (environmental tests)
- IEC 61000 series (EMC test methods)
- IEC 62368-1:2018+AMD1:2020 (audio-video
  safety)
- ISO 9001:2015
- ISO/IEC 17000:2020, ISO/IEC 17021-1:2015,
  ISO/IEC 17025:2017, ISO/IEC 17065:2012
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 8615,
  RFC 6962
- W3C Trace Context
- EU Low Voltage Directive 2014/35/EU, EU EMC
  Directive 2014/30/EU, EU RoHS 2011/65/EU, EU
  REACH (EC) 1907/2006

---

## §1 IEC 62977 / SID IDMS Optical-Measurement Discipline

### §1.1 Measurement-geometry binding

Every optical-performance record carries the
measurement geometry declared in PHASE-1 §4
(`flat-state`, `bent-at-declared-radius`,
`rolled-fully`, `rolled-partially`, `folded-
fully`, `folded-partially`). The operator's API
enforces that a `flexibilityClass: foldable`
device's optical record set covers at least the
flat-state and the folded-fully geometries; a
record set missing one of the two is rejected
with `422 Unprocessable Entity` at
`/problems/iec62977-geometry-set-incomplete`.

### §1.2 SID IDMS procedure traceability

Every optical-record measurement procedure
declared in `testStandard` is bound to the SID
IDMS 1.03 procedure reference. The operator's
API publishes the per-measurement traceability
link so that a downstream metrology laboratory
can verify the procedure-of-record without
contacting the operator.

### §1.3 Calibration-record linkage

The optical record carries a calibration-record
reference for the spectroradiometer or
luminance meter used in the measurement. The
calibration record's accreditation reference
(ISO/IEC 17025) and its expiry date are bound
to the optical record so that a downstream
auditor can detect the use of an out-of-
calibration instrument.

## §2 IEC 62715 Mechanical-Flexibility Discipline

### §2.1 Bend-radius enforcement

Every mechanical-reliability record declares the
bend radius applied during the test. The
operator's API rejects a record whose declared
bend radius is below the device's
`declaredBendRadiusMm` (the device's qualified
minimum bend radius) — a test conducted below
the qualified envelope cannot be used as
evidence for the qualification claim.

### §2.2 Fold-cycle endurance binding

A foldable device's `declaredFoldCycles` value
declared in PHASE-1 §3 binds the device to the
IEC 62715-6-1 test record that demonstrates the
declared cycle count. The operator's API
verifies the binding on each declaration and
rejects a publication where the device's
`declaredFoldCycles` exceeds the test-record
fold-cycle endurance.

### §2.3 Crease-formation criterion

The IEC 62715-6-1 test result declares the cycle
count at first crease formation. The operator's
API records the criterion under which crease
formation was identified (visual inspection
under controlled lighting, optical-degradation
threshold, or strain-gauge reading) so that
the criterion is part of the test-of-record.

## §3 IEC 62341 OLED-Specific Discipline

### §3.1 Emissive-pixel binding

An OLED-class flexible display (`displayFamily:
oled-foldable`, `oled-rollable`, `oled-bendable`)
is qualified against the IEC 62341-5-2 image-
quality measurement and the IEC 62341-6-2
mechanical-characteristics test method. The
operator's API rejects an OLED-class device
whose record set omits at least one of the
two test methods.

### §3.2 Burn-in and lifetime binding

The IEC 62341-2-2 lifetime test (the L70
half-life criterion — the time at which the
panel's luminance decays to 70 % of its
initial value) is recorded against the
display record. The operator's API publishes
the L70 hours value as part of the device's
performance envelope so that a buyer can
parameterise the device's expected lifetime.

## §4 ISO 9241-307 Ergonomic-Compliance Discipline

A device intended for general consumer use
satisfies the ISO 9241-307 ergonomic compliance
test methods and is classified at the
ergonomic-class declared in the device record.
The operator's API publishes the per-device
ergonomic class and the underlying ISO 9241-307
test-of-record reference; a device classified
above its substantiation evidence is rejected
with `422 Unprocessable Entity`.

## §5 IEC 60068 Environmental-Reliability Discipline

### §5.1 Test-method-to-application binding

The environmental-reliability record's
`testStandard` is bound to the device's
intended-use environment declaration: an
automotive curved-cluster display is expected
to satisfy IEC 60068-2-14 thermal cycling at
the automotive-grade temperature envelope, while
a consumer foldable phone is expected to satisfy
IEC 60068-2-30 damp heat at the consumer-grade
envelope.

### §5.2 Thermal-cycling endpoint

The IEC 60068-2-14 thermal-cycling endpoint is
declared as the post-cycle pixel-defect count,
the post-cycle dark-defect area, and the post-
cycle brightness-and-colour drift. The
operator's API enforces that the endpoint
metrics fall within the manufacturer's declared
envelope.

## §6 EMC and Safety Discipline

### §6.1 IEC 61000 EMC binding

Every device record carries the IEC 61000 test-
report reference for line harmonics (IEC
61000-3-2), line-voltage variations (IEC
61000-3-3), ESD immunity (IEC 61000-4-2), and
radiated-RF immunity (IEC 61000-4-3) so that
the EU EMC Directive 2014/30/EU declaration is
substantiated.

### §6.2 IEC 62368-1 hazard-based safety
       engineering

Every device record carries the IEC 62368-1
HBSE classification (the energy-source class
table — ES1 / ES2 / ES3, MS1 / MS2 / MS3, TS1
/ TS2 / TS3, etc.) and the per-class
substantiation evidence so that the EU LVD
declaration is substantiated.

## §7 Laboratory-Accreditation Discipline

Every test record is uploaded under an HTTP
Message Signature (RFC 9421) issued under the
testing laboratory's ISO/IEC 17025:2017
accreditation. The operator's API verifies the
signature, the certificate's currency, and the
scope of the accreditation against the declared
test method. A scope mismatch returns `403
Forbidden`.

## §8 Chain-of-Custody Anchoring Discipline

### §8.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency
log modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure.

### §8.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended.

## §9 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the device design,
the line-acceptance test, the per-batch
release, the warranty-claim handling, and the
recall workflow. Internal audits run on a
frequency declared in the quality manual; the
nonconformity register is reviewed in the ISO
9001 §9.3 management-review cycle.

## §10 Recall and Corrective-Action Discipline

### §10.1 EU LVD Article 8 recall

A device found to expose users to an
unacceptable safety risk under EU LVD 2014/35/EU
triggers a recall declaration. The operator's
API publishes the recall notice on the public
retrieval endpoint and notifies registered
distributors via the webhook endpoint declared
in PHASE-2 §14.

### §10.2 Corrective-action record

Every recall is bound to a corrective-action
record under the operator's ISO 9001 §10.2
discipline. The record carries the root-cause
analysis, the containment actions, the
production-line corrective actions, and the
verification-of-effectiveness evidence.

## §11 KR-Jurisdiction Discipline

### §11.1 KR 전파법 적합성평가 binding

A device sold in the KR market is bound to the
KR 전파법 방송통신기자재등의 적합성평가 인증
번호 declared in the device record. The KR
Ministry of Science and ICT operates the
register; the operator's API queries the
register on each retrieval after the caching
TTL.

### §11.2 KR 전기용품안전관리법 binding

Where the device is in scope of the KR
전기용품 및 생활용품 안전관리법 KC marking,
the operator declares the KC certificate
reference. The KR National Institute of
Technology and Standards operates the register;
the operator's API queries the register on
each retrieval after the caching TTL.

## §12 Warranty-Claim Discipline

### §12.1 Warranty-period binding

The device's warranty period is bound to the
device record. Where a warranty claim
references a device whose `declaredFoldCycles`
or `declaredBendRadiusMm` was exceeded, the
operator's warranty-claim service may decline
the claim per the warranty terms; the decline
is recorded as a chain-of-custody event so
that the consumer-protection authority can
audit the claim outcome.

### §12.2 Field-failure feedback loop

Field-failure data flowing from warranty
claims feeds the operator's reliability-
engineering team. The team reviews the failure
modes against the IEC 62715 test-of-record and
adjusts the test-method-of-record where the
field failures expose a gap in the qualification
envelope.

## §13 RoHS and REACH Substance Discipline

### §13.1 EU RoHS Directive 2011/65/EU compliance

A device sold in the EU market is bound to the
EU RoHS Directive 2011/65/EU restricted-
substance limits. The operator's API publishes
the per-device material declaration declaring
the lead, mercury, cadmium, hexavalent chromium,
PBB, PBDE, DEHP, BBP, DBP, and DIBP content
ranges so that a downstream auditor can
substantiate the compliance claim.

### §13.2 EU REACH SVHC supply-chain communication

A device whose component-of-record contains a
substance of very high concern (SVHC) above the
0.1 % weight-by-weight threshold publishes the
EU REACH Article 33 supply-chain communication
envelope so that the downstream system
integrator and the consumer can be informed of
the substance presence.

## §14 Bistable-Display Discipline

An electronic-paper bistable display
(`displayFamily: electronic-paper-flexible`) is
characterised against the SID IDMS bistable-
display measurement section (the bistable
state-retention time, the per-state contrast
ratio, the per-state response time, the
per-state luminance). The operator's API records
the per-state measurements separately so that a
downstream consumer can parameterise the
expected behaviour of the bistable display.
