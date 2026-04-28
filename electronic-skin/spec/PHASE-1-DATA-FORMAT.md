# WIA-electronic-skin PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-electronic-skin
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-electronic-skin. The standard
covers the persistent record shapes that an
electronic-skin device manufacturer (a wearable
health-monitoring patch, a continuous biosignal
recorder, a sweat-chemistry sensor, a wound-
healing telemetry patch, an electromyography or
electrocardiography body-conformal patch), a
contract manufacturer running the medical-device
quality system, a notified body issuing the EU
MDR conformity assessment, an institutional review
board approving a clinical investigation, a
healthcare provider operating the patch under a
treating-clinician's order, and a healthcare
analytics service ingesting the device telemetry
maintain. Records are consumed by the treating
clinician at the point of care, by the patient
through the patient-portal export, by the
notified body during the EU MDR Annex IX surveillance
audit, by the institutional review board reviewing
the clinical investigation, by the device's
software-update service, and — where the device
is sold across borders — by the supervisory data-
protection authority overseeing the cross-border
transfer of body-borne sensor data.

References (CITATION-POLICY ALLOW only):

- IEC 60601-1:2005+AMD1:2012+AMD2:2020 (general
  requirements for basic safety and essential
  performance of medical electrical equipment)
- IEC 60601-1-2:2014+AMD1:2020 (electromagnetic
  disturbances — requirements and tests)
- IEC 60601-1-6:2010+AMD1:2013+AMD2:2020 (general
  requirements for usability)
- IEC 60601-1-9:2007+AMD1:2013+AMD2:2020
  (environmentally conscious design)
- IEC 80601-2-49:2018 (basic safety and essential
  performance of multifunction patient
  monitoring equipment)
- IEC 60601-2-25:2011 (electrocardiographs),
  IEC 60601-2-27:2011 (electrocardiographic
  monitoring equipment), IEC 60601-2-47:2012
  (ambulatory electrocardiographic systems),
  IEC 60601-2-26:2012 (electroencephalographs),
  IEC 60601-2-40:2016 (electromyographs and
  evoked-response equipment)
- IEEE 11073-10101:2019 (nomenclature for
  personal-health-device communication —
  cited normatively for the term codes carried
  by every observation in §4)
- IEEE 11073-10103:2014 (point-of-care medical-
  device nomenclature for implantable cardiac
  devices), IEEE 11073-10406:2011 (basic ECG),
  IEEE 11073-10407:2010 (blood pressure),
  IEEE 11073-10408:2010 (thermometer),
  IEEE 11073-10417:2017 (glucose meter),
  IEEE 11073-10418:2014 (INR), IEEE 11073-
  10421:2010 (peak expiratory flow),
  IEEE 11073-10472:2010 (medication monitor) —
  the per-device specialisation profiles cited
  where the e-skin patch implements the
  specialisation
- IEEE 802.15.6:2012 (wireless body area
  networks, the BAN baseline)
- ISO 14971:2019 (application of risk management
  to medical devices)
- ISO 13485:2016+A11:2021 (quality management
  systems for medical-device manufacturers)
- IEC 62366-1:2015+AMD1:2020 (application of
  usability engineering to medical devices)
- IEC 62304:2006+AMD1:2015 (medical-device
  software life-cycle processes)
- ISO 10993-1:2018 (biological evaluation of
  medical devices — evaluation and testing
  within a risk-management process), ISO 10993-5:
  2009 (in-vitro cytotoxicity), ISO 10993-10:2010
  (irritation and skin sensitisation), ISO 10993-
  23:2021 (irritation, replacing parts of -10)
- HL7 FHIR Release 5 (the international standard
  for representing and exchanging healthcare
  information electronically — cited normatively
  for the FHIR resources carrying the e-skin
  observation envelope in §5)
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)
- ISO/IEC 27001:2022 (information-security
  management — used for the chain-of-custody
  record discipline in §8)
- EU Medical Device Regulation (EU) 2017/745
  (general safety and performance requirements,
  Annex I, II, IX clinical-evaluation and
  surveillance discipline)
- US 21 CFR Part 820 (Quality System Regulation
  for medical devices)
- KR 의료기기법 (Medical Devices Act) and KR
  PIPA Articles 17, 23, 28-2 (sensitive personal
  information clauses for body-borne sensor data)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when an electronic-skin
device is registered, characterised under its
applicable IEC 60601 / IEC 80601 collateral and
particular standard, qualified through ISO 10993
biocompatibility testing, deployed under a
treating clinician's order, generating clinical
observations under the IEEE 11073-10101
nomenclature, and tracked through its software-
update and chain-of-custody history.
Implementations covered include:

- A continuous-cardiac-rhythm patch issuing ECG
  rhythm strips under IEC 60601-2-25/-27/-47.
- A continuous-glucose monitor with a body-
  conformal sensor, communicating per IEEE 11073-
  10417.
- A respiratory-rate / oxygen-saturation patch
  combining pulse-oximetry and accelerometry.
- A wound-healing telemetry patch reporting
  exudate volume and pH.
- An electromyography patch for rehabilitation.

The device-side IEC 60601 record, the IEEE 11073
observation record, and the FHIR R5 clinical-
observation record receive distinct encodings
in this PHASE; the additional safeguards required
by each regulator's review are encoded in
PHASE-3 §4.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       device manufacturer or
                       service operator)
operatorRole         : enum ("device-manufacturer" |
                       "contract-manufacturer" |
                       "notified-body" |
                       "clinical-investigation-
                       sponsor" | "treating-
                       healthcare-provider" |
                       "analytics-service" |
                       "user-defined")
governingFrameworks  : array of enum ("IEC-60601-
                       1" | "IEC-60601-1-2" |
                       "IEC-60601-1-6" |
                       "IEC-60601-1-9" |
                       "IEC-80601-2-49" |
                       "IEC-60601-2-25" |
                       "IEC-60601-2-27" |
                       "IEC-60601-2-47" |
                       "IEC-60601-2-26" |
                       "IEC-60601-2-40" |
                       "IEEE-11073-10101" |
                       "IEEE-11073-10406" |
                       "IEEE-11073-10417" |
                       "IEEE-802.15.6" |
                       "ISO-14971" |
                       "ISO-13485" |
                       "IEC-62366-1" |
                       "IEC-62304" |
                       "ISO-10993-1" |
                       "ISO-10993-5" |
                       "ISO-10993-10" |
                       "ISO-10993-23" |
                       "HL7-FHIR-R5" |
                       "EU-MDR-2017-745" |
                       "US-21-CFR-820" |
                       "KR-Medical-Devices-Act" |
                       "user-defined")
qmsCertification     : object (the ISO 13485
                       certification reference,
                       the issuing certification
                       body, and the certificate's
                       expiry)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Device Record

```
deviceRecord:
  deviceId           : string (uuidv7)
  identifierBindings : array of object (per-
                       jurisdiction device
                       identifiers — for example
                       the EU MDR Unique Device
                       Identifier (UDI-DI / UDI-
                       PI), the US FDA UDI, the
                       KR 의료기기 품목허가번호
                       — each carrying the issuing
                       authority and the scope of
                       use)
  deviceClass        : enum ("EU-MDR-class-IIa" |
                       "EU-MDR-class-IIb" |
                       "EU-MDR-class-III" |
                       "US-FDA-class-I" |
                       "US-FDA-class-II" |
                       "US-FDA-class-III" |
                       "user-defined")
  applicableParticular : array of enum ("IEC-
                       60601-2-25" | "IEC-60601-
                       2-27" | "IEC-60601-2-47" |
                       "IEC-60601-2-26" | "IEC-
                       60601-2-40" | "IEC-80601-
                       2-49" | "user-defined")
  signalChannels     : array of object (per-
                       channel modality with the
                       IEEE 11073-10101 term
                       code, sampling rate,
                       resolution, dynamic range,
                       and electrode count where
                       applicable)
  energySource       : enum ("battery-rechargeable"
                       | "battery-primary" |
                       "wireless-power-transfer"
                       | "energy-harvesting-
                       triboelectric" | "energy-
                       harvesting-photovoltaic" |
                       "energy-harvesting-
                       thermal" | "user-defined")
  patchSubstrate     : enum ("polyimide" |
                       "polydimethylsiloxane" |
                       "thermoplastic-
                       polyurethane" | "silicon-
                       elastomer" | "user-defined")
  biocompatibilityRecord : array of object (the
                       ISO 10993 test references
                       and the per-test result
                       summary)
```

## §4 Observation Record

The observation record carries per-channel
samples under the IEEE 11073-10101 nomenclature:

```
observationRecord:
  observationId      : string (uuidv7)
  deviceRef          : string (PHASE-1 §3 record
                       reference)
  patientRef         : string (PHASE-1 §6 record
                       reference; the operator's
                       master-patient identifier
                       resolves to the FHIR
                       Patient resource)
  channelRef         : string (PHASE-1 §3
                       `signalChannels` index)
  sampleStartTime    : string (ISO 8601 date-time
                       at sample number 0)
  sampleRateHz       : number (per the channel's
                       IEEE 11073-10101 unit code)
  sampleResolution   : number (per the channel's
                       IEEE 11073-10101 unit code)
  samples            : object (the encoded sample
                       block with the encoding
                       declaration — base64 binary
                       carrying signed 16-bit big-
                       endian samples for ECG /
                       EMG, signed 24-bit for
                       EEG, IEEE 754 single-
                       precision floats for
                       temperature)
  qualityFlags       : array of object (per-
                       segment artefact flags
                       under the IEC 60601-2-47
                       cardiac-rhythm cardiac-
                       event nomenclature)
```

## §5 FHIR Observation Mapping

The operator publishes a FHIR R5 `Observation`
resource for every clinically reportable event
derived from the §4 channel samples:

```
fhirObservation:
  identifier         : array (one is the operator's
                       internal observation
                       reference)
  status             : enum ("final" | "amended" |
                       "corrected" | "cancelled")
  category           : object (LOINC vital-signs)
  code               : object (LOINC + IEEE
                       11073-10101 cross-reference)
  subject            : object (FHIR Patient
                       reference)
  device             : object (FHIR Device
                       reference)
  effectiveDateTime  : string (ISO 8601)
  valueQuantity      : object (UCUM unit symbol)
  component          : array of object (per-
                       sub-channel value)
```

## §6 Patient and Consent Record

```
patientBinding:
  patientId          : string (uuidv7)
  pseudonymousId     : string (the operator's
                       master-patient identifier)
  consentDirective   : object (the FHIR R5 Consent
                       resource carrying the
                       patient's permission to
                       process body-borne sensor
                       data, the GDPR Article 9(2)
                       (a) explicit consent or
                       Article 9(2)(h) treatment
                       basis, the HIPAA 45 CFR
                       164.508 authorisation, the
                       KR PIPA Article 18 sensitive-
                       information consent)
```

## §7 Software-Update Record

```
softwareUpdate:
  updateId           : string (uuidv7)
  deviceRef          : string (PHASE-1 §3)
  releaseLevel       : enum ("major" | "minor" |
                       "patch" | "security-only" |
                       "user-defined")
  iec62304Class      : enum ("A" | "B" | "C")
  signedManifest     : string (URI of the IEC
                       62304 §5 software-of-known-
                       provenance manifest)
  releaseDate        : string (ISO 8601)
  postmarketSurveillanceRef : string (the EU MDR
                       Annex III post-market
                       surveillance plan reference
                       under which the update is
                       evaluated)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the device,
                       observation, software-
                       update, or biocompatibility
                       record identifier)
  custodyEvent       : enum ("manufacture-
                       complete" | "lot-release" |
                       "shipment" | "patient-
                       application" | "patient-
                       removal" | "data-
                       transmission" |
                       "decommission" | "recall" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "electronic-skin"),
`version`, `implementation`, the QMS certification
reference, and the `profile` declaration that
selects which of the optional records (FHIR
mapping, software update, biocompatibility) the
implementation supports. The manifest is signed
using a key whose public part is published on the
operator's `.well-known/wia/electronic-skin/`
discovery endpoint declared in PHASE-2.
