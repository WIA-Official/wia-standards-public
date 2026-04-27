# WIA-medical-iot PHASE 1 — Data Format Specification

**Standard:** WIA-medical-iot
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for medical IoT
(MedIoT) deployments: connected patient-monitoring devices,
ambulatory and in-hospital biosensors, infusion pumps, ventilators,
imaging gateways, and the personal-health devices that feed clinical
records. The shape interoperates with HL7 FHIR R5 Device,
DeviceMetric, and Observation resources, and with IEEE 11073-10101
nomenclature, so that an existing IHE Patient Care Device (PCD)
deployment adopts this PHASE without parallel data models.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Device (R5/device.html), DeviceMetric (R5/devicemetric.html),
  Observation (R5/observation.html), DeviceAssociation, DeviceUsage
- ISO/IEEE 11073-10101:2020 — Health informatics — Point-of-care medical
  device communication — Nomenclature
- IEEE 11073-10206:2021 — Personal health device communication, abstract
  content information model
- IEEE 11073-20601:2019 — Personal health device communication,
  application profile (optimised exchange protocol)
- ISO 13485:2016 — Medical devices — Quality management systems
- ISO 14971:2019 — Medical devices — Application of risk management
- IEC 62304:2006/A1:2015 — Medical device software — Software life cycle
- IEC 60601-1:2005/A2:2020 — Medical electrical equipment — General
  requirements for basic safety and essential performance
- IEC 60601-1-8:2020 — Alarm systems
- IEC 80001-1:2021 — Application of risk management for IT-networks
  incorporating medical devices
- IHE Patient Care Device (PCD) Technical Framework
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339 (timestamps)

---

## §1 Scope

This PHASE applies to systems that collect, normalise, or relay
data from medical devices subject to medical-device regulation
(US FDA 21 CFR §820, EU MDR 2017/745, K-MFDS Medical Devices Act,
Japan PMDA, Brazil ANVISA). It standardises the *shape* of device
records, observation streams, alarm conditions, and the cross-
references that bind them to a clinical record. Wireless-link
behaviour, encryption-key generation, and transport are addressed
in PHASE 3 (Protocol).

Devices are classified by IEC 62304 software-safety class
(Class A / B / C) and by their regulatory class (Class I / II /
III for FDA; Class I / IIa / IIb / III for MDR). The classification
travels with every record so downstream systems apply the correct
risk handling.

## §2 Device identity

Every connected device carries a structured identifier that maps
to FHIR Device:

- `deviceRef` — URN of form `urn:wia:miot:device:<udi-di>:<udi-pi>`
  where UDI-DI is the Device Identifier portion of the Unique Device
  Identification per FDA 21 CFR §830 / EU MDR Article 27, and UDI-PI
  is the Production Identifier (lot, serial, expiry, manufacture date)
- `manufacturerRef` — GS1 GLN of the manufacturer, mirrored to
  FHIR Device.manufacturer
- `modelRef` — UDI-DI alone, for cross-device-instance aggregation
- `softwareRef` — IEC 62304 software identifier (versionString +
  build hash) for the firmware/embedded-software version
- `udiCarrier` — the carrier that transmitted the UDI to the
  boundary (HRF text, AIDC barcode, RFID tag, GS1-128, GS1 DataMatrix)

A record without a recognised UDI is rejected at the edge with
`urn:wia:miot:problem:udi-required`. Unknown UDIs are queued for
the device-registry sync described in PHASE 4 §2 rather than
auto-admitted.

## §3 Observation envelope (FHIR + IEEE 11073)

Every measurement emits a FHIR Observation profiled to
IEEE 11073-10101:

| FHIR field         | Binding                                                          |
|--------------------|------------------------------------------------------------------|
| `status`           | one of registered / preliminary / final / amended                |
| `category`         | from FHIR ObservationCategoryCodes (vital-signs, laboratory, …)  |
| `code`             | a 11073-10101 nomenclature code in `system: urn:iso:std:iso:11073:10101` |
| `subject`          | Reference to the pseudonymous patient (cross to medical-data-privacy) |
| `effectiveDateTime`| RFC 3339 with offset; device-clock plus authoritative skew (PHASE 3 §6) |
| `device`           | Reference to the originating Device                              |
| `valueQuantity`    | `value` + `unit` + `system: http://unitsofmeasure.org` (UCUM)    |
| `interpretation`   | one of normal / high / low / critical-high / critical-low (subset of v3-ObservationInterpretation) |
| `referenceRange`   | per the device's calibration sheet                               |

Numeric values are accompanied by an explicit measurement uncertainty
band (per ISO/IEC Guide 98-3 GUM principles); a value without
uncertainty is permitted only when the device's calibration sheet
declares the metric as out-of-scope for uncertainty reporting.

## §4 Vital-sign metric set

The closed set of metrics this PHASE recognises uses IEEE 11073-10101
codes (selection):

- MDC_PULS_OXIM_SAT_O2 (150456) — peripheral capillary oxygen saturation, %
- MDC_PRESS_BLD_NONINV_SYS / DIA / MEAN — non-invasive blood pressure, mmHg
- MDC_TEMP_BODY (188424) — body temperature, °C
- MDC_RESP_RATE (151562) — respiratory rate, breaths/min
- MDC_PULS_RATE (147842) — pulse rate, /min
- MDC_ECG_HEART_RATE (147842) — ECG-derived heart rate, /min
- MDC_FLOW_AWAY_INSP (151612) — inspired airflow, L/min
- MDC_GLU_CAP_WB (160368) — capillary whole-blood glucose, mg/dL or mmol/L
- MDC_MASS_BODY (188736) — body mass, kg
- MDC_BODY_FAT (57664) — body-fat percentage, %

Additional metrics MAY be carried with vendor-extension codes from
the IEEE 11073-10101 partition reserved for extensions (range
524288–589823). Boundary publishes the accepted vendor-extension
list in the capability document.

## §5 Alarm condition record

When a device asserts an alarm condition (per IEC 60601-1-8), the
boundary captures:

- `alarmConditionId` — URN
- `deviceRef` — emitting device
- `subject` — patient under monitoring
- `alarmKind` — high / medium / low priority (IEC 60601-1-8 §6.1.2)
- `alarmCategory` — physiological / technical
- `triggeringObservationRef` — the Observation that crossed threshold
- `assertedAt`, `acknowledgedAt`, `resolvedAt` — RFC 3339 timestamps
- `acknowledgedBy` — principal URN of the clinician acknowledging
- `escalation[]` — escalation path (caregiver→nurse→rapid-response)
  with timestamps; per IEC 60601-1-8 the priority escalates if
  unacknowledged within the configured tau

Alarms are append-only; subsequent state transitions are added as
new entries that reference the alarm-condition ID rather than
overwriting.

## §6 Calibration record

Each device carries a calibration record updated on every successful
calibration:

- `calibrationId` — URN
- `deviceRef` — calibrated device
- `metricCode` — 11073 code being calibrated
- `referenceMaterial` — traceable reference (NIST SRM, BIPM
  primary-realisation reference, manufacturer-traceable secondary)
- `result` — declared correction factor + residual uncertainty
- `performedAt`, `nextDueAt` — RFC 3339
- `performedBy` — biomedical-engineering principal URN
- `evidenceRef` — URI of the signed calibration certificate

Devices with `nextDueAt < now` enter a degraded mode: observations
are flagged `interpretation: "calibration-overdue"` until calibration
is performed.

## §7 Connectivity binding

The connectivity binding describes how the device is reaching the
boundary:

- `connectivityKind` — one of {`wifi-802.11`, `bluetooth-le`,
  `bluetooth-classic`, `cellular-3gpp`, `lorawan`, `zigbee`,
  `wired-ethernet`, `usb-classic`, `nfc`, `proprietary`}
- `effectiveBitRate` — declared link bit rate, bps
- `linkQualityRsrp` — for cellular; dBm
- `linkQualityRssi` — for Wi-Fi / BLE; dBm
- `gatewayRef` — URN of the IoT gateway that aggregates, if any
- `dnsResolverRef` — internal-DNS-resolver URN
- `lastHandshakeAt` — RFC 3339 of the most recent successful TLS
  handshake (PHASE 3 §1)

Devices that lack any active connectivity binding for longer than
the deployment-declared offline-tolerance window emit a
`device-offline` alarm condition.

## §8 Patient-association record

A device is associated with a patient via FHIR DeviceAssociation:

- `associationId` — URN
- `deviceRef` — instrumented device
- `subject` — pseudonymous patient (cross-domain reference per §10)
- `period.start` / `period.end` — RFC 3339
- `bodysite` — SNOMED CT bodysite code where the device is applied
- `operator` — the clinical role authorising the association
  (attending, RN, RRT, patient self-care)

Disassociation closes the period and triggers a check that the
device's data buffers contain no residual patient-identified data
before the device is reissued. The check emits an audit event so
auditors can confirm hygiene.

## §9 Cross-domain references

| Reference                  | Use site                                                  |
|----------------------------|-----------------------------------------------------------|
| WIA-medical-data-privacy   | every observation references the consent record for purpose-of-use |
| WIA-medical-imaging        | imaging-gateway devices cross-reference imaging study UID |
| WIA-network-security       | TLS cipher-suite floor and certificate trust roots        |
| WIA-pq-crypto              | post-quantum migration phase that governs key rotation    |

The boundary verifies the cross-domain reference exists at the
referenced standard's boundary before delivery, so downstream
readers do not see dangling references.

## §10 Subject identifier scope

Subject identifiers in this standard are *pseudonymous* and follow
the WIA-medical-data-privacy `subjectRef` shape (URN of form
`urn:wia:mdp:subject:<opaque>`). The MedIoT boundary holds no direct
identifiers and does not log them in audit chains; full PHI lives
in the medical-data-privacy boundary, which mediates re-identification.

## §11 Versioning and deprecation

Versioning follows Semantic Versioning 2.0.0. Major bumps require
≥ 90 days overlap with the prior major version on every fielded
reference implementation. Deprecation enters a 12-month sunset
window with migration notes; deprecated metric codes remain
verifiable in historical observations even after the verification
cutoff date so audit-chain reconstruction is possible.

## §12 Conformance levels

| Level     | Scope                                                  |
|-----------|--------------------------------------------------------|
| Surface   | data formats accepted; self-attested                   |
| Verified  | annual third-party audit including IEC 62304 review    |
| Anchored  | continuous evidence package + IEC 80001-1 risk file    |

Implementations declare their level in the capability advertisement.
Deployments that span multiple safety classes inherit the highest
class's verification cadence.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked observation example (informative)

```json
{
  "resourceType": "Observation",
  "status": "final",
  "category": [{"coding": [{"system": "http://terminology.hl7.org/CodeSystem/observation-category", "code": "vital-signs"}]}],
  "code": {"coding": [{"system": "urn:iso:std:iso:11073:10101", "code": "150456", "display": "MDC_PULS_OXIM_SAT_O2"}]},
  "subject": {"reference": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e"},
  "effectiveDateTime": "2026-04-28T03:14:00+09:00",
  "device": {"reference": "urn:wia:miot:device:M-PULSE-1.2.3:SN-91A7"},
  "valueQuantity": {"value": 97, "unit": "%", "system": "http://unitsofmeasure.org", "code": "%"},
  "interpretation": [{"coding": [{"system": "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation", "code": "N", "display": "Normal"}]}],
  "referenceRange": [{"low": {"value": 95}, "high": {"value": 100}}]
}
```

## Annex B — Negative test vectors (informative)

| Stimulus                                      | Expected outcome                            |
|-----------------------------------------------|---------------------------------------------|
| Observation without UDI                       | 422 + `udi-required`                        |
| Observation with calibration-overdue device   | accepted, flagged `calibration-overdue`     |
| Alarm without IEC 60601-1-8 priority          | 422 + `alarm-priority-required`             |
| Patient association without `period.start`    | 422 + `association-period-required`         |
| Vendor-extension code outside reserved range  | 422 + `metric-code-out-of-range`            |
