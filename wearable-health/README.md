# WIA-MED-020: Wearable Health Monitoring Standard

**Status:** ✅ Complete
**Version:** 1.0.0
**Philosophy:** 弘益人間 — Benefit All Humanity

---

## Overview

WIA-MED-020 defines an open standard for wearable health devices —
fitness trackers, smartwatches, continuous glucose monitors, ECG
patches, blood-pressure cuffs, and continuous monitoring devices —
so that data from any conformant device can flow to any conformant
clinician dashboard, EHR system, or research platform without
vendor-specific integration.

---

## Why this standard exists

* **Vendor lock-in is the dominant pattern today.** A patient owning
  an Apple Watch, a Fitbit, and a Withings scale cannot present a
  single data view to their physician without manual export-import
  cycles. The standard collapses the three vendor APIs into one
  conformant wire format.
* **Clinical-grade quality flag is missing in most consumer formats.**
  The standard's Phase 1 envelope carries an explicit
  `clinical_grade` boolean and a calibration provenance chain, so
  that a clinician can distinguish FDA-cleared continuous glucose
  monitor data from a fitness-tracker's optical heart-rate estimate.
* **Federated identity for the patient.** The patient's WIA-OMNI-API
  identity is the binding identifier; vendor account IDs become
  internal mappings rather than exposed integration burden.

---

## 4-Phase architecture

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Device, measurement, session, alert, calibration envelopes |
| 2 — API Interface | HTTP surface for device sync, measurement query, alert dispatch |
| 3 — Federation Protocol | Cross-device handshake, replay defence, patient consent |
| 4 — Integration | HL7 FHIR R5, IEEE 11073, Apple HealthKit, Google Fit, WIA family |

---

## Quick start

```bash
docker run -p 8080:8080 wia/wearable-health-host:1.0.0

# Subscribe to a device's measurement stream
curl -N "http://localhost:8080/wh/measurement/stream/dev-001" \
     -H "Accept: text/event-stream"

# Submit a device sync packet
curl -X POST "http://localhost:8080/wh/sync" \
     -H "Content-Type: application/json" \
     -d @sync.json
```

---

## CLI

A reference CLI ships under `cli/wearable-health.sh` with subcommands
`validate`, `device`, `measurement`, `alert`, `consent`, `info`.

---

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 device sync + measurement query |
| Core | Plus Phase 3 federation, FHIR R5 export bridge |
| Full | Plus HealthKit / Google Fit bidirectional, WIA-AIR-SHIELD scoring, WIA-ACCESSIBILITY profiles |

---

## Companion standards

* **HL7 FHIR R5** — Observation, Patient, Consent resources
* **IEEE 11073** — Personal health device communication family
* **Apple HealthKit** — iOS health data backend
* **Google Fit** — Android health data backend
* **WIA-OMNI-API** — credential storage for patient identity
* **WIA-ACCESSIBILITY** — accommodations for clinician and patient interfaces
* **WIA Vital Sign Streaming** — high-rate continuous waveform companion standard

---

**홍익인간 (弘益人間) - Benefit All Humanity**

© 2025 WIA | MIT License
