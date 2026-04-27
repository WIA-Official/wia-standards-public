# WIA-MED-021: Sleep Monitoring Standard

**Status**: ✅ Complete
**Version**: 1.0.0
**Philosophy**: 弘益人間 — Benefit All Humanity

---

## Overview

WIA-MED-021 defines an open standard for consumer and clinical sleep
monitoring devices — fitness wearables tracking sleep stages,
clinical polysomnography (PSG) units in sleep labs, home sleep
apnoea testing devices, and continuous wrist-worn sleep trackers —
so that data from any conformant device flows portably to any
conformant clinical dashboard, EHR system, or research platform.

---

## Why this standard exists

* **Sleep medicine is fragmented.** Clinical sleep labs use vendor PSG
  hardware speaking proprietary protocols; consumer wearables use a
  different set of vendor-locked clouds; home sleep apnoea tests sit
  in a third silo. A patient with a clinical sleep lab study and a
  consumer wearable cannot easily reconcile the two.
* **Sleep stage scoring should be reproducible.** AASM (American
  Academy of Sleep Medicine) sleep-stage scoring rules are
  well-defined, but vendor implementations vary. The standard's
  Phase 1 envelope explicitly carries the scoring algorithm version
  and the AASM compliance level so downstream consumers can compare
  apples to apples.
* **Clinical-grade vs consumer-grade clarity.** A consumer fitness
  tracker estimating REM minutes is not equivalent to a clinical
  PSG measuring REM via EEG. The standard's `clinical_grade` flag
  and per-channel signal-quality metrics make the distinction
  explicit so clinical decisions are made on appropriate evidence.

---

## 4-Phase architecture

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Session, sleep stage, respiratory event, arousal, scoring metadata |
| 2 — API Interface | HTTP surface for session ingest, query, stream, scoring re-computation |
| 3 — Federation Protocol | Cross-device handshake, replay defence, patient consent |
| 4 — Integration | HL7 FHIR R5, AASM scoring, EDF / EDF+ raw recordings, WIA family |

---

## Quick start

```bash
docker run -p 8080:8080 wia/sleep-monitoring-host:1.0.0

curl -X POST "http://localhost:8080/sm/session" \
     -H "Content-Type: application/json" \
     -d @session.json

curl "http://localhost:8080/sm/session/{session_id}/scoring"
```

---

## CLI

A reference CLI ships under `cli/sleep-monitoring.sh` with
subcommands `validate`, `session`, `event`, `score`, `consent`, `info`.

---

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 session ingest + query |
| Core | Plus Phase 3 federation, FHIR R5 export bridge |
| Full | Plus AASM-compliant scoring, EDF+ raw recording bridge, HealthKit / Google Fit bidirectional |

---

## Companion standards

* **AASM Manual for the Scoring of Sleep and Associated Events** (current edition)
* **EDF / EDF+** — European Data Format for raw biosignal recordings
* **HL7 FHIR R5** — Observation, Patient, Procedure resources
* **WIA Wearable Health (WIA-MED-020)** — sister standard for general wearable monitoring
* **WIA Vital Sign Streaming** — high-rate continuous waveform companion
* **WIA-OMNI-API** — credential storage for patient identity

---

MIT License — © 2025 WIA (World Certification Industry Association)

弘益人間 — Benefit All Humanity.
