# WIA-MED-003 Phase 1: Signal Format Standard

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

## Overview

Phase 1 defines the standardized data format for all vital sign measurements in the WIA-MED-003 ecosystem.

## Base Signal Structure

All vital sign data must include these common fields:

```json
{
  "standard": "WIA-MED-003",
  "version": "1.0.0",
  "signalType": "ECG|SPO2|BLOOD_PRESSURE|HRV|TEMPERATURE|RESPIRATORY_RATE",
  "deviceId": "string",
  "timestamp": "ISO 8601 string",
  "data": {}
}
```

## Signal Types

### ECG Signal
- Sampling rate: 250-1000 Hz
- Unit: mV (millivolts)
- Leads: 1, 3, 5, or 12-lead configurations

### SpO2 Signal
- Range: 0-100%
- Pulse rate: 30-250 bpm
- PPG waveform: Red and infrared channels

### Blood Pressure
- Unit: mmHg
- Components: Systolic, Diastolic, Mean Arterial Pressure
- Methods: Auscultatory, Oscillometric, Continuous

### HRV (Heart Rate Variability)
- Time domain: SDNN, RMSSD, pNN50
- Frequency domain: VLF, LF, HF, LF/HF ratio
- Non-linear: Poincaré plot metrics

### Temperature
- Unit: Celsius or Fahrenheit
- Sites: Oral, rectal, axillary, tympanic, forehead
- Range: 30-45°C

### Respiratory Rate
- Unit: breaths/minute
- Range: 8-30 breaths/min
- Pattern: Regular, irregular, labored

## Data Compression

Supported compression methods:
1. Delta encoding
2. Run-length encoding
3. Gzip
4. Downsampling with anti-aliasing

---

## Frame Envelope (canonical)

Phase 1 fixes a single canonical frame envelope that wraps any vital
signal type. The envelope is the unit of streaming, signing, and
replay-defence.

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "frame",
  "stream_id": "stream-001",
  "patient_id": "did:wia:patient:01HXY",
  "channel": "ecg",
  "sample_rate_hz": 250,
  "captured_at": "2026-04-27T10:00:00.000Z",
  "samples": [0.012, 0.014, 0.013],
  "encoding": "float32",
  "calibration": { "lsb": 0.000244, "offset": 0.0, "unit": "mV" },
  "lead_id": "II",
  "frame_seq": 4017,
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

### Required fields

| Field | Type | Notes |
|-------|------|-------|
| `wia_vital_sign_streaming_version` | string | Spec version (semver). Receivers MUST refuse a major version they do not implement |
| `type` | string | MUST be `frame` |
| `stream_id` | string | The producer's stream identifier |
| `patient_id` | URI | DID-shaped to avoid carrying directly identifying numbers (RFC 3986) |
| `channel` | enum | One of the registered channels (§ Signal Types above) |
| `sample_rate_hz` | integer | Strictly positive; the producer's emit rate |
| `captured_at` | RFC 3339 timestamp | UTC, millisecond precision required for ECG, second precision sufficient for vitals like temperature |
| `samples` | array of number | Encoded per `encoding` |
| `encoding` | enum | `int16`, `int24`, `int32`, `float32` |
| `signature` | object | Detached Ed25519 signature over the canonical encoding (Appendix A) |

### Optional fields

| Field | Default | Notes |
|-------|---------|-------|
| `calibration` | none | If present, MUST include `unit`. The receiver applies `lsb` and `offset` for raw integer encodings |
| `lead_id` | none | For multi-lead ECG, the lead identifier (`I`, `II`, `III`, `aVR`, `aVL`, `aVF`, `V1`–`V6`) |
| `frame_seq` | none | Monotonic per-stream counter, used for gap detection |
| `qos` | `best_effort` | `best_effort`, `assured`, `lossless` |
| `compression` | none | Algorithm used in `samples`; one of `none`, `delta`, `rle`, `gzip` |

### Encoding rules

* UTF-8 JSON per IETF RFC 8259, `snake_case` keys.
* Timestamps RFC 3339 in UTC with `Z` suffix; ECG and other ≥ 100 Hz
  channels MUST encode millisecond precision.
* Numbers fit in IEEE 754 double precision; integer fields fit in
  signed 64-bit.
* Binary fields base64url without padding per IETF RFC 4648 §5.
* The frame MUST be self-describing: a receiver presented with one
  envelope MUST be able to reconstruct calibrated values without
  out-of-band metadata.

## Channel Descriptor

For high-rate streams the producer MAY publish a one-shot channel
descriptor that the receiver caches; subsequent frames omit the
descriptor's fields and inherit them.

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "channel_descriptor",
  "stream_id": "stream-001",
  "channel": "ecg",
  "default_calibration": { "lsb": 0.000244, "offset": 0.0, "unit": "mV" },
  "default_encoding": "int16",
  "default_lead_id": "II",
  "issued_at": "2026-04-27T10:00:00Z",
  "valid_until": "2026-04-27T22:00:00Z",
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

Receivers MUST honour the descriptor only for the `(stream_id,
channel)` pair it covers and MUST refuse frames received outside the
descriptor's `valid_until` window.

## Reserved channels

| Channel | Typical rate | Unit | Notes |
|---------|--------------|------|-------|
| `ecg`   | 250–1 000 Hz | mV   | `lead_id` recommended |
| `ppg`   | 50–250 Hz   | a.u. | infrared and red channels MAY ride as separate streams |
| `eeg`   | 250–1 000 Hz | µV   | `lead_id` per 10–20 system |
| `emg`   | 1 000 Hz    | µV   | bandpass filter range carried in `calibration.unit` extension |
| `spo2`  | 1 Hz        | %    | range 0–100 |
| `hr`    | 1 Hz        | bpm  | range 0–300 |
| `sbp`   | 0.05 Hz (≈ every 20 s) | mmHg | continuous variants emit at 1 Hz |
| `dbp`   | 0.05 Hz | mmHg | as above |
| `map`   | 0.05 Hz | mmHg | mean arterial pressure |
| `temp`  | 0.0167 Hz (≈ minutely) | °C | range 30–45 |
| `rr`    | 0.0167 Hz | breaths/min | range 4–60 |
| `etco2` | 1 Hz | mmHg | end-tidal CO2 |

Implementations MUST treat unknown channels as `unrecognised` rather
than fail the frame. Future minor versions add channels but never
remove them.

## Conformance

A Phase 1 conformant implementation MUST:

1. Emit and accept the frame envelope above with all required fields.
2. Honour the channel descriptor inheritance rules.
3. Round-trip every frame byte-identically through encode/decode when
   no normalisation is requested.
4. Reject frames that violate the JSON Schema (Schema files are served
   from `https://wiastandards.com/vital-sign-streaming/schemas/`).
5. Treat unknown optional fields as non-fatal extensions.

## Appendix A — Canonicalisation for Signatures

When a frame's signature must be re-verified, implementations MUST:

1. Sort object keys lexicographically by Unicode code point.
2. Use compact JSON form (no insignificant whitespace).
3. Render numbers using shortest round-trip form for floats; integers
   without leading zeros.
4. Encode arrays preserving authored order.
5. UTF-8 encode the result with no BOM.
6. Hash with SHA-256 prefixed by the literal ASCII string
   `WIA-VITAL-SIGN-STREAMING/1.0\n` for domain separation.

## References

* IETF RFC 8259 — JSON
* IETF RFC 3339 — Date/Time
* IETF RFC 3986 — URI Generic Syntax
* IETF RFC 4648 — base64
* IETF RFC 8032 — EdDSA / Ed25519
* IEEE 11073 — Point-of-care medical device communication
* HL7 FHIR R5 — Observation resource
* W3C DID 1.0
* JSON Schema Draft 2020-12

---

## Appendix B — Worked Frame (12-lead ECG @ 500 Hz)

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "frame",
  "stream_id": "stream-icu-A-bed-7-ecg",
  "patient_id": "did:wia:patient:01HXY",
  "channel": "ecg",
  "lead_id": "II",
  "sample_rate_hz": 500,
  "captured_at": "2026-04-27T10:00:00.000Z",
  "encoding": "int16",
  "calibration": { "lsb": 0.000244, "offset": 0, "unit": "mV" },
  "frame_seq": 12483017,
  "qos": "assured",
  "compression": "delta",
  "samples": [3, 4, 5, 4, 3, 2, 1, 0, -1, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8],
  "channel_descriptor_ref": {
    "issued_at": "2026-04-27T08:00:00Z",
    "valid_until": "2026-04-27T20:00:00Z"
  },
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

For multi-lead capture, each lead is a separate `(stream_id, channel,
lead_id)` triple. Receivers MUST NOT assume that frames from different
leads share a clock; each lead carries its own `captured_at`.

## Appendix C — Worked Frame (continuous BP @ 1 Hz)

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "frame",
  "stream_id": "stream-icu-A-bed-7-bp",
  "patient_id": "did:wia:patient:01HXY",
  "channel": "sbp",
  "sample_rate_hz": 1,
  "captured_at": "2026-04-27T10:00:00.000Z",
  "samples": [128],
  "encoding": "int16",
  "calibration": { "lsb": 1.0, "offset": 0, "unit": "mmHg" },
  "frame_seq": 47821,
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

Continuous BP traces typically also emit `dbp` and `map` channels;
implementations MAY bundle the three into a single `composite` frame
when the device exposes them at the same sample rate, with each
channel as a sub-record:

```json
{
  "type": "composite_frame",
  "stream_id": "stream-icu-A-bed-7-bp",
  "captured_at": "2026-04-27T10:00:00.000Z",
  "channels": {
    "sbp": { "samples": [128], "unit": "mmHg" },
    "dbp": { "samples": [78],  "unit": "mmHg" },
    "map": { "samples": [95],  "unit": "mmHg" }
  },
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

## Appendix D — Reserved Tokens

| Field | Reserved tokens |
|-------|-----------------|
| `type` | `frame`, `composite_frame`, `channel_descriptor` |
| `encoding` | `int16`, `int24`, `int32`, `float32` |
| `qos` | `best_effort`, `assured`, `lossless` |
| `compression` | `none`, `delta`, `rle`, `gzip` |
| `lead_id` (ECG) | `I`, `II`, `III`, `aVR`, `aVL`, `aVF`, `V1`–`V6` |

Future minor versions add tokens but never remove them.

## Appendix E — Calibration Cross-Walk

Devices commonly publish raw integer ADC samples plus calibration
metadata; receivers reconstruct calibrated physical values per:

```
calibrated = (raw + offset) * lsb
```

Examples:

| Channel | Encoding | LSB | Offset | Unit |
|---------|----------|-----|--------|------|
| ECG II  | int16    | 0.000244 | 0 | mV |
| EEG Cz  | int24    | 0.0000003 | 0 | µV |
| SpO2    | int16    | 0.1 | 0 | % |
| Temp tympanic | int16 | 0.01 | 0 | °C |

When a device publishes already-calibrated values (`encoding =
float32` and the samples are real-world units), the calibration
object MAY be omitted but the `unit` MUST appear inline as
`samples_unit` to keep the frame self-describing.

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "frame",
  "stream_id": "stream-001",
  "patient_id": "did:wia:patient:01HXY",
  "channel": "spo2",
  "sample_rate_hz": 1,
  "captured_at": "2026-04-27T10:00:00Z",
  "encoding": "float32",
  "samples": [97.5],
  "samples_unit": "%",
  "signature": { "alg": "Ed25519", "value": "Vr3w…" }
}
```

## Appendix F — Frame Loss and Gap Marking

Producers MUST monotonically increment `frame_seq` per stream.
Receivers detect gaps when `frame_seq` advances by more than 1.
The receiver MUST emit a local `gap_marker` event for any gap and
SHOULD attempt a Phase 2 replay request for the missed window when
the QoS is `assured` or `lossless`.

```json
{
  "type": "gap_marker",
  "stream_id": "stream-001",
  "from_seq": 12483018,
  "to_seq":   12483077,
  "detected_at": "2026-04-27T10:00:01Z"
}
```

Implementations MUST NOT silently fill gaps with zeros; doing so would
present a clinical reader with an apparently flat ECG that is in fact
missing data, which is a patient-safety hazard.

## Appendix G — Multi-Channel Stream Bundling

Many bedside monitors emit multiple channels at different rates from a
single device. The standard supports two bundling modes:

### G.1 Independent streams (recommended for high-rate channels)

Each channel becomes its own `(stream_id, channel)` pair. Receivers
subscribe per channel and clock-align via `captured_at`. This mode is
RECOMMENDED for ECG, EEG, EMG, and PPG.

### G.2 Composite frames (for low-rate vitals captured together)

When a monitor captures SBP, DBP, MAP, HR, SpO2, and temperature at
synchronised low rates, a composite frame avoids per-channel overhead:

```json
{
  "wia_vital_sign_streaming_version": "1.0.0",
  "type": "composite_frame",
  "stream_id": "stream-icu-A-bed-7-vitals",
  "patient_id": "did:wia:patient:01HXY",
  "captured_at": "2026-04-27T10:00:00Z",
  "frame_seq": 39482,
  "channels": {
    "hr":   { "samples": [78],  "unit": "bpm"  },
    "sbp":  { "samples": [128], "unit": "mmHg" },
    "dbp":  { "samples": [78],  "unit": "mmHg" },
    "map":  { "samples": [95],  "unit": "mmHg" },
    "spo2": { "samples": [97],  "unit": "%"    },
    "temp": { "samples": [36.7],"unit": "°C"   }
  },
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

Receivers MUST treat composite frames as if each `channels[k]`
sub-record were an independent frame for the purposes of alert
classification, FHIR mapping, and audit logging.

## Appendix H — Time Synchronisation

* Devices SHOULD synchronise their clock via NTS (IETF RFC 8915) or
  a clinical-grade time source. For real-time alerting, clock skew
  > ±1 second between producer and consumer is a clinical-safety
  hazard and SHOULD trigger an operational alert (not a clinical
  alert) to the on-call biomedical engineer.
* For replay correlation across multiple devices on the same patient,
  the receiver MUST clock-align via `captured_at` rather than
  arrival time. Devices that cannot guarantee monotonic timestamps
  MUST emit `clock_uncertainty_ms` in their channel descriptor.

---

**Copyright 2025 WIA / SmileStory Inc.**
**License:** MIT
**弘益人間 · Benefit All Humanity**
