# WIA-MED-BE-001 · Bionic Eye — Phase 1: Data Format

**Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 1 defines the **end-to-end data schema** of a visual prosthesis: the captured camera frame, the processed visual scene, the phosphene map that maps the scene onto the retinal or cortical grid, and the electrical stimulation descriptor that is ultimately written to the electrode array. Every other phase depends on these contracts.

### 1.1 Scope and audience

- Vendors of retinal (epiretinal / subretinal / suprachoroidal) and cortical implants.
- Low-vision rehabilitation teams who configure fittings.
- Regulators (FDA 21 CFR 882.5360 neurological stimulator; EU IVDR / MDR class III).

### 1.2 Normative references

- ISO 14708-1:2014 — Implants for surgery; active implantable medical devices.
- IEC 60601-1 / 60601-2-10 — Medical electrical equipment and stimulators.
- IEC 62366-1 — Usability engineering for medical devices.
- ISO 10993-1 — Biological evaluation of medical devices.
- DICOM PS3.3 — IOD: Ophthalmic Visual Field Static Perimetry.
- Shannon (1992) — Charge-density/charge-per-phase safety limit (`k ≤ 1.85`).

---

## 2. Top-level envelope

A single *visual cycle* (camera frame → electrode command) MUST carry these four sections:

```jsonc
{
  "cycle_id":        "01JCG0T0A2V5M8W7P3Q9S1R6N4",
  "patient_id":      "WIA-BE-KR-Seoul-PID-0042",    // hashed, never PHI in the clear
  "device_id":       "WIA-BE-SN-0001",
  "timestamp":       "2026-04-25T14:30:00.128Z",
  "captured_frame":  { /* § 3 */ },
  "processed_visual":{ /* § 4 */ },
  "phosphene_map":   { /* § 5 */ },
  "stimulation":     { /* § 6 */ }
}
```

Each cycle is immutable; correlation with later clinical records goes through `cycle_id`.

---

## 3. Captured frame

### 3.1 Metadata

| Field           | Type     | Notes                                  |
|-----------------|----------|----------------------------------------|
| `frame_id`      | string   | UUIDv7 (monotonic)                     |
| `sequence`      | uint32   | Monotonic per session                  |
| `device_id`     | string   | Camera serial (distinct from implant)  |
| `timestamp`     | RFC 3339 | ±2 ms of implant clock                 |

### 3.2 Image data

```ts
interface ImageData {
  width:      uint;            // 320–1920
  height:     uint;            // 240–1080
  format:     "gray8" | "gray16" | "rgb24" | "rgbd32" | "depth16" | "ir8";
  encoding:   "raw" | "jpeg" | "h264" | "h265";
  data:       bytes;           // encoded bitstream or raw plane
  stride?:    uint;            // bytes per row (raw only)
  color_space?: "sRGB" | "Rec.709" | "BT.2020";
  bit_depth:  uint;            // 8, 10, 12, or 16
}
```

Cameras MUST expose `gray16` at ≥ 60 fps for the safety loop; lower-rate RGBD may be used for high-level scene analysis.

### 3.3 Camera parameters

```ts
interface CameraParams {
  fov: { horizontal_deg: 60..120, vertical_deg: 45..90 };
  exposure_ms: 0.1..100;
  gain_iso:     100..6400;
  white_balance_k: 2500..10000;
  focus_distance_m: number | "auto";
  aperture_fstop?: number;
  stabilisation: "off" | "optical" | "digital" | "hybrid";
}
```

### 3.4 Sensor fusion

IMU + ambient-light sensors are mandatory for gaze-contingent and lux-adaptive stimulation:

```ts
interface SensorData {
  ambient_lux: 0..100000;
  proximity_cm: 0..500;
  imu: {
    accel_ms2:   [x, y, z];
    gyro_rads:   [x, y, z];
    mag_ut:      [x, y, z];
    quaternion:  [w, x, y, z];
  };
  timestamp: RFC3339;
}
```

### 3.5 Minimum frame-rate targets

| Use case             | Min fps | Target fps | End-to-end latency |
|----------------------|---------|------------|--------------------|
| Obstacle / stairs    |   10    |   15–20    |   ≤ 100 ms         |
| Object detection     |    5    |   10       |   ≤ 200 ms         |
| Face recognition     |   10    |   15       |   ≤ 150 ms         |
| Reading / OCR        |    1    |    5       |   ≤ 500 ms         |

---

## 4. Processed visual

### 4.1 Edge map

```ts
interface EdgeMap {
  data: bytes;
  algorithm: "canny" | "sobel" | "laplacian" | "structured-forests";
  params: { low_threshold?: 0..255, high_threshold?: 0..255, kernel: 3|5|7 };
  processing_ms: number;
}
```

### 4.2 Detected objects

```ts
interface DetectedObject {
  object_id:   string;
  label:       string;               // COCO class or domain vocab
  label_i18n:  { [bcp47]: string };
  confidence:  0..1;
  bbox:        { x: 0..1, y: 0..1, w: 0..1, h: 0..1 };  // normalised
  distance_m:  number;
  priority:    "critical" | "high" | "medium" | "low";
  threat:      boolean;
  moving:      boolean;
  velocity_ms?: { x: number, y: number };
}
```

Priority ordering drives stimulation intensity scaling (§ 6.3):

| Priority | Examples                                   |
|----------|--------------------------------------------|
| critical | vehicles, bicycles, descending stairs, pit |
| high     | people, faces, doors, traffic signals      |
| medium   | furniture, walls, non-moving objects       |
| low      | background, decoration                     |

### 4.3 Faces, OCR, motion, depth

The nested schemas for `DetectedFace`, `TextRegion`, `MotionData`, and `DepthData` are normative but omitted here for brevity; implementations MUST expose all six feature channels and MUST fail gracefully (`null`) when a channel is not available.

### 4.4 Saliency and attention

A saliency map (`float32[H×W] ∈ [0,1]`) MUST be included whenever gaze-contingent stimulation is enabled, since attention-biased phosphene allocation is the primary mechanism by which patients report higher object-recognition accuracy under fixed electrode budgets.

---

## 5. Phosphene map

The phosphene map translates *processed visual* into the patient's perceived percept space. It MUST be re-calibrated at every fitting session and stored with a `calibration_revision`.

### 5.1 Electrode-to-phosphene descriptor

```ts
interface PhospheneLocus {
  electrode_index: uint;
  retinal_xy_mm?:  [x, y];    // for retinal implants (epi/subretinal)
  cortical_xy_mm?: [x, y];    // for cortical implants (e.g., Orion, PRIMA)
  visual_field_deg: { azimuth: number, elevation: number };
  perceived_size_deg: number;
  perceived_colour: "white" | "yellow" | "blue" | "none";
  flicker_hz?:     number;    // percept flicker when stimulated ≥ threshold
  threshold_uA:    number;    // 50 % perception (psychophysical staircase)
  jnd_slope:       number;    // d(perceived brightness)/d(current)
}
```

### 5.2 Retinotopic and cortico-topic mapping

- Retinal: `visual_field_deg` derived from OCT-A fundus alignment, anchored at the fovea.
- Cortical: a retinotopic atlas applied to the patient's MRI; V1 surface coordinates are *patient-specific*.

### 5.3 Percept stability

Percepts drift between sessions; Phase 1 mandates a *percept drift budget*: ≤ 1 electrode out of 60 reclassified between adjacent sessions; ≥ 3 triggers a recalibration requirement (see Phase 2 § 5).

---

## 6. Stimulation descriptor

### 6.1 Waveform

```ts
interface StimulationWaveform {
  type: "biphasic_symmetric" | "biphasic_asymmetric" | "triphasic";
  cathodic_first: boolean;         // vast majority MUST be true
  cathodic_uA: number;             // current amplitude (negative phase)
  cathodic_us: number;             // pulse width, μs
  anodic_uA: number;
  anodic_us: number;
  interphase_gap_us: 0..100;
  frequency_hz: number;
  pulses_per_burst: uint;
  burst_duration_ms: number;
}
```

### 6.2 Safety envelope (Shannon limit)

For each electrode, the pair `(charge_density, charge_per_phase)` MUST satisfy:

```
log10(Q/A) ≤ k - log10(Q)           with k ≤ 1.5 (default) or ≤ 1.85 (expert)
```

where `Q` is charge per phase (μC) and `A` is the electrode geometric area (cm²). Implementations MUST reject any command violating this bound at the driver level — the refusal is logged as `safety.shannon_violation`.

### 6.3 Priority-driven amplitude scaling

```
final_uA = threshold_uA · (1 + α · salience + β · priority_weight)
```

Default `α = 0.35`, `β = 0.25`. Values are clamped to `max_safe_current_uA` from the array (Phase 3 § 2.3).

### 6.4 Charge balance monitor

A DC leakage of ≤ 100 nA is the regulatory line. The device MUST sample the residual voltage between phases and compensate by shortening the anodic phase by Δt such that:

```
cathodic_uA · cathodic_us  =  anodic_uA · (anodic_us − Δt)   (within ±1 %)
```

Failure to converge over 50 cycles asserts `emergency_stop.charge_imbalance` (Phase 4 § 3).

---

## 7. Reference JSON example (cycle 4 218 017)

```json
{
  "cycle_id": "01JCG0T0A2V5M8W7P3Q9S1R6N4",
  "patient_id": "WIA-BE-KR-Seoul-PID-0042",
  "device_id": "WIA-BE-SN-0001",
  "timestamp": "2026-04-25T14:30:00.128Z",
  "captured_frame": {
    "frame_id": "f-7a9…",
    "sequence": 4218017,
    "image_data": {
      "width": 640, "height": 480,
      "format": "gray8", "encoding": "raw", "data": "…base64…"
    },
    "camera_params": {"fov":{"horizontal_deg":82,"vertical_deg":62},
                      "exposure_ms": 8.0, "gain_iso": 200}
  },
  "processed_visual": {
    "objects": [
      {"object_id":"o-1","label":"stairs_down","confidence":0.94,
       "bbox":{"x":0.35,"y":0.42,"w":0.28,"h":0.30},
       "distance_m":1.7,"priority":"critical","threat":true,"moving":false}
    ]
  },
  "phosphene_map": { "calibration_revision": "2026-04-20" },
  "stimulation": {
    "electrodes": {
      "12":{"type":"biphasic_symmetric","cathodic_first":true,
            "cathodic_uA":180,"cathodic_us":200,
            "anodic_uA":180,"anodic_us":200,
            "interphase_gap_us":20,"frequency_hz":60,
            "pulses_per_burst":6,"burst_duration_ms":100}
    },
    "safety_envelope": {"k_shannon":1.5, "max_total_current_mA":4.0}
  }
}
```

---

## 8. JSON Schema excerpt (normative)

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title":   "WIA-MED-BE-001 VisualCycle",
  "type":    "object",
  "required":["cycle_id","patient_id","device_id","timestamp",
              "captured_frame","processed_visual","phosphene_map","stimulation"],
  "properties": {
    "cycle_id":   {"type":"string","pattern":"^[0-9A-HJKMNP-TV-Z]{26}$"},
    "patient_id": {"type":"string","pattern":"^WIA-BE-[A-Z]{2}-[A-Za-z0-9-]+-PID-\\d+$"},
    "timestamp":  {"type":"string","format":"date-time"},
    "stimulation": { "$ref": "#/definitions/stimulation" }
  }
}
```

A complete schema (≈ 1200 lines) ships in `schema/wia-be-001.json` and is pinned by content hash in every device firmware release.

---

## 9. Data at rest — privacy, retention, de-identification

- **PHI minimisation**: patient ID is hashed with a per-jurisdiction salt (`HMAC-SHA256`, key custody at the clinic). Raw names, DOBs, MRNs MUST NOT appear in device telemetry.
- **Encryption**: payloads at rest in the device log ring buffer are AES-256-GCM with per-session keys rolled by the fitting console.
- **Retention**: clinical fitting records keep phosphene maps and psychophysics traces for the regulator-mandated period (US 21 CFR 820 + 7 years; EU IVDR Annex IX); raw frames are retained only when the patient or clinician opts in.

---

## 10. Conformance checklist

- [ ] Every `VisualCycle` validates against the JSON Schema.
- [ ] Gray-scale frame path sustains ≥ 60 fps.
- [ ] Latency from frame capture to electrode command ≤ 150 ms (P95).
- [ ] Shannon-limit check rejects violating commands at the driver.
- [ ] Phosphene map versioned; cross-session drift within budget.
- [ ] PHI never leaves the device in the clear; AES-256-GCM at rest.
- [ ] `DetectedObject.priority` respects the taxonomy in § 4.2.
- [ ] Charge-balance DC residual ≤ 100 nA (averaged over 1 s).

---

## 11. Normative References

1. ISO 14708-1:2014 — Implants for surgery, active implantable medical devices.
2. IEC 60601-1; IEC 60601-2-10 — Medical electrical equipment safety.
3. ISO 10993-1 — Biocompatibility evaluation of medical devices.
4. DICOM PS3.3 — Ophthalmic Information Object Definitions.
5. HL7 FHIR R5 — Device, Procedure, Observation resources.

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
