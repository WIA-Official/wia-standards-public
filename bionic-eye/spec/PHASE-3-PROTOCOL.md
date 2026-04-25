# WIA-MED-BE-001 · Bionic Eye — Phase 3: Protocol

**Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 3 specifies **the electrical, wireless, and logical protocols** that sit between the processing unit outside the eye and the implanted electronics. Four layers are standardised:

1. Electrode array physical and electrical interface (§ 2).
2. Transcutaneous inductive data-and-power link (§ 3).
3. External-band radio (MICS/MedRadio) for service and firmware (§ 4).
4. Security and integrity at the bit level (§ 5).

Together with the safety envelope defined in Phase 1 § 6.2 and the e-stop hardware in Phase 4, these establish the electrical and communication contract against which every WIA-compliant implant is certified.

### 1.1 Normative references

- ISO 14708-1:2014 — Active implantable medical devices.
- IEC 60601-1, 60601-1-2 (EMC), 60601-2-10 (stimulators).
- AIMD Directive 90/385/EEC (historical) and EU MDR (2017/745).
- FCC 47 CFR §95 Subpart I — MedRadio (401–406 MHz, 413–419, 426–432, 438–444 MHz).
- ITU-R RR Article 5 — 402–405 MHz allocation (MICS).
- ISO/IEC 14443-4 — Inductive coupling (reference for close-range link).
- IEEE 802.15.6 — Wireless Body Area Networks (optional).
- ISO/TS 10974 — MR-safety labelling for active implants.

---

## 2. Electrode array physical interface

### 2.1 Supported implant topologies

| Family          | Location       | Typical count | Electrode pitch |
|-----------------|----------------|---------------|-----------------|
| Epiretinal      | retinal ILM    | 60 – 1 500    | 100–500 μm      |
| Subretinal      | subretinal space| 378 – 10 000 | 50–100 μm       |
| Suprachoroidal  | above choroid  | 44 – 99       | 500–1000 μm     |
| Optic-nerve cuff| nerve sheath   | 4 – 20        | 200–300 μm      |
| Cortical V1     | occipital cortex| 60 – 1 000   | 400 μm          |

### 2.2 Electrode descriptor

```ts
interface ElectrodeArray {
  array_id: string;
  type: "epiretinal" | "subretinal" | "suprachoroidal"
      | "optic_nerve" | "cortical";
  manufacturer: string;
  model: string;
  serial: string;

  physical: {
    total: uint; active: uint; rows: uint; cols: uint;
    diameter_um: 100..500;
    spacing_um:  200..1000;
    area_mm2:    number;      // per electrode (geometric)
    thickness_um:number;
    material: "platinum" | "pt_ir" | "iridium_oxide"
            | "titanium_nitride" | "pedot" | "carbon_nanotube";
    substrate: "parylene_c" | "polyimide" | "silicone" | "liquid_crystal_polymer";
  };

  placement: {
    eye: "left" | "right" | "n/a-cortical";
    implant_date: RFC3339;
    surgeon: string;
    institution: string;
    oct_alignment_revision: string;
  };
}
```

### 2.3 Per-electrode electrical parameters

```ts
interface ElectrodeInfo {
  index: uint;
  grid: { row: uint, col: uint };
  status: { functional: boolean, enabled: boolean,
            disabled_reason?: "short" | "open" | "impedance_drift"
                            | "patient_discomfort" | "regulatory_limit" };
  electrical: {
    impedance_kOhm: number;        // at 1 kHz
    phase_deg: number;             // expected −20° to −80°
    charge_capacity_mC_cm2: number;// material-limited
    threshold_uA: number;          // psychophysical 50 % (Phase 2 § 5)
    max_safe_uA: number;           // Shannon clamp (Phase 1 § 6.2)
    compliance_V: number;
    dc_leakage_nA: number;
  };
  last_measurement: RFC3339;
}
```

### 2.4 Impedance bands

| Band        | Z (kΩ @ 1 kHz)  | Action                               |
|-------------|------------------|--------------------------------------|
| normal      | 1 – 50           | nominal operation                    |
| warning     | 50 – 100         | diagnostic log entry; no stimulation |
| high / open | > 100            | electrode disabled                   |
| low / short | < 0.5            | electrode disabled; fault alarm      |

### 2.5 Material charge limits

| Material            | Capacity (mC/cm²) | Recommended max |
|---------------------|-------------------|-----------------|
| Platinum            | 0.05–0.15         | 0.05            |
| Pt-Ir               | 0.05–0.15         | 0.05            |
| Iridium oxide (IrOx)| 1–5               | 1.0             |
| Titanium nitride    | 0.5–1             | 0.5             |
| PEDOT               | 2–15              | 2.0             |

### 2.6 Histocompatibility and encapsulation

- All contacting materials MUST meet ISO 10993-1 biocompatibility.
- Encapsulation: parylene-C + liquid-crystal polymer composite; helium leak ≤ 1e-9 atm·cc/s.
- Expected service life: ≥ 10 years (retinal), ≥ 5 years (cortical).

---

## 3. Transcutaneous inductive link

The *primary* link between the external visor/processor and the implant is an inductive coil pair. It carries both power and bidirectional data.

### 3.1 Link parameters

| Parameter                   | Value                           |
|-----------------------------|---------------------------------|
| Carrier frequency           | 13.56 MHz or 6.78 MHz           |
| Modulation (down / up)      | ASK 100 % / LSK load-shift      |
| Data rate (down / up)       | 1.5 Mbit/s / 250 kbit/s         |
| Coil coupling k             | 0.05 – 0.25                     |
| Delivered power (nominal)   | 40–120 mW to implant            |
| SAR (1 g, head)             | ≤ 1.6 W/kg                      |
| Alignment tolerance         | ±3 mm lateral, ±10° tilt        |
| Fail-safe power cap         | 200 mW                          |

### 3.2 Frame structure (inductive air interface)

```
┌──────┬──────┬──────────┬──────────────┬────────┐
│ SYNC │ LEN  │ HEADER   │ PAYLOAD      │ CRC32  │
│  4B  │ 2B   │ 6B       │ 0–512 B      │ 4 B    │
└──────┴──────┴──────────┴──────────────┴────────┘
```

- `SYNC`: `0xA5 5A C3 3C` (anti-alias pattern).
- `HEADER`: `opcode (1B) | seq (2B) | flags (1B) | dest (1B) | reserved (1B)`.
- Payload encrypted with AES-128-GCM (nonce = seq ‖ device_id lo32); integrity via CRC32 *plus* GCM tag.

### 3.3 Opcode table (excerpt)

| Opcode | Name                    | Direction |
|--------|--------------------------|-----------|
| 0x10   | STIM_COMMAND             | down      |
| 0x11   | STIM_BURST               | down      |
| 0x12   | AMP_OVERRIDE             | down      |
| 0x20   | IMPEDANCE_SWEEP_REQUEST  | down      |
| 0x21   | IMPEDANCE_SWEEP_RESULT   | up        |
| 0x30   | EMERGENCY_STOP           | down      |
| 0x31   | ESTOP_ACK                | up        |
| 0x40   | TELEMETRY                | up        |
| 0xFE   | FIRMWARE_CHUNK           | down      |
| 0xFF   | FIRMWARE_APPLY           | down      |

### 3.4 Reliability

- Every downlink frame has a unique sequence number; implant discards duplicates.
- Up to 4 retransmissions on `NACK`; after that the external device triggers an e-stop.
- Keep-alive frames every 100 ms; loss > 300 ms puts the implant in `standby` (stimulation disabled, impedance monitoring continues).

### 3.5 Tissue heating and SAR

The external coil drives ≤ 1.6 W/kg SAR averaged over 1 g of tissue. Thermistors embedded in the implant package report every 250 ms; sustained tissue ΔT > 1 °C triggers the thermal e-stop of Phase 4 § 3.

---

## 4. MedRadio / MICS band services

Service, firmware, and non-real-time telemetry use MedRadio (FCC Part 95 Subpart I) 401–406 MHz, cooperating with MICS. This channel is **not** used for live stimulation commands.

### 4.1 Frequency plan

| Slot      | Frequency (MHz) | Use                        |
|-----------|-----------------|----------------------------|
| MICS 1    | 402.0 – 402.1   | Session establishment      |
| MICS 2–10 | 402.1 – 405.0   | Data channels (10 × 300 kHz)|
| MEDS      | 413 – 419       | Secondary (region A only)  |
| MEDS      | 426 – 432       | Secondary (region B only)  |

### 4.2 Link layer

- Modulation: GFSK 250 kbit/s (default), 500 kbit/s (good-link).
- LBT (listen-before-talk) with energy threshold −95 dBm.
- Max continuous TX duty cycle: 1 % unless `service_mode=true` (paired clinician, in a Faraday enclosure).

### 4.3 Firmware update flow

1. Clinician console authenticates via PAI/CPI, proves patient presence.
2. External unit switches to MICS service mode, listens for emissions ≥ 250 ms.
3. Firmware image (signed, see § 5.3) is streamed in 256-byte chunks, each individually signed and acknowledged.
4. Implant validates the full image signature before committing; on failure, rolls back and reports `fw_apply_failed`.

---

## 5. Security and integrity

### 5.1 Device identity

Each implant ships with an ECDSA P-256 key-pair generated at factory inside the secure element. Public keys are signed by the manufacturer CA and cross-signed by WIA's MED-BE root.

### 5.2 Key hierarchy

```
WIA-MED-BE Root CA (offline HSM, 15-year key)
       │
       ├── Manufacturer Intermediate CA (5-year)
       │        └── Device certificates (2-year, auto-rotated)
       └── Clinician institution CA (3-year)
                └── Clinician smart-card certificates (1-year)
```

Revocation is published via CRL + OCSP under `https://pki.bionic-eye.wia.org`.

### 5.3 Firmware signing

- Signature: ECDSA P-256 over canonicalised manifest + image SHA-384.
- Manifest fields: `version`, `min_previous`, `max_previous`, `board_id`, `image_sha384`, `issuer`, `expires_at`.
- Downgrade attacks blocked by `min_previous` monotonicity.

### 5.4 Replay and impersonation

- All commands carry a monotonic `seq` and a time-bounded nonce.
- GCM tag on every frame; nonce = `seq ‖ device_id[lo32]`.
- Implant tracks a 1024-entry nonce window; wild resets require a full re-pairing ceremony (in-clinic only).

### 5.5 Tamper response

Attempted firmware load with invalid signature → implant enters `quarantine`: stimulation disabled, impedance monitoring only, audit log flushed via inductive link at next session, administrator notified.

---

## 6. MR safety and labelling

The implant MUST be labelled per ISO/TS 10974 as **MR conditional** with at least the following conditions:

| Parameter           | Condition                           |
|---------------------|-------------------------------------|
| Static field B0     | ≤ 1.5 T or ≤ 3 T (model-dependent)  |
| Gradient slew       | ≤ 200 T/m/s                         |
| RF SAR (whole body) | ≤ 2 W/kg (normal operating mode)    |
| Scan duration       | ≤ 30 min continuous                 |

Scan-room labelling, patient card, and DICOM private tag `(0018, xxxx)` record the last MR event; clinicians MUST re-image the fundus and re-impedance before re-enabling stimulation.

---

## 7. Observability at the protocol layer

Every frame observed or emitted by the external unit is tagged and exported to the RTI telemetry bus (Phase 2 § 8). Mandatory fields:

- `frame_id`, `direction` (`down`/`up`), `opcode`, `seq`, `crc_ok`, `gcm_ok`.
- `rssi_dBm`, `snr_dB` (MedRadio), `coil_k_estimate` (inductive).
- `timestamp` with nanosecond resolution (drift tracked per § 2.3 Phase 1).

OpenTelemetry spans carry `wia.be.cycle_id`, `wia.be.opcode`, `wia.be.seq` so that clinical dashboards can correlate radio anomalies with perceptual reports from the patient.

---

## 8. Worked scenarios

### 8.1 Downlink a stimulation burst

```
Down: SYNC A55AC33C | LEN 002C
      OP=0x11 SEQ=0x18F3 FLAGS=0x00 DEST=0x01
      PAYLOAD (enc): 11 electrodes × (cathodic_uA, cathodic_us,
                                     interphase_us, anodic_uA, anodic_us)
      + frequency_hz, pulses_per_burst
      CRC32 = 0xA3…B1, GCM_TAG = 0x4F…92
```

### 8.2 Impedance sweep

```
Down: OP=0x20 (start, 16 electrodes, 1 kHz, 10 μA probe)
Up:   OP=0x21 | array of (index, |Z|, phase)
```

### 8.3 Firmware update (summary)

```
Clinician console → CPI: POST /devices/{id}/firmware (manifest + image)
CPI → External unit: WebSocket push of signed chunks
External unit ⇄ Implant: OP=0xFE stream … OP=0xFF apply
Implant → External unit: OP=0x40 TELEMETRY{fw=v1.7.2, sig_verified=true}
```

---

## 9. Emissions, coexistence, and EMC

- Inductive link meets FCC 47 CFR §18 and EN 300 330 (non-specific SRD).
- MedRadio meets FCC 47 CFR §95 Subpart I and ETSI EN 301 839.
- EMC per IEC 60601-1-2 edition 4.1: immunity to radiated RF (10 V/m, 80 MHz–2.7 GHz), ESD (±8 kV air, ±6 kV contact), burst (±2 kV mains), surge (±2 kV common-mode).
- Coexistence with other near-body radios (Bluetooth LE, Wi-Fi 2.4/5 GHz) validated by IEC 60601-1-11 home-use testing.

---

## 10. Compliance checklist

- [ ] Array physical metadata present and matches implanted serial.
- [ ] Material-specific charge caps enforced.
- [ ] Inductive link SAR ≤ 1.6 W/kg over 1 g; thermistor alarm wired to e-stop.
- [ ] MedRadio channels respect LBT + 1 % duty cycle outside service mode.
- [ ] AES-128-GCM on every inductive frame; signed firmware only.
- [ ] Revocation (OCSP) reachable from CPI; failures degrade to last cached CRL.
- [ ] MR-conditional label matches product; DICOM `(0018, xxxx)` updated.
- [ ] OpenTelemetry spans on every frame; RSSI/SNR/coil-k recorded.

---

## 11. References

1. ISO 14708-1:2014; IEC 60601-1 / 2-10; IEC 60601-1-2 ed. 4.1.
2. FCC 47 CFR §95 Subpart I (MedRadio); ETSI EN 301 839.
3. ITU-R RR Article 5 — MICS allocation 402–405 MHz.
4. ISO/TS 10974 — MR safety for active implantable medical devices.
5. IEEE 802.15.6 — Body Area Networks.
6. NIST SP 800-175B — Guide to cryptographic standards.

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
