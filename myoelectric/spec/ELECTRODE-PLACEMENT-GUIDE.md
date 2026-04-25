# WIA EMG Electrode Placement Guide

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This guide defines standardized electrode placement protocols for EMG-based myoelectric prosthetic control. Proper electrode placement is critical for reliable signal acquisition and consistent gesture recognition performance.

## 2. General Principles

### 2.1 SENIAM Guidelines

This guide follows SENIAM (Surface EMG for Non-Invasive Assessment of Muscles) recommendations with adaptations for prosthetic applications.

### 2.2 Key Principles

1. **Electrode over muscle belly**: Place electrodes over the thickest part of the muscle
2. **Parallel to muscle fibers**: Orient electrodes along the muscle fiber direction
3. **Avoid motor points**: Stay away from neuromuscular junctions
4. **Avoid tendons**: Place electrodes on contractile tissue, not tendons
5. **Consistent placement**: Use anatomical landmarks for reproducibility

## 3. Coordinate System

### 3.1 Reference Frame

```
Coordinate System Origin: Lateral epicondyle of humerus

X-axis: Distal direction along forearm (positive toward wrist)
Y-axis: Radial direction (positive toward thumb side)
Z-axis: Perpendicular to forearm surface (positive outward)

Circumference (θ): Angular position around forearm
  - 0°: Posterior (dorsal) midline
  - 90°: Radial (lateral) side
  - 180°: Anterior (volar) midline
  - 270°: Ulnar (medial) side
```

### 3.2 Placement Specification

```rust
struct ElectrodePlacement {
    /// Target muscle identifier
    muscle: String,

    /// Distance from elbow (lateral epicondyle) in cm
    /// Positive values: toward wrist
    x_distance_cm: f32,

    /// Offset from muscle center line in cm
    /// Positive: toward radial side
    y_offset_cm: f32,

    /// Angular position around forearm circumference (degrees)
    /// 0° = posterior midline, measured clockwise when viewing from elbow
    circumference_deg: f32,

    /// Electrode orientation relative to muscle fibers
    orientation: ElectrodeOrientation,

    /// Reference anatomical landmark
    landmark: AnatomicalLandmark,
}

enum ElectrodeOrientation {
    Parallel,       // Along muscle fibers (recommended)
    Perpendicular,  // Across muscle fibers
    Oblique(f32),   // Angle in degrees
}

enum AnatomicalLandmark {
    LateralEpicondyle,
    MedialEpicondyle,
    RadialStyloid,
    UlnarStyloid,
    OlecranonProcess,
}
```

## 4. Forearm Flexor Muscles

### 4.1 Flexor Carpi Radialis (FCR)

**Function:** Wrist flexion, radial deviation

| Parameter | Value |
|-----------|-------|
| X distance | 8-10 cm from lateral epicondyle |
| Circumference | 135-150° |
| Depth | Superficial |
| Orientation | Parallel to forearm axis |

**Palpation Method:**
1. Resist wrist flexion with radial deviation
2. Locate prominent tendon medial to radial artery at wrist
3. Trace proximally to muscle belly

**Electrode Position:**
- Center: 1/3 distance from medial epicondyle to radial styloid
- Inter-electrode distance: 20 mm

### 4.2 Flexor Carpi Ulnaris (FCU)

**Function:** Wrist flexion, ulnar deviation

| Parameter | Value |
|-----------|-------|
| X distance | 10-12 cm from lateral epicondyle |
| Circumference | 200-220° |
| Depth | Superficial |
| Orientation | Parallel to ulna |

**Palpation Method:**
1. Resist wrist flexion with ulnar deviation
2. Locate tendon proximal to pisiform bone
3. Trace to muscle belly along ulnar border

**Electrode Position:**
- Center: 1/3 distance from medial epicondyle to ulnar styloid
- Located on ulnar aspect of forearm

### 4.3 Flexor Digitorum Superficialis (FDS)

**Function:** Finger flexion (PIP joints)

| Parameter | Value |
|-----------|-------|
| X distance | 6-8 cm from medial epicondyle |
| Circumference | 160-180° |
| Depth | Intermediate |
| Orientation | Parallel to forearm axis |

**Palpation Method:**
1. Resist PIP flexion while DIP is extended
2. Palpate volar forearm between FCR and FCU

**Electrode Position:**
- Midline of volar forearm
- Proximal third of forearm

### 4.4 Pronator Teres (PT)

**Function:** Forearm pronation

| Parameter | Value |
|-----------|-------|
| X distance | 4-6 cm from medial epicondyle |
| Circumference | 120-140° |
| Depth | Superficial |
| Orientation | Oblique (follows muscle fiber direction) |

**Palpation Method:**
1. Resist pronation with elbow flexed
2. Palpate medial proximal forearm

## 5. Forearm Extensor Muscles

### 5.1 Extensor Carpi Radialis Longus (ECRL)

**Function:** Wrist extension, radial deviation

| Parameter | Value |
|-----------|-------|
| X distance | 6-8 cm from lateral epicondyle |
| Circumference | 45-60° |
| Depth | Superficial |
| Orientation | Parallel to radius |

**Palpation Method:**
1. Resist wrist extension with radial deviation
2. Palpate lateral proximal forearm
3. Muscle belly is proximal to ECRB

**Electrode Position:**
- 1/3 distance from lateral epicondyle to radial styloid
- Over lateral muscle mass

### 5.2 Extensor Carpi Radialis Brevis (ECRB)

**Function:** Wrist extension

| Parameter | Value |
|-----------|-------|
| X distance | 8-10 cm from lateral epicondyle |
| Circumference | 30-45° |
| Depth | Partially deep to ECRL |
| Orientation | Parallel to forearm axis |

**Palpation Method:**
1. Resist wrist extension (neutral)
2. Palpate distal to ECRL on posterior forearm

### 5.3 Extensor Carpi Ulnaris (ECU)

**Function:** Wrist extension, ulnar deviation

| Parameter | Value |
|-----------|-------|
| X distance | 10-12 cm from lateral epicondyle |
| Circumference | 315-330° |
| Depth | Superficial |
| Orientation | Parallel to ulna |

**Palpation Method:**
1. Resist wrist extension with ulnar deviation
2. Palpate between ulna and extensor digitorum

### 5.4 Extensor Digitorum (ED)

**Function:** Finger extension (MCP joints)

| Parameter | Value |
|-----------|-------|
| X distance | 8-10 cm from lateral epicondyle |
| Circumference | 0-15° |
| Depth | Superficial |
| Orientation | Parallel to forearm axis |

**Palpation Method:**
1. Resist finger extension at MCP joints
2. Palpate central posterior forearm

**Electrode Position:**
- Center of posterior forearm
- 2/5 distance from lateral epicondyle to radial styloid

### 5.5 Supinator

**Function:** Forearm supination

| Parameter | Value |
|-----------|-------|
| X distance | 4-6 cm from lateral epicondyle |
| Circumference | 60-80° |
| Depth | Deep (covered by other extensors) |
| Orientation | Wraps around radius |

**Note:** Difficult to isolate; signals often mixed with ECRL/ECRB

## 6. Standard Electrode Configurations

### 6.1 Minimal Configuration (2 Channels)

For basic open/close control:

| Channel | Muscle | Function |
|---------|--------|----------|
| CH0 | Flexor group (FCR/FDS) | Close grip |
| CH1 | Extensor group (ED/ECRB) | Open grip |

### 6.2 Standard Configuration (4 Channels)

For multi-grip prosthetics:

| Channel | Muscle | Function |
|---------|--------|----------|
| CH0 | FCR | Wrist flexion, grip |
| CH1 | FCU | Ulnar grip component |
| CH2 | ECRL/ECRB | Wrist extension, release |
| CH3 | ED | Finger extension |

### 6.3 Advanced Configuration (8 Channels)

For gesture recognition:

| Channel | Muscle | Primary Action |
|---------|--------|----------------|
| CH0 | FCR | Wrist flex + radial dev |
| CH1 | FCU | Wrist flex + ulnar dev |
| CH2 | FDS | Finger flexion |
| CH3 | PT | Pronation |
| CH4 | ECRL | Wrist ext + radial dev |
| CH5 | ECU | Wrist ext + ulnar dev |
| CH6 | ED | Finger extension |
| CH7 | Supinator | Supination |

## 7. Electrode Specifications

### 7.1 Electrode Types

| Type | Size | Application |
|------|------|-------------|
| Pre-gelled Ag/AgCl | 10 mm | Clinical, evaluation |
| Dry textile | 10-15 mm | Wearable prosthetic |
| Dry metal | 8-12 mm | Research, high density |

### 7.2 Inter-Electrode Distance

| Configuration | IED | Notes |
|---------------|-----|-------|
| Bipolar | 20 mm | Standard, SENIAM compliant |
| High-density | 8-10 mm | Better spatial resolution |
| Large muscle | 25-30 mm | For large muscle bulk |

### 7.3 Reference Electrode

**Placement Options:**
1. **Bony prominence**: Olecranon, ulnar styloid
2. **Electrically quiet area**: Lateral epicondyle
3. **Common reference**: Single reference for all channels

## 8. Skin Preparation Protocol

### 8.1 Standard Protocol

1. **Clean skin**: Remove oils with alcohol wipe
2. **Light abrasion**: Use prep gel or fine sandpaper (optional)
3. **Check impedance**: Target < 10 kΩ (< 5 kΩ preferred)
4. **Apply electrode**: Ensure good contact, no air bubbles
5. **Secure placement**: Use adhesive or wrap

### 8.2 Impedance Guidelines

| Impedance | Quality | Action |
|-----------|---------|--------|
| < 5 kΩ | Excellent | Proceed |
| 5-10 kΩ | Good | Acceptable |
| 10-20 kΩ | Fair | Re-prep skin |
| > 20 kΩ | Poor | Full re-application |

## 9. Residual Limb Considerations

### 9.1 Transradial Amputation

**Challenges:**
- Limited muscle bulk
- Altered muscle positions
- Potential neuroma sensitivity

**Recommendations:**
- Map available muscles individually
- Consider muscle reinnervation sites (TMR)
- Use higher electrode density if needed

### 9.2 Socket Integration

**Embedded Electrode Requirements:**
- Maintain consistent pressure
- Account for socket donning/doffing
- Allow for limb volume changes
- Ensure easy cleaning/replacement

## 10. Placement Verification

### 10.1 Signal Quality Check

```rust
struct PlacementVerification {
    channel: u8,
    muscle: String,
    rest_amplitude: f32,      // Should be < 0.02 mV RMS
    mvc_amplitude: f32,       // Should be > 0.1 mV RMS
    snr_db: f32,              // Should be > 20 dB
    cross_talk_db: f32,       // Should be < -20 dB
}
```

### 10.2 Functional Tests

1. **Rest test**: Verify low baseline (< 20 μV RMS)
2. **MVC test**: Verify adequate signal amplitude
3. **Isolation test**: Confirm target muscle activation
4. **Cross-talk test**: Check adjacent channel independence
5. **Motion artifact test**: Verify signal during movement

### 10.3 Acceptance Criteria

| Test | Pass Criteria |
|------|---------------|
| Rest baseline | < 20 μV RMS |
| MVC signal | > 100 μV RMS |
| SNR | > 20 dB |
| Cross-talk | < -20 dB |
| Impedance | < 10 kΩ |

## 11. Troubleshooting

### 11.1 Common Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| High baseline | Poor contact | Re-prep skin, check electrode |
| 50/60 Hz noise | Power line | Check ground, shielding |
| Low amplitude | Wrong location | Reposition over muscle belly |
| Cross-talk | Electrodes too close | Increase spacing, reposition |
| Motion artifact | Loose electrode | Secure with tape/wrap |

### 11.2 Signal Quality Assessment

```rust
fn assess_placement_quality(
    signal: &[f32],
    mvc_reference: f32,
) -> PlacementQuality {
    let rest_rms = compute_rms(&signal[..1000]);  // First second
    let active_rms = compute_rms(&signal[2000..3000]);  // Active period

    PlacementQuality {
        rest_level: rest_rms,
        activation_level: active_rms,
        dynamic_range: 20.0 * (active_rms / rest_rms).log10(),
        mvc_ratio: active_rms / mvc_reference,
    }
}
```

---

*WIA Myoelectric Standard - Electrode Placement Guide v1.0.0*
