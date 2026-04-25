# Circadian Phase Estimation Algorithm

## WIA-SLEEP Standard Algorithm Specification

**Version:** 1.0.0
**Last Updated:** 2025-01-15

---

## 1. Overview

This algorithm estimates an individual's current circadian phase based on available biomarkers and behavioral data. Circadian phase is expressed relative to Dim Light Melatonin Onset (DLMO), the gold standard phase marker.

---

## 2. Phase Markers Hierarchy

| Marker | Priority | Accuracy | Availability |
|--------|----------|----------|--------------|
| DLMO (measured) | 1 | Very High | Low (lab) |
| Core Body Temperature Nadir | 2 | High | Medium |
| Cortisol Awakening Response | 3 | Medium-High | Medium |
| Actigraphy + MCTQ | 4 | Medium | High |
| Behavioral estimation | 5 | Low-Medium | Very High |

---

## 3. Primary Algorithm: DLMO-Based Phase

### 3.1 Direct DLMO Measurement

When DLMO is directly measured:

```typescript
interface CircadianPhase {
  dlmo: Time;                    // DLMO time
  cbtNadir: Time;               // Core Body Temp nadir
  currentPhase: number;          // 0-24 circadian hours
  entrainmentStatus: EntrainmentStatus;
}
```

### 3.2 Phase Calculation

```python
def calculate_current_phase(
    current_time: datetime,
    dlmo: Time
) -> float:
    """
    Calculate current circadian phase in hours.
    Phase 0 = DLMO
    Phase 6-8 = Typical wake time (melatonin offset)
    Phase 14-16 = Peak alertness
    Phase 22-24 = Sleep onset zone
    """
    dlmo_hours = time_to_decimal(dlmo)
    current_hours = current_time.hour + current_time.minute / 60

    phase = current_hours - dlmo_hours

    # Normalize to 0-24
    while phase < 0:
        phase += 24
    while phase >= 24:
        phase -= 24

    return phase
```

---

## 4. Secondary Algorithm: CBT Nadir Estimation

### 4.1 DLMO to CBT Nadir Relationship

Core Body Temperature nadir typically occurs ~7 hours after DLMO:

```python
CBT_NADIR_OFFSET = 7.0  # hours after DLMO

def estimate_cbt_nadir_from_dlmo(dlmo: Time) -> Time:
    dlmo_hours = time_to_decimal(dlmo)
    cbt_nadir_hours = dlmo_hours + CBT_NADIR_OFFSET

    # Handle wraparound
    if cbt_nadir_hours >= 24:
        cbt_nadir_hours -= 24

    return decimal_to_time(cbt_nadir_hours)
```

### 4.2 Phase Estimation from CBT Nadir

```python
def estimate_dlmo_from_cbt_nadir(cbt_nadir: Time) -> Time:
    cbt_hours = time_to_decimal(cbt_nadir)
    dlmo_hours = cbt_hours - CBT_NADIR_OFFSET

    if dlmo_hours < 0:
        dlmo_hours += 24

    return decimal_to_time(dlmo_hours)
```

---

## 5. Tertiary Algorithm: Behavioral Estimation

### 5.1 From Habitual Sleep Times

```python
DLMO_TO_SLEEP_INTERVAL = 2.5  # hours

def estimate_dlmo_from_sleep_behavior(
    habitual_bedtime: Time,
    habitual_wake_time: Time,
    chronotype: ChronotypeClass
) -> Time:
    bedtime_hours = time_to_decimal(habitual_bedtime)

    # Adjust for chronotype
    chronotype_adjustment = {
        "extreme_early": -0.5,
        "moderate_early": -0.25,
        "intermediate": 0,
        "moderate_late": 0.25,
        "extreme_late": 0.5
    }

    adjustment = chronotype_adjustment.get(chronotype, 0)
    dlmo_hours = bedtime_hours - DLMO_TO_SLEEP_INTERVAL + adjustment

    if dlmo_hours < 0:
        dlmo_hours += 24

    return decimal_to_time(dlmo_hours)
```

### 5.2 From Actigraphy Data

```python
def estimate_phase_from_actigraphy(
    l5_onset: Time,      # Onset of 5 least active hours
    m10_onset: Time,     # Onset of 10 most active hours
    relative_amplitude: float
) -> CircadianPhaseEstimate:
    """
    L5 onset approximates sleep midpoint
    M10 onset approximates wake/activity onset
    """
    l5_hours = time_to_decimal(l5_onset)

    # L5 onset typically occurs ~6 hours after DLMO
    estimated_dlmo = l5_hours - 6.0
    if estimated_dlmo < 0:
        estimated_dlmo += 24

    # Confidence based on rhythm robustness
    confidence = relative_amplitude * 0.8

    return CircadianPhaseEstimate(
        dlmo=decimal_to_time(estimated_dlmo),
        confidence=confidence,
        method="actigraphy"
    )
```

---

## 6. Entrainment Status Assessment

### 6.1 Algorithm

```python
def assess_entrainment(
    activity_rhythm: ActivityRhythm
) -> EntrainmentStatus:
    IS = activity_rhythm.interdaily_stability
    IV = activity_rhythm.intradaily_variability
    RA = activity_rhythm.relative_amplitude

    # Well-entrained: High IS, Low IV, High RA
    if IS > 0.6 and IV < 1.0 and RA > 0.8:
        return EntrainmentStatus.ENTRAINED

    # Free-running: Low IS, preserved RA
    if IS < 0.4 and RA > 0.6:
        return EntrainmentStatus.FREE_RUNNING

    # Irregular: Low IS, Low RA
    if IS < 0.4 and RA < 0.6:
        return EntrainmentStatus.IRREGULAR

    # Check for phase disorders based on L5 timing
    l5_hours = time_to_decimal(activity_rhythm.l5_onset)

    # Delayed: L5 onset after 03:00
    if 3.0 < l5_hours < 6.0:
        return EntrainmentStatus.DELAYED

    # Advanced: L5 onset before 23:00
    if 20.0 < l5_hours < 23.0:
        return EntrainmentStatus.ADVANCED

    return EntrainmentStatus.ENTRAINED
```

### 6.2 Rhythm Parameters

| Parameter | Range | Interpretation |
|-----------|-------|----------------|
| Interdaily Stability (IS) | 0-1 | Day-to-day consistency |
| Intradaily Variability (IV) | 0-2+ | Rhythm fragmentation |
| Relative Amplitude (RA) | 0-1 | Rhythm robustness |

---

## 7. Phase Response Curve (PRC)

### 7.1 Light PRC

```python
def calculate_light_phase_response(
    light_timing_relative_to_cbt_nadir: float,  # hours
    intensity_lux: float,
    duration_min: float
) -> float:
    """
    Returns expected phase shift in hours.
    Negative = advance, Positive = delay
    """
    MAX_SHIFT = 2.0  # Maximum achievable shift per day

    # Intensity factor (logarithmic saturation)
    intensity_factor = min(1.0, log10(intensity_lux + 1) / 4)

    # Duration factor
    duration_factor = min(1.0, duration_min / 60)

    # PRC
    timing = light_timing_relative_to_cbt_nadir

    if -6 <= timing < 0:
        # Delay zone (before CBT nadir)
        phase_shift = MAX_SHIFT * sin((timing + 3) / 3 * pi / 2)
    elif 0 <= timing <= 6:
        # Advance zone (after CBT nadir)
        phase_shift = -MAX_SHIFT * sin(timing / 3 * pi / 2)
    else:
        # Dead zone
        phase_shift = 0

    return phase_shift * intensity_factor * duration_factor
```

### 7.2 Melatonin PRC

```python
def calculate_melatonin_phase_response(
    admin_time_relative_to_dlmo: float,  # hours
    dose_mg: float
) -> float:
    """
    Melatonin PRC is roughly opposite to light PRC.
    """
    MAX_SHIFT = 1.5  # Smaller effect than light

    # Dose factor (saturates around 0.5-1 mg)
    dose_factor = min(1.0, dose_mg / 0.5)

    timing = admin_time_relative_to_dlmo

    if -6 <= timing < 0:
        # Advance zone (before DLMO)
        phase_shift = -MAX_SHIFT * sin((timing + 3) / 3 * pi / 2)
    elif 0 <= timing <= 6:
        # Delay zone (after DLMO) - less used
        phase_shift = MAX_SHIFT * sin(timing / 3 * pi / 2) * 0.5
    else:
        phase_shift = 0

    return phase_shift * dose_factor
```

---

## 8. Alertness Prediction

### 8.1 Two-Process Model (Simplified)

```python
def predict_alertness(
    current_phase: float,       # 0-24 circadian hours
    hours_awake: float,         # Time since last sleep
    sleep_pressure: float = 0   # Accumulated sleep debt factor
) -> float:
    """
    Returns alertness score 0-100.
    Based on simplified Borbély two-process model.
    """
    # Process C: Circadian alertness rhythm
    # Peak at phase ~14-16 (afternoon), nadir at phase ~4-6 (night)
    circadian_component = 50 + 30 * cos((current_phase - 15) * 2 * pi / 24)

    # Process S: Homeostatic sleep pressure
    # Increases exponentially with time awake
    tau = 18.2  # Time constant (hours)
    homeostatic_pressure = 100 * (1 - exp(-hours_awake / tau))

    # Combined alertness
    alertness = circadian_component - (homeostatic_pressure * 0.4)

    # Account for sleep debt
    alertness -= sleep_pressure * 10

    return max(0, min(100, alertness))
```

### 8.2 Alertness Windows

```python
def generate_alertness_windows(dlmo: Time) -> List[AlertnessWindow]:
    dlmo_hours = time_to_decimal(dlmo)

    windows = []

    # Morning good zone: ~10-14 hours before DLMO (6-10 AM for DLMO at 20:00)
    morning_start = dlmo_hours - 14
    morning_end = dlmo_hours - 10
    windows.append(AlertnessWindow(
        start=normalize_time(morning_start),
        end=normalize_time(morning_end),
        level="good"
    ))

    # Peak zone: ~6-10 hours before DLMO (10 AM - 2 PM)
    peak_start = dlmo_hours - 10
    peak_end = dlmo_hours - 6
    windows.append(AlertnessWindow(
        start=normalize_time(peak_start),
        end=normalize_time(peak_end),
        level="peak"
    ))

    # Post-lunch dip: ~6-8 hours before DLMO (12-2 PM)
    dip_start = dlmo_hours - 8
    dip_end = dlmo_hours - 6
    windows.append(AlertnessWindow(
        start=normalize_time(dip_start),
        end=normalize_time(dip_end),
        level="moderate"
    ))

    # Second peak: ~4-6 hours before DLMO (2-4 PM)
    peak2_start = dlmo_hours - 6
    peak2_end = dlmo_hours - 4
    windows.append(AlertnessWindow(
        start=normalize_time(peak2_start),
        end=normalize_time(peak2_end),
        level="peak"
    ))

    # Evening decline: ~2-4 hours before DLMO
    evening_start = dlmo_hours - 4
    evening_end = dlmo_hours - 2
    windows.append(AlertnessWindow(
        start=normalize_time(evening_start),
        end=normalize_time(evening_end),
        level="good"
    ))

    # Low zone: ~DLMO to 2h after
    low_start = dlmo_hours
    low_end = dlmo_hours + 2
    windows.append(AlertnessWindow(
        start=normalize_time(low_start),
        end=normalize_time(low_end),
        level="low"
    ))

    return windows
```

---

## 9. Implementation Notes

### 9.1 Data Requirements

- Minimum 7 days of actigraphy for reliable rhythm assessment
- DLMO measurement requires controlled dim light conditions (<10 lux)
- Temperature measurements should be continuous core body (not skin)

### 9.2 Confidence Scoring

```python
def calculate_phase_confidence(
    data_sources: List[str],
    data_duration_days: int
) -> float:
    confidence = 0.50  # Base confidence

    if "dlmo_measured" in data_sources:
        confidence += 0.30
    elif "cbt_nadir" in data_sources:
        confidence += 0.25

    if "actigraphy" in data_sources:
        confidence += 0.15
        if data_duration_days >= 14:
            confidence += 0.05

    if "mctq" in data_sources:
        confidence += 0.10

    return min(1.0, confidence)
```

---

## 10. References

1. 선행 연구. Comparisons of the variability of three markers of the human circadian pacemaker. Journal of Biological Rhythms.
2. Borbély, A.A. (1982). A two process model of sleep regulation. Human Neurobiology.
3. Czeisler, C.A., et al. (1989). Bright light induction of strong (type 0) resetting of the human circadian pacemaker. Science.

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
