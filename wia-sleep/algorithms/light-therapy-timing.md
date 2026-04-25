# Light Therapy Timing Algorithm

## WIA-SLEEP Standard Algorithm Specification

**Version:** 1.0.0
**Last Updated:** 2025-01-15

---

## 1. Overview

This algorithm determines optimal timing, intensity, and duration for light therapy interventions to achieve desired circadian phase shifts. Light is the most powerful zeitgeber (time-giver) for the human circadian system.

---

## 2. Phase Response Curve (PRC)

### 2.1 Light PRC Characteristics

| Timing (relative to CBT nadir) | Effect | Magnitude |
|--------------------------------|--------|-----------|
| 0-3 hours after | Phase advance | Maximum |
| 3-6 hours after | Phase advance | Moderate |
| 6-9 hours after | Minimal effect | Dead zone |
| 9-12 hours before | Minimal effect | Dead zone |
| 6-9 hours before | Phase delay | Moderate |
| 3-6 hours before | Phase delay | Maximum |
| 0-3 hours before | Phase delay | Moderate |

### 2.2 PRC Implementation

```python
def calculate_light_prc(
    timing_relative_to_cbt_nadir: float  # hours
) -> float:
    """
    Returns phase response in hours.
    Negative = advance, Positive = delay
    """
    t = timing_relative_to_cbt_nadir

    # Normalize to -12 to +12 range
    while t > 12:
        t -= 24
    while t < -12:
        t += 24

    # Maximum phase shift magnitude
    MAX_SHIFT = 2.5  # hours per day (theoretical max ~3h)

    # PRC function (simplified sinusoidal approximation)
    if -6 <= t < 0:
        # Delay zone (before CBT nadir)
        # Peak delay at t = -3 hours
        phase_shift = MAX_SHIFT * sin((t + 3) / 3 * pi / 2)
    elif 0 <= t <= 6:
        # Advance zone (after CBT nadir)
        # Peak advance at t = +3 hours
        phase_shift = -MAX_SHIFT * sin(t / 3 * pi / 2)
    else:
        # Dead zone (-12 to -6, +6 to +12)
        phase_shift = 0

    return phase_shift
```

---

## 3. Light Dose Parameters

### 3.1 Intensity Effects

```python
def calculate_intensity_factor(
    illuminance_lux: float,
    melanopic_edi: Optional[float] = None
) -> float:
    """
    Circadian response shows logarithmic relationship with intensity.
    Saturation begins around 1000-2000 lux.
    Full response at ~10,000 lux.
    """
    # Use melanopic EDI if available (more accurate)
    effective_intensity = melanopic_edi if melanopic_edi else illuminance_lux

    # Logarithmic dose-response
    # 50% response at ~500 lux
    # 90% response at ~5000 lux
    # Near 100% at ~10000 lux

    if effective_intensity <= 0:
        return 0

    # Weber-Fechner relationship
    half_saturation = 500  # lux for 50% response

    factor = effective_intensity / (effective_intensity + half_saturation)

    return min(1.0, factor)
```

### 3.2 Duration Effects

```python
def calculate_duration_factor(duration_min: float) -> float:
    """
    Phase shift increases with duration but with diminishing returns.
    Near-maximum effect achieved at 30-60 minutes.
    """
    # Time constant for duration response
    tau = 30  # minutes for 63% of maximum effect

    factor = 1 - exp(-duration_min / tau)

    return min(1.0, factor)
```

### 3.3 Spectrum Effects

```python
def calculate_spectrum_factor(
    color_temperature_K: int,
    spectrum_type: str = "broad"
) -> float:
    """
    Blue-enriched light (higher color temperature) more effective
    for circadian entrainment due to melanopsin sensitivity.
    """
    spectrum_factors = {
        "red": 0.2,           # 600-700nm, minimal circadian effect
        "green": 0.6,         # 500-560nm, moderate effect
        "blue_depleted": 0.5, # Reduced 460-480nm
        "broad": 0.7,         # Standard white light
        "blue_enriched": 0.9  # Enhanced 460-480nm
    }

    base_factor = spectrum_factors.get(spectrum_type, 0.7)

    # Color temperature adjustment
    if color_temperature_K >= 6500:
        ct_adjustment = 1.0
    elif color_temperature_K >= 4000:
        ct_adjustment = 0.7 + 0.3 * (color_temperature_K - 4000) / 2500
    elif color_temperature_K >= 2700:
        ct_adjustment = 0.5 + 0.2 * (color_temperature_K - 2700) / 1300
    else:
        ct_adjustment = 0.5

    return base_factor * ct_adjustment
```

---

## 4. Combined Phase Shift Calculation

```python
def calculate_expected_phase_shift(
    timing_relative_to_cbt_nadir: float,  # hours
    intensity_lux: float,
    duration_min: float,
    color_temperature_K: int = 5000,
    melanopic_edi: Optional[float] = None
) -> float:
    """
    Calculate expected phase shift from a single light exposure.
    Returns shift in hours (negative = advance, positive = delay).
    """
    # Base phase response from timing
    base_shift = calculate_light_prc(timing_relative_to_cbt_nadir)

    # Apply modifying factors
    intensity_factor = calculate_intensity_factor(intensity_lux, melanopic_edi)
    duration_factor = calculate_duration_factor(duration_min)
    spectrum_factor = calculate_spectrum_factor(color_temperature_K)

    # Combined shift
    expected_shift = base_shift * intensity_factor * duration_factor * spectrum_factor

    # Practical limit per day
    MAX_DAILY_SHIFT = 2.0  # hours
    expected_shift = max(-MAX_DAILY_SHIFT, min(MAX_DAILY_SHIFT, expected_shift))

    return round(expected_shift, 2)
```

---

## 5. Prescription Generation

### 5.1 Phase Advance Protocol

```python
def generate_phase_advance_prescription(
    target_advance_hours: float,
    current_cbt_nadir: Time,          # Estimated CBT nadir
    constraints: Optional[LightConstraints] = None
) -> LightPrescription:
    """
    Generate light therapy prescription for phase advance.
    """
    # Default constraints
    if constraints is None:
        constraints = LightConstraints(
            earliest_start="06:00",
            latest_end="12:00",
            max_duration_min=45,
            max_intensity_lux=10000
        )

    # Optimal timing: 0-3 hours after CBT nadir
    cbt_hours = time_to_decimal(current_cbt_nadir)
    optimal_start = cbt_hours + 1  # 1 hour after CBT nadir

    # Adjust for constraints
    earliest_hours = time_to_decimal(constraints.earliest_start)
    if optimal_start < earliest_hours:
        optimal_start = earliest_hours

    # Calculate parameters for target shift
    # Assuming ~0.5-1.0 hours shift per day at optimal conditions
    days_needed = ceil(target_advance_hours / 0.75)

    sessions = []
    sessions.append(LightSession(
        timing=decimal_to_time(optimal_start),
        duration_min=min(constraints.max_duration_min, 30),
        intensity_lux=min(constraints.max_intensity_lux, 10000),
        color_temperature_K=6500,  # Blue-enriched for advance
        relative_to="wake",
        offset_minutes=0
    ))

    # Light avoidance in evening
    avoid_start = cbt_hours - 6  # 6 hours before CBT nadir
    avoid_end = cbt_hours + 1    # Until CBT nadir + 1 hour

    return LightPrescription(
        goal="phase_advance",
        target_shift_hours=-target_advance_hours,
        sessions=sessions,
        avoidance_window=(
            decimal_to_time(avoid_start),
            decimal_to_time(avoid_end)
        ),
        duration_days=days_needed,
        expected_shift_per_day=-0.75,
        notes=[
            "Expose eyes to light box at specified time",
            "Position light box at 45-60cm from face",
            "Light should enter from above eye level",
            "Dim lights and avoid screens after avoidance start time"
        ]
    )
```

### 5.2 Phase Delay Protocol

```python
def generate_phase_delay_prescription(
    target_delay_hours: float,
    current_cbt_nadir: Time,
    constraints: Optional[LightConstraints] = None
) -> LightPrescription:
    """
    Generate light therapy prescription for phase delay.
    """
    if constraints is None:
        constraints = LightConstraints(
            earliest_start="18:00",
            latest_end="23:00",
            max_duration_min=45,
            max_intensity_lux=10000
        )

    # Optimal timing: 3-6 hours before CBT nadir
    cbt_hours = time_to_decimal(current_cbt_nadir)
    optimal_start = cbt_hours - 4  # 4 hours before CBT nadir

    # Normalize for evening
    if optimal_start < 0:
        optimal_start += 24

    # Adjust for constraints
    latest_hours = time_to_decimal(constraints.latest_end)
    if optimal_start > latest_hours:
        optimal_start = latest_hours - 0.5

    days_needed = ceil(target_delay_hours / 0.75)

    sessions = []
    sessions.append(LightSession(
        timing=decimal_to_time(optimal_start),
        duration_min=min(constraints.max_duration_min, 30),
        intensity_lux=min(constraints.max_intensity_lux, 10000),
        color_temperature_K=6500,
        relative_to="dlmo",
        offset_minutes=-120  # 2 hours before DLMO
    ))

    # Morning light avoidance
    avoid_start = cbt_hours + 1
    avoid_end = cbt_hours + 4

    return LightPrescription(
        goal="phase_delay",
        target_shift_hours=target_delay_hours,
        sessions=sessions,
        avoidance_window=(
            decimal_to_time(avoid_start),
            decimal_to_time(avoid_end)
        ),
        morning_blue_blockers=True,
        duration_days=days_needed,
        expected_shift_per_day=0.75,
        notes=[
            "Use bright light in evening before bed",
            "Wear blue-blocking glasses in morning",
            "Delay wake time if possible during treatment"
        ]
    )
```

---

## 6. Practical Considerations

### 6.1 Light Source Recommendations

| Source Type | Typical Lux | Melanopic EDI | Notes |
|-------------|-------------|---------------|-------|
| Light therapy box (10K) | 10,000 | 200-300 | Gold standard |
| Light therapy box (2.5K) | 2,500 | 50-75 | Portable option |
| Natural daylight (outdoor) | 10,000-100,000 | 200-1000 | Best but variable |
| Bright indoor lighting | 300-500 | 50-100 | Often insufficient |
| Computer screen | 50-100 | 20-50 | Evening avoidance |

### 6.2 Safety Considerations

```python
def check_light_therapy_contraindications(
    patient_conditions: List[str]
) -> List[Contraindication]:
    """
    Check for conditions that require caution with light therapy.
    """
    contraindications = []

    serious_conditions = {
        "bipolar_disorder": {
            "severity": "high",
            "action": "Consult psychiatrist before starting; may trigger mania"
        },
        "retinal_disease": {
            "severity": "high",
            "action": "Consult ophthalmologist; may worsen condition"
        },
        "macular_degeneration": {
            "severity": "high",
            "action": "Contraindicated; consult ophthalmologist"
        }
    }

    caution_conditions = {
        "photosensitive_medication": {
            "severity": "medium",
            "action": "Reduce intensity or duration; monitor for skin reaction"
        },
        "migraine": {
            "severity": "medium",
            "action": "Start with lower intensity; bright light may trigger"
        },
        "cataracts": {
            "severity": "low",
            "action": "Generally safe; consult ophthalmologist if concerns"
        }
    }

    for condition in patient_conditions:
        if condition in serious_conditions:
            contraindications.append(Contraindication(
                condition=condition,
                **serious_conditions[condition]
            ))
        elif condition in caution_conditions:
            contraindications.append(Contraindication(
                condition=condition,
                **caution_conditions[condition]
            ))

    return contraindications
```

### 6.3 Gradual Phase Shift

```python
def generate_gradual_shift_protocol(
    target_shift_hours: float,
    direction: str,  # "advance" or "delay"
    max_shift_per_day: float = 0.5,  # Conservative approach
    current_schedule: SleepSchedule
) -> List[DailyProtocol]:
    """
    Generate day-by-day protocol for gradual phase shift.
    More conservative than maximum possible shifts.
    """
    days_needed = ceil(abs(target_shift_hours) / max_shift_per_day)
    shift_per_day = target_shift_hours / days_needed

    protocols = []
    cumulative_shift = 0

    for day in range(days_needed):
        daily_shift = shift_per_day
        cumulative_shift += daily_shift

        # Adjust light timing based on current phase
        if direction == "advance":
            light_time = current_schedule.wake_time  # Wake + light
            adjusted_bedtime = shift_time(current_schedule.bedtime, -daily_shift)
            adjusted_wake = shift_time(current_schedule.wake_time, -daily_shift)
        else:
            light_time = shift_time(current_schedule.bedtime, -2)  # 2h before bed
            adjusted_bedtime = shift_time(current_schedule.bedtime, daily_shift)
            adjusted_wake = shift_time(current_schedule.wake_time, daily_shift)

        protocols.append(DailyProtocol(
            day=day + 1,
            light_session=LightSession(
                timing=light_time,
                duration_min=30,
                intensity_lux=10000
            ),
            target_bedtime=adjusted_bedtime,
            target_wake_time=adjusted_wake,
            cumulative_shift=cumulative_shift
        ))

        # Update schedule for next day
        current_schedule.bedtime = adjusted_bedtime
        current_schedule.wake_time = adjusted_wake

    return protocols
```

---

## 7. Monitoring and Adjustment

### 7.1 Treatment Response Assessment

```python
def assess_treatment_response(
    baseline_metrics: SleepMetrics,
    current_metrics: SleepMetrics,
    treatment_days: int
) -> TreatmentResponse:
    """
    Assess response to light therapy treatment.
    """
    # Calculate phase shift achieved
    phase_shift = calculate_phase_shift(
        baseline_metrics.sleep_midpoint,
        current_metrics.sleep_midpoint
    )

    # Expected vs actual
    expected_shift = treatment_days * 0.5  # Conservative expectation
    shift_ratio = abs(phase_shift) / expected_shift if expected_shift > 0 else 0

    # Assess sleep quality changes
    efficiency_change = current_metrics.efficiency - baseline_metrics.efficiency
    latency_change = baseline_metrics.latency - current_metrics.latency

    if shift_ratio >= 0.8 and efficiency_change >= 0:
        response = "good"
        recommendation = "Continue current protocol"
    elif shift_ratio >= 0.5:
        response = "partial"
        recommendation = "Consider increasing intensity or duration"
    else:
        response = "poor"
        recommendation = "Review timing; consider DLMO assessment"

    return TreatmentResponse(
        response_level=response,
        phase_shift_achieved=phase_shift,
        expected_shift=expected_shift,
        efficiency_change=efficiency_change,
        latency_change=latency_change,
        recommendation=recommendation
    )
```

---

## 8. References

1. St 선행 연구. Human phase response curve to a 1 h pulse of bright white light. Journal of Physiology.
2. 선행 연구. A phase response curve to single bright light pulses in human subjects. Journal of Physiology.
3. 선행 연구. Human responses to bright light of different durations. Journal of Physiology.

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
