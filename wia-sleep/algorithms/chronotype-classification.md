# Chronotype Classification Algorithm

## WIA-SLEEP Standard Algorithm Specification

**Version:** 1.0.0
**Last Updated:** 2025-01-15

---

## 1. Overview

This algorithm classifies individuals into chronotype categories based on the Munich Chronotype Questionnaire (MCTQ) methodology and supplementary biological markers.

### 1.1 Chronotype Categories

| Category | MSFsc Range | Description |
|----------|-------------|-------------|
| Extreme Early | < 2.5 hours | Strong morning preference ("larks") |
| Moderate Early | 2.5 - 3.5 hours | Morning-oriented |
| Intermediate | 3.5 - 5.0 hours | Neither type |
| Moderate Late | 5.0 - 6.5 hours | Evening-oriented |
| Extreme Late | > 6.5 hours | Strong evening preference ("owls") |

---

## 2. Primary Algorithm: MCTQ-Based Classification

### 2.1 Input Data

```typescript
interface MCTQInput {
  workdays: {
    bedtime: Time;           // Time going to bed
    sleepPrepTime_min: number; // Minutes before attempting sleep
    sleepLatency_min: number;  // Minutes to fall asleep
    wakeTime: Time;          // Wake time
    sleepDuration_hours: number;
  };
  freeDays: {
    bedtime: Time;
    sleepPrepTime_min: number;
    sleepLatency_min: number;
    wakeTime: Time;
    sleepDuration_hours: number;
  };
  workDaysPerWeek: number;     // Typically 5
  alarmUsageWorkdays: boolean;
  alarmUsageFreeDays: boolean;
}
```

### 2.2 Step 1: Calculate Sleep Onset

```
SleepOnset = Bedtime + SleepPrepTime + SleepLatency
```

For times crossing midnight, convert to decimal hours (e.g., 01:00 = 25.0).

### 2.3 Step 2: Calculate Mid-Sleep Points

```
MSW = SleepOnset_workdays + (SleepDuration_workdays / 2)
MSF = SleepOnset_freedays + (SleepDuration_freedays / 2)
```

### 2.4 Step 3: Calculate Average Sleep Duration

```
SDweek = (SDw × n_workdays + SDf × n_freedays) / 7
```

Where:
- SDw = Sleep duration on workdays
- SDf = Sleep duration on free days
- n_workdays = Number of work days (typically 5)
- n_freedays = Number of free days (typically 2)

### 2.5 Step 4: Calculate Sleep Debt

```
if SDf > SDw:
    SleepDebt = SDweek - SDw
else:
    SleepDebt = 0
```

### 2.6 Step 5: Calculate Corrected Mid-Sleep on Free Days (MSFsc)

```
if SDf > SDweek AND alarmUsageFreeDays == false:
    MSFsc = MSF - (SleepDebt / 2)
else:
    MSFsc = MSF
```

**Normalization:**
```
if MSFsc > 24:
    MSFsc = MSFsc - 24
```

### 2.7 Step 6: Classify Chronotype

```python
def classify_chronotype(msfsc: float) -> str:
    if msfsc < 2.5:
        return "extreme_early"
    elif msfsc < 3.5:
        return "moderate_early"
    elif msfsc < 5.0:
        return "intermediate"
    elif msfsc < 6.5:
        return "moderate_late"
    else:
        return "extreme_late"
```

---

## 3. Secondary Algorithm: Social Jetlag Calculation

### 3.1 Definition

Social jetlag quantifies the discrepancy between biological and social clocks.

### 3.2 Calculation

```
SJL = |MSF - MSW|
```

### 3.3 Interpretation

| Social Jetlag | Interpretation |
|---------------|----------------|
| < 1 hour | Minimal - well-aligned |
| 1-2 hours | Moderate - some misalignment |
| > 2 hours | Significant - health risk factor |

---

## 4. Supplementary: Biological Marker Enhancement

### 4.1 DLMO-Based Confirmation

When Dim Light Melatonin Onset (DLMO) is available:

```
DLMO_expected = Bedtime_habitual - 2.5 hours

Chronotype_biological = classify_by_dlmo(DLMO_measured)

def classify_by_dlmo(dlmo: Time) -> str:
    dlmo_hours = to_decimal_hours(dlmo)
    if dlmo_hours < 19.0:
        return "extreme_early"
    elif dlmo_hours < 20.0:
        return "moderate_early"
    elif dlmo_hours < 21.5:
        return "intermediate"
    elif dlmo_hours < 23.0:
        return "moderate_late"
    else:
        return "extreme_late"
```

### 4.2 Confidence Score

```python
def calculate_confidence(
    mctq_result: str,
    dlmo_result: str | None,
    actigraphy_data: bool
) -> float:
    confidence = 0.70  # Base MCTQ confidence

    if actigraphy_data:
        confidence += 0.15

    if dlmo_result:
        confidence += 0.15
        if dlmo_result != mctq_result:
            confidence -= 0.10  # Discrepancy penalty

    return min(1.0, confidence)
```

---

## 5. Optimal Sleep Window Calculation

### 5.1 Algorithm

```python
def calculate_optimal_window(msfsc: float, sleep_need: float) -> TimeWindow:
    optimal_midpoint = msfsc
    half_sleep = sleep_need / 2

    optimal_bedtime = optimal_midpoint - half_sleep
    optimal_wake = optimal_midpoint + half_sleep

    # Normalize to 24-hour clock
    if optimal_bedtime > 24:
        optimal_bedtime -= 24
    if optimal_bedtime < 0:
        optimal_bedtime += 24

    return TimeWindow(
        bedtime=decimal_to_time(optimal_bedtime),
        wake_time=decimal_to_time(optimal_wake)
    )
```

### 5.2 Individual Sleep Need Estimation

Default to population average (7.5-8 hours) unless individual data available:

```python
def estimate_sleep_need(age: int, historical_data: list | None) -> float:
    # Age-based baseline
    if age < 18:
        baseline = 9.0
    elif age < 65:
        baseline = 8.0
    else:
        baseline = 7.5

    # Adjust based on historical data if available
    if historical_data:
        observed_need = calculate_observed_need(historical_data)
        return (baseline + observed_need) / 2

    return baseline
```

---

## 6. Implementation Notes

### 6.1 Time Handling

- Always convert times to decimal hours for calculations
- Handle midnight crossing (add 24 to early morning times)
- Store results in ISO 8601 format (HH:MM:SS)

### 6.2 Edge Cases

1. **Shift workers**: MCTQ may not apply; use actigraphy-based assessment
2. **Irregular schedules**: Require minimum 7 days of data
3. **Alarm on free days**: Consider MSF uncorrected if alarm used

### 6.3 Validation

- MSFsc should be between 0 and 12 hours
- Sleep duration should be between 3 and 14 hours
- Reject data with impossible time combinations

---

## 7. References

1. 선행 연구. Life between clocks: daily temporal patterns of human chronotypes. Journal of Biological Rhythms.
2. 선행 연구. Chronotype and Social Jetlag: A (Self-) Critical Review. Biology.
3. 선행 연구. The Munich ChronoType Questionnaire (MCTQ). Journal of Biological Rhythms.

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
