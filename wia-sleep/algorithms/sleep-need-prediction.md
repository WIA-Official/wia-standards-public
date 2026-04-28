# Sleep Need Prediction Algorithm

## WIA-SLEEP Standard Algorithm Specification

**Version:** 1.0.0
**Last Updated:** 2025-01-15

---

## 1. Overview

This algorithm predicts individual sleep need based on genetic factors, age, activity level, and historical sleep-performance data. Sleep need varies significantly between individuals (6-10 hours for adults).

---

## 2. Factors Affecting Sleep Need

| Factor | Impact | Modifiability |
|--------|--------|---------------|
| Genetics (PER3, CLOCK) | High | None |
| Age | High | None |
| Physical activity | Medium | High |
| Cognitive load | Medium | Medium |
| Health status | Medium | Variable |
| Sleep debt | Temporary | High |

---

## 3. Primary Algorithm: Multi-Factor Prediction

### 3.1 Base Model

```python
def predict_sleep_need(
    age: int,
    genetic_factors: Optional[GeneticFactors] = None,
    activity_level: str = "moderate",
    cognitive_load: str = "moderate",
    historical_data: Optional[List[SleepPerformanceRecord]] = None
) -> SleepNeedPrediction:

    # Start with age-based baseline
    baseline = get_age_baseline(age)

    # Apply genetic adjustment
    if genetic_factors:
        genetic_adjustment = calculate_genetic_adjustment(genetic_factors)
    else:
        genetic_adjustment = 0

    # Apply activity adjustment
    activity_adjustment = get_activity_adjustment(activity_level)

    # Apply cognitive load adjustment
    cognitive_adjustment = get_cognitive_adjustment(cognitive_load)

    # Calculate predicted need
    predicted_need = (
        baseline +
        genetic_adjustment +
        activity_adjustment +
        cognitive_adjustment
    )

    # Refine with historical data if available
    if historical_data and len(historical_data) >= 7:
        empirical_need = calculate_empirical_need(historical_data)
        predicted_need = (predicted_need + empirical_need) / 2

    # Calculate range
    range_min = predicted_need - 0.75
    range_max = predicted_need + 0.75

    return SleepNeedPrediction(
        predicted_need_hours=round(predicted_need, 2),
        range=(round(range_min, 2), round(range_max, 2)),
        confidence=calculate_confidence(genetic_factors, historical_data)
    )
```

### 3.2 Age-Based Baseline

```python
def get_age_baseline(age: int) -> float:
    """
    Age-based sleep need (hours per 24h)
    Based on National Sleep Foundation guidelines
    """
    if age < 1:
        return 16.0  # Infant: 14-17 hours
    elif age < 3:
        return 13.0  # Toddler: 11-14 hours
    elif age < 6:
        return 11.5  # Preschool: 10-13 hours
    elif age < 14:
        return 10.0  # School age: 9-11 hours
    elif age < 18:
        return 9.0   # Teenager: 8-10 hours
    elif age < 26:
        return 8.0   # Young adult: 7-9 hours
    elif age < 65:
        return 7.5   # Adult: 7-9 hours
    else:
        return 7.25  # Older adult: 7-8 hours
```

---

## 4. Genetic Factors

### 4.1 PER3 VNTR Polymorphism

The PER3 gene variable number tandem repeat affects sleep homeostasis:

```python
def get_per3_adjustment(per3_vntr: str) -> dict:
    """
    PER3 VNTR affects sleep need and vulnerability to sleep deprivation.
    5/5 genotype associated with higher sleep need.
    """
    adjustments = {
        "4/4": {
            "sleep_need_adjustment": -0.25,
            "sleep_deprivation_tolerance": "high",
            "description": "Lower sleep need, tolerates sleep loss better"
        },
        "4/5": {
            "sleep_need_adjustment": 0,
            "sleep_deprivation_tolerance": "moderate",
            "description": "Average sleep need"
        },
        "5/5": {
            "sleep_need_adjustment": 0.5,
            "sleep_deprivation_tolerance": "low",
            "description": "Higher sleep need, more vulnerable to sleep loss"
        }
    }
    return adjustments.get(per3_vntr, adjustments["4/5"])
```

### 4.2 CLOCK 3111T/C Polymorphism

```python
def get_clock_adjustment(clock_3111: str) -> dict:
    """
    CLOCK gene affects chronotype and possibly sleep duration.
    C allele associated with evening preference.
    """
    adjustments = {
        "T/T": {
            "chronotype_tendency": "morning",
            "sleep_need_adjustment": 0,
            "evening_alertness": "lower"
        },
        "T/C": {
            "chronotype_tendency": "intermediate",
            "sleep_need_adjustment": 0,
            "evening_alertness": "moderate"
        },
        "C/C": {
            "chronotype_tendency": "evening",
            "sleep_need_adjustment": 0.25,
            "evening_alertness": "higher"
        }
    }
    return adjustments.get(clock_3111, adjustments["T/C"])
```

### 4.3 Adenosine Sensitivity (ADA G22A)

```python
def get_adenosine_adjustment(ada_g22a: str) -> dict:
    """
    ADA gene affects adenosine metabolism (caffeine sensitivity).
    A allele associated with reduced ADA activity.
    """
    adjustments = {
        "G/G": {
            "adenosine_sensitivity": "low",
            "caffeine_metabolism": "fast",
            "sleep_need_adjustment": -0.25,
            "description": "Less sleep pressure buildup"
        },
        "G/A": {
            "adenosine_sensitivity": "medium",
            "caffeine_metabolism": "moderate",
            "sleep_need_adjustment": 0,
            "description": "Average sleep pressure"
        },
        "A/A": {
            "adenosine_sensitivity": "high",
            "caffeine_metabolism": "slow",
            "sleep_need_adjustment": 0.25,
            "description": "Higher sleep pressure, caffeine-sensitive"
        }
    }
    return adjustments.get(ada_g22a, adjustments["G/A"])
```

### 4.4 Combined Genetic Adjustment

```python
def calculate_genetic_adjustment(genetic_factors: GeneticFactors) -> float:
    total_adjustment = 0

    if genetic_factors.per3_vntr:
        per3 = get_per3_adjustment(genetic_factors.per3_vntr)
        total_adjustment += per3["sleep_need_adjustment"]

    if genetic_factors.clock_3111:
        clock = get_clock_adjustment(genetic_factors.clock_3111)
        total_adjustment += clock["sleep_need_adjustment"]

    if genetic_factors.ada_g22a:
        ada = get_adenosine_adjustment(genetic_factors.ada_g22a)
        total_adjustment += ada["sleep_need_adjustment"]

    return total_adjustment
```

---

## 5. Activity and Cognitive Load Adjustments

### 5.1 Physical Activity

```python
def get_activity_adjustment(activity_level: str) -> float:
    """
    Physical activity increases sleep need, particularly SWS.
    """
    adjustments = {
        "sedentary": -0.25,    # May need less sleep but quality suffers
        "light": 0,
        "moderate": 0.25,
        "high": 0.5,
        "athlete": 0.75        # Elite athletes often need 9-10 hours
    }
    return adjustments.get(activity_level, 0)
```

### 5.2 Cognitive Load

```python
def get_cognitive_adjustment(cognitive_load: str) -> float:
    """
    High cognitive demands increase need for REM sleep.
    """
    adjustments = {
        "low": -0.25,
        "moderate": 0,
        "high": 0.5,           # Students, knowledge workers
        "extreme": 0.75        # Learning intensive periods
    }
    return adjustments.get(cognitive_load, 0)
```

---

## 6. Empirical Need Calculation

### 6.1 From Historical Data

```python
def calculate_empirical_need(
    historical_data: List[SleepPerformanceRecord]
) -> float:
    """
    Determine sleep need from sleep-performance relationship.
    Identifies duration at which performance stabilizes.
    """
    # Group by sleep duration bins
    duration_performance = defaultdict(list)
    for record in historical_data:
        bin_key = round(record.sleep_duration_hours * 2) / 2  # 0.5h bins
        duration_performance[bin_key].append(record.performance_score)

    # Calculate mean performance per duration
    mean_performance = {
        duration: np.mean(scores)
        for duration, scores in duration_performance.items()
    }

    # Find duration at which performance plateaus
    durations = sorted(mean_performance.keys())
    performances = [mean_performance[d] for d in durations]

    # Identify knee point (where additional sleep stops helping)
    knee_index = find_knee_point(durations, performances)

    if knee_index is not None:
        return durations[knee_index]
    else:
        # Default: duration with best performance
        best_idx = performances.index(max(performances))
        return durations[best_idx]
```

### 6.2 Sleep Performance Metrics

```python
@dataclass
class SleepPerformanceRecord:
    date: date
    sleep_duration_hours: float
    sleep_efficiency_pct: float
    performance_score: float      # 0-100 (reaction time, PVT, etc.)
    subjective_alertness: int     # 1-10 scale
    mood_score: int               # 1-10 scale
```

---

## 7. Sleep Debt and Recovery

### 7.1 Sleep Debt Calculation

```python
def calculate_sleep_debt(
    recent_sleep: List[float],    # Hours slept per night
    predicted_need: float,
    recovery_rate: float = 0.15   # Fraction recovered per day
) -> float:
    """
    Calculate accumulated sleep debt over recent period.
    """
    debt = 0

    for hours_slept in recent_sleep:
        daily_deficit = predicted_need - hours_slept

        if daily_deficit > 0:
            # Accumulate debt
            debt += daily_deficit
        else:
            # Recovery (partial)
            recovery = min(-daily_deficit, debt * recovery_rate)
            debt -= recovery

    return max(0, debt)
```

### 7.2 Recovery Sleep Estimation

```python
def estimate_recovery_sleep(
    current_debt_hours: float,
    nights_available: int
) -> List[float]:
    """
    Estimate extended sleep needed to recover from debt.
    Recovery is not 1:1 - approximately 1/3 of debt recovered per hour.
    """
    RECOVERY_EFFICIENCY = 0.33

    required_extra_sleep = current_debt_hours * RECOVERY_EFFICIENCY
    extra_per_night = required_extra_sleep / nights_available

    # Cap at 2 hours extra per night (biological limit)
    extra_per_night = min(2.0, extra_per_night)

    return [extra_per_night] * nights_available
```

---

## 8. Output Format

```python
@dataclass
class SleepNeedPrediction:
    predicted_need_hours: float
    range: Tuple[float, float]    # (minimum, maximum)
    confidence: float             # 0-1
    factors: List[SleepNeedFactor]
    recommendations: List[str]

@dataclass
class SleepNeedFactor:
    factor: str                   # e.g., "PER3 5/5 genotype"
    impact: str                   # "increases" | "decreases" | "neutral"
    magnitude: str                # "small" | "moderate" | "large"
    adjustment_hours: float
```

---

## 9. Example Output

```json
{
  "predicted_need_hours": 8.25,
  "range": [7.5, 9.0],
  "confidence": 0.75,
  "factors": [
    {
      "factor": "Age (32 years)",
      "impact": "neutral",
      "magnitude": "moderate",
      "adjustment_hours": 0
    },
    {
      "factor": "PER3 5/5 genotype",
      "impact": "increases",
      "magnitude": "moderate",
      "adjustment_hours": 0.5
    },
    {
      "factor": "High physical activity",
      "impact": "increases",
      "magnitude": "moderate",
      "adjustment_hours": 0.5
    },
    {
      "factor": "Moderate cognitive load",
      "impact": "neutral",
      "magnitude": "small",
      "adjustment_hours": 0
    }
  ],
  "recommendations": [
    "Aim for 8-8.5 hours of sleep opportunity",
    "Allow for additional sleep after intense training",
    "Avoid caffeine after 14:00 due to adenosine sensitivity"
  ]
}
```

---

## 10. Implementation Notes

### 10.1 Data Collection Period

- Minimum 7 days for behavioral assessment
- 14+ days recommended for reliable empirical need calculation
- Include both workdays and free days

### 10.2 Performance Metrics

Recommended assessments for sleep-performance relationship:
- Psychomotor Vigilance Task (PVT)
- Subjective alertness (KSS or Stanford Sleepiness Scale)
- Mood assessment (POMS or similar)
- Cognitive tasks (working memory, reaction time)

### 10.3 Limitations

1. Genetic testing not always available
2. Individual variation within genotype groups
3. Environmental factors not fully captured
4. Acute stressors may temporarily alter need

---

## 11. References

2. Dijk, D.J., & Archer, S.N. (2010). PERIOD3, circadian phenotypes, and sleep homeostasis. Sleep Medicine Reviews.
3. Van 관련 분야 자료. Systematic interindividual differences in neurobehavioral impairment from sleep loss. Sleep.

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
