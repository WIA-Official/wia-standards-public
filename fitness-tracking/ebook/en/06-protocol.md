# Chapter 6: Protocol Specifications and Calculations

## Overview

This chapter provides detailed formulas, algorithms, and protocols for calculating fitness metrics according to WIA-IND-012 standards. These scientifically validated methods ensure accuracy and consistency across implementations.

---

## 6.1 Basal Metabolic Rate (BMR) Calculation

### 6.1.1 Mifflin-St Jeor Equation (Recommended)

The most accurate and widely validated BMR formula for general populations.

**For Men:**
```
BMR = (10 × weight_kg) + (6.25 × height_cm) - (5 × age_years) + 5
```

**For Women:**
```
BMR = (10 × weight_kg) + (6.25 × height_cm) - (5 × age_years) - 161
```

**Example Calculation:**
```
Male, 30 years old, 75 kg, 178 cm:
BMR = (10 × 75) + (6.25 × 178) - (5 × 30) + 5
BMR = 750 + 1,112.5 - 150 + 5
BMR = 1,717.5 kcal/day
```

**TypeScript Implementation:**
```typescript
function calculateBMR(
  weight: number,    // kg
  height: number,    // cm
  age: number,       // years
  sex: 'male' | 'female'
): number {
  const baseBMR = (10 * weight) + (6.25 * height) - (5 * age);
  return sex === 'male' ? baseBMR + 5 : baseBMR - 161;
}
```

### 6.1.2 Harris-Benedict Equation (Alternative)

Original formula, slightly less accurate but still acceptable.

**For Men:**
```
BMR = 88.362 + (13.397 × weight_kg) + (4.799 × height_cm) - (5.677 × age_years)
```

**For Women:**
```
BMR = 447.593 + (9.247 × weight_kg) + (3.098 × height_cm) - (4.330 × age_years)
```

**Example:**
```
Same individual (Male, 30, 75 kg, 178 cm):
BMR = 88.362 + (13.397 × 75) + (4.799 × 178) - (5.677 × 30)
BMR = 88.362 + 1,004.775 + 854.222 - 170.31
BMR = 1,777.05 kcal/day

Difference: 59.55 kcal/day (~3.5% higher)
```

---

## 6.2 Total Daily Energy Expenditure (TDEE)

### 6.2.1 TDEE Calculation

```
TDEE = BMR × Activity Factor
```

**Activity Factors:**
```
Sedentary (little/no exercise):
  Factor = 1.2

Lightly Active (1-3 days/week):
  Factor = 1.375

Moderately Active (3-5 days/week):
  Factor = 1.55

Very Active (6-7 days/week):
  Factor = 1.725

Extremely Active (physical job + daily training):
  Factor = 1.9
```

**Example:**
```
BMR: 1,717.5 kcal/day
Activity Level: Moderately Active (1.55)

TDEE = 1,717.5 × 1.55
TDEE = 2,662 kcal/day
```

**TypeScript Implementation:**
```typescript
enum ActivityLevel {
  SEDENTARY = 1.2,
  LIGHTLY_ACTIVE = 1.375,
  MODERATELY_ACTIVE = 1.55,
  VERY_ACTIVE = 1.725,
  EXTREMELY_ACTIVE = 1.9
}

function calculateTDEE(bmr: number, activityLevel: ActivityLevel): number {
  return bmr * activityLevel;
}
```

---

## 6.3 Activity Calorie Calculation

### 6.3.1 MET-Based Method (Default)

**Formula:**
```
Calories = MET × weight_kg × duration_hours
```

**MET Values for Common Activities:**
```
Walking (2.5 mph / 4.0 km/h):     3.0 MET
Walking (3.0 mph / 4.8 km/h):     3.5 MET
Walking (4.0 mph / 6.4 km/h):     5.0 MET

Running (5 mph / 8.0 km/h):       8.3 MET
Running (6 mph / 9.7 km/h):       9.8 MET
Running (7 mph / 11.3 km/h):     11.0 MET
Running (8 mph / 12.9 km/h):     11.8 MET

Cycling (10-12 mph):              6.8 MET
Cycling (12-14 mph):              8.0 MET
Cycling (14-16 mph):             10.0 MET

Swimming (moderate):              7.0 MET
Weight training (moderate):       5.0 MET
HIIT training:                    8.0 MET
Yoga:                             3.0 MET
```

**Example Calculation:**
```
Activity: Running at 6 mph
MET: 9.8
Weight: 75 kg
Duration: 30 minutes (0.5 hours)

Calories = 9.8 × 75 × 0.5
Calories = 367.5 kcal
```

**TypeScript Implementation:**
```typescript
function calculateCaloriesMET(
  met: number,
  weightKg: number,
  durationMinutes: number
): number {
  const hours = durationMinutes / 60;
  return met * weightKg * hours;
}
```

### 6.3.2 Heart Rate-Based Method

More accurate when heart rate data is available.

**For Men:**
```
Calories/min = ((Age × 0.2017) - (Weight_kg × 0.09036) + (HR × 0.6309) - 55.0969) × Duration_min / 4.184
```

**For Women:**
```
Calories/min = ((Age × 0.074) - (Weight_kg × 0.05741) + (HR × 0.4472) - 20.4022) × Duration_min / 4.184
```

**Example:**
```
Male, 30 years, 75 kg, Average HR 155 BPM, 30 minutes

Calories/min = ((30 × 0.2017) - (75 × 0.09036) + (155 × 0.6309) - 55.0969) / 4.184
             = (6.051 - 6.777 + 97.7895 - 55.0969) / 4.184
             = 41.9666 / 4.184
             = 10.03 kcal/min

Total = 10.03 × 30 = 300.9 kcal
```

**TypeScript Implementation:**
```typescript
function calculateCaloriesHeartRate(
  age: number,
  weightKg: number,
  avgHeartRate: number,
  durationMinutes: number,
  sex: 'male' | 'female'
): number {
  let caloriesPerMin: number;

  if (sex === 'male') {
    caloriesPerMin = (
      (age * 0.2017) -
      (weightKg * 0.09036) +
      (avgHeartRate * 0.6309) -
      55.0969
    ) / 4.184;
  } else {
    caloriesPerMin = (
      (age * 0.074) -
      (weightKg * 0.05741) +
      (avgHeartRate * 0.4472) -
      20.4022
    ) / 4.184;
  }

  return caloriesPerMin * durationMinutes;
}
```

### 6.3.3 VO2-Based Method (Most Accurate)

When VO2 max is known and heart rate is tracked.

**Formula:**
```
VO2 (ml/kg/min) = (HR_avg / HR_max) × VO2_max
Calories/min = (VO2 × weight_kg × 5) / 1000
Total Calories = Calories/min × duration_minutes
```

**Example:**
```
VO2 max: 50 ml/kg/min
Max HR: 190 BPM
Avg HR during exercise: 155 BPM
Weight: 75 kg
Duration: 30 minutes

VO2 = (155 / 190) × 50
    = 0.8158 × 50
    = 40.79 ml/kg/min

Calories/min = (40.79 × 75 × 5) / 1000
             = 15,296.25 / 1000
             = 15.30 kcal/min

Total = 15.30 × 30 = 459 kcal
```

**TypeScript Implementation:**
```typescript
function calculateCaloriesVO2(
  vo2Max: number,
  maxHR: number,
  avgHR: number,
  weightKg: number,
  durationMinutes: number
): number {
  const vo2 = (avgHR / maxHR) * vo2Max;
  const caloriesPerMin = (vo2 * weightKg * 5) / 1000;
  return caloriesPerMin * durationMinutes;
}
```

---

## 6.4 EPOC (Excess Post-Exercise Oxygen Consumption)

### 6.4.1 Afterburn Calculation

**Formula:**
```
EPOC_Calories = Base_Calories × EPOC_Factor
Total_Calories = Base_Calories + EPOC_Calories
```

**EPOC Factors by Intensity:**
```
Low Intensity (< 50% VO2 max):
  Factor = 1.05 (5% additional calories)

Moderate Intensity (50-75% VO2 max):
  Factor = 1.10 (10% additional calories)

High Intensity (> 75% VO2 max):
  Factor = 1.15 (15% additional calories)

HIIT / Strength Training:
  Factor = 1.20-1.25 (20-25% additional calories)
```

**Example:**
```
Running workout: 400 kcal (base)
Intensity: High (80% VO2 max)
EPOC Factor: 1.15

EPOC Calories = 400 × 0.15 = 60 kcal
Total = 400 + 60 = 460 kcal
```

---

## 6.5 Heart Rate Zone Calculation

### 6.5.1 Maximum Heart Rate Estimation

**Traditional Formula (Fox):**
```
Max_HR = 220 - Age
```

**Tanaka Formula (More Accurate):**
```
Max_HR = 208 - (0.7 × Age)
```

**Gulati Formula (For Women):**
```
Max_HR = 206 - (0.88 × Age)
```

**Example:**
```
30-year-old male:

Fox:    220 - 30 = 190 BPM
Tanaka: 208 - (0.7 × 30) = 208 - 21 = 187 BPM
Difference: 3 BPM
```

### 6.5.2 Heart Rate Reserve (Karvonen) Method

**More accurate than simple percentage method.**

**Formula:**
```
HR_Reserve = Max_HR - Resting_HR
Target_HR = Resting_HR + (HR_Reserve × Intensity_Percentage)
```

**Zone Definitions:**
```
Zone 1 (Recovery):     50-60% of HR Reserve
Zone 2 (Aerobic):      60-70% of HR Reserve
Zone 3 (Tempo):        70-80% of HR Reserve
Zone 4 (Threshold):    80-90% of HR Reserve
Zone 5 (Maximum):      90-100% of HR Reserve
```

**Example:**
```
Max HR: 187 BPM
Resting HR: 52 BPM
HR Reserve: 187 - 52 = 135 BPM

Zone 1 (50-60%):
  Lower: 52 + (135 × 0.50) = 119.5 BPM
  Upper: 52 + (135 × 0.60) = 133 BPM

Zone 2 (60-70%):
  Lower: 52 + (135 × 0.60) = 133 BPM
  Upper: 52 + (135 × 0.70) = 146.5 BPM

Zone 3 (70-80%):
  Lower: 146.5 BPM
  Upper: 160 BPM

Zone 4 (80-90%):
  Lower: 160 BPM
  Upper: 173.5 BPM

Zone 5 (90-100%):
  Lower: 173.5 BPM
  Upper: 187 BPM
```

**TypeScript Implementation:**
```typescript
interface HeartRateZone {
  name: string;
  lower: number;
  upper: number;
  intensity: string;
}

function calculateHeartRateZones(
  maxHR: number,
  restingHR: number
): HeartRateZone[] {
  const hrReserve = maxHR - restingHR;

  return [
    {
      name: 'Zone 1',
      lower: Math.round(restingHR + (hrReserve * 0.50)),
      upper: Math.round(restingHR + (hrReserve * 0.60)),
      intensity: 'Recovery'
    },
    {
      name: 'Zone 2',
      lower: Math.round(restingHR + (hrReserve * 0.60)),
      upper: Math.round(restingHR + (hrReserve * 0.70)),
      intensity: 'Aerobic'
    },
    {
      name: 'Zone 3',
      lower: Math.round(restingHR + (hrReserve * 0.70)),
      upper: Math.round(restingHR + (hrReserve * 0.80)),
      intensity: 'Tempo'
    },
    {
      name: 'Zone 4',
      lower: Math.round(restingHR + (hrReserve * 0.80)),
      upper: Math.round(restingHR + (hrReserve * 0.90)),
      intensity: 'Threshold'
    },
    {
      name: 'Zone 5',
      lower: Math.round(restingHR + (hrReserve * 0.90)),
      upper: maxHR,
      intensity: 'Maximum'
    }
  ];
}
```

---

## 6.6 Heart Rate Variability (HRV) Calculation

### 6.6.1 RMSSD (Root Mean Square of Successive Differences)

**Formula:**
```
RMSSD = √(Σ(RR[i+1] - RR[i])² / (N-1))
```

Where RR[i] are R-R intervals in milliseconds.

**Example:**
```
RR intervals (ms): [820, 840, 835, 845, 830, 850, 838, 842]

Successive differences:
840-820 = 20
835-840 = -5
845-835 = 10
830-845 = -15
850-830 = 20
838-850 = -12
842-838 = 4

Squared differences:
400, 25, 100, 225, 400, 144, 16

Sum = 1,310
RMSSD = √(1,310 / 7) = √187.14 = 13.68 ms
```

**TypeScript Implementation:**
```typescript
function calculateRMSSD(rrIntervals: number[]): number {
  if (rrIntervals.length < 2) return 0;

  let sumSquaredDiffs = 0;
  for (let i = 0; i < rrIntervals.length - 1; i++) {
    const diff = rrIntervals[i + 1] - rrIntervals[i];
    sumSquaredDiffs += diff * diff;
  }

  return Math.sqrt(sumSquaredDiffs / (rrIntervals.length - 1));
}
```

### 6.6.2 SDNN (Standard Deviation of NN Intervals)

**Formula:**
```
SDNN = √(Σ(RR[i] - mean_RR)² / (N-1))
```

**TypeScript Implementation:**
```typescript
function calculateSDNN(rrIntervals: number[]): number {
  if (rrIntervals.length < 2) return 0;

  const mean = rrIntervals.reduce((a, b) => a + b) / rrIntervals.length;
  const sumSquaredDiffs = rrIntervals.reduce(
    (sum, rr) => sum + Math.pow(rr - mean, 2),
    0
  );

  return Math.sqrt(sumSquaredDiffs / (rrIntervals.length - 1));
}
```

---

## 6.7 VO2 Max Estimation

### 6.7.1 Cooper Test Method

**Formula:**
```
VO2_max (ml/kg/min) = (Distance_meters - 504.9) / 44.73
```

Where distance is maximum distance covered in 12 minutes.

**Example:**
```
Distance in 12 minutes: 3,000 meters

VO2_max = (3,000 - 504.9) / 44.73
        = 2,495.1 / 44.73
        = 55.8 ml/kg/min
```

### 6.7.2 Heart Rate-Based Estimation

**Formula:**
```
VO2_max = 15.3 × (Max_HR / Resting_HR)
```

**With Age Adjustment:**
```
Age_factor:
  20-29: 1.0
  30-39: 0.93
  40-49: 0.83
  50-59: 0.74
  60+:   0.65

VO2_max = 15 × (Max_HR / Resting_HR) × Age_factor
```

**Example:**
```
Max HR: 187 BPM
Resting HR: 52 BPM
Age: 30 (factor = 0.93)

VO2_max = 15 × (187 / 52) × 0.93
        = 15 × 3.596 × 0.93
        = 50.2 ml/kg/min
```

---

## 6.8 Training Load Calculation

### 6.8.1 TRIMP (Training Impulse)

**Formula:**
```
TRIMP = Duration_minutes × HR_ratio × e^(k × HR_ratio)

Where:
HR_ratio = (HR_avg - HR_rest) / (HR_max - HR_rest)
k = 1.92 (men), 1.67 (women)
```

**Example:**
```
Male athlete:
Max HR: 187 BPM
Resting HR: 52 BPM
Avg HR during workout: 155 BPM
Duration: 45 minutes

HR_ratio = (155 - 52) / (187 - 52)
         = 103 / 135
         = 0.763

TRIMP = 45 × 0.763 × e^(1.92 × 0.763)
      = 45 × 0.763 × e^1.465
      = 45 × 0.763 × 4.327
      = 148.4
```

**TypeScript Implementation:**
```typescript
function calculateTRIMP(
  durationMinutes: number,
  avgHR: number,
  restingHR: number,
  maxHR: number,
  sex: 'male' | 'female'
): number {
  const hrRatio = (avgHR - restingHR) / (maxHR - restingHR);
  const k = sex === 'male' ? 1.92 : 1.67;

  return durationMinutes * hrRatio * Math.exp(k * hrRatio);
}
```

### 6.8.2 Training Stress Score (TSS)

**Power-Based (for cycling):**
```
TSS = (Duration_seconds × NP × IF) / (FTP × 3600) × 100

Where:
NP = Normalized Power
IF = Intensity Factor = NP / FTP
FTP = Functional Threshold Power
```

**Heart Rate-Based (alternative):**
```
TSS = (Duration_seconds × HR_ratio²) / 36

Where HR_ratio as defined in TRIMP
```

**Example (HR-based):**
```
Duration: 45 minutes = 2,700 seconds
HR_ratio: 0.763 (from previous example)

TSS = (2,700 × 0.763²) / 36
    = (2,700 × 0.582) / 36
    = 1,571.4 / 36
    = 43.7
```

**TypeScript Implementation:**
```typescript
function calculateTSS(
  durationSeconds: number,
  hrRatio: number
): number {
  return (durationSeconds * Math.pow(hrRatio, 2)) / 36;
}
```

---

## 6.9 Recovery Metrics

### 6.9.1 Recovery Heart Rate

**Formula:**
```
Recovery_HR = HR_at_exercise_end - HR_1_minute_later
```

**Interpretation:**
```
Excellent:  > 25 BPM drop
Good:       15-25 BPM drop
Fair:       10-15 BPM drop
Poor:       < 10 BPM drop
```

### 6.9.2 Recovery Score

**Composite Formula:**
```
Recovery_Score = (HRV_score × 0.4) + (Sleep_score × 0.3) + (RHR_score × 0.3)
```

**Component Scoring (0-100):**
```
HRV_score:
  Based on RMSSD deviation from baseline
  +1 SD: 100
  Baseline: 75
  -1 SD: 50
  -2 SD: 25

Sleep_score:
  Based on duration and quality
  8+ hours, 85+ quality: 100
  7-8 hours, 75+ quality: 75
  6-7 hours, 65+ quality: 50
  < 6 hours: 25

RHR_score:
  Based on deviation from baseline
  -5 BPM: 100
  Baseline: 75
  +3 BPM: 50
  +5 BPM: 25
```

---

## Key Takeaways

✓ BMR calculated using Mifflin-St Jeor equation (most accurate)

✓ TDEE derived from BMR multiplied by activity factor

✓ Calorie calculation supports MET, heart rate, and VO2 methods

✓ EPOC adds 5-25% afterburn depending on intensity

✓ Heart rate zones use Karvonen method with HR reserve

✓ HRV metrics (RMSSD, SDNN) indicate recovery status

✓ VO2 max estimated via Cooper test or heart rate ratio

✓ Training load quantified using TRIMP or TSS formulas

✓ Recovery score combines HRV, sleep, and resting heart rate

✓ All formulas validated by scientific research

---

**Next:** [Chapter 7: System Integration →](07-system-integration.md)

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
