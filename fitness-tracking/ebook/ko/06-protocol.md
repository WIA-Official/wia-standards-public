# 6장: 프로토콜 사양 및 계산

## 개요

이 장은 WIA-IND-012 표준에 따라 피트니스 지표를 계산하기 위한 상세한 공식, 알고리즘 및 프로토콜을 제공합니다.

---

## 6.1 기초 대사율 (BMR) 계산

### 6.1.1 Mifflin-St Jeor 공식 (권장)

일반 인구에 대해 가장 정확하고 널리 검증된 BMR 공식입니다.

**남성:**
```
BMR = (10 × 체중_kg) + (6.25 × 키_cm) - (5 × 나이) + 5
```

**여성:**
```
BMR = (10 × 체중_kg) + (6.25 × 키_cm) - (5 × 나이) - 161
```

**예시 계산:**
```
남성, 30세, 75kg, 178cm:
BMR = (10 × 75) + (6.25 × 178) - (5 × 30) + 5
BMR = 750 + 1,112.5 - 150 + 5
BMR = 1,717.5 kcal/일
```

**TypeScript 구현:**
```typescript
function calculateBMR(
  weight: number,    // kg
  height: number,    // cm
  age: number,       // 년
  sex: 'male' | 'female'
): number {
  const baseBMR = (10 * weight) + (6.25 * height) - (5 * age);
  return sex === 'male' ? baseBMR + 5 : baseBMR - 161;
}
```

### 6.1.2 Harris-Benedict 공식 (대안)

**남성:**
```
BMR = 88.362 + (13.397 × 체중_kg) + (4.799 × 키_cm) - (5.677 × 나이)
```

**여성:**
```
BMR = 447.593 + (9.247 × 체중_kg) + (3.098 × 키_cm) - (4.330 × 나이)
```

---

## 6.2 총 일일 에너지 소비 (TDEE)

### 6.2.1 TDEE 계산

```
TDEE = BMR × 활동 계수
```

**활동 계수:**
```
거의 활동 없음 (운동 거의/전혀 없음):
  계수 = 1.2

가벼운 활동 (주 1-3일):
  계수 = 1.375

보통 활동 (주 3-5일):
  계수 = 1.55

매우 활동적 (주 6-7일):
  계수 = 1.725

극도로 활동적 (육체 노동 + 훈련):
  계수 = 1.9
```

**예시:**
```
BMR: 1,717.5 kcal/일
활동 수준: 보통 활동적 (1.55)

TDEE = 1,717.5 × 1.55
TDEE = 2,662 kcal/일
```

---

## 6.3 활동 칼로리 계산

### 6.3.1 MET 기반 방법 (기본)

**공식:**
```
칼로리 = MET × 체중_kg × 지속시간_시간
```

**일반 활동의 MET 값:**
```
걷기 (4.0 km/h):     3.0 MET
걷기 (4.8 km/h):     3.5 MET
걷기 (6.4 km/h):     5.0 MET

달리기 (8.0 km/h):   8.3 MET
달리기 (9.7 km/h):   9.8 MET
달리기 (11.3 km/h): 11.0 MET
달리기 (12.9 km/h): 11.8 MET

사이클링 (16-19 km/h):  6.8 MET
사이클링 (19-22 km/h):  8.0 MET
사이클링 (22-26 km/h): 10.0 MET

수영 (보통):          7.0 MET
웨이트 트레이닝 (보통): 5.0 MET
HIIT 트레이닝:        8.0 MET
요가:                3.0 MET
```

**예시 계산:**
```
활동: 시속 9.7km 달리기
MET: 9.8
체중: 75kg
지속 시간: 30분 (0.5시간)

칼로리 = 9.8 × 75 × 0.5
칼로리 = 367.5 kcal
```

**TypeScript 구현:**
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

### 6.3.2 심박수 기반 방법

심박수 데이터를 사용할 수 있을 때 더 정확합니다.

**남성:**
```
칼로리/분 = ((나이 × 0.2017) - (체중_kg × 0.09036) + (심박수 × 0.6309) - 55.0969) × 지속시간_분 / 4.184
```

**여성:**
```
칼로리/분 = ((나이 × 0.074) - (체중_kg × 0.05741) + (심박수 × 0.4472) - 20.4022) × 지속시간_분 / 4.184
```

**TypeScript 구현:**
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

### 6.3.3 VO2 기반 방법 (가장 정확)

**공식:**
```
VO2 (ml/kg/min) = (평균심박수 / 최대심박수) × VO2_max
칼로리/분 = (VO2 × 체중_kg × 5) / 1000
총 칼로리 = 칼로리/분 × 지속시간_분
```

---

## 6.4 EPOC (운동 후 과잉 산소 소비)

### 6.4.1 애프터번 계산

**공식:**
```
EPOC_칼로리 = 기본_칼로리 × EPOC_계수
총_칼로리 = 기본_칼로리 + EPOC_칼로리
```

**강도별 EPOC 계수:**
```
저강도 (< VO2 max의 50%):
  계수 = 1.05 (5% 추가 칼로리)

중강도 (VO2 max의 50-75%):
  계수 = 1.10 (10% 추가 칼로리)

고강도 (> VO2 max의 75%):
  계수 = 1.15 (15% 추가 칼로리)

HIIT / 근력 트레이닝:
  계수 = 1.20-1.25 (20-25% 추가 칼로리)
```

---

## 6.5 심박수 구역 계산

### 6.5.1 최대 심박수 추정

**전통 공식 (Fox):**
```
최대_심박수 = 220 - 나이
```

**Tanaka 공식 (더 정확):**
```
최대_심박수 = 208 - (0.7 × 나이)
```

**Gulati 공식 (여성용):**
```
최대_심박수 = 206 - (0.88 × 나이)
```

### 6.5.2 심박수 예비량 (Karvonen) 방법

**공식:**
```
심박수_예비량 = 최대_심박수 - 안정시_심박수
목표_심박수 = 안정시_심박수 + (심박수_예비량 × 강도_백분율)
```

**구역 정의:**
```
구역 1 (회복):     심박수 예비량의 50-60%
구역 2 (유산소):   심박수 예비량의 60-70%
구역 3 (템포):     심박수 예비량의 70-80%
구역 4 (역치):     심박수 예비량의 80-90%
구역 5 (최대):     심박수 예비량의 90-100%
```

**예시:**
```
최대 심박수: 187 BPM
안정시 심박수: 52 BPM
심박수 예비량: 187 - 52 = 135 BPM

구역 1 (50-60%):
  하한: 52 + (135 × 0.50) = 119.5 BPM
  상한: 52 + (135 × 0.60) = 133 BPM

구역 2 (60-70%):
  하한: 133 BPM
  상한: 146.5 BPM
```

**TypeScript 구현:**
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
      name: '구역 1',
      lower: Math.round(restingHR + (hrReserve * 0.50)),
      upper: Math.round(restingHR + (hrReserve * 0.60)),
      intensity: '회복'
    },
    {
      name: '구역 2',
      lower: Math.round(restingHR + (hrReserve * 0.60)),
      upper: Math.round(restingHR + (hrReserve * 0.70)),
      intensity: '유산소'
    }
    // ... 나머지 구역
  ];
}
```

---

## 6.6 심박 변이도 (HRV) 계산

### 6.6.1 RMSSD (연속 차이의 제곱평균제곱근)

**공식:**
```
RMSSD = √(Σ(RR[i+1] - RR[i])² / (N-1))
```

**TypeScript 구현:**
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

### 6.6.2 SDNN (NN 간격의 표준편차)

**공식:**
```
SDNN = √(Σ(RR[i] - 평균_RR)² / (N-1))
```

---

## 6.7 VO2 Max 추정

### 6.7.1 Cooper 테스트 방법

**공식:**
```
VO2_max (ml/kg/min) = (거리_미터 - 504.9) / 44.73
```

거리는 12분 동안 이동한 최대 거리입니다.

### 6.7.2 심박수 기반 추정

**공식:**
```
VO2_max = 15.3 × (최대_심박수 / 안정시_심박수)
```

**나이 조정 포함:**
```
나이_계수:
  20-29: 1.0
  30-39: 0.93
  40-49: 0.83
  50-59: 0.74
  60+:   0.65

VO2_max = 15 × (최대_심박수 / 안정시_심박수) × 나이_계수
```

---

## 6.8 트레이닝 부하 계산

### 6.8.1 TRIMP (트레이닝 임펄스)

**공식:**
```
TRIMP = 지속시간_분 × 심박수_비율 × e^(k × 심박수_비율)

여기서:
심박수_비율 = (평균심박수 - 안정시심박수) / (최대심박수 - 안정시심박수)
k = 1.92 (남성), 1.67 (여성)
```

**TypeScript 구현:**
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

### 6.8.2 트레이닝 스트레스 점수 (TSS)

**심박수 기반:**
```
TSS = (지속시간_초 × 심박수_비율²) / 36
```

**TypeScript 구현:**
```typescript
function calculateTSS(
  durationSeconds: number,
  hrRatio: number
): number {
  return (durationSeconds * Math.pow(hrRatio, 2)) / 36;
}
```

---

## 6.9 회복 지표

### 6.9.1 회복 심박수

**공식:**
```
회복_심박수 = 운동_종료시_심박수 - 1분_후_심박수
```

**해석:**
```
우수:  > 25 BPM 감소
좋음:  15-25 BPM 감소
보통:  10-15 BPM 감소
나쁨:  < 10 BPM 감소
```

### 6.9.2 회복 점수

**복합 공식:**
```
회복_점수 = (HRV_점수 × 0.4) + (수면_점수 × 0.3) + (안정시심박수_점수 × 0.3)
```

---

## 핵심 요점

✓ BMR은 Mifflin-St Jeor 공식 사용 (가장 정확)

✓ TDEE는 BMR에 활동 계수를 곱하여 도출

✓ 칼로리 계산은 MET, 심박수, VO2 방법 지원

✓ EPOC는 강도에 따라 5-25% 애프터번 추가

✓ 심박수 구역은 심박수 예비량을 사용하는 Karvonen 방법 사용

✓ HRV 지표 (RMSSD, SDNN)는 회복 상태 표시

✓ VO2 max는 Cooper 테스트 또는 심박수 비율로 추정

✓ 트레이닝 부하는 TRIMP 또는 TSS 공식으로 정량화

✓ 모든 공식은 과학적 연구로 검증됨

---

**다음:** [7장: 시스템 통합 →](07-system-integration.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
