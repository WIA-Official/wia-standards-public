# Phase 2: Pet Health Passport Algorithm Specification

## WIA-PET-HEALTH-PASSPORT Core Algorithms

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Standard ID**: WIA-PET-HEALTH-PASSPORT-PHASE2-001

---

## 1. 개요

WIA-PET-HEALTH-PASSPORT 알고리즘은 반려동물 건강 기록의 검증, 검역 요구사항 판정,
위험도 평가, 그리고 글로벌 호환성 검사를 위한 핵심 로직을 정의합니다.

### 1.1 알고리즘 영역

```
┌────────────────────────────────────────────────────────────┐
│              WIA-PET-HEALTH-PASSPORT 알고리즘 구조         │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  1. 검역 적격성 판정 (Quarantine Eligibility)              │
│     └─→ 국가별 요구사항 검증                               │
│                                                            │
│  2. 백신 유효성 검사 (Vaccination Validation)              │
│     └─→ 접종 일정 및 유효 기간 검증                        │
│                                                            │
│  3. 건강 위험도 평가 (Health Risk Assessment)              │
│     └─→ 유전 질환 + 현재 상태 종합 평가                    │
│                                                            │
│  4. 문서 무결성 검증 (Document Integrity)                  │
│     └─→ 디지털 서명 및 체인 검증                           │
│                                                            │
│  5. 품종 검증 (Breed Verification)                         │
│     └─→ DNA 기반 품종 확인                                 │
│                                                            │
│  6. 응급 정보 추출 (Emergency Info Extraction)             │
│     └─→ 빠른 수의사 정보 접근                              │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

### 1.2 철학

**홍익인간 (弘益人間)** - 인류와 동물 모두를 이롭게 하라

---

## 2. Quarantine Eligibility Algorithm

### 2.1 개요

국경 통과 시 반려동물의 검역 요구사항 충족 여부를 판정합니다.

### 2.2 알고리즘 흐름

```
Algorithm: QuarantineEligibility
Purpose: 국가 간 이동 시 검역 적격성 판정

Input:
  - passport: PetHealthPassport
  - departureCountry: ISO3166Alpha2
  - arrivalCountry: ISO3166Alpha2
  - travelDate: ISO8601

Output:
  - eligible: boolean
  - requirements: RequirementStatus[]
  - missingItems: string[]
  - recommendations: string[]

Steps:
1. 도착국 요구사항 조회
   requirements = getCountryRequirements(arrivalCountry)

2. 마이크로칩 검증
   IF requirements.microchipRequired:
     IF NOT passport.microchip:
       FAIL("Microchip required")
     IF requirements.microchipStandard != 'any':
       VERIFY microchip standard matches

3. 광견병 백신 검증
   IF requirements.rabiesVaccination.required:
     rabiesRecord = findLatestRabiesVaccination(passport)
     VERIFY:
       - 접종일이 최소 대기 기간 이후
       - 유효 기간 내
       - 필요 시 항체가 검사 결과

4. 기타 백신 검증
   FOR EACH vaccine IN requirements.otherVaccinations:
     IF vaccine.required:
       VERIFY vaccination exists and is valid

5. 기생충 치료 검증
   IF requirements.parasiteTreatment:
     VERIFY treatment within required timeframe

6. 금지 품종 검사
   IF passport.identity.breed IN requirements.bannedBreeds:
     FAIL("Breed not allowed")

7. 필요 서류 검사
   VERIFY all required documents exist and are valid

8. 결과 집계
   RETURN eligibility result with details
```

### 2.3 Pseudocode

```typescript
interface EligibilityResult {
  eligible: boolean;
  overallScore: number;         // 0-100
  requirements: RequirementCheck[];
  missingItems: string[];
  warnings: string[];
  recommendations: string[];
  estimatedProcessingDays: number;
}

interface RequirementCheck {
  requirement: string;
  status: 'passed' | 'failed' | 'warning' | 'pending';
  details: string;
  actionRequired?: string;
  deadline?: ISO8601;
}

function checkQuarantineEligibility(
  passport: PetHealthPassport,
  departure: ISO3166Alpha2,
  arrival: ISO3166Alpha2,
  travelDate: ISO8601
): EligibilityResult {
  const requirements = getCountryRequirements(arrival);
  const checks: RequirementCheck[] = [];
  const missing: string[] = [];
  const warnings: string[] = [];

  // 1. 마이크로칩 검증
  if (requirements.microchipRequired) {
    const microchipCheck = verifyMicrochip(passport, requirements);
    checks.push(microchipCheck);
    if (microchipCheck.status === 'failed') {
      missing.push('Valid microchip');
    }
  }

  // 2. 광견병 백신 검증
  if (requirements.rabiesVaccination.required) {
    const rabiesCheck = verifyRabiesVaccination(
      passport,
      requirements.rabiesVaccination,
      travelDate
    );
    checks.push(rabiesCheck);
    if (rabiesCheck.status === 'failed') {
      missing.push('Valid rabies vaccination');
    }
  }

  // 3. 항체가 검사 (Titer Test)
  if (requirements.rabiesVaccination.titerTestRequired) {
    const titerCheck = verifyRabiesTiter(
      passport,
      requirements.rabiesVaccination.minimumTiter
    );
    checks.push(titerCheck);
    if (titerCheck.status === 'failed') {
      missing.push('Rabies titer test (≥ 0.5 IU/ml)');
    }
  }

  // 4. 기타 백신
  for (const vaccine of requirements.otherVaccinations) {
    if (vaccine.required) {
      const vaccineCheck = verifyVaccination(
        passport,
        vaccine.disease,
        travelDate,
        vaccine.waitPeriod
      );
      checks.push(vaccineCheck);
      if (vaccineCheck.status === 'failed') {
        missing.push(`${vaccine.disease} vaccination`);
      }
    }
  }

  // 5. 기생충 치료
  if (requirements.parasiteTreatment?.required) {
    const parasiteCheck = verifyParasiteTreatment(
      passport,
      requirements.parasiteTreatment,
      travelDate
    );
    checks.push(parasiteCheck);
    if (parasiteCheck.status === 'failed') {
      missing.push('Parasite treatment');
    }
  }

  // 6. 금지 품종
  if (requirements.bannedBreeds?.length) {
    const breedCheck = verifyBreedAllowed(
      passport.identity.breed,
      requirements.bannedBreeds
    );
    checks.push(breedCheck);
    if (breedCheck.status === 'failed') {
      missing.push('Breed allowed');
    }
  }

  // 7. 서류 확인
  const documentChecks = verifyRequiredDocuments(
    passport,
    requirements,
    travelDate
  );
  checks.push(...documentChecks);

  // 8. 점수 계산
  const passedCount = checks.filter(c => c.status === 'passed').length;
  const totalCount = checks.length;
  const score = Math.round((passedCount / totalCount) * 100);

  // 9. 권장사항 생성
  const recommendations = generateRecommendations(
    checks,
    travelDate,
    requirements
  );

  return {
    eligible: missing.length === 0,
    overallScore: score,
    requirements: checks,
    missingItems: missing,
    warnings,
    recommendations,
    estimatedProcessingDays: calculateProcessingTime(arrival, missing)
  };
}
```

### 2.4 광견병 백신 검증 상세

```typescript
interface RabiesValidationConfig {
  required: boolean;
  minimumAge: number;           // 개월
  waitPeriod: number;           // 일 (접종 후 대기)
  validityPeriod: number;       // 개월
  titerTestRequired: boolean;
  minimumTiter: number;         // IU/ml
}

function verifyRabiesVaccination(
  passport: PetHealthPassport,
  config: RabiesValidationConfig,
  travelDate: ISO8601
): RequirementCheck {
  // 광견병 백신 기록 찾기
  const rabiesRecords = passport.medicalRecords.vaccinations
    .filter(v => v.targetDiseases.includes('RABIES'))
    .sort((a, b) => new Date(b.administeredAt).getTime() -
                    new Date(a.administeredAt).getTime());

  if (rabiesRecords.length === 0) {
    return {
      requirement: 'Rabies Vaccination',
      status: 'failed',
      details: 'No rabies vaccination record found',
      actionRequired: 'Get rabies vaccination from licensed veterinarian'
    };
  }

  const latestRabies = rabiesRecords[0];
  const vaccinationDate = new Date(latestRabies.administeredAt);
  const travel = new Date(travelDate);
  const validUntil = new Date(latestRabies.validUntil);

  // 1. 유효 기간 확인
  if (travel > validUntil) {
    return {
      requirement: 'Rabies Vaccination',
      status: 'failed',
      details: `Vaccination expired on ${latestRabies.validUntil}`,
      actionRequired: 'Get booster vaccination'
    };
  }

  // 2. 대기 기간 확인
  const waitPeriodEnd = new Date(vaccinationDate);
  waitPeriodEnd.setDate(waitPeriodEnd.getDate() + config.waitPeriod);

  if (travel < waitPeriodEnd) {
    const daysRemaining = Math.ceil(
      (waitPeriodEnd.getTime() - travel.getTime()) / (1000 * 60 * 60 * 24)
    );
    return {
      requirement: 'Rabies Vaccination',
      status: 'failed',
      details: `Wait period not complete. ${daysRemaining} days remaining`,
      actionRequired: `Travel date must be after ${waitPeriodEnd.toISOString()}`
    };
  }

  // 3. 최소 연령 확인 (접종 시점)
  if (passport.identity.dateOfBirth) {
    const birthDate = new Date(passport.identity.dateOfBirth);
    const ageAtVaccination = monthsBetween(birthDate, vaccinationDate);

    if (ageAtVaccination < config.minimumAge) {
      return {
        requirement: 'Rabies Vaccination',
        status: 'warning',
        details: `Pet was ${ageAtVaccination} months old at vaccination (minimum: ${config.minimumAge})`,
        actionRequired: 'May require revaccination after reaching minimum age'
      };
    }
  }

  return {
    requirement: 'Rabies Vaccination',
    status: 'passed',
    details: `Valid until ${latestRabies.validUntil}`,
  };
}
```

---

## 3. Vaccination Schedule Algorithm

### 3.1 개요

반려동물 종/품종별 권장 백신 접종 일정을 계산하고 다음 접종 시기를 알려줍니다.

### 3.2 알고리즘

```
Algorithm: VaccinationScheduler
Purpose: 권장 백신 접종 일정 계산

Input:
  - pet: PetIdentity
  - existingVaccinations: VaccinationRecord[]
  - country: ISO3166Alpha2

Output:
  - upcomingVaccinations: ScheduledVaccination[]
  - overdueVaccinations: OverdueVaccination[]
  - completedSchedule: CompletedVaccination[]

Steps:
1. 종별 기본 일정 조회
   baseSchedule = getBaseVaccinationSchedule(pet.species)

2. 지역별 추가 백신 확인
   regionalVaccines = getRegionalRecommendations(country, pet.species)

3. 생년월일 기반 계산
   petAge = calculateAge(pet.dateOfBirth)

4. 각 백신별 상태 계산
   FOR EACH vaccine IN baseSchedule + regionalVaccines:
     lastDose = findLastDose(existingVaccinations, vaccine)

     IF lastDose:
       nextDue = calculateNextDue(lastDose, vaccine.boosterInterval)
       IF nextDue < TODAY:
         ADD to overdueVaccinations
       ELSE IF nextDue < TODAY + 30days:
         ADD to upcomingVaccinations with priority
       ELSE:
         ADD to completedSchedule
     ELSE:
       IF petAge >= vaccine.minimumAge:
         ADD to overdueVaccinations (first dose)
       ELSE:
         ADD to upcomingVaccinations when pet reaches minimum age

5. 우선순위 정렬
   SORT upcomingVaccinations by:
     - Core vs Non-core
     - Due date
     - Legal requirement

RETURN results
```

### 3.3 Pseudocode

```typescript
interface VaccinationSchedule {
  vaccine: string;
  disease: VaccineDisease;
  isCore: boolean;              // 핵심 vs 선택
  minimumAge: number;           // 주
  firstSeriesDoses: number;     // 초기 접종 횟수
  firstSeriesInterval: number;  // 주
  boosterInterval: number;      // 개월 (이후 추가 접종 간격)
  legallyRequired: boolean;
}

interface ScheduledVaccination {
  vaccine: string;
  dueDate: ISO8601;
  priority: 'urgent' | 'high' | 'medium' | 'low';
  doseNumber: number;
  reason: string;
}

function calculateVaccinationSchedule(
  pet: PetIdentity,
  vaccinations: VaccinationRecord[],
  country: ISO3166Alpha2
): VaccinationScheduleResult {
  const baseSchedule = getSpeciesSchedule(pet.species);
  const regionalSchedule = getRegionalSchedule(country, pet.species);
  const allVaccines = [...baseSchedule, ...regionalSchedule];

  const petAgeWeeks = calculateAgeInWeeks(pet.dateOfBirth);
  const today = new Date();

  const upcoming: ScheduledVaccination[] = [];
  const overdue: ScheduledVaccination[] = [];
  const completed: CompletedVaccination[] = [];

  for (const schedule of allVaccines) {
    const records = vaccinations.filter(v =>
      v.targetDiseases.includes(schedule.disease)
    ).sort((a, b) =>
      new Date(a.administeredAt).getTime() - new Date(b.administeredAt).getTime()
    );

    const doseCount = records.length;

    if (doseCount === 0) {
      // 미접종
      if (petAgeWeeks >= schedule.minimumAge) {
        overdue.push({
          vaccine: schedule.vaccine,
          dueDate: today.toISOString(),
          priority: schedule.isCore ? 'urgent' : 'high',
          doseNumber: 1,
          reason: 'Initial vaccination overdue'
        });
      } else {
        // 최소 연령 도달 시 접종
        const dueDate = addWeeks(
          new Date(pet.dateOfBirth),
          schedule.minimumAge
        );
        upcoming.push({
          vaccine: schedule.vaccine,
          dueDate: dueDate.toISOString(),
          priority: schedule.isCore ? 'high' : 'medium',
          doseNumber: 1,
          reason: `First dose at ${schedule.minimumAge} weeks of age`
        });
      }
    } else if (doseCount < schedule.firstSeriesDoses) {
      // 초기 시리즈 진행 중
      const lastDose = records[records.length - 1];
      const nextDue = addWeeks(
        new Date(lastDose.administeredAt),
        schedule.firstSeriesInterval
      );

      if (nextDue < today) {
        overdue.push({
          vaccine: schedule.vaccine,
          dueDate: nextDue.toISOString(),
          priority: 'urgent',
          doseNumber: doseCount + 1,
          reason: `Dose ${doseCount + 1} of ${schedule.firstSeriesDoses} overdue`
        });
      } else {
        upcoming.push({
          vaccine: schedule.vaccine,
          dueDate: nextDue.toISOString(),
          priority: schedule.isCore ? 'high' : 'medium',
          doseNumber: doseCount + 1,
          reason: `Dose ${doseCount + 1} of ${schedule.firstSeriesDoses}`
        });
      }
    } else {
      // 부스터 단계
      const lastDose = records[records.length - 1];
      const nextBooster = addMonths(
        new Date(lastDose.administeredAt),
        schedule.boosterInterval
      );

      if (nextBooster < today) {
        overdue.push({
          vaccine: schedule.vaccine,
          dueDate: nextBooster.toISOString(),
          priority: schedule.legallyRequired ? 'urgent' : 'high',
          doseNumber: doseCount + 1,
          reason: 'Booster overdue'
        });
      } else if (nextBooster < addDays(today, 30)) {
        upcoming.push({
          vaccine: schedule.vaccine,
          dueDate: nextBooster.toISOString(),
          priority: 'medium',
          doseNumber: doseCount + 1,
          reason: 'Booster due soon'
        });
      } else {
        completed.push({
          vaccine: schedule.vaccine,
          lastDose: lastDose.administeredAt,
          nextDue: nextBooster.toISOString(),
          status: 'up_to_date'
        });
      }
    }
  }

  return {
    upcoming: sortByPriority(upcoming),
    overdue: sortByPriority(overdue),
    completed,
    summary: {
      totalVaccines: allVaccines.length,
      upToDate: completed.length,
      overdueCount: overdue.length,
      upcomingCount: upcoming.length
    }
  };
}
```

### 3.4 종별 기본 접종 일정

```typescript
const DOG_VACCINATION_SCHEDULE: VaccinationSchedule[] = [
  {
    vaccine: 'DHPP',
    disease: VaccineDisease.DISTEMPER,
    isCore: true,
    minimumAge: 6,              // 주
    firstSeriesDoses: 3,
    firstSeriesInterval: 4,     // 주
    boosterInterval: 12,        // 개월
    legallyRequired: false
  },
  {
    vaccine: 'Rabies',
    disease: VaccineDisease.RABIES,
    isCore: true,
    minimumAge: 12,             // 주
    firstSeriesDoses: 1,
    firstSeriesInterval: 0,
    boosterInterval: 12,        // 또는 36 (3년)
    legallyRequired: true
  },
  {
    vaccine: 'Bordetella',
    disease: VaccineDisease.BORDETELLA,
    isCore: false,
    minimumAge: 8,
    firstSeriesDoses: 2,
    firstSeriesInterval: 4,
    boosterInterval: 12,
    legallyRequired: false
  }
];

const CAT_VACCINATION_SCHEDULE: VaccinationSchedule[] = [
  {
    vaccine: 'FVRCP',
    disease: VaccineDisease.FELINE_PANLEUKOPENIA,
    isCore: true,
    minimumAge: 6,
    firstSeriesDoses: 3,
    firstSeriesInterval: 4,
    boosterInterval: 12,
    legallyRequired: false
  },
  {
    vaccine: 'Rabies',
    disease: VaccineDisease.RABIES,
    isCore: true,
    minimumAge: 12,
    firstSeriesDoses: 1,
    firstSeriesInterval: 0,
    boosterInterval: 12,
    legallyRequired: true
  },
  {
    vaccine: 'FeLV',
    disease: VaccineDisease.FELINE_LEUKEMIA,
    isCore: false,
    minimumAge: 8,
    firstSeriesDoses: 2,
    firstSeriesInterval: 4,
    boosterInterval: 12,
    legallyRequired: false
  }
];
```

---

## 4. Health Risk Assessment Algorithm

### 4.1 개요

유전 정보와 현재 건강 상태를 종합하여 반려동물의 건강 위험도를 평가합니다.

### 4.2 알고리즘

```
Algorithm: HealthRiskAssessment
Purpose: 종합 건강 위험도 평가

Input:
  - genetics: GeneticProfile
  - conditions: ChronicCondition[]
  - allergies: AllergyRecord[]
  - medications: MedicationRecord[]
  - diagnostics: DiagnosticRecord[]
  - identity: PetIdentity (품종, 나이)

Output:
  - overallRisk: 'low' | 'moderate' | 'high' | 'critical'
  - riskScore: number (0-100)
  - riskFactors: RiskFactor[]
  - recommendations: HealthRecommendation[]

Steps:
1. 유전 위험도 계산
   geneticRisk = calculateGeneticRisk(genetics)

2. 품종 특이적 위험도
   breedRisk = getBreedSpecificRisks(identity.breed)

3. 연령 관련 위험도
   ageRisk = calculateAgeRelatedRisk(identity)

4. 현재 건강 상태
   currentHealthRisk = assessCurrentConditions(conditions, allergies)

5. 약물 상호작용 위험
   medicationRisk = checkMedicationInteractions(medications)

6. 생활습관 위험 (진단 기록 기반)
   lifestyleRisk = assessLifestyleFactors(diagnostics)

7. 종합 점수 계산
   totalScore = weightedAverage([
     geneticRisk * 0.25,
     breedRisk * 0.15,
     ageRisk * 0.15,
     currentHealthRisk * 0.30,
     medicationRisk * 0.10,
     lifestyleRisk * 0.05
   ])

8. 권장사항 생성
   recommendations = generateHealthRecommendations(riskFactors)

RETURN results
```

### 4.3 Pseudocode

```typescript
interface HealthRiskResult {
  overallRisk: 'low' | 'moderate' | 'high' | 'critical';
  riskScore: number;            // 0-100
  riskFactors: RiskFactor[];
  recommendations: HealthRecommendation[];
  monitoringSchedule: MonitoringItem[];
}

interface RiskFactor {
  category: string;
  factor: string;
  severity: 'low' | 'moderate' | 'high';
  score: number;
  description: string;
  evidence?: string;
}

function assessHealthRisk(
  passport: PetHealthPassport
): HealthRiskResult {
  const riskFactors: RiskFactor[] = [];
  let totalScore = 0;

  // 1. 유전 위험도
  if (passport.genetics) {
    const geneticRisks = assessGeneticRisks(passport.genetics);
    riskFactors.push(...geneticRisks.factors);
    totalScore += geneticRisks.score * 0.25;
  }

  // 2. 품종 특이적 위험도
  const breedRisks = assessBreedRisks(
    passport.identity.species,
    passport.identity.breed
  );
  riskFactors.push(...breedRisks.factors);
  totalScore += breedRisks.score * 0.15;

  // 3. 연령 관련 위험도
  const ageRisks = assessAgeRisks(passport.identity);
  riskFactors.push(...ageRisks.factors);
  totalScore += ageRisks.score * 0.15;

  // 4. 현재 건강 상태
  const conditionRisks = assessCurrentConditions(
    passport.medicalRecords.conditions,
    passport.medicalRecords.allergies
  );
  riskFactors.push(...conditionRisks.factors);
  totalScore += conditionRisks.score * 0.30;

  // 5. 약물 상호작용
  const medicationRisks = assessMedicationRisks(
    passport.medicalRecords.medications
  );
  riskFactors.push(...medicationRisks.factors);
  totalScore += medicationRisks.score * 0.10;

  // 6. 검사 결과 분석
  const diagnosticRisks = assessDiagnosticTrends(
    passport.medicalRecords.diagnostics
  );
  riskFactors.push(...diagnosticRisks.factors);
  totalScore += diagnosticRisks.score * 0.05;

  // 점수 정규화
  const normalizedScore = Math.min(100, Math.max(0, totalScore));

  // 위험 등급 결정
  const overallRisk = determineRiskLevel(normalizedScore);

  // 권장사항 생성
  const recommendations = generateRecommendations(
    riskFactors,
    passport.identity
  );

  // 모니터링 일정
  const monitoringSchedule = createMonitoringSchedule(
    riskFactors,
    passport.identity
  );

  return {
    overallRisk,
    riskScore: normalizedScore,
    riskFactors,
    recommendations,
    monitoringSchedule
  };
}

function assessGeneticRisks(genetics: GeneticProfile): {
  factors: RiskFactor[];
  score: number;
} {
  const factors: RiskFactor[] = [];
  let score = 0;

  for (const marker of genetics.healthMarkers) {
    if (marker.riskLevel === 'at_risk') {
      factors.push({
        category: 'Genetic',
        factor: marker.condition,
        severity: 'high',
        score: 30,
        description: `Genetic risk for ${marker.condition}`,
        evidence: `Genotype: ${marker.genotype}`
      });
      score += 30;
    } else if (marker.riskLevel === 'carrier') {
      factors.push({
        category: 'Genetic',
        factor: marker.condition,
        severity: 'low',
        score: 10,
        description: `Carrier for ${marker.condition}`,
        evidence: `Genotype: ${marker.genotype}`
      });
      score += 10;
    }
  }

  return {
    factors,
    score: Math.min(100, score)
  };
}

function assessBreedRisks(
  species: PetSpecies,
  breed: string
): { factors: RiskFactor[]; score: number } {
  const factors: RiskFactor[] = [];
  let score = 0;

  const breedRisks = BREED_HEALTH_RISKS[species]?.[breed] || [];

  for (const risk of breedRisks) {
    factors.push({
      category: 'Breed',
      factor: risk.condition,
      severity: risk.prevalence > 0.3 ? 'high' : risk.prevalence > 0.1 ? 'moderate' : 'low',
      score: Math.round(risk.prevalence * 100),
      description: `${breed} has ${Math.round(risk.prevalence * 100)}% prevalence of ${risk.condition}`
    });
    score += risk.prevalence * 50;
  }

  return {
    factors,
    score: Math.min(100, score)
  };
}

function determineRiskLevel(score: number): 'low' | 'moderate' | 'high' | 'critical' {
  if (score < 25) return 'low';
  if (score < 50) return 'moderate';
  if (score < 75) return 'high';
  return 'critical';
}
```

### 4.4 품종별 건강 위험 데이터베이스

```typescript
const BREED_HEALTH_RISKS: Record<PetSpecies, Record<string, BreedRisk[]>> = {
  [PetSpecies.DOG]: {
    'German Shepherd': [
      { condition: 'Hip Dysplasia', prevalence: 0.19 },
      { condition: 'Degenerative Myelopathy', prevalence: 0.15 },
      { condition: 'Bloat (GDV)', prevalence: 0.05 }
    ],
    'Golden Retriever': [
      { condition: 'Hip Dysplasia', prevalence: 0.20 },
      { condition: 'Cancer', prevalence: 0.60 },
      { condition: 'Heart Disease', prevalence: 0.10 }
    ],
    'Bulldog': [
      { condition: 'Brachycephalic Syndrome', prevalence: 0.70 },
      { condition: 'Skin Allergies', prevalence: 0.40 },
      { condition: 'Hip Dysplasia', prevalence: 0.35 }
    ],
    // ... 더 많은 품종
  },
  [PetSpecies.CAT]: {
    'Persian': [
      { condition: 'Polycystic Kidney Disease', prevalence: 0.40 },
      { condition: 'Brachycephalic Syndrome', prevalence: 0.30 },
      { condition: 'Eye Problems', prevalence: 0.25 }
    ],
    'Maine Coon': [
      { condition: 'Hypertrophic Cardiomyopathy', prevalence: 0.30 },
      { condition: 'Hip Dysplasia', prevalence: 0.18 },
      { condition: 'Spinal Muscular Atrophy', prevalence: 0.10 }
    ],
    // ... 더 많은 품종
  }
};
```

---

## 5. Document Verification Algorithm

### 5.1 개요

여권 문서의 무결성과 진위를 검증합니다.

### 5.2 알고리즘

```
Algorithm: DocumentVerification
Purpose: 디지털 서명 및 체인 검증

Input:
  - passport: PetHealthPassport
  - trustedIssuers: TrustedIssuer[]

Output:
  - valid: boolean
  - signatureValid: boolean
  - chainValid: boolean
  - issuerTrusted: boolean
  - details: VerificationDetail[]

Steps:
1. 디지털 서명 검증
   signature = passport.verification.digitalSignature
   publicKey = getIssuerPublicKey(passport.verification.issuingAuthority)
   signatureValid = verifySignature(passport, signature, publicKey)

2. 인증서 체인 검증
   chain = passport.verification.certificateChain
   chainValid = verifyCertificateChain(chain, ROOT_CA)

3. 발급 기관 신뢰도 확인
   issuerTrusted = trustedIssuers.includes(
     passport.verification.issuingAuthority
   )

4. 개별 레코드 검증
   FOR EACH record IN passport.medicalRecords:
     IF record.digitalSignature:
       VERIFY record signature

5. 타임스탬프 검증
   VERIFY timestamps are consistent and not in future

6. 마이크로칩 ID 형식 검증
   IF passport.microchip:
     VERIFY ISO 11784/11785 format

RETURN verification result
```

### 5.3 Pseudocode

```typescript
interface VerificationResult {
  valid: boolean;
  signatureValid: boolean;
  chainValid: boolean;
  issuerTrusted: boolean;
  recordsVerified: number;
  recordsFailed: number;
  details: VerificationDetail[];
  warnings: string[];
}

interface VerificationDetail {
  component: string;
  status: 'valid' | 'invalid' | 'unknown';
  message: string;
  timestamp?: ISO8601;
}

async function verifyDocument(
  passport: PetHealthPassport,
  trustedIssuers: TrustedIssuer[]
): Promise<VerificationResult> {
  const details: VerificationDetail[] = [];
  const warnings: string[] = [];
  let recordsVerified = 0;
  let recordsFailed = 0;

  // 1. 서명 검증
  const signatureValid = await verifyPassportSignature(
    passport,
    passport.verification.digitalSignature
  );

  details.push({
    component: 'Passport Signature',
    status: signatureValid ? 'valid' : 'invalid',
    message: signatureValid
      ? 'Digital signature verified'
      : 'Digital signature verification failed'
  });

  // 2. 인증서 체인 검증
  const chainValid = await verifyCertificateChain(
    passport.verification.certificateChain
  );

  details.push({
    component: 'Certificate Chain',
    status: chainValid ? 'valid' : 'invalid',
    message: chainValid
      ? 'Certificate chain valid'
      : 'Certificate chain verification failed'
  });

  // 3. 발급 기관 확인
  const issuer = trustedIssuers.find(
    i => i.id === passport.verification.issuingAuthority
  );
  const issuerTrusted = issuer !== undefined;

  details.push({
    component: 'Issuing Authority',
    status: issuerTrusted ? 'valid' : 'unknown',
    message: issuerTrusted
      ? `Trusted issuer: ${issuer.name}`
      : `Unknown issuer: ${passport.verification.issuingAuthority}`
  });

  // 4. 개별 백신 기록 검증
  for (const vaccination of passport.medicalRecords.vaccinations) {
    if (vaccination.digitalSignature) {
      const valid = await verifyRecordSignature(vaccination);
      if (valid) {
        recordsVerified++;
      } else {
        recordsFailed++;
        details.push({
          component: `Vaccination: ${vaccination.vaccine.name}`,
          status: 'invalid',
          message: 'Record signature verification failed',
          timestamp: vaccination.administeredAt
        });
      }
    }
  }

  // 5. 타임스탬프 검증
  const timestampValid = validateTimestamps(passport);
  if (!timestampValid.valid) {
    warnings.push(...timestampValid.issues);
  }

  // 6. 마이크로칩 형식 검증
  if (passport.microchip) {
    const chipValid = validateMicrochipFormat(passport.microchip.chipNumber);
    details.push({
      component: 'Microchip Format',
      status: chipValid ? 'valid' : 'invalid',
      message: chipValid
        ? 'ISO 11784/11785 compliant'
        : 'Invalid microchip format'
    });
  }

  const valid = signatureValid && chainValid && recordsFailed === 0;

  return {
    valid,
    signatureValid,
    chainValid,
    issuerTrusted,
    recordsVerified,
    recordsFailed,
    details,
    warnings
  };
}

function validateMicrochipFormat(chipNumber: string): boolean {
  // ISO 11784/11785: 15자리 숫자
  if (!/^\d{15}$/.test(chipNumber)) {
    return false;
  }

  // 국가 코드 검증 (첫 3자리)
  const countryCode = chipNumber.substring(0, 3);
  const validCountryCodes = ['900', '956', '981', '982', '985', /* ... */];

  // 또는 제조사 코드
  const manufacturerCodes = ['900', '956', '977', '981', '985', /* ... */];

  return validCountryCodes.includes(countryCode) ||
         manufacturerCodes.includes(countryCode);
}
```

---

## 6. Emergency Information Extraction

### 6.1 개요

응급 상황에서 수의사가 빠르게 필요한 정보에 접근할 수 있도록 추출합니다.

### 6.2 알고리즘

```
Algorithm: EmergencyExtraction
Purpose: 응급 시 필수 정보 빠른 추출

Input:
  - passport: PetHealthPassport
  - qrData?: QRCodePayload

Output:
  - criticalInfo: CriticalPetInfo
  - extractionTime: number (ms)

Steps:
1. QR 코드 우선 파싱 (있는 경우)
   IF qrData:
     EXTRACT mini info (species, rabies status, emergency contact)

2. 알레르기 우선 추출
   allergies = passport.medicalRecords.allergies
     .filter(a => a.status === 'active')
     .filter(a => a.reactionSeverity >= 'severe')

3. 현재 투약 중인 약물
   currentMedications = passport.medicalRecords.medications
     .filter(m => isCurrentlyActive(m))

4. 만성 질환
   conditions = passport.medicalRecords.conditions
     .filter(c => c.status === 'active')

5. 최근 수술/시술
   recentSurgeries = passport.medicalRecords.surgeries
     .filter(s => withinLast6Months(s))

6. 연락처 정보
   contacts = passport.guardian.emergencyContacts

RETURN structured emergency info
```

### 6.3 Pseudocode

```typescript
interface EmergencyInfo {
  // 즉시 확인 필요
  criticalAlerts: CriticalAlert[];

  // 기본 정보
  pet: {
    name: string;
    species: PetSpecies;
    breed: string;
    age: string;
    weight: number;
    sex: string;
  };

  // 의료 정보
  medical: {
    severeAllergies: string[];
    currentMedications: MedicationSummary[];
    activeConditions: string[];
    recentSurgeries: SurgerySummary[];
  };

  // 연락처
  contacts: EmergencyContact[];

  // 수의사 참고
  veterinaryNotes: string[];
}

interface CriticalAlert {
  type: 'allergy' | 'medication' | 'condition' | 'warning';
  severity: 'high' | 'critical';
  message: string;
  action?: string;
}

function extractEmergencyInfo(
  passport: PetHealthPassport
): EmergencyInfo {
  const startTime = performance.now();
  const alerts: CriticalAlert[] = [];

  // 1. 심각한 알레르기 (생명 위협)
  const severeAllergies = passport.medicalRecords.allergies
    .filter(a => a.status === 'active')
    .filter(a => ['severe', 'life_threatening'].includes(a.reactionSeverity))
    .map(a => {
      alerts.push({
        type: 'allergy',
        severity: a.reactionSeverity === 'life_threatening' ? 'critical' : 'high',
        message: `ALLERGY: ${a.allergen}`,
        action: a.emergencyTreatment
      });
      return a.allergen;
    });

  // 2. 현재 복용 약물
  const currentMedications = passport.medicalRecords.medications
    .filter(m => {
      const now = new Date();
      const start = new Date(m.startDate);
      const end = m.endDate ? new Date(m.endDate) : null;
      return start <= now && (!end || end >= now);
    })
    .map(m => ({
      name: m.medication.name,
      dosage: `${m.dosage.amount} ${m.dosage.unit} ${m.dosage.frequency}`,
      route: m.dosage.route,
      indication: m.indication
    }));

  // 약물 상호작용 경고
  const interactions = checkDrugInteractions(currentMedications);
  for (const interaction of interactions) {
    alerts.push({
      type: 'medication',
      severity: 'high',
      message: `Drug interaction: ${interaction.drug1} + ${interaction.drug2}`,
      action: interaction.recommendation
    });
  }

  // 3. 활성 질환
  const activeConditions = passport.medicalRecords.conditions
    .filter(c => c.status === 'active')
    .map(c => {
      if (c.severity === 'severe') {
        alerts.push({
          type: 'condition',
          severity: 'high',
          message: `Active condition: ${c.conditionName}`,
          action: c.managementPlan
        });
      }
      return c.conditionName;
    });

  // 4. 최근 수술 (6개월 이내)
  const sixMonthsAgo = new Date();
  sixMonthsAgo.setMonth(sixMonthsAgo.getMonth() - 6);

  const recentSurgeries = passport.medicalRecords.surgeries
    .filter(s => new Date(s.performedAt) >= sixMonthsAgo)
    .map(s => ({
      procedure: s.procedureName,
      date: s.performedAt,
      restrictions: s.restrictions,
      followUpRequired: s.followUpRequired
    }));

  // 5. 나이 계산
  const age = calculatePetAge(passport.identity.dateOfBirth);

  // 6. 수의사 참고사항 생성
  const veterinaryNotes = generateVetNotes(
    passport,
    currentMedications,
    activeConditions
  );

  return {
    criticalAlerts: alerts.sort((a, b) =>
      a.severity === 'critical' ? -1 : b.severity === 'critical' ? 1 : 0
    ),
    pet: {
      name: passport.identity.name,
      species: passport.identity.species,
      breed: passport.identity.breed,
      age: age,
      weight: passport.identity.weight?.value || 0,
      sex: formatSex(passport.identity.sex)
    },
    medical: {
      severeAllergies,
      currentMedications,
      activeConditions,
      recentSurgeries
    },
    contacts: passport.guardian.emergencyContacts,
    veterinaryNotes
  };
}

function generateVetNotes(
  passport: PetHealthPassport,
  medications: MedicationSummary[],
  conditions: string[]
): string[] {
  const notes: string[] = [];

  // 마취 관련 주의사항
  const anesthesiaRisks: string[] = [];

  if (passport.identity.breed?.toLowerCase().includes('bulldog') ||
      passport.identity.breed?.toLowerCase().includes('pug')) {
    anesthesiaRisks.push('Brachycephalic - increased anesthesia risk');
  }

  if (conditions.includes('Heart Disease')) {
    anesthesiaRisks.push('Cardiac condition - adjust anesthesia protocol');
  }

  if (medications.some(m => m.name.toLowerCase().includes('nsaid'))) {
    notes.push('Currently on NSAIDs - watch for bleeding');
  }

  if (anesthesiaRisks.length > 0) {
    notes.push(`ANESTHESIA CAUTION: ${anesthesiaRisks.join('; ')}`);
  }

  // 광견병 상태
  const rabiesRecord = passport.medicalRecords.vaccinations
    .find(v => v.targetDiseases.includes('RABIES'));

  if (rabiesRecord) {
    const valid = new Date(rabiesRecord.validUntil) > new Date();
    notes.push(`Rabies: ${valid ? 'Current' : 'EXPIRED'} (until ${rabiesRecord.validUntil})`);
  } else {
    notes.push('Rabies: NO RECORD - Handle with caution');
  }

  return notes;
}
```

---

## 7. Performance Requirements

### 7.1 알고리즘 성능 요구사항

| 알고리즘 | 최대 시간 | 목표 시간 | 복잡도 |
|---------|----------|----------|--------|
| Quarantine Eligibility | 500ms | 100ms | O(n) |
| Vaccination Schedule | 200ms | 50ms | O(n) |
| Health Risk Assessment | 1s | 200ms | O(n*m) |
| Document Verification | 2s | 500ms | O(n) |
| Emergency Extraction | 100ms | 20ms | O(n) |
| QR Code Parse | 50ms | 10ms | O(1) |

### 7.2 최적화 전략

```typescript
// 1. 캐싱
const countryRequirementsCache = new LRUCache<string, CountryRequirements>({
  max: 200,
  ttl: 1000 * 60 * 60  // 1시간
});

// 2. 병렬 처리
async function parallelVerification(passport: PetHealthPassport) {
  const [
    signatureResult,
    chainResult,
    recordResults
  ] = await Promise.all([
    verifySignature(passport),
    verifyChain(passport),
    verifyRecords(passport)
  ]);

  return combineResults(signatureResult, chainResult, recordResults);
}

// 3. 지연 로딩
function getGeneticRisks(genetics: GeneticProfile) {
  // 필요할 때만 상세 분석
  if (!genetics._analyzedRisks) {
    genetics._analyzedRisks = analyzeGeneticRisks(genetics);
  }
  return genetics._analyzedRisks;
}

// 4. 인덱싱
interface PassportIndex {
  vaccinationsByDisease: Map<VaccineDisease, VaccinationRecord[]>;
  activeConditions: ChronicCondition[];
  currentMedications: MedicationRecord[];
}

function buildIndex(passport: PetHealthPassport): PassportIndex {
  // 한 번 인덱스 빌드, 이후 O(1) 조회
}
```

---

## 8. Validation

### 8.1 테스트 케이스

```typescript
describe('QuarantineEligibility', () => {
  it('should pass for fully vaccinated pet traveling to EU', () => {
    const passport = createTestPassport({
      microchip: true,
      rabiesVaccination: { valid: true, titerTest: true },
      tapewormTreatment: true
    });

    const result = checkQuarantineEligibility(
      passport,
      'US',
      'DE',
      '2025-06-01'
    );

    expect(result.eligible).toBe(true);
    expect(result.missingItems).toHaveLength(0);
  });

  it('should fail for missing rabies vaccination', () => {
    const passport = createTestPassport({
      microchip: true,
      rabiesVaccination: null
    });

    const result = checkQuarantineEligibility(
      passport,
      'US',
      'UK',
      '2025-06-01'
    );

    expect(result.eligible).toBe(false);
    expect(result.missingItems).toContain('Valid rabies vaccination');
  });

  it('should fail for banned breed', () => {
    const passport = createTestPassport({
      breed: 'Pit Bull',
      destination: 'UK'
    });

    const result = checkQuarantineEligibility(
      passport,
      'US',
      'UK',
      '2025-06-01'
    );

    expect(result.eligible).toBe(false);
    expect(result.requirements.find(r => r.requirement === 'Breed Allowed').status)
      .toBe('failed');
  });
});

describe('EmergencyExtraction', () => {
  it('should extract critical allergies first', () => {
    const passport = createTestPassport({
      allergies: [
        { allergen: 'Penicillin', severity: 'life_threatening' },
        { allergen: 'Chicken', severity: 'mild' }
      ]
    });

    const result = extractEmergencyInfo(passport);

    expect(result.criticalAlerts[0].severity).toBe('critical');
    expect(result.criticalAlerts[0].message).toContain('Penicillin');
  });

  it('should complete extraction within 100ms', async () => {
    const passport = createLargeTestPassport(); // 많은 기록

    const start = performance.now();
    const result = extractEmergencyInfo(passport);
    const duration = performance.now() - start;

    expect(duration).toBeLessThan(100);
  });
});
```

---

**Document ID**: WIA-PET-HEALTH-PASSPORT-PHASE2-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License

**홍익인간 (弘益人間)** - 인류와 동물 모두를 이롭게 하라
