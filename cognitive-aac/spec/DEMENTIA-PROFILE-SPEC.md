# WIA Cognitive AAC - Dementia Profile Specification
# 치매 프로파일 표준 명세서

**Version**: 1.0.0
**Date**: 2025-01-14
**Status**: Draft
**Extends**: CognitiveProfile

---

## 철학
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

---

## 1. 개요

### 1.1 목적
치매를 가진 AAC 사용자를 위한 특화된 프로파일 구조를 정의합니다. 이 프로파일은 기본 `CognitiveProfile`을 확장하여 치매 특유의 인지 저하 패턴, 보존된 능력, 단계별 적응을 포함합니다.

### 1.2 대상
- 치매 진단을 받은 사용자
- 경도 인지장애(MCI)에서 중증 치매까지 포괄
- AAC 기기로 의사소통 지원이 필요한 사용자

### 1.3 핵심 원칙
- **보존된 능력 활용**: 남아있는 기능을 최대한 활용
- **단계별 적응**: 진행 단계에 따른 점진적 지원 강화
- **존엄성 존중**: 인격과 역사를 존중하는 접근

---

## 2. DementiaProfile 구조

### 2.1 전체 구조

```typescript
interface DementiaProfile extends CognitiveProfile {
  // 프로파일 유형 식별
  profileType: 'dementia';

  // 진단 정보
  diagnosis: DementiaDiagnosis;

  // 보존된 능력
  preservedAbilities: PreservedAbilities;

  // 손상된 영역
  impairedDomains: ImpairedDomains;

  // 일상생활 기능
  functionalStatus: FunctionalStatus;

  // 행동/심리 증상 (BPSD)
  behavioralSymptoms: BehavioralSymptoms;

  // 개인사/선호
  personalHistory: PersonalHistory;

  // 치매 특화 AAC 적응
  dementiaAACAdaptation: DementiaAACAdaptation;
}
```

---

## 3. 진단 정보

### 3.1 DementiaDiagnosis

```typescript
interface DementiaDiagnosis {
  // 치매 유형
  type: DementiaType;

  // 진행 단계
  stage: DementiaStage;

  // 진단 일자
  diagnosisDate: string;

  // 마지막 평가 일자
  lastAssessmentDate: string;

  // 예상 진행 속도
  progressionRate?: 'slow' | 'typical' | 'rapid';

  // 동반 질환
  comorbidities?: string[];

  // 인지 평가 점수
  cognitiveScores?: {
    mmse?: number;           // 0-30
    moca?: number;           // 0-30
    cdr?: number;            // 0-3 (Clinical Dementia Rating)
  };
}

enum DementiaType {
  ALZHEIMERS = 'alzheimers',           // 알츠하이머병 (60-80%)
  VASCULAR = 'vascular',               // 혈관성 치매
  LEWY_BODY = 'lewy_body',             // 루이소체 치매
  FRONTOTEMPORAL = 'frontotemporal',   // 전두측두엽 치매
  MIXED = 'mixed',                     // 혼합형
  PARKINSONS = 'parkinsons',           // 파킨슨병 치매
  OTHER = 'other'                      // 기타
}

enum DementiaStage {
  PRECLINICAL = 'preclinical',   // 전임상 (증상 없음, 바이오마커만)
  MCI = 'mci',                   // 경도 인지장애
  EARLY = 'early',               // 초기 (경도 치매)
  MIDDLE = 'middle',             // 중기 (중등도 치매)
  LATE = 'late'                  // 말기 (중증 치매)
}
```

### 3.2 치매 유형별 특성

| 유형 | 초기 증상 | 진행 패턴 | AAC 고려사항 |
|------|----------|-----------|--------------|
| **알츠하이머** | 기억력 저하 | 점진적 | 친숙한 사진/회상 |
| **혈관성** | 갑작스런 변화 | 계단식 | 변동성 대응 |
| **루이소체** | 환각, 주의력 변동 | 변동적 | 시각적 혼란 최소화 |
| **전두측두엽** | 성격/언어 변화 | 행동→언어 or 역순 | 행동 지원 통합 |

### 3.3 단계별 특성

| 단계 | MMSE | CDR | 의사소통 특성 | AAC 복잡도 |
|------|:----:|:---:|--------------|-----------|
| **MCI** | 24-30 | 0.5 | 단어 찾기 어려움 | 표준 |
| **초기** | 20-24 | 1 | 복잡한 대화 어려움 | 중간 |
| **중기** | 13-20 | 2 | 간단한 문장만 | 낮음 |
| **말기** | <13 | 3 | 최소 언어/비언어 | 최소 |

---

## 4. 보존된 능력

### 4.1 PreservedAbilities

치매에서도 **마지막까지 보존되는 능력**을 식별합니다.

```typescript
interface PreservedAbilities {
  // 절차 기억 (마지막까지 보존)
  proceduralMemory: {
    preserved: boolean;
    examples: string[];        // 예: ["피아노 연주", "자전거 타기"]
    usableForAAC: boolean;     // AAC 조작에 활용 가능
  };

  // 정서적 기억/반응
  emotionalProcessing: {
    preserved: boolean;
    recognizesFamiliarFaces: boolean;
    respondsToTone: boolean;   // 목소리 톤에 반응
    respondsToMusic: boolean;  // 음악에 반응
  };

  // 음악 능력
  musicalAbility: {
    preserved: boolean;
    canSingFamiliarSongs: boolean;
    respondsToFamiliarMusic: boolean;
    preferredMusic: string[];  // 선호하는 음악/시대
  };

  // 사회적 예절
  socialEtiquette: {
    preserved: boolean;
    greetings: boolean;
    turnTaking: boolean;       // 차례 주고받기
  };

  // 원격 기억 (오래된 기억)
  remoteMemory: {
    preserved: boolean;
    accessiblePeriods: string[]; // 예: ["1960년대", "젊은 시절"]
  };

  // 읽기 능력 (종종 보존)
  readingAbility: {
    preserved: boolean;
    level: 'single_words' | 'short_phrases' | 'sentences';
  };
}
```

### 4.2 보존된 능력 활용 전략

| 보존 능력 | AAC 활용 |
|----------|----------|
| 절차 기억 | 일관된 조작 패턴 유지 |
| 음악 반응 | 음악 기반 피드백 |
| 원격 기억 | 과거 사진/회상 요소 |
| 읽기 능력 | 텍스트 레이블 활용 |
| 사회적 예절 | 인사/사교 표현 우선 배치 |

---

## 5. 손상된 영역

### 5.1 ImpairedDomains

```typescript
interface ImpairedDomains {
  // 기억력
  memory: {
    recentMemory: ImpairmentLevel;       // 최근 기억 (가장 먼저 손상)
    workingMemory: ImpairmentLevel;      // 작업 기억
    prospectiveMemory: ImpairmentLevel;  // 미래 기억 (할 일 기억)
    semanticMemory: ImpairmentLevel;     // 의미 기억 (단어/개념)
  };

  // 언어
  language: {
    wordFinding: ImpairmentLevel;        // 단어 찾기
    naming: ImpairmentLevel;             // 이름대기
    comprehension: ImpairmentLevel;      // 이해력
    repetition: ImpairmentLevel;         // 따라말하기
    reading: ImpairmentLevel;            // 읽기
    writing: ImpairmentLevel;            // 쓰기
  };

  // 실행 기능
  executive: {
    planning: ImpairmentLevel;           // 계획
    sequencing: ImpairmentLevel;         // 순서화
    judgment: ImpairmentLevel;           // 판단력
    problemSolving: ImpairmentLevel;     // 문제 해결
  };

  // 시공간
  visuospatial: {
    navigation: ImpairmentLevel;         // 길찾기
    objectRecognition: ImpairmentLevel;  // 물체 인식
    faceRecognition: ImpairmentLevel;    // 얼굴 인식
  };

  // 주의력
  attention: {
    sustained: ImpairmentLevel;          // 지속 주의
    divided: ImpairmentLevel;            // 분할 주의
    selective: ImpairmentLevel;          // 선택적 주의
  };
}

enum ImpairmentLevel {
  INTACT = 0,           // 정상
  MILD = 1,             // 경미한 손상
  MODERATE = 2,         // 중등도 손상
  SEVERE = 3,           // 심각한 손상
  PROFOUND = 4          // 완전 손상
}
```

---

## 6. 일상생활 기능

### 6.1 FunctionalStatus

```typescript
interface FunctionalStatus {
  // 기본 일상생활 활동 (ADL)
  basicADL: {
    eating: FunctionLevel;
    dressing: FunctionLevel;
    grooming: FunctionLevel;
    toileting: FunctionLevel;
    mobility: FunctionLevel;
  };

  // 도구적 일상생활 활동 (IADL)
  instrumentalADL: {
    communication: FunctionLevel;     // 의사소통
    transportation: FunctionLevel;    // 이동
    shopping: FunctionLevel;          // 쇼핑
    mealPreparation: FunctionLevel;   // 식사 준비
    housekeeping: FunctionLevel;      // 집안일
    medication: FunctionLevel;        // 약 복용
    finances: FunctionLevel;          // 재정 관리
  };

  // 의사결정 능력
  decisionMaking: {
    level: FunctionLevel;
    canMakeBasicChoices: boolean;     // 기본 선택 가능
    canExpressPreferences: boolean;   // 선호 표현 가능
    needsGuardian: boolean;           // 법정 후견인 필요
  };

  // 안전 인식
  safetyAwareness: {
    level: FunctionLevel;
    wanderingRisk: boolean;           // 배회 위험
    fallRisk: boolean;                // 낙상 위험
    needsSupervision: SupervisionLevel;
  };
}

enum FunctionLevel {
  INDEPENDENT = 0,      // 독립적
  MINIMAL_HELP = 1,     // 최소 도움 필요
  MODERATE_HELP = 2,    // 상당한 도움 필요
  MAXIMAL_HELP = 3,     // 최대 도움 필요
  TOTAL_DEPENDENCE = 4  // 완전 의존
}

enum SupervisionLevel {
  INDEPENDENT = 'independent',
  OCCASIONAL = 'occasional',       // 가끔
  FREQUENT = 'frequent',           // 자주
  CONSTANT = 'constant'            // 상시
}
```

---

## 7. 행동/심리 증상

### 7.1 BehavioralSymptoms (BPSD)

```typescript
interface BehavioralSymptoms {
  // 기분 관련
  mood: {
    depression: SeverityLevel;
    anxiety: SeverityLevel;
    apathy: SeverityLevel;          // 무관심/무의욕
    irritability: SeverityLevel;    // 짜증
  };

  // 정신병적 증상
  psychotic: {
    delusions: SeverityLevel;       // 망상
    hallucinations: SeverityLevel;  // 환각
    types?: string[];               // 환각 유형
  };

  // 행동 증상
  behavioral: {
    agitation: SeverityLevel;       // 초조/흥분
    aggression: SeverityLevel;      // 공격성
    wandering: SeverityLevel;       // 배회
    repetitiveActions: SeverityLevel; // 반복 행동
    sundowning: boolean;            // 일몰 증후군
  };

  // 수면
  sleep: {
    insomnia: SeverityLevel;
    daytimeSleepiness: SeverityLevel;
    sleepWakeCycleDisruption: boolean;
  };

  // 트리거 및 대처
  management: {
    knownTriggers: string[];        // 알려진 트리거
    calmingStrategies: string[];    // 진정 전략
    bestTimeOfDay: string;          // 가장 좋은 시간대
    worstTimeOfDay: string;         // 가장 어려운 시간대
  };
}

enum SeverityLevel {
  NONE = 0,
  MILD = 1,
  MODERATE = 2,
  SEVERE = 3
}
```

### 7.2 BPSD와 AAC 고려사항

| 증상 | AAC 고려사항 |
|------|-------------|
| 초조/흥분 | 진정/도움 요청 버튼 |
| 배회 | 위치/도움 요청 기능 |
| 무관심 | 자동 프롬프트, 친숙한 얼굴 |
| 일몰 증후군 | 시간대별 UI 조정 |
| 환각 | 시각적 혼란 최소화 |

---

## 8. 개인사/선호

### 8.1 PersonalHistory

**인간 중심 케어의 핵심**: 개인의 역사와 선호를 파악합니다.

```typescript
interface PersonalHistory {
  // 중요한 생애 정보
  lifeHistory: {
    occupation: string[];          // 직업 이력
    hobbies: string[];             // 취미
    significantPlaces: string[];   // 중요한 장소
    achievements: string[];        // 성취
  };

  // 가족/사회 관계
  socialConnections: {
    familyMembers: FamilyMember[];
    closeFriends: string[];
    pets: string[];
    significantRelationships: string[];
  };

  // 선호/기호
  preferences: {
    favoriteMusic: string[];       // 좋아하는 음악
    favoriteMovies: string[];      // 좋아하는 영화
    favoriteFood: string[];        // 좋아하는 음식
    dislikedThings: string[];      // 싫어하는 것
    dailyRoutines: string[];       // 일상 루틴
  };

  // 문화/종교
  culturalBackground: {
    ethnicity?: string;
    religion?: string;
    language: string;              // 모국어
    culturalPractices: string[];   // 문화적 관습
  };

  // 의사소통 이력
  communicationHistory: {
    premorbidStyle: string;        // 치매 전 의사소통 스타일
    preferredTopics: string[];     // 선호하는 대화 주제
    tabooTopics: string[];         // 피해야 할 주제
    meaningfulPhrases: string[];   // 의미 있는 표현
  };
}

interface FamilyMember {
  relationship: string;            // 관계 (예: "딸")
  name: string;                    // 이름
  photoId?: string;                // 사진 ID
  recognitionLevel: RecognitionLevel;
  visitFrequency?: string;         // 방문 빈도
}

enum RecognitionLevel {
  RECOGNIZES_FULLY = 'full',       // 완전히 인식
  RECOGNIZES_SOMETIMES = 'sometimes', // 가끔 인식
  RECOGNIZES_AS_FAMILIAR = 'familiar', // 친숙하게 느낌
  DOES_NOT_RECOGNIZE = 'none'      // 인식 못함
}
```

### 8.2 개인사 활용

| 정보 | AAC 활용 |
|------|----------|
| 가족 사진 | 홈 화면/빠른 접근 |
| 직업 이력 | 관련 어휘 포함 |
| 좋아하는 음악 | 피드백/진정 도구 |
| 일상 루틴 | 루틴 기반 어휘 구성 |
| 의미 있는 표현 | 자주 사용 버튼 |

---

## 9. 치매 특화 AAC 적응

### 9.1 DementiaAACAdaptation

```typescript
interface DementiaAACAdaptation {
  // 단계별 복잡도
  stageBasedComplexity: {
    currentStage: DementiaStage;
    itemsPerScreen: number;        // 화면당 항목 수
    navigationDepth: number;       // 메뉴 깊이
    autoSimplification: boolean;   // 자동 단순화
  };

  // 시각적 설계
  visualDesign: {
    useRealPhotos: boolean;        // 실제 사진 사용
    useFamiliarFaces: boolean;     // 친숙한 얼굴 포함
    highContrast: boolean;         // 고대비
    largeButtons: boolean;         // 큰 버튼
    minimalAnimation: boolean;     // 최소 애니메이션
    clearLabels: boolean;          // 명확한 레이블
  };

  // 회상 요소
  reminiscenceElements: {
    enabled: boolean;
    includeLifePhotos: boolean;    // 인생 사진 포함
    includeEraMusic: boolean;      // 시대 음악 포함
    includeMemoryPrompts: boolean; // 기억 촉진 프롬프트
  };

  // 시간 지원
  temporalSupport: {
    showDateTime: boolean;         // 날짜/시간 표시
    showDayPhase: boolean;         // 아침/점심/저녁 표시
    orientationPrompts: boolean;   // 지남력 프롬프트
  };

  // 반복/확인
  repetitionSupport: {
    allowRepeating: boolean;       // 반복 허용
    confirmationRequired: boolean; // 확인 필요
    undoAvailable: boolean;        // 취소 가능
    recentChoicesVisible: boolean; // 최근 선택 표시
  };

  // 돌봄자 연동
  caregiverIntegration: {
    quickAlerts: boolean;          // 빠른 알림 버튼
    needsButtons: string[];        // 필요 표현 버튼
    caregiverDashboard: boolean;   // 돌봄자 대시보드
  };

  // 일상 루틴 지원
  routineSupport: {
    enabled: boolean;
    mealTimePrompts: boolean;      // 식사 시간 프롬프트
    medicationReminders: boolean;  // 약 복용 알림
    activitySuggestions: boolean;  // 활동 제안
  };
}
```

### 9.2 단계별 AAC 복잡도 권장

| 단계 | 화면 항목 | 메뉴 깊이 | 상징 유형 | 주요 기능 |
|------|:--------:|:--------:|----------|-----------|
| **MCI** | 12-16 | 2-3 | 픽토그램 | 단어 찾기 지원 |
| **초기** | 8-12 | 1-2 | 사진+픽토그램 | 기본 표현, 회상 |
| **중기** | 4-8 | 1 | 실제 사진 | 필수 요구, 감정 |
| **말기** | 2-4 | 1 | 친숙한 얼굴 | 예/아니오, 기본 요구 |

---

## 10. 평가 도구 연동

### 10.1 지원 평가 도구

| 도구 | 용도 | 연동 데이터 |
|------|------|------------|
| **MMSE** | 인지 선별 | 점수 → 단계 |
| **MoCA** | 인지 선별 (MCI) | 점수 → 단계 |
| **CDR** | 치매 중증도 | 등급 → 복잡도 |
| **GDS** | 전반적 퇴화 척도 | 단계 → 적응 |
| **NPI** | 행동 증상 | BPSD → 적응 |

### 10.2 점수 → 프로파일 매핑

```typescript
function mapAssessmentToStage(scores: CognitiveScores): DementiaStage {
  const { mmse, moca, cdr } = scores;

  // CDR 우선
  if (cdr !== undefined) {
    if (cdr === 0) return DementiaStage.PRECLINICAL;
    if (cdr === 0.5) return DementiaStage.MCI;
    if (cdr === 1) return DementiaStage.EARLY;
    if (cdr === 2) return DementiaStage.MIDDLE;
    if (cdr === 3) return DementiaStage.LATE;
  }

  // MMSE 기반
  if (mmse !== undefined) {
    if (mmse >= 27) return DementiaStage.PRECLINICAL;
    if (mmse >= 24) return DementiaStage.MCI;
    if (mmse >= 20) return DementiaStage.EARLY;
    if (mmse >= 13) return DementiaStage.MIDDLE;
    return DementiaStage.LATE;
  }

  // MoCA 기반
  if (moca !== undefined) {
    if (moca >= 26) return DementiaStage.PRECLINICAL;
    if (moca >= 22) return DementiaStage.MCI;
    if (moca >= 17) return DementiaStage.EARLY;
    if (moca >= 10) return DementiaStage.MIDDLE;
    return DementiaStage.LATE;
  }

  return DementiaStage.MCI; // 기본값
}
```

---

## 11. 사용 예시

### 11.1 중기 알츠하이머 프로파일

```json
{
  "profileType": "dementia",
  "id": "uuid-example",
  "version": "1.0.0",
  "diagnosis": {
    "type": "alzheimers",
    "stage": "middle",
    "diagnosisDate": "2022-06-15",
    "lastAssessmentDate": "2024-12-01",
    "cognitiveScores": {
      "mmse": 16,
      "cdr": 2
    }
  },
  "preservedAbilities": {
    "proceduralMemory": {
      "preserved": true,
      "examples": ["바느질", "노래 부르기"],
      "usableForAAC": true
    },
    "emotionalProcessing": {
      "preserved": true,
      "recognizesFamiliarFaces": true,
      "respondsToTone": true,
      "respondsToMusic": true
    },
    "musicalAbility": {
      "preserved": true,
      "canSingFamiliarSongs": true,
      "preferredMusic": ["1970년대 가요", "찬송가"]
    },
    "remoteMemory": {
      "preserved": true,
      "accessiblePeriods": ["결혼 시절", "자녀 양육기"]
    }
  },
  "impairedDomains": {
    "memory": {
      "recentMemory": 3,
      "workingMemory": 3,
      "semanticMemory": 2
    },
    "language": {
      "wordFinding": 3,
      "naming": 2,
      "comprehension": 2
    },
    "executive": {
      "planning": 3,
      "sequencing": 3
    }
  },
  "functionalStatus": {
    "basicADL": {
      "eating": 1,
      "dressing": 2,
      "grooming": 2
    },
    "decisionMaking": {
      "level": 2,
      "canMakeBasicChoices": true,
      "canExpressPreferences": true
    },
    "safetyAwareness": {
      "level": 3,
      "wanderingRisk": true,
      "needsSupervision": "constant"
    }
  },
  "behavioralSymptoms": {
    "mood": {
      "anxiety": 2,
      "apathy": 1
    },
    "behavioral": {
      "agitation": 1,
      "sundowning": true
    },
    "management": {
      "knownTriggers": ["낯선 환경", "소음"],
      "calmingStrategies": ["좋아하는 음악", "손 마사지"],
      "bestTimeOfDay": "오전",
      "worstTimeOfDay": "저녁"
    }
  },
  "personalHistory": {
    "lifeHistory": {
      "occupation": ["초등학교 교사"],
      "hobbies": ["바느질", "정원 가꾸기"]
    },
    "socialConnections": {
      "familyMembers": [
        {
          "relationship": "딸",
          "name": "영희",
          "recognitionLevel": "full"
        },
        {
          "relationship": "아들",
          "name": "철수",
          "recognitionLevel": "sometimes"
        }
      ]
    },
    "preferences": {
      "favoriteMusic": ["옛날 가요"],
      "favoriteFood": ["된장찌개", "김치"]
    }
  },
  "dementiaAACAdaptation": {
    "stageBasedComplexity": {
      "currentStage": "middle",
      "itemsPerScreen": 6,
      "navigationDepth": 1,
      "autoSimplification": true
    },
    "visualDesign": {
      "useRealPhotos": true,
      "useFamiliarFaces": true,
      "highContrast": true,
      "largeButtons": true
    },
    "reminiscenceElements": {
      "enabled": true,
      "includeLifePhotos": true,
      "includeEraMusic": true
    },
    "caregiverIntegration": {
      "quickAlerts": true,
      "needsButtons": ["화장실", "물", "도움", "아파요"]
    },
    "routineSupport": {
      "enabled": true,
      "mealTimePrompts": true,
      "medicationReminders": true
    }
  }
}
```

---

## 12. 윤리적 고려사항

### 12.1 동의 및 의사결정

```typescript
interface ConsentConsiderations {
  // 동의 능력 평가
  capacityAssessment: {
    hasCapacity: boolean;
    assessmentDate: string;
    assessor: string;
  };

  // 사전 의향서
  advanceDirectives: {
    exists: boolean;
    communicationPreferences?: string;
    healthcareProxy?: string;
  };

  // 대리 의사결정자
  surrogate: {
    name: string;
    relationship: string;
    contactInfo: string;
    authority: 'full' | 'limited';
  };
}
```

### 12.2 존엄성 원칙

| 원칙 | 적용 |
|------|------|
| **자율성 존중** | 가능한 한 선택권 제공 |
| **비악의** | 좌절감 최소화 |
| **선행** | 의사소통 능력 극대화 |
| **정의** | 모든 단계에서 접근 가능 |

### 12.3 AAC에서의 존엄성

- **실패 최소화**: 오류를 줄이는 설계
- **아동화 방지**: 성인에 맞는 콘텐츠와 어휘
- **선호 존중**: 개인 이력과 선호 반영
- **긍정적 상호작용**: 성공 경험 강화

---

## 13. 참조

### 13.1 평가 도구
- [MMSE](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5889638/)
- [MoCA](https://www.mocatest.org/)
- [CDR - Clinical Dementia Rating](https://knightadrc.wustl.edu/cdr/cdr.htm)

### 13.2 치매 케어 가이드라인
- Alzheimer's Association Guidelines
- WHO iSupport Guidelines
- Dementia Care Practice Recommendations (Alzheimer's Association)

### 13.3 AAC 관련
- [ASHA Dementia Practice Portal](https://www.asha.org/practice-portal/clinical-topics/dementia/)
- [AAC and Dementia - USSAAC](https://ussaac.org/)

---

## 14. 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-01-14 | 초기 버전 |

---

<div align="center">

**WIA Cognitive AAC - Dementia Profile Standard**

*"기억이 희미해져도, 연결은 남아있다"*

**홍익인간** - 널리 인간을 이롭게 하라

</div>
