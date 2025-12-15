# WIA Cognitive AAC - Phase 1: Cognitive Profile Standard

## 철학
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

## 배경

인지장애(치매, 자폐 스펙트럼, 지적장애)를 가진 사용자는 기존 AAC 시스템을
효과적으로 사용하기 어렵습니다. 왜냐하면:

1. **획일적 인터페이스**: 모든 사용자에게 동일한 UI
2. **과부하**: 너무 많은 선택지로 인지 과부하
3. **일반화 어려움**: 한 상황에서 배운 것을 다른 상황에 적용 어려움
4. **평가 표준 부재**: 어떤 AAC가 더 효과적인지 비교 불가

---

## Phase 1 목표: 인지 프로파일 표준 정의

### 1.1 인지 영역 모델

```typescript
// 핵심 인지 영역
interface CognitiveDomains {
  memory: MemoryDomain;
  attention: AttentionDomain;
  language: LanguageDomain;
  executive: ExecutiveDomain;
  visualSpatial: VisualSpatialDomain;
  socialCognition: SocialCognitionDomain;
}

interface MemoryDomain {
  // 단기 기억 (초 단위)
  immediateRecall: {
    level: CognitiveLevel;
    span: number;              // 기억 폭 (항목 수)
    assessmentDate: Date;
  };

  // 작업 기억 (조작 필요)
  workingMemory: {
    level: CognitiveLevel;
    complexity: number;        // 처리 가능 복잡도
  };

  // 장기 기억
  longTermMemory: {
    episodic: CognitiveLevel;  // 일화 기억
    semantic: CognitiveLevel;  // 의미 기억
    procedural: CognitiveLevel; // 절차 기억
  };
}

interface AttentionDomain {
  // 지속 주의 (한 과제에 집중)
  sustained: {
    level: CognitiveLevel;
    durationMinutes: number;   // 집중 가능 시간
  };

  // 선택적 주의 (방해 자극 무시)
  selective: {
    level: CognitiveLevel;
    distractibility: number;   // 방해받기 쉬운 정도
  };

  // 분할 주의 (멀티태스킹)
  divided: {
    level: CognitiveLevel;
    maxTasks: number;
  };

  // 주의 전환
  shifting: {
    level: CognitiveLevel;
    transitionTime: number;    // 전환에 필요한 시간 (초)
  };
}

enum CognitiveLevel {
  PROFOUND = 1,    // 심각한 손상
  SEVERE = 2,      // 중증 손상
  MODERATE = 3,    // 중등도 손상
  MILD = 4,        // 경미한 손상
  TYPICAL = 5      // 정상 범위
}
```

### 1.2 자폐 스펙트럼 특화 프로파일

```typescript
interface AutismProfile extends CognitiveProfile {
  // 사회적 의사소통
  socialCommunication: {
    jointAttention: CognitiveLevel;     // 공동 주의
    socialReciprocity: CognitiveLevel;  // 사회적 상호성
    nonverbalCues: CognitiveLevel;      // 비언어적 단서 이해
  };

  // 제한적/반복적 행동
  restrictedBehaviors: {
    routineAdherence: SeverityLevel;    // 루틴 고집 정도
    sensoryInterests: string[];         // 감각 관심사
    specialInterests: string[];         // 특별 관심사
  };

  // 감각 처리
  sensoryProcessing: {
    visual: SensoryReactivity;
    auditory: SensoryReactivity;
    tactile: SensoryReactivity;
    vestibular: SensoryReactivity;
    proprioceptive: SensoryReactivity;
  };

  // AAC 적용 시사점
  aacImplications: {
    preferredModalitiy: 'visual' | 'auditory' | 'tactile';
    symbolPreference: 'photos' | 'pictograms' | 'text';
    layoutPreference: 'grid' | 'scene' | 'linear';
    transitionSupport: boolean;  // 전환 지원 필요
  };
}

enum SensoryReactivity {
  HYPO = 'hypo',       // 저반응
  TYPICAL = 'typical',
  HYPER = 'hyper'      // 과반응
}
```

### 1.3 치매 특화 프로파일

```typescript
interface DementiaProfile extends CognitiveProfile {
  // 진단 정보
  diagnosis: {
    type: DementiaType;
    stage: DementiaStage;
    diagnosisDate: Date;
    lastAssessment: Date;
  };

  // 보존된 능력
  preservedAbilities: {
    procedural: boolean;       // 절차 기억 보존
    emotional: boolean;        // 감정 인식 보존
    music: boolean;            // 음악 반응 보존
    longTermRecall: boolean;   // 장기 기억 일부 보존
  };

  // 일상생활 기능
  adlFunctions: {
    communication: FunctionLevel;
    orientation: FunctionLevel;
    decisionMaking: FunctionLevel;
  };

  // AAC 적용 시사점
  aacImplications: {
    usePhotos: boolean;        // 실제 사진 사용 (친숙한 얼굴)
    simplifiedLayout: boolean; // 단순화된 레이아웃
    audioSupport: boolean;     // 음성 지원
    reminiscence: boolean;     // 회상 요소 포함
  };
}

enum DementiaType {
  ALZHEIMERS = 'alzheimers',
  VASCULAR = 'vascular',
  LEWY_BODY = 'lewy_body',
  FRONTOTEMPORAL = 'frontotemporal',
  MIXED = 'mixed'
}

enum DementiaStage {
  EARLY = 'early',
  MIDDLE = 'middle',
  LATE = 'late'
}
```

---

## 연구 과제

1. **기존 평가 도구 조사**
   - ADOS-2 (자폐)
   - Vineland Adaptive Behavior Scales
   - MMSE, MoCA (치매)
   - Communication Matrix

2. **AAC 효과 측정 메트릭**
   - 의사소통 빈도
   - 의사소통 성공률
   - 어휘 확장 속도
   - 독립성 향상도

3. **윤리/프라이버시**
   - 인지 프로파일 데이터 보호
   - 동의 능력 고려
   - 대리 결정 프로토콜

---

## 산출물

```
cognitive-aac/
├── spec/
│   ├── RESEARCH-PHASE-1.md
│   ├── COGNITIVE-PROFILE-SPEC.md
│   ├── AUTISM-PROFILE-SPEC.md
│   ├── DEMENTIA-PROFILE-SPEC.md
│   └── ASSESSMENT-METRICS.md
├── schemas/
│   ├── cognitive-profile.schema.json
│   ├── autism-profile.schema.json
│   └── dementia-profile.schema.json
└── docs/
    └── PHASE-1-SUMMARY.md
```

---

## 다음 단계

Phase 1 완료 후:
- `prompts/PHASE-2-PROMPT.md` 읽고 Phase 2 (적응형 UI 엔진) 시작
