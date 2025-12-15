# WIA Cognitive AAC - Cognitive Profile Specification
# 인지 프로파일 표준 명세서

**Version**: 1.0.0
**Date**: 2025-01-14
**Status**: Draft

---

## 철학
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

---

## 1. 개요

### 1.1 목적
본 명세서는 인지장애를 가진 AAC 사용자의 인지 능력을 표준화된 형식으로 표현하기 위한 데이터 구조를 정의합니다.

### 1.2 범위
- 핵심 인지 영역 정의
- 수준 체계 정의
- AAC 적응 매개변수 정의
- 데이터 교환 형식 (JSON Schema)

### 1.3 용어 정의
| 용어 | 정의 |
|------|------|
| **CognitiveProfile** | 개인의 인지 능력을 구조화한 데이터 객체 |
| **CognitiveLevel** | 1-5 척도의 인지 능력 수준 |
| **AAC Adaptation** | 인지 프로파일 기반 AAC 인터페이스 조정 |

---

## 2. 인지 영역 모델

### 2.1 핵심 인지 영역
```
CognitiveDomains
├── Memory (기억)
├── Attention (주의)
├── Language (언어)
├── ExecutiveFunction (실행 기능)
├── VisualSpatial (시공간)
└── SocialCognition (사회 인지)
```

### 2.2 영역별 상세 정의

#### 2.2.1 기억 영역 (Memory Domain)

| 하위 영역 | 설명 | 측정 지표 |
|-----------|------|-----------|
| **Immediate Recall** | 초 단위 즉시 회상 | 기억 폭 (span) |
| **Working Memory** | 정보 조작 능력 | 복잡도 수준 |
| **Long-term Episodic** | 개인 경험 기억 | 회상 정확도 |
| **Long-term Semantic** | 사실/개념 기억 | 지식 접근성 |
| **Long-term Procedural** | 기술/절차 기억 | 자동화 수준 |

```typescript
interface MemoryDomain {
  immediateRecall: {
    level: CognitiveLevel;     // 1-5
    span: number;              // 기억 폭 (항목 수, 일반적 7±2)
    assessmentDate: string;    // ISO 8601
  };

  workingMemory: {
    level: CognitiveLevel;
    complexity: number;        // 처리 가능 복잡도 (1-10)
  };

  longTermMemory: {
    episodic: CognitiveLevel;
    semantic: CognitiveLevel;
    procedural: CognitiveLevel;
  };
}
```

#### 2.2.2 주의 영역 (Attention Domain)

| 하위 영역 | 설명 | 측정 지표 |
|-----------|------|-----------|
| **Sustained** | 지속 주의 | 집중 가능 시간 (분) |
| **Selective** | 선택적 주의 | 방해 저항도 |
| **Divided** | 분할 주의 | 동시 과제 수 |
| **Shifting** | 주의 전환 | 전환 시간 (초) |

```typescript
interface AttentionDomain {
  sustained: {
    level: CognitiveLevel;
    durationMinutes: number;   // 최대 집중 시간
  };

  selective: {
    level: CognitiveLevel;
    distractibility: number;   // 1-10 (높을수록 방해받기 쉬움)
  };

  divided: {
    level: CognitiveLevel;
    maxTasks: number;          // 동시 처리 가능 과제 수
  };

  shifting: {
    level: CognitiveLevel;
    transitionTimeSeconds: number;  // 전환에 필요한 시간
  };
}
```

#### 2.2.3 언어 영역 (Language Domain)

| 하위 영역 | 설명 | 측정 지표 |
|-----------|------|-----------|
| **Receptive** | 수용 언어 | 이해 복잡도 |
| **Expressive** | 표현 언어 | 표현 복잡도 |
| **Naming** | 이름대기 | 정확도/속도 |
| **Repetition** | 따라말하기 | 길이/정확도 |

```typescript
interface LanguageDomain {
  receptive: {
    level: CognitiveLevel;
    complexityLevel: number;   // 이해 가능한 문장 복잡도
  };

  expressive: {
    level: CognitiveLevel;
    complexityLevel: number;   // 표현 가능한 문장 복잡도
  };

  naming: {
    level: CognitiveLevel;
    accuracy: number;          // 0-100%
  };

  repetition: {
    level: CognitiveLevel;
    maxLength: number;         // 따라말할 수 있는 최대 길이
  };
}
```

#### 2.2.4 실행 기능 영역 (Executive Function Domain)

| 하위 영역 | 설명 | 측정 지표 |
|-----------|------|-----------|
| **Planning** | 계획 수립 | 단계 수 |
| **Inhibition** | 억제 | 오류율 |
| **Flexibility** | 인지 유연성 | 적응 속도 |
| **Problem Solving** | 문제 해결 | 복잡도 |

```typescript
interface ExecutiveFunctionDomain {
  planning: {
    level: CognitiveLevel;
    maxSteps: number;          // 계획 가능한 최대 단계
  };

  inhibition: {
    level: CognitiveLevel;
    errorRate: number;         // 억제 실패율 (0-100%)
  };

  flexibility: {
    level: CognitiveLevel;
    adaptationSpeed: number;   // 1-10
  };

  problemSolving: {
    level: CognitiveLevel;
    maxComplexity: number;     // 해결 가능한 문제 복잡도
  };
}
```

#### 2.2.5 시공간 영역 (Visual-Spatial Domain)

```typescript
interface VisualSpatialDomain {
  visualPerception: {
    level: CognitiveLevel;
    acuity: string;            // 예: "20/20"
  };

  spatialOrientation: {
    level: CognitiveLevel;
  };

  construction: {
    level: CognitiveLevel;     // 구성 능력
  };
}
```

#### 2.2.6 사회 인지 영역 (Social Cognition Domain)

```typescript
interface SocialCognitionDomain {
  emotionRecognition: {
    level: CognitiveLevel;
    modalities: ('face' | 'voice' | 'body')[];
  };

  theoryOfMind: {
    level: CognitiveLevel;     // 타인의 마음 이해
  };

  socialJudgment: {
    level: CognitiveLevel;     // 사회적 판단
  };
}
```

---

## 3. 인지 수준 척도

### 3.1 CognitiveLevel 정의

| 수준 | 값 | 레이블 | 설명 | AAC 복잡도 권장 |
|------|:--:|--------|------|-----------------|
| **PROFOUND** | 1 | 심각한 손상 | 기능 거의 불가 | 최소 (2-4 항목) |
| **SEVERE** | 2 | 중증 손상 | 심각한 제한 | 매우 낮음 (4-6 항목) |
| **MODERATE** | 3 | 중등도 손상 | 상당한 지원 필요 | 낮음 (6-9 항목) |
| **MILD** | 4 | 경미한 손상 | 약간의 지원 필요 | 중간 (9-16 항목) |
| **TYPICAL** | 5 | 정상 범위 | 지원 불필요 | 표준 (16+ 항목) |

### 3.2 수준 결정 기준

```typescript
enum CognitiveLevel {
  PROFOUND = 1,    // < 1st percentile
  SEVERE = 2,      // 1st - 5th percentile
  MODERATE = 3,    // 5th - 16th percentile
  MILD = 4,        // 16th - 25th percentile
  TYPICAL = 5      // > 25th percentile
}
```

---

## 4. 통합 인지 프로파일

### 4.1 CognitiveProfile 구조

```typescript
interface CognitiveProfile {
  // 메타데이터
  id: string;                      // UUID
  version: string;                 // 스키마 버전
  createdAt: string;               // ISO 8601
  updatedAt: string;               // ISO 8601

  // 대상자 정보
  subject: {
    id: string;                    // 익명화된 ID
    birthYear?: number;            // 출생년도 (선택)
    primaryDiagnosis?: string;     // 주 진단명 (선택)
  };

  // 인지 영역
  domains: {
    memory: MemoryDomain;
    attention: AttentionDomain;
    language: LanguageDomain;
    executive: ExecutiveFunctionDomain;
    visualSpatial: VisualSpatialDomain;
    socialCognition: SocialCognitionDomain;
  };

  // 전체 요약
  summary: {
    overallLevel: CognitiveLevel;  // 종합 수준
    strengths: string[];           // 강점 영역
    challenges: string[];          // 도전 영역
  };

  // AAC 적응 권장사항
  aacRecommendations: AACAdaptation;

  // 평가 정보
  assessment: {
    tool: string;                  // 사용된 평가 도구
    assessor?: string;             // 평가자 (선택)
    date: string;                  // 평가 일자
    notes?: string;                // 추가 메모
  };
}
```

### 4.2 AAC 적응 권장사항

```typescript
interface AACAdaptation {
  // 인터페이스 복잡도
  interface: {
    maxItemsPerScreen: number;     // 화면당 최대 항목 수
    layoutType: 'grid' | 'list' | 'scene';
    iconSize: 'large' | 'medium' | 'small';
    animationsEnabled: boolean;
  };

  // 상징 유형
  symbols: {
    preferredType: 'photos' | 'pictograms' | 'text' | 'mixed';
    abstractionLevel: 1 | 2 | 3;   // 1=구체적, 3=추상적
    labelPosition: 'above' | 'below' | 'none';
  };

  // 입력 방식
  input: {
    methods: ('touch' | 'switch' | 'eyeGaze' | 'voice')[];
    dwellTimeMs?: number;          // 응시 시간 (시선 추적)
    scanSpeed?: number;            // 스캔 속도 (스위치)
  };

  // 피드백
  feedback: {
    auditory: boolean;
    visual: boolean;
    haptic: boolean;
    confirmationRequired: boolean;
  };

  // 진행 속도
  pacing: {
    autoAdvance: boolean;
    delayBetweenItemsMs: number;
    timeoutEnabled: boolean;
    timeoutMs?: number;
  };
}
```

---

## 5. 프로파일 생성 및 업데이트

### 5.1 초기 프로파일 생성

```typescript
function createCognitiveProfile(
  subjectId: string,
  assessmentData: AssessmentData
): CognitiveProfile {
  return {
    id: generateUUID(),
    version: "1.0.0",
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString(),
    subject: {
      id: subjectId,
    },
    domains: mapAssessmentToDomains(assessmentData),
    summary: calculateSummary(assessmentData),
    aacRecommendations: generateRecommendations(assessmentData),
    assessment: {
      tool: assessmentData.toolName,
      date: assessmentData.date,
    }
  };
}
```

### 5.2 프로파일 업데이트

```typescript
function updateCognitiveProfile(
  existing: CognitiveProfile,
  newAssessment: AssessmentData
): CognitiveProfile {
  return {
    ...existing,
    updatedAt: new Date().toISOString(),
    domains: mergeDomains(existing.domains, mapAssessmentToDomains(newAssessment)),
    summary: calculateSummary(newAssessment),
    aacRecommendations: generateRecommendations(newAssessment),
    assessment: {
      tool: newAssessment.toolName,
      date: newAssessment.date,
    }
  };
}
```

---

## 6. 프라이버시 및 보안

### 6.1 데이터 민감성
인지 프로파일은 **민감한 건강 정보**로 분류되며, 다음 원칙을 따라야 합니다:

| 원칙 | 설명 |
|------|------|
| **최소화** | 필요한 정보만 수집 |
| **익명화** | 직접 식별 정보 제거 |
| **암호화** | 전송/저장 시 암호화 |
| **동의** | 명시적 동의 획득 |
| **접근 제어** | 권한 기반 접근 |

### 6.2 동의 프로토콜

```typescript
interface ConsentRecord {
  profileId: string;
  consentGivenBy: 'self' | 'guardian' | 'legal_representative';
  consentType: 'full' | 'limited' | 'research_only';
  consentDate: string;
  expirationDate?: string;
  withdrawalDate?: string;
}
```

### 6.3 대리 결정

인지 능력이 심각하게 저하된 경우, 다음 순서로 대리 동의를 구합니다:

1. 법정 대리인 (Legal Guardian)
2. 의료 대리인 (Healthcare Proxy)
3. 가까운 가족 구성원 (Next of Kin)
4. 윤리 위원회 결정 (Ethics Committee)

---

## 7. 상호운용성

### 7.1 데이터 형식
- **JSON**: 기본 교환 형식
- **JSON Schema**: 검증용 스키마 (draft-07)
- **FHIR**: 의료 시스템 연동 (선택)

### 7.2 API 엔드포인트 (예시)

```
GET    /profiles/{id}           # 프로파일 조회
POST   /profiles                # 프로파일 생성
PUT    /profiles/{id}           # 프로파일 업데이트
DELETE /profiles/{id}           # 프로파일 삭제
GET    /profiles/{id}/history   # 변경 이력 조회
```

### 7.3 이벤트

```typescript
type ProfileEvent =
  | { type: 'PROFILE_CREATED'; payload: CognitiveProfile }
  | { type: 'PROFILE_UPDATED'; payload: { id: string; changes: Partial<CognitiveProfile> } }
  | { type: 'PROFILE_ARCHIVED'; payload: { id: string; reason: string } };
```

---

## 8. 확장

### 8.1 조건 특화 프로파일
기본 `CognitiveProfile`을 확장하여 특정 조건에 특화된 프로파일을 정의할 수 있습니다:

- **AutismProfile**: 자폐 스펙트럼 특화 (별도 명세서)
- **DementiaProfile**: 치매 특화 (별도 명세서)
- **TBIProfile**: 외상성 뇌손상 특화 (향후)
- **StrokeProfile**: 뇌졸중 특화 (향후)

### 8.2 확장 예시

```typescript
interface AutismProfile extends CognitiveProfile {
  autismSpecific: {
    socialCommunication: { ... };
    restrictedBehaviors: { ... };
    sensoryProcessing: { ... };
  };
}
```

---

## 9. 참조

### 9.1 관련 표준
- ISO 21801-1:2020 - Cognitive accessibility
- WCAG 2.2 - Web Content Accessibility Guidelines
- ADA Standards for Accessible Design

### 9.2 참고 평가 도구
- ADOS-2 (자폐)
- Vineland-3 (적응 행동)
- MMSE/MoCA (치매)
- Communication Matrix (의사소통)

---

## 10. 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-01-14 | 초기 버전 |

---

<div align="center">

**WIA Cognitive AAC Standard**

**홍익인간** - 널리 인간을 이롭게 하라

</div>
