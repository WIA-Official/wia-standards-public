# WIA Cognitive AAC - Autism Profile Specification
# 자폐 스펙트럼 프로파일 표준 명세서

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
자폐 스펙트럼 장애(ASD)를 가진 AAC 사용자를 위한 특화된 프로파일 구조를 정의합니다. 이 프로파일은 기본 `CognitiveProfile`을 확장하여 ASD 특유의 인지, 감각, 사회적 특성을 포함합니다.

### 1.2 대상
- 자폐 스펙트럼 장애 진단을 받은 사용자
- 최소 언어 또는 비언어 자폐인
- AAC 기기를 사용하거나 필요로 하는 자폐인

### 1.3 DSM-5 진단 기준 기반
본 명세서는 DSM-5의 자폐 스펙트럼 장애 진단 기준을 참조합니다:
- 영역 A: 사회적 의사소통 및 상호작용의 결함
- 영역 B: 제한적, 반복적 행동/관심/활동 패턴

---

## 2. AutismProfile 구조

### 2.1 전체 구조

```typescript
interface AutismProfile extends CognitiveProfile {
  // 프로파일 유형 식별
  profileType: 'autism';

  // 진단 정보
  diagnosis: AutismDiagnosis;

  // 사회적 의사소통 (DSM-5 영역 A)
  socialCommunication: SocialCommunicationDomain;

  // 제한적/반복적 행동 (DSM-5 영역 B)
  restrictedBehaviors: RestrictedBehaviorsDomain;

  // 감각 처리
  sensoryProcessing: SensoryProcessingDomain;

  // 강점 프로파일
  strengthsProfile: AutismStrengths;

  // AAC 특화 적응
  autismAACAdaptation: AutismAACAdaptation;
}
```

---

## 3. 진단 정보

### 3.1 AutismDiagnosis

```typescript
interface AutismDiagnosis {
  // DSM-5 지원 수준
  supportLevel: SupportLevel;

  // 언어 동반 여부
  languageStatus: LanguageStatus;

  // 지적 동반 여부
  intellectualStatus: IntellectualStatus;

  // 진단 연령
  diagnosisAge?: number;

  // 진단 일자
  diagnosisDate?: string;

  // 동반 진단
  comorbidities?: string[];
}

enum SupportLevel {
  LEVEL_1 = 1,  // "지원 필요" (Requiring Support)
  LEVEL_2 = 2,  // "상당한 지원 필요" (Requiring Substantial Support)
  LEVEL_3 = 3   // "매우 상당한 지원 필요" (Requiring Very Substantial Support)
}

type LanguageStatus =
  | 'nonverbal'           // 구어 없음
  | 'minimally_verbal'    // 최소 언어 (< 30 단어)
  | 'verbal_limited'      // 제한적 언어
  | 'verbal_fluent';      // 유창한 언어

type IntellectualStatus =
  | 'intellectual_disability'  // 지적장애 동반
  | 'borderline'              // 경계선
  | 'average'                 // 평균
  | 'above_average';          // 평균 이상
```

### 3.2 DSM-5 지원 수준 설명

| 수준 | 사회적 의사소통 | 제한적/반복적 행동 |
|:----:|----------------|-------------------|
| **Level 1** | 지원 없이 눈에 띄는 결함. 사회적 상호작용 시작 어려움 | 행동 유연성 문제로 일부 상황에서 기능 저하 |
| **Level 2** | 명확한 결함. 제한된 사회적 상호작용 시작 | 행동 변화 어려움. 빈번한 제한적 행동 |
| **Level 3** | 심각한 결함. 매우 제한된 사회적 상호작용 시작 | 극도의 행동 변화 어려움. 심각한 고통 |

---

## 4. 사회적 의사소통 영역

### 4.1 SocialCommunicationDomain

```typescript
interface SocialCommunicationDomain {
  // 공동 주의 (Joint Attention)
  jointAttention: {
    level: CognitiveLevel;
    canInitiate: boolean;      // 공동 주의 시작 가능 여부
    canRespond: boolean;       // 공동 주의 반응 가능 여부
    preferredCues: ('pointing' | 'gaze' | 'verbal')[];
  };

  // 사회적 상호성 (Social Reciprocity)
  socialReciprocity: {
    level: CognitiveLevel;
    turnTaking: boolean;       // 차례 주고받기 가능
    emotionalReciprocity: CognitiveLevel;  // 정서적 상호성
  };

  // 비언어적 의사소통
  nonverbalCommunication: {
    level: CognitiveLevel;
    usesFacialExpressions: boolean;
    usesGestures: boolean;
    understandsFacialExpressions: CognitiveLevel;
    understandsBodyLanguage: CognitiveLevel;
    eyeContact: EyeContactLevel;
  };

  // 사회적 관계
  socialRelationships: {
    level: CognitiveLevel;
    interestInPeers: boolean;
    abilityToMaintainFriendships: CognitiveLevel;
  };
}

enum EyeContactLevel {
  AVOIDS = 1,          // 회피
  MINIMAL = 2,         // 최소
  INCONSISTENT = 3,    // 불규칙
  FUNCTIONAL = 4,      // 기능적
  TYPICAL = 5          // 정상
}
```

### 4.2 AAC 적용 시사점

| 특성 | AAC 적응 |
|------|----------|
| 공동 주의 제한 | 명확한 시각적 하이라이트 |
| 낮은 눈맞춤 | 응시 기반 인터페이스 주의 |
| 비언어적 단서 이해 어려움 | 명시적 레이블/설명 |
| 사회적 상호성 제한 | 스크립트/템플릿 제공 |

---

## 5. 제한적/반복적 행동 영역

### 5.1 RestrictedBehaviorsDomain

```typescript
interface RestrictedBehaviorsDomain {
  // 반복적 운동/언어
  stereotypedBehaviors: {
    severity: SeverityLevel;
    motorStereotypies: string[];   // 예: ["손 펄럭이기", "회전"]
    vocalStereotypies: string[];   // 예: ["반향어", "지연 반향어"]
    selfStimulation: string[];     // 예: ["시각 자극", "청각 자극"]
  };

  // 루틴/동일성 고집
  routineAdherence: {
    severity: SeverityLevel;
    rigidityLevel: number;         // 1-10
    transitionDifficulty: number;  // 1-10
    ritualBehaviors: string[];     // 특정 의식 행동
  };

  // 특별 관심사
  specialInterests: {
    topics: SpecialInterest[];
    intensity: number;             // 1-10
    usableForAAC: boolean;         // AAC 동기부여로 활용 가능 여부
  };

  // 감각 관심/혐오
  sensoryInterests: {
    seeking: string[];             // 추구하는 감각
    avoiding: string[];            // 회피하는 감각
  };
}

interface SpecialInterest {
  topic: string;                   // 주제
  intensity: number;               // 강도 (1-10)
  ageOfOnset?: number;             // 시작 연령
  functionalUse: boolean;          // 기능적 활용 가능
}

enum SeverityLevel {
  NONE = 0,
  MILD = 1,
  MODERATE = 2,
  SEVERE = 3
}
```

### 5.2 AAC 적용 시사점

| 특성 | AAC 적응 |
|------|----------|
| 루틴 고집 | 일관된 인터페이스 레이아웃 |
| 전환 어려움 | 전환 지원 시각 타이머 |
| 특별 관심사 | 관심사 기반 어휘 개인화 |
| 감각 추구 | 감각 피드백 옵션 |

---

## 6. 감각 처리 영역

### 6.1 SensoryProcessingDomain

```typescript
interface SensoryProcessingDomain {
  // 시각
  visual: SensoryProfile;

  // 청각
  auditory: SensoryProfile;

  // 촉각
  tactile: SensoryProfile;

  // 전정 (균형)
  vestibular: SensoryProfile;

  // 고유수용 (신체 위치)
  proprioceptive: SensoryProfile;

  // 미각/후각
  gustatory: SensoryProfile;
  olfactory: SensoryProfile;

  // 통합 처리
  multiSensoryIntegration: CognitiveLevel;
}

interface SensoryProfile {
  reactivity: SensoryReactivity;
  seekingBehaviors: string[];      // 추구 행동
  avoidanceBehaviors: string[];    // 회피 행동
  specificTriggers?: string[];     // 특정 트리거
  copingStrategies?: string[];     // 대처 전략
}

enum SensoryReactivity {
  HYPO = 'hypo',         // 저반응
  TYPICAL = 'typical',   // 정상
  HYPER = 'hyper',       // 과반응
  MIXED = 'mixed'        // 혼합
}
```

### 6.2 감각 반응성 설명

| 반응성 | 설명 | 예시 |
|--------|------|------|
| **Hypo (저반응)** | 자극에 둔감, 강한 입력 추구 | 큰 소리 선호, 강한 압력 추구 |
| **Hyper (과반응)** | 자극에 과민, 회피 | 밝은 빛 회피, 가벼운 터치 거부 |
| **Mixed (혼합)** | 상황에 따라 저/과반응 | 일부 소리 추구, 다른 소리 회피 |

### 6.3 AAC 감각 적응

```typescript
interface SensoryAACAdaptation {
  visual: {
    brightness: 'low' | 'medium' | 'high';
    contrast: 'low' | 'medium' | 'high';
    animationsEnabled: boolean;
    preferredColors?: string[];
    avoidColors?: string[];
  };

  auditory: {
    soundEnabled: boolean;
    volume: number;               // 0-100
    preferredSoundTypes: string[];
    avoidSoundTypes: string[];
  };

  tactile: {
    hapticFeedback: boolean;
    feedbackIntensity: 'light' | 'medium' | 'strong';
  };
}
```

---

## 7. 강점 프로파일

### 7.1 AutismStrengths

자폐 스펙트럼의 강점을 식별하고 AAC에 활용합니다.

```typescript
interface AutismStrengths {
  // 시각적 강점
  visualStrengths: {
    hasStrength: boolean;
    visualMemory: CognitiveLevel;
    patternRecognition: CognitiveLevel;
    detailOrientation: CognitiveLevel;
  };

  // 체계적 사고
  systematicThinking: {
    hasStrength: boolean;
    categorization: CognitiveLevel;
    ruleFollowing: CognitiveLevel;
    sequencing: CognitiveLevel;
  };

  // 기억 강점
  memoryStrengths: {
    hasStrength: boolean;
    roteMemory: CognitiveLevel;
    factualMemory: CognitiveLevel;
    proceduralMemory: CognitiveLevel;
  };

  // 관심사 지식
  interestExpertise: {
    hasStrength: boolean;
    depthOfKnowledge: CognitiveLevel;
    topics: string[];
  };

  // 정직성/신뢰성
  reliabilityStrengths: {
    honesty: boolean;
    consistency: boolean;
    attention_to_rules: boolean;
  };
}
```

### 7.2 강점 기반 AAC 설계

| 강점 | AAC 활용 |
|------|----------|
| 시각적 강점 | 시각 기반 인터페이스 |
| 패턴 인식 | 일관된 레이아웃 패턴 |
| 규칙 따르기 | 명확한 구조/규칙 |
| 관심사 전문성 | 관심사 기반 어휘 |
| 절차 기억 | 일관된 조작 순서 |

---

## 8. 자폐 특화 AAC 적응

### 8.1 AutismAACAdaptation

```typescript
interface AutismAACAdaptation {
  // 상징 선호
  symbolPreferences: {
    type: 'photos' | 'pictograms' | 'text' | 'mixed';
    realismLevel: 1 | 2 | 3 | 4 | 5;  // 1=매우 사실적, 5=매우 추상적
    consistentStyle: boolean;         // 일관된 스타일 필요
    preferredSystems?: string[];      // 예: ["PCS", "PECS", "Widgit"]
  };

  // 레이아웃 선호
  layoutPreferences: {
    type: 'grid' | 'scene' | 'linear' | 'categorical';
    predictable: boolean;             // 예측 가능한 위치
    minimalClutter: boolean;          // 최소 시각적 혼란
    categoryBased: boolean;           // 범주 기반 구성
  };

  // 전환 지원
  transitionSupport: {
    required: boolean;
    visualTimers: boolean;            // 시각 타이머
    countdowns: boolean;              // 카운트다운
    socialStories: boolean;           // 사회 이야기
    firstThenBoards: boolean;         // "먼저-다음" 보드
  };

  // 사회적 의사소통 지원
  socialSupport: {
    conversationScripts: boolean;     // 대화 스크립트
    socialPhraseBank: boolean;        // 사회적 표현 모음
    emotionRecognitionAids: boolean;  // 감정 인식 도움
    pragmaticPrompts: boolean;        // 화용론적 프롬프트
  };

  // 자기 조절 도구
  selfRegulation: {
    calmingStrategies: boolean;       // 진정 전략 메뉴
    breakRequests: boolean;           // 휴식 요청 버튼
    sensoryTools: boolean;            // 감각 도구 접근
    emotionScales: boolean;           // 감정 척도/온도계
  };

  // 동기 부여
  motivation: {
    useSpecialInterests: boolean;     // 특별 관심사 활용
    tokenEconomy: boolean;            // 토큰 이코노미
    visualRewards: boolean;           // 시각적 보상
    predictableRewards: boolean;      // 예측 가능한 보상
  };
}
```

---

## 9. 평가 도구 연동

### 9.1 지원 평가 도구

| 도구 | 연동 영역 |
|------|-----------|
| **ADOS-2** | 사회적 의사소통, 제한적 행동 |
| **Vineland-3** | 적응 행동, 의사소통 |
| **Sensory Profile 2** | 감각 처리 |
| **Communication Matrix** | 의사소통 수준 |
| **ABLLS-R** | 언어/학습 기술 |

### 9.2 ADOS-2 → AutismProfile 매핑

```typescript
function mapADOS2ToProfile(ados2Results: ADOS2Results): Partial<AutismProfile> {
  return {
    diagnosis: {
      supportLevel: calculateSupportLevel(ados2Results.comparisonScore),
      languageStatus: mapLanguageFromModule(ados2Results.module),
    },
    socialCommunication: {
      jointAttention: {
        level: mapADOSItemToLevel(ados2Results.items.jointAttention),
        // ...
      },
      // ...
    },
    restrictedBehaviors: {
      stereotypedBehaviors: {
        severity: mapADOSSeverity(ados2Results.rrbDomain),
        // ...
      },
      // ...
    }
  };
}
```

---

## 10. 사용 예시

### 10.1 최소 언어 자폐 아동 프로파일

```json
{
  "profileType": "autism",
  "id": "uuid-example",
  "version": "1.0.0",
  "diagnosis": {
    "supportLevel": 2,
    "languageStatus": "minimally_verbal",
    "intellectualStatus": "average",
    "diagnosisAge": 3
  },
  "socialCommunication": {
    "jointAttention": {
      "level": 3,
      "canInitiate": false,
      "canRespond": true,
      "preferredCues": ["pointing"]
    },
    "nonverbalCommunication": {
      "level": 2,
      "usesFacialExpressions": false,
      "usesGestures": true,
      "eyeContact": 2
    }
  },
  "restrictedBehaviors": {
    "routineAdherence": {
      "severity": 2,
      "rigidityLevel": 7,
      "transitionDifficulty": 8
    },
    "specialInterests": {
      "topics": [
        {"topic": "기차", "intensity": 9, "functionalUse": true}
      ],
      "usableForAAC": true
    }
  },
  "sensoryProcessing": {
    "auditory": {
      "reactivity": "hyper",
      "avoidanceBehaviors": ["귀 막기", "시끄러운 곳 탈출"]
    },
    "visual": {
      "reactivity": "typical"
    },
    "tactile": {
      "reactivity": "hyper",
      "avoidanceBehaviors": ["특정 옷감 거부"]
    }
  },
  "strengthsProfile": {
    "visualStrengths": {
      "hasStrength": true,
      "visualMemory": 5,
      "patternRecognition": 5
    },
    "memoryStrengths": {
      "hasStrength": true,
      "roteMemory": 5
    }
  },
  "autismAACAdaptation": {
    "symbolPreferences": {
      "type": "photos",
      "realismLevel": 2,
      "consistentStyle": true
    },
    "layoutPreferences": {
      "type": "grid",
      "predictable": true,
      "minimalClutter": true
    },
    "transitionSupport": {
      "required": true,
      "visualTimers": true,
      "firstThenBoards": true
    },
    "motivation": {
      "useSpecialInterests": true
    }
  }
}
```

---

## 11. 프라이버시 고려사항

### 11.1 민감 정보 처리
자폐 프로파일은 특히 민감한 정보를 포함하므로:

- 진단 정보는 필요한 경우에만 공유
- 감각 트리거 정보는 안전 목적으로만 사용
- 특별 관심사는 동기 부여 목적으로 긍정적으로 활용
- 행동 특성은 낙인 없이 지원 계획에 활용

### 11.2 강점 기반 접근
프로파일은 결함보다 **강점 기반** 관점을 취합니다:

> "자폐는 다름이지, 결함이 아닙니다."

---

## 12. 참조

### 12.1 진단 기준
- DSM-5: Diagnostic and Statistical Manual of Mental Disorders, 5th Edition
- ICD-11: International Classification of Diseases, 11th Revision

### 12.2 평가 도구
- [ADOS-2](https://www.wpspublish.com/ados-2)
- [Sensory Profile 2](https://www.pearsonassessments.com/sensory-profile-2)
- [Vineland-3](https://www.pearsonassessments.com/vineland-3)

### 12.3 AAC 관련
- [ASHA AAC Guidelines](https://www.asha.org/practice-portal/professional-issues/augmentative-and-alternative-communication/)
- [ISAAC](https://www.isaac-online.org)

---

## 13. 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-01-14 | 초기 버전 |

---

<div align="center">

**WIA Cognitive AAC - Autism Profile Standard**

*"다름을 이해하고, 강점을 활용하여, 소통을 연결하다"*

**홍익인간** - 널리 인간을 이롭게 하라

</div>
