# WIA Voice-Sign: Emergency Communication Specification

## 1. Overview

본 문서는 긴급 상황에서의 음성-수화 통신 프로토콜을 정의합니다.

**Version:** 1.0.0
**Status:** Draft

---

## 2. Emergency Detection

### 2.1 Keyword Detection

```typescript
interface EmergencyKeywords {
  // 다국어 긴급 키워드
  keywords: {
    en: ['help', 'emergency', '911', 'fire', 'police', 'ambulance', 'danger'];
    ko: ['도와주세요', '긴급', '119', '불이야', '경찰', '구급차', '위험'];
    ja: ['助けて', '緊急', '110', '火事', '警察', '救急'];
    zh: ['救命', '紧急', '110', '着火', '警察', '救护车'];
  };

  // 감지 설정
  detection: {
    caseSensitive: false;
    fuzzyMatch: true;          // 유사 발음 허용
    minConfidence: 0.8;        // 긴급 상황은 높은 임계값
    contextAware: true;        // 문맥 고려
  };

  // 우선순위
  priority: {
    lifeThreating: ['help', 'fire', 'ambulance', '도와주세요', '불이야'];
    urgent: ['police', 'danger', '경찰', '위험'];
    standard: ['emergency', '긴급'];
  };
}
```

### 2.2 Context Analysis

```typescript
interface EmergencyContext {
  // 긴급 상황 판단 요소
  indicators: {
    repeatedKeyword: boolean;      // 키워드 반복
    raisedVoice: boolean;          // 높은 음성 (볼륨/피치)
    rapidSpeech: boolean;          // 빠른 말속도
    breathlessness: boolean;       // 숨가쁨 감지
    backgroundNoise: string[];     // 배경 소음 (사이렌 등)
  };

  // 신뢰도 점수 계산
  calculateUrgency(indicators): UrgencyLevel;

  // 긴급 레벨
  urgencyLevels: {
    CRITICAL: 5;    // 즉시 대응
    HIGH: 4;        // 우선 처리
    MEDIUM: 3;      // 신속 처리
    LOW: 2;         // 일반 처리
    NORMAL: 1;      // 표준 처리
  };
}
```

### 2.3 Detection Pipeline

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Audio     │───▶│  Keyword    │───▶│  Context    │
│   Input     │    │  Detection  │    │  Analysis   │
└─────────────┘    └─────────────┘    └─────────────┘
                          │                  │
                          ▼                  ▼
                   [Emergency?]      [Urgency Level]
                          │                  │
                          └────────┬─────────┘
                                   ▼
                         ┌─────────────────┐
                         │ Priority Queue  │
                         │   Processing    │
                         └─────────────────┘
```

---

## 3. Priority Processing

### 3.1 Fast Track Mode

```typescript
interface FastTrackMode {
  // 활성화 조건
  activation: {
    urgencyLevel: 'HIGH' | 'CRITICAL';
    manualTrigger: boolean;
    systemOverride: boolean;
  };

  // 최적화
  optimizations: {
    skipNonEssentialProcessing: true;
    useSimplifiedGloss: true;
    prioritizeSpeed: true;
    reduceAnimationQuality: boolean;  // 속도 vs 품질
  };

  // 지연 목표
  latencyTargets: {
    transcription: 100;    // ms (표준: 200ms)
    translation: 150;      // ms (표준: 300ms)
    rendering: 50;         // ms (표준: 100ms)
    total: 300;           // ms (표준: 600ms)
  };
}
```

### 3.2 Resource Allocation

```typescript
interface EmergencyResources {
  // 리소스 우선 할당
  allocation: {
    cpuPriority: 'highest';
    memoryReservation: true;
    networkPriority: true;
    gpuAcceleration: true;
  };

  // 다른 작업 처리
  otherTasks: {
    pauseNonCritical: true;
    queueNormal: true;
    dropLowPriority: boolean;
  };

  // 스케일링
  scaling: {
    autoScale: true;
    maxInstances: 'unlimited';
    preWarmInstances: true;
  };
}
```

---

## 4. Emergency Sign Library

### 4.1 Pre-loaded Signs

```typescript
interface EmergencySigns {
  // 필수 긴급 수화 (사전 로드)
  critical: {
    signs: [
      { gloss: 'HELP', category: 'request', preloaded: true },
      { gloss: 'EMERGENCY', category: 'alert', preloaded: true },
      { gloss: 'FIRE', category: 'danger', preloaded: true },
      { gloss: 'POLICE', category: 'service', preloaded: true },
      { gloss: 'AMBULANCE', category: 'service', preloaded: true },
      { gloss: 'HOSPITAL', category: 'location', preloaded: true },
      { gloss: 'HURT', category: 'condition', preloaded: true },
      { gloss: 'DANGER', category: 'alert', preloaded: true },
      { gloss: 'STOP', category: 'command', preloaded: true },
      { gloss: 'CALL', category: 'action', preloaded: true },
    ];
    cacheStrategy: 'permanent';
    loadOnStartup: true;
  };

  // 의료 용어
  medical: {
    signs: [
      'PAIN', 'HEART', 'BREATHE', 'BLOOD', 'MEDICINE',
      'ALLERGY', 'DIABETES', 'SEIZURE', 'UNCONSCIOUS',
      'CPR', 'DOCTOR', 'NURSE', 'WHEELCHAIR'
    ];
    cacheStrategy: 'session';
    loadOnDemand: false;
  };

  // 위치/방향
  location: {
    signs: [
      'HERE', 'THERE', 'LEFT', 'RIGHT', 'UP', 'DOWN',
      'INSIDE', 'OUTSIDE', 'NEAR', 'FAR', 'ADDRESS'
    ];
    cacheStrategy: 'session';
  };
}
```

### 4.2 Emergency Phrases

```typescript
const EMERGENCY_PHRASES = {
  en: {
    'CALL_911': { gloss: ['PLEASE', 'CALL', '911'], priority: 'critical' },
    'NEED_HELP': { gloss: ['I', 'NEED', 'HELP'], priority: 'critical' },
    'WHERE_HOSPITAL': { gloss: ['WHERE', 'HOSPITAL'], priority: 'high' },
    'ALLERGIC_TO': { gloss: ['I', 'ALLERGY', 'HAVE'], priority: 'high' },
  },
  ko: {
    'CALL_119': { gloss: ['119', 'CALL', 'PLEASE'], priority: 'critical' },
    'NEED_HELP': { gloss: ['HELP', 'NEED'], priority: 'critical' },
    'WHERE_HOSPITAL': { gloss: ['HOSPITAL', 'WHERE'], priority: 'high' },
  }
};
```

### 4.3 Medical ID Integration

```typescript
interface MedicalIDIntegration {
  // 의료 정보 접근
  medicalInfo: {
    allergies: string[];
    conditions: string[];
    medications: string[];
    bloodType: string;
    emergencyContacts: Contact[];
  };

  // 빠른 표현 생성
  generateQuickPhrases(): EmergencyPhrase[];

  // 개인정보 보호
  privacy: {
    requireAuthentication: boolean;
    shareOnlyInEmergency: boolean;
    userConsent: boolean;
  };
}
```

---

## 5. Failover Mechanisms

### 5.1 Text Fallback

```typescript
interface TextFallback {
  // 수화 렌더링 실패 시
  activation: {
    renderingFailed: boolean;
    latencyExceeded: boolean;
    lowBandwidth: boolean;
  };

  // 텍스트 표시
  display: {
    fontSize: 'extra-large';
    highContrast: true;
    flashAlert: boolean;        // 주의 환기
    readAloud: boolean;         // TTS (청인 위해)
  };

  // 형식
  format: {
    uppercase: true;            // 대문자
    simplified: true;           // 단순화된 문장
    bulletPoints: true;         // 핵심만
  };
}
```

### 5.2 Simplified Mode

```typescript
interface SimplifiedMode {
  // 핵심 정보만 전달
  content: {
    removeFillers: true;        // 불필요 단어 제거
    useBasicVocabulary: true;   // 기본 어휘만
    shortenSentences: true;     // 짧은 문장
  };

  // 속도 최적화
  speed: {
    reducedAnimationFrames: true;
    skipTransitions: true;
    fastFingerspelling: true;
  };

  // 예시 변환
  // Input: "Could you please call an ambulance because I'm having difficulty breathing?"
  // Simplified: "CALL AMBULANCE. BREATHE HARD."
}
```

### 5.3 Offline Capability

```typescript
interface OfflineCapability {
  // 오프라인 데이터
  offlineData: {
    emergencySigns: true;       // 긴급 수화
    basicVocabulary: 1000;      // 기본 어휘 수
    phraseTemplates: true;      // 문장 템플릿
    localASR: boolean;          // 로컬 음성인식
  };

  // 동기화
  sync: {
    autoSyncWhenOnline: true;
    prioritizeEmergencyUpdates: true;
    lastSyncRequired: '24h';    // 최대 오프라인 시간
  };

  // 제한 사항
  limitations: {
    reducedAccuracy: true;
    limitedVocabulary: true;
    noCustomization: true;
  };
}
```

---

## 6. Emergency Response Integration

### 6.1 911/119 Integration

```typescript
interface EmergencyServiceIntegration {
  // 직접 연결
  directConnect: {
    enabled: boolean;
    countries: ['US', 'KR', 'UK', 'EU'];
    protocolVersion: 'NG911' | 'legacy';
  };

  // 위치 정보
  location: {
    autoShare: boolean;
    gpsAccuracy: 'high';
    addressLookup: boolean;
    nearbyLandmarks: boolean;
  };

  // 통신 모드
  communicationMode: {
    textRelay: true;            // 텍스트 릴레이
    videoRelay: boolean;        // 화상 릴레이 (VRS)
    signLanguageInterpreter: boolean;
  };

  // 자동 메시지
  autoMessage: {
    enabled: boolean;
    includeLocation: true;
    includeMedicalInfo: boolean;
    includeCallbackNumber: true;
  };
}
```

### 6.2 Video Relay Service (VRS)

```typescript
interface VideoRelayService {
  // VRS 연결
  connection: {
    providers: string[];        // 인증된 VRS 제공자
    fallbackOrder: string[];
    connectionTimeout: 10000;   // ms
  };

  // 품질 요구사항
  quality: {
    minResolution: { width: 640, height: 480 };
    minFrameRate: 24;
    maxLatency: 200;            // ms
  };

  // 긴급 우선
  emergencyPriority: {
    skipQueue: true;
    dedicatedInterpreters: boolean;
    24x7Available: true;
  };
}
```

---

## 7. Testing & Drills

### 7.1 System Testing

```typescript
interface EmergencyTesting {
  // 자동 테스트
  automated: {
    keywordDetection: 'daily';
    latencyBenchmark: 'daily';
    failoverSwitch: 'weekly';
    offlineMode: 'weekly';
  };

  // 부하 테스트
  loadTesting: {
    concurrentEmergencies: 100;
    sustainedDuration: '1h';
    targetLatency: 300;         // ms
  };

  // 시뮬레이션
  simulation: {
    realWorldScenarios: true;
    multiLanguage: true;
    networkFailure: true;
    highLoad: true;
  };
}
```

### 7.2 User Drills

```typescript
interface UserDrills {
  // 연습 모드
  practiceMode: {
    enabled: true;
    noRealEmergencyCall: true;
    feedback: true;
    timing: true;
  };

  // 교육 콘텐츠
  education: {
    emergencySigns: true;
    howToUseSystem: true;
    whatToExpect: true;
    practiceScenarios: string[];
  };

  // 알림
  reminders: {
    suggestPractice: 'monthly';
    updateEmergencyInfo: 'quarterly';
  };
}
```

---

## 8. Compliance & Certification

### 8.1 Standards

| Standard | Description | Status |
|----------|-------------|--------|
| FCC 911 | US 긴급 통신 | Required |
| EENA NG112 | EU 긴급 통신 | Required |
| KCC 119 | 한국 긴급 통신 | Required |
| TIA-1082 | VRS 품질 표준 | Recommended |
| WCAG 2.1 AAA | 접근성 | Required |

### 8.2 Certification Requirements

```yaml
certification:
  testing:
    - keyword_detection_accuracy: ">= 99%"
    - false_positive_rate: "< 1%"
    - latency_p99: "< 500ms"
    - availability: ">= 99.99%"

  documentation:
    - emergency_procedures
    - user_training_materials
    - system_architecture
    - audit_logs

  auditing:
    frequency: "annual"
    third_party: true
    penetration_testing: true
```

---

## 9. References

- FCC Emergency Communications Guidelines
- EENA NG112 Technical Specification
- TIA-1082 VRS Quality Standard
- NENA i3 Standard for NG9-1-1
- ITU-T F.930 Multimedia Telecommunications Relay Service
