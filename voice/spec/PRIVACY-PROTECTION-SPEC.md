# WIA Voice-Sign: Privacy Protection Specification

## 1. Overview

본 문서는 음성-수화 번역 시스템의 개인정보 보호 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft

---

## 2. Data Classification

### 2.1 Data Categories

| Category | Examples | Sensitivity | Retention |
|----------|----------|-------------|-----------|
| **Audio** | 음성 녹음 | High | Session only |
| **Transcript** | 텍스트 변환 결과 | High | Anonymized |
| **Gloss** | 수화 표현 | Medium | Anonymized |
| **Pose** | 3D 애니메이션 데이터 | Low | Configurable |
| **Metadata** | 타임스탬프, 언어 | Low | Aggregated |
| **User Data** | 계정 정보, 설정 | High | Until deletion |

### 2.2 Sensitivity Levels

```typescript
enum SensitivityLevel {
  PUBLIC = 'public',           // 공개 가능
  INTERNAL = 'internal',       // 내부 사용
  CONFIDENTIAL = 'confidential', // 기밀
  RESTRICTED = 'restricted',   // 제한 (PII)
  TOP_SECRET = 'top_secret'    // 최고 기밀 (의료/법률)
}

interface DataClassification {
  audioData: SensitivityLevel.RESTRICTED;
  transcriptData: SensitivityLevel.RESTRICTED;
  glossData: SensitivityLevel.CONFIDENTIAL;
  poseData: SensitivityLevel.INTERNAL;
  aggregatedStats: SensitivityLevel.PUBLIC;
}
```

---

## 3. Data Handling Policies

### 3.1 Audio Data

```typescript
interface AudioDataPolicy {
  // 수집
  collection: {
    minimumNecessary: true;    // 최소 필요 원칙
    purposeLimitation: true;   // 목적 제한
    explicitConsent: true;     // 명시적 동의
  };

  // 처리
  processing: {
    location: 'on_device' | 'server_side' | 'hybrid';
    encryption: {
      inTransit: 'TLS_1.3';
      atRest: 'AES_256';
    };
    anonymization: {
      voicePrint: 'removed';
      backgroundNoise: 'stripped';
      metadata: 'minimized';
    };
  };

  // 보존
  retention: {
    default: 'session_only';
    options: ['none', 'session_only', 'encrypted_24h'];
    userOverride: true;
    deletionMethod: 'secure_wipe';
  };

  // 공유
  sharing: {
    thirdParty: false;
    internalAnalysis: 'anonymized_only';
    lawEnforcement: 'with_warrant_only';
  };
}
```

### 3.2 Transcript Data

```typescript
interface TranscriptDataPolicy {
  // 처리 옵션
  processingOptions: {
    // 즉시 삭제 (가장 안전)
    ephemeral: {
      retention: 'none';
      logging: false;
      improvement: false;
    };

    // 익명화 저장 (모델 개선용)
    anonymized: {
      retention: '90_days';
      piiRemoved: true;
      aggregatedOnly: true;
    };

    // 암호화 저장 (사용자 히스토리)
    encrypted: {
      retention: 'user_controlled';
      e2eEncryption: true;
      userKeyManagement: true;
    };
  };

  // PII 탐지 및 제거
  piiHandling: {
    autoDetect: true;
    categories: [
      'names', 'addresses', 'phone_numbers', 'emails',
      'ssn', 'credit_cards', 'medical_ids'
    ];
    action: 'redact' | 'hash' | 'remove';
  };
}
```

### 3.3 User Preferences

```typescript
interface UserDataPreferences {
  // 사용자 선택 가능 옵션
  options: {
    saveHistory: boolean;
    improveService: boolean;
    personalizedExperience: boolean;
    shareAnonymousUsage: boolean;
  };

  // 기본값 (프라이버시 우선)
  defaults: {
    saveHistory: false;
    improveService: false;
    personalizedExperience: false;
    shareAnonymousUsage: false;
  };

  // 세분화된 제어
  granularControl: {
    audioRetention: 'none' | 'session' | '24h' | 'custom';
    transcriptRetention: 'none' | 'anonymized' | 'encrypted';
    locationData: 'never' | 'emergency_only' | 'always';
    analyticsData: 'none' | 'basic' | 'detailed';
  };
}
```

---

## 4. Consent Management

### 4.1 Consent Types

```typescript
interface ConsentTypes {
  // 필수 동의 (서비스 이용에 필요)
  required: {
    termsOfService: true;
    privacyPolicy: true;
    dataProcessing: true;
  };

  // 선택 동의
  optional: {
    serviceImprovement: boolean;
    marketingCommunication: boolean;
    thirdPartySharing: boolean;
    researchParticipation: boolean;
  };

  // 특별 동의 (민감 데이터)
  special: {
    biometricData: boolean;      // 음성 지문
    healthData: boolean;         // 의료 관련 번역
    childrenData: boolean;       // 13세 미만 (COPPA)
  };
}
```

### 4.2 Consent Flow

```
┌─────────────────┐
│  First Access   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐    ┌─────────────────┐
│  Essential Info │───▶│  Privacy Notice │
│  (minimal)      │    │  (详细)          │
└────────┬────────┘    └─────────────────┘
         │
         ▼
┌─────────────────┐
│ Consent Choices │
│  □ Required     │
│  □ Optional 1   │
│  □ Optional 2   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐    ┌─────────────────┐
│ Accept Selected │───▶│  Start Using    │
│                 │    │  Service        │
└─────────────────┘    └─────────────────┘
```

### 4.3 Consent Withdrawal

```typescript
interface ConsentWithdrawal {
  // 철회 방법
  methods: {
    inApp: true;              // 앱 내 설정
    website: true;            // 웹사이트
    email: true;              // 이메일 요청
    customerSupport: true;    // 고객 지원
  };

  // 처리 시간
  processingTime: {
    optionalConsent: 'immediate';
    dataExport: '30_days';
    dataDeletion: '30_days';
  };

  // 영향
  impact: {
    requiredConsent: 'service_termination';
    optionalConsent: 'feature_disabled';
    fullWithdrawal: 'account_deletion';
  };
}
```

---

## 5. Data Subject Rights

### 5.1 GDPR Rights (EU)

```typescript
interface GDPRRights {
  // 접근권 (Article 15)
  rightToAccess: {
    available: true;
    responseTime: '30_days';
    format: 'machine_readable';
    free: true;
  };

  // 정정권 (Article 16)
  rightToRectification: {
    available: true;
    process: 'self_service' | 'request';
    responseTime: '30_days';
  };

  // 삭제권 (Article 17)
  rightToErasure: {
    available: true;
    scope: 'all_personal_data';
    exceptions: ['legal_obligation', 'public_interest'];
    responseTime: '30_days';
  };

  // 처리 제한권 (Article 18)
  rightToRestriction: {
    available: true;
    conditions: ['accuracy_contested', 'unlawful_processing'];
  };

  // 이동권 (Article 20)
  rightToPortability: {
    available: true;
    format: 'JSON' | 'CSV';
    directTransfer: boolean;
  };

  // 반대권 (Article 21)
  rightToObject: {
    available: true;
    processingTypes: ['direct_marketing', 'profiling'];
  };

  // 자동화된 결정 관련 권리 (Article 22)
  automatedDecisionMaking: {
    humanReviewAvailable: true;
    explainability: true;
    rightToContest: true;
  };
}
```

### 5.2 Korean PIPA (개인정보보호법)

```typescript
interface KoreanPIPARights {
  // 열람권 (제35조)
  accessRight: {
    available: true;
    responseTime: '10_days';
  };

  // 정정·삭제권 (제36조)
  correctionDeletionRight: {
    available: true;
    responseTime: '10_days';
  };

  // 처리정지권 (제37조)
  processingStopRight: {
    available: true;
    exceptions: ['legal_basis', 'contract_fulfillment'];
  };

  // 동의 철회권
  consentWithdrawalRight: {
    available: true;
    equalAccessibility: true;  // 동의만큼 쉽게
  };

  // 손해배상청구권 (제39조)
  damageClaimRight: {
    available: true;
    proofBurdenOnController: true;
  };
}
```

### 5.3 HIPAA (US Healthcare)

```typescript
interface HIPAACompliance {
  // PHI 보호
  phiProtection: {
    minimumNecessary: true;
    authorization: 'required';
    businessAssociateAgreement: boolean;
  };

  // 환자 권리
  patientRights: {
    accessToRecords: true;
    amendmentRequest: true;
    accountingOfDisclosures: true;
    restrictionRequest: true;
  };

  // 보안 요구사항
  securityRequirements: {
    encryption: 'required';
    accessControls: 'required';
    auditLogs: 'required';
    riskAssessment: 'annual';
  };

  // 적용 조건
  applicableWhen: {
    healthcareProvider: boolean;
    healthPlan: boolean;
    healthcareContent: boolean;
  };
}
```

---

## 6. Technical Safeguards

### 6.1 Encryption

```typescript
interface EncryptionStandards {
  // 전송 중 암호화
  inTransit: {
    protocol: 'TLS_1.3';
    cipherSuites: [
      'TLS_AES_256_GCM_SHA384',
      'TLS_CHACHA20_POLY1305_SHA256'
    ];
    certificatePinning: boolean;
    hsts: true;
  };

  // 저장 시 암호화
  atRest: {
    algorithm: 'AES-256-GCM';
    keyManagement: 'HSM' | 'KMS';
    keyRotation: '90_days';
  };

  // 종단간 암호화 (선택)
  e2e: {
    available: true;
    userControlled: true;
    keyDerivation: 'Argon2id';
  };
}
```

### 6.2 Anonymization

```typescript
interface AnonymizationTechniques {
  // 기법
  techniques: {
    // 가명화
    pseudonymization: {
      method: 'tokenization' | 'hashing';
      reversible: boolean;
      keyStorage: 'separate';
    };

    // 일반화
    generalization: {
      ageRanges: true;         // 25세 → 20-30세
      locationBlur: true;      // 정확한 위치 → 도시
      timeRounding: true;      // 10:23 → 10:00
    };

    // 데이터 마스킹
    masking: {
      names: '***';
      phones: '010-****-1234';
      emails: 'u***@***.com';
    };

    // 차분 프라이버시
    differentialPrivacy: {
      enabled: boolean;
      epsilon: 1.0;
      mechanism: 'laplace' | 'gaussian';
    };
  };
}
```

### 6.3 Access Control

```typescript
interface AccessControl {
  // 역할 기반 접근 제어
  rbac: {
    roles: ['user', 'support', 'analyst', 'admin', 'dpo'];
    permissions: Record<Role, Permission[]>;
    leastPrivilege: true;
  };

  // 인증
  authentication: {
    mfa: 'required' | 'optional';
    passwordPolicy: PasswordPolicy;
    sessionManagement: SessionPolicy;
    biometric: boolean;
  };

  // 감사 로그
  auditLogging: {
    accessLogs: true;
    modificationLogs: true;
    exportLogs: true;
    retentionPeriod: '1_year';
  };
}
```

---

## 7. Breach Response

### 7.1 Incident Response Plan

```typescript
interface BreachResponse {
  // 탐지
  detection: {
    automatedMonitoring: true;
    anomalyDetection: true;
    employeeReporting: true;
    userReporting: true;
  };

  // 대응 단계
  responseSteps: [
    'identify_and_contain',
    'assess_scope',
    'notify_authorities',
    'notify_affected_users',
    'remediate',
    'post_incident_review'
  ];

  // 통지 요구사항
  notification: {
    authorities: {
      gdpr: '72_hours';
      koreanPipa: 'without_delay';
      hipaa: '60_days';
    };
    users: {
      when: 'high_risk';
      method: ['email', 'in_app', 'sms'];
      content: ['nature', 'consequences', 'measures', 'contact'];
    };
  };

  // 문서화
  documentation: {
    incidentLog: true;
    impactAssessment: true;
    remediationSteps: true;
    lessonsLearned: true;
  };
}
```

### 7.2 Breach Severity Classification

| Severity | Criteria | Response Time | Notification |
|----------|----------|---------------|--------------|
| **Critical** | 대량 PII 유출 | 즉시 | 당국 + 사용자 |
| **High** | 민감 데이터 접근 | 24시간 | 당국 + 사용자 |
| **Medium** | 제한적 데이터 노출 | 72시간 | 당국 |
| **Low** | 내부 정책 위반 | 1주일 | 내부 |

---

## 8. Implementation

### 8.1 Privacy by Design

```typescript
interface PrivacyByDesign {
  principles: {
    proactiveNotReactive: true;
    privacyAsDefault: true;
    privacyEmbedded: true;
    fullFunctionality: true;
    endToEndSecurity: true;
    visibilityTransparency: true;
    respectForUserPrivacy: true;
  };

  implementation: {
    dataMinimization: true;
    purposeSpecification: true;
    useLimitation: true;
    securitySafeguards: true;
    openness: true;
    individualParticipation: true;
    accountability: true;
  };
}
```

### 8.2 API Privacy Headers

```http
# Request
Privacy-Preference: minimal
Do-Not-Track: 1
Consent-Token: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...

# Response
X-Privacy-Policy: https://wia.org/privacy
X-Data-Retention: session
X-Anonymization: enabled
X-Consent-Required: false
```

### 8.3 Configuration

```yaml
privacy:
  data_retention:
    audio: none
    transcript: anonymized
    gloss: session
    pose: session
    user_preferences: encrypted

  consent:
    required:
      - data_processing
      - privacy_policy
    optional:
      - service_improvement
      - analytics

  anonymization:
    pii_detection: enabled
    auto_redact: true
    differential_privacy: false

  security:
    encryption_in_transit: TLS_1.3
    encryption_at_rest: AES_256
    access_logging: enabled

  compliance:
    gdpr: enabled
    korean_pipa: enabled
    hipaa: when_applicable
    coppa: enabled

  user_rights:
    data_export: enabled
    data_deletion: enabled
    consent_management: enabled
```

---

## 9. References

- GDPR (EU 2016/679)
- Korean PIPA (개인정보보호법)
- HIPAA Privacy Rule (US)
- CCPA (California)
- ISO 27701:2019 Privacy Information Management
- NIST Privacy Framework
