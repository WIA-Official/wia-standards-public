# WIA Cognitive AAC - Privacy Specification

## 프라이버시 보호 및 데이터 관리 명세

**홍익인간 (弘益人間) - 널리 인간을 이롭게 하라**

---

## 1. 개요

### 1.1 목적

인지 장애 사용자의 개인정보를 최대한 보호하면서 효과적인 학습을 가능하게 하는 프라이버시 보호 프레임워크.

### 1.2 핵심 원칙

| 원칙 | 설명 |
|------|------|
| **온디바이스 우선** | 모든 데이터는 기기에 저장, 학습도 로컬에서 |
| **데이터 최소화** | 필요한 최소한의 데이터만 수집 |
| **명시적 동의** | 모든 데이터 수집/공유에 명시적 동의 필요 |
| **완전 삭제 지원** | 언제든 모든 데이터 완전 삭제 가능 |
| **투명성** | 수집/사용 데이터에 대한 완전한 투명성 |

### 1.3 규정 준수

- GDPR (유럽 일반 데이터 보호 규정)
- CCPA (캘리포니아 소비자 프라이버시법)
- PIPA (한국 개인정보 보호법)
- HIPAA (의료 정보 보호 - 해당 시)

---

## 2. 데이터 분류

### 2.1 수집 데이터 유형

| 유형 | 예시 | 민감도 | 저장 위치 |
|------|------|--------|----------|
| **심볼 사용** | 선택한 심볼 ID, 시간 | 낮음 | 로컬 |
| **사용 패턴** | 빈도, 시퀀스 | 중간 | 로컬 |
| **컨텍스트** | 시간대, 활동 | 중간 | 로컬 |
| **위치** | GPS, 실내 위치 | 높음 | 로컬 (선택적) |
| **대화 내용** | 생성된 문장 | 매우 높음 | 수집 안함 |
| **개인 식별** | 이름, 사진 | 매우 높음 | 수집 안함 |

### 2.2 데이터 최소화 설정

```typescript
interface DataMinimizationSettings {
  excludePersonalInfo: boolean;      // 기본: true
  excludeLocationData: boolean;      // 기본: false (선택적)
  excludeConversationContent: boolean; // 기본: true
}
```

---

## 3. 동의 관리

### 3.1 동의 유형

| 동의 유형 | 설명 | 필수 여부 |
|----------|------|----------|
| **data_collection** | 로컬 데이터 수집 | 학습에 필수 |
| **federated_learning** | 익명화 통계 공유 | 선택 |
| **analytics** | 사용 분석 | 선택 |

### 3.2 동의 기록

```typescript
interface ConsentRecord {
  timestamp: number;
  type: 'data_collection' | 'federated_learning' | 'analytics';
  granted: boolean;
  grantor: 'user' | 'caregiver' | 'guardian';
  expiresAt?: number;  // 만료 시간 (선택)
}
```

### 3.3 특수 사용자 동의

#### 인지 장애 사용자

- 케어기버 또는 법적 대리인 동의 가능
- 사용자 이해 수준에 맞는 설명 제공
- 시각적/단순화된 동의 인터페이스

#### 치매 사용자

- **필수**: 케어기버/보호자 동의
- 주기적 동의 재확인 (6개월)
- 더 엄격한 프라이버시 설정 기본값

---

## 4. 온디바이스 학습

### 4.1 로컬 학습 아키텍처

```
┌─────────────────────────────────────┐
│           사용자 기기               │
│  ┌─────────────────────────────┐   │
│  │      학습 모듈              │   │
│  │  ┌─────────┐ ┌───────────┐  │   │
│  │  │ N-gram  │ │ Context   │  │   │
│  │  │ Model   │ │ Model     │  │   │
│  │  └────┬────┘ └─────┬─────┘  │   │
│  │       │            │        │   │
│  │  ┌────┴────────────┴────┐   │   │
│  │  │    로컬 스토리지     │   │   │
│  │  │  (암호화된 패턴)     │   │   │
│  │  └──────────────────────┘   │   │
│  └─────────────────────────────┘   │
│                                     │
│  ❌ 클라우드로 전송 없음            │
└─────────────────────────────────────┘
```

### 4.2 로컬 저장소 설정

```typescript
interface LocalLearningSettings {
  enabled: boolean;              // 기본: true
  modelStorage: 'local_only';    // 항상 로컬
  noCloudUpload: boolean;        // 기본: true
}
```

### 4.3 데이터 보존 정책

```typescript
interface RetentionPolicy {
  maxDays: number;       // 기본: 90일
  maxRecords: number;    // 기본: 10,000
  autoDelete: boolean;   // 기본: true
  cleanupInterval: number; // 24시간 (ms)
}
```

---

## 5. 연합 학습 (선택적)

### 5.1 개요

연합 학습은 **선택적** 기능으로, 개인 데이터를 공유하지 않고 모델 개선에 기여할 수 있습니다.

```
┌────────┐     ┌────────┐     ┌────────┐
│기기 A  │     │기기 B  │     │기기 C  │
│        │     │        │     │        │
│ 로컬   │     │ 로컬   │     │ 로컬   │
│ 학습   │     │ 학습   │     │ 학습   │
└───┬────┘     └───┬────┘     └───┬────┘
    │              │              │
    │   익명화     │   익명화     │
    │   통계만     │   통계만     │
    │              │              │
    └──────────────┼──────────────┘
                   │
                   ▼
           ┌──────────────┐
           │   집계 서버   │
           │  (통계만 수신)│
           └──────────────┘
```

### 5.2 연합 학습 설정

```typescript
interface FederatedLearningSettings {
  enabled: boolean;              // 기본: false
  consentGiven: boolean;         // 명시적 동의 필수
  anonymization: 'full' | 'partial';  // 기본: 'full'
  aggregationOnly: boolean;      // 기본: true
}
```

### 5.3 공유되는 정보 (동의 시)

**공유됨:**
- 익명화된 사용 통계 (빈도, 패턴 길이)
- 모델 그래디언트 (차등 프라이버시 적용)
- 집계된 정확도 메트릭

**절대 공유 안함:**
- 개별 심볼 선택 기록
- 대화 내용
- 위치 정보
- 개인 식별 정보
- 케어기버/가족 정보

---

## 6. 차등 프라이버시

### 6.1 노이즈 추가

모든 공유 데이터에 차등 프라이버시 노이즈 적용:

```python
def add_noise(data, epsilon=1.0, sensitivity=1.0):
    scale = sensitivity / epsilon
    noise = np.random.laplace(0, scale, data.shape)
    return data + noise
```

### 6.2 그래디언트 클리핑

```python
def clip_gradients(gradients, max_norm=1.0):
    norm = np.linalg.norm(gradients)
    if norm > max_norm:
        gradients = gradients * (max_norm / norm)
    return gradients
```

### 6.3 프라이버시 예산

| 설정 | 값 | 의미 |
|------|-----|------|
| ε (epsilon) | 1.0 | 강한 프라이버시 보장 |
| δ (delta) | 10⁻⁵ | 예외 확률 |
| Clip norm | 1.0 | 그래디언트 제한 |

---

## 7. 데이터 익명화

### 7.1 익명화 수준

| 수준 | 설명 | 적용 |
|------|------|------|
| **none** | 익명화 없음 | 로컬 저장만 |
| **partial** | 부분 익명화 | 일부 정보 해싱 |
| **full** | 완전 익명화 | 모든 식별자 제거 |

### 7.2 익명화 방법

```typescript
class DataAnonymizer {
  // 심볼 ID 해싱
  anonymizeSymbolId(symbolId: string): string {
    const salted = `${salt}:${symbolId}`;
    return `anon_${sha256(salted).slice(0, 16)}`;
  }

  // 패턴 익명화
  anonymizePattern(pattern: object): object {
    // 개인 식별 정보 제거
    // 위치, 이름, 대화내용 등
  }
}
```

---

## 8. 사용자 권리

### 8.1 잊혀질 권리 (Right to Erasure)

```typescript
// 모든 데이터 완전 삭제
privacyManager.deleteAllData();

// 반환: { local_models: 15, updates: 230 }
// 결과: 모든 학습 데이터 영구 삭제
```

### 8.2 데이터 이동권 (Data Portability)

```typescript
// 모든 데이터 내보내기
const exportData = privacyManager.exportData(patterns, includeSettings);

// JSON 또는 암호화된 백업 형식
```

### 8.3 정보 접근권

```typescript
// 감사 로그 조회
const auditLog = privacyManager.getAuditLog();

// 저장 데이터 목록
const storedData = privacyManager.getStorageQuota();
```

---

## 9. 감사 및 로깅

### 9.1 감사 로그

```typescript
interface PrivacyAuditLog {
  timestamp: number;
  action: 'access' | 'export' | 'delete' | 'consent_change';
  details: string;
  actor: 'user' | 'system' | 'caregiver';
}
```

### 9.2 로깅 정책

- 모든 데이터 접근 기록
- 동의 변경 기록
- 삭제 요청 기록
- 로그 보존: 최대 90일

---

## 10. 보안 요구사항

### 10.1 저장 데이터 보안

| 항목 | 요구사항 |
|------|----------|
| 암호화 | AES-256 (저장 시) |
| 키 관리 | 기기 보안 저장소 |
| 접근 제어 | 앱 샌드박스 |

### 10.2 전송 데이터 보안 (연합 학습 시)

| 항목 | 요구사항 |
|------|----------|
| 프로토콜 | TLS 1.3 |
| 인증 | 상호 TLS (mTLS) |
| 익명화 | 차등 프라이버시 |

### 10.3 취약점 대응

- 정기 보안 감사
- 버그 바운티 프로그램
- 신속한 패치 배포

---

## 11. 특수 고려사항

### 11.1 인지 장애 사용자

- 간소화된 프라이버시 설정 UI
- 케어기버 동의 지원
- 명확한 시각적 피드백

### 11.2 치매 사용자

- **더 엄격한 기본 설정**
  - 연합 학습: 기본 비활성화
  - 익명화 수준: 항상 'full'
- **케어기버 동의 필수**
- **더 짧은 데이터 보존 기간** (30일)

### 11.3 미성년자

- 보호자 동의 필수
- 데이터 수집 최소화
- 연합 학습 비활성화

---

## 12. 구현 체크리스트

### 12.1 필수 구현

- [ ] 로컬 저장소 암호화
- [ ] 동의 관리 시스템
- [ ] 데이터 삭제 기능
- [ ] 감사 로깅
- [ ] 데이터 내보내기

### 12.2 선택 구현

- [ ] 연합 학습 모듈
- [ ] 차등 프라이버시
- [ ] 케어기버 대시보드
- [ ] 자동 데이터 만료

---

## 13. API 요약

### TypeScript

```typescript
// 동의 기록
privacyManager.recordConsent('data_collection', true, 'user');

// 동의 확인
if (privacyManager.hasConsent('federated_learning')) {
  // 연합 학습 참여
}

// 데이터 필터링
const filtered = privacyManager.filterLearningEvent(event);

// 데이터 삭제
privacyManager.deleteAllData();

// 내보내기
const export = privacyManager.exportData(patterns);
```

### Python

```python
# 연합 학습자
learner = FederatedLearner()

# 동의 기록
learner.record_consent(ConsentType.DATA_COLLECTION, True)

# 로컬 학습
learner.local_update(patterns)

# 연합 참여 (동의 시)
if learner.can_participate_federated():
    update = learner.prepare_federated_update()

# 데이터 삭제
learner.delete_all_data()
```

---

*WIA Cognitive AAC Privacy Specification v1.0*
*Last Updated: 2024*
