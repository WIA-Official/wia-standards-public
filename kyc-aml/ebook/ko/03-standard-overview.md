# 제3장: WIA KYC/AML 표준 개요

## 학습 목표

이 장을 마치면 다음을 할 수 있습니다:
- WIA KYC/AML 표준의 비전과 핵심 원칙 이해하기
- 5가지 핵심 구성 요소와 그 상호작용 파악하기
- 표준 데이터 모델 및 API 아키텍처 설명하기
- 다양한 배포 모델과 통합 패턴 구분하기
- 보안 및 개인정보 보호 요구사항 인식하기

---

## 개요

WIA KYC/AML 표준은 고객 신원 확인, 위험 평가 및 규정 준수 모니터링을 위한 포괄적이고 상호 운용 가능한 프레임워크를 제공합니다. 이 장에서는 표준의 아키텍처, 핵심 구성 요소 및 설계 철학을 소개합니다.

---

## 표준 비전

**미션 선언문:**
> 기관, 공급업체 및 관할권 전반에서 작동하는 표준화된 데이터 형식, API 및 프로토콜을 통해 안전하고 효율적이며 규정을 준수하는 고객 실사를 가능하게 합니다.

### 핵심 원칙

1. **🌐 상호 운용성 우선** - 시스템은 원활하게 데이터를 교환해야 합니다
2. **🔒 설계부터 개인정보 보호** - 데이터 수집 최소화, 보호 극대화
3. **⚖️ 위험 기반 접근** - 실제 위험에 맞게 조치를 조정
4. **🚀 효율성** - 효과를 개선하면서 비용 절감
5. **🤝 협업** - 안전한 정보 공유 가능
6. **📱 고객 중심** - 빠르고 투명하며 사용자 친화적인 경험
7. **🌍 글로벌 적용성** - 로컬 맞춤화로 관할권 전반에 걸쳐 작동

---

## 아키텍처 개요

### 고수준 구성 요소

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA KYC/AML 표준                         │
└─────────────────────────────────────────────────────────────┘

┌────────────────┐  ┌────────────────┐  ┌────────────────┐
│   신원 확인    │  │   위험 평가    │  │   거래 모니터링│
│     계층       │  │     엔진       │  │     시스템     │
└───────┬────────┘  └───────┬────────┘  └───────┬────────┘
        │                   │                    │
        └───────────────────┼────────────────────┘
                            │
                  ┌─────────▼─────────┐
                  │   스크리닝 및     │
                  │  워치리스트 모듈  │
                  └─────────┬─────────┘
                            │
                  ┌─────────▼─────────┐
                  │  사례 관리 및     │
                  │     보고          │
                  └───────────────────┘
```

### 구성 요소 상호작용

```
고객 온보딩 흐름:

1. 고객 신청
        ↓
2. 신원 확인 계층
   → 문서 확인
   → 생체 인식 매칭
   → 데이터 검증
        ↓
3. 스크리닝 모듈
   → 제재 목록
   → PEP 데이터베이스
   → 부정적 미디어
        ↓
4. 위험 평가 엔진
   → 위험 요소 평가
   → 점수 계산
   → 카테고리 할당
        ↓
5. 승인 결정
        ↓
6. 지속적 모니터링
   → 거래 모니터링
   → 정기 검토
   → 경고 생성
        ↓
7. 사례 관리 (필요시)
   → 조사
   → SAR 제출
   → 보고
```

---

## 핵심 구성 요소

### 1. 신원 확인 계층

**목적:** 여러 확인 방법을 통해 고객 신원을 확인합니다.

#### 기능

**문서 확인:**
```json
{
  "documentType": "passport",
  "verificationMethod": "automated",
  "checks": {
    "authenticity": {
      "status": "pass",
      "methods": ["MRZ_validation", "security_features", "template_matching"]
    },
    "dataExtraction": {
      "firstName": "홍",
      "lastName": "길동",
      "dateOfBirth": "1985-06-15",
      "documentNumber": "M1234567",
      "expiryDate": "2030-06-14",
      "confidence": 0.98
    },
    "tampering": {
      "status": "pass",
      "indicators": []
    }
  }
}
```

**생체 인식 확인:**
```json
{
  "biometricType": "facial",
  "livenessCheck": {
    "status": "pass",
    "method": "active",
    "confidence": 0.95
  },
  "matching": {
    "status": "match",
    "similarity": 0.92,
    "threshold": 0.85
  }
}
```

**데이터 확인:**
- 이메일 검증 (형식, 도메인, 전달 가능성)
- 전화번호 확인 (SMS OTP, 통신사 조회)
- 주소 확인 (우편 서비스 검증)
- 데이터베이스 교차 확인 (신용평가기관, 공공 기록)

#### 확인 수준

| 수준 | 방법 | 사용 사례 | 비용 | 시간 |
|------|------|----------|------|------|
| **기본** | 이메일 + 전화 | 저위험, 저가치 | $0.10 | 2분 |
| **표준** | + 정부 발급 ID | 일반 은행 | $1-3 | 5분 |
| **강화** | + 생체 인식 | 고가치 계정 | $3-8 | 10분 |
| **최대** | + 대면 / 비디오 | PEP, 고위험 | $20-50 | 30분 이상 |

---

### 2. 위험 평가 엔진

**목적:** 여러 요소를 기반으로 고객 위험을 평가하고 분류합니다.

#### 위험 요소 프레임워크

```
위험 차원:

1. 지리적 위험
   ├── 고객 거주 국가
   ├── 시민권/국적
   ├── 사업 운영 위치
   └── 거래 목적지

2. 제품/서비스 위험
   ├── 계정 유형
   ├── 거래 방법
   ├── 예상 거래량
   └── 국경간 서비스

3. 고객 유형 위험
   ├── 개인 vs. 기업
   ├── 산업 분야
   ├── 직업
   └── 법인 구조 (기업의 경우)

4. 행동 위험
   ├── 거래 패턴
   ├── 자금 출처 명확성
   ├── 사업 근거
   └── 이력 컴플라이언스

5. 관계 위험
   ├── 최종 실소유자
   ├── 연결된 당사자
   ├── PEP 관계
   └── 부정적 미디어 연관
```

#### 위험 점수 모델

**계산:**
```
총 위험 점수 =
  (지리적 × W1) +
  (제품 × W2) +
  (고객 유형 × W3) +
  (행동 × W4) +
  (관계 × W5)

여기서 W1...W5는 구성 가능한 가중치
```

**예시:**
```json
{
  "customerId": "CUST-789012",
  "riskAssessment": {
    "overallScore": 42,
    "category": "medium",
    "factors": [
      {
        "dimension": "geographic",
        "score": 35,
        "weight": 0.25,
        "details": {
          "residence": "싱가포르 (저위험: 10)",
          "citizenship": "말레이시아 (저위험: 10)",
          "transactionCountries": ["태국 (중위험: 40)", "베트남 (중위험: 45)"]
        }
      },
      {
        "dimension": "product",
        "score": 50,
        "weight": 0.20,
        "details": {
          "accountType": "국제 송금 기능이 있는 비즈니스 당좌 예금"
        }
      },
      {
        "dimension": "customerType",
        "score": 40,
        "weight": 0.20,
        "details": {
          "type": "중소기업",
          "industry": "수출입 (중위험)"
        }
      },
      {
        "dimension": "behavior",
        "score": 25,
        "weight": 0.20,
        "details": {
          "expectedVolume": "비즈니스 모델과 일치",
          "sourceOfFunds": "잘 문서화됨"
        }
      },
      {
        "dimension": "relationship",
        "score": 30,
        "weight": 0.15,
        "details": {
          "beneficialOwners": "명확히 식별됨, PEP 아님",
          "adverseMedia": "발견되지 않음"
        }
      }
    ],
    "recommendedAction": "standard_due_diligence",
    "reviewFrequency": "annual",
    "calculatedAt": "2025-01-09T10:30:00Z"
  }
}
```

#### 위험 카테고리

| 카테고리 | 점수 범위 | CDD 수준 | 검토 빈도 | 승인 권한 |
|----------|-----------|----------|-----------|-----------|
| **금지** | 90-100 | 거부 | 해당 없음 | 자동 거부 |
| **고위험** | 70-89 | EDD | 분기별 | 수석 컴플라이언스 |
| **중위험** | 40-69 | 표준 CDD | 연간 | 컴플라이언스 담당자 |
| **저위험** | 0-39 | 간소화 CDD | 2년마다 | 자동/주니어 |

---

### 3. 스크리닝 및 워치리스트 모듈

**목적:** 제재 목록, PEP 데이터베이스 및 부정적 미디어 소스에 대해 고객을 확인합니다.

#### 스크리닝 유형

**1. 제재 스크리닝**

확인 대상:
- OFAC (미국 외국자산통제국)
- UN 안보리 통합 목록
- EU 제재 목록
- 영국 HM Treasury 금융 제재
- 지역 목록 (호주 DFAT, 스위스 SECO 등)

**2. PEP 스크리닝**

카테고리:
- **Tier 1**: 국가 원수, 고위 정치인
- **Tier 2**: 중간급 정부 관리
- **Tier 3**: 가족 및 가까운 동료
- **RCA**: 친척 및 가까운 동료

**3. 부정적 미디어 스크리닝**

모니터링 대상:
- 형사 유죄 판결
- 금융 범죄 수사
- 규제 집행 조치
- 부패 혐의
- 사기 보고
- 자금 세탁 연결

#### 매칭 알고리즘

```
스크리닝 프로세스:

1. 이름 정규화
   → 직함, 구두점 제거
   → 문자 인코딩 표준화
   → 약어 확장

2. 퍼지 매칭
   → 레벤슈타인 거리
   → 음성 매칭 (Soundex, Metaphone)
   → 음역 처리
   → 별명/별칭 감지

3. 맥락 필터링
   → 생년월일 매칭
   → 국적 매칭
   → 주소 매칭
   → 알려진 별칭

4. 점수 및 임계값
   → 매칭 신뢰도: 0-100
   → 구성 가능한 임계값 (일반적으로 85+)
   → 70-84에 대한 수동 검토 대기열
```

**예시 결과:**
```json
{
  "screeningId": "SCR-2025-001234",
  "customerId": "CUST-789012",
  "screeningType": "comprehensive",
  "executedAt": "2025-01-09T10:35:00Z",
  "results": {
    "sanctions": {
      "status": "no_match",
      "listsChecked": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"],
      "totalRecordsChecked": 125000
    },
    "pep": {
      "status": "potential_match",
      "matches": [
        {
          "matchId": "PEP-2025-5678",
          "confidence": 0.78,
          "name": "홍길순",
          "position": "재무부 차관",
          "country": "가상국",
          "tier": 2,
          "dateOfBirth": "1985-06-10",
          "requiresReview": true,
          "reason": "이름 유사성, 생년월일 근접하지만 정확하지 않음"
        }
      ]
    },
    "adverseMedia": {
      "status": "clear",
      "articlesReviewed": 0
    },
    "overallRisk": "medium",
    "recommendedAction": "manual_review_required"
  }
}
```

---

### 4. 거래 모니터링 시스템

**목적:** 지속적인 감시를 통해 의심스러운 패턴과 활동을 감지합니다.

#### 모니터링 시나리오

**다루는 유형:**

1. **구조화 / 스머핑**
   - 보고 임계값 바로 아래의 여러 거래
   - 계정/기간에 걸친 분할 입금

2. **신속한 자금 이동**
   - 빠르게 입출금되는 자금 (< 24-48시간)
   - 경제적 근거 없음

3. **왕복 거래**
   - 계정 간 순환 이체
   - 동일한 금액이 왕복

4. **지리적 위험 신호**
   - 고위험 관할권으로/로부터의 거래
   - 고객 프로필과 일치하지 않는 패턴

5. **거래량 이상**
   - 거래 빈도의 급격한 증가
   - 정상 패턴보다 훨씬 많은 금액

6. **쉘 회사 지표**
   - 휴면 계정에 대한 많은 지불
   - 명확한 목적이 없는 복잡한 이체 네트워크

#### 규칙 엔진

**예시 규칙:**
```json
{
  "rules": [
    {
      "ruleId": "TM-001",
      "name": "고액 현금 입금",
      "type": "threshold",
      "condition": {
        "transactionType": "cash_deposit",
        "amount": {"greaterThan": 10000},
        "currency": "USD"
      },
      "action": "generate_alert",
      "severity": "medium"
    },
    {
      "ruleId": "TM-045",
      "name": "신속한 자금 이동",
      "type": "behavioral",
      "condition": {
        "creditAmount": {"greaterThan": 5000},
        "debitWithin": "24 hours",
        "debitPercentage": {"greaterThan": 0.95}
      },
      "action": "generate_alert",
      "severity": "high"
    },
    {
      "ruleId": "TM-078",
      "name": "고위험 관할권 송금",
      "type": "geographic",
      "condition": {
        "transactionType": "international_wire",
        "destinationCountry": {"in": ["FATF_HIGH_RISK_LIST"]},
        "amount": {"greaterThan": 1000}
      },
      "action": "generate_alert",
      "severity": "high"
    }
  ]
}
```

#### 머신러닝 향상

규칙 외에도 ML 모델은 다음을 감지합니다:
- **피어 그룹 분석**: 유사한 프로필과 고객 비교
- **시계열 이상**: 비정상적인 패턴 식별
- **네트워크 분석**: 숨겨진 관계 감지
- **유형 인식**: 새로운 세탁 패턴 학습

**이점:**
- 오탐지를 50-70% 감소
- 규칙으로 포착되지 않는 새로운 패턴 감지
- 진화하는 고객 행동에 적응
- 조사 효율성 향상

---

### 5. 사례 관리 및 보고

**목적:** 조사를 관리하고 규제 보고서를 생성합니다.

#### 사례 워크플로우

```
경고 생성
    ↓
├─ 자동 처리 (명백한 오탐지인 경우)
└─ 수동 검토 대기열
    ↓
    분류 (5-10분)
    ├─ 조사로 에스컬레이션
    └─ 종료 (오탐지)
        ↓
        조사 (30-60분)
        ├─ 추가 정보 요청
        ├─ 과거 활동 검토
        ├─ 외부 데이터베이스 확인
        └─ 조사 결과 문서화
            ↓
            결정
            ├─ 해제 (의심스러운 활동 없음)
            ├─ 모니터링 계속
            └─ SAR 제출
                ↓
                SAR 준비 (2-4시간)
                ├─ 내러티브 초안 작성
                ├─ 지원 문서
                ├─ 감독자 검토
                └─ 당국에 제출
```

#### SAR 생성

**표준 SAR 형식:**
```json
{
  "sarId": "SAR-2025-001234",
  "filingInstitution": {
    "name": "예제 은행",
    "identifier": "12-3456789",
    "contactInfo": {...}
  },
  "subject": {
    "type": "individual",
    "name": "홍길동",
    "identifiers": {
      "customerId": "CUST-123456",
      "ssn": "XXX-XX-6789",
      "dob": "1980-03-15"
    },
    "address": {...}
  },
  "suspiciousActivity": {
    "type": ["structuring", "suspected_money_laundering"],
    "description": "고객이 30일 동안 각각 $9,000-$9,900 사이의 15건의 현금 입금을 했으며, 총 $142,500입니다. CTR 보고 임계값을 피하기 위한 구조화와 일치하는 패턴입니다.",
    "dateRange": {
      "from": "2024-11-15",
      "to": "2024-12-15"
    },
    "totalAmount": 142500,
    "currency": "USD"
  },
  "transactions": [
    {
      "date": "2024-11-15",
      "type": "cash_deposit",
      "amount": 9850,
      "account": "XXXX-1234"
    }
    // ... 추가 거래
  ],
  "narrative": "의심스러운 활동, 수행된 조사 단계 및 제출 근거에 대한 자세한 설명...",
  "filedBy": {
    "name": "김조사관",
    "title": "수석 AML 분석가",
    "date": "2025-01-09"
  }
}
```

#### 규제 보고

**지원 보고서:**
- **SAR**: 의심스러운 활동 보고서
- **CTR**: 통화 거래 보고서
- **CMIR**: 국제 통화 운송 보고서
- **FBAR**: 해외 은행 계좌 보고서
- **DOEP**: 면제 대상자 지정
- **사용자 지정**: 관할권별 보고서

---

## 데이터 모델

### 핵심 엔티티

#### 1. 고객 프로필

```json
{
  "customerId": "CUST-789012",
  "type": "individual",
  "personalInfo": {
    "firstName": "길동",
    "middleName": "",
    "lastName": "홍",
    "dateOfBirth": "1985-06-15",
    "nationality": ["KOR"],
    "citizenship": "KOR",
    "placeOfBirth": {
      "city": "서울",
      "country": "KOR"
    }
  },
  "identityDocuments": [
    {
      "type": "passport",
      "number": "M1234567",
      "issuingCountry": "KOR",
      "issueDate": "2020-06-14",
      "expiryDate": "2030-06-14",
      "verified": true,
      "verificationDate": "2025-01-09"
    }
  ],
  "contactInfo": {
    "email": "hong.gildong@example.com",
    "phone": "+82-10-1234-5678",
    "address": {
      "street": "강남대로 123",
      "city": "서울",
      "state": "서울특별시",
      "postalCode": "06236",
      "country": "KOR"
    }
  },
  "riskProfile": {
    "category": "low",
    "score": 25,
    "lastAssessment": "2025-01-09",
    "nextReview": "2027-01-09"
  },
  "pep": false,
  "onboardingDate": "2025-01-09",
  "status": "active"
}
```

#### 2. 법인 고객 (KYB)

```json
{
  "customerId": "CORP-456789",
  "type": "business",
  "companyInfo": {
    "legalName": "주식회사 예제",
    "tradingName": "예제코퍼레이션",
    "registrationNumber": "110111-1234567",
    "registrationCountry": "KOR",
    "registrationDate": "2010-03-15",
    "businessType": "주식회사",
    "industry": "소프트웨어 개발",
    "naicsCode": "541511"
  },
  "beneficialOwners": [
    {
      "customerId": "CUST-111111",
      "name": "김영희",
      "ownershipPercentage": 55,
      "controlType": ["ownership", "voting_rights"],
      "isPep": false
    },
    {
      "customerId": "CUST-222222",
      "name": "이철수",
      "ownershipPercentage": 45,
      "controlType": ["ownership", "voting_rights"],
      "isPep": false
    }
  ],
  "controlPersons": [
    {
      "customerId": "CUST-111111",
      "title": "대표이사",
      "role": "executive_control"
    }
  ],
  "riskProfile": {
    "category": "medium",
    "score": 45,
    "lastAssessment": "2025-01-09"
  }
}
```

---

## API 아키텍처

### RESTful 설계

**기본 URL:** `https://api.wia-kyc.org/v1`

**인증:** OAuth 2.0 / API Key

**표준 응답 형식:**
```json
{
  "status": "success",
  "data": {...},
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-01-09T10:30:00Z",
    "version": "1.0"
  }
}
```

### 주요 엔드포인트 카테고리

| 카테고리 | 엔드포인트 | 목적 |
|----------|-----------|------|
| **신원** | `/identity/verify`, `/identity/documents` | 확인 작업 |
| **스크리닝** | `/screening/sanctions`, `/screening/pep` | 워치리스트 확인 |
| **위험** | `/risk/assess`, `/risk/score` | 위험 평가 |
| **모니터링** | `/monitoring/alerts`, `/monitoring/rules` | 거래 감시 |
| **사례** | `/cases`, `/cases/{id}/sar` | 조사 관리 |
| **고객** | `/customers`, `/customers/{id}` | 고객 데이터 |

---

## 통합 패턴

### 1. 임베디드 확인

금융 기관이 온보딩에 직접 확인을 통합:

```
고객 신청 양식
        ↓
WIA 신원 확인 API
        ↓
실시간 결과 → 계속 또는 추가 정보 요청
```

### 2. 백그라운드 스크리닝

기존 고객에 대한 일괄 스크리닝:

```
고객 목록 내보내기
        ↓
WIA 스크리닝 API (대량)
        ↓
결과 수신 → 매치 조사
```

### 3. 지속적 모니터링

실시간 거래 모니터링:

```
거래 발생
        ↓
WIA 모니터링 API로 푸시
        ↓
규칙/ML 모델에 대해 평가
        ↓
의심스러운 경우 경고 → 사례 관리
```

---

## 배포 모델

### 1. 클라우드 SaaS
- 완전 관리형 서비스
- 가장 빠른 구현
- 거래당 가격
- 적합: 소/중형 기관, 핀테크

### 2. 온프레미스
- 기관의 데이터 센터에 배포
- 완전한 데이터 제어
- 라이선스 기반 가격
- 적합: 대형 은행, 규제 요구사항

### 3. 하이브리드
- 클라우드에서 핵심 처리
- 온프레미스에서 민감한 데이터
- 유연한 아키텍처
- 적합: 글로벌 은행, 규제 산업

---

## 보안 및 개인정보 보호

### 데이터 보호

- **암호화**: 저장 시 AES-256, 전송 시 TLS 1.3
- **액세스 제어**: 역할 기반(RBAC), 최소 권한
- **감사 로깅**: 모든 액세스의 불변 로그
- **데이터 최소화**: 필요한 필드만 수집
- **보존**: 구성 가능 (기본값: 계정 폐쇄 후 5년)

### 컴플라이언스 지원

- **GDPR**: 액세스, 삭제, 이동성에 대한 권리
- **CCPA**: 캘리포니아 소비자 개인정보 보호법
- **SOC 2**: 보안, 가용성, 기밀성
- **ISO 27001**: 정보 보안 관리
- **PCI DSS**: 결제 관련 데이터

---

## 핵심 요점

1. 🏗️ **5가지 핵심 구성 요소**: 신원, 위험, 스크리닝, 모니터링, 사례 관리
2. 📊 **표준화된 데이터 모델**로 상호 운용성 보장
3. 🔌 **RESTful API**로 쉬운 통합
4. ⚖️ FATF 권장 사항에 부합하는 **위험 기반 접근**
5. 🤖 **ML 향상** 감지로 오탐지 감소
6. 🔒 설계에 내장된 **보안 및 개인정보 보호**
7. 🌐 모든 기관에 적합한 **유연한 배포** 옵션

---

## 복습 질문

1. WIA KYC/AML 표준의 7가지 핵심 원칙은 무엇입니까?
2. 5가지 핵심 구성 요소와 그 주요 목적을 나열하십시오.
3. 위험 평가 엔진의 5가지 위험 차원을 설명하십시오.
4. 스크리닝 모듈에서 확인하는 3가지 주요 유형은 무엇입니까?
5. 거래 모니터링이 감지하는 일반적인 자금 세탁 유형을 설명하십시오.
6. 4가지 확인 수준의 차이점은 무엇이며 각각 언제 사용됩니까?
7. 클라우드 SaaS, 온프레미스 및 하이브리드 배포 모델을 비교하십시오.
8. WIA 표준이 데이터 보호 및 개인정보 보호를 어떻게 보장합니까?
9. SAR(의심스러운 활동 보고서)가 포함해야 하는 핵심 요소는 무엇입니까?
10. 머신러닝이 거래 모니터링 시스템을 어떻게 향상시킵니까?

---

**이전**: [← 제2장 - 현재 과제](02-current-challenges.md) | **다음**: [제4장 - 데이터 형식 →](04-data-format.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
