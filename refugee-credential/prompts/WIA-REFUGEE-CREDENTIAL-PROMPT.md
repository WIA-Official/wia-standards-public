# WIA-REFUGEE-CREDENTIAL 구현 프롬프트

## 개요

이 문서는 Claude Code 세션에서 WIA-REFUGEE-CREDENTIAL 표준을 구현하기 위한 상세 프롬프트입니다.

**철학**: 홍익인간 (弘益人間) - Benefit All Humanity

**배경**: 난민들은 국가 붕괴로 학력 증빙을 못해 인생이 리셋됩니다. 시리아 의사가 독일에서 청소부를 합니다. 이건 인류의 비극입니다.

---

## 프롬프트 (복사해서 새 Claude Code 세션에 붙여넣기)

```
당신은 WIA(World Certification Industry Association)의 표준 개발자입니다.

# 임무
WIA-REFUGEE-CREDENTIAL v1.0 표준을 구현하세요.
이 표준은 국가 붕괴/전쟁으로 학력 증빙을 잃은 사람들이 자신의 역량을 증명할 수 있게 합니다.

# 배경
- 전쟁/재난으로 국가 시스템 붕괴 시 학력 증빙 불가능
- 시리아 의사가 증빙 없이 독일에서 청소부로 생활
- 우크라이나 교수가 폴란드에서 공장 노동자로 생활
- 이건 개인의 문제가 아닌 인류 전체의 손실
- 현재 이 문제를 해결하는 국제 표준 없음
- WIA가 인류애의 인프라를 구축

# 핵심 원칙

1. **국가 독립성**: 특정 국가에 의존하지 않음
2. **영구 보존**: WIA-PQ-CRYPTO 기반 양자내성 암호로 저장
3. **역량 기반**: 서류 없이도 실력으로 증명 가능
4. **무료**: 난민에게 모든 서비스 무료
5. **호환성**: 전 세계 교육기관과 호환

# 표준이 정의해야 할 것

## 1. Universal Credential Schema (범용 학력 스키마)
```yaml
wia_credential:
  version: "1.0"

  # 신원 정보 (프라이버시 보호)
  identity:
    credential_id: "uuid"
    holder_did: "did:wia:..."      # 탈중앙화 식별자
    anonymous_option: boolean       # 익명 증명 가능

  # 학력 정보
  education:
    - level: "doctorate | masters | bachelors | high_school | primary"
      field: "string"               # 전공
      institution: "string"         # 학교명 (기억 기반)
      country: "string"             # 국가
      year_start: integer
      year_end: integer
      status: "completed | incomplete | unknown"

      # 증빙 방법
      verification:
        method: "document | assessment | peer | self_declaration"
        confidence: float           # 0.0 ~ 1.0
        verifier: "string"          # 검증 기관/개인
        date: "ISO8601"

  # 역량 평가 (서류 없이 실력으로)
  competencies:
    - domain: "string"              # 의학, 공학, 법학 등
      skill: "string"               # 구체적 기술
      level: "novice | competent | proficient | expert"
      assessment_type: "test | portfolio | interview | simulation"
      assessed_by: "string"
      score: float
      date: "ISO8601"

  # 언어 능력
  languages:
    - code: "string"                # ISO 639-1
      speaking: "A1 | A2 | B1 | B2 | C1 | C2"
      writing: "A1 | A2 | B1 | B2 | C1 | C2"
      reading: "A1 | A2 | B1 | B2 | C1 | C2"

  # 직업 경력
  experience:
    - title: "string"
      field: "string"
      years: integer
      description: "string"
      verification: {}

  # 메타데이터
  metadata:
    created: "ISO8601"
    updated: "ISO8601"
    issuer: "WIA"
    signature: "string"             # WIA-PQ-CRYPTO 서명
    storage_hash: "string"          # 분산 저장 해시
```

## 2. Verification Protocol (검증 프로토콜)

### Level 1: Self-Declaration (자기 선언)
- 본인이 직접 입력
- 신뢰도: 낮음 (0.3)
- 즉시 발급

### Level 2: Peer Verification (동료 검증)
- 같은 학교/직장 출신 2명 이상 확인
- 신뢰도: 중간 (0.6)
- 동료 찾기 네트워크 제공

### Level 3: Assessment (역량 평가)
- 온라인 테스트, 포트폴리오, 인터뷰
- 신뢰도: 높음 (0.8)
- WIA 공인 평가기관 연계

### Level 4: Document Verification (문서 검증)
- 부분적 문서라도 있으면 검증
- 신뢰도: 매우 높음 (0.95)
- AI 기반 문서 진위 확인

## 3. Storage Layer (저장 계층)

### WIA-PQ-CRYPTO 기반
- 양자내성 암호화 (CRYSTALS-Dilithium)
- 분산 저장 (국가 단일 장애점 제거)
- 암호화된 백업 (본인만 복호화 가능)

### 저장 구조
```
credential_storage:
  primary: "WIA 서버 (다중 국가)"
  backup: "IPFS / Arweave"
  local: "사용자 기기 (암호화)"

  access:
    holder: "full"
    verifier: "with_consent"
    public: "anonymized_stats_only"
```

## 4. Recognition Protocol (인정 프로토콜)

### 교육기관 연동
```yaml
recognition_agreement:
  institution: "University of Berlin"
  country: "Germany"
  accepts:
    - credential_levels: ["Level 3", "Level 4"]
    - minimum_confidence: 0.7
  benefits:
    - "입학 서류 대체"
    - "학점 인정 검토"
  wia_provides:
    - "API 무료 사용"
    - "검증 시스템 연동"
```

### API for Institutions
```typescript
interface CredentialVerification {
  verify(credentialId: string): Promise<VerificationResult>;
  checkCompetency(holderId: string, domain: string): Promise<CompetencyResult>;
  issueRecognition(credentialId: string, recognition: Recognition): Promise<void>;
}
```

## 5. Refugee Support Protocol (난민 지원 프로토콜)

### 신청 프로세스
1. 난민 지위 확인 (UNHCR 연계 또는 자가 선언)
2. 기본 정보 입력 (기억 기반)
3. 동료 찾기 네트워크 연결
4. 역량 평가 기회 제공 (무료)
5. Credential 발급
6. 교육기관/고용주 연결

### 다국어 지원
- 아랍어, 우크라이나어, 다리어(페르시아어), 영어, 프랑스어, 스페인어 등
- 모든 인터페이스 다국어

### 접근성
- 저사양 기기 지원
- 오프라인 모드
- SMS 기반 인증 옵션

# 구현 요구사항

## Phase 1: Specification (스펙 문서)
- `/refugee-credential/spec/WIA-REFUGEE-CREDENTIAL-v1.0.md` 작성
- 최소 800줄 이상의 상세 스펙
- 모든 스키마, 프로토콜, 프로세스 정의
- 난민 사용자 시나리오 포함

## Phase 2: Schema Definition
- `/refugee-credential/schema/credential.schema.json`
- `/refugee-credential/schema/verification.schema.json`
- `/refugee-credential/schema/recognition.schema.json`
- JSON Schema 및 검증 규칙

## Phase 3: TypeScript SDK
경로: `/refugee-credential/api/typescript/`
```
src/
  types.ts              # 타입 정의

  credential/
    builder.ts          # Credential 생성
    validator.ts        # Credential 검증
    storage.ts          # 저장/조회

  verification/
    self.ts             # Level 1: 자기선언
    peer.ts             # Level 2: 동료검증
    assessment.ts       # Level 3: 역량평가
    document.ts         # Level 4: 문서검증

  recognition/
    institution.ts      # 교육기관 연동
    employer.ts         # 고용주 연동

  crypto/
    pq-sign.ts          # WIA-PQ-CRYPTO 서명
    pq-verify.ts        # 서명 검증
    encrypt.ts          # 암호화

  i18n/
    ar.ts               # 아랍어
    uk.ts               # 우크라이나어
    fa.ts               # 페르시아어
    ...

  index.ts              # 메인 export
```

## Phase 4: Examples & User Stories
- 시리아 의사 시나리오
- 우크라이나 교수 시나리오
- 아프가니스탄 교사 시나리오
- 에리트레아 엔지니어 시나리오

# 참고 자료
- UNHCR (유엔난민기구) 자료
- European Qualifications Passport for Refugees
- UNESCO Global Convention on Recognition of Qualifications
- W3C Verifiable Credentials
- WIA-PQ-CRYPTO 표준: `/pq-crypto/spec/WIA-PQ-CRYPTO-v1.0.md`
- WIA-QUANTUM 구현체 참고: `/quantum/api/typescript/`

# 철학
홍익인간 (弘益人間)

이 표준은 돈을 위한 것이 아닙니다.
국가가 무너져도 사람의 가치는 무너지지 않습니다.
시리아 의사는 여전히 의사입니다.
우크라이나 교수는 여전히 교수입니다.

WIA는 그들의 가치를 증명합니다.
무료로, 영원히.

# 완료 조건
1. 스펙 문서 완성 (800줄+)
2. JSON Schema 완성
3. TypeScript SDK 완성 (빌드 가능)
4. 난민 시나리오 4개 이상
5. 다국어 지원 (최소 5개 언어)
6. README.md 작성
7. 모든 파일 wia-standards 레포에 커밋/푸시
```

---

## 예상 결과물

```
refugee-credential/
├── spec/
│   └── WIA-REFUGEE-CREDENTIAL-v1.0.md
├── schema/
│   ├── credential.schema.json
│   ├── verification.schema.json
│   └── recognition.schema.json
├── api/
│   └── typescript/
│       ├── src/
│       ├── package.json
│       └── README.md
├── examples/
│   ├── syrian-doctor/
│   ├── ukrainian-professor/
│   ├── afghan-teacher/
│   └── eritrean-engineer/
└── i18n/
    ├── ar.json
    ├── uk.json
    ├── fa.json
    └── ...
```

---

## 연계 시스템

```
WIA-REFUGEE-CREDENTIAL
         │
         ├── WIA-PQ-CRYPTO (양자내성 암호)
         │
         ├── WIA Academy (무상 교육)
         │
         └── WIA Book (무료 열람)
```

---

**WIA - World Certification Industry Association**
**홍익인간 (弘益人間)**

*"국가가 무너져도, 사람의 가치는 무너지지 않습니다."*
