# WIA-LLM-CAPABILITY 구현 프롬프트

## 개요

이 문서는 Claude Code 세션에서 WIA-LLM-CAPABILITY 표준을 구현하기 위한 상세 프롬프트입니다.

**철학**: 홍익인간 (弘益人間) - Benefit All Humanity

---

## 프롬프트 (복사해서 새 Claude Code 세션에 붙여넣기)

```
당신은 WIA(World Certification Industry Association)의 표준 개발자입니다.

# 임무
WIA-LLM-CAPABILITY v1.0 표준을 구현하세요.
이 표준은 AI/LLM들이 자신의 능력을 선언하는 방식을 정의합니다.

# 배경
- 수많은 LLM들이 등장하고 있음 (GPT, Claude, Gemini, Llama, 특화 AI들...)
- 이들이 협업하려면 서로의 "능력"을 알아야 함
- 현재 표준 없음 → WIA가 de facto 표준 선점

# 표준이 정의해야 할 것

## 1. Capability Schema (능력 스키마)
```yaml
wia_llm_capability:
  version: "1.0"
  model_id: string          # 고유 식별자
  model_name: string        # 표시 이름
  provider: string          # 제공자 (OpenAI, Anthropic, etc.)

  # 레벨 분류
  level: 1 | 2 | 3 | 4
  # 1: 단순 응답
  # 2: 추론 가능
  # 3: 특정 도메인 전문
  # 4: 범용 고급 추론 + 도구 사용

  # 도메인 전문성
  domains:
    - domain: string        # "medical", "legal", "code", "finance", etc.
      proficiency: float    # 0.0 ~ 1.0
      sub_domains: [string]
      certifications: [string]  # 선택적 인증 정보

  # 지원 언어
  languages:
    - code: string          # ISO 639-1 (ko, en, ja, zh, ...)
      proficiency: float    # 0.0 ~ 1.0

  # 기능 플래그
  capabilities:
    text_generation: boolean
    code_generation: boolean
    image_understanding: boolean
    image_generation: boolean
    audio_understanding: boolean
    audio_generation: boolean
    tool_use: boolean
    web_search: boolean
    file_operations: boolean
    long_context: boolean
    reasoning: boolean
    math: boolean

  # 제한사항
  limits:
    max_input_tokens: integer
    max_output_tokens: integer
    max_context_window: integer
    rate_limit_rpm: integer     # requests per minute
    rate_limit_tpm: integer     # tokens per minute

  # 신뢰도 메타데이터
  trust:
    confidence_threshold: float  # 이 이하 확신도면 "모름" 응답
    hallucination_rate: float    # 자체 평가된 환각률
    last_knowledge_update: date

  # 연락처
  contact:
    endpoint: url
    documentation: url
    support: email
```

## 2. Discovery Protocol (발견 프로토콜)
- AI가 자신의 capability를 어디에 등록하는가
- 다른 AI가 어떻게 조회하는가
- JSON-LD 또는 유사한 linked data 형식 고려

## 3. Capability Negotiation (능력 협상)
- 두 AI가 대화 시작 전 서로의 capability 교환
- 어떤 형식으로? 어떤 프로토콜로?
- Handshake 과정 정의

## 4. Versioning (버전 관리)
- Capability schema 버전
- 하위 호환성 규칙

# 구현 요구사항

## Phase 1: Specification (스펙 문서)
- `/llm-interop/capability/spec/WIA-LLM-CAPABILITY-v1.0.md` 작성
- 최소 500줄 이상의 상세 스펙
- 모든 필드 설명, 예시, 에러 케이스 포함

## Phase 2: Schema Definition
- `/llm-interop/capability/schema/capability.schema.json` - JSON Schema
- `/llm-interop/capability/schema/capability.schema.yaml` - YAML 버전
- 검증 가능한 스키마

## Phase 3: TypeScript SDK
경로: `/llm-interop/capability/api/typescript/`
```
src/
  types.ts          # 타입 정의
  validator.ts      # Capability 검증기
  registry.ts       # 등록/조회 클라이언트
  negotiator.ts     # Capability 협상 로직
  index.ts          # 메인 export
```

## Phase 4: Examples
- 다양한 AI의 capability 예시 파일들
- GPT-4 예시, Claude 예시, 의료AI 예시, 법률AI 예시 등

# 참고 자료
- OpenAPI Specification 구조 참고
- JSON-LD for linked data
- OAuth 2.0의 scope 개념 참고
- 기존 WIA-QUANTUM 구현체 구조 참고: `/quantum/api/typescript/`

# 철학
홍익인간 (弘益人間) - 이 표준은 특정 회사가 아닌 인류 전체를 위한 것입니다.
모든 AI가 자유롭게 사용할 수 있어야 합니다.

# 완료 조건
1. 스펙 문서 완성
2. JSON Schema 완성
3. TypeScript SDK 완성 (빌드 가능)
4. 예시 파일 5개 이상
5. README.md 작성
6. 모든 파일 wia-standards 레포에 커밋/푸시
```

---

## 예상 결과물

```
llm-interop/
└── capability/
    ├── spec/
    │   └── WIA-LLM-CAPABILITY-v1.0.md
    ├── schema/
    │   ├── capability.schema.json
    │   └── capability.schema.yaml
    ├── api/
    │   └── typescript/
    │       ├── src/
    │       ├── package.json
    │       └── README.md
    └── examples/
        ├── gpt-4.capability.yaml
        ├── claude-opus.capability.yaml
        ├── medical-ai.capability.yaml
        └── ...
```

---

**WIA - World Certification Industry Association**
**홍익인간 (弘益人間)**
