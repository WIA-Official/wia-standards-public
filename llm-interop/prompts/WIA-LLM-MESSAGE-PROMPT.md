# WIA-LLM-MESSAGE 구현 프롬프트

## 개요

이 문서는 Claude Code 세션에서 WIA-LLM-MESSAGE 표준을 구현하기 위한 상세 프롬프트입니다.

**철학**: 홍익인간 (弘益人間) - Benefit All Humanity

---

## 프롬프트 (복사해서 새 Claude Code 세션에 붙여넣기)

```
당신은 WIA(World Certification Industry Association)의 표준 개발자입니다.

# 임무
WIA-LLM-MESSAGE v1.0 표준을 구현하세요.
이 표준은 AI/LLM들이 서로 메시지를 주고받는 형식을 정의합니다.

# 배경
- LLM들이 협업해야 복잡한 문제 해결 가능
- 현재: 각 회사 API 형식이 다름 (OpenAI ≠ Anthropic ≠ Google)
- 필요: AI-to-AI 통신의 표준 메시지 형식
- 목표: WIA가 de facto 표준 선점

# 표준이 정의해야 할 것

## 1. Message Envelope (메시지 봉투)
```json
{
  "wia_llm_message": {
    "version": "1.0",
    "message_id": "uuid",
    "timestamp": "ISO8601",
    "trace_id": "uuid",           // 대화 추적용
    "parent_message_id": "uuid",  // 답변인 경우

    "from": {
      "model_id": "string",
      "capability_uri": "url"     // WIA-LLM-CAPABILITY 참조
    },

    "to": {
      "model_id": "string",       // 특정 모델 지정
      "capability_query": {}      // 또는 능력 기반 라우팅
    },

    "type": "query | response | stream | error | handshake | heartbeat",

    "priority": "low | normal | high | critical",

    "ttl": integer,               // Time-to-live (초)

    "payload": {},                // 실제 내용

    "metadata": {}                // 확장 가능한 메타데이터
  }
}
```

## 2. Message Types (메시지 유형)

### Query (질의)
```json
{
  "type": "query",
  "payload": {
    "intent": "string",           // 의도 분류
    "content": "string",          // 실제 질문/요청
    "context": [],                // 이전 대화 컨텍스트
    "constraints": {
      "max_tokens": integer,
      "response_format": "text | json | markdown",
      "language": "string",
      "confidence_min": float     // 최소 확신도 요구
    },
    "attachments": []             // 첨부 데이터
  }
}
```

### Response (응답)
```json
{
  "type": "response",
  "payload": {
    "content": "string",
    "confidence": float,          // 0.0 ~ 1.0
    "reasoning": "string",        // 추론 과정 (선택적)
    "sources": [],                // 참조 소스
    "suggestions": [],            // 후속 질문 제안
    "delegations": []             // 다른 AI에게 위임 제안
  }
}
```

### Stream (스트리밍)
```json
{
  "type": "stream",
  "payload": {
    "chunk_index": integer,
    "total_chunks": integer,      // 알 수 있는 경우
    "content": "string",
    "is_final": boolean
  }
}
```

### Error (에러)
```json
{
  "type": "error",
  "payload": {
    "code": "string",             // 표준 에러 코드
    "message": "string",
    "recoverable": boolean,
    "retry_after": integer,       // 초
    "fallback_suggestion": {}     // 대안 제안
  }
}
```

### Handshake (연결 수립)
```json
{
  "type": "handshake",
  "payload": {
    "phase": "init | ack | complete",
    "capability": {},             // WIA-LLM-CAPABILITY 객체
    "supported_versions": [],
    "preferred_encoding": "utf-8",
    "compression": "none | gzip | zstd"
  }
}
```

## 3. Error Codes (에러 코드)
표준 에러 코드 체계 정의:
- WIA-E001: Capability Mismatch
- WIA-E002: Message Format Invalid
- WIA-E003: Rate Limit Exceeded
- WIA-E004: Confidence Below Threshold
- WIA-E005: Context Too Long
- WIA-E006: Unsupported Language
- WIA-E007: Service Unavailable
- WIA-E008: Authentication Failed
- WIA-E009: Authorization Denied
- WIA-E010: Timeout
- ... (더 정의)

## 4. Transport Layer (전송 계층)
- HTTP/HTTPS 기반 (REST-like)
- WebSocket 기반 (실시간)
- gRPC 기반 (고성능)
- 각각의 바인딩 명세

## 5. Security (보안)
- 메시지 서명
- 암호화 옵션
- 인증 토큰 처리

# 구현 요구사항

## Phase 1: Specification (스펙 문서)
- `/llm-interop/message/spec/WIA-LLM-MESSAGE-v1.0.md` 작성
- 최소 600줄 이상의 상세 스펙
- 모든 메시지 유형, 필드, 에러 코드 정의

## Phase 2: Schema Definition
- `/llm-interop/message/schema/message.schema.json` - JSON Schema
- `/llm-interop/message/schema/error-codes.json` - 에러 코드 정의
- 검증 가능한 스키마

## Phase 3: TypeScript SDK
경로: `/llm-interop/message/api/typescript/`
```
src/
  types.ts          # 메시지 타입 정의
  builder.ts        # 메시지 빌더 (Fluent API)
  parser.ts         # 메시지 파서
  validator.ts      # 메시지 검증기
  transport/
    http.ts         # HTTP 전송
    websocket.ts    # WebSocket 전송
  security/
    signer.ts       # 메시지 서명
    verifier.ts     # 서명 검증
  index.ts          # 메인 export
```

## Phase 4: Examples
- 다양한 메시지 교환 시나리오 예시
- Query-Response 예시
- Streaming 예시
- Error handling 예시
- Multi-hop 대화 예시 (A → B → C → B → A)

# 참고 자료
- JSON-RPC 2.0 명세
- GraphQL 메시지 구조
- gRPC/Protocol Buffers
- MQTT 메시지 형식
- 기존 WIA-QUANTUM 구현체 구조 참고: `/quantum/api/typescript/`

# 철학
홍익인간 (弘益人間) - 이 표준은 모든 AI가 자유롭게 대화할 수 있게 합니다.
언어, 국경, 회사의 벽을 넘어.

# 완료 조건
1. 스펙 문서 완성
2. JSON Schema 완성
3. TypeScript SDK 완성 (빌드 가능)
4. 예시 시나리오 5개 이상
5. README.md 작성
6. 모든 파일 wia-standards 레포에 커밋/푸시
```

---

## 예상 결과물

```
llm-interop/
└── message/
    ├── spec/
    │   └── WIA-LLM-MESSAGE-v1.0.md
    ├── schema/
    │   ├── message.schema.json
    │   └── error-codes.json
    ├── api/
    │   └── typescript/
    │       ├── src/
    │       ├── package.json
    │       └── README.md
    └── examples/
        ├── query-response.json
        ├── streaming.json
        ├── multi-hop.json
        └── ...
```

---

**WIA - World Certification Industry Association**
**홍익인간 (弘益人間)**
