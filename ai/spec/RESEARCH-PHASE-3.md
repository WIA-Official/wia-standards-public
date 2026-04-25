# Phase 3 사전 조사 결과

**작성일**: 2025-01
**버전**: 1.0.0
**상태**: Complete

---

## 1. AI API 프로토콜 비교

### 1.1 OpenAI API

**스트리밍 방식**:
- Server-Sent Events (SSE) 기반 스트리밍
- `data: {json}\n\n` 형식으로 이벤트 전송
- 스트림 종료 시 `data: [DONE]` 마커 사용
- HTTP POST 요청에서 `stream: true` 설정으로 활성화

**함수 호출 (Function Calling)**:
- `tools` 파라미터로 함수 정의
- `tool_choice`: "auto", "none", "required" 또는 특정 함수 지정
- 함수 실행 결과를 `tool` 역할 메시지로 반환

**스트리밍 형식**:
```
event: message
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {"content": "Hello"}}]}

event: message
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {"content": " world"}}]}

data: [DONE]
```

**참고 문서**:
- [Streaming API responses](https://platform.openai.com/docs/guides/streaming-responses)
- [OpenAI streaming API reference](https://platform.openai.com/docs/api-reference/streaming)

---

### 1.2 Anthropic Claude API

**스트리밍 방식**:
- Server-Sent Events (SSE) 기반
- `"stream": true` 설정으로 활성화
- API 버전 헤더: `anthropic-version: 2023-06-01`

**스트리밍 이벤트 타입**:
| Event | Description |
|-------|-------------|
| `message_start` | 초기 메시지 메타데이터 |
| `content_block_start` | 콘텐츠 블록 시작 |
| `content_block_delta` | 증분 콘텐츠 업데이트 (text 또는 tool input) |
| `content_block_stop` | 콘텐츠 블록 종료 |
| `message_delta` | 메시지 수준 업데이트 |
| `message_stop` | 메시지 종료 |
| `ping` | Keep-alive 이벤트 |

**도구 사용 (Tool Use)**:
- Claude 3 모델 패밀리에서 정식 지원
- `tool_choice`: "auto", "none", "required" 또는 특정 도구 지정
- Fine-grained tool streaming (Beta): `fine-grained-tool-streaming-2025-05-14` 헤더

**스트리밍 형식**:
```
event: message_start
data: {"type": "message_start", "message": {"id": "msg_xxx", "model": "claude-opus-4-5"}}

event: content_block_delta
data: {"type": "content_block_delta", "delta": {"type": "text_delta", "text": "Hello"}}

event: message_stop
data: {"type": "message_stop"}
```

**참고 문서**:
- [Streaming Messages - Anthropic](https://docs.anthropic.com/en/docs/build-with-claude/streaming)
- [Fine-grained tool streaming](https://docs.anthropic.com/en/docs/agents-and-tools/tool-use/fine-grained-tool-streaming)

---

### 1.3 MCP (Model Context Protocol)

**개요**:
- 2024년 11월 Anthropic에서 발표한 오픈 표준
- AI 어시스턴트와 데이터 소스 간 보안 양방향 연결 표준화
- "MxN 문제" 해결: M개의 LLM과 N개의 도구 통합 표준화
- 2025년 12월 Linux Foundation 산하 Agentic AI Foundation (AAIF)에 기부

**아키텍처**:
- Language Server Protocol (LSP) 메시지 흐름 재사용
- JSON-RPC 2.0 기반 전송
- 표준 전송 메커니즘: stdio, HTTP (선택적 SSE)

**서버 프리미티브**:
| Primitive | Description |
|-----------|-------------|
| **Prompts** | 지시사항 또는 템플릿 |
| **Resources** | LLM 프롬프트 컨텍스트에 포함될 구조화된 데이터 |
| **Tools** | 정보 검색 또는 작업 수행을 위한 실행 가능 함수 |

**클라이언트 프리미티브**:
| Primitive | Description |
|-----------|-------------|
| **Roots** | 파일 시스템 진입점, 서버에 클라이언트 파일 접근 권한 부여 |
| **Sampling** | 서버가 클라이언트 측 LLM에서 완성 요청 |

**SDK 지원**:
- Python, TypeScript, C#, Java

**채택 현황**:
- 2025년 3월: OpenAI 공식 채택
- Google, Microsoft, AWS, Cloudflare, Bloomberg 지원

**참고 문서**:
- [MCP GitHub](https://github.com/modelcontextprotocol)
- [Anthropic MCP Documentation](https://docs.anthropic.com/en/docs/mcp)
- [Official Announcement](https://www.anthropic.com/news/model-context-protocol)

---

### 1.4 A2A (Agent-to-Agent Protocol)

**개요**:
- 2025년 4월 Google에서 발표한 오픈 표준
- AI 에이전트 간 통신 및 협업 표준화
- Linux Foundation 산하 오픈소스 프로젝트
- 50개 이상 기업 지원 (Atlassian, Cohere, Salesforce 등)

**핵심 목표**:
1. **Break Down Silos**: 서로 다른 에코시스템의 에이전트 연결
2. **Enable Complex Collaboration**: 단일 에이전트가 처리할 수 없는 복잡한 작업 협업
3. **Promote Open Standards**: 커뮤니티 주도 접근 방식
4. **Preserve Opacity**: 내부 메모리, 독점 로직, 도구 구현 공유 없이 협업

**기술 사양**:
- JSON-RPC 2.0 over HTTP(S)
- 동기 요청/응답, 스트리밍 (SSE), 비동기 푸시 알림 지원

**핵심 컴포넌트**:
| Component | Description |
|-----------|-------------|
| **Agent Card** | 에이전트 기능 및 연결 정보 광고 (JSON 형식) |
| **Task Management** | 태스크 객체 기반 클라이언트-에이전트 통신 |
| **Collaboration** | 컨텍스트, 응답, 아티팩트, 사용자 지시사항 메시지 교환 |
| **Content Negotiation** | 콘텐츠 타입이 지정된 "parts" 포함 메시지 |

**A2A vs MCP**:
| Protocol | Purpose |
|----------|---------|
| **A2A** | 에이전트 간 통신 (Agent-to-Agent) |
| **MCP** | 에이전트와 도구 간 통신 (Agent-to-Tool) |

**참고 문서**:
- [A2A GitHub](https://github.com/google-a2a/A2A)
- [Google Developers Blog](https://developers.googleblog.com/en/a2a-a-new-era-of-agent-interoperability/)

---

## 2. 멀티 에이전트 프레임워크

### 2.1 LangChain Agent Protocol

**Agent Protocol (2024년 11월)**:
- LLM 에이전트 간 상호운용성을 위한 표준 인터페이스
- 프레임워크 독립적 API
- LangGraph, 다른 프레임워크, 또는 프레임워크 없이도 구현 가능

**프로토콜 기능**:
| Endpoint | Description |
|----------|-------------|
| `POST /threads` | 스레드 생성 |
| `POST /threads/search` | 스레드 검색 |
| `GET /threads/{thread_id}` | 스레드 조회 |
| `GET /threads/{thread_id}/runs` | 스레드의 실행 목록 |
| `POST /runs` | 백그라운드 실행 생성 |
| `GET /runs/{run_id}` | 실행 및 상태 조회 |

**Multi-Agent Workflows (LangGraph)**:
- 에이전트를 그래프 노드로, 연결을 엣지로 표현
- 엣지로 제어 흐름 관리
- 그래프 상태에 추가하여 통신

**Command 도구 (2024년 12월)**:
- 멀티 에이전트 아키텍처에서 쉬운 핸드오프 지원
- 이벤트 기반 시스템

**A2A 통합**:
- LangSmith에서 A2A 엔드포인트 지원
- A2A 호환 에이전트와의 표준화된 통신

**참고 문서**:
- [Agent Protocol GitHub](https://github.com/langchain-ai/agent-protocol)
- [LangGraph Multi-Agent Workflows](https://blog.langchain.com/langgraph-multi-agent-workflows/)

---

### 2.2 Microsoft AutoGen

**개요**:
- 멀티 에이전트 대화를 통한 LLM 애플리케이션 구축 프레임워크
- 2024년 가을 AutoGen v0.4 프리뷰, 2025년 초 정식 릴리스
- COLM 2024 컨퍼런스에서 논문 발표

**아키텍처**:
- **Actor Model**: 멀티 에이전트 오케스트레이션을 위한 동시성 프로그래밍 모델
- **비동기 이벤트 기반 아키텍처**: 더 넓은 범위의 에이전트 시나리오 지원
- 더 강력한 관찰성, 유연한 협업 패턴, 재사용 가능한 컴포넌트

**레이어 설계**:
| Layer | Description |
|-------|-------------|
| **AutoGen Core** | Actor 모델 기반 에이전트 구현 |
| **AutoGen AgentChat** | 빠른 프로토타이핑을 위한 간단하고 사용하기 쉬운 API |

**핵심 컴포넌트**:
- **ConversableAgent**: 모든 에이전트의 기본 클래스, 통신 및 작업 실행 담당
- **UserProxyAgent**: 사용자와 시스템 간 중개자 역할

**통신 패턴**:
- 비동기 메시지를 통한 에이전트 통신
- 이벤트 기반 및 요청/응답 상호작용 패턴 지원

**참고 문서**:
- [AutoGen GitHub](https://github.com/microsoft/autogen)
- [AutoGen Research](https://www.microsoft.com/en-us/research/project/autogen/)

---

## 3. 실시간 스트리밍 기술

### 3.1 Server-Sent Events (SSE)

**특징**:
- 서버에서 클라이언트로의 단방향 스트리밍
- 단일 장기 HTTP 연결 유지
- 자동 재연결 지원
- HTTP와의 호환성 우수

**형식**:
```
event: {event_type}
data: {json_data}
id: {event_id}
retry: {milliseconds}

```

**장점**:
- 간단한 구현
- HTTP 인프라와 호환
- 네이티브 브라우저 지원 (EventSource API)
- 자동 재연결

**단점**:
- 단방향 통신 (서버 → 클라이언트)
- 바이너리 데이터 전송 제한
- 연결당 하나의 스트림

---

### 3.2 WebSocket

**특징**:
- 전이중 양방향 통신
- 단일 TCP 연결 위에서 지속적 연결
- 낮은 지연 시간
- 바이너리 및 텍스트 데이터 지원

**AI 에이전트에서의 활용**:
- OpenAI Realtime API: 실시간 음성 상호작용
- Google Gemini: 실시간 기능
- MCP-WebSocket: 실시간 에이전트 시스템 구축

**장점**:
- 양방향 실시간 통신
- 낮은 오버헤드
- 고성능 실시간 업데이트

**단점**:
- 구현 복잡성
- 프록시/방화벽 호환성 문제 가능
- 리소스 집약적

**참고**:
- [WebSocket AI real-time communication](https://hammadulhaq.medium.com/the-demise-of-rest-as-we-know-it-websockets-as-the-new-standard-for-ai-agents-72c505098320)

---

### 3.3 gRPC Streaming

**스트리밍 유형**:
| Type | Description |
|------|-------------|
| **Server streaming** | 클라이언트 요청 → 서버 응답 스트림 |
| **Client streaming** | 클라이언트 요청 스트림 → 서버 응답 |
| **Bidirectional streaming** | 양방향 동시 스트리밍 |

**특징**:
- HTTP/2 기반 고성능
- Protocol Buffers를 통한 효율적인 직렬화
- 강력한 타입 지정
- 다양한 언어 지원

**AI 에이전트에서의 적용**:
- 고처리량 프로덕션 시스템
- 실시간 데이터 피드
- 밀리초가 중요한 성능 요구사항

**현재 상태**:
- gRPC-web: 양방향 스트리밍 미구현 (2024년 3월 기준)
- MCP는 AI 에이전트 전용으로 설계, gRPC는 범용 RPC

---

### 3.4 JSON Lines (JSONL)

**특징**:
- 줄바꿈으로 구분된 JSON 객체
- 스트리밍 친화적 형식
- 점진적 파싱 가능

**형식**:
```jsonl
{"id": 1, "delta": {"text": "Hello"}}
{"id": 2, "delta": {"text": " world"}}
{"id": 3, "done": true}
```

**장점**:
- 간단한 파싱
- 부분 데이터 처리 가능
- 대용량 데이터셋에 적합

---

## 4. 프로토콜 비교 요약

| Protocol | Transport | Direction | Use Case |
|----------|-----------|-----------|----------|
| **SSE** | HTTP | 단방향 | LLM 응답 스트리밍 |
| **WebSocket** | TCP | 양방향 | 실시간 에이전트 통신 |
| **gRPC** | HTTP/2 | 양방향 | 고성능 마이크로서비스 |
| **MCP** | stdio, HTTP+SSE | 요청/응답 | 에이전트-도구 통신 |
| **A2A** | HTTP+JSON-RPC | 요청/응답+스트리밍 | 에이전트 간 협업 |

---

## 5. 권장 통신 방식

### 5.1 WIA AI 프로토콜 설계 방향

1. **기본 전송**: SSE 기반 스트리밍 (OpenAI/Anthropic 호환)
2. **실시간 통신**: WebSocket 지원 (선택적)
3. **에이전트 간 통신**: A2A 프로토콜 호환
4. **도구 호출**: MCP 프로토콜 호환
5. **메시지 형식**: JSON-RPC 2.0 기반

### 5.2 메시지 프로토콜 설계 방향

1. **Phase 1 호환성**: 기존 데이터 형식을 메시지 페이로드로 사용
2. **확장성**: 새로운 메시지 타입 추가 용이
3. **상호운용성**: MCP, A2A, LangChain Agent Protocol과 호환
4. **스트리밍 우선**: 실시간 스트리밍을 기본 기능으로
5. **안전성**: 인증, 암호화, 감사 로깅 포함

### 5.3 핵심 메시지 타입

| Category | Message Types |
|----------|--------------|
| **Connection** | connect, connect_ack, disconnect, ping, pong |
| **Streaming** | stream_start, stream_delta, stream_end |
| **Tool** | tool_call, tool_result |
| **Agent** | handoff, handoff_ack, broadcast |
| **Error** | error |

---

## 6. 참고 문헌

### AI API
- OpenAI API: https://platform.openai.com/docs/api-reference
- Anthropic API: https://docs.anthropic.com/en/api
- AI SDK Stream Protocol: https://ai-sdk.dev/docs/ai-sdk-ui/stream-protocol

### Protocols
- MCP Specification: https://modelcontextprotocol.io
- A2A Protocol: https://github.com/google-a2a/A2A
- LangChain Agent Protocol: https://github.com/langchain-ai/agent-protocol

### Streaming Technologies
- SSE Specification: https://html.spec.whatwg.org/multipage/server-sent-events.html
- WebSocket Protocol: https://www.rfc-editor.org/rfc/rfc6455
- gRPC Streaming: https://grpc.io/docs/what-is-grpc/core-concepts/#server-streaming-rpc

### Frameworks
- AutoGen: https://github.com/microsoft/autogen
- LangGraph: https://langchain-ai.github.io/langgraph/

---

**문서 버전**: 1.0.0
**최종 수정**: 2025-01
**작성자**: WIA AI Working Group
