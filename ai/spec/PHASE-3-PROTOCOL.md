# WIA AI Communication Protocol Specification

**Phase 3: Communication Protocol**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01

---

## 1. Overview

### 1.1 Purpose

WIA AI Communication Protocol은 AI 에이전트와 시스템 간의 통신을 표준화하는 프로토콜입니다. 이 프로토콜은 다음을 지원합니다:

- 에이전트 간 통신 (Agent-to-Agent, A2A)
- 에이전트와 도구 간 통신 (Agent-to-Tool)
- 실시간 스트리밍 (Real-time Streaming)
- 멀티 에이전트 조율 (Multi-Agent Coordination)
- 에러 처리 및 재시도 (Error Handling & Retry)

### 1.2 Scope

- **In Scope**:
  - 메시지 형식 및 프로토콜 정의
  - 스트리밍 프로토콜 (SSE, WebSocket)
  - 에이전트 간 핸드오프
  - 도구 호출 프로토콜
  - 에러 코드 및 재시도 정책

- **Out of Scope** (Phase 3):
  - 보안 인프라 구현 세부사항 (Phase 4)
  - 특정 클라우드 프로바이더 통합 (Phase 4)

### 1.3 Design Principles

1. **Interoperability**: MCP, A2A, LangChain Agent Protocol과 호환
2. **Streaming-first**: 실시간 스트리밍을 기본 기능으로 지원
3. **Phase 1 Compatibility**: 기존 데이터 형식을 페이로드로 활용
4. **Extensibility**: 새로운 메시지 타입 추가 용이
5. **Resilience**: 견고한 에러 처리 및 재시도 메커니즘

---

## 2. Terminology

| Term | Definition |
|------|------------|
| **Agent** | WIA AI 표준을 따르는 AI 에이전트 |
| **Client** | 프로토콜을 통해 서버에 연결하는 주체 |
| **Server** | 프로토콜 요청을 처리하는 주체 |
| **Message** | 프로토콜에서 교환되는 데이터 단위 |
| **Payload** | 메시지에 포함된 실제 데이터 |
| **Stream** | 연속적인 메시지의 흐름 |
| **Handoff** | 에이전트 간 작업 전달 |
| **Tool** | 에이전트가 호출할 수 있는 외부 기능 |
| **Orchestrator** | 멀티 에이전트 시스템을 조율하는 에이전트 |

---

## 3. Message Format

### 3.1 Base Message Structure

모든 WIA AI 프로토콜 메시지는 다음 구조를 따릅니다:

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "msg-uuid-v4",
  "timestamp": 1702483200000,
  "type": "message_type",
  "source": {
    "id": "source-id",
    "type": "agent|client|server|tool",
    "name": "Source Name"
  },
  "target": {
    "id": "target-id",
    "type": "agent|client|server|tool",
    "name": "Target Name"
  },
  "payload": {},
  "metadata": {}
}
```

### 3.2 Required Fields

| Field | Type | Description |
|-------|------|-------------|
| `protocol` | string | 항상 "wia-ai" |
| `version` | string | 프로토콜 버전 (semver) |
| `message_id` | string | 고유 메시지 ID (UUID v4) |
| `timestamp` | number | Unix timestamp (밀리초) |
| `type` | string | 메시지 유형 |

### 3.3 Optional Fields

| Field | Type | Description |
|-------|------|-------------|
| `source` | object | 메시지 발신자 정보 |
| `target` | object | 메시지 수신자 정보 |
| `payload` | object | 메시지 데이터 |
| `metadata` | object | 추가 메타데이터 |

### 3.4 Entity Reference

```json
{
  "id": "agent-researcher-001",
  "type": "agent",
  "name": "Research Assistant",
  "version": "1.0.0"
}
```

---

## 4. Message Types

### 4.1 Connection Messages

#### 4.1.1 connect

클라이언트가 서버에 연결을 요청합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "msg-001",
  "timestamp": 1702483200000,
  "type": "connect",
  "source": {
    "id": "client-001",
    "type": "client"
  },
  "payload": {
    "capabilities": ["streaming", "tools"],
    "agent_id": "agent-researcher-001",
    "auth": {
      "type": "bearer",
      "token": "..."
    }
  }
}
```

#### 4.1.2 connect_ack

서버가 연결 요청을 승인합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "msg-002",
  "timestamp": 1702483200100,
  "type": "connect_ack",
  "source": {
    "id": "server-001",
    "type": "server"
  },
  "target": {
    "id": "client-001",
    "type": "client"
  },
  "payload": {
    "session_id": "session-uuid",
    "capabilities": ["streaming", "tools", "handoff"],
    "timeout_seconds": 3600
  }
}
```

#### 4.1.3 disconnect

연결을 종료합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "msg-003",
  "timestamp": 1702483200200,
  "type": "disconnect",
  "payload": {
    "reason": "normal",
    "code": 1000
  }
}
```

#### 4.1.4 ping / pong

연결 상태를 확인합니다.

```json
{
  "type": "ping",
  "message_id": "ping-001",
  "timestamp": 1702483200300
}
```

```json
{
  "type": "pong",
  "message_id": "pong-001",
  "timestamp": 1702483200301,
  "payload": {
    "ping_id": "ping-001"
  }
}
```

---

### 4.2 Content Messages

#### 4.2.1 message

일반 메시지를 전송합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "msg-004",
  "timestamp": 1702483200400,
  "type": "message",
  "source": {
    "id": "client-001",
    "type": "client"
  },
  "target": {
    "id": "agent-researcher-001",
    "type": "agent"
  },
  "payload": {
    "role": "user",
    "content": [
      {
        "type": "text",
        "text": "What is the WIA AI standard?"
      }
    ]
  }
}
```

#### 4.2.2 Content Types

| Type | Description |
|------|-------------|
| `text` | 텍스트 콘텐츠 |
| `image` | 이미지 (base64 또는 URL) |
| `tool_use` | 도구 호출 |
| `tool_result` | 도구 실행 결과 |

---

### 4.3 Streaming Messages

#### 4.3.1 stream_start

스트리밍이 시작됨을 알립니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "stream-001",
  "timestamp": 1702483200500,
  "type": "stream_start",
  "source": {
    "id": "agent-researcher-001",
    "type": "agent"
  },
  "payload": {
    "stream_id": "stream-uuid",
    "model": "claude-opus-4-5-20251101",
    "input_tokens": 150
  }
}
```

#### 4.3.2 stream_delta

스트리밍 콘텐츠 조각을 전송합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "delta-001",
  "timestamp": 1702483200600,
  "type": "stream_delta",
  "source": {
    "id": "agent-researcher-001",
    "type": "agent"
  },
  "payload": {
    "stream_id": "stream-uuid",
    "index": 0,
    "delta": {
      "type": "text_delta",
      "text": "WIA AI is "
    }
  }
}
```

#### 4.3.3 Delta Types

| Type | Description |
|------|-------------|
| `text_delta` | 텍스트 조각 |
| `tool_use_delta` | 도구 호출 조각 (JSON 부분) |
| `thinking_delta` | 사고 과정 조각 |

#### 4.3.4 stream_end

스트리밍이 종료됨을 알립니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "stream-end-001",
  "timestamp": 1702483200700,
  "type": "stream_end",
  "source": {
    "id": "agent-researcher-001",
    "type": "agent"
  },
  "payload": {
    "stream_id": "stream-uuid",
    "stop_reason": "end_turn",
    "usage": {
      "input_tokens": 150,
      "output_tokens": 250,
      "total_tokens": 400
    }
  }
}
```

#### 4.3.5 Stop Reasons

| Reason | Description |
|--------|-------------|
| `end_turn` | 정상 종료 |
| `max_tokens` | 최대 토큰 도달 |
| `stop_sequence` | 중지 시퀀스 감지 |
| `tool_use` | 도구 사용으로 일시 중단 |
| `error` | 에러로 인한 중단 |

---

### 4.4 Tool Messages

#### 4.4.1 tool_call

에이전트가 도구를 호출합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "tool-call-001",
  "timestamp": 1702483200800,
  "type": "tool_call",
  "source": {
    "id": "agent-researcher-001",
    "type": "agent"
  },
  "target": {
    "id": "web_search",
    "type": "tool"
  },
  "payload": {
    "call_id": "call-uuid",
    "tool_name": "web_search",
    "arguments": {
      "query": "WIA standards specification",
      "num_results": 10
    }
  }
}
```

#### 4.4.2 tool_result

도구가 실행 결과를 반환합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "tool-result-001",
  "timestamp": 1702483200900,
  "type": "tool_result",
  "source": {
    "id": "web_search",
    "type": "tool"
  },
  "target": {
    "id": "agent-researcher-001",
    "type": "agent"
  },
  "payload": {
    "call_id": "call-uuid",
    "status": "success",
    "result": [
      {
        "title": "WIA Standards",
        "url": "https://wia.live",
        "snippet": "Open standards for humanity..."
      }
    ],
    "execution_time_ms": 1500
  }
}
```

#### 4.4.3 Tool Result Status

| Status | Description |
|--------|-------------|
| `success` | 성공적으로 실행됨 |
| `error` | 실행 중 에러 발생 |
| `timeout` | 실행 시간 초과 |
| `cancelled` | 실행 취소됨 |

---

### 4.5 Agent Coordination Messages

#### 4.5.1 handoff

에이전트가 다른 에이전트에게 작업을 전달합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "handoff-001",
  "timestamp": 1702483201000,
  "type": "handoff",
  "source": {
    "id": "agent-researcher-001",
    "type": "agent",
    "name": "Researcher"
  },
  "target": {
    "id": "agent-coder-001",
    "type": "agent",
    "name": "Coder"
  },
  "payload": {
    "handoff_id": "handoff-uuid",
    "reason": "Code implementation required",
    "priority": "normal",
    "context": {
      "task": "Implement the search algorithm",
      "requirements": ["Python 3.10+", "async support"],
      "artifacts": ["research/algorithm.md"],
      "conversation_history": []
    },
    "deadline_ms": 300000
  }
}
```

#### 4.5.2 handoff_ack

핸드오프 수락을 응답합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "handoff-ack-001",
  "timestamp": 1702483201100,
  "type": "handoff_ack",
  "source": {
    "id": "agent-coder-001",
    "type": "agent"
  },
  "target": {
    "id": "agent-researcher-001",
    "type": "agent"
  },
  "payload": {
    "handoff_id": "handoff-uuid",
    "accepted": true,
    "estimated_duration_ms": 60000
  }
}
```

#### 4.5.3 Handoff Priority

| Priority | Description |
|----------|-------------|
| `critical` | 즉시 처리 필요 |
| `high` | 우선 처리 |
| `normal` | 일반 처리 |
| `low` | 낮은 우선순위 |

#### 4.5.4 broadcast

오케스트레이터가 모든 에이전트에게 메시지를 전송합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "broadcast-001",
  "timestamp": 1702483201200,
  "type": "broadcast",
  "source": {
    "id": "agent-orchestrator-001",
    "type": "agent",
    "name": "Orchestrator"
  },
  "payload": {
    "broadcast_type": "status_update",
    "message": "Project deadline extended",
    "recipients": ["agent-researcher-001", "agent-coder-001", "agent-reviewer-001"]
  }
}
```

---

### 4.6 Error Messages

#### 4.6.1 error

에러를 전송합니다.

```json
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "error-001",
  "timestamp": 1702483201300,
  "type": "error",
  "source": {
    "id": "server-001",
    "type": "server"
  },
  "payload": {
    "error_code": 3002,
    "error_name": "TOOL_EXECUTION_FAILED",
    "message": "Tool execution failed: connection timeout",
    "details": {
      "tool_name": "web_search",
      "call_id": "call-uuid"
    },
    "retryable": true,
    "retry_after_ms": 5000
  }
}
```

---

## 5. Streaming Protocol

### 5.1 SSE (Server-Sent Events)

WIA AI는 HTTP/SSE를 기본 스트리밍 프로토콜로 사용합니다.

#### 5.1.1 Request

```http
POST /v1/messages HTTP/1.1
Host: api.wia.live
Content-Type: application/json
Accept: text/event-stream
Authorization: Bearer <token>

{
  "agent_id": "agent-researcher-001",
  "messages": [
    {"role": "user", "content": "Hello"}
  ],
  "stream": true
}
```

#### 5.1.2 Response

```
HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-cache
Connection: keep-alive

event: stream_start
data: {"message_id": "msg-001", "model": "claude-opus-4-5-20251101"}

event: stream_delta
data: {"delta": {"type": "text_delta", "text": "Hello"}}

event: stream_delta
data: {"delta": {"type": "text_delta", "text": ", "}}

event: stream_delta
data: {"delta": {"type": "text_delta", "text": "how can I help you?"}}

event: stream_end
data: {"usage": {"input_tokens": 10, "output_tokens": 8}}

```

#### 5.1.3 SSE Event Types

| Event | Description |
|-------|-------------|
| `stream_start` | 스트림 시작 |
| `stream_delta` | 콘텐츠 조각 |
| `stream_end` | 스트림 종료 |
| `ping` | Keep-alive |
| `error` | 에러 |

### 5.2 WebSocket

실시간 양방향 통신이 필요한 경우 WebSocket을 사용합니다.

#### 5.2.1 Connection

```
GET /v1/ws HTTP/1.1
Host: api.wia.live
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: <key>
Sec-WebSocket-Version: 13
Authorization: Bearer <token>
```

#### 5.2.2 Message Flow

```
Client                                      Server
   │                                           │
   │ ──────── connect ────────────────────────►│
   │                                           │
   │ ◄──────── connect_ack ─────────────────── │
   │                                           │
   │ ──────── message ────────────────────────►│
   │                                           │
   │ ◄──────── stream_start ────────────────── │
   │ ◄──────── stream_delta ────────────────── │
   │ ◄──────── stream_delta ────────────────── │
   │ ◄──────── stream_end ──────────────────── │
   │                                           │
   │ ◄────────── ping ─────────────────────────│
   │ ──────────── pong ───────────────────────►│
   │                                           │
   │ ──────── disconnect ─────────────────────►│
   │                                           │
```

---

## 6. Agent-to-Agent Communication

### 6.1 Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Orchestrator Agent                           │
│                    (agent-orchestrator-001)                       │
│                                                                   │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                    Message Router                            │ │
│  └─────────────────────────────────────────────────────────────┘ │
└────────────────────────┬────────────────────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         │               │               │
         ▼               ▼               ▼
  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
  │  Researcher │ │   Coder     │ │  Reviewer   │
  │    Agent    │ │   Agent     │ │   Agent     │
  │ ┌─────────┐ │ │ ┌─────────┐ │ │ ┌─────────┐ │
  │ │ Message │ │ │ │ Message │ │ │ │ Message │ │
  │ │ Handler │ │ │ │ Handler │ │ │ │ Handler │ │
  │ └─────────┘ │ │ └─────────┘ │ │ └─────────┘ │
  └─────────────┘ └─────────────┘ └─────────────┘
         │               │               │
         └───────► handoff ──────►───────┘
```

### 6.2 Handoff Protocol

#### 6.2.1 Handoff Flow

```
Researcher                Coder
    │                       │
    │ ─────── handoff ─────►│
    │                       │
    │ ◄──── handoff_ack ─── │
    │                       │
    │     (Coder works)     │
    │                       │
    │ ◄────── message ───── │
    │    (task complete)    │
    │                       │
```

#### 6.2.2 Handoff Context

핸드오프 시 전달되는 컨텍스트:

```json
{
  "task": "Implement feature X",
  "requirements": ["requirement 1", "requirement 2"],
  "artifacts": ["path/to/file1.md", "path/to/file2.json"],
  "conversation_history": [
    {"role": "user", "content": "..."},
    {"role": "assistant", "content": "..."}
  ],
  "metadata": {
    "priority": "high",
    "deadline": "2025-01-20T00:00:00Z"
  }
}
```

### 6.3 Discovery

#### 6.3.1 Agent Card

에이전트는 자신의 기능을 Agent Card로 광고합니다:

```json
{
  "agent_id": "agent-researcher-001",
  "name": "Research Assistant",
  "description": "AI agent for conducting research and analysis",
  "version": "1.0.0",
  "capabilities": {
    "tools": ["web_search", "file_read"],
    "handoff": {
      "can_receive": true,
      "can_send": true,
      "preferred_agents": ["agent-coder-001"]
    },
    "streaming": true
  },
  "endpoint": "https://api.wia.live/agents/researcher-001",
  "status": "available"
}
```

---

## 7. Tool Invocation Protocol

### 7.1 Tool Definition (MCP Compatible)

```json
{
  "name": "web_search",
  "description": "Search the web for information",
  "parameters": {
    "type": "object",
    "required": ["query"],
    "properties": {
      "query": {
        "type": "string",
        "description": "Search query"
      },
      "num_results": {
        "type": "integer",
        "default": 10,
        "minimum": 1,
        "maximum": 100
      }
    }
  }
}
```

### 7.2 Tool Call Flow

```
Agent                          Tool Server
   │                               │
   │ ─────── tool_call ───────────►│
   │                               │
   │   (Tool executes)             │
   │                               │
   │ ◄────── tool_result ───────── │
   │                               │
```

### 7.3 Streaming Tool Results

도구가 스트리밍 결과를 반환할 수 있습니다:

```
Agent                          Tool Server
   │                               │
   │ ─────── tool_call ───────────►│
   │                               │
   │ ◄────── tool_stream_start ─── │
   │ ◄────── tool_stream_delta ─── │
   │ ◄────── tool_stream_delta ─── │
   │ ◄────── tool_stream_end ───── │
   │                               │
```

---

## 8. Multi-Agent Coordination

### 8.1 Topology Types

| Type | Description |
|------|-------------|
| **Hierarchical** | 중앙 오케스트레이터가 조율 |
| **Flat** | 에이전트 간 동등한 협력 |
| **Pipeline** | 순차적 처리 파이프라인 |
| **Mesh** | 모든 에이전트가 상호 연결 |

### 8.2 Workflow Execution

#### 8.2.1 Sequential Workflow

```json
{
  "workflow_type": "sequential",
  "steps": [
    {"step": 1, "agent": "researcher", "action": "research"},
    {"step": 2, "agent": "coder", "action": "implement"},
    {"step": 3, "agent": "reviewer", "action": "review"},
    {"step": 4, "agent": "tester", "action": "test"}
  ]
}
```

#### 8.2.2 Parallel Workflow

```json
{
  "workflow_type": "parallel",
  "steps": [
    {
      "step": 1,
      "parallel": [
        {"agent": "researcher-1", "action": "research_topic_a"},
        {"agent": "researcher-2", "action": "research_topic_b"}
      ]
    },
    {"step": 2, "agent": "synthesizer", "action": "combine_results"}
  ]
}
```

### 8.3 State Synchronization

멀티 에이전트 시스템에서 상태 동기화:

```json
{
  "type": "state_sync",
  "source": {"id": "orchestrator-001"},
  "payload": {
    "swarm_id": "swarm-dev-team-001",
    "state": {
      "current_step": 2,
      "completed_tasks": ["task-1", "task-2"],
      "pending_tasks": ["task-3"],
      "shared_artifacts": {
        "research_results": "artifacts/research.json"
      }
    }
  }
}
```

---

## 9. Error Handling

### 9.1 Error Codes

#### 9.1.1 Connection Errors (1xxx)

| Code | Name | Description |
|------|------|-------------|
| 1000 | `CONNECTION_CLOSED` | 정상 종료 |
| 1001 | `CONNECTION_LOST` | 연결 끊김 |
| 1002 | `PROTOCOL_ERROR` | 프로토콜 오류 |
| 1003 | `INVALID_MESSAGE` | 잘못된 메시지 형식 |
| 1004 | `VERSION_MISMATCH` | 프로토콜 버전 불일치 |

#### 9.1.2 Agent Errors (2xxx)

| Code | Name | Description |
|------|------|-------------|
| 2001 | `AGENT_NOT_FOUND` | 에이전트 없음 |
| 2002 | `AGENT_BUSY` | 에이전트 바쁨 |
| 2003 | `AGENT_ERROR` | 에이전트 내부 오류 |
| 2004 | `AGENT_TIMEOUT` | 에이전트 응답 시간 초과 |
| 2005 | `HANDOFF_REJECTED` | 핸드오프 거부됨 |

#### 9.1.3 Tool Errors (3xxx)

| Code | Name | Description |
|------|------|-------------|
| 3001 | `TOOL_NOT_FOUND` | 도구 없음 |
| 3002 | `TOOL_EXECUTION_FAILED` | 도구 실행 실패 |
| 3003 | `TOOL_TIMEOUT` | 도구 시간 초과 |
| 3004 | `TOOL_INVALID_ARGS` | 잘못된 도구 인자 |
| 3005 | `TOOL_PERMISSION_DENIED` | 도구 권한 없음 |

#### 9.1.4 Resource Errors (4xxx)

| Code | Name | Description |
|------|------|-------------|
| 4001 | `RATE_LIMITED` | 속도 제한 |
| 4002 | `TOKEN_LIMIT_EXCEEDED` | 토큰 한도 초과 |
| 4003 | `CONTEXT_OVERFLOW` | 컨텍스트 오버플로우 |
| 4004 | `BUDGET_EXCEEDED` | 예산 초과 |
| 4005 | `QUOTA_EXCEEDED` | 할당량 초과 |

#### 9.1.5 Security Errors (5xxx)

| Code | Name | Description |
|------|------|-------------|
| 5001 | `AUTH_FAILED` | 인증 실패 |
| 5002 | `PERMISSION_DENIED` | 권한 없음 |
| 5003 | `TOKEN_EXPIRED` | 토큰 만료 |
| 5004 | `INVALID_TOKEN` | 잘못된 토큰 |
| 5005 | `SESSION_EXPIRED` | 세션 만료 |

### 9.2 Retry Policy

#### 9.2.1 Retry Configuration

```json
{
  "retry_policy": {
    "max_retries": 3,
    "backoff_strategy": "exponential",
    "initial_delay_ms": 1000,
    "max_delay_ms": 30000,
    "jitter": true,
    "retryable_errors": [1001, 3003, 4001]
  }
}
```

#### 9.2.2 Backoff Strategies

| Strategy | Description |
|----------|-------------|
| `none` | 재시도 없음 |
| `linear` | 선형 증가 (delay * attempt) |
| `exponential` | 지수 증가 (delay * 2^attempt) |
| `fibonacci` | 피보나치 수열 기반 |

#### 9.2.3 Exponential Backoff Example

```
Attempt 1: 1000ms
Attempt 2: 2000ms
Attempt 3: 4000ms
(with jitter: ±10%)
```

---

## 10. Security

### 10.1 Authentication

#### 10.1.1 Bearer Token

```http
Authorization: Bearer <access_token>
```

#### 10.1.2 API Key

```http
X-API-Key: <api_key>
```

### 10.2 Message Signing

메시지 무결성 검증을 위한 서명:

```json
{
  "message": {...},
  "signature": {
    "algorithm": "HMAC-SHA256",
    "value": "<signature>",
    "timestamp": 1702483200000
  }
}
```

### 10.3 Encryption

#### 10.3.1 Transport Layer

- TLS 1.3 필수
- 인증서 검증 필수

#### 10.3.2 Message Level

민감한 페이로드 암호화:

```json
{
  "type": "message",
  "payload": {
    "encrypted": true,
    "algorithm": "AES-256-GCM",
    "ciphertext": "<base64_encoded>",
    "iv": "<initialization_vector>"
  }
}
```

### 10.4 Rate Limiting

```json
{
  "rate_limits": {
    "requests_per_second": 10,
    "requests_per_minute": 100,
    "tokens_per_minute": 100000
  }
}
```

Rate limit 초과 시 응답:

```http
HTTP/1.1 429 Too Many Requests
Retry-After: 60
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 0
X-RateLimit-Reset: 1702483260
```

---

## 11. Examples

### 11.1 Simple Chat

```json
// Request
{
  "protocol": "wia-ai",
  "version": "1.0.0",
  "message_id": "msg-001",
  "type": "message",
  "source": {"id": "user-001", "type": "client"},
  "target": {"id": "agent-assistant-001", "type": "agent"},
  "payload": {
    "role": "user",
    "content": [{"type": "text", "text": "Hello!"}]
  }
}

// Response (stream)
{"type": "stream_start", "payload": {"stream_id": "s-001"}}
{"type": "stream_delta", "payload": {"delta": {"type": "text_delta", "text": "Hello! "}}}
{"type": "stream_delta", "payload": {"delta": {"type": "text_delta", "text": "How can I help?"}}}
{"type": "stream_end", "payload": {"usage": {"output_tokens": 6}}}
```

### 11.2 Tool Use

```json
// Agent calls tool
{
  "type": "tool_call",
  "source": {"id": "agent-001"},
  "target": {"id": "web_search"},
  "payload": {
    "call_id": "call-001",
    "tool_name": "web_search",
    "arguments": {"query": "WIA standards"}
  }
}

// Tool returns result
{
  "type": "tool_result",
  "source": {"id": "web_search"},
  "target": {"id": "agent-001"},
  "payload": {
    "call_id": "call-001",
    "status": "success",
    "result": [{"title": "WIA Standards", "url": "https://wia.live"}]
  }
}
```

### 11.3 Multi-Agent Handoff

```json
// Researcher hands off to Coder
{
  "type": "handoff",
  "source": {"id": "agent-researcher-001", "name": "Researcher"},
  "target": {"id": "agent-coder-001", "name": "Coder"},
  "payload": {
    "handoff_id": "h-001",
    "reason": "Implementation required",
    "context": {
      "task": "Implement search algorithm",
      "requirements": ["Python 3.10+"]
    }
  }
}

// Coder accepts
{
  "type": "handoff_ack",
  "source": {"id": "agent-coder-001"},
  "target": {"id": "agent-researcher-001"},
  "payload": {
    "handoff_id": "h-001",
    "accepted": true
  }
}
```

---

## 12. References

### 12.1 Standards

- [OpenAI API](https://platform.openai.com/docs/api-reference)
- [Anthropic API](https://docs.anthropic.com/en/api)
- [MCP Specification](https://modelcontextprotocol.io)
- [A2A Protocol](https://github.com/google-a2a/A2A)
- [JSON-RPC 2.0](https://www.jsonrpc.org/specification)

### 12.2 Streaming

- [SSE Specification](https://html.spec.whatwg.org/multipage/server-sent-events.html)
- [WebSocket Protocol (RFC 6455)](https://www.rfc-editor.org/rfc/rfc6455)

### 12.3 Security

- [OAuth 2.0 (RFC 6749)](https://www.rfc-editor.org/rfc/rfc6749)
- [JWT (RFC 7519)](https://www.rfc-editor.org/rfc/rfc7519)
- [TLS 1.3 (RFC 8446)](https://www.rfc-editor.org/rfc/rfc8446)

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA AI Working Group

---

弘益人間 - *Benefit All Humanity*
