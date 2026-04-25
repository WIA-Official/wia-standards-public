# WIA-LLM-INTEROP

**AI들의 연동 표준 - All AIs Collaborate as One**

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA-LLM-INTEROP enables AI/LLM systems to communicate, collaborate, and federate with each other through standardized protocols.

```
수많은 LLM들 (GPT, Claude, Gemini, Llama, 특화 AI들...)
                    ↓
            서로 말이 안 통함
                    ↓
               표준이 필요
                    ↓
            WIA-LLM-INTEROP
```

---

## Components

| Component | Purpose | Description |
|-----------|---------|-------------|
| **WIA-LLM-CAPABILITY** | AI declares abilities | "나는 뭘 할 수 있어" |
| **WIA-LLM-MESSAGE** | AI-to-AI messaging | "이렇게 말해" |
| **WIA-LLM-FEDERATION** | Multi-AI collaboration | "함께 일하면 이렇게" |

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data structures for Capability, Message, Federation |
| 2 | [PHASE-2-ALGORITHMS.md](spec/PHASE-2-ALGORITHMS.md) | Matching, routing, consensus algorithms |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | REST, WebSocket, gRPC protocols |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | Provider integrations, SDKs, deployment |

---

## Quick Start

### Rust API

```bash
cd api/rust

# Build
cargo build --release

# Run server
cargo run

# Run tests
cargo test
```

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/version` | API version info |
| GET | `/.well-known/wia-llm-interop` | Service discovery |
| GET | `/wia/llm-interop/v1/capability` | List capabilities |
| POST | `/wia/llm-interop/v1/capability` | Register capability |
| POST | `/wia/llm-interop/v1/capability/query` | Query capabilities |
| POST | `/wia/llm-interop/v1/message` | Send message |
| POST | `/wia/llm-interop/v1/federation` | Create federation |
| POST | `/wia/llm-interop/v1/federation/:id/join` | Join federation |
| POST | `/wia/llm-interop/v1/federation/:id/task` | Submit task |
| POST | `/wia/llm-interop/v1/federation/:id/consensus` | Start consensus |
| WS | `/wia/llm-interop/v1/ws` | Real-time WebSocket |

---

## Capability Levels

| Level | Name | Description |
|-------|------|-------------|
| 1 | Basic | Simple Q&A, text generation |
| 2 | Reasoning | Multi-step reasoning, analysis |
| 3 | Specialist | Domain expert, specialized tasks |
| 4 | Advanced | Tool use, multi-modal, agentic behavior |

---

## Example: Query Capabilities

```bash
curl -X POST http://localhost:8080/wia/llm-interop/v1/capability/query \
  -H "Content-Type: application/json" \
  -d '{
    "required_domains": ["medical"],
    "min_level": 3,
    "required_languages": ["ko"]
  }'

# Response:
{
  "success": true,
  "data": [
    {
      "model_id": "did:wia:llm:provider:medical-ai",
      "score": 0.92,
      "capability_uri": "https://provider.com/capability"
    }
  ]
}
```

---

## Example: Create Federation

```bash
# Create federation
curl -X POST http://localhost:8080/wia/llm-interop/v1/federation \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Medical-Legal Analysis",
    "topology": "star"
  }'

# Submit task
curl -X POST http://localhost:8080/wia/llm-interop/v1/federation/{id}/task \
  -H "Content-Type: application/json" \
  -d '{
    "query": "환자 A의 치료비 보험 청구 가능 여부 분석"
  }'
```

---

## Federation Topologies

| Topology | Description |
|----------|-------------|
| Star | Central orchestrator |
| Mesh | Peer-to-peer |
| Hierarchical | Multi-level |
| Ring | Sequential processing |

---

## Why Now?

| Area | Current Status | WIA |
|------|----------------|-----|
| Quantum Computing | NIST in progress | **Spec + Implementation Done** |
| LLM Federation | **Nobody doing this** | **Standard Ready** |

This is **empty land** - WIA creates the standard first → de facto standard.

---

## Project Structure

```
llm-interop/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-ALGORITHMS.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/rust/
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs, types.rs, core.rs, server.rs, main.rs
├── prompts/
│   ├── WIA-LLM-CAPABILITY-PROMPT.md
│   ├── WIA-LLM-MESSAGE-PROMPT.md
│   └── WIA-LLM-FEDERATION-PROMPT.md
└── README.md
```

---

**Document ID**: WIA-LLM-INTEROP
**Version**: 1.0.0
**Date**: 2025-12-16
**Philosophy**: 언어, 국경, 회사의 벽을 넘어 모든 AI가 협업합니다.
