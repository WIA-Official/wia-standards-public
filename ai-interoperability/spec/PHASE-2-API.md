# WIA AI Interoperability Standard
## Phase 2: API Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2025-12-25

---

## Philosophy

**弘益人間 (홍익인간)** - *Benefit All Humanity*

This API standard enables AI systems to serve humanity better by providing a unified interface for AI capabilities. By standardizing how we interact with AI, we make advanced AI accessible to everyone, regardless of the underlying platform.

---

## Overview

Phase 2 defines the RESTful API specification for AI interoperability. This API enables:
- Sending messages to AI systems
- Receiving AI responses
- Managing conversations
- Discovering AI capabilities
- Streaming real-time responses
- Accessing conversation history

### Design Principles

1. **REST-first**: Follow REST conventions
2. **Stateless**: Each request contains all necessary information
3. **Versioned**: API version in URL path
4. **Documented**: OpenAPI 3.0 compatible
5. **Secure**: Authentication and authorization built-in

---

## Base URL

```
https://api.wia-ai-interop.org/v1
```

All endpoints are prefixed with the version number for future compatibility.

---

## Authentication

### API Key Authentication

```http
Authorization: Bearer wia_sk_abc123xyz789
```

### OAuth 2.0 (Optional)

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## Schema

### Common Response Format

All API responses follow this structure:

```json
{
  "success": true,
  "data": { /* response data */ },
  "metadata": {
    "requestId": "req_abc123",
    "timestamp": "2025-12-25T10:30:00Z",
    "version": "1.0"
  },
  "error": null
}
```

### Error Response Format

```json
{
  "success": false,
  "data": null,
  "metadata": {
    "requestId": "req_abc123",
    "timestamp": "2025-12-25T10:30:00Z",
    "version": "1.0"
  },
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Missing required field: content",
    "details": {
      "field": "content",
      "reason": "required_field_missing"
    }
  }
}
```

---

## Endpoints

### 1. Send Message

Send a message to an AI system.

**Endpoint**: `POST /messages`

**Request Body**:
```json
{
  "message": {
    /* AIMessage object from Phase 1 */
  },
  "options": {
    "stream": false,
    "maxTokens": 4096,
    "temperature": 0.7,
    "stopSequences": []
  }
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "message": {
      /* AIMessage response */
    },
    "usage": {
      "inputTokens": 150,
      "outputTokens": 320,
      "totalTokens": 470
    }
  },
  "metadata": {
    "requestId": "req_001",
    "timestamp": "2025-12-25T10:30:01Z",
    "version": "1.0"
  }
}
```

### 2. Stream Message

Send a message and receive streaming response.

**Endpoint**: `POST /messages/stream`

**Request**: Same as Send Message with `stream: true`

**Response**: Server-Sent Events (SSE)

```
event: message_start
data: {"messageId": "msg_002", "model": "claude-sonnet-4-5"}

event: content_block_start
data: {"index": 0, "contentBlock": {"type": "text"}}

event: content_block_delta
data: {"index": 0, "delta": {"type": "text", "text": "The"}}

event: content_block_delta
data: {"index": 0, "delta": {"type": "text", "text": " capital"}}

event: content_block_delta
data: {"index": 0, "delta": {"type": "text", "text": " of France"}}

event: content_block_delta
data: {"index": 0, "delta": {"type": "text", "text": " is Paris."}}

event: content_block_stop
data: {"index": 0}

event: message_stop
data: {"usage": {"inputTokens": 10, "outputTokens": 15}}
```

### 3. Get Message

Retrieve a specific message by ID.

**Endpoint**: `GET /messages/:messageId`

**Response**:
```json
{
  "success": true,
  "data": {
    "message": {
      /* AIMessage object */
    }
  },
  "metadata": {
    "requestId": "req_002",
    "timestamp": "2025-12-25T10:30:02Z",
    "version": "1.0"
  }
}
```

### 4. List Messages

List messages in a conversation.

**Endpoint**: `GET /conversations/:conversationId/messages`

**Query Parameters**:
- `limit`: Number of messages (default: 50, max: 100)
- `before`: Message ID to paginate before
- `after`: Message ID to paginate after
- `order`: `asc` or `desc` (default: `desc`)

**Response**:
```json
{
  "success": true,
  "data": {
    "messages": [
      /* Array of AIMessage objects */
    ],
    "pagination": {
      "hasMore": true,
      "nextCursor": "msg_xyz",
      "previousCursor": "msg_abc"
    }
  },
  "metadata": {
    "requestId": "req_003",
    "timestamp": "2025-12-25T10:30:03Z",
    "version": "1.0"
  }
}
```

### 5. Create Conversation

Create a new conversation.

**Endpoint**: `POST /conversations`

**Request Body**:
```json
{
  "title": "Q&A about France",
  "metadata": {
    "userId": "user_alice",
    "tags": ["geography", "education"]
  }
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "conversation": {
      "conversationId": "conv_001",
      "title": "Q&A about France",
      "createdAt": "2025-12-25T10:30:00Z",
      "updatedAt": "2025-12-25T10:30:00Z",
      "metadata": {
        "userId": "user_alice",
        "tags": ["geography", "education"]
      }
    }
  },
  "metadata": {
    "requestId": "req_004",
    "timestamp": "2025-12-25T10:30:04Z",
    "version": "1.0"
  }
}
```

### 6. Get Conversation

Retrieve conversation details.

**Endpoint**: `GET /conversations/:conversationId`

**Response**:
```json
{
  "success": true,
  "data": {
    "conversation": {
      "conversationId": "conv_001",
      "title": "Q&A about France",
      "createdAt": "2025-12-25T10:30:00Z",
      "updatedAt": "2025-12-25T10:35:00Z",
      "messageCount": 5,
      "metadata": {
        "userId": "user_alice",
        "tags": ["geography", "education"]
      }
    }
  },
  "metadata": {
    "requestId": "req_005",
    "timestamp": "2025-12-25T10:30:05Z",
    "version": "1.0"
  }
}
```

### 7. List Conversations

List all conversations for a user.

**Endpoint**: `GET /conversations`

**Query Parameters**:
- `limit`: Number of conversations (default: 20, max: 100)
- `offset`: Pagination offset
- `order`: `asc` or `desc` (default: `desc`)

**Response**:
```json
{
  "success": true,
  "data": {
    "conversations": [
      /* Array of conversation objects */
    ],
    "pagination": {
      "total": 42,
      "limit": 20,
      "offset": 0,
      "hasMore": true
    }
  },
  "metadata": {
    "requestId": "req_006",
    "timestamp": "2025-12-25T10:30:06Z",
    "version": "1.0"
  }
}
```

### 8. Delete Conversation

Delete a conversation and all its messages.

**Endpoint**: `DELETE /conversations/:conversationId`

**Response**:
```json
{
  "success": true,
  "data": {
    "deleted": true,
    "conversationId": "conv_001"
  },
  "metadata": {
    "requestId": "req_007",
    "timestamp": "2025-12-25T10:30:07Z",
    "version": "1.0"
  }
}
```

### 9. Get Model Capabilities

Discover what an AI model can do.

**Endpoint**: `GET /models/:modelId/capabilities`

**Response**:
```json
{
  "success": true,
  "data": {
    "model": {
      "modelId": "claude-sonnet-4-5",
      "provider": "anthropic",
      "version": "20250929",
      "capabilities": {
        "modalities": ["text", "image", "code"],
        "maxTokens": 200000,
        "supportedLanguages": ["en", "es", "fr", "de", "ja", "ko", "zh"],
        "features": [
          "reasoning",
          "tool_use",
          "vision",
          "code_generation",
          "multilingual"
        ],
        "streaming": true,
        "contextWindow": 200000
      },
      "pricing": {
        "inputTokenPrice": 0.003,
        "outputTokenPrice": 0.015,
        "currency": "USD",
        "unit": "per_1k_tokens"
      }
    }
  },
  "metadata": {
    "requestId": "req_008",
    "timestamp": "2025-12-25T10:30:08Z",
    "version": "1.0"
  }
}
```

### 10. List Available Models

List all available AI models.

**Endpoint**: `GET /models`

**Response**:
```json
{
  "success": true,
  "data": {
    "models": [
      {
        "modelId": "claude-sonnet-4-5",
        "provider": "anthropic",
        "name": "Claude Sonnet 4.5",
        "description": "Balanced model for most tasks"
      },
      {
        "modelId": "gpt-4-turbo",
        "provider": "openai",
        "name": "GPT-4 Turbo",
        "description": "Fast and capable model"
      }
    ]
  },
  "metadata": {
    "requestId": "req_009",
    "timestamp": "2025-12-25T10:30:09Z",
    "version": "1.0"
  }
}
```

---

## Fields

### Message Options

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `stream` | boolean | No | false | Enable streaming response |
| `maxTokens` | number | No | 4096 | Maximum tokens to generate |
| `temperature` | number | No | 0.7 | Sampling temperature (0-1) |
| `stopSequences` | array | No | [] | Sequences to stop generation |
| `topP` | number | No | 1.0 | Nucleus sampling parameter |
| `topK` | number | No | null | Top-K sampling parameter |

---

## Validation

### Request Validation

1. **Authentication**: All requests must include valid API key
2. **Content-Type**: Must be `application/json` for POST/PUT requests
3. **Message Format**: Must conform to Phase 1 AIMessage schema
4. **Rate Limits**:
   - Free tier: 60 requests/minute
   - Pro tier: 600 requests/minute
   - Enterprise: Custom limits

### Response Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created successfully |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Service temporarily unavailable |

---

## Examples

### Example 1: Simple Message Exchange

**Request**:
```bash
curl -X POST https://api.wia-ai-interop.org/v1/messages \
  -H "Authorization: Bearer wia_sk_abc123" \
  -H "Content-Type: application/json" \
  -d '{
    "message": {
      "version": "1.0",
      "messageId": "msg_001",
      "timestamp": "2025-12-25T10:30:00Z",
      "sender": {
        "type": "human",
        "id": "user_alice"
      },
      "content": {
        "type": "text",
        "parts": [{"type": "text", "data": "Hello, AI!"}]
      }
    },
    "options": {
      "maxTokens": 1000
    }
  }'
```

**Response**:
```json
{
  "success": true,
  "data": {
    "message": {
      "version": "1.0",
      "messageId": "msg_002",
      "timestamp": "2025-12-25T10:30:01Z",
      "sender": {
        "type": "ai",
        "id": "claude_assistant"
      },
      "content": {
        "type": "text",
        "parts": [
          {
            "type": "text",
            "data": "Hello! How can I help you today?"
          }
        ]
      },
      "model": {
        "provider": "anthropic",
        "modelId": "claude-sonnet-4-5"
      }
    },
    "usage": {
      "inputTokens": 10,
      "outputTokens": 12,
      "totalTokens": 22
    }
  },
  "metadata": {
    "requestId": "req_xyz",
    "timestamp": "2025-12-25T10:30:01Z",
    "version": "1.0"
  }
}
```

### Example 2: Streaming Response

**Request**:
```bash
curl -X POST https://api.wia-ai-interop.org/v1/messages/stream \
  -H "Authorization: Bearer wia_sk_abc123" \
  -H "Content-Type: application/json" \
  -H "Accept: text/event-stream" \
  -d '{
    "message": {
      "version": "1.0",
      "messageId": "msg_003",
      "timestamp": "2025-12-25T10:31:00Z",
      "sender": {
        "type": "human",
        "id": "user_alice"
      },
      "content": {
        "type": "text",
        "parts": [{"type": "text", "data": "Write a haiku about AI"}]
      }
    },
    "options": {
      "stream": true
    }
  }'
```

**Response** (Server-Sent Events):
```
event: message_start
data: {"messageId": "msg_004"}

event: content_block_delta
data: {"delta": {"text": "Silicon minds awake"}}

event: content_block_delta
data: {"delta": {"text": "\nLearning from human wisdom"}}

event: content_block_delta
data: {"delta": {"text": "\nServing all with care"}}

event: message_stop
data: {"usage": {"inputTokens": 15, "outputTokens": 20}}
```

### Example 3: Creating and Using a Conversation

**Step 1: Create Conversation**
```bash
curl -X POST https://api.wia-ai-interop.org/v1/conversations \
  -H "Authorization: Bearer wia_sk_abc123" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Poetry Discussion",
    "metadata": {"topic": "creative_writing"}
  }'
```

**Step 2: Send Message in Conversation**
```bash
curl -X POST https://api.wia-ai-interop.org/v1/messages \
  -H "Authorization: Bearer wia_sk_abc123" \
  -H "Content-Type: application/json" \
  -d '{
    "message": {
      "version": "1.0",
      "messageId": "msg_005",
      "timestamp": "2025-12-25T10:32:00Z",
      "sender": {"type": "human", "id": "user_alice"},
      "content": {
        "type": "text",
        "parts": [{"type": "text", "data": "Tell me about haikus"}]
      },
      "context": {
        "conversationId": "conv_poetry_001"
      }
    }
  }'
```

---

## Implementation Notes

1. **Idempotency**: Use `Idempotency-Key` header for safe retries
2. **Compression**: Support gzip compression for large payloads
3. **CORS**: Enable CORS for browser-based applications
4. **Webhooks**: Optional webhook support for async processing
5. **SDKs**: Official SDKs available for TypeScript, Python, Go, Rust

---

## Next Steps

Phase 2 establishes the API. Continue to:
- **Phase 3**: Protocol for real-time, bidirectional communication
- **Phase 4**: Integration patterns with existing platforms

---

© 2025 SmileStory Inc. / WIA
**弘익人間 (홍익인간)** · Benefit All Humanity
