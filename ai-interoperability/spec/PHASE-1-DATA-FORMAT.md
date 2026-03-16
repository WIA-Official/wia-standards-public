# WIA AI Interoperability Standard
## Phase 1: Data Format Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2025-12-25

---

## Philosophy

**弘益人間 (홍익인간)** - *Benefit All Humanity*

This standard aims to create a universal data format that enables AI systems to communicate seamlessly, breaking down silos and fostering collaboration across different AI platforms for the benefit of all humanity.

---

## Overview

Phase 1 defines the foundational data format for AI interoperability. This format standardizes how AI systems exchange information about:
- User requests and intents
- AI responses and outputs
- Model capabilities and metadata
- Context and conversation history
- Multi-modal data (text, images, audio, video)

### Goals

1. **Universal Compatibility**: Work across all AI platforms
2. **Extensibility**: Support future AI capabilities
3. **Human-Readable**: Easy to understand and debug
4. **Efficient**: Minimal overhead for transmission
5. **Type-Safe**: Strong typing for reliability

---

## Schema

### AIMessage

The core data structure for AI communication.

```json
{
  "version": "1.0",
  "messageId": "msg_abc123",
  "timestamp": "2025-12-25T10:30:00Z",
  "sender": {
    "type": "human" | "ai" | "system",
    "id": "user_123",
    "name": "John Doe",
    "metadata": {}
  },
  "content": {
    "type": "text" | "multimodal",
    "parts": [
      {
        "type": "text" | "image" | "audio" | "video" | "code" | "tool_use" | "tool_result",
        "data": "...",
        "metadata": {}
      }
    ]
  },
  "context": {
    "conversationId": "conv_xyz789",
    "threadId": "thread_456",
    "previousMessageIds": ["msg_abc122"],
    "metadata": {}
  },
  "model": {
    "provider": "anthropic" | "openai" | "google" | "custom",
    "modelId": "claude-sonnet-4-5",
    "version": "20250929",
    "capabilities": ["text", "vision", "code", "reasoning"]
  },
  "intent": {
    "primary": "answer_question" | "generate_code" | "analyze_data" | "creative_writing",
    "confidence": 0.95,
    "parameters": {}
  },
  "metadata": {
    "language": "en",
    "priority": "normal" | "high" | "low",
    "tags": [],
    "custom": {}
  }
}
```

---

## Fields

### Root Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `version` | string | Yes | WIA AI Interoperability version |
| `messageId` | string | Yes | Unique message identifier (UUID or custom) |
| `timestamp` | string | Yes | ISO 8601 timestamp |
| `sender` | object | Yes | Information about message sender |
| `content` | object | Yes | Message content |
| `context` | object | No | Conversation context |
| `model` | object | No | AI model information (for AI senders) |
| `intent` | object | No | Interpreted user intent |
| `metadata` | object | No | Additional metadata |

### Sender Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | enum | Yes | Sender type: human, ai, system |
| `id` | string | Yes | Unique sender identifier |
| `name` | string | No | Display name |
| `metadata` | object | No | Custom sender metadata |

### Content Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | enum | Yes | Content type: text or multimodal |
| `parts` | array | Yes | Content parts (supports multiple modalities) |

### Content Part

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | enum | Yes | Part type: text, image, audio, video, code, tool_use, tool_result |
| `data` | any | Yes | Actual content (string, base64, URL, object) |
| `metadata` | object | No | Part-specific metadata (format, encoding, etc.) |

### Context Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `conversationId` | string | No | Unique conversation identifier |
| `threadId` | string | No | Thread identifier for branching conversations |
| `previousMessageIds` | array | No | IDs of previous messages in sequence |
| `metadata` | object | No | Custom context metadata |

### Model Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `provider` | string | Yes | AI provider name |
| `modelId` | string | Yes | Specific model identifier |
| `version` | string | No | Model version |
| `capabilities` | array | No | List of model capabilities |

### Intent Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `primary` | string | Yes | Primary intent classification |
| `confidence` | number | No | Confidence score (0-1) |
| `parameters` | object | No | Intent-specific parameters |

---

## Validation

### Required Validation Rules

1. **version**: Must match `^\d+\.\d+$` pattern
2. **messageId**: Must be unique within conversation
3. **timestamp**: Must be valid ISO 8601 format
4. **sender.type**: Must be one of: human, ai, system
5. **content.type**: Must be one of: text, multimodal
6. **content.parts**: Must have at least one part
7. **intent.confidence**: If present, must be between 0 and 1

### Optional Best Practices

1. Use UUIDs for messageId for global uniqueness
2. Include conversationId for multi-turn conversations
3. Always include model information for AI responses
4. Add language metadata for i18n support

---

## Examples

### Example 1: Simple Text Message (Human)

```json
{
  "version": "1.0",
  "messageId": "msg_001",
  "timestamp": "2025-12-25T10:30:00Z",
  "sender": {
    "type": "human",
    "id": "user_alice",
    "name": "Alice"
  },
  "content": {
    "type": "text",
    "parts": [
      {
        "type": "text",
        "data": "What is the capital of France?"
      }
    ]
  },
  "context": {
    "conversationId": "conv_001"
  },
  "metadata": {
    "language": "en"
  }
}
```

### Example 2: AI Response with Model Info

```json
{
  "version": "1.0",
  "messageId": "msg_002",
  "timestamp": "2025-12-25T10:30:01Z",
  "sender": {
    "type": "ai",
    "id": "claude_assistant",
    "name": "Claude"
  },
  "content": {
    "type": "text",
    "parts": [
      {
        "type": "text",
        "data": "The capital of France is Paris. It has been the capital since 987 AD and is located in the north-central part of the country."
      }
    ]
  },
  "context": {
    "conversationId": "conv_001",
    "previousMessageIds": ["msg_001"]
  },
  "model": {
    "provider": "anthropic",
    "modelId": "claude-sonnet-4-5",
    "version": "20250929",
    "capabilities": ["text", "reasoning", "multilingual"]
  },
  "intent": {
    "primary": "answer_question",
    "confidence": 0.98,
    "parameters": {
      "topic": "geography",
      "questionType": "factual"
    }
  },
  "metadata": {
    "language": "en",
    "processingTime": 234
  }
}
```

### Example 3: Multimodal Message (Image + Text)

```json
{
  "version": "1.0",
  "messageId": "msg_003",
  "timestamp": "2025-12-25T10:35:00Z",
  "sender": {
    "type": "human",
    "id": "user_alice",
    "name": "Alice"
  },
  "content": {
    "type": "multimodal",
    "parts": [
      {
        "type": "text",
        "data": "What's in this image?"
      },
      {
        "type": "image",
        "data": "data:image/jpeg;base64,/9j/4AAQSkZJRg...",
        "metadata": {
          "format": "jpeg",
          "width": 1024,
          "height": 768
        }
      }
    ]
  },
  "context": {
    "conversationId": "conv_001",
    "previousMessageIds": ["msg_001", "msg_002"]
  },
  "metadata": {
    "language": "en"
  }
}
```

### Example 4: Code Generation Request

```json
{
  "version": "1.0",
  "messageId": "msg_004",
  "timestamp": "2025-12-25T11:00:00Z",
  "sender": {
    "type": "human",
    "id": "user_bob",
    "name": "Bob"
  },
  "content": {
    "type": "text",
    "parts": [
      {
        "type": "text",
        "data": "Write a Python function to calculate fibonacci numbers"
      }
    ]
  },
  "context": {
    "conversationId": "conv_002"
  },
  "intent": {
    "primary": "generate_code",
    "confidence": 0.92,
    "parameters": {
      "language": "python",
      "task": "algorithm"
    }
  },
  "metadata": {
    "language": "en"
  }
}
```

### Example 5: Tool Use and Result

```json
{
  "version": "1.0",
  "messageId": "msg_005",
  "timestamp": "2025-12-25T11:00:01Z",
  "sender": {
    "type": "ai",
    "id": "ai_assistant",
    "name": "Assistant"
  },
  "content": {
    "type": "multimodal",
    "parts": [
      {
        "type": "tool_use",
        "data": {
          "toolName": "code_interpreter",
          "toolId": "tool_001",
          "input": {
            "language": "python",
            "code": "def fibonacci(n):\n    if n <= 1:\n        return n\n    return fibonacci(n-1) + fibonacci(n-2)"
          }
        }
      },
      {
        "type": "code",
        "data": "def fibonacci(n):\n    if n <= 1:\n        return n\n    return fibonacci(n-1) + fibonacci(n-2)",
        "metadata": {
          "language": "python",
          "tested": true
        }
      },
      {
        "type": "text",
        "data": "Here's a recursive implementation of the Fibonacci function in Python. Note that for large values of n, you might want to use dynamic programming for better performance."
      }
    ]
  },
  "context": {
    "conversationId": "conv_002",
    "previousMessageIds": ["msg_004"]
  },
  "model": {
    "provider": "custom",
    "modelId": "ai-assistant-v1",
    "capabilities": ["text", "code", "tool_use"]
  },
  "metadata": {
    "language": "en"
  }
}
```

---

## Implementation Notes

1. **Backward Compatibility**: Parsers should ignore unknown fields
2. **Forward Compatibility**: New fields can be added to metadata sections
3. **Encoding**: All text must be UTF-8
4. **Size Limits**: Implementations should support messages up to 10MB
5. **Binary Data**: Use base64 encoding for binary content in JSON

---

## Next Steps

Phase 1 establishes the data format. Continue to:
- **Phase 2**: API specification for exchanging these messages
- **Phase 3**: Protocol for real-time communication
- **Phase 4**: Integration patterns with existing AI platforms

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간)** · Benefit All Humanity
