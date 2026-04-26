# WIA-AI-016 Phase 1: Data Format Specification

> 弘益人間 (홍익인간) · Benefit All Humanity

## Overview

Phase 1 defines the standard data formats, message structures, and knowledge representation schemes for multi-agent systems compliant with WIA-AI-016.

## Agent Message Format

### FIPA-ACL Message Structure

```json
{
  "performative": "request | inform | query-if | propose | accept-proposal | reject-proposal | agree | refuse | failure | cfp",
  "sender": {
    "name": "string",
    "address": "uri"
  },
  "receiver": [
    {
      "name": "string",
      "address": "uri"
    }
  ],
  "content": "any",
  "language": "string",
  "ontology": "string",
  "protocol": "string",
  "conversationId": "string",
  "replyWith": "string",
  "inReplyTo": "string",
  "replyBy": "ISO8601 datetime"
}
```

### Performatives

| Performative | Purpose | Example |
|--------------|---------|---------|
| INFORM | Share information | "Temperature is 25°C" |
| REQUEST | Ask to perform action | "Please send status" |
| QUERY-IF | Ask if proposition true | "Is task completed?" |
| PROPOSE | Make proposal | "I can deliver for $50" |
| ACCEPT-PROPOSAL | Accept proposal | "I accept your offer" |
| REJECT-PROPOSAL | Reject proposal | "Price too high" |
| AGREE | Agree to perform | "I will complete by 5pm" |
| REFUSE | Refuse to perform | "Cannot complete" |
| FAILURE | Report failure | "Task failed" |
| CFP | Call for proposals | "Need quotes" |

### Agent Identifier Format

```json
{
  "name": "agent-001",
  "address": "wia://domain.com/agents/agent-001",
  "resolvers": [
    "wia://resolver.domain.com"
  ],
  "userDefinedProperties": {}
}
```

## Knowledge Representation

### Ontology Format

```json
{
  "name": "ecommerce-ontology",
  "version": "1.0.0",
  "namespace": "wia:ecommerce",
  "concepts": {
    "Product": {
      "properties": ["name", "price", "category"],
      "relationships": ["hasReview", "inCategory"]
    },
    "Order": {
      "properties": ["orderId", "totalAmount", "status"],
      "relationships": ["containsProduct", "placedBy"]
    }
  },
  "relationships": {
    "hasReview": {
      "domain": "Product",
      "range": "Review"
    },
    "inCategory": {
      "domain": "Product",
      "range": "Category"
    }
  }
}
```

### Belief Format

```json
{
  "agentId": "agent-001",
  "beliefs": [
    {
      "predicate": "temperature",
      "arguments": ["room-101", 25],
      "confidence": 0.95,
      "timestamp": "2025-12-25T10:00:00Z",
      "source": "sensor-042"
    }
  ]
}
```

## Task Format

```json
{
  "taskId": "task-001",
  "type": "measurement",
  "priority": 5,
  "requiredSkills": ["temperature-sensing", "data-logging"],
  "requiredAgents": 1,
  "dependencies": ["task-000"],
  "deadline": "2025-12-25T12:00:00Z",
  "parameters": {
    "location": "room-101",
    "frequency": "1/min"
  },
  "constraints": {
    "maxCost": 100,
    "minQuality": 0.9
  }
}
```

## Capability Description

```json
{
  "agentId": "agent-001",
  "capabilities": [
    {
      "name": "temperature-sensing",
      "type": "sensing",
      "range": [-40, 120],
      "accuracy": 0.5,
      "units": "celsius"
    },
    {
      "name": "data-logging",
      "type": "storage",
      "capacity": 10000,
      "retention": "30d"
    }
  ],
  "constraints": {
    "maxConcurrentTasks": 5,
    "operatingHours": "24/7"
  }
}
```

## Coalition Structure

```json
{
  "coalitionId": "coalition-001",
  "members": ["agent-001", "agent-002", "agent-003"],
  "leader": "agent-001",
  "goal": "complete-project-alpha",
  "formation": "2025-12-25T10:00:00Z",
  "expiration": "2025-12-26T10:00:00Z",
  "roles": {
    "agent-001": "coordinator",
    "agent-002": "worker",
    "agent-003": "worker"
  }
}
```

## Performance Metrics

```json
{
  "agentId": "agent-001",
  "period": {
    "start": "2025-12-25T00:00:00Z",
    "end": "2025-12-25T23:59:59Z"
  },
  "metrics": {
    "tasksCompleted": 150,
    "tasksFailedWIA": 5,
    "avgResponseTime": 250,
    "avgQuality": 0.95,
    "utilization": 0.75,
    "messagesReceived": 500,
    "messagesSent": 450
  }
}
```

## Event Format

```json
{
  "eventId": "event-001",
  "type": "task-completed | agent-joined | agent-failed | message-sent",
  "timestamp": "2025-12-25T10:00:00Z",
  "source": "agent-001",
  "data": {
    "taskId": "task-001",
    "result": "success",
    "duration": 5000
  }
}
```

## Reputation Data

```json
{
  "agentId": "agent-001",
  "reputation": {
    "overall": 0.85,
    "reliability": 0.90,
    "quality": 0.85,
    "responsiveness": 0.80,
    "interactionCount": 1000,
    "lastUpdated": "2025-12-25T10:00:00Z"
  },
  "reviews": [
    {
      "reviewer": "agent-002",
      "rating": 0.9,
      "comment": "Excellent performance",
      "timestamp": "2025-12-25T09:00:00Z"
    }
  ]
}
```

## Data Validation Rules

### Message Validation

1. **Required Fields**: `performative`, `sender`, `receiver` must be present
2. **Performative Values**: Must be one of the 10 standard FIPA-ACL performatives
3. **Agent Identifiers**: Must include `name` and `address`
4. **Timestamps**: Must be ISO8601 format with timezone
5. **Content**: Must be valid JSON or specified content language

### Size Limits

- Maximum message size: 1 MB
- Maximum array length: 10,000 items
- Maximum string length: 64 KB
- Maximum nesting depth: 10 levels

## Encoding

- **Character Encoding**: UTF-8
- **JSON Format**: RFC 8259 compliant
- **Date/Time**: ISO 8601
- **URIs**: RFC 3986

## Versioning

Data format version included in all messages:

```json
{
  "formatVersion": "1.0.0",
  "..." : "..."
}
```

## Agent Card (Discovery Manifest)

Every WIA-AI-016 agent MUST publish an Agent Card at a stable URL so that peers can discover its capabilities without prior coupling. The Agent Card is JSON, served over HTTPS with `Content-Type: application/json`, and SHOULD be cacheable per RFC 9111.

```json
{
  "schemaVersion": "wia.ai-016.agent-card/1",
  "agentId": "agent-001",
  "displayName": "Temperature Sensing Agent",
  "description": "Reports calibrated temperature for a single zone.",
  "url": "https://api.example.com/v1/agents/agent-001",
  "discoveryUrl": "https://api.example.com/.well-known/wia-agent-card",
  "provider": {
    "name": "Example Smart-Home Co.",
    "url": "https://example.com",
    "contact": "ops@example.com"
  },
  "capabilities": ["temperature-sensing", "data-logging"],
  "supportedProtocols": [
    "wia-ai-016/fipa-acl",
    "wia-ai-016/jsonrpc-tools",
    "wia-ai-016/a2a-bridge"
  ],
  "supportedContentLanguages": ["application/json", "application/ld+json"],
  "auth": {
    "schemes": ["Bearer", "mTLS"],
    "openIdConfig": "https://example.com/.well-known/openid-configuration"
  },
  "skills": [
    {
      "id": "measure-temperature",
      "input": "wia:ecommerce/MeasurementRequest",
      "output": "wia:ecommerce/MeasurementResult",
      "modalities": ["text", "structured"]
    }
  ],
  "interoperability": {
    "a2a": true,
    "mcp": true,
    "fipa": true
  }
}
```

The Agent Card MUST be served from `/.well-known/wia-agent-card` (analogous to RFC 8615 well-known URIs) when the agent has its own host. Aggregators MAY publish multi-agent indexes at `/.well-known/wia-agent-index`.

## Tool Descriptor (MCP-compatible)

Agents that expose discrete actions to LLM-driven peers MUST describe each action with a Tool Descriptor whose shape mirrors the Model Context Protocol `tools/list` reply. This lets WIA agents register directly with MCP-aware hosts without translation.

```json
{
  "name": "measure-temperature",
  "title": "Measure room temperature",
  "description": "Returns the current temperature for a named zone in degrees Celsius.",
  "inputSchema": {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "type": "object",
    "required": ["zone"],
    "properties": {
      "zone": {"type": "string", "minLength": 1, "maxLength": 64},
      "samplePeriodSeconds": {"type": "integer", "minimum": 1, "maximum": 3600}
    },
    "additionalProperties": false
  },
  "outputSchema": {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "type": "object",
    "required": ["zone", "celsius", "measuredAt"],
    "properties": {
      "zone": {"type": "string"},
      "celsius": {"type": "number"},
      "measuredAt": {"type": "string", "format": "date-time"}
    }
  },
  "annotations": {
    "readOnlyHint": true,
    "destructiveHint": false,
    "idempotentHint": true
  }
}
```

Tool input/output schemas MUST conform to JSON Schema draft 2020-12 (defined by JSON Schema Specification draft 2020-12, IETF document `draft-bhutton-json-schema-01`). Schemas SHOULD set `additionalProperties: false` on the top level so that incompatible peers fail loudly.

## Content Language Identifiers

The `language` field on a FIPA-ACL message indicates how the receiver MUST parse `content`. WIA-AI-016 agents MUST recognise the following identifiers:

| Identifier | Format | Notes |
|------------|--------|-------|
| `application/json` | RFC 8259 JSON | Default for typed content |
| `application/ld+json` | W3C JSON-LD 1.1 | Linked-data ontologies |
| `text/turtle` | W3C Turtle 1.1 | RDF graphs |
| `application/cbor` | RFC 8949 CBOR | Compact binary encoding |
| `application/wia-acl+json` | WIA-AI-016 §1 | Native FIPA-ACL JSON |

Agents MAY advertise additional identifiers in their Agent Card. Receivers that cannot parse the announced language MUST reply with the FIPA-ACL `not-understood` performative (see §3.4).

## Capability Vocabulary

Capability names use the URI form `wia:cap/<domain>/<name>` to avoid collisions across providers. The reserved domain `wia:cap/core/*` is curated by WIA; vendor extensions MUST use `wia:cap/<vendor>/*`. Aliases without the prefix (e.g. `temperature-sensing`) are permitted in human-facing surfaces but normalize to `wia:cap/core/temperature-sensing` during discovery.

```json
{
  "capability": "wia:cap/core/temperature-sensing",
  "version": "1.2.0",
  "compatibility": ["wia:cap/core/temperature-sensing@^1"],
  "deprecated": false,
  "documentationUrl": "https://standards.wia.example/multi-agent-system/capabilities/temperature-sensing"
}
```

## Lifecycle States

Every agent MUST report one of the following lifecycle states. Transitions MUST emit an event of type `agent-state-changed` (see §Event Format).

| State | Description | Permitted next states |
|-------|-------------|-----------------------|
| `provisioning` | Bootstrapping credentials and config | `idle`, `failed` |
| `idle` | Healthy, no active tasks | `busy`, `draining`, `offline` |
| `busy` | Executing one or more tasks | `idle`, `degraded`, `failed` |
| `degraded` | Operating with reduced capability | `idle`, `failed`, `offline` |
| `draining` | Refusing new work, finishing current | `offline` |
| `offline` | Cleanly stopped | `provisioning` |
| `failed` | Stopped due to error | `provisioning`, `offline` |

## Schema Registry Reference

JSON Schemas for every type defined in this Phase are published at:

```
https://standards.wia.example/multi-agent-system/schemas/v1/
  ├── agent-card.schema.json
  ├── tool-descriptor.schema.json
  ├── fipa-acl-message.schema.json
  ├── task.schema.json
  ├── capability.schema.json
  ├── event.schema.json
  └── reputation.schema.json
```

Schemas MUST validate cleanly with the Ajv 2020 validator or any JSON Schema 2020-12-conformant tool. Implementations SHOULD ship the schemas inside their distribution to allow offline validation.

## Normative References

- IETF RFC 8259 — The JavaScript Object Notation (JSON) Data Interchange Format
- IETF RFC 8615 — Well-Known URIs
- IETF RFC 9111 — HTTP Caching
- IETF RFC 8949 — Concise Binary Object Representation (CBOR)
- IETF RFC 5234 — Augmented BNF for Syntax Specifications: ABNF
- ISO 8601:2019 — Date and time representations
- W3C JSON-LD 1.1 — A JSON-based Serialization for Linked Data
- W3C RDF 1.1 Turtle — Terse RDF Triple Language

---

**WIA-AI-016 Phase 1 Specification v1.0**
© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 (홍익인간) · Benefit All Humanity
