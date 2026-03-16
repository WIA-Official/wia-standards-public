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

---

**WIA-AI-016 Phase 1 Specification v1.0**
© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 (홍익인간) · Benefit All Humanity
