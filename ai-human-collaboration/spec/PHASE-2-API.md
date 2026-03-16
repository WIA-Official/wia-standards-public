# WIA-AI-015 Phase 2: API Specification

> **Version:** 1.0.0
> **Status:** Stable
> **Last Updated:** 2025-12-25

## Overview

This document defines the RESTful API for AI-human collaboration systems. The API enables creation, management, and monitoring of collaborative workflows.

## Base URL

```
https://api.example.com/v1/collaboration
```

## Authentication

All requests require authentication using API keys:

```http
Authorization: Bearer <your-api-key>
```

## Core Endpoints

### Sessions

#### Create Session

```http
POST /sessions
Content-Type: application/json

{
  "configuration": {
    "confidence_threshold": 0.75,
    "escalation_strategy": "uncertainty",
    "feedback_enabled": true
  },
  "participants": {
    "ai_agents": [...],
    "human_agents": [...]
  }
}

Response: 201 Created
{
  "session_id": "uuid",
  "status": "active",
  "created_at": "2025-12-25T10:00:00Z"
}
```

#### Get Session

```http
GET /sessions/{session_id}

Response: 200 OK
{
  "session_id": "uuid",
  "status": "active",
  "configuration": {...},
  "metrics": {...}
}
```

### Tasks

#### Submit Task

```http
POST /sessions/{session_id}/tasks
Content-Type: application/json

{
  "input_data": {
    "type": "classification",
    "content": {...},
    "features": {...}
  },
  "priority": "high",
  "deadline": "2025-12-25T18:00:00Z"
}

Response: 202 Accepted
{
  "task_id": "uuid",
  "status": "pending",
  "estimated_completion": "2025-12-25T10:05:00Z"
}
```

#### Get Task Status

```http
GET /tasks/{task_id}

Response: 200 OK
{
  "task_id": "uuid",
  "status": "completed",
  "result": {...},
  "processing_history": [...]
}
```

### Predictions

#### Submit Prediction

```http
POST /tasks/{task_id}/predictions
Content-Type: application/json

{
  "agent_id": "ai-model-v1",
  "prediction": {
    "class": "category_a",
    "confidence": 0.87
  },
  "explanation": {...}
}

Response: 201 Created
{
  "prediction_id": "uuid",
  "escalation_required": false
}
```

### Human Review

#### Get Review Queue

```http
GET /review/queue?reviewer_id={id}&priority=high

Response: 200 OK
{
  "queue_depth": 15,
  "tasks": [
    {
      "task_id": "uuid",
      "priority": "high",
      "ai_prediction": {...},
      "context": {...}
    }
  ]
}
```

#### Submit Decision

```http
POST /tasks/{task_id}/decisions
Content-Type: application/json

{
  "reviewer_id": "reviewer-123",
  "decision": {
    "action": "modify",
    "final_output": {...},
    "confidence": 0.95
  },
  "feedback": {
    "ai_helpfulness": 4,
    "comments": "..."
  }
}

Response: 201 Created
{
  "decision_id": "uuid",
  "task_completed": true
}
```

### Metrics

#### Get Performance Metrics

```http
GET /sessions/{session_id}/metrics?period=7d

Response: 200 OK
{
  "period_start": "2025-12-18T00:00:00Z",
  "period_end": "2025-12-25T00:00:00Z",
  "productivity": {...},
  "quality": {...},
  "collaboration": {...}
}
```

## Webhooks

Subscribe to events:

```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://your-app.com/webhook",
  "events": ["task.completed", "escalation.created"],
  "secret": "your-webhook-secret"
}
```

## Rate Limits

- **Free tier**: 100 requests/minute
- **Pro tier**: 1,000 requests/minute
- **Enterprise**: Custom limits

## Error Responses

```json
{
  "error": {
    "code": "INVALID_THRESHOLD",
    "message": "Confidence threshold must be between 0 and 1",
    "details": {...}
  }
}
```

---

**弘益人間** (Hongik Ingan) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA
