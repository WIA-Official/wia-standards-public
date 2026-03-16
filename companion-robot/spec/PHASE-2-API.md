# Phase 2: API Interface Specification - WIA-ROB-013

## WIA-ROB-013 Companion Robot API Standard

**Version**: 1.0.0  
**Date**: 2025-01-15  
**Status**: Active  
**Standard ID**: WIA-ROB-013-PHASE2-001

---

## 1. API Overview

This phase defines standard APIs for companion robot systems enabling interoperability, integration, and cross-platform compatibility. All WIA-ROB-013 compliant systems must implement these core endpoints.

### 1.1 Base Requirements

- RESTful design principles
- JSON request/response format
- HTTP/2 or HTTP/3 support
- TLS 1.3+ for all connections
- JWT-based authentication
- Rate limiting and quota management
- Comprehensive error handling
- API versioning support

### 1.2 Base URL Structure

```
https://api.companion.example.com/v1/{resource}
```

---

## 2. Authentication

### 2.1 JWT Token Structure

```typescript
interface JWTPayload {
  sub: string;           // User ID
  cid: string;           // Companion ID
  iat: number;           // Issued at
  exp: number;           // Expiration
  scope: string[];       // Permissions
}
```

### 2.2 Authentication Endpoints

**Login**
```
POST /v1/auth/login
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "secure_password"
}

Response: 200 OK
{
  "token": "eyJhbGc...",
  "refreshToken": "eyJhbGc...",
  "expiresIn": 3600
}
```

---

## 3. Session Management

### 3.1 Create Session

```
POST /v1/sessions
Authorization: Bearer {token}
Content-Type: application/json

{
  "companionId": "uuid",
  "preferences": {
    "language": "en",
    "notificationsEnabled": true
  }
}

Response: 201 Created
{
  "sessionId": "uuid",
  "companionId": "uuid",
  "createdAt": "2025-01-15T10:00:00Z",
  "expiresAt": "2025-01-15T18:00:00Z"
}
```

### 3.2 Get Session

```
GET /v1/sessions/{sessionId}
Authorization: Bearer {token}

Response: 200 OK
{
  "sessionId": "uuid",
  "status": "active",
  "messageCount": 42,
  "duration": 3600
}
```

---

## 4. Messaging API

### 4.1 Send Message

```
POST /v1/sessions/{sessionId}/messages
Authorization: Bearer {token}
Content-Type: application/json

{
  "text": "I'm feeling overwhelmed today",
  "language": "en",
  "context": {
    "location": "home",
    "activity": "relaxing",
    "timeOfDay": "evening"
  }
}

Response: 200 OK
{
  "messageId": "uuid",
  "companionResponse": {
    "text": "I hear that you're feeling overwhelmed...",
    "emotionalTone": {
      "valence": 0.4,
      "arousal": 0.3,
      "dominance": 0.2
    },
    "suggestions": [
      "Would you like to talk about what's overwhelming you?",
      "Sometimes a short walk can help clear your mind.",
      "Let's try a brief breathing exercise together."
    ]
  },
  "processingTime": 850
}
```

### 4.2 Get Conversation History

```
GET /v1/sessions/{sessionId}/messages?limit=50&offset=0
Authorization: Bearer {token}

Response: 200 OK
{
  "messages": [...],
  "pagination": {
    "limit": 50,
    "offset": 0,
    "total": 150
  }
}
```

---

## 5. Emotion Analysis API

### 5.1 Analyze Emotion

```
POST /v1/emotions/analyze
Authorization: Bearer {token}
Content-Type: application/json

{
  "text": "I can't believe how happy I am!",
  "voice": "base64_encoded_audio",
  "language": "en"
}

Response: 200 OK
{
  "primaryEmotion": "joy",
  "confidence": 0.92,
  "secondaryEmotions": [
    {"emotion": "excitement", "probability": 0.78},
    {"emotion": "contentment", "probability": 0.45}
  ],
  "dimensions": {
    "valence": 0.85,
    "arousal": 0.72,
    "dominance": 0.55
  },
  "sources": {
    "text": {"emotion": "joy", "confidence": 0.88},
    "voice": {"emotion": "joy", "confidence": 0.96}
  }
}
```

---

## 6. Personality Management API

### 6.1 Get Companion Personality

```
GET /v1/companions/{companionId}/personality
Authorization: Bearer {token}

Response: 200 OK
{
  "personalityTraits": {
    "openness": 75,
    "conscientiousness": 80,
    "extraversion": 65,
    "agreeableness": 90,
    "neuroticism": 25,
    "playfulness": 60,
    "formality": 40,
    "proactivity": 70
  },
  "communicationStyle": "warm and supportive",
  "lastUpdated": "2025-01-15T10:00:00Z"
}
```

### 6.2 Update Personality Traits

```
PATCH /v1/companions/{companionId}/personality
Authorization: Bearer {token}
Content-Type: application/json

{
  "personalityTraits": {
    "extraversion": 70,
    "playfulness": 65
  }
}

Response: 200 OK
{
  "updated": true,
  "personalityTraits": {...}
}
```

---

## 7. Memory and Context API

### 7.1 Store Memory

```
POST /v1/memories
Authorization: Bearer {token}
Content-Type: application/json

{
  "type": "preference",
  "content": "User prefers morning check-ins",
  "importance": 0.8
}

Response: 201 Created
{
  "memoryId": "uuid",
  "stored": true
}
```

### 7.2 Query Memories

```
GET /v1/memories?type=preference&limit=10
Authorization: Bearer {token}

Response: 200 OK
{
  "memories": [...]
}
```

---

## 8. WebSocket API

### 8.1 Connection

```
wss://api.companion.example.com/v1/stream?sessionId={sessionId}&token={jwt}
```

### 8.2 Message Types

```typescript
// User message
{
  "type": "user_message",
  "payload": {
    "text": "Hello!",
    "language": "en"
  },
  "timestamp": "2025-01-15T10:00:00Z"
}

// Companion response
{
  "type": "companion_response",
  "payload": {
    "text": "Hi! How are you feeling today?",
    "emotionalTone": {...}
  },
  "timestamp": "2025-01-15T10:00:01Z"
}

// Emotion update
{
  "type": "emotion_update",
  "payload": {
    "primaryEmotion": "joy",
    "confidence": 0.85
  },
  "timestamp": "2025-01-15T10:00:02Z"
}
```

---

## 9. Error Handling

### 9.1 Standard Error Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Missing required field: text",
    "details": {
      "field": "text",
      "requirement": "must be non-empty string"
    },
    "timestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 9.2 Error Codes

- `INVALID_REQUEST` (400)
- `UNAUTHORIZED` (401)
- `FORBIDDEN` (403)
- `NOT_FOUND` (404)
- `RATE_LIMIT_EXCEEDED` (429)
- `INTERNAL_ERROR` (500)
- `SERVICE_UNAVAILABLE` (503)

---

## 10. Rate Limiting

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1705320000
```

Limits:
- Free tier: 1,000 requests/hour
- Standard tier: 10,000 requests/hour
- Premium tier: 100,000 requests/hour

---

**WIA-ROB-013 PHASE 2 - API Interface Specification**  
© 2025 World Certification Industry Association  
弘益人間 · Benefit All Humanity
