# WIA-EDU-025: API Reference

**Version:** 1.0.0
**Last Updated:** 2025-12-26

---

## Base URL

```
Production: https://api.wiastandards.com/entertainment-robot/v1
Sandbox: https://sandbox-api.wiastandards.com/entertainment-robot/v1
```

---

## Authentication

All API requests require authentication using OAuth 2.0 or DID (Decentralized Identifier).

### OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your-client-id",
  "client_secret": "your-client-secret"
}
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

---

## REST API Endpoints

### Stories

#### GET /stories
List available interactive stories.

**Query Parameters:**
- `genre` (string): Filter by genre
- `ageRange` (string): Filter by target age range (e.g., "7-12")
- `language` (string): ISO 639-1 language code
- `page` (integer): Page number for pagination
- `limit` (integer): Results per page (max 100)

**Response:**
```json
{
  "stories": [
    {
      "storyId": "STORY-2025-001",
      "title": "The Magic Forest Adventure",
      "genre": "fantasy",
      "targetAgeRange": "7-12",
      "duration": 1200,
      "thumbnailUrl": "https://...",
      "rating": 4.8,
      "learningObjectives": ["critical-thinking", "decision-making"]
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "totalPages": 5,
    "totalItems": 95
  }
}
```

#### GET /stories/{storyId}
Get complete story data.

**Response:**
```json
{
  "@context": "https://wiastandards.com/contexts/entertainment-robot/v1",
  "@type": "InteractiveStory",
  "storyId": "STORY-2025-001",
  "title": "The Magic Forest Adventure",
  "chapters": [...],
  "characters": [...],
  "learningObjectives": [...]
}
```

#### POST /stories/{storyId}/sessions
Start a new story session.

**Request:**
```json
{
  "userId": "anonymous-hash-12345",
  "robotId": "ENT-ROBOT-001",
  "sessionMetadata": {
    "location": "home",
    "parentalSupervision": true
  }
}
```

**Response:**
```json
{
  "sessionId": "SESSION-2025-001",
  "startTime": "2025-12-26T10:00:00Z",
  "websocketUrl": "wss://ws.wiastandards.com/sessions/SESSION-2025-001",
  "expiresIn": 7200
}
```

### Performances

#### GET /performances
List available performance programs.

#### GET /performances/{performanceId}
Get complete performance program.

#### POST /performances/{performanceId}/schedule
Schedule a performance.

### Therapeutic Sessions

#### GET /therapeutic-protocols
List validated therapeutic protocols.

**Requires:** Professional credentials verification

#### POST /therapeutic-sessions
Create therapeutic session.

**Requires:** Professional supervision authorization

### Emotions

#### POST /emotions/detect
Process emotion detection data.

**Request:**
```json
{
  "sessionId": "SESSION-2025-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "multimodalData": {
    "facial": {...},
    "voice": {...},
    "gesture": {...}
  }
}
```

**Response:**
```json
{
  "emotionalState": {
    "primary": "happy",
    "confidence": 0.87,
    "secondary": ["excited"],
    "engagementLevel": 0.85
  },
  "recommendedResponse": {
    "strategy": "encourage-continue",
    "tone": "enthusiastic-supportive"
  }
}
```

---

## WebSocket Protocol

### Connection

```javascript
const ws = new WebSocket('wss://ws.wiastandards.com/sessions/SESSION-2025-001');
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'your-access-token'
  }));
};
```

### Message Types

#### Story Progression
```json
{
  "type": "story-beat",
  "beatId": "B01",
  "text": "Once upon a time...",
  "characterId": "narrator",
  "emotionalTone": "mysterious"
}
```

#### User Choice
```json
{
  "type": "user-choice",
  "choiceId": "C01",
  "selectedOption": "O01",
  "timestamp": "2025-12-26T10:35:00Z"
}
```

#### Emotion Update
```json
{
  "type": "emotion-update",
  "emotionalState": {
    "primary": "happy",
    "confidence": 0.87
  }
}
```

#### Robot Action
```json
{
  "type": "robot-action",
  "actionType": "movement",
  "choreography": "celebratory-dance",
  "duration": 5
}
```

---

## Rate Limiting

- **Standard Tier:** 1000 requests/hour
- **Professional Tier:** 10,000 requests/hour
- **Enterprise Tier:** Custom limits

Rate limit headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1640534400
```

---

## Error Codes

| Code | Meaning |
|------|---------|
| 400 | Bad Request - Invalid parameters |
| 401 | Unauthorized - Invalid or missing authentication |
| 403 | Forbidden - Insufficient permissions |
| 404 | Not Found - Resource doesn't exist |
| 429 | Too Many Requests - Rate limit exceeded |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

Error response format:
```json
{
  "error": {
    "code": "INVALID_AGE_RANGE",
    "message": "Age range must be in format 'X-Y'",
    "details": "Received: '7-'",
    "timestamp": "2025-12-26T10:40:00Z",
    "requestId": "req-12345"
  }
}
```

---

## SDK Example

### TypeScript/JavaScript

```typescript
import { WIAEntertainmentRobot } from '@wia/entertainment-robot';

const client = new WIAEntertainmentRobot({
  apiKey: 'your-api-key',
  robotId: 'ENT-ROBOT-001'
});

// Start a story session
const session = await client.stories.startSession('STORY-2025-001', {
  userId: 'anonymous-hash-12345',
  parentalSupervision: true
});

// Listen for story events
session.on('storyBeat', (beat) => {
  console.log(`Narrator: ${beat.text}`);
});

// Make a choice
await session.makeChoice('C01', 'O01');

// Track emotions
session.on('emotionDetected', (emotion) => {
  console.log(`Child is feeling: ${emotion.primary}`);
});
```

---

**© 2025 WIA - World Certification Industry Association**
