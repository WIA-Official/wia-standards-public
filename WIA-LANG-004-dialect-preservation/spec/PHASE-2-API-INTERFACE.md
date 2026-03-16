# WIA-LANG PHASE 2: API Interface Specification

## Version 1.0 | 弘益人間 · Benefit All Humanity

## 1. RESTful API Endpoints

### 1.1 Base URL
```
https://api.wia-lang.org/v1
```

### 1.2 Authentication
```http
Authorization: Bearer {API_KEY}
Content-Type: application/json
```

### 1.3 Core Endpoints

#### Create Recording
```http
POST /languages/{iso-code}/recordings
{
  "format": "flac",
  "sampleRate": 96000,
  "speaker": {
    "id": "speaker-uuid",
    "age": 65,
    "nativeStatus": true
  },
  "metadata": {
    "context": "storytelling",
    "location": "gps-coordinates"
  }
}
```

#### Analyze Language
```http
POST /languages/{iso-code}/analyze
{
  "text": "Sample text",
  "features": ["phonemes", "morphology", "syntax"]
}
```

#### Search Archive
```http
GET /archive/search?q=language:ainu+type:folktale&limit=10
```

## 2. WebSocket API

### 2.1 Real-time Streaming
```javascript
ws://api.wia-lang.org/v1/stream/{session-id}

// Message format
{
  "type": "audio_chunk",
  "data": "base64-encoded-audio",
  "timestamp": 1703635200000
}
```

## 3. GraphQL API

```graphql
query GetLanguage($code: String!) {
  language(isoCode: $code) {
    name
    endangerment
    speakers {
      native
      l2
    }
    recordings {
      id
      duration
      speaker {
        name
        age
      }
    }
  }
}
```

## 4. Rate Limiting

- Free Tier: 1,000 requests/day
- Pro Tier: 10,000 requests/day
- Enterprise: Unlimited

## 5. Error Handling

```json
{
  "error": {
    "code": "LANG_NOT_FOUND",
    "message": "Language with code 'xyz' not found",
    "details": {...}
  }
}
```

## 6. Webhooks

```http
POST /webhooks/register
{
  "url": "https://your-app.com/webhook",
  "events": ["recording.completed", "analysis.finished"]
}
```

---
© 2025 SmileStory Inc. / WIA · Licensed under CC BY-SA 4.0
