# WIA Sign Language Recognition Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-12-25  
**Author:** WIA Technical Committee  

---

## Table of Contents

1. [Introduction](#introduction)
2. [REST API Endpoints](#rest-api-endpoints)
3. [WebSocket Streaming API](#websocket-streaming-api)
4. [Authentication](#authentication)
5. [Rate Limiting](#rate-limiting)
6. [Error Handling](#error-handling)

---

## 1. Introduction

This specification defines standard REST and WebSocket APIs for sign language recognition services.

---

## 2. REST API Endpoints

### 2.1 Recognition Endpoint

**POST /api/v1/recognize**

Request:
```json
{
  "video": "base64_data",
  "language": "ase",
  "options": {
    "realtime": false
  }
}
```

Response:
```json
{
  "jobId": "uuid",
  "status": "processing",
  "estimatedTime": 5000
}
```

### 2.2 Status Check

**GET /api/v1/status/{jobId}**

Response:
```json
{
  "jobId": "uuid",
  "status": "completed",
  "result": {
    "text": "Hello, how are you?",
    "confidence": 0.98
  }
}
```

### 2.3 Translation Endpoint

**POST /api/v1/translate**

Translate between sign languages.

---

## 3. WebSocket Streaming API

### 3.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia.sign/v1/stream');
```

### 3.2 Message Format

```json
{
  "type": "video_frame",
  "timestamp": 1234567890,
  "data": "base64_frame"
}
```

Response:
```json
{
  "type": "recognition",
  "text": "Hello",
  "confidence": 0.98
}
```

---

## 4. Authentication

Bearer token authentication:

```
Authorization: Bearer YOUR_API_KEY
```

---

## 5. Rate Limiting

- 100 requests per minute (free tier)
- 1000 requests per minute (paid tier)

---

## 6. Error Handling

HTTP Status Codes:
- 200: Success
- 400: Bad Request
- 401: Unauthorized
- 429: Rate Limit Exceeded
- 500: Server Error

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA
