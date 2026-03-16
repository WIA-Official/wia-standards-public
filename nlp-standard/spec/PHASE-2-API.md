# WIA-AI-023 NLP Standard - Phase 2: API Interface

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 2 defines RESTful API specifications for NLP services, ensuring consistent interfaces across different implementations. This enables plug-and-play interoperability and simplifies integration.

### 1.1 Design Principles

- RESTful architecture
- Stateless communication
- Standard HTTP methods
- JSON payloads
- Versioned endpoints
- Authentication and authorization

## 2. API Endpoints

### 2.1 Base URL Structure

```
https://api.example.com/nlp/v1/{task}
```

### 2.2 Core Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/tokenize` | POST | Tokenize text |
| `/ner` | POST | Named entity recognition |
| `/sentiment` | POST | Sentiment analysis |
| `/classify` | POST | Text classification |
| `/generate` | POST | Text generation |
| `/summarize` | POST | Text summarization |
| `/translate` | POST | Machine translation |
| `/embedding` | POST | Get text embeddings |

### 2.3 Utility Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Health check |
| `/info` | GET | Service information |
| `/models` | GET | List available models |
| `/languages` | GET | List supported languages |

## 3. Request/Response Specifications

### 3.1 Tokenization API

**Endpoint:** `POST /nlp/v1/tokenize`

**Request:**
```http
POST /nlp/v1/tokenize HTTP/1.1
Host: api.example.com
Content-Type: application/json
Authorization: Bearer <token>

{
  "text": "Natural language processing is transforming AI.",
  "language": "en",
  "method": "word",
  "include_offsets": true
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "output": {
    "tokens": ["Natural", "language", "processing", "is", "transforming", "AI", "."],
    "token_count": 7
  },
  "metadata": {
    "request_id": "550e8400-e29b-41d4-a716-446655440000",
    "processing_time_ms": 12
  }
}
```

### 3.2 NER API

**Endpoint:** `POST /nlp/v1/ner`

**Request Headers:**
```
Content-Type: application/json
Authorization: Bearer <token>
X-Request-ID: <optional-client-id>
```

**Request Body:**
```json
{
  "text": "Apple CEO Tim Cook announced new products in Cupertino.",
  "language": "en",
  "entity_types": ["PERSON", "ORGANIZATION", "LOCATION"]
}
```

**Response:** (See Phase 1 NER format)

### 3.3 Sentiment Analysis API

**Endpoint:** `POST /nlp/v1/sentiment`

**Request:**
```json
{
  "text": "This is absolutely wonderful!",
  "language": "en",
  "return_scores": true
}
```

### 3.4 Text Classification API

**Endpoint:** `POST /nlp/v1/classify`

**Request:**
```json
{
  "text": "Scientists discovered a new planet.",
  "language": "en",
  "categories": ["Science", "Technology", "Business", "Sports"],
  "top_k": 2
}
```

### 3.5 Text Generation API

**Endpoint:** `POST /nlp/v1/generate`

**Request:**
```json
{
  "prompt": "Natural language processing enables",
  "max_length": 100,
  "temperature": 0.8,
  "top_p": 0.95
}
```

### 3.6 Summarization API

**Endpoint:** `POST /nlp/v1/summarize`

**Request:**
```json
{
  "text": "Long document text...",
  "method": "abstractive",
  "max_length": 150,
  "min_length": 50
}
```

## 4. Authentication

### 4.1 Bearer Token Authentication

```http
Authorization: Bearer <access_token>
```

### 4.2 API Key Authentication

```http
X-API-Key: <api_key>
```

### 4.3 OAuth 2.0

Supports standard OAuth 2.0 flows:
- Client Credentials
- Authorization Code
- Refresh Token

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640995200
```

### 5.2 Rate Limit Response

When exceeded:
```http
HTTP/1.1 429 Too Many Requests
Retry-After: 60

{
  "status": {
    "code": 429,
    "message": "Rate limit exceeded",
    "details": "Limit of 1000 requests per hour exceeded"
  }
}
```

## 6. Batch Processing

### 6.1 Batch Endpoint

**Endpoint:** `POST /nlp/v1/batch`

**Request:**
```json
{
  "requests": [
    {
      "task": "sentiment",
      "input": {"text": "Great product!", "language": "en"}
    },
    {
      "task": "ner",
      "input": {"text": "Tim Cook leads Apple", "language": "en"}
    }
  ]
}
```

**Response:**
```json
{
  "responses": [
    {
      "index": 0,
      "status": "success",
      "output": {"sentiment": "positive", "confidence": 0.95}
    },
    {
      "index": 1,
      "status": "success",
      "output": {"entities": [...]}
    }
  ]
}
```

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "status": {
    "code": 400,
    "message": "Invalid request",
    "details": "Missing required field: text"
  },
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601"
  }
}
```

### 7.2 HTTP Status Codes

| Code | Status | Description |
|------|--------|-------------|
| 200 | OK | Success |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid input |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Endpoint not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary unavailable |

## 8. Versioning

### 8.1 URL Versioning

```
/nlp/v1/sentiment  (version 1)
/nlp/v2/sentiment  (version 2)
```

### 8.2 Header Versioning (Optional)

```http
API-Version: 1.0
```

## 9. CORS Support

### 9.1 CORS Headers

```http
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST, OPTIONS
Access-Control-Allow-Headers: Content-Type, Authorization
Access-Control-Max-Age: 86400
```

## 10. Service Discovery

### 10.1 Info Endpoint

**Endpoint:** `GET /nlp/v1/info`

**Response:**
```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "service": {
    "name": "NLP Service",
    "vendor": "Example Corp",
    "api_version": "1.0.0"
  },
  "supported_tasks": [
    "tokenization",
    "ner",
    "sentiment",
    "classification",
    "generation",
    "summarization"
  ],
  "supported_languages": ["en", "ko", "es", "fr", "de"],
  "rate_limits": {
    "requests_per_hour": 1000,
    "requests_per_day": 10000
  }
}
```

### 10.2 Health Check

**Endpoint:** `GET /nlp/v1/health`

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-25T10:30:00Z",
  "version": "1.0.0",
  "uptime_seconds": 86400
}
```

## 11. Compliance Checklist

- [ ] All endpoints follow RESTful conventions
- [ ] Authentication implemented
- [ ] Rate limiting configured
- [ ] CORS headers set appropriately
- [ ] Error responses follow standard format
- [ ] API versioning implemented
- [ ] Health check endpoint available
- [ ] Service info endpoint available
- [ ] Batch processing supported
- [ ] Documentation available

---

**Previous:** [Phase 1: Data Format](PHASE-1-DATA-FORMAT.md)
**Next:** [Phase 3: Protocol](PHASE-3-PROTOCOL.md)

**弘益人間** · Benefit All Humanity

© 2025 WIA - World Certification Industry Association
