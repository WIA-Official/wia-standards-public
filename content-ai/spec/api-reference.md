# WIA-EDU-020: Content AI Standard
## API Reference

**Version:** 1.0
**Base URL:** `https://api.wia.org/v1/content-ai`

---

## Authentication

All API requests require authentication using API keys and/or OAuth 2.0 tokens.

```http
Authorization: Bearer {access_token}
X-API-Key: {api_key}
Content-Type: application/json
```

---

## 1. Content Generation

### 1.1 Generate Lesson

Generate a complete lesson plan with activities and assessments.

**Endpoint:** `POST /generate/lesson`

**Request:**
```json
{
  "subject": "mathematics",
  "topic": "quadratic-equations",
  "gradeLevel": "8th",
  "objectives": [
    "Understand the standard form of quadratic equations",
    "Solve quadratic equations using factoring",
    "Apply quadratic equations to real-world problems"
  ],
  "duration": 45,
  "includeAssessment": true,
  "differentiation": true
}
```

**Response:**
```json
{
  "id": "lesson-abc123",
  "title": "Mastering Quadratic Equations",
  "sections": [
    {
      "id": "sec-1",
      "type": "introduction",
      "title": "Introduction to Quadratic Equations",
      "content": "...",
      "duration": 5
    }
  ],
  "assessments": [...],
  "resources": [...],
  "standards": ["CCSS.MATH.8.EE.A.2"],
  "metadata": {
    "difficulty": 0.6,
    "readingLevel": 8.2,
    "estimatedTime": 45
  },
  "generatedAt": "2025-01-15T10:30:00Z"
}
```

### 1.2 Generate Problems

Generate practice problems with solutions.

**Endpoint:** `POST /generate/problems`

**Request:**
```json
{
  "topic": "factoring-polynomials",
  "difficulty": 0.6,
  "count": 10,
  "includeHints": true,
  "includeSolutions": true,
  "format": "short-answer"
}
```

**Response:**
```json
{
  "problems": [
    {
      "id": "prob-1",
      "question": "Factor the expression: x² + 5x + 6",
      "hints": [
        "Find two numbers that multiply to 6 and add to 5"
      ],
      "solution": {
        "answer": "(x + 2)(x + 3)",
        "steps": [
          "Identify a=1, b=5, c=6",
          "Find factors of 6 that sum to 5: 2 and 3",
          "Write as (x + 2)(x + 3)"
        ]
      },
      "difficulty": 0.6,
      "tags": ["factoring", "quadratic"]
    }
  ],
  "metadata": {
    "generatedCount": 10,
    "averageDifficulty": 0.62
  }
}
```

### 1.3 Generate Assessment

Create assessments with various question types.

**Endpoint:** `POST /generate/assessment`

**Request:**
```json
{
  "topic": "quadratic-equations",
  "questionTypes": ["multiple-choice", "short-answer", "essay"],
  "questionCount": 15,
  "difficulty": 0.7,
  "standards": ["CCSS.MATH.8.EE.A.2"],
  "includeRubric": true
}
```

**Response:**
```json
{
  "id": "assessment-xyz789",
  "title": "Quadratic Equations Assessment",
  "questions": [...],
  "rubric": {...},
  "estimatedTime": 30,
  "totalPoints": 100
}
```

### 1.4 Generate Video

Create educational video content.

**Endpoint:** `POST /generate/video`

**Request:**
```json
{
  "topic": "quadratic-equations-intro",
  "script": "In this lesson, we'll explore...",
  "duration": 300,
  "style": "animated",
  "voiceConfig": {
    "gender": "female",
    "accent": "en-US",
    "speed": 1.0
  },
  "captions": true,
  "language": "en"
}
```

**Response:**
```json
{
  "id": "video-vid123",
  "videoUrl": "https://cdn.wia.org/videos/vid123.mp4",
  "thumbnailUrl": "https://cdn.wia.org/thumbnails/vid123.jpg",
  "captionsUrl": "https://cdn.wia.org/captions/vid123.vtt",
  "transcript": "...",
  "duration": 302,
  "status": "ready",
  "metadata": {
    "resolution": "1080p",
    "size": 45678901,
    "codec": "h264"
  }
}
```

---

## 2. Personalization

### 2.1 Personalize Content

Adapt content to individual learner needs.

**Endpoint:** `POST /personalize`

**Request:**
```json
{
  "contentId": "lesson-abc123",
  "learnerProfile": {
    "readingLevel": 7.5,
    "learningStyle": "visual",
    "pace": "accelerated",
    "interests": ["sports", "engineering"],
    "language": "en"
  },
  "adaptations": {
    "readingLevel": true,
    "learningStyle": true,
    "interests": true
  }
}
```

**Response:**
```json
{
  "personalizedContentId": "lesson-abc123-personalized-xyz",
  "content": {...},
  "adaptationLog": [
    {
      "type": "reading-level",
      "from": 8.2,
      "to": 7.5,
      "changes": ["simplified vocabulary", "shorter sentences"]
    },
    {
      "type": "interests",
      "additions": ["sports example", "engineering application"]
    }
  ],
  "recommendedNext": [...]
}
```

### 2.2 Get Recommendations

Get personalized content recommendations.

**Endpoint:** `POST /recommendations`

**Request:**
```json
{
  "studentId": "student-123",
  "currentTopic": "algebra",
  "context": {
    "timeAvailable": 30,
    "objectives": ["master-quadratic-equations"]
  }
}
```

**Response:**
```json
{
  "recommendations": [
    {
      "contentId": "lesson-def456",
      "title": "Advanced Quadratic Applications",
      "type": "lesson",
      "relevance": 0.95,
      "estimatedTime": 25,
      "difficulty": 0.7,
      "reason": "Builds on current progress in quadratic equations"
    }
  ]
}
```

---

## 3. Translation

### 3.1 Translate Content

Translate content to multiple languages.

**Endpoint:** `POST /translate`

**Request:**
```json
{
  "contentId": "lesson-abc123",
  "sourceLanguage": "en",
  "targetLanguages": ["es", "fr", "zh", "ar"],
  "culturalAdaptation": true,
  "preserveFormatting": true
}
```

**Response:**
```json
{
  "translations": [
    {
      "language": "es",
      "contentId": "lesson-abc123-es",
      "content": {...},
      "culturalAdaptations": [
        {
          "type": "example",
          "original": "football field",
          "adapted": "soccer field"
        }
      ],
      "quality": {
        "bleuScore": 0.67,
        "reviewStatus": "auto"
      }
    }
  ],
  "status": "completed"
}
```

---

## 4. Analytics

### 4.1 Get Content Analytics

Retrieve analytics for content performance.

**Endpoint:** `GET /analytics/{contentId}`

**Query Parameters:**
- `timeRange`: Date range (e.g., "last-30-days", "2025-01-01/2025-01-31")
- `metrics`: Comma-separated metrics (e.g., "engagement,effectiveness")

**Response:**
```json
{
  "contentId": "lesson-abc123",
  "timeRange": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-01-31T23:59:59Z"
  },
  "engagement": {
    "views": 1250,
    "completionRate": 0.84,
    "averageTimeOnContent": 1820,
    "interactionRate": 0.92,
    "returnRate": 0.45
  },
  "effectiveness": {
    "averageLearningGain": 0.32,
    "retentionRate": 0.78,
    "satisfactionScore": 4.3
  },
  "usage": {
    "totalStudents": 450,
    "activeStudents": 380,
    "completedStudents": 315
  }
}
```

### 4.2 A/B Test

Create and manage A/B tests.

**Endpoint:** `POST /ab-test`

**Request:**
```json
{
  "name": "Video vs Text Lesson",
  "variants": [
    {
      "name": "Video",
      "contentId": "lesson-video-123",
      "weight": 0.5
    },
    {
      "name": "Text",
      "contentId": "lesson-text-456",
      "weight": 0.5
    }
  ],
  "metrics": ["completion-rate", "learning-gain", "satisfaction"],
  "sampleSize": 1000,
  "duration": 30
}
```

**Response:**
```json
{
  "testId": "ab-test-789",
  "status": "running",
  "startDate": "2025-01-15T00:00:00Z",
  "endDate": "2025-02-14T23:59:59Z",
  "currentResults": {
    "sampleSize": 234,
    "variants": [...]
  }
}
```

---

## 5. Quality Assurance

### 5.1 Validate Content

Check content quality and compliance.

**Endpoint:** `POST /validate`

**Request:**
```json
{
  "content": {...},
  "checks": [
    "grammar",
    "factual-accuracy",
    "bias",
    "accessibility",
    "standards-alignment"
  ]
}
```

**Response:**
```json
{
  "validationId": "val-456",
  "overallScore": 0.92,
  "checks": [
    {
      "type": "grammar",
      "score": 0.98,
      "issues": [
        {
          "severity": "low",
          "location": "section-2, paragraph-3",
          "message": "Consider using active voice",
          "suggestion": "Students solve equations..."
        }
      ]
    },
    {
      "type": "accessibility",
      "score": 0.95,
      "wcagLevel": "AA",
      "issues": []
    }
  ],
  "recommendations": [...]
}
```

---

## 6. Content Management

### 6.1 Create Content

Store custom content.

**Endpoint:** `POST /content`

**Request:**
```json
{
  "type": "lesson",
  "title": "Custom Lesson",
  "content": {...},
  "metadata": {
    "subject": "mathematics",
    "gradeLevel": "8th",
    "standards": ["CCSS.MATH.8.EE.A.2"]
  }
}
```

### 6.2 Update Content

Update existing content.

**Endpoint:** `PUT /content/{contentId}`

### 6.3 Delete Content

Delete content.

**Endpoint:** `DELETE /content/{contentId}`

### 6.4 List Content

List content with filtering.

**Endpoint:** `GET /content`

**Query Parameters:**
- `subject`: Filter by subject
- `gradeLevel`: Filter by grade level
- `type`: Filter by content type
- `page`: Page number
- `limit`: Items per page

---

## 7. Error Handling

### Error Response Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Missing required field: topic",
    "details": {
      "field": "topic",
      "requirement": "required"
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-xyz789"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Invalid request parameters |
| `UNAUTHORIZED` | 401 | Missing or invalid authentication |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `SERVER_ERROR` | 500 | Internal server error |
| `SERVICE_UNAVAILABLE` | 503 | Service temporarily unavailable |

---

## 8. Rate Limiting

Rate limits are enforced per API key:

- **Free Tier**: 100 requests/hour
- **Pro Tier**: 1,000 requests/hour
- **Enterprise**: Custom limits

Rate limit headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1705320000
```

---

## 9. Webhooks

Subscribe to events via webhooks.

**Supported Events:**
- `content.generated`
- `content.personalized`
- `content.translated`
- `analytics.updated`
- `test.completed`

**Webhook Payload:**
```json
{
  "event": "content.generated",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "contentId": "lesson-abc123",
    "status": "completed"
  }
}
```

---

## 10. SDK Examples

### TypeScript/JavaScript

```typescript
import { ContentAI } from '@wia/content-ai';

const client = new ContentAI({
  apiKey: process.env.WIA_API_KEY
});

const lesson = await client.generateLesson({
  subject: 'mathematics',
  topic: 'quadratic-equations',
  gradeLevel: '8th',
  objectives: [...]
});
```

### Python

```python
from wia_content_ai import ContentAI

client = ContentAI(api_key=os.environ['WIA_API_KEY'])

lesson = client.generate_lesson(
    subject='mathematics',
    topic='quadratic-equations',
    grade_level='8th',
    objectives=[...]
)
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
