# Chapter 5: API Interface Specifications

## Overview

This chapter defines the RESTful API endpoints and WebSocket interfaces for WIA-IND-012 compliant systems. These APIs enable standardized communication between fitness tracking devices, applications, and services.

---

## 5.1 API Design Principles

### 5.1.1 REST Architecture

**Core Principles:**
- **Resource-based URLs**: `/api/v1/activities/{id}`
- **HTTP methods**: GET, POST, PUT, DELETE, PATCH
- **Stateless**: Each request contains all necessary information
- **JSON format**: Request and response bodies
- **Versioned**: `/api/v1/` for stability

### 5.1.2 HTTP Status Codes

```
Success Codes:
200 OK              - Successful GET, PUT, PATCH
201 Created         - Successful POST
204 No Content      - Successful DELETE

Client Error Codes:
400 Bad Request     - Invalid request format
401 Unauthorized    - Missing or invalid authentication
403 Forbidden       - Authenticated but not authorized
404 Not Found       - Resource doesn't exist
409 Conflict        - Resource conflict (e.g., duplicate)
422 Unprocessable   - Validation error
429 Too Many Requests - Rate limit exceeded

Server Error Codes:
500 Internal Server Error - Unexpected server error
502 Bad Gateway     - Upstream service error
503 Service Unavailable - Maintenance or overload
```

### 5.1.3 Authentication

**OAuth 2.0 Flow:**
```http
POST /oauth/token HTTP/1.1
Host: api.example.com
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTHORIZATION_CODE&
redirect_uri=https://app.example.com/callback&
client_id=CLIENT_ID&
client_secret=CLIENT_SECRET
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "tGzv3JOkF0XG5Qx2TlKWIA",
  "scope": "activities:read activities:write profile:read"
}
```

**Using Access Token:**
```http
GET /api/v1/activities HTTP/1.1
Host: api.example.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## 5.2 Activity Endpoints

### 5.2.1 Create Activity

**Request:**
```http
POST /api/v1/activities HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "running",
  "startTime": "2025-12-27T07:00:00Z",
  "endTime": "2025-12-27T07:32:15Z",
  "distance": 8000,
  "calories": 680,
  "route": {
    "points": [
      {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "elevation": 52.3,
        "timestamp": 1735283400000
      }
    ]
  }
}
```

**Response (201 Created):**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "userId": "user_12345",
  "type": "running",
  "startTime": "2025-12-27T07:00:00Z",
  "endTime": "2025-12-27T07:32:15Z",
  "duration": 1935,
  "distance": 8000,
  "calories": 680,
  "avgPace": 4.03,
  "createdAt": "2025-12-27T07:32:15Z",
  "updatedAt": "2025-12-27T07:32:15Z"
}
```

### 5.2.2 Get Activity

**Request:**
```http
GET /api/v1/activities/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "userId": "user_12345",
  "type": "running",
  "startTime": "2025-12-27T07:00:00Z",
  "endTime": "2025-12-27T07:32:15Z",
  "duration": 1935,
  "distance": 8000,
  "steps": 9840,
  "calories": 680,
  "avgPace": 4.03,
  "avgSpeed": 14.88,
  "heartRate": {
    "avg": 155,
    "max": 178,
    "min": 142,
    "zones": {
      "zone1": 0,
      "zone2": 5.2,
      "zone3": 18.5,
      "zone4": 8.3,
      "zone5": 0
    }
  },
  "route": {
    "points": [...],
    "totalDistance": 8000,
    "elevationGain": 125,
    "elevationLoss": 118
  }
}
```

### 5.2.3 List Activities

**Request:**
```http
GET /api/v1/activities?startDate=2025-12-01&endDate=2025-12-31&type=running&limit=20&offset=0 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Query Parameters:**
- `startDate`: ISO 8601 date (optional)
- `endDate`: ISO 8601 date (optional)
- `type`: Activity type filter (optional)
- `limit`: Results per page (default: 20, max: 100)
- `offset`: Pagination offset (default: 0)
- `sort`: Sort field (default: startTime)
- `order`: Sort order (asc/desc, default: desc)

**Response (200 OK):**
```json
{
  "data": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "type": "running",
      "startTime": "2025-12-27T07:00:00Z",
      "duration": 1935,
      "distance": 8000,
      "calories": 680
    },
    {
      "id": "660f9511-f39c-52e5-b827-557766551111",
      "type": "running",
      "startTime": "2025-12-25T06:30:00Z",
      "duration": 2145,
      "distance": 9200,
      "calories": 742
    }
  ],
  "pagination": {
    "total": 45,
    "limit": 20,
    "offset": 0,
    "hasMore": true
  }
}
```

### 5.2.4 Update Activity

**Request:**
```http
PUT /api/v1/activities/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "notes": "Great morning run!",
  "tags": ["morning", "easy"],
  "perceivedExertion": 6
}
```

**Response (200 OK):**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "notes": "Great morning run!",
  "tags": ["morning", "easy"],
  "perceivedExertion": 6,
  "updatedAt": "2025-12-27T08:15:00Z"
}
```

### 5.2.5 Delete Activity

**Request:**
```http
DELETE /api/v1/activities/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (204 No Content)**

---

## 5.3 Workout Endpoints

### 5.3.1 Create Workout

**Request:**
```http
POST /api/v1/workouts HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "interval_training",
  "name": "5x1000m Intervals",
  "startTime": "2025-12-27T17:00:00Z",
  "intervals": [
    {
      "number": 1,
      "type": "warmup",
      "duration": 600,
      "targetPace": 6.0
    },
    {
      "number": 2,
      "type": "work",
      "duration": 240,
      "distance": 1000,
      "targetPace": 4.0
    },
    {
      "number": 3,
      "type": "rest",
      "duration": 120
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "id": "workout_770a1022",
  "userId": "user_12345",
  "type": "interval_training",
  "name": "5x1000m Intervals",
  "startTime": "2025-12-27T17:00:00Z",
  "intervals": [...],
  "createdAt": "2025-12-27T17:45:00Z"
}
```

### 5.3.2 Get Workout Details

**Request:**
```http
GET /api/v1/workouts/workout_770a1022 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response includes full workout data with intervals, heart rate zones, training load, etc.**

---

## 5.4 Metrics Endpoints

### 5.4.1 Get Heart Rate Data

**Request:**
```http
GET /api/v1/metrics/heart-rate?startTime=2025-12-27T07:00:00Z&endTime=2025-12-27T07:32:00Z HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "userId": "user_12345",
  "startTime": "2025-12-27T07:00:00Z",
  "endTime": "2025-12-27T07:32:00Z",
  "readings": [
    {"timestamp": "2025-12-27T07:00:00Z", "bpm": 142},
    {"timestamp": "2025-12-27T07:00:05Z", "bpm": 145},
    {"timestamp": "2025-12-27T07:00:10Z", "bpm": 148}
  ],
  "avgBpm": 155,
  "maxBpm": 178,
  "minBpm": 142
}
```

### 5.4.2 Get Step Count Data

**Request:**
```http
GET /api/v1/metrics/steps?date=2025-12-27 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "date": "2025-12-27",
  "totalSteps": 12547,
  "goal": 10000,
  "progress": 125.47,
  "hourlyBreakdown": [
    {"hour": 0, "steps": 0},
    {"hour": 1, "steps": 0},
    {"hour": 7, "steps": 9840},
    {"hour": 8, "steps": 524},
    {"hour": 9, "steps": 1203}
  ]
}
```

### 5.4.3 Get Calorie Data

**Request:**
```http
GET /api/v1/metrics/calories?date=2025-12-27 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "date": "2025-12-27",
  "totalCalories": 2847,
  "breakdown": {
    "bmr": 1685,
    "active": 1052,
    "epoc": 110
  },
  "tdee": 2847,
  "goal": 2500,
  "activities": [
    {
      "activityId": "550e8400-e29b-41d4-a716-446655440000",
      "type": "running",
      "calories": 680,
      "startTime": "2025-12-27T07:00:00Z"
    }
  ]
}
```

---

## 5.5 Goal Endpoints

### 5.5.1 Create Goal

**Request:**
```http
POST /api/v1/goals HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "daily_steps",
  "name": "Daily 10K Steps",
  "target": 10000,
  "unit": "steps",
  "period": "daily",
  "startDate": "2025-12-27"
}
```

**Response (201 Created):**
```json
{
  "id": "goal_880b2133",
  "userId": "user_12345",
  "type": "daily_steps",
  "name": "Daily 10K Steps",
  "target": 10000,
  "current": 12547,
  "unit": "steps",
  "period": "daily",
  "progress": 125.47,
  "status": "active",
  "createdAt": "2025-12-27T08:00:00Z"
}
```

### 5.5.2 List Goals

**Request:**
```http
GET /api/v1/goals?status=active HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "data": [
    {
      "id": "goal_880b2133",
      "type": "daily_steps",
      "name": "Daily 10K Steps",
      "target": 10000,
      "current": 12547,
      "progress": 125.47,
      "status": "active"
    },
    {
      "id": "goal_991c3244",
      "type": "weekly_distance",
      "name": "Run 25km per week",
      "target": 25,
      "current": 17.2,
      "progress": 68.8,
      "status": "active"
    }
  ]
}
```

### 5.5.3 Update Goal Progress

**Request:**
```http
PATCH /api/v1/goals/goal_880b2133 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "current": 13250
}
```

**Response (200 OK):**
```json
{
  "id": "goal_880b2133",
  "current": 13250,
  "progress": 132.5,
  "status": "completed",
  "completedAt": "2025-12-27T14:30:00Z"
}
```

---

## 5.6 Summary Endpoints

### 5.6.1 Daily Summary

**Request:**
```http
GET /api/v1/summary/daily?date=2025-12-27 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "date": "2025-12-27",
  "steps": 12547,
  "distance": 9.8,
  "calories": 2847,
  "activeMinutes": 87,
  "workouts": 2,
  "heartRate": {
    "resting": 52,
    "avg": 68,
    "max": 178
  },
  "sleep": {
    "duration": 465,
    "quality": 85,
    "efficiency": 94.6
  },
  "goals": {
    "completed": 2,
    "total": 3
  }
}
```

### 5.6.2 Weekly Summary

**Request:**
```http
GET /api/v1/summary/weekly?startDate=2025-12-21 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "weekStartDate": "2025-12-21",
  "weekEndDate": "2025-12-27",
  "totalSteps": 78432,
  "totalDistance": 62.3,
  "totalCalories": 18247,
  "totalWorkouts": 8,
  "activeMinutes": 428,
  "avgSleep": 452,
  "dailyBreakdown": [
    {
      "date": "2025-12-21",
      "steps": 10234,
      "distance": 8.2,
      "calories": 2456,
      "workouts": 1
    }
  ]
}
```

### 5.6.3 Monthly Summary

**Request:**
```http
GET /api/v1/summary/monthly?year=2025&month=12 HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
```

---

## 5.7 WebSocket Real-Time Streaming

### 5.7.1 Heart Rate Stream

**Connection:**
```javascript
const ws = new WebSocket('wss://api.example.com/v1/stream/heart-rate');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Heart Rate:', data.bpm);
};
```

**Server Messages:**
```json
{
  "type": "heartRateUpdate",
  "timestamp": "2025-12-27T07:15:30Z",
  "bpm": 155,
  "zone": 3,
  "confidence": 98
}
```

### 5.7.2 Activity Stream

**Connection:**
```javascript
const ws = new WebSocket('wss://api.example.com/v1/stream/activity');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  updateDashboard(data);
};
```

**Server Messages:**
```json
{
  "type": "activityUpdate",
  "timestamp": "2025-12-27T07:15:30Z",
  "distance": 5230,
  "pace": 4.05,
  "calories": 456,
  "duration": 1620,
  "heartRate": 155
}
```

### 5.7.3 Goal Achievement

**Server Messages:**
```json
{
  "type": "goalAchieved",
  "goalId": "goal_880b2133",
  "goalName": "Daily 10K Steps",
  "achievedAt": "2025-12-27T14:30:00Z",
  "finalValue": 10000
}
```

---

## 5.8 Rate Limiting

### 5.8.1 Rate Limit Headers

**Response Headers:**
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1735290000
```

### 5.8.2 Rate Limit Tiers

```
Free Tier:
- 1,000 requests/hour
- 10,000 requests/day

Standard Tier:
- 5,000 requests/hour
- 50,000 requests/day

Premium Tier:
- 20,000 requests/hour
- 200,000 requests/day

Enterprise:
- Custom limits
- Dedicated infrastructure
```

### 5.8.3 Rate Limit Exceeded

**Response (429 Too Many Requests):**
```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Retry after 3600 seconds.",
    "retryAfter": 3600
  }
}
```

---

## 5.9 Error Handling

### 5.9.1 Error Response Format

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "Additional context"
    },
    "requestId": "req_abc123",
    "timestamp": "2025-12-27T08:00:00Z"
  }
}
```

### 5.9.2 Common Error Codes

```
AUTHENTICATION_FAILED      - Invalid credentials
AUTHORIZATION_FAILED       - Insufficient permissions
RESOURCE_NOT_FOUND        - Requested resource doesn't exist
VALIDATION_ERROR          - Request validation failed
CONFLICT                  - Resource conflict (e.g., duplicate)
RATE_LIMIT_EXCEEDED       - Too many requests
INTERNAL_ERROR            - Server error
SERVICE_UNAVAILABLE       - Temporary service outage
```

---

## 5.10 Webhook Subscriptions

### 5.10.1 Register Webhook

**Request:**
```http
POST /api/v1/webhooks HTTP/1.1
Host: api.example.com
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://myapp.com/webhooks/fitness",
  "events": ["workout.completed", "goal.achieved"],
  "secret": "webhook_secret_key"
}
```

**Response (201 Created):**
```json
{
  "id": "webhook_123abc",
  "url": "https://myapp.com/webhooks/fitness",
  "events": ["workout.completed", "goal.achieved"],
  "active": true,
  "createdAt": "2025-12-27T08:00:00Z"
}
```

### 5.10.2 Webhook Payload

```http
POST /webhooks/fitness HTTP/1.1
Host: myapp.com
Content-Type: application/json
X-WIA-Signature: sha256=abc123...
X-WIA-Event: workout.completed

{
  "event": "workout.completed",
  "timestamp": "2025-12-27T07:32:15Z",
  "userId": "user_12345",
  "data": {
    "workoutId": "workout_770a1022",
    "type": "running",
    "duration": 1935,
    "distance": 8000,
    "calories": 680
  }
}
```

---

## Key Takeaways

✓ RESTful API follows standard HTTP conventions and status codes

✓ OAuth 2.0 provides secure authentication and authorization

✓ Comprehensive endpoints cover activities, workouts, metrics, goals, and summaries

✓ WebSocket support enables real-time heart rate and activity streaming

✓ Rate limiting protects API infrastructure and ensures fair usage

✓ Consistent error handling with detailed error codes and messages

✓ Webhook subscriptions enable event-driven integrations

✓ Pagination and filtering support efficient data retrieval

✓ All responses use standardized JSON format matching data schemas

---

**Next:** [Chapter 6: Protocol Specifications →](06-protocol.md)

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
