# WIA-IND-014: Virtual Fitness Standard
## Phase 2: API Interface Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-27
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document defines RESTful and WebSocket APIs for virtual fitness platforms, enabling seamless integration between VR/AR applications, fitness tracking systems, and third-party services. APIs follow OpenAPI 3.0 specification and prioritize security, scalability, and developer experience.

## 2. Authentication and Authorization

### 2.1 OAuth 2.0 Implementation

All API endpoints require OAuth 2.0 authentication:

```http
POST /api/v1/auth/token
Content-Type: application/json

{
  "grant_type": "authorization_code",
  "client_id": "your-client-id",
  "client_secret": "your-client-secret",
  "code": "authorization-code",
  "redirect_uri": "https://your-app.com/callback"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh-token-string",
  "scope": "workouts:read workouts:write profile:read"
}
```

### 2.2 Scopes

- `profile:read` - Read user profile
- `profile:write` - Update user profile
- `workouts:read` - Read workout history
- `workouts:write` - Create/update workouts
- `social:read` - Read social connections
- `social:write` - Manage friends and teams
- `achievements:read` - Read achievements
- `biometrics:read` - Read biometric data (requires explicit consent)

## 3. Session Management APIs

### 3.1 Start Workout Session

```http
POST /api/v1/sessions/start
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "workoutType": "vr-boxing",
  "environment": "mountain-peak",
  "difficulty": "hard",
  "trackingMode": "full-body",
  "device": {
    "type": "Meta-Quest-3",
    "firmware": "v60.0"
  }
}
```

**Response:**
```json
{
  "sessionId": "550e8400-e29b-41d4-a716-446655440000",
  "startTime": "2025-01-27T10:00:00Z",
  "trackingEndpoint": "wss://api.wia-fitness.com/tracking/550e8400",
  "maxDuration": 7200,
  "philosophy": "弘益人間"
}
```

### 3.2 Stream Motion Data (WebSocket)

```javascript
const ws = new WebSocket('wss://api.wia-fitness.com/tracking/550e8400');

ws.onopen = () => {
  // Send pose data frames
  ws.send(JSON.stringify({
    type: 'pose-frame',
    sessionId: '550e8400-e29b-41d4-a716-446655440000',
    timestamp: Date.now(),
    keypoints: [...]
  }));
};

ws.onmessage = (event) => {
  const feedback = JSON.parse(event.data);
  // Receive real-time form feedback
  console.log(feedback);
};
```

### 3.3 End Workout Session

```http
POST /api/v1/sessions/{sessionId}/end
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "endTime": "2025-01-27T10:30:00Z",
  "metrics": {
    "totalFrames": 54000,
    "avgHeartRate": 138,
    "caloriesBurned": 450,
    "performanceScore": 89
  }
}
```

## 4. Workout Library APIs

### 4.1 Get Available Workouts

```http
GET /api/v1/workouts?category=vr-boxing&difficulty=hard&page=1&limit=20
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "total": 150,
  "page": 1,
  "limit": 20,
  "workouts": [
    {
      "id": "workout-001",
      "name": "Mountain Peak Combat",
      "type": "vr-boxing",
      "duration": 1800,
      "difficulty": "hard",
      "calorieEstimate": 450,
      "equipment": ["vr-headset", "hand-controllers"],
      "instructor": "Coach Sarah",
      "rating": 4.8,
      "completions": 12500
    }
  ]
}
```

### 4.2 Get Workout Details

```http
GET /api/v1/workouts/{workoutId}
Authorization: Bearer {access_token}
```

## 5. Leaderboard APIs

### 5.1 Get Global Leaderboard

```http
GET /api/v1/leaderboards/global?metric=calories&timeframe=weekly&page=1
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "leaderboard": "global-weekly-calories",
  "updateTime": "2025-01-27T12:00:00Z",
  "entries": [
    {
      "rank": 1,
      "userId": "user-001",
      "username": "FitnessWarrior",
      "avatar": "https://cdn.wia.com/avatars/001.jpg",
      "value": 8450,
      "metric": "calories",
      "country": "USA"
    }
  ],
  "userRank": 342,
  "userValue": 3200
}
```

### 5.2 Get Friends Leaderboard

```http
GET /api/v1/leaderboards/friends?metric=workouts&timeframe=monthly
Authorization: Bearer {access_token}
```

## 6. Achievement APIs

### 6.1 Get User Achievements

```http
GET /api/v1/users/{userId}/achievements
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "total": 48,
  "unlocked": 35,
  "achievements": [
    {
      "id": "ach-100-workouts",
      "name": "Century Club",
      "description": "Complete 100 workouts",
      "category": "consistency",
      "tier": "gold",
      "unlocked": true,
      "unlockedDate": "2025-01-15T08:30:00Z",
      "icon": "https://cdn.wia.com/achievements/century.png",
      "points": 500
    }
  ]
}
```

### 6.2 Unlock Achievement

```http
POST /api/v1/achievements/unlock
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "achievementId": "ach-first-workout",
  "sessionId": "550e8400-e29b-41d4-a716-446655440000"
}
```

## 7. Social APIs

### 7.1 Get Friends List

```http
GET /api/v1/social/friends?status=active
Authorization: Bearer {access_token}
```

### 7.2 Send Friend Request

```http
POST /api/v1/social/friends/request
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "targetUserId": "user-002",
  "message": "Let's workout together!"
}
```

### 7.3 Get Activity Feed

```http
GET /api/v1/social/feed?limit=20
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "activities": [
    {
      "id": "activity-001",
      "userId": "user-002",
      "username": "Sarah",
      "type": "workout-completed",
      "timestamp": "2025-01-27T09:30:00Z",
      "data": {
        "workoutType": "vr-cycling",
        "duration": 2400,
        "calories": 380,
        "achievement": "personal-record"
      },
      "likes": 12,
      "comments": 3
    }
  ]
}
```

## 8. Team and Guild APIs

### 8.1 Create Team

```http
POST /api/v1/teams
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "name": "Virtual Fitness Warriors",
  "description": "Daily VR workouts and challenges",
  "visibility": "public",
  "maxMembers": 50,
  "tags": ["vr-boxing", "competitive", "daily"]
}
```

### 8.2 Join Team

```http
POST /api/v1/teams/{teamId}/join
Authorization: Bearer {access_token}
```

### 8.3 Get Team Leaderboard

```http
GET /api/v1/teams/{teamId}/leaderboard?metric=workouts&timeframe=weekly
Authorization: Bearer {access_token}
```

## 9. Instructor and Content Creator APIs

### 9.1 Get Instructor Profile

```http
GET /api/v1/instructors/{instructorId}
```

**Response:**
```json
{
  "id": "instructor-001",
  "name": "Coach Sarah",
  "specialties": ["vr-boxing", "hiit", "strength"],
  "certifications": ["NASM-CPT", "ACE-GFI"],
  "rating": 4.9,
  "totalStudents": 125000,
  "classes": 342,
  "biography": "10+ years experience...",
  "social": {
    "instagram": "@coach_sarah_fit",
    "youtube": "CoachSarahFitness"
  }
}
```

### 9.2 Get Instructor Schedule

```http
GET /api/v1/instructors/{instructorId}/schedule?start=2025-01-27&end=2025-02-03
Authorization: Bearer {access_token}
```

## 10. Biometric Data APIs

### 10.1 Upload Heart Rate Data

```http
POST /api/v1/biometrics/heartrate
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "sessionId": "550e8400-e29b-41d4-a716-446655440000",
  "source": "apple-watch",
  "data": [
    {"timestamp": 1706371200000, "bpm": 125},
    {"timestamp": 1706371205000, "bpm": 128}
  ]
}
```

### 10.2 Get Biometric History

```http
GET /api/v1/biometrics/history?type=heartrate&start=2025-01-20&end=2025-01-27
Authorization: Bearer {access_token}
Scope: biometrics:read
```

## 11. Challenge APIs

### 11.1 Get Active Challenges

```http
GET /api/v1/challenges/active
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "challenges": [
    {
      "id": "challenge-monthly-jan",
      "name": "January Fitness Challenge",
      "description": "Complete 20 workouts in January",
      "type": "individual",
      "startDate": "2025-01-01",
      "endDate": "2025-01-31",
      "goal": 20,
      "progress": 15,
      "reward": {
        "type": "badge",
        "name": "January Champion",
        "points": 1000
      },
      "participants": 45000
    }
  ]
}
```

### 11.2 Join Challenge

```http
POST /api/v1/challenges/{challengeId}/join
Authorization: Bearer {access_token}
```

## 12. Analytics and Insights APIs

### 12.1 Get Workout Statistics

```http
GET /api/v1/analytics/stats?period=monthly
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "period": "2025-01",
  "totalWorkouts": 24,
  "totalDuration": 28800,
  "totalCalories": 9600,
  "avgHeartRate": 135,
  "topWorkoutType": "vr-boxing",
  "streak": 15,
  "personalRecords": 3,
  "insights": [
    "Your workout frequency increased 25% this month!",
    "You're most consistent on Mondays and Wednesdays",
    "Average session duration: 20 minutes"
  ]
}
```

## 13. Webhook Integration

### 13.1 Register Webhook

```http
POST /api/v1/webhooks
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://your-app.com/webhooks/fitness",
  "events": ["workout.completed", "achievement.unlocked", "friend.request"],
  "secret": "your-webhook-secret"
}
```

### 13.2 Webhook Payload Example

```json
{
  "event": "workout.completed",
  "timestamp": "2025-01-27T10:30:00Z",
  "data": {
    "sessionId": "550e8400-e29b-41d4-a716-446655440000",
    "userId": "user-001",
    "workoutType": "vr-boxing",
    "duration": 1800,
    "calories": 450
  },
  "signature": "sha256-signature-here"
}
```

## 14. Rate Limiting

- **Free Tier**: 100 requests/minute, 10,000 requests/day
- **Premium Tier**: 1,000 requests/minute, 100,000 requests/day
- **Enterprise**: Custom limits

Headers:
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1706371260
```

## 15. Error Handling

Standard error response format:

```json
{
  "error": {
    "code": "INVALID_SESSION",
    "message": "Session not found or expired",
    "details": "SessionId 550e8400... does not exist",
    "timestamp": "2025-01-27T12:00:00Z",
    "requestId": "req-12345"
  }
}
```

Common error codes:
- `UNAUTHORIZED` (401)
- `FORBIDDEN` (403)
- `NOT_FOUND` (404)
- `RATE_LIMIT_EXCEEDED` (429)
- `INTERNAL_ERROR` (500)

---

**Document Version:** 1.0
**Status:** Draft for Public Comment
**Contact:** standards@wia.org
**License:** CC BY-SA 4.0

弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-014-virtual-fitness is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-IND-014-virtual-fitness/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-014-virtual-fitness/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-014-virtual-fitness/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
