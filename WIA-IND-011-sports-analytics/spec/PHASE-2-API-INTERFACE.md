# WIA-IND-011 PHASE 2: API INTERFACE SPECIFICATION
## Sports Analytics Standard - RESTful API and Data Access Protocols

**Standard:** WIA-IND-011
**Phase:** 2 of 4
**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 of the WIA-IND-011 Sports Analytics Standard defines RESTful API endpoints, authentication protocols, and data access methodologies for sports analytics systems. This enables seamless integration between analytics platforms, databases, broadcasting systems, and third-party applications.

**Guiding Principle (弘益人間):** APIs should be accessible, well-documented, and designed to serve developers, organizations, and end-users equally.

---

## 2. Base API Architecture

### 2.1 Base URL Structure

```
https://api.sports-analytics.org/v1/{resource}

Examples:
https://api.sports-analytics.org/v1/players
https://api.sports-analytics.org/v1/matches
https://api.sports-analytics.org/v1/performance
```

### 2.2 Standard Headers

```http
X-WIA-Standard: IND-011
X-WIA-Version: 1.0
X-WIA-Philosophy: 弘益人間
Content-Type: application/json
Accept: application/json
Authorization: Bearer {token}
```

---

## 3. Player API Endpoints

### 3.1 Get Player Information

**Endpoint:** `GET /v1/players/{playerId}`

**Description:** Retrieve comprehensive player information

**Request:**
```http
GET /v1/players/P123456 HTTP/1.1
Host: api.sports-analytics.org
X-WIA-Standard: IND-011
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "status": "success",
  "data": {
    "playerId": "P123456",
    "personalInfo": {
      "firstName": "John",
      "lastName": "Athlete",
      "dateOfBirth": "1995-03-15",
      "nationality": "US"
    },
    "sportInfo": {
      "sport": "Soccer",
      "position": "Forward",
      "teamId": "T789",
      "jerseyNumber": 10
    },
    "careerStats": {
      "matches": 250,
      "goals": 87,
      "assists": 45
    }
  },
  "metadata": {
    "retrievedAt": "2025-01-15T14:30:00Z",
    "cacheValid": "300"
  }
}
```

### 3.2 Get Player Performance Data

**Endpoint:** `GET /v1/players/{playerId}/performance`

**Query Parameters:**
- `matchId` (optional): Filter by specific match
- `startDate` (optional): ISO8601 date
- `endDate` (optional): ISO8601 date
- `metrics` (optional): Comma-separated list of metrics

**Example:**
```http
GET /v1/players/P123456/performance?startDate=2025-01-01&endDate=2025-01-15&metrics=distance,speed,heartRate
```

**Response:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間 - Data-driven athlete development",
  "status": "success",
  "data": {
    "playerId": "P123456",
    "dateRange": {
      "start": "2025-01-01",
      "end": "2025-01-15"
    },
    "performances": [
      {
        "performanceId": "PERF_001",
        "date": "2025-01-05",
        "matchId": "M789012",
        "metrics": {
          "totalDistance": 11250.5,
          "topSpeed": 32.1,
          "avgHeartRate": 165
        }
      }
    ],
    "aggregates": {
      "avgDistance": 10750.3,
      "avgTopSpeed": 31.5
    }
  },
  "pagination": {
    "total": 15,
    "page": 1,
    "pageSize": 10,
    "hasMore": true
  }
}
```

### 3.3 List Players

**Endpoint:** `GET /v1/players`

**Query Parameters:**
- `teamId`: Filter by team
- `position`: Filter by position
- `nationality`: Filter by nationality
- `page`: Page number (default: 1)
- `pageSize`: Results per page (default: 20, max: 100)
- `sort`: Sort field (e.g., `lastName`, `-age` for descending)

---

## 4. Match API Endpoints

### 4.1 Get Match Details

**Endpoint:** `GET /v1/matches/{matchId}`

**Response:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "status": "success",
  "data": {
    "matchId": "M789012",
    "competition": {
      "name": "League Championship",
      "season": "2024-25"
    },
    "teams": {
      "home": {
        "teamId": "T789",
        "name": "Team A",
        "score": 2,
        "xG": 1.85
      },
      "away": {
        "teamId": "T790",
        "name": "Team B",
        "score": 1,
        "xG": 0.92
      }
    },
    "matchInfo": {
      "date": "2025-01-15",
      "kickoffTime": "2025-01-15T15:00:00Z",
      "venue": "Stadium Name",
      "attendance": 45000
    },
    "status": "completed"
  }
}
```

### 4.2 Get Match Events

**Endpoint:** `GET /v1/matches/{matchId}/events`

**Query Parameters:**
- `eventType`: Filter by type (pass, shot, tackle, etc.)
- `teamId`: Filter by team
- `playerId`: Filter by player
- `startTime`: Match clock start (seconds)
- `endTime`: Match clock end (seconds)

**Response:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "status": "success",
  "data": {
    "matchId": "M789012",
    "events": [
      {
        "eventId": "E001",
        "timestamp": "2025-01-15T15:12:30Z",
        "matchClock": 750,
        "eventType": "goal",
        "playerId": "P123456",
        "teamId": "T789",
        "location": {
          "x": 85.5,
          "y": 45.2
        },
        "xG": 0.35,
        "outcome": "successful"
      }
    ],
    "count": 157
  }
}
```

### 4.3 Get Live Match Data

**Endpoint:** `GET /v1/matches/{matchId}/live`

**Description:** Real-time match statistics (updates every 5 seconds)

**Response:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間 - Real-time insights for all",
  "status": "success",
  "matchState": "live",
  "data": {
    "matchId": "M789012",
    "currentMinute": 67,
    "score": {
      "home": 2,
      "away": 1
    },
    "liveStats": {
      "possession": {
        "home": 58,
        "away": 42
      },
      "shots": {
        "home": 12,
        "away": 7
      },
      "xG": {
        "home": 1.85,
        "away": 0.92
      }
    },
    "recentEvents": [
      {
        "eventId": "E145",
        "type": "shot",
        "minute": 67,
        "playerId": "P456789"
      }
    ]
  },
  "nextUpdate": "2025-01-15T16:07:35Z"
}
```

---

## 5. Statistics API Endpoints

### 5.1 Team Statistics

**Endpoint:** `GET /v1/stats/teams/{teamId}`

**Query Parameters:**
- `season`: Season identifier
- `competition`: Competition ID
- `metric`: Specific metric (e.g., possession, xG, distance)

### 5.2 League Standings

**Endpoint:** `GET /v1/stats/leagues/{leagueId}/standings`

### 5.3 Advanced Analytics

**Endpoint:** `POST /v1/analytics/query`

**Description:** Custom analytics queries using WIA Query Language

**Request:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "query": {
    "select": ["playerId", "AVG(distance)", "AVG(topSpeed)"],
    "from": "performance",
    "where": {
      "teamId": "T789",
      "date": {"gte": "2025-01-01", "lte": "2025-01-31"}
    },
    "groupBy": ["playerId"],
    "orderBy": [{"field": "AVG(distance)", "direction": "DESC"}],
    "limit": 10
  }
}
```

---

## 6. Authentication and Authorization

### 6.1 OAuth 2.0 Authentication

**Token Endpoint:** `POST /v1/auth/token`

**Request:**
```http
POST /v1/auth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=read:players read:matches write:analytics
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read:players read:matches write:analytics",
  "philosophy": "弘益人間 - Secure access for all authorized users"
}
```

### 6.2 API Key Authentication

**Alternative Method:**
```http
GET /v1/players HTTP/1.1
X-API-Key: wia_live_1234567890abcdef
```

### 6.3 Permission Scopes

| Scope | Description |
|-------|-------------|
| `read:players` | Read player information and statistics |
| `read:matches` | Read match data and events |
| `read:performance` | Access performance metrics |
| `write:analytics` | Submit analytics queries |
| `read:live` | Access real-time data feeds |
| `admin:all` | Full administrative access |

---

## 7. Rate Limiting

### 7.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642348800
```

### 7.2 Tier-Based Limits

| Tier | Requests/Hour | Live Data Access |
|------|---------------|------------------|
| Free | 100 | No |
| Developer | 1,000 | Yes (1 match) |
| Professional | 10,000 | Yes (5 matches) |
| Enterprise | Unlimited | Yes (unlimited) |

### 7.3 Rate Limit Exceeded Response

```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 2025-01-15T17:00:00Z",
    "retryAfter": 1800
  }
}
```

---

## 8. WebSocket API (Real-Time Streaming)

### 8.1 Connection

```javascript
const ws = new WebSocket('wss://api.sports-analytics.org/v1/stream');

ws.onopen = () => {
  // Subscribe to match updates
  ws.send(JSON.stringify({
    action: 'subscribe',
    channels: ['match:M789012', 'player:P123456'],
    auth: 'Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('弘益人間 - Real-time update:', data);
};
```

### 8.2 Message Format

```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "channel": "match:M789012",
  "type": "event",
  "timestamp": "2025-01-15T16:12:30.500Z",
  "data": {
    "eventId": "E156",
    "eventType": "goal",
    "playerId": "P123456",
    "minute": 72
  }
}
```

---

## 9. Error Handling

### 9.1 Standard Error Response

```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "status": "error",
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Player with ID P999999 not found",
    "timestamp": "2025-01-15T16:30:00Z",
    "requestId": "req_abc123",
    "documentation": "https://docs.wia.org/errors/RESOURCE_NOT_FOUND"
  }
}
```

### 9.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request syntax |
| `UNAUTHORIZED` | 401 | Authentication required |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `RESOURCE_NOT_FOUND` | 404 | Resource doesn't exist |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |
| `SERVICE_UNAVAILABLE` | 503 | Temporary unavailability |

---

## 10. Pagination

### 10.1 Cursor-Based Pagination

**Request:**
```http
GET /v1/matches?pageSize=20&cursor=eyJsYXN0SWQiOiJNNzg5MDEyIn0=
```

**Response:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "data": [...],
  "pagination": {
    "pageSize": 20,
    "hasMore": true,
    "nextCursor": "eyJsYXN0SWQiOiJNNzg5MDMyIn0=",
    "total": 157
  }
}
```

---

## 11. Webhooks

### 11.1 Configuration

**Endpoint:** `POST /v1/webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhooks/wia",
  "events": ["match.started", "match.ended", "player.injured"],
  "secret": "your_webhook_secret"
}
```

### 11.2 Payload Example

```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "event": "match.ended",
  "timestamp": "2025-01-15T17:45:00Z",
  "data": {
    "matchId": "M789012",
    "finalScore": {
      "home": 2,
      "away": 1
    }
  },
  "signature": "sha256=abc123..."
}
```

---

## 12. SDK Examples

### 12.1 JavaScript/TypeScript

```typescript
import { WIASportsAnalytics } from '@wia/sports-analytics';

const client = new WIASportsAnalytics({
  apiKey: 'wia_live_1234567890abcdef',
  philosophy: '弘益人間'
});

// Get player data
const player = await client.players.get('P123456');

// Get live match data
const liveMatch = await client.matches.getLive('M789012');

// Subscribe to real-time updates
client.stream.subscribe('match:M789012', (event) => {
  console.log('Live event:', event);
});
```

### 12.2 Python

```python
from wia_sports_analytics import WIAClient

client = WIAClient(
    api_key='wia_live_1234567890abcdef',
    philosophy='弘益人間'
)

# Get player performance
performance = client.players.get_performance(
    player_id='P123456',
    start_date='2025-01-01',
    end_date='2025-01-31'
)

# Query analytics
results = client.analytics.query({
    'select': ['playerId', 'AVG(distance)'],
    'from': 'performance',
    'groupBy': ['playerId']
})
```

---

## 13. API Versioning

- Current Version: `v1`
- Version in URL: `/v1/`
- Deprecation Notice: 6 months minimum
- Sunset Period: 12 months minimum

---

## 14. Support and Documentation

- API Documentation: https://docs.wia.org/IND-011/api
- OpenAPI Spec: https://api.sports-analytics.org/v1/openapi.json
- Support: api-support@wia.org
- Status Page: https://status.wia.org

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA Technical Committee
**Philosophy:** 弘益人間 - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
Licensed under MIT License
