# WIA-EDU-022: API Reference

> **弘益人間** (Benefit All Humanity)

## Base URL

```
Production: https://api.wia-esports.edu/v1
Sandbox: https://sandbox-api.wia-esports.edu/v1
```

## Authentication

All API requests require authentication using Bearer tokens:

```http
Authorization: Bearer <your_access_token>
```

### Obtaining Access Token

```http
POST /auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "programs:read programs:write teams:read teams:write"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "programs:read programs:write teams:read teams:write"
}
```

## Programs API

### List Programs

```http
GET /programs
```

**Query Parameters:**
- `institutionId` (optional): Filter by institution
- `status` (optional): Filter by status (active, planning, suspended, archived)
- `page` (optional): Page number (default: 1)
- `limit` (optional): Results per page (default: 20, max: 100)

**Response:**
```json
{
  "data": [
    {
      "id": "prog_123456",
      "standard": "WIA-EDU-022",
      "institution": {
        "id": "inst_789",
        "name": "Lincoln High School",
        "type": "high_school"
      },
      "program": {
        "name": "Varsity Esports Program",
        "type": "varsity",
        "status": "active",
        "games": ["League of Legends", "Rocket League"]
      }
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 150,
    "totalPages": 8
  }
}
```

### Get Program

```http
GET /programs/{programId}
```

**Response:** Full program object (see Technical Specification)

### Create Program

```http
POST /programs
Content-Type: application/json

{
  "institution": {
    "id": "inst_789",
    "name": "Lincoln High School",
    "type": "high_school",
    "location": {
      "country": "US",
      "state": "CA",
      "city": "San Francisco"
    }
  },
  "program": {
    "name": "Varsity Esports Program",
    "type": "varsity",
    "status": "planning",
    "games": ["Rocket League"],
    "gradeLevels": {"min": 9, "max": 12},
    "learningObjectives": [
      "Develop teamwork and communication skills",
      "Build strategic thinking abilities"
    ]
  }
}
```

**Response:** 201 Created with program object

### Update Program

```http
PATCH /programs/{programId}
Content-Type: application/json

{
  "program": {
    "status": "active",
    "games": ["Rocket League", "Valorant"]
  }
}
```

**Response:** 200 OK with updated program object

### Delete Program

```http
DELETE /programs/{programId}
```

**Response:** 204 No Content

## Teams API

### List Teams

```http
GET /teams?programId={programId}
```

**Query Parameters:**
- `programId` (required): Program ID
- `game` (optional): Filter by game
- `tier` (optional): Filter by tier (varsity, jv, novice)
- `season` (optional): Filter by season year

**Response:**
```json
{
  "data": [
    {
      "id": "team_abc123",
      "programId": "prog_123456",
      "name": "Dragons Varsity",
      "game": "Rocket League",
      "tier": "varsity",
      "roster": {
        "starters": ["player_001", "player_002", "player_003"],
        "substitutes": ["player_004"]
      },
      "record": {
        "wins": 12,
        "losses": 5,
        "ties": 1
      }
    }
  ]
}
```

### Get Team

```http
GET /teams/{teamId}
```

### Create Team

```http
POST /teams
Content-Type: application/json

{
  "programId": "prog_123456",
  "name": "Dragons Varsity",
  "game": "Rocket League",
  "tier": "varsity",
  "season": {
    "year": 2025,
    "league": "PlayVS",
    "division": "California - Division 1"
  }
}
```

**Response:** 201 Created with team object

### Update Team Roster

```http
PATCH /teams/{teamId}/roster
Content-Type: application/json

{
  "starters": ["player_001", "player_002", "player_003"],
  "substitutes": ["player_004", "player_005"]
}
```

**Response:** 200 OK with updated team object

## Players API

### List Players

```http
GET /players?programId={programId}
```

**Query Parameters:**
- `programId` (required): Program ID
- `teamId` (optional): Filter by team
- `grade` (optional): Filter by grade level
- `status` (optional): Filter by status (active, inactive, graduated)

### Get Player

```http
GET /players/{playerId}
```

### Create Player

```http
POST /players
Content-Type: application/json

{
  "personal": {
    "studentId": "STU123456",
    "gamerTag": "ProGamer2025",
    "grade": 10,
    "enrollmentYear": 2023
  },
  "consent": {
    "parentalConsent": true,
    "dataSharing": true,
    "mediaRelease": false,
    "consentDate": "2025-01-15T00:00:00Z"
  }
}
```

**Response:** 201 Created with player object

### Update Player

```http
PATCH /players/{playerId}
Content-Type: application/json

{
  "performance": {
    "individualStats": {
      "gamesPlayed": 15,
      "winRate": 68.5,
      "skillRating": 1850
    }
  }
}
```

### Assign Player to Team

```http
POST /players/{playerId}/teams
Content-Type: application/json

{
  "teamId": "team_abc123",
  "role": "starter",
  "position": "midfielder",
  "joinedDate": "2025-01-20T00:00:00Z"
}
```

**Response:** 200 OK with updated player object

## Matches API

### List Matches

```http
GET /matches?teamId={teamId}
```

**Query Parameters:**
- `teamId` (required): Team ID
- `type` (optional): Filter by match type
- `startDate` (optional): Filter matches after date
- `endDate` (optional): Filter matches before date

### Get Match

```http
GET /matches/{matchId}
```

### Create Match

```http
POST /matches
Content-Type: application/json

{
  "type": "league",
  "teamId": "team_abc123",
  "opponent": {
    "name": "Riverside Warriors",
    "institution": "Riverside High School"
  },
  "schedule": {
    "date": "2025-02-15T18:00:00Z",
    "location": "online",
    "venue": "PlayVS Platform"
  },
  "roster": {
    "starters": ["player_001", "player_002", "player_003"]
  }
}
```

**Response:** 201 Created with match object

### Update Match Result

```http
PATCH /matches/{matchId}/result
Content-Type: application/json

{
  "score": {
    "team": 3,
    "opponent": 1
  },
  "outcome": "win",
  "duration": 45,
  "analysis": {
    "vod": "https://youtube.com/watch?v=...",
    "coachNotes": "Great teamwork and communication"
  }
}
```

**Response:** 200 OK with updated match object

## Career Pathways API

### Get Player Career Profile

```http
GET /players/{playerId}/career
```

### Update Career Interests

```http
PATCH /players/{playerId}/career/interests
Content-Type: application/json

{
  "interests": ["content_creator", "analyst", "coach"]
}
```

### Add Experience

```http
POST /players/{playerId}/career/experiences
Content-Type: application/json

{
  "type": "internship",
  "title": "Content Creator Intern",
  "organization": "Cloud9 Esports",
  "dateRange": {
    "start": "2025-06-01T00:00:00Z",
    "end": "2025-08-15T00:00:00Z"
  },
  "description": "Created social media content for competitive team",
  "skills": ["video editing", "social media marketing", "content strategy"]
}
```

### Add Achievement

```http
POST /players/{playerId}/career/achievements
Content-Type: application/json

{
  "type": "scholarship",
  "title": "UC Irvine Esports Scholarship",
  "issuer": "University of California, Irvine",
  "date": "2025-03-01T00:00:00Z",
  "amount": 15000
}
```

## Webhooks

Subscribe to real-time events:

```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://your-app.com/webhooks/esports",
  "events": ["match.completed", "player.joined", "team.updated"],
  "secret": "your_webhook_secret"
}
```

**Event Payload:**
```json
{
  "event": "match.completed",
  "timestamp": "2025-02-15T20:30:00Z",
  "data": {
    "matchId": "match_xyz789",
    "teamId": "team_abc123",
    "result": "win",
    "score": {"team": 3, "opponent": 1}
  },
  "signature": "sha256=..."
}
```

## Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| UNAUTHORIZED | 401 | Invalid or expired authentication token |
| FORBIDDEN | 403 | Insufficient permissions for requested resource |
| NOT_FOUND | 404 | Resource does not exist |
| VALIDATION_ERROR | 422 | Request data validation failed |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests, retry after specified time |
| INTERNAL_ERROR | 500 | Server error, contact support with requestId |

## SDK Examples

### TypeScript/JavaScript

```typescript
import { createClient } from '@wia/esports-sdk';

const client = createClient({
  apiKey: 'your_api_key',
  environment: 'production'
});

// List programs
const programs = await client.programs.list({
  status: 'active',
  limit: 50
});

// Create team
const team = await client.teams.create({
  programId: 'prog_123456',
  name: 'Dragons Varsity',
  game: 'Rocket League',
  tier: 'varsity'
});

// Update match result
await client.matches.updateResult('match_xyz789', {
  score: { team: 3, opponent: 1 },
  outcome: 'win'
});
```

### Python

```python
from wia_esports import Client

client = Client(api_key='your_api_key')

# List players
players = client.players.list(program_id='prog_123456', status='active')

# Add player experience
client.players.career.add_experience(
    player_id='player_001',
    experience={
        'type': 'workshop',
        'title': 'Esports Broadcasting Workshop',
        'organization': 'NASEF',
        'date_range': {'start': '2025-04-10', 'end': '2025-04-12'},
        'skills': ['commentary', 'obs studio', 'production']
    }
)
```

---

© 2025 WIA - World Certification Industry Association
Licensed under MIT License

**弘益人間** · Benefit All Humanity
