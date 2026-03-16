# WIA-EDU-022: Technical Specification

> **弘益人間** (Benefit All Humanity)

## Data Models

### Program Model

```json
{
  "id": "string (UUID)",
  "standard": "WIA-EDU-022",
  "version": "1.0.0",
  "institution": {
    "id": "string",
    "name": "string",
    "type": "middle_school | high_school | college",
    "location": {
      "country": "string (ISO 3166-1)",
      "state": "string",
      "city": "string"
    }
  },
  "program": {
    "name": "string",
    "type": "club | class | varsity | hybrid",
    "status": "planning | active | suspended | archived",
    "startDate": "string (ISO 8601)",
    "games": ["string"],
    "gradeLevels": {
      "min": "number (6-16)",
      "max": "number (6-16)"
    },
    "learningObjectives": ["string"],
    "codeOfConduct": "string (URL to document)"
  },
  "staff": [
    {
      "id": "string",
      "role": "director | head_coach | assistant_coach | advisor",
      "name": "string",
      "contact": "string (email)"
    }
  ],
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)",
    "certificationLevel": "bronze | silver | gold | platinum | null"
  }
}
```

### Team Model

```json
{
  "id": "string (UUID)",
  "programId": "string (UUID, ref to Program)",
  "name": "string",
  "game": "string",
  "tier": "varsity | jv | novice | practice",
  "roster": {
    "starters": ["string (player IDs)"],
    "substitutes": ["string (player IDs)"],
    "coaches": ["string (staff IDs)"]
  },
  "season": {
    "year": "number",
    "league": "string",
    "division": "string"
  },
  "record": {
    "wins": "number",
    "losses": "number",
    "ties": "number",
    "tournamentPlacements": [
      {
        "tournament": "string",
        "placement": "number",
        "date": "string (ISO 8601)"
      }
    ]
  },
  "practice": {
    "weeklyHours": "number",
    "schedule": [
      {
        "day": "monday | tuesday | ...",
        "startTime": "string (HH:MM)",
        "duration": "number (minutes)"
      }
    ]
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

### Player Model

```json
{
  "id": "string (UUID)",
  "standard": "WIA-EDU-022",
  "personal": {
    "studentId": "string (institutional ID)",
    "gamerTag": "string",
    "grade": "number (6-16)",
    "enrollmentYear": "number"
  },
  "teamAssignments": [
    {
      "teamId": "string (UUID)",
      "role": "starter | substitute | captain | practice",
      "position": "string (game-specific role)",
      "joinedDate": "string (ISO 8601)",
      "status": "active | inactive | graduated"
    }
  ],
  "eligibility": {
    "academicStanding": "boolean",
    "conductStanding": "boolean",
    "attendanceRequirement": "boolean",
    "lastVerified": "string (ISO 8601)"
  },
  "performance": {
    "individualStats": {
      "gamesPlayed": "number",
      "winRate": "number (0-100)",
      "skillRating": "number",
      "improvementRate": "number"
    },
    "teamContributions": {
      "communicationRating": "number (1-10)",
      "teamworkRating": "number (1-10)",
      "leadershipRating": "number (1-10)"
    }
  },
  "wellness": {
    "screenTimeWeekly": "number (hours)",
    "physicalActivityWeekly": "number (hours)",
    "sleepAverageHours": "number",
    "lastWellnessCheck": "string (ISO 8601)"
  },
  "consent": {
    "parentalConsent": "boolean",
    "dataSharing": "boolean",
    "mediaRelease": "boolean",
    "consentDate": "string (ISO 8601)"
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

### Match Model

```json
{
  "id": "string (UUID)",
  "type": "scrimmage | league | tournament | championship",
  "teamId": "string (UUID)",
  "opponent": {
    "teamId": "string (UUID) | null",
    "name": "string",
    "institution": "string"
  },
  "schedule": {
    "date": "string (ISO 8601)",
    "location": "home | away | neutral | online",
    "venue": "string"
  },
  "roster": {
    "starters": ["string (player IDs)"],
    "substitutes": ["string (player IDs)"]
  },
  "result": {
    "score": {
      "team": "number",
      "opponent": "number"
    },
    "outcome": "win | loss | tie | cancelled",
    "duration": "number (minutes)",
    "forfeit": "boolean"
  },
  "analysis": {
    "vod": "string (URL)",
    "stats": "object (game-specific stats)",
    "coachNotes": "string",
    "playerReflections": ["string"]
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

### Career Pathway Model

```json
{
  "id": "string (UUID)",
  "playerId": "string (UUID)",
  "interests": ["player | coach | content_creator | analyst | ..."],
  "experiences": [
    {
      "type": "competition | internship | workshop | mentorship",
      "title": "string",
      "organization": "string",
      "dateRange": {
        "start": "string (ISO 8601)",
        "end": "string (ISO 8601) | null"
      },
      "description": "string",
      "skills": ["string"]
    }
  ],
  "achievements": [
    {
      "type": "award | certification | scholarship | recognition",
      "title": "string",
      "issuer": "string",
      "date": "string (ISO 8601)",
      "verificationUrl": "string (URL) | null"
    }
  ],
  "goals": [
    {
      "category": "short_term | medium_term | long_term",
      "description": "string",
      "targetDate": "string (ISO 8601) | null",
      "status": "pending | in_progress | achieved | revised",
      "milestones": ["string"]
    }
  ],
  "portfolio": {
    "streamingChannels": ["string (URLs)"],
    "contentSamples": ["string (URLs)"],
    "resume": "string (URL)",
    "personalWebsite": "string (URL) | null"
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

## Data Privacy & Security

### Personal Information Protection

1. **PII Handling**
   - Student names, addresses, contact information must be encrypted at rest
   - Transmission over HTTPS/TLS 1.3 only
   - Access restricted to authorized personnel only
   - Audit logging for all PII access

2. **Data Minimization**
   - Collect only data necessary for program operation
   - Anonymous identifiers used where possible
   - Retention policies: delete data after graduation + 2 years

3. **Parental Consent**
   - Required for students under 18
   - Granular consent for data sharing, media release, research
   - Revocable at any time with 30-day grace period

4. **Compliance**
   - FERPA (Family Educational Rights and Privacy Act)
   - COPPA (Children's Online Privacy Protection Act)
   - GDPR (General Data Protection Regulation) where applicable
   - State-specific privacy laws (CCPA, etc.)

### Access Control

```json
{
  "roles": [
    {
      "role": "student_player",
      "permissions": ["read:own_profile", "update:own_preferences"]
    },
    {
      "role": "parent_guardian",
      "permissions": ["read:own_student", "update:consent"]
    },
    {
      "role": "coach",
      "permissions": ["read:team_players", "update:team_roster", "create:matches"]
    },
    {
      "role": "program_director",
      "permissions": ["read:all_program_data", "update:program", "delete:archived_data"]
    },
    {
      "role": "system_admin",
      "permissions": ["*"]
    }
  ]
}
```

### Data Retention

| Data Type | Retention Period | After Retention |
|-----------|------------------|-----------------|
| Student records | Graduation + 2 years | Deleted or anonymized |
| Match statistics | Indefinite (anonymized) | Archived |
| Practice logs | Current year + 1 year | Deleted |
| Consent forms | Until revoked + 7 years | Securely archived |
| Health/wellness data | Current year only | Deleted |

## Integration Standards

### Authentication

- OAuth 2.0 for third-party integrations
- SAML 2.0 for institutional SSO
- API keys for server-to-server communication
- JWT tokens for session management

### API Versioning

- Semantic versioning (MAJOR.MINOR.PATCH)
- API version in URL: `/api/v1/programs`
- Backward compatibility maintained for MINOR versions
- Deprecation notices 6 months before MAJOR changes

### Rate Limiting

- 1000 requests per hour per API key
- 100 requests per minute for burst traffic
- 429 Too Many Requests response with Retry-After header
- Higher limits available for certified partners

### Error Responses

```json
{
  "error": {
    "code": "string (ERROR_CODE)",
    "message": "string (human-readable)",
    "details": "object (additional context)",
    "timestamp": "string (ISO 8601)",
    "requestId": "string (UUID for support)"
  }
}
```

---

© 2025 WIA - World Certification Industry Association
Licensed under MIT License

**弘益人間** · Benefit All Humanity
