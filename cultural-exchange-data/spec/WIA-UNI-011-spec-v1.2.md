# WIA-UNI-011: Cultural Exchange Data Standard
## Specification v1.2

**Status:** Released  
**Date:** 2025-07-01  
**Category:** Unification/Peace (UNI)  
**Changes from v1.1:** AI heritage recognition, enhanced search, mobile SDK

---

## Changelog from v1.1

### Added
- AI-powered heritage item recognition from photos
- GraphQL API alongside REST
- Mobile SDKs (iOS, Android)
- Blockchain-based provenance tracking
- Virtual Reality event documentation

### Changed
- Search algorithm now uses semantic embeddings
- Enhanced UNESCO thesaurus integration
- Improved conflict resolution for regional variations

### Fixed
- Edge cases in multilingual search
- Performance optimization for large media collections
- Sync protocol reliability improvements

---

## 1. AI Heritage Recognition

### 1.1 Photo Analysis

Upload photos of cultural heritage items for automatic identification.

```
POST /heritage/recognize
Content-Type: multipart/form-data

{
  "image": "binary data",
  "region": "both",
  "confidence_threshold": 0.75
}

Response:
{
  "matches": [
    {
      "heritageId": "HER-KR-MUSIC-001",
      "name": { "ko": "아리랑", "en": "Arirang" },
      "confidence": 0.92,
      "detectedFeatures": ["traditional costume", "musical instrument"]
    }
  ]
}
```

---

## 2. GraphQL API

### 2.1 Schema

```graphql
type CulturalEvent {
  eventId: ID!
  eventType: EventType!
  title: I18nString!
  description: I18nString
  startDate: DateTime!
  endDate: DateTime!
  location: Location!
  participants: [Participant!]
  media: [MediaAsset!]
}

type Query {
  events(filter: EventFilter, limit: Int, offset: Int): [CulturalEvent!]!
  event(id: ID!): CulturalEvent
  heritage(filter: HeritageFilter): [HeritageItem!]!
}

type Mutation {
  createEvent(input: CreateEventInput!): CulturalEvent!
  updateEvent(id: ID!, input: UpdateEventInput!): CulturalEvent!
}
```

---

## 3. Mobile SDKs

### 3.1 iOS SDK (Swift)

```swift
import WIACulturalExchange

let client = WIAClient(apiKey: "your-api-key")

// List events
let events = try await client.events.list(
    type: .music,
    startDate: Date(),
    limit: 20
)

// Create event
let event = try await client.events.create(
    eventType: .music,
    title: ["ko": "평화 콘서트", "en": "Peace Concert"],
    startDate: futureDate
)
```

### 3.2 Android SDK (Kotlin)

```kotlin
import org.wia.culturalexchange.*

val client = WIAClient("your-api-key")

// List events
val events = client.events.list(
    type = EventType.MUSIC,
    startDate = LocalDateTime.now(),
    limit = 20
)
```

---

## 4. Blockchain Provenance

### 4.1 Heritage Item Tracking

Track heritage item ownership, transfers, and restoration using blockchain.

```
POST /heritage/{id}/provenance
{
  "event": "restoration",
  "date": "2025-06-15",
  "organization": "National Heritage Service",
  "details": "Traditional painting restoration completed",
  "evidence": ["photo1.jpg", "report.pdf"]
}
```

Each provenance event is recorded on blockchain with immutable timestamp and cryptographic proof.

---

## 5. Virtual Reality Support

### 5.1 VR Event Documentation

```json
{
  "eventId": "UNI-011-VR-2025-001",
  "eventType": "heritage",
  "vrData": {
    "format": "WebXR",
    "scenes": [
      {
        "sceneId": "scene-001",
        "type": "360-video",
        "url": "https://vr.cultural-exchange.org/performance-001.mp4"
      }
    ],
    "interactiveElements": ["heritage-hotspots", "audio-guides"]
  }
}
```

---

**Document ID:** WIA-UNI-011-SPEC-v1.2  
**Previous Version:** v1.1  
**Next Version:** v2.0 (Planned: 2025-10-01)

© 2025 SmileStory Inc. / WIA · MIT License
