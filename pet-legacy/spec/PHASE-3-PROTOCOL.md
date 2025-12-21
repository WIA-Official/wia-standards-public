# WIA PET-LEGACY PHASE 3: PROTOCOL SPECIFICATION

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [Data Exchange Protocols](#data-exchange-protocols)
4. [Synchronization Protocol](#synchronization-protocol)
5. [Sharing Protocol](#sharing-protocol)
6. [Memorial Portability Protocol](#memorial-portability-protocol)
7. [Cemetery Integration Protocol](#cemetery-integration-protocol)
8. [Grief Support Protocol](#grief-support-protocol)
9. [Community Protocol](#community-protocol)
10. [Security Protocols](#security-protocols)

---

## 1. Overview

### 1.1 Purpose

The WIA PET-LEGACY Phase 3 specification defines comprehensive protocols for communication, data exchange, and interoperability between memorial platforms, veterinary systems, pet cemeteries, crematoriums, and third-party services. These protocols ensure seamless, secure, and ethical handling of pet memorial data across the ecosystem.

### 1.2 Protocol Stack

```
┌─────────────────────────────────────────────────┐
│         Application Layer                       │
│  Memorial Management, Media Sharing, AI         │
├─────────────────────────────────────────────────┤
│         Protocol Layer                          │
│  PET-LEGACY Protocol Suite                      │
├─────────────────────────────────────────────────┤
│         Transport Layer                         │
│  HTTPS, WebSocket, gRPC                         │
├─────────────────────────────────────────────────┤
│         Network Layer                           │
│  TCP/IP, TLS 1.3                                │
└─────────────────────────────────────────────────┘
```

### 1.3 Protocol Categories

| Category | Protocol | Purpose |
|----------|----------|---------|
| **Communication** | WebSocket, Server-Sent Events | Real-time updates |
| **Data Exchange** | REST, GraphQL, gRPC | API communication |
| **Synchronization** | Event Sourcing, CQRS | Data consistency |
| **Sharing** | OAuth 2.0, Deep Links | Social sharing |
| **Portability** | Export/Import, Migration | Platform switching |
| **Integration** | Webhooks, APIs | Third-party services |
| **Security** | TLS, JWT, Encryption | Data protection |

### 1.4 Protocol Design Principles

| Principle | Description | Implementation |
|-----------|-------------|----------------|
| **Resilience** | Handle network failures gracefully | Retry logic, circuit breakers |
| **Idempotency** | Same request produces same result | Idempotency keys |
| **Versioning** | Support protocol evolution | Version negotiation |
| **Extensibility** | Allow protocol extensions | Plugin architecture |
| **Privacy** | Protect sensitive information | End-to-end encryption |

---

## 2. Communication Protocols

### 2.1 Real-Time Updates via WebSocket

#### 2.1.1 Connection Establishment

```
Client                                Server
  │                                     │
  │  WSS Connection Request             │
  ├────────────────────────────────────>│
  │  wss://ws.petlegacy.wia.org/v1      │
  │                                     │
  │  Connection Accepted                │
  │<────────────────────────────────────┤
  │  (WebSocket Handshake Complete)     │
  │                                     │
  │  Authentication Message             │
  ├────────────────────────────────────>│
  │  {"type":"auth","token":"..."}      │
  │                                     │
  │  Authentication Success             │
  │<────────────────────────────────────┤
  │  {"type":"auth_success"}            │
  │                                     │
  │  Subscribe to Memorial              │
  ├────────────────────────────────────>│
  │  {"type":"subscribe",               │
  │   "channel":"memorial.123"}         │
  │                                     │
  │  Subscription Confirmed             │
  │<────────────────────────────────────┤
  │                                     │
```

#### 2.1.2 WebSocket Message Format

```json
{
  "version": "1.0",
  "messageId": "msg_9k2m5n7p8q",
  "timestamp": "2024-12-18T16:30:00Z",
  "type": "event",
  "channel": "memorial.550e8400-e29b-41d4-a716-446655440000",
  "event": "media.uploaded",
  "data": {
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "type": "photo",
    "uploadedBy": "Sarah Johnson",
    "title": "New memory added"
  },
  "metadata": {
    "priority": "normal",
    "requiresAck": false
  }
}
```

#### 2.1.3 Event Types

| Event Type | Description | Example Payload |
|------------|-------------|-----------------|
| `memorial.updated` | Memorial profile modified | Changed fields |
| `media.uploaded` | New media asset added | Asset metadata |
| `media.processed` | Media processing complete | Asset URLs |
| `timeline.event_created` | New timeline event | Event details |
| `comment.added` | New comment posted | Comment content |
| `family.member_joined` | New family member | Member info |
| `presence.online` | User came online | User ID |
| `presence.offline` | User went offline | User ID |

#### 2.1.4 Presence Protocol

```json
{
  "type": "presence.update",
  "channel": "memorial.550e8400",
  "users": [
    {
      "userId": "user_123",
      "displayName": "Sarah Johnson",
      "status": "online",
      "lastSeen": "2024-12-18T16:32:00Z",
      "currentActivity": "viewing_timeline"
    },
    {
      "userId": "user_456",
      "displayName": "Michael Johnson",
      "status": "idle",
      "lastSeen": "2024-12-18T16:25:00Z"
    }
  ]
}
```

### 2.2 Server-Sent Events (SSE)

For clients that don't support WebSocket:

```http
GET /v1/memorials/550e8400/events
Accept: text/event-stream
Authorization: Bearer {token}
```

**Server Response:**
```
event: media.uploaded
id: evt_001
data: {"assetId":"770e8400","type":"photo"}

event: comment.added
id: evt_002
data: {"commentId":"cmt_123","author":"Sarah"}

event: heartbeat
id: evt_003
data: {"timestamp":"2024-12-18T16:35:00Z"}
```

### 2.3 Push Notifications

#### 2.3.1 Push Notification Payload

```json
{
  "notification": {
    "title": "New memory added to Max's memorial",
    "body": "Michael Johnson uploaded a new photo",
    "icon": "https://cdn.petlegacy.wia.org/icons/photo.png",
    "badge": "https://cdn.petlegacy.wia.org/badge.png",
    "sound": "gentle_chime.mp3",
    "tag": "memorial_update",
    "requireInteraction": false
  },
  "data": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "media.uploaded",
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "deepLink": "petlegacy://memorial/550e8400/media/770e8400"
  },
  "platform": {
    "ios": {
      "badge": 1,
      "sound": "default",
      "category": "MEMORIAL_UPDATE"
    },
    "android": {
      "channelId": "memorial_updates",
      "priority": "high"
    }
  }
}
```

#### 2.3.2 Notification Preferences

| Notification Type | Default | Can Disable | Frequency Limit |
|-------------------|---------|-------------|-----------------|
| New media uploaded | On | Yes | Batched hourly |
| Timeline event added | On | Yes | Immediate |
| Comment on your post | On | Yes | Immediate |
| Family member joined | On | Yes | Immediate |
| Memorial anniversary | On | No | Once per year |
| Tribute received | On | Yes | Immediate |
| Export completed | On | No | Immediate |

---

## 3. Data Exchange Protocols

### 3.1 GraphQL Protocol

#### 3.1.1 GraphQL Endpoint

```http
POST /v1/graphql
Content-Type: application/json
Authorization: Bearer {token}
```

#### 3.1.2 Example Query

```graphql
query GetMemorialWithTimeline($memorialId: ID!) {
  memorial(id: $memorialId) {
    profileId
    petIdentity {
      name
      species
      breed
      birthDate
      passingDate
    }
    timeline(
      orderBy: DATE_DESC
      first: 10
    ) {
      edges {
        node {
          eventId
          eventType
          date
          title
          attachedMedia {
            assetId
            type
            thumbnailUrl
          }
        }
      }
      pageInfo {
        hasNextPage
        endCursor
      }
    }
    statistics {
      totalPhotos
      totalVideos
      totalEvents
    }
  }
}
```

#### 3.1.3 Example Mutation

```graphql
mutation CreateTimelineEvent($input: TimelineEventInput!) {
  createTimelineEvent(input: $input) {
    eventId
    title
    date
    createdAt
    errors {
      field
      message
    }
  }
}
```

**Variables:**
```json
{
  "input": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "CELEBRATION",
    "date": "2020-05-20T15:00:00Z",
    "title": "Max's 8th Birthday",
    "description": "Celebrated with cake and treats",
    "attachedMedia": ["770e8400-e29b-41d4-a716-446655440002"]
  }
}
```

#### 3.1.4 GraphQL Subscriptions

```graphql
subscription MemorialUpdates($memorialId: ID!) {
  memorialUpdated(memorialId: $memorialId) {
    updateType
    timestamp
    updatedBy {
      userId
      displayName
    }
    changes {
      field
      oldValue
      newValue
    }
  }
}
```

### 3.2 gRPC Protocol

#### 3.2.1 Protocol Buffer Definition

```protobuf
syntax = "proto3";

package petlegacy.v1;

service MemorialService {
  rpc GetMemorial(GetMemorialRequest) returns (Memorial) {}
  rpc CreateMemorial(CreateMemorialRequest) returns (Memorial) {}
  rpc UpdateMemorial(UpdateMemorialRequest) returns (Memorial) {}
  rpc DeleteMemorial(DeleteMemorialRequest) returns (DeleteMemorialResponse) {}
  rpc StreamMemorialUpdates(StreamRequest) returns (stream MemorialUpdate) {}
}

message Memorial {
  string profile_id = 1;
  PetIdentity pet_identity = 2;
  MemorialStatus memorial_status = 3;
  int64 created_at = 4;
  int64 last_modified = 5;
}

message PetIdentity {
  string name = 1;
  string species = 2;
  string breed = 3;
  string gender = 4;
  string birth_date = 5;
  string passing_date = 6;
}

message GetMemorialRequest {
  string memorial_id = 1;
  repeated string include = 2;
}

message CreateMemorialRequest {
  PetIdentity pet_identity = 1;
  GuardianInfo guardian_info = 2;
}
```

#### 3.2.2 gRPC Streaming

```go
// Server-side streaming
stream, err := client.StreamMemorialUpdates(ctx, &StreamRequest{
    MemorialId: "550e8400-e29b-41d4-a716-446655440000",
})

for {
    update, err := stream.Recv()
    if err == io.EOF {
        break
    }
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Received update: %v\n", update)
}
```

### 3.3 Message Queue Protocol

#### 3.3.1 Event Publishing

```json
{
  "messageId": "msg_5k7m9n2p4q",
  "topic": "petlegacy.memorial.events",
  "partition": 0,
  "timestamp": "2024-12-18T16:45:00Z",
  "headers": {
    "eventType": "media.uploaded",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "version": "1.0"
  },
  "payload": {
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "type": "photo",
    "uploadedBy": "660e8400-e29b-41d4-a716-446655440001",
    "uploadedAt": "2024-12-18T16:45:00Z"
  }
}
```

#### 3.3.2 Consumer Groups

| Consumer Group | Topic | Purpose |
|----------------|-------|---------|
| `analytics-processors` | memorial.events | Generate analytics |
| `notification-senders` | memorial.events | Send push notifications |
| `ai-processors` | media.events | Process images with AI |
| `search-indexers` | memorial.events | Update search index |
| `backup-workers` | memorial.events | Backup data changes |

---

## 4. Synchronization Protocol

### 4.1 Event Sourcing

#### 4.1.1 Event Store Structure

```json
{
  "eventId": "evt_9k2m5n7p8q",
  "aggregateId": "550e8400-e29b-41d4-a716-446655440000",
  "aggregateType": "Memorial",
  "eventType": "MemorialCreated",
  "version": 1,
  "timestamp": "2024-03-15T10:30:00Z",
  "userId": "660e8400-e29b-41d4-a716-446655440001",
  "data": {
    "petIdentity": {
      "name": "Max",
      "species": "dog",
      "breed": "Golden Retriever"
    }
  },
  "metadata": {
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0...",
    "correlationId": "cor_5x7k9m2n4p"
  }
}
```

#### 4.1.2 Event Types

| Event Type | Description | Triggers |
|------------|-------------|----------|
| `MemorialCreated` | New memorial initialized | Create memorial |
| `MemorialUpdated` | Memorial modified | Update any field |
| `MediaAdded` | Media asset uploaded | Upload media |
| `MediaRemoved` | Media asset deleted | Delete media |
| `TimelineEventAdded` | Event added to timeline | Create event |
| `FamilyMemberInvited` | Invitation sent | Invite member |
| `FamilyMemberJoined` | Member accepted | Accept invitation |
| `CommentPosted` | Comment added | Post comment |
| `MemorialPublished` | Made public | Publish memorial |
| `MemorialArchived` | Archived | Archive memorial |

#### 4.1.3 Event Replay

```javascript
function replayEvents(events) {
  let memorial = createEmptyMemorial();

  for (const event of events) {
    switch (event.eventType) {
      case 'MemorialCreated':
        memorial = applyMemorialCreated(memorial, event);
        break;
      case 'MediaAdded':
        memorial = applyMediaAdded(memorial, event);
        break;
      case 'TimelineEventAdded':
        memorial = applyTimelineEventAdded(memorial, event);
        break;
      // ... other event handlers
    }
  }

  return memorial;
}
```

### 4.2 CQRS (Command Query Responsibility Segregation)

#### 4.2.1 Command Flow

```
┌──────────┐
│  Client  │
└────┬─────┘
     │ Command
     ▼
┌────────────────┐
│ Command Handler│
└────┬───────────┘
     │ Validate & Execute
     ▼
┌────────────────┐
│  Write Model   │
│  (Event Store) │
└────┬───────────┘
     │ Publish Event
     ▼
┌────────────────┐
│  Event Bus     │
└────┬───────────┘
     │
     ├──────────────┬──────────────┐
     ▼              ▼              ▼
┌─────────┐  ┌──────────┐  ┌──────────┐
│Read Model│  │Analytics │  │ Notifier │
└─────────┘  └──────────┘  └──────────┘
```

#### 4.2.2 Command Example

```json
{
  "commandId": "cmd_5k7m9n2p4q",
  "commandType": "CreateMemorial",
  "timestamp": "2024-12-18T17:00:00Z",
  "userId": "660e8400-e29b-41d4-a716-446655440001",
  "data": {
    "petIdentity": {
      "name": "Max",
      "species": "dog"
    }
  },
  "expectedVersion": null,
  "metadata": {
    "correlationId": "cor_7x9k2m4n6p",
    "causationId": null
  }
}
```

### 4.3 Conflict Resolution

#### 4.3.1 Conflict Detection

```json
{
  "conflictId": "conf_3k5m7n9p2q",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "conflictType": "concurrent_modification",
  "timestamp": "2024-12-18T17:05:00Z",
  "conflicts": [
    {
      "field": "memorialCustomization.epitaph",
      "version1": {
        "value": "Forever in our hearts",
        "modifiedBy": "user_123",
        "timestamp": "2024-12-18T17:04:30Z"
      },
      "version2": {
        "value": "Always remembered, never forgotten",
        "modifiedBy": "user_456",
        "timestamp": "2024-12-18T17:04:32Z"
      }
    }
  ],
  "resolutionStrategy": "last_write_wins"
}
```

#### 4.3.2 Resolution Strategies

| Strategy | Description | When to Use |
|----------|-------------|-------------|
| **Last Write Wins** | Latest modification wins | Non-critical fields |
| **First Write Wins** | First modification wins | Create operations |
| **Merge** | Combine both versions | List/array fields |
| **Guardian Decides** | Ask guardian to choose | Critical fields |
| **Automatic Merge** | AI-assisted merging | Text descriptions |

---

## 5. Sharing Protocol

### 5.1 Public Memorial Sharing

#### 5.1.1 Share Link Generation

```http
POST /v1/memorials/{memorialId}/share
Authorization: Bearer {token}
Content-Type: application/json

{
  "shareType": "public_link",
  "accessLevel": "view_only",
  "expiresIn": 2592000,
  "requiresPassword": false,
  "allowedDomains": null,
  "customMessage": "I'd like to share Max's memorial with you."
}
```

**Response:**
```json
{
  "shareId": "shr_5k7m9n2p4q",
  "shareUrl": "https://petlegacy.wia.org/s/shr_5k7m9n2p4q",
  "shortUrl": "https://pet.wia/m/abc123",
  "qrCode": "https://cdn.petlegacy.wia.org/qr/shr_5k7m9n2p4q.png",
  "expiresAt": "2025-01-17T17:10:00Z",
  "analytics": {
    "trackViews": true,
    "trackReferrers": true,
    "viewsUrl": "/v1/shares/shr_5k7m9n2p4q/analytics"
  }
}
```

#### 5.1.2 Social Media Sharing

```json
{
  "shareType": "social_media",
  "platform": "facebook",
  "content": {
    "title": "Remembering Max - Golden Retriever (2012-2024)",
    "description": "Forever in our hearts. View Max's memorial and share your memories.",
    "image": "https://cdn.petlegacy.wia.org/share/550e8400.jpg",
    "url": "https://petlegacy.wia.org/memorial/550e8400"
  },
  "openGraph": {
    "og:type": "website",
    "og:title": "Remembering Max",
    "og:description": "A loving tribute to our beloved companion",
    "og:image": "https://cdn.petlegacy.wia.org/share/550e8400-og.jpg",
    "og:url": "https://petlegacy.wia.org/memorial/550e8400"
  },
  "twitterCard": {
    "card": "summary_large_image",
    "title": "Remembering Max",
    "description": "Forever in our hearts",
    "image": "https://cdn.petlegacy.wia.org/share/550e8400-twitter.jpg"
  }
}
```

### 5.2 Deep Link Protocol

#### 5.2.1 Deep Link Structure

```
petlegacy://memorial/{memorialId}[/{section}[/{itemId}]]

Examples:
petlegacy://memorial/550e8400
petlegacy://memorial/550e8400/timeline
petlegacy://memorial/550e8400/media/770e8400
petlegacy://memorial/550e8400/timeline/880e8400
```

#### 5.2.2 Universal Links (iOS/Android)

```json
{
  "applinks": {
    "apps": [],
    "details": [
      {
        "appID": "TEAM_ID.org.wia.petlegacy",
        "paths": [
          "/memorial/*",
          "/s/*",
          "/invite/*"
        ]
      }
    ]
  },
  "webcredentials": {
    "apps": ["TEAM_ID.org.wia.petlegacy"]
  }
}
```

### 5.3 Embedding Protocol

#### 5.3.1 iFrame Embed

```html
<iframe
  src="https://petlegacy.wia.org/embed/550e8400"
  width="600"
  height="800"
  frameborder="0"
  allow="fullscreen"
  sandbox="allow-scripts allow-same-origin"
></iframe>
```

#### 5.3.2 Widget Embed

```javascript
<script src="https://cdn.petlegacy.wia.org/widget.js"></script>
<script>
  PetLegacyWidget.create({
    memorialId: '550e8400-e29b-41d4-a716-446655440000',
    container: '#memorial-widget',
    theme: 'light',
    displayMode: 'timeline',
    maxHeight: 600,
    showHeader: true,
    showFooter: false,
    allowInteraction: false
  });
</script>
<div id="memorial-widget"></div>
```

---

## 6. Memorial Portability Protocol

### 6.1 Export Protocol

#### 6.1.1 Export Request

```http
POST /v1/memorials/{memorialId}/export
Authorization: Bearer {token}
Content-Type: application/json

{
  "exportFormat": "wia-pet-legacy-1.0",
  "includeMedia": true,
  "mediaQuality": "original",
  "includeComments": true,
  "includeAnalytics": false,
  "compression": "zip",
  "encryptionPassword": "optional_password",
  "deliveryMethod": "download"
}
```

#### 6.1.2 Export Package Structure

```
memorial-export-max-20241218.zip
├── manifest.json
├── memorial-profile.json
├── timeline-events.json
├── family-members.json
├── comments.json
├── media/
│   ├── photos/
│   │   ├── 2012/
│   │   ├── 2013/
│   │   └── ...
│   ├── videos/
│   └── documents/
├── thumbnails/
├── analytics-report.pdf
└── README.txt
```

#### 6.1.3 Manifest File

```json
{
  "exportFormat": "wia-pet-legacy-1.0",
  "exportDate": "2024-12-18T17:20:00Z",
  "exportVersion": "1.0.0",
  "memorialProfile": {
    "profileId": "550e8400-e29b-41d4-a716-446655440000",
    "petName": "Max",
    "exportedBy": "660e8400-e29b-41d4-a716-446655440001"
  },
  "contents": {
    "memorialProfile": "memorial-profile.json",
    "timelineEvents": "timeline-events.json",
    "familyMembers": "family-members.json",
    "comments": "comments.json",
    "mediaDirectory": "media/"
  },
  "statistics": {
    "totalFiles": 418,
    "totalSize": 2457280000,
    "photos": 324,
    "videos": 47,
    "documents": 12
  },
  "checksums": {
    "algorithm": "SHA-256",
    "manifest": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "files": {
      "memorial-profile.json": "a1b2c3d4...",
      "timeline-events.json": "e5f6g7h8..."
    }
  }
}
```

### 6.2 Import Protocol

#### 6.2.1 Import Request

```http
POST /v1/import
Authorization: Bearer {token}
Content-Type: multipart/form-data

{
  "file": "[export package]",
  "importOptions": {
    "preserveIds": false,
    "skipExistingMedia": true,
    "updateExisting": false,
    "assignToUser": "660e8400-e29b-41d4-a716-446655440001"
  }
}
```

#### 6.2.2 Import Validation

```json
{
  "validationId": "val_7x9k2m4n6p",
  "status": "validating",
  "checks": [
    {
      "check": "format_version",
      "status": "passed",
      "message": "Format version 1.0.0 is supported"
    },
    {
      "check": "manifest_integrity",
      "status": "passed",
      "message": "Manifest checksum verified"
    },
    {
      "check": "file_integrity",
      "status": "warning",
      "message": "2 files have checksum mismatches",
      "files": ["photo-123.jpg", "video-456.mp4"]
    },
    {
      "check": "quota_check",
      "status": "passed",
      "message": "Sufficient storage available (2.3 GB required)"
    },
    {
      "check": "schema_validation",
      "status": "passed",
      "message": "All JSON files conform to schema"
    }
  ],
  "canProceed": true,
  "warnings": 1,
  "errors": 0
}
```

### 6.3 Migration Protocol

#### 6.3.1 Platform Migration

```json
{
  "migrationType": "platform_to_platform",
  "source": {
    "platform": "competitor_platform",
    "authToken": "source_platform_token",
    "memorialIds": ["mem_123", "mem_456"]
  },
  "destination": {
    "platform": "wia-pet-legacy",
    "targetUserId": "660e8400-e29b-41d4-a716-446655440001"
  },
  "options": {
    "preserveUrls": false,
    "migrateComments": true,
    "migrateFamilyMembers": true,
    "notifyMembers": true
  }
}
```

#### 6.3.2 Migration Status Tracking

```json
{
  "migrationId": "mig_5k7m9n2p4q",
  "status": "in_progress",
  "progress": {
    "percentage": 45,
    "currentStep": "migrating_media",
    "totalSteps": 6
  },
  "steps": [
    {
      "step": "export_from_source",
      "status": "completed",
      "startedAt": "2024-12-18T17:30:00Z",
      "completedAt": "2024-12-18T17:35:00Z"
    },
    {
      "step": "migrate_profile",
      "status": "completed"
    },
    {
      "step": "migrate_timeline",
      "status": "completed"
    },
    {
      "step": "migrating_media",
      "status": "in_progress",
      "itemsProcessed": 187,
      "totalItems": 418
    },
    {
      "step": "migrate_comments",
      "status": "pending"
    },
    {
      "step": "finalize",
      "status": "pending"
    }
  ]
}
```

---

## 7. Cemetery Integration Protocol

### 7.1 Cemetery Service Discovery

```http
GET /v1/integrations/cemeteries/search
Authorization: Bearer {token}
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `location` | string | City, state, or coordinates |
| `radius` | integer | Search radius in miles |
| `services` | string | burial, cremation, both |

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "cemeteryId": "cem_5k7m9n2p4q",
      "name": "Rainbow Bridge Pet Memorial",
      "location": {
        "address": "123 Memorial Lane",
        "city": "Portland",
        "state": "OR",
        "zipCode": "97201",
        "coordinates": {
          "latitude": 45.523064,
          "longitude": -122.676483
        }
      },
      "services": ["burial", "cremation", "memorial_services"],
      "integration": {
        "status": "active",
        "apiVersion": "1.0",
        "capabilities": [
          "plot_reservation",
          "service_scheduling",
          "digital_linking"
        ]
      },
      "contact": {
        "phone": "555-0123",
        "email": "info@rainbowbridge.example.com",
        "website": "https://rainbowbridge.example.com"
      }
    }
  ]
}
```

### 7.2 Cemetery Linking Protocol

#### 7.2.1 Link Memorial to Cemetery

```http
POST /v1/memorials/{memorialId}/cemetery-link
Authorization: Bearer {token}
Content-Type: application/json

{
  "cemeteryId": "cem_5k7m9n2p4q",
  "serviceType": "cremation",
  "plotNumber": "Section A, Plot 127",
  "urnLocation": "Columbarium Wall, Niche 45",
  "serviceDate": "2024-03-12T14:00:00Z",
  "certificateNumber": "CERT-2024-00127",
  "additionalInfo": {
    "urnsDescription": "Wooden urn with brass nameplate",
    "markerType": "Bronze plaque",
    "markerInscription": "Max - Forever in Our Hearts"
  }
}
```

#### 7.2.2 Cemetery Data Sync

```json
{
  "syncType": "bidirectional",
  "frequency": "daily",
  "syncFields": [
    "plotNumber",
    "serviceDate",
    "certificateNumber",
    "markerInscription",
    "visitorLogs"
  ],
  "lastSync": "2024-12-18T17:45:00Z",
  "nextSync": "2024-12-19T02:00:00Z",
  "syncStatus": {
    "status": "success",
    "recordsUpdated": 3,
    "errors": []
  }
}
```

### 7.3 Virtual Visitation Protocol

```json
{
  "visitationType": "virtual",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "cemeteryId": "cem_5k7m9n2p4q",
  "visitor": {
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "displayName": "Sarah Johnson"
  },
  "visitDate": "2024-12-18T18:00:00Z",
  "activities": [
    {
      "type": "virtual_flower_placement",
      "flower": "white_rose",
      "message": "Missing you today, sweet boy",
      "displayDuration": 604800
    },
    {
      "type": "candle_lighting",
      "candleType": "memorial",
      "burnDuration": 86400
    },
    {
      "type": "tribute_reading",
      "tributeId": "trib_3k5m7n9p2q"
    }
  ],
  "logVisit": true
}
```

---

## 8. Grief Support Protocol

### 8.1 Support Resource Discovery

```http
GET /v1/grief-support/resources
Authorization: Bearer {token}
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `resourceType` | string | counseling, support_groups, articles |
| `location` | string | For in-person services |
| `language` | string | Preferred language |
| `petType` | string | Species-specific support |

**Response:**
```json
{
  "success": true,
  "data": {
    "immediateSupport": {
      "hotlines": [
        {
          "name": "Pet Loss Support Hotline",
          "phone": "1-800-PET-LOSS",
          "hours": "24/7",
          "languages": ["en", "es"]
        }
      ],
      "chat": {
        "available": true,
        "url": "https://petlegacy.wia.org/support/chat",
        "averageWaitTime": 120
      }
    },
    "professionalCounselors": [
      {
        "counselorId": "coun_5k7m9n2p4q",
        "name": "Dr. Emily Chen",
        "specialization": "Pet loss grief counseling",
        "credentials": ["Licensed Therapist", "Certified Pet Loss Counselor"],
        "availability": "telehealth",
        "languages": ["en", "zh"],
        "bookingUrl": "https://example.com/book/dr-chen"
      }
    ],
    "supportGroups": [
      {
        "groupId": "grp_7x9k2m4n6p",
        "name": "Portland Pet Loss Support Group",
        "meetingType": "hybrid",
        "schedule": "Every Tuesday, 7:00 PM",
        "location": "Community Center, Portland, OR",
        "virtualLink": "https://zoom.us/j/123456789",
        "facilitator": "Licensed Social Worker"
      }
    ],
    "articles": [
      {
        "articleId": "art_3k5m7n9p2q",
        "title": "Coping with the Loss of a Pet",
        "author": "Pet Loss Expert",
        "readTime": 8,
        "url": "https://petlegacy.wia.org/resources/articles/coping-with-loss"
      }
    ]
  }
}
```

### 8.2 Grief Journey Tracking

```json
{
  "userId": "660e8400-e29b-41d4-a716-446655440001",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "journeyStartDate": "2024-03-10",
  "milestones": [
    {
      "date": "2024-03-10",
      "type": "loss",
      "description": "Max passed away"
    },
    {
      "date": "2024-03-12",
      "type": "memorial_service",
      "description": "Held memorial service"
    },
    {
      "date": "2024-03-15",
      "type": "memorial_creation",
      "description": "Created digital memorial"
    },
    {
      "date": "2024-04-10",
      "type": "one_month",
      "supportOffered": ["gentle_reminder", "support_resources"]
    },
    {
      "date": "2024-09-10",
      "type": "six_months",
      "supportOffered": ["reflection_prompt", "community_connection"]
    }
  ],
  "preferences": {
    "receiveReminders": true,
    "reminderFrequency": "major_milestones",
    "supportType": "gentle"
  }
}
```

### 8.3 Crisis Detection Protocol

```json
{
  "detectionType": "pattern_analysis",
  "triggers": [
    {
      "type": "concerning_language",
      "detected": "Phrase patterns suggesting crisis",
      "confidence": 0.85
    },
    {
      "type": "activity_change",
      "detected": "Significant decrease in engagement",
      "baseline": "Daily visits",
      "current": "No visits in 14 days"
    }
  ],
  "recommendedAction": {
    "priority": "high",
    "actions": [
      "Display crisis resources prominently",
      "Offer immediate chat support",
      "Suggest professional counseling",
      "Notify emergency contacts (if authorized)"
    ],
    "resources": [
      {
        "name": "National Suicide Prevention Lifeline",
        "phone": "988",
        "available": "24/7"
      },
      {
        "name": "Crisis Text Line",
        "text": "HELLO to 741741",
        "available": "24/7"
      }
    ]
  }
}
```

---

## 9. Community Protocol

### 9.1 Community Features

```http
GET /v1/community/discover
Authorization: Bearer {token}
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `category` | string | stories, tributes, support |
| `petType` | string | Filter by species |
| `sortBy` | string | recent, popular, relevant |

**Response:**
```json
{
  "success": true,
  "data": {
    "featuredStories": [
      {
        "storyId": "story_5k7m9n2p4q",
        "title": "How Max Taught Me About Unconditional Love",
        "author": {
          "userId": "660e8400",
          "displayName": "Sarah Johnson",
          "petName": "Max"
        },
        "excerpt": "In the 11 years we had together...",
        "publishedAt": "2024-04-15T10:00:00Z",
        "reactions": {
          "hearts": 342,
          "paws": 189
        },
        "commentCount": 47
      }
    ],
    "tributeWall": [
      {
        "tributeId": "trib_7x9k2m4n6p",
        "memorialId": "550e8400",
        "petName": "Max",
        "from": "Jennifer Smith",
        "message": "Max was such a special dog. He brought joy to everyone.",
        "postedAt": "2024-12-17T14:30:00Z"
      }
    ],
    "supportThreads": [
      {
        "threadId": "thread_3k5m7n9p2q",
        "title": "Recently lost my Golden Retriever",
        "author": "Anonymous",
        "replies": 28,
        "lastActivity": "2024-12-18T16:00:00Z"
      }
    ]
  }
}
```

### 9.2 Tribute Protocol

```http
POST /v1/memorials/{memorialId}/tributes
Authorization: Bearer {token}
Content-Type: application/json

{
  "from": {
    "name": "Jennifer Smith",
    "relationship": "Friend and neighbor",
    "email": "jennifer@example.com"
  },
  "message": "Max was such a wonderful companion. He always greeted us with a wagging tail and brought so much joy to the neighborhood. He will be deeply missed.",
  "includeContact": false,
  "isAnonymous": false,
  "virtualGift": {
    "type": "candle",
    "duration": 86400
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "tributeId": "trib_9k2m5n7p8q",
    "postedAt": "2024-12-18T18:30:00Z",
    "status": "published",
    "moderationRequired": false,
    "notificationSent": true
  }
}
```

### 9.3 Memorial Anniversary Protocol

```json
{
  "anniversaryType": "passing",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "anniversaryDate": "2025-03-10",
  "yearsSince": 1,
  "notifications": {
    "guardian": {
      "enabled": true,
      "advanceNotice": 604800,
      "channels": ["email", "push", "in_app"]
    },
    "familyMembers": {
      "enabled": true,
      "advanceNotice": 259200,
      "channels": ["email", "in_app"]
    }
  },
  "communityFeatures": {
    "enableTributePrompt": true,
    "createHighlight": true,
    "suggestMemorySharing": true
  },
  "supportResources": {
    "offerCounselingInfo": true,
    "shareGriefArticles": true,
    "connectToSupportGroups": false
  }
}
```

---

## 10. Security Protocols

### 10.1 Encryption Protocol

#### 10.1.1 Transport Layer Security

```json
{
  "tls": {
    "version": "1.3",
    "cipherSuites": [
      "TLS_AES_256_GCM_SHA384",
      "TLS_CHACHA20_POLY1305_SHA256",
      "TLS_AES_128_GCM_SHA256"
    ],
    "certificateAuthority": "Let's Encrypt",
    "certificateValidity": "90 days",
    "hsts": {
      "enabled": true,
      "maxAge": 31536000,
      "includeSubDomains": true,
      "preload": true
    }
  }
}
```

#### 10.1.2 End-to-End Encryption (Optional)

```json
{
  "e2ee": {
    "enabled": true,
    "algorithm": "AES-256-GCM",
    "keyDerivation": "PBKDF2",
    "iterations": 100000,
    "scope": ["private_messages", "sensitive_documents"],
    "keyStorage": "client_side_only",
    "description": "End-to-end encrypted content can only be decrypted by authorized users with the encryption key"
  }
}
```

### 10.2 Access Control Protocol

```json
{
  "resourceType": "memorial",
  "resourceId": "550e8400-e29b-41d4-a716-446655440000",
  "accessControl": {
    "model": "RBAC",
    "roles": [
      {
        "role": "guardian",
        "permissions": ["read", "write", "delete", "manage"],
        "users": ["660e8400"]
      },
      {
        "role": "contributor",
        "permissions": ["read", "write"],
        "users": ["660e8400-e29b-41d4-a716-446655440010"]
      },
      {
        "role": "viewer",
        "permissions": ["read"],
        "users": ["public"]
      }
    ],
    "conditions": [
      {
        "action": "write",
        "condition": "user.role == 'guardian' OR user.role == 'contributor'"
      },
      {
        "action": "delete",
        "condition": "user.role == 'guardian'"
      }
    ]
  }
}
```

### 10.3 Audit Log Protocol

```json
{
  "auditLogId": "audit_5k7m9n2p4q",
  "timestamp": "2024-12-18T18:45:00Z",
  "eventType": "memorial.updated",
  "actor": {
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "displayName": "Sarah Johnson",
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0..."
  },
  "resource": {
    "type": "memorial",
    "id": "550e8400-e29b-41d4-a716-446655440000"
  },
  "action": "update",
  "changes": [
    {
      "field": "memorialCustomization.epitaph",
      "oldValue": "Forever in our hearts",
      "newValue": "Forever in our hearts. The best friend anyone could ask for."
    }
  ],
  "metadata": {
    "correlationId": "cor_7x9k2m4n6p",
    "sessionId": "sess_3k5m7n9p2q"
  },
  "result": {
    "success": true,
    "responseCode": 200
  }
}
```

---

## Appendix A: Protocol Compliance Checklist

- [ ] All API endpoints use HTTPS with TLS 1.3
- [ ] WebSocket connections use WSS protocol
- [ ] Authentication uses OAuth 2.0 or JWT
- [ ] Rate limiting implemented on all endpoints
- [ ] Event sourcing for critical operations
- [ ] CQRS pattern for read/write separation
- [ ] Audit logging for all data modifications
- [ ] End-to-end encryption for sensitive data
- [ ] GDPR compliance for data portability
- [ ] Webhook verification with HMAC signatures
- [ ] Idempotency keys for critical operations
- [ ] Graceful degradation for network failures
- [ ] Circuit breakers for external services
- [ ] Data validation on all inputs
- [ ] Proper error handling and reporting

---

## Appendix B: Protocol Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-18 | Initial protocol specification |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
