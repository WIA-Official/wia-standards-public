# WIA PET-LEGACY PHASE 3: 프로토콜 명세서

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## 목차

1. [개요](#개요)
2. [통신 프로토콜](#통신-프로토콜)
3. [데이터 교환 프로토콜](#데이터-교환-프로토콜)
4. [동기화 프로토콜](#동기화-프로토콜)
5. [공유 프로토콜](#공유-프로토콜)
6. [추모관 이식성 프로토콜](#추모관-이식성-프로토콜)
7. [묘지 통합 프로토콜](#묘지-통합-프로토콜)
8. [애도 지원 프로토콜](#애도-지원-프로토콜)
9. [커뮤니티 프로토콜](#커뮤니티-프로토콜)
10. [보안 프로토콜](#보안-프로토콜)

---

## 1. 개요

### 1.1 목적

WIA PET-LEGACY Phase 3 명세서는 추모 플랫폼, 동물병원 시스템, 반려동물 묘지, 화장장 및 타사 서비스 간의 통신, 데이터 교환 및 상호 운용성을 위한 포괄적인 프로토콜을 정의합니다. 이러한 프로토콜은 생태계 전반에 걸쳐 반려동물 추모 데이터의 원활하고 안전하며 윤리적인 처리를 보장합니다.

### 1.2 프로토콜 스택

```
┌─────────────────────────────────────────────────┐
│         애플리케이션 계층                        │
│  추모관 관리, 미디어 공유, AI                    │
├─────────────────────────────────────────────────┤
│         프로토콜 계층                            │
│  PET-LEGACY 프로토콜 제품군                      │
├─────────────────────────────────────────────────┤
│         전송 계층                                │
│  HTTPS, WebSocket, gRPC                         │
├─────────────────────────────────────────────────┤
│         네트워크 계층                            │
│  TCP/IP, TLS 1.3                                │
└─────────────────────────────────────────────────┘
```

### 1.3 프로토콜 카테고리

| 카테고리 | 프로토콜 | 목적 |
|---------|---------|------|
| **통신** | WebSocket, Server-Sent Events | 실시간 업데이트 |
| **데이터 교환** | REST, GraphQL, gRPC | API 통신 |
| **동기화** | Event Sourcing, CQRS | 데이터 일관성 |
| **공유** | OAuth 2.0, Deep Links | 소셜 공유 |
| **이식성** | Export/Import, Migration | 플랫폼 전환 |
| **통합** | Webhooks, APIs | 타사 서비스 |
| **보안** | TLS, JWT, Encryption | 데이터 보호 |

### 1.4 프로토콜 설계 원칙

| 원칙 | 설명 | 구현 |
|------|------|------|
| **복원력** | 네트워크 장애를 우아하게 처리 | 재시도 로직, 회로 차단기 |
| **멱등성** | 동일한 요청이 동일한 결과 생성 | 멱등성 키 |
| **버전 관리** | 프로토콜 진화 지원 | 버전 협상 |
| **확장성** | 프로토콜 확장 허용 | 플러그인 아키텍처 |
| **프라이버시** | 민감한 정보 보호 | 종단 간 암호화 |

---

## 2. 통신 프로토콜

### 2.1 WebSocket을 통한 실시간 업데이트

#### 2.1.1 연결 설정

```
클라이언트                                  서버
  │                                        │
  │  WSS 연결 요청                         │
  ├───────────────────────────────────────>│
  │  wss://ws.petlegacy.wia.org/v1         │
  │                                        │
  │  연결 수락                             │
  │<───────────────────────────────────────┤
  │  (WebSocket 핸드셰이크 완료)            │
  │                                        │
  │  인증 메시지                           │
  ├───────────────────────────────────────>│
  │  {"type":"auth","token":"..."}         │
  │                                        │
  │  인증 성공                             │
  │<───────────────────────────────────────┤
  │  {"type":"auth_success"}               │
  │                                        │
  │  추모관 구독                           │
  ├───────────────────────────────────────>│
  │  {"type":"subscribe",                  │
  │   "channel":"memorial.123"}            │
  │                                        │
  │  구독 확인                             │
  │<───────────────────────────────────────┤
```

#### 2.1.2 WebSocket 메시지 형식

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
    "uploadedBy": "사라 존슨",
    "title": "새로운 추억 추가됨"
  },
  "metadata": {
    "priority": "normal",
    "requiresAck": false
  }
}
```

#### 2.1.3 이벤트 유형

| 이벤트 유형 | 설명 | 예시 페이로드 |
|-----------|------|-------------|
| `memorial.updated` | 추모 프로필 수정됨 | 변경된 필드 |
| `media.uploaded` | 새 미디어 자산 추가됨 | 자산 메타데이터 |
| `media.processed` | 미디어 처리 완료 | 자산 URL |
| `timeline.event_created` | 새 타임라인 이벤트 | 이벤트 세부 정보 |
| `comment.added` | 새 댓글 게시됨 | 댓글 내용 |
| `family.member_joined` | 새 가족 구성원 | 구성원 정보 |
| `presence.online` | 사용자가 온라인 상태가 됨 | 사용자 ID |
| `presence.offline` | 사용자가 오프라인 상태가 됨 | 사용자 ID |

#### 2.1.4 Presence 프로토콜

```json
{
  "type": "presence.update",
  "channel": "memorial.550e8400",
  "users": [
    {
      "userId": "user_123",
      "displayName": "사라 존슨",
      "status": "online",
      "lastSeen": "2024-12-18T16:32:00Z",
      "currentActivity": "viewing_timeline"
    },
    {
      "userId": "user_456",
      "displayName": "마이클 존슨",
      "status": "idle",
      "lastSeen": "2024-12-18T16:25:00Z"
    }
  ]
}
```

### 2.2 Server-Sent Events (SSE)

WebSocket을 지원하지 않는 클라이언트용:

```http
GET /v1/memorials/550e8400/events
Accept: text/event-stream
Authorization: Bearer {token}
```

**서버 응답:**
```
event: media.uploaded
id: evt_001
data: {"assetId":"770e8400","type":"photo"}

event: comment.added
id: evt_002
data: {"commentId":"cmt_123","author":"사라"}

event: heartbeat
id: evt_003
data: {"timestamp":"2024-12-18T16:35:00Z"}
```

### 2.3 푸시 알림

#### 2.3.1 푸시 알림 페이로드

```json
{
  "notification": {
    "title": "맥스의 추모관에 새로운 추억이 추가되었습니다",
    "body": "마이클 존슨님이 새 사진을 업로드했습니다",
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

#### 2.3.2 알림 환경설정

| 알림 유형 | 기본값 | 비활성화 가능 | 빈도 제한 |
|----------|-------|-------------|---------|
| 새 미디어 업로드 | On | 예 | 시간당 일괄 처리 |
| 타임라인 이벤트 추가 | On | 예 | 즉시 |
| 내 게시물에 댓글 | On | 예 | 즉시 |
| 가족 구성원 가입 | On | 예 | 즉시 |
| 추모관 기념일 | On | 아니오 | 연 1회 |
| 헌사 수신 | On | 예 | 즉시 |
| 내보내기 완료 | On | 아니오 | 즉시 |

---

## 3. 데이터 교환 프로토콜

### 3.1 GraphQL 프로토콜

#### 3.1.1 GraphQL 엔드포인트

```http
POST /v1/graphql
Content-Type: application/json
Authorization: Bearer {token}
```

#### 3.1.2 예시 쿼리

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

#### 3.1.3 예시 뮤테이션

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

**변수:**
```json
{
  "input": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "CELEBRATION",
    "date": "2020-05-20T15:00:00Z",
    "title": "맥스의 8번째 생일",
    "description": "케이크와 간식으로 축하했어요",
    "attachedMedia": ["770e8400-e29b-41d4-a716-446655440002"]
  }
}
```

#### 3.1.4 GraphQL 구독

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

### 3.2 gRPC 프로토콜

#### 3.2.1 Protocol Buffer 정의

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
```

### 3.3 메시지 큐 프로토콜

#### 3.3.1 이벤트 게시

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

#### 3.3.2 컨슈머 그룹

| 컨슈머 그룹 | 토픽 | 목적 |
|-----------|------|------|
| `analytics-processors` | memorial.events | 분석 생성 |
| `notification-senders` | memorial.events | 푸시 알림 전송 |
| `ai-processors` | media.events | AI로 이미지 처리 |
| `search-indexers` | memorial.events | 검색 인덱스 업데이트 |
| `backup-workers` | memorial.events | 데이터 변경 백업 |

---

## 4. 동기화 프로토콜

### 4.1 Event Sourcing

#### 4.1.1 이벤트 저장소 구조

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
      "name": "맥스",
      "species": "dog",
      "breed": "골든 리트리버"
    }
  },
  "metadata": {
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0...",
    "correlationId": "cor_5x7k9m2n4p"
  }
}
```

#### 4.1.2 이벤트 유형

| 이벤트 유형 | 설명 | 트리거 |
|-----------|------|--------|
| `MemorialCreated` | 새 추모관 초기화됨 | 추모관 생성 |
| `MemorialUpdated` | 추모관 수정됨 | 모든 필드 업데이트 |
| `MediaAdded` | 미디어 자산 업로드됨 | 미디어 업로드 |
| `MediaRemoved` | 미디어 자산 삭제됨 | 미디어 삭제 |
| `TimelineEventAdded` | 타임라인에 이벤트 추가됨 | 이벤트 생성 |
| `FamilyMemberInvited` | 초대장 전송됨 | 구성원 초대 |
| `FamilyMemberJoined` | 구성원이 수락함 | 초대 수락 |
| `CommentPosted` | 댓글 추가됨 | 댓글 게시 |
| `MemorialPublished` | 공개됨 | 추모관 게시 |
| `MemorialArchived` | 보관됨 | 추모관 보관 |

### 4.2 CQRS (Command Query Responsibility Segregation)

#### 4.2.1 명령 흐름

```
┌──────────┐
│클라이언트│
└────┬─────┘
     │ 명령
     ▼
┌────────────────┐
│ 명령 핸들러     │
└────┬───────────┘
     │ 검증 및 실행
     ▼
┌────────────────┐
│  쓰기 모델     │
│ (이벤트 저장소) │
└────┬───────────┘
     │ 이벤트 게시
     ▼
┌────────────────┐
│  이벤트 버스    │
└────┬───────────┘
     │
     ├──────────────┬──────────────┐
     ▼              ▼              ▼
┌─────────┐  ┌──────────┐  ┌──────────┐
│읽기 모델│  │  분석    │  │  알림    │
└─────────┘  └──────────┘  └──────────┘
```

### 4.3 충돌 해결

#### 4.3.1 충돌 감지

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
        "value": "영원히 우리 마음속에",
        "modifiedBy": "user_123",
        "timestamp": "2024-12-18T17:04:30Z"
      },
      "version2": {
        "value": "항상 기억되고, 결코 잊히지 않을",
        "modifiedBy": "user_456",
        "timestamp": "2024-12-18T17:04:32Z"
      }
    }
  ],
  "resolutionStrategy": "last_write_wins"
}
```

#### 4.3.2 해결 전략

| 전략 | 설명 | 사용 시기 |
|------|------|---------|
| **Last Write Wins** | 최신 수정이 승리 | 중요하지 않은 필드 |
| **First Write Wins** | 첫 번째 수정이 승리 | 생성 작업 |
| **Merge** | 두 버전 결합 | 목록/배열 필드 |
| **Guardian Decides** | 보호자에게 선택 요청 | 중요한 필드 |
| **Automatic Merge** | AI 지원 병합 | 텍스트 설명 |

---

## 5. 공유 프로토콜

### 5.1 공개 추모관 공유

#### 5.1.1 공유 링크 생성

```http
POST /v1/memorials/{memorialId}/share
Authorization: Bearer {token}
Content-Type: application/json

{
  "shareType": "public_link",
  "accessLevel": "view_only",
  "expiresIn": 2592000,
  "requiresPassword": false,
  "customMessage": "맥스의 추모관을 공유하고 싶어요."
}
```

**응답:**
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

#### 5.1.2 소셜 미디어 공유

```json
{
  "shareType": "social_media",
  "platform": "facebook",
  "content": {
    "title": "맥스 추모 - 골든 리트리버 (2012-2024)",
    "description": "영원히 우리 마음속에. 맥스의 추모관을 보고 당신의 추억을 공유하세요.",
    "image": "https://cdn.petlegacy.wia.org/share/550e8400.jpg",
    "url": "https://petlegacy.wia.org/memorial/550e8400"
  },
  "openGraph": {
    "og:type": "website",
    "og:title": "맥스 추모",
    "og:description": "우리의 사랑받는 반려견에 대한 따뜻한 헌사",
    "og:image": "https://cdn.petlegacy.wia.org/share/550e8400-og.jpg",
    "og:url": "https://petlegacy.wia.org/memorial/550e8400"
  }
}
```

### 5.2 Deep Link 프로토콜

#### 5.2.1 Deep Link 구조

```
petlegacy://memorial/{memorialId}[/{section}[/{itemId}]]

예시:
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
  }
}
```

### 5.3 임베딩 프로토콜

#### 5.3.1 iFrame 임베드

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

#### 5.3.2 위젯 임베드

```javascript
<script src="https://cdn.petlegacy.wia.org/widget.js"></script>
<script>
  PetLegacyWidget.create({
    memorialId: '550e8400-e29b-41d4-a716-446655440000',
    container: '#memorial-widget',
    theme: 'light',
    displayMode: 'timeline',
    maxHeight: 600
  });
</script>
<div id="memorial-widget"></div>
```

---

## 6. 추모관 이식성 프로토콜

### 6.1 내보내기 프로토콜

#### 6.1.1 내보내기 요청

```http
POST /v1/memorials/{memorialId}/export
Authorization: Bearer {token}
Content-Type: application/json

{
  "exportFormat": "wia-pet-legacy-1.0",
  "includeMedia": true,
  "mediaQuality": "original",
  "includeComments": true,
  "compression": "zip",
  "encryptionPassword": "optional_password"
}
```

#### 6.1.2 내보내기 패키지 구조

```
memorial-export-max-20241218.zip
├── manifest.json
├── memorial-profile.json
├── timeline-events.json
├── family-members.json
├── comments.json
├── media/
│   ├── photos/
│   ├── videos/
│   └── documents/
└── thumbnails/
```

#### 6.1.3 매니페스트 파일

```json
{
  "exportFormat": "wia-pet-legacy-1.0",
  "exportDate": "2024-12-18T17:20:00Z",
  "exportVersion": "1.0.0",
  "memorialProfile": {
    "profileId": "550e8400-e29b-41d4-a716-446655440000",
    "petName": "맥스",
    "exportedBy": "660e8400-e29b-41d4-a716-446655440001"
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
    "manifest": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
  }
}
```

### 6.2 가져오기 프로토콜

#### 6.2.1 가져오기 검증

```json
{
  "validationId": "val_7x9k2m4n6p",
  "status": "validating",
  "checks": [
    {
      "check": "format_version",
      "status": "passed",
      "message": "형식 버전 1.0.0이 지원됩니다"
    },
    {
      "check": "manifest_integrity",
      "status": "passed",
      "message": "매니페스트 체크섬 확인됨"
    },
    {
      "check": "quota_check",
      "status": "passed",
      "message": "충분한 저장 공간 사용 가능 (2.3 GB 필요)"
    }
  ],
  "canProceed": true,
  "warnings": 0,
  "errors": 0
}
```

---

## 7. 묘지 통합 프로토콜

### 7.1 묘지 서비스 탐색

```http
GET /v1/integrations/cemeteries/search
Authorization: Bearer {token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 |
|---------|-----|------|
| `location` | string | 도시, 주 또는 좌표 |
| `radius` | integer | 마일 단위 검색 반경 |
| `services` | string | burial, cremation, both |

### 7.2 묘지 연결 프로토콜

#### 7.2.1 추모관을 묘지에 연결

```http
POST /v1/memorials/{memorialId}/cemetery-link
Authorization: Bearer {token}
Content-Type: application/json

{
  "cemeteryId": "cem_5k7m9n2p4q",
  "serviceType": "cremation",
  "plotNumber": "섹션 A, 구역 127",
  "serviceDate": "2024-03-12T14:00:00Z",
  "certificateNumber": "CERT-2024-00127"
}
```

### 7.3 가상 방문 프로토콜

```json
{
  "visitationType": "virtual",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "cemeteryId": "cem_5k7m9n2p4q",
  "visitor": {
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "displayName": "사라 존슨"
  },
  "visitDate": "2024-12-18T18:00:00Z",
  "activities": [
    {
      "type": "virtual_flower_placement",
      "flower": "white_rose",
      "message": "오늘 네가 보고 싶구나, 착한 아이야",
      "displayDuration": 604800
    },
    {
      "type": "candle_lighting",
      "candleType": "memorial",
      "burnDuration": 86400
    }
  ]
}
```

---

## 8. 애도 지원 프로토콜

### 8.1 지원 리소스 탐색

```http
GET /v1/grief-support/resources
Authorization: Bearer {token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 |
|---------|-----|------|
| `resourceType` | string | counseling, support_groups, articles |
| `location` | string | 대면 서비스용 |
| `language` | string | 선호 언어 |
| `petType` | string | 종별 지원 |

### 8.2 애도 여정 추적

```json
{
  "userId": "660e8400-e29b-41d4-a716-446655440001",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "journeyStartDate": "2024-03-10",
  "milestones": [
    {
      "date": "2024-03-10",
      "type": "loss",
      "description": "맥스가 별세함"
    },
    {
      "date": "2024-03-12",
      "type": "memorial_service",
      "description": "추도식 거행"
    },
    {
      "date": "2024-04-10",
      "type": "one_month",
      "supportOffered": ["gentle_reminder", "support_resources"]
    }
  ],
  "preferences": {
    "receiveReminders": true,
    "reminderFrequency": "major_milestones",
    "supportType": "gentle"
  }
}
```

---

## 9. 커뮤니티 프로토콜

### 9.1 헌사 프로토콜

```http
POST /v1/memorials/{memorialId}/tributes
Authorization: Bearer {token}
Content-Type: application/json

{
  "from": {
    "name": "제니퍼 스미스",
    "relationship": "친구이자 이웃",
    "email": "jennifer@example.com"
  },
  "message": "맥스는 정말 훌륭한 반려견이었어요. 항상 꼬리를 흔들며 우리를 맞이해주고 이웃에 많은 기쁨을 가져다주었습니다. 정말 그리울 거예요.",
  "isAnonymous": false,
  "virtualGift": {
    "type": "candle",
    "duration": 86400
  }
}
```

### 9.2 추모관 기념일 프로토콜

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
  }
}
```

---

## 10. 보안 프로토콜

### 10.1 암호화 프로토콜

#### 10.1.1 전송 계층 보안

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
    "hsts": {
      "enabled": true,
      "maxAge": 31536000,
      "includeSubDomains": true
    }
  }
}
```

### 10.2 접근 제어 프로토콜

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
    ]
  }
}
```

### 10.3 감사 로그 프로토콜

```json
{
  "auditLogId": "audit_5k7m9n2p4q",
  "timestamp": "2024-12-18T18:45:00Z",
  "eventType": "memorial.updated",
  "actor": {
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "displayName": "사라 존슨",
    "ipAddress": "192.168.1.100"
  },
  "resource": {
    "type": "memorial",
    "id": "550e8400-e29b-41d4-a716-446655440000"
  },
  "action": "update",
  "changes": [
    {
      "field": "memorialCustomization.epitaph",
      "oldValue": "영원히 우리 마음속에",
      "newValue": "영원히 우리 마음속에. 누구든 바랄 수 있는 최고의 친구."
    }
  ],
  "result": {
    "success": true,
    "responseCode": 200
  }
}
```

---

## 부록 A: 프로토콜 준수 체크리스트

- [ ] 모든 API 엔드포인트가 TLS 1.3과 함께 HTTPS 사용
- [ ] WebSocket 연결이 WSS 프로토콜 사용
- [ ] 인증에 OAuth 2.0 또는 JWT 사용
- [ ] 모든 엔드포인트에 속도 제한 구현
- [ ] 중요한 작업에 Event Sourcing 사용
- [ ] 읽기/쓰기 분리를 위한 CQRS 패턴
- [ ] 모든 데이터 수정에 대한 감사 로깅
- [ ] 민감한 데이터에 대한 종단 간 암호화
- [ ] 데이터 이식성을 위한 GDPR 준수
- [ ] HMAC 서명을 사용한 웹훅 검증

---

**弘益人間 (홍익인간)** - 모든 인류에 이로움
© 2025 WIA
MIT License
