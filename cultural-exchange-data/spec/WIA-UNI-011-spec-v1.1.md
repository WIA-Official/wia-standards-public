# WIA-UNI-011: Cultural Exchange Data Standard
## Specification v1.1

**Status:** Released  
**Date:** 2025-04-01  
**Category:** Unification/Peace (UNI)  
**Changes from v1.0:** Enhanced media API, real-time collaboration features

---

## Changelog from v1.0

### Added
- WebSocket protocol for real-time collaboration
- Enhanced media upload with resumable transfers
- Participant verification with W3C Verifiable Credentials
- Advanced analytics endpoints

### Changed
- Heritage schema now supports UNESCO thesaurus integration
- Event schema includes virtual/hybrid event support
- Improved multilingual search with semantic matching

### Deprecated
- Legacy participant ID format (will be removed in v2.0)

---

## 1. Enhanced Media API

### 1.1 Resumable Uploads

Large media files support resumable uploads using TUS protocol.

```
POST /media/upload-url
{
  "filename": "cultural-performance.mp4",
  "size": 524288000,
  "mimeType": "video/mp4",
  "resumable": true
}
```

### 1.2 Streaming Support

Media API now supports HLS and DASH protocols for adaptive streaming of cultural performances.

---

## 2. Real-Time Collaboration

### 2.1 WebSocket Protocol

```
wss://collab.cultural-exchange.org/events/{eventId}

Message Format:
{
  "type": "EVENT_UPDATE",
  "timestamp": "ISO 8601",
  "user": { "id": "string", "name": "string" },
  "changes": { "field": "string", "action": "string", "value": "any" }
}
```

Enables joint event planning between North and South Korean organizers with live updates.

---

## 3. Verifiable Credentials

### 3.1 Participant Credentials

W3C Verifiable Credentials for event participants, artists, and organizers.

```json
{
  "@context": ["https://www.w3.org/2018/credentials/v1"],
  "type": ["VerifiableCredential", "CulturalEventCredential"],
  "issuer": "did:wia:uni-011:issuer",
  "credentialSubject": {
    "id": "did:wia:participant:123",
    "eventId": "UNI-011-MUSIC-2025-001",
    "role": "artist",
    "permissions": ["event-access", "media-documentation"]
  }
}
```

---

## 4. Analytics Enhancements

New endpoints for cultural exchange trend analysis, participation metrics, and impact measurement.

```
GET /analytics/trends?period=2025-01:2025-12&type=music
GET /analytics/participation?region=both&granularity=month
GET /analytics/heritage/preservation-status
```

---

## 5. Backward Compatibility

v1.1 maintains full backward compatibility with v1.0 clients. All deprecated features continue to work with deprecation warnings.

---

**Document ID:** WIA-UNI-011-SPEC-v1.1  
**Previous Version:** v1.0  
**Next Version:** v1.2 (Planned: 2025-07-01)

© 2025 SmileStory Inc. / WIA · MIT License
