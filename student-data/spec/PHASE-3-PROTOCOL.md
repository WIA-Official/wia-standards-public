# WIA-EDU-010 — Phase 3: Protocol

> Student-data canonical Phase 3: real-time sync + conflict resolution + transfer.

# WIA-EDU-010 Student Data Standard v1.2

## Phase 3: Protocol & Synchronization

**Status:** ✅ Complete
**Version:** 1.2.0
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 introduces real-time synchronization, conflict resolution, and offline-first capabilities for student data systems. This phase enables seamless multi-device access and institutional data sharing.

## 2. Changes from v1.1

- Added real-time synchronization protocol
- Defined conflict resolution strategies
- Added offline-first architecture
- Specified data transfer protocols
- Added institutional data sharing framework
- Defined event streaming architecture

## 3. Synchronization Protocol

### 3.1 WebSocket Connection

**Endpoint:**
```
wss://sync.{institution}.edu/wia/v1/sync
```

**Connection Handshake:**
```json
{
  "action": "connect",
  "token": "Bearer {access_token}",
  "clientId": "client_abc123",
  "deviceId": "device_xyz789",
  "syncFrom": "2025-01-15T10:00:00Z"
}
```

**Server Response:**
```json
{
  "action": "connected",
  "syncId": "sync_123456",
  "serverTime": "2025-01-15T10:30:00Z",
  "pendingChanges": 42
}
```

### 3.2 Real-Time Updates

**Change Notification:**
```json
{
  "action": "change",
  "entity": "student",
  "operation": "update",
  "entityId": "2024-CS-001",
  "timestamp": "2025-01-15T10:30:15Z",
  "changes": {
    "email": {
      "old": "jane.old@university.edu",
      "new": "jane.smith@university.edu"
    }
  },
  "version": "v123",
  "author": "user_456"
}
```

## 4. Conflict Resolution

### 4.1 Operational Transformation

Use Operational Transformation (OT) for concurrent modifications.

**Conflict Example:**
```json
{
  "conflictId": "conflict_abc",
  "entity": "student",
  "entityId": "2024-CS-001",
  "field": "phone",
  "baseVersion": "v122",
  "changes": [
    {
      "version": "v123",
      "value": "+1-555-111-2222",
      "timestamp": "2025-01-15T10:30:00Z",
      "author": "user_123"
    },
    {
      "version": "v124",
      "value": "+1-555-333-4444",
      "timestamp": "2025-01-15T10:30:05Z",
      "author": "user_456"
    }
  ]
}
```

### 4.2 Resolution Strategies

1. **Last Write Wins (LWW):** Use most recent timestamp
2. **Manual Resolution:** Flag for human review
3. **Field-Level Merge:** Combine non-conflicting fields
4. **Priority-Based:** Higher authority wins

### 4.3 Conflict Notification

```json
{
  "action": "conflict",
  "conflictId": "conflict_abc",
  "requiresResolution": true,
  "resolutionStrategy": "manual",
  "conflictData": { }
}
```

## 5. Offline-First Architecture

### 5.1 Local Storage

- IndexedDB for structured data
- Service Worker for API caching
- Delta sync for bandwidth optimization

### 5.2 Sync Queue

```json
{
  "queueId": "queue_123",
  "operations": [
    {
      "id": "op_001",
      "action": "update",
      "entity": "student",
      "entityId": "2024-CS-001",
      "changes": { },
      "timestamp": "2025-01-15T10:25:00Z",
      "status": "pending"
    }
  ]
}
```

### 5.3 Sync Process

1. Client queues changes locally
2. When online, push changes to server
3. Server processes and responds
4. Client updates local state
5. Client pulls server changes
6. Conflicts resolved automatically or flagged

## 6. Data Transfer Protocol

### 6.1 Institution-to-Institution Transfer

**Transfer Request:**
```json
{
  "transferId": "transfer_abc123",
  "sourceInstitution": "university-a",
  "destinationInstitution": "university-b",
  "studentId": "2024-CS-001",
  "dataTypes": [
    "profile",
    "academic_records",
    "transcripts",
    "attendance"
  ],
  "authorizationCode": "AUTH-2025-XXYYZZ",
  "studentConsent": true,
  "consentTimestamp": "2025-01-15T09:00:00Z"
}
```

**Transfer Response:**
```json
{
  "transferId": "transfer_abc123",
  "status": "initiated",
  "estimatedCompletion": "2025-01-15T10:05:00Z",
  "trackingUrl": "https://transfer.wia.org/track/transfer_abc123"
}
```

### 6.2 Data Package Format

```json
{
  "packageId": "pkg_abc123",
  "version": "1.2.0",
  "standard": "WIA-EDU-010",
  "created": "2025-01-15T10:00:00Z",
  "sourceInstitution": {
    "id": "university-a",
    "name": "University A",
    "country": "US"
  },
  "encryption": {
    "algorithm": "AES-256-GCM",
    "keyId": "key_xyz789"
  },
  "signature": {
    "algorithm": "RSA-SHA256",
    "value": "base64_signature",
    "certificate": "base64_cert"
  },
  "data": {
    "student": { },
    "records": [ ],
    "transcripts": [ ]
  },
  "checksums": {
    "sha256": "hash_value"
  }
}
```

### 6.3 Transfer Security

- End-to-end encryption (AES-256-GCM)
- Digital signatures (RSA-SHA256)
- Certificate-based authentication
- Transfer authorization codes
- Audit logging at both institutions

## 7. Event Streaming

### 7.1 Event Stream API

```
GET /events/stream

Accept: text/event-stream
Authorization: Bearer {token}
```

**Event Format:**
```
event: student.updated
data: {"studentId": "2024-CS-001", "changes": {...}}
id: event_123
retry: 5000
```

### 7.2 Event Types

- `student.*` - Student profile events
- `record.*` - Academic record events
- `attendance.*` - Attendance events
- `privacy.*` - Privacy setting events
- `transfer.*` - Data transfer events

## 8. Delta Synchronization

### 8.1 Delta Request

```http
GET /students/{studentId}/delta?since=2025-01-15T10:00:00Z
```

**Delta Response:**
```json
{
  "deltaId": "delta_abc",
  "since": "2025-01-15T10:00:00Z",
  "until": "2025-01-15T10:30:00Z",
  "changes": [
    {
      "type": "update",
      "entity": "student",
      "entityId": "2024-CS-001",
      "field": "email",
      "oldValue": "old@email.com",
      "newValue": "new@email.com",
      "timestamp": "2025-01-15T10:15:00Z"
    }
  ],
  "hasMore": false
}
```

## 9. Bandwidth Optimization

### 9.1 Compression

- Gzip compression for API responses
- Binary Protocol Buffers for high-volume transfers
- Delta updates instead of full records

### 9.2 Selective Sync

```json
{
  "syncConfig": {
    "entities": ["student", "records"],
    "fields": ["firstName", "lastName", "email"],
    "since": "2025-01-15T10:00:00Z",
    "limit": 100
  }
}
```

## 10. Multi-Tenant Support

### 10.1 Tenant Isolation

- Separate databases per institution
- Encrypted at-rest with tenant-specific keys
- Row-level security policies

### 10.2 Cross-Tenant Transfer

- Requires explicit authorization
- Audit trail at both tenants
- GDPR-compliant transfer logs

## 11. Monitoring and Health

### 11.1 Health Check

```http
GET /health

{
  "status": "healthy",
  "version": "1.2.0",
  "services": {
    "database": "healthy",
    "sync": "healthy",
    "transfer": "healthy"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 11.2 Metrics

- Sync latency
- Conflict rate
- Transfer success rate
- Active connections
- Queue depth

---

**Philosophy:** 弘益人間 (Benefit All Humanity)

*Enabling seamless data synchronization while maintaining security and privacy.*

---

## A.1 Synchronization protocol (WebSocket)

The sync WebSocket runs at `wss://sync.{institution}.edu/wia/v1/sync`. Handshake carries the bearer token, client identifier, device identifier, and a `syncFrom` watermark; the server replies with a sync identifier, server time, and the count of pending changes. Real-time change notifications carry the entity, operation, entity identifier, change set, version, and author.

## A.2 Conflict resolution

Concurrent modifications to the same field are detected at write time and surfaced to subscribers with a typed `conflict` event. Resolution strategies: last-write-wins, manual review, field-level merge, and priority-based (registrar > instructor > student). The default is field-level merge with manual fallback when both sides modify the same scalar.

## A.3 Offline-first architecture

Clients buffer mutations locally (IndexedDB or platform equivalent) and replay them when connectivity returns. The sync queue carries an idempotency key per mutation so duplicate replays are detected and discarded. Delta sync compresses the over-the-wire payload by carrying only changed fields.

## A.4 Institution-to-institution transfer

`POST /transfers` initiates a student-record transfer between institutions. The envelope carries the source and destination institution identifiers, the student identifier, the requested data types (profile, academic records, transcripts, attendance), an authorization code, and the student's signed consent. Transfers are end-to-end encrypted with AES-256-GCM and signed with RSA-SHA256 or Ed25519; both institutions retain audit logs for the FERPA-mandated retention period.

## A.5 Event streaming

Server-sent events are exposed at `GET /events/stream`. Event types: `student.*`, `record.*`, `attendance.*`, `privacy.*`, `transfer.*`. Each event carries a stable identifier so clients can re-establish position after a reconnect. Event payloads are signed by the issuing institution's key so downstream consumers can verify provenance.

## A.6 Replay defence and bandwidth

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for control-plane calls. Bandwidth is reduced through gzip on JSON, Protocol Buffers for high-volume transfers, and delta-only updates. Selective sync allows clients to request a subset of entities and fields.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/student-data/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-student-data-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/student-data-host:1.0.0` ships every student-data envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/student-data.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Student-data deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
