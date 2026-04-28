# WIA-EDU-006: Virtual Classroom Standard
# PHASE 1 - Foundation

## Version: 1.0
## Status: Active
## Last Updated: 2025-12-25

---

## 1. Executive Summary

PHASE 1 establishes the foundational infrastructure for WIA-EDU-006 compliant virtual classrooms. This phase focuses on core functionality, basic security, and essential user experience elements necessary for conducting synchronous online education.

**Key Objectives:**
- ✅ Real-time video and audio communication
- ✅ Basic session management (create, join, leave)
- ✅ Text chat functionality
- ✅ Screen sharing capabilities
- ✅ Participant roster management
- ✅ WCAG 2.1 AA accessibility baseline

---

## 2. Core Requirements

### 2.1 Real-Time Communication

**Video Conferencing**
- **Protocol:** WebRTC with SFU architecture
- **Resolution:** Minimum 720p (HD), Adaptive up to 1080p
- **Frame Rate:** 30 fps (standard), 24 fps (minimum acceptable)
- **Codec:** VP9 (primary), H.264 (fallback)
- **Bitrate:** Adaptive 500 kbps - 5 Mbps based on network
- **Latency:** < 200ms end-to-end

**Audio Communication**
- **Codec:** Opus (primary), AAC (fallback)
- **Sample Rate:** 48 kHz
- **Bitrate:** 50-100 kbps
- **Latency:** < 150ms
- **Features:** Echo cancellation, noise suppression, automatic gain control

**Quality of Service**
```yaml
minimum_requirements:
  bandwidth_upload: 1.5 Mbps
  bandwidth_download: 2.5 Mbps
  latency: <150ms (audio), <200ms (video)
  jitter: <30ms
  packet_loss: <1%

recommended_requirements:
  bandwidth_upload: 3 Mbps
  bandwidth_download: 5 Mbps
  latency: <100ms
  jitter: <20ms
  packet_loss: <0.5%
```

### 2.2 Session Management

**Session Lifecycle**
1. **Creation:** Instructor creates session with parameters
2. **Scheduling:** Start time, end time, recurrence (optional)
3. **Admission:** Participants join via link or code
4. **Active:** Real-time interaction period
5. **Termination:** Manual end or scheduled conclusion
6. **Archival:** Recordings and metadata stored

**Session Parameters**
```json
{
  "id": "session-uuid",
  "name": "Course Name - Session Title",
  "type": "live|webinar|hybrid|recorded",
  "maxParticipants": 100,
  "startTime": "2025-12-25T10:00:00Z",
  "endTime": "2025-12-25T11:30:00Z",
  "timeZone": "America/New_York",
  "features": {
    "chat": true,
    "screenShare": true,
    "recording": false,
    "waitingRoom": true
  }
}
```

### 2.3 User Roles and Permissions

**Role Hierarchy**
| Role | Permissions | Use Case |
|------|------------|----------|
| **Owner** | Full control, delete session | Institution admin |
| **Instructor** | Manage session, admit users, control features | Teacher, professor |
| **Co-Host/TA** | Moderate chat, manage breakouts | Teaching assistant |
| **Participant** | Join, interact (chat, reactions) | Student |
| **Observer** | View-only, invisible to participants | Quality assurance |

**Permission Matrix**
```yaml
permissions:
  create_session: [owner, instructor]
  delete_session: [owner]
  start_recording: [owner, instructor]
  mute_participant: [owner, instructor, co-host]
  remove_participant: [owner, instructor]
  share_screen: [owner, instructor, co-host, participant_with_permission]
  send_message: [all_except_observer]
  raise_hand: [participant]
```

### 2.4 Text Chat

**Features**
- Public chat (visible to all)
- Private messages (1-on-1)
- Message types: text, links, emojis
- Message length: 500 characters maximum
- Rate limiting: 5 messages/minute per user

**Chat Data Structure**
```typescript
interface Message {
  id: string;
  sessionId: string;
  senderId: string;
  senderName: string;
  recipientId?: string; // null = public
  content: string;
  timestamp: Date;
  type: 'text' | 'system' | 'announcement';
  edited: boolean;
  deleted: boolean;
}
```

### 2.5 Screen Sharing

**Technical Specifications**
- **Protocol:** WebRTC getDisplayMedia API
- **Frame Rate:** 15-30 fps
- **Resolution:** Match source display, max 1920x1080
- **Bitrate:** 1-3 Mbps adaptive
- **Audio:** Include system audio (optional)

**Controls**
- Only one screen share active at a time (instructor priority)
- Automatic stop when presenter leaves
- Viewing modes: Fit to window, original size

---

## 3. User Interface Requirements

### 3.1 Layout

**Primary View Components**
```
┌─────────────────────────────────────────────────┐
│  Header (Session name, time, controls)         │
├───────────────────┬─────────────────────────────┤
│                   │                             │
│  Video Grid       │  Sidebar                    │
│  (Participants)   │  - Participant List         │
│                   │  - Chat                     │
│                   │  - Shared Content           │
│                   │                             │
├───────────────────┴─────────────────────────────┤
│  Control Bar (Mic, Camera, Share, Leave)       │
└─────────────────────────────────────────────────┘
```

**Responsive Breakpoints**
- Desktop: > 1024px (grid layout)
- Tablet: 768-1023px (stacked layout)
- Mobile: < 768px (single column, swipeable tabs)

### 3.2 Accessibility (WCAG 2.1 AA)

**Keyboard Navigation**
- All features accessible via keyboard
- Logical tab order
- Visible focus indicators
- Keyboard shortcuts documented

**Screen Reader Support**
- Semantic HTML5 elements
- ARIA labels and roles
- Live region announcements for dynamic content
- Alt text for all images

**Visual Accessibility**
- Color contrast ratio: ≥ 4.5:1 for normal text
- Color contrast ratio: ≥ 3:1 for large text
- Text resizable to 200% without loss of functionality
- No reliance on color alone to convey information

**Captions**
- Auto-generated captions (minimum)
- Human-corrected captions (recommended)
- Caption positioning, size, color customizable

---

## 4. Security Requirements

### 4.1 Authentication

**Supported Methods**
1. **Password-based:** Username + password
2. **OAuth 2.0:** Google, Microsoft, etc.
3. **SAML 2.0:** Enterprise SSO
4. **LTI 1.3:** LMS integration

**Session Security**
- JWT tokens with 1-hour expiration
- Refresh tokens (7 days)
- Secure cookie storage (HttpOnly, SameSite=Strict)

### 4.2 Encryption

**In Transit**
- TLS 1.3 for all HTTP/WebSocket traffic
- DTLS-SRTP for WebRTC media streams
- Certificate pinning for mobile apps

**At Rest**
- AES-256 for stored content and recordings
- Database encryption for sensitive fields
- Encrypted backups

### 4.3 Access Control

**Waiting Room**
- Default enabled for public sessions
- Instructor approves admission
- Queue management (FIFO or manual)

**Session Locking**
- Instructor can lock session after start
- Late arrivals require explicit admission
- Prevents zoom-bombing and unauthorized access

---

## 5. Performance Standards

### 5.1 Scalability

**PHASE 1 Targets**
| Metric | Target |
|--------|--------|
| Concurrent participants per session | 100 |
| Concurrent active sessions | 500 |
| Messages per second (chat) | 1000 |
| API requests per second | 5000 |
| Page load time | < 2 seconds |
| Time to first video frame | < 3 seconds |

### 5.2 Reliability

**Uptime**
- Target: 99.5% monthly uptime
- Planned maintenance windows: 4 hours/month maximum
- Automated failover for critical services

**Error Handling**
- Graceful degradation (audio-only fallback if video fails)
- Automatic reconnection (3 attempts with exponential backoff)
- Clear error messages with actionable guidance

---

## 6. Data Requirements

### 6.1 Data Model

**Core Entities**
```sql
-- Users
CREATE TABLE users (
  id UUID PRIMARY KEY,
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255) NOT NULL,
  role VARCHAR(50) NOT NULL,
  created_at TIMESTAMP DEFAULT NOW()
);

-- Sessions
CREATE TABLE sessions (
  id UUID PRIMARY KEY,
  name VARCHAR(255) NOT NULL,
  type VARCHAR(50) NOT NULL,
  start_time TIMESTAMP NOT NULL,
  end_time TIMESTAMP,
  max_participants INTEGER DEFAULT 100,
  created_by UUID REFERENCES users(id),
  created_at TIMESTAMP DEFAULT NOW()
);

-- Participants
CREATE TABLE session_participants (
  session_id UUID REFERENCES sessions(id) ON DELETE CASCADE,
  user_id UUID REFERENCES users(id),
  role VARCHAR(50) NOT NULL,
  joined_at TIMESTAMP DEFAULT NOW(),
  left_at TIMESTAMP,
  PRIMARY KEY (session_id, user_id, joined_at)
);

-- Messages
CREATE TABLE messages (
  id UUID PRIMARY KEY,
  session_id UUID REFERENCES sessions(id) ON DELETE CASCADE,
  sender_id UUID REFERENCES users(id),
  recipient_id UUID REFERENCES users(id), -- NULL for public
  content TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  edited BOOLEAN DEFAULT FALSE,
  deleted BOOLEAN DEFAULT FALSE
);
```

### 6.2 Data Retention

**PHASE 1 Policies**
- Session metadata: 1 year
- Chat logs: 90 days (end of term + 30 days)
- User data: Duration of account + 30 days post-deletion request
- Analytics: Aggregated data only, 1 year

---

## 7. API Specifications

### 7.1 RESTful Endpoints

**Sessions**
```http
POST   /api/v1/sessions
GET    /api/v1/sessions/:id
PUT    /api/v1/sessions/:id
DELETE /api/v1/sessions/:id
GET    /api/v1/sessions (list with pagination)

POST   /api/v1/sessions/:id/join
POST   /api/v1/sessions/:id/leave
```

**Participants**
```http
GET    /api/v1/sessions/:id/participants
POST   /api/v1/sessions/:id/participants/:userId/admit
DELETE /api/v1/sessions/:id/participants/:userId
```

**Messages**
```http
POST   /api/v1/sessions/:id/messages
GET    /api/v1/sessions/:id/messages (with pagination)
```

### 7.2 WebSocket Events

**Client → Server**
```javascript
{
  "type": "join_session",
  "sessionId": "uuid",
  "userId": "uuid"
}

{
  "type": "send_message",
  "sessionId": "uuid",
  "content": "Hello everyone"
}

{
  "type": "toggle_audio",
  "enabled": false
}
```

**Server → Client**
```javascript
{
  "type": "participant_joined",
  "participant": {
    "id": "uuid",
    "name": "John Doe",
    "role": "student"
  }
}

{
  "type": "message_received",
  "message": {...}
}

{
  "type": "session_ended"
}
```

---

## 8. Testing Requirements

### 8.1 Unit Tests

**Coverage Target:** ≥ 80%

**Critical Components**
- Session lifecycle management
- User authentication and authorization
- Message delivery and ordering
- WebRTC connection establishment

### 8.2 Integration Tests

**Scenarios**
- User joins session and sees video
- Messages delivered to all participants
- Screen sharing visible to all
- Participant removed by instructor

### 8.3 Performance Tests

**Load Testing**
- 100 concurrent participants in single session
- 500 concurrent active sessions
- 1000 messages per second throughput

---

## 9. Deployment Requirements

### 9.1 Infrastructure

**Minimum Production Environment**
```yaml
application_servers:
  count: 3
  cpu: 4 cores
  memory: 16 GB
  storage: 100 GB

media_servers:
  count: 2
  cpu: 8 cores
  memory: 32 GB
  network: 10 Gbps

database:
  type: PostgreSQL 14+
  cpu: 4 cores
  memory: 16 GB
  storage: 500 GB SSD

cache:
  type: Redis 7+
  memory: 8 GB
```

### 9.2 Monitoring

**Required Metrics**
- Application: Request rate, response time, error rate
- Infrastructure: CPU, memory, disk, network
- Media: Bitrate, packet loss, jitter, latency
- Business: Active sessions, concurrent users

**Alerting Thresholds**
- Error rate > 1%
- Response time > 500ms (p95)
- CPU > 80%
- Memory > 90%

---

## 10. Success Criteria

PHASE 1 is considered complete when:

✅ 100 concurrent participants can join a single session with acceptable quality
✅ All core features (video, audio, chat, screen share) function reliably
✅ WCAG 2.1 AA accessibility verified by third-party audit
✅ Security audit shows no critical or high vulnerabilities
✅ 99.5% uptime achieved over 30-day period
✅ User acceptance testing passes with ≥ 85% satisfaction

---

## 11. Next Steps

After PHASE 1 completion, proceed to:
- **PHASE 2:** Enhanced features (breakout rooms, polls, whiteboard)
- **PHASE 3:** Advanced integration (LMS, calendar, third-party tools)
- **PHASE 4:** AI-powered features and optimization

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
