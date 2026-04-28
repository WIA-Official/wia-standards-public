# WIA-EDU-006: Virtual Classroom Standard
# PHASE 2 - Implementation & Enhancement

## Version: 1.0
## Status: Active
## Prerequisites: PHASE 1 Complete

---

## 1. Overview

PHASE 2 builds upon the foundation established in PHASE 1, adding advanced interactive features that enhance pedagogical effectiveness and student engagement. This phase focuses on collaborative tools, assessment capabilities, and improved content management.

**Key Additions:**
- 🎯 Breakout rooms for small group work
- 📊 Polls and surveys
- 🎨 Interactive whiteboard
- 📝 Live Q&A system
- 👋 Reactions and non-verbal feedback
- 📹 Recording and playback
- 📚 Enhanced content library

---

## 2. Breakout Rooms

### 2.1 Requirements

**Core Functionality**
- Create up to 50 breakout rooms per session
- 2-20 participants per breakout room
- Manual or automatic assignment
- Instructor can broadcast messages to all rooms
- Timer with audio/visual countdown
- Automatic return to main session

**Room Configuration**
```typescript
interface BreakoutRoom {
  id: string;
  name: string;
  sessionId: string;
  participants: string[]; // user IDs
  duration: number; // minutes
  startTime?: Date;
  endTime?: Date;
  status: 'pending' | 'active' | 'ended';
  allowParticipantsToReturn: boolean;
  recordingEnabled: boolean;
}
```

### 2.2 Technical Implementation

**Architecture**
- Each breakout room is a child WebRTC session
- Separate SFU media routing per room
- Persistent main session connection for instant return
- Optimized bandwidth (participants only receive media from their room)

**API Endpoints**
```http
POST   /api/v1/sessions/:id/breakouts
GET    /api/v1/sessions/:id/breakouts
PUT    /api/v1/sessions/:id/breakouts/:breakoutId
DELETE /api/v1/sessions/:id/breakouts/:breakoutId

POST   /api/v1/sessions/:id/breakouts/:breakoutId/assign
POST   /api/v1/sessions/:id/breakouts/:breakoutId/broadcast
POST   /api/v1/sessions/:id/breakouts/close-all
```

### 2.3 User Experience

**Instructor Workflow**
1. Click "Breakout Rooms" button
2. Set number of rooms (or let system auto-calculate)
3. Choose assignment: Manual, Random, or Previous Groups
4. Set duration (5-60 minutes recommended)
5. Participants automatically moved to rooms
6. Monitor rooms (view participant lists, join any room)
7. Broadcast messages or close all rooms

**Participant Experience**
- Notification: "You've been assigned to Breakout Room 3"
- Automatic transition (countdown: 10 seconds)
- Isolated audio/video space
- Option to request help (notifies instructor)
- Timer visible in UI
- Automatic return when time expires or instructor closes rooms

---

## 3. Polls and Surveys

### 3.1 Poll Types

**Multiple Choice**
```typescript
interface MultipleChoicePoll {
  question: string;
  options: string[]; // 2-10 options
  allowMultiple: boolean;
  correctAnswer?: number[]; // for quiz mode
  anonymous: boolean;
}
```

**Yes/No/Abstain**
```typescript
interface BinaryPoll {
  question: string;
  type: 'yes_no' | 'true_false' | 'agree_disagree';
  anonymous: boolean;
}
```

**Rating Scale**
```typescript
interface RatingPoll {
  question: string;
  min: number; // typically 1
  max: number; // typically 5 or 10
  labels: {min: string, max: string}; // e.g., "Not at all" / "Very much"
  anonymous: boolean;
}
```

### 3.2 Features

**Real-time Results**
- Live vote counting as participants respond
- Visual representations: Bar chart, pie chart, word cloud
- Export results as CSV/PDF
- Compare with previous polls (trend analysis)

**Quiz Mode**
- Correct answers defined
- Automatic scoring
- Leaderboard display (if not anonymous)
- Integration with LMS gradebook

### 3.3 API Specification

```http
POST   /api/v1/sessions/:id/polls
GET    /api/v1/sessions/:id/polls
GET    /api/v1/sessions/:id/polls/:pollId
DELETE /api/v1/sessions/:id/polls/:pollId

POST   /api/v1/sessions/:id/polls/:pollId/open
POST   /api/v1/sessions/:id/polls/:pollId/close
POST   /api/v1/sessions/:id/polls/:pollId/responses
GET    /api/v1/sessions/:id/polls/:pollId/results
```

---

## 4. Interactive Whiteboard

### 4.1 Drawing Tools

**Basic Tools**
- Pen (freehand drawing)
- Highlighter (semi-transparent)
- Shapes (rectangle, circle, line, arrow)
- Text insertion
- Eraser
- Select/Move tool

**Colors and Styles**
- Color palette: 12 preset + custom color picker
- Line width: 1-10px
- Fill options for shapes

### 4.2 Advanced Features

**Collaboration**
- Multiple users drawing simultaneously
- User color coding (each user gets unique color)
- Conflict resolution (last-write-wins with timestamps)
- Undo/Redo (per user, 50 actions)

**Content Integration**
- Upload images as background
- PDF annotation
- Screen capture to whiteboard
- Export as PNG/PDF
- Save and load whiteboard states

### 4.3 Technical Architecture

**Data Structure**
```typescript
interface WhiteboardStroke {
  id: string;
  userId: string;
  type: 'pen' | 'highlighter' | 'shape' | 'text' | 'eraser';
  color: string;
  width: number;
  points: Point[]; // {x, y, pressure?}
  timestamp: number;
}

interface WhiteboardState {
  sessionId: string;
  strokes: WhiteboardStroke[];
  backgroundImage?: string; // URL or base64
  dimensions: {width: number, height: number};
  version: number; // for conflict resolution
}
```

**Synchronization**
- WebSocket for real-time stroke transmission
- Operational Transformation (OT) for conflict resolution
- Periodic snapshots (every 100 strokes)
- Lazy loading for large whiteboards

---

## 5. Live Q&A System

### 5.1 Features

**Question Submission**
- Participants submit questions via dedicated Q&A panel
- Optional anonymous submission
- Character limit: 500
- Attach images/links (optional)

**Moderation**
- Instructor can dismiss/answer questions
- Upvoting system (participants +1 questions)
- Sort by: Time, Upvotes, Unanswered
- Mark as answered with text response or "Answered verbally"

**Integration**
- Export Q&A log as PDF
- Unanswered questions carried over to next session
- Email digest to participants

### 5.2 Data Model

```typescript
interface Question {
  id: string;
  sessionId: string;
  authorId: string;
  authorName: string; // "Anonymous" if anonymous submission
  content: string;
  upvotes: string[]; // array of user IDs who upvoted
  answered: boolean;
  answer?: string;
  answeredAt?: Date;
  createdAt: Date;
}
```

---

## 6. Reactions and Non-Verbal Feedback

### 6.1 Quick Reactions

**Emoji Reactions**
- 👍 Agree / Like
- 👎 Disagree
- ❤️ Love this
- 😂 Funny
- 🎉 Celebrate
- 🤔 Confused

**Display**
- Floating animation (emoji rises from participant video)
- Aggregate count shown to instructor
- Disappears after 5 seconds

### 6.2 Status Indicators

**Participant Status**
- ✋ Raised hand (persistent until lowered)
- ⏰ Away / Break
- ❓ Need help
- ⚡ Slow down
- 🏃 Speed up
- ✅ Done / Ready

**Instructor View**
- Notification badge when hands raised
- Filter/sort participant list by status
- Clear all statuses with one click

---

## 7. Recording and Playback

### 7.1 Recording Features

**What's Recorded**
- Active speaker video (auto-switching)
- Gallery view (all participants)
- Shared screen/content
- Audio (mixed or separate tracks)
- Chat transcript
- Polls and Q&A
- Whiteboard activity

**Recording Modes**
```yaml
modes:
  cloud: 
    storage: Cloud storage (S3/Azure/GCS)
    processing: Server-side encoding
    accessibility: Automatic captions
  local:
    storage: Instructor's device
    processing: Client-side encoding
    accessibility: Manual caption upload
```

### 7.2 Playback Interface

**Video Player**
- Standard controls: Play, pause, seek, volume
- Playback speed: 0.5x, 0.75x, 1x, 1.25x, 1.5x, 2x
- Captions toggle
- Quality selector
- Picture-in-picture mode

**Interactive Transcript**
- Click transcript text to jump to that moment
- Search within transcript
- Download transcript (TXT, SRT, VTT)

**Chapters and Bookmarks**
- Auto-generated chapters (every 10 minutes or on screen share change)
- Manual bookmarks by instructor
- Jump to polls, Q&A, whiteboard sessions

### 7.3 Storage and Delivery

**Compression**
- Video codec: H.264 (compatibility) or AV1 (efficiency)
- Audio codec: AAC
- Target bitrate: 1.5 Mbps (720p), 2.5 Mbps (1080p)

**CDN Delivery**
- HLS streaming for adaptive bitrate
- Pre-signed URLs with expiration
- Geo-distributed caching

**Retention**
```yaml
default_policy:
  recordings: End of term + 1 year
  transcripts: End of term + 2 years
  analytics: Aggregated data indefinitely
```

---

## 8. Enhanced Content Library

### 8.1 Content Types

**Supported Formats**
- Documents: PDF, DOCX, PPTX, XLSX
- Images: JPG, PNG, GIF, SVG, WebP
- Video: MP4, WebM, MOV (transcoded to web formats)
- Audio: MP3, WAV, AAC
- Archives: ZIP (auto-extracted for browsing)
- Web content: HTML5, SCORM packages

### 8.2 Features

**Organization**
- Folders and subfolders
- Tags and metadata
- Search by name, tag, content (full-text)
- Version control (track updates)

**Sharing**
- Share to specific sessions
- Share with all sessions in a course
- Share with individuals
- Public link generation (optional expiration)

**Previews**
- Thumbnail generation
- In-browser preview (no download required)
- Annotation tools (comments, highlights)

### 8.3 Integration

**LMS Sync**
- Bi-directional sync with LMS content libraries
- Automatic import of course materials
- Grade passback for submitted assignments

**Third-Party Tools**
- Google Drive integration
- OneDrive/SharePoint integration
- Dropbox integration
- YouTube embed

---

## 9. Performance Optimizations

### 9.1 Scalability Improvements

**PHASE 2 Targets**
| Metric | PHASE 1 | PHASE 2 | Improvement |
|--------|---------|---------|-------------|
| Max participants/session | 100 | 300 | +200% |
| Concurrent sessions | 500 | 2000 | +300% |
| Breakout rooms/session | N/A | 50 | New |
| Whiteboard strokes/second | N/A | 500 | New |

**Architecture Enhancements**
- Horizontal scaling for media servers
- Database read replicas for analytics
- Redis cluster for session state
- CDN for static assets and recordings

### 9.2 Bandwidth Optimization

**Simulcast**
- Each participant sends 3 video streams: Low (180p), Med (360p), High (720p)
- SFU selects appropriate stream per recipient based on bandwidth
- Reduces overall bandwidth consumption by 40-60%

**Dynamic Quality Adjustment**
- Monitor network conditions every 2 seconds
- Automatic quality downgrade if packet loss > 5%
- Visual indicator to user when quality reduced

---

## 10. Accessibility Enhancements

### 10.1 New Features

**Keyboard Shortcuts**
- Raise hand: Alt+H
- Toggle mute: Alt+M
- Toggle video: Alt+V
- Leave session: Alt+L
- Open chat: Alt+C
- Open Q&A: Alt+Q

**Screen Reader Improvements**
- Live announcements for participant join/leave
- Status updates (poll started, breakout rooms opening)
- Whiteboard stroke descriptions (optional, performance impact)

**Visual Enhancements**
- High contrast mode
- Adjustable UI scaling (100%-200%)
- Color blind friendly palette options

---

## 11. Success Criteria

PHASE 2 is complete when:

✅ 300 concurrent participants in single session with breakout rooms
✅ Whiteboard synchronization latency < 100ms
✅ Recording generated within 15 minutes of session end
✅ Poll response time < 2 seconds from submission to display
✅ Accessibility audit confirms continued WCAG 2.1 AA compliance
✅ User satisfaction ≥ 90% for new features

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
