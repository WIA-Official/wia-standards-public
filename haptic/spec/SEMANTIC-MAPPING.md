# WIA Haptic Semantic Mapping Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines the semantic categories and meanings assigned to haptic patterns. By establishing consistent mappings between tactile sensations and their intended meanings, users can develop intuitive understanding of haptic feedback without visual reference.

## 2. Design Philosophy

### 2.1 Natural Metaphors
- Patterns should align with intuitive physical metaphors
- "Heavy" sensations for importance, "light" for information
- Spatial patterns that match physical direction

### 2.2 Learnability
- Core vocabulary of 10-15 distinct patterns
- Hierarchical complexity (basic → advanced)
- Consistent patterns across contexts

### 2.3 Cultural Universality
- Avoid culturally-specific associations
- Based on physical/physiological responses
- Tested across diverse user populations

## 3. Semantic Categories

### 3.1 Category Enumeration

```typescript
enum HapticCategory {
  NAVIGATION = 'navigation',      // 방향, 거리, 장애물
  NOTIFICATION = 'notification',  // 알림, 경고
  CONFIRMATION = 'confirmation',  // 성공, 실패, 인정
  SPATIAL = 'spatial',            // 공간 정보
  TEMPORAL = 'temporal',          // 시간 정보
  SOCIAL = 'social',              // 사회적 신호
  SYSTEM = 'system',              // 시스템 상태
  CONTENT = 'content',            // 콘텐츠 탐색
}
```

### 3.2 Category Hierarchy

```
HapticCategory
├── NAVIGATION
│   ├── Direction
│   ├── Distance
│   ├── Obstacle
│   └── Landmark
├── NOTIFICATION
│   ├── Alert
│   ├── Reminder
│   └── Message
├── CONFIRMATION
│   ├── Success
│   ├── Failure
│   └── Acknowledgment
├── SPATIAL
│   ├── Layout
│   ├── Boundary
│   └── Position
├── TEMPORAL
│   ├── Timer
│   ├── Progress
│   └── Rhythm
├── SOCIAL
│   ├── Presence
│   ├── Attention
│   └── Emotion
├── SYSTEM
│   ├── Status
│   ├── Mode
│   └── Error
└── CONTENT
    ├── Selection
    ├── Scroll
    └── Type
```

## 4. Navigation Patterns

### 4.1 Direction Indicators

| Pattern ID | Symbol | Description | Haptic Signature |
|------------|--------|-------------|------------------|
| `nav_forward` | ▲ | Proceed straight | Single pulse, medium intensity |
| `nav_left` | ◀ | Turn left | Left-weighted pulse pattern |
| `nav_right` | ▶ | Turn right | Right-weighted pulse pattern |
| `nav_back` | ▼ | Go back | Reverse ramp pattern |
| `nav_arrived` | ◉ | Destination reached | Success melody pattern |

### 4.2 Distance Encoding

```typescript
interface DistancePattern {
  // Pulse rate increases as distance decreases
  calculatePulseRate(distanceMeters: number): number;

  // Intensity increases as distance decreases
  calculateIntensity(distanceMeters: number): number;
}

const DISTANCE_MAPPINGS = {
  FAR: {        // > 10m
    pulseRate: 0.5,     // Hz
    intensity: 0.3,
  },
  MEDIUM: {     // 3-10m
    pulseRate: 2,       // Hz
    intensity: 0.5,
  },
  NEAR: {       // 1-3m
    pulseRate: 5,       // Hz
    intensity: 0.7,
  },
  IMMEDIATE: {  // < 1m
    pulseRate: 10,      // Hz
    intensity: 0.9,
  },
};
```

### 4.3 Obstacle Patterns

| Pattern ID | Condition | Pattern | Urgency |
|------------|-----------|---------|---------|
| `obstacle_detected` | Object detected | Soft noise texture | Low |
| `obstacle_near` | 1-3m distance | Increasing pulse | Medium |
| `obstacle_warning` | < 1m | Rapid strong pulses | High |
| `obstacle_critical` | < 0.5m | Continuous strong vibration | Critical |

### 4.4 Path Patterns

```typescript
const PATH_PATTERNS = {
  // Clear path ahead
  CLEAR_PATH: {
    pattern: '▁▁▁',
    description: 'Low, continuous, reassuring',
    waveform: 'sine',
    frequency: 50,
    intensity: 0.3,
    duration: 500,
  },

  // Path with obstacles
  OBSTRUCTED_PATH: {
    pattern: '▁▓▁',
    description: 'Interrupted by texture',
    waveform: 'noise',
    frequency: 100,
    intensity: 0.6,
  },

  // Narrow passage
  NARROW_PATH: {
    pattern: '▓▁▓',
    description: 'Boundaries indicated on sides',
    bilateral: true,
  },

  // Stairs/steps
  STAIRS_UP: {
    pattern: '▂▄▆█',
    description: 'Ascending intensity',
  },

  STAIRS_DOWN: {
    pattern: '█▆▄▂',
    description: 'Descending intensity',
  },
};
```

## 5. Notification Patterns

### 5.1 Alert Levels

```typescript
enum AlertLevel {
  INFO = 'info',           // 정보성 알림
  NOTICE = 'notice',       // 주의 필요
  WARNING = 'warning',     // 경고
  URGENT = 'urgent',       // 긴급
  CRITICAL = 'critical',   // 위험
}

const ALERT_PATTERNS: Record<AlertLevel, HapticPattern> = {
  info: {
    pulses: 1,
    intensity: 0.4,
    frequency: 80,
    character: 'gentle',
  },
  notice: {
    pulses: 2,
    intensity: 0.5,
    frequency: 120,
    character: 'attention',
  },
  warning: {
    pulses: 3,
    intensity: 0.7,
    frequency: 180,
    character: 'alerting',
  },
  urgent: {
    pulses: 5,
    intensity: 0.85,
    frequency: 220,
    character: 'insistent',
  },
  critical: {
    pulses: 'continuous',
    intensity: 1.0,
    frequency: 280,
    character: 'alarming',
  },
};
```

### 5.2 Message Type Patterns

| Pattern ID | Message Type | Signature |
|------------|--------------|-----------|
| `msg_text` | Text message | Short double tap |
| `msg_voice` | Voice message | Pulsing wave pattern |
| `msg_image` | Image received | Texture sweep |
| `msg_video` | Video received | Rhythmic pulses |
| `msg_call` | Incoming call | Repeated long pulses |

### 5.3 Calendar/Reminder Patterns

```typescript
const REMINDER_PATTERNS = {
  // Approaching event (5 min)
  EVENT_SOON: {
    pattern: 'gentle_pulse_x3',
    intensity: 0.5,
  },

  // Event now
  EVENT_NOW: {
    pattern: 'strong_pulse_x5',
    intensity: 0.8,
  },

  // Event overdue
  EVENT_OVERDUE: {
    pattern: 'persistent_pulse',
    intensity: 0.6,
    repeat: true,
  },
};
```

## 6. Confirmation Patterns

### 6.1 Success/Failure Dichotomy

```typescript
const CONFIRMATION_PATTERNS = {
  // Success - ascending, pleasant
  SUCCESS: {
    sequence: ['tick', 'tick_high'],
    character: 'ascending',
    metaphor: 'Rising, positive',
    waveform: 'sine',
    frequencies: [100, 150],
    intensity: 0.6,
  },

  // Failure - descending, definitive
  FAILURE: {
    sequence: ['buzz', 'buzz_low'],
    character: 'descending',
    metaphor: 'Falling, negative',
    waveform: 'square',
    frequencies: [150, 80],
    intensity: 0.7,
  },

  // Acknowledgment - neutral tap
  ACKNOWLEDGE: {
    sequence: ['tick'],
    character: 'neutral',
    metaphor: 'Simple confirmation',
    waveform: 'square',
    frequencies: [120],
    intensity: 0.5,
  },

  // Processing/waiting
  PROCESSING: {
    sequence: ['pulse_loop'],
    character: 'rhythmic',
    metaphor: 'Ongoing activity',
    waveform: 'sine',
    continuous: true,
  },
};
```

### 6.2 Selection Feedback

| Action | Pattern | Description |
|--------|---------|-------------|
| `select_item` | Single tap | Item selected |
| `deselect_item` | Reverse tap | Item deselected |
| `select_all` | Rising sweep | All items selected |
| `clear_selection` | Falling sweep | Selection cleared |
| `toggle` | Double tap | State toggled |

## 7. Spatial Patterns

### 7.1 Layout Communication

```typescript
interface SpatialPattern {
  // Room/area shape indication
  shape: 'rectangular' | 'circular' | 'irregular' | 'corridor';

  // Size indication through duration
  sizeEncoding: {
    small: number;    // < 10 sq m
    medium: number;   // 10-30 sq m
    large: number;    // > 30 sq m
  };

  // Exit/opening positions
  openings: BodyLocation[];
}

const SPATIAL_PATTERNS = {
  // Open space
  OPEN_AREA: {
    pattern: 'wide_sweep',
    description: 'Expansive, unobstructed',
    intensity: 0.3,
    duration: 'long',
  },

  // Enclosed space
  ENCLOSED_AREA: {
    pattern: 'contained_pulse',
    description: 'Bounded, defined',
    intensity: 0.5,
    duration: 'short',
  },

  // Doorway/passage
  DOORWAY: {
    pattern: 'focused_beam',
    description: 'Directional opening',
    intensity: 0.6,
    bilateral: false,
  },

  // Wall/boundary
  BOUNDARY: {
    pattern: 'edge_texture',
    description: 'Solid barrier',
    waveform: 'noise',
    intensity: 0.7,
  },
};
```

### 7.2 Position Markers

| Pattern ID | Position | Haptic Representation |
|------------|----------|----------------------|
| `pos_center` | Center of space | Balanced bilateral pulse |
| `pos_edge` | Near boundary | Asymmetric intensity |
| `pos_corner` | Corner position | Dual-direction texture |
| `pos_entrance` | Entry point | Welcoming sweep |
| `pos_exit` | Exit point | Guiding pulse sequence |

## 8. Temporal Patterns

### 8.1 Time Communication

```typescript
const TEMPORAL_PATTERNS = {
  // Countdown
  COUNTDOWN: {
    // Pulse rate increases as time decreases
    calculatePattern(remainingSeconds: number) {
      return {
        pulseInterval: Math.max(100, remainingSeconds * 100),
        intensity: Math.min(1.0, 0.3 + (60 - remainingSeconds) / 60),
      };
    },
  },

  // Progress indication
  PROGRESS: {
    // Intensity correlates with completion
    calculatePattern(percentComplete: number) {
      return {
        intensity: 0.3 + (percentComplete / 100) * 0.5,
        frequency: 80 + (percentComplete / 100) * 70,
      };
    },
  },

  // Rhythm/tempo
  RHYTHM: {
    patterns: {
      slow: { bpm: 60, intensity: 0.4 },
      moderate: { bpm: 100, intensity: 0.5 },
      fast: { bpm: 140, intensity: 0.6 },
      urgent: { bpm: 180, intensity: 0.8 },
    },
  },
};
```

### 8.2 Duration Encoding

| Duration | Representation |
|----------|----------------|
| Brief (< 5s) | Single pulse |
| Short (5-30s) | Double pulse |
| Medium (30s-5m) | Triple pulse |
| Long (5-30m) | Sustained low pulse |
| Extended (> 30m) | Periodic reminder pulses |

## 9. Social Patterns

### 9.1 Presence Indicators

```typescript
const SOCIAL_PATTERNS = {
  // Someone nearby
  PRESENCE_DETECTED: {
    pattern: 'soft_wave',
    description: 'Person nearby',
    intensity: 0.4,
    direction_aware: true,
  },

  // Someone approaching
  PRESENCE_APPROACHING: {
    pattern: 'growing_pulse',
    description: 'Person coming closer',
    intensity_ramp: true,
  },

  // Someone leaving
  PRESENCE_DEPARTING: {
    pattern: 'fading_pulse',
    description: 'Person moving away',
    intensity_ramp: true,
    direction: 'decreasing',
  },

  // Group nearby
  GROUP_DETECTED: {
    pattern: 'multiple_waves',
    description: 'Multiple people nearby',
    complexity: 'high',
  },
};
```

### 9.2 Attention Signals

| Pattern ID | Signal | Use Case |
|------------|--------|----------|
| `attention_request` | Gentle persistent pulse | Someone wants attention |
| `attention_urgent` | Strong rapid pulse | Urgent attention needed |
| `speaking_turn` | Rising sweep | Your turn to speak |
| `applause` | Rolling texture | Approval/applause detected |

### 9.3 Emotional Indicators

```typescript
// For conveying detected emotions (from voice/context analysis)
const EMOTION_PATTERNS = {
  POSITIVE: {
    pattern: 'warm_wave',
    character: 'smooth_ascending',
    intensity: 0.5,
  },
  NEUTRAL: {
    pattern: 'steady_pulse',
    character: 'even',
    intensity: 0.4,
  },
  NEGATIVE: {
    pattern: 'tense_pulse',
    character: 'sharp',
    intensity: 0.6,
  },
  UNCERTAIN: {
    pattern: 'wavering',
    character: 'variable',
    intensity: 0.4,
  },
};
```

## 10. System Patterns

### 10.1 Status Indicators

```typescript
const SYSTEM_PATTERNS = {
  // Battery status
  BATTERY: {
    FULL: { pulses: 4, intensity: 0.5, character: 'strong' },
    MEDIUM: { pulses: 3, intensity: 0.5, character: 'moderate' },
    LOW: { pulses: 2, intensity: 0.6, character: 'warning' },
    CRITICAL: { pulses: 1, intensity: 0.8, character: 'urgent', repeat: true },
  },

  // Connectivity
  CONNECTIVITY: {
    CONNECTED: { pattern: 'affirm_tick', intensity: 0.5 },
    DISCONNECTED: { pattern: 'warning_buzz', intensity: 0.6 },
    RECONNECTING: { pattern: 'searching_pulse', intensity: 0.4 },
  },

  // Mode changes
  MODE_CHANGE: {
    pattern: 'transition_sweep',
    intensity: 0.5,
    duration: 200,
  },
};
```

### 10.2 Error Patterns

| Error Type | Pattern | Description |
|------------|---------|-------------|
| `error_minor` | Single buzz | Recoverable error |
| `error_major` | Triple buzz | Significant error |
| `error_critical` | Continuous alarm | System failure |
| `error_input` | Rejection tap | Invalid input |

## 11. Content Navigation Patterns

### 11.1 Scroll and Browse

```typescript
const CONTENT_PATTERNS = {
  // Scrolling feedback
  SCROLL: {
    // Tick per item
    ITEM_PASS: { intensity: 0.2, duration: 10 },

    // Section boundary
    SECTION_BOUNDARY: { intensity: 0.5, duration: 30 },

    // End of content
    END_REACHED: { pattern: 'bounce', intensity: 0.6 },
  },

  // Selection
  SELECTION: {
    FOCUS: { intensity: 0.3, duration: 20 },
    SELECT: { intensity: 0.5, duration: 30 },
    ACTIVATE: { intensity: 0.6, duration: 50 },
  },

  // Content types
  CONTENT_TYPE: {
    TEXT: { pattern: 'smooth', frequency: 100 },
    IMAGE: { pattern: 'texture', frequency: 80 },
    VIDEO: { pattern: 'pulse', frequency: 120 },
    INTERACTIVE: { pattern: 'dynamic', frequency: 150 },
  },
};
```

## 12. Pattern Library

### 12.1 Core Vocabulary (Must Learn)

| # | Pattern Name | Category | Meaning | Difficulty |
|---|--------------|----------|---------|------------|
| 1 | Single Tap | Confirmation | Acknowledged | Easy |
| 2 | Double Tap | Confirmation | Success | Easy |
| 3 | Triple Tap | Notification | Attention | Easy |
| 4 | Long Press | Confirmation | Processing | Easy |
| 5 | Rising Pulse | Navigation | Forward/Yes | Easy |
| 6 | Falling Pulse | Navigation | Back/No | Easy |
| 7 | Left Pulse | Navigation | Turn left | Easy |
| 8 | Right Pulse | Navigation | Turn right | Easy |
| 9 | Buzz | Error | Problem | Easy |
| 10 | Texture | Spatial | Obstacle | Moderate |

### 12.2 Extended Vocabulary

Additional patterns for advanced users, building on core vocabulary.

## 13. Customization Guidelines

### 13.1 User Preferences

```typescript
interface SemanticPreferences {
  // Intensity scaling by category
  categoryIntensity: Record<HapticCategory, number>;

  // Pattern substitutions for user preference
  patternOverrides: Map<string, string>;

  // Learning mode settings
  learningMode: {
    enabled: boolean;
    verbosity: 'minimal' | 'normal' | 'detailed';
    confirmationRequired: boolean;
  };
}
```

### 13.2 Context Adaptation

- Patterns may be simplified in noisy environments
- Intensity adjusted based on activity level
- Reduced patterns during sleep/quiet hours

---

## Appendix A: Pattern Quick Reference

```
╔═══════════════════════════════════════════════════════════════════╗
║                    WIA SEMANTIC MAPPING QUICK REFERENCE           ║
╠═══════════════════════════════════════════════════════════════════╣
║ NAVIGATION          │ NOTIFICATION       │ CONFIRMATION           ║
║ ───────────────────│───────────────────│───────────────────────  ║
║ ▲ Forward   ⋅⋅⋅    │ ℹ Info     ⋅       │ ✓ Success   ⋅⋅↑        ║
║ ◀ Left      ←⋅⋅    │ ⚠ Warning  ⋅⋅⋅     │ ✗ Failure   ⋅⋅↓        ║
║ ▶ Right     ⋅⋅→    │ ⚡ Urgent   ⋅⋅⋅⋅⋅   │ ⋅ Ack       ⋅          ║
║ ▼ Back      ⋅⋅⋅↓   │ 🔴 Critical ▓▓▓▓▓   │ ◌ Processing ⋅⋅⋅~      ║
║ ◉ Arrived   ♪      │                    │                        ║
╠═══════════════════════════════════════════════════════════════════╣
║ SPATIAL             │ TEMPORAL           │ SOCIAL                 ║
║ ───────────────────│───────────────────│───────────────────────  ║
║ ○ Open      ~~~    │ ⏱ Timer    ⋅_⋅_⋅   │ 👤 Presence  ≈          ║
║ □ Enclosed  ▓▓▓    │ ▰ Progress ▂▄▆█   │ → Approach  ≈≈≈        ║
║ ▢ Boundary  ░░░    │ ♩ Rhythm   ⋅⋅⋅⋅    │ ← Depart    ≈≈⋅        ║
╚═══════════════════════════════════════════════════════════════════╝
```
