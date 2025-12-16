# WIA Education Standard - Phase 1: Data Format Specification

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01

---

## 1. Overview

This specification defines the data format standards for the WIA Education accessibility ecosystem. The formats are designed to be interoperable with existing standards (AccessForAll, UDL, WCAG) while providing a unified approach for the WIA ecosystem.

### 1.1 Design Principles

1. **Accessibility First**: All formats prioritize accessibility metadata
2. **Interoperability**: Compatible with 1EdTech AccessForAll, Dublin Core, and LTI
3. **UDL Alignment**: Support Universal Design for Learning principles
4. **WIA Integration**: Seamless connection with WIA AT devices (AAC, BCI, Eye Gaze)
5. **Privacy**: Learner data protection with granular disclosure controls

### 1.2 Schema Overview

| Schema | Purpose |
|--------|---------|
| `learner-profile` | Personal accessibility needs and preferences |
| `course` | Course structure with accessibility features |
| `content` | Educational content accessibility metadata |
| `assessment` | Assessment accommodations and accessible design |

---

## 2. Learner Profile Schema

### 2.1 Purpose

The Learner Profile stores accessibility preferences following the AccessForAll Personal Needs & Preferences (PNP) model, enhanced with UDL learning style preferences and WIA device integration.

### 2.2 Core Structure

```json
{
  "profile_id": "uuid",
  "schema_version": "1.0.0",
  "created_at": "ISO-8601",
  "learner_info": { },
  "disability_profile": { },
  "display_preferences": { },
  "control_preferences": { },
  "content_preferences": { },
  "learning_style": { },
  "assessment_accommodations": { },
  "assistive_technology": { },
  "wia_integrations": { }
}
```

### 2.3 Key Components

#### 2.3.1 Display Preferences (AccessForAll Display)

Controls how content is presented visually:

| Field | Description |
|-------|-------------|
| `screen_reader` | Screen reader settings (enabled, reader, rate, verbosity) |
| `magnification` | Zoom level and focus tracking |
| `text_settings` | Font size, family, spacing (including dyslexia fonts) |
| `color_settings` | High contrast, inversion, color blind filters |
| `reading_guide` | Line highlight, ruler, mask tools |

#### 2.3.2 Control Preferences (AccessForAll Control)

Defines input and interaction methods:

| Field | Description |
|-------|-------------|
| `input_method` | Primary input (keyboard, voice, eye gaze, switch, BCI) |
| `timing` | Extended time, auto-advance, pause settings |
| `click_settings` | Target size, sticky keys, double-click delay |

#### 2.3.3 Content Preferences (AccessForAll Content)

Specifies alternative content needs:

| Field | Description |
|-------|-------------|
| `captions` | Caption requirements and style |
| `audio_description` | Audio description preferences |
| `transcripts` | Transcript requirements |
| `sign_language` | Sign language preference (ASL, KSL, etc.) |
| `text_to_speech` | TTS settings |
| `simplification` | Reading level, chunked content |
| `alternative_formats` | Braille, large print, tactile |

#### 2.3.4 Learning Style (UDL Alignment)

Preferences aligned with UDL principles:

```json
{
  "engagement": {
    "interest_triggers": ["choice", "gamification", "social"],
    "collaboration_preference": "small_groups",
    "feedback_style": "immediate"
  },
  "representation": {
    "preferred_modalities": ["visual", "auditory"],
    "background_knowledge_support": true
  },
  "action_expression": {
    "expression_preferences": ["multimedia", "verbal"],
    "planning_support": true
  }
}
```

#### 2.3.5 WIA Integrations

Links to WIA ecosystem devices:

```json
{
  "aac_profile_id": "uuid",
  "bci_profile_id": "uuid",
  "eye_gaze_profile_id": "uuid",
  "wheelchair_profile_id": "uuid",
  "smart_home_profile_id": "uuid",
  "sync_enabled": true
}
```

---

## 3. Course Schema

### 3.1 Purpose

Defines course structure with comprehensive accessibility metadata, accommodation options, and UDL implementation support.

### 3.2 Core Structure

```json
{
  "course_id": "uuid",
  "schema_version": "1.0.0",
  "title": "string",
  "course_info": { },
  "accessibility_statement": { },
  "modules": [ ],
  "learning_outcomes": [ ],
  "accessibility_features": { },
  "accommodations_available": { }
}
```

### 3.3 Key Components

#### 3.3.1 Accessibility Statement

Required accessibility commitment and conformance information:

```json
{
  "commitment": "Accessibility commitment text",
  "wcag_conformance": "AA",
  "conformance_date": "2025-01-01",
  "known_issues": [
    {
      "description": "Issue description",
      "impact": "medium",
      "workaround": "Alternative approach"
    }
  ],
  "contact": {
    "email": "accessibility@example.edu"
  },
  "accommodation_request_process": "How to request accommodations"
}
```

#### 3.3.2 Module Structure

Each module supports UDL options:

```json
{
  "module_id": "uuid",
  "title": "Module Title",
  "content_items": [ ],
  "udl_options": {
    "engagement_options": {
      "choice_available": true,
      "collaboration_options": ["individual", "pairs", "small_group"]
    },
    "representation_options": {
      "modalities_available": ["text", "audio", "video"]
    },
    "action_expression_options": {
      "response_formats": ["written", "verbal", "multimedia"]
    }
  }
}
```

#### 3.3.3 Available Accommodations

Declares what accommodations the course supports:

- **Timing**: Extended time, flexible deadlines, breaks
- **Format**: Alternative formats (large print, braille, audio)
- **Support**: Note-taking, live captioning, interpreters

---

## 4. Content Schema

### 4.1 Purpose

Describes educational content accessibility following AccessForAll Digital Resource Description (DRD).

### 4.2 Core Structure

```json
{
  "content_id": "uuid",
  "schema_version": "1.0.0",
  "title": "string",
  "type": "video|text|interactive|...",
  "dublin_core": { },
  "educational_metadata": { },
  "accessibility": { },
  "technical": { },
  "alternatives": [ ]
}
```

### 4.3 Accessibility Metadata

Following Schema.org and AccessForAll patterns:

#### 4.3.1 Accessibility Features

```json
{
  "accessibility_features": [
    "captions",
    "transcript",
    "audio_description",
    "alternative_text",
    "structural_navigation"
  ]
}
```

#### 4.3.2 Accessibility Hazards

```json
{
  "accessibility_hazards": [
    "no_flashing_hazard",
    "no_motion_simulation_hazard",
    "no_sound_hazard"
  ]
}
```

#### 4.3.3 Access Modes

Defines sensory requirements:

```json
{
  "access_mode": ["auditory", "visual"],
  "access_mode_sufficient": [
    ["auditory"],
    ["visual", "textual"]
  ]
}
```

The `access_mode_sufficient` indicates that the content can be fully understood through either audio alone OR visual combined with text.

#### 4.3.4 WCAG Conformance

```json
{
  "wcag_conformance": {
    "level": "AA",
    "version": "2.1",
    "evaluation_date": "2025-01-15",
    "issues": [
      {
        "criterion": "1.4.3",
        "description": "Some text contrast is 4.3:1",
        "severity": "minor"
      }
    ]
  }
}
```

### 4.4 Alternative Resources

```json
{
  "alternatives": [
    {
      "type": "transcript",
      "language": "ko",
      "location": "https://...",
      "quality_verified": true
    },
    {
      "type": "captions",
      "language": "ko",
      "is_machine_generated": false
    },
    {
      "type": "sign_language",
      "language": "ksl"
    }
  ]
}
```

---

## 5. Assessment Schema

### 5.1 Purpose

Defines assessments with comprehensive accessibility accommodations following QTI accessibility patterns.

### 5.2 Core Structure

```json
{
  "assessment_id": "uuid",
  "schema_version": "1.0.0",
  "title": "string",
  "type": "quiz|exam|assignment|...",
  "timing": { },
  "questions": [ ],
  "accessibility_settings": { },
  "accommodations_available": { },
  "grading": { }
}
```

### 5.3 Question Accessibility

Each question includes accessibility metadata:

```json
{
  "question_id": "uuid",
  "type": "multiple_choice",
  "content": {
    "text": "Question with <strong>formatting</strong>",
    "plain_text": "Question in plain text for screen readers",
    "audio_url": "https://..."
  },
  "accessibility": {
    "screen_reader_hint": "Additional context",
    "cognitive_load": "low",
    "requires_vision": false,
    "requires_hearing": false,
    "alternative_available": true
  }
}
```

### 5.4 Accommodations Categories

#### 5.4.1 Timing Accommodations

| Accommodation | Description |
|--------------|-------------|
| `extended_time` | Time multipliers (1.5x, 2x, etc.) |
| `unlimited_time` | No time limit |
| `stop_clock_for_breaks` | Pause timer during breaks |
| `flexible_scheduling` | Take at different time |

#### 5.4.2 Presentation Accommodations

| Accommodation | Description |
|--------------|-------------|
| `large_print` | Enlarged text |
| `high_contrast` | High contrast mode |
| `screen_reader` | Screen reader compatible |
| `text_to_speech` | Questions read aloud |
| `braille` | Braille output support |
| `sign_language_video` | Questions in sign language |
| `line_reader` | Reading guide overlay |

#### 5.4.3 Response Accommodations

| Accommodation | Description |
|--------------|-------------|
| `keyboard_only` | No mouse required |
| `speech_to_text` | Voice input |
| `scribe` | Human records answers |
| `spell_check` | Spelling assistance |
| `audio_response` | Verbal answers |

#### 5.4.4 Setting Accommodations

| Accommodation | Description |
|--------------|-------------|
| `separate_location` | Distraction-free room |
| `small_group` | Small group setting |
| `breaks` | Scheduled breaks allowed |

---

## 6. Interoperability

### 6.1 Standards Alignment

| WIA EDU | Maps To |
|---------|---------|
| Learner Profile | AccessForAll PNP (ISO 24751-3) |
| Content Metadata | AccessForAll DRD (ISO 24751-2) |
| Learning Style | UDL Guidelines 3.0 |
| Accessibility Features | Schema.org accessibilityFeature |
| Course Structure | IMS Common Cartridge |

### 6.2 LTI Integration

Learner preferences can be passed via LTI 1.3 custom claims:

```json
{
  "https://wia.live/edu/profile_id": "uuid",
  "https://wia.live/edu/extended_time": 1.5,
  "https://wia.live/edu/screen_reader": true
}
```

### 6.3 WIA Ecosystem Integration

Profile links to other WIA standards:

```
┌─────────────────┐
│  WIA EDU        │
│  Learner Profile│
└────────┬────────┘
         │ Links to
    ┌────┴────┬─────────┬─────────┐
    ▼         ▼         ▼         ▼
┌───────┐ ┌───────┐ ┌───────┐ ┌───────┐
│WIA AAC│ │WIA BCI│ │WIA Eye│ │WIA    │
│Profile│ │Profile│ │ Gaze  │ │Wheel- │
└───────┘ └───────┘ └───────┘ │chair  │
                              └───────┘
```

---

## 7. Privacy Considerations

### 7.1 Disclosure Control

- `disclosed` flag controls whether disability info is shared
- Learners control what information is visible to instructors
- Profile sync requires explicit consent

### 7.2 Data Minimization

- Only collect necessary accessibility preferences
- Support anonymous/pseudonymous profiles
- Allow selective sharing per course/institution

### 7.3 GDPR/FERPA Compliance

- Right to access, correct, delete profile data
- Educational records protection
- Consent required for data sharing

---

## 8. Implementation Notes

### 8.1 Profile Matching

When a learner accesses content:

1. Load learner profile preferences
2. Query content accessibility metadata
3. Match preferences with available features
4. Apply adaptations or suggest alternatives

### 8.2 Cumulative Accessibility

Following AccessForAll philosophy:
- Not every resource needs 100% accessibility
- System provides alternatives when needed
- Third parties can contribute accessible versions

### 8.3 Progressive Enhancement

1. Start with baseline accessibility (WCAG AA)
2. Add alternatives based on learner needs
3. Integrate AT devices for enhanced access

---

## 9. Schema Files

All schemas are available in JSON Schema format:

| File | Description |
|------|-------------|
| `schemas/learner-profile.schema.json` | Learner accessibility profile |
| `schemas/course.schema.json` | Course structure and accessibility |
| `schemas/content.schema.json` | Content accessibility metadata |
| `schemas/assessment.schema.json` | Assessment accommodations |

---

## 10. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

**弘益人間** - Education for Everyone

© 2025 WIA / SmileStory Inc.
