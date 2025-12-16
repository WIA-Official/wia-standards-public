# WIA Education Standard

**Educational Technology Accessibility Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20EDU-orange.svg)](https://edu.wia.live)

---

<div align="center">

**Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) | [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## Overview

WIA Education is an open standard for educational technology accessibility, based on:
- **AccessForAll (ISO 24751)** - Learner preferences and content accessibility
- **Universal Design for Learning (UDL)** - Multiple means of engagement, representation, and expression
- **LTI 1.3** - Learning Tools Interoperability
- **xAPI** - Experience API for learning analytics

---

## Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | JSON schemas for learner profiles, courses, content, assessments | ✅ Complete |
| **2** | API Interface | Rust SDK for profile, course, and assessment management | ✅ Complete |
| **3** | Communication Protocol | LTI 1.3, xAPI, Profile Sync, Real-time Events | ✅ Complete |
| **4** | Ecosystem Integration | WIA assistive devices + external LMS platforms | ✅ Complete |

---

## Quick Start

### Rust SDK

```rust
use wia_edu::{EduController, types::*, protocol::*};

fn main() {
    let mut controller = EduController::new();

    // Create profile for blind learner with recommended settings
    let profile = controller.create_profile_for_disability(DisabilityType::Blind);

    // Profile has screen reader enabled, audio description required
    assert!(profile.display_preferences.screen_reader.enabled);
    assert!(profile.content_preferences.audio_description.required);

    // Check content accessibility
    let result = EduController::check_content_accessibility(&profile, &content);

    // Generate xAPI statement for learning activity
    let generator = StatementGenerator::new("https://lms.example.com");
    let statement = generator.course_completed(&profile, &course, true, Some(0.95), None);
}
```

### LTI 1.3 Integration

```rust
use wia_edu::protocol::{LtiAdapter, PlatformConfig, ToolConfig};

// Create LTI adapter for LMS integration
let adapter = LtiAdapter::new(platform_config, tool_config);

// Create launch request with accessibility claims
let request = adapter.create_launch_request(
    "user-123",
    resource_link,
    vec![LtiRole::Learner],
    Some(context),
    Some(&learner_profile),  // Includes accessibility needs
);
```

### Profile Synchronization

```rust
use wia_edu::protocol::{ProfileSyncManager, SyncConfig};

// Sync profiles across platforms
let mut sync_manager = ProfileSyncManager::new(config);
let result = sync_manager.sync(
    profile_id,
    &mut local_profile,
    &remote_profile,
    "remote-lms",
);
```

### Ecosystem Integration

```rust
use wia_edu::ecosystem::{EcosystemManager, AACAdapter, BCIAdapter, CanvasAdapter};

// Create ecosystem manager
let mut ecosystem = EcosystemManager::new();

// Register WIA assistive devices
ecosystem.register_aac(Box::new(AACAdapter::new()));
ecosystem.register_bci(Box::new(BCIAdapter::new()));

// Register external LMS
let canvas = CanvasAdapter::with_api_key("https://canvas.example.com", "api-key");
ecosystem.register_lms(Box::new(canvas));

// Set learner profile and sync to all platforms
ecosystem.set_profile(profile);
ecosystem.sync_profile_to_all_lms().await;
```

---

## Structure

```
edu/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md   # Data format specification
│   └── schemas/
│       ├── learner-profile.schema.json  # Learner accessibility profile
│       ├── course.schema.json           # Course structure
│       ├── content.schema.json          # Content accessibility metadata
│       └── assessment.schema.json       # Assessment accommodations
├── api/
│   └── rust/                    # Rust SDK
│       ├── src/
│       │   ├── lib.rs           # Main controller
│       │   ├── types.rs         # Type definitions
│       │   ├── error.rs         # Error types
│       │   ├── core/            # Profile, Course, Assessment managers
│       │   ├── adapters/        # Storage adapters, simulators
│       │   ├── protocol/        # Phase 3 protocols
│       │   │   ├── lti/         # LTI 1.3 adapter
│       │   │   ├── xapi/        # xAPI statement generator
│       │   │   ├── sync/        # Profile synchronization
│       │   │   └── events/      # Real-time accessibility events
│       │   └── ecosystem/       # Phase 4 integrations
│       │       ├── aac.rs       # AAC device integration
│       │       ├── bci.rs       # BCI device integration
│       │       ├── eye_gaze.rs  # Eye gaze integration
│       │       ├── wheelchair.rs # Smart wheelchair integration
│       │       ├── haptic.rs    # Haptic/braille integration
│       │       └── external/    # External LMS platforms
│       │           ├── canvas.rs    # Canvas LMS
│       │           ├── moodle.rs    # Moodle
│       │           ├── blackboard.rs # Blackboard
│       │           └── google.rs    # Google Classroom
│       ├── tests/               # Integration tests
│       └── examples/            # Usage examples
└── prompts/                     # Claude Code prompts
```

---

## Phase 3 Protocol Features

### LTI 1.3 Integration
- LTI message types (ResourceLink, DeepLinking)
- Accessibility claims transmission via custom JWT claims
- OAuth 2.0 authentication flow
- Platform/Tool communication

### xAPI Statement Generation
- Standard ADL verbs + WIA accessibility verbs
- Accessibility activity types and extensions
- Statement generation for learning activities
- Barrier reporting and accommodation tracking

### Profile Synchronization
- Version management and conflict resolution
- Incremental sync support
- Field-level conflict detection
- Accessibility-aware resolution (prefer more accommodating)

### Real-time Events
- Adaptation request/response protocol
- Setting change events
- Content alternative requests
- Event streaming with subscriptions

---

## Links

| Resource | URL |
|----------|-----|
| **Website** | https://edu.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/edu |

---

## License

MIT License - This standard belongs to humanity.

---

<div align="center">

**弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
