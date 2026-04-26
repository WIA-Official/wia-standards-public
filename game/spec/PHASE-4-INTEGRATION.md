# WIA Game Phase 4: Ecosystem Integration

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01-XX

---

## 1. Overview

Phase 4 integrates WIA Game with the broader WIA assistive technology ecosystem and external gaming platforms, enabling comprehensive accessibility for gamers with diverse disabilities.

### 1.1 Design Principles

- **Multi-Modal Gaming**: Support any combination of input devices
- **Platform Agnostic**: Work across Xbox, PlayStation, PC, Cloud
- **Profile Portability**: Take your accessibility settings anywhere
- **Immersive Feedback**: Environmental integration for deaf/blind gamers

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                      WIA Game Ecosystem Hub                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │
│  │  Eye Gaze   │  │     BCI     │  │     AAC     │  │   Haptic   │  │
│  │   Aiming    │  │  Commands   │  │   Voice     │  │  Feedback  │  │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬─────┘  │
│         │                │                │                │        │
│  ┌──────┴────────────────┴────────────────┴────────────────┴─────┐  │
│  │                  Unified Input Processor                       │  │
│  └────────────────────────────┬──────────────────────────────────┘  │
│                               │                                      │
│  ┌────────────────────────────┴──────────────────────────────────┐  │
│  │                 Game Controller (Phase 1-3)                    │  │
│  │        Profiles │ Protocol │ Devices │ Macros │ Switch         │  │
│  └────────────────────────────┬──────────────────────────────────┘  │
│                               │                                      │
├───────────────────────────────┼──────────────────────────────────────┤
│                               ▼                                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │
│  │  Wheelchair │  │ Smart Home  │  │ Exoskeleton │  │   Cloud    │  │
│  │   Bridge    │  │   Effects   │  │   Input     │  │   Sync     │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  └────────────┘  │
│                                                                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │
│  │    Xbox     │  │ PlayStation │  │    Steam    │  │   Cloud    │  │
│  │  Platform   │  │   Access    │  │   Input     │  │  Gaming    │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  └────────────┘  │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. WIA Eye Gaze Gaming Integration

### 3.1 Gaze-Based Game Controls

| Control Type | Description | Use Cases |
|--------------|-------------|-----------|
| **Aim Assist** | Gaze point influences aim magnetism | FPS, TPS games |
| **Camera Control** | Look to rotate camera | All 3D games |
| **UI Navigation** | Dwell to select menu items | All games |
| **Quick Look** | Glance at minimap, HUD elements | Strategy, RPG |
| **Target Lock** | Dwell on enemy to lock target | Action games |

### 3.2 Gaze Aiming Configuration

```json
{
  "gaze_aiming": {
    "enabled": true,
    "mode": "aim_assist",
    "sensitivity": 0.7,
    "smoothing": 0.3,
    "dead_zone_radius": 0.05,
    "aim_magnetism": {
      "enabled": true,
      "strength": 0.8,
      "target_priority": ["enemy", "interactable", "ally"]
    },
    "dwell_select": {
      "enabled": true,
      "duration_ms": 800,
      "visual_feedback": "shrinking_circle"
    }
  }
}
```

### 3.3 Eye Tracker Event Mapping

```rust
pub struct GazeGameInput {
    pub gaze_point: (f32, f32),     // Normalized 0.0-1.0
    pub aim_vector: (f32, f32),     // Calculated aim direction
    pub dwell_target: Option<GameEntity>,
    pub blink_detected: bool,
    pub calibration_quality: f32,
}

// Gaze to game action mapping
pub enum GazeAction {
    AimAt { target: AimTarget, magnetism: f32 },
    LookAt { camera_delta: (f32, f32) },
    DwellSelect { entity_id: EntityId },
    QuickGlance { ui_element: UIElement },
    Blink { action: BlinkAction },
}
```

---

## 4. WIA BCI Gaming Integration

### 4.1 Supported BCI Paradigms for Gaming

| Paradigm | Signal | Gaming Application |
|----------|--------|-------------------|
| **Motor Imagery** | μ/β rhythm | Movement, abilities |
| **SSVEP** | Steady-state VEP | Quick menu selection |
| **P300** | Event-related potential | Grid-based selection |
| **Error Potential** | Automatic correction | Undo/cancel actions |
| **Mental State** | Alpha/theta waves | Pause on fatigue |

### 4.2 Mental Command Mapping

```
┌─────────────────────────────────────────┐
│        BCI Mental Commands              │
├─────────────────────────────────────────┤
│  Left Hand Imagery  → Move Left/Strafe  │
│  Right Hand Imagery → Move Right/Strafe │
│  Feet Imagery       → Jump/Move Forward │
│  Relaxation         → Crouch/Sneak      │
│  Push Imagery       → Attack/Fire       │
│  Pull Imagery       → Use/Interact      │
│  Error Detection    → Cancel Action     │
├─────────────────────────────────────────┤
│  Confidence Thresholds:                 │
│    Movement: 0.6  │  Combat: 0.8        │
│    Critical: 0.9  │  Menu: 0.5          │
└─────────────────────────────────────────┘
```

### 4.3 BCI Gaming Protocol

```rust
pub struct BCIGameState {
    pub mental_command: Option<MentalCommand>,
    pub confidence: f32,
    pub fatigue_level: f32,
    pub session_duration: Duration,
    pub last_error_potential: Option<Instant>,
}

pub enum MentalCommand {
    MotorImagery { limb: ImaginaryLimb, intensity: f32 },
    SSVEP { frequency_hz: f32, target_id: u8 },
    P300 { selected_row: u8, selected_col: u8 },
    Relax,
    Push,
    Pull,
}

// Fatigue detection and auto-pause
pub fn check_fatigue(state: &BCIGameState) -> Option<GamePauseReason> {
    if state.fatigue_level > 0.8 {
        Some(GamePauseReason::BCIFatigue {
            recommended_break_mins: 10
        })
    } else {
        None
    }
}
```

---

## 5. WIA AAC Gaming Integration

### 5.1 Voice Command Grammar for Gaming

```
English (en-US):
  "{action}" - Direct commands
  "{action} {target}" - Targeted commands
  "{ability} on {target}" - Ability usage

  Examples:
    - "Fire" / "Shoot" / "Attack"
    - "Reload" / "Switch weapon"
    - "Jump" / "Crouch" / "Sprint"
    - "Heal" / "Use potion"
    - "Target enemy" / "Lock on"
    - "Open map" / "Pause game"
    - "Quick save" / "Quick load"

Korean (ko-KR):
  "{동작}" / "{동작} {대상}"

  Examples:
    - "공격" / "발사" / "사격"
    - "재장전" / "무기 교체"
    - "점프" / "앉기" / "달리기"
    - "회복" / "물약 사용"
    - "적 타겟" / "락온"
    - "지도 열기" / "일시정지"
```

### 5.2 Symbol-Based Quick Actions

| Symbol | Action | Cooldown |
|--------|--------|----------|
| ⚔️ | Primary Attack | None |
| 🛡️ | Block/Defend | None |
| 💊 | Use Health Item | 3s |
| 🗺️ | Toggle Map | 1s |
| ⏸️ | Pause Game | 1s |
| 💬 | Quick Chat | None |
| 🎯 | Lock Target | 0.5s |
| 🏃 | Toggle Sprint | None |

### 5.3 Voice Macro System

```rust
pub struct VoiceMacro {
    pub trigger_phrase: String,
    pub alternate_phrases: Vec<String>,
    pub language: Language,
    pub actions: Vec<GameAction>,
    pub cooldown_ms: u32,
    pub confirmation_required: bool,
}

// Example: "Super combo" triggers a complex sequence
let combo_macro = VoiceMacro {
    trigger_phrase: "super combo".to_string(),
    alternate_phrases: vec!["ultimate attack".to_string()],
    actions: vec![
        GameAction::Press(Button::LB),
        GameAction::Wait(100),
        GameAction::Press(Button::RB),
        GameAction::Wait(50),
        GameAction::Press(Button::A),
    ],
    cooldown_ms: 5000,
    confirmation_required: false,
};
```

---

## 6. WIA Smart Wheelchair Gaming Integration

### 6.1 Wheelchair Motion as Game Input

| Motion | Game Action |
|--------|-------------|
| **Tilt Forward** | Move forward, accelerate |
| **Tilt Back** | Move backward, brake |
| **Tilt Left** | Turn/strafe left |
| **Tilt Right** | Turn/strafe right |
| **Quick Spin** | 180° turn, dodge |
| **Recline** | Enter sniper mode, steady aim |

### 6.2 Comfort Sync During Gaming

```json
{
  "wheelchair_gaming_sync": {
    "posture_check_interval_mins": 30,
    "auto_recline_on_cutscene": true,
    "vibration_feedback": {
      "enabled": true,
      "intensity_multiplier": 0.8
    },
    "pressure_relief_reminder": {
      "enabled": true,
      "interval_mins": 45,
      "pause_game": false,
      "notification": "overlay"
    }
  }
}
```

### 6.3 Wheelchair Input Protocol

```rust
pub struct WheelchairGameInput {
    pub tilt_x: f32,        // -1.0 (left) to 1.0 (right)
    pub tilt_y: f32,        // -1.0 (back) to 1.0 (forward)
    pub recline_angle: f32, // 0-45 degrees
    pub speed: f32,         // Current movement speed
    pub is_stationary: bool,
    pub pressure_map: Option<PressureMatrix>,
}

pub enum WheelchairGameAction {
    Movement { direction: Vec2, speed: f32 },
    Camera { rotation: Vec2 },
    SteadyAim { active: bool },
    QuickTurn { degrees: f32 },
}
```

---

## 7. WIA Haptic Gaming Integration

### 7.1 Game Event Haptic Patterns

| Game Event | Haptic Pattern | Duration | Intensity |
|------------|---------------|----------|-----------|
| **Damage Taken** | Directional pulse | 200ms | Strong |
| **Health Low** | Heartbeat rhythm | Continuous | Medium |
| **Enemy Nearby** | Subtle vibration | 500ms | Light |
| **Weapon Fire** | Recoil pattern | 100ms | Varies |
| **Explosion** | Rumble + decay | 800ms | Maximum |
| **Footsteps** | Rhythmic pulse | Per step | Light |
| **UI Selection** | Click feedback | 50ms | Light |
| **Achievement** | Celebration | 1000ms | Medium |

### 7.2 Spatial Audio to Haptic Conversion

```
┌──────────────────────────────────────────┐
│     Spatial Haptic for Deaf Gamers       │
├──────────────────────────────────────────┤
│                                          │
│    Audio Direction → Haptic Location     │
│                                          │
│         [Front Left]   [Front Right]     │
│              ↖             ↗             │
│                   ⬆️                      │
│         ⬅️    [Player]    ➡️             │
│                   ⬇️                      │
│              ↙             ↘             │
│         [Rear Left]    [Rear Right]      │
│                                          │
│    Sound Intensity → Haptic Strength     │
│    Sound Type → Haptic Pattern           │
│      Gunshot: Sharp pulse                │
│      Footstep: Soft tap                  │
│      Explosion: Rolling wave             │
│      Voice: Rhythmic pattern             │
│                                          │
└──────────────────────────────────────────┘
```

### 7.3 Haptic Feedback Protocol

```rust
pub struct GameHapticFeedback {
    pub event: GameEvent,
    pub pattern: HapticPattern,
    pub direction: Option<Direction8>,  // 8-directional
    pub intensity: f32,
    pub duration_ms: u32,
    pub priority: HapticPriority,
}

pub enum HapticPattern {
    Pulse { count: u8, interval_ms: u32 },
    Rumble { attack_ms: u32, decay_ms: u32 },
    Heartbeat { bpm: u32 },
    Directional { angle: f32, spread: f32 },
    Wave { frequency_hz: f32 },
    Custom { waveform: Vec<f32> },
}
```

---

## 8. WIA Smart Home Gaming Integration

### 8.1 Immersive Environment Effects

| Game State | Smart Home Response |
|------------|---------------------|
| **Day/Night Cycle** | Adjust room lighting color/brightness |
| **Weather: Rain** | Blue tint lighting, optional fan |
| **Weather: Fire** | Orange/red lighting, warm tint |
| **Combat Mode** | Red alert lighting, increase brightness |
| **Stealth Mode** | Dim lights, reduce distractions |
| **Victory** | Celebration lighting sequence |
| **Damage Taken** | Flash red briefly |
| **Game Paused** | Restore normal lighting |

### 8.2 Environment Sync Protocol

```json
{
  "smart_home_gaming_sync": {
    "enabled": true,
    "lighting_sync": {
      "enabled": true,
      "zones": ["gaming_room", "ambient"],
      "intensity": 0.7,
      "transition_speed": "smooth"
    },
    "climate_sync": {
      "enabled": false,
      "fan_on_hot_environments": false
    },
    "do_not_disturb": {
      "enabled": true,
      "mute_doorbell": true,
      "dim_notifications": true
    }
  }
}
```

---

## 9. External Platform Integration

### 9.1 Xbox Platform Integration

```rust
pub struct XboxIntegration {
    pub copilot_enabled: bool,
    pub accessibility_features: XboxAccessibilityFeatures,
    pub cloud_save_sync: bool,
    pub adaptive_controller_config: Option<AdaptiveControllerConfig>,
}

pub struct XboxAccessibilityFeatures {
    pub narrator: bool,
    pub magnifier: bool,
    pub high_contrast: bool,
    pub button_remapping: HashMap<XboxButton, XboxButton>,
    pub copilot_partner_profile: Option<ProfileId>,
}

// Sync WIA profile to Xbox settings
pub async fn sync_to_xbox(profile: &PlayerProfile) -> Result<XboxProfile, Error> {
    let xbox_profile = XboxProfile {
        narrator: profile.visual_settings.screen_reader.enabled,
        magnifier: profile.visual_settings.magnification.enabled,
        high_contrast: profile.visual_settings.high_contrast,
        button_remapping: convert_remapping(&profile.motor_settings.remapping),
        copilot: profile.motor_settings.copilot_mode.enabled,
        // ... more conversions
    };

    xbox_api::apply_accessibility_settings(&xbox_profile).await
}
```

### 9.2 PlayStation Access Integration

```rust
pub struct PlayStationIntegration {
    pub access_controller: Option<AccessControllerConfig>,
    pub custom_button_assignments: HashMap<PSButton, PSButton>,
    pub touch_pad_settings: TouchPadSettings,
    pub motion_control: MotionControlSettings,
}

pub struct AccessControllerConfig {
    pub profile_slots: [Option<AccessProfile>; 4],
    pub button_mapping: HashMap<AccessButton, PSButton>,
    pub stick_sensitivity: (f32, f32),
    pub orientation: ControllerOrientation,
}
```

### 9.3 Steam Integration

```rust
pub struct SteamIntegration {
    pub steam_input: SteamInputConfig,
    pub cloud_save: SteamCloudConfig,
    pub remote_play: RemotePlaySettings,
    pub big_picture_mode: bool,
}

pub struct SteamInputConfig {
    pub controller_type: SteamControllerType,
    pub action_sets: HashMap<String, ActionSet>,
    pub gyro_settings: Option<GyroSettings>,
    pub touch_settings: Option<TouchSettings>,
}

// Export WIA profile to Steam Input configuration
pub fn export_to_steam_input(profile: &PlayerProfile) -> SteamInputConfig {
    // Convert WIA remapping to Steam action sets
}
```

### 9.4 Cloud Gaming Optimization

```rust
pub struct CloudGamingSettings {
    pub latency_compensation: LatencyCompensation,
    pub input_buffering: InputBufferSettings,
    pub predictive_input: bool,
    pub quality_vs_latency: QualityPreference,
}

pub struct LatencyCompensation {
    pub enabled: bool,
    pub aim_assist_boost: f32,      // Increase aim assist for latency
    pub input_prediction_ms: u32,   // Predict input ahead
    pub dead_zone_reduction: f32,   // Reduce dead zones for responsiveness
}
```

---

## 10. Cloud Profile Sync

### 10.1 Profile Storage Structure

```json
{
  "cloud_profile": {
    "id": "uuid:profile-123",
    "version": "1.0.0",
    "user_id": "uuid:user-456",
    "created_at": "2025-01-15T10:00:00Z",
    "updated_at": "2025-01-15T12:00:00Z",
    "sync_status": "synced",

    "profile_data": {
      "visual_settings": { /* ... */ },
      "audio_settings": { /* ... */ },
      "motor_settings": { /* ... */ },
      "cognitive_settings": { /* ... */ }
    },

    "platform_exports": {
      "xbox": { "last_sync": "2025-01-15T11:00:00Z" },
      "playstation": { "last_sync": null },
      "steam": { "last_sync": "2025-01-15T10:30:00Z" }
    },

    "game_overrides": {
      "game-id-1": { /* game-specific tweaks */ },
      "game-id-2": { /* game-specific tweaks */ }
    }
  }
}
```

### 10.2 Sync Protocol

```rust
pub struct CloudSyncManager {
    api_endpoint: String,
    auth_token: String,
    local_cache: ProfileCache,
    sync_queue: VecDeque<SyncOperation>,
}

impl CloudSyncManager {
    pub async fn sync(&mut self) -> Result<SyncResult, SyncError> {
        // 1. Pull remote changes
        let remote = self.fetch_remote_profile().await?;

        // 2. Merge with local
        let merged = self.merge_profiles(remote, self.local_cache.clone())?;

        // 3. Push if local had newer changes
        if merged.has_local_changes {
            self.push_profile(&merged.profile).await?;
        }

        // 4. Update local cache
        self.local_cache = merged.profile.clone();

        Ok(SyncResult { profile: merged.profile, conflicts: merged.conflicts })
    }

    pub async fn sync_to_platform(&self, platform: Platform) -> Result<(), Error> {
        match platform {
            Platform::Xbox => self.sync_to_xbox().await,
            Platform::PlayStation => self.sync_to_playstation().await,
            Platform::Steam => self.sync_to_steam().await,
        }
    }
}
```

---

## 11. Analytics & Telemetry

### 11.1 Accessibility Usage Metrics

```rust
pub struct AccessibilityAnalytics {
    pub session_id: Uuid,
    pub profile_id: Uuid,
    pub game_id: String,
    pub metrics: SessionMetrics,
}

pub struct SessionMetrics {
    pub duration: Duration,
    pub input_sources_used: HashSet<InputSource>,
    pub features_used: HashSet<AccessibilityFeature>,
    pub average_input_latency_ms: f32,
    pub successful_actions: u32,
    pub failed_actions: u32,
    pub fatigue_events: u32,
    pub pause_reasons: Vec<PauseReason>,
}

pub struct FeatureEffectiveness {
    pub feature: AccessibilityFeature,
    pub usage_time_percent: f32,
    pub success_rate: f32,
    pub user_adjustments: u32,  // How often user tweaked settings
}
```

### 11.2 Privacy-Preserving Analytics

```json
{
  "analytics_consent": {
    "anonymous_usage": true,
    "feature_effectiveness": true,
    "crash_reports": true,
    "detailed_input_logs": false,
    "share_with_developers": false,

    "data_retention_days": 30,
    "deletion_on_request": true
  }
}
```

---

## 12. Unified Input Processor

### 12.1 Multi-Modal Input Fusion

```rust
pub struct UnifiedInputProcessor {
    input_sources: HashMap<InputSourceId, Box<dyn InputSource>>,
    fusion_rules: Vec<FusionRule>,
    output_queue: VecDeque<GameAction>,
}

pub struct FusionRule {
    pub name: String,
    pub inputs: Vec<InputCondition>,
    pub output: GameAction,
    pub priority: u8,
    pub cooldown_ms: u32,
}

// Example: Gaze + Blink = Confirm
let gaze_blink_confirm = FusionRule {
    name: "Gaze + Blink Confirm".to_string(),
    inputs: vec![
        InputCondition::GazeDwell { duration_ms: 500 },
        InputCondition::BlinkDetected,
    ],
    output: GameAction::Confirm,
    priority: 10,
    cooldown_ms: 500,
};

// Example: BCI Push + Eye Gaze Target = Attack Target
let bci_gaze_attack = FusionRule {
    name: "BCI Gaze Attack".to_string(),
    inputs: vec![
        InputCondition::BCICommand { command: MentalCommand::Push, min_confidence: 0.7 },
        InputCondition::GazeTarget { entity_type: EntityType::Enemy },
    ],
    output: GameAction::AttackTarget { use_gaze_target: true },
    priority: 8,
    cooldown_ms: 200,
};
```

### 12.2 Input Priority Resolution

| Priority | Input Source | Override Behavior |
|----------|--------------|-------------------|
| **10** | Emergency (voice "help") | Override all |
| **9** | Direct physical input | Override AT |
| **8** | BCI high-confidence (>0.9) | Override automation |
| **7** | Eye Gaze + confirmation | Standard |
| **6** | Voice command | Standard |
| **5** | AAC symbol | Standard |
| **4** | Wheelchair motion | Standard |
| **3** | Automation/macro | Lowest |

---

## 13. Security Considerations

### 13.1 Authentication per Source

| Input Source | Authentication Method |
|--------------|----------------------|
| Eye Tracker | Device pairing + calibration signature |
| BCI | Neural pattern verification |
| Voice/AAC | Voiceprint + device binding |
| Wheelchair | Proximity + certificate |
| Platforms | OAuth 2.0 |

### 13.2 Cloud Security

```rust
pub struct CloudSecurityConfig {
    pub encryption: EncryptionMethod::AES256GCM,
    pub transit_encryption: true,
    pub at_rest_encryption: true,
    pub token_refresh_interval_hours: 24,
    pub mfa_required: false,  // Accessibility consideration
    pub biometric_fallback: true,
}
```

---

## 14. Implementation Checklist

- [ ] Eye Gaze gaming adapter with aim assist
- [ ] BCI adapter with motor imagery mapping
- [ ] AAC voice command processor
- [ ] Wheelchair tilt input processor
- [ ] Haptic spatial audio converter
- [ ] Smart Home immersive effects
- [ ] Xbox platform sync
- [ ] PlayStation Access sync
- [ ] Steam Input export
- [ ] Cloud gaming latency compensation
- [ ] Cloud profile sync manager
- [ ] Analytics collection (privacy-preserving)
- [ ] Unified input processor with fusion rules
- [ ] Integration test suite

---

## 15. References

- WIA Eye Gaze Standard v1.0
- WIA BCI Standard v1.0
- WIA AAC Standard v1.0
- WIA Smart Wheelchair Standard v1.0
- WIA Haptic Standard v1.0
- WIA Smart Home Standard v1.0
- Xbox Accessibility Features Documentation
- PlayStation Access Controller Specification
- Steam Input API Documentation
- Cloud Gaming Accessibility Guidelines

---

**弘益人間** - Gaming for Everyone


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
