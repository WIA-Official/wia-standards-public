# WIA XR Accessibility Data Format Specification

Version: 1.0.0
Date: 2025-01-15
Status: Draft

## 1. Overview

This document defines the data formats for XR (Extended Reality) accessibility in the WIA ecosystem. It covers user accessibility profiles, device capabilities, environment configurations, and real-time adaptation data.

## 2. Core Data Structures

### 2.1 XR Accessibility Profile

The accessibility profile captures a user's complete accessibility needs and preferences for XR experiences.

```typescript
interface XRAccessibilityProfile {
  // Profile metadata
  profile_id: string;
  version: string;
  created_at: string;  // ISO 8601
  updated_at: string;  // ISO 8601

  // User information (anonymized)
  user_hash?: string;

  // Disability categories
  disabilities: DisabilityProfile;

  // Sensory preferences
  sensory: SensoryPreferences;

  // Input preferences
  input: InputPreferences;

  // Output preferences
  output: OutputPreferences;

  // Comfort settings
  comfort: ComfortSettings;

  // WIA device integrations
  wia_integrations?: WIAIntegrations;
}
```

### 2.2 Disability Profile

```typescript
interface DisabilityProfile {
  // Visual impairments
  visual?: VisualDisability;

  // Hearing impairments
  auditory?: AuditoryDisability;

  // Motor/physical impairments
  motor?: MotorDisability;

  // Cognitive/neurological conditions
  cognitive?: CognitiveDisability;

  // Speech impairments
  speech?: SpeechDisability;

  // Multiple disabilities flag
  has_multiple: boolean;
}

interface VisualDisability {
  level: "none" | "low_vision" | "legally_blind" | "totally_blind";

  // Specific conditions
  conditions?: VisualCondition[];

  // Color vision
  color_vision: "normal" | "protanopia" | "deuteranopia" | "tritanopia" | "achromatopsia";

  // Field of vision
  field_of_vision?: {
    horizontal_degrees: number;  // Normal: ~180
    vertical_degrees: number;    // Normal: ~120
    blind_spots?: BlindSpotRegion[];
  };

  // Light sensitivity
  photosensitivity: "none" | "mild" | "moderate" | "severe";
}

type VisualCondition =
  | "macular_degeneration"
  | "glaucoma"
  | "diabetic_retinopathy"
  | "retinitis_pigmentosa"
  | "cataracts"
  | "nystagmus"
  | "other";

interface BlindSpotRegion {
  center_x: number;  // -90 to 90 degrees
  center_y: number;  // -60 to 60 degrees
  radius: number;    // degrees
}

interface AuditoryDisability {
  level: "none" | "hard_of_hearing" | "deaf" | "deafblind";

  // Hearing thresholds (dB HL)
  hearing_threshold?: {
    left_ear: number;   // 0-120, >90 = profound
    right_ear: number;
  };

  // Frequency response
  frequency_response?: {
    low_freq_loss: boolean;    // <500Hz
    mid_freq_loss: boolean;    // 500-2000Hz
    high_freq_loss: boolean;   // >2000Hz
  };

  // Preferred communication
  preferred_communication: ("spoken" | "sign_language" | "written" | "cued_speech")[];

  // Sign language preference
  sign_language?: SignLanguageCode;

  // Uses hearing aids/cochlear implant
  uses_hearing_device: boolean;
  hearing_device_type?: "hearing_aid" | "cochlear_implant" | "bone_anchored";
}

type SignLanguageCode =
  | "ASL"   // American Sign Language
  | "BSL"   // British Sign Language
  | "KSL"   // Korean Sign Language
  | "JSL"   // Japanese Sign Language
  | "DGS"   // German Sign Language
  | "LSF"   // French Sign Language
  | "Auslan" // Australian Sign Language
  | "other";

interface MotorDisability {
  level: "none" | "mild" | "moderate" | "severe";

  // Affected body parts
  affected_areas: MotorAffectedArea[];

  // Mobility status
  mobility: "ambulatory" | "assisted_walking" | "wheelchair" | "bed_bound";

  // Fine motor control
  fine_motor: {
    dominant_hand: "left" | "right" | "ambidextrous" | "neither";
    grip_strength: "normal" | "reduced" | "minimal" | "none";
    precision: "normal" | "reduced" | "minimal";
    tremor: "none" | "mild" | "moderate" | "severe";
  };

  // Range of motion
  range_of_motion?: {
    neck_rotation: number;      // 0-100%
    arm_reach: number;          // 0-100%
    wrist_rotation: number;     // 0-100%
    finger_dexterity: number;   // 0-100%
  };

  // Stamina/fatigue
  fatigue_threshold_minutes?: number;

  // Assistive devices used
  assistive_devices?: MotorAssistiveDevice[];
}

type MotorAffectedArea =
  | "upper_limb_left"
  | "upper_limb_right"
  | "lower_limb_left"
  | "lower_limb_right"
  | "trunk"
  | "neck"
  | "face";

type MotorAssistiveDevice =
  | "wheelchair_manual"
  | "wheelchair_powered"
  | "walker"
  | "cane"
  | "prosthetic"
  | "wia_exoskeleton"
  | "adaptive_controller"
  | "mouth_stick"
  | "head_pointer"
  | "eye_tracker";

interface CognitiveDisability {
  level: "none" | "mild" | "moderate" | "significant";

  // Conditions
  conditions?: CognitiveCondition[];

  // Processing characteristics
  processing: {
    speed: "normal" | "slower" | "variable";
    working_memory: "normal" | "reduced" | "significantly_reduced";
    attention_span_minutes?: number;
    multitasking_ability: "normal" | "limited" | "single_task_only";
  };

  // Sensory processing
  sensory_processing?: {
    visual_overload_threshold: "high" | "normal" | "low" | "very_low";
    auditory_overload_threshold: "high" | "normal" | "low" | "very_low";
    haptic_sensitivity: "hypo" | "normal" | "hyper";
  };

  // Specific needs
  needs_predictability: boolean;
  needs_simplified_interface: boolean;
  needs_extended_time: boolean;

  // Photosensitive epilepsy
  photosensitive_epilepsy: boolean;

  // Motion sensitivity
  motion_sensitivity: "none" | "mild" | "moderate" | "severe";
}

type CognitiveCondition =
  | "autism_spectrum"
  | "adhd"
  | "dyslexia"
  | "dyscalculia"
  | "intellectual_disability"
  | "traumatic_brain_injury"
  | "dementia"
  | "epilepsy"
  | "ptsd"
  | "anxiety_disorder"
  | "other";

interface SpeechDisability {
  level: "none" | "mild" | "moderate" | "severe" | "nonverbal";

  // Conditions
  conditions?: SpeechCondition[];

  // Voice characteristics
  voice_characteristics?: {
    intelligibility: "clear" | "reduced" | "limited" | "unintelligible";
    volume: "normal" | "quiet" | "loud" | "variable";
    rate: "normal" | "slow" | "fast" | "variable";
  };

  // Alternative communication
  uses_aac: boolean;  // Augmentative and Alternative Communication
  aac_type?: "text_based" | "symbol_based" | "speech_generating";
}

type SpeechCondition =
  | "stuttering"
  | "dysarthria"
  | "apraxia"
  | "aphasia"
  | "voice_disorder"
  | "laryngectomy"
  | "other";
```

### 2.3 Sensory Preferences

```typescript
interface SensoryPreferences {
  // Visual preferences
  visual: {
    // Brightness and contrast
    brightness_multiplier: number;       // 0.5-2.0, default 1.0
    contrast_multiplier: number;         // 0.5-3.0, default 1.0
    saturation_multiplier: number;       // 0.0-2.0, default 1.0

    // Color filters
    color_filter?: ColorFilter;

    // Text preferences
    text_size_multiplier: number;        // 0.5-3.0, default 1.0
    font_preference: "default" | "sans_serif" | "dyslexia_friendly" | "high_legibility";

    // UI preferences
    ui_scale: number;                    // 0.5-2.0, default 1.0
    high_contrast_mode: boolean;
    reduce_transparency: boolean;
    reduce_motion: boolean;

    // Flash/strobe protection
    flash_threshold_hz?: number;         // Block flashes above this rate

    // Field of view
    preferred_fov_degrees?: number;      // Override default FOV
  };

  // Auditory preferences
  auditory: {
    // Volume
    master_volume: number;               // 0.0-1.0
    voice_volume: number;                // 0.0-1.0
    effects_volume: number;              // 0.0-1.0
    music_volume: number;                // 0.0-1.0

    // Balance
    balance: number;                     // -1.0 (left) to 1.0 (right)

    // Audio processing
    mono_audio: boolean;
    background_noise_reduction: boolean;
    voice_enhancement: boolean;

    // Spatial audio
    spatial_audio_enabled: boolean;
    spatial_audio_intensity: number;     // 0.0-1.0

    // Frequency adjustments
    bass_boost: number;                  // -1.0 to 1.0
    treble_boost: number;                // -1.0 to 1.0
  };

  // Haptic preferences
  haptic: {
    enabled: boolean;
    intensity: number;                   // 0.0-1.0

    // Haptic channels
    controller_haptics: boolean;
    vest_haptics: boolean;
    glove_haptics: boolean;

    // Haptic types
    vibration_enabled: boolean;
    force_feedback_enabled: boolean;
    texture_feedback_enabled: boolean;
    temperature_feedback_enabled: boolean;
  };
}

interface ColorFilter {
  type: "none" | "protanopia" | "deuteranopia" | "tritanopia" | "grayscale" | "inverted" | "custom";
  custom_matrix?: number[];  // 3x3 color transformation matrix
}
```

### 2.4 Input Preferences

```typescript
interface InputPreferences {
  // Primary input method
  primary_input: InputMethod;

  // Fallback input methods
  fallback_inputs: InputMethod[];

  // Controller settings
  controller: {
    // Hand preference
    dominant_hand: "left" | "right" | "either";

    // One-handed mode
    one_handed_mode: boolean;
    one_handed_controller: "left" | "right";

    // Button remapping
    button_remapping?: ButtonRemapping[];

    // Sensitivity
    stick_sensitivity: number;           // 0.1-2.0
    trigger_sensitivity: number;         // 0.1-2.0

    // Dead zones
    stick_deadzone: number;              // 0.0-0.5
    trigger_deadzone: number;            // 0.0-0.5

    // Hold vs toggle
    grip_toggle: boolean;
    trigger_toggle: boolean;
  };

  // Eye tracking settings
  eye_tracking?: {
    enabled: boolean;
    dwell_time_ms: number;               // Time to select by looking
    gaze_smoothing: number;              // 0.0-1.0
    blink_to_select: boolean;
  };

  // Voice control settings
  voice_control?: {
    enabled: boolean;
    language: string;                    // BCP-47 language code
    wake_word?: string;
    continuous_listening: boolean;
    command_timeout_ms: number;
  };

  // Head tracking settings
  head_tracking?: {
    enabled: boolean;
    sensitivity: number;                 // 0.1-2.0
    snap_turning: boolean;
    snap_angle_degrees?: number;         // 15, 30, 45, 60, 90
  };

  // Hand tracking settings
  hand_tracking?: {
    enabled: boolean;
    gesture_recognition: boolean;
    custom_gestures?: CustomGesture[];
  };
}

type InputMethod =
  | "controller_standard"
  | "controller_adaptive"
  | "eye_tracking"
  | "voice_control"
  | "head_tracking"
  | "hand_tracking"
  | "brain_computer_interface"
  | "switch_access"
  | "mouth_controller"
  | "wia_exoskeleton";

interface ButtonRemapping {
  original_button: string;
  remapped_to: string;
}

interface CustomGesture {
  gesture_id: string;
  gesture_name: string;
  hand: "left" | "right" | "both";
  action: string;
  gesture_data: GestureData;
}

interface GestureData {
  // Simplified gesture representation
  finger_states: {
    thumb: "extended" | "curled" | "any";
    index: "extended" | "curled" | "any";
    middle: "extended" | "curled" | "any";
    ring: "extended" | "curled" | "any";
    pinky: "extended" | "curled" | "any";
  };
  palm_direction?: "up" | "down" | "forward" | "back" | "any";
  motion?: "static" | "swipe" | "circle" | "wave";
}
```

### 2.5 Output Preferences

```typescript
interface OutputPreferences {
  // Caption settings
  captions: {
    enabled: boolean;

    // Appearance
    font_size: "small" | "medium" | "large" | "extra_large";
    font_family: string;
    text_color: string;                  // Hex color
    background_color: string;            // Hex color with alpha
    background_opacity: number;          // 0.0-1.0

    // Positioning in 3D space
    position: CaptionPosition;

    // Content options
    speaker_identification: boolean;
    sound_descriptions: boolean;
    music_descriptions: boolean;

    // Languages
    language: string;                    // BCP-47
    translate_from?: string[];           // Auto-translate from these languages
  };

  // Audio description settings
  audio_description: {
    enabled: boolean;
    voice: "male" | "female" | "neutral";
    speed: number;                       // 0.5-2.0
    detail_level: "minimal" | "standard" | "detailed";
    describe_actions: boolean;
    describe_scenes: boolean;
    describe_ui: boolean;
  };

  // Sign language settings
  sign_language?: {
    enabled: boolean;
    language: SignLanguageCode;
    avatar_style: "realistic" | "stylized" | "minimal";
    avatar_position: AvatarPosition;
    avatar_size: number;                 // 0.5-2.0
    signing_speed: number;               // 0.5-2.0
  };

  // Screen reader settings
  screen_reader: {
    enabled: boolean;
    verbosity: "low" | "medium" | "high";
    speak_hints: boolean;
    speak_notifications: boolean;
    voice_rate: number;                  // 0.5-3.0
    voice_pitch: number;                 // 0.5-2.0
  };

  // Notification settings
  notifications: {
    visual_alerts: boolean;
    audio_alerts: boolean;
    haptic_alerts: boolean;
    flash_screen: boolean;
    notification_position: NotificationPosition;
  };
}

interface CaptionPosition {
  mode: "floating" | "fixed" | "attached_to_speaker";

  // For floating/fixed modes
  position_3d?: {
    distance_meters: number;             // Distance from user
    vertical_angle: number;              // Degrees from horizon
    horizontal_offset: number;           // Degrees from center
  };

  // Following behavior
  follow_head: boolean;
  follow_speed: number;                  // 0.0-1.0 (0=static, 1=instant)
}

interface AvatarPosition {
  mode: "corner" | "beside_speaker" | "custom";
  corner?: "top_left" | "top_right" | "bottom_left" | "bottom_right";
  custom_position?: {
    x: number;                           // -1.0 to 1.0
    y: number;                           // -1.0 to 1.0
    z: number;                           // Distance in meters
  };
}

type NotificationPosition =
  | "top_center"
  | "bottom_center"
  | "top_left"
  | "top_right"
  | "wrist"
  | "floating";
```

### 2.6 Comfort Settings

```typescript
interface ComfortSettings {
  // Motion comfort
  motion: {
    // Movement type
    movement_type: "smooth" | "teleport" | "snap_turn" | "hybrid";

    // Vignette (tunnel vision effect to reduce motion sickness)
    vignette_enabled: boolean;
    vignette_intensity: number;          // 0.0-1.0
    vignette_on_movement: boolean;
    vignette_on_rotation: boolean;

    // Speed limits
    max_movement_speed: number;          // m/s
    max_rotation_speed: number;          // deg/s

    // Acceleration
    acceleration_smoothing: number;      // 0.0-1.0

    // Seated mode
    seated_mode: boolean;
    seated_height_offset_meters?: number;

    // Snap turn
    snap_turn_angle_degrees?: number;
  };

  // Physical comfort
  physical: {
    // Play area
    play_area_type: "seated" | "standing" | "room_scale";
    play_area_bounds?: {
      width_meters: number;
      depth_meters: number;
    };

    // Rest reminders
    rest_reminder_enabled: boolean;
    rest_reminder_interval_minutes: number;

    // Guardian/boundary
    boundary_style: "grid" | "wall" | "outline" | "passthrough";
    boundary_sensitivity: "close" | "normal" | "far";
  };

  // Cognitive comfort
  cognitive: {
    // Pacing
    allow_pause_anytime: boolean;
    auto_pause_on_overwhelm: boolean;

    // Complexity
    simplified_ui: boolean;
    reduce_simultaneous_stimuli: boolean;
    max_simultaneous_sounds: number;

    // Safe space
    safe_space_enabled: boolean;
    safe_space_trigger: "button" | "gesture" | "voice" | "automatic";
    safe_space_environment: string;      // Environment ID

    // Content warnings
    content_warnings_enabled: boolean;
    trigger_warnings?: string[];
  };

  // Session limits
  session: {
    max_session_duration_minutes?: number;
    required_break_duration_minutes?: number;
    warning_before_limit_minutes: number;
  };
}
```

### 2.7 WIA Device Integrations

```typescript
interface WIAIntegrations {
  // Exoskeleton integration
  exoskeleton?: {
    enabled: boolean;
    device_id: string;

    // Haptic feedback mapping
    haptic_mapping: {
      environment_textures: boolean;
      collision_feedback: boolean;
      ui_feedback: boolean;
      notification_feedback: boolean;
    };

    // Movement assistance
    movement_assistance: {
      gesture_assistance: boolean;
      assistance_level: "full" | "partial" | "guide";
    };
  };

  // Bionic eye integration
  bionic_eye?: {
    enabled: boolean;
    device_id: string;

    // Visual routing
    visual_mode: "vr_only" | "passthrough_only" | "mixed" | "picture_in_picture";

    // Enhancement settings
    contrast_enhancement: boolean;
    edge_detection: boolean;
    object_highlighting: boolean;
  };

  // Voice-Sign integration
  voice_sign?: {
    enabled: boolean;

    // Sign language display
    display_mode: "avatar" | "notation" | "both";
    avatar_in_vr: boolean;

    // Voice-to-sign for VR audio
    translate_vr_audio: boolean;
    translate_npc_speech: boolean;
    translate_player_voice: boolean;
  };
}
```

### 2.8 XR Device Capabilities

```typescript
interface XRDeviceCapabilities {
  // Device identification
  device_id: string;
  device_name: string;
  manufacturer: string;
  model: string;
  firmware_version: string;

  // Display capabilities
  display: {
    type: "lcd" | "oled" | "micro_oled" | "waveguide";
    resolution_per_eye: {
      width: number;
      height: number;
    };
    refresh_rates: number[];             // Hz
    field_of_view: {
      horizontal: number;
      vertical: number;
    };
    supports_passthrough: boolean;
    passthrough_color: boolean;
    supports_foveated_rendering: boolean;
  };

  // Audio capabilities
  audio: {
    has_speakers: boolean;
    speaker_type?: "integrated" | "over_ear" | "open_ear";
    has_microphone: boolean;
    supports_spatial_audio: boolean;
    supports_bone_conduction: boolean;
  };

  // Input capabilities
  input: {
    controller_type: "6dof" | "3dof" | "none";
    has_eye_tracking: boolean;
    has_hand_tracking: boolean;
    has_face_tracking: boolean;
    has_body_tracking: boolean;
    voice_control: boolean;

    // Controller details
    controller_features?: {
      has_haptics: boolean;
      has_triggers: boolean;
      has_grip: boolean;
      has_thumbstick: boolean;
      has_touchpad: boolean;
      button_count: number;
    };
  };

  // Haptic capabilities
  haptics: {
    controller_haptics: boolean;
    haptic_fidelity: "basic" | "hd" | "advanced";
    supports_external_haptics: boolean;
  };

  // Accessibility features
  built_in_accessibility: {
    screen_reader: boolean;
    magnification: boolean;
    color_correction: boolean;
    caption_support: boolean;
    voice_control: boolean;
    one_handed_mode: boolean;
    seated_mode: boolean;
  };

  // WIA compatibility
  wia_compatibility: {
    exoskeleton_compatible: boolean;
    bionic_eye_compatible: boolean;
    voice_sign_compatible: boolean;
    protocol_version: string;
  };
}
```

### 2.9 XR Environment Configuration

```typescript
interface XREnvironmentConfig {
  // Environment identification
  environment_id: string;
  environment_name: string;
  environment_type: "game" | "social" | "educational" | "therapeutic" | "productivity" | "entertainment";

  // Accessibility rating
  accessibility: {
    overall_rating: "not_accessible" | "partially_accessible" | "accessible" | "fully_accessible";

    // Per-category ratings
    visual_accessibility: AccessibilityRating;
    auditory_accessibility: AccessibilityRating;
    motor_accessibility: AccessibilityRating;
    cognitive_accessibility: AccessibilityRating;

    // Specific features
    features: {
      has_captions: boolean;
      has_audio_description: boolean;
      has_sign_language: boolean;
      has_colorblind_mode: boolean;
      has_high_contrast: boolean;
      has_simplified_mode: boolean;
      has_one_handed_mode: boolean;
      has_seated_mode: boolean;
      has_teleport_locomotion: boolean;
      has_comfort_settings: boolean;
    };

    // Content warnings
    content_warnings: ContentWarning[];
  };

  // Sensory characteristics
  sensory: {
    // Visual
    visual_intensity: "calm" | "moderate" | "intense";
    has_flashing: boolean;
    flash_frequency_max_hz?: number;
    has_bright_lights: boolean;
    has_darkness: boolean;
    dominant_colors: string[];

    // Auditory
    audio_intensity: "quiet" | "moderate" | "loud";
    has_sudden_sounds: boolean;
    has_continuous_background: boolean;

    // Motion
    motion_intensity: "minimal" | "moderate" | "intense";
    has_forced_movement: boolean;
    has_falling_sequences: boolean;
    has_vehicle_motion: boolean;
  };

  // Interaction requirements
  interaction: {
    minimum_input_methods: InputMethod[];
    required_physical_actions: PhysicalAction[];
    requires_voice: boolean;
    requires_precise_aim: boolean;
    requires_quick_reactions: boolean;
    reaction_time_required_ms?: number;
  };
}

interface AccessibilityRating {
  score: number;                         // 0-100
  issues: AccessibilityIssue[];
}

interface AccessibilityIssue {
  category: string;
  severity: "minor" | "moderate" | "major" | "blocker";
  description: string;
  workaround?: string;
}

interface ContentWarning {
  type: ContentWarningType;
  severity: "mild" | "moderate" | "strong";
  can_be_disabled: boolean;
  description?: string;
}

type ContentWarningType =
  | "flashing_lights"
  | "loud_sounds"
  | "sudden_events"
  | "heights"
  | "enclosed_spaces"
  | "crowds"
  | "violence"
  | "gore"
  | "body_horror"
  | "motion_intense"
  | "other";

type PhysicalAction =
  | "standing"
  | "walking"
  | "crouching"
  | "reaching_high"
  | "reaching_low"
  | "two_handed"
  | "pointing"
  | "grabbing"
  | "throwing"
  | "head_movement";
```

### 2.10 Real-Time Adaptation Event

```typescript
interface XRAdaptationEvent {
  // Event metadata
  event_id: string;
  timestamp: string;                     // ISO 8601
  session_id: string;

  // Event type
  event_type: AdaptationEventType;

  // Trigger
  trigger: {
    type: "automatic" | "user_initiated" | "system";
    reason: string;
    sensor_data?: SensorData;
  };

  // Adaptation applied
  adaptation: {
    category: "visual" | "auditory" | "haptic" | "input" | "comfort" | "content";
    parameter: string;
    previous_value: any;
    new_value: any;
    duration_ms?: number;                // For temporary adaptations
  };

  // User feedback (optional)
  user_feedback?: {
    helpful: boolean;
    rating?: number;                     // 1-5
    comment?: string;
  };
}

type AdaptationEventType =
  | "comfort_adjustment"
  | "sensory_overload_mitigation"
  | "fatigue_response"
  | "motion_sickness_prevention"
  | "accessibility_enhancement"
  | "emergency_response"
  | "user_preference_change";

interface SensorData {
  // Biometric data (if available)
  heart_rate?: number;
  heart_rate_variability?: number;
  skin_conductance?: number;

  // Movement data
  head_movement_velocity?: number;
  body_sway?: number;

  // Eye data
  pupil_dilation?: number;
  blink_rate?: number;
  gaze_stability?: number;

  // Session data
  session_duration_minutes: number;
  movement_intensity: number;            // 0.0-1.0
}
```

## 3. JSON Schema Files

The following JSON Schema files define the validation rules:

- `accessibility-profile.schema.json` - User accessibility profile
- `device-capabilities.schema.json` - XR device capabilities
- `environment-config.schema.json` - Environment accessibility configuration
- `adaptation-event.schema.json` - Real-time adaptation events
- `sensory-preferences.schema.json` - Sensory preference settings
- `input-preferences.schema.json` - Input method preferences
- `output-preferences.schema.json` - Output and feedback preferences
- `comfort-settings.schema.json` - Comfort and safety settings

## 4. Data Exchange Format

### 4.1 Profile Export/Import

```json
{
  "$schema": "https://wia.org/schemas/xr/accessibility-profile.schema.json",
  "profile_id": "prof_abc123",
  "version": "1.0.0",
  "created_at": "2025-01-15T10:00:00Z",
  "updated_at": "2025-01-15T10:00:00Z",
  "disabilities": {
    "visual": {
      "level": "low_vision",
      "color_vision": "deuteranopia",
      "photosensitivity": "moderate"
    },
    "motor": {
      "level": "moderate",
      "mobility": "wheelchair",
      "fine_motor": {
        "dominant_hand": "right",
        "grip_strength": "reduced",
        "precision": "reduced",
        "tremor": "mild"
      }
    },
    "has_multiple": true
  },
  "sensory": {
    "visual": {
      "brightness_multiplier": 0.8,
      "contrast_multiplier": 1.5,
      "color_filter": { "type": "deuteranopia" },
      "text_size_multiplier": 1.5,
      "high_contrast_mode": true,
      "reduce_motion": true,
      "flash_threshold_hz": 3
    },
    "auditory": {
      "master_volume": 0.7,
      "spatial_audio_enabled": true
    },
    "haptic": {
      "enabled": true,
      "intensity": 0.8
    }
  },
  "input": {
    "primary_input": "eye_tracking",
    "fallback_inputs": ["voice_control", "head_tracking"],
    "eye_tracking": {
      "enabled": true,
      "dwell_time_ms": 800,
      "gaze_smoothing": 0.5
    }
  },
  "comfort": {
    "motion": {
      "movement_type": "teleport",
      "vignette_enabled": true,
      "seated_mode": true
    }
  },
  "wia_integrations": {
    "voice_sign": {
      "enabled": true,
      "translate_vr_audio": true
    }
  }
}
```

## 5. Implementation Notes

### 5.1 Privacy Considerations

- Disability information is highly sensitive PII
- Profiles should be stored encrypted
- User consent required for data collection
- Anonymization for analytics

### 5.2 Interoperability

- JSON Schema validation ensures consistency
- Semantic versioning for schema evolution
- Backward compatibility maintained

### 5.3 Performance

- Profiles loaded at session start
- Real-time adaptations cached locally
- Incremental updates to reduce bandwidth

---

弘益人間 🤟
