//! Core type definitions for WIA Voice-Sign API
//!
//! Based on WIA Voice-Sign Phase 1 Data Format Specification

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

// ============================================================================
// Audio Input Types
// ============================================================================

/// Audio input data for voice-to-sign translation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioInput {
    /// Unique identifier for the audio input
    pub audio_id: String,

    /// ISO 8601 timestamp of audio capture
    pub timestamp: DateTime<Utc>,

    /// Audio file format
    pub format: AudioFormat,

    /// Sample rate in Hz
    pub sample_rate: u32,

    /// Number of audio channels (1=mono, 2=stereo)
    pub channels: u8,

    /// Bit depth per sample
    pub bit_depth: u8,

    /// Duration in milliseconds
    pub duration_ms: f64,

    /// Optional audio source metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<AudioSource>,

    /// ISO 639-1 language hint
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language_hint: Option<String>,

    /// Base64 encoded audio data
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(with = "serde_bytes")]
    pub data: Option<Vec<u8>>,

    /// URL to audio data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_url: Option<String>,
}

/// Supported audio formats
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AudioFormat {
    Wav,
    Pcm,
    Flac,
    Ogg,
    Mp3,
    Webm,
}

/// Audio source metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioSource {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub device_name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<GeoLocation>,
}

/// Geographic location
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GeoLocation {
    pub latitude: f64,
    pub longitude: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub accuracy_m: Option<f64>,
}

// ============================================================================
// Transcription Types
// ============================================================================

/// Speech recognition (ASR) result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranscriptionResult {
    /// Reference to source audio
    pub audio_id: String,

    /// Unique identifier for this transcription
    pub transcription_id: String,

    /// Timestamp of transcription
    pub timestamp: DateTime<Utc>,

    /// ISO 639-1 detected language code
    pub language: String,

    /// Overall transcription confidence (0.0 - 1.0)
    pub confidence: f32,

    /// Full transcribed text
    pub text: String,

    /// Word-level transcription details
    pub words: Vec<TranscribedWord>,

    /// Alternative transcription hypotheses
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alternatives: Option<Vec<TranscriptionAlternative>>,

    /// ASR model used
    pub model: String,

    /// Processing time in milliseconds
    pub processing_time_ms: f64,
}

/// Single transcribed word with timing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranscribedWord {
    pub word: String,
    pub start_ms: f64,
    pub end_ms: f64,
    pub confidence: f32,

    /// Part of speech
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pos: Option<PartOfSpeech>,

    /// Base form of the word
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lemma: Option<String>,
}

/// Part of speech tags
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PartOfSpeech {
    Noun,
    Verb,
    Adjective,
    Adverb,
    Pronoun,
    Preposition,
    Conjunction,
    Interjection,
    Determiner,
    Particle,
}

/// Alternative transcription hypothesis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranscriptionAlternative {
    pub text: String,
    pub confidence: f32,
}

// ============================================================================
// Sign Gloss Types
// ============================================================================

/// Sign language gloss sequence
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignGlossSequence {
    /// Reference to source transcription
    pub transcription_id: String,

    /// Unique identifier for this gloss sequence
    pub gloss_id: String,

    /// Timestamp
    pub timestamp: DateTime<Utc>,

    /// Target sign language
    pub sign_language: SignLanguageCode,

    /// Gloss entries
    pub glosses: Vec<SignGloss>,

    /// Sentence type
    pub sentence_type: SentenceType,

    /// Overall confidence
    pub confidence: f32,

    /// Translation model used
    pub translation_model: String,

    /// Processing time in milliseconds
    pub processing_time_ms: f64,
}

/// Sign language codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum SignLanguageCode {
    /// American Sign Language
    Asl,
    /// British Sign Language
    Bsl,
    /// French Sign Language
    Lsf,
    /// German Sign Language
    Dgs,
    /// Japanese Sign Language
    Jsl,
    /// Korean Sign Language
    Ksl,
    /// Chinese Sign Language
    Csl,
    /// Australian Sign Language
    Auslan,
    /// New Zealand Sign Language
    Nzsl,
}

impl std::fmt::Display for SignLanguageCode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Asl => write!(f, "ASL"),
            Self::Bsl => write!(f, "BSL"),
            Self::Lsf => write!(f, "LSF"),
            Self::Dgs => write!(f, "DGS"),
            Self::Jsl => write!(f, "JSL"),
            Self::Ksl => write!(f, "KSL"),
            Self::Csl => write!(f, "CSL"),
            Self::Auslan => write!(f, "Auslan"),
            Self::Nzsl => write!(f, "NZSL"),
        }
    }
}

/// Single sign gloss entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignGloss {
    /// Gloss identifier (e.g., "HELLO", "THANK-YOU")
    pub gloss: String,

    /// Start time in milliseconds
    pub start_ms: f64,

    /// End time in milliseconds
    pub end_ms: f64,

    /// Duration in milliseconds
    pub duration_ms: f64,

    /// Type of sign
    pub sign_type: SignType,

    /// HamNoSys notation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hamnosys: Option<String>,

    /// SiGML XML notation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sigml: Option<String>,

    /// SignWriting FSW notation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signwriting: Option<String>,

    /// Modifiers
    #[serde(skip_serializing_if = "Option::is_none")]
    pub modifiers: Option<Vec<GlossModifier>>,

    /// Spatial reference
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spatial_ref: Option<SpatialReference>,

    /// Confidence score
    pub confidence: f32,
}

/// Sign types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SignType {
    /// Lexical sign (vocabulary word)
    Lexical,
    /// Fingerspelling
    Fingerspell,
    /// Classifier
    Classifier,
    /// Depicting sign
    Depicting,
    /// Pointing/indexing
    Pointing,
    /// Number
    Number,
}

/// Sentence types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SentenceType {
    Declarative,
    Interrogative,
    Imperative,
    Exclamatory,
}

/// Gloss modifier
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GlossModifier {
    #[serde(rename = "type")]
    pub modifier_type: ModifierType,
    pub value: ModifierValue,
}

/// Modifier types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ModifierType {
    Intensity,
    Speed,
    Repetition,
    Negation,
    Question,
    Aspect,
}

/// Modifier value (string or number)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ModifierValue {
    String(String),
    Number(f64),
}

/// Spatial reference in signing space
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SpatialReference {
    /// Left to right (-1.0 to 1.0)
    pub x: f32,
    /// Down to up (-1.0 to 1.0)
    pub y: f32,
    /// Back to front (-1.0 to 1.0)
    pub z: f32,
}

// ============================================================================
// Sign Notation Types
// ============================================================================

/// Detailed sign notation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignNotation {
    pub gloss_id: String,
    pub gloss: String,
    pub notation_type: NotationType,
    pub manual: ManualComponent,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub non_manual: Option<NonManualComponent>,
}

/// Notation types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NotationType {
    Hamnosys,
    Sigml,
    Custom,
}

/// Manual (hand) component of a sign
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManualComponent {
    /// Dominant hand configuration
    pub dominant: HandConfiguration,

    /// Non-dominant hand configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub non_dominant: Option<HandConfiguration>,

    /// Relationship between hands
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hand_relationship: Option<HandRelationship>,

    /// Movement
    pub movement: MovementComponent,
}

/// Hand configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandConfiguration {
    /// Handshape code
    pub handshape: String,

    /// Thumb position
    #[serde(skip_serializing_if = "Option::is_none")]
    pub thumb: Option<ThumbPosition>,

    /// Palm orientation
    pub palm_orientation: Orientation,

    /// Finger direction
    pub finger_direction: Orientation,

    /// Location relative to body
    pub location: BodyLocation,

    /// Contact type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub contact: Option<ContactType>,
}

/// Thumb positions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ThumbPosition {
    Out,
    Across,
    Tucked,
}

/// Hand relationship types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HandRelationship {
    Parallel,
    Crossed,
    Interlocked,
    Stacked,
    Mirror,
}

/// Contact types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ContactType {
    Touch,
    Grasp,
    Brush,
    None,
}

/// Orientation in 3D space
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Orientation {
    pub direction: Direction,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub rotation_deg: Option<f32>,
}

/// Direction types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
    Forward,
    Back,
    UpLeft,
    UpRight,
    DownLeft,
    DownRight,
}

/// Body location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyLocation {
    pub region: BodyRegion,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub specific: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub offset: Option<Vector3>,
}

/// Body regions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyRegion {
    Head,
    Forehead,
    Eyes,
    Nose,
    Mouth,
    Chin,
    Cheek,
    Ear,
    Neck,
    Shoulder,
    Chest,
    Stomach,
    Arm,
    Elbow,
    Wrist,
    NeutralSpace,
}

/// Movement component
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementComponent {
    pub path: MovementPath,
    pub dynamics: MovementDynamics,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub repetition: Option<RepetitionInfo>,
}

/// Movement path
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementPath {
    #[serde(rename = "type")]
    pub path_type: PathType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub direction: Option<Direction>,

    pub size: PathSize,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_direction: Option<ArcDirection>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub waypoints: Option<Vec<Vector3>>,
}

/// Path types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PathType {
    Straight,
    Arc,
    Circle,
    Zigzag,
    Wave,
    Spiral,
    None,
}

/// Path sizes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PathSize {
    Small,
    Medium,
    Large,
}

/// Arc directions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ArcDirection {
    Clockwise,
    Counterclockwise,
}

/// Movement dynamics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementDynamics {
    pub speed: Speed,
    pub tension: Tension,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub acceleration: Option<Acceleration>,
}

/// Speed types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Speed {
    Slow,
    Normal,
    Fast,
}

/// Tension types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Tension {
    Relaxed,
    Normal,
    Tense,
}

/// Acceleration types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Acceleration {
    Constant,
    Accelerate,
    Decelerate,
}

/// Repetition info
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RepetitionInfo {
    pub count: u32,
    pub bidirectional: bool,
}

// ============================================================================
// Non-Manual Component Types
// ============================================================================

/// Non-manual component (facial expressions, etc.)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NonManualComponent {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub facial: Option<FacialExpression>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub mouth: Option<MouthComponent>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub eyes: Option<EyeComponent>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub head: Option<HeadComponent>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub body: Option<BodyComponent>,
}

/// Facial expression
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FacialExpression {
    pub expression: ExpressionType,
    pub intensity: f32,
}

/// Expression types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExpressionType {
    Neutral,
    RaisedBrows,
    FurrowedBrows,
    Squint,
    WideEyes,
    Smile,
    Frown,
    PursedLips,
}

/// Mouth component
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MouthComponent {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mouthing: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gesture: Option<MouthGesture>,
}

/// Mouth gestures
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MouthGesture {
    Neutral,
    Pah,
    Cha,
    Mm,
    Th,
    PuffCheeks,
    BiteLip,
}

/// Eye component
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeComponent {
    pub gaze: GazeDirection,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub blink: Option<bool>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub squint: Option<bool>,
}

/// Gaze directions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GazeDirection {
    Forward,
    Up,
    Down,
    Left,
    Right,
    Addressee,
    Referent,
}

/// Head component
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeadComponent {
    pub movement: HeadMovement,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub tilt: Option<HeadTilt>,
}

/// Head movements
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HeadMovement {
    None,
    Nod,
    Shake,
    TiltForward,
    TiltBack,
}

/// Head tilt
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HeadTilt {
    Left,
    Right,
    None,
}

/// Body component
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyComponent {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lean: Option<BodyLean>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub shoulder_raise: Option<ShoulderRaise>,
}

/// Body lean directions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BodyLean {
    Forward,
    Back,
    Left,
    Right,
    None,
}

/// Shoulder raise options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ShoulderRaise {
    Left,
    Right,
    Both,
    None,
}

// ============================================================================
// Pose Types
// ============================================================================

/// Sign pose sequence for animation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignPoseSequence {
    pub gloss_id: String,
    pub pose_sequence_id: String,
    pub timestamp: DateTime<Utc>,
    pub frame_rate: u32,
    pub total_frames: u32,
    pub duration_ms: f64,
    pub skeleton: SkeletonDefinition,
    pub frames: Vec<PoseFrame>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub blend_shapes: Option<Vec<BlendShapeTrack>>,
}

/// Skeleton definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SkeletonDefinition {
    pub standard: SkeletonStandard,
    pub joints: Vec<JointDefinition>,
}

/// Skeleton standards
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SkeletonStandard {
    MediapipeHolistic,
    Openpose,
    WiaSignSkeleton,
    Custom,
}

/// Joint definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointDefinition {
    pub id: u32,
    pub name: String,
    pub parent_id: Option<u32>,
}

/// Single pose frame
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseFrame {
    pub frame_index: u32,
    pub timestamp_ms: f64,
    pub joints: Vec<JointPose>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub left_hand: Option<HandPose>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub right_hand: Option<HandPose>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub face: Option<FacePose>,
}

/// Joint pose data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointPose {
    pub joint_id: u32,
    pub position: Vector3,
    pub rotation: Quaternion,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f32>,
}

/// 3D vector
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self::default()
    }
}

/// Quaternion for rotation
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

impl Quaternion {
    pub fn identity() -> Self {
        Self::default()
    }
}

/// Hand pose with 21 landmarks
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandPose {
    pub landmarks: Vec<HandLandmark>,
}

/// Hand landmark
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandLandmark {
    pub id: u8,
    pub name: HandLandmarkName,
    pub position: Vector3,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f32>,
}

/// Hand landmark names (MediaPipe standard)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HandLandmarkName {
    Wrist,
    ThumbCmc,
    ThumbMcp,
    ThumbIp,
    ThumbTip,
    IndexMcp,
    IndexPip,
    IndexDip,
    IndexTip,
    MiddleMcp,
    MiddlePip,
    MiddleDip,
    MiddleTip,
    RingMcp,
    RingPip,
    RingDip,
    RingTip,
    PinkyMcp,
    PinkyPip,
    PinkyDip,
    PinkyTip,
}

/// Face pose
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FacePose {
    pub landmarks: Vec<FaceLandmark>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub blend_weights: Option<std::collections::HashMap<String, f32>>,
}

/// Face landmark
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaceLandmark {
    pub id: u32,
    pub position: Vector3,
}

/// Blend shape animation track
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlendShapeTrack {
    pub name: String,
    pub keyframes: Vec<BlendShapeKeyframe>,
}

/// Blend shape keyframe
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BlendShapeKeyframe {
    pub timestamp_ms: f64,
    pub value: f32,
}

// ============================================================================
// Request/Response Types
// ============================================================================

/// Translation request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranslationRequest {
    pub request_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio: Option<AudioInput>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<TextInput>,

    pub target_language: SignLanguageCode,
    pub output: OutputPreferences,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<TranslationOptions>,
}

/// Text input
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextInput {
    pub text: String,
    pub language: String,
}

/// Output preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputPreferences {
    pub include_transcript: bool,
    pub include_gloss: bool,
    pub include_notation: bool,
    pub include_pose: bool,
    pub include_render: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub render_settings: Option<RenderSettings>,
}

/// Render settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RenderSettings {
    pub format: OutputType,
    pub resolution: Resolution,
    pub frame_rate: u32,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub avatar_id: Option<String>,
}

/// Output types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum OutputType {
    Video,
    Animation,
    Streaming,
}

/// Resolution
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Resolution {
    pub width: u32,
    pub height: u32,
}

/// Translation options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranslationOptions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub asr_model: Option<String>,

    #[serde(default = "default_true")]
    pub language_detection: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub translation_model: Option<String>,

    #[serde(default)]
    pub formality_level: FormalityLevel,

    #[serde(default)]
    pub streaming: bool,

    #[serde(default = "default_chunk_duration")]
    pub chunk_duration_ms: u32,
}

fn default_true() -> bool {
    true
}

fn default_chunk_duration() -> u32 {
    500
}

/// Formality levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FormalityLevel {
    Formal,
    Informal,
    #[default]
    Neutral,
}

/// Translation response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranslationResponse {
    pub request_id: String,
    pub response_id: String,
    pub timestamp: DateTime<Utc>,
    pub status: ResponseStatus,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ErrorInfo>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub transcript: Option<TranscriptionResult>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gloss: Option<SignGlossSequence>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub notation: Option<Vec<SignNotation>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pose: Option<SignPoseSequence>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub render: Option<RenderOutput>,

    pub processing: ProcessingSummary,
}

/// Response status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ResponseStatus {
    Success,
    Partial,
    Error,
}

/// Error info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorInfo {
    pub code: String,
    pub message: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

/// Render output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RenderOutput {
    pub pose_sequence_id: String,
    pub render_id: String,
    pub timestamp: DateTime<Utc>,
    pub format: OutputFormat,
    pub resolution: Resolution,
    pub frame_rate: u32,
    pub duration_ms: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub file_size_bytes: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_url: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub avatar: Option<AvatarInfo>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_score: Option<f32>,
}

/// Output format details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputFormat {
    #[serde(rename = "type")]
    pub output_type: OutputType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub codec: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub container: Option<String>,
}

/// Avatar info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AvatarInfo {
    pub avatar_id: String,
    pub name: String,
    pub style: AvatarStyle,
}

/// Avatar styles
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AvatarStyle {
    Realistic,
    Stylized,
    Cartoon,
    Minimal,
}

/// Processing summary
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessingSummary {
    pub total_time_ms: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub asr_time_ms: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub translation_time_ms: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pose_generation_time_ms: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub render_time_ms: Option<f64>,
}
