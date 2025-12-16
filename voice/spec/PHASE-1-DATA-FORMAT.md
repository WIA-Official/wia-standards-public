# WIA Voice-Sign Phase 1: Data Format Specification

## 1. Overview

본 문서는 음성-수화 변환 시스템의 데이터 형식을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 2. Pipeline Data Flow

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ AudioInput  │───▶│ Transcript  │───▶│  SignGloss  │───▶│ SignPose    │
│             │    │   Result    │    │   Sequence  │    │  Sequence   │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
      │                  │                  │                  │
      ▼                  ▼                  ▼                  ▼
  [audio.*]        [transcript.*]     [gloss.*]         [pose.*]
```

---

## 3. Core Data Types

### 3.1 AudioInput

음성 입력 데이터 및 메타데이터

```typescript
interface AudioInput {
  // Unique identifier
  audio_id: string;

  // Timestamps
  timestamp: string;  // ISO 8601

  // Audio properties
  format: AudioFormat;
  sample_rate: number;    // Hz (16000, 44100, 48000)
  channels: number;       // 1 = mono, 2 = stereo
  bit_depth: number;      // 16, 24, 32
  duration_ms: number;

  // Optional metadata
  source?: AudioSource;
  language_hint?: string;  // ISO 639-1

  // Data (base64 or reference)
  data?: string;           // base64 encoded
  data_url?: string;       // URL reference
}

type AudioFormat =
  | "wav"
  | "pcm"
  | "flac"
  | "ogg"
  | "mp3"
  | "webm";

interface AudioSource {
  device_id?: string;
  device_name?: string;
  location?: GeoLocation;
}
```

### 3.2 TranscriptionResult

음성 인식 결과

```typescript
interface TranscriptionResult {
  // Reference to audio
  audio_id: string;

  // Result metadata
  transcription_id: string;
  timestamp: string;

  // Recognition results
  language: string;           // ISO 639-1 detected language
  confidence: number;         // 0.0 - 1.0

  // Full text
  text: string;

  // Word-level details
  words: TranscribedWord[];

  // Alternative hypotheses
  alternatives?: TranscriptionAlternative[];

  // Processing info
  model: string;
  processing_time_ms: number;
}

interface TranscribedWord {
  word: string;
  start_ms: number;
  end_ms: number;
  confidence: number;

  // Linguistic annotations
  pos?: PartOfSpeech;        // Part of speech
  lemma?: string;            // Base form
}

type PartOfSpeech =
  | "noun"
  | "verb"
  | "adjective"
  | "adverb"
  | "pronoun"
  | "preposition"
  | "conjunction"
  | "interjection"
  | "determiner"
  | "particle";

interface TranscriptionAlternative {
  text: string;
  confidence: number;
}
```

### 3.3 SignGloss

수화 표현의 중간 표현 (Gloss)

```typescript
interface SignGlossSequence {
  // References
  transcription_id: string;

  // Sequence metadata
  gloss_id: string;
  timestamp: string;

  // Target sign language
  sign_language: SignLanguageCode;

  // Gloss entries
  glosses: SignGloss[];

  // Sentence-level info
  sentence_type: SentenceType;

  // Translation confidence
  confidence: number;

  // Processing info
  translation_model: string;
  processing_time_ms: number;
}

interface SignGloss {
  // Gloss identifier
  gloss: string;              // e.g., "HELLO", "THANK-YOU"

  // Timing
  start_ms: number;
  end_ms: number;
  duration_ms: number;

  // Sign details
  sign_type: SignType;

  // Notation representations
  hamnosys?: string;          // HamNoSys notation
  sigml?: string;             // SiGML XML
  signwriting?: string;       // SignWriting FSW

  // Modifiers
  modifiers?: GlossModifier[];

  // Spatial reference
  spatial_ref?: SpatialReference;

  // Confidence
  confidence: number;
}

type SignLanguageCode =
  | "ASL"    // American
  | "BSL"    // British
  | "LSF"    // French
  | "DGS"    // German
  | "JSL"    // Japanese
  | "KSL"    // Korean
  | "CSL"    // Chinese
  | "Auslan" // Australian
  | "NZSL"   // New Zealand
  | string;  // ISO 639-3 sgn-* codes

type SignType =
  | "lexical"        // 어휘 수화
  | "fingerspell"    // 지문자
  | "classifier"     // 분류사
  | "depicting"      // 묘사 수화
  | "pointing"       // 지시
  | "number";        // 숫자

type SentenceType =
  | "declarative"    // 평서문
  | "interrogative"  // 의문문
  | "imperative"     // 명령문
  | "exclamatory";   // 감탄문

interface GlossModifier {
  type: ModifierType;
  value: string | number;
}

type ModifierType =
  | "intensity"      // 강도 (크게, 작게)
  | "speed"          // 속도 (빠르게, 느리게)
  | "repetition"     // 반복
  | "negation"       // 부정
  | "question"       // 의문
  | "aspect";        // 상 (진행, 완료)

interface SpatialReference {
  // 3D signing space coordinates
  x: number;  // -1.0 to 1.0 (left to right)
  y: number;  // -1.0 to 1.0 (down to up)
  z: number;  // -1.0 to 1.0 (back to front)
}
```

### 3.4 SignNotation

상세 수화 표기 (HamNoSys/SiGML)

```typescript
interface SignNotation {
  // Reference
  gloss_id: string;
  gloss: string;

  // Notation type
  notation_type: NotationType;

  // Manual components (손 동작)
  manual: ManualComponent;

  // Non-manual components (비수지 요소)
  non_manual?: NonManualComponent;
}

type NotationType = "hamnosys" | "sigml" | "custom";

interface ManualComponent {
  // Dominant hand
  dominant: HandConfiguration;

  // Non-dominant hand (optional)
  non_dominant?: HandConfiguration;

  // Two-hand relationship
  hand_relationship?: HandRelationship;

  // Movement
  movement: MovementComponent;
}

interface HandConfiguration {
  // Handshape (손 모양)
  handshape: HandshapeCode;

  // Thumb position
  thumb: ThumbPosition;

  // Finger bend
  finger_bend?: FingerBend;

  // Palm orientation (손바닥 방향)
  palm_orientation: Orientation;

  // Extended finger direction (손가락 방향)
  finger_direction: Orientation;

  // Location relative to body
  location: BodyLocation;

  // Contact with body
  contact?: ContactType;
}

// HamNoSys handshape codes (simplified)
type HandshapeCode =
  | "fist"           // 주먹
  | "flat"           // 펴진 손
  | "flat_bent"      // 굽힌 펴진 손
  | "index"          // 검지만 펴짐
  | "index_mid"      // 검지+중지
  | "thumb_index_l"  // 엄지+검지 L자
  | "thumb_index_o"  // 엄지+검지 O자
  | "five_spread"    // 다섯 손가락 펼침
  | "claw"           // 갈퀴 모양
  | "hook"           // 갈고리
  | string;          // Custom code

type ThumbPosition = "out" | "across" | "tucked";

interface FingerBend {
  index: BendDegree;
  middle: BendDegree;
  ring: BendDegree;
  pinky: BendDegree;
}

type BendDegree = "straight" | "bent" | "hooked" | "closed";

interface Orientation {
  // Direction in 3D space
  direction: Direction;
  // Rotation angle (optional fine-tuning)
  rotation_deg?: number;
}

type Direction =
  | "up" | "down"
  | "left" | "right"
  | "forward" | "back"
  | "up_left" | "up_right"
  | "down_left" | "down_right";

interface BodyLocation {
  // Major body region
  region: BodyRegion;

  // Specific location within region
  specific?: string;

  // Offset from center
  offset?: {
    x: number;
    y: number;
    z: number;
  };
}

type BodyRegion =
  | "head"
  | "forehead"
  | "eyes"
  | "nose"
  | "mouth"
  | "chin"
  | "cheek"
  | "ear"
  | "neck"
  | "shoulder"
  | "chest"
  | "stomach"
  | "arm"
  | "elbow"
  | "wrist"
  | "neutral_space";  // 중립 공간 (몸 앞)

type ContactType = "touch" | "grasp" | "brush" | "none";

type HandRelationship =
  | "parallel"       // 평행
  | "crossed"        // 교차
  | "interlocked"    // 깍지
  | "stacked"        // 겹침
  | "mirror";        // 거울 대칭

interface MovementComponent {
  // Movement path
  path: MovementPath;

  // Movement dynamics
  dynamics: MovementDynamics;

  // Repetition
  repetition?: RepetitionInfo;
}

interface MovementPath {
  type: PathType;
  direction?: Direction;
  size: PathSize;

  // For curved/arc paths
  arc_direction?: "clockwise" | "counterclockwise";

  // Path waypoints (optional for complex movements)
  waypoints?: Point3D[];
}

type PathType =
  | "straight"
  | "arc"
  | "circle"
  | "zigzag"
  | "wave"
  | "spiral"
  | "none";       // 고정 위치

type PathSize = "small" | "medium" | "large";

interface MovementDynamics {
  speed: Speed;
  tension: Tension;

  // Acceleration profile
  acceleration?: "constant" | "accelerate" | "decelerate";
}

type Speed = "slow" | "normal" | "fast";
type Tension = "relaxed" | "normal" | "tense";

interface RepetitionInfo {
  count: number;        // 1 = no repetition
  bidirectional: boolean;
}

interface Point3D {
  x: number;
  y: number;
  z: number;
}

// Non-manual components (표정, 입모양 등)
interface NonManualComponent {
  // Facial expression
  facial?: FacialExpression;

  // Mouth shape/movement
  mouth?: MouthComponent;

  // Eye behavior
  eyes?: EyeComponent;

  // Head movement
  head?: HeadComponent;

  // Body posture
  body?: BodyComponent;
}

interface FacialExpression {
  expression: ExpressionType;
  intensity: number;  // 0.0 - 1.0
}

type ExpressionType =
  | "neutral"
  | "raised_brows"      // 의문
  | "furrowed_brows"    // WH-의문
  | "squint"
  | "wide_eyes"
  | "smile"
  | "frown"
  | "pursed_lips";

interface MouthComponent {
  // Mouthing (단어 입모양)
  mouthing?: string;

  // Mouth gesture
  gesture?: MouthGesture;
}

type MouthGesture =
  | "neutral"
  | "pah"           // 갑자기/빨리
  | "cha"           // 크다
  | "mm"            // 보통/일반
  | "th"            // 부주의하게
  | "puff_cheeks"
  | "bite_lip";

interface EyeComponent {
  gaze: GazeDirection;
  blink?: boolean;
  squint?: boolean;
}

type GazeDirection =
  | "forward"
  | "up" | "down"
  | "left" | "right"
  | "addressee"     // 대화 상대 방향
  | "referent";     // 지시 대상 방향

interface HeadComponent {
  movement: HeadMovement;
  tilt?: HeadTilt;
}

type HeadMovement =
  | "none"
  | "nod"           // 긍정
  | "shake"         // 부정
  | "tilt_forward"
  | "tilt_back";

type HeadTilt = "left" | "right" | "none";

interface BodyComponent {
  lean?: BodyLean;
  shoulder_raise?: "left" | "right" | "both" | "none";
}

type BodyLean = "forward" | "back" | "left" | "right" | "none";
```

### 3.5 SignPose (Animation Data)

3D 아바타 포즈 데이터

```typescript
interface SignPoseSequence {
  // Reference
  gloss_id: string;

  // Sequence metadata
  pose_sequence_id: string;
  timestamp: string;

  // Animation properties
  frame_rate: number;        // FPS (30, 60)
  total_frames: number;
  duration_ms: number;

  // Skeleton definition
  skeleton: SkeletonDefinition;

  // Frame data
  frames: PoseFrame[];

  // Blend shapes for face (optional)
  blend_shapes?: BlendShapeTrack[];
}

interface SkeletonDefinition {
  // Skeleton standard
  standard: SkeletonStandard;

  // Joint definitions
  joints: JointDefinition[];
}

type SkeletonStandard =
  | "mediapipe_holistic"   // Google MediaPipe
  | "openpose"             // CMU OpenPose
  | "wia_sign_skeleton"    // WIA Standard
  | "custom";

interface JointDefinition {
  id: number;
  name: string;
  parent_id: number | null;  // null for root
}

interface PoseFrame {
  frame_index: number;
  timestamp_ms: number;

  // Joint positions/rotations
  joints: JointPose[];

  // Hand details (higher resolution)
  left_hand?: HandPose;
  right_hand?: HandPose;

  // Facial landmarks (optional)
  face?: FacePose;
}

interface JointPose {
  joint_id: number;

  // Position in 3D space (meters from root)
  position: Vector3;

  // Rotation (quaternion)
  rotation: Quaternion;

  // Confidence (from pose estimation)
  confidence?: number;
}

interface Vector3 {
  x: number;
  y: number;
  z: number;
}

interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

interface HandPose {
  // 21 landmarks per hand (MediaPipe standard)
  landmarks: HandLandmark[];
}

interface HandLandmark {
  id: number;  // 0-20
  name: HandLandmarkName;
  position: Vector3;
  confidence?: number;
}

type HandLandmarkName =
  | "wrist"
  | "thumb_cmc" | "thumb_mcp" | "thumb_ip" | "thumb_tip"
  | "index_mcp" | "index_pip" | "index_dip" | "index_tip"
  | "middle_mcp" | "middle_pip" | "middle_dip" | "middle_tip"
  | "ring_mcp" | "ring_pip" | "ring_dip" | "ring_tip"
  | "pinky_mcp" | "pinky_pip" | "pinky_dip" | "pinky_tip";

interface FacePose {
  // 468 landmarks (MediaPipe standard) or subset
  landmarks: FaceLandmark[];

  // Blend shape weights
  blend_weights?: Record<string, number>;
}

interface FaceLandmark {
  id: number;
  position: Vector3;
}

interface BlendShapeTrack {
  name: string;  // e.g., "browInnerUp", "mouthSmileLeft"
  keyframes: BlendShapeKeyframe[];
}

interface BlendShapeKeyframe {
  timestamp_ms: number;
  value: number;  // 0.0 - 1.0
}
```

### 3.6 RenderOutput

최종 렌더링 출력 정보

```typescript
interface RenderOutput {
  // Reference
  pose_sequence_id: string;

  // Output metadata
  render_id: string;
  timestamp: string;

  // Output format
  format: OutputFormat;

  // Video/animation properties
  resolution: Resolution;
  frame_rate: number;
  duration_ms: number;

  // File info
  file_size_bytes?: number;
  data_url?: string;

  // Avatar info
  avatar?: AvatarInfo;

  // Quality metrics
  quality_score?: number;
}

interface OutputFormat {
  type: OutputType;
  codec?: string;
  container?: string;
}

type OutputType =
  | "video"      // Pre-rendered video
  | "animation"  // Animation data (glTF, BVH)
  | "streaming"; // Real-time stream

interface Resolution {
  width: number;
  height: number;
}

interface AvatarInfo {
  avatar_id: string;
  name: string;
  style: AvatarStyle;
}

type AvatarStyle =
  | "realistic"
  | "stylized"
  | "cartoon"
  | "minimal";
```

---

## 4. Request/Response Formats

### 4.1 Translation Request

```typescript
interface TranslationRequest {
  // Request ID
  request_id: string;

  // Input (one of)
  audio?: AudioInput;
  text?: TextInput;

  // Target sign language
  target_language: SignLanguageCode;

  // Output preferences
  output: OutputPreferences;

  // Processing options
  options?: TranslationOptions;
}

interface TextInput {
  text: string;
  language: string;  // ISO 639-1
}

interface OutputPreferences {
  // What to return
  include_transcript: boolean;
  include_gloss: boolean;
  include_notation: boolean;
  include_pose: boolean;
  include_render: boolean;

  // Render settings (if include_render)
  render_settings?: RenderSettings;
}

interface RenderSettings {
  format: OutputType;
  resolution: Resolution;
  frame_rate: number;
  avatar_id?: string;
}

interface TranslationOptions {
  // ASR options
  asr_model?: string;
  language_detection?: boolean;

  // Translation options
  translation_model?: string;
  formality_level?: "formal" | "informal" | "neutral";

  // Real-time options
  streaming?: boolean;
  chunk_duration_ms?: number;
}
```

### 4.2 Translation Response

```typescript
interface TranslationResponse {
  // Request reference
  request_id: string;

  // Response metadata
  response_id: string;
  timestamp: string;

  // Status
  status: ResponseStatus;
  error?: ErrorInfo;

  // Results (based on request preferences)
  transcript?: TranscriptionResult;
  gloss?: SignGlossSequence;
  notation?: SignNotation[];
  pose?: SignPoseSequence;
  render?: RenderOutput;

  // Processing summary
  processing: ProcessingSummary;
}

type ResponseStatus =
  | "success"
  | "partial"    // Some components failed
  | "error";

interface ErrorInfo {
  code: string;
  message: string;
  details?: Record<string, any>;
}

interface ProcessingSummary {
  total_time_ms: number;

  asr_time_ms?: number;
  translation_time_ms?: number;
  pose_generation_time_ms?: number;
  render_time_ms?: number;
}
```

---

## 5. Streaming Formats

### 5.1 WebSocket Protocol

```typescript
// Client -> Server
interface StreamingRequest {
  type: "start" | "audio_chunk" | "stop";

  // For "start"
  config?: StreamingConfig;

  // For "audio_chunk"
  audio_data?: string;  // base64
  sequence_number?: number;
}

interface StreamingConfig {
  target_language: SignLanguageCode;
  output_preferences: OutputPreferences;

  // Audio format
  audio_format: AudioFormat;
  sample_rate: number;
}

// Server -> Client
interface StreamingResponse {
  type: "transcript" | "gloss" | "pose" | "render" | "error" | "end";

  // Partial results
  transcript_delta?: TranscriptDelta;
  gloss_delta?: GlossDelta;
  pose_frame?: PoseFrame;
  render_frame?: RenderFrame;

  // Error
  error?: ErrorInfo;

  // Timing
  latency_ms: number;
}

interface TranscriptDelta {
  text: string;
  is_final: boolean;
  confidence: number;
}

interface GlossDelta {
  gloss: SignGloss;
  is_final: boolean;
}

interface RenderFrame {
  frame_index: number;
  data: string;  // base64 encoded frame
}
```

---

## 6. Common Types

### 6.1 Geographic Location

```typescript
interface GeoLocation {
  latitude: number;
  longitude: number;
  accuracy_m?: number;
}
```

### 6.2 Time Formats

모든 타임스탬프는 ISO 8601 형식을 사용합니다:
- `2025-01-15T10:30:00Z` (UTC)
- `2025-01-15T19:30:00+09:00` (with timezone)

밀리초 단위 시간은 정수로 표현합니다:
- `start_ms: 1500` (1.5초)

---

## 7. Schema Validation Rules

### 7.1 Required Fields

각 타입의 필수 필드:

| Type | Required Fields |
|------|-----------------|
| AudioInput | audio_id, timestamp, format, sample_rate, channels, duration_ms |
| TranscriptionResult | audio_id, transcription_id, language, text, words |
| SignGlossSequence | transcription_id, gloss_id, sign_language, glosses |
| SignGloss | gloss, start_ms, end_ms, sign_type, confidence |
| SignPoseSequence | gloss_id, pose_sequence_id, frame_rate, frames |

### 7.2 Value Constraints

```yaml
Confidence values:
  min: 0.0
  max: 1.0

Sample rates (Hz):
  allowed: [8000, 16000, 22050, 44100, 48000]

Frame rates (FPS):
  allowed: [24, 25, 30, 50, 60]

Duration:
  min_ms: 0
  max_ms: 3600000  # 1 hour

Coordinates:
  position: -10.0 to 10.0 meters
  rotation: -1.0 to 1.0 (quaternion components)
```

---

## 8. Examples

### 8.1 Simple Translation Flow

**Input (Audio):**
```json
{
  "audio_id": "aud_001",
  "timestamp": "2025-01-15T10:30:00Z",
  "format": "wav",
  "sample_rate": 16000,
  "channels": 1,
  "bit_depth": 16,
  "duration_ms": 2000,
  "data_url": "https://api.example.com/audio/aud_001.wav"
}
```

**Transcription Result:**
```json
{
  "audio_id": "aud_001",
  "transcription_id": "tr_001",
  "timestamp": "2025-01-15T10:30:00.200Z",
  "language": "en",
  "confidence": 0.95,
  "text": "Hello, how are you?",
  "words": [
    {"word": "Hello", "start_ms": 100, "end_ms": 500, "confidence": 0.98},
    {"word": "how", "start_ms": 600, "end_ms": 800, "confidence": 0.95},
    {"word": "are", "start_ms": 850, "end_ms": 950, "confidence": 0.94},
    {"word": "you", "start_ms": 1000, "end_ms": 1200, "confidence": 0.96}
  ],
  "model": "whisper-large-v3",
  "processing_time_ms": 180
}
```

**Gloss Sequence:**
```json
{
  "transcription_id": "tr_001",
  "gloss_id": "gl_001",
  "timestamp": "2025-01-15T10:30:00.400Z",
  "sign_language": "ASL",
  "glosses": [
    {
      "gloss": "HELLO",
      "start_ms": 0,
      "end_ms": 600,
      "duration_ms": 600,
      "sign_type": "lexical",
      "hamnosys": "hamfinger2,hamextfingeru,hampalml",
      "confidence": 0.92
    },
    {
      "gloss": "HOW",
      "start_ms": 650,
      "end_ms": 1000,
      "duration_ms": 350,
      "sign_type": "lexical",
      "confidence": 0.88
    },
    {
      "gloss": "YOU",
      "start_ms": 1050,
      "end_ms": 1400,
      "duration_ms": 350,
      "sign_type": "pointing",
      "spatial_ref": {"x": 0.0, "y": 0.0, "z": 0.5},
      "confidence": 0.95
    }
  ],
  "sentence_type": "interrogative",
  "confidence": 0.91,
  "translation_model": "wia-sign-v1",
  "processing_time_ms": 150
}
```

---

## 9. Versioning

### 9.1 Schema Versioning

스키마 버전은 Semantic Versioning을 따릅니다:
- **MAJOR**: 호환되지 않는 변경
- **MINOR**: 하위 호환 기능 추가
- **PATCH**: 하위 호환 버그 수정

현재 버전: `1.0.0`

### 9.2 Deprecation Policy

- 필드 제거 전 최소 2 minor 버전 동안 deprecated 표시
- Deprecated 필드는 `@deprecated` 주석 추가
- Major 버전 업데이트 시 deprecated 필드 제거 가능

---

## 10. Next Steps

다음 문서에서 JSON Schema로 위 타입들을 정의합니다:
- `audio-input.schema.json`
- `transcription-result.schema.json`
- `sign-gloss.schema.json`
- `sign-notation.schema.json`
- `sign-pose.schema.json`
- `render-output.schema.json`
