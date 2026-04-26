# 🗺️ WIA PubScript Implementation Roadmap v3.1

## 개요 (Overview)

이 문서는 WIA PubScript 엔진의 단계별 구현 로드맵입니다.

**핵심 철학**: Visual, Auditory, Tactile, Spatial, Gestural - **모두 동등, 기본값 없음**

---

## Phase 1: Foundation (1-2일) 🏗️

### 목표
- Rust 프로젝트 구조 확립
- IR v3.0 스키마 완전 구현
- 기본 테스트 작성

### 1.1 프로젝트 초기화

```bash
cd pubscript
cargo init --name wia_pubscript
```

**디렉토리 구조**:
```
pubscript/
├── Cargo.toml
├── src/
│   ├── lib.rs              # 라이브러리 루트
│   ├── ir/                 # IR v3.0 정의
│   │   ├── mod.rs
│   │   ├── document.rs     # PubScriptDocument
│   │   ├── node.rs         # ContentNode
│   │   ├── representations/
│   │   │   ├── mod.rs
│   │   │   ├── visual.rs   # VisualRep
│   │   │   ├── auditory.rs # AuditoryRep
│   │   │   ├── tactile.rs  # TactileRep
│   │   │   ├── spatial.rs  # SpatialRep
│   │   │   └── gestural.rs # GesturalRep
│   │   └── metadata.rs     # Metadata, NodeMetadata
│   └── main.rs             # CLI 엔트리포인트
├── tests/
│   ├── integration_test.rs
│   └── fixtures/
│       └── simple_text.txt
└── examples/
    └── hello_world.rs
```

### 1.2 IR v3.0 스키마 구현

#### Core Types (`src/ir/document.rs`)

```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PubScriptDocument {
    /// 문서 메타데이터
    pub metadata: Metadata,

    /// 콘텐츠 노드 트리
    pub content: Vec<ContentNode>,

    /// 타임라인 (옵션)
    pub timeline: Option<Timeline>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metadata {
    pub title: Option<String>,
    pub author: Option<String>,
    pub language: Option<String>,
    pub version: String, // IR 버전 (e.g., "3.0")
    pub created_at: Option<String>,
    pub updated_at: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timeline {
    pub duration: f64, // 초 단위
    pub events: Vec<TimelineEvent>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimelineEvent {
    pub timestamp: f64, // 초 단위
    pub node_id: String,
    pub event_type: TimelineEventType,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TimelineEventType {
    Show,
    Hide,
    Play,
    Pause,
    Sync,
}
```

#### Content Node (`src/ir/node.rs`)

```rust
use super::representations::Representations;
use super::metadata::NodeMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentNode {
    /// 고유 ID
    pub id: String,

    /// 다섯 가지 동등한 표현
    pub representations: Representations,

    /// 자식 노드들
    pub children: Vec<ContentNode>,

    /// 메타데이터
    pub metadata: NodeMetadata,
}
```

#### Representations (`src/ir/representations/mod.rs`)

```rust
use super::visual::VisualRep;
use super::auditory::AuditoryRep;
use super::tactile::TactileRep;
use super::spatial::SpatialRep;
use super::gestural::GesturalRep;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Representations {
    /// Visual representation (시각)
    pub visual: Option<VisualRep>,

    /// Auditory representation (청각)
    pub auditory: Option<AuditoryRep>,

    /// Tactile representation (촉각)
    pub tactile: Option<TactileRep>,

    /// Spatial representation (공간)
    pub spatial: Option<SpatialRep>,

    /// Gestural representation (제스처)
    pub gestural: Option<GesturalRep>,
}

// NO DEFAULT PREFERENCE!
// ALL FIVE ARE EQUAL!
```

#### Visual Representation (`src/ir/representations/visual.rs`)

```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualRep {
    /// 텍스트 콘텐츠
    pub text: Option<String>,

    /// 이미지
    pub image: Option<ImageData>,

    /// 레이아웃
    pub layout: Layout,

    /// 스타일
    pub style: VisualStyle,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageData {
    pub url: String,
    pub alt_text: Option<String>,
    pub width: Option<u32>,
    pub height: Option<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Layout {
    Block,
    Inline,
    Flex,
    Grid,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualStyle {
    pub font_family: Option<String>,
    pub font_size: Option<f32>,
    pub font_weight: Option<FontWeight>,
    pub color: Option<Color>,
    pub background: Option<Color>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FontWeight {
    Normal,
    Bold,
    Light,
    Custom(u16),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}
```

#### Auditory Representation (`src/ir/representations/auditory.rs`)

```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditoryRep {
    /// 음성 합성 텍스트
    pub speech_text: Option<String>,

    /// SSML (Speech Synthesis Markup Language)
    pub ssml: Option<String>,

    /// 오디오 파일
    pub audio_file: Option<String>,

    /// 음향 효과
    pub sound_effects: Vec<SoundEffect>,

    /// 공간 오디오
    pub spatial_audio: Option<SpatialAudio>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoundEffect {
    pub effect_type: SoundEffectType,
    pub volume: f32, // 0.0 ~ 1.0
    pub duration: Option<f64>, // 초 단위
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SoundEffectType {
    Beep,
    Click,
    Whoosh,
    Custom(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialAudio {
    pub position: Vec3,
    pub direction: Vec3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
```

#### Tactile Representation (`src/ir/representations/tactile.rs`)

```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TactileRep {
    /// 점자 텍스트
    pub braille: Option<String>,

    /// 햅틱 패턴
    pub haptic_pattern: Option<HapticPattern>,

    /// 텍스처
    pub texture: Option<Texture>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HapticPattern {
    SectionBoundary,
    Heading,
    Link,
    Custom(Vec<HapticEvent>),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticEvent {
    pub intensity: f32, // 0.0 ~ 1.0
    pub duration: f64,  // 초 단위
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Texture {
    pub texture_type: TextureType,
    pub intensity: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TextureType {
    Smooth,
    Rough,
    Bumpy,
    Custom(String),
}
```

#### Spatial Representation (`src/ir/representations/spatial.rs`)

```rust
use serde::{Deserialize, Serialize};
use super::auditory::Vec3; // 재사용

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialRep {
    /// 3D 위치
    pub position: Vec3,

    /// 회전 (쿼터니언)
    pub rotation: Quat,

    /// 스케일
    pub scale: Vec3,

    /// 바운딩 박스
    pub bounds: BoundingBox,

    /// 공간적 관계
    pub spatial_relations: Vec<SpatialRelation>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialRelation {
    pub relation_type: SpatialRelationType,
    pub target_node_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SpatialRelationType {
    Above,
    Below,
    LeftOf,
    RightOf,
    Near,
    Far,
}
```

#### Gestural Representation (`src/ir/representations/gestural.rs`)

```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GesturalRep {
    /// 지원하는 제스처들
    pub gestures: Vec<Gesture>,

    /// 음성 명령들
    pub voice_commands: Vec<VoiceCommand>,

    /// 인터랙션 힌트
    pub interaction_hints: Vec<InteractionHint>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Gesture {
    Tap,
    DoubleTap,
    LongPress,
    Swipe(SwipeDirection),
    Pinch,
    Rotate,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SwipeDirection {
    Up,
    Down,
    Left,
    Right,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommand {
    pub phrase: String,
    pub action: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InteractionHint {
    pub hint_type: InteractionHintType,
    pub description: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InteractionHintType {
    Tooltip,
    Announcement,
    Guidance,
}
```

### 1.3 간단한 텍스트 → IR 변환 테스트

#### 목표
평문 텍스트를 IR로 변환하는 간단한 예제 작성

**예시 입력** (`tests/fixtures/simple_text.txt`):
```
Hello, World!

This is a simple paragraph.
```

**예시 출력** (IR JSON):
```json
{
  "metadata": {
    "title": "Simple Document",
    "version": "3.0"
  },
  "content": [
    {
      "id": "node-1",
      "representations": {
        "visual": {
          "text": "Hello, World!",
          "layout": "Block",
          "style": {}
        },
        "auditory": {
          "speech_text": "Hello, World!"
        },
        "tactile": {
          "braille": "⠠⠓⠑⠇⠇⠕⠂ ⠠⠺⠕⠗⠇⠙⠖"
        }
      },
      "children": [],
      "metadata": {}
    }
  ]
}
```

#### 테스트 코드 (`tests/integration_test.rs`)

```rust
use wia_pubscript::ir::*;

#[test]
fn test_simple_text_to_ir() {
    let text = "Hello, World!";

    let doc = PubScriptDocument {
        metadata: Metadata {
            title: Some("Simple Document".to_string()),
            version: "3.0".to_string(),
            ..Default::default()
        },
        content: vec![
            ContentNode {
                id: "node-1".to_string(),
                representations: Representations {
                    visual: Some(VisualRep {
                        text: Some(text.to_string()),
                        layout: Layout::Block,
                        ..Default::default()
                    }),
                    auditory: Some(AuditoryRep {
                        speech_text: Some(text.to_string()),
                        ..Default::default()
                    }),
                    tactile: Some(TactileRep {
                        braille: Some("⠠⠓⠑⠇⠇⠕⠂ ⠠⠺⠕⠗⠇⠙⠖".to_string()),
                        ..Default::default()
                    }),
                    ..Default::default()
                },
                children: vec![],
                metadata: NodeMetadata::default(),
            }
        ],
        timeline: None,
    };

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&doc).unwrap();
    println!("{}", json);

    // Deserialize back
    let parsed: PubScriptDocument = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed.metadata.version, "3.0");
}
```

### 1.4 Cargo.toml 설정

```toml
[package]
name = "wia_pubscript"
version = "0.1.0"
edition = "2021"
authors = ["WIA Team"]
description = "Multi-sensory publishing engine with equal representations"
license = "MIT"

[dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"

[dev-dependencies]
# 테스트용

[lib]
name = "wia_pubscript"
path = "src/lib.rs"

[[bin]]
name = "wia_pubscript"
path = "src/main.rs"
```

---

## Phase 2: Core Engine (3-5일) ⚙️

### 2.1 Parser 구현

**목표**: Markdown, HTML 등을 IR로 변환

```rust
// src/parser/mod.rs
pub trait Parser {
    fn parse(&self, input: &str) -> Result<PubScriptDocument, ParserError>;
}

pub struct MarkdownParser;
pub struct HtmlParser;
```

**우선순위**:
1. Markdown → IR (CommonMark 기준)
2. HTML → IR (semantic HTML 우선)

### 2.2 Renderer 구현

**목표**: IR → 다양한 출력 포맷

```rust
// src/renderer/mod.rs
pub trait Renderer {
    fn render(&self, doc: &PubScriptDocument) -> Result<String, RenderError>;
}

pub struct HtmlRenderer;    // Visual 출력
pub struct SsmlRenderer;    // Auditory 출력
pub struct BrfRenderer;     // Tactile 출력 (Braille Ready Format)
pub struct VrmlRenderer;    // Spatial 출력
```

### 2.3 Transformer 구현

**목표**: IR 변환 및 최적화

```rust
// src/transformer/mod.rs
pub trait Transformer {
    fn transform(&self, doc: &mut PubScriptDocument) -> Result<(), TransformError>;
}

pub struct TextToBrailleTransformer;
pub struct TextToSpeechTransformer;
pub struct LayoutToSpatialTransformer;
```

---

## Phase 3: Advanced Features (5-7일) 🚀

### 3.1 타임라인 동기화

**목표**: 여러 표현을 시간 기반으로 동기화

- Video + Audio + Haptic 동시 재생
- 이벤트 기반 트리거

### 3.2 플러그인 시스템

**목표**: 커스텀 Parser, Renderer, Transformer 추가 가능

```rust
// src/plugin/mod.rs
pub trait Plugin {
    fn name(&self) -> &str;
    fn version(&self) -> &str;
    fn init(&mut self) -> Result<(), PluginError>;
}
```

### 3.3 CLI 도구

**목표**: 커맨드라인에서 사용 가능

```bash
# Markdown → IR
wia_pubscript parse input.md -o output.json

# IR → HTML
wia_pubscript render output.json --format html -o output.html

# IR → SSML
wia_pubscript render output.json --format ssml -o output.ssml

# IR → BRF (Braille)
wia_pubscript render output.json --format brf -o output.brf
```

---

## Phase 4: Integration (7-10일) 🔌

### 4.1 Python 바인딩 (PyO3)

**목표**: Python에서 사용 가능

```python
from wia_pubscript import PubScriptDocument, MarkdownParser

parser = MarkdownParser()
doc = parser.parse("# Hello\n\nWorld!")

print(doc.to_json())
```

### 4.2 WASM 빌드

**목표**: 웹 브라우저에서 사용 가능

```javascript
import init, { parse_markdown } from './wia_pubscript.js';

await init();
const doc = parse_markdown('# Hello\n\nWorld!');
console.log(doc);
```

### 4.3 REST API 서버

**목표**: HTTP API로 제공

```bash
POST /api/parse
Content-Type: text/markdown

# Hello World

---

Response:
{
  "metadata": {...},
  "content": [...]
}
```

---

## 개발 가이드라인 (Development Guidelines)

### 코드 스타일

```bash
# 포맷팅
cargo fmt

# 린팅
cargo clippy

# 테스트
cargo test

# 벤치마크
cargo bench
```

### 테스트 전략

1. **단위 테스트**: 각 모듈별 (`#[cfg(test)]`)
2. **통합 테스트**: `tests/` 디렉토리
3. **예제**: `examples/` 디렉토리
4. **문서 테스트**: docstring examples

### 문서화

- 모든 public API에 rustdoc 주석
- 예제 코드 포함
- 철학 명시: "ALL FIVE ARE EQUAL"

---

## 마일스톤 (Milestones)

### M1: Foundation Complete ✅
- [x] Rust 프로젝트 생성
- [x] IR v3.0 스키마 완전 구현
- [x] 간단한 텍스트 → IR 변환 테스트

### M2: Core Engine Complete
- [ ] Markdown Parser
- [ ] HTML Parser
- [ ] HTML Renderer
- [ ] SSML Renderer
- [ ] BRF Renderer

### M3: Advanced Features Complete
- [ ] 타임라인 동기화
- [ ] 플러그인 시스템
- [ ] CLI 도구

### M4: Integration Complete
- [ ] Python 바인딩
- [ ] WASM 빌드
- [ ] REST API 서버

---

## 다음 단계 (Next Steps)

### 지금 당장 (Phase 1)

1. ✅ `cargo init` 실행
2. ✅ `src/ir/` 디렉토리 구조 생성
3. ✅ IR v3.0 타입 정의
4. ✅ 간단한 테스트 작성
5. ✅ `cargo test` 실행하여 검증

### 내일 (Phase 2 시작)

1. Markdown Parser 구현 시작
2. `pulldown-cmark` 크레이트 사용
3. Markdown AST → IR 변환 로직

---

## 리소스 (Resources)

### Rust 크레이트

- **serde**: Serialization/Deserialization
- **serde_json**: JSON support
- **pulldown-cmark**: Markdown parsing
- **scraper**: HTML parsing
- **pyo3**: Python bindings
- **wasm-bindgen**: WASM support
- **actix-web**: REST API server

### 참고 문서

- [Rust Book](https://doc.rust-lang.org/book/)
- [Serde Documentation](https://serde.rs/)
- [PyO3 Guide](https://pyo3.rs/)
- [WASM Book](https://rustwasm.github.io/book/)

---

## 철학 재확인 (Philosophy Reconfirmation)

> "장애인을 위한 기술"이 아니라 "모든 인간을 위한 선택지"
>
> Visual, Auditory, Tactile, Spatial, Gestural
>
> **모두 동등합니다. 기본값은 없습니다.**

---

🗺️ **Let's start building!** 🚀
