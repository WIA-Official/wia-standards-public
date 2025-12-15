# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA AAC (Augmentative & Alternative Communication)
**Phase**: 2 of 4  
**Language**: **Rust** (Primary)
**목표**: Rust 기반 고성능 AAC API 구현

---

## 🎯 목표

Rust로 AAC 표준 API 구현 (TypeScript/Python은 바인딩으로 제공)

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (실시간 센서 처리)
2. 안전: 메모리 안전 보장
3. 현대적: 최신 언어 기능
4. 일관성: WIA Braille API와 동일 스택
```

---

## 📦 프로젝트 구조

```
/api/rust/
├── Cargo.toml
├── src/
│   ├── lib.rs              # 메인 라이브러리
│   ├── types.rs            # 타입 정의
│   ├── error.rs            # 에러 타입
│   ├── core/
│   │   ├── mod.rs
│   │   ├── sensor.rs       # 센서 인터페이스
│   │   └── signal.rs       # 신호 처리
│   ├── adapters/
│   │   ├── mod.rs
│   │   ├── eye_tracker.rs  # 시선 추적
│   │   ├── switch.rs       # 스위치
│   │   ├── head.rs         # 머리 움직임
│   │   ├── muscle.rs       # 근육 센서
│   │   ├── brain.rs        # 뇌파 (BCI 연동)
│   │   └── breath.rs       # 호흡
│   └── output/
│       ├── mod.rs
│       ├── tts.rs          # 음성 출력
│       ├── isp.rs          # 수어 변환 (ISP)
│       └── braille.rs      # 점자 변환
├── tests/
│   └── integration_test.rs
└── examples/
    ├── basic_usage.rs
    └── eye_tracker_demo.rs
```

---

## 🔧 핵심 구현 코드

### Error 타입 (error.rs)
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum AacError {
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),
    
    #[error("Sensor not found: {0}")]
    SensorNotFound(String),
    
    #[error("Signal processing error: {0}")]
    SignalError(String),
    
    #[error("Calibration required")]
    CalibrationRequired,
    
    #[error("Output adapter error: {0}")]
    OutputError(String),
}

pub type Result<T> = std::result::Result<T, AacError>;
```

### 센서 타입 (types.rs)
```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorType {
    EyeTracker,
    Switch,
    HeadMovement,
    MuscleSensor,
    BrainInterface,
    Breath,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorData {
    pub sensor_type: SensorType,
    pub timestamp: u64,
    pub values: Vec<f64>,
    pub confidence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AacIntent {
    pub action: String,
    pub confidence: f32,
    pub source: SensorType,
}
```

### Sensor Adapter Trait (core/sensor.rs)
```rust
use async_trait::async_trait;
use crate::{Result, SensorData, SensorType};

#[async_trait]
pub trait SensorAdapter: Send + Sync {
    fn sensor_type(&self) -> SensorType;
    
    async fn connect(&mut self) -> Result<()>;
    async fn disconnect(&mut self) -> Result<()>;
    async fn read(&self) -> Result<SensorData>;
    async fn calibrate(&mut self) -> Result<()>;
    
    fn is_connected(&self) -> bool;
}
```

### Output Adapter Trait (output/mod.rs)
```rust
use async_trait::async_trait;
use crate::{Result, AacIntent};

#[async_trait]
pub trait OutputAdapter: Send + Sync {
    async fn speak(&self, text: &str) -> Result<()>;
    async fn to_sign(&self, text: &str) -> Result<String>;  // ISP 코드 반환
    async fn to_braille(&self, text: &str) -> Result<String>;
}
```

### 메인 AAC 구조체 (lib.rs)
```rust
use std::sync::Arc;
use tokio::sync::RwLock;

pub struct WiaAac {
    sensors: Vec<Arc<RwLock<dyn SensorAdapter>>>,
    output: Arc<dyn OutputAdapter>,
    config: AacConfig,
}

impl WiaAac {
    pub fn new(config: AacConfig) -> Self {
        Self {
            sensors: Vec::new(),
            output: Arc::new(DefaultOutput::new()),
            config,
        }
    }
    
    pub async fn add_sensor<S: SensorAdapter + 'static>(&mut self, sensor: S) {
        self.sensors.push(Arc::new(RwLock::new(sensor)));
    }
    
    pub async fn process(&self) -> Result<AacIntent> {
        // 모든 센서에서 데이터 수집
        let mut best_intent: Option<AacIntent> = None;
        
        for sensor in &self.sensors {
            let s = sensor.read().await;
            if s.is_connected() {
                let data = s.read().await?;
                let intent = self.interpret(&data)?;
                
                if best_intent.is_none() || intent.confidence > best_intent.as_ref().unwrap().confidence {
                    best_intent = Some(intent);
                }
            }
        }
        
        best_intent.ok_or(AacError::SensorNotFound("No active sensors".into()))
    }
    
    pub async fn communicate(&self, intent: &AacIntent) -> Result<()> {
        // TTS + ISP + Braille 동시 출력
        let text = &intent.action;
        
        tokio::try_join!(
            self.output.speak(text),
            async {
                let isp = self.output.to_sign(text).await?;
                println!("ISP: {}", isp);
                Ok(())
            },
            async {
                let braille = self.output.to_braille(text).await?;
                println!("Braille: {}", braille);
                Ok(())
            }
        )?;
        
        Ok(())
    }
}
```

---

## 📋 Cargo.toml

```toml
[package]
name = "wia-aac"
version = "1.0.0"
edition = "2021"
description = "WIA AAC Standard - Rust SDK"
license = "MIT"
repository = "https://github.com/WIA-Official/wia-standards"

[dependencies]
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
thiserror = "1"
async-trait = "0.1"
tracing = "0.1"

# WebAssembly 지원
wasm-bindgen = { version = "0.2", optional = true }

# Python 바인딩
pyo3 = { version = "0.20", optional = true }

[features]
default = []
wasm = ["wasm-bindgen"]
python = ["pyo3"]

[dev-dependencies]
tokio-test = "0.4"
```

---

## 🎨 UI/CSS 이모지 분리 패턴

**중요**: 사이트 구현 시 이모지와 그라데이션 텍스트 분리 필수!

```html
<!-- ✅ 올바른 방식 -->
<h1>
    <span class="emoji">♿</span>
    <span class="gradient-text">WIA AAC</span>
</h1>
```

```css
.gradient-text {
    background: linear-gradient(90deg, #00d4ff, #7b2cbf);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
}

.emoji {
    background: none !important;
    -webkit-background-clip: initial !important;
    -webkit-text-fill-color: initial !important;
}
```

---

## 🚀 사용 예시

### Basic Usage
```rust
use wia_aac::{WiaAac, AacConfig};
use wia_aac::adapters::EyeTrackerAdapter;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = AacConfig::default();
    let mut aac = WiaAac::new(config);
    
    // 시선 추적기 추가
    let eye_tracker = EyeTrackerAdapter::new("tobii://localhost");
    aac.add_sensor(eye_tracker).await;
    
    // 센서 연결
    aac.connect_all().await?;
    
    // 의도 처리 및 출력
    loop {
        let intent = aac.process().await?;
        println!("Intent: {:?}", intent);
        
        aac.communicate(&intent).await?;
    }
}
```

---

## 📁 산출물

```
/api/rust/Cargo.toml
/api/rust/src/lib.rs
/api/rust/src/types.rs
/api/rust/src/error.rs
/api/rust/src/core/mod.rs
/api/rust/src/core/sensor.rs
/api/rust/src/core/signal.rs
/api/rust/src/adapters/*.rs (6개 센서)
/api/rust/src/output/*.rs (3개 출력)
/api/rust/tests/*.rs
/api/rust/examples/*.rs
```

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성
□ Error 타입 정의
□ 센서 타입 정의
□ SensorAdapter trait 구현
□ OutputAdapter trait 구현
□ 6개 센서 어댑터 구현
  □ EyeTracker
  □ Switch
  □ HeadMovement
  □ MuscleSensor
  □ BrainInterface
  □ Breath
□ 3개 출력 어댑터 구현
  □ TTS
  □ ISP (수어)
  □ Braille (점자)
□ WiaAac 메인 구조체
□ 테스트 작성
□ 예제 코드 작성
□ README 업데이트
```

---

## 🔗 연동 표준

- **WIA BCI**: brain.rs에서 BCI 표준 연동
- **ISP**: output/isp.rs에서 수어 변환
- **WIA Braille**: output/braille.rs에서 점자 변환
- **WIA Talk**: 통합 커뮤니케이션 프로토콜

---

弘益人間 🤟🦀
