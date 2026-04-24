# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA BCI (Brain-Computer Interface)
**Phase**: 2 of 4
**Language**: **Rust** (Primary)
**목표**: Rust 기반 고성능 BCI API 구현

---

## 🎯 목표

Rust로 BCI 표준 API 구현 (TypeScript/Python은 바인딩으로 제공)

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (실시간 뇌파 처리)
2. 안전: 메모리 안전 보장 (의료 등급)
3. 현대적: 최신 언어 기능
4. 일관성: WIA AAC/Braille API와 동일 스택
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
│   │   ├── device.rs       # 기기 인터페이스
│   │   └── signal.rs       # 신호 처리
│   ├── adapters/
│   │   ├── mod.rs
│   │   ├── openbci.rs      # OpenBCI
│   │   ├── emotiv.rs       # Emotiv EPOC
│   │   ├── muse.rs         # Muse headband
│   │   ├── neurable.rs     # Neurable
│   │   ├── lsl.rs          # Lab Streaming Layer
│   │   └── simulator.rs    # 테스트용 시뮬레이터
│   ├── processing/
│   │   ├── mod.rs
│   │   ├── filter.rs       # 필터 (고역/저역/대역)
│   │   ├── fft.rs          # FFT 분석
│   │   └── features.rs     # 특징 추출
│   └── output/
│       ├── mod.rs
│       ├── tts.rs          # 음성 출력
│       ├── isp.rs          # 수어 변환 (ISP)
│       └── braille.rs      # 점자 변환
├── tests/
│   └── integration_test.rs
└── examples/
    ├── basic_usage.rs
    └── motor_imagery.rs
```

---

## 🔧 핵심 구현 코드

### Error 타입 (error.rs)
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum BciError {
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    #[error("Stream error: {0}")]
    StreamError(String),

    #[error("Signal processing error: {0}")]
    ProcessingError(String),

    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),

    #[error("Output adapter error: {0}")]
    OutputError(String),
}

pub type Result<T> = std::result::Result<T, BciError>;
```

### 기기 타입 (types.rs)
```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceType {
    EegHeadset,
    EegCap,
    ImplantCortical,
    Fnirs,
    Hybrid,
    Simulator,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalEvent {
    pub timestamp: u64,
    pub sample_index: u64,
    pub channels: Vec<u32>,
    pub data: Vec<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BandPowers {
    pub delta: f64,   // 0.5-4 Hz
    pub theta: f64,   // 4-8 Hz
    pub alpha: f64,   // 8-13 Hz
    pub beta: f64,    // 13-30 Hz
    pub gamma: f64,   // 30-100 Hz
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Classification {
    pub class_id: u32,
    pub class_name: String,
    pub confidence: f32,
}
```

### BCI Adapter Trait (core/device.rs)
```rust
use async_trait::async_trait;
use crate::{Result, DeviceType, SignalEvent};

#[async_trait]
pub trait BciAdapter: Send + Sync {
    fn device_type(&self) -> DeviceType;

    async fn connect(&mut self, config: &DeviceConfig) -> Result<()>;
    async fn disconnect(&mut self) -> Result<()>;
    async fn start_stream(&mut self) -> Result<()>;
    async fn stop_stream(&mut self) -> Result<()>;

    fn is_connected(&self) -> bool;
    fn is_streaming(&self) -> bool;

    fn on_data(&mut self, handler: Box<dyn Fn(SignalEvent) + Send + Sync>);
}
```

### Signal Processor (processing/filter.rs)
```rust
use crate::{BandPowers, Result};

pub struct SignalProcessor;

impl SignalProcessor {
    pub fn highpass(data: &[f32], cutoff: f64, fs: f64) -> Vec<f32> { ... }
    pub fn lowpass(data: &[f32], cutoff: f64, fs: f64) -> Vec<f32> { ... }
    pub fn bandpass(data: &[f32], low: f64, high: f64, fs: f64) -> Vec<f32> { ... }
    pub fn notch(data: &[f32], freq: f64, fs: f64) -> Vec<f32> { ... }

    pub fn bandpower(data: &[f32], fs: f64, band: (f64, f64)) -> f64 { ... }
    pub fn all_band_powers(data: &[f32], fs: f64) -> BandPowers { ... }
}
```

### Output Adapter Trait (output/mod.rs)
```rust
use async_trait::async_trait;
use crate::Result;

#[async_trait]
pub trait OutputAdapter: Send + Sync {
    async fn speak(&self, text: &str) -> Result<()>;
    async fn to_sign(&self, text: &str) -> Result<String>;  // ISP 코드 반환
    async fn to_braille(&self, text: &str) -> Result<String>;
}
```

### 메인 BCI 구조체 (lib.rs)
```rust
use std::sync::Arc;
use tokio::sync::RwLock;

pub struct WiaBci {
    adapter: Option<Arc<RwLock<dyn BciAdapter>>>,
    output: Arc<dyn OutputAdapter>,
    config: BciConfig,
    state: BciState,
}

impl WiaBci {
    pub fn new(config: BciConfig) -> Self {
        Self {
            adapter: None,
            output: Arc::new(DefaultOutput::new()),
            config,
            state: BciState::default(),
        }
    }

    pub async fn connect(&mut self, device_config: DeviceConfig) -> Result<()> {
        let adapter = self.create_adapter(&device_config)?;
        adapter.write().await.connect(&device_config).await?;
        self.adapter = Some(adapter);
        self.state.connected = true;
        Ok(())
    }

    pub async fn start_stream(&mut self) -> Result<()> {
        if let Some(adapter) = &self.adapter {
            adapter.write().await.start_stream().await?;
            self.state.streaming = true;
        }
        Ok(())
    }

    pub async fn on_classification<F>(&self, handler: F)
    where
        F: Fn(Classification) + Send + Sync + 'static,
    {
        // Classification 결과를 output으로 전달
    }

    pub async fn communicate(&self, text: &str) -> Result<()> {
        // TTS + ISP + Braille 동시 출력
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
name = "wia-bci"
version = "1.0.0"
edition = "2021"
description = "WIA BCI Standard - Rust SDK"
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
    <span class="emoji">🧠</span>
    <span class="gradient-text">WIA BCI</span>
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
use wia_bci::{WiaBci, BciConfig, DeviceConfig, DeviceType};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = BciConfig::default();
    let mut bci = WiaBci::new(config);

    // 기기 연결
    bci.connect(DeviceConfig {
        device_type: DeviceType::EegHeadset,
        manufacturer: Some("OpenBCI".into()),
        sampling_rate: 250,
        ..Default::default()
    }).await?;

    // 분류 결과 핸들러
    bci.on_classification(|result| {
        println!("Class: {} ({}%)", result.class_name, result.confidence * 100.0);
    }).await;

    // 스트리밍 시작
    bci.start_stream().await?;

    // 결과를 음성/수어/점자로 출력
    bci.on_classification(|result| {
        bci.communicate(&result.class_name).await.ok();
    }).await;

    Ok(())
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
/api/rust/src/core/device.rs
/api/rust/src/core/signal.rs
/api/rust/src/adapters/*.rs (6개 어댑터)
/api/rust/src/processing/*.rs (필터, FFT, 특징추출)
/api/rust/src/output/*.rs (3개 출력)
/api/rust/tests/*.rs
/api/rust/examples/*.rs
```

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성
□ Error 타입 정의
□ 기기/신호 타입 정의
□ BciAdapter trait 구현
□ OutputAdapter trait 구현
□ 6개 기기 어댑터 구현
  □ OpenBCI
  □ Emotiv
  □ Muse
  □ Neurable
  □ LSL
  □ Simulator
□ 신호 처리 구현
  □ 필터 (고역/저역/대역/노치)
  □ FFT / 밴드파워
  □ 특징 추출
□ 3개 출력 어댑터 구현
  □ TTS
  □ ISP (수어)
  □ Braille (점자)
□ WiaBci 메인 구조체
□ 테스트 작성
□ 예제 코드 작성
□ README 업데이트
```

---

## 🔗 연동 표준

- **WIA AAC**: BCI를 AAC 입력 소스로 활용
- **ISP**: output/isp.rs에서 수어 변환
- **WIA Braille**: output/braille.rs에서 점자 변환
- **WIA Talk**: 통합 커뮤니케이션 프로토콜

---

弘益人間 🤟🦀
