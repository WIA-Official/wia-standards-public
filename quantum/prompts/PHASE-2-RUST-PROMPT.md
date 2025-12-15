# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA Quantum  
**Phase**: 2 of 4  
**Language**: **Rust** (Primary)
**목표**: Rust 기반 양자 컴퓨팅 API 구현

---

## 🎯 목표

Rust로 양자 표준 API 구현 (24개 세부 표준)

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (양자 시뮬레이션)
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
│   │   ├── qubit.rs        # 큐비트 상태
│   │   ├── gate.rs         # 양자 게이트
│   │   └── circuit.rs      # 양자 회로
│   ├── adapters/
│   │   ├── mod.rs
│   │   ├── ibm.rs          # IBM Quantum
│   │   ├── google.rs       # Google Cirq
│   │   ├── ionq.rs         # IonQ
│   │   └── simulator.rs    # 로컬 시뮬레이터
│   ├── crypto/
│   │   ├── mod.rs
│   │   ├── qkd.rs          # 양자키분배
│   │   └── qrng.rs         # 양자난수생성
│   └── output/
│       ├── mod.rs
│       └── measurement.rs  # 측정 결과
├── tests/
└── examples/
    ├── basic_circuit.rs
    └── qkd_session.rs
```

---

## 🔧 핵심 구현 코드

### Error 타입 (error.rs)
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum QuantumError {
    #[error("Qubit index out of range: {0}")]
    QubitOutOfRange(usize),
    #[error("Gate error: {0}")]
    GateError(String),
    #[error("Measurement failed: {0}")]
    MeasurementFailed(String),
    #[error("QKD session error: {0}")]
    QkdError(String),
    #[error("Backend connection failed: {0}")]
    BackendError(String),
}

pub type Result<T> = std::result::Result<T, QuantumError>;
```

### 타입 정의 (types.rs)
```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Qubit {
    pub index: usize,
    pub state: QubitState,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum QubitState {
    Zero,
    One,
    Superposition { alpha: Complex, beta: Complex },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Gate {
    H,      // Hadamard
    X,      // Pauli-X
    Y,      // Pauli-Y
    Z,      // Pauli-Z
    CNOT,   // Controlled-NOT
    T,      // T gate
    S,      // S gate
    Custom(Vec<Vec<Complex>>),
}
```

### QuantumAdapter Trait
```rust
use async_trait::async_trait;

#[async_trait]
pub trait QuantumAdapter: Send + Sync {
    async fn connect(&mut self) -> Result<()>;
    async fn execute(&self, circuit: &Circuit) -> Result<MeasurementResult>;
    async fn get_qubit_count(&self) -> Result<usize>;
    fn backend_name(&self) -> &str;
}
```

---

## 📋 Cargo.toml

```toml
[package]
name = "wia-quantum"
version = "1.0.0"
edition = "2021"
license = "MIT"

[dependencies]
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
thiserror = "1"
async-trait = "0.1"
num-complex = "0.4"  # 복소수 연산

[features]
default = []
wasm = ["wasm-bindgen"]
python = ["pyo3"]
```

---

## 🎨 UI/CSS 이모지 분리 패턴

```html
<h1>
    <span class="emoji">⚛️</span>
    <span class="gradient-text">WIA Quantum</span>
</h1>
```

```css
.emoji { background: none !important; -webkit-text-fill-color: initial !important; }
.gradient-text { background: linear-gradient(90deg, #00d4ff, #7b2cbf); -webkit-background-clip: text; }
```

---

## 🔗 연동 표준

- **WIA Security**: 양자내성암호 연동
- **WIA AI**: 양자머신러닝 연동

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성
□ Error 타입 정의
□ Qubit/Gate/Circuit 타입
□ QuantumAdapter trait
□ 4개 백엔드 어댑터 (IBM, Google, IonQ, Simulator)
□ QKD 모듈
□ 측정 결과 처리
□ 테스트 작성
□ 예제 코드 작성
□ README 업데이트
```

---

弘益人間 🤟🦀⚛️
