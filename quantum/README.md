# WIA Quantum Standard

**Quantum Computing Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20QUANTUM-orange.svg)](https://quantum.wia.live)

---

<div align="center">

⚛️ **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Quantum은 양자 컴퓨팅 생태계를 위한 개방형 표준입니다.

This standard aims to:
- Unify data formats for quantum circuits, jobs, and results
- Standardize post-quantum cryptography (PQC) and QKD data
- Enable interoperability between quantum platforms
- Support quantum sensors and networks

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format | ✅ Complete |
| **2** | API Interface | Rust SDK for developers | ✅ Complete |
| **3** | Communication Protocol | Backend communication | ✅ Complete |
| **4** | Ecosystem Integration | WIA integration | ✅ Complete |

---

## 📖 Phase 1: Data Format Standard

WIA Quantum Data Format은 양자 컴퓨팅 데이터의 저장, 전송, 교환을 위한 통합 표준입니다.

### Key Documents

| Document | Description |
|----------|-------------|
| [Research Report](spec/RESEARCH-PHASE-1.md) | 양자 컴퓨팅 기술 조사 보고서 |
| [Data Format Spec](spec/PHASE-1-DATA-FORMAT.md) | 데이터 형식 명세서 |

### JSON Schemas

| Schema | Description |
|--------|-------------|
| [circuit.schema.json](spec/schemas/circuit.schema.json) | Quantum circuits |
| [job.schema.json](spec/schemas/job.schema.json) | Execution jobs |
| [result.schema.json](spec/schemas/result.schema.json) | Execution results |
| [state.schema.json](spec/schemas/state.schema.json) | Quantum states |
| [pqc-key.schema.json](spec/schemas/pqc-key.schema.json) | PQC keys |
| [qkd-session.schema.json](spec/schemas/qkd-session.schema.json) | QKD sessions |
| [atomic-clock.schema.json](spec/schemas/atomic-clock.schema.json) | Atomic clocks |
| [magnetometer.schema.json](spec/schemas/magnetometer.schema.json) | Magnetometers |
| [node.schema.json](spec/schemas/node.schema.json) | Quantum network nodes |
| [entanglement.schema.json](spec/schemas/entanglement.schema.json) | Entanglement resources |

### Supported Data Types

```
WiaQuantum
├── Circuit         # 양자 회로 (OpenQASM 3 호환)
├── Job             # 실행 작업
├── Result          # 실행 결과
├── State           # 양자 상태 (상태벡터, 밀도행렬)
├── Crypto          # 암호화
│   ├── PqcKey      # PQC 키 (ML-KEM, ML-DSA, SLH-DSA)
│   └── QkdSession  # QKD 세션 (BB84, E91)
├── Sensor          # 양자 센서
│   ├── AtomicClock # 원자시계
│   └── Magnetometer # 자력계
└── Network         # 양자 네트워크
    ├── Node        # 노드
    └── Entanglement # 얽힘 자원
```

---

## 📖 Phase 2: Rust SDK

WIA Quantum Rust SDK는 양자 회로 구성, 시뮬레이션, PQC 암호화를 위한 고성능 API입니다.

### Installation

```toml
[dependencies]
wia-quantum = "1.0.0"
```

### Rust SDK Example

```rust
use wia_quantum::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create Bell state circuit
    let mut circuit = QuantumCircuit::new(2, 2);
    circuit.h(0)?;
    circuit.cx(0, 1)?;
    circuit.measure_all()?;

    // Run on simulator
    let backend = SimulatorBackend::new();
    let result = backend.run_default(&circuit).await?;

    println!("Counts: {:?}", result.counts);
    // Output: {"00": 512, "11": 512}
    Ok(())
}
```

### PQC Key Generation

```rust
use wia_quantum::crypto::{PqcAlgorithm, PqcKeyPair};

let keypair = PqcKeyPair::generate(PqcAlgorithm::MlKem768)?;
println!("Key ID: {}", keypair.metadata.key_id);
```

### Features

- **Quantum Circuits**: Build and manipulate (OpenQASM 3 compatible)
- **Simulator**: Local state vector simulator (up to 20 qubits)
- **PQC**: ML-KEM, ML-DSA, SLH-DSA, HQC (NIST standards)
- **QKD**: Session management (BB84, E91, etc.)
- **States**: Quantum state manipulation and analysis

---

## 📖 Phase 3: Communication Protocol

WIA Quantum Communication Protocol은 양자 백엔드와의 통신을 위한 표준 프로토콜입니다.

### Key Documents

| Document | Description |
|----------|-------------|
| [Protocol Spec](spec/PHASE-3-PROTOCOL.md) | 통신 프로토콜 명세서 |

### Protocol Schemas

| Schema | Description |
|--------|-------------|
| [message.schema.json](spec/schemas/protocol/message.schema.json) | Base message format |
| [submit-job.schema.json](spec/schemas/protocol/submit-job.schema.json) | Job submission |
| [job-result.schema.json](spec/schemas/protocol/job-result.schema.json) | Job results |
| [backend-status.schema.json](spec/schemas/protocol/backend-status.schema.json) | Backend status |
| [error.schema.json](spec/schemas/protocol/error.schema.json) | Error messages |

### Protocol Client Example

```rust
use wia_quantum::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Connect to quantum backend
    let client = Client::new_mock(ClientConfig {
        url: "wss://quantum.wia.live/wia-quantum".to_string(),
        api_key: Some("your_api_key".to_string()),
        ..Default::default()
    });

    client.connect().await?;

    // Submit a job
    let mut circuit = QuantumCircuit::new(2, 2);
    circuit.h(0)?;
    circuit.cx(0, 1)?;
    circuit.measure_all()?;

    let job = client.submit_job(&circuit, "simulator", None).await?;
    println!("Job ID: {}", job.job_id);

    client.disconnect().await?;
    Ok(())
}
```

### Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `connect` | Client → Backend | 연결 요청 |
| `submit_job` | Client → Backend | 작업 제출 |
| `job_completed` | Backend → Client | 작업 완료 |
| `backend_status` | Backend → Client | 백엔드 상태 |
| `calibration` | Backend → Client | 캘리브레이션 데이터 |

---

## 📖 Phase 4: Ecosystem Integration

WIA Quantum Ecosystem Integration은 다양한 양자 플랫폼과 WIA 생태계를 연동합니다.

### Key Documents

| Document | Description |
|----------|-------------|
| [Integration Spec](spec/PHASE-4-INTEGRATION.md) | 생태계 연동 명세서 |

### Supported Providers

| Provider | Type | Qubits |
|----------|------|--------|
| **Local Simulator** | Simulator | 30 |
| **IBM Quantum** | Hardware | 127 |
| **Amazon Braket** | Hardware | 50 |

### Multi-Backend Example

```rust
use wia_quantum::prelude::*;
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<()> {
    let manager = Arc::new(ProviderManager::new());
    manager.register(Box::new(LocalSimulatorProvider::new())).await?;

    let mut circuit = QuantumCircuit::new(2, 2);
    circuit.h(0)?;
    circuit.cx(0, 1)?;
    circuit.measure_all()?;

    let job = manager.submit(JobRequest::from_circuit(&circuit)).await?;
    println!("Job: {}", job.job_id);
    Ok(())
}
```

### VQE Example

```rust
use wia_quantum::prelude::*;

let executor = HybridExecutor::new(manager);
let result = executor.run_vqe(VQEConfig {
    num_qubits: 2,
    ansatz: AnsatzType::RealAmplitudes,
    ..Default::default()
}).await?;

println!("Ground state energy: {:.6}", result.optimal_value);
```

### Features

- **Multi-Backend**: IBM, Google, Amazon, Local 지원
- **Hybrid Workflows**: VQE, QAOA 알고리즘
- **Result Analysis**: 통계 분석 및 시각화
- **WIA Integration**: BCI, AAC 연동

---

## 🚀 Quick Start

### Quantum Circuit Example

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "circuit",
  "circuit_id": "550e8400-e29b-41d4-a716-446655440000",
  "name": "Bell State",
  "circuit": {
    "num_qubits": 2,
    "num_clbits": 2,
    "qasm": "OPENQASM 3.0;\ninclude \"stdgates.inc\";\nqubit[2] q;\nbit[2] c;\nh q[0];\ncx q[0], q[1];\nc = measure q;",
    "gates": [
      {"name": "h", "qubits": [0]},
      {"name": "cx", "qubits": [0, 1]},
      {"name": "measure", "qubits": [0, 1], "clbits": [0, 1]}
    ]
  }
}
```

### PQC Key Example

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "crypto",
  "crypto_type": "pqc_key",
  "algorithm": {
    "name": "ML-KEM",
    "variant": "ML-KEM-768",
    "nist_level": 3,
    "type": "kem"
  },
  "key": {
    "format": "base64",
    "public_key": "..."
  }
}
```

---

## 📁 Structure

```
quantum/
├── spec/                    # Specifications
│   ├── RESEARCH-PHASE-1.md
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-3-PROTOCOL.md
│   ├── PHASE-4-INTEGRATION.md
│   └── schemas/             # JSON Schemas
│       ├── circuit.schema.json
│       ├── job.schema.json
│       ├── result.schema.json
│       └── protocol/        # Protocol Schemas
├── api/
│   └── rust/                # Rust SDK
│       ├── src/
│       │   ├── circuit/     # Quantum circuits
│       │   ├── backend/     # Simulators & backends
│       │   ├── crypto/      # PQC & QKD
│       │   ├── state/       # Quantum states
│       │   ├── protocol/    # Communication protocol
│       │   └── integration/ # Ecosystem integration
│       └── examples/
├── prompts/                 # Claude Code prompts
└── docs/
```

---

## 🔗 Compatibility

| Platform | Compatibility |
|----------|---------------|
| **OpenQASM 3** | Circuit representation |
| **Qiskit** | IBM Quantum |
| **Cirq** | Google Quantum |
| **PennyLane** | Hybrid ML |
| **NIST PQC** | ML-KEM, ML-DSA, SLH-DSA |

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://quantum.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/quantum |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
