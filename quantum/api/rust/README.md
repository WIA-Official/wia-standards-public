# WIA Quantum Rust SDK

**Phase 2: Rust Implementation**

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-12-16
**Authors**: Yeon Sam-Heum, Ph.D.

---

## Overview

WIA Quantum Rust SDK는 양자 컴퓨팅 데이터를 Rust에서 안전하고 효율적으로 다루기 위한 공식 구현체입니다.

### Features

- **Type-Safe Quantum Data**: 컴파일 타임 타입 안전성
- **Zero-Copy Deserialization**: 고성능 파싱
- **OpenQASM 3 Support**: 양자 회로 표준 지원
- **NIST PQC Integration**: 포스트-양자 암호화
- **Async Runtime**: Tokio 기반 비동기 처리

---

## Installation

```toml
[dependencies]
wia-quantum = "1.0"
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
```

---

## Quick Start

```rust
use wia_quantum::{QuantumCircuit, QuantumDevice, QuantumResult};

// Quantum Circuit 생성
let circuit = QuantumCircuit::builder()
    .qubits(5)
    .add_gate("H", vec![0])
    .add_gate("CNOT", vec![0, 1])
    .build()?;

// 실행
let result = QuantumDevice::new("ibm_quantum")
    .execute(&circuit)
    .await?;

println!("Measurement: {:?}", result.measurements);
```

---

## Core Types

### QuantumCircuit

```rust
pub struct QuantumCircuit {
    pub qubits: usize,
    pub gates: Vec<QuantumGate>,
    pub measurements: Vec<usize>,
}
```

### QuantumGate

```rust
pub enum QuantumGate {
    H(usize),           // Hadamard
    CNOT(usize, usize), // Controlled-NOT
    RZ(usize, f64),     // Rotation-Z
    Custom(String, Vec<usize>),
}
```

### QuantumResult

```rust
pub struct QuantumResult {
    pub measurements: HashMap<String, f64>,
    pub state_vector: Option<Vec<Complex<f64>>>,
    pub execution_time: Duration,
}
```

---

## OpenQASM 3 Support

```rust
use wia_quantum::qasm::QasmParser;

let qasm = r#"
OPENQASM 3;
qubit[2] q;
h q[0];
cx q[0], q[1];
"#;

let circuit = QasmParser::parse(qasm)?;
```

---

## Quantum Communication

```rust
use wia_quantum::comm::{QuantumChannel, QKD};

// Quantum Key Distribution
let qkd = QKD::bb84();
let key = qkd.generate_key(256).await?;

// Quantum Teleportation
let channel = QuantumChannel::new("alice", "bob");
channel.teleport(qubit).await?;
```

---

## Quantum Sensors

```rust
use wia_quantum::sensor::{QuantumSensor, SensorType};

let sensor = QuantumSensor::new(SensorType::Magnetometer)?;
let reading = sensor.read().await?;

println!("Magnetic field: {} T", reading.value);
```

---

## NIST PQC Integration

```rust
use wia_quantum::crypto::{Kyber, Dilithium};

// Post-Quantum Key Exchange
let (public, secret) = Kyber::keypair()?;
let ciphertext = Kyber::encapsulate(&public)?;
let shared_secret = Kyber::decapsulate(&secret, &ciphertext)?;

// Post-Quantum Signatures
let (signing_key, verify_key) = Dilithium::keypair()?;
let signature = Dilithium::sign(&signing_key, b"message")?;
Dilithium::verify(&verify_key, b"message", &signature)?;
```

---

## Error Handling

```rust
use wia_quantum::error::{QuantumError, Result};

fn process_quantum_data() -> Result<()> {
    let circuit = QuantumCircuit::load("circuit.json")
        .map_err(|e| QuantumError::ParseError(e))?;
    
    if circuit.qubits > 50 {
        return Err(QuantumError::TooManyQubits);
    }
    
    Ok(())
}
```

---

## Testing

```bash
# Run all tests
cargo test

# Run with quantum backend
cargo test --features quantum-backend

# Benchmark
cargo bench
```

---

## Examples

See `examples/` directory:
- `basic_circuit.rs` - Simple quantum circuit
- `qkd_bb84.rs` - Quantum key distribution
- `teleportation.rs` - Quantum teleportation
- `grover_search.rs` - Grover's algorithm
- `shor_factoring.rs` - Shor's algorithm

---

## Performance

- Circuit parsing: < 1ms for 1000 gates
- State vector simulation: 2^25 amplitudes
- Zero-copy deserialization with `serde`
- Async I/O with Tokio

---

## Standards Compliance

- ✅ OpenQASM 3.0
- ✅ NIST PQC (Kyber, Dilithium)
- ✅ WIA Quantum Data Format v1.0
- ✅ WIA Quantum Protocol v1.0

---

## License

MIT License - See LICENSE file

---

## Links

- [WIA Quantum Standard](https://quantum.wiastandards.com)
- [GitHub Repository](https://github.com/WIA-Official/wia-standards)
- [API Documentation](https://api.wiastandards.com/quantum)
