# WIA Quantum Data Format Specification

**Phase 1: Data Format Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01

---

## 1. Overview

### 1.1 Purpose

WIA Quantum Data Format은 양자 컴퓨팅 생태계의 데이터 교환을 위한 통합 표준입니다. 양자 회로, 실행 결과, 암호화 키, 센서 데이터 등을 일관된 형식으로 표현합니다.

### 1.2 Design Goals

1. **OpenQASM 3 호환**: 업계 표준 양자 회로 표현
2. **NIST PQC 호환**: 포스트-양자 암호화 표준 지원
3. **확장성**: 새로운 양자 기술 수용
4. **상호운용성**: Qiskit, Cirq, PennyLane 등과 호환
5. **사람 친화적**: JSON 기반 가독성

### 1.3 Scope

| 포함 | 미포함 |
|------|--------|
| 양자 회로 | 하드웨어 드라이버 |
| 실행 결과 | 실시간 프로토콜 (Phase 3) |
| PQC 키/서명 | SDK 구현 (Phase 2) |
| 센서 데이터 | 에코시스템 통합 (Phase 4) |
| 네트워크 토폴로지 | |

---

## 2. Data Types

### 2.1 Type Hierarchy

```
WiaQuantum
├── Circuit         # 양자 회로
├── Job             # 실행 작업
├── Result          # 실행 결과
├── State           # 양자 상태
├── Crypto          # 암호화 데이터
│   ├── PqcKey      # PQC 키
│   ├── PqcSignature # PQC 서명
│   └── QkdSession  # QKD 세션
├── Sensor          # 센서 데이터
│   ├── AtomicClock # 원자시계
│   ├── Magnetometer # 자력계
│   └── Gravimeter  # 중력계
└── Network         # 네트워크
    ├── Node        # 양자 노드
    ├── Link        # 양자 링크
    └── Entanglement # 얽힘 자원
```

---

## 3. Quantum Circuit Format

### 3.1 Circuit Object

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "circuit",
  "circuit_id": "uuid-string",
  "name": "Bell State",
  "description": "Create Bell state |Φ+⟩",
  "created_at": "2025-01-15T10:30:00Z",

  "metadata": {
    "author": "WIA Quantum Team",
    "tags": ["entanglement", "bell-state"],
    "license": "MIT"
  },

  "circuit": {
    "num_qubits": 2,
    "num_clbits": 2,
    "qasm": "OPENQASM 3.0;\ninclude \"stdgates.inc\";\nqubit[2] q;\nbit[2] c;\nh q[0];\ncx q[0], q[1];\nc = measure q;",
    "gates": [
      {
        "name": "h",
        "qubits": [0],
        "params": []
      },
      {
        "name": "cx",
        "qubits": [0, 1],
        "params": []
      },
      {
        "name": "measure",
        "qubits": [0, 1],
        "clbits": [0, 1]
      }
    ]
  },

  "visualization": {
    "layout": "linear",
    "style": "mpl"
  }
}
```

### 3.2 Gate Definitions

| 게이트 | 설명 | 파라미터 |
|--------|------|----------|
| `h` | Hadamard | - |
| `x` | Pauli-X | - |
| `y` | Pauli-Y | - |
| `z` | Pauli-Z | - |
| `cx` | CNOT | - |
| `cz` | CZ | - |
| `swap` | SWAP | - |
| `rx` | Rotation-X | theta |
| `ry` | Rotation-Y | theta |
| `rz` | Rotation-Z | theta |
| `u` | Universal | theta, phi, lambda |
| `measure` | Measurement | - |
| `reset` | Reset | - |
| `barrier` | Barrier | - |

### 3.3 Supported Instruction Types

```typescript
type GateInstruction = {
  name: string;
  qubits: number[];
  params?: number[];
  condition?: {
    clbit: number;
    value: number;
  };
};

type MeasureInstruction = {
  name: "measure";
  qubits: number[];
  clbits: number[];
};

type BarrierInstruction = {
  name: "barrier";
  qubits: number[];
};
```

---

## 4. Job Format

### 4.1 Job Request

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "job",
  "job_id": "uuid-string",
  "status": "queued",
  "created_at": "2025-01-15T10:30:00Z",

  "circuit_id": "uuid-circuit",

  "backend": {
    "provider": "ibm_quantum",
    "name": "ibm_brisbane",
    "type": "hardware",
    "num_qubits": 127
  },

  "execution": {
    "shots": 1024,
    "seed": 42,
    "optimization_level": 3,
    "error_mitigation": {
      "method": "ZNE",
      "enabled": true
    }
  },

  "transpilation": {
    "basis_gates": ["cx", "id", "rz", "sx", "x"],
    "coupling_map": [[0,1], [1,2], [2,3]],
    "initial_layout": [0, 1]
  }
}
```

### 4.2 Job Status

| 상태 | 설명 |
|------|------|
| `queued` | 대기 중 |
| `running` | 실행 중 |
| `completed` | 완료 |
| `failed` | 실패 |
| `cancelled` | 취소됨 |

---

## 5. Result Format

### 5.1 Execution Result

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "result",
  "result_id": "uuid-string",
  "job_id": "uuid-job",
  "circuit_id": "uuid-circuit",

  "status": "completed",
  "success": true,
  "created_at": "2025-01-15T10:35:00Z",

  "counts": {
    "00": 512,
    "11": 512
  },

  "quasi_probabilities": {
    "00": 0.5,
    "11": 0.5
  },

  "memory": ["00", "11", "00", "11", "..."],

  "metadata": {
    "shots": 1024,
    "execution_time_ms": 1500,
    "backend": "ibm_brisbane"
  },

  "statevector": {
    "real": [0.707, 0, 0, 0.707],
    "imag": [0, 0, 0, 0]
  }
}
```

### 5.2 Error Information

```json
{
  "error": {
    "code": "EXECUTION_ERROR",
    "message": "Qubit 5 failed calibration check",
    "details": {
      "qubit": 5,
      "t1": 45.2,
      "t2": 30.1,
      "gate_error": 0.015
    }
  }
}
```

---

## 6. Quantum State Format

### 6.1 State Vector

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "state",
  "state_type": "statevector",
  "num_qubits": 2,

  "data": {
    "format": "dense",
    "real": [0.707, 0, 0, 0.707],
    "imag": [0, 0, 0, 0]
  },

  "basis": "computational",
  "normalization": 1.0
}
```

### 6.2 Density Matrix

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "state",
  "state_type": "density_matrix",
  "num_qubits": 2,

  "data": {
    "format": "dense",
    "matrix": {
      "real": [[0.5, 0, 0, 0.5], [0, 0, 0, 0], [0, 0, 0, 0], [0.5, 0, 0, 0.5]],
      "imag": [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    }
  },

  "purity": 1.0,
  "trace": 1.0
}
```

---

## 7. Cryptography Formats

### 7.1 PQC Key

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
    "public_key": "base64-encoded-public-key...",
    "secret_key": "base64-encoded-secret-key...",
    "key_size_bytes": {
      "public": 1184,
      "secret": 2400
    }
  },

  "metadata": {
    "generated_at": "2025-01-15T10:30:00Z",
    "expires_at": "2026-01-15T10:30:00Z",
    "key_id": "uuid-key"
  }
}
```

### 7.2 PQC Signature

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "crypto",
  "crypto_type": "pqc_signature",

  "algorithm": {
    "name": "ML-DSA",
    "variant": "ML-DSA-65",
    "nist_level": 3,
    "type": "signature"
  },

  "signature": {
    "format": "base64",
    "value": "base64-encoded-signature...",
    "size_bytes": 3309
  },

  "message_hash": {
    "algorithm": "SHA3-256",
    "value": "hex-encoded-hash..."
  },

  "public_key_id": "uuid-key",
  "signed_at": "2025-01-15T10:30:00Z"
}
```

### 7.3 QKD Session

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "crypto",
  "crypto_type": "qkd_session",

  "session_id": "uuid-session",
  "protocol": "BB84",

  "participants": {
    "alice": {
      "node_id": "node-alice",
      "address": "192.168.1.10"
    },
    "bob": {
      "node_id": "node-bob",
      "address": "192.168.1.20"
    }
  },

  "channel": {
    "type": "fiber",
    "length_km": 50,
    "loss_db": 10
  },

  "statistics": {
    "raw_key_rate_bps": 10000,
    "qber": 0.02,
    "secure_key_rate_bps": 5000,
    "total_key_bits": 1000000
  },

  "status": "active",
  "started_at": "2025-01-15T10:00:00Z"
}
```

---

## 8. Sensor Data Formats

### 8.1 Atomic Clock

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "sensor",
  "sensor_type": "atomic_clock",

  "device": {
    "manufacturer": "Infleqtion",
    "model": "Tiqker",
    "type": "optical",
    "serial": "TQK-2025-001"
  },

  "measurement": {
    "timestamp": "2025-01-15T10:30:00.000000000Z",
    "frequency_hz": 429228004229873.0,
    "fractional_frequency_offset": 1.5e-15,
    "allan_deviation": 2.0e-16
  },

  "timing": {
    "utc_offset_ns": 0.5,
    "tai_offset_s": 37,
    "pps_accuracy_ns": 1.0
  },

  "status": {
    "locked": true,
    "temperature_c": 25.0,
    "battery_percent": 85
  }
}
```

### 8.2 Magnetometer

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "sensor",
  "sensor_type": "magnetometer",

  "device": {
    "manufacturer": "QuSpin",
    "model": "QZFM",
    "type": "optically_pumped",
    "serial": "QS-2025-001"
  },

  "measurement": {
    "timestamp": "2025-01-15T10:30:00.000Z",
    "field_vector": {
      "bx_nT": 23456.7,
      "by_nT": -12345.6,
      "bz_nT": 45678.9
    },
    "magnitude_nT": 52341.2,
    "sensitivity_fT_sqrt_hz": 15.0
  },

  "calibration": {
    "last_calibrated": "2025-01-01T00:00:00Z",
    "offset_nT": [0.5, -0.3, 0.2],
    "scale_factor": [1.001, 0.999, 1.002]
  }
}
```

### 8.3 Gravimeter

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "sensor",
  "sensor_type": "gravimeter",

  "device": {
    "manufacturer": "Muquans",
    "model": "AQG-A10",
    "type": "atom_interferometer",
    "serial": "MQ-2025-001"
  },

  "measurement": {
    "timestamp": "2025-01-15T10:30:00.000Z",
    "gravity_m_s2": 9.80665,
    "gravity_mgal": 980665.0,
    "gradient_E": 3000.0,
    "sensitivity_ugal": 1.0
  },

  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude_m": 50.0
  }
}
```

---

## 9. Network Formats

### 9.1 Quantum Node

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "network",
  "network_type": "node",

  "node_id": "uuid-node",
  "name": "Seoul QNode",

  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "address": "Seoul, South Korea"
  },

  "capabilities": {
    "quantum_memory": true,
    "memory_coherence_ms": 100,
    "entanglement_generation": true,
    "entanglement_rate_hz": 1000,
    "qkd_protocols": ["BB84", "E91"],
    "num_ports": 4
  },

  "status": "online",
  "uptime_hours": 720
}
```

### 9.2 Quantum Link

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "network",
  "network_type": "link",

  "link_id": "uuid-link",
  "source_node": "node-seoul",
  "target_node": "node-busan",

  "physical": {
    "type": "fiber",
    "length_km": 325,
    "loss_db_km": 0.2,
    "total_loss_db": 65
  },

  "performance": {
    "entanglement_rate_hz": 100,
    "fidelity": 0.95,
    "latency_ms": 1.5
  },

  "status": "active"
}
```

### 9.3 Entanglement Resource

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "network",
  "network_type": "entanglement",

  "entanglement_id": "uuid-ent",

  "parties": [
    {"node_id": "node-seoul", "qubit_id": 0},
    {"node_id": "node-busan", "qubit_id": 0}
  ],

  "state": {
    "type": "bell",
    "name": "phi_plus",
    "fidelity": 0.98
  },

  "created_at": "2025-01-15T10:30:00Z",
  "expires_at": "2025-01-15T10:30:00.100Z",
  "coherence_time_ms": 100
}
```

---

## 10. File Extensions

| 확장자 | 용도 |
|--------|------|
| `.wqc` | WIA Quantum Circuit |
| `.wqj` | WIA Quantum Job |
| `.wqr` | WIA Quantum Result |
| `.wqs` | WIA Quantum State |
| `.wqk` | WIA Quantum Key (PQC/QKD) |
| `.wqn` | WIA Quantum Network |
| `.wqd` | WIA Quantum Sensor Data |

---

## 11. Common Schema

### 11.1 Base Object

모든 WIA Quantum 객체는 다음 필드를 포함합니다:

```json
{
  "wia_quantum_version": "1.0.0",
  "type": "circuit | job | result | state | crypto | sensor | network",
  "id": "uuid-string",
  "created_at": "ISO-8601 timestamp"
}
```

### 11.2 Version Format

```
MAJOR.MINOR.PATCH

- MAJOR: 호환되지 않는 변경
- MINOR: 하위 호환 기능 추가
- PATCH: 하위 호환 버그 수정
```

---

## 12. JSON Schema Files

다음 스키마 파일이 제공됩니다:

| 스키마 | 설명 |
|--------|------|
| `circuit.schema.json` | 양자 회로 |
| `job.schema.json` | 실행 작업 |
| `result.schema.json` | 실행 결과 |
| `state.schema.json` | 양자 상태 |
| `pqc-key.schema.json` | PQC 키 |
| `pqc-signature.schema.json` | PQC 서명 |
| `qkd-session.schema.json` | QKD 세션 |
| `atomic-clock.schema.json` | 원자시계 |
| `magnetometer.schema.json` | 자력계 |
| `gravimeter.schema.json` | 중력계 |
| `node.schema.json` | 양자 노드 |
| `link.schema.json` | 양자 링크 |
| `entanglement.schema.json` | 얽힘 자원 |

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA Quantum Working Group

---

弘益人間 - *Benefit All Humanity*
