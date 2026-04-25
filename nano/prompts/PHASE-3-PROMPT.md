# Phase 3: Communication Protocol
## Claude Code 작업 프롬프트

---

**Standard**: WIA Nano
**Phase**: 3 of 4
**목표**: 나노 시스템 간 통신 프로토콜 표준화
**난이도**: ★★★★★
**예상 작업량**: 스펙 문서 1개 + Protocol 구현 + 예제

---

## 🎯 Phase 3 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 API Interface를 만들었다.

 이제 나노 시스템들이 실제로 어떻게 통신할 것인가?

 - 나노머신 간 분자 신호 전달?
 - 나노로봇 군집 제어?
 - 바이오-나노 인터페이스?

 모든 통신 방식에서 동일한 메시지 형식을 사용할 수 있을까?"
```

### 목표
```
나노 시스템 간 통신을 위한
WIA Nano Protocol (WNP)을 정의한다.

- 메시지 형식 (Message Format)
- 분자 신호 프로토콜 (Molecular Signaling)
- 나노 네트워크 (Nano Network)
- 바이오-나노 인터페이스 (Bio-Nano Interface)
- 다중 전송 방식 지원
```

---

## 📋 Phase 1 & 2 결과물 활용

| 이전 Phase 산출물 | Phase 3 활용 |
|-----------------|-------------|
| Phase 1: Data Format | 메시지 페이로드 (payload) |
| Phase 2: Rust API | 메시지 핸들러 연동 |
| JSON Schema | 메시지 검증 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 나노 통신 기술 조사

| 기술 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **Molecular Communication** | 분자 통신 이론 | "molecular communication nanonetwork signal propagation" |
| **Quorum Sensing** | 세균 군집 감지 | "quorum sensing bacterial communication signaling molecule" |
| **DNA Computing** | DNA 기반 정보 처리 | "DNA computing molecular logic gates circuit" |
| **Protein Signaling** | 단백질 신호 전달 | "protein signaling pathway cascade transduction" |
| **Nanorobot Swarm** | 나노로봇 군집 | "nanorobot swarm communication coordination control" |

### 2단계: 바이오-나노 인터페이스 조사

| 인터페이스 | 조사 내용 | 웹서치 키워드 |
|-----------|----------|--------------|
| **Cell Membrane** | 세포막 상호작용 | "cell membrane nanoparticle interaction endocytosis" |
| **Ion Channels** | 이온 채널 | "ion channel molecular signaling nanopore" |
| **Receptor Binding** | 수용체 결합 | "receptor ligand binding nanoparticle targeting" |
| **Neuron Interface** | 신경 인터페이스 | "neuron nanoelectrode brain computer interface" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-3.md`에 다음을 정리:

```markdown
# Phase 3 사전 조사 결과

## 1. 분자 통신 (Molecular Communication)

### 기본 원리
- 확산 기반 통신: [조사 내용]
- 신호 분자 종류: [조사 내용]
- WIA Nano 적용: [분석]

### 수학적 모델
- Fick의 확산 법칙: [조사 내용]
- 신호 감쇠: [조사 내용]

## 2. Quorum Sensing

### 메커니즘
- AHL (Acyl Homoserine Lactone): [조사 내용]
- 임계 농도: [조사 내용]
- 나노로봇 군집 적용: [분석]

## 3. DNA Computing

### 논리 게이트
- AND, OR, NOT 게이트: [조사 내용]
- 분자 회로: [조사 내용]
- 통신 프로토콜 적용: [분석]

## 4. 바이오-나노 인터페이스

### 세포 통신
- 엔도사이토시스: [조사 내용]
- 수용체 매개 통신: [조사 내용]

## 5. 결론
- 권장 프로토콜 아키텍처: [제안]
- 메시지 형식 설계 방향: [제안]
```

---

## 🏗️ 프로토콜 설계

### 1. 메시지 형식 (Message Format)

#### 기본 메시지 구조
```json
{
  "protocol": "wia-nano",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "메시지 유형",
  "source": {
    "id": "송신자 ID",
    "type": "node 유형",
    "location_nm": {"x": 0, "y": 0, "z": 0}
  },
  "destination": {
    "id": "수신자 ID",
    "type": "node 유형",
    "broadcast": false
  },
  "transport": {
    "method": "전송 방식",
    "carrier_molecule": "운반 분자",
    "diffusion_coefficient": "확산 계수"
  },
  "payload": {
    "메시지 데이터"
  },
  "ttl": "time-to-live (초)"
}
```

#### 메시지 유형 (Message Types)

| Type | 설명 | 예시 |
|------|------|------|
| `signal` | 분자 신호 전달 | Quorum sensing |
| `command` | 명령 전송 | 나노로봇 제어 |
| `telemetry` | 센싱 데이터 | 환경 측정값 |
| `acknowledgment` | 응답 확인 | 수신 확인 |
| `coordination` | 군집 조정 | 스웜 동기화 |
| `emergency` | 긴급 신호 | 독성 감지 |

#### 노드 유형 (Node Types)

| Type | 설명 | 예시 |
|------|------|------|
| `nanorobot` | 나노로봇 | DNA 오리가미 로봇 |
| `nanomachine` | 나노머신 | 분자 모터 |
| `biosensor` | 바이오센서 | 양자점 센서 |
| `controller` | 제어기 | 외부 자기장 제어 |
| `cell` | 세포 | 표적 세포 |
| `gateway` | 게이트웨이 | 체외-체내 통신 |

### 2. 전송 방식 (Transport Methods)

#### 확산 기반 통신 (Diffusion-Based)
```json
{
  "transport": {
    "method": "diffusion",
    "carrier_molecule": "acetylcholine",
    "diffusion_coefficient_m2_per_s": 4e-10,
    "medium_viscosity_pa_s": 0.001,
    "temperature_k": 310
  }
}
```

#### 유도 기반 통신 (Guided)
```json
{
  "transport": {
    "method": "guided",
    "guidance_type": "magnetic_field",
    "field_strength_tesla": 0.1,
    "gradient_t_per_m": 0.01
  }
}
```

#### 직접 전달 (Direct Transfer)
```json
{
  "transport": {
    "method": "direct",
    "mechanism": "gap_junction",
    "channel_conductance_ps": 100
  }
}
```

### 3. 분자 신호 프로토콜

#### Quorum Sensing 메시지
```json
{
  "type": "signal",
  "payload": {
    "signal_type": "quorum_sensing",
    "molecule": "AHL",
    "concentration_nm": 100,
    "threshold_nm": 50,
    "action_triggered": true,
    "collective_behavior": "biofilm_formation"
  }
}
```

#### DNA 기반 논리 연산
```json
{
  "type": "command",
  "payload": {
    "logic_operation": "AND",
    "input_strands": ["strand_A", "strand_B"],
    "output_strand": "strand_C",
    "reaction_time_sec": 60,
    "success_rate": 0.95
  }
}
```

### 4. 바이오-나노 인터페이스 메시지

#### 세포 표적화
```json
{
  "type": "signal",
  "destination": {
    "type": "cell",
    "cell_type": "cancer_cell",
    "marker": "folate_receptor"
  },
  "payload": {
    "targeting": {
      "ligand": "folate",
      "affinity_kd_nm": 0.1,
      "binding_site": "cell_surface"
    },
    "cargo": {
      "type": "therapeutic",
      "drug": "doxorubicin",
      "release_trigger": "ph_6.5"
    }
  }
}
```

### 5. 에러 코드 (Error Codes)

| 코드 범위 | 카테고리 | 설명 |
|----------|---------|------|
| `1xxx` | Diffusion | 확산 오류 |
| `2xxx` | Molecular | 분자 신호 오류 |
| `3xxx` | Network | 네트워크 오류 |
| `4xxx` | Bio-Interface | 바이오 인터페이스 오류 |
| `5xxx` | System | 시스템 오류 |

---

## 🔧 Rust 구현

### 프로토콜 메시지 타입 (protocol/message.rs)
```rust
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WnpMessage {
    pub protocol: String,
    pub version: String,
    pub message_id: Uuid,
    pub timestamp: u64,
    pub message_type: MessageType,
    pub source: NodeInfo,
    pub destination: NodeInfo,
    pub transport: TransportInfo,
    pub payload: serde_json::Value,
    pub ttl: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Signal,
    Command,
    Telemetry,
    Acknowledgment,
    Coordination,
    Emergency,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeInfo {
    pub id: String,
    pub node_type: NodeType,
    pub location_nm: Option<Vector3D>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NodeType {
    Nanorobot,
    Nanomachine,
    Biosensor,
    Controller,
    Cell,
    Gateway,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransportInfo {
    pub method: TransportMethod,
    pub parameters: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransportMethod {
    Diffusion,
    Guided,
    Direct,
}
```

### 확산 시뮬레이션 (transport/diffusion.rs)
```rust
use crate::{NanoResult, Vector3D};

pub struct DiffusionSimulator {
    pub diffusion_coefficient_m2_per_s: f64,
    pub temperature_k: f64,
    pub viscosity_pa_s: f64,
}

impl DiffusionSimulator {
    /// Fick의 제1법칙: 농도 기울기에 의한 확산
    pub fn calculate_flux(
        &self,
        concentration_gradient_per_m: f64,
    ) -> f64 {
        -self.diffusion_coefficient_m2_per_s * concentration_gradient_per_m
    }

    /// 확산 시간 계산 (평균 제곱 변위)
    pub fn estimate_diffusion_time(&self, distance_m: f64) -> f64 {
        // t = <x²> / (2D)
        distance_m.powi(2) / (2.0 * self.diffusion_coefficient_m2_per_s)
    }

    /// 3D 확산 농도 분포 (가우스 분포)
    pub fn concentration_at_position(
        &self,
        source_pos: &Vector3D,
        target_pos: &Vector3D,
        time_s: f64,
        initial_molecules: usize,
    ) -> f64 {
        let dx = (target_pos.x - source_pos.x) * 1e-9;
        let dy = (target_pos.y - source_pos.y) * 1e-9;
        let dz = (target_pos.z - source_pos.z) * 1e-9;

        let r_squared = dx.powi(2) + dy.powi(2) + dz.powi(2);

        let denominator = (4.0 * std::f64::consts::PI
                          * self.diffusion_coefficient_m2_per_s
                          * time_s).powf(1.5);

        let exponent = -r_squared / (4.0 * self.diffusion_coefficient_m2_per_s * time_s);

        (initial_molecules as f64 / denominator) * exponent.exp()
    }

    /// 신호 도달 확률
    pub fn signal_reception_probability(
        &self,
        distance_nm: f64,
        time_s: f64,
        threshold_molecules: usize,
        transmitted_molecules: usize,
    ) -> f64 {
        let source_pos = Vector3D::new(0.0, 0.0, 0.0);
        let target_pos = Vector3D::new(distance_nm, 0.0, 0.0);

        let concentration = self.concentration_at_position(
            &source_pos,
            &target_pos,
            time_s,
            transmitted_molecules,
        );

        if concentration >= threshold_molecules as f64 {
            1.0
        } else {
            concentration / threshold_molecules as f64
        }
    }
}
```

### Quorum Sensing (protocol/quorum_sensing.rs)
```rust
use crate::{NanoResult, NanoError};

pub struct QuorumSensingNetwork {
    pub nodes: Vec<QuorumNode>,
    pub signal_molecule: String,
    pub diffusion_coefficient: f64,
}

#[derive(Debug, Clone)]
pub struct QuorumNode {
    pub id: String,
    pub position_nm: Vector3D,
    pub signal_production_rate: f64,  // molecules/sec
    pub signal_threshold_nm: f64,
    pub current_concentration_nm: f64,
    pub activated: bool,
}

impl QuorumSensingNetwork {
    /// 네트워크 상태 업데이트
    pub fn update(&mut self, delta_time_s: f64) -> NanoResult<()> {
        // 1. 각 노드의 신호 생성
        for node in &mut self.nodes {
            let produced = node.signal_production_rate * delta_time_s;
            node.current_concentration_nm += produced;
        }

        // 2. 확산 시뮬레이션
        self.simulate_diffusion(delta_time_s)?;

        // 3. 임계값 확인 및 활성화
        for node in &mut self.nodes {
            if node.current_concentration_nm >= node.signal_threshold_nm {
                node.activated = true;
            }
        }

        Ok(())
    }

    fn simulate_diffusion(&mut self, _delta_time_s: f64) -> NanoResult<()> {
        // 단순화된 확산 모델
        // 실제로는 PDE 솔버 필요
        Ok(())
    }

    /// 군집 동기화 확인
    pub fn is_synchronized(&self) -> bool {
        let activated_count = self.nodes.iter().filter(|n| n.activated).count();
        let total = self.nodes.len();

        activated_count as f64 / total as f64 > 0.8
    }
}
```

### 나노로봇 군집 제어 (protocol/swarm_control.rs)
```rust
use crate::{NanoResult, Vector3D};

pub struct NanorobotSwarm {
    pub robots: Vec<Nanorobot>,
    pub communication_range_nm: f64,
    pub coordination_protocol: CoordinationProtocol,
}

#[derive(Debug, Clone)]
pub struct Nanorobot {
    pub id: String,
    pub position_nm: Vector3D,
    pub velocity_nm_per_s: Vector3D,
    pub neighbors: Vec<String>,
    pub state: RobotState,
}

#[derive(Debug, Clone)]
pub enum RobotState {
    Idle,
    Moving,
    Working,
    Signaling,
}

#[derive(Debug, Clone)]
pub enum CoordinationProtocol {
    LeaderFollower,
    Consensus,
    BehaviorBased,
}

impl NanorobotSwarm {
    /// 이웃 노드 탐색
    pub fn update_neighbors(&mut self) {
        for i in 0..self.robots.len() {
            let mut neighbors = Vec::new();

            for j in 0..self.robots.len() {
                if i == j {
                    continue;
                }

                let distance = self.distance_between(i, j);
                if distance <= self.communication_range_nm {
                    neighbors.push(self.robots[j].id.clone());
                }
            }

            self.robots[i].neighbors = neighbors;
        }
    }

    fn distance_between(&self, i: usize, j: usize) -> f64 {
        let pos1 = &self.robots[i].position_nm;
        let pos2 = &self.robots[j].position_nm;

        let dx = pos1.x - pos2.x;
        let dy = pos1.y - pos2.y;
        let dz = pos1.z - pos2.z;

        (dx.powi(2) + dy.powi(2) + dz.powi(2)).sqrt()
    }

    /// 군집 중심 계산
    pub fn calculate_centroid(&self) -> Vector3D {
        let mut sum = Vector3D::new(0.0, 0.0, 0.0);

        for robot in &self.robots {
            sum.x += robot.position_nm.x;
            sum.y += robot.position_nm.y;
            sum.z += robot.position_nm.z;
        }

        let n = self.robots.len() as f64;
        Vector3D::new(sum.x / n, sum.y / n, sum.z / n)
    }

    /// 군집 분산 계산
    pub fn calculate_variance(&self) -> f64 {
        let centroid = self.calculate_centroid();
        let mut sum_sq_dist = 0.0;

        for robot in &self.robots {
            let dx = robot.position_nm.x - centroid.x;
            let dy = robot.position_nm.y - centroid.y;
            let dz = robot.position_nm.z - centroid.z;

            sum_sq_dist += dx.powi(2) + dy.powi(2) + dz.powi(2);
        }

        sum_sq_dist / self.robots.len() as f64
    }
}
```

---

## 📁 산출물 목록

Phase 3 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-3.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-3-PROTOCOL.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 메시지 형식 (Message Format)
4. 메시지 유형 (Message Types)
5. 전송 방식 (Transport Methods)
   - 확산 기반 (Diffusion)
   - 유도 기반 (Guided)
   - 직접 전달 (Direct)
6. 분자 신호 프로토콜 (Molecular Signaling)
7. 바이오-나노 인터페이스 (Bio-Nano Interface)
8. 나노 네트워크 (Nano Network)
9. 에러 처리 (Error Handling)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. JSON Schema
```
/spec/schemas/
├── wnp-message.schema.json     # 프로토콜 메시지 스키마
└── wnp-error.schema.json       # 에러 메시지 스키마
```

### 4. Rust Protocol 구현
```
/api/rust/src/
├── protocol/
│   ├── mod.rs
│   ├── message.rs              # 메시지 타입 정의
│   ├── builder.rs              # 메시지 생성
│   ├── handler.rs              # 프로토콜 처리
│   ├── quorum_sensing.rs       # Quorum sensing
│   ├── swarm_control.rs        # 군집 제어
│   └── error.rs                # 에러 타입
├── transport/
│   ├── mod.rs
│   ├── base.rs                 # 전송 인터페이스
│   ├── diffusion.rs            # 확산 시뮬레이션
│   ├── guided.rs               # 유도 전송
│   └── mock.rs                 # 테스트용
└── ...
```

### 5. 예제 코드
```
/api/rust/examples/
├── protocol_demo.rs            # 프로토콜 데모
├── quorum_sensing_sim.rs       # Quorum sensing 시뮬레이션
└── swarm_coordination.rs       # 군집 조정 예제
```

---

## ✅ 완료 체크리스트

Phase 3 완료 전 확인:

```
□ 웹서치로 나노 통신 프로토콜 조사 완료
□ /spec/RESEARCH-PHASE-3.md 작성 완료
□ /spec/PHASE-3-PROTOCOL.md 작성 완료
□ 메시지 형식 JSON Schema 정의 완료
□ Rust protocol 모듈 구현 완료
□ Rust transport 모듈 구현 완료
□ 확산 시뮬레이터 구현 완료
□ Quorum sensing 구현 완료
□ 나노로봇 군집 제어 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 예제 코드 완료
□ README 업데이트 (Phase 3 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 나노 통신 프로토콜 조사
   ↓
2. /spec/RESEARCH-PHASE-3.md 작성
   ↓
3. 프로토콜 설계
   ↓
4. /spec/PHASE-3-PROTOCOL.md 작성
   ↓
5. 메시지 형식 JSON Schema 작성
   ↓
6. Rust protocol 모듈 구현
   ↓
7. Rust transport 모듈 구현
   ↓
8. 확산 시뮬레이터 구현
   ↓
9. Quorum sensing 구현
   ↓
10. 군집 제어 구현
   ↓
11. 테스트 작성 및 실행
   ↓
12. 예제 코드 작성
   ↓
13. 완료 체크리스트 확인
   ↓
14. Phase 4 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 Data Format을 메시지 payload로 사용
✅ Phase 2 Rust API와 연동 가능하도록 설계
✅ 전송 계층 추상화 (다양한 전송 방식 지원)
✅ 확산 기반 통신 고려 (지연은 기본)
✅ 메시지 순서 보장 불가 (비동기 환경)
✅ 생물학적 제약 고려
✅ 확산 시뮬레이션 기능 포함
```

### DON'T (하지 말 것)

```
❌ 특정 전송 방식에만 종속되는 설계
❌ 실시간/동기식 통신 가정
❌ Phase 1/2 형식과 불일치
❌ 에러 처리 없는 happy path만 구현
❌ 생물학적 현실성 무시
```

---

## 🚀 작업 시작

이제 Phase 3 작업을 시작하세요.

첫 번째 단계: **웹서치로 분자 통신 조사**

```
검색 키워드: "molecular communication nanonetwork diffusion-based"
```

화이팅! ⚛️

---

<div align="center">

**Phase 3 of 4**

WIA Nano Protocol (WNP)

🔬 Communication at the Nanoscale 🔬

弘益人間 - Benefit All Humanity

</div>
