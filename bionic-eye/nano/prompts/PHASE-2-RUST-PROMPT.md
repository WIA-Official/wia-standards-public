# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA Nano
**Phase**: 2 of 4
**Language**: **Rust** (Primary)
**목표**: Rust 기반 고성능 나노기술 API 구현
**난이도**: ★★★★★
**예상 작업량**: Rust 라이브러리 + 테스트 + 예제

---

## 🎯 Phase 2 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의했다.

 이제 이 데이터를 프로그래밍 방식으로 어떻게 다룰 것인가?

 - 분자 조립기 시뮬레이션?
 - 나노머신 동역학 계산?
 - DNA 메모리 인코딩/디코딩?

 모든 계산을 표준 API로 제공할 수 있을까?"
```

### 목표
```
나노기술 데이터를 처리하는 Rust API 구현

- 데이터 타입 정의 (Phase 1 스키마 기반)
- 핵심 계산 함수 구현
- 분자 시뮬레이션 어댑터
- WASM/Python 바인딩 지원 준비
```

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (복잡한 분자 시뮬레이션)
2. 안전: 메모리 안전 보장 (대규모 데이터 처리)
3. 정밀도: 과학 계산에 필요한 정밀도 지원
4. 크로스 플랫폼: WASM으로 브라우저 지원
5. 일관성: WIA 표준 전체에서 Rust 사용
```

---

## 📦 프로젝트 구조

```
/api/rust/
├── Cargo.toml
├── src/
│   ├── lib.rs                 # 메인 라이브러리
│   ├── types.rs               # 타입 정의
│   ├── error.rs               # 에러 타입
│   ├── core/
│   │   ├── mod.rs
│   │   ├── project.rs         # 프로젝트 관리
│   │   ├── calculator.rs      # 핵심 계산
│   │   └── simulator.rs       # 시뮬레이션
│   ├── adapters/
│   │   ├── mod.rs
│   │   ├── assembler.rs       # 분자 조립기
│   │   ├── nanomachine.rs     # 나노머신
│   │   ├── memory.rs          # 분자 메모리
│   │   ├── medicine.rs        # 나노의학
│   │   ├── robotics.rs        # 나노로봇
│   │   └── quantum_dots.rs    # 양자점
│   ├── physics/
│   │   ├── mod.rs
│   │   ├── constants.rs       # 물리 상수
│   │   ├── forces.rs          # 힘 계산
│   │   └── thermodynamics.rs  # 열역학
│   └── prelude.rs             # 편의 re-exports
├── tests/
│   └── integration_test.rs
└── examples/
    ├── basic_usage.rs
    ├── assembler_simulation.rs
    └── dna_encoding.rs
```

---

## 🔧 핵심 구현 코드

### Error 타입 (error.rs)
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum NanoError {
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("Calculation overflow: {0}")]
    CalculationOverflow(String),

    #[error("Physical constraint violated: {0}")]
    PhysicsViolation(String),

    #[error("Molecular structure invalid: {0}")]
    StructureInvalid(String),

    #[error("Simulation error: {0}")]
    SimulationError(String),

    #[error("Encoding error: {0}")]
    EncodingError(String),

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

pub type NanoResult<T> = std::result::Result<T, NanoError>;
```

### 물리 상수 (physics/constants.rs)
```rust
/// Boltzmann constant (J/K)
pub const BOLTZMANN_CONSTANT: f64 = 1.380649e-23;

/// Avogadro's number (mol^-1)
pub const AVOGADRO_NUMBER: f64 = 6.02214076e23;

/// Planck constant (J·s)
pub const PLANCK_CONSTANT: f64 = 6.62607015e-34;

/// Elementary charge (C)
pub const ELEMENTARY_CHARGE: f64 = 1.602176634e-19;

/// Speed of light in vacuum (m/s)
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Atomic mass unit (kg)
pub const ATOMIC_MASS_UNIT: f64 = 1.66053906660e-27;

/// Angstrom to meter conversion
pub const ANGSTROM_TO_METER: f64 = 1e-10;

/// Nanometer to meter conversion
pub const NANOMETER_TO_METER: f64 = 1e-9;

/// Picometer to meter conversion
pub const PICOMETER_TO_METER: f64 = 1e-12;
```

### 기본 타입 정의 (types.rs)
```rust
use serde::{Deserialize, Serialize};

/// 기술 유형
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TechnologyType {
    MolecularAssembler,
    Nanomachine,
    MolecularMemory,
    Nanomedicine,
    Nanorobotics,
    QuantumDots,
}

/// 기술 성숙도 레벨 (NASA TRL)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TRL(pub u8);

impl TRL {
    pub fn new(level: u8) -> NanoResult<Self> {
        if level >= 1 && level <= 9 {
            Ok(TRL(level))
        } else {
            Err(NanoError::InvalidParameter(
                "TRL must be between 1 and 9".into()
            ))
        }
    }

    pub fn description(&self) -> &'static str {
        match self.0 {
            1 => "Basic principles observed",
            2 => "Technology concept formulated",
            3 => "Experimental proof of concept",
            4 => "Technology validated in lab",
            5 => "Technology validated in relevant environment",
            6 => "Technology demonstrated in relevant environment",
            7 => "System prototype demonstrated",
            8 => "System complete and qualified",
            9 => "System proven in operational environment",
            _ => "Unknown",
        }
    }
}

/// 3D 벡터 (나노미터 단위)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        Self {
            x: self.x / mag,
            y: self.y / mag,
            z: self.z / mag,
        }
    }

    pub fn dot(&self, other: &Vector3D) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

/// 원자 (Atom)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Atom {
    pub element: String,
    pub atomic_number: u8,
    pub mass_daltons: f64,
    pub position_nm: Vector3D,
    pub charge: f64,
}

impl Atom {
    pub fn distance_to(&self, other: &Atom) -> f64 {
        let dx = self.position_nm.x - other.position_nm.x;
        let dy = self.position_nm.y - other.position_nm.y;
        let dz = self.position_nm.z - other.position_nm.z;
        (dx.powi(2) + dy.powi(2) + dz.powi(2)).sqrt()
    }
}

/// 분자 (Molecule)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Molecule {
    pub name: String,
    pub atoms: Vec<Atom>,
    pub bonds: Vec<Bond>,
}

impl Molecule {
    pub fn atom_count(&self) -> usize {
        self.atoms.len()
    }

    pub fn total_mass_daltons(&self) -> f64 {
        self.atoms.iter().map(|a| a.mass_daltons).sum()
    }

    pub fn center_of_mass(&self) -> Vector3D {
        let total_mass = self.total_mass_daltons();
        let mut com = Vector3D::new(0.0, 0.0, 0.0);

        for atom in &self.atoms {
            com.x += atom.position_nm.x * atom.mass_daltons;
            com.y += atom.position_nm.y * atom.mass_daltons;
            com.z += atom.position_nm.z * atom.mass_daltons;
        }

        Vector3D::new(
            com.x / total_mass,
            com.y / total_mass,
            com.z / total_mass,
        )
    }
}

/// 화학 결합 (Bond)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bond {
    pub atom1_index: usize,
    pub atom2_index: usize,
    pub bond_type: BondType,
    pub energy_kj_per_mol: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BondType {
    Single,
    Double,
    Triple,
    Aromatic,
    Ionic,
    Hydrogen,
}
```

### 분자 조립기 (adapters/assembler.rs)
```rust
use crate::{NanoResult, NanoError, Atom, Vector3D};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MolecularAssembler {
    pub assembler_type: AssemblerType,
    pub workspace_nm: Vector3D,
    pub precision_nm: f64,
    pub active_sites: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssemblerType {
    Mechanosynthesis,
    SelfAssembly,
    GuidedAssembly,
}

impl MolecularAssembler {
    /// 원자 배치 정밀도 검증
    pub fn validate_placement(&self, target: &Vector3D, actual: &Vector3D) -> NanoResult<bool> {
        let error = ((target.x - actual.x).powi(2)
                   + (target.y - actual.y).powi(2)
                   + (target.z - actual.z).powi(2)).sqrt();

        if error <= self.precision_nm {
            Ok(true)
        } else {
            Err(NanoError::PhysicsViolation(
                format!("Placement error {:.3} nm exceeds precision {:.3} nm",
                       error, self.precision_nm)
            ))
        }
    }

    /// 조립 속도 계산 (atoms/second)
    pub fn calculate_assembly_rate(&self, temperature_k: f64) -> f64 {
        // 단순화된 Arrhenius 방정식
        let activation_energy_j = 1e-19; // 0.6 eV
        let pre_exponential_factor = 1e6;
        let k_b = 1.380649e-23; // Boltzmann constant

        pre_exponential_factor * (-activation_energy_j / (k_b * temperature_k)).exp()
    }

    /// 작업 공간 내 위치 검증
    pub fn is_within_workspace(&self, position: &Vector3D) -> bool {
        position.x >= 0.0 && position.x <= self.workspace_nm.x &&
        position.y >= 0.0 && position.y <= self.workspace_nm.y &&
        position.z >= 0.0 && position.z <= self.workspace_nm.z
    }

    /// 예상 조립 시간 계산 (초)
    pub fn estimate_assembly_time(&self, atom_count: usize, temperature_k: f64) -> f64 {
        let rate = self.calculate_assembly_rate(temperature_k);
        atom_count as f64 / rate
    }
}
```

### 나노머신 (adapters/nanomachine.rs)
```rust
use crate::{NanoResult, NanoError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Nanomachine {
    pub machine_type: MachineType,
    pub pdb_id: Option<String>,
    pub total_atoms: usize,
    pub mass_daltons: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MachineType {
    MolecularMotor,
    Enzyme,
    ProteinMachine,
    Synthetic,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MolecularMotor {
    pub energy_source: EnergySource,
    pub rotation_speed_rpm: f64,
    pub torque_pn_nm: f64,
    pub efficiency: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EnergySource {
    ATP,
    ProtonGradient,
    Light,
    Chemical,
}

impl MolecularMotor {
    /// ATP synthase 회전 에너지 계산
    pub fn calculate_rotation_energy(&self) -> NanoResult<f64> {
        if self.efficiency <= 0.0 || self.efficiency > 1.0 {
            return Err(NanoError::InvalidParameter(
                "Efficiency must be between 0 and 1".into()
            ));
        }

        // 회전 에너지 = 토크 × 각도
        let angle_per_step_rad = 2.0 * std::f64::consts::PI / 3.0; // 120도
        let energy_per_step_j = self.torque_pn_nm * 1e-12 * angle_per_step_rad * 1e-9;

        Ok(energy_per_step_j)
    }

    /// 출력 파워 계산 (watts)
    pub fn calculate_power_output(&self) -> f64 {
        let rps = self.rotation_speed_rpm / 60.0;
        let energy_per_rotation = self.torque_pn_nm * 1e-12 * 2.0 * std::f64::consts::PI * 1e-9;
        energy_per_rotation * rps * self.efficiency
    }

    /// ATP 소비율 계산 (molecules/second)
    pub fn calculate_atp_consumption(&self, atp_per_rotation: usize) -> f64 {
        let rps = self.rotation_speed_rpm / 60.0;
        rps * atp_per_rotation as f64
    }
}
```

### 분자 메모리 (adapters/memory.rs)
```rust
use crate::{NanoResult, NanoError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MolecularMemory {
    pub memory_type: MemoryType,
    pub capacity_bytes: u64,
    pub density_bytes_per_nm3: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MemoryType {
    DnaStorage,
    MolecularSwitch,
    QuantumState,
    Conformational,
}

/// DNA 인코딩/디코딩
pub struct DnaEncoder {
    pub encoding_scheme: EncodingScheme,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EncodingScheme {
    Base4,      // A=00, T=01, G=10, C=11
    Base3,      // A=0, T=1, G=2, C는 사용 안 함
    Huffman,    // 가변 길이 인코딩
}

impl DnaEncoder {
    /// 바이트를 DNA 염기서열로 인코딩
    pub fn encode_bytes_to_dna(&self, data: &[u8]) -> NanoResult<String> {
        match self.encoding_scheme {
            EncodingScheme::Base4 => self.encode_base4(data),
            EncodingScheme::Base3 => self.encode_base3(data),
            EncodingScheme::Huffman => Err(NanoError::EncodingError(
                "Huffman encoding not implemented".into()
            )),
        }
    }

    fn encode_base4(&self, data: &[u8]) -> NanoResult<String> {
        let mut dna = String::new();

        for byte in data {
            for i in (0..4).rev() {
                let two_bits = (byte >> (i * 2)) & 0b11;
                let base = match two_bits {
                    0b00 => 'A',
                    0b01 => 'T',
                    0b10 => 'G',
                    0b11 => 'C',
                    _ => unreachable!(),
                };
                dna.push(base);
            }
        }

        Ok(dna)
    }

    fn encode_base3(&self, data: &[u8]) -> NanoResult<String> {
        let mut dna = String::new();
        let bases = ['A', 'T', 'G'];

        for byte in data {
            let mut value = *byte as usize;
            let mut encoded = Vec::new();

            while value > 0 || encoded.is_empty() {
                encoded.push(bases[value % 3]);
                value /= 3;
            }

            encoded.reverse();
            dna.extend(encoded);
        }

        Ok(dna)
    }

    /// DNA 염기서열을 바이트로 디코딩
    pub fn decode_dna_to_bytes(&self, dna: &str) -> NanoResult<Vec<u8>> {
        match self.encoding_scheme {
            EncodingScheme::Base4 => self.decode_base4(dna),
            EncodingScheme::Base3 => self.decode_base3(dna),
            EncodingScheme::Huffman => Err(NanoError::EncodingError(
                "Huffman decoding not implemented".into()
            )),
        }
    }

    fn decode_base4(&self, dna: &str) -> NanoResult<Vec<u8>> {
        if dna.len() % 4 != 0 {
            return Err(NanoError::EncodingError(
                "DNA length must be multiple of 4 for base4 encoding".into()
            ));
        }

        let mut bytes = Vec::new();

        for chunk in dna.chars().collect::<Vec<_>>().chunks(4) {
            let mut byte = 0u8;

            for (i, &base) in chunk.iter().enumerate() {
                let two_bits = match base {
                    'A' => 0b00,
                    'T' => 0b01,
                    'G' => 0b10,
                    'C' => 0b11,
                    _ => return Err(NanoError::EncodingError(
                        format!("Invalid DNA base: {}", base)
                    )),
                };
                byte |= two_bits << ((3 - i) * 2);
            }

            bytes.push(byte);
        }

        Ok(bytes)
    }

    fn decode_base3(&self, dna: &str) -> NanoResult<Vec<u8>> {
        // Base3 디코딩 구현
        Err(NanoError::EncodingError("Base3 decoding not implemented".into()))
    }
}
```

### 나노의학 (adapters/medicine.rs)
```rust
use crate::{NanoResult, NanoError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Nanoparticle {
    pub particle_type: NanoparticleType,
    pub core_diameter_nm: f64,
    pub shell_thickness_nm: f64,
    pub zeta_potential_mv: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NanoparticleType {
    Liposome,
    Polymer,
    Dendrimer,
    GoldNp,
    CarbonNanotube,
}

impl Nanoparticle {
    /// 나노입자 표면적 계산 (nm²)
    pub fn calculate_surface_area(&self) -> f64 {
        let total_radius = self.core_diameter_nm / 2.0 + self.shell_thickness_nm;
        4.0 * std::f64::consts::PI * total_radius.powi(2)
    }

    /// 나노입자 부피 계산 (nm³)
    pub fn calculate_volume(&self) -> f64 {
        let total_radius = self.core_diameter_nm / 2.0 + self.shell_thickness_nm;
        (4.0 / 3.0) * std::f64::consts::PI * total_radius.powi(3)
    }

    /// 약물 적재 용량 계산
    pub fn calculate_drug_capacity(&self, loading_efficiency: f64) -> NanoResult<f64> {
        if loading_efficiency < 0.0 || loading_efficiency > 1.0 {
            return Err(NanoError::InvalidParameter(
                "Loading efficiency must be between 0 and 1".into()
            ));
        }

        let core_volume = (4.0 / 3.0) * std::f64::consts::PI * (self.core_diameter_nm / 2.0).powi(3);
        Ok(core_volume * loading_efficiency)
    }

    /// Stokes-Einstein 방정식으로 확산 계수 계산 (m²/s)
    pub fn calculate_diffusion_coefficient(&self, temperature_k: f64, viscosity_pa_s: f64) -> f64 {
        let k_b = 1.380649e-23; // Boltzmann constant
        let total_radius_m = (self.core_diameter_nm / 2.0 + self.shell_thickness_nm) * 1e-9;

        k_b * temperature_k / (6.0 * std::f64::consts::PI * viscosity_pa_s * total_radius_m)
    }
}
```

---

## 📋 Cargo.toml

```toml
[package]
name = "wia-nano"
version = "1.0.0"
edition = "2021"
description = "WIA Nanotechnology Standard - Rust SDK"
license = "MIT"
repository = "https://github.com/WIA-Official/wia-standards"
keywords = ["nanotechnology", "nanomedicine", "molecular", "simulation", "science"]
categories = ["science", "simulation"]

[dependencies]
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
thiserror = "1"
async-trait = "0.1"
chrono = { version = "0.4", features = ["serde"] }
uuid = { version = "1", features = ["v4", "serde"] }

# 과학 계산
nalgebra = "0.32"
ndarray = "0.15"

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
approx = "0.5"  # 부동소수점 비교
```

---

## 🚀 사용 예시

### Basic Usage
```rust
use wia_nano::prelude::*;

fn main() -> NanoResult<()> {
    // DNA 메모리 인코딩
    let encoder = DnaEncoder {
        encoding_scheme: EncodingScheme::Base4,
    };

    let data = b"Hello, Nano!";
    let dna = encoder.encode_bytes_to_dna(data)?;
    println!("DNA sequence: {}", dna);

    let decoded = encoder.decode_dna_to_bytes(&dna)?;
    println!("Decoded: {}", String::from_utf8_lossy(&decoded));

    // 분자 조립기 시뮬레이션
    let assembler = MolecularAssembler {
        assembler_type: AssemblerType::Mechanosynthesis,
        workspace_nm: Vector3D::new(100.0, 100.0, 100.0),
        precision_nm: 0.1,
        active_sites: 6,
    };

    let assembly_time = assembler.estimate_assembly_time(10000, 298.15);
    println!("Assembly time: {:.2} seconds", assembly_time);

    // 나노입자 특성 계산
    let nanoparticle = Nanoparticle {
        particle_type: NanoparticleType::Liposome,
        core_diameter_nm: 100.0,
        shell_thickness_nm: 5.0,
        zeta_potential_mv: -20.0,
    };

    let surface_area = nanoparticle.calculate_surface_area();
    println!("Surface area: {:.2} nm²", surface_area);

    Ok(())
}
```

---

## 📁 산출물 목록

```
/api/rust/Cargo.toml
/api/rust/src/lib.rs
/api/rust/src/types.rs
/api/rust/src/error.rs
/api/rust/src/prelude.rs
/api/rust/src/core/mod.rs
/api/rust/src/core/project.rs
/api/rust/src/core/calculator.rs
/api/rust/src/core/simulator.rs
/api/rust/src/physics/mod.rs
/api/rust/src/physics/constants.rs
/api/rust/src/physics/forces.rs
/api/rust/src/physics/thermodynamics.rs
/api/rust/src/adapters/mod.rs
/api/rust/src/adapters/assembler.rs
/api/rust/src/adapters/nanomachine.rs
/api/rust/src/adapters/memory.rs
/api/rust/src/adapters/medicine.rs
/api/rust/src/adapters/robotics.rs
/api/rust/src/adapters/quantum_dots.rs
/api/rust/tests/integration_test.rs
/api/rust/examples/basic_usage.rs
/api/rust/README.md
```

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성
□ Error 타입 정의
□ 물리 상수 정의
□ 기본 타입 정의 (Atom, Molecule, Vector3D)
□ 6개 기술 어댑터 구현
  □ MolecularAssembler (조립 시뮬레이션)
  □ Nanomachine (동역학 계산)
  □ MolecularMemory (DNA 인코딩/디코딩)
  □ Nanomedicine (나노입자 특성)
  □ Nanorobotics (로봇 제어)
  □ QuantumDots (광학 특성)
□ 핵심 계산 함수 구현
□ 물리 검증 함수 구현
□ 단위 테스트 작성
□ 통합 테스트 작성
□ 예제 코드 작성
□ cargo test 통과
□ cargo clippy 경고 없음
□ README 업데이트
```

---

## 🔄 작업 순서

```
1. Cargo.toml 생성
   ↓
2. error.rs - 에러 타입 정의
   ↓
3. physics/constants.rs - 물리 상수 정의
   ↓
4. types.rs - 기본 타입 정의
   ↓
5. adapters/assembler.rs - 분자 조립기
   ↓
6. adapters/nanomachine.rs - 나노머신
   ↓
7. adapters/memory.rs - 분자 메모리
   ↓
8. adapters/medicine.rs - 나노의학
   ↓
9. adapters/robotics.rs - 나노로봇
   ↓
10. adapters/quantum_dots.rs - 양자점
   ↓
11. core/ 모듈 통합
   ↓
12. 테스트 작성 및 실행
   ↓
13. 예제 코드 작성
   ↓
14. 완료 체크리스트 확인
   ↓
15. Phase 3 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 스키마와 1:1 대응되는 타입 정의
✅ 모든 계산에 물리 단위 명시
✅ 불가능한 값에 대한 검증 (NanoError 반환)
✅ 과학적으로 정확한 공식 사용
✅ f64 사용 (과학 계산 정밀도)
✅ serde 지원으로 JSON 변환 가능
```

### DON'T (하지 말 것)

```
❌ 하드코딩된 물리 상수 (상수는 const로 정의)
❌ panic! 사용 (Result 반환)
❌ unwrap() 남용 (? 연산자 사용)
❌ 부동소수점 직접 비교 (approx 크레이트 사용)
```

---

## 🚀 작업 시작

이제 Phase 2 작업을 시작하세요.

첫 번째 단계: **Cargo.toml 생성 후 error.rs 구현**

```bash
cargo new --lib wia-nano
```

화이팅! 🦀⚛️

---

<div align="center">

**Phase 2 of 4**

Rust API Implementation

🦀 Safe, Fast, Precise 🦀

弘益人間 - Benefit All Humanity

</div>
