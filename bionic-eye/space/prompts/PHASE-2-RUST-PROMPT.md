# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA Space
**Phase**: 2 of 4
**Language**: **Rust** (Primary)
**목표**: Rust 기반 고성능 우주 기술 API 구현
**난이도**: ★★★★☆
**예상 작업량**: Rust 라이브러리 + 테스트 + 예제

---

## 🎯 Phase 2 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의했다.

 이제 이 데이터를 프로그래밍 방식으로 어떻게 다룰 것인가?

 - 다이슨 구체 에너지 수집량 계산?
 - 화성 테라포밍 진행률 시뮬레이션?
 - 워프 드라이브 에너지 요구량 산출?

 모든 계산을 표준 API로 제공할 수 있을까?"
```

### 목표
```
우주 기술 데이터를 처리하는 Rust API 구현

- 데이터 타입 정의 (Phase 1 스키마 기반)
- 핵심 계산 함수 구현
- 시뮬레이션 어댑터
- WASM/Python 바인딩 지원 준비
```

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (복잡한 물리 시뮬레이션)
2. 안전: 메모리 안전 보장
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
│   ├── lib.rs               # 메인 라이브러리
│   ├── types.rs             # 타입 정의
│   ├── error.rs             # 에러 타입
│   ├── core/
│   │   ├── mod.rs
│   │   ├── project.rs       # 프로젝트 관리
│   │   ├── calculator.rs    # 핵심 계산
│   │   └── simulator.rs     # 시뮬레이션
│   ├── adapters/
│   │   ├── mod.rs
│   │   ├── dyson.rs         # 다이슨 구체
│   │   ├── mars.rs          # 화성 테라포밍
│   │   ├── warp.rs          # 워프 드라이브
│   │   ├── elevator.rs      # 우주 엘리베이터
│   │   ├── asteroid.rs      # 소행성 채굴
│   │   └── interstellar.rs  # 성간 여행
│   └── prelude.rs           # 편의 re-exports
├── tests/
│   └── integration_test.rs
└── examples/
    ├── basic_usage.rs
    ├── dyson_calculator.rs
    └── mars_simulation.rs
```

---

## 🔧 핵심 구현 코드

### Error 타입 (error.rs)
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum SpaceError {
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("Calculation overflow: {0}")]
    CalculationOverflow(String),

    #[error("Physical constraint violated: {0}")]
    PhysicsViolation(String),

    #[error("Resource not found: {0}")]
    ResourceNotFound(String),

    #[error("Simulation error: {0}")]
    SimulationError(String),

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

pub type SpaceResult<T> = std::result::Result<T, SpaceError>;
```

### 기본 타입 정의 (types.rs)
```rust
use serde::{Deserialize, Serialize};

/// 기술 유형
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TechnologyType {
    DysonSphere,
    MarsTerraforming,
    WarpDrive,
    SpaceElevator,
    AsteroidMining,
    InterstellarTravel,
}

/// 기술 성숙도 레벨 (NASA TRL)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TRL(pub u8);

impl TRL {
    pub fn new(level: u8) -> SpaceResult<Self> {
        if level >= 1 && level <= 9 {
            Ok(TRL(level))
        } else {
            Err(SpaceError::InvalidParameter(
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

/// 궤도 파라미터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrbitalParameters {
    pub semi_major_axis_au: f64,
    pub eccentricity: f64,
    pub inclination_deg: f64,
    pub period_days: Option<f64>,
}

impl OrbitalParameters {
    /// 케플러 제3법칙으로 공전 주기 계산
    pub fn calculate_period(&self) -> f64 {
        // T^2 = a^3 (AU와 년 단위)
        let period_years = (self.semi_major_axis_au.powi(3)).sqrt();
        period_years * 365.25
    }
}

/// 에너지 단위 변환
#[derive(Debug, Clone, Copy)]
pub struct Energy {
    pub joules: f64,
}

impl Energy {
    pub fn from_joules(j: f64) -> Self {
        Self { joules: j }
    }

    pub fn from_watts_hours(wh: f64) -> Self {
        Self { joules: wh * 3600.0 }
    }

    pub fn to_kwh(&self) -> f64 {
        self.joules / 3_600_000.0
    }

    pub fn to_solar_luminosity(&self) -> f64 {
        self.joules / 3.828e26
    }
}
```

### 다이슨 구체 계산 (adapters/dyson.rs)
```rust
use crate::{SpaceResult, SpaceError, Energy};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DysonSphereSpec {
    pub dyson_type: DysonType,
    pub star_luminosity_watts: f64,
    pub radius_au: f64,
    pub collection_efficiency: f64,
    pub coverage_fraction: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DysonType {
    Swarm,
    Bubble,
    Shell,
}

impl DysonSphereSpec {
    /// 수집 가능한 총 에너지 계산
    pub fn calculate_collected_energy(&self) -> SpaceResult<Energy> {
        if self.collection_efficiency <= 0.0 || self.collection_efficiency > 1.0 {
            return Err(SpaceError::InvalidParameter(
                "Efficiency must be between 0 and 1".into()
            ));
        }

        let collected = self.star_luminosity_watts
            * self.coverage_fraction
            * self.collection_efficiency;

        Ok(Energy::from_joules(collected))
    }

    /// 필요한 수집기 면적 계산 (km²)
    pub fn calculate_collector_area_km2(&self) -> f64 {
        // 구체 표면적: 4πr²
        let radius_km = self.radius_au * 1.496e8; // AU to km
        let sphere_area = 4.0 * std::f64::consts::PI * radius_km.powi(2);
        sphere_area * self.coverage_fraction
    }

    /// 건설에 필요한 물질 질량 추정 (kg)
    pub fn estimate_material_mass(&self) -> f64 {
        // 대략적 추정: 1 km² 당 1000 kg
        self.calculate_collector_area_km2() * 1000.0
    }
}
```

### 화성 테라포밍 시뮬레이션 (adapters/mars.rs)
```rust
use crate::{SpaceResult, SpaceError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarsConditions {
    pub avg_temp_celsius: f64,
    pub pressure_kpa: f64,
    pub co2_percent: f64,
    pub o2_percent: f64,
    pub n2_percent: f64,
}

impl MarsConditions {
    /// 현재 화성 조건
    pub fn current() -> Self {
        Self {
            avg_temp_celsius: -60.0,
            pressure_kpa: 0.636,
            co2_percent: 95.3,
            o2_percent: 0.13,
            n2_percent: 2.7,
        }
    }

    /// 지구와 유사한 목표 조건
    pub fn earth_like_target() -> Self {
        Self {
            avg_temp_celsius: 15.0,
            pressure_kpa: 101.3,
            co2_percent: 0.04,
            o2_percent: 21.0,
            n2_percent: 78.0,
        }
    }

    /// 테라포밍 진행률 계산 (0.0 ~ 1.0)
    pub fn calculate_progress(&self, target: &Self) -> f64 {
        let current = MarsConditions::current();

        let temp_progress = (self.avg_temp_celsius - current.avg_temp_celsius)
            / (target.avg_temp_celsius - current.avg_temp_celsius);
        let pressure_progress = (self.pressure_kpa - current.pressure_kpa)
            / (target.pressure_kpa - current.pressure_kpa);
        let o2_progress = (self.o2_percent - current.o2_percent)
            / (target.o2_percent - current.o2_percent);

        // 가중 평균 (온도 40%, 기압 30%, 산소 30%)
        (temp_progress * 0.4 + pressure_progress * 0.3 + o2_progress * 0.3)
            .clamp(0.0, 1.0)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerraformingMethod {
    pub name: String,
    pub energy_required_joules: f64,
    pub duration_years: f64,
    pub effect_temp_delta: f64,
    pub effect_pressure_delta: f64,
}
```

### 워프 드라이브 계산 (adapters/warp.rs)
```rust
use crate::{SpaceResult, SpaceError};
use serde::{Deserialize, Serialize};

pub const SPEED_OF_LIGHT: f64 = 299_792_458.0; // m/s

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WarpDriveSpec {
    pub warp_factor: f64,          // c 배수
    pub bubble_radius_meters: f64,
    pub exotic_matter_kg: f64,     // 음수 = 음에너지
}

impl WarpDriveSpec {
    /// 실제 속도 계산 (m/s)
    pub fn calculate_velocity(&self) -> f64 {
        SPEED_OF_LIGHT * self.warp_factor
    }

    /// 목적지까지 이동 시간 계산 (초)
    pub fn calculate_travel_time(&self, distance_ly: f64) -> f64 {
        let distance_m = distance_ly * 9.461e15; // 광년 -> 미터
        distance_m / self.calculate_velocity()
    }

    /// 알쿠비에레 메트릭 에너지 요구량 (대략적)
    pub fn estimate_energy_requirement(&self) -> SpaceResult<f64> {
        if self.warp_factor <= 0.0 {
            return Err(SpaceError::InvalidParameter(
                "Warp factor must be positive".into()
            ));
        }

        // 매우 단순화된 추정
        // 실제로는 음에너지가 필요하며 정확한 계산은 매우 복잡
        let base_energy = 1e18; // 1 엑사줄 기준
        Ok(base_energy * self.warp_factor.powi(3) * self.bubble_radius_meters.powi(2))
    }

    /// 물리적 가능성 검증
    pub fn validate_physics(&self) -> SpaceResult<()> {
        if self.bubble_radius_meters <= 0.0 {
            return Err(SpaceError::PhysicsViolation(
                "Bubble radius must be positive".into()
            ));
        }

        if self.exotic_matter_kg >= 0.0 {
            return Err(SpaceError::PhysicsViolation(
                "Exotic matter must have negative mass (negative energy)".into()
            ));
        }

        Ok(())
    }
}
```

### 소행성 채굴 (adapters/asteroid.rs)
```rust
use crate::{OrbitalParameters, SpaceResult};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TargetAsteroid {
    pub name: String,
    pub asteroid_type: AsteroidType,
    pub diameter_km: f64,
    pub mass_kg: f64,
    pub orbit: OrbitalParameters,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AsteroidType {
    CType,  // 탄소질
    SType,  // 규산염
    MType,  // 금속
    XType,  // 기타
}

impl TargetAsteroid {
    /// 추정 자원 매장량 계산
    pub fn estimate_resources(&self) -> HashMap<String, f64> {
        let mut resources = HashMap::new();

        match self.asteroid_type {
            AsteroidType::MType => {
                resources.insert("iron_kg".into(), self.mass_kg * 0.90);
                resources.insert("nickel_kg".into(), self.mass_kg * 0.08);
                resources.insert("cobalt_kg".into(), self.mass_kg * 0.005);
                resources.insert("platinum_kg".into(), self.mass_kg * 1e-6);
                resources.insert("gold_kg".into(), self.mass_kg * 1e-7);
            }
            AsteroidType::CType => {
                resources.insert("water_kg".into(), self.mass_kg * 0.10);
                resources.insert("carbon_kg".into(), self.mass_kg * 0.05);
                resources.insert("organic_kg".into(), self.mass_kg * 0.02);
            }
            AsteroidType::SType => {
                resources.insert("silicate_kg".into(), self.mass_kg * 0.80);
                resources.insert("iron_kg".into(), self.mass_kg * 0.15);
            }
            AsteroidType::XType => {
                resources.insert("unknown_kg".into(), self.mass_kg);
            }
        }

        resources
    }

    /// 델타-V 요구량 추정 (km/s)
    pub fn estimate_delta_v_from_earth(&self) -> f64 {
        // 대략적인 추정 (실제로는 훨씬 복잡)
        let base_dv = 4.0; // LEO 탈출
        let transfer_dv = (self.orbit.semi_major_axis_au - 1.0).abs() * 3.0;
        base_dv + transfer_dv
    }
}
```

### 성간 여행 (adapters/interstellar.rs)
```rust
use crate::{SpaceResult, SpaceError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterstellarMission {
    pub destination_name: String,
    pub distance_ly: f64,
    pub spacecraft_mass_kg: f64,
    pub propulsion: PropulsionType,
    pub cruise_velocity_c: f64,  // 광속 대비 비율
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PropulsionType {
    LightSail,
    NuclearPulse,
    Fusion,
    Antimatter,
    BussardRamjet,
}

impl InterstellarMission {
    /// 여행 시간 계산 (년)
    pub fn calculate_travel_time_years(&self) -> f64 {
        if self.cruise_velocity_c <= 0.0 {
            return f64::INFINITY;
        }
        self.distance_ly / self.cruise_velocity_c
    }

    /// 상대론적 시간 지연 계산 (우주선 내 시간)
    pub fn calculate_proper_time_years(&self) -> f64 {
        let v = self.cruise_velocity_c;
        let gamma = 1.0 / (1.0 - v * v).sqrt();
        self.calculate_travel_time_years() / gamma
    }

    /// 에너지 요구량 추정 (줄)
    pub fn estimate_energy_requirement(&self) -> SpaceResult<f64> {
        let c = 299_792_458.0;
        let v = self.cruise_velocity_c * c;

        // 상대론적 운동 에너지
        let gamma = 1.0 / (1.0 - self.cruise_velocity_c.powi(2)).sqrt();
        let kinetic_energy = self.spacecraft_mass_kg * c.powi(2) * (gamma - 1.0);

        Ok(kinetic_energy)
    }

    /// 추진 방식별 효율 계산
    pub fn propulsion_efficiency(&self) -> f64 {
        match self.propulsion {
            PropulsionType::LightSail => 0.99,      // 외부 에너지 사용
            PropulsionType::NuclearPulse => 0.001,  // Orion 프로젝트 기준
            PropulsionType::Fusion => 0.01,        // D-He3 기준
            PropulsionType::Antimatter => 0.5,     // 이론적 최대
            PropulsionType::BussardRamjet => 0.1,  // 성간 물질 수집
        }
    }
}
```

---

## 📋 Cargo.toml

```toml
[package]
name = "wia-space"
version = "1.0.0"
edition = "2021"
description = "WIA Space Technology Standard - Rust SDK"
license = "MIT"
repository = "https://github.com/WIA-Official/wia-standards"
keywords = ["space", "aerospace", "simulation", "dyson", "terraform"]
categories = ["science", "simulation"]

[dependencies]
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
thiserror = "1"
async-trait = "0.1"
chrono = { version = "0.4", features = ["serde"] }
uuid = { version = "1", features = ["v4", "serde"] }

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
use wia_space::prelude::*;

fn main() -> SpaceResult<()> {
    // 다이슨 스웜 계산
    let dyson = DysonSphereSpec {
        dyson_type: DysonType::Swarm,
        star_luminosity_watts: 3.828e26,  // 태양
        radius_au: 1.0,
        collection_efficiency: 0.85,
        coverage_fraction: 0.01,  // 1% 커버리지
    };

    let energy = dyson.calculate_collected_energy()?;
    println!("수집 에너지: {:.2e} W", energy.joules);
    println!("필요 면적: {:.2e} km²", dyson.calculate_collector_area_km2());

    // 화성 테라포밍 진행률
    let current_mars = MarsConditions {
        avg_temp_celsius: -40.0,  // 약간 따뜻해짐
        pressure_kpa: 5.0,        // 기압 상승
        o2_percent: 2.0,          // 산소 증가
        ..MarsConditions::current()
    };

    let target = MarsConditions::earth_like_target();
    let progress = current_mars.calculate_progress(&target);
    println!("테라포밍 진행률: {:.1}%", progress * 100.0);

    // 성간 미션 계획
    let mission = InterstellarMission {
        destination_name: "Proxima Centauri b".into(),
        distance_ly: 4.24,
        spacecraft_mass_kg: 1.0,  // 1 그램 탐사선
        propulsion: PropulsionType::LightSail,
        cruise_velocity_c: 0.2,
    };

    println!("여행 시간: {:.1} 년", mission.calculate_travel_time_years());
    println!("우주선 내 시간: {:.1} 년", mission.calculate_proper_time_years());

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
/api/rust/src/adapters/mod.rs
/api/rust/src/adapters/dyson.rs
/api/rust/src/adapters/mars.rs
/api/rust/src/adapters/warp.rs
/api/rust/src/adapters/elevator.rs
/api/rust/src/adapters/asteroid.rs
/api/rust/src/adapters/interstellar.rs
/api/rust/tests/integration_test.rs
/api/rust/examples/basic_usage.rs
/api/rust/README.md
```

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성
□ Error 타입 정의
□ 기본 타입 정의 (TRL, OrbitalParameters, Energy)
□ 6개 기술 어댑터 구현
  □ DysonSphere (에너지 수집 계산)
  □ MarsTerraforming (환경 시뮬레이션)
  □ WarpDrive (물리 계산)
  □ SpaceElevator (구조 계산)
  □ AsteroidMining (자원 추정)
  □ InterstellarTravel (궤적 계산)
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
3. types.rs - 기본 타입 정의
   ↓
4. adapters/dyson.rs - 다이슨 구체
   ↓
5. adapters/mars.rs - 화성 테라포밍
   ↓
6. adapters/warp.rs - 워프 드라이브
   ↓
7. adapters/elevator.rs - 우주 엘리베이터
   ↓
8. adapters/asteroid.rs - 소행성 채굴
   ↓
9. adapters/interstellar.rs - 성간 여행
   ↓
10. core/ 모듈 통합
   ↓
11. 테스트 작성 및 실행
   ↓
12. 예제 코드 작성
   ↓
13. 완료 체크리스트 확인
   ↓
14. Phase 3 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 스키마와 1:1 대응되는 타입 정의
✅ 모든 계산에 물리 단위 명시
✅ 불가능한 값에 대한 검증 (SpaceError 반환)
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

## 🔗 참고 자료

- **Dyson Sphere**: Freeman Dyson, "Search for Artificial Stellar Sources of Infrared Radiation" (1960)
- **Alcubierre Drive**: Miguel Alcubierre, "The warp drive: hyper-fast travel within general relativity" (1994)
- **Space Elevator**: Bradley Edwards, "The Space Elevator" (2003)
- **Asteroid Mining**: John Lewis, "Mining the Sky" (1997)

---

## 🚀 작업 시작

이제 Phase 2 작업을 시작하세요.

첫 번째 단계: **Cargo.toml 생성 후 error.rs 구현**

```bash
cargo new --lib wia-space
```

화이팅! 🚀🦀

---

<div align="center">

**Phase 2 of 4**

Rust API Implementation

🦀 Safe, Fast, Concurrent 🦀

</div>
