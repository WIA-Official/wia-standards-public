# Phase 1 사전 조사 결과
# Phase 1 Research Findings

---

**작성일**: 2025년 12월 14일
**작성**: Claude Code (Opus 4.5)
**목적**: Material Science 분야별 데이터 형식 조사 및 표준 설계 방향 도출

---

## 목차 (Table of Contents)

1. [Room Temperature Superconductor (상온 초전도체)](#1-room-temperature-superconductor-상온-초전도체)
2. [Metamaterial (메타물질)](#2-metamaterial-메타물질)
3. [Programmable Matter (프로그래머블 물질)](#3-programmable-matter-프로그래머블-물질)
4. [Holographic Data Storage (홀로그래픽 저장)](#4-holographic-data-storage-홀로그래픽-저장)
5. [Memristor (멤리스터)](#5-memristor-멤리스터)
6. [Topological Insulator (토폴로지 절연체)](#6-topological-insulator-토폴로지-절연체)
7. [기존 표준 조사](#7-기존-표준-조사)
8. [공통점 분석](#8-공통점-분석)
9. [결론 및 설계 방향](#9-결론-및-설계-방향)

---

## 1. Room Temperature Superconductor (상온 초전도체)

### 1.1 현재 산업 현황

**정의**: 상온(약 25°C/300K) 및 상압에서 전기 저항이 0인 초전도 현상을 나타내는 물질

#### LK-99 사례 (2023-2024)

- **발표**: 2023년 7월, 한국 연구팀이 Cu-doped lead apatite (LK-99) 발표
- **주장**: 임계온도 127°C (400K), 상압에서 초전도성
- **검증 결과**: 2023년 8-11월 전 세계 복제 시도 실패
- **원인 분석**: Cu₂S (황화구리) 불순물의 구조적 상전이로 인한 저항 급락 현상
- **현재 상태**: 과학적 합의는 "LK-99는 초전도체가 아님"

#### 실제 고온 초전도체 현황

| 물질 | 임계온도 (Tc) | 압력 조건 | 년도 |
|------|--------------|----------|------|
| YBCO | 93 K (-180°C) | 상압 | 1987 |
| BSCCO | 110 K (-163°C) | 상압 | 1988 |
| H₃S | 203 K (-70°C) | 155 GPa | 2015 |
| LaH₁₀ | 250 K (-23°C) | 170 GPa | 2019 |
| CSH | 288 K (15°C) | 267 GPa | 2020* |

*CSH는 논문 철회됨

### 1.2 데이터 측정 파라미터

```json
{
  "critical_temperature_K": 93.0,
  "critical_pressure_GPa": 0.0,
  "critical_current_density_A_cm2": 1e6,
  "critical_magnetic_field_T": 100.0,
  "meissner_effect": true,
  "resistivity_ohm_m": 0.0,
  "sample_composition": "YBa2Cu3O7-x",
  "measurement_method": "four_probe",
  "magnetic_susceptibility": -1.0
}
```

### 1.3 주요 측정 기법

1. **4-Point Probe Method**: 저항 측정
2. **SQUID Magnetometry**: 자기 특성 측정
3. **AC Susceptibility**: Meissner 효과 확인
4. **Specific Heat**: 상전이 확인

### 1.4 주요 기관 및 기업

- 미국: Argonne National Lab, NREL
- 유럽: CERN, Max Planck Institute
- 일본: RIKEN, NIMS
- 한국: 퀀텀에너지연구소, KAIST

---

## 2. Metamaterial (메타물질)

### 2.1 현재 산업 현황

**정의**: 자연에서 발견되지 않는 전자기적/음향적/역학적 특성을 가지도록 인공적으로 설계된 구조재

#### 시장 규모

- 2025년: USD 2,087 million
- 2032년: USD 18,145 million (예상)
- CAGR: 36.2%

#### 주요 응용 분야

| 분야 | 응용 | 시장 비중 |
|------|------|----------|
| **전자기 메타물질** | 5G/6G 안테나, 레이더, 위성통신 | 최대 |
| **음향 메타물질** | 방음벽, 초음파 이미징, 소나 | 성장 중 |
| **역학 메타물질** | 충격흡수, 건축, 항공우주 | 신규 |

### 2.2 전자기 메타물질 데이터 형식

```json
{
  "type": "electromagnetic",
  "structure": {
    "unit_cell": "split_ring_resonator",
    "period_um": 100.0,
    "dimensions": {
      "outer_radius_um": 45.0,
      "gap_um": 5.0,
      "track_width_um": 10.0
    }
  },
  "properties": {
    "permittivity_real": -2.5,
    "permittivity_imag": 0.1,
    "permeability_real": -1.2,
    "permeability_imag": 0.05,
    "refractive_index": -1.73,
    "operating_frequency_GHz": 10.0,
    "bandwidth_GHz": 2.0,
    "absorption_percent": 95.0
  }
}
```

### 2.3 음향 메타물질 데이터 형식

```json
{
  "type": "acoustic",
  "structure": {
    "unit_cell": "helmholtz_resonator",
    "period_mm": 50.0,
    "cavity_volume_mm3": 1000.0,
    "neck_length_mm": 5.0,
    "neck_radius_mm": 2.0
  },
  "properties": {
    "effective_density_kg_m3": -1500.0,
    "effective_bulk_modulus_Pa": -2.2e9,
    "operating_frequency_Hz": 1000.0,
    "transmission_loss_dB": 40.0,
    "sound_absorption_coefficient": 0.95
  }
}
```

### 2.4 AI 기반 설계 동향

- Deep Learning을 통한 split-ring resonator 최적화
- 마이크로파 주파수에서 90% 이상 흡수율 달성
- 다중 목표 최적화 (대역폭 + 흡수율)

### 2.5 주요 기업

- **Kymeta**: 위성 통신 안테나
- **Metamaterial Inc.**: 광학 메타물질
- **Echodyne**: 레이더 시스템
- **Metawave**: 자율주행 레이더

---

## 3. Programmable Matter (프로그래머블 물질)

### 3.1 현재 연구 현황

**정의**: 물리적 특성(형상, 밀도, 전도성, 광학적 특성 등)을 프로그래밍 방식으로 변경할 수 있는 물질

#### Claytronics 프로젝트 (Carnegie Mellon University)

- **시작**: 2002년, Seth Goldstein & Todd Mowry
- **핵심 개념**: Catom (Claytronic Atom) - 나노/마이크로 스케일 로봇 모듈
- **목표**: 수백만 개의 협력 로봇 모듈이 임의의 형상과 질감을 재현

#### Catom 특성

| 속성 | 현재 수준 | 목표 수준 |
|------|----------|----------|
| 크기 | 밀리미터 | 마이크로미터~나노미터 |
| 작동 방식 | 정전기 구동 | 자기조립 |
| 이동 부품 | 없음 | 없음 |
| 제조 | 개별 제작 | 배치 리소그래피 |

### 3.2 데이터 형식 설계

```json
{
  "module_type": "catom",
  "module_id": "catom_001",
  "state": {
    "position": {"x": 10.5, "y": 20.3, "z": 5.0},
    "orientation": {"roll": 0, "pitch": 0, "yaw": 45},
    "bonds": ["catom_002", "catom_003", "catom_007"],
    "power_state": "active",
    "energy_level_percent": 85.0
  },
  "capabilities": {
    "locomotion": true,
    "bonding": true,
    "communication": true,
    "computation": true,
    "color_change": false
  },
  "physical_properties": {
    "diameter_um": 1000.0,
    "mass_ug": 50.0,
    "bonding_force_uN": 100.0,
    "max_speed_um_s": 500.0
  }
}
```

### 3.3 집합체 (Ensemble) 데이터 형식

```json
{
  "ensemble_id": "shape_001",
  "target_shape": "sphere",
  "modules": ["catom_001", "catom_002", "..."],
  "total_count": 10000,
  "current_configuration": {
    "bounding_box": {
      "min": {"x": 0, "y": 0, "z": 0},
      "max": {"x": 100, "y": 100, "z": 100}
    },
    "shape_accuracy_percent": 95.0,
    "connectivity_graph": "base64_encoded_adjacency_matrix"
  },
  "reconfiguration": {
    "in_progress": false,
    "target_shape": null,
    "estimated_time_s": 0
  }
}
```

### 3.4 잠재적 응용 분야

- **건축**: 날씨/용도에 따라 변형되는 건축 자재
- **패션**: 환경/개인 취향에 맞게 변형되는 의류
- **의료**: 실시간 조정 가능한 임플란트, 약물 전달
- **제조**: 자가 조립/재구성 기계

---

## 4. Holographic Data Storage (홀로그래픽 저장)

### 4.1 현재 산업 현황

**정의**: 레이저 간섭 패턴을 이용해 3차원 볼륨에 데이터를 기록하는 저장 기술

#### 시장 전망

- 2025년: USD 294 million
- CAGR: 21.6% (2025-2033)
- 상용화: 2025-2027년 (특수 용도), 2028-2030년 (기업용)

#### Microsoft Project HSD

- **협력**: Microsoft Research Cambridge + Microsoft Azure
- **목표**: 클라우드 스케일 스토리지용 홀로그래픽 기술
- **특징**: 기계적 움직임 없음, 고내구성, 비용 효율적

### 4.2 기술적 특성

| 특성 | HDS | HDD | SSD |
|------|-----|-----|-----|
| 기록 방식 | 볼륨 | 표면 | 셀 |
| 병렬 R/W | 수백만 비트 | 1비트 | 수천 비트 |
| 수명 | 50년+ | 5-10년 | 5-10년 |
| 에너지 효율 | 높음 (목표) | 중간 | 높음 |
| WORM | 지원 | 미지원 | 미지원 |

### 4.3 데이터 형식

```json
{
  "medium": {
    "type": "photorefractive_crystal",
    "material": "LiNbO3",
    "dimensions_mm": {"x": 10, "y": 10, "z": 5},
    "refractive_index": 2.286
  },
  "recording": {
    "wavelength_nm": 532.0,
    "reference_angle_deg": 45.0,
    "multiplexing_method": "angular",
    "hologram_count": 1000,
    "page_size_bits": 1048576
  },
  "data": {
    "hologram_id": "holo_0001",
    "reference_beam_angle_deg": 45.0,
    "signal_beam_pattern": "2d_bitmap",
    "error_correction": "ldpc",
    "checksum": "sha256_hash"
  },
  "performance": {
    "capacity_GB": 1000.0,
    "read_speed_Mbps": 1000.0,
    "write_speed_Mbps": 500.0,
    "retention_years": 50
  }
}
```

### 4.4 주요 도전 과제

1. **에너지 효율**: 1-2 orders of magnitude 개선 필요
2. **표준화 부재**: 업계 표준 미확립
3. **생산 비용**: 기존 기술 대비 높음
4. **스케일업**: 대량 생산 기술 개발 필요

---

## 5. Memristor (멤리스터)

### 5.1 현재 산업 현황

**정의**: 전류 이력에 따라 저항이 변하는 비휘발성 메모리 소자 (Memory + Resistor)

#### 역사

- 1971년: Leon O. Chua가 이론적 제안 (제4의 기본 회로 소자)
- 2008년: HP Labs에서 TiO₂ 기반 최초 실험적 구현
- 현재: 뉴로모픽 컴퓨팅의 핵심 소자로 각광

#### 시장 전망

- 2024년: 80% 이상 성장
- 2027년: USD 13.5 billion (예상)

### 5.2 응용 분야

| 분야 | 응용 | 장점 |
|------|------|------|
| **메모리** | ReRAM, NVRAM | 고밀도, 비휘발성 |
| **뉴로모픽** | 인공 시냅스 | 아날로그 가중치, 병렬 연산 |
| **암호화** | 물리적 PUF | 고유 특성 기반 보안 |
| **센싱** | 신호 처리 | 저전력 에지 컴퓨팅 |

### 5.3 데이터 형식

```json
{
  "device": {
    "type": "memristor",
    "material": "TiO2",
    "structure": "metal_insulator_metal",
    "dimensions": {
      "length_nm": 50.0,
      "width_nm": 50.0,
      "thickness_nm": 10.0
    }
  },
  "electrical_properties": {
    "resistance_high_ohm": 1e6,
    "resistance_low_ohm": 1e3,
    "on_off_ratio": 1000,
    "set_voltage_V": 1.0,
    "reset_voltage_V": -0.8,
    "read_voltage_V": 0.1,
    "endurance_cycles": 1e12,
    "retention_s": 3.15e7
  },
  "state": {
    "current_resistance_ohm": 5e4,
    "conductance_S": 2e-5,
    "state_variable": 0.5,
    "last_operation": "read"
  },
  "neuromorphic": {
    "synaptic_weight": 0.5,
    "plasticity": "stdp",
    "learning_rate": 0.01,
    "analog_states": 128
  }
}
```

### 5.4 뉴로모픽 컴퓨팅 아키텍처

```json
{
  "array_type": "crossbar",
  "dimensions": {
    "rows": 1024,
    "columns": 1024
  },
  "operation": {
    "mode": "vector_matrix_multiply",
    "input_vector": "row_voltages",
    "output_vector": "column_currents",
    "precision_bits": 8
  },
  "power": {
    "static_power_uW": 100.0,
    "dynamic_power_uW_per_op": 0.1,
    "energy_efficiency_TOPS_W": 100.0
  }
}
```

### 5.5 주요 기업/기관

- **HP Labs**: 원천 기술 개발
- **Intel**: Loihi 뉴로모픽 칩
- **IBM**: TrueNorth, 신경망 가속기
- **Samsung**: ReRAM 상용화
- **Knowm**: 상용 멤리스터 제품

---

## 6. Topological Insulator (토폴로지 절연체)

### 6.1 현재 산업 현황

**정의**: 내부는 절연체이면서 표면/에지는 전도성을 갖는 양자 물질

#### 시장 전망

- 2024년: USD 6.6 million
- 2035년: USD 15.2 million
- CAGR: 7.9%

#### 핵심 특성

- **시간 반전 대칭성**: 표면 상태가 불순물에 강건
- **스핀-모멘텀 잠금**: 스핀과 운동량이 결합
- **소산 없는 스핀 전류**: 에너지 손실 없는 전류 흐름

### 6.2 응용 분야

| 분야 | 응용 | 주요 기업 |
|------|------|----------|
| **양자 컴퓨팅** | 토폴로지 큐비트 | Microsoft, Google |
| **스핀트로닉스** | 스핀 트랜지스터 | Intel, Toshiba |
| **저전력 전자** | 차세대 트랜지스터 | IBM, Intel |
| **센서** | 양자 센서 | 연구 단계 |

### 6.3 데이터 형식

```json
{
  "material": {
    "name": "Bi2Se3",
    "crystal_structure": "rhombohedral",
    "space_group": "R-3m",
    "lattice_parameters": {
      "a_angstrom": 4.14,
      "c_angstrom": 28.64
    }
  },
  "topological_properties": {
    "band_gap_eV": 0.3,
    "z2_invariant": [1, 0, 0, 0],
    "dirac_point_eV": -0.1,
    "surface_state": {
      "fermi_velocity_m_s": 5e5,
      "spin_texture": "helical"
    }
  },
  "transport_properties": {
    "bulk_resistivity_ohm_m": 1e-3,
    "surface_conductivity_S": 1e-4,
    "carrier_mobility_cm2_Vs": 1000,
    "spin_hall_angle": 0.3
  },
  "measurement": {
    "method": "ARPES",
    "temperature_K": 10.0,
    "magnetic_field_T": 0.0
  }
}
```

### 6.4 Majorana 페르미온 관련

```json
{
  "hybrid_structure": {
    "type": "topological_superconductor",
    "ti_material": "Bi2Se3",
    "sc_material": "NbSe2",
    "interface": "heterostructure"
  },
  "majorana_signature": {
    "zero_bias_peak": true,
    "peak_height_2e2_h": 0.95,
    "splitting_ueV": 0.0,
    "temperature_K": 0.1
  },
  "qubit_properties": {
    "coherence_time_us": 100.0,
    "gate_fidelity": 0.999,
    "braiding_supported": true
  }
}
```

### 6.5 최근 연구 동향 (2025)

- 자기 토폴로지 절연체 기반 멤리스터 개발
- 고차 토폴로지 절연체 연구
- 실온 작동 가능한 토폴로지 소자 개발

---

## 7. 기존 표준 조사

### 7.1 Materials Data Curation System (MDCS) - NIST

**기관**: National Institute of Standards and Technology
**목적**: 재료 데이터 큐레이션 및 공유

#### 데이터 모델

```xml
<material>
  <identity>
    <name>Bismuth Selenide</name>
    <formula>Bi2Se3</formula>
    <classification>Topological Insulator</classification>
  </identity>
  <properties>
    <property name="band_gap" value="0.3" unit="eV"/>
    <property name="crystal_system" value="rhombohedral"/>
  </properties>
  <provenance>
    <source>DOI:10.xxxx/xxxxx</source>
    <date>2024-01-15</date>
  </provenance>
</material>
```

### 7.2 Materials Project API

**URL**: https://materialsproject.org
**데이터**: 15만+ 무기 화합물 데이터베이스

#### REST API 응답 형식

```json
{
  "material_id": "mp-541837",
  "formula": "Bi2Se3",
  "formation_energy_per_atom": -0.234,
  "band_gap": 0.3,
  "density": 7.68,
  "structure": {
    "lattice": {...},
    "sites": [...]
  }
}
```

### 7.3 CIF (Crystallographic Information File)

**표준**: IUCr (International Union of Crystallography)
**용도**: 결정 구조 데이터 교환

```
data_Bi2Se3
_chemical_formula_sum    'Bi2 Se3'
_cell_length_a           4.14
_cell_length_b           4.14
_cell_length_c           28.64
_cell_angle_alpha        90
_cell_angle_beta         90
_cell_angle_gamma        120
_symmetry_space_group_name_H-M  'R -3 m'
```

### 7.4 OPTIMADE API

**목적**: 재료 데이터베이스 간 상호운용성
**스펙**: JSON:API 기반

```json
{
  "data": {
    "type": "structures",
    "id": "example/1",
    "attributes": {
      "chemical_formula_reduced": "Bi2Se3",
      "nelements": 2,
      "elements": ["Bi", "Se"],
      "lattice_vectors": [[4.14, 0, 0], ...]
    }
  }
}
```

---

## 8. 공통점 분석

### 8.1 모든 분야에 공통으로 필요한 필드

| 필드 | 설명 | 필수 여부 |
|------|------|----------|
| `material_type` | 물질/기술 유형 | 필수 |
| `material_id` | 고유 식별자 | 필수 |
| `composition` | 화학 조성 | 필수 |
| `structure` | 구조 정보 | 필수 |
| `properties` | 물성 데이터 | 필수 |
| `measurement` | 측정 조건 | 권장 |
| `provenance` | 출처/이력 | 권장 |
| `timestamp` | 기록 시간 | 필수 |

### 8.2 물성 데이터 분류

| 분류 | 속성 예시 | 단위 |
|------|----------|------|
| **전기적** | 저항률, 전도도, 이동도 | Ω·m, S/m, cm²/V·s |
| **자기적** | 자화율, 투자율, 자속밀도 | 무차원, H/m, T |
| **열적** | 열전도도, 비열, Tc | W/m·K, J/kg·K, K |
| **광학적** | 굴절률, 흡수계수, 밴드갭 | 무차원, cm⁻¹, eV |
| **역학적** | 영률, 경도, 밀도 | GPa, HV, kg/m³ |

### 8.3 측정 조건

| 조건 | 범위 | 중요도 |
|------|------|--------|
| 온도 | mK ~ 수천 K | 필수 |
| 압력 | 진공 ~ 수백 GPa | 필수 |
| 자기장 | 0 ~ 수십 T | 분야별 |
| 전기장 | 0 ~ MV/m | 분야별 |
| 주파수 | DC ~ THz | 분야별 |

---

## 9. 결론 및 설계 방향

### 9.1 표준 형식 설계 원칙

1. **통일성 (Uniformity)**
   - 모든 재료 과학 분야에 적용 가능한 공통 스키마
   - 일관된 단위 체계 (SI 기본)

2. **확장성 (Extensibility)**
   - 분야별 특수 속성 추가 용이
   - 커스텀 필드 허용

3. **상호운용성 (Interoperability)**
   - JSON 기반 (REST API 친화)
   - 기존 표준(CIF, Materials Project)과 호환

4. **추적성 (Traceability)**
   - 측정 조건 상세 기록
   - 출처 및 이력 관리

### 9.2 권장 기본 구조

```json
{
  "$schema": "https://wia.live/material/v1/schema.json",
  "version": "1.0.0",
  "material_type": "topological_insulator",
  "material_id": "wia-mat-00001",
  "timestamp": {
    "created": "2025-12-14T00:00:00Z",
    "modified": "2025-12-14T00:00:00Z"
  },
  "identity": {
    "name": "Bismuth Selenide",
    "formula": "Bi2Se3",
    "classification": ["chalcogenide", "topological_insulator"]
  },
  "structure": {
    "crystal_system": "rhombohedral",
    "space_group": "R-3m",
    "lattice_parameters": {...}
  },
  "properties": {
    "electrical": {...},
    "magnetic": {...},
    "thermal": {...},
    "optical": {...},
    "mechanical": {...}
  },
  "measurement": {
    "temperature_K": 300.0,
    "pressure_Pa": 101325.0,
    "method": "four_probe",
    "instrument": "Keithley 2400"
  },
  "provenance": {
    "source": "DOI:10.xxxx/xxxxx",
    "lab": "KAIST Materials Lab",
    "operator": "researcher@kaist.ac.kr"
  },
  "meta": {
    "confidence": 0.95,
    "validated": true,
    "notes": ""
  }
}
```

### 9.3 분야별 확장 스키마

| 분야 | 확장 스키마 | 주요 추가 필드 |
|------|------------|---------------|
| Superconductor | superconductor.schema.json | Tc, Hc, Jc, Meissner |
| Metamaterial | metamaterial.schema.json | permittivity, permeability, unit_cell |
| Programmable Matter | programmable-matter.schema.json | module, bonds, reconfiguration |
| Holographic Storage | holographic-storage.schema.json | hologram, multiplexing, capacity |
| Memristor | memristor.schema.json | resistance_states, neuromorphic |
| Topological Insulator | topological-insulator.schema.json | band_topology, surface_states |

### 9.4 다음 단계

1. **PHASE-1-DATA-FORMAT.md** 작성
   - 위 조사 결과를 바탕으로 정식 스펙 문서 작성

2. **JSON Schema 생성**
   - 기본 스키마 + 분야별 스키마 6개

3. **예제 데이터 생성**
   - 각 분야별 실제 사용 시나리오 반영

4. **검증 스크립트**
   - TypeScript 및 Python으로 스키마 검증 도구 구현

---

## 참고 문헌 (References)

### 학술 자료

- [LK-99 Wikipedia](https://en.wikipedia.org/wiki/LK-99)
- [Metamaterial Market Report 2025-2032](https://www.coherentmarketinsights.com/market-insight/metamaterials-market-2834)
- [Claytronics - Carnegie Mellon](https://www.cs.cmu.edu/~claytronics/)
- [Holographic Storage - ACM TOS](https://dl.acm.org/doi/10.1145/3708993)
- [Memristor Research - Science](https://spj.science.org/doi/10.34133/research.0758)
- [Topological Insulators Market - TMR](https://www.globenewswire.com/news-release/2025/03/05/3037464/32656/en/Topological-Insulators-Market-to-Reach-USD-15-2-Million-by-2035-Growth-Driven-by-Quantum-Tech-Spintronics-TMR-Analysis.html)

### 데이터베이스/API

- [NIST Materials Data Curation System](https://www.nist.gov/mml/materials-data-curation-system)
- [Materials Project](https://materialsproject.org)
- [OPTIMADE](https://www.optimade.org/)
- [Crystallography Open Database](https://www.crystallography.net/cod/)

### 표준 문서

- IUCr CIF Specification
- UK Metamaterials Network Roadmaps

---

<div align="center">

**Phase 1 Research Complete**

---

弘益人間 🤟

</div>
