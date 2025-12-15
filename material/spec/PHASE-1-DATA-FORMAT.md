# WIA Material Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-14
**Authors**: WIA (World Industry Authentication Association) / SmileStory Inc.
**License**: MIT

---

## 목차 (Table of Contents)

1. [개요 (Overview)](#1-개요-overview)
2. [용어 정의 (Terminology)](#2-용어-정의-terminology)
3. [기본 구조 (Base Structure)](#3-기본-구조-base-structure)
4. [분야별 데이터 형식 (Domain-Specific Data)](#4-분야별-데이터-형식-domain-specific-data)
5. [확장성 (Extensibility)](#5-확장성-extensibility)
6. [버전 관리 (Versioning)](#6-버전-관리-versioning)
7. [예제 (Examples)](#7-예제-examples)
8. [참고문헌 (References)](#8-참고문헌-references)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

WIA Material Data Format Standard는 첨단 재료과학 분야의 데이터 형식을 표준화하기 위한 규격입니다.

**핵심 목표**:
- 재료과학 데이터가 동일한 JSON 형식으로 교환 가능
- 연구 기관 및 기업 간 데이터 상호운용성 제공
- 소프트웨어 개발자가 다양한 재료 데이터를 일관된 방식으로 처리

### 1.2 적용 범위 (Scope)

본 표준은 다음 재료과학 분야를 포함합니다:

| 분야 | 영문명 | 설명 |
|------|--------|------|
| 상온 초전도체 | Room Temperature Superconductor | 상온에서 초전도성을 나타내는 물질 |
| 메타물질 | Metamaterial | 인공적으로 설계된 특수 물성 물질 |
| 프로그래머블 물질 | Programmable Matter | 형상 변형 가능한 스마트 물질 |
| 홀로그래픽 저장 | Holographic Data Storage | 3D 볼륨 기반 데이터 저장 |
| 멤리스터 | Memristor | 저항 기억 소자 |
| 토폴로지 절연체 | Topological Insulator | 양자 토폴로지 물질 |

### 1.3 설계 원칙 (Design Principles)

1. **통일성 (Uniformity)**: 모든 분야에 적용 가능한 공통 스키마
2. **확장성 (Extensibility)**: 새로운 분야 추가 용이
3. **상호운용성 (Interoperability)**: 기존 표준(CIF, Materials Project)과 호환
4. **추적성 (Traceability)**: 측정 조건 및 출처 상세 기록
5. **검증 가능성 (Validation)**: JSON Schema로 형식 검증

---

## 2. 용어 정의 (Terminology)

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **Material** | 특정 조성과 구조를 가진 물질 |
| **Property** | 물질의 측정 가능한 물리적/화학적 특성 |
| **Measurement** | 특정 조건에서 수행된 물성 측정 |
| **Provenance** | 데이터의 출처 및 이력 정보 |
| **Structure** | 물질의 결정 구조 또는 형태 정보 |
| **Domain** | 특정 재료과학 분야 (초전도체, 메타물질 등) |

### 2.2 데이터 타입

| 타입 | 설명 | 예시 |
|------|------|------|
| `string` | UTF-8 문자열 | `"Bi2Se3"` |
| `number` | 64-bit IEEE 754 부동소수점 | `0.3`, `300.0` |
| `integer` | 정수 | `1`, `255` |
| `boolean` | 불리언 | `true`, `false` |
| `null` | 널 값 | `null` |
| `object` | JSON 객체 | `{"a": 4.14, "c": 28.64}` |
| `array` | JSON 배열 | `[1, 0, 0, 0]` |

### 2.3 필드 요구사항

| 표기 | 의미 |
|------|------|
| **REQUIRED** | 반드시 포함해야 함 |
| **OPTIONAL** | 선택적으로 포함 가능 |
| **CONDITIONAL** | 특정 조건에서 필수 |

### 2.4 단위 체계

본 표준은 SI 단위계를 기본으로 사용합니다:

| 물리량 | 단위 | 기호 |
|--------|------|------|
| 온도 | 켈빈 | K |
| 압력 | 파스칼 | Pa |
| 길이 | 미터 (나노~밀리) | nm, um, mm, m |
| 전기저항 | 옴 | Ω |
| 자기장 | 테슬라 | T |
| 에너지 | 전자볼트 | eV |
| 주파수 | 헤르츠 | Hz |

---

## 3. 기본 구조 (Base Structure)

### 3.1 메시지 형식 (Message Format)

모든 WIA Material Data Message는 다음 기본 구조를 따릅니다:

```json
{
    "$schema": "https://wia.live/material/v1/schema.json",
    "version": "1.0.0",
    "material_type": "<domain_type>",
    "material_id": "<unique_identifier>",
    "timestamp": {
        "created": "<ISO 8601 string>",
        "modified": "<ISO 8601 string>"
    },
    "identity": {
        "name": "<material_name>",
        "formula": "<chemical_formula>",
        "classification": ["<tag1>", "<tag2>"]
    },
    "structure": {
        // 구조 정보
    },
    "properties": {
        // 물성 데이터
    },
    "measurement": {
        // 측정 조건
    },
    "provenance": {
        // 출처 정보
    },
    "meta": {
        "confidence": <0.0-1.0>,
        "validated": <boolean>,
        "notes": "<string>"
    }
}
```

### 3.2 필드 상세

#### 3.2.1 `$schema` (OPTIONAL)

```
타입: string
형식: URI
설명: JSON Schema 위치
예시: "https://wia.live/material/v1/schema.json"
```

#### 3.2.2 `version` (REQUIRED)

```
타입: string
형식: Semantic Versioning (MAJOR.MINOR.PATCH)
설명: 스펙 버전
예시: "1.0.0"
```

#### 3.2.3 `material_type` (REQUIRED)

```
타입: string
설명: 재료과학 분야 식별자
유효값:
  - "superconductor"         : 초전도체
  - "metamaterial"           : 메타물질
  - "programmable_matter"    : 프로그래머블 물질
  - "holographic_storage"    : 홀로그래픽 저장
  - "memristor"              : 멤리스터
  - "topological_insulator"  : 토폴로지 절연체
  - "custom"                 : 사용자 정의 (확장용)
```

#### 3.2.4 `material_id` (REQUIRED)

```
타입: string
형식: "wia-mat-" + 8자리 영숫자
설명: 고유 식별자
예시: "wia-mat-00000001"
```

#### 3.2.5 `timestamp` (REQUIRED)

```
타입: object
설명: 레코드 생성/수정 시간

하위 필드:
  - created (REQUIRED): string
    형식: ISO 8601
    예시: "2025-12-14T00:00:00Z"

  - modified (OPTIONAL): string
    형식: ISO 8601
    예시: "2025-12-14T12:00:00Z"
```

#### 3.2.6 `identity` (REQUIRED)

```
타입: object
설명: 물질 식별 정보

하위 필드:
  - name (REQUIRED): string
    설명: 물질명
    예시: "Bismuth Selenide", "LK-99"

  - formula (REQUIRED): string
    설명: 화학식
    예시: "Bi2Se3", "Pb9Cu(PO4)6O"

  - classification (OPTIONAL): array of string
    설명: 분류 태그
    예시: ["chalcogenide", "topological_insulator"]
```

#### 3.2.7 `structure` (CONDITIONAL)

```
타입: object
설명: 결정 구조 정보 (결정질 물질의 경우 REQUIRED)

공통 하위 필드:
  - crystal_system (OPTIONAL): string
    유효값: "cubic", "tetragonal", "orthorhombic",
            "hexagonal", "trigonal", "monoclinic", "triclinic"

  - space_group (OPTIONAL): string
    설명: Hermann-Mauguin 표기
    예시: "R-3m", "Fm-3m"

  - lattice_parameters (OPTIONAL): object
    하위: a, b, c (Angstrom), alpha, beta, gamma (degree)
```

#### 3.2.8 `properties` (REQUIRED)

```
타입: object
설명: 물성 데이터

분류별 하위 객체:
  - electrical (OPTIONAL): 전기적 물성
  - magnetic (OPTIONAL): 자기적 물성
  - thermal (OPTIONAL): 열적 물성
  - optical (OPTIONAL): 광학적 물성
  - mechanical (OPTIONAL): 역학적 물성
  - domain_specific (OPTIONAL): 분야별 특수 물성
```

#### 3.2.9 `measurement` (OPTIONAL)

```
타입: object
설명: 측정 조건

하위 필드:
  - temperature_K (OPTIONAL): number
    설명: 측정 온도 (Kelvin)

  - pressure_Pa (OPTIONAL): number
    설명: 측정 압력 (Pascal)

  - magnetic_field_T (OPTIONAL): number
    설명: 외부 자기장 (Tesla)

  - method (OPTIONAL): string
    설명: 측정 방법
    예시: "four_probe", "ARPES", "SQUID"

  - instrument (OPTIONAL): string
    설명: 측정 장비
    예시: "Keithley 2400", "PPMS"
```

#### 3.2.10 `provenance` (OPTIONAL)

```
타입: object
설명: 출처 및 이력 정보

하위 필드:
  - source (OPTIONAL): string
    설명: DOI 또는 URL
    예시: "DOI:10.1038/s41586-023-00001-1"

  - lab (OPTIONAL): string
    설명: 연구실/기관명

  - operator (OPTIONAL): string
    설명: 연구자/담당자
```

#### 3.2.11 `meta` (OPTIONAL)

```
타입: object
설명: 메타데이터

하위 필드:
  - confidence (OPTIONAL): number
    범위: 0.0 ~ 1.0
    설명: 데이터 신뢰도

  - validated (OPTIONAL): boolean
    설명: 검증 여부

  - notes (OPTIONAL): string
    설명: 추가 메모
```

---

## 4. 분야별 데이터 형식 (Domain-Specific Data)

### 4.1 Superconductor (초전도체)

#### 4.1.1 필수 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `critical_temperature_K` | number | 임계 온도 | K |
| `critical_pressure_Pa` | number | 임계 압력 | Pa |

#### 4.1.2 선택 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `critical_current_density_A_m2` | number | 임계 전류 밀도 | A/m² |
| `critical_magnetic_field_T` | number | 임계 자기장 | T |
| `meissner_effect` | boolean | Meissner 효과 관측 여부 | - |
| `superconductor_type` | string | 타입 I/II | - |
| `coherence_length_nm` | number | 결맞음 길이 | nm |
| `penetration_depth_nm` | number | 침투 깊이 | nm |

#### 4.1.3 예시

```json
{
  "material_type": "superconductor",
  "identity": {
    "name": "YBCO",
    "formula": "YBa2Cu3O7-x"
  },
  "properties": {
    "domain_specific": {
      "critical_temperature_K": 93.0,
      "critical_pressure_Pa": 101325,
      "critical_magnetic_field_T": 100.0,
      "meissner_effect": true,
      "superconductor_type": "type_ii"
    }
  }
}
```

---

### 4.2 Metamaterial (메타물질)

#### 4.2.1 필수 필드

| 필드 | 타입 | 설명 |
|------|------|------|
| `metamaterial_type` | string | "electromagnetic", "acoustic", "mechanical" |
| `unit_cell` | object | 단위 셀 구조 |

#### 4.2.2 전자기 메타물질 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `permittivity_real` | number | 유전율 실수부 | - |
| `permittivity_imag` | number | 유전율 허수부 | - |
| `permeability_real` | number | 투자율 실수부 | - |
| `permeability_imag` | number | 투자율 허수부 | - |
| `refractive_index` | number | 굴절률 | - |
| `operating_frequency_Hz` | number | 동작 주파수 | Hz |
| `absorption_percent` | number | 흡수율 | % |

#### 4.2.3 음향 메타물질 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `effective_density_kg_m3` | number | 유효 밀도 | kg/m³ |
| `effective_bulk_modulus_Pa` | number | 유효 체적탄성률 | Pa |
| `transmission_loss_dB` | number | 투과 손실 | dB |
| `sound_absorption_coefficient` | number | 흡음 계수 | - |

#### 4.2.4 예시

```json
{
  "material_type": "metamaterial",
  "identity": {
    "name": "Split Ring Resonator Array",
    "classification": ["electromagnetic", "negative_index"]
  },
  "properties": {
    "domain_specific": {
      "metamaterial_type": "electromagnetic",
      "unit_cell": {
        "type": "split_ring_resonator",
        "period_um": 100.0,
        "dimensions": {
          "outer_radius_um": 45.0,
          "gap_um": 5.0
        }
      },
      "permittivity_real": -2.5,
      "permeability_real": -1.2,
      "refractive_index": -1.73,
      "operating_frequency_Hz": 10e9
    }
  }
}
```

---

### 4.3 Programmable Matter (프로그래머블 물질)

#### 4.3.1 모듈 (Catom) 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `module_type` | string | 모듈 유형 | - |
| `module_id` | string | 모듈 식별자 | - |
| `position` | object | 3D 위치 {x, y, z} | um |
| `bonds` | array | 결합된 모듈 ID 목록 | - |
| `capabilities` | object | 기능 플래그 | - |

#### 4.3.2 집합체 (Ensemble) 필드

| 필드 | 타입 | 설명 |
|------|------|------|
| `ensemble_id` | string | 집합체 식별자 |
| `target_shape` | string | 목표 형상 |
| `module_count` | integer | 모듈 수 |
| `shape_accuracy_percent` | number | 형상 정확도 |

#### 4.3.3 예시

```json
{
  "material_type": "programmable_matter",
  "identity": {
    "name": "Claytronics Sphere Assembly",
    "classification": ["claytronics", "catom"]
  },
  "properties": {
    "domain_specific": {
      "ensemble_id": "sphere_001",
      "target_shape": "sphere",
      "module_count": 10000,
      "modules": [
        {
          "module_id": "catom_001",
          "position": {"x": 10.5, "y": 20.3, "z": 5.0},
          "bonds": ["catom_002", "catom_003"]
        }
      ]
    }
  }
}
```

---

### 4.4 Holographic Storage (홀로그래픽 저장)

#### 4.4.1 필수 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `medium_type` | string | 저장 매체 유형 | - |
| `material` | string | 매체 물질 | - |
| `capacity_GB` | number | 저장 용량 | GB |

#### 4.4.2 기록 관련 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `wavelength_nm` | number | 레이저 파장 | nm |
| `multiplexing_method` | string | 다중화 방식 | - |
| `hologram_count` | integer | 홀로그램 수 | - |
| `page_size_bits` | integer | 페이지 크기 | bits |

#### 4.4.3 성능 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `read_speed_Mbps` | number | 읽기 속도 | Mbps |
| `write_speed_Mbps` | number | 쓰기 속도 | Mbps |
| `retention_years` | number | 보존 기간 | 년 |

#### 4.4.4 예시

```json
{
  "material_type": "holographic_storage",
  "identity": {
    "name": "LiNbO3 Holographic Medium",
    "formula": "LiNbO3"
  },
  "properties": {
    "domain_specific": {
      "medium_type": "photorefractive_crystal",
      "material": "LiNbO3",
      "capacity_GB": 1000.0,
      "wavelength_nm": 532.0,
      "multiplexing_method": "angular",
      "hologram_count": 1000,
      "read_speed_Mbps": 1000.0,
      "retention_years": 50
    }
  }
}
```

---

### 4.5 Memristor (멤리스터)

#### 4.5.1 필수 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `material` | string | 활성층 물질 | - |
| `structure` | string | 소자 구조 | - |
| `resistance_high_ohm` | number | 고저항 상태 | Ω |
| `resistance_low_ohm` | number | 저저항 상태 | Ω |

#### 4.5.2 전기적 특성 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `on_off_ratio` | number | ON/OFF 비율 | - |
| `set_voltage_V` | number | SET 전압 | V |
| `reset_voltage_V` | number | RESET 전압 | V |
| `endurance_cycles` | number | 내구성 (사이클) | - |
| `retention_s` | number | 유지 시간 | s |

#### 4.5.3 뉴로모픽 필드

| 필드 | 타입 | 설명 |
|------|------|------|
| `synaptic_weight` | number | 시냅스 가중치 (0-1) |
| `plasticity` | string | 가소성 유형 (STDP 등) |
| `analog_states` | integer | 아날로그 상태 수 |

#### 4.5.4 예시

```json
{
  "material_type": "memristor",
  "identity": {
    "name": "TiO2 Memristor",
    "formula": "TiO2"
  },
  "properties": {
    "domain_specific": {
      "material": "TiO2",
      "structure": "metal_insulator_metal",
      "resistance_high_ohm": 1e6,
      "resistance_low_ohm": 1e3,
      "on_off_ratio": 1000,
      "set_voltage_V": 1.0,
      "reset_voltage_V": -0.8,
      "neuromorphic": {
        "synaptic_weight": 0.5,
        "plasticity": "stdp",
        "analog_states": 128
      }
    }
  }
}
```

---

### 4.6 Topological Insulator (토폴로지 절연체)

#### 4.6.1 필수 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `band_gap_eV` | number | 밴드 갭 | eV |
| `z2_invariant` | array | Z2 토폴로지 불변량 | - |

#### 4.6.2 토폴로지 특성 필드

| 필드 | 타입 | 설명 | 단위 |
|------|------|------|------|
| `dirac_point_eV` | number | Dirac 포인트 | eV |
| `fermi_velocity_m_s` | number | 페르미 속도 | m/s |
| `spin_texture` | string | 스핀 텍스처 | - |
| `surface_conductivity_S` | number | 표면 전도도 | S |

#### 4.6.3 스핀트로닉스 필드

| 필드 | 타입 | 설명 |
|------|------|------|
| `spin_hall_angle` | number | 스핀 홀 각도 |
| `spin_diffusion_length_nm` | number | 스핀 확산 길이 |

#### 4.6.4 예시

```json
{
  "material_type": "topological_insulator",
  "identity": {
    "name": "Bismuth Selenide",
    "formula": "Bi2Se3",
    "classification": ["chalcogenide", "topological_insulator"]
  },
  "structure": {
    "crystal_system": "rhombohedral",
    "space_group": "R-3m",
    "lattice_parameters": {
      "a_angstrom": 4.14,
      "c_angstrom": 28.64
    }
  },
  "properties": {
    "domain_specific": {
      "band_gap_eV": 0.3,
      "z2_invariant": [1, 0, 0, 0],
      "dirac_point_eV": -0.1,
      "surface_state": {
        "fermi_velocity_m_s": 5e5,
        "spin_texture": "helical"
      },
      "spin_hall_angle": 0.3
    }
  }
}
```

---

## 5. 확장성 (Extensibility)

### 5.1 새로운 분야 추가

새로운 재료과학 분야를 추가하려면:

1. `material_type`에 새 식별자 등록
2. 해당 분야의 스키마 정의 (`/spec/schemas/xxx.schema.json`)
3. 이 문서에 분야별 섹션 추가

### 5.2 커스텀 필드

`properties.custom` 객체를 사용하여 표준에 정의되지 않은 필드 추가 가능:

```json
{
  "properties": {
    "domain_specific": {...},
    "custom": {
      "my_custom_field": "value",
      "another_field": 123.45
    }
  }
}
```

### 5.3 기존 표준 호환

다른 데이터베이스/표준과의 상호 참조:

```json
{
  "external_references": {
    "materials_project_id": "mp-541837",
    "icsd_id": "12345",
    "cod_id": "9000001",
    "doi": "10.1038/s41586-023-00001-1"
  }
}
```

---

## 6. 버전 관리 (Versioning)

### 6.1 Semantic Versioning

본 표준은 Semantic Versioning 2.0.0을 따릅니다:

```
MAJOR.MINOR.PATCH

예: 1.0.0
    │ │ └── 버그 수정, 문서 개선
    │ └──── 하위 호환 기능 추가
    └────── 하위 호환 불가 변경
```

### 6.2 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-12-14 | 초기 버전 |

---

## 7. 예제 (Examples)

### 7.1 완전한 초전도체 데이터

```json
{
  "$schema": "https://wia.live/material/v1/superconductor.schema.json",
  "version": "1.0.0",
  "material_type": "superconductor",
  "material_id": "wia-mat-00000001",
  "timestamp": {
    "created": "2025-12-14T00:00:00Z",
    "modified": "2025-12-14T00:00:00Z"
  },
  "identity": {
    "name": "YBCO",
    "formula": "YBa2Cu3O7-x",
    "classification": ["cuprate", "high_tc", "type_ii"]
  },
  "structure": {
    "crystal_system": "orthorhombic",
    "space_group": "Pmmm",
    "lattice_parameters": {
      "a_angstrom": 3.82,
      "b_angstrom": 3.89,
      "c_angstrom": 11.68,
      "alpha_degree": 90,
      "beta_degree": 90,
      "gamma_degree": 90
    }
  },
  "properties": {
    "electrical": {
      "resistivity_ohm_m": 0.0,
      "normal_state_resistivity_ohm_m": 1e-6
    },
    "domain_specific": {
      "critical_temperature_K": 93.0,
      "critical_pressure_Pa": 101325,
      "critical_current_density_A_m2": 1e10,
      "critical_magnetic_field_T": 100.0,
      "meissner_effect": true,
      "superconductor_type": "type_ii",
      "coherence_length_nm": 1.5,
      "penetration_depth_nm": 150.0
    }
  },
  "measurement": {
    "temperature_K": 77.0,
    "pressure_Pa": 101325,
    "method": "four_probe",
    "instrument": "PPMS DynaCool"
  },
  "provenance": {
    "source": "DOI:10.1038/nature12345",
    "lab": "MIT Superconductivity Lab",
    "operator": "researcher@mit.edu"
  },
  "meta": {
    "confidence": 0.98,
    "validated": true,
    "notes": "Sample synthesized via solid-state reaction"
  }
}
```

### 7.2 완전한 토폴로지 절연체 데이터

```json
{
  "$schema": "https://wia.live/material/v1/topological-insulator.schema.json",
  "version": "1.0.0",
  "material_type": "topological_insulator",
  "material_id": "wia-mat-00000002",
  "timestamp": {
    "created": "2025-12-14T00:00:00Z"
  },
  "identity": {
    "name": "Bismuth Selenide",
    "formula": "Bi2Se3",
    "classification": ["chalcogenide", "3d_topological_insulator", "strong_ti"]
  },
  "structure": {
    "crystal_system": "rhombohedral",
    "space_group": "R-3m",
    "lattice_parameters": {
      "a_angstrom": 4.14,
      "c_angstrom": 28.64
    }
  },
  "properties": {
    "electrical": {
      "bulk_resistivity_ohm_m": 1e-3,
      "carrier_mobility_cm2_Vs": 1000
    },
    "domain_specific": {
      "band_gap_eV": 0.3,
      "z2_invariant": [1, 0, 0, 0],
      "dirac_point_eV": -0.1,
      "surface_state": {
        "fermi_velocity_m_s": 5e5,
        "spin_texture": "helical",
        "surface_conductivity_S": 1e-4
      },
      "spin_hall_angle": 0.3,
      "spin_diffusion_length_nm": 10.0
    }
  },
  "measurement": {
    "temperature_K": 10.0,
    "pressure_Pa": 1e-8,
    "method": "ARPES",
    "instrument": "Scienta R4000"
  },
  "provenance": {
    "source": "DOI:10.1126/science.1167733"
  },
  "meta": {
    "confidence": 0.95,
    "validated": true
  }
}
```

---

## 8. 참고문헌 (References)

### 8.1 관련 표준

- [NIST Materials Data Curation System](https://www.nist.gov/mml/materials-data-curation-system)
- [Materials Project API](https://materialsproject.org/api)
- [OPTIMADE Specification](https://www.optimade.org/specification/)
- [IUCr CIF Format](https://www.iucr.org/resources/cif)

### 8.2 JSON Schema

- [JSON Schema Draft-07](https://json-schema.org/specification-links.html#draft-7)
- [Understanding JSON Schema](https://json-schema.org/understanding-json-schema/)

### 8.3 단위 체계

- [NIST SI Units](https://www.nist.gov/pml/weights-and-measures/metric-si/si-units)
- [CODATA Recommended Values](https://physics.nist.gov/cuu/Constants/)

---

<div align="center">

**WIA Material Data Format Standard v1.0.0**

---

弘益人間 🤟

© 2025 SmileStory Inc. / WIA

</div>
