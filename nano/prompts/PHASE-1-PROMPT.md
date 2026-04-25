# Phase 1: Data Format Standard
## Claude Code 작업 프롬프트

---

**Standard**: WIA Nano
**Phase**: 1 of 4
**목표**: 나노기술 데이터의 표준 형식 정의
**난이도**: ★★★★★
**예상 작업량**: 스펙 문서 1개 + JSON Schema + 예제 파일

---

## 🎯 Phase 1 목표

### 핵심 질문
```
"분자 조립기, 나노머신, 분자 메모리, 나노의학, 나노로봇...

 각각 다른 형식으로 데이터를 정의하면 호환이 안 된다.

 이걸 하나의 표준 형식으로 통일할 수 있을까?"
```

### 목표
```
나노기술 유형에 관계없이
모든 프로젝트가 동일한 JSON 형식으로 데이터를 표현하도록
Data Format Standard를 정의한다.
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 나노기술 조사

아래 기술 유형별로 웹서치하여 실제 데이터 형식을 조사하세요:

| 기술 유형 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **Molecular Assembler** | 분자 조립 시스템 | "molecular assembler nanomachine Drexler specifications" |
| **Nanomachine** | 나노 기계 | "nanomachine molecular motor protein engineering" |
| **Molecular Memory** | 분자 단위 기억장치 | "molecular memory storage DNA computing" |
| **Nanomedicine** | 나노의학 | "nanomedicine drug delivery nanoparticle therapy" |
| **Nanorobotics** | 나노로봇 | "nanorobot medical DNA origami molecular robotics" |
| **Quantum Dots** | 양자점 | "quantum dot nanoparticle optical properties" |

### 2단계: 기존 표준/기관 조사

| 표준/기관 | 조사 내용 | 웹서치 키워드 |
|----------|----------|--------------|
| **PDB** | 단백질 데이터 뱅크 | "Protein Data Bank PDB file format" |
| **CIF** | 결정 구조 정보 | "Crystallographic Information File format" |
| **XYZ** | 분자 좌표 형식 | "XYZ molecular coordinate file format" |
| **LAMMPS** | 분자 동역학 시뮬레이션 | "LAMMPS data file format molecular dynamics" |
| **GROMACS** | 분자 시뮬레이션 | "GROMACS topology file format" |
| **SMILES** | 화학 구조 표현 | "SMILES chemical structure notation" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-1.md`에 다음을 정리:

```markdown
# Phase 1 사전 조사 결과

## 1. Molecular Assembler

### 이론적 배경
- Drexler의 분자 조립기 이론: [조사 내용]
- 원자 정밀도 제조: [조사 내용]
- 필요 데이터 필드: [분석]

### 기존 연구 데이터
- Foresight Institute 논문 참조: [조사 내용]
...

## 2. Nanomachine

### 기술 현황
- 분자 모터 (ATP synthase 등): [조사 내용]
- 단백질 기반 나노머신: [조사 내용]
...

## 3. Molecular Memory

### 저장 메커니즘
- DNA 기반 스토리지: [조사 내용]
- 분자 스위치: [조사 내용]
...

## 4. Nanomedicine

### 약물 전달 시스템
- 리포좀 나노입자: [조사 내용]
- 표적 치료: [조사 내용]
...

## 5. Nanorobotics

### 의료용 나노로봇
- DNA 오리가미 로봇: [조사 내용]
- 혈관 내 나노로봇: [조사 내용]
...

## 6. Quantum Dots

### 광학적 특성
- 밴드갭 엔지니어링: [조사 내용]
- 바이오이미징 응용: [조사 내용]
...

## 7. 공통점 분석
- 모든 기술에 공통으로 필요한 필드: [분석]
- 기술별 고유 필드: [분석]

## 8. 결론
- 표준 형식 설계 방향: [제안]
```

---

## 🏗️ 표준 설계

### 기본 구조 (제안)

```json
{
  "$schema": "https://wia.live/schemas/nano/project.schema.json",
  "version": "1.0.0",
  "project": {
    "id": "고유 ID",
    "name": "프로젝트명",
    "type": "기술 유형",
    "status": "상태",
    "trl": "기술 성숙도 레벨 (1-9)"
  },
  "spec": {
    "기술별 고유 데이터"
  },
  "scale": {
    "size_nm": "나노미터 크기",
    "mass_daltons": "분자량",
    "complexity": "구조 복잡도"
  },
  "environment": {
    "temperature_kelvin": "작동 온도",
    "ph": "pH 범위",
    "medium": "작동 매질"
  },
  "performance": {
    "efficiency": "효율성",
    "speed": "작동 속도",
    "accuracy": "정밀도"
  },
  "meta": {
    "created_at": "생성일",
    "updated_at": "수정일",
    "author": "작성자",
    "references": ["참고 논문"]
  }
}
```

### 기술별 `spec` 필드 정의

#### Molecular Assembler
```json
{
  "spec": {
    "assembler_type": "mechanosynthesis",  // "mechanosynthesis", "self_assembly", "guided_assembly"
    "workspace": {
      "dimensions_nm": {
        "x": 100,
        "y": 100,
        "z": 100
      },
      "active_sites": 6,
      "precision_nm": 0.1
    },
    "manipulator": {
      "type": "scanning_probe",
      "dof": 6,
      "force_pn": 100,
      "positioning_accuracy_nm": 0.05
    },
    "feedstock": {
      "molecules": ["C60", "diamond_tooltip", "hydrogen"],
      "delivery_rate_molecules_per_sec": 1000000
    },
    "assembly_rate": {
      "atoms_per_second": 1000000,
      "defect_rate": 0.001,
      "yield": 0.95
    },
    "target_structures": [
      {
        "name": "diamond_rod",
        "atoms": 10000,
        "bonds": 40000,
        "estimated_time_sec": 10
      }
    ]
  }
}
```

#### Nanomachine
```json
{
  "spec": {
    "machine_type": "molecular_motor",  // "molecular_motor", "enzyme", "protein_machine", "synthetic"
    "structure": {
      "pdb_id": "1E79",
      "components": [
        {
          "name": "rotor",
          "residues": "1-90",
          "mass_daltons": 10000
        },
        {
          "name": "stator",
          "residues": "91-550",
          "mass_daltons": 45000
        }
      ],
      "total_atoms": 8000
    },
    "mechanism": {
      "energy_source": "ATP",  // "ATP", "proton_gradient", "light", "chemical"
      "atp_binding_sites": 3,
      "rotation_direction": "counterclockwise",
      "steps_per_revolution": 3
    },
    "performance": {
      "rotation_speed_rpm": 6000,
      "torque_pn_nm": 40,
      "efficiency": 0.9,
      "power_output_watts": 1e-18
    },
    "operating_conditions": {
      "temperature_range_k": [273, 310],
      "optimal_ph": 7.4,
      "ion_requirements": {
        "Mg2+": "5mM",
        "K+": "100mM"
      }
    }
  }
}
```

#### Molecular Memory
```json
{
  "spec": {
    "memory_type": "dna_storage",  // "dna_storage", "molecular_switch", "quantum_state", "conformational"
    "storage_mechanism": {
      "encoding": "base4",  // A, T, G, C
      "bits_per_molecule": 2,
      "error_correction": "reed_solomon"
    },
    "capacity": {
      "molecules": 1000000000000,
      "bytes": 250000000000,
      "density_bytes_per_nm3": 1e6
    },
    "performance": {
      "write_speed_mbps": 0.001,
      "read_speed_mbps": 0.1,
      "retention_years": 1000,
      "error_rate": 1e-9
    },
    "physical_properties": {
      "dna_length_bp": 150,
      "synthesis_method": "enzymatic",
      "sequencing_method": "nanopore"
    },
    "addressing": {
      "index_method": "primer_based",
      "random_access": false,
      "parallelism": 1000000
    }
  }
}
```

#### Nanomedicine
```json
{
  "spec": {
    "application": "drug_delivery",  // "drug_delivery", "imaging", "therapy", "diagnostics"
    "nanoparticle": {
      "type": "liposome",  // "liposome", "polymer", "dendrimer", "gold_np", "carbon_nanotube"
      "core_diameter_nm": 100,
      "shell_thickness_nm": 5,
      "surface_area_nm2": 31416,
      "zeta_potential_mv": -20
    },
    "therapeutic_payload": {
      "drug": "doxorubicin",
      "loading_capacity_percent": 30,
      "release_mechanism": "ph_sensitive",
      "release_rate_percent_per_hour": 5
    },
    "targeting": {
      "method": "active",  // "active", "passive", "magnetic"
      "ligands": [
        {
          "name": "folate",
          "count_per_particle": 500,
          "affinity_kd_nm": 0.1
        }
      ],
      "target_cells": ["cancer_cell"],
      "specificity": 0.95
    },
    "biodistribution": {
      "circulation_half_life_hours": 24,
      "tumor_accumulation_percent": 15,
      "clearance_route": "hepatic",
      "biocompatibility_score": 0.9
    },
    "toxicity": {
      "ic50_ug_ml": 50,
      "therapeutic_index": 10,
      "immune_response": "minimal"
    }
  }
}
```

#### Nanorobotics
```json
{
  "spec": {
    "robot_type": "dna_origami",  // "dna_origami", "protein_based", "hybrid", "inorganic"
    "structure": {
      "scaffold": "M13mp18",
      "scaffold_length_bases": 7249,
      "staple_strands": 200,
      "dimensions_nm": {
        "length": 100,
        "width": 80,
        "height": 2
      },
      "mass_daltons": 5000000
    },
    "actuation": {
      "mechanism": "dna_strand_displacement",
      "triggers": ["pH_change", "specific_RNA"],
      "response_time_sec": 60,
      "reversible": true
    },
    "locomotion": {
      "method": "brownian_motor",  // "brownian_motor", "catalytic", "magnetic", "passive_diffusion"
      "speed_nm_per_sec": 10,
      "directionality": "biased_random_walk",
      "fuel": "ATP"
    },
    "cargo": {
      "type": "therapeutic_molecule",
      "capacity_molecules": 100,
      "loading_method": "hybridization",
      "release_trigger": "target_binding"
    },
    "sensing": {
      "detection_targets": ["cancer_biomarker"],
      "sensitivity_nm": 1,
      "specificity": 0.99,
      "signal_output": "conformational_change"
    },
    "control": {
      "programmable": true,
      "logic_gates": ["AND", "OR"],
      "states": 4,
      "external_control": "magnetic_field"
    }
  }
}
```

#### Quantum Dots
```json
{
  "spec": {
    "material": "CdSe_ZnS",  // "CdSe_ZnS", "InP", "CsPbBr3", "graphene_qd"
    "structure": {
      "core_material": "CdSe",
      "core_diameter_nm": 3.5,
      "shell_material": "ZnS",
      "shell_thickness_nm": 2,
      "total_diameter_nm": 7.5,
      "shape": "spherical"
    },
    "optical_properties": {
      "emission_wavelength_nm": 620,
      "emission_fwhm_nm": 30,
      "absorption_wavelength_nm": 400,
      "quantum_yield": 0.85,
      "extinction_coefficient_m_cm": 500000,
      "stokes_shift_nm": 220
    },
    "photophysics": {
      "lifetime_ns": 20,
      "blinking": true,
      "photobleaching_half_life_hours": 100,
      "two_photon_cross_section_gm": 5000
    },
    "surface_chemistry": {
      "ligands": "carboxyl_PEG",
      "functionalization": ["biotin", "streptavidin"],
      "colloidal_stability": "high",
      "hydrodynamic_diameter_nm": 15
    },
    "applications": {
      "imaging": true,
      "sensing": true,
      "display": false,
      "solar_cell": false,
      "target_specificity": "protein_A"
    }
  }
}
```

---

## 📁 산출물 목록

Phase 1 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-1.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-1-DATA-FORMAT.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 기본 구조 (Base Structure)
4. 기술별 데이터 형식 (Technology-Specific Data)
   - Molecular Assembler
   - Nanomachine
   - Molecular Memory
   - Nanomedicine
   - Nanorobotics
   - Quantum Dots
5. 스케일과 단위 (Scale and Units)
6. 환경 파라미터 (Environmental Parameters)
7. 확장성 (Extensibility)
8. 버전 관리 (Versioning)
9. 예제 (Examples)
10. 참고문헌 (References)
```

### 3. JSON Schema 파일
```
/spec/schemas/
├── project.schema.json              (기본 프로젝트 스키마)
├── technology.schema.json           (기술 유형 정의)
├── molecular-assembler.schema.json
├── nanomachine.schema.json
├── molecular-memory.schema.json
├── nanomedicine.schema.json
├── nanorobotics.schema.json
└── quantum-dots.schema.json
```

### 4. 예제 데이터 파일
```
/examples/sample-data/
├── diamond-assembler-example.json
├── atp-synthase-example.json
├── dna-storage-example.json
├── liposome-delivery-example.json
├── dna-origami-robot-example.json
└── quantum-dot-imaging-example.json
```

---

## ✅ 완료 체크리스트

Phase 1 완료 전 확인:

```
□ 웹서치로 6개 나노기술 데이터 형식 조사 완료
□ /spec/RESEARCH-PHASE-1.md 작성 완료
□ /spec/PHASE-1-DATA-FORMAT.md 작성 완료
□ JSON Schema 파일 생성 완료 (기본 + 기술별 6개)
□ 예제 데이터 파일 생성 완료 (6개)
□ JSON Schema로 예제 데이터 검증 통과
□ README 업데이트 (Phase 1 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 나노기술 및 기존 표준 조사
   ↓
2. /spec/RESEARCH-PHASE-1.md 작성
   ↓
3. 조사 결과 바탕으로 표준 설계
   ↓
4. /spec/PHASE-1-DATA-FORMAT.md 작성
   ↓
5. JSON Schema 파일 생성
   ↓
6. 예제 데이터 파일 생성
   ↓
7. 스키마 검증 테스트
   ↓
8. 완료 체크리스트 확인
   ↓
9. Phase 2 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ PDB, CIF 등 실제 분자 데이터 형식 참조
✅ 모든 필드에 명확한 단위 명시 (SI 단위 + 나노스케일 단위)
✅ 확장 가능한 구조로 설계 (미래 기술 유형 고려)
✅ JSON Schema는 draft-07 표준 사용
✅ 과학적으로 검증된 수치 사용
✅ 나노미터, 달톤, 피코뉴튼 등 적절한 단위 사용
```

### DON'T (하지 말 것)

```
❌ 추측으로 데이터 형식 정의 (반드시 조사 후)
❌ SF 소설 설정에만 의존하는 설계
❌ 필수 필드와 선택 필드 구분 없이 작성
❌ 과학적 근거 없는 수치 사용
❌ 단위 없이 숫자만 사용
```

---

## 🚀 작업 시작

이제 Phase 1 작업을 시작하세요.

첫 번째 단계: **웹서치로 Molecular Assembler 조사**

```
검색 키워드: "molecular assembler Eric Drexler mechanosynthesis specifications"
```

화이팅! ⚛️

---

<div align="center">

**Phase 1 of 4**

Data Format Standard

🔬 弘益人間 - Benefit All Humanity 🔬

</div>
