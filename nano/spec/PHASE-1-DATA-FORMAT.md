# WIA Nano Data Format Standard

## Phase 1 Specification v1.0.0

---

## 1. Overview

WIA Nano Data Format Standard는 나노기술 분야의 데이터 교환을 위한 표준 JSON 형식을 정의합니다.

### 1.1 Scope

이 표준은 다음 나노시스템의 데이터 형식을 정의합니다:

| 시스템 | 설명 |
|-------|------|
| Molecular Assembler | 분자 조립기 |
| Nano Machine | 나노 머신 (분자 모터 등) |
| Molecular Memory | 분자 메모리 (DNA 저장소 등) |
| Nanomedicine | 나노의약품 (약물 전달 시스템) |
| Nanorobot | 나노로봇 |
| Nanosensor | 나노센서 |

### 1.2 Conformance

이 표준을 준수하는 구현은:
- 모든 필수(required) 필드를 포함해야 합니다
- 정의된 데이터 타입과 범위를 준수해야 합니다
- JSON Schema draft-07으로 검증 가능해야 합니다

---

## 2. Terminology

| 용어 | 정의 |
|-----|------|
| **Nanoscale** | 1-100 nm 범위의 스케일 (ISO TC 229 정의) |
| **Nanomachine** | 나노스케일에서 작동하는 기계 장치 |
| **Nanorobot** | 자율 제어 가능한 나노스케일 로봇 |
| **Payload** | 나노운반체가 전달하는 화물 (약물 등) |
| **Analyte** | 센서가 검출하는 대상 물질 |

---

## 3. Base Structure

### 3.1 NanoMessage

모든 나노시스템 메시지의 기본 구조:

```json
{
  "$schema": "https://wia.live/nano/v1/schema.json",
  "version": "1.0.0",
  "system_type": "nanorobot",
  "device": {
    "manufacturer": "WIA Labs",
    "model": "NR-1000",
    "serial": "NR-2025-00001",
    "firmware": "1.0.0"
  },
  "timestamp": {
    "iso8601": "2025-01-15T10:30:00.000Z",
    "unix_ms": 1736937000000
  },
  "sequence": 12345,
  "environment": {
    "temperature_k": 310.15,
    "pressure_pa": 101325,
    "medium": "blood",
    "ph": 7.4
  },
  "scale": {
    "length_unit": "nanometer",
    "time_unit": "nanosecond",
    "mass_unit": "dalton"
  },
  "data": { },
  "meta": {
    "confidence": 0.95,
    "calibration_id": "CAL-2025-001"
  }
}
```

### 3.2 Field Definitions

#### Required Fields

| Field | Type | Description |
|-------|------|-------------|
| `$schema` | string | JSON Schema URL |
| `version` | string | 표준 버전 (semver) |
| `system_type` | enum | 시스템 유형 |
| `device` | object | 장치 정보 |
| `timestamp` | object | 타임스탬프 |
| `sequence` | integer | 시퀀스 번호 (0 이상) |
| `data` | object | 시스템별 데이터 |

#### Optional Fields

| Field | Type | Description |
|-------|------|-------------|
| `environment` | object | 환경 조건 |
| `scale` | object | 단위 정보 |
| `meta` | object | 메타데이터 |

### 3.3 System Types

```typescript
type SystemType =
  | "molecular_assembler"
  | "nano_machine"
  | "molecular_memory"
  | "nanomedicine"
  | "nanorobot"
  | "nanosensor";
```

---

## 4. Domain-Specific Data

### 4.1 Molecular Assembler

분자 조립기 데이터 형식:

```json
{
  "data": {
    "operation": "assembly",
    "target": {
      "molecule_id": "MOL-C60-001",
      "name": "Buckminsterfullerene",
      "formula": "C60",
      "structure_type": "fullerene",
      "mass_da": 720.66
    },
    "building_blocks": [
      {
        "element": "C",
        "count": 60,
        "source": "graphene_feedstock"
      }
    ],
    "assembly_state": {
      "status": "in_progress",
      "progress_percent": 75.5,
      "current_step": 45,
      "total_steps": 60,
      "bonds_formed": 90,
      "bonds_target": 120
    },
    "workspace": {
      "center": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "radius_nm": 5.0,
      "atoms_count": 45
    },
    "energy": {
      "total_kj_mol": -2500.5,
      "binding_kj_mol": -45.2,
      "barrier_kj_mol": 12.3
    },
    "error": null
  }
}
```

#### Fields

| Field | Type | Required | Description |
|-------|------|:--------:|-------------|
| `operation` | enum | ✓ | 작업 유형: assembly, disassembly, modification |
| `target` | object | ✓ | 목표 분자 정보 |
| `building_blocks` | array | ✓ | 빌딩 블록 목록 |
| `assembly_state` | object | ✓ | 조립 상태 |
| `workspace` | object | - | 작업 공간 정보 |
| `energy` | object | - | 에너지 정보 |
| `error` | object | - | 에러 정보 (있을 경우) |

---

### 4.2 Nano Machine

나노 머신 (분자 모터 등) 데이터 형식:

```json
{
  "data": {
    "machine_type": "rotary_motor",
    "machine_id": "ATP-SYNTHASE-001",
    "position": {
      "x_nm": 125.5,
      "y_nm": 250.3,
      "z_nm": 15.2
    },
    "orientation": {
      "quaternion": { "w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0 }
    },
    "state": {
      "mode": "active",
      "power_source": "atp",
      "energy_level_percent": 85.5
    },
    "kinematics": {
      "rotation_deg": 120.0,
      "angular_velocity_rpm": 9000,
      "torque_pn_nm": 40.0,
      "cycles_completed": 15000
    },
    "payload": {
      "loaded": true,
      "cargo_type": "proton",
      "cargo_count": 3
    },
    "diagnostics": {
      "structural_integrity": 0.99,
      "efficiency": 0.85
    }
  }
}
```

#### Machine Types

| Type | Description |
|------|-------------|
| `rotary_motor` | 회전 모터 (ATP synthase 등) |
| `linear_motor` | 선형 모터 (Kinesin, Myosin 등) |
| `molecular_switch` | 분자 스위치 |
| `molecular_pump` | 분자 펌프 |

---

### 4.3 Molecular Memory

분자 메모리 (DNA 저장소 등) 데이터 형식:

```json
{
  "data": {
    "memory_type": "dna_storage",
    "memory_id": "DNA-MEM-001",
    "address": {
      "pool_id": 0,
      "strand_id": 1024,
      "offset_bp": 0
    },
    "capacity": {
      "total_bits": 1000000000,
      "used_bits": 500000000,
      "available_bits": 500000000
    },
    "encoding": {
      "method": "goldman",
      "bits_per_nucleotide": 1.77,
      "error_correction": "reed_solomon",
      "redundancy_factor": 4
    },
    "content": {
      "data_type": "binary",
      "hash_sha256": "a1b2c3d4...",
      "fragment_index": 1,
      "total_fragments": 100
    },
    "physical": {
      "strand_count": 1000000,
      "strand_length_bp": 200,
      "gc_content_percent": 50.0,
      "storage_density_pb_per_g": 215
    },
    "access": {
      "read_latency_s": 3600,
      "write_latency_s": 86400,
      "read_cycles": 50,
      "write_cycles": 5
    }
  }
}
```

#### Encoding Methods

| Method | Description | Bits/Nucleotide |
|--------|-------------|:---------------:|
| `binary` | 직접 2-bit 매핑 (00=A, 01=G, 10=C, 11=T) | 2.0 |
| `ternary` | Base-3 인코딩 | ~1.6 |
| `goldman` | Huffman + ternary | 1.77 |
| `fountain` | DNA Fountain 코딩 | ~1.9 |

---

### 4.4 Nanomedicine

나노의약품 (약물 전달 시스템) 데이터 형식:

```json
{
  "data": {
    "carrier_type": "liposome",
    "carrier_id": "LIPO-DOX-001",
    "dimensions": {
      "diameter_nm": 100.0,
      "wall_thickness_nm": 5.0,
      "volume_nm3": 523598.8
    },
    "payload": {
      "drug_name": "Doxorubicin",
      "drug_id": "DOX",
      "mass_pg": 0.5,
      "loading_efficiency_percent": 85.0,
      "molecules_count": 10000
    },
    "targeting": {
      "mechanism": "antibody",
      "target_receptor": "HER2",
      "ligand": "Trastuzumab",
      "binding_affinity_nm": 0.5,
      "specificity": 0.95
    },
    "release": {
      "trigger_type": "ph",
      "trigger_threshold": 5.5,
      "release_rate_percent_per_hour": 10.0,
      "released_percent": 25.0
    },
    "location": {
      "tissue": "tumor",
      "position_um": { "x": 1500.0, "y": 2300.0, "z": 500.0 },
      "in_target_zone": true
    },
    "status": {
      "structural_integrity": 0.98,
      "time_since_injection_h": 2.5,
      "estimated_remaining_h": 8.0
    }
  }
}
```

#### Carrier Types

| Type | Description | Size Range |
|------|-------------|------------|
| `liposome` | 인지질 이중층 소포 | 50-300 nm |
| `solid_lipid` | 고체 지질 나노입자 | 50-100 nm |
| `polymeric` | 폴리머 나노입자 | 10-200 nm |
| `dendrimer` | 덴드리머 | 1-10 nm |
| `micelle` | 미셀 | 10-100 nm |

#### Release Triggers

| Trigger | Description |
|---------|-------------|
| `ph` | pH 감응 (종양: 5.5-6.5) |
| `temperature` | 온도 감응 |
| `enzyme` | 효소 감응 |
| `magnetic` | 자기장 트리거 |
| `ultrasound` | 초음파 트리거 |
| `light` | 광 트리거 |

---

### 4.5 Nanorobot

나노로봇 데이터 형식:

```json
{
  "data": {
    "robot_type": "medical",
    "robot_id": "NBOT-MED-001",
    "dimensions": {
      "length_nm": 500.0,
      "width_nm": 200.0,
      "height_nm": 200.0
    },
    "position": {
      "x_um": 1250.5,
      "y_um": 3200.8,
      "z_um": 150.2,
      "coordinate_system": "body_relative",
      "reference_organ": "liver"
    },
    "orientation": {
      "quaternion": { "w": 0.707, "x": 0.0, "y": 0.707, "z": 0.0 }
    },
    "navigation": {
      "mode": "autonomous",
      "target_um": { "x": 1500.0, "y": 3500.0, "z": 200.0 },
      "algorithm": "a_star",
      "eta_seconds": 120.5
    },
    "propulsion": {
      "type": "flagellar",
      "speed_um_per_s": 50.0,
      "power_fw": 10.0,
      "efficiency_percent": 75.0
    },
    "sensors": [
      {
        "id": "glucose_1",
        "type": "chemical",
        "target": "glucose",
        "value": 5.5,
        "unit": "mmol/L",
        "accuracy": 0.95
      },
      {
        "id": "ph_1",
        "type": "ph",
        "value": 7.2,
        "accuracy": 0.98
      }
    ],
    "actuators": [
      {
        "id": "gripper_1",
        "type": "gripper",
        "state": "open",
        "grip_force_pn": 0.0
      },
      {
        "id": "injector_1",
        "type": "drug_injector",
        "payload_remaining_percent": 80.0
      }
    ],
    "communication": {
      "protocol": "ultrasonic",
      "frequency_mhz": 40.0,
      "signal_strength_dbm": -50.0,
      "last_contact_ms_ago": 100
    },
    "power": {
      "source": "glucose_fuel_cell",
      "level_percent": 75.0,
      "consumption_fw": 15.0,
      "estimated_runtime_h": 5.0
    },
    "mission": {
      "type": "drug_delivery",
      "status": "in_progress",
      "objectives_completed": 2,
      "objectives_total": 5,
      "priority": "high"
    }
  }
}
```

#### Robot Types

| Type | Description |
|------|-------------|
| `medical` | 의료용 나노로봇 |
| `diagnostic` | 진단용 나노로봇 |
| `repair` | 조직 수리용 나노로봇 |
| `cleaning` | 청소/제거용 나노로봇 |

#### Propulsion Types

| Type | Description | Speed Range |
|------|-------------|-------------|
| `flagellar` | 편모 추진 | 10-100 μm/s |
| `chemical` | 화학 추진 | 1-50 μm/s |
| `magnetic` | 자기 추진 | 10-500 μm/s |
| `acoustic` | 음향 추진 | 1-100 μm/s |

---

### 4.6 Nanosensor

나노센서 데이터 형식:

```json
{
  "data": {
    "sensor_type": "quantum_dot",
    "sensor_id": "QD-TROP-001",
    "physical": {
      "diameter_nm": 5.0,
      "coating": "peg",
      "core_material": "CdSe",
      "shell_material": "ZnS",
      "functionalization": "anti_troponin_antibody"
    },
    "target": {
      "analyte": "troponin_i",
      "category": "cardiac_biomarker"
    },
    "measurement": {
      "value": 0.04,
      "unit": "ng/mL",
      "timestamp": "2025-01-15T10:30:00Z"
    },
    "performance": {
      "detection_limit": 0.001,
      "dynamic_range_min": 0.001,
      "dynamic_range_max": 100.0,
      "sensitivity": 0.95,
      "specificity": 0.98
    },
    "signal": {
      "raw_intensity": 45000,
      "baseline": 1000,
      "snr_db": 35.5,
      "emission_wavelength_nm": 650.0
    },
    "calibration": {
      "last_calibrated": "2025-01-15T08:00:00Z",
      "curve_id": "CAL-TROP-001",
      "r_squared": 0.9995,
      "valid_until": "2025-01-16T08:00:00Z"
    },
    "status": {
      "operational": true,
      "saturation_percent": 15.0,
      "remaining_lifetime_h": 48.0,
      "contamination_level": 0.02
    }
  }
}
```

#### Sensor Types

| Type | Principle | Applications |
|------|-----------|--------------|
| `quantum_dot` | 형광 발광 | 바이오마커, 이미징 |
| `nanowire` | 전도도 변화 | 가스, 이온 검출 |
| `plasmonic` | 표면 플라즈몬 공명 | 분자 검출 |
| `carbon_nanotube` | 전기적 특성 변화 | 가스, 화학물질 |
| `graphene` | 전도도 변화 | DNA, 단백질 |

---

## 5. Units and Coordinate Systems

### 5.1 Length Units

| Unit | Symbol | Size | Usage |
|------|--------|------|-------|
| Picometer | pm | 10⁻¹² m | 원자 반경, 결합 길이 |
| Angstrom | Å | 10⁻¹⁰ m | 분자 구조 (PDB 호환) |
| Nanometer | nm | 10⁻⁹ m | 나노입자, 나노머신 |
| Micrometer | μm | 10⁻⁶ m | 세포 내 위치, 나노로봇 |

### 5.2 Mass Units

| Unit | Symbol | Size | Usage |
|------|--------|------|-------|
| Dalton | Da | 1.66×10⁻²⁷ kg | 분자량 |
| Femtogram | fg | 10⁻¹⁵ g | 나노입자 질량 |
| Picogram | pg | 10⁻¹² g | 약물 용량 |

### 5.3 Energy Units

| Unit | Symbol | Usage |
|------|--------|-------|
| kJ/mol | kJ/mol | 결합 에너지, 활성화 에너지 |
| eV | eV | 전자 에너지 |
| kcal/mol | kcal/mol | 생화학 반응 |

### 5.4 Force Units

| Unit | Symbol | Size | Usage |
|------|--------|------|-------|
| Piconewton | pN | 10⁻¹² N | 분자 모터 토크, 결합력 |
| Femtonewton | fN | 10⁻¹⁵ N | 단백질 접힘 |

### 5.5 Power Units

| Unit | Symbol | Size | Usage |
|------|--------|------|-------|
| Femtowatt | fW | 10⁻¹⁵ W | 나노로봇 전력 |
| Attowatt | aW | 10⁻¹⁸ W | 분자 모터 |

---

## 6. Extensibility

### 6.1 Custom System Types

새로운 나노시스템 유형 추가:

```json
{
  "system_type": "custom",
  "custom_type": "nano_factory",
  "data": {
    "custom_fields": { }
  }
}
```

### 6.2 Extension Fields

표준 필드 외 확장:

```json
{
  "data": {
    "sensor_type": "quantum_dot",
    "extensions": {
      "vendor_specific": { },
      "experimental": { }
    }
  }
}
```

---

## 7. Versioning

### 7.1 Version Format

Semantic Versioning (semver) 사용:
- MAJOR: 호환되지 않는 변경
- MINOR: 하위 호환 기능 추가
- PATCH: 하위 호환 버그 수정

### 7.2 Compatibility

| Version | Compatibility |
|---------|---------------|
| 1.x.x | 1.0.0과 호환 |
| 2.x.x | Breaking changes |

---

## 8. References

1. ISO/TC 229 Nanotechnologies - https://www.iso.org/committee/381983.html
2. IEEE 1906.1-2015 - https://standards.ieee.org/ieee/1906.1/5171/
3. IEEE 1906.1.1-2020 - https://standards.ieee.org/standard/1906_1_1-2020.html
4. PDBx/mmCIF Dictionary - https://mmcif.wwpdb.org/
5. DNA Digital Data Storage - https://en.wikipedia.org/wiki/DNA_digital_data_storage
6. JSON Schema - https://json-schema.org/

---

<div align="center">

**WIA Nano Data Format Standard v1.0.0**

Phase 1 of 4

弘益人間 - *나노스케일에서 인류를 이롭게*

</div>
