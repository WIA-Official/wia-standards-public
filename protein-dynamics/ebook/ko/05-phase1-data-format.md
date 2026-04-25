# 제4장: 1단계 - 데이터 형식

## 단백질 동역학 데이터 표준화

**弘益人間 (홍익인간)**

---

## 4.1 표준화의 필요성

### 현재 상태: 파편화

단백질 동역학 데이터는 현재 호환되지 않는 형식으로 존재합니다:

| 소스 | 형식 | 문제점 |
|------|------|--------|
| MD 궤적 | XTC, DCD, TRR | 바이너리, 소프트웨어별 |
| NMR 동역학 | 텍스트, XML | 표준 스키마 없음 |
| 크라이오-EM 앙상블 | 다중 모델 PDB | 제한된 메타데이터 |
| 정규 모드 | 사용자 정의 배열 | 표준 표현 없음 |
| 유연성 메트릭 | CSV, JSON | 불일치하는 필드 |

### WIA 솔루션

어떤 소스의 동역학도 표현할 수 있는 통합 JSON 스키마:
- MD, NMR, 크라이오-EM, 또는 ML 기반 예측
- 사람이 읽을 수 있고 기계가 파싱 가능
- 검증 및 품질 검사 지원
- 데이터베이스 통합 가능
- 방법별 세부 사항 보존

---

## 4.2 핵심 스키마 구조

### 최상위 조직

```json
{
  "$schema": "https://wia.live/schemas/protein-dynamics/v1.0.0",
  "protein_dynamics": {
    "protein_id": "UniProt_ID",
    "metadata": { ... },
    "static_structure": { ... },
    "conformational_ensemble": { ... },
    "dynamics_metrics": { ... },
    "allosteric_network": { ... },
    "functional_dynamics": { ... }
  }
}
```

### 메타데이터 블록

```json
{
  "metadata": {
    "standard": "WIA-PROTEIN-DYNAMICS",
    "version": "1.0.0",
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "generator": {
      "software": "WIA-PD-Toolkit",
      "version": "1.2.0"
    },
    "source": {
      "type": "simulation",
      "method": "molecular_dynamics",
      "software": "GROMACS",
      "version": "2024.1"
    },
    "philosophy": "弘益人間"
  }
}
```

---

## 4.3 정적 구조 참조

### 실험적/예측된 구조 연결

```json
{
  "static_structure": {
    "pdb_ids": ["1ATP", "2GS2", "3POZ"],
    "alphafold_id": "AF-P00533-F1",
    "primary_reference": "1ATP",

    "alphafold_confidence": {
      "mean_plddt": 87.3,
      "plddt_per_residue": [92.1, 91.5, 88.2, ...],
      "high_confidence_regions": [
        {"start": 1, "end": 450, "mean_plddt": 91.2}
      ],
      "low_confidence_regions": [
        {"start": 451, "end": 520, "mean_plddt": 45.3, "interpretation": "likely_disordered"}
      ]
    }
  }
}
```

---

## 4.4 형태 앙상블

### 완전한 앙상블 표현

```json
{
  "conformational_ensemble": {
    "num_states": 5,
    "generation_method": {
      "type": "enhanced_sampling",
      "subtype": "metadynamics",
      "software": "GROMACS+PLUMED"
    },

    "states": [
      {
        "state_id": "ground",
        "name": "활성 키나아제",
        "population": 0.65,
        "population_error": 0.03,
        "coordinates": {
          "pdb_file": "state_ground.pdb"
        },
        "relative_energy": {
          "value": 0.0,
          "unit": "kcal/mol"
        },
        "key_features": [
          "DFG-in 형태",
          "αC-나선 in",
          "활성화 루프 확장"
        ]
      }
    ],

    "free_energy_landscape": {
      "collective_variables": ["CV1", "CV2"],
      "minima": [...],
      "barriers": [...]
    }
  }
}
```

---

## 4.5 동역학 메트릭

### 다중 시간 척도 특성화

```json
{
  "dynamics_metrics": {
    "timescales": {
      "ps_motions": {
        "description": "결합 진동, 메틸 회전, 루프 요동",
        "amplitude_angstrom": 0.5
      },
      "ns_motions": {
        "description": "측쇄 회전, 루프 이동, 작은 도메인 운동",
        "amplitude_angstrom": 2.0
      },
      "us_ms_motions": {
        "description": "대규모 형태 변화, 도메인 이동",
        "rate_per_second": 1000
      }
    },

    "flexibility": {
      "b_factors": {
        "source": "MD_derived",
        "values": [15.2, 14.8, 18.3, ...]
      },
      "rmsf": {
        "values": [0.8, 0.7, 1.2, ...],
        "units": "angstrom"
      },
      "order_parameters_S2": {
        "values": [0.92, 0.90, 0.45, ...]
      },
      "flexible_regions": [
        {
          "start": 145,
          "end": 165,
          "type": "activation_loop",
          "mean_rmsf": 3.2
        }
      ]
    },

    "disorder": {
      "idr_regions": [
        {
          "start": 980,
          "end": 1050,
          "disorder_score": 0.85,
          "function": "조절 꼬리"
        }
      ],
      "total_disorder_percent": 12.5
    },

    "dynamics_index": {
      "value": 0.72,
      "interpretation": "상당한 조절 유연성을 가진 중간 정도의 동적 단백질"
    }
  }
}
```

---

## 4.6 알로스테릭 네트워크

### 통신 경로 매핑

```json
{
  "allosteric_network": {
    "active_sites": [
      {
        "name": "ATP_binding",
        "residues": [695, 745, 762, 790, 855]
      }
    ],

    "allosteric_sites": [
      {
        "name": "Juxtamembrane",
        "residues": [650, 653, 656, 660],
        "effect": "inhibitory",
        "mechanism": "αC-나선 회전 차단"
      }
    ],

    "communication_pathways": [
      {
        "id": "pathway_1",
        "from_site": "Juxtamembrane",
        "to_site": "ATP_binding",
        "pathway_residues": [656, 680, 695, 720, 745, 762],
        "correlation": 0.78,
        "mechanism": "백본 수소 결합 네트워크"
      }
    ]
  }
}
```

---

## 4.7 약물 결합 동역학 스키마

### 완전한 결합 특성화

```json
{
  "drug_binding_dynamics": {
    "complex_id": "EGFR_Erlotinib",
    "protein_id": "P00533",
    "ligand": {
      "name": "Erlotinib",
      "pubchem_cid": 176870,
      "smiles": "COc1cc2ncnc(Nc3cccc(c3)C#C)c2cc1OCCOC"
    },

    "binding_site": {
      "residues": [695, 719, 743, 745, 762, 790],
      "druggability_score": 0.92
    },

    "thermodynamics": {
      "delta_g": {"value": -10.2, "unit": "kcal/mol"},
      "delta_h": {"value": -12.5, "unit": "kcal/mol"},
      "t_delta_s": {"value": -2.3, "unit": "kcal/mol"},
      "kd": {"value": 0.5, "unit": "nM"}
    },

    "kinetics": {
      "kon": {"value": 2.5e6, "unit": "M-1s-1"},
      "koff": {"value": 1.3e-3, "unit": "s-1"},
      "residence_time": {"value": 770, "unit": "seconds"}
    },

    "binding_pathway": {
      "mechanism": "conformational_selection",
      "binding_competent_population": 0.25
    }
  }
}
```

---

## 4.8 검증 및 품질 관리

### 스키마 검증

```python
import jsonschema
from jsonschema import validate

def validate_protein_dynamics(data, schema_version="1.0.0"):
    """
    WIA 스키마에 대해 단백질 동역학 JSON 검증.
    """
    schema = load_wia_schema(schema_version)

    try:
        validate(instance=data, schema=schema)
        return {'valid': True, 'errors': []}
    except jsonschema.ValidationError as e:
        return {'valid': False, 'errors': [str(e)]}
```

### 품질 검사

```python
def quality_check_ensemble(ensemble_data):
    """형태 앙상블 데이터의 품질 검사."""
    checks = []

    # 집단 합계가 1인지 확인
    total_pop = sum(s['population'] for s in ensemble_data['states'])
    checks.append({
        'name': 'population_sum',
        'passed': abs(total_pop - 1.0) < 0.01,
        'value': total_pop
    })

    return checks
```

---

## 요약

WIA-PROTEIN-DYNAMICS 데이터 형식은 다음을 제공합니다:
- 모든 동역학 데이터 유형에 대한 통합 표현
- 메타데이터에서 기능적 동역학까지의 명확한 계층 구조
- 형태 앙상블의 풍부한 주석
- 결합 동역학의 완전한 특성화
- 내장된 검증 및 품질 검사
- 도구와 데이터베이스 간 상호운용성

---

**다음 장:** [2단계: API 인터페이스](./06-phase2-api-interface.md)

弘益人間 - 널리 인간을 이롭게 하라
