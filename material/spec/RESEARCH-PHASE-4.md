# Phase 4 Research Findings
# 사전 조사 결과

---

**Date**: 2025-12-14
**Author**: Claude Code (Opus 4.5)
**Purpose**: WIA Material 생태계 연동을 위한 기술 조사

---

## Table of Contents

1. [외부 재료 데이터베이스 API](#1-외부-재료-데이터베이스-api)
2. [데이터 교환 형식](#2-데이터-교환-형식)
3. [구조 시각화 도구](#3-구조-시각화-도구)
4. [상호운용성 라이브러리](#4-상호운용성-라이브러리)
5. [결론 및 설계 방향](#5-결론-및-설계-방향)

---

## 1. 외부 재료 데이터베이스 API

### 1.1 Materials Project API

**URL**: https://materialsproject.org

**개요**:
- Lawrence Berkeley National Laboratory 운영
- 150,000+ 무기 화합물 데이터
- mp-api Python 클라이언트 제공

**API 특징**:
```python
from mp_api.client import MPRester

with MPRester("API_KEY") as mpr:
    # 물질 검색
    docs = mpr.materials.summary.search(
        elements=["Li", "Fe", "O"],
        fields=["material_id", "formula_pretty", "band_gap"]
    )
```

**주요 필드**:
| 필드 | 설명 |
|------|------|
| `material_id` | mp-XXXXX 형식 ID |
| `formula_pretty` | 화학식 |
| `structure` | pymatgen Structure |
| `band_gap` | 밴드갭 (eV) |
| `formation_energy_per_atom` | 생성 에너지 |
| `density` | 밀도 |

**WIA Material 연동**:
- external_references.materials_project_id 매핑
- structure → Phase 1 Structure 변환
- band_gap → properties.electrical.band_gap_ev

### 1.2 OPTIMADE API

**URL**: https://www.optimade.org

**개요**:
- Open Databases Integration for Materials Design
- 60M+ 구조, 30+ 데이터베이스 연합
- RESTful API, JSON:API 스펙 준수

**버전**: 1.2.0 (2024)

**클라이언트 라이브러리**:
- `optimade` Python 패키지 (PyPI)
- optimade-python-tools

**API 형식**:
```
GET https://provider.org/v1/structures?filter=elements HAS "Fe"
GET https://provider.org/v1/structures/{id}
```

**응답 형식**:
```json
{
  "data": {
    "type": "structures",
    "id": "example/1",
    "attributes": {
      "chemical_formula_reduced": "Fe2O3",
      "nelements": 2,
      "elements": ["Fe", "O"],
      "lattice_vectors": [[...], [...], [...]]
    }
  },
  "meta": {
    "api_version": "1.2.0"
  }
}
```

**WIA Material 연동**:
- Phase 3 REST API와 호환 설계
- filter 문법 상호 변환
- 구조 데이터 매핑

### 1.3 NOMAD

**URL**: https://nomad-lab.eu

**개요**:
- 유럽 연구 인프라
- 1억+ 시뮬레이션 데이터
- FAIR 원칙 준수

**특징**:
- RESTful API
- NeXus 형식 (HDF5 기반)
- OPTIMADE 호환

**데이터 형식**:
- JSON (메타데이터)
- HDF5 (대용량 배열)

### 1.4 AFLOW

**URL**: http://aflowlib.org

**개요**:
- Duke University 운영
- 360만+ 화합물
- AFLUX 쿼리 언어

**API 형식**:
```
http://aflowlib.org/API/aflux/?
  catalog(lib)
  filter(Egap>0.5)
  format(json)
```

---

## 2. 데이터 교환 형식

### 2.1 CIF (Crystallographic Information File)

**표준**: IUCr (International Union of Crystallography)

**특징**:
- 텍스트 기반 자기 설명 형식
- STAR 파일 구조
- 결정학 데이터 표준

**예시**:
```cif
data_YBCO
_chemical_formula_sum 'Ba2 Cu3 O7 Y'
_cell_length_a 3.8231
_cell_length_b 3.8864
_cell_length_c 11.6807
_cell_angle_alpha 90
_cell_angle_beta 90
_cell_angle_gamma 90
_symmetry_space_group_name_H-M 'P m m m'
```

**파서 라이브러리**:
| 라이브러리 | 언어 | 특징 |
|-----------|------|------|
| PyCifRW | Python | IUCr 공식 지원 |
| GEMMI | C++/Python | 고성능, mmCIF 지원 |
| iotbx.cif | Python | 종합 툴박스 |
| crystcif-parse | JavaScript | 웹 브라우저용 |

**WIA Material 연동**:
- CIF → Phase 1 Structure 변환
- Phase 1 → CIF 내보내기

### 2.2 VASP POSCAR

**용도**: VASP 시뮬레이션 입력 파일

**구조**:
```
YBCO                    # 주석
1.0                     # 스케일링 팩터
3.8231  0.0000  0.0000  # 격자 벡터 a
0.0000  3.8864  0.0000  # 격자 벡터 b
0.0000  0.0000  11.6807 # 격자 벡터 c
Y Ba Cu O               # 원소 (VASP 5+)
1 2  3  7               # 각 원소 개수
Direct                  # 좌표계 (Direct/Cartesian)
0.5 0.5 0.5             # 원자 좌표
...
```

**Rust 라이브러리**:
- `vasp-poscar` crate (crates.io)
  - POSCAR 읽기/쓰기
  - Direct ↔ Cartesian 변환

### 2.3 XYZ Format

**용도**: 분자 좌표

**형식**:
```
12
YBCO unit cell
Y   0.5  0.5  0.5
Ba  0.0  0.0  0.18
...
```

### 2.4 PDB (Protein Data Bank)

**용도**: 주로 생체 분자, 일부 무기 구조

**특징**:
- ATOM/HETATM 레코드
- 좌표 + 연결 정보

---

## 3. 구조 시각화 도구

### 3.1 웹 기반 도구

| 도구 | URL | 특징 |
|------|-----|------|
| **MolView** | molview.org | WebGL, 데이터베이스 검색 |
| **Mol*** | molstar.org | 고품질 렌더링, 플러그인 |
| **3DStructGen** | - | 분자/결정 편집 |
| **Materials Cloud** | materialscloud.org | OPTIMADE 클라이언트 |

### 3.2 데스크톱 도구

| 도구 | 특징 | 라이선스 |
|------|------|----------|
| **Mercury** | CSD 통합, 고급 분석 | 무료 버전 있음 |
| **VESTA** | 결정/전자밀도 시각화 | 무료 |
| **CrystalMaker** | 교육용, Mac/Windows | 상용 |
| **OVITO** | 시뮬레이션 분석 | 오픈소스 |
| **Jmol** | Java 기반, 크로스플랫폼 | 오픈소스 |

### 3.3 프로그래밍 라이브러리

| 라이브러리 | 언어 | 특징 |
|-----------|------|------|
| **py3Dmol** | Python | Jupyter 통합 |
| **nglview** | Python | 대화형 위젯 |
| **three.js** | JavaScript | WebGL 3D 렌더링 |
| **vtk** | C++/Python | 과학 시각화 |

---

## 4. 상호운용성 라이브러리

### 4.1 Pymatgen

**URL**: https://pymatgen.org

**개요**:
- Python Materials Genomics
- Materials Project 공식 라이브러리
- 광범위한 IO 지원

**지원 형식**:
```python
from pymatgen.core import Structure
from pymatgen.io.cif import CifWriter
from pymatgen.io.vasp import Poscar
from pymatgen.io.ase import AseAtomsAdaptor

# CIF 읽기
structure = Structure.from_file("structure.cif")

# POSCAR 쓰기
poscar = Poscar(structure)
poscar.write_file("POSCAR")

# ASE 변환
atoms = AseAtomsAdaptor.get_atoms(structure)
```

**WIA Material 연동**:
- pymatgen.core.Structure ↔ Phase 1 Structure
- IO 모듈을 통한 형식 변환

### 4.2 ASE (Atomic Simulation Environment)

**URL**: https://wiki.fysik.dtu.dk/ase

**개요**:
- Python 원자 시뮬레이션 도구
- LGPL 라이선스
- 광범위한 형식 지원

**주요 기능**:
```python
from ase import Atoms
from ase.io import read, write

# 구조 읽기
atoms = read("structure.cif")

# 다른 형식으로 쓰기
write("structure.xyz", atoms)
write("POSCAR", atoms, format="vasp")
```

**지원 형식**:
- CIF, POSCAR, XYZ, PDB
- Quantum ESPRESSO, GPAW
- LAMMPS, GROMACS
- 등 50+ 형식

### 4.3 GEMMI

**URL**: https://gemmi.readthedocs.io

**개요**:
- C++11 라이브러리 + Python 바인딩
- 고성능 CIF/mmCIF 파서
- 결정학 데이터 처리

**특징**:
- 매우 빠른 파싱
- CIF 1.1 + 확장 지원
- 구조 조작 도구

---

## 5. 결론 및 설계 방향

### 5.1 Integration Layer 설계

Phase 4에서 구현할 연동 계층:

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Material Core                         │
│                  (Phase 1-3 구현체)                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Integration Layer                          │
│                     (Phase 4)                                │
├──────────────────┬──────────────────┬───────────────────────┤
│   DataProviders  │    Exporters     │    Visualizers        │
│   (데이터 연동)   │   (형식 변환)    │    (시각화)           │
├──────────────────┼──────────────────┼───────────────────────┤
│ MaterialsProject │ CIF Exporter     │ Structure3D           │
│ OPTIMADE Client  │ POSCAR Exporter  │ PropertyPlot          │
│ NOMAD Client     │ XYZ Exporter     │ (향후 확장)           │
└──────────────────┴──────────────────┴───────────────────────┘
                              │
                              ▼
         ┌────────────────────┴────────────────────┐
         │                                         │
    External DBs                              File Formats
    ├─ Materials Project                      ├─ CIF
    ├─ OPTIMADE Providers                     ├─ POSCAR
    ├─ NOMAD                                  ├─ XYZ
    └─ AFLOW                                  └─ PDB
```

### 5.2 권장 구현 우선순위

1. **필수 (Core)**
   - CIF 내보내기/가져오기
   - POSCAR 내보내기/가져오기
   - OPTIMADE 클라이언트

2. **권장 (Recommended)**
   - Materials Project 클라이언트
   - XYZ 내보내기
   - 기본 구조 시각화 데이터

3. **선택 (Optional)**
   - NOMAD/AFLOW 클라이언트
   - PDB 형식 지원
   - 고급 시각화

### 5.3 Rust 모듈 구조

```
src/
├── integration/
│   ├── mod.rs                 # 모듈 진입점
│   ├── providers/             # 데이터 제공자
│   │   ├── mod.rs
│   │   ├── optimade.rs        # OPTIMADE 클라이언트
│   │   └── materials_project.rs # MP 클라이언트
│   ├── exporters/             # 형식 변환
│   │   ├── mod.rs
│   │   ├── cif.rs             # CIF 내보내기
│   │   ├── poscar.rs          # POSCAR 내보내기
│   │   └── xyz.rs             # XYZ 내보내기
│   └── visualizers/           # 시각화 데이터
│       ├── mod.rs
│       └── structure.rs       # 구조 시각화 데이터
```

### 5.4 인터페이스 설계

#### DataProvider Trait
```rust
#[async_trait]
pub trait DataProvider: Send + Sync {
    fn name(&self) -> &str;

    async fn connect(&mut self, config: ProviderConfig) -> Result<()>;
    async fn disconnect(&mut self) -> Result<()>;

    async fn search(&self, query: &ProviderQuery) -> Result<Vec<MaterialData>>;
    async fn get_by_id(&self, id: &str) -> Result<MaterialData>;

    fn is_connected(&self) -> bool;
}
```

#### Exporter Trait
```rust
pub trait Exporter: Send + Sync {
    fn format(&self) -> ExportFormat;

    fn export(&self, material: &MaterialData) -> Result<String>;
    fn export_to_file(&self, material: &MaterialData, path: &Path) -> Result<()>;

    fn import(&self, content: &str) -> Result<MaterialData>;
    fn import_from_file(&self, path: &Path) -> Result<MaterialData>;
}
```

---

## References

### APIs
- [Materials Project API](https://api.materialsproject.org)
- [OPTIMADE Specification](https://www.optimade.org/optimade)
- [NOMAD Repository](https://nomad-lab.eu)
- [AFLOW](http://aflowlib.org)

### File Formats
- [IUCr CIF](https://www.iucr.org/resources/cif)
- [VASP POSCAR](https://www.vasp.at/wiki/index.php/POSCAR)

### Libraries
- [pymatgen](https://pymatgen.org)
- [ASE](https://wiki.fysik.dtu.dk/ase)
- [GEMMI](https://gemmi.readthedocs.io)
- [vasp-poscar crate](https://crates.io/crates/vasp-poscar)

### Visualization
- [Mol*](https://molstar.org)
- [MolView](https://molview.org)
- [CCDC Mercury](https://www.ccdc.cam.ac.uk/solutions/software/mercury/)

---

<div align="center">

**Phase 4 Research Complete**

---

弘益人間 🤟

</div>
