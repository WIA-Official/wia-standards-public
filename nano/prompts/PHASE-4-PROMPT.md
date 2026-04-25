# Phase 4: Ecosystem Integration
## Claude Code 작업 프롬프트

---

**Standard**: WIA Nano
**Phase**: 4 of 4
**목표**: 나노기술 데이터를 외부 시스템과 연동
**난이도**: ★★★★★
**예상 작업량**: 스펙 문서 1개 + 출력 모듈 구현 + 예제

---

## 🎯 Phase 4 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 API Interface를 만들고,
 Phase 3에서 Communication Protocol을 정의했다.

 이제 WIA Nano 데이터를 외부 시스템과 어떻게 연동할 것인가?

 - LAMMPS로 분자 동역학 시뮬레이션?
 - VMD로 3D 분자 시각화?
 - PDB 형식으로 내보내기?

 모든 출력 방식에서 동일한 인터페이스를 사용할 수 있을까?"
```

### 목표
```
WIA Nano 데이터 → 외부 시스템 연동

출력 경로:
├─ Visualization: 분자 시각화 (VMD, PyMOL, Chimera)
├─ Export: 표준 형식 내보내기 (PDB, XYZ, CIF, MOL2)
├─ Simulation: 시뮬레이션 도구 (LAMMPS, GROMACS, NAMD)
└─ Analysis: 분석 도구 (Python, MATLAB)

단일 API로 모든 출력 방식 지원
```

---

## 📋 Phase 1-3 결과물 활용

| 이전 Phase 산출물 | Phase 4 활용 |
|-----------------|-------------|
| Phase 1: Data Format | 내보내기 데이터 소스 |
| Phase 2: Rust API | 데이터 처리 API |
| Phase 3: Protocol | 실시간 데이터 스트리밍 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 시각화 도구 조사

| 도구 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **VMD** | Visual Molecular Dynamics | "VMD molecular visualization PDB trajectory" |
| **PyMOL** | 분자 그래픽 도구 | "PyMOL protein structure visualization tutorial" |
| **Chimera** | UCSF Chimera | "UCSF Chimera molecular modeling" |
| **NGL Viewer** | 웹 기반 뷰어 | "NGL Viewer WebGL molecular visualization" |
| **Avogadro** | 분자 편집기 | "Avogadro molecular editor" |

### 2단계: 데이터 형식 조사

| 형식 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **PDB** | Protein Data Bank | "PDB file format specification ATOM HETATM" |
| **XYZ** | Cartesian coordinates | "XYZ molecular coordinate file format" |
| **CIF** | Crystallographic Information | "mmCIF file format molecular structure" |
| **MOL2** | Tripos Mol2 | "MOL2 file format Tripos molecular" |
| **SMILES** | 화학 구조 표기 | "SMILES notation chemical structure" |
| **LAMMPS Data** | LAMMPS 입력 | "LAMMPS data file format atoms bonds" |

### 3단계: 시뮬레이션 도구 조사

| 도구 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **LAMMPS** | 분자 동역학 | "LAMMPS molecular dynamics simulation input" |
| **GROMACS** | MD 시뮬레이션 | "GROMACS topology file force field" |
| **NAMD** | 나노스케일 MD | "NAMD configuration file simulation" |
| **Quantum ESPRESSO** | 양자 시뮬레이션 | "Quantum ESPRESSO input file DFT" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-4.md`에 다음을 정리:

```markdown
# Phase 4 사전 조사 결과

## 1. 분자 시각화 도구 비교

### VMD (Visual Molecular Dynamics)
- 기능: [조사 내용]
- 지원 형식: [조사 내용]
- WIA Nano 적용: [분석]

### PyMOL
- 기능: [조사 내용]
- 스크립팅: [조사 내용]
- WIA Nano 적용: [분석]

## 2. 데이터 내보내기 형식

### PDB (Protein Data Bank)
- 형식 구조: [조사 내용]
- ATOM/HETATM 레코드: [조사 내용]

### LAMMPS Data File
- 형식 구조: [조사 내용]
- 원자/결합 정의: [조사 내용]

## 3. 시뮬레이션 도구 연동

### LAMMPS
- 입력 파일 구조: [조사 내용]
- 힘장 정의: [조사 내용]

### GROMACS
- 토폴로지 파일: [조사 내용]
- 시뮬레이션 파라미터: [조사 내용]

## 4. 결론
- 권장 내보내기 형식: [제안]
- 시각화 파이프라인: [제안]
- 시뮬레이션 통합: [제안]
```

---

## 🏗️ 출력 연동 설계

### 1. 출력 인터페이스 (Output Interface)

#### 기본 출력 인터페이스
```rust
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// 출력 유형
    fn output_type(&self) -> OutputType;

    /// 어댑터 이름
    fn name(&self) -> &str;

    /// 초기화
    async fn initialize(&mut self, config: &OutputConfig) -> Result<(), OutputError>;

    /// 출력
    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError>;

    /// 사용 가능 여부
    fn is_available(&self) -> bool;

    /// 정리
    async fn dispose(&mut self) -> Result<(), OutputError>;
}

pub enum OutputType {
    Visualization,  // 분자 시각화
    Export,         // 데이터 내보내기
    Simulation,     // 시뮬레이션
    Analysis,       // 분석
    Custom(String), // 사용자 정의
}
```

### 2. PDB Exporter

#### PDB 형식 구조
```
HEADER    NANOROBOT DNA ORIGAMI          01-JAN-25   1XXX
TITLE     WIA NANO STANDARD NANOROBOT
AUTHOR    WIA STANDARDS
ATOM      1  C   DNA A   1      10.000  20.000  30.000  1.00 20.00           C
ATOM      2  N   DNA A   1      11.200  20.500  30.200  1.00 20.00           N
...
CONECT    1    2    3
END
```

#### Rust 구현
```rust
pub struct PdbExporter {
    pub name: String,
    pub title: String,
    pub author: String,
}

impl PdbExporter {
    pub fn export_pdb(&self, molecule: &Molecule) -> Result<String, OutputError> {
        let mut pdb = String::new();

        // Header
        pdb.push_str(&format!("HEADER    {:<40} {:>9}\n",
                             self.title,
                             chrono::Local::now().format("%d-%b-%y")));

        pdb.push_str(&format!("TITLE     {}\n", self.title));
        pdb.push_str(&format!("AUTHOR    {}\n", self.author));

        // Atoms
        for (i, atom) in molecule.atoms.iter().enumerate() {
            let atom_line = format!(
                "ATOM  {:5}  {:<4}{:<4}A{:4}    {:8.3}{:8.3}{:8.3}{:6.2}{:6.2}          {:>2}\n",
                i + 1,
                atom.element,
                "MOL",
                1,
                atom.position_nm.x,
                atom.position_nm.y,
                atom.position_nm.z,
                1.00,
                20.00,
                atom.element
            );
            pdb.push_str(&atom_line);
        }

        // Bonds
        for (i, bond) in molecule.bonds.iter().enumerate() {
            if i % 4 == 0 {
                if i > 0 {
                    pdb.push('\n');
                }
                pdb.push_str(&format!("CONECT{:5}", bond.atom1_index + 1));
            }
            pdb.push_str(&format!("{:5}", bond.atom2_index + 1));
        }

        pdb.push_str("\nEND\n");

        Ok(pdb)
    }
}
```

### 3. XYZ Exporter

#### XYZ 형식 구조
```
10
WIA Nano Molecule
C   10.000  20.000  30.000
N   11.200  20.500  30.200
...
```

#### Rust 구현
```rust
pub struct XyzExporter {
    pub name: String,
    pub comment: String,
}

impl XyzExporter {
    pub fn export_xyz(&self, molecule: &Molecule) -> Result<String, OutputError> {
        let mut xyz = String::new();

        // Atom count
        xyz.push_str(&format!("{}\n", molecule.atoms.len()));

        // Comment line
        xyz.push_str(&format!("{}\n", self.comment));

        // Atoms
        for atom in &molecule.atoms {
            xyz.push_str(&format!(
                "{:<2}  {:12.6}  {:12.6}  {:12.6}\n",
                atom.element,
                atom.position_nm.x,
                atom.position_nm.y,
                atom.position_nm.z
            ));
        }

        Ok(xyz)
    }
}
```

### 4. LAMMPS Data Exporter

#### LAMMPS 데이터 파일 구조
```
LAMMPS Data File - WIA Nano

10 atoms
5 bonds
2 atom types
1 bond types

0.0 100.0 xlo xhi
0.0 100.0 ylo yhi
0.0 100.0 zlo zhi

Atoms

1 1 1 0.0 10.0 20.0 30.0
2 1 1 0.0 11.2 20.5 30.2
...

Bonds

1 1 1 2
...
```

#### Rust 구현
```rust
pub struct LammpsDataExporter {
    pub name: String,
    pub box_bounds: (f64, f64, f64),
}

impl LammpsDataExporter {
    pub fn export_lammps(&self, molecule: &Molecule) -> Result<String, OutputError> {
        let mut lmp = String::new();

        // Header
        lmp.push_str("LAMMPS Data File - WIA Nano\n\n");

        // Counts
        lmp.push_str(&format!("{} atoms\n", molecule.atoms.len()));
        lmp.push_str(&format!("{} bonds\n", molecule.bonds.len()));

        // Atom types (simplified - would need proper type mapping)
        let atom_types = self.count_atom_types(molecule);
        lmp.push_str(&format!("{} atom types\n", atom_types));

        let bond_types = 1; // Simplified
        lmp.push_str(&format!("{} bond types\n\n", bond_types));

        // Box bounds
        lmp.push_str(&format!("0.0 {} xlo xhi\n", self.box_bounds.0));
        lmp.push_str(&format!("0.0 {} ylo yhi\n", self.box_bounds.1));
        lmp.push_str(&format!("0.0 {} zlo zhi\n\n", self.box_bounds.2));

        // Atoms
        lmp.push_str("Atoms\n\n");
        for (i, atom) in molecule.atoms.iter().enumerate() {
            let atom_type = self.get_atom_type(&atom.element);
            lmp.push_str(&format!(
                "{} 1 {} {} {} {} {}\n",
                i + 1,
                atom_type,
                atom.charge,
                atom.position_nm.x,
                atom.position_nm.y,
                atom.position_nm.z
            ));
        }

        // Bonds
        if !molecule.bonds.is_empty() {
            lmp.push_str("\nBonds\n\n");
            for (i, bond) in molecule.bonds.iter().enumerate() {
                lmp.push_str(&format!(
                    "{} 1 {} {}\n",
                    i + 1,
                    bond.atom1_index + 1,
                    bond.atom2_index + 1
                ));
            }
        }

        Ok(lmp)
    }

    fn count_atom_types(&self, molecule: &Molecule) -> usize {
        let mut types = std::collections::HashSet::new();
        for atom in &molecule.atoms {
            types.insert(&atom.element);
        }
        types.len()
    }

    fn get_atom_type(&self, element: &str) -> usize {
        // Simplified mapping
        match element {
            "C" => 1,
            "N" => 2,
            "O" => 3,
            "H" => 4,
            _ => 5,
        }
    }
}
```

### 5. GROMACS Topology Exporter

```rust
pub struct GromacsTopExporter {
    pub name: String,
    pub force_field: String,
}

impl GromacsTopExporter {
    pub fn export_topology(&self, molecule: &Molecule) -> Result<String, OutputError> {
        let mut top = String::new();

        // Header
        top.push_str("; GROMACS Topology - WIA Nano\n");
        top.push_str(&format!("; Generated by WIA Nano Standard\n\n"));

        // Force field
        top.push_str(&format!("#include \"{}.ff/forcefield.itp\"\n\n", self.force_field));

        // Molecule type
        top.push_str("[ moleculetype ]\n");
        top.push_str("; Name  nrexcl\n");
        top.push_str(&format!("{}      3\n\n", molecule.name));

        // Atoms
        top.push_str("[ atoms ]\n");
        top.push_str(";   nr  type  resnr residue  atom   cgnr     charge       mass\n");
        for (i, atom) in molecule.atoms.iter().enumerate() {
            top.push_str(&format!(
                "{:6}  {:>4}  {:5}  {:>5}  {:>4}  {:5}  {:10.6}  {:10.5}\n",
                i + 1,
                self.get_gromacs_type(&atom.element),
                1,
                "MOL",
                atom.element,
                i + 1,
                atom.charge,
                atom.mass_daltons
            ));
        }

        // Bonds
        if !molecule.bonds.is_empty() {
            top.push_str("\n[ bonds ]\n");
            top.push_str(";  ai    aj   funct   c0   c1\n");
            for bond in &molecule.bonds {
                top.push_str(&format!(
                    "{:6}{:6}      1\n",
                    bond.atom1_index + 1,
                    bond.atom2_index + 1
                ));
            }
        }

        // System
        top.push_str("\n[ system ]\n");
        top.push_str(&format!("{}\n\n", molecule.name));

        // Molecules
        top.push_str("[ molecules ]\n");
        top.push_str(&format!("{}  1\n", molecule.name));

        Ok(top)
    }

    fn get_gromacs_type(&self, element: &str) -> &str {
        match element {
            "C" => "opls_135",
            "N" => "opls_237",
            "O" => "opls_236",
            "H" => "opls_140",
            _ => "opls_001",
        }
    }
}
```

### 6. 통합 출력 매니저

```rust
pub struct OutputManager {
    adapters: HashMap<String, Box<dyn OutputAdapter>>,
}

impl OutputManager {
    pub fn new() -> Self {
        Self {
            adapters: HashMap::new(),
        }
    }

    /// 어댑터 등록
    pub async fn register(
        &mut self,
        name: String,
        adapter: Box<dyn OutputAdapter>,
    ) -> NanoResult<()> {
        self.adapters.insert(name, adapter);
        Ok(())
    }

    /// 특정 어댑터로 출력
    pub async fn output_to(
        &self,
        name: &str,
        data: &OutputData,
    ) -> Result<OutputResult, OutputError> {
        match self.adapters.get(name) {
            Some(adapter) => adapter.output(data).await,
            None => Err(OutputError::AdapterNotFound(name.to_string())),
        }
    }

    /// 모든 어댑터로 브로드캐스트
    pub async fn broadcast(
        &self,
        output_type: OutputType,
        data: &OutputData,
    ) -> Vec<Result<OutputResult, OutputError>> {
        let mut results = Vec::new();

        for adapter in self.adapters.values() {
            if std::mem::discriminant(&adapter.output_type())
                == std::mem::discriminant(&output_type)
            {
                results.push(adapter.output(data).await);
            }
        }

        results
    }

    /// 사용 가능한 어댑터 목록
    pub fn list_available(&self, output_type: Option<OutputType>) -> Vec<String> {
        self.adapters
            .iter()
            .filter(|(_, adapter)| {
                output_type.is_none()
                    || std::mem::discriminant(&adapter.output_type())
                        == std::mem::discriminant(&output_type.as_ref().unwrap())
            })
            .map(|(name, _)| name.clone())
            .collect()
    }
}
```

---

## 📁 산출물 목록

Phase 4 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-4.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-4-INTEGRATION.md

내용:
1. 개요 (Overview)
2. 출력 계층 아키텍처 (Output Layer Architecture)
3. 출력 인터페이스 (Output Interface)
4. 분자 시각화 (Molecular Visualization)
   - VMD, PyMOL, Chimera
5. 데이터 내보내기 (Data Export)
   - PDB, XYZ, CIF, MOL2
   - LAMMPS, GROMACS
6. 시뮬레이션 통합 (Simulation Integration)
7. 분석 도구 연동 (Analysis Tools)
8. 통합 출력 매니저 (Output Manager)
9. 에러 처리 (Error Handling)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. Rust 출력 모듈
```
/api/rust/src/
├── output/
│   ├── mod.rs
│   ├── adapter.rs              # 출력 인터페이스
│   ├── manager.rs              # 통합 매니저
│   ├── exporter.rs             # 내보내기 구현
│   │   - PdbExporter
│   │   - XyzExporter
│   │   - CifExporter
│   │   - Mol2Exporter
│   │   - LammpsDataExporter
│   │   - GromacsTopExporter
│   │   - JsonExporter
│   ├── data.rs                 # 출력 데이터 타입
│   └── error.rs                # 에러 타입
└── ...
```

### 4. 예제 코드
```
/api/rust/examples/
├── output_demo.rs              # 출력 계층 데모
├── pdb_export.rs               # PDB 내보내기 예제
├── lammps_export.rs            # LAMMPS 내보내기
└── visualization_pipeline.rs   # 시각화 파이프라인
```

---

## ✅ 완료 체크리스트

Phase 4 완료 전 확인:

```
□ 웹서치로 시각화/내보내기 기술 조사 완료
□ /spec/RESEARCH-PHASE-4.md 작성 완료
□ /spec/PHASE-4-INTEGRATION.md 작성 완료
□ Rust output 모듈 구현 완료
□ PDB Exporter 구현 완료
□ XYZ Exporter 구현 완료
□ LAMMPS Data Exporter 구현 완료
□ GROMACS Topology Exporter 구현 완료
□ CIF Exporter 구현 완료
□ MOL2 Exporter 구현 완료
□ OutputManager 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 예제 코드 완료
□ README 업데이트 (Phase 4 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 시각화/내보내기 기술 조사
   ↓
2. /spec/RESEARCH-PHASE-4.md 작성
   ↓
3. 출력 인터페이스 설계
   ↓
4. /spec/PHASE-4-INTEGRATION.md 작성
   ↓
5. Rust OutputAdapter trait 정의
   ↓
6. PDB Exporter 구현
   ↓
7. XYZ Exporter 구현
   ↓
8. LAMMPS Data Exporter 구현
   ↓
9. GROMACS Topology Exporter 구현
   ↓
10. CIF/MOL2 Exporter 구현
   ↓
11. OutputManager 구현
   ↓
12. 테스트 작성 및 실행
   ↓
13. 예제 코드 작성
   ↓
14. 완료 체크리스트 확인
   ↓
15. WIA Nano Standard 완료! 🎉
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1-3 결과물과 연동 가능하도록 설계
✅ 출력 어댑터 추상화 (새로운 출력 방식 쉽게 추가)
✅ 표준 형식 지원 (PDB, XYZ, LAMMPS)
✅ 비동기 처리 (async/await)
✅ Mock 구현으로 테스트 가능하게
✅ 에러 처리 포함
✅ 단위 변환 정확성 (nm to Angstrom 등)
```

### DON'T (하지 말 것)

```
❌ 특정 시각화 도구에만 종속
❌ 실제 외부 서비스 필수 의존 (Mock 필요)
❌ 동기 블로킹 처리
❌ Phase 1-3 형식과 불일치
❌ 단위 변환 오류
```

---

## 🔗 WIA Nano 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     나노기술 데이터                          │
│  (Assembler, Nanomachine, Memory, Medicine, Robotics...)   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 1: Data Format Standard                   │
│                    표준 JSON 스키마                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 2: API Interface Standard                 │
│                     Rust API 구현                            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 3: Communication Protocol                 │
│                 WIA Nano Protocol (WNP)                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 4: Ecosystem Integration                  │
│                     OutputManager                            │
├──────────┬──────────┬──────────┬──────────┬─────────────────┤
│PdbExport │XyzExport │LammpsExp │GromacsExp│  CifExporter    │
└────┬─────┴────┬─────┴────┬─────┴────┬─────┴────┬────────────┘
     │          │          │          │          │
     ▼          ▼          ▼          ▼          ▼
┌───────┐  ┌───────┐  ┌───────┐  ┌───────┐  ┌───────┐
│  VMD  │  │PyMOL  │  │LAMMPS │  │GROMACS│  │ CIF   │
│ Viewer│  │ Viewer│  │  MD   │  │  MD   │  │Viewer │
└───────┘  └───────┘  └───────┘  └───────┘  └───────┘
```

---

## 🚀 작업 시작

이제 Phase 4 작업을 시작하세요.

첫 번째 단계: **웹서치로 분자 시각화 및 내보내기 기술 조사**

```
검색 키워드: "PDB file format specification molecular visualization"
```

화이팅! ⚛️

WIA Nano Standard의 마지막 Phase입니다.
완료되면 데이터 정의부터 외부 시스템 연동까지 전체 파이프라인이 완성됩니다!

---

<div align="center">

**Phase 4 of 4**

Ecosystem Integration

🎯 최종 목표: 데이터 → 시각화/시뮬레이션/분석

🔬 弘益人間 - Benefit All Humanity 🔬

</div>
