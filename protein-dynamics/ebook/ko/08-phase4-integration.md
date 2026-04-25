# 제7장: 4단계 - 통합

## 데이터베이스 및 파이프라인과의 연결

**弘益人間 (홍익인간)**

---

## 7.1 통합 아키텍처

### 통합 과제

단백질 동역학 데이터는 여러 생태계와 연결되어야 합니다:

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA-PROTEIN-DYNAMICS                     │
│                        통합 허브                             │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │   PDB   │  │AlphaFold│  │ UniProt │  │ ChEMBL  │       │
│  │  구조   │  │   DB    │  │  기능   │  │ 생활성  │       │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘       │
│       └────────────┴─────┬──────┴────────────┘             │
│                          │                                  │
│                 ┌────────┴────────┐                        │
│                 │   WIA 동역학    │                        │
│                 │     표준        │                        │
│                 └────────┬────────┘                        │
│       ┌──────────────────┼──────────────────┐              │
│  ┌────┴────┐       ┌─────┴─────┐      ┌────┴────┐         │
│  │  신약   │       │  단백질   │      │  연구   │         │
│  │  개발   │       │  엔지니어링│      │  도구   │         │
│  └─────────┘       └───────────┘      └─────────┘         │
└─────────────────────────────────────────────────────────────┘
```

### 통합 원칙

1. **식별자 매핑**: ID 시스템 간 원활한 변환
2. **데이터 강화**: 동역학과 기능 및 질병 결합
3. **형식 변환**: 모든 주요 파일 형식 지원
4. **실시간 동기화**: 소스 데이터베이스와 최신 상태 유지
5. **출처 추적**: 데이터 계보 유지

---

## 7.2 PDB 통합

### 구조 가져오기

```python
from wia_pd.integration import PDBClient

pdb = PDBClient()

# 메타데이터와 함께 구조 가져오기
structure = pdb.fetch("1ATP", include_metadata=True)

print(f"제목: {structure.metadata.title}")
print(f"해상도: {structure.metadata.resolution} Å")
print(f"방법: {structure.metadata.experimental_method}")

# 단백질의 모든 구조 가져오기
structures = pdb.search(
    uniprot_id="P00533",
    resolution_max=2.5,
    experimental_method="X-RAY"
)
```

### 동역학 정보 추출

```python
# 유연성 대리로 B-인자 추출
bfactors = pdb.get_bfactors("1ATP", chain="A", atoms="CA")

# 이용 가능한 경우 다중 형태 가져오기
conformers = pdb.get_conformers("1ATP")

# NMR 앙상블 가져오기
nmr_ensemble = pdb.fetch_nmr_ensemble("1D3Z")

# WIA 형식으로 변환
wia_dynamics = pdb.to_wia_format("1ATP", include_bfactors=True)
```

---

## 7.3 AlphaFold DB 통합

### 예측 가져오기

```python
from wia_pd.integration import AlphaFoldClient

af = AlphaFoldClient()

# 예측 가져오기
prediction = af.fetch("P00533")

print(f"pLDDT 평균: {prediction.plddt_mean:.1f}")
print(f"고신뢰 영역: {len(prediction.high_confidence_regions)}")
print(f"무질서 영역: {len(prediction.low_confidence_regions)}")

# PAE 행렬 가져오기
pae = af.get_pae("P00533")

# PAE에서 도메인 경계 식별
domains = af.identify_domains("P00533", pae_threshold=5.0)
```

### 신뢰도와 동역학 매핑

```python
def plddt_to_flexibility(plddt):
    """
    AlphaFold pLDDT를 유연성 추정으로 변환.

    높은 pLDDT (>90) = 신뢰, 구조화 가능성
    중간 pLDDT (70-90) = 중간 신뢰
    낮은 pLDDT (<70) = 불확실, 유연/무질서 가능성
    """
    flexibility = []
    for score in plddt:
        if score > 90:
            flex = 0.5  # 낮은 유연성
        elif score > 70:
            flex = 1.0 + (90 - score) * 0.05  # 중간
        else:
            flex = 2.0 + (70 - score) * 0.1  # 높은 유연성
        flexibility.append(flex)
    return flexibility
```

---

## 7.4 UniProt 통합

### 기능 주석

```python
from wia_pd.integration import UniProtClient

uniprot = UniProtClient()

# 단백질 항목 가져오기
entry = uniprot.fetch("P00533")

print(f"이름: {entry.name}")
print(f"기능: {entry.function}")
print(f"세포 내 위치: {entry.subcellular_location}")

# 도메인 가져오기
for domain in entry.domains:
    print(f"  {domain.name}: {domain.start}-{domain.end}")

# 변이 가져오기
for variant in entry.variants:
    print(f"  {variant.original}{variant.position}{variant.mutation}")
```

---

## 7.5 ChEMBL/BindingDB 통합

### 약물 및 생활성 데이터

```python
from wia_pd.integration import ChEMBLClient

chembl = ChEMBLClient()

# 단백질을 표적으로 하는 화합물 가져오기
compounds = chembl.get_compounds_for_target("P00533")

print(f"{len(compounds)}개 화합물 발견")

for compound in compounds[:10]:
    print(f"  {compound.chembl_id}: IC50={compound.ic50_nM} nM")

# 동역학 데이터 가져오기 (가능한 경우)
kinetics = chembl.get_binding_kinetics("CHEMBL203", target="P00533")
if kinetics:
    print(f"  kon: {kinetics.kon}")
    print(f"  체류 시간: {kinetics.residence_time}")
```

---

## 7.6 형식 변환기

### 지원 형식

| 형식 | 읽기 | 쓰기 | 용도 |
|------|------|------|------|
| WIA JSON | ✓ | ✓ | 표준 교환 |
| PDB | ✓ | ✓ | 구조 |
| mmCIF | ✓ | ✓ | 현대 PDB 형식 |
| XTC/DCD | ✓ | - | MD 궤적 |
| MOL2/SDF | ✓ | ✓ | 리간드 |
| NMR-STAR | ✓ | ✓ | NMR 데이터 |

### 변환 API

```python
from wia_pd.converters import convert

# MD 궤적을 WIA 앙상블로
wia_data = convert(
    source="trajectory.xtc",
    source_topology="system.gro",
    target_format="wia",
    options={
        "clustering": True,
        "n_clusters": 5
    }
)

# WIA를 PDB 다중 모델로
convert(
    source=wia_data,
    target="ensemble.pdb",
    target_format="pdb"
)
```

---

## 7.7 신약 개발 파이프라인 통합

### Schrodinger 통합

```python
from wia_pd.pipelines import SchrodingerPipeline

pipeline = SchrodingerPipeline(
    host="schrodinger-server",
    license_file="/path/to/license"
)

# 동역학을 사용한 Glide 도킹 실행
results = pipeline.dock_with_dynamics(
    protein_id="P00533",
    ligands="compounds.sdf",
    dynamics_profile=wia_dynamics,
    options={
        "use_ensemble_docking": True,
        "n_poses_per_state": 3
    }
)
```

---

## 7.8 클라우드 플랫폼 통합

### AWS 통합

```python
from wia_pd.cloud import AWSProvider

aws = AWSProvider(region="us-east-1")

# S3에 동역학 데이터 저장
aws.store(
    data=wia_dynamics,
    bucket="my-dynamics-data",
    key="P00533/dynamics_v1.json"
)

# EC2/Batch에서 시뮬레이션 실행
job = aws.run_simulation(
    protocol="WIA-PD-ENS-001",
    instance_type="p3.2xlarge",
    spot=True
)
```

---

## 7.9 식별자 매핑 서비스

### 교차 참조 해결

```python
from wia_pd.integration import IdentifierMapper

mapper = IdentifierMapper()

# ID 시스템 간 매핑
mappings = mapper.map(
    source_id="P00533",
    source_type="uniprot",
    target_types=["pdb", "alphafold", "ensembl"]
)

print(mappings)
# {
#   'pdb': ['1ATP', '1M17', '2GS2', ...],
#   'alphafold': 'AF-P00533-F1',
#   'ensembl': 'ENSP00000275493'
# }
```

---

## 7.10 데이터 동기화

### 데이터 최신 유지

```python
from wia_pd.sync import DataSynchronizer

sync = DataSynchronizer()

# 업데이트 확인
updates = sync.check_updates(
    proteins=["P00533", "P04626"],
    sources=["pdb", "uniprot", "alphafold"]
)

for update in updates:
    print(f"{update.protein}: {update.source} {update.date}에 업데이트됨")

# 업데이트 적용
sync.apply_updates(updates, conflict_resolution="merge")

# 자동 동기화 예약
sync.schedule(
    frequency="daily",
    sources=["pdb", "uniprot"],
    notify_on_changes=True
)
```

---

## 요약

WIA-PROTEIN-DYNAMICS 통합 레이어는 다음을 제공합니다:
- PDB, AlphaFold, UniProt, ChEMBL과의 원활한 연결
- 양방향 형식 변환
- 신약 개발 파이프라인 통합
- 클라우드 플랫폼 지원
- 데이터베이스 간 식별자 매핑
- 자동 데이터 동기화

---

**다음 장:** [신약 개발 응용](./09-drug-discovery.md)

弘益人間 - 널리 인간을 이롭게 하라
