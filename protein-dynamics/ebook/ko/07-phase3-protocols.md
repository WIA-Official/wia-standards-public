# 제6장: 3단계 - 프로토콜

## 시뮬레이션 및 분석 워크플로우

**弘益人間 (홍익인간)**

---

## 6.1 프로토콜 개요

### 표준화된 워크플로우

WIA-PROTEIN-DYNAMICS 표준은 다음을 위한 프로토콜을 정의합니다:
1. **앙상블 생성**: 형태 앙상블 생성
2. **동역학 분석**: 동역학 메트릭 추출
3. **알로스테릭 매핑**: 통신 경로 식별
4. **결합 시뮬레이션**: 약물 결합 동역학
5. **검증**: 실험과 비교

### 프로토콜 요구 사항

각 프로토콜은 다음을 지정합니다:
- 입력 요구 사항 및 형식
- 처리 단계 및 매개변수
- 품질 관리 체크포인트
- 출력 사양
- 검증 기준

---

## 6.2 앙상블 생성 프로토콜

### 프로토콜 1: 표준 MD 앙상블

**목적:** 기존 MD 시뮬레이션에서 형태 앙상블 생성.

```yaml
ensemble_generation_md:
  version: "1.0"
  protocol_id: "WIA-PD-ENS-001"

  step_1_structure_preparation:
    actions:
      - add_hydrogens:
          ph: 7.0
          histidine_protonation: "auto"
      - check_structure:
          missing_atoms: "add"
          clashes: "minimize"

  step_2_system_setup:
    solvation:
      water_model: "tip3p"
      box_shape: "dodecahedron"
      min_distance_to_wall_nm: 1.2
    ions:
      neutralize: true
      concentration_M: 0.15

  step_3_energy_minimization:
    algorithm: "steepest_descent"
    max_steps: 50000
    tolerance_kJ_mol_nm: 1000

  step_4_equilibration:
    nvt:
      duration_ps: 100
      temperature_K: 300
    npt:
      duration_ps: 1000
      pressure_bar: 1.0

  step_5_production:
    duration_ns: 1000
    temperature_K: 300
    timestep_fs: 2
    save_interval_ps: 10

  step_6_analysis:
    clustering:
      method: "gromos"
      rmsd_cutoff_nm: 0.15
    convergence:
      block_averaging: true

  quality_control:
    - check: "temperature_stability"
      tolerance_K: 2
    - check: "energy_drift"
      max_drift_percent: 0.1
```

### 프로토콜 2: 향상된 샘플링 앙상블

**목적:** 더 나은 형태 샘플링을 위해 메타다이나믹스 사용.

```yaml
ensemble_generation_metadynamics:
  version: "1.0"
  protocol_id: "WIA-PD-ENS-002"

  collective_variables:
    - cv_1:
        type: "rmsd"
        atoms: "backbone"
    - cv_2:
        type: "distance"
        group_1: "residues 50-60"
        group_2: "residues 150-160"

  metadynamics_parameters:
    bias_factor: 15
    gaussian_height_kJ: 1.0
    deposit_interval_ps: 1.0

  simulation:
    duration_ns: 500
    walkers: 4

  reweighting:
    method: "tiwary-parrinello"
```

### 프로토콜 3: AlphaFlow 앙상블

```yaml
ensemble_generation_alphaflow:
  version: "1.0"
  protocol_id: "WIA-PD-ENS-003"

  input:
    sequence: "FASTA 형식"

  sampling:
    num_samples: 100
    temperature: 1.0

  post_processing:
    energy_minimization: true
    clash_removal: true

  clustering:
    method: "k-medoids"
    metric: "tm-score"
```

---

## 6.3 동역학 분석 프로토콜

### 유연성 분석

```yaml
flexibility_analysis:
  version: "1.0"
  protocol_id: "WIA-PD-FLEX-001"

  rmsf_calculation:
    atoms: "name CA"
    fit_atoms: "backbone"

  bfactor_derivation:
    convert_rmsf: true
    formula: "B = (8 * pi^2 / 3) * RMSF^2"

  order_parameters:
    calculate_S2: true
    bond_vectors: "NH"

  flexible_region_identification:
    threshold_percentile: 90
    min_length: 3
```

### 주성분 분석

```yaml
pca_analysis:
  version: "1.0"
  protocol_id: "WIA-PD-PCA-001"

  pca_parameters:
    n_components: 20
    method: "svd"

  analysis:
    scree_plot: true
    variance_threshold: 0.90

  mode_visualization:
    modes: [1, 2, 3]
    amplitude_angstrom: 3.0
```

---

## 6.4 알로스테릭 매핑 프로토콜

### 교차 상관 분석

```yaml
allosteric_mapping:
  version: "1.0"
  protocol_id: "WIA-PD-ALLO-001"

  step_1_correlation_matrix:
    method: "pearson"
    normalize: true

  step_2_community_detection:
    method: "girvan-newman"
    min_community_size: 5

  step_3_pathway_identification:
    source_sites:
      - name: "알로스테릭_부위"
        residues: [55, 57, 58, 60]
    target_sites:
      - name: "활성_부위"
        residues: [145, 147, 166, 168]

    pathway_algorithm: "dijkstra"
    n_pathways: 5
```

---

## 6.5 약물 결합 프로토콜

### 결합 경로 시뮬레이션

```yaml
binding_simulation:
  version: "1.0"
  protocol_id: "WIA-PD-BIND-001"

  system_preparation:
    protein:
      structure: "평형화된 PDB"
    ligand:
      parameterization: "gaff2"
      partial_charges: "am1-bcc"

  steered_md:
    enabled: true
    pull_rate_nm_ps: 0.001
    duration_ns: 20

  analysis:
    binding_pathway:
      identify_intermediates: true
    interaction_analysis:
      hydrogen_bonds: true
      hydrophobic_contacts: true
```

### 결합 자유 에너지 계산

```yaml
binding_free_energy:
  version: "1.0"
  protocol_id: "WIA-PD-BIND-002"

  method: "fep"

  fep_protocol:
    n_lambda: 20
    equilibration_per_lambda_ns: 1
    production_per_lambda_ns: 5

  analysis:
    convergence_check: true
    error_estimation: "bootstrap"
```

---

## 6.6 검증 프로토콜

### 실험적 검증

```yaml
validation_protocol:
  version: "1.0"
  protocol_id: "WIA-PD-VAL-001"

  nmr_validation:
    order_parameters:
      compare_S2: true
      correlation_threshold: 0.8

  cryoem_validation:
    heterogeneity:
      compare_class_populations: true

  cross_validation:
    holdout_fraction: 0.2
    metrics:
      - "population_divergence"
      - "structural_similarity"
```

---

## 6.7 프로토콜 실행

### 명령줄 인터페이스

```bash
# 앙상블 생성 실행
wia-pd protocol run WIA-PD-ENS-001 \
  --input structure.pdb \
  --output ensemble_results/

# 동역학 분석 실행
wia-pd protocol run WIA-PD-FLEX-001 \
  --trajectory md.xtc \
  --topology system.gro \
  --output flexibility_analysis/
```

### Python API

```python
from wia_pd import protocols

# 프로토콜 로드 및 실행
protocol = protocols.load("WIA-PD-ENS-001")

result = protocol.run(
    input_structure="structure.pdb",
    parameters={
        "production_duration_ns": 1000,
        "temperature_K": 310
    },
    output_dir="ensemble_results"
)

# 결과 접근
ensemble = result.get_ensemble()
for state in ensemble.states:
    print(f"  {state.name}: {state.population:.2%}")
```

---

## 6.8 품질 관리 체크포인트

### 시뮬레이션 품질 메트릭

```python
def check_simulation_quality(trajectory, topology):
    """MD 시뮬레이션에 대한 포괄적 품질 관리."""
    checks = {
        'passed': [],
        'warnings': [],
        'failed': []
    }

    # 1. 온도 안정성
    temp_mean, temp_std = analyze_temperature(trajectory)
    if temp_std > 5:
        checks['warnings'].append(f"온도 요동: {temp_std:.1f} K")
    else:
        checks['passed'].append("온도 안정")

    # 2. 에너지 보존
    energy_drift = calculate_energy_drift(trajectory)
    if abs(energy_drift) > 0.01:
        checks['failed'].append(f"에너지 드리프트: {energy_drift:.3f} kJ/mol/ns")

    return checks
```

---

## 요약

WIA-PROTEIN-DYNAMICS 프로토콜은 다음을 제공합니다:
- 모든 동역학 작업을 위한 표준화된 워크플로우
- 상세한 매개변수 사양
- 품질 관리 체크포인트
- 여러 실행 인터페이스 (CLI, Python, API)
- 실험 데이터에 대한 검증

---

**다음 장:** [4단계: 통합](./08-phase4-integration.md)

弘益人間 - 널리 인간을 이롭게 하라
