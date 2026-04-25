# 제3장: 형태 앙상블

## 단백질 상태 표현하기

**弘益人間 (홍익인간)**

---

## 3.1 앙상블 패러다임

### 단일 구조를 넘어서

단일 구조는 형태의 연속체에서 한 스냅샷만을 나타냅니다. 앙상블 관점은 단백질이 상태의 집합으로 존재하며, 각각 열역학에 의해 결정된 정의된 집단을 갖는다는 것을 인식합니다:

```
앙상블 = {(구조₁, 집단₁), (구조₂, 집단₂), ...}

집단_i = exp(-ΔG_i/RT) / Σ exp(-ΔG_j/RT)
```

### 왜 앙상블이 중요한가

**기능적 함의**

| 단일 구조 관점 | 앙상블 관점 |
|---------------|------------|
| 효소는 하나의 활성 부위 기하학을 가짐 | 여러 촉매적으로 유능한 상태를 샘플링 |
| 약물은 한 형태에 결합 | 약물이 특정 형태를 선택하거나 유도 |
| 돌연변이가 하나의 구조를 변경 | 돌연변이가 집단 분포를 이동 |
| 알로스테리가 신비로움 | 알로스테리가 형태 평형을 이동 |

---

## 3.2 MD로부터 앙상블 생성

### 표준 MD 접근법

분자 동역학은 자연스럽게 형태 공간을 샘플링합니다:

```python
import mdtraj as md
import numpy as np

def generate_md_ensemble(topology, trajectory_file,
                         n_clusters=10, stride=10):
    """
    MD 궤적에서 형태 앙상블 생성.

    매개변수:
    -----------
    topology : str
        토폴로지 파일 경로 (PDB, GRO)
    trajectory_file : str
        궤적 파일 경로 (XTC, DCD)
    n_clusters : int
        대표 구조 수
    stride : int
        N번째 프레임마다 읽기

    반환값:
    --------
    dict : 구조와 집단이 포함된 앙상블
    """
    # 궤적 로드
    traj = md.load(trajectory_file, top=topology, stride=stride)

    # 참조 정렬 (첫 번째 프레임)
    traj.superpose(traj[0])

    # 클러스터링
    # ... 구현 ...

    return ensemble
```

### 수렴 평가

MD 시뮬레이션은 앙상블을 적절히 샘플링할 만큼 충분히 길어야 합니다. 블록 평균화를 통해 수렴을 평가합니다.

---

## 3.3 향상된 샘플링 방법

### 샘플링 문제

표준 MD는 동역학적 장벽에 의해 제한됩니다. 상태 간 전이는 마이크로초에서 밀리초가 걸릴 수 있습니다—일반적인 시뮬레이션보다 깁니다.

### 레플리카 교환 MD (REMD)

다른 온도에서 여러 레플리카를 실행하고 주기적으로 교환을 시도합니다:

```
T₁=300K ↔ T₂=310K ↔ T₃=320K ↔ ... ↔ T₃₂=450K

교환 확률: P = min(1, exp(Δβ × ΔE))
```

고온 레플리카가 장벽을 넘고; 교환이 샘플링을 생리학적 온도로 전파합니다.

### 메타다이나믹스

방문한 형태에 가우시안 바이어스 잠재력을 적용하여 새로운 영역 탐색을 강제합니다:

```
V_bias(s,t) = Σ w × exp(-(s - s(t'))² / 2σ²)

s = 집합 변수 (CV)
w = 가우시안 높이
σ = 가우시안 너비
```

### AlphaFlow

AlphaFold를 플로우 매칭과 결합하여 형태 앙상블을 생성:

```
서열 → AlphaFlow → 앙상블 {상태1: 0.65, 상태2: 0.25, 상태3: 0.10}
```

---

## 3.4 머신 러닝 앙상블 생성

### Boltzmann 생성기

Boltzmann 분포에서 샘플링하도록 훈련된 신경망:

```
z ~ N(0,1) → 신경망 → x with P(x) ∝ exp(-E(x)/kT)
```

장점:
- 희귀 상태를 효율적으로 샘플링
- 희귀 사건에 대해 MD보다 100-1000배 빠름
- 제한된 데이터에서 학습

### 분포적 Graphormer

단일 점 추정이 아닌 구조에 대한 전체 분포를 예측:

```
입력: 서열
출력: P(구조 | 서열) - 전체 분포
```

다양한 구조를 샘플링하고 집단을 추정할 수 있습니다.

---

## 3.5 자유 에너지 지형

### 2D 투영

고차원 형태 공간은 종종 시각화를 위해 2D로 투영됩니다:

```
자유 에너지 (kcal/mol)
  8 │                     ╔══╗
  7 │         ╔══╗        ║  ║
  6 │  ╔══╗   ║  ║   ╔══╗ ║  ║
  5 │  ║  ║   ║  ║   ║  ║ ╚══╝
  4 │  ║  ╚═══╝  ║   ║  ║
  3 │  ║        ║   ║  ║
  2 │  ║   ╔════╝   ║  ║
  1 │  ╚═══╝        ╚══╝
  0 │     ★ 전역    ★ 준안정
    └──────────────────────────→
                CV1 (예: RMSD)
```

### 지형 계산

시뮬레이션 또는 앙상블 데이터에서:

```python
def compute_free_energy_landscape(cv1_values, cv2_values,
                                  n_bins=50, temperature=300):
    """
    CV 값에서 2D 자유 에너지 지형 계산.
    """
    kB = 0.001987  # kcal/mol/K
    kT = kB * temperature

    # 2D 히스토그램
    H, xedges, yedges = np.histogram2d(
        cv1_values, cv2_values, bins=n_bins, density=True
    )

    # 자유 에너지로 변환
    # G = -kT * ln(P)
    G = -kT * np.log(H)
    G = G - np.nanmin(G)

    return G, xedges, yedges
```

---

## 3.6 집단 추정

### 시뮬레이션에서

각 상태에서 보낸 시뮬레이션 시간으로부터 집단:

```python
def estimate_populations(trajectory, state_assignments):
    """궤적에서 집단 추정."""
    unique_states = np.unique(state_assignments)
    populations = {}

    for state in unique_states:
        count = np.sum(state_assignments == state)
        populations[state] = count / len(state_assignments)

    return populations
```

### 바이어스 시뮬레이션 재가중

향상된 샘플링은 제거해야 할 바이어스를 도입합니다:

**WHAM (가중 히스토그램 분석 방법)**
여러 바이어스 시뮬레이션의 데이터를 결합합니다.

**Tiwary-Parrinello 재가중**
메타다이나믹스용:

```
w(t) = exp(V_bias(s(t), t) / kT)
P_unbiased(s) = Σ δ(s - s(t)) × w(t) / Σ w(t)
```

---

## 3.7 WIA 앙상블 형식

### 스키마 정의

```json
{
  "$schema": "https://wia.live/schemas/conformational-ensemble/v1.0.0",
  "conformational_ensemble": {
    "protein_id": "P00533",
    "generation_method": "메타다이나믹스",
    "num_states": 5,

    "states": [
      {
        "state_id": "ground",
        "name": "활성 형태",
        "population": 0.65,
        "relative_energy": {"value": 0.0, "unit": "kcal/mol"},
        "coordinates_pdb": "state_ground.pdb"
      },
      {
        "state_id": "excited_1",
        "name": "DFG-out 비활성",
        "population": 0.25,
        "relative_energy": {"value": 0.8, "unit": "kcal/mol"},
        "coordinates_pdb": "state_excited1.pdb"
      }
    ],

    "free_energy_landscape": {
      "collective_variables": ["RMSD_to_active", "Activation_loop_distance"],
      "minima": [...],
      "barriers": [...]
    }
  }
}
```

---

## 요약

- 단백질은 단일 구조가 아닌 형태 앙상블로 존재
- MD 시뮬레이션은 앙상블을 샘플링하지만 희귀 상태를 놓칠 수 있음
- 향상된 샘플링 (REMD, 메타다이나믹스)이 장벽을 극복
- ML 방법 (AlphaFlow, Boltzmann 생성기)이 빠른 앙상블 생성을 가능하게 함
- 자유 에너지 지형이 형태 공간을 시각화
- 집단 추정은 적절한 재가중을 필요로 함
- WIA 형식이 앙상블 표현을 표준화

---

**다음 장:** [1단계: 데이터 형식](./05-phase1-data-format.md)

弘益人間 - 널리 인간을 이롭게 하라
