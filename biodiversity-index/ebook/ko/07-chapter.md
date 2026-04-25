# 제7장: 다양성 지수

## 생물다양성 측정의 수학적 기초

### 다양성 정량화 이면의 수학 이해

---

## 개요

생물다양성 지수는 복잡한 생태적 군집을 해석 가능한 값으로 요약하는 정량적 측정을 제공합니다. 이 장에서는 WIA 생물다양성 지수 표준이 지원하는 다양성 지수의 수학적 기초, 계산 방법, 통계적 특성 및 실제 응용을 다룹니다.

---

## 핵심 개념

### 생물다양성이란?

생물다양성은 세 가지 계층적 수준을 포함합니다:

1. **유전적 다양성**: 종 내의 변이
2. **종 다양성**: 지역 내 종의 다양함
3. **생태계 다양성**: 지역 내 생태계의 다양함

이 장은 **종 다양성**에 초점을 맞추며, 이는 세 가지 구성요소를 가집니다:

| 구성요소 | 정의 | 예시 |
|----------|------|------|
| 알파(α) 다양성 | 단일 군집 내 종 다양성 | 한 산림 플롯의 종 풍부도 |
| 베타(β) 다양성 | 군집 간 종 구성의 변이 | 산림 플롯 간 회전율 |
| 감마(γ) 다양성 | 모든 군집에 걸친 총 종 다양성 | 전체 경관의 총 종 |

**관계:** γ = α × β (곱셈) 또는 γ = α + β (덧셈)

### 풍도 데이터 유형

다양성 지수는 다양한 유형의 풍도 데이터를 사용합니다:

| 데이터 유형 | 설명 | 예시 |
|-------------|------|------|
| 존재/부재 | 이진: 종 감지 또는 아님 | 체크리스트 데이터 |
| 계수 | 정수 개체 수 | 조류 정점 조사 |
| 밀도 | 단위 면적당 개체 | 헥타르당 수목 |
| 피도 | 면적의 백분율 | 방형구 내 식물 피도 |
| 생물량 | 단위 면적당 질량 | m²당 어류 생물량 |

---

## 종 풍부도

### 관찰 풍부도 (S)

가장 간단한 다양성 측정: 관찰된 종의 수.

**공식:**
$$S = \sum_{i=1}^{S} I(n_i > 0)$$

여기서 $n_i$는 종 $i$의 풍도이고 $I()$는 지표 함수입니다.

**장점:**
- 직관적이고 쉽게 이해됨
- 풍도 분포에 대한 가정 없음
- 연구 간 비교 가능 (표준화 시)

**한계:**
- 샘플링 노력에 매우 민감
- 희귀 종을 쉽게 놓침
- 상대 풍도를 고려하지 않음

### 희박화 (Rarefaction)

공정한 비교를 가능하게 하기 위해 공통 샘플 크기로 풍부도를 표준화합니다.

**개체 기반 희박화:**
$$E[S_n] = S - \frac{\sum_{i=1}^{S} \binom{N-n_i}{n}}{\binom{N}{n}}$$

여기서:
- $S$ = 관찰된 종 풍부도
- $N$ = 샘플의 총 개체
- $n_i$ = 종 $i$의 풍도
- $n$ = 표준화된 샘플 크기

**Python 구현:**

```python
import numpy as np
from scipy.special import comb

def rarefaction(abundances, n):
    """
    희박화된 종 풍부도 계산.

    Args:
        abundances: 종 풍도 배열
        n: 희박화를 위한 샘플 크기

    Returns:
        샘플 크기 n에서의 예상 종 풍부도
    """
    N = sum(abundances)
    S = len(abundances)

    if n > N:
        raise ValueError("n은 총 풍도 N을 초과할 수 없음")

    expected_richness = S - sum(
        comb(N - ni, n, exact=True) / comb(N, n, exact=True)
        for ni in abundances
        if ni > 0
    )

    return expected_richness
```

### 풍부도 추정기

미탐지 종을 고려하여 실제 풍부도를 추정합니다.

**Chao1 추정기 (풍도 기반):**
$$\hat{S}_{Chao1} = S_{obs} + \frac{f_1^2}{2f_2}$$

여기서:
- $S_{obs}$ = 관찰된 종 풍부도
- $f_1$ = 싱글톤 수 (1개체인 종)
- $f_2$ = 더블톤 수 (2개체인 종)

**Chao2 추정기 (발생 기반):**
$$\hat{S}_{Chao2} = S_{obs} + \frac{Q_1^2}{2Q_2}$$

여기서:
- $Q_1$ = 유니크 수 (1개 샘플에서만 있는 종)
- $Q_2$ = 듀플리케이트 수 (2개 샘플에서만 있는 종)

---

## 다양성 지수

### Shannon 다양성 지수 (H')

정보 이론을 기반으로 한 가장 널리 사용되는 다양성 지수.

**공식:**
$$H' = -\sum_{i=1}^{S} p_i \ln(p_i)$$

여기서 $p_i$ = 종 $i$에 속하는 개체의 비율

**특성:**
- 범위: 0에서 ln(S)
- H' = 0 군집에 하나의 종만 있을 때
- H' = ln(S) 모든 종이 동등하게 풍부할 때
- 풍부도와 균등도에 따라 증가

**유효 종 수:**
$$^1D = e^{H'} = \exp\left(-\sum_{i=1}^{S} p_i \ln(p_i)\right)$$

이것은 H'를 "동등하게 일반적인 종의 동등한 수"로 변환합니다.

**분산 추정 (Hutcheson, 1970):**
$$Var(H') = \frac{\sum p_i (\ln p_i)^2 - (\sum p_i \ln p_i)^2}{N} + \frac{S-1}{2N^2}$$

**Python 구현:**

```python
def shannon_diversity(abundances):
    """
    신뢰구간과 함께 Shannon 다양성 지수 계산.

    Args:
        abundances: 종 풍도 배열

    Returns:
        H', 분산, SE 및 95% CI가 있는 딕셔너리
    """
    N = sum(abundances)
    proportions = [a / N for a in abundances if a > 0]

    # Shannon 지수
    H = -sum(p * np.log(p) for p in proportions)

    # 분산 (Hutcheson 1970)
    sum_p_lnp_sq = sum(p * np.log(p)**2 for p in proportions)
    sum_p_lnp = sum(p * np.log(p) for p in proportions)
    S = len(proportions)

    var_H = (sum_p_lnp_sq - sum_p_lnp**2) / N + (S - 1) / (2 * N**2)
    se = np.sqrt(var_H)

    return {
        'H': H,
        'variance': var_H,
        'se': se,
        'ci_lower': max(0, H - 1.96 * se),
        'ci_upper': H + 1.96 * se,
        'effective_species': np.exp(H)
    }
```

### Simpson 지수

무작위로 선택한 두 개체가 같은 종에 속할 확률을 측정합니다.

**Simpson 우점도 (D):**
$$D = \sum_{i=1}^{S} p_i^2$$

**Simpson 다양성 (1-D):**
$$1 - D = 1 - \sum_{i=1}^{S} p_i^2$$

**역 Simpson (1/D):**
$$\frac{1}{D} = \frac{1}{\sum_{i=1}^{S} p_i^2}$$

**비교:**

| 지수 | 범위 | 해석 |
|------|------|------|
| D (우점도) | 0-1 | 낮을수록 = 더 다양 |
| 1-D (다양성) | 0-1 | 높을수록 = 더 다양 |
| 1/D (역) | 1-S | 우점종의 유효 수 |

### Hill 수

다양한 다양성 지수를 연결하는 통합 프레임워크.

**일반 공식:**
$$^qD = \left(\sum_{i=1}^{S} p_i^q\right)^{1/(1-q)}$$

| 차수 (q) | 지수 | 강조 |
|----------|------|------|
| q = 0 | 종 풍부도 | 모든 종 동등 |
| q → 1 | Shannon의 지수 | 비례적 가중치 |
| q = 2 | 역 Simpson | 우점종 |
| q > 2 | 고차 | 점점 더 우점종 |

---

## 균등도 측정

### Pielou 균등도 (J')

**공식:**
$$J' = \frac{H'}{\ln(S)} = \frac{H'}{H'_{max}}$$

**범위:** 0 (하나의 우점종)에서 1 (모든 종이 동등하게 풍부)

### Simpson 균등도 (E_D)

**공식:**
$$E_D = \frac{1/D}{S}$$

---

## 베타 다양성

### 회전율 측정

**Whittaker 베타:**
$$\beta_W = \frac{\gamma}{\bar{\alpha}} - 1$$

**Jaccard 비유사성:**
$$\beta_J = \frac{b + c}{a + b + c}$$

여기서:
- a = 두 사이트 모두에 있는 종
- b = 사이트 1에만 있는 종
- c = 사이트 2에만 있는 종

**Sørensen 비유사성:**
$$\beta_S = \frac{b + c}{2a + b + c}$$

**Bray-Curtis 비유사성 (정량적):**
$$BC_{ij} = \frac{\sum_k |n_{ik} - n_{jk}|}{\sum_k (n_{ik} + n_{jk})}$$

---

## 통계적 추론

### 다양성 비교

**Shannon 지수에 대한 Hutcheson t-검정:**

$$t = \frac{H'_1 - H'_2}{\sqrt{Var(H'_1) + Var(H'_2)}}$$

**자유도:**
$$df = \frac{(Var(H'_1) + Var(H'_2))^2}{\frac{Var(H'_1)^2}{N_1} + \frac{Var(H'_2)^2}{N_2}}$$

### 부트스트랩 신뢰구간

```python
def bootstrap_diversity(abundances, n_iterations=1000, confidence=0.95):
    """
    다양성 지수에 대한 부트스트랩 신뢰구간 계산.
    """
    N = sum(abundances)
    species_ids = []
    for i, count in enumerate(abundances):
        species_ids.extend([i] * count)

    bootstrap_results = {
        'shannon': [],
        'simpson': [],
        'richness': []
    }

    for _ in range(n_iterations):
        # 개체 리샘플링
        sample = np.random.choice(species_ids, size=N, replace=True)
        boot_abundances = np.bincount(sample, minlength=len(abundances))

        # 지수 계산
        boot_shannon = shannon_diversity(boot_abundances)
        boot_simpson = simpson_indices(boot_abundances)

        bootstrap_results['shannon'].append(boot_shannon['H'])
        bootstrap_results['simpson'].append(boot_simpson['diversity_1_minus_d'])
        bootstrap_results['richness'].append(sum(1 for a in boot_abundances if a > 0))

    # 백분위 신뢰구간 계산
    alpha = 1 - confidence
    results = {}
    for metric, values in bootstrap_results.items():
        results[metric] = {
            'mean': np.mean(values),
            'se': np.std(values),
            'ci_lower': np.percentile(values, alpha/2 * 100),
            'ci_upper': np.percentile(values, (1 - alpha/2) * 100)
        }

    return results
```

---

## 실제 응용

### 적절한 지수 선택

| 상황 | 권장 지수 | 이유 |
|------|----------|------|
| 사이트 비교 | 희박화된 풍부도 | 샘플 크기 제어 |
| 보전 우선순위 | Simpson (1/D) | 위험에 처한 일반 종 강조 |
| 군집 생태학 | Shannon (H') | 풍부도와 균등도 균형 |
| 서식지 품질 | 결합 지수 | 다중 관점 |
| 장기 모니터링 | 모든 Hill 수 | 시간에 따른 다양성 프로필 |

### 해석 지침

**Shannon 지수 (H') 해석:**

| H' 값 | 일반적인 해석 |
|-------|---------------|
| 0-1.0 | 매우 낮은 다양성 |
| 1.0-2.0 | 낮은 다양성 |
| 2.0-3.0 | 중간 다양성 |
| 3.0-4.0 | 높은 다양성 |
| 4.0+ | 매우 높은 다양성 |

*참고: 해석은 분류군 그룹과 생태계 유형에 따라 다름*

---

## 핵심 내용

1. **종 풍부도**는 샘플 간 공정한 비교를 위해 희박화 필요
2. **Shannon 지수**는 희귀종과 일반종에 대한 민감도 균형
3. **Simpson 지수**는 우점종 강조, 보전에 유용
4. **Hill 수**는 모든 지수를 연결하는 통합 프레임워크 제공
5. **신뢰구간**은 의미 있는 통계적 비교에 필수

## 복습 문제

1. 샘플 간 종 풍부도를 비교할 때 희박화가 왜 필요합니까?
2. Shannon 지수와 유효 종 수 사이의 관계는 무엇입니까?
3. Simpson의 D와 1-D는 해석에서 어떻게 다릅니까?
4. 베타 다양성을 구성하는 요소는 무엇입니까?
5. Chao1 추정기와 관찰된 풍부도를 언제 사용하시겠습니까?

---

**다음 장 미리보기:** 8장에서는 데이터베이스 설계, SDK 사용 및 WIA 준수 생물다양성 시스템 배포를 위한 실제 구현 지침을 제공합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 모든 생명을 보존
