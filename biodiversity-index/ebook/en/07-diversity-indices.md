# Chapter 7: Diversity Indices

## Mathematical Foundations of Biodiversity Measurement

### Understanding the Mathematics Behind Diversity Quantification

---

## Overview

Biodiversity indices provide quantitative measures that summarize complex ecological communities into interpretable values. This chapter covers the mathematical foundations, calculation methods, statistical properties, and practical applications of diversity indices supported by the WIA Biodiversity Index Standard.

---

## Core Concepts

### What is Biodiversity?

Biodiversity encompasses three hierarchical levels:

1. **Genetic diversity**: Variation within species
2. **Species diversity**: Variety of species in an area
3. **Ecosystem diversity**: Variety of ecosystems in a region

This chapter focuses on **species diversity**, which itself has three components:

| Component | Definition | Example |
|-----------|------------|---------|
| Alpha (α) diversity | Species diversity within a single community | Species richness in one forest plot |
| Beta (β) diversity | Variation in species composition between communities | Turnover between forest plots |
| Gamma (γ) diversity | Total species diversity across all communities | Total species in entire landscape |

**Relationship:** γ = α × β (multiplicative) or γ = α + β (additive)

### Abundance Data Types

Diversity indices use different types of abundance data:

| Data Type | Description | Example |
|-----------|-------------|---------|
| Presence/absence | Binary: species detected or not | Checklist data |
| Counts | Integer number of individuals | Point count birds |
| Density | Individuals per unit area | Trees per hectare |
| Cover | Percentage of area covered | Plant cover in quadrat |
| Biomass | Mass per unit area | Fish biomass per m² |

---

## Species Richness

### Observed Richness (S)

The simplest diversity measure: count of species observed.

**Formula:**
$$S = \sum_{i=1}^{S} I(n_i > 0)$$

Where $n_i$ is the abundance of species $i$ and $I()$ is the indicator function.

**Advantages:**
- Intuitive and easily understood
- No assumptions about abundance distribution
- Comparable across studies (with standardization)

**Limitations:**
- Highly sensitive to sampling effort
- Rare species easily missed
- Does not account for relative abundance

### Rarefaction

Standardizes richness to a common sample size to enable fair comparisons.

**Individual-based Rarefaction:**
$$E[S_n] = S - \frac{\sum_{i=1}^{S} \binom{N-n_i}{n}}{\binom{N}{n}}$$

Where:
- $S$ = observed species richness
- $N$ = total individuals in sample
- $n_i$ = abundance of species $i$
- $n$ = standardized sample size

**Python Implementation:**

```python
import numpy as np
from scipy.special import comb

def rarefaction(abundances, n):
    """
    Calculate rarefied species richness.

    Args:
        abundances: Array of species abundances
        n: Sample size for rarefaction

    Returns:
        Expected species richness at sample size n
    """
    N = sum(abundances)
    S = len(abundances)

    if n > N:
        raise ValueError("n cannot exceed total abundance N")

    expected_richness = S - sum(
        comb(N - ni, n, exact=True) / comb(N, n, exact=True)
        for ni in abundances
        if ni > 0
    )

    return expected_richness

# Example usage
abundances = [50, 30, 20, 15, 10, 8, 5, 3, 2, 1, 1, 1]
for sample_size in [50, 100, 145]:
    expected_S = rarefaction(abundances, sample_size)
    print(f"E[S] at n={sample_size}: {expected_S:.2f}")
```

**Rarefaction Curve:**

```
Species
Richness
    |
 12 |                    ●---●---●
    |               ●----
 10 |          ●----
    |      ●---
  8 |   ●--
    | ●-
  6 |●
    |
  4 +---+---+---+---+---+---+---+---
    0  20  40  60  80 100 120 140
              Individuals
```

### Richness Estimators

Estimate true richness accounting for undetected species.

**Chao1 Estimator (abundance-based):**
$$\hat{S}_{Chao1} = S_{obs} + \frac{f_1^2}{2f_2}$$

Where:
- $S_{obs}$ = observed species richness
- $f_1$ = number of singletons (species with 1 individual)
- $f_2$ = number of doubletons (species with 2 individuals)

**Chao2 Estimator (incidence-based):**
$$\hat{S}_{Chao2} = S_{obs} + \frac{Q_1^2}{2Q_2}$$

Where:
- $Q_1$ = number of uniques (species in 1 sample only)
- $Q_2$ = number of duplicates (species in 2 samples only)

**ACE (Abundance-based Coverage Estimator):**
$$\hat{S}_{ACE} = S_{abund} + \frac{S_{rare}}{\hat{C}_{rare}} + \frac{f_1}{\hat{C}_{rare}} \gamma^2_{rare}$$

Where abundant species have ≥10 individuals, rare species have <10.

**Jackknife Estimators:**

| Order | Formula | Notes |
|-------|---------|-------|
| First-order | $S_{obs} + f_1 \frac{m-1}{m}$ | Bias-corrected using singletons |
| Second-order | $S_{obs} + \frac{f_1(2m-3)}{m} - \frac{f_2(m-2)^2}{m(m-1)}$ | Also uses doubletons |

**Python Implementation:**

```python
def chao1_estimator(abundances):
    """
    Calculate Chao1 species richness estimator.

    Args:
        abundances: Array of species abundances

    Returns:
        Dictionary with estimate, SE, and 95% CI
    """
    S_obs = sum(1 for a in abundances if a > 0)
    f1 = sum(1 for a in abundances if a == 1)
    f2 = sum(1 for a in abundances if a == 2)

    # Handle edge case
    if f2 == 0:
        f2 = 1

    # Point estimate
    S_chao1 = S_obs + (f1 * f1) / (2 * f2)

    # Standard error (simplified)
    if f1 > 0 and f2 > 0:
        variance = f2 * ((f1/f2)**4 / 4 + (f1/f2)**3 + (f1/f2)**2 / 2)
        se = np.sqrt(variance)
    else:
        se = 0

    # 95% confidence interval (log-transformation)
    k = np.exp(1.96 * np.sqrt(np.log(1 + (se / (S_chao1 - S_obs))**2)))

    return {
        'estimate': S_chao1,
        'se': se,
        'ci_lower': S_obs + (S_chao1 - S_obs) / k,
        'ci_upper': S_obs + (S_chao1 - S_obs) * k
    }
```

---

## Diversity Indices

### Shannon Diversity Index (H')

The most widely used diversity index, based on information theory.

**Formula:**
$$H' = -\sum_{i=1}^{S} p_i \ln(p_i)$$

Where $p_i$ = proportion of individuals belonging to species $i$

**Properties:**
- Range: 0 to ln(S)
- H' = 0 when community has only one species
- H' = ln(S) when all species equally abundant
- Increases with richness and evenness

**Effective Number of Species:**
$$^1D = e^{H'} = \exp\left(-\sum_{i=1}^{S} p_i \ln(p_i)\right)$$

This converts H' to "equivalent number of equally common species."

**Variance Estimation (Hutcheson, 1970):**
$$Var(H') = \frac{\sum p_i (\ln p_i)^2 - (\sum p_i \ln p_i)^2}{N} + \frac{S-1}{2N^2}$$

**Python Implementation:**

```python
def shannon_diversity(abundances):
    """
    Calculate Shannon diversity index with confidence interval.

    Args:
        abundances: Array of species abundances

    Returns:
        Dictionary with H', variance, SE, and 95% CI
    """
    N = sum(abundances)
    proportions = [a / N for a in abundances if a > 0]

    # Shannon index
    H = -sum(p * np.log(p) for p in proportions)

    # Variance (Hutcheson 1970)
    sum_p_lnp_sq = sum(p * np.log(p)**2 for p in proportions)
    sum_p_lnp = sum(p * np.log(p) for p in proportions)
    S = len(proportions)

    var_H = (sum_p_lnp_sq - sum_p_lnp**2) / N + (S - 1) / (2 * N**2)
    se = np.sqrt(var_H)

    # 95% CI
    ci_lower = H - 1.96 * se
    ci_upper = H + 1.96 * se

    # Effective number of species
    effective_S = np.exp(H)

    return {
        'H': H,
        'variance': var_H,
        'se': se,
        'ci_lower': max(0, ci_lower),
        'ci_upper': ci_upper,
        'effective_species': effective_S
    }
```

### Simpson's Index

Measures the probability that two randomly selected individuals belong to the same species.

**Simpson's Dominance (D):**
$$D = \sum_{i=1}^{S} p_i^2$$

**Simpson's Diversity (1-D):**
$$1 - D = 1 - \sum_{i=1}^{S} p_i^2$$

**Inverse Simpson (1/D):**
$$\frac{1}{D} = \frac{1}{\sum_{i=1}^{S} p_i^2}$$

**Gini-Simpson Index:**
Probability that two random individuals are different species.

**Finite Sample Correction:**
$$D = \frac{\sum_{i=1}^{S} n_i(n_i-1)}{N(N-1)}$$

**Comparison:**

| Index | Range | Interpretation |
|-------|-------|----------------|
| D (dominance) | 0-1 | Lower = more diverse |
| 1-D (diversity) | 0-1 | Higher = more diverse |
| 1/D (inverse) | 1-S | Effective number of dominant species |

**Python Implementation:**

```python
def simpson_indices(abundances):
    """
    Calculate Simpson's diversity indices.

    Args:
        abundances: Array of species abundances

    Returns:
        Dictionary with D, 1-D, 1/D, and SE
    """
    N = sum(abundances)
    abundances = [a for a in abundances if a > 0]

    # Finite sample correction
    D = sum(n * (n - 1) for n in abundances) / (N * (N - 1))

    # Variance (from Solow 1993)
    proportions = [a / N for a in abundances]
    sum_p2 = sum(p**2 for p in proportions)
    sum_p3 = sum(p**3 for p in proportions)
    sum_p4 = sum(p**4 for p in proportions)

    var_D = (4 / N) * (sum_p3 - sum_p2**2) + (2 / (N * (N-1))) * (sum_p2 - sum_p4)
    se = np.sqrt(max(0, var_D))

    return {
        'D': D,
        'diversity_1_minus_D': 1 - D,
        'inverse_D': 1 / D if D > 0 else float('inf'),
        'se': se,
        'ci_lower': max(0, D - 1.96 * se),
        'ci_upper': min(1, D + 1.96 * se)
    }
```

### Hill Numbers

Unified framework connecting different diversity indices.

**General Formula:**
$$^qD = \left(\sum_{i=1}^{S} p_i^q\right)^{1/(1-q)}$$

| Order (q) | Index | Emphasis |
|-----------|-------|----------|
| q = 0 | Species richness | All species equal |
| q → 1 | Exponential of Shannon | Proportional weighting |
| q = 2 | Inverse Simpson | Dominant species |
| q > 2 | Higher orders | Increasingly dominant species |

**Diversity Profile:**

```
Effective
Species
    |
 50 |●
    |  ●
 40 |    ●
    |      ●
 30 |        ●-----●-----●
    |
 20 +---+---+---+---+---+---+
    0   1   2   3   4   5
              Order q

High diversity: flat profile (all orders similar)
Uneven community: steep decline with increasing q
```

---

## Evenness Measures

### Pielou's Evenness (J')

**Formula:**
$$J' = \frac{H'}{\ln(S)} = \frac{H'}{H'_{max}}$$

**Range:** 0 (one dominant species) to 1 (all species equally abundant)

### Simpson's Evenness (E_D)

**Formula:**
$$E_D = \frac{1/D}{S}$$

### Smith and Wilson's Evenness (Evar)

**Formula:**
$$E_{var} = 1 - \frac{2}{\pi} \arctan\left(\frac{\sum_{i=1}^{S} (\ln(n_i) - \sum \ln(n_j)/S)^2}{S}\right)$$

**Comparison:**

| Index | Sensitive To | Best Use |
|-------|--------------|----------|
| J' (Pielou) | Rare species | General-purpose |
| E_D (Simpson) | Dominant species | When dominance important |
| Evar | Overall spread | Ecological modeling |

---

## Beta Diversity

### Turnover Measures

**Whittaker's Beta:**
$$\beta_W = \frac{\gamma}{\bar{\alpha}} - 1$$

**Jaccard Dissimilarity:**
$$\beta_J = \frac{b + c}{a + b + c}$$

Where:
- a = species in both sites
- b = species only in site 1
- c = species only in site 2

**Sørensen Dissimilarity:**
$$\beta_S = \frac{b + c}{2a + b + c}$$

**Bray-Curtis Dissimilarity (quantitative):**
$$BC_{ij} = \frac{\sum_k |n_{ik} - n_{jk}|}{\sum_k (n_{ik} + n_{jk})}$$

### Partitioning Beta Diversity

Beta diversity can be partitioned into turnover and nestedness components:

$$\beta_{total} = \beta_{turnover} + \beta_{nestedness}$$

**Python Implementation:**

```python
def beta_diversity(site1_abundances, site2_abundances):
    """
    Calculate beta diversity metrics between two sites.

    Args:
        site1_abundances: Dict of {species: abundance} for site 1
        site2_abundances: Dict of {species: abundance} for site 2

    Returns:
        Dictionary with various beta diversity metrics
    """
    species1 = set(s for s, a in site1_abundances.items() if a > 0)
    species2 = set(s for s, a in site2_abundances.items() if a > 0)

    # Presence/absence metrics
    a = len(species1 & species2)  # Shared species
    b = len(species1 - species2)  # Only in site 1
    c = len(species2 - species1)  # Only in site 2

    # Jaccard dissimilarity
    jaccard = (b + c) / (a + b + c) if (a + b + c) > 0 else 0

    # Sørensen dissimilarity
    sorensen = (b + c) / (2 * a + b + c) if (2 * a + b + c) > 0 else 0

    # Bray-Curtis (quantitative)
    all_species = species1 | species2
    numerator = sum(
        abs(site1_abundances.get(s, 0) - site2_abundances.get(s, 0))
        for s in all_species
    )
    denominator = sum(
        site1_abundances.get(s, 0) + site2_abundances.get(s, 0)
        for s in all_species
    )
    bray_curtis = numerator / denominator if denominator > 0 else 0

    return {
        'shared_species': a,
        'unique_site1': b,
        'unique_site2': c,
        'jaccard_dissimilarity': jaccard,
        'sorensen_dissimilarity': sorensen,
        'bray_curtis': bray_curtis,
        'jaccard_similarity': 1 - jaccard,
        'sorensen_similarity': 1 - sorensen
    }
```

---

## Statistical Inference

### Comparing Diversity

**Hutcheson's t-test for Shannon indices:**

$$t = \frac{H'_1 - H'_2}{\sqrt{Var(H'_1) + Var(H'_2)}}$$

**Degrees of freedom:**
$$df = \frac{(Var(H'_1) + Var(H'_2))^2}{\frac{Var(H'_1)^2}{N_1} + \frac{Var(H'_2)^2}{N_2}}$$

### Bootstrap Confidence Intervals

```python
def bootstrap_diversity(abundances, n_iterations=1000, confidence=0.95):
    """
    Calculate bootstrap confidence intervals for diversity indices.

    Args:
        abundances: Array of species abundances
        n_iterations: Number of bootstrap iterations
        confidence: Confidence level

    Returns:
        Dictionary with point estimates and CIs
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
        # Resample individuals
        sample = np.random.choice(species_ids, size=N, replace=True)

        # Count species abundances in bootstrap sample
        boot_abundances = np.bincount(sample, minlength=len(abundances))

        # Calculate indices
        boot_shannon = shannon_diversity(boot_abundances)
        boot_simpson = simpson_indices(boot_abundances)

        bootstrap_results['shannon'].append(boot_shannon['H'])
        bootstrap_results['simpson'].append(boot_simpson['diversity_1_minus_D'])
        bootstrap_results['richness'].append(sum(1 for a in boot_abundances if a > 0))

    # Calculate percentile confidence intervals
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

## Practical Applications

### Selecting Appropriate Indices

| Situation | Recommended Index | Reason |
|-----------|-------------------|--------|
| Comparing sites | Rarefied richness | Controls for sample size |
| Conservation priority | Simpson (1/D) | Emphasizes common species at risk |
| Community ecology | Shannon (H') | Balances richness and evenness |
| Habitat quality | Combined indices | Multiple perspectives |
| Long-term monitoring | All Hill numbers | Diversity profile over time |
| Impact assessment | Before/after comparison | t-test on H' |

### Interpretation Guidelines

**Shannon Index (H') Interpretation:**

| H' Value | Typical Interpretation |
|----------|----------------------|
| 0-1.0 | Very low diversity |
| 1.0-2.0 | Low diversity |
| 2.0-3.0 | Moderate diversity |
| 3.0-4.0 | High diversity |
| 4.0+ | Very high diversity |

*Note: Interpretation depends on taxon group and ecosystem type*

### Reporting Standards

**WIA requires reporting:**
1. Sample size (N) and observed richness (S)
2. Rarefied richness at standardized sample size
3. Shannon index with variance and 95% CI
4. Simpson diversity with 95% CI
5. At least one richness estimator (Chao1 preferred)
6. Sampling completeness estimate

---

## Key Takeaways

1. **Species richness** requires rarefaction for fair comparison across samples
2. **Shannon index** balances sensitivity to rare and common species
3. **Simpson index** emphasizes dominant species, useful for conservation
4. **Hill numbers** provide a unified framework connecting all indices
5. **Confidence intervals** essential for meaningful statistical comparisons

## Review Questions

1. Why is rarefaction necessary when comparing species richness between samples?
2. What is the relationship between Shannon index and effective number of species?
3. How do Simpson's D and 1-D differ in interpretation?
4. What components comprise beta diversity?
5. When would you use the Chao1 estimator vs. observed richness?

---

**Next Chapter Preview:** Chapter 8 provides practical implementation guidance for database design, SDK usage, and deployment of WIA-compliant biodiversity systems.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Preserve All Life
