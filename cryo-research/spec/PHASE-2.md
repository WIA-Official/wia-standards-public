# WIA-CRYO-010 PHASE 2: ALGORITHMS

**Standard**: WIA-CRYO-010
**Phase**: 2 - Research Algorithms and Calculations
**Version**: 1.0.0
**Date**: January 2025
**Status**: Active

## Overview

Phase 2 defines algorithms, statistical methods, and calculations for analyzing cryopreservation research data.

## 2.1 Viability Calculations

### Basic Viability Percentage
```
Viability (%) = (Viable Cells / Total Cells) × 100
```

### Recovery Rate
```
Recovery (%) = (Cells Post-Thaw / Cells Pre-Freeze) × 100
```

### Effective Viability
```
Effective Viability = Viability × Recovery / 100
```

### Confidence Interval (95%)
```
CI = p ± 1.96 × √(p(1-p)/n)
where p = viability (as decimal), n = sample size
```

## 2.2 Success Rate Analysis

### Binary Success Rate
```
Success Rate = (Successful Outcomes / Total Attempts) × 100
```

### Wilson Score Confidence Interval
```
Center = (p + z²/2n) / (1 + z²/n)
Margin = z × √(p(1-p)/n + z²/4n²) / (1 + z²/n)
CI = [Center - Margin, Center + Margin]
where z = 1.96 for 95% CI
```

## 2.3 Statistical Power Analysis

### Sample Size for Proportions
```
n = [(Zα + Zβ)² × (p₁(1-p₁) + p₂(1-p₂))] / (p₁ - p₂)²
where:
Zα = 1.96 (for α=0.05, two-tailed)
Zβ = 0.84 (for power=0.80)
p₁ = expected proportion group 1
p₂ = expected proportion group 2
```

### Sample Size for Continuous Outcomes
```
n = 2 × (Zα + Zβ)² × σ² / δ²
where:
σ = standard deviation
δ = minimum detectable difference
```

## 2.4 Quality Metrics

### VQI (Vitrification Quality Index)
```
VQI = (w₁×V + w₂×M + w₃×F - w₄×I - w₅×A) / 10
where:
V = viability (0-100)
M = metabolic activity (0-100)
F = functional score (0-100)
I = ice crystal score (0-100, lower better)
A = apoptosis rate (0-100, lower better)
w₁-w₅ = weights (default: 0.4, 0.3, 0.2, 0.05, 0.05)
Result scaled to 0-10
```

### Data Quality Score
```
DQ = (Completeness + Consistency + Accuracy + Timeliness) / 4
where each component scored 0-1
```

## 2.5 Temporal Analysis

### Decline Rate (Linear)
```
Decline Rate = (V₀ - Vₜ) / t
where:
V₀ = initial viability
Vₜ = viability at time t
t = time in specified units
```

### Exponential Decay Model
```
V(t) = V₀ × e^(-λt)
where λ = decay constant
```

## 2.6 Comparative Statistics

### T-test (Two-sample)
```
t = (x̄₁ - x̄₂) / √(s₁²/n₁ + s₂²/n₂)
df = smaller of (n₁-1, n₂-1) for conservative estimate
```

### Effect Size (Cohen's d)
```
d = (x̄₁ - x̄₂) / s_pooled
s_pooled = √[(s₁²(n₁-1) + s₂²(n₂-1)) / (n₁ + n₂ - 2)]
```

### ANOVA F-statistic
```
F = MS_between / MS_within
MS_between = SS_between / (k-1)
MS_within = SS_within / (N-k)
where k = number of groups, N = total sample size
```

## 2.7 Meta-Analysis

### Weighted Mean Effect
```
θ̂ = Σ(wᵢθᵢ) / Σwᵢ
where wᵢ = 1/SEᵢ² (inverse variance weighting)
```

### Heterogeneity (I²)
```
I² = 100% × (Q - df) / Q
where Q = Cochran's Q statistic
I² > 75% indicates high heterogeneity
```

## 2.8 Regression Models

### Linear Regression
```
Y = β₀ + β₁X₁ + β₂X₂ + ... + ε
β̂ = (X'X)⁻¹X'Y (OLS estimator)
```

### Logistic Regression (for binary outcomes)
```
log(p/(1-p)) = β₀ + β₁X₁ + β₂X₂ + ...
where p = probability of success
```

## 2.9 Survival Analysis

### Kaplan-Meier Estimator
```
S(t) = Π[1 - dᵢ/nᵢ] for all i where tᵢ ≤ t
where:
dᵢ = number of events at time tᵢ
nᵢ = number at risk at time tᵢ
```

## 2.10 Machine Learning Algorithms

### Random Forest Feature Importance
Recommended for identifying predictors of viability:
- n_estimators: 100-500
- max_depth: 10-20
- min_samples_split: 5-10

### Neural Network Architecture
For outcome prediction:
- Input layer: normalized features
- Hidden layers: 2-3 layers, 64-128 neurons
- Output: sigmoid for binary, softmax for multi-class
- Loss: binary crossentropy or categorical crossentropy
- Optimizer: Adam (lr=0.001)

## 2.11 Implementation Examples

### Python
```python
import numpy as np
from scipy import stats

def calculate_viability(viable_cells, total_cells):
    """Calculate viability percentage with 95% CI"""
    viability = viable_cells / total_cells * 100
    se = np.sqrt(viability * (100-viability) / total_cells)
    ci_lower = viability - 1.96 * se
    ci_upper = viability + 1.96 * se
    return {
        'viability': viability,
        'ci_95': [ci_lower, ci_upper],
        'se': se
    }

def calculate_vqi(viability, metabolic, functional, ice, apoptosis):
    """Calculate Vitrification Quality Index"""
    vqi = (0.4*viability + 0.3*metabolic + 0.2*functional 
           - 0.05*ice - 0.05*apoptosis) / 10
    return max(0, min(10, vqi))  # Clamp to 0-10
```

### R
```r
calculate_viability <- function(viable_cells, total_cells) {
  viability <- viable_cells / total_cells * 100
  se <- sqrt(viability * (100-viability) / total_cells)
  ci <- c(viability - 1.96*se, viability + 1.96*se)
  list(viability=viability, ci_95=ci, se=se)
}

calculate_power <- function(p1, p2, alpha=0.05, power=0.80) {
  za <- qnorm(1 - alpha/2)
  zb <- qnorm(power)
  n <- ((za + zb)^2 * (p1*(1-p1) + p2*(1-p2))) / (p1-p2)^2
  ceiling(n)
}
```

## References

- Statistical Methods for Biologists (Sokal & Rohlf)
- Clinical Trials: A Methodologic Perspective (Piantadosi)
- Machine Learning in Medicine (Suzuki)

---

**Previous**: [PHASE-1: Data Formats](PHASE-1.md)  
**Next**: [PHASE-3: Protocols](PHASE-3.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
