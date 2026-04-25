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

---

## Annex A — Conformance Tier Matrix

WIA conformance for cryo-research is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/cryo-research/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cryo-research/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cryo-research/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-2

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
