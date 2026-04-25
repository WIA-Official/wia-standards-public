# WIA-CORE-005: Hongik Impact Metric Specification v1.0

> **Standard ID:** WIA-CORE-005
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Core Standards Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Impact Dimensions](#2-impact-dimensions)
3. [Scoring Methodology](#3-scoring-methodology)
4. [Stakeholder Analysis](#4-stakeholder-analysis)
5. [Accessibility Metrics](#5-accessibility-metrics)
6. [Temporal Impact Analysis](#6-temporal-impact-analysis)
7. [Data Collection and Verification](#7-data-collection-and-verification)
8. [Integration with WIA Ecosystem](#8-integration-with-wia-ecosystem)
9. [Certification Requirements](#9-certification-requirements)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

The WIA-CORE-005 standard establishes a universal framework for quantifying the **弘益人間 (Hongik Ingan - Benefit All Humanity)** impact of projects, technologies, policies, and initiatives. By providing objective, measurable metrics across seven key dimensions of human welfare, this standard enables:

- Comparative analysis of initiatives' humanitarian impact
- Evidence-based decision making in resource allocation
- Impact tracking and continuous improvement
- Certification of high-impact projects
- Alignment with UN Sustainable Development Goals (SDGs)

### 1.2 Scope

This standard applies to:
- Technology products and platforms
- Research and development projects
- Government policies and programs
- Corporate social responsibility initiatives
- Non-profit and humanitarian organizations
- Educational programs and institutions
- Healthcare services and interventions
- Environmental and sustainability projects

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** is the foundational philosophy of the WIA ecosystem. This ancient Korean principle, meaning "widely benefit all humankind," guides the development of technologies and standards that serve the collective good rather than narrow interests.

The Hongik Impact Metric operationalizes this philosophy by:
- Prioritizing universal benefit over exclusive advantage
- Emphasizing accessibility and inclusivity
- Valuing long-term sustainability over short-term gains
- Measuring impact holistically across multiple dimensions
- Encouraging transparency and accountability

### 1.4 Terminology

- **Hongik Impact Score (HIS)**: Overall score from 0-1000 representing humanitarian impact
- **Impact Dimension**: One of seven categories of benefit measurement
- **Stakeholder**: Individual or group affected by the project/initiative
- **Beneficiary**: Stakeholder who receives positive impact
- **Direct Impact**: Immediate, measurable effects on beneficiaries
- **Indirect Impact**: Secondary and cascading effects beyond direct beneficiaries
- **Per-Capita Impact**: Impact score normalized by number of beneficiaries

---

## 2. Impact Dimensions

### 2.1 Overview

The Hongik Impact Score is calculated across seven dimensions, each weighted according to its fundamental importance to human welfare.

| Dimension | Weight | Symbol | Description |
|-----------|--------|--------|-------------|
| Social Good | 20% | SG | Community benefit and social cohesion |
| Accessibility | 20% | AC | Universal access and inclusivity |
| Sustainability | 15% | SU | Environmental and long-term viability |
| Health & Wellbeing | 15% | HW | Physical and mental health benefits |
| Economic Equity | 10% | EE | Fair distribution of economic benefits |
| Education & Growth | 10% | ED | Knowledge sharing and human development |
| Innovation & Progress | 10% | IP | Advancement of human capabilities |

### 2.2 Social Good (SG)

**Weight: 20%**

Social Good measures the project's contribution to community welfare, social cohesion, and collective benefit.

#### Assessment Criteria:

1. **Community Engagement** (25%)
   - Level of community participation in design/implementation
   - Stakeholder consultation and co-creation
   - Local ownership and governance

2. **Social Equity** (30%)
   - Reduction of social disparities
   - Inclusion of marginalized groups
   - Anti-discrimination measures

3. **Public Benefit** (25%)
   - Common good vs. private profit balance
   - Open access and knowledge sharing
   - Non-excludable benefits

4. **Cultural Impact** (20%)
   - Cultural preservation and respect
   - Diversity celebration
   - Cross-cultural understanding

#### Scoring Formula:

```
SG = (CE × 0.25) + (SE × 0.30) + (PB × 0.25) + (CI × 0.20)
```

Where each component is scored 0-1.

### 2.3 Accessibility (AC)

**Weight: 20%**

Accessibility evaluates how universally available and usable the project is across all populations and circumstances.

#### Assessment Criteria:

1. **Physical Accessibility** (20%)
   - Mobility access (wheelchair, assistive devices)
   - Sensory accessibility (visual, auditory accommodations)
   - Cognitive accessibility (clarity, simplicity)

2. **Digital Accessibility** (25%)
   - WCAG 2.1 Level AA compliance minimum (AAA preferred)
   - Screen reader compatibility
   - Keyboard navigation
   - Alternative input methods

3. **Language & Cultural** (15%)
   - Number of languages supported (weight by global speaker population)
   - Cultural localization quality
   - Right-to-left language support

4. **Economic Accessibility** (20%)
   - Affordability (% of median income)
   - Free tier or subsidized access
   - Sliding scale pricing

5. **Geographic Accessibility** (20%)
   - Rural and remote availability
   - Offline functionality
   - Low-bandwidth optimization
   - Regional infrastructure requirements

#### Scoring Formula:

```
AC = Σ(Ai × Wi)
```

Where:
- `Ai` = Score for accessibility criterion i (0-1)
- `Wi` = Weight for criterion i
- Sum of weights = 1.0

#### Language Coverage Score:

```
LCS = Σ(Li × Pi) / Σ(Pi)
```

Where:
- `Li` = 1 if language i is supported, 0 otherwise
- `Pi` = Global speaker population for language i
- Sum over all major languages (top 100 by speakers)

### 2.4 Sustainability (SU)

**Weight: 15%**

Sustainability measures environmental impact and long-term viability.

#### Assessment Criteria:

1. **Carbon Footprint** (30%)
   - Net carbon emissions (negative is best)
   - Renewable energy usage
   - Carbon offset programs

2. **Resource Efficiency** (25%)
   - Material usage and waste reduction
   - Circular economy integration
   - Energy efficiency

3. **Ecosystem Impact** (20%)
   - Biodiversity effects
   - Habitat preservation
   - Pollution levels (air, water, soil)

4. **Long-term Viability** (15%)
   - Financial sustainability without continuous external funding
   - Organizational resilience
   - Succession planning

5. **Adaptability** (10%)
   - Climate change resilience
   - Technological adaptability
   - Scalability

#### Carbon Footprint Scoring:

```
CF_score = 1 - (actual_emissions / baseline_emissions)
```

If negative emissions (carbon-negative), score = 1.0 + |net_removal / baseline|, capped at 1.0.

### 2.5 Health & Wellbeing (HW)

**Weight: 15%**

Health & Wellbeing quantifies impact on physical and mental health.

#### Assessment Criteria:

1. **Disease Prevention** (30%)
   - Morbidity reduction
   - Mortality reduction
   - Quality-Adjusted Life Years (QALYs) gained

2. **Mental Health** (25%)
   - Stress reduction
   - Depression/anxiety mitigation
   - Social connection and support

3. **Nutrition & Food Security** (15%)
   - Nutritional quality improvement
   - Food access and availability
   - Malnutrition reduction

4. **Safety** (15%)
   - Injury prevention
   - Violence reduction
   - Occupational health improvements

5. **Quality of Life** (15%)
   - Functional status improvement
   - Pain reduction
   - Independence and autonomy

#### QALY Calculation:

```
QALY = Σ(Years_lived × Quality_of_life_weight)
```

Where quality of life weight ranges from 0 (death) to 1 (perfect health).

### 2.6 Economic Equity (EE)

**Weight: 10%**

Economic Equity measures fairness in distribution of economic benefits.

#### Assessment Criteria:

1. **Income Equality** (35%)
   - Gini coefficient improvement
   - Living wage provision
   - Income distribution across quintiles

2. **Wealth Distribution** (25%)
   - Asset ownership dispersion
   - Wealth accumulation for lower-income groups
   - Inheritance and intergenerational wealth

3. **Economic Opportunity** (25%)
   - Job creation quality
   - Skill development and career mobility
   - Entrepreneurship support

4. **Poverty Reduction** (15%)
   - Extreme poverty (< $2.15/day) reduction
   - Moderate poverty reduction
   - Vulnerability reduction

#### Gini Improvement Score:

```
GIS = (Gini_before - Gini_after) / Gini_before
```

Normalized to 0-1 scale (0.5 improvement = 1.0 score).

### 2.7 Education & Growth (ED)

**Weight: 10%**

Education & Growth evaluates contribution to knowledge and human development.

#### Assessment Criteria:

1. **Educational Access** (30%)
   - Number of learners reached
   - Underserved population access
   - Barriers removed

2. **Learning Quality** (30%)
   - Learning outcomes and retention
   - Skill acquisition
   - Competency development

3. **Knowledge Sharing** (20%)
   - Open educational resources
   - Peer learning and collaboration
   - Documentation quality

4. **Capacity Building** (20%)
   - Teacher/trainer development
   - Institutional strengthening
   - Community of practice creation

### 2.8 Innovation & Progress (IP)

**Weight: 10%**

Innovation & Progress assesses advancement of human capabilities.

#### Assessment Criteria:

1. **Technological Breakthrough** (35%)
   - Novelty and originality
   - Technical advancement over state-of-art
   - Patent significance

2. **Scientific Contribution** (25%)
   - Research publications and citations
   - New knowledge creation
   - Methodology advancement

3. **Creative Innovation** (15%)
   - Cultural and artistic contribution
   - Design innovation
   - User experience advancement

4. **Problem-Solving** (15%)
   - Complexity of problem addressed
   - Solution effectiveness
   - Generalizability

5. **Future Potential** (10%)
   - Scalability
   - Derivative innovation potential
   - Long-term impact trajectory

---

## 3. Scoring Methodology

### 3.1 Hongik Impact Score (HIS)

The overall Hongik Impact Score is calculated as:

```
HIS = [Σ(Di × Wi) × SRF × TF] × 1000
```

Where:
- `Di` = Dimension score i (0-1)
- `Wi` = Weight for dimension i (Σ Wi = 1)
- `SRF` = Stakeholder Reach Factor (1-10)
- `TF` = Temporal Factor (0.5-1.5)
- Result multiplied by 1000 for 0-1000 scale

### 3.2 Stakeholder Reach Factor (SRF)

```
SRF = 1 + 9 × [log10(AB) / log10(8,000,000,000)]
```

Where:
- `AB` = Affected beneficiaries (direct + indirect)
- `8,000,000,000` = Approximate global population
- Range: 1 (minimal reach) to 10 (global reach)

Examples:
- 100 people: SRF ≈ 1.2
- 10,000 people: SRF ≈ 2.4
- 1 million people: SRF ≈ 4.7
- 100 million people: SRF ≈ 7.8
- 1 billion people: SRF ≈ 9.0

### 3.3 Temporal Factor (TF)

The Temporal Factor adjusts for impact sustainability over time:

```
TF = 0.5 + (Duration_years / Max_expected_duration)
```

Where:
- Temporary (<1 year): TF = 0.5-0.6
- Short-term (1-3 years): TF = 0.7-0.8
- Medium-term (3-10 years): TF = 0.9-1.0
- Long-term (10-30 years): TF = 1.1-1.3
- Permanent (>30 years): TF = 1.4-1.5

### 3.4 Per-Capita Impact

```
PCI = HIS / AB
```

Where:
- `PCI` = Per-Capita Impact
- `HIS` = Hongik Impact Score
- `AB` = Affected beneficiaries

This metric is useful for comparing intensive vs. extensive impact strategies.

---

## 4. Stakeholder Analysis

### 4.1 Stakeholder Categories

1. **Direct Beneficiaries**: Individuals who directly use or receive the service/product
2. **Indirect Beneficiaries**: Families, communities, and systems that benefit secondarily
3. **Future Beneficiaries**: Generations that will benefit from sustainable practices
4. **Negative Stakeholders**: Those who may be adversely affected

### 4.2 Stakeholder Weighting

Not all beneficiaries have equal impact weight:

```
Weighted_Beneficiaries = Σ(Bi × VFi)
```

Where:
- `Bi` = Number of beneficiaries in group i
- `VFi` = Vulnerability Factor for group i

#### Vulnerability Factors:

| Group | VF | Rationale |
|-------|----|-----------|
| Children (0-12) | 1.5 | Higher developmental impact |
| Youth (13-24) | 1.2 | Critical formation period |
| Adults (25-64) | 1.0 | Baseline |
| Elderly (65+) | 1.3 | Higher vulnerability |
| People with disabilities | 1.4 | Accessibility premium |
| Extreme poverty | 1.6 | Highest need |
| Refugees/displaced | 1.5 | Crisis context |
| Indigenous peoples | 1.3 | Historical marginalization |

### 4.3 Negative Impact Adjustment

If negative stakeholders exist:

```
HIS_adjusted = HIS × (1 - NIS)
```

Where:
- `NIS` = Negative Impact Score (0-1)
- Calculated similarly to positive dimensions but inverted

---

## 5. Accessibility Metrics

### 5.1 WCAG Compliance Scoring

| WCAG Level | Base Score | Notes |
|------------|------------|-------|
| None | 0.0 | No accessibility consideration |
| A (partial) | 0.3 | Minimal compliance |
| A (full) | 0.5 | Basic accessibility |
| AA (partial) | 0.7 | Good progress |
| AA (full) | 0.85 | Industry standard |
| AAA (partial) | 0.95 | Advanced |
| AAA (full) | 1.0 | Exceptional |

### 5.2 Multi-Language Support

```
MLS = (Supported_languages / 100) × (Coverage_weight)
```

Where coverage is weighted by global speaker population.

Minimum for high score: 10 languages covering 80%+ of global population.

### 5.3 Economic Accessibility

```
EA = 1 - (Cost / Annual_median_income)
```

Adjusted for:
- Free tier availability (+0.2)
- Sliding scale pricing (+0.15)
- Open source (+0.1)
- Subsidized access for low-income (+0.2)

### 5.4 Offline and Low-Bandwidth

Progressive enhancement scoring:
- Requires high-speed internet only: 0.3
- Works on 3G: 0.6
- Works on 2G: 0.8
- Full offline functionality: 1.0

---

## 6. Temporal Impact Analysis

### 6.1 Impact Trajectory

Projects should measure impact over time:

```
HIS(t) = HIS_baseline × [1 + growth_rate × t]
```

Track quarterly and report annually.

### 6.2 Continuous Improvement

Year-over-year improvement is rewarded:

```
Improvement_bonus = (HIS_current - HIS_previous) / HIS_previous × 0.1
```

Capped at +10% bonus.

### 6.3 Impact Decay

For non-permanent projects, model impact decay:

```
HIS_residual(t) = HIS_peak × e^(-λt)
```

Where:
- `λ` = Decay constant (project-specific)
- `t` = Time since project end

---

## 7. Data Collection and Verification

### 7.1 Required Data

For each dimension, collect:
1. **Quantitative metrics**: Numbers, percentages, counts
2. **Qualitative evidence**: Case studies, testimonials
3. **Third-party verification**: Independent audits
4. **Longitudinal tracking**: Time-series data

### 7.2 Data Sources

Acceptable sources (in order of preference):
1. Randomized controlled trials (RCTs)
2. Quasi-experimental studies
3. Longitudinal observational studies
4. Cross-sectional surveys
5. Administrative data
6. Self-reported data (lowest weight)

### 7.3 Verification Process

1. **Self-Assessment**: Organization completes initial scoring
2. **Peer Review**: Independent experts review methodology and data
3. **Stakeholder Validation**: Beneficiaries confirm impact claims
4. **Audit**: Third-party audit of high-scoring projects (>700)
5. **Certification**: WIA certification board final approval

### 7.4 Confidence Intervals

All scores should report 95% confidence intervals:

```
HIS = X ± CI
```

Where CI reflects data quality, sample size, and methodology rigor.

---

## 8. Integration with WIA Ecosystem

### 8.1 WIA-INTENT Integration

Hongik scores enable intent-based queries:
```
"Find the top 10 highest-impact healthcare projects"
"Show me accessibility-focused initiatives above 800 HIS"
```

### 8.2 WIA-OMNI-API Integration

Standard API endpoints for:
- `/hongik/assess` - Calculate impact score
- `/hongik/compare` - Compare multiple projects
- `/hongik/certify` - Request certification
- `/hongik/report` - Generate impact report

### 8.3 WIA-SOCIAL Integration

Social platforms display Hongik badges:
- 🌟 Exceptional (900+)
- ⭐⭐⭐ Elite (800-899)
- ⭐⭐ High (700-799)

### 8.4 Cross-Standard Synergies

- **WIA-HEALTH**: Health dimension directly feeds HW score
- **WIA-EDU**: Education platform impacts ED score
- **WIA-ENV**: Environmental data populates SU dimension
- **WIA-AAC**: Accessibility metrics enhance AC score

---

## 9. Certification Requirements

### 9.1 Certification Levels

| Level | HIS Range | Requirements |
|-------|-----------|--------------|
| Bronze | 400-599 | Self-assessment + peer review |
| Silver | 600-699 | + Stakeholder validation |
| Gold | 700-799 | + Independent audit |
| Platinum | 800-899 | + RCT evidence for key claims |
| Diamond | 900-1000 | + Multi-year longitudinal data |

### 9.2 Renewal

Certifications valid for 2 years. Renewal requires:
- Updated impact data
- Maintained or improved HIS
- Continuous improvement plan
- No ethical violations

### 9.3 Revocation

Certification may be revoked for:
- Data fraud or misrepresentation
- Significant negative impact on stakeholders
- Ethical violations
- Failure to maintain minimum HIS

---

## 10. References

### 10.1 Foundational Philosophy

- **弘益人間 (Hongik Ingan)**: Ancient Korean philosophy from Dangun mythology, founding principle of Korean education and governance
- Korean National Ethics: "Broadly benefiting all humankind"

### 10.2 International Frameworks

- **UN Sustainable Development Goals (SDGs)**: All 17 goals align with Hongik dimensions
- **Universal Declaration of Human Rights**: Accessibility and equity principles
- **Paris Climate Agreement**: Sustainability metrics
- **WHO Quality of Life Framework**: Health & Wellbeing metrics

### 10.3 Standards and Methodologies

- **ISO 26000**: Social Responsibility
- **GRI Standards**: Sustainability Reporting
- **WCAG 2.1**: Web Content Accessibility Guidelines
- **B Corp Impact Assessment**: Multi-dimensional impact scoring
- **SROI (Social Return on Investment)**: Economic value of social impact

### 10.4 Academic Research

- Sen, A. (1999). *Development as Freedom* - Capability approach
- Rawls, J. (1971). *A Theory of Justice* - Equity frameworks
- Ostrom, E. (1990). *Governing the Commons* - Collective benefit
- IPCC Reports - Climate and sustainability science

### 10.5 Related WIA Standards

- WIA-INTENT: Intent Expression Standard
- WIA-OMNI-API: Universal API Gateway
- WIA-SOCIAL: Social Platform Standard
- WIA-HEALTH: Healthcare Data Standard
- WIA-EDU: Education Platform Standard
- WIA-ENV: Environmental Monitoring Standard
- WIA-AAC: Accessibility and Assistive Technology Standard

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
