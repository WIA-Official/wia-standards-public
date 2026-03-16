# WIA-BIO-003: Gene Therapy Specification v1.0

> **Standard ID:** WIA-BIO-003
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Viral Vector Systems](#2-viral-vector-systems)
3. [CRISPR/Cas9 Delivery](#3-crisprcas9-delivery)
4. [Dosage Calculations](#4-dosage-calculations)
5. [Safety Assessment](#5-safety-assessment)
6. [Gene Expression Monitoring](#6-gene-expression-monitoring)
7. [Clinical Trial Protocols](#7-clinical-trial-protocols)
8. [Ethics and Consent](#8-ethics-and-consent)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for gene therapy applications, from vector design to clinical implementation, ensuring safety, efficacy, and ethical compliance.

### 1.2 Scope

The standard covers:
- Viral and non-viral vector systems
- Gene editing and delivery protocols
- Dosing and pharmacokinetics
- Safety monitoring and assessment
- Clinical trial design
- Regulatory compliance

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize access to curative gene therapies while maintaining the highest safety and ethical standards to benefit all of humanity.

### 1.4 Terminology

- **Viral Genome (vg)**: Complete viral particle containing therapeutic gene
- **Transduction**: Process of delivering genetic material into cells
- **Tropism**: Natural cell/tissue targeting preference of a vector
- **Promoter**: DNA sequence controlling gene expression
- **Off-target**: Unintended genomic modification
- **Insertional Mutagenesis**: Disruption of genes by vector integration

---

## 2. Viral Vector Systems

### 2.1 Adeno-Associated Virus (AAV)

#### 2.1.1 AAV Serotypes

Common therapeutic serotypes and their tropism:

| Serotype | Primary Tropism | Genome Capacity | Clinical Use |
|----------|----------------|-----------------|--------------|
| AAV1 | Muscle, heart | 4.7 kb | Muscular dystrophy |
| AAV2 | CNS, liver | 4.7 kb | Hemophilia, CNS disorders |
| AAV5 | Lung, CNS | 4.7 kb | Cystic fibrosis |
| AAV8 | Liver, muscle | 4.7 kb | Hemophilia, metabolic disorders |
| AAV9 | CNS, heart, muscle | 4.7 kb | SMA, cardiac disorders |
| AAVrh10 | Liver, muscle | 4.7 kb | Liver-directed therapies |

#### 2.1.2 AAV Vector Design

AAV vector components:
```
5'-ITR-[Promoter]-[Therapeutic Gene]-[PolyA]-ITR-3'
```

Where:
- `ITR` = Inverted Terminal Repeats (145 bp each)
- `Promoter` = Tissue-specific or ubiquitous promoter
- `Therapeutic Gene` = Coding sequence (max ~4.5 kb)
- `PolyA` = Polyadenylation signal

#### 2.1.3 AAV Production Titer

Vector titer calculation:
```
Titer (vg/mL) = (DNA copies × Dilution factor) / Volume
```

Quality control thresholds:
- Minimum: 1×10¹³ vg/mL
- Clinical grade: >1×10¹³ vg/mL
- Empty/full capsid ratio: <10%

### 2.2 Lentiviral Vectors

#### 2.2.1 Lentiviral Vector Design

Third-generation lentiviral system:
```
5'-LTR-ψ-RRE-[Promoter]-[Gene]-[WPRE]-cPPT-LTR-3'
```

Components:
- `LTR` = Long Terminal Repeats
- `ψ` = Packaging signal
- `RRE` = Rev Response Element
- `WPRE` = Woodchuck Posttranscriptional Regulatory Element
- `cPPT` = Central Polypurine Tract

#### 2.2.2 Lentiviral Titer

Functional titer (TU/mL):
```
Titer = (Transduced cells × Dilution) / Volume
```

Typical titers:
- Research grade: 1×10⁷ TU/mL
- Clinical grade: >1×10⁸ TU/mL

### 2.3 Lipid Nanoparticles (LNP)

#### 2.3.1 LNP Composition

Standard LNP formulation:
- Ionizable lipid: 45-50%
- Phospholipid (DSPC): 10-15%
- Cholesterol: 38-40%
- PEG-lipid: 1.5-2.5%

#### 2.3.2 Encapsulation Efficiency

```
EE = (Encapsulated mRNA / Total mRNA) × 100%
```

Target: >90% encapsulation efficiency

---

## 3. CRISPR/Cas9 Delivery

### 3.1 CRISPR Components

Complete CRISPR system:
```
Cas9 protein + sgRNA + Donor template (optional)
```

Guide RNA design:
```
5'-[20 nt target sequence]-[Scaffold]-3'
```

### 3.2 Delivery Methods

| Method | Efficiency | Cell Type | Duration | Off-target Risk |
|--------|-----------|-----------|---------|-----------------|
| AAV-CRISPR | High | Non-dividing | Persistent | Low-Medium |
| LNP-mRNA | Medium-High | Broad | Transient | Low |
| Electroporation | High | Ex vivo | Transient | Very Low |
| RNP delivery | Medium | Ex vivo | Transient | Very Low |

### 3.3 On-target Efficiency

```
OTE = (Edited alleles / Total alleles) × 100%
```

Where:
- OTE = On-target editing efficiency
- Target: >70% for therapeutic applications

### 3.4 Off-target Prediction

Off-target score calculation:
```
OT_score = Σ (Mismatch penalty × Position weight)
```

Mismatch penalties:
- PAM-proximal (1-10 bp): -4 to -6 per mismatch
- PAM-distal (11-20 bp): -1 to -2 per mismatch

Target: Minimize sites with OT_score > -10

### 3.5 CRISPR Safety Assessment

Required assessments:
1. **In silico prediction**: CRISPOR, Cas-OFFinder
2. **In vitro validation**: Next-generation sequencing of predicted sites
3. **Whole genome sequencing**: Unbiased detection (optional for research)
4. **Functional assays**: Cell viability and karyotyping

---

## 4. Dosage Calculations

### 4.1 AAV Dosing Formula

```
Dose (vg/kg) = (Target cells × vg per cell) / Patient weight
```

Standard calculations:

#### 4.1.1 Liver-directed Therapy
```
Dose = (2×10¹¹ hepatocytes × 10⁵ vg/cell) / 70 kg
Dose ≈ 3×10¹⁴ vg/kg
```

#### 4.1.2 Muscle-directed Therapy
```
Dose = (Target muscle mass × Cell density × vg/cell) / Weight
Dose = (500 g × 10⁹ cells/g × 10⁴ vg/cell) / 70 kg
Dose ≈ 7×10¹³ vg/kg
```

#### 4.1.3 CNS-directed Therapy
```
Dose (intrathecal) = 1×10¹³ to 2×10¹⁴ vg total
```

### 4.2 Dose Scaling

Allometric scaling from preclinical to clinical:
```
Dose_human = Dose_animal × (Weight_human / Weight_animal)^0.75
```

### 4.3 Maximum Tolerated Dose (MTD)

Safety margins:
- Preclinical MTD: Highest dose without toxicity
- Clinical starting dose: MTD / 10 (safety factor)
- Dose escalation: 3-fold increments (3+3 design)

### 4.4 Dose-Response Relationship

```
Response = R_max × (Dose^n) / (ED50^n + Dose^n)
```

Where:
- `R_max` = Maximum response
- `ED50` = Dose producing 50% response
- `n` = Hill coefficient (cooperativity)

---

## 5. Safety Assessment

### 5.1 Immunogenicity Testing

#### 5.1.1 Humoral Immunity

Pre-existing antibody testing:
```
NAb titer = 1 / (Highest dilution blocking ≥50% transduction)
```

Exclusion criteria:
- AAV: NAb titer >1:5
- Lentivirus: Not typically tested (ex vivo use)

#### 5.1.2 Cellular Immunity

T-cell response monitoring:
```
IFN-γ ELISpot: >50 spots per 10⁶ cells = positive
```

Monitor at: Days 0, 7, 14, 28, 90, 180, 365

### 5.2 Liver Toxicity

Transaminase monitoring:
```
ALT/AST elevation = (Current level / Baseline) × ULN
```

Action thresholds:
- Grade 1 (1-3× ULN): Continue, monitor weekly
- Grade 2 (3-5× ULN): Consider corticosteroids
- Grade 3 (5-20× ULN): Initiate immunosuppression
- Grade 4 (>20× ULN): Emergency intervention

### 5.3 Thrombotic Microangiopathy (TMA)

For high-dose AAV (>2×10¹⁴ vg/kg):

Monitor:
- Platelet count
- LDH levels
- Haptoglobin
- Schistocytes

TMA risk score:
```
Risk = 0.3×(Dose/10¹⁴) + 0.2×(Age/10) + 0.5×(Baseline_platelets/100)
```

Risk > 2.0: Enhanced monitoring required

### 5.4 Insertional Mutagenesis

For integrating vectors (lentivirus):
```
Integration site analysis via LAM-PCR or targeted sequencing
```

Monitor:
- Clonal expansion (>30% of population)
- Integration near oncogenes (±50 kb)
- Chromosome abnormalities

### 5.5 Biodistribution

Vector biodistribution measurement:
```
Copies/diploid genome = (2 × vg detected) / Total genomic DNA
```

Expected tissues:
- Target tissue: >1 copy/cell
- Liver: Variable (systemic administration)
- Gonads: <0.001 copy/cell (regulatory limit)

---

## 6. Gene Expression Monitoring

### 6.1 mRNA Quantification

RT-qPCR measurement:
```
Relative expression = 2^(-ΔΔCt)
```

Where:
```
ΔΔCt = (Ct_target - Ct_reference)_treated - (Ct_target - Ct_reference)_control
```

### 6.2 Protein Expression

#### 6.2.1 ELISA Quantification
```
Concentration = (Sample OD - Blank) / Standard curve slope
```

#### 6.2.2 Western Blot Densitometry
```
Relative protein = (Band intensity_target / Band intensity_loading control)
```

### 6.3 Functional Assays

For Factor VIII (hemophilia):
```
FVIII activity (%) = (Patient clotting time / Normal clotting time) × 100%
```

Therapeutic range: 40-150% of normal

### 6.4 Expression Kinetics

```
E(t) = E_max × (1 - e^(-k×t))
```

Where:
- `E(t)` = Expression at time t
- `E_max` = Maximum expression
- `k` = Expression rate constant
- `t` = Time post-administration

Typical kinetics:
- AAV: Peak at 4-8 weeks, plateau by 12 weeks
- Lentivirus: Peak at 2-4 weeks (ex vivo)

---

## 7. Clinical Trial Protocols

### 7.1 Phase I/II Trial Design

#### 7.1.1 Dose Escalation (3+3 Design)

```
Cohort 1: 3 patients at dose level 1
├─ If 0 DLT: Escalate to dose level 2
├─ If 1 DLT: Add 3 more patients
│   ├─ If total ≤1 DLT in 6: Escalate
│   └─ If total ≥2 DLT in 6: De-escalate
└─ If ≥2 DLT: De-escalate
```

#### 7.1.2 Inclusion Criteria

Standard criteria:
- Confirmed genetic diagnosis
- Measurable disease phenotype
- AAV NAb titer <1:5
- Normal liver/kidney function (within 1.5× ULN)
- No active infections
- Age: Typically ≥18 years (adult trials)

#### 7.1.3 Exclusion Criteria

- Active hepatitis B/C
- HIV positive (for lentiviral trials)
- Severe immunodeficiency
- Pregnancy/nursing
- Recent immunosuppression
- Malignancy within 5 years

### 7.2 Endpoints

#### 7.2.1 Primary Endpoints
- Safety: Adverse events, immune responses
- Dose-limiting toxicity (DLT)
- Maximum tolerated dose (MTD)

#### 7.2.2 Secondary Endpoints
- Transgene expression levels
- Functional improvement
- Quality of life (QoL) scores
- Pharmacokinetics/pharmacodynamics

### 7.3 Monitoring Schedule

| Timepoint | Assessments |
|-----------|-------------|
| Screening | Genotype, NAb, baseline labs, imaging |
| Day 0 | Administration, vitals (q15min × 2hr) |
| Day 1-7 | Daily labs (CBC, CMP, LFTs) |
| Week 2-4 | Weekly labs, transgene expression |
| Week 8-12 | Monthly labs, functional tests |
| Month 6-12 | Quarterly follow-up |
| Year 2-5 | Annual long-term monitoring |

### 7.4 Dose-Limiting Toxicity (DLT)

Definition of DLT:
- Grade ≥3 immune-related adverse events
- Grade ≥3 hepatotoxicity lasting >7 days
- Grade ≥3 non-hematologic toxicity (excluding transient fever)
- Grade 4 hematologic toxicity
- Thrombotic microangiopathy
- Death or life-threatening event

---

## 8. Ethics and Consent

### 8.1 Informed Consent Requirements

#### 8.1.1 Essential Elements

1. **Purpose**: Clear explanation of gene therapy goals
2. **Risks**: Detailed risks including:
   - Immune responses
   - Liver toxicity
   - Cancer risk (integrating vectors)
   - Unknown long-term effects
   - Germline transmission potential
3. **Benefits**: Potential therapeutic benefit
4. **Alternatives**: Standard of care options
5. **Confidentiality**: Genetic privacy protection
6. **Long-term monitoring**: Commitment to 5-15 year follow-up

#### 8.1.2 Pediatric Considerations

For patients <18 years:
- Parental consent required
- Child assent (age ≥7 years)
- Age-appropriate explanations
- Best interest standard

### 8.2 Germline Modification

**Prohibited**: Intentional germline modification

Monitoring requirements:
- Gonadal biodistribution testing
- Contraception requirements (6-12 months post-treatment)
- Partner notification protocols

### 8.3 Ethical Review

Required approvals:
1. Institutional Review Board (IRB) / Ethics Committee
2. Institutional Biosafety Committee (IBC)
3. Regulatory authorities (FDA, EMA, etc.)
4. Data Safety Monitoring Board (DSMB) for multi-site trials

### 8.4 Genetic Privacy

Data protection:
- De-identified genetic data storage
- Restricted access controls
- Encryption of genomic databases
- Patient rights to data deletion (GDPR compliance)

---

## 9. Implementation Guidelines

### 9.1 Required Components

Any WIA-BIO-003 compliant system must include:

1. **Dosage Calculator**: Compute optimal vector dose
2. **Safety Assessor**: Monitor adverse events and biomarkers
3. **Expression Tracker**: Quantify transgene expression
4. **Protocol Generator**: Create clinical trial protocols
5. **Consent Manager**: Handle informed consent documentation

### 9.2 API Interface

#### 9.2.1 Calculate Dosage
```typescript
interface DosageRequest {
  patientWeight: number;        // kg
  targetTissue: string;         // 'liver', 'muscle', 'CNS', etc.
  vectorType: string;           // 'AAV9', 'AAV8', 'lentivirus', etc.
  therapeuticGene: string;      // Gene symbol
  serotype?: string;            // AAV serotype
}

interface DosageResponse {
  viralGenomes: number;         // Total vg required
  vgPerKg: number;              // Dose in vg/kg
  volumeMl: number;             // Injection volume
  expectedEfficiency: number;   // Predicted transduction %
  feasibility: 'standard' | 'high-dose' | 'experimental';
}
```

#### 9.2.2 Assess Safety
```typescript
interface SafetyAssessment {
  vectorDose: number;           // vg/kg
  immuneStatus: string;         // 'normal', 'suppressed', 'autoimmune'
  preexistingAntibodies: boolean;
  liverFunction: {
    ALT: number;                // U/L
    AST: number;                // U/L
    bilirubin: number;          // mg/dL
  };
  age: number;                  // years
}

interface SafetyResult {
  score: number;                // 0-100 safety score
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';
  warnings: string[];
  recommendations: string[];
  monitoringPlan: {
    frequency: string;
    biomarkers: string[];
  };
}
```

#### 9.2.3 Monitor Expression
```typescript
interface ExpressionMonitoring {
  gene: string;                 // Therapeutic gene
  timePoints: number[];         // Days post-treatment
  method: 'qPCR' | 'ELISA' | 'activity-assay';
  baselineValue?: number;
}

interface ExpressionResult {
  measurements: {
    timepoint: number;
    value: number;
    unit: string;
    percentOfNormal: number;
  }[];
  trend: 'increasing' | 'stable' | 'decreasing';
  therapeuticRange: boolean;
  peakExpression: {
    value: number;
    timepoint: number;
  };
}
```

### 9.3 Data Formats

#### 9.3.1 Patient Record
```json
{
  "patientId": "PT-12345",
  "diagnosis": "Hemophilia A",
  "genotype": "F8 intron-22 inversion",
  "baselineFactorLevel": 0.5,
  "NAb_titer": {
    "AAV9": "<1:5",
    "AAV8": "1:10"
  },
  "weight": 70,
  "age": 35,
  "liverFunction": {
    "ALT": 25,
    "AST": 30,
    "totalBilirubin": 0.8
  }
}
```

#### 9.3.2 Treatment Plan
```json
{
  "treatmentId": "TX-67890",
  "patientId": "PT-12345",
  "vector": {
    "type": "AAV9",
    "gene": "F8",
    "promoter": "HLP",
    "dose_vg": 2e13,
    "dose_vg_per_kg": 2.86e12,
    "volume_ml": 50,
    "route": "IV infusion"
  },
  "premedication": [
    "Methylprednisolone 500mg IV",
    "Diphenhydramine 50mg IV"
  ],
  "monitoring": {
    "hospital_observation": "24 hours",
    "weekly_labs": "Weeks 1-4",
    "monthly_labs": "Months 2-6"
  }
}
```

### 9.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Dose exceeds MTD | Reduce dose or split administration |
| B002 | High NAb titer detected | Consider alternative serotype |
| B003 | Severe liver dysfunction | Defer treatment until resolved |
| B004 | Off-target rate too high | Redesign guide RNA |
| B005 | Immunogenicity alert | Initiate immunosuppression |
| B006 | Vector quality failure | Do not use, investigate production |

---

## 10. References

### 10.1 Seminal Publications

1. Dunbar, C.E., et al. (2018). "Gene therapy comes of age." *Science* 359(6372).
2. Naldini, L. (2015). "Gene therapy returns to centre stage." *Nature* 526:351-360.
3. High, K.A., Roncarolo, M.G. (2019). "Gene Therapy." *NEJM* 381:455-464.
4. Wang, D., Tai, P.W.L., Gao, G. (2019). "Adeno-associated virus vector as a platform for gene therapy." *Nat Rev Drug Discov* 18:358-378.
5. Doudna, J.A., Charpentier, E. (2014). "The new frontier of genome engineering with CRISPR-Cas9." *Science* 346(6213).

### 10.2 Regulatory Guidance

- FDA: "Human Gene Therapy for Rare Diseases" (2020)
- EMA: "Guideline on quality, non-clinical and clinical aspects of gene therapy" (2018)
- ICH S12: "Nonclinical Biodistribution Considerations for Gene Therapy Products" (2022)

### 10.3 Clinical Trial Examples

| Disease | Vector | Gene | Dose | Status | Reference |
|---------|--------|------|------|--------|-----------|
| Hemophilia A | AAV5-hFVIII-SQ | F8 | 6×10¹³ vg/kg | Approved | Zolgensma |
| Hemophilia B | AAV-Spark200-hFIX | F9 | 2×10¹² vg/kg | Approved | Etranacogene dezaparvovec |
| SMA | AAV9-SMN1 | SMN1 | 1.1×10¹⁴ vg | Approved | Onasemnogene abeparvovec |
| LCA | AAV2-hRPE65 | RPE65 | 1.5×10¹¹ vg/eye | Approved | Voretigene neparvovec |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based therapy design
- WIA-OMNI-API: Universal genomics data integration
- WIA-HEALTH: Electronic health record standards
- WIA-DATA: Clinical research data management

---

## Appendix A: Example Calculations

### A.1 Hemophilia A Treatment (AAV9-F8)

```
Given:
- Patient weight: 70 kg
- Target: Liver hepatocytes
- Vector: AAV9-F8
- Target expression: 40% of normal FVIII

Calculation:
- Hepatocyte count: ~2×10¹¹ cells
- Required vg/cell: 10⁵ (for 40% expression)
- Total vg needed: 2×10¹¹ × 10⁵ = 2×10¹⁶ vg
- Dose per kg: 2×10¹⁶ / 70 = 2.86×10¹⁴ vg/kg

Volume:
- Vector titer: 1×10¹³ vg/mL
- Required volume: 2×10¹⁶ / 10¹³ = 2000 mL (impractical)
- Concentrate to: 5×10¹³ vg/mL → 400 mL (feasible)

Result:
- Dose: 2.86×10¹⁴ vg/kg
- Volume: 400 mL IV infusion over 2 hours
```

### A.2 Off-target Risk Calculation

```
Given:
- Guide RNA: 5'-GACCCCCTCCACCCCGCCTC-3'
- Off-target site: 5'-GACCCGCTCCACCCCGCCTC-3' (1 mismatch at position 6)

Calculation:
- Mismatch at position 6 (PAM-proximal): -5 penalty
- OT_score = -5
- Risk: HIGH (score > -10)

Action:
- Redesign guide RNA or use high-fidelity Cas9 variant
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-003 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
