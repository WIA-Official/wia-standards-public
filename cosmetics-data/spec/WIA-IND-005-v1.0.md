# WIA-IND-005: Cosmetics Data Specification v1.0

> **Standard ID:** WIA-IND-005
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Ingredient Database](#2-ingredient-database)
3. [Safety Assessment](#3-safety-assessment)
4. [Efficacy Tracking](#4-efficacy-tracking)
5. [Regulatory Compliance](#5-regulatory-compliance)
6. [Supply Chain Transparency](#6-supply-chain-transparency)
7. [Product Formulation](#7-product-formulation)
8. [Quality Control](#8-quality-control)
9. [Consumer Safety](#9-consumer-safety)
10. [Sustainability](#10-sustainability)
11. [Data Formats](#11-data-formats)
12. [API Interface](#12-api-interface)
13. [Security and Privacy](#13-security-and-privacy)
14. [References](#14-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for cosmetics and personal care product data management. The standard provides standardized methods for ingredient tracking, safety assessment, efficacy validation, regulatory compliance, and supply chain transparency across the global cosmetics industry.

### 1.2 Scope

The standard covers:
- Complete ingredient database with INCI nomenclature and chemical data
- Safety assessment methodologies including toxicological endpoints
- Efficacy tracking through clinical trials and consumer studies
- Regulatory compliance across major markets (US, EU, Asia-Pacific)
- Supply chain transparency from raw materials to finished products
- Product formulation design and stability testing
- Quality control protocols and manufacturing standards
- Consumer safety including allergen management and adverse event reporting
- Sustainability metrics for ingredients, packaging, and manufacturing

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to ensure that cosmetics products are safe, effective, and transparent. By providing a unified framework for ingredient safety, regulatory compliance, and consumer protection, we enable the beauty industry to serve humanity's wellbeing while fostering innovation and sustainability.

### 1.4 Terminology

- **INCI**: International Nomenclature of Cosmetic Ingredients
- **CAS**: Chemical Abstracts Service registry number
- **CPSR**: Cosmetic Product Safety Report (EU requirement)
- **CPNP**: Cosmetic Products Notification Portal (EU)
- **NOAEL**: No Observed Adverse Effect Level
- **LOAEL**: Lowest Observed Adverse Effect Level
- **MoS**: Margin of Safety
- **SED**: Systemic Exposure Dose
- **HRIPT**: Human Repeat Insult Patch Test
- **RIPT**: Repeat Insult Patch Test
- **SPF**: Sun Protection Factor
- **PA**: Protection Grade of UVA (Asian standard)
- **LLNA**: Local Lymph Node Assay (sensitization test)
- **GMP**: Good Manufacturing Practice
- **ISO 22716**: Cosmetics GMP standard

---

## 2. Ingredient Database

### 2.1 INCI Nomenclature System

#### 2.1.1 Naming Conventions

The INCI (International Nomenclature of Cosmetic Ingredients) system provides standardized ingredient naming:

**Botanical Ingredients**:
```
Format: [Latin binomial name] + [Part used] + [Extract type]
Example: Camellia Sinensis Leaf Extract
```

**Chemical Compounds**:
```
Format: [IUPAC-based systematic name]
Example: Sodium Hyaluronate (not Hyaluronic Acid Sodium Salt)
```

**Mixtures**:
```
Format: [Component 1], [Component 2], ... (in descending order)
Example: Cetearyl Alcohol (mixture of Cetyl and Stearyl alcohols)
```

#### 2.1.2 Ingredient Identification

Each ingredient must have:

```typescript
interface IngredientIdentification {
  inci_name: string;                    // INCI nomenclature
  cas_number?: string;                  // CAS registry number
  ec_number?: string;                   // European Commission number
  einecs_number?: string;               // European Inventory number
  iupac_name?: string;                  // IUPAC chemical name
  common_names?: string[];              // Trade names, common names
  molecular_formula?: string;           // Chemical formula
  molecular_weight?: number;            // g/mol
  chemical_structure?: string;          // SMILES or InChI notation
}
```

#### 2.1.3 Ingredient Categories

**Primary Classification**:
1. **Actives**: Functional ingredients providing claimed benefits
2. **Emollients**: Skin softening and moisturizing agents
3. **Humectants**: Water-binding agents
4. **Preservatives**: Antimicrobial protection
5. **Emulsifiers**: Oil-water phase stabilizers
6. **Surfactants**: Cleansing and foaming agents
7. **Thickeners**: Viscosity modifiers
8. **Colorants**: Pigments and dyes
9. **Fragrances**: Perfuming compounds
10. **pH Adjusters**: Acid-base modifiers
11. **Antioxidants**: Oxidation inhibitors
12. **Chelating Agents**: Metal ion sequestrants
13. **Solvents**: Liquid carriers and diluents
14. **Film Formers**: Surface coating agents
15. **Opacifiers**: Opacity-enhancing agents

#### 2.1.4 Functional Properties

```typescript
interface IngredientFunction {
  primary_function: FunctionCategory;
  secondary_functions?: FunctionCategory[];
  mechanism_of_action?: string;
  efficacy_data?: EfficacyData;
  typical_usage_range: {
    min_percent: number;
    max_percent: number;
    optimal_percent: number;
  };
  solubility: {
    water: SolubilityLevel;
    oil: SolubilityLevel;
    alcohol: SolubilityLevel;
  };
  stability: {
    ph_range: { min: number; max: number };
    temperature_range: { min: number; max: number };
    light_sensitivity: 'none' | 'low' | 'moderate' | 'high';
    oxidation_sensitivity: 'none' | 'low' | 'moderate' | 'high';
  };
}
```

### 2.2 Chemical Data

#### 2.2.1 Physical Properties

```typescript
interface PhysicalProperties {
  appearance: {
    physical_state: 'solid' | 'liquid' | 'gas' | 'semi-solid';
    color: string;
    odor: string;
  };
  melting_point?: { value: number; unit: 'celsius' | 'fahrenheit' };
  boiling_point?: { value: number; unit: 'celsius' | 'fahrenheit' };
  density?: { value: number; unit: 'g/mL' };
  refractive_index?: number;
  viscosity?: { value: number; unit: 'cP' | 'mPa·s'; temperature: number };
  ph?: { value: number; concentration: string };
  flash_point?: { value: number; unit: 'celsius' };
  partition_coefficient?: number;  // log P (octanol/water)
}
```

#### 2.2.2 Chemical Properties

```typescript
interface ChemicalProperties {
  reactivity: {
    stability: 'stable' | 'unstable' | 'reactive';
    incompatibilities: string[];
    decomposition_products?: string[];
    polymerization: 'none' | 'possible' | 'hazardous';
  };
  hygroscopicity: 'none' | 'slight' | 'moderate' | 'high';
  oxidation_potential?: number;
  reduction_potential?: number;
  pka?: number;  // Acid dissociation constant
  ionic_character?: 'non-ionic' | 'anionic' | 'cationic' | 'amphoteric';
}
```

### 2.3 Botanical Ingredients

#### 2.3.1 Plant Identification

```typescript
interface BotanicalIngredient {
  scientific_name: {
    genus: string;
    species: string;
    variety?: string;
    author?: string;
  };
  common_names: Record<string, string>;  // locale -> name
  plant_part: 'leaf' | 'root' | 'flower' | 'seed' | 'bark' | 'fruit' | 'whole plant';
  extraction_method: ExtractionMethod;
  solvent_used?: string;
  standardization?: {
    marker_compound: string;
    concentration: number;
    unit: string;
  };
  geographical_origin?: string[];
  harvest_season?: string;
  cultivation_method?: 'wild-crafted' | 'organic' | 'conventional';
}
```

#### 2.3.2 Extraction Methods

**Common Extraction Types**:
1. **Aqueous Extract**: Water-based extraction
2. **Hydroalcoholic Extract**: Water-alcohol mixture
3. **Glycolic Extract**: Glycerin-based
4. **Oil Infusion**: Lipid-soluble extraction
5. **CO2 Extract**: Supercritical CO2 extraction
6. **Steam Distillation**: Essential oil production
7. **Cold Press**: Mechanical extraction
8. **Maceration**: Solvent steeping
9. **Percolation**: Solvent flow-through
10. **Enfleurage**: Fat-based extraction (rare)

#### 2.3.3 Active Constituents

```typescript
interface BotanicalConstituents {
  primary_actives: Array<{
    compound_name: string;
    chemical_class: string;
    concentration_range: { min: number; max: number; unit: string };
    bioactivity: string[];
  }>;
  secondary_compounds?: Array<{
    compound_name: string;
    concentration: number;
    role: string;
  }>;
  allergens?: string[];
  potential_contaminants?: string[];
}
```

### 2.4 Synthetic Ingredients

#### 2.4.1 Manufacturing Process

```typescript
interface SyntheticIngredient {
  synthesis_method: string;
  starting_materials: string[];
  reaction_conditions: {
    temperature?: number;
    pressure?: number;
    catalyst?: string;
    solvent?: string;
    reaction_time?: number;
  };
  purification_method: string[];
  purity_specification: {
    minimum_purity: number;
    impurity_limits: Record<string, number>;
  };
  manufacturing_standard?: 'GMP' | 'USP' | 'EP' | 'BP' | 'JP';
}
```

### 2.5 Ingredient Grades

#### 2.5.1 Quality Grades

**Cosmetic Grade**: Specifically manufactured for cosmetic use
- Purity: Typically 95-100%
- Microbial limits: < 100 CFU/g (total aerobic)
- Heavy metals: Controlled to cosmetic limits
- Allergens: Disclosed and controlled

**Pharmaceutical Grade**: Higher purity for medicinal use
- Purity: 99%+ (USP/EP/JP standards)
- Microbial limits: < 10 CFU/g
- Heavy metals: Strict pharmaceutical limits
- Endotoxins: Controlled (< 5 EU/mg)

**Food Grade**: Suitable for oral consumption
- Purity: Varies by ingredient
- Microbial: Food safety standards
- Contaminants: FDA/EFSA limits
- GMO status: Labeled

**Technical/Industrial Grade**: Lower purity, not for cosmetics
- Purity: < 95%
- Contaminants: Not controlled for cosmetic use
- Not suitable for skin contact

---

## 3. Safety Assessment

### 3.1 Toxicological Profile

#### 3.1.1 Acute Toxicity

**Oral Toxicity**:
```
LD50 (Lethal Dose 50%)
- Measured in mg/kg body weight
- Categories (GHS):
  - Category 1: LD50 ≤ 5 mg/kg (Fatal)
  - Category 2: 5 < LD50 ≤ 50 mg/kg (Fatal)
  - Category 3: 50 < LD50 ≤ 300 mg/kg (Toxic)
  - Category 4: 300 < LD50 ≤ 2000 mg/kg (Harmful)
  - Category 5: 2000 < LD50 ≤ 5000 mg/kg (May be harmful)
  - Unclassified: LD50 > 5000 mg/kg
```

**Dermal Toxicity**:
```
LD50 (Dermal)
- Categories similar to oral
- Important for cosmetic ingredients
- Considers skin penetration rate
```

**Inhalation Toxicity**:
```
LC50 (Lethal Concentration 50%)
- Measured in mg/L air for 4 hours
- Relevant for aerosols, powders, sprays
```

#### 3.1.2 Skin Irritation

**Primary Irritation Index (PII)**:
```
PII = Σ (Erythema scores + Edema scores) / Number of observations

Classification:
- Non-irritating: PII = 0
- Negligible: 0 < PII < 0.5
- Slight: 0.5 ≤ PII < 2.0
- Moderate: 2.0 ≤ PII < 5.0
- Severe: 5.0 ≤ PII ≤ 8.0
```

**Alternative Methods** (non-animal):
- EpiDerm™ (reconstructed human epidermis)
- SkinEthic™ RHE (reconstructed human epidermis)
- In vitro irritation testing

#### 3.1.3 Eye Irritation

**Modified Draize Test** (being phased out):
```
Scoring System:
- Cornea opacity: 0-4
- Iris inflammation: 0-2
- Conjunctival redness: 0-3
- Conjunctival edema: 0-4

Total score ≥ 3 considered irritating
```

**Alternative Methods**:
- Bovine Corneal Opacity and Permeability (BCOP)
- Isolated Chicken Eye (ICE)
- HET-CAM (Hen's Egg Test-Chorioallantoic Membrane)
- EpiOcular™ (reconstructed human cornea)

#### 3.1.4 Skin Sensitization

**Local Lymph Node Assay (LLNA)**:
```
Stimulation Index (SI) = (Test group DPM) / (Control group DPM)

Classification:
- Non-sensitizer: SI < 3
- Weak sensitizer: 3 ≤ SI < 10
- Moderate sensitizer: 10 ≤ SI < 30
- Strong sensitizer: SI ≥ 30

EC3 = Concentration giving SI = 3
- Strong: EC3 < 0.1%
- Moderate: 0.1% ≤ EC3 < 1%
- Weak: 1% ≤ EC3 < 10%
```

**Human Repeat Insult Patch Test (HRIPT)**:
```
Protocol:
- Induction phase: 9 applications over 3 weeks
- Rest period: 2 weeks
- Challenge phase: Single application
- Evaluation: 24h, 48h, 72h post-challenge

Interpretation:
- No reaction in ≥95% of subjects: Non-sensitizing
- Reaction in 5-10%: Weak sensitizer
- Reaction in >10%: Sensitizer (not acceptable)
```

#### 3.1.5 Phototoxicity and Photoallergenicity

**3T3 Neutral Red Uptake Phototoxicity Test (3T3 NRU PT)**:
```
Photo-Irritation Factor (PIF) = IC50 (-UV) / IC50 (+UV)

Classification:
- No phototoxic potential: PIF < 2
- Probable phototoxic potential: 2 ≤ PIF < 5
- Phototoxic: PIF ≥ 5
```

**Photopatch Test** (for photoallergenicity):
```
Protocol:
- Duplicate patches applied
- One side irradiated with UVA (5-10 J/cm²)
- Other side protected from light
- Comparison of reactions

Positive: Reaction only on irradiated side
```

### 3.2 Systemic Toxicity

#### 3.2.1 Repeated Dose Toxicity

**Subacute Toxicity** (28-day study):
- Daily exposure for 28 days
- Observations: Body weight, organ weights, histopathology
- Identifies target organs

**Subchronic Toxicity** (90-day study):
- Daily exposure for 90 days
- More comprehensive than 28-day
- Determines NOAEL/LOAEL

**Chronic Toxicity** (> 6 months):
- Long-term exposure effects
- Combined with carcinogenicity studies

#### 3.2.2 Genotoxicity

**Bacterial Reverse Mutation Test (Ames Test)**:
```
Strains used:
- S. typhimurium: TA98, TA100, TA1535, TA1537
- E. coli: WP2 uvrA

With and without metabolic activation (S9)

Result: Positive if ≥2x increase in revertants
```

**In Vitro Micronucleus Test**:
```
- Uses mammalian cells (CHO, V79, or human lymphocytes)
- Detects chromosomal damage
- With/without S9 activation
```

**In Vivo Micronucleus Test**:
```
- Rodent bone marrow or peripheral blood
- Gold standard for genotoxicity
- If positive: serious concern for mutagenicity
```

#### 3.2.3 Carcinogenicity

**Assessment Approaches**:
1. **2-year rodent bioassay**: Long-term exposure study
2. **Structure-activity relationships (SAR)**: Chemical structure analysis
3. **Short-term tests**: Genotoxicity battery
4. **Epidemiological data**: Human exposure history

**Classification** (IARC):
- Group 1: Carcinogenic to humans
- Group 2A: Probably carcinogenic to humans
- Group 2B: Possibly carcinogenic to humans
- Group 3: Not classifiable
- Group 4: Probably not carcinogenic

#### 3.2.4 Reproductive and Developmental Toxicity (DART)

**Reproductive Toxicity**:
```
Endpoints:
- Fertility indices
- Mating behavior
- Conception rates
- Pregnancy outcomes
- Lactation
- Offspring development
```

**Developmental Toxicity**:
```
Endpoints:
- Embryo-fetal mortality
- Fetal weight and growth
- Structural abnormalities
- Functional deficits
- Behavioral alterations
```

### 3.3 Risk Assessment

#### 3.3.1 Margin of Safety (MoS)

```
MoS = NOAEL / SED

Where:
- NOAEL = No Observed Adverse Effect Level (mg/kg bw/day)
- SED = Systemic Exposure Dose (mg/kg bw/day)

SED Calculation:
SED = (A × C × DAp) / BW

Where:
- A = Amount of product applied (g or mL)
- C = Concentration of ingredient (%)
- DAp = Dermal absorption percentage (%)
- BW = Body weight (kg)

Acceptability:
- MoS ≥ 100: Generally safe
- MoS 10-100: Marginal, requires justification
- MoS < 10: Unacceptable risk
```

#### 3.3.2 Aggregate Exposure

```
Total Daily Exposure = Σ (Product exposure)ᵢ

For each product:
Exposure = (Amount used × Concentration × Frequency) / Body weight

Considerations:
- Multiple products using same ingredient
- Different application sites
- Frequency of use
- Duration of exposure
- Age-specific exposure scenarios
```

#### 3.3.3 Population-Specific Risk

**Vulnerable Populations**:
1. **Infants and Children** (< 3 years)
   - Higher surface area to body weight ratio
   - Immature metabolic systems
   - Additional safety factor: 10x

2. **Pregnant Women**
   - Fetal exposure considerations
   - Endocrine disruption concerns
   - Teratogenic potential

3. **Elderly** (> 65 years)
   - Compromised skin barrier
   - Reduced metabolic capacity
   - Polypharmacy interactions

4. **Sensitive Skin Conditions**
   - Atopic dermatitis
   - Rosacea
   - Psoriasis
   - Compromised barrier function

### 3.4 Safety Data Sheet (SDS)

#### 3.4.1 SDS Sections (GHS Format)

**Section 1: Identification**
- Product identifier
- Recommended use
- Supplier details
- Emergency telephone number

**Section 2: Hazard Identification**
- GHS classification
- Hazard statements
- Precautionary statements
- Pictograms

**Section 3: Composition/Ingredients**
- Chemical identity
- CAS number
- Concentration ranges
- Impurities

**Section 4: First-Aid Measures**
- Eye contact
- Skin contact
- Ingestion
- Inhalation

**Section 5: Fire-Fighting Measures**
- Suitable extinguishing media
- Specific hazards
- Special protective equipment

**Section 6: Accidental Release Measures**
- Personal precautions
- Environmental precautions
- Containment and cleanup methods

**Section 7: Handling and Storage**
- Precautions for safe handling
- Storage conditions
- Incompatibilities

**Section 8: Exposure Controls/Personal Protection**
- Occupational exposure limits
- Engineering controls
- Personal protective equipment

**Section 9: Physical and Chemical Properties**
- Appearance, odor, pH
- Melting/boiling point
- Flash point
- Solubility

**Section 10: Stability and Reactivity**
- Chemical stability
- Possibility of hazardous reactions
- Conditions to avoid
- Incompatible materials

**Section 11: Toxicological Information**
- Acute toxicity
- Skin/eye irritation
- Sensitization
- Carcinogenicity
- Reproductive toxicity

**Section 12: Ecological Information**
- Ecotoxicity
- Persistence and degradability
- Bioaccumulative potential
- Mobility in soil

**Section 13: Disposal Considerations**
- Waste treatment methods
- Contaminated packaging

**Section 14: Transport Information**
- UN number
- Proper shipping name
- Transport hazard class
- Packing group

**Section 15: Regulatory Information**
- Safety, health, environmental regulations
- Chemical safety assessment

**Section 16: Other Information**
- Date of preparation
- References
- Abbreviations

---

## 4. Efficacy Tracking

### 4.1 Clinical Trial Design

#### 4.1.1 Study Types

**In Vitro Studies**:
```typescript
interface InVitroStudy {
  study_type: 'cell_culture' | 'enzyme_assay' | 'receptor_binding' | 'antioxidant';
  cell_line?: string;
  methodology: string;
  endpoints: string[];
  concentration_range: { min: number; max: number; unit: string };
  results: {
    ec50?: number;  // Half maximal effective concentration
    ic50?: number;  // Half maximal inhibitory concentration
    statistical_significance: boolean;
    p_value?: number;
  };
}
```

**Ex Vivo Studies**:
```typescript
interface ExVivoStudy {
  study_type: 'skin_explant' | 'skin_penetration' | 'skin_absorption';
  tissue_source: string;
  sample_size: number;
  methodology: string;
  endpoints: string[];
  results: {
    penetration_depth?: number;
    absorption_percentage?: number;
    barrier_function?: string;
  };
}
```

**In Vivo Studies (Human)**:
```typescript
interface InVivoStudy {
  study_type: 'clinical_trial' | 'consumer_use_test' | 'instrumental_assessment';
  study_design: 'open_label' | 'single_blind' | 'double_blind' | 'split_face';
  randomized: boolean;
  placebo_controlled: boolean;
  sample_size: number;
  duration: { value: number; unit: 'days' | 'weeks' | 'months' };
  inclusion_criteria: string[];
  exclusion_criteria: string[];
  demographics: {
    age_range: { min: number; max: number };
    gender_distribution?: { male: number; female: number };
    skin_type?: string[];
    ethnicity?: string[];
  };
  endpoints: ClinicalEndpoint[];
  adverse_events?: AdverseEvent[];
}
```

#### 4.1.2 Instrumental Measurements

**Skin Hydration**:
```
Corneometer (Capacitance Method)
- Measures: Stratum corneum hydration
- Units: Arbitrary units (AU), typically 0-120
- Interpretation:
  - < 30 AU: Very dry
  - 30-40 AU: Dry
  - 40-50 AU: Moderately hydrated
  - > 50 AU: Well hydrated
```

**Transepidermal Water Loss (TEWL)**:
```
Tewameter
- Measures: Water evaporation rate
- Units: g/h/m²
- Interpretation:
  - < 10: Excellent barrier function
  - 10-15: Normal
  - 15-25: Impaired barrier
  - > 25: Severely impaired barrier
```

**Elasticity**:
```
Cutometer (Suction Method)
- Measures: Skin elasticity and firmness
- Parameters:
  - Uf: Final deformation
  - Ur: Immediate retraction
  - R2: Gross elasticity = Ua/Uf
  - R5: Net elasticity = Ur/Ue
  - R7: Bio-elasticity = Ur/Uf
```

**Roughness**:
```
Profilometry
- Measures: Surface topography
- Parameters:
  - Ra: Average roughness
  - Rz: Maximum peak-to-valley height
  - Rmax: Maximum roughness depth
```

**Sebum Level**:
```
Sebumeter
- Measures: Skin surface lipids
- Units: μg/cm²
- Interpretation:
  - < 50: Dry skin
  - 50-100: Normal
  - 100-150: Slightly oily
  - > 150: Oily skin
```

**Melanin and Erythema**:
```
Mexameter
- Measures: Melanin index and erythema index
- Uses: Tristimulus colorimetry
- Applications: Whitening efficacy, irritation assessment
```

**Wrinkle Analysis**:
```
PRIMOS (Phase-shift Rapid In-vivo Measurement Of Skin)
- 3D imaging of skin surface
- Parameters:
  - Wrinkle volume
  - Wrinkle depth
  - Surface area
  - Roughness parameters
```

#### 4.1.3 Subjective Assessment

**Grading Scales**:
```
Visual Analogue Scale (VAS)
- 0-10 scale or 0-100 mm line
- Subjects mark their perception
- Examples: Hydration feeling, smoothness, radiance

Likert Scale
- Categorical responses
- Example: 1=Strongly Disagree, 2=Disagree, 3=Neutral, 4=Agree, 5=Strongly Agree

Before-After Photography
- Standardized lighting and positioning
- Blinded expert grading
- Consumer perception surveys
```

**Self-Assessment Questionnaires**:
```typescript
interface SelfAssessment {
  parameter: string;
  question: string;
  scale_type: 'VAS' | 'Likert' | 'Binary';
  timepoint: 'baseline' | 'week_2' | 'week_4' | 'week_8' | 'week_12';
  response: number | string;
}
```

### 4.2 Claims Substantiation

#### 4.2.1 Claim Categories

**Efficacy Claims**:
1. **Moisturizing**: Requires corneometer or TEWL data
2. **Anti-aging**: Requires wrinkle measurement, elasticity data
3. **Whitening/Brightening**: Requires melanin index measurement
4. **Firming**: Requires cutometer data
5. **Smoothing**: Requires roughness measurement
6. **Pore-minimizing**: Requires visual assessment or imaging
7. **Anti-acne**: Requires lesion count, comedone count
8. **Soothing**: Requires erythema measurement, subjective assessment

**Time-Related Claims**:
```
- "Instant": Within seconds to minutes
- "Immediate": Within 1 hour
- "After 2 weeks": Minimum 14 days of use
- "After 4 weeks": Minimum 28 days of use
- "Long-lasting": At least 8 hours duration
- "All day": 12+ hours duration
```

**Percentage Claims**:
```
Example: "90% of users saw improved hydration"

Requirements:
- Minimum sample size: 30-50 subjects
- Statistical analysis
- Clear definition of "improved"
- Timeframe specified
```

#### 4.2.2 Evidence Requirements

**Level A (Strong Evidence)**:
- Randomized, double-blind, placebo-controlled clinical trial
- Instrumental measurements
- Statistical significance (p < 0.05)
- Published in peer-reviewed journal (optional but preferred)

**Level B (Moderate Evidence)**:
- Clinical trial with instrumental measurements
- May be open-label or single-blind
- Statistical significance demonstrated

**Level C (Consumer Perception)**:
- Self-assessment questionnaires
- Consumer use tests
- Acceptable for subjective claims (e.g., "feels smoother")

**Level D (In Vitro)**:
- Cell culture or biochemical assays
- Supports mechanism of action
- Cannot be sole evidence for efficacy claims

### 4.3 Statistical Analysis

#### 4.3.1 Sample Size Calculation

```
n = (Zα/2 + Zβ)² × 2σ² / Δ²

Where:
- n = Required sample size per group
- Zα/2 = Z-score for desired significance level (e.g., 1.96 for α=0.05)
- Zβ = Z-score for desired power (e.g., 0.84 for 80% power)
- σ = Standard deviation
- Δ = Expected difference between groups

Example:
For detecting 10% improvement in hydration with 80% power and α=0.05:
n ≈ 30-40 subjects per group
```

#### 4.3.2 Statistical Tests

**Parametric Tests** (normal distribution):
- **Paired t-test**: Before vs. After comparison
- **Independent t-test**: Treatment vs. Placebo
- **ANOVA**: Multiple groups or timepoints
- **Repeated measures ANOVA**: Multiple timepoints, same subjects

**Non-Parametric Tests** (non-normal distribution):
- **Wilcoxon signed-rank test**: Paired comparisons
- **Mann-Whitney U test**: Independent groups
- **Kruskal-Wallis test**: Multiple independent groups
- **Friedman test**: Multiple dependent groups

**Correlation Analysis**:
- **Pearson correlation**: Linear relationship (parametric)
- **Spearman correlation**: Monotonic relationship (non-parametric)

#### 4.3.3 Reporting Standards

```typescript
interface StudyResults {
  primary_endpoint: {
    parameter: string;
    baseline_mean: number;
    baseline_sd: number;
    endpoint_mean: number;
    endpoint_sd: number;
    change_from_baseline: {
      mean: number;
      sd: number;
      percent_change: number;
    };
    statistical_test: string;
    p_value: number;
    confidence_interval_95: { lower: number; upper: number };
  };
  secondary_endpoints?: Array<{...}>;
  responder_analysis?: {
    definition: string;
    responder_rate: number;
    non_responder_rate: number;
  };
}
```

---

## 5. Regulatory Compliance

### 5.1 United States (FDA)

#### 5.1.1 FD&C Act Definitions

**Cosmetic Definition** (FD&C Act, Section 201(i)):
```
Articles intended to be:
- Rubbed, poured, sprinkled, or sprayed on
- Introduced into, or otherwise applied to
- The human body or any part thereof
For:
- Cleansing, beautifying, promoting attractiveness
- Altering appearance
```

**Drug Definition** (FD&C Act, Section 201(g)):
```
Articles intended for:
- Diagnosis, cure, mitigation, treatment, or prevention of disease
- Affecting structure or function of the body

Examples in cosmetics:
- Sunscreens (OTC drugs)
- Anti-dandruff shampoos (OTC drugs)
- Acne treatments (OTC drugs)
- Antiperspirants (OTC drugs)
```

**Cosmetic vs. Drug**:
```
Cosmetic: "Moisturizes dry skin"
Drug: "Treats eczema"

Cosmetic: "Reduces the appearance of fine lines"
Drug: "Reduces wrinkles"

Product can be both: "Moisturizing sunscreen"
```

#### 5.1.2 FDA Requirements

**Pre-Market Approval**:
```
NOT required for cosmetics
EXCEPT:
- Color additives (pre-market approval)
- New OTC drug ingredients
```

**Safety Substantiation**:
```
Manufacturer's responsibility:
- Ensure safety before marketing
- Maintain supporting data
- No specific format required
- FDA can request data if safety questioned
```

**Labeling Requirements** (Fair Packaging and Labeling Act):
```
Principal Display Panel (PDP):
1. Identity statement ("Shampoo", "Moisturizer")
2. Net contents (by weight, measure, or count)

Information Panel:
1. Ingredient declaration (descending order of predominance)
   - Exception: ≤1% ingredients may be listed in any order
   - Exception: Color additives may be listed after other ingredients
2. Manufacturer/distributor name and address
3. Warning and caution statements (if applicable)
4. Directions for safe use (if needed)

Special Rules:
- Fragrances: May list as "Fragrance" or "Parfum"
- Trade secrets: Can petition FDA for exemption
- "And other ingredients": Not allowed
```

**Color Additives** (21 CFR Parts 73, 74, 82):
```
Categories:
1. FD&C colors: Foods, drugs, cosmetics
2. D&C colors: Drugs and cosmetics only
3. External D&C colors: External use only
4. Lakes: Insoluble forms of certified colors

Certification:
- Batch certification required for synthetic colors
- Certification-exempt: Iron oxides, titanium dioxide, zinc oxide, etc.

Restrictions:
- Specific usage levels
- Allowed applications (e.g., "not for use in eye area")
```

#### 5.1.3 Voluntary Programs

**Voluntary Cosmetic Registration Program (VCRP)**:
```
- Manufacturer registration
- Product ingredient statement
- Discontinued (as of 2022, replaced by MoCRA)
```

**Modernization of Cosmetics Regulation Act (MoCRA) - 2022**:
```
New Requirements:
1. Facility registration (mandatory)
2. Product listing (mandatory)
3. Adverse event reporting (mandatory, serious events)
4. Safety substantiation records (mandatory)
5. Fragrance allergen disclosure (top 26 allergens if > threshold)
6. Good Manufacturing Practices (to be established)
7. Recall authority for FDA

Implementation: Phased rollout 2023-2025
```

### 5.2 European Union

#### 5.2.1 Regulation (EC) No 1223/2009

**Key Requirements**:
```
1. Cosmetic Product Safety Report (CPSR) - MANDATORY
2. Product Information File (PIF) - MANDATORY
3. Notification via CPNP - MANDATORY (before marketing)
4. Responsible Person (RP) - Must be established in EU
5. Good Manufacturing Practice (ISO 22716)
6. Labeling requirements
7. Claims substantiation
```

#### 5.2.2 Cosmetic Product Safety Report (CPSR)

**Part A: Cosmetic Product Safety Information**:
```
Section A.1: Quantitative and qualitative composition
Section A.2: Physical/chemical characteristics and stability
Section A.3: Microbiological quality
Section A.4: Impurities, traces, information about packaging material
Section A.5: Normal and reasonably foreseeable use
Section A.6: Exposure to the cosmetic product
Section A.7: Exposure to the substances
Section A.8: Toxicological profile of the substances
Section A.9: Undesirable effects and serious undesirable effects
Section A.10: Information on the cosmetic product
```

**Part B: Cosmetic Product Safety Assessment**:
```
Section B.1: Assessment conclusion
Section B.2: Labelled warnings and instructions of use
Section B.3: Reasoning
Section B.4: Assessor's credentials and approval
```

**Safety Assessor Qualifications**:
```
Required:
- Diploma in pharmacy, toxicology, medicine, or similar
- AND practical experience in cosmetic product safety
```

#### 5.2.3 Annexes to Regulation

**Annex II**: Prohibited Substances (> 1,600 entries)
```
Examples:
- Formaldehyde (prohibited in aerosols)
- Mercury and its compounds
- Lead and its compounds
- Aristolochic acid
- Asbestos
```

**Annex III**: Restricted Substances (> 250 entries)
```
Format: Substance | Restriction | Conditions | Warnings

Examples:
- Benzoyl peroxide: Max 5% leave-on, 2.5% rinse-off
- Salicylic acid: Max 3% leave-on, 2% rinse-off
- Hydroquinone: Max 2% (hair dyes only)
- Retinol/retinyl: Max 0.3% (face), 0.05% (body, hands)
```

**Annex IV**: Allowed Colorants
```
- Lists permitted color additives
- CI (Color Index) numbers
- Specific restrictions (e.g., "not for use near eyes")
```

**Annex V**: Allowed Preservatives
```
- Maximum concentrations
- Product type restrictions
- pH restrictions
- Mandatory warnings
```

**Annex VI**: Allowed UV Filters
```
- Maximum concentrations
- Combinations allowed
- Warnings required
```

#### 5.2.4 CPNP (Cosmetic Products Notification Portal)

**Information Required**:
```
1. Category and name of cosmetic product
2. Name and address of Responsible Person
3. Country of origin (if imported)
4. Member State(s) of marketing
5. Label and packaging (if appropriate)
6. Original formulation (presence and ranges of substances)
7. Nanomaterials (if any)
8. Name and CAS/EC number of CMR substances (Category 1A/1B)
9. Frame formulation (simplified for product lines)
```

**Notification Timing**:
```
- BEFORE placing on EU market
- Updates required for formula changes
```

### 5.3 South Korea (K-Beauty)

#### 5.3.1 Cosmetics Act (MFDS)

**Product Categories**:
```
General Cosmetics:
- All cosmetic products not classified as functional

Functional Cosmetics (Special approval required):
1. Skin whitening
2. Wrinkle improvement
3. Sun protection (SPF or PA claims)
4. Hair loss prevention/relief
5. Dyeing hair

Quasi-Drugs:
- Medicated cosmetics
- More stringent requirements
```

#### 5.3.2 Registration Requirements

**Import Registration**:
```
1. Company registration (importer)
2. Product notification or approval:
   - General: Notification
   - Functional: Approval (requires safety/efficacy data)
3. Manufacturing facility registration (for manufacturer)
```

**Functional Cosmetics - Approval Process**:
```
Required Documents:
1. Product specification
2. Manufacturing method
3. Safety data
4. Efficacy data (clinical trials or in vitro studies)
5. Stability data
6. Raw material specifications

Review Time: 3-6 months
```

#### 5.3.3 Labeling Requirements

**Mandatory Label Information**:
```
1. Product name
2. Company name and address
3. Manufacturing date or expiry date
4. All ingredients (Korean)
5. Contents
6. Functional cosmetic indication (if applicable)
7. Usage precautions
8. Country of manufacture
9. "Functional Cosmetic" label (if applicable)
```

**Ingredient Disclosure**:
```
- Full ingredient list required
- Descending order by weight
- Korean language mandatory
- INCI names with Korean translation
- Fragrance components > 0.01% must be disclosed
```

### 5.4 Japan (J-Beauty)

#### 5.4.1 Pharmaceutical and Medical Device Act (PMDA)

**Product Classification**:
```
Cosmetics (化粧品):
- General beauty products
- Notification system

Quasi-Drugs (医薬部外品):
- Medicated cosmetics
- Approval required
- Examples: Whitening, acne treatment, deodorant

Pharmaceuticals (医薬品):
- Therapeutic products
- Strict approval process
```

#### 5.4.2 Notification/Approval Process

**Cosmetics**:
```
- Notification to PMDA
- Manufacturer registration required
- Marketing Authorization Holder (MAH) required
- GMP compliance (ISO 22716 or equivalent)
```

**Quasi-Drugs**:
```
- Pre-market approval required
- Safety and efficacy data
- Active ingredient must be in approved list
- Manufacturing facility inspection
```

#### 5.4.3 Standards and Specifications

**Japanese Standards of Cosmetic Ingredients (JSCI)**:
```
- Official ingredient list
- Purity specifications
- Test methods
- Quality standards
```

**Negative List**:
```
Prohibited or Restricted Substances:
- Similar to EU Annexes II and III
- Some differences (e.g., whitening ingredients)
```

### 5.5 China

#### 5.5.1 NMPA Regulations

**Classification System**:
```
General Cosmetics (普通化妆品):
- Notification system (since 2021)

Special Cosmetics (特殊用途化妆品):
- Hair dyeing
- Perming
- Sunscreen
- Whitening/spot-fading
- Anti-hair loss
- New materials/functions
→ Requires registration and approval
```

**Import vs. Domestic**:
```
Imported (进口):
- Overseas manufacturer registration
- Product registration/notification
- Importer registration
- More stringent requirements

Domestic (国产):
- Simpler process
- Still requires NMPA compliance
```

#### 5.5.2 Animal Testing Requirements

**Current Status (as of 2021)**:
```
General Cosmetics (imported via cross-border e-commerce):
- Animal testing waived

General Cosmetics (imported via traditional channels):
- Animal testing may be required (transitioning)

Special Cosmetics:
- Animal testing still required

Domestic products:
- Reduced animal testing requirements
```

#### 5.5.3 IECIC (Inventory)

**Ingredient Registration**:
```
IECIC = Inventory of Existing Cosmetic Ingredients in China

- Ingredients not on IECIC require registration
- New ingredient dossier required
- Safety assessment
- Efficacy data (if applicable)
- Long approval process (1-2 years)
```

### 5.6 Global Harmonization

#### 5.6.1 ISO Standards

**ISO 22716:2007**: Cosmetics - GMP
```
Covers:
- Personnel
- Premises and equipment
- Raw materials and packaging materials
- Production
- Quality control laboratory
- Storage and distribution
- Waste management
- Subcontracting
- Deviations
- Complaints and recalls
```

**ISO 22718**: Cosmetics - Microbiology - General requirements for microbiological testing

**ISO 22717**: Cosmetics - Microbiology - Detection of Pseudomonas aeruginosa

**ISO 24442**: Cosmetics - Sun protection test methods - UVA photoprotection in vivo

#### 5.6.2 ICH Guidelines (for drugs)

Relevant for cosmetic-drug overlap:
- ICH Q1A: Stability testing
- ICH Q3C: Impurities - Residual solvents
- ICH M7: Assessment of DNA reactive (mutagenic) impurities

---

## 6. Supply Chain Transparency

### 6.1 Ingredient Sourcing

#### 6.1.1 Raw Material Traceability

```typescript
interface IngredientSource {
  supplier: {
    name: string;
    country: string;
    certifications: string[];  // GMP, ISO, Organic, Fair Trade
    audit_date?: string;
    audit_score?: number;
  };
  origin: {
    country_of_origin: string;
    region?: string;
    farm_or_facility?: string;
    gps_coordinates?: { latitude: number; longitude: number };
  };
  harvest_or_production: {
    date: string;
    batch_number: string;
    method: 'wild-crafted' | 'organic' | 'conventional' | 'synthetic';
    certifications?: string[];
  };
  transportation: {
    route: string[];
    shipping_method: 'air' | 'sea' | 'land';
    temperature_controlled: boolean;
    arrival_date: string;
  };
  quality_verification: {
    coa_number: string;  // Certificate of Analysis
    testing_lab: string;
    test_results: Record<string, any>;
    release_date: string;
  };
}
```

#### 6.1.2 Ethical Sourcing

**Fair Trade Principles**:
```
1. Fair pricing: Producers receive fair compensation
2. Fair labor: No child labor, safe working conditions
3. Direct trade: Minimize intermediaries
4. Community development: Premiums for community projects
5. Environmental sustainability: Eco-friendly practices
```

**Certifications**:
```
- Fair Trade Certified™
- Rainforest Alliance
- UTZ Certified
- B Corporation
- Leaping Bunny (cruelty-free)
- RSPO (Roundtable on Sustainable Palm Oil)
- FSC (Forest Stewardship Council)
```

#### 6.1.3 Conflict Minerals

**Regulated Materials** (Dodd-Frank Act, Section 1502):
```
3TG Minerals:
- Tin (Cassiterite)
- Tantalum (Coltan)
- Tungsten (Wolframite)
- Gold

Regions of Concern:
- Democratic Republic of Congo (DRC)
- Adjoining countries

Requirements:
- Due diligence in supply chain
- Conflict-free sourcing
- Reporting (for US-listed companies)
```

**Mica Mining**:
```
Issue: Child labor in mica mining (India, Madagascar)

Responsible Mica Initiative (RMI):
- Supply chain mapping
- Mine certification
- Child labor elimination
- Community development

Alternative: Synthetic mica (Fluorphlogopite)
```

### 6.2 Manufacturing Transparency

#### 6.2.1 Contract Manufacturing

```typescript
interface Manufacturer {
  company_name: string;
  location: {
    facility_name: string;
    address: string;
    country: string;
  };
  certifications: {
    gmp: { standard: 'ISO 22716' | 'FDA GMP'; valid_until: string };
    quality: string[];  // ISO 9001, etc.
    environmental: string[];  // ISO 14001, etc.
    social: string[];  // SA8000, etc.
  };
  capabilities: {
    product_types: string[];
    max_capacity: { value: number; unit: 'units/month' | 'kg/month' };
    moq: { value: number; unit: string };  // Minimum Order Quantity
  };
  quality_systems: {
    batch_record_system: string;
    traceability: boolean;
    recall_procedures: boolean;
    deviation_management: boolean;
  };
  audit_history: Array<{
    date: string;
    auditor: string;
    type: 'internal' | 'second_party' | 'third_party';
    score?: number;
    findings: Array<{
      severity: 'critical' | 'major' | 'minor' | 'observation';
      description: string;
      capa_status: 'open' | 'closed';  // Corrective and Preventive Action
    }>;
  }>;
}
```

#### 6.2.2 Batch Records

```typescript
interface BatchRecord {
  batch_number: string;
  product_id: string;
  product_name: string;
  manufacturing_date: string;
  expiry_date: string;
  batch_size: { value: number; unit: string };
  formula_version: string;

  raw_materials: Array<{
    ingredient_name: string;
    supplier: string;
    lot_number: string;
    quantity_used: { value: number; unit: string };
    receipt_date: string;
    coa_verified: boolean;
  }>;

  packaging_materials: Array<{
    component: string;
    supplier: string;
    lot_number: string;
    quantity_used: number;
  }>;

  manufacturing_steps: Array<{
    step_number: number;
    description: string;
    equipment_id: string;
    operator_id: string;
    start_time: string;
    end_time: string;
    parameters: Record<string, any>;  // Temperature, speed, time, etc.
    in_process_checks: Array<{
      parameter: string;
      specification: string;
      actual_value: string;
      pass: boolean;
    }>;
  }>;

  quality_control: {
    tests_performed: Array<{
      test_name: string;
      method: string;
      specification: string;
      result: string;
      pass: boolean;
      tested_by: string;
      test_date: string;
    }>;
    release_status: 'released' | 'rejected' | 'on_hold' | 'quarantine';
    release_date?: string;
    released_by?: string;
  };

  deviations?: Array<{
    deviation_number: string;
    description: string;
    investigation: string;
    impact_assessment: string;
    capa: string;
    status: 'open' | 'closed';
  }>;
}
```

### 6.3 Distribution Tracking

#### 6.3.1 Serialization

```typescript
interface ProductSerialization {
  gtin: string;  // Global Trade Item Number (barcode)
  serial_number: string;  // Unique per unit
  batch_number: string;
  manufacturing_date: string;
  expiry_date: string;

  supply_chain_events: Array<{
    event_type: 'manufacturing' | 'packaging' | 'warehousing' | 'shipment' | 'received' | 'sold';
    timestamp: string;
    location: {
      facility_name: string;
      address: string;
      gps: { latitude: number; longitude: number };
    };
    actor: string;  // Company/person responsible
    next_destination?: string;
  }>;

  blockchain_hash?: string;  // For blockchain-based tracking
}
```

#### 6.3.2 Recall Management

```typescript
interface RecallEvent {
  recall_number: string;
  recall_date: string;
  recall_type: 'voluntary' | 'mandatory';
  recall_level: 'class_I' | 'class_II' | 'class_III';  // FDA classification

  product_details: {
    product_name: string;
    product_id: string;
    affected_batches: string[];
    affected_units: number;
  };

  reason: {
    category: 'safety' | 'quality' | 'labeling' | 'contamination';
    description: string;
    risk_assessment: string;
  };

  distribution: {
    countries: string[];
    distribution_channels: string[];
    units_distributed: number;
  };

  recall_actions: {
    consumer_notification: boolean;
    notification_method: string[];  // Email, website, press release, etc.
    retailer_notification: boolean;
    return_instructions: string;
    refund_policy: string;
  };

  effectiveness_check: {
    units_recalled: number;
    recall_effectiveness: number;  // Percentage
    completion_date?: string;
  };
}
```

---

## 7. Product Formulation

### 7.1 Formula Design

#### 7.1.1 Basic Formula Structure

**Emulsion Types**:
```
Oil-in-Water (O/W):
- Continuous phase: Water
- Dispersed phase: Oil
- Properties: Light, non-greasy
- Examples: Lotions, light creams

Water-in-Oil (W/O):
- Continuous phase: Oil
- Dispersed phase: Water
- Properties: Rich, occlusive
- Examples: Cold cream, barrier creams

Multiple Emulsions:
- W/O/W or O/W/O
- Complex structure for controlled release
```

**Anhydrous Products**:
```
- No water phase
- Examples: Lip balms, body butters, hair pomades
- Advantages: No preservative needed, stable
- Challenges: Can feel heavy, limited actives
```

**Surfactant-Based Systems**:
```
- Micellar solutions
- Shampoos, body washes
- Cleansing waters
- Rely on surfactant micelles
```

#### 7.1.2 Formula Template (O/W Cream)

```
Phase A (Water Phase):
1. Distilled Water: 60-70%
2. Humectants (Glycerin, Propylene Glycol): 3-10%
3. Water-soluble actives: as specified
4. Chelating agent (EDTA): 0.05-0.1%

Phase B (Oil Phase):
1. Emulsifier: 2-8%
2. Oils/Esters: 10-20%
3. Waxes/Butters: 1-5%
4. Oil-soluble actives: as specified

Phase C (Cool Down):
1. Preservative: as required
2. pH adjuster: as needed
3. Fragrance: 0.1-1%
4. Heat-sensitive actives: as specified

Manufacturing Process:
1. Heat Phase A to 75°C
2. Heat Phase B to 75°C
3. Add Phase B to Phase A with homogenization
4. Cool to 40°C
5. Add Phase C ingredients
6. Cool to room temperature
7. pH adjustment
8. Final QC checks
```

### 7.2 Ingredient Compatibility

#### 7.2.1 Incompatibility Matrix

**pH-Dependent Incompatibilities**:
```
Retinol:
- Stable: pH 5.5-6.5
- Incompatible: Niacinamide (pH conflict - debated), Strong acids

Vitamin C (L-Ascorbic Acid):
- Stable: pH < 3.5
- Incompatible: Niacinamide (immediate use okay), Peptides (pH conflict)

Niacinamide:
- Stable: pH 5-7
- Incompatible: Strong acids (converts to niacin)

AHAs/BHAs:
- Effective: pH 3-4
- Incompatible: Retinoids (irritation), Peptides (hydrolysis)

Peptides:
- Stable: pH 4-7
- Incompatible: Strong acids/bases, enzymes
```

**Chemical Incompatibilities**:
```
Ascorbic Acid + Metal ions (Cu²⁺, Fe³⁺) → Oxidation
Phenolic compounds + Oxidizers → Polymerization
Thiols + Disulfides → Thiol-disulfide exchange
Amine + Aldehyde → Schiff base formation
Benzoyl peroxide + Retinoids → Degradation
```

**Physical Incompatibilities**:
```
Cationic surfactants + Anionic surfactants → Precipitation
Electrolytes + Non-ionic emulsifiers → Salting out
Proteins + High salt → Precipitation
Polymers + Incompatible solvents → Phase separation
```

### 7.3 Stability Testing

#### 7.3.1 ICH Guidelines Adaptation

**Storage Conditions**:
```
Long-term: 25°C ± 2°C / 60% RH ± 5% RH
- Duration: 12 months minimum, 24-36 months ideal
- Testing: 0, 3, 6, 9, 12 months

Intermediate: 30°C ± 2°C / 65% RH ± 5% RH
- Duration: 6 months
- Testing: 0, 3, 6 months

Accelerated: 40°C ± 2°C / 75% RH ± 5% RH
- Duration: 6 months
- Testing: 0, 1, 2, 3, 6 months
```

**Additional Conditions**:
```
Freeze-Thaw Cycling:
- -10°C to +40°C
- 24-hour cycles
- Minimum 5 cycles

Low Temperature: 4°C ± 2°C
- Assess cold storage effects
- 3 months minimum

Light Exposure:
- Photostability chamber
- ICH Q1B guidance
- Overall illumination: 1.2 million lux hours
- UV energy: 200 Wh/m²
```

#### 7.3.2 Stability Parameters

**Physical Stability**:
```
- Appearance (color, clarity, texture)
- Odor
- pH
- Viscosity
- Specific gravity
- Phase separation
- Syneresis (liquid separation)
- Crystallization
- Sedimentation
```

**Chemical Stability**:
```
- Active ingredient assay (90-110% of label claim)
- Degradation products
- Preservative efficacy
- Antioxidant levels
- Fragrance stability
- Color stability (colorants)
```

**Microbiological Stability**:
```
- Total aerobic count (< 100 CFU/g)
- Yeast and mold (< 10 CFU/g)
- Specific pathogens (absent):
  - E. coli
  - S. aureus
  - P. aeruginosa
  - C. albicans
- Preservative efficacy testing (challenge test)
```

**Packaging Compatibility**:
```
- Container integrity
- Closure seal
- Leakage
- Product-package interaction
- Pump/dispenser functionality
```

#### 7.3.3 Shelf Life Determination

**Arrhenius Equation** (Chemical kinetics):
```
k = A × e^(-Ea / RT)

Where:
- k = Rate constant
- A = Pre-exponential factor
- Ea = Activation energy
- R = Gas constant (8.314 J/mol·K)
- T = Absolute temperature (K)

Shelf Life Prediction:
t₉₀ (at 25°C) = t₉₀ (at 40°C) × 2^((40-25)/10)

(Assumes Q₁₀ = 2, meaning reaction rate doubles per 10°C increase)
```

**Acceptance Criteria**:
```
Chemical:
- Active ingredient: 90-110% of label claim
- Impurities/degradation products: < specified limits

Physical:
- No significant change in appearance, odor, texture
- pH within ± 0.5 units
- Viscosity within ± 10%

Microbiological:
- Meets limits throughout shelf life
- Preservative efficacy maintained
```

---

## 8. Quality Control

### 8.1 Raw Material Testing

#### 8.1.1 Certificate of Analysis (COA)

**Required Information**:
```typescript
interface CertificateOfAnalysis {
  supplier: string;
  material_name: string;
  inci_name: string;
  lot_number: string;
  manufacturing_date: string;
  expiry_date: string;

  specifications: Array<{
    parameter: string;
    method: string;
    specification: string;
    result: string;
    pass: boolean;
  }>;

  typical_tests: {
    identity: { method: string; result: string };
    purity: { method: string; result: string };
    assay?: { method: string; result: string };
    heavy_metals: Record<string, string>;
    microbiology: {
      total_count: string;
      yeast_mold: string;
      pathogens: string;
    };
    physical: {
      appearance: string;
      odor: string;
      color: string;
    };
  };

  certification: {
    certified_by: string;
    position: string;
    date: string;
    signature: string;
  };
}
```

#### 8.1.2 Identity Testing

**Spectroscopic Methods**:
```
FTIR (Fourier Transform Infrared Spectroscopy):
- Identifies functional groups
- Compares to reference spectrum
- Pass: Correlation > 0.98

HPLC (High Performance Liquid Chromatography):
- Retention time matching
- UV spectrum comparison
- Quantification

NMR (Nuclear Magnetic Resonance):
- Structural confirmation
- High specificity
- Gold standard for complex molecules

MS (Mass Spectrometry):
- Molecular weight confirmation
- Fragmentation pattern
- High sensitivity
```

**Classical Methods**:
```
Melting Point:
- Range specified (e.g., 62-65°C)
- Indicates purity

Refractive Index:
- Specific for liquids
- Temperature-dependent

Specific Gravity:
- Density measurement
- Quick identity check

Color Reaction Tests:
- Chemical color tests
- Qualitative identification
```

### 8.2 In-Process Controls

#### 8.2.1 Critical Control Points

**Mixing Phase**:
```
Parameters to monitor:
- Temperature (± 2°C)
- Mixing speed (± 10 RPM)
- Mixing time (± 5 min)
- pH (± 0.2 units)
- Appearance (visual check)
```

**Homogenization**:
```
Parameters:
- Homogenization pressure (± 5 bar)
- Number of passes (specified)
- Temperature (± 2°C)
- Particle size (< specified μm)
```

**Filling**:
```
Parameters:
- Fill weight (target ± 3%)
- Fill volume (for liquids)
- Torque (for screw caps)
- Seal integrity
```

### 8.3 Finished Product Testing

#### 8.3.1 Physico-Chemical Tests

**pH Measurement**:
```
Method: pH meter with glass electrode
Temperature: 25°C ± 2°C
Sample: 10% dispersion in water (for emulsions)
Specification: Typically ± 0.5 units from target
```

**Viscosity**:
```
Instruments:
- Brookfield viscometer (most common)
- Cone-and-plate rheometer (more precise)

Conditions:
- Spindle: Specified
- Speed: Specified (e.g., 12 RPM)
- Temperature: 25°C ± 1°C
- Shear rate: Specified

Result: cP or mPa·s
Specification: Typically ± 10-20% from target
```

**Specific Gravity**:
```
Method: Pycnometer or densitometer
Temperature: 25°C
Calculation: SG = (Weight of product) / (Weight of equal volume of water)
Specification: ± 0.02 units
```

**Particle Size** (for emulsions, suspensions):
```
Methods:
- Laser diffraction (Malvern)
- Microscopy
- Coulter counter

Parameters:
- D50 (median diameter)
- D90 (90% below this size)
- Polydispersity index (PDI)

Specification: D90 < 10 μm (typical for emulsions)
```

#### 8.3.2 Microbiological Tests

**Total Aerobic Microbial Count (TAMC)**:
```
Method: Plate count (ISO 21149)
Medium: Tryptic Soy Agar (TSA)
Incubation: 30-35°C for 48-72 hours
Limits:
- Eye area products: < 100 CFU/g
- Other products: < 1000 CFU/g
- Baby products: < 100 CFU/g
```

**Total Yeast and Mold Count (TYMC)**:
```
Method: Plate count (ISO 16212)
Medium: Sabouraud Dextrose Agar (SDA) with antibiotics
Incubation: 20-25°C for 5-7 days
Limits:
- All products: < 100 CFU/g
- Eye area: < 10 CFU/g
```

**Specific Pathogens**:
```
Tests (all must be ABSENT in 1g or 1mL):
1. Staphylococcus aureus (ISO 22718)
2. Pseudomonas aeruginosa (ISO 22717)
3. Escherichia coli (ISO 21150)
4. Candida albicans (ISO 18416)

Optional (depending on product type):
5. Salmonella spp.
6. Clostridium spp. (for anaerobic conditions)
```

#### 8.3.3 Preservative Efficacy Testing (PET)

**Challenge Test** (USP <51>, ISO 11930):
```
Inoculum: ~10⁶ CFU/mL or CFU/g

Organisms:
- Pseudomonas aeruginosa (ATCC 9027)
- Staphylococcus aureus (ATCC 6538)
- Escherichia coli (ATCC 8739)
- Candida albicans (ATCC 10231)
- Aspergillus brasiliensis (ATCC 16404)

Time Points: 0, 7, 14, 21, 28 days

Acceptance Criteria A (strictest):
- Bacteria: ≥2 log reduction by day 7, ≥3 log by day 14, no increase to day 28
- Yeast/Mold: No increase from initial count, ≥2 log reduction by day 14, no increase to day 28

Acceptance Criteria B (moderate):
- Bacteria: ≥2 log reduction by day 14, no increase to day 28
- Yeast/Mold: No increase from initial count to day 28
```

---

## 9. Consumer Safety

### 9.1 Allergen Management

#### 9.1.1 Common Allergens

**EU 26 Fragrance Allergens** (must be declared if > threshold):
```
1. Amyl Cinnamal
2. Amylcinnamyl Alcohol
3. Anisyl Alcohol
4. Benzyl Alcohol
5. Benzyl Benzoate
6. Benzyl Cinnamate
7. Benzyl Salicylate
8. Cinnamal
9. Cinnamyl Alcohol
10. Citral
11. Citronellol
12. Coumarin
13. Eugenol
14. Farnesol
15. Geraniol
16. Hexyl Cinnamal
17. Hydroxycitronellal
18. Hydroxyisohexyl 3-Cyclohexene Carboxaldehyde (HICC/Lyral)
19. Isoeugenol
20. Limonene
21. Linalool
22. Methyl 2-Octynoate
23. Alpha-Isomethyl Ionone
24. Oakmoss Extract (Evernia Prunastri)
25. Treemoss Extract (Evernia Furfuracea)
26. Butylphenyl Methylpropional (Lilial) - BANNED in EU as of 2022

Thresholds:
- Leave-on: > 0.001% (10 ppm)
- Rinse-off: > 0.01% (100 ppm)
```

**Preservative Allergens**:
```
- Formaldehyde and releasers (e.g., Diazolidinyl Urea)
- Methylisothiazolinone (MI)
- Methylchloroisothiazolinone (MCI)
- Parabens (rare sensitization)
- Iodopropynyl Butylcarbamate (IPBC)
```

**Other Common Allergens**:
```
- Propolis
- Lanolin
- Colophonium (Rosin)
- Propylene Glycol (rare, mainly irritant)
- Cocamidopropyl Betaine
- Balsam of Peru (Myroxylon Pereirae)
```

#### 9.1.2 Patch Testing

**Procedure**:
```
1. Application: Allergen applied to skin (usually upper back)
2. Occlusion: Covered with patch
3. Wearing time: 48 hours
4. First reading: 48 hours
5. Second reading: 72-96 hours

Scoring (ICDRG scale):
- (-): Negative reaction
- (?+): Doubtful reaction
- (+): Weak positive (erythema, infiltration, papules)
- (++): Strong positive (erythema, infiltration, papules, vesicles)
- (+++): Extreme positive (intense erythema, infiltration, coalescing vesicles)
- IR: Irritant reaction
```

### 9.2 Adverse Event Reporting

#### 9.2.1 Classification

**Severity**:
```
Mild:
- Temporary discomfort
- No medical intervention required
- Examples: Mild stinging, temporary redness

Moderate:
- May require medical attention
- Causes discomfort or inconvenience
- Examples: Persistent irritation, mild allergic reaction

Severe:
- Requires immediate medical intervention
- Life-threatening or causes permanent damage
- Examples: Anaphylaxis, severe chemical burn, Stevens-Johnson syndrome
```

**Causality Assessment**:
```
Certain: Clear temporal relationship, no other causes, positive rechallenge
Probable/Likely: Temporal relationship, unlikely other causes, no rechallenge
Possible: Temporal relationship, other causes possible
Unlikely: Poor temporal relationship, other causes likely
Unrelated: No temporal relationship, other causes certain
```

#### 9.2.2 Reporting Requirements

**FDA (USA - under MoCRA)**:
```
Serious Adverse Events must be reported within 15 business days:
- Death
- Life-threatening experience
- Hospitalization
- Persistent or significant disability
- Congenital anomaly
- Requires medical intervention to prevent permanent impairment

Information to include:
- Product identification
- Adverse event description
- Patient information (age, gender)
- Outcome
- Reporter information
```

**EU CPNP**:
```
Serious Undesirable Effects (SUEs) reported to national authorities:
- Temporary or permanent functional disability
- Hospitalization
- Congenital anomalies
- Vital risk

Timeline: Immediately upon knowledge
```

### 9.3 Product Warnings

#### 9.3.1 Mandatory Warnings

**Examples**:
```
Alcohol-based products (> 70% ethanol):
"Flammable. Keep away from flames and heat."

Aerosols:
"Pressurized container: may burst if heated. Keep away from heat, hot surfaces, sparks, open flames, and other ignition sources. Do not pierce or burn, even after use. Protect from sunlight. Do not expose to temperatures exceeding 50°C."

Hair dyes (oxidative):
"Hair colorants can cause severe allergic reactions. Read and follow instructions. This product is not intended for use on persons under the age of 16. Temporary black henna tattoos may increase your risk of allergy. Do not color your hair if: you have a rash on your face or sensitive, irritated and damaged scalp, you have ever experienced any reaction after coloring your hair, you have experienced a reaction to a temporary black henna tattoo in the past."

AHAs/BHAs (> certain concentrations):
"This product contains an alpha hydroxy acid (AHA) that may increase your skin's sensitivity to the sun and particularly the possibility of sunburn. Use a sunscreen, wear protective clothing, and limit sun exposure while using this product and for a week afterwards."

Retinoids:
"For external use only. Avoid contact with eyes. If you are pregnant or nursing, consult a physician before use."

Sunscreens:
"For external use only. When using this product keep out of eyes. Rinse with water to remove. Stop use and ask a doctor if rash occurs."
```

---

## 10. Sustainability

### 10.1 Environmental Impact

#### 10.1.1 Life Cycle Assessment (LCA)

**Phases**:
```
1. Raw Material Extraction
   - Agricultural impact
   - Mining impact
   - Water usage
   - Land use change

2. Manufacturing
   - Energy consumption
   - Water consumption
   - Waste generation
   - Emissions (air, water, soil)

3. Packaging
   - Material production
   - Printing/labeling
   - Transportation

4. Distribution
   - Transportation modes
   - Fuel consumption
   - Refrigeration (if needed)

5. Use Phase
   - Water usage (rinse-off products)
   - Energy (heated water)
   - Microplastic release

6. End of Life
   - Packaging disposal/recycling
   - Product disposal
   - Wastewater treatment
```

#### 10.1.2 Carbon Footprint

```
Total CO₂e = Production + Packaging + Transportation + Use + Disposal

Carbon Footprint Calculation:
CF (kg CO₂e) = Σ (Activity × Emission Factor)

Example for Manufacturing:
CF = (Electricity kWh × EF_electricity) + (Natural gas m³ × EF_gas) + ...

Typical Emission Factors:
- Electricity (grid): 0.4-0.9 kg CO₂e/kWh (varies by region)
- Natural gas: 2.0 kg CO₂e/m³
- Diesel: 2.7 kg CO₂e/L
```

### 10.2 Green Chemistry

#### 10.2.1 12 Principles of Green Chemistry

```
1. Waste Prevention: Design to minimize waste
2. Atom Economy: Maximize incorporation of reactants into product
3. Less Hazardous Synthesis: Use safer chemicals and conditions
4. Designing Safer Chemicals: Maintain efficacy while reducing toxicity
5. Safer Solvents: Use water or eliminate solvents
6. Energy Efficiency: Ambient temperature and pressure preferred
7. Renewable Feedstocks: Use renewable materials when feasible
8. Reduce Derivatives: Minimize derivatization steps
9. Catalysis: Use catalytic reagents instead of stoichiometric
10. Design for Degradation: Design products to degrade after use
11. Real-Time Pollution Prevention: Monitor to prevent pollution
12. Inherently Safer Chemistry: Choose substances to minimize accidents
```

#### 10.2.2 Biodegradability

**Testing Methods**:
```
OECD 301: Ready Biodegradability
- 301A: DOC Die-Away
- 301B: CO₂ Evolution (Sturm test)
- 301C: MITI (Modified MITI test)
- 301D: Closed Bottle
- 301E: Modified OECD Screening
- 301F: Manometric Respirometry

Pass Level: ≥60% degradation in 28 days (301B, 301F)
           ≥70% degradation in 28 days (301A, 301E)

Readily Biodegradable: Passes OECD 301 test
```

**Bioaccumulation**:
```
log Kow (Octanol-Water Partition Coefficient):
- < 3: Low bioaccumulation potential
- 3-5: Moderate bioaccumulation potential
- > 5: High bioaccumulation potential, environmental concern

BCF (Bioconcentration Factor):
- < 100: Low bioaccumulation
- 100-1000: Moderate bioaccumulation
- > 1000: High bioaccumulation
```

### 10.3 Sustainable Packaging

#### 10.3.1 Material Types

**Plastics**:
```
PET (Polyethylene Terephthalate) - #1:
- Widely recyclable
- Clear, lightweight
- Good barrier properties
- Uses: Bottles, jars

HDPE (High-Density Polyethylene) - #2:
- Recyclable
- Chemical resistant
- Opaque
- Uses: Bottles, tubes

PP (Polypropylene) - #5:
- Recyclable (limited infrastructure)
- Heat resistant
- Chemical resistant
- Uses: Jars, caps, tubes

PCR (Post-Consumer Recycled):
- Made from recycled plastics
- Reduces virgin plastic use
- May have slight color variation
```

**Glass**:
```
Advantages:
- Infinitely recyclable
- Inert (no leaching)
- Premium perception

Disadvantages:
- Heavy (higher transportation emissions)
- Fragile
- Energy-intensive production
```

**Aluminum**:
```
Advantages:
- Highly recyclable (95% less energy than virgin)
- Light-blocking
- Lightweight

Disadvantages:
- Energy-intensive production (virgin)
- Denting issues
```

**Bioplastics**:
```
PLA (Polylactic Acid):
- From corn starch, sugarcane
- Compostable (industrial)
- Not recyclable with conventional plastics

PHA (Polyhydroxyalkanoates):
- From bacterial fermentation
- Biodegradable in various environments
- Expensive

Bio-PE (Bio-Polyethylene):
- From sugarcane ethanol
- Same properties as conventional PE
- Recyclable with conventional PE
```

#### 10.3.2 Sustainable Design

**Reduce**:
```
- Lightweighting: Reduce material thickness
- Concentrate formulas: Smaller packaging
- Solid formats: No water = lighter, smaller
- Refill systems: One-time container purchase
```

**Reuse**:
```
- Refillable containers
- Return programs
- Durable, attractive packaging
- Multi-use containers
```

**Recycle**:
```
- Mono-material packaging
- Avoid mixed materials
- Recyclability labeling
- Design for disassembly
- Use recycled content
```

**Compost**:
```
- Compostable materials (certified)
- No coatings that prevent composting
- Clear labeling (industrial vs. home compost)
```

---

## 11. Data Formats

### 11.1 Ingredient Data

```json
{
  "ingredient": {
    "identification": {
      "inci_name": "Sodium Hyaluronate",
      "cas_number": "9067-32-7",
      "ec_number": "232-678-0",
      "molecular_formula": "(C14H20NNaO11)n",
      "molecular_weight_range": { "min": 10000, "max": 2000000, "unit": "Da" }
    },
    "function": {
      "primary": "Humectant",
      "secondary": ["Film Former", "Skin Conditioning"],
      "mechanism": "Binds water molecules, increases skin hydration"
    },
    "usage": {
      "typical_range_percent": { "min": 0.05, "max": 2.0 },
      "optimal_percent": 1.0,
      "maximum_allowed_percent": null
    },
    "safety": {
      "safety_score": 95,
      "allergenicity": "Low",
      "irritation_potential": "Low",
      "sensitization_potential": "Very Low",
      "comedogenicity": 0,
      "toxicity_ld50_oral": "> 5000 mg/kg"
    },
    "regulatory": {
      "us_fda": { "status": "approved", "restrictions": null },
      "eu": { "status": "approved", "restrictions": null },
      "china": { "status": "approved", "iecic_listed": true },
      "korea": { "status": "approved", "restrictions": null },
      "japan": { "status": "approved", "jsci_listed": true }
    }
  }
}
```

### 11.2 Formulation Data

```json
{
  "formulation": {
    "product_info": {
      "product_id": "PRD-2025-001",
      "product_name": "Hydrating Facial Serum",
      "product_type": "facial_serum",
      "formula_version": "v2.1",
      "created_date": "2025-01-15",
      "ph_target": 5.8,
      "ph_range": { "min": 5.6, "max": 6.0 }
    },
    "ingredients": [
      {
        "inci_name": "Aqua",
        "percentage": 70.0,
        "function": "Solvent",
        "phase": "A"
      },
      {
        "inci_name": "Glycerin",
        "percentage": 10.0,
        "function": "Humectant",
        "phase": "A"
      },
      {
        "inci_name": "Sodium Hyaluronate",
        "percentage": 1.5,
        "function": "Humectant",
        "phase": "C"
      },
      {
        "inci_name": "Niacinamide",
        "percentage": 5.0,
        "function": "Active",
        "phase": "C"
      },
      {
        "inci_name": "Phenoxyethanol",
        "percentage": 0.8,
        "function": "Preservative",
        "phase": "C"
      },
      {
        "inci_name": "Parfum",
        "percentage": 0.3,
        "function": "Fragrance",
        "phase": "C"
      }
    ],
    "total_percentage": 100.0,
    "stability": {
      "tested": true,
      "accelerated_passed": true,
      "shelf_life_months": 24
    }
  }
}
```

---

## 12. API Interface

### 12.1 REST Endpoints

```
GET /api/v1/ingredients
POST /api/v1/ingredients
GET /api/v1/ingredients/{inci_name}
PUT /api/v1/ingredients/{inci_name}

GET /api/v1/formulations
POST /api/v1/formulations
GET /api/v1/formulations/{id}
PUT /api/v1/formulations/{id}

POST /api/v1/validate/ingredient
POST /api/v1/validate/formulation
POST /api/v1/check-regulatory
POST /api/v1/calculate-safety-score

GET /api/v1/batch/{batch_number}
POST /api/v1/batch
PUT /api/v1/batch/{batch_number}
```

### 12.2 GraphQL Schema

```graphql
type Ingredient {
  inciName: String!
  casNumber: String
  function: String!
  safetyScore: Int
  regulatory: RegulatoryStatus
}

type Formulation {
  productId: String!
  productName: String!
  ingredients: [FormulationIngredient!]!
  safetyAssessment: SafetyAssessment
  regulatoryCompliance: [ComplianceResult!]
}

type Query {
  ingredient(inciName: String!): Ingredient
  formulation(id: String!): Formulation
  searchIngredients(query: String!): [Ingredient!]
}

type Mutation {
  createFormulation(input: FormulationInput!): Formulation
  validateFormulation(id: String!): ValidationResult
}
```

---

## 13. Security and Privacy

### 13.1 Data Protection

**GDPR Compliance** (EU):
```
Personal Data in Cosmetics Context:
- Consumer purchase history
- Skin type, sensitivities, allergies
- Adverse event reports
- Clinical trial participant data
- Product reviews with personal info

Requirements:
- Consent for data collection
- Right to access data
- Right to erasure ("right to be forgotten")
- Data portability
- Privacy by design
- Data breach notification (72 hours)
```

### 13.2 Intellectual Property

**Trade Secrets**:
```
Formula Protection:
- Exact concentrations may be trade secret
- Manufacturing process details
- Supplier information
- Proprietary ingredient blends

Label disclosure requirements override trade secret protection
```

---

## 14. References

### 14.1 Regulatory References

1. FDA - Federal Food, Drug, and Cosmetic Act (FD&C Act)
2. FDA - MoCRA (Modernization of Cosmetics Regulation Act of 2022)
3. EU - Regulation (EC) No 1223/2009 on Cosmetic Products
4. Korea - Cosmetics Act (MFDS)
5. Japan - Pharmaceutical and Medical Device Act
6. China - NMPA Regulations

### 14.2 Standards and Guidelines

1. ISO 22716:2007 - Cosmetics GMP
2. ISO 22718 - Microbiology guidelines
3. OECD Guidelines for Testing of Chemicals
4. ICH Guidelines (Q1A, Q3C, M7)
5. SCCS (Scientific Committee on Consumer Safety) Notes of Guidance
6. Cosmetics Europe - Safety Assessment Guidelines

### 14.3 Technical References

1. INCI Dictionary (Personal Care Products Council)
2. CIR (Cosmetic Ingredient Review) Database
3. EWG Skin Deep Database
4. SCCS Opinions
5. HERA (Human and Environmental Risk Assessment) Project

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
