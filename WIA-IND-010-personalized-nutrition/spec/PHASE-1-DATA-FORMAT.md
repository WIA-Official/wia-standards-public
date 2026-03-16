# WIA-IND-010 Phase 1: Data Format Specification
## Personalized Nutrition Standard
### 弘益人間 · Benefit All Humanity

---

## Overview

Phase 1 of the WIA-IND-010 Personalized Nutrition Standard defines standardized data formats for health profiles, genetic information, microbiome analysis, meal logging, and biomarker measurements. These schemas ensure interoperability between different personalized nutrition systems, healthcare providers, and research platforms.

## 1. Health Profile Schema

### 1.1 User Profile Structure

```json
{
  "userId": "string (UUID)",
  "createdAt": "ISO 8601 timestamp",
  "lastUpdated": "ISO 8601 timestamp",
  "demographics": {
    "age": "integer",
    "biologicalSex": "enum: male|female|intersex",
    "gender": "enum: man|woman|non-binary|other",
    "ethnicity": "string",
    "ancestry": "array of strings"
  },
  "anthropometrics": {
    "weight_kg": "float",
    "height_cm": "float",
    "bmi": "float (calculated)",
    "bodyFat_percent": "float (optional)",
    "leanMass_kg": "float (optional)",
    "waistCircumference_cm": "float (optional)"
  },
  "healthGoals": {
    "primary": "enum: weight_loss|muscle_gain|athletic_performance|disease_prevention|disease_management|digestive_health|mental_wellbeing|longevity",
    "secondary": "array of enums",
    "targetWeight_kg": "float (optional)",
    "targetBodyFat_percent": "float (optional)",
    "timeframe_weeks": "integer (optional)"
  },
  "dietaryRestrictions": {
    "allergies": "array of strings",
    "intolerances": "array of strings",
    "preferences": "enum: omnivore|vegetarian|vegan|pescatarian|ketogenic|paleo|mediterranean|other",
    "religionBased": "array of strings",
    "excludedFoods": "array of strings"
  },
  "medicalHistory": {
    "conditions": "array of strings",
    "medications": "array of objects {name, dosage, frequency}",
    "supplements": "array of objects {name, dosage, frequency}",
    "familyHistory": "array of strings"
  },
  "wiaStandard": "WIA-IND-010",
  "version": "1.0"
}
```

## 2. Genetic Profile Format

### 2.1 SNP (Single Nucleotide Polymorphism) Data

```json
{
  "userId": "string (UUID)",
  "testProvider": "string (e.g., 23andMe, AncestryDNA)",
  "testDate": "ISO 8601 date",
  "testType": "enum: genotyping_array|whole_exome|whole_genome",
  "snps": [
    {
      "rsid": "string (e.g., rs1801133)",
      "gene": "string (e.g., MTHFR)",
      "chromosome": "string (e.g., 1-22, X, Y, MT)",
      "position": "integer",
      "genotype": "string (e.g., CT, AA, GG)",
      "referenceAllele": "string",
      "alternateAllele": "string",
      "zygosity": "enum: homozygous_ref|heterozygous|homozygous_alt",
      "nutritionalImpact": {
        "category": "enum: folate_metabolism|obesity_risk|lipid_metabolism|lactose_tolerance|caffeine_metabolism|vitamin_d_receptor|other",
        "effect": "string (description)",
        "riskLevel": "enum: low|moderate|high",
        "evidenceStrength": "enum: definitive|strong|moderate|limited"
      }
    }
  ],
  "polygenic_scores": [
    {
      "trait": "string (e.g., Type 2 Diabetes)",
      "score": "float",
      "percentile": "float (0-100)",
      "interpretation": "string"
    }
  ],
  "wiaStandard": "WIA-IND-010",
  "version": "1.0"
}
```

### 2.2 Key Nutrition-Related Genes

#### MTHFR (Methylenetetrahydrofolate Reductase)
- **rs1801133 (C677T)**: Folate metabolism efficiency
- **Impact**: Homozygous T/T requires increased folate intake
- **Recommendation**: 800-1000mcg methylfolate daily

#### FTO (Fat Mass and Obesity-Associated)
- **rs9939609**: Obesity susceptibility
- **Impact**: AA genotype 70% increased obesity risk
- **Recommendation**: High protein diet (30-35%), portion control

#### APOE (Apolipoprotein E)
- **rs429358 & rs7412**: Lipid metabolism, Alzheimer's risk
- **Impact**: e4 allele impairs fat clearance
- **Recommendation**: Limit saturated fat <7%, increase omega-3

#### LCT (Lactase)
- **rs4988235**: Lactose tolerance
- **Impact**: CC genotype = lactose intolerance
- **Recommendation**: Lactose-free dairy alternatives

#### CYP1A2 (Cytochrome P450 1A2)
- **rs762551**: Caffeine metabolism
- **Impact**: Slow metabolizers (AA) experience prolonged caffeine exposure
- **Recommendation**: Limit to 100-200mg/day

## 3. Microbiome Data Format

### 3.1 Bacterial Community Structure

```json
{
  "userId": "string (UUID)",
  "testProvider": "string (e.g., Viome, Thorne)",
  "testDate": "ISO 8601 date",
  "sampleType": "enum: stool|oral|skin|other",
  "sequencingMethod": "enum: 16S_rRNA|shotgun_metagenomic|metatranscriptomic",
  "diversityMetrics": {
    "shannonIndex": "float (higher = more diverse)",
    "simpsonIndex": "float",
    "speciesRichness": "integer (number of species)",
    "evenness": "float (0-1)",
    "interpretationScore": "float (0-10, higher = better)"
  },
  "phylaAbundance": {
    "Firmicutes": "float (percentage)",
    "Bacteroidetes": "float",
    "Actinobacteria": "float",
    "Proteobacteria": "float",
    "other": "float"
  },
  "keySpecies": [
    {
      "name": "string (e.g., Faecalibacterium prausnitzii)",
      "abundance": "float (percentage)",
      "status": "enum: optimal|good|low|depleted|elevated",
      "function": "array of strings (e.g., butyrate_producer, mucin_degrader)",
      "healthImplication": "string"
    }
  ],
  "functionalGenes": [
    {
      "pathway": "string (e.g., butyrate_synthesis)",
      "abundance": "float",
      "capacity": "enum: high|moderate|low"
    }
  ],
  "metabolites": {
    "scfa": {
      "butyrate_umol_g": "float",
      "acetate_umol_g": "float",
      "propionate_umol_g": "float"
    },
    "bileAcids": {
      "primary_umol_g": "float",
      "secondary_umol_g": "float"
    }
  },
  "dysbiosis_indicators": {
    "overall_score": "float (0-10, 10 = healthy)",
    "inflammation_markers": "float",
    "pathogen_abundance": "float"
  },
  "recommendations": "array of strings",
  "wiaStandard": "WIA-IND-010",
  "version": "1.0"
}
```

## 4. Meal Log Format

### 4.1 Individual Meal Entry

```json
{
  "mealId": "string (UUID)",
  "userId": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "mealType": "enum: breakfast|lunch|dinner|snack|other",
  "name": "string (e.g., Grilled Salmon with Quinoa)",
  "location": {
    "type": "enum: home|restaurant|work|other",
    "name": "string (optional, restaurant name)"
  },
  "nutrition": {
    "totalCalories": "float (kcal)",
    "macronutrients": {
      "protein_g": "float",
      "carbohydrates_g": "float",
      "fiber_g": "float",
      "sugar_g": "float",
      "fat_g": "float",
      "saturatedFat_g": "float",
      "monounsaturatedFat_g": "float",
      "polyunsaturatedFat_g": "float",
      "omega3_g": "float",
      "omega6_g": "float"
    },
    "micronutrients": {
      "vitaminA_mcg": "float",
      "vitaminD_iu": "float",
      "vitaminE_mg": "float",
      "vitaminK_mcg": "float",
      "vitaminC_mg": "float",
      "thiamin_mg": "float",
      "riboflavin_mg": "float",
      "niacin_mg": "float",
      "vitaminB6_mg": "float",
      "folate_mcg": "float",
      "vitaminB12_mcg": "float",
      "calcium_mg": "float",
      "iron_mg": "float",
      "magnesium_mg": "float",
      "phosphorus_mg": "float",
      "potassium_mg": "float",
      "sodium_mg": "float",
      "zinc_mg": "float",
      "selenium_mcg": "float"
    },
    "bioactiveCompounds": {
      "polyphenols_mg": "float",
      "carotenoids_mg": "float",
      "glucosinolates_mg": "float"
    }
  },
  "ingredients": [
    {
      "name": "string",
      "weight_g": "float",
      "category": "enum: protein|grain|vegetable|fruit|dairy|fat|other"
    }
  ],
  "preparationMethod": "enum: raw|boiled|baked|fried|grilled|steamed|other",
  "tags": "array of strings (e.g., high-protein, omega3-rich, gluten-free)",
  "portionSize": {
    "serving": "float",
    "unit": "string (e.g., cup, oz, g)"
  },
  "imageUrl": "string (optional)",
  "userRating": "integer 1-5 (optional)",
  "symptoms": "array of strings (optional, e.g., bloating, heartburn)",
  "wiaStandard": "WIA-IND-010",
  "version": "1.0"
}
```

## 5. Biomarker Data Format

### 5.1 Laboratory Results

```json
{
  "userId": "string (UUID)",
  "testDate": "ISO 8601 date",
  "laboratory": "string",
  "fasting": "boolean",
  "biomarkers": {
    "lipidPanel": {
      "totalCholesterol_mgdL": "float",
      "ldlCholesterol_mgdL": "float",
      "hdlCholesterol_mgdL": "float",
      "triglycerides_mgdL": "float",
      "apoB_mgdL": "float (optional)",
      "apoA1_mgdL": "float (optional)",
      "lipoproteina_mgdL": "float (optional)"
    },
    "glucoseMetabolism": {
      "fastingGlucose_mgdL": "float",
      "hba1c_percent": "float",
      "fastingInsulin_uIUmL": "float",
      "homaIR": "float (calculated)"
    },
    "inflammation": {
      "hsCRP_mgL": "float",
      "homocysteine_umolL": "float",
      "il6_pgmL": "float (optional)",
      "tnfAlpha_pgmL": "float (optional)"
    },
    "vitamins": {
      "vitaminD_ngmL": "float",
      "vitaminB12_pgmL": "float",
      "folate_ngmL": "float",
      "vitaminA_mcgdL": "float (optional)",
      "vitaminE_mgdL": "float (optional)"
    },
    "minerals": {
      "iron_ugdL": "float",
      "ferritin_ngmL": "float",
      "tibc_ugdL": "float",
      "transferrinSaturation_percent": "float",
      "magnesium_mgdL": "float",
      "zinc_ugdL": "float (optional)",
      "selenium_ugL": "float (optional)"
    },
    "omega3": {
      "omega3Index_percent": "float",
      "epa_mg": "float",
      "dha_mg": "float"
    },
    "liverFunction": {
      "alt_UL": "float",
      "ast_UL": "float",
      "ggt_UL": "float (optional)"
    },
    "kidneyFunction": {
      "creatinine_mgdL": "float",
      "bun_mgdL": "float",
      "gfr_mLmin": "float (calculated)"
    },
    "thyroid": {
      "tsh_mIUL": "float",
      "freeT4_ngdL": "float (optional)",
      "freeT3_pgmL": "float (optional)"
    }
  },
  "interpretations": [
    {
      "biomarker": "string",
      "value": "float",
      "unit": "string",
      "referenceRange": "object {low, high}",
      "status": "enum: optimal|normal|borderline|abnormal",
      "clinicalSignificance": "string",
      "nutritionalRecommendations": "array of strings"
    }
  ],
  "wiaStandard": "WIA-IND-010",
  "version": "1.0"
}
```

## 6. Wearable Device Data Format

### 6.1 Continuous Glucose Monitor Data

```json
{
  "userId": "string (UUID)",
  "deviceType": "enum: dexcom|freestyle_libre|medtronic|other",
  "measurements": [
    {
      "timestamp": "ISO 8601 datetime",
      "glucose_mgdL": "float",
      "trend": "enum: rising_fast|rising|stable|falling|falling_fast"
    }
  ],
  "mealCorrelations": [
    {
      "mealId": "string (UUID reference)",
      "preMealGlucose_mgdL": "float",
      "peakGlucose_mgdL": "float",
      "timeToPeak_minutes": "integer",
      "glucoseExcursion_mgdL": "float (peak - pre-meal)",
      "auc_mgdL_minutes": "float (area under curve)"
    }
  ],
  "dailyMetrics": {
    "date": "ISO 8601 date",
    "averageGlucose_mgdL": "float",
    "glucoseVariability_cv": "float (coefficient of variation)",
    "timeInRange_percent": "float (70-180 mg/dL)",
    "timeAboveRange_percent": "float (>180 mg/dL)",
    "timeBelowRange_percent": "float (<70 mg/dL)"
  },
  "wiaStandard": "WIA-IND-010",
  "version": "1.0"
}
```

## 7. Data Validation Rules

### 7.1 Required Fields
- All schemas MUST include: userId, timestamp/date, wiaStandard, version
- Genetic data MUST include: testProvider, testDate, at least one SNP
- Microbiome data MUST include: testProvider, testDate, diversityMetrics
- Meal logs MUST include: timestamp, mealType, totalCalories, macronutrients
- Biomarkers MUST include: testDate, at least one biomarker category

### 7.2 Data Quality Standards
- Numeric values MUST be validated against physiologically plausible ranges
- Dates MUST be in ISO 8601 format
- UUIDs MUST conform to RFC 4122
- Enum values MUST match exactly (case-sensitive)
- Arrays MUST NOT contain null or undefined values

### 7.3 Privacy and Security
- All personally identifiable information (PII) MUST be encrypted at rest using AES-256
- Data in transit MUST use TLS 1.3 or higher
- UserId SHOULD be a pseudonymous UUID, not containing PII
- Genetic data REQUIRES additional consent and encryption
- PHI (Protected Health Information) MUST comply with HIPAA, GDPR, or applicable regulations

## 8. Interoperability Standards

### 8.1 FHIR Mapping
- Health Profile → FHIR Patient resource
- Biomarker Data → FHIR Observation resources
- Meal Log → FHIR NutritionOrder resource
- Genetic Data → FHIR MolecularSequence resource

### 8.2 Data Exchange Formats
- Primary format: JSON
- Alternative formats: XML, CSV (for bulk export)
- Compression: gzip for large datasets
- Character encoding: UTF-8

## Conclusion

Phase 1 data formats provide the foundation for interoperable personalized nutrition systems. By standardizing health profiles, genetic data, microbiome analysis, meal logging, and biomarker measurements, WIA-IND-010 enables seamless data exchange between platforms, healthcare providers, and research institutions while maintaining the principle of 弘益人間 (Benefit All Humanity) through accessible, well-documented specifications.

---

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity
WIA-IND-010 Personalized Nutrition Standard v1.0
