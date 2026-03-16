# WIA Smart Breeding Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Genomic Data Schema](#genomic-data-schema)
5. [Pedigree Data Format](#pedigree-data-format)
6. [Phenotype Data Format](#phenotype-data-format)
7. [Genotype Data Format](#genotype-data-format)
8. [Breeding Value Format](#breeding-value-format)
9. [Genetic Diversity Metrics](#genetic-diversity-metrics)
10. [Validation Rules](#validation-rules)
11. [Examples](#examples)
12. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Smart Breeding Data Format Standard defines a unified framework for genomic breeding programs, enabling genomic selection, phenotyping, breeding value estimation, and genetic diversity management across animal and plant breeding systems.

**Core Objectives**:
- Standardize genomic data (SNP, sequence, markers) for breeding programs
- Enable genomic selection (GBLUP, ssGBLUP) with standardized inputs
- Support high-throughput phenotyping data collection and management
- Facilitate breeding value (EBV/GEBV) estimation and exchange
- Promote genetic diversity conservation in breeding populations
- Enable cross-species and cross-platform data interoperability
- Accelerate genetic gain through data-driven breeding decisions

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Genomic Data | SNP genotypes, whole genome sequences, genetic markers |
| Pedigree Records | Parent-offspring relationships, pedigree depth, inbreeding |
| Phenotype Data | Trait measurements, performance records, environmental effects |
| Breeding Values | EBV (Estimated Breeding Value), GEBV (Genomic EBV) |
| Genetic Diversity | Heterozygosity, effective population size, genetic distance |
| Selection Indices | Multi-trait selection, economic weights, breeding objectives |
| Mating Plans | Optimal mating, avoiding inbreeding, mate allocation |
| Gene Banks | Germplasm conservation, cryopreservation records |

### 1.3 Design Principles

1. **Precision**: Support high-density SNP arrays (50K-777K) and whole genome sequencing
2. **Interoperability**: Compatible with VCF, PLINK, HapMap, GFF3 formats
3. **Scalability**: Handle populations from 100 to 1,000,000+ individuals
4. **Privacy**: Support encrypted genomic data and federated learning
5. **Traceability**: Link genomic data to phenotypes, pedigrees, and breeding values
6. **Sustainability**: Promote genetic diversity and long-term population viability

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Genomic Selection** | Breeding method using genome-wide markers to predict breeding values |
| **SNP** | Single Nucleotide Polymorphism (genetic marker) |
| **EBV** | Estimated Breeding Value (traditional pedigree-based) |
| **GEBV** | Genomic Estimated Breeding Value (genomic marker-based) |
| **BLUP** | Best Linear Unbiased Prediction (statistical method) |
| **GBLUP** | Genomic BLUP (using genomic relationship matrix) |
| **Heritability (h²)** | Proportion of phenotypic variance due to genetics |
| **Inbreeding (F)** | Probability that two alleles are identical by descent |
| **MAF** | Minor Allele Frequency (frequency of less common allele) |
| **LD** | Linkage Disequilibrium (non-random association of alleles) |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `individual_id` | Unique identifier for individual | `"BULL-2025-001"` |
| `snp_id` | SNP marker identifier | `"rs110449061"` |
| `genotype` | SNP genotype (0/1/2 or AA/AB/BB) | `1` (heterozygous) |
| `phenotype` | Trait measurement | `{"trait": "MILK_YIELD", "value": 9200}` |
| `ebv` | Breeding value estimate | `{" ebv": 850, "accuracy": 0.85}` |
| `pedigree` | Parent IDs | `{"sire": "BULL-2020-045", "dam": "COW-2018-123"}` |
| `chromosome` | Chromosome number | `1` to `30` (cattle), `1` to `12` (rice) |
| `base_position` | Genomic position (bp) | `12345678` |

---

## Base Structure

### 3.1 Root Schema

All breeding data follows this root structure:

```json
{
  "@context": "https://wiastandards.com/breeding/v1",
  "@type": "BreedingData",
  "version": "1.0.0",
  "species": "CATTLE|PIG|CHICKEN|RICE|CORN|WHEAT|...",
  "breed_variety": "Holstein|Duroc|Japonica|...",
  "data_provider": {
    "organization": "National Livestock Research Institute",
    "contact": "breeding@nias.go.kr",
    "country": "KR"
  },
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "genomic": [...],
    "pedigree": [...],
    "phenotype": [...],
    "breeding_values": [...]
  }
}
```

### 3.2 Individual Record Structure

Each individual (animal or plant) has this core structure:

```json
{
  "individual_id": "BULL-2025-001",
  "species": "CATTLE",
  "breed": "Holstein",
  "sex": "MALE|FEMALE",
  "birth_date": "2025-01-15",
  "status": "ALIVE|DECEASED|CULLED",
  "pedigree": {
    "sire_id": "BULL-2020-045",
    "dam_id": "COW-2018-123",
    "inbreeding_coefficient": 0.045
  },
  "genomic_data": {...},
  "phenotype_records": [...],
  "breeding_values": {...}
}
```

---

## Genomic Data Schema

### 4.1 SNP Genotype Format

SNP genotypes are encoded as:
- **0**: Homozygous for major allele (AA)
- **1**: Heterozygous (AB)
- **2**: Homozygous for minor allele (BB)
- **-9** or **null**: Missing genotype

```json
{
  "genotyping": {
    "individual_id": "BULL-2025-001",
    "platform": "Illumina BovineSNP50",
    "chip_version": "v3",
    "total_markers": 54001,
    "call_rate": 0.985,
    "genotyping_date": "2025-01-10",
    "markers": [
      {
        "marker_id": "SNP_1",
        "rsid": "rs110449061",
        "chromosome": 1,
        "position": 135098,
        "genotype": 1,
        "allele_A": "C",
        "allele_B": "T",
        "allele_frequency": 0.35,
        "imputation_quality": 0.98
      },
      {
        "marker_id": "SNP_2",
        "chromosome": 1,
        "position": 267891,
        "genotype": 2,
        "allele_A": "G",
        "allele_B": "A",
        "allele_frequency": 0.12
      }
    ]
  }
}
```

### 4.2 Genomic Relationship Matrix (G-Matrix)

```json
{
  "g_matrix": {
    "type": "VanRaden_Method1",
    "dimension": 1000,
    "individuals": ["BULL-2025-001", "BULL-2025-002", ...],
    "matrix": [
      [1.05, 0.23, 0.15, ...],
      [0.23, 0.98, 0.31, ...],
      ...
    ],
    "computed_date": "2025-01-15",
    "snp_count": 50000,
    "maf_threshold": 0.01
  }
}
```

### 4.3 Sequence Data Reference

For whole genome sequencing:

```json
{
  "sequence_data": {
    "individual_id": "BULL-2025-001",
    "sequencing_platform": "Illumina NovaSeq 6000",
    "coverage": "30x",
    "file_format": "VCF|BAM|CRAM",
    "file_url": "s3://breeding-data/sequences/BULL-2025-001.vcf.gz",
    "reference_genome": "ARS-UCD1.2",
    "variant_count": 12458903,
    "quality_score": 35.2
  }
}
```

---

## Pedigree Data Format

### 5.1 Pedigree Structure

```json
{
  "pedigree": {
    "individual_id": "BULL-2025-001",
    "sire_id": "BULL-2020-045",
    "dam_id": "COW-2018-123",
    "paternal_grandsire": "BULL-2015-012",
    "paternal_granddam": "COW-2016-089",
    "maternal_grandsire": "BULL-2014-034",
    "maternal_granddam": "COW-2015-067",
    "pedigree_completeness": {
      "generation_1": 1.0,
      "generation_2": 1.0,
      "generation_3": 0.875,
      "max_generation": 8
    },
    "inbreeding": {
      "coefficient": 0.045,
      "method": "Meuwissen_Luo_1992",
      "common_ancestors": ["BULL-2010-001"]
    }
  }
}
```

### 5.2 Inbreeding Calculation

Inbreeding coefficient (F) ranges from 0 (outbred) to 1 (complete inbreeding):

- **F < 0.05**: Low inbreeding (acceptable)
- **0.05 ≤ F < 0.10**: Moderate inbreeding (monitor)
- **0.10 ≤ F < 0.25**: High inbreeding (avoid breeding)
- **F ≥ 0.25**: Very high inbreeding (cull from breeding)

---

## Phenotype Data Format

### 6.1 Trait Measurement

```json
{
  "phenotype": {
    "individual_id": "BULL-2025-001",
    "measurements": [
      {
        "trait_code": "MILK_YIELD",
        "trait_name": "305-day Milk Yield",
        "value": 9200,
        "unit": "kg",
        "measurement_date": "2026-11-15",
        "lactation_number": 1,
        "contemporary_group": "HERD_A_2026_WINTER"
      },
      {
        "trait_code": "FAT_PERCENTAGE",
        "value": 3.85,
        "unit": "%",
        "measurement_date": "2026-11-15"
      },
      {
        "trait_code": "PROTEIN_PERCENTAGE",
        "value": 3.28,
        "unit": "%",
        "measurement_date": "2026-11-15"
      }
    ],
    "environmental_effects": {
      "herd": "FARM-A",
      "season": "WINTER",
      "year": 2026,
      "management_group": "HIGH_INPUT"
    }
  }
}
```

### 6.2 Common Traits by Species

**Cattle (Dairy)**:
- MILK_YIELD (kg), FAT_PERCENTAGE (%), PROTEIN_PERCENTAGE (%)
- SOMATIC_CELL_COUNT (cells/mL), FERTILITY_INDEX

**Cattle (Beef)**:
- DAILY_GAIN (kg/day), CARCASS_WEIGHT (kg), MARBLING_SCORE

**Pigs**:
- DAILY_GAIN (g/day), BACKFAT_THICKNESS (mm), LITTER_SIZE

**Chickens**:
- EGG_PRODUCTION (eggs/year), EGG_WEIGHT (g), FEED_CONVERSION_RATIO

**Crops (Rice)**:
- GRAIN_YIELD (tons/ha), HEADING_DATE (days), PLANT_HEIGHT (cm)

---

## Breeding Value Format

### 7.1 Traditional EBV (Pedigree-based)

```json
{
  "breeding_value": {
    "individual_id": "BULL-2025-001",
    "method": "BLUP",
    "evaluation_date": "2025-01-15",
    "traits": [
      {
        "trait_code": "MILK_YIELD",
        "ebv": 850,
        "accuracy": 0.75,
        "reliability": 0.56,
        "percentile": 92,
        "genetic_trend": "+45 kg/year"
      },
      {
        "trait_code": "FAT_PERCENTAGE",
        "ebv": 0.15,
        "accuracy": 0.72,
        "percentile": 85
      }
    ]
  }
}
```

### 7.2 Genomic EBV (GBLUP)

```json
{
  "genomic_breeding_value": {
    "individual_id": "BULL-2025-001",
    "method": "GBLUP",
    "reference_population_size": 15000,
    "evaluation_date": "2025-01-15",
    "traits": [
      {
        "trait_code": "MILK_YIELD",
        "gebv": 920,
        "accuracy": 0.88,
        "reliability": 0.77,
        "genomic_gain_over_ebv": "+70 kg",
        "percentile": 95
      }
    ],
    "snp_effects": {
      "total_markers_used": 48532,
      "significant_qtl": 23,
      "explained_variance": 0.42
    }
  }
}
```

### 7.3 Selection Index

Multi-trait selection combining multiple traits:

```json
{
  "selection_index": {
    "index_name": "Total Merit Index",
    "individual_id": "BULL-2025-001",
    "total_index": 2450,
    "components": [
      {"trait": "MILK_YIELD", "weight": 0.40, "contribution": 980},
      {"trait": "FAT_PERCENTAGE", "weight": 0.25, "contribution": 612},
      {"trait": "PROTEIN_PERCENTAGE", "weight": 0.25, "contribution": 612},
      {"trait": "FERTILITY", "weight": 0.10, "contribution": 246}
    ],
    "economic_value": "$2,450 lifetime profit potential"
  }
}
```

---

## Genetic Diversity Metrics

### 8.1 Population Diversity

```json
{
  "genetic_diversity": {
    "population_name": "Holstein_Korea",
    "population_size": 25000,
    "effective_population_size": 85,
    "metrics": {
      "observed_heterozygosity": 0.352,
      "expected_heterozygosity": 0.368,
      "inbreeding_rate_per_generation": 0.0059,
      "genetic_drift": 0.0118
    },
    "diversity_status": "MODERATE_CONCERN",
    "recommendation": "Increase Ne to >100 through strategic mating"
  }
}
```

### 8.2 Genetic Distance Matrix

```json
{
  "genetic_distance": {
    "method": "Nei_1972",
    "breeds": ["Holstein", "Jersey", "Angus", "Hanwoo"],
    "distance_matrix": [
      [0.000, 0.125, 0.342, 0.456],
      [0.125, 0.000, 0.378, 0.489],
      [0.342, 0.378, 0.000, 0.234],
      [0.456, 0.489, 0.234, 0.000]
    ]
  }
}
```

---

## Validation Rules

### 9.1 Data Quality Checks

| Field | Validation Rule |
|-------|-----------------|
| `individual_id` | Unique, alphanumeric, 5-50 characters |
| `genotype` | Must be 0, 1, 2, or -9 (missing) |
| `call_rate` | Must be ≥ 0.90 (90% of SNPs called) |
| `maf` | Minor allele frequency ≥ 0.01 (1%) |
| `inbreeding_coefficient` | Must be 0.0 ≤ F ≤ 1.0 |
| `ebv_accuracy` | Must be 0.0 ≤ accuracy ≤ 1.0 |
| `phenotype_value` | Must be within biological range for trait |

### 9.2 Genomic Data Quality

```javascript
// SNP quality control
if (snp.call_rate < 0.90) reject("Low call rate");
if (snp.maf < 0.01) reject("Rare variant");
if (snp.hwe_p_value < 0.0001) warn("HWE deviation");

// Individual quality control
if (individual.call_rate < 0.85) reject("Low sample quality");
if (individual.heterozygosity < 0.25 || individual.heterozygosity > 0.45) warn("Unusual heterozygosity");
```

---

## Examples

### 10.1 Complete Breeding Record

```json
{
  "@context": "https://wiastandards.com/breeding/v1",
  "@type": "BreedingData",
  "version": "1.0.0",
  "individual_id": "BULL-2025-001",
  "species": "CATTLE",
  "breed": "Holstein",
  "sex": "MALE",
  "birth_date": "2025-01-15",
  "pedigree": {
    "sire_id": "BULL-2020-045",
    "dam_id": "COW-2018-123",
    "inbreeding_coefficient": 0.045
  },
  "genomic_data": {
    "platform": "Illumina BovineSNP50",
    "total_markers": 54001,
    "call_rate": 0.985
  },
  "phenotype_records": [
    {
      "trait_code": "DAILY_GAIN",
      "value": 1.45,
      "unit": "kg/day",
      "measurement_date": "2026-07-15"
    }
  ],
  "breeding_values": {
    "method": "GBLUP",
    "traits": [
      {
        "trait_code": "DAILY_GAIN",
        "gebv": 0.18,
        "accuracy": 0.88
      }
    ]
  },
  "certification": {
    "genomic_certified": true,
    "elite_status": "TOP_1_PERCENT",
    "breeding_approval": "APPROVED"
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
**License**: MIT
