# WIA-CANCER-METABOLISM Phase 1: Data Format Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2025-12-29

## Overview

The WIA-CANCER-METABOLISM data format specification defines standardized structures for representing cancer metabolic data across research institutions, clinical laboratories, and treatment facilities. This specification enables interoperable exchange of metabolomic profiles, biomarker measurements, metabolic pathway analyses, and patient-specific cancer metabolism data.

### Design Principles

1. **Clinical Relevance**: Data structures optimized for clinical decision support
2. **Research Compatibility**: Support for multi-omics integration and systems biology
3. **Scalability**: Handle datasets from single samples to population-scale studies
4. **Precision**: Maintain scientific accuracy with appropriate data types
5. **Extensibility**: Allow custom fields while maintaining core compatibility

## Core Data Types

### 1. MetabolicProfile

The foundational data structure representing a comprehensive metabolic assessment of cancer tissue or biological sample.

```json
{
  "$schema": "https://wia.org/schemas/cancer-metabolism/v1/metabolic-profile.json",
  "profileId": "string (UUID v4)",
  "patientId": "string (pseudonymized identifier)",
  "sampleId": "string (laboratory identifier)",
  "collectionDate": "string (ISO 8601 datetime)",
  "cancerType": "string (ICD-O-3 topography code)",
  "cancerStage": "string (TNM staging)",
  "sampleType": {
    "tissue": "enum [tumor, adjacent-normal, metastatic, circulating]",
    "source": "string (anatomical location)",
    "quality": {
      "cellularity": "number (0-100 percentage)",
      "viability": "number (0-100 percentage)",
      "necrosisLevel": "number (0-100 percentage)"
    }
  },
  "metabolites": [
    {
      "metaboliteId": "string (HMDB, KEGG, or ChEBI identifier)",
      "name": "string (common name)",
      "concentration": {
        "value": "number",
        "unit": "string (μM, mM, nmol/mg protein, etc.)",
        "method": "string (MS, NMR, enzymatic assay)"
      },
      "confidence": "number (0-1 probability)",
      "detectionMethod": "string",
      "relativeTo": "string (normalization reference)"
    }
  ],
  "pathways": [
    {
      "pathwayId": "string (KEGG, Reactome identifier)",
      "name": "string",
      "activity": {
        "score": "number (-1 to 1 normalized)",
        "pValue": "number",
        "method": "string (GSEA, pathway enrichment, flux analysis)"
      },
      "keyMetabolites": ["string (metaboliteId references)"]
    }
  ],
  "biomarkers": ["string (biomarkerId references)"],
  "metadata": {
    "institution": "string",
    "platform": "string (instrument/technology)",
    "protocol": "string (SOP identifier)",
    "dataQuality": {
      "completeness": "number (0-100)",
      "technicalReplicates": "number",
      "cvPercentage": "number (coefficient of variation)"
    }
  },
  "createdAt": "string (ISO 8601)",
  "updatedAt": "string (ISO 8601)"
}
```

**Required Fields**: `profileId`, `patientId`, `sampleId`, `collectionDate`, `cancerType`, `sampleType`, `metabolites`

**Validation Rules**:
- `profileId` must be UUID v4 format
- `collectionDate` must not be in the future
- `cancerType` must be valid ICD-O-3 code
- `metabolites` array must contain at least 1 element
- All concentration values must be positive numbers
- `confidence` scores must be between 0 and 1

### 2. Biomarker

Represents a cancer metabolism biomarker with clinical significance.

```json
{
  "$schema": "https://wia.org/schemas/cancer-metabolism/v1/biomarker.json",
  "biomarkerId": "string (UUID v4)",
  "name": "string",
  "type": "enum [metabolite, ratio, signature, pattern]",
  "category": "enum [diagnostic, prognostic, predictive, monitoring]",
  "cancerTypes": ["string (ICD-O-3 codes)"],
  "clinicalUtility": {
    "sensitivity": "number (0-100)",
    "specificity": "number (0-100)",
    "ppv": "number (0-100, positive predictive value)",
    "npv": "number (0-100, negative predictive value)",
    "auc": "number (0-1, area under ROC curve)"
  },
  "components": [
    {
      "metaboliteId": "string (HMDB/KEGG/ChEBI)",
      "role": "string",
      "weight": "number (for composite biomarkers)"
    }
  ],
  "referenceRanges": [
    {
      "population": "string (healthy, early-stage, advanced)",
      "min": "number",
      "max": "number",
      "unit": "string",
      "percentile": {
        "p25": "number",
        "p50": "number",
        "p75": "number",
        "p95": "number"
      }
    }
  ],
  "interpretation": {
    "elevated": {
      "threshold": "number",
      "significance": "string (clinical meaning)"
    },
    "depleted": {
      "threshold": "number",
      "significance": "string"
    }
  },
  "evidence": {
    "pubmedIds": ["string"],
    "validationStudies": "number",
    "cohortSize": "number",
    "evidenceLevel": "enum [1A, 1B, 2A, 2B, 3, 4, 5]"
  },
  "regulatoryStatus": {
    "fdaApproved": "boolean",
    "ceMarked": "boolean",
    "clinicalUseApproved": ["string (country codes)"]
  },
  "version": "string (semantic version)",
  "createdAt": "string (ISO 8601)",
  "updatedAt": "string (ISO 8601)"
}
```

**Required Fields**: `biomarkerId`, `name`, `type`, `category`, `cancerTypes`, `components`

**Validation Rules**:
- All percentage values (sensitivity, specificity) must be 0-100
- AUC must be between 0 and 1
- `evidenceLevel` follows Oxford CEBM levels
- At least one component must be specified
- Reference ranges must have min < max

### 3. MetabolicPathway

Represents a metabolic pathway with relevance to cancer biology.

```json
{
  "$schema": "https://wia.org/schemas/cancer-metabolism/v1/pathway.json",
  "pathwayId": "string (UUID v4 or external DB ID)",
  "externalIds": {
    "kegg": "string",
    "reactome": "string",
    "wikipathways": "string",
    "biocyc": "string"
  },
  "name": "string",
  "description": "string",
  "category": "enum [glycolysis, TCA, PPP, fatty-acid, amino-acid, nucleotide, one-carbon, redox, other]",
  "cancerRelevance": {
    "hallmark": "string (Hanahan & Weinberg hallmark)",
    "alterationFrequency": "number (0-100 percentage)",
    "therapeuticTarget": "boolean",
    "targetingDrugs": ["string (drug names)"]
  },
  "reactions": [
    {
      "reactionId": "string (EC number or database ID)",
      "enzyme": "string (gene symbol)",
      "substrates": ["string (metaboliteId)"],
      "products": ["string (metaboliteId)"],
      "cofactors": ["string (metaboliteId)"],
      "reversible": "boolean",
      "regulation": {
        "type": "enum [allosteric, transcriptional, post-translational]",
        "regulators": ["string (metaboliteId or gene symbol)"],
        "effect": "enum [activation, inhibition]"
      }
    }
  ],
  "metabolites": [
    {
      "metaboliteId": "string",
      "role": "enum [substrate, product, intermediate, regulator]",
      "essentiality": "enum [essential, important, minor]"
    }
  ],
  "genes": [
    {
      "symbol": "string (HGNC)",
      "name": "string",
      "function": "string",
      "alterationTypes": ["enum [mutation, amplification, deletion, methylation]"],
      "alterationFrequency": "number (0-100)"
    }
  ],
  "interactions": [
    {
      "interactingPathway": "string (pathwayId)",
      "interactionType": "enum [feeds-into, receives-from, regulates, competes]",
      "metabolicCrosstalk": ["string (shared metaboliteIds)"]
    }
  ],
  "clinicalSignificance": {
    "prognosticValue": "enum [favorable, unfavorable, neutral, context-dependent]",
    "predictiveBiomarkers": ["string (biomarkerId references)"],
    "therapeuticImplications": "string (free text)"
  },
  "references": {
    "pubmedIds": ["string"],
    "reviews": ["string"]
  },
  "version": "string",
  "createdAt": "string (ISO 8601)",
  "updatedAt": "string (ISO 8601)"
}
```

**Required Fields**: `pathwayId`, `name`, `category`, `reactions`, `metabolites`

**Validation Rules**:
- Must have at least one reaction
- Each reaction must have at least one substrate and one product
- Gene symbols must follow HGNC nomenclature
- Alteration frequencies must be 0-100
- Interaction references must point to valid pathwayIds

### 4. Metabolite

Detailed representation of individual metabolites.

```json
{
  "$schema": "https://wia.org/schemas/cancer-metabolism/v1/metabolite.json",
  "metaboliteId": "string (UUID v4)",
  "externalIds": {
    "hmdb": "string (HMDB0000001 format)",
    "kegg": "string (C00001 format)",
    "chebi": "string (CHEBI:15377 format)",
    "pubchem": "string",
    "cas": "string"
  },
  "name": {
    "common": "string",
    "iupac": "string",
    "synonyms": ["string"]
  },
  "structure": {
    "smiles": "string",
    "inchi": "string",
    "inchiKey": "string",
    "molecularFormula": "string",
    "molecularWeight": "number",
    "exactMass": "number"
  },
  "classification": {
    "superClass": "string (lipid, amino acid, nucleotide, etc.)",
    "class": "string",
    "subClass": "string",
    "chemOntology": ["string (ChEBI ontology terms)"]
  },
  "cancerMetabolism": {
    "roleinCancer": "string (free text)",
    "alterationPattern": "enum [elevated, depleted, variable, unchanged]",
    "cancerTypes": [
      {
        "icdCode": "string",
        "alterationDirection": "enum [up, down, variable]",
        "foldChange": "number",
        "pValue": "number",
        "studies": "number (count of supporting studies)"
      }
    ],
    "mechanisticRole": "string",
    "therapeuticRelevance": "string"
  },
  "biologicalContext": {
    "pathways": ["string (pathwayId references)"],
    "biologicalFluids": [
      {
        "fluid": "enum [blood, urine, saliva, CSF, tissue]",
        "normalRange": {
          "min": "number",
          "max": "number",
          "unit": "string"
        }
      }
    ],
    "cellularLocation": ["enum [cytoplasm, mitochondria, nucleus, ER, golgi, extracellular]"]
  },
  "analyticalMethods": [
    {
      "method": "enum [LC-MS, GC-MS, CE-MS, NMR, enzymatic]",
      "sensitivity": "number (detection limit)",
      "dynamicRange": {
        "min": "number",
        "max": "number"
      },
      "preparationProtocol": "string (reference to SOP)"
    }
  ],
  "references": {
    "pubmedIds": ["string"],
    "databases": ["string (source databases)"]
  },
  "version": "string",
  "createdAt": "string (ISO 8601)",
  "updatedAt": "string (ISO 8601)"
}
```

**Required Fields**: `metaboliteId`, `name`, `structure`, `classification`

**Validation Rules**:
- At least one external ID must be provided
- SMILES and InChI must be valid chemical structure representations
- Molecular weight must be positive
- Fold changes should typically be > 0
- p-values must be between 0 and 1

## JSON Schema Definitions

### Complete Schema for MetabolicProfile

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/cancer-metabolism/v1/metabolic-profile.json",
  "title": "Cancer Metabolic Profile",
  "type": "object",
  "required": ["profileId", "patientId", "sampleId", "collectionDate", "cancerType", "sampleType", "metabolites"],
  "properties": {
    "profileId": {
      "type": "string",
      "pattern": "^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$",
      "description": "UUID v4 unique identifier"
    },
    "patientId": {
      "type": "string",
      "minLength": 1,
      "maxLength": 255,
      "description": "Pseudonymized patient identifier"
    },
    "sampleId": {
      "type": "string",
      "minLength": 1,
      "maxLength": 255,
      "description": "Laboratory sample identifier"
    },
    "collectionDate": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 datetime of sample collection"
    },
    "cancerType": {
      "type": "string",
      "pattern": "^[C][0-9]{2}(\\.[0-9])?$",
      "description": "ICD-O-3 topography code"
    },
    "cancerStage": {
      "type": "string",
      "description": "TNM staging classification"
    },
    "sampleType": {
      "type": "object",
      "required": ["tissue", "source"],
      "properties": {
        "tissue": {
          "type": "string",
          "enum": ["tumor", "adjacent-normal", "metastatic", "circulating"]
        },
        "source": {
          "type": "string",
          "description": "Anatomical location"
        },
        "quality": {
          "type": "object",
          "properties": {
            "cellularity": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            },
            "viability": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            },
            "necrosisLevel": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            }
          }
        }
      }
    },
    "metabolites": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "required": ["metaboliteId", "name", "concentration"],
        "properties": {
          "metaboliteId": {
            "type": "string"
          },
          "name": {
            "type": "string"
          },
          "concentration": {
            "type": "object",
            "required": ["value", "unit"],
            "properties": {
              "value": {
                "type": "number",
                "minimum": 0
              },
              "unit": {
                "type": "string"
              },
              "method": {
                "type": "string"
              }
            }
          },
          "confidence": {
            "type": "number",
            "minimum": 0,
            "maximum": 1
          },
          "detectionMethod": {
            "type": "string"
          },
          "relativeTo": {
            "type": "string"
          }
        }
      }
    }
  }
}
```

## Example JSON Payloads

### Example 1: Breast Cancer Metabolic Profile

```json
{
  "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
  "patientId": "PATIENT-BC-00123",
  "sampleId": "SAMPLE-2025-001234",
  "collectionDate": "2025-03-15T09:30:00Z",
  "cancerType": "C50.9",
  "cancerStage": "T2N1M0",
  "sampleType": {
    "tissue": "tumor",
    "source": "breast-upper-outer-quadrant",
    "quality": {
      "cellularity": 85,
      "viability": 92,
      "necrosisLevel": 5
    }
  },
  "metabolites": [
    {
      "metaboliteId": "HMDB0000190",
      "name": "Lactate",
      "concentration": {
        "value": 15.8,
        "unit": "mM",
        "method": "LC-MS/MS"
      },
      "confidence": 0.98,
      "detectionMethod": "targeted-metabolomics",
      "relativeTo": "protein-normalized"
    },
    {
      "metaboliteId": "HMDB0000122",
      "name": "Glucose",
      "concentration": {
        "value": 2.3,
        "unit": "mM",
        "method": "LC-MS/MS"
      },
      "confidence": 0.99,
      "detectionMethod": "targeted-metabolomics",
      "relativeTo": "protein-normalized"
    },
    {
      "metaboliteId": "HMDB0000158",
      "name": "Glutamine",
      "concentration": {
        "value": 8.5,
        "unit": "mM",
        "method": "LC-MS/MS"
      },
      "confidence": 0.97,
      "detectionMethod": "targeted-metabolomics",
      "relativeTo": "protein-normalized"
    }
  ],
  "pathways": [
    {
      "pathwayId": "hsa00010",
      "name": "Glycolysis / Gluconeogenesis",
      "activity": {
        "score": 0.78,
        "pValue": 0.0001,
        "method": "pathway-enrichment-GSEA"
      },
      "keyMetabolites": ["HMDB0000190", "HMDB0000122"]
    }
  ],
  "biomarkers": ["BIOMARKER-WARBURG-001"],
  "metadata": {
    "institution": "Memorial Cancer Center",
    "platform": "Agilent-6495-QQQ-LC-MS",
    "protocol": "SOP-METABOLISM-2025-v2.1",
    "dataQuality": {
      "completeness": 98,
      "technicalReplicates": 3,
      "cvPercentage": 4.2
    }
  },
  "createdAt": "2025-03-15T14:20:00Z",
  "updatedAt": "2025-03-15T14:20:00Z"
}
```

### Example 2: Pancreatic Cancer Biomarker

```json
{
  "biomarkerId": "f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a",
  "name": "Warburg Effect Index",
  "type": "ratio",
  "category": "diagnostic",
  "cancerTypes": ["C25.0", "C25.1", "C25.2"],
  "clinicalUtility": {
    "sensitivity": 87.5,
    "specificity": 82.3,
    "ppv": 78.9,
    "npv": 89.2,
    "auc": 0.89
  },
  "components": [
    {
      "metaboliteId": "HMDB0000190",
      "role": "numerator",
      "weight": 1.0
    },
    {
      "metaboliteId": "HMDB0000122",
      "role": "denominator",
      "weight": 1.0
    }
  ],
  "referenceRanges": [
    {
      "population": "healthy",
      "min": 2.0,
      "max": 4.5,
      "unit": "ratio",
      "percentile": {
        "p25": 2.5,
        "p50": 3.2,
        "p75": 3.9,
        "p95": 4.3
      }
    },
    {
      "population": "pancreatic-cancer",
      "min": 8.5,
      "max": 25.0,
      "unit": "ratio",
      "percentile": {
        "p25": 10.2,
        "p50": 14.5,
        "p75": 18.3,
        "p95": 22.8
      }
    }
  ],
  "interpretation": {
    "elevated": {
      "threshold": 6.0,
      "significance": "Indicates enhanced glycolytic activity and Warburg effect, suggestive of malignancy"
    },
    "depleted": {
      "threshold": 4.0,
      "significance": "Within normal metabolic range"
    }
  },
  "evidence": {
    "pubmedIds": ["34567890", "34567891", "34567892"],
    "validationStudies": 12,
    "cohortSize": 2847,
    "evidenceLevel": "2A"
  },
  "regulatoryStatus": {
    "fdaApproved": false,
    "ceMarked": false,
    "clinicalUseApproved": []
  },
  "version": "1.2.0",
  "createdAt": "2024-06-01T00:00:00Z",
  "updatedAt": "2025-02-15T10:30:00Z"
}
```

## Validation Rules

### Data Type Validation

1. **UUID Format**: All IDs must be valid UUID v4 format
2. **Date/Time**: ISO 8601 format with timezone (YYYY-MM-DDTHH:MM:SSZ)
3. **Numeric Ranges**:
   - Percentages: 0-100
   - Probabilities: 0-1
   - Concentrations: > 0
   - Fold changes: > 0
4. **Enumerations**: Must match defined enum values exactly (case-sensitive)

### Cross-Reference Validation

1. **Metabolite References**: All metaboliteId references must resolve to valid Metabolite objects
2. **Pathway References**: All pathwayId references must resolve to valid MetabolicPathway objects
3. **Biomarker References**: All biomarkerId references must resolve to valid Biomarker objects
4. **External IDs**: When provided, must match database format specifications

### Semantic Validation

1. **Sample Quality**: cellularity + necrosisLevel should not exceed 100%
2. **Statistical Values**: p-values must be between 0 and 1
3. **Time Constraints**: collectionDate must not be in the future
4. **Reference Ranges**: min must be less than max
5. **Percentiles**: p25 < p50 < p75 < p95

### Business Logic Validation

1. **Minimum Data Requirements**:
   - At least 3 metabolites for a valid metabolic profile
   - At least 1 component for a biomarker
   - At least 1 reaction for a pathway
2. **Confidence Thresholds**: Metabolite measurements with confidence < 0.7 should be flagged
3. **Platform Compatibility**: Detection methods must be appropriate for metabolite types

## Extension Mechanisms

### Custom Fields

Any data type can include an `extensions` object for institution-specific or experimental data:

```json
{
  "profileId": "...",
  "extensions": {
    "institutionCode": "MCC",
    "researchProtocol": "STUDY-2025-ABC",
    "customMetrics": {
      "oxidativeStress": 0.75,
      "metabolicFlexibility": 0.42
    }
  }
}
```

### Versioning

All data types support semantic versioning through the `version` field. Backward compatibility is maintained within major versions.

---
弘益人間 (홍익인간) - Benefit All Humanity
© 2025 WIA Standards | MIT License
