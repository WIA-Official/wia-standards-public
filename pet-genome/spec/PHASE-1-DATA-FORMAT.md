# WIA-PET-001: Pet Genome Data Format Specification
## Phase 1: Data Format Standards

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. Introduction

### 1.1 Purpose
This specification defines standardized data formats for pet genetic information, enabling interoperability between veterinary genomics laboratories, pet DNA testing services, breeding programs, and research institutions. The Pet Genome Data Format Standard ensures consistent representation of genetic sequences, breed identification, health markers, and ancestry data across the pet genomics ecosystem.

### 1.2 Scope
This standard covers:
- DNA sequence data formats for companion animals (dogs, cats, horses, birds, exotic pets)
- Variant call format (VCF) extensions for pet-specific genomics
- Reference genome specifications
- Breed identification and lineage tracking
- Genetic health markers and disease predisposition
- Ancestry and heritage analysis data structures
- Genetic diversity metrics
- Cloning and genetic preservation data
- Cross-species compatibility standards
- Privacy and consent metadata for genetic data

### 1.3 Target Audience
- Veterinary genomics laboratories
- Pet DNA testing service providers
- Animal breeding organizations
- Veterinary clinics and hospitals
- Pet insurance companies
- Animal research institutions
- Pet owners and breeders
- Genetic counselors for animals

### 1.4 Design Principles
1. **Interoperability**: Seamless data exchange between different systems
2. **Accuracy**: Precise representation of genetic information
3. **Privacy**: Protection of pet owner and genetic data
4. **Extensibility**: Support for emerging genomic technologies
5. **Species-Agnostic**: Applicable across different companion animal species
6. **Backward Compatibility**: Support for legacy genomic data formats

---

## 2. Core Data Structures

### 2.1 Pet Genome Profile Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Pet Genome Profile",
  "type": "object",
  "required": [
    "profileId",
    "speciesCode",
    "genomicData",
    "sampleMetadata",
    "version"
  ],
  "properties": {
    "profileId": {
      "type": "string",
      "pattern": "^PGP-[A-Z0-9]{12}$",
      "description": "Unique identifier for pet genome profile"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Schema version following semantic versioning"
    },
    "speciesCode": {
      "type": "string",
      "enum": [
        "CANIS_FAMILIARIS",
        "FELIS_CATUS",
        "EQUUS_CABALLUS",
        "PSITTACIFORMES",
        "ORYCTOLAGUS_CUNICULUS",
        "OTHER"
      ],
      "description": "Scientific species classification"
    },
    "petIdentification": {
      "type": "object",
      "properties": {
        "registeredName": {
          "type": "string",
          "maxLength": 200
        },
        "microchipId": {
          "type": "string",
          "pattern": "^[0-9]{15}$"
        },
        "registrationNumber": {
          "type": "string"
        },
        "birthDate": {
          "type": "string",
          "format": "date"
        },
        "sex": {
          "type": "string",
          "enum": ["MALE", "FEMALE", "INTERSEX", "UNKNOWN"]
        },
        "neutered": {
          "type": "boolean"
        }
      }
    },
    "genomicData": {
      "type": "object",
      "required": ["sequencingMethod", "referenceGenome", "variants"],
      "properties": {
        "sequencingMethod": {
          "type": "string",
          "enum": [
            "WHOLE_GENOME_SEQUENCING",
            "WHOLE_EXOME_SEQUENCING",
            "TARGETED_PANEL",
            "GENOTYPING_ARRAY",
            "RNA_SEQUENCING"
          ]
        },
        "referenceGenome": {
          "type": "object",
          "properties": {
            "assembly": {
              "type": "string",
              "description": "e.g., CanFam3.1, CanFam4, Felis_catus_9.0"
            },
            "version": {
              "type": "string"
            },
            "source": {
              "type": "string",
              "format": "uri"
            }
          }
        },
        "coverage": {
          "type": "object",
          "properties": {
            "meanDepth": {
              "type": "number",
              "minimum": 0
            },
            "medianDepth": {
              "type": "number",
              "minimum": 0
            },
            "percentageAbove10x": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            },
            "percentageAbove30x": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            }
          }
        },
        "variants": {
          "type": "object",
          "properties": {
            "vcfFileUri": {
              "type": "string",
              "format": "uri",
              "description": "URI to VCF file containing variant calls"
            },
            "totalVariants": {
              "type": "integer",
              "minimum": 0
            },
            "snpCount": {
              "type": "integer",
              "minimum": 0
            },
            "indelCount": {
              "type": "integer",
              "minimum": 0
            },
            "structuralVariants": {
              "type": "integer",
              "minimum": 0
            },
            "clinicalVariants": {
              "type": "array",
              "items": {
                "$ref": "#/definitions/ClinicalVariant"
              }
            }
          }
        },
        "rawDataUri": {
          "type": "string",
          "format": "uri",
          "description": "URI to raw sequencing data (FASTQ/BAM)"
        }
      }
    },
    "breedInformation": {
      "type": "object",
      "properties": {
        "primaryBreed": {
          "type": "object",
          "properties": {
            "breedName": {
              "type": "string"
            },
            "breedCode": {
              "type": "string",
              "description": "AKC/FCI/TICA standardized breed code"
            },
            "percentage": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            },
            "confidence": {
              "type": "number",
              "minimum": 0,
              "maximum": 1
            }
          }
        },
        "ancestryComposition": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "breedName": {
                "type": "string"
              },
              "percentage": {
                "type": "number",
                "minimum": 0,
                "maximum": 100
              },
              "generationsBack": {
                "type": "integer",
                "minimum": 1
              }
            }
          }
        },
        "breedGroup": {
          "type": "string",
          "description": "e.g., Sporting, Hound, Working, Terrier"
        },
        "wildcatAncestry": {
          "type": "object",
          "description": "For cats with wildcat heritage",
          "properties": {
            "wildcatSpecies": {
              "type": "string"
            },
            "percentage": {
              "type": "number"
            }
          }
        }
      }
    },
    "healthMarkers": {
      "type": "object",
      "properties": {
        "geneticDiseaseRisks": {
          "type": "array",
          "items": {
            "$ref": "#/definitions/DiseaseRisk"
          }
        },
        "pharmacogenomics": {
          "type": "array",
          "items": {
            "$ref": "#/definitions/DrugResponse"
          }
        },
        "traits": {
          "type": "array",
          "items": {
            "$ref": "#/definitions/GeneticTrait"
          }
        }
      }
    },
    "sampleMetadata": {
      "type": "object",
      "required": ["collectionDate", "sampleType", "laboratory"],
      "properties": {
        "sampleId": {
          "type": "string"
        },
        "collectionDate": {
          "type": "string",
          "format": "date-time"
        },
        "sampleType": {
          "type": "string",
          "enum": [
            "BLOOD",
            "SALIVA",
            "BUCCAL_SWAB",
            "TISSUE",
            "HAIR_FOLLICLE"
          ]
        },
        "laboratory": {
          "type": "object",
          "properties": {
            "name": {
              "type": "string"
            },
            "certifications": {
              "type": "array",
              "items": {
                "type": "string"
              }
            },
            "contactInfo": {
              "type": "object"
            }
          }
        },
        "processingDate": {
          "type": "string",
          "format": "date-time"
        }
      }
    },
    "privacyConsent": {
      "type": "object",
      "properties": {
        "ownerConsent": {
          "type": "boolean"
        },
        "researchParticipation": {
          "type": "boolean"
        },
        "dataSharing": {
          "type": "object",
          "properties": {
            "allowBreedingDatabase": {
              "type": "boolean"
            },
            "allowResearch": {
              "type": "boolean"
            },
            "allowCommercial": {
              "type": "boolean"
            }
          }
        },
        "consentDate": {
          "type": "string",
          "format": "date-time"
        }
      }
    },
    "qualityMetrics": {
      "type": "object",
      "properties": {
        "overallQualityScore": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        },
        "contaminationCheck": {
          "type": "boolean"
        },
        "sexConsistency": {
          "type": "boolean"
        },
        "callRate": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        }
      }
    }
  },
  "definitions": {
    "ClinicalVariant": {
      "type": "object",
      "properties": {
        "variantId": {
          "type": "string"
        },
        "chromosome": {
          "type": "string"
        },
        "position": {
          "type": "integer"
        },
        "referenceAllele": {
          "type": "string"
        },
        "alternateAllele": {
          "type": "string"
        },
        "genotype": {
          "type": "string",
          "pattern": "^(0|1|2)/(0|1|2)$"
        },
        "significance": {
          "type": "string",
          "enum": [
            "PATHOGENIC",
            "LIKELY_PATHOGENIC",
            "UNCERTAIN",
            "LIKELY_BENIGN",
            "BENIGN"
          ]
        },
        "associatedConditions": {
          "type": "array",
          "items": {
            "type": "string"
          }
        }
      }
    },
    "DiseaseRisk": {
      "type": "object",
      "properties": {
        "diseaseName": {
          "type": "string"
        },
        "diseaseCode": {
          "type": "string",
          "description": "OMIA (Online Mendelian Inheritance in Animals) code"
        },
        "riskLevel": {
          "type": "string",
          "enum": ["HIGH", "MODERATE", "LOW", "CARRIER"]
        },
        "inheritancePattern": {
          "type": "string",
          "enum": [
            "AUTOSOMAL_DOMINANT",
            "AUTOSOMAL_RECESSIVE",
            "X_LINKED",
            "POLYGENIC"
          ]
        },
        "affectedGenes": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "penetrance": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        }
      }
    },
    "DrugResponse": {
      "type": "object",
      "properties": {
        "drugName": {
          "type": "string"
        },
        "gene": {
          "type": "string"
        },
        "variant": {
          "type": "string"
        },
        "responseType": {
          "type": "string",
          "enum": [
            "NORMAL_METABOLIZER",
            "POOR_METABOLIZER",
            "RAPID_METABOLIZER",
            "ADVERSE_REACTION_RISK"
          ]
        },
        "clinicalRecommendation": {
          "type": "string"
        }
      }
    },
    "GeneticTrait": {
      "type": "object",
      "properties": {
        "traitName": {
          "type": "string"
        },
        "category": {
          "type": "string",
          "enum": [
            "PHYSICAL",
            "BEHAVIORAL",
            "PERFORMANCE",
            "COAT_COLOR",
            "SIZE"
          ]
        },
        "prediction": {
          "type": "string"
        },
        "confidence": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        }
      }
    }
  }
}
```

### 2.2 Variant Call Format (VCF) Extensions

#### Pet-Specific VCF Header Fields

```
##fileformat=VCFv4.3
##fileDate=20251218
##source=PetGenomeAnalyzer_v1.0
##reference=CanFam4.0
##INFO=<ID=BREED_AF,Number=A,Type=Float,Description="Allele frequency in specific breed">
##INFO=<ID=OMIA,Number=1,Type=String,Description="OMIA disease identifier">
##INFO=<ID=TRAIT,Number=.,Type=String,Description="Associated phenotypic trait">
##INFO=<ID=PATHOGENIC,Number=0,Type=Flag,Description="Known pathogenic variant">
##INFO=<ID=CARRIER_FREQ,Number=1,Type=Float,Description="Carrier frequency in population">
##FORMAT=<ID=GT,Number=1,Type=String,Description="Genotype">
##FORMAT=<ID=DP,Number=1,Type=Integer,Description="Read depth">
##FORMAT=<ID=GQ,Number=1,Type=Integer,Description="Genotype quality">
##FORMAT=<ID=PL,Number=G,Type=Integer,Description="Phred-scaled genotype likelihoods">
```

### 2.3 Breed Ancestry Data Format

```json
{
  "ancestryAnalysis": {
    "analysisId": "ANC-DOG-2025-001234",
    "petId": "PGP-ABCD12345678",
    "analysisDate": "2025-12-18T10:30:00Z",
    "methodology": "ADMIXTURE_K12",
    "breedComposition": [
      {
        "breedName": "Labrador Retriever",
        "breedCode": "AKC-122",
        "percentage": 62.5,
        "confidence": 0.95,
        "chromosomalSegments": [
          {
            "chromosome": "1",
            "startPosition": 1000000,
            "endPosition": 25000000,
            "breedOrigin": "Labrador Retriever",
            "confidence": 0.98
          }
        ]
      },
      {
        "breedName": "Golden Retriever",
        "breedCode": "AKC-111",
        "percentage": 25.0,
        "confidence": 0.92
      },
      {
        "breedName": "Mixed Heritage",
        "percentage": 12.5,
        "confidence": 0.75,
        "possibleBreeds": ["Chesapeake Bay Retriever", "Flat-Coated Retriever"]
      }
    ],
    "geneticDiversity": {
      "heterozygosity": 0.342,
      "inbreedingCoefficient": 0.023,
      "effectivePopulationSize": 450
    },
    "haplotypeAnalysis": {
      "maternalHaplogroup": "H1a",
      "paternalHaplogroup": "Y2b",
      "ancientBreedSignature": false
    }
  }
}
```

---

## 3. Reference Genome Standards

### 3.1 Supported Reference Assemblies

| Species | Common Name | Assembly | Version | Release Date | URL |
|---------|-------------|----------|---------|--------------|-----|
| Canis familiaris | Dog | CanFam4 | GCA_011100685.1 | 2020-03 | https://www.ncbi.nlm.nih.gov/assembly/GCF_011100685.1 |
| Canis familiaris | Dog | CanFam3.1 | GCA_000002285.2 | 2011-09 | https://www.ncbi.nlm.nih.gov/assembly/GCF_000002285.3 |
| Felis catus | Cat | Felis_catus_9.0 | GCA_000181335.4 | 2017-11 | https://www.ncbi.nlm.nih.gov/assembly/GCF_000181335.3 |
| Equus caballus | Horse | EquCab3.0 | GCA_002863925.1 | 2018-01 | https://www.ncbi.nlm.nih.gov/assembly/GCF_002863925.1 |
| Oryctolagus cuniculus | Rabbit | OryCun2.0 | GCA_000003625.1 | 2009-04 | https://www.ncbi.nlm.nih.gov/assembly/GCF_000003625.3 |

### 3.2 Chromosome Naming Conventions

```json
{
  "chromosomeMapping": {
    "speciesCode": "CANIS_FAMILIARIS",
    "assembly": "CanFam4",
    "chromosomes": {
      "autosomes": ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10",
                    "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
                    "21", "22", "23", "24", "25", "26", "27", "28", "29", "30",
                    "31", "32", "33", "34", "35", "36", "37", "38"],
      "sexChromosomes": ["X", "Y"],
      "mitochondrial": ["MT"],
      "unplaced": ["Un"],
      "namingConvention": "CHR_PREFIX",
      "vcfFormat": "chr1, chr2, ..., chrX, chrY, chrM"
    }
  }
}
```

---

## 4. Health Marker Standards

### 4.1 Disease Risk Classification

| Risk Level | Genotype Interpretation | Clinical Action | Testing Frequency |
|------------|------------------------|-----------------|-------------------|
| AFFECTED | Homozygous for pathogenic variant | Immediate veterinary consultation | Annual monitoring |
| HIGH_RISK | Heterozygous for dominant pathogenic variant | Preventive care recommended | Semi-annual monitoring |
| CARRIER | Heterozygous for recessive variant | Breeding considerations | Pre-breeding screening |
| MODERATE | Polygenic risk factors present | Lifestyle modifications | Every 2-3 years |
| LOW | Protective or benign variants | Standard care | Every 5 years |
| UNKNOWN | Novel variant of uncertain significance | Genetic counseling | As needed |

### 4.2 Common Genetic Diseases Schema

```json
{
  "geneticDiseases": [
    {
      "diseaseId": "OMIA-001424",
      "diseaseName": "Progressive Retinal Atrophy (PRA)",
      "speciesAffected": ["CANIS_FAMILIARIS"],
      "affectedBreeds": [
        "Labrador Retriever",
        "Cocker Spaniel",
        "Poodle",
        "Irish Setter"
      ],
      "inheritancePattern": "AUTOSOMAL_RECESSIVE",
      "causativeGene": "PRCD",
      "knownVariants": [
        {
          "variantId": "c.5G>A",
          "chromosome": "9",
          "position": 69632163,
          "consequence": "MISSENSE",
          "pathogenicity": "PATHOGENIC"
        }
      ],
      "clinicalSigns": [
        "Night blindness",
        "Progressive vision loss",
        "Complete blindness (advanced stages)"
      ],
      "ageOfOnset": "3-5 years",
      "penetrance": 1.0,
      "treatmentAvailable": false,
      "preventiveScreening": "Recommended before breeding"
    },
    {
      "diseaseId": "OMIA-000347",
      "diseaseName": "Multidrug Resistance (MDR1)",
      "speciesAffected": ["CANIS_FAMILIARIS"],
      "affectedBreeds": [
        "Collie",
        "Australian Shepherd",
        "Shetland Sheepdog",
        "German Shepherd"
      ],
      "inheritancePattern": "AUTOSOMAL_RECESSIVE",
      "causativeGene": "ABCB1",
      "knownVariants": [
        {
          "variantId": "c.227_230delATAG",
          "chromosome": "14",
          "position": 23515518,
          "consequence": "FRAMESHIFT",
          "pathogenicity": "PATHOGENIC"
        }
      ],
      "clinicalImpact": "Drug sensitivity",
      "affectedDrugs": [
        "Ivermectin",
        "Loperamide",
        "Vincristine",
        "Doxorubicin"
      ],
      "penetrance": 1.0,
      "clinicalManagement": "Avoid contraindicated drugs",
      "pharmacogenomicGuidance": "Use alternative medications"
    }
  ]
}
```

---

## 5. Code Examples

### 5.1 Parse Pet Genome Profile

```python
import json
from typing import Dict, List, Optional
from datetime import datetime

class PetGenomeProfile:
    """Parser for WIA Pet Genome Profile data format"""

    def __init__(self, profile_data: Dict):
        self.data = profile_data
        self.profile_id = profile_data['profileId']
        self.species = profile_data['speciesCode']

    def get_breed_composition(self) -> List[Dict]:
        """Extract breed ancestry information"""
        breed_info = self.data.get('breedInformation', {})
        ancestry = breed_info.get('ancestryComposition', [])

        return sorted(ancestry,
                     key=lambda x: x.get('percentage', 0),
                     reverse=True)

    def get_disease_risks(self, risk_level: Optional[str] = None) -> List[Dict]:
        """Retrieve genetic disease risk markers"""
        health_markers = self.data.get('healthMarkers', {})
        diseases = health_markers.get('geneticDiseaseRisks', [])

        if risk_level:
            diseases = [d for d in diseases
                       if d.get('riskLevel') == risk_level]

        return diseases

    def get_pharmacogenomic_alerts(self) -> List[Dict]:
        """Get drug response predictions"""
        health_markers = self.data.get('healthMarkers', {})
        return health_markers.get('pharmacogenomics', [])

    def calculate_diversity_score(self) -> float:
        """Calculate genetic diversity index"""
        breed_comp = self.get_breed_composition()

        if not breed_comp:
            return 0.0

        # Shannon diversity index
        diversity = 0.0
        for breed in breed_comp:
            p = breed.get('percentage', 0) / 100.0
            if p > 0:
                diversity -= p * math.log(p)

        return diversity

    def is_suitable_for_breeding(self) -> Dict:
        """Assess breeding suitability based on genetic health"""
        high_risk_diseases = self.get_disease_risks(risk_level='HIGH')
        affected_diseases = self.get_disease_risks(risk_level='AFFECTED')

        carrier_status = [d for d in self.get_disease_risks(risk_level='CARRIER')]

        suitable = len(affected_diseases) == 0 and len(high_risk_diseases) == 0

        return {
            'suitable': suitable,
            'affectedConditions': len(affected_diseases),
            'carrierConditions': len(carrier_status),
            'recommendations': self._generate_breeding_recommendations(
                affected_diseases, high_risk_diseases, carrier_status
            )
        }

    def _generate_breeding_recommendations(self, affected, high_risk, carriers):
        """Generate breeding recommendations"""
        recommendations = []

        if affected:
            recommendations.append("DO NOT BREED: Affected by genetic disease(s)")
        if high_risk:
            recommendations.append("Consult genetic counselor before breeding")
        if carriers:
            recommendations.append(f"Carrier for {len(carriers)} condition(s) - test breeding partner")

        return recommendations if recommendations else ["Suitable for breeding program"]

# Usage example
with open('pet_genome_profile.json', 'r') as f:
    profile_data = json.load(f)

pet_genome = PetGenomeProfile(profile_data)

# Get breed composition
breeds = pet_genome.get_breed_composition()
print(f"Primary breed: {breeds[0]['breedName']} ({breeds[0]['percentage']}%)")

# Check disease risks
high_risk = pet_genome.get_disease_risks(risk_level='HIGH')
for disease in high_risk:
    print(f"Warning: {disease['diseaseName']} - {disease['riskLevel']}")

# Breeding assessment
breeding_assessment = pet_genome.is_suitable_for_breeding()
print(f"Breeding suitable: {breeding_assessment['suitable']}")
```

### 5.2 VCF Variant Analysis

```python
import vcf
from typing import List, Dict

class PetVCFAnalyzer:
    """Analyze VCF files with pet-specific annotations"""

    def __init__(self, vcf_path: str):
        self.vcf_reader = vcf.Reader(open(vcf_path, 'r'))

    def extract_pathogenic_variants(self) -> List[Dict]:
        """Extract all pathogenic variants from VCF"""
        pathogenic = []

        for record in self.vcf_reader:
            if 'PATHOGENIC' in record.INFO:
                variant = {
                    'chromosome': record.CHROM,
                    'position': record.POS,
                    'reference': record.REF,
                    'alternate': ','.join([str(alt) for alt in record.ALT]),
                    'genotype': record.samples[0]['GT'] if record.samples else None,
                    'omia': record.INFO.get('OMIA', None),
                    'trait': record.INFO.get('TRAIT', None)
                }
                pathogenic.append(variant)

        return pathogenic

    def calculate_breed_specific_frequency(self, breed: str) -> Dict:
        """Calculate allele frequencies for specific breed"""
        breed_variants = {}

        for record in self.vcf_reader:
            breed_af = record.INFO.get('BREED_AF', None)
            if breed_af:
                variant_id = f"{record.CHROM}:{record.POS}"
                breed_variants[variant_id] = {
                    'allele_frequency': breed_af,
                    'carrier_frequency': record.INFO.get('CARRIER_FREQ', 0)
                }

        return breed_variants

    def identify_compound_heterozygotes(self) -> List[Dict]:
        """Identify compound heterozygous disease variants"""
        gene_variants = {}

        for record in self.vcf_reader:
            if record.samples and record.samples[0]['GT'] == '0/1':
                gene = record.INFO.get('GENE', 'UNKNOWN')
                if gene not in gene_variants:
                    gene_variants[gene] = []
                gene_variants[gene].append(record)

        # Find genes with multiple het variants
        compound_het = []
        for gene, variants in gene_variants.items():
            if len(variants) >= 2:
                compound_het.append({
                    'gene': gene,
                    'variant_count': len(variants),
                    'variants': [f"{v.CHROM}:{v.POS}" for v in variants]
                })

        return compound_het

# Usage
analyzer = PetVCFAnalyzer('pet_genome.vcf')

# Find pathogenic variants
pathogenic = analyzer.extract_pathogenic_variants()
print(f"Found {len(pathogenic)} pathogenic variants")

for variant in pathogenic:
    print(f"  {variant['chromosome']}:{variant['position']} - {variant['omia']}")
```

### 5.3 Breed Prediction Algorithm

```python
import numpy as np
from sklearn.mixture import GaussianMixture
from typing import List, Tuple

class BreedIdentificationEngine:
    """Machine learning-based breed identification"""

    def __init__(self, reference_panel_path: str):
        self.reference_panel = self._load_reference_panel(reference_panel_path)
        self.breed_models = {}

    def _load_reference_panel(self, path: str) -> Dict:
        """Load breed reference genotypes"""
        # Load pre-computed breed signatures
        import pickle
        with open(path, 'rb') as f:
            return pickle.load(f)

    def predict_breed_composition(self,
                                  genotype_data: np.ndarray,
                                  k_breeds: int = 5) -> List[Dict]:
        """
        Predict breed composition using admixture analysis

        Args:
            genotype_data: SNP genotype matrix (samples x markers)
            k_breeds: Number of ancestral breeds to estimate

        Returns:
            List of breed predictions with percentages
        """
        # ADMIXTURE-like algorithm
        gmm = GaussianMixture(n_components=k_breeds,
                            covariance_type='diagonal',
                            random_state=42)
        gmm.fit(genotype_data)

        # Get ancestry proportions
        ancestry_props = gmm.predict_proba(genotype_data)

        # Match components to known breeds
        breed_assignments = self._match_components_to_breeds(
            gmm.means_, ancestry_props
        )

        return breed_assignments

    def _match_components_to_breeds(self,
                                   component_means: np.ndarray,
                                   ancestry_props: np.ndarray) -> List[Dict]:
        """Match ancestry components to known breeds"""
        breed_results = []

        for i, component in enumerate(component_means):
            # Find closest breed in reference panel
            breed_match = self._find_closest_breed(component)
            percentage = np.mean(ancestry_props[:, i]) * 100

            if percentage > 5.0:  # Threshold for reporting
                breed_results.append({
                    'breedName': breed_match['name'],
                    'breedCode': breed_match['code'],
                    'percentage': round(percentage, 2),
                    'confidence': breed_match['confidence']
                })

        # Sort by percentage
        breed_results.sort(key=lambda x: x['percentage'], reverse=True)

        # Normalize to 100%
        total = sum([b['percentage'] for b in breed_results])
        for breed in breed_results:
            breed['percentage'] = (breed['percentage'] / total) * 100

        return breed_results

    def _find_closest_breed(self, component_signature: np.ndarray) -> Dict:
        """Find closest matching breed in reference panel"""
        min_distance = float('inf')
        best_match = None

        for breed_name, breed_data in self.reference_panel.items():
            signature = breed_data['signature']
            distance = np.linalg.norm(component_signature - signature)

            if distance < min_distance:
                min_distance = distance
                best_match = {
                    'name': breed_name,
                    'code': breed_data['code'],
                    'confidence': 1.0 / (1.0 + distance)
                }

        return best_match

    def validate_breed_claim(self,
                           claimed_breed: str,
                           genotype_data: np.ndarray) -> Dict:
        """Validate claimed breed against genetic data"""
        predicted_breeds = self.predict_breed_composition(genotype_data)

        # Check if claimed breed is in top predictions
        claimed_match = None
        for i, breed in enumerate(predicted_breeds):
            if breed['breedName'].lower() == claimed_breed.lower():
                claimed_match = {
                    'rank': i + 1,
                    'percentage': breed['percentage'],
                    'validated': breed['percentage'] >= 75.0
                }
                break

        return {
            'claimedBreed': claimed_breed,
            'match': claimed_match,
            'predictedBreeds': predicted_breeds[:3],
            'isPurebred': predicted_breeds[0]['percentage'] >= 87.5 if predicted_breeds else False
        }
```

### 5.4 Genetic Diversity Calculator

```python
import numpy as np
from scipy.stats import chi2
from typing import Dict, List

class GeneticDiversityAnalyzer:
    """Calculate genetic diversity metrics for pet populations"""

    def __init__(self, genotype_matrix: np.ndarray, sample_ids: List[str]):
        """
        Args:
            genotype_matrix: Matrix of genotypes (samples x SNPs), coded as 0,1,2
            sample_ids: List of sample identifiers
        """
        self.genotypes = genotype_matrix
        self.sample_ids = sample_ids
        self.n_samples = len(sample_ids)
        self.n_snps = genotype_matrix.shape[1]

    def calculate_heterozygosity(self) -> Dict:
        """Calculate observed and expected heterozygosity"""
        # Observed heterozygosity (proportion of het genotypes)
        het_genotypes = np.sum(self.genotypes == 1, axis=0)
        ho = np.mean(het_genotypes / self.n_samples)

        # Expected heterozygosity (Hardy-Weinberg)
        allele_freqs = self._calculate_allele_frequencies()
        he_per_snp = 2 * allele_freqs * (1 - allele_freqs)
        he = np.mean(he_per_snp)

        return {
            'observedHeterozygosity': round(ho, 4),
            'expectedHeterozygosity': round(he, 4),
            'heterozygosityDeficit': round((he - ho) / he, 4) if he > 0 else 0,
            'interpretation': self._interpret_heterozygosity(ho, he)
        }

    def calculate_inbreeding_coefficient(self) -> Dict:
        """Calculate inbreeding coefficient (F)"""
        het_stats = self.calculate_heterozygosity()
        ho = het_stats['observedHeterozygosity']
        he = het_stats['expectedHeterozygosity']

        # F = (He - Ho) / He
        f_coeff = (he - ho) / he if he > 0 else 0

        # Runs of homozygosity (ROH) analysis
        roh_segments = self._detect_roh_segments()

        return {
            'inbreedingCoefficient': round(f_coeff, 4),
            'rohBasedInbreeding': self._calculate_roh_f(roh_segments),
            'rohSegments': len(roh_segments),
            'totalRohLength': sum([s['length'] for s in roh_segments]),
            'interpretation': self._interpret_inbreeding(f_coeff)
        }

    def _calculate_allele_frequencies(self) -> np.ndarray:
        """Calculate minor allele frequency for each SNP"""
        # Count alleles (genotypes: 0=AA, 1=AB, 2=BB)
        allele_counts = np.sum(self.genotypes, axis=0)
        total_alleles = self.n_samples * 2
        freq = allele_counts / total_alleles

        # Return minor allele frequency
        return np.minimum(freq, 1 - freq)

    def _detect_roh_segments(self, min_snps: int = 50) -> List[Dict]:
        """Detect runs of homozygosity"""
        roh_segments = []

        for sample_idx in range(self.n_samples):
            genotypes = self.genotypes[sample_idx, :]

            current_run = []
            for snp_idx, gt in enumerate(genotypes):
                if gt != 1:  # Homozygous
                    current_run.append(snp_idx)
                else:
                    if len(current_run) >= min_snps:
                        roh_segments.append({
                            'sampleId': self.sample_ids[sample_idx],
                            'startSnp': current_run[0],
                            'endSnp': current_run[-1],
                            'length': len(current_run)
                        })
                    current_run = []

            # Check final run
            if len(current_run) >= min_snps:
                roh_segments.append({
                    'sampleId': self.sample_ids[sample_idx],
                    'startSnp': current_run[0],
                    'endSnp': current_run[-1],
                    'length': len(current_run)
                })

        return roh_segments

    def _calculate_roh_f(self, roh_segments: List[Dict]) -> float:
        """Calculate inbreeding based on ROH"""
        if not roh_segments:
            return 0.0

        # Assume genome length of 2.5 Gb (dog genome)
        genome_length = 2500000000
        total_roh = sum([s['length'] for s in roh_segments])

        # Estimate physical length (rough approximation)
        physical_roh = (total_roh / self.n_snps) * genome_length

        return round(physical_roh / genome_length, 4)

    def _interpret_heterozygosity(self, ho: float, he: float) -> str:
        """Interpret heterozygosity values"""
        if ho < he * 0.85:
            return "Low diversity - possible inbreeding"
        elif ho > he * 1.05:
            return "High diversity - outbred population"
        else:
            return "Normal diversity levels"

    def _interpret_inbreeding(self, f: float) -> str:
        """Interpret inbreeding coefficient"""
        if f < 0.05:
            return "Low inbreeding"
        elif f < 0.125:
            return "Moderate inbreeding (equivalent to cousins)"
        elif f < 0.25:
            return "High inbreeding (equivalent to siblings)"
        else:
            return "Very high inbreeding - health concerns"

    def calculate_effective_population_size(self) -> int:
        """Estimate effective population size"""
        # Based on linkage disequilibrium
        r2_values = self._calculate_ld()

        # Ne = 1 / (3 * mean(r2))
        mean_r2 = np.mean(r2_values)
        ne = int(1 / (3 * mean_r2)) if mean_r2 > 0 else 0

        return max(ne, 1)

    def _calculate_ld(self) -> np.ndarray:
        """Calculate linkage disequilibrium (r²) between adjacent SNPs"""
        r2_values = []

        # Sample 1000 SNP pairs for efficiency
        sample_size = min(1000, self.n_snps - 1)
        indices = np.random.choice(self.n_snps - 1, sample_size, replace=False)

        for i in indices:
            gt1 = self.genotypes[:, i]
            gt2 = self.genotypes[:, i + 1]

            # Calculate r²
            r2 = self._calculate_r2(gt1, gt2)
            if not np.isnan(r2):
                r2_values.append(r2)

        return np.array(r2_values)

    def _calculate_r2(self, gt1: np.ndarray, gt2: np.ndarray) -> float:
        """Calculate r² between two SNPs"""
        # Convert to allele counts
        p1 = np.mean(gt1) / 2
        p2 = np.mean(gt2) / 2

        # Calculate haplotype frequency
        d = np.mean(gt1 * gt2) / 4 - p1 * p2

        # r² = D² / (p1 * (1-p1) * p2 * (1-p2))
        denominator = p1 * (1 - p1) * p2 * (1 - p2)

        if denominator > 0:
            return (d ** 2) / denominator
        else:
            return np.nan

# Usage example
genotypes = np.random.randint(0, 3, size=(100, 10000))  # 100 samples, 10k SNPs
sample_ids = [f"PET-{i:04d}" for i in range(100)]

analyzer = GeneticDiversityAnalyzer(genotypes, sample_ids)

# Calculate metrics
het_results = analyzer.calculate_heterozygosity()
print(f"Observed heterozygosity: {het_results['observedHeterozygosity']}")
print(f"Expected heterozygosity: {het_results['expectedHeterozygosity']}")

inbreeding = analyzer.calculate_inbreeding_coefficient()
print(f"Inbreeding coefficient: {inbreeding['inbreedingCoefficient']}")
print(f"ROH segments detected: {inbreeding['rohSegments']}")

ne = analyzer.calculate_effective_population_size()
print(f"Effective population size: {ne}")
```

### 5.5 Data Export and Conversion

```python
import json
import csv
from datetime import datetime
from typing import Dict, List

class PetGenomeExporter:
    """Export pet genome data in various formats"""

    def __init__(self, genome_profile: Dict):
        self.profile = genome_profile

    def export_to_pedigree_format(self, output_path: str):
        """Export for pedigree management software"""
        pedigree_data = {
            'individualId': self.profile['profileId'],
            'registeredName': self.profile.get('petIdentification', {}).get('registeredName'),
            'microchipId': self.profile.get('petIdentification', {}).get('microchipId'),
            'sex': self.profile.get('petIdentification', {}).get('sex'),
            'birthDate': self.profile.get('petIdentification', {}).get('birthDate'),
            'breedComposition': self.profile.get('breedInformation', {}).get('ancestryComposition', []),
            'geneticHealth': {
                'affectedConditions': [],
                'carrierStatus': [],
                'clearConditions': []
            },
            'inbreedingCoefficient': self._extract_inbreeding_coefficient(),
            'parentage': {
                'sire': None,  # To be linked
                'dam': None    # To be linked
            }
        }

        # Categorize health conditions
        diseases = self.profile.get('healthMarkers', {}).get('geneticDiseaseRisks', [])
        for disease in diseases:
            risk = disease.get('riskLevel')
            if risk == 'AFFECTED':
                pedigree_data['geneticHealth']['affectedConditions'].append(disease['diseaseName'])
            elif risk == 'CARRIER':
                pedigree_data['geneticHealth']['carrierStatus'].append(disease['diseaseName'])

        with open(output_path, 'w') as f:
            json.dump(pedigree_data, f, indent=2)

    def export_to_veterinary_report(self, output_path: str):
        """Generate veterinary clinical report"""
        report = f"""
VETERINARY GENETIC HEALTH REPORT
================================
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

PATIENT INFORMATION
-------------------
Profile ID: {self.profile['profileId']}
Species: {self.profile['speciesCode']}
Microchip: {self.profile.get('petIdentification', {}).get('microchipId', 'N/A')}

BREED IDENTIFICATION
--------------------
"""
        breeds = self.profile.get('breedInformation', {}).get('ancestryComposition', [])
        for breed in breeds[:5]:
            report += f"  {breed['breedName']}: {breed['percentage']:.1f}%\n"

        report += "\nGENETIC HEALTH SCREENING\n"
        report += "------------------------\n"

        diseases = self.profile.get('healthMarkers', {}).get('geneticDiseaseRisks', [])

        # Group by risk level
        for risk_level in ['AFFECTED', 'HIGH', 'CARRIER', 'MODERATE']:
            relevant = [d for d in diseases if d.get('riskLevel') == risk_level]
            if relevant:
                report += f"\n{risk_level} RISK:\n"
                for disease in relevant:
                    report += f"  - {disease['diseaseName']}\n"
                    report += f"    Inheritance: {disease.get('inheritancePattern', 'Unknown')}\n"
                    if 'affectedGenes' in disease:
                        report += f"    Genes: {', '.join(disease['affectedGenes'])}\n"

        report += "\nPHARMACOGENOMIC CONSIDERATIONS\n"
        report += "------------------------------\n"

        pharmacogenomics = self.profile.get('healthMarkers', {}).get('pharmacogenomics', [])
        for drug_response in pharmacogenomics:
            if drug_response.get('responseType') in ['POOR_METABOLIZER', 'ADVERSE_REACTION_RISK']:
                report += f"  WARNING: {drug_response['drugName']}\n"
                report += f"    Type: {drug_response['responseType']}\n"
                report += f"    Recommendation: {drug_response.get('clinicalRecommendation', 'Consult specialist')}\n\n"

        with open(output_path, 'w') as f:
            f.write(report)

    def export_to_csv_summary(self, output_path: str):
        """Export summary data to CSV"""
        with open(output_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Header
            writer.writerow(['Category', 'Item', 'Value', 'Details'])

            # Basic info
            writer.writerow(['Identification', 'Profile ID', self.profile['profileId'], ''])
            writer.writerow(['Identification', 'Species', self.profile['speciesCode'], ''])

            # Breeds
            breeds = self.profile.get('breedInformation', {}).get('ancestryComposition', [])
            for breed in breeds:
                writer.writerow(['Breed', breed['breedName'],
                               f"{breed['percentage']:.2f}%",
                               f"Confidence: {breed.get('confidence', 'N/A')}"])

            # Health markers
            diseases = self.profile.get('healthMarkers', {}).get('geneticDiseaseRisks', [])
            for disease in diseases:
                writer.writerow(['Health', disease['diseaseName'],
                               disease['riskLevel'],
                               disease.get('inheritancePattern', '')])

    def _extract_inbreeding_coefficient(self) -> float:
        """Extract inbreeding coefficient from profile"""
        diversity = self.profile.get('breedInformation', {}).get('geneticDiversity', {})
        return diversity.get('inbreedingCoefficient', 0.0)

# Usage
with open('pet_genome_profile.json', 'r') as f:
    profile = json.load(f)

exporter = PetGenomeExporter(profile)

# Export to different formats
exporter.export_to_pedigree_format('pedigree_export.json')
exporter.export_to_veterinary_report('vet_report.txt')
exporter.export_to_csv_summary('genome_summary.csv')
```

---

## 6. Data Privacy and Security

### 6.1 Privacy Requirements

| Data Category | Privacy Level | Encryption | Retention Period | Owner Access |
|--------------|--------------|------------|------------------|--------------|
| Raw Genomic Data | HIGHLY SENSITIVE | AES-256 | 10 years | Full access |
| Processed Variants | SENSITIVE | AES-256 | 15 years | Full access |
| Breed Information | MODERATE | TLS in transit | Indefinite | Full access |
| Health Markers | HIGHLY SENSITIVE | AES-256 | Lifetime + 5 years | Full access |
| Owner Identity | PERSONALLY IDENTIFIABLE | AES-256 | Per consent | Restricted |
| Research Data (anonymized) | PUBLIC | None | Indefinite | Aggregate only |

### 6.2 Consent Management Schema

```json
{
  "consentRecord": {
    "consentId": "CONSENT-2025-12-18-001",
    "petProfileId": "PGP-ABCD12345678",
    "ownerDetails": {
      "ownerId": "OWNER-ENCRYPTED-HASH",
      "contactEmail": "owner@example.com",
      "verificationMethod": "EMAIL_AND_SIGNATURE"
    },
    "consentGiven": {
      "geneticTesting": true,
      "dataStorage": true,
      "researchParticipation": false,
      "commercialUse": false,
      "breedRegistry": true,
      "dataSharing": {
        "veterinarians": true,
        "breeders": false,
        "researchers": false,
        "insurance": false
      }
    },
    "dataRetention": {
      "retainIndefinitely": false,
      "deletionDate": "2035-12-18",
      "rightToErasure": true
    },
    "consentTimestamp": "2025-12-18T14:30:00Z",
    "consentVersion": "1.0",
    "legalJurisdiction": "EU-GDPR"
  }
}
```

---

## 7. Quality Control Standards

### 7.1 Sequencing Quality Metrics

| Metric | Minimum Threshold | Recommended | Excellent |
|--------|------------------|-------------|-----------|
| Mean Coverage Depth | 10x | 30x | 50x |
| Percentage ≥10x | 85% | 95% | 98% |
| Call Rate | 95% | 98% | 99.5% |
| Ti/Tv Ratio (WGS) | 2.0 | 2.1-2.3 | 2.2 |
| Het/Hom Ratio | 1.3-1.8 | 1.4-1.6 | 1.5 |
| Contamination Rate | <3% | <1% | <0.5% |
| Sex Consistency | 100% | 100% | 100% |

---

## 8. File Format Specifications

### 8.1 Supported File Types

- **FASTQ**: Raw sequencing reads
- **BAM/CRAM**: Aligned sequence data
- **VCF/BCF**: Variant calls
- **GFF3/GTF**: Genome annotations
- **FASTA**: Reference genomes
- **BED**: Genomic regions
- **JSON**: Structured genomic metadata
- **HDF5**: Large-scale genomic matrices

---

## 9. Interoperability Standards

### 9.1 Integration with External Systems

- **NCBI Sequence Read Archive (SRA)**: Raw data deposition
- **European Nucleotide Archive (ENA)**: International data sharing
- **OMIA (Online Mendelian Inheritance in Animals)**: Disease variant database
- **AKC/FCI Registries**: Breed verification
- **Veterinary Practice Management Systems**: Clinical integration
- **Pet Insurance Platforms**: Risk assessment (with consent)

---

## 10. Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | Initial draft specification | WIA Standards Committee |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
