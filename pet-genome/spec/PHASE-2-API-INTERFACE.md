# WIA-PET-002: Pet Genome API Interface Specification
## Phase 2: API Interface Standards

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. Introduction

### 1.1 Purpose
This specification defines RESTful API interfaces for pet genomic data services, enabling standardized communication between genomic testing laboratories, veterinary clinics, breeding organizations, and pet owner applications. The API supports secure data exchange, real-time analysis requests, and integration with existing veterinary and breeding management systems.

### 1.2 Scope
This standard covers:
- RESTful API endpoints for genomic data operations
- Authentication and authorization mechanisms
- Request and response data formats
- Webhook notifications for asynchronous processing
- Rate limiting and quota management
- Error handling and status codes
- Batch processing operations
- Real-time streaming interfaces
- Third-party integration patterns

### 1.3 Base URL Structure

```
Production: https://api.pet-genome.wia.org/v1
Staging: https://api-staging.pet-genome.wia.org/v1
Sandbox: https://api-sandbox.pet-genome.wia.org/v1
```

### 1.4 API Versioning Strategy

- **URL-based versioning**: `/v1/`, `/v2/`
- **Backward compatibility**: Maintained for 24 months
- **Deprecation notice**: 6 months minimum before removal
- **Version migration guides**: Provided for breaking changes

---

## 2. Authentication and Authorization

### 2.1 Authentication Methods

#### OAuth 2.0 Flow

```http
POST /oauth/token HTTP/1.1
Host: api.pet-genome.wia.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=genomic:read genomic:write breed:analyze
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "genomic:read genomic:write breed:analyze",
  "refresh_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6..."
}
```

#### API Key Authentication

```http
GET /v1/profiles/PGP-ABCD12345678 HTTP/1.1
Host: api.pet-genome.wia.org
X-API-Key: pgp_live_1234567890abcdef
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 2.2 Authorization Scopes

| Scope | Description | Access Level |
|-------|-------------|--------------|
| `genomic:read` | Read genomic profiles and variants | Read-only |
| `genomic:write` | Create and update genomic data | Write |
| `genomic:delete` | Delete genomic profiles | Delete |
| `breed:analyze` | Access breed identification services | Execute |
| `health:read` | Read health markers and disease risks | Read-only |
| `health:write` | Update health marker interpretations | Write |
| `ancestry:compute` | Perform ancestry analysis | Execute |
| `research:contribute` | Submit data to research databases | Write |
| `admin:manage` | Administrative operations | Admin |

### 2.3 Security Headers

```http
X-API-Key: pgp_live_1234567890abcdef
Authorization: Bearer {access_token}
X-Request-ID: req_1234567890abcdef
X-Idempotency-Key: idem_1234567890abcdef
X-Client-Version: PetGenomeSDK/1.2.3
```

---

## 3. Core API Endpoints

### 3.1 Genomic Profile Management

#### Create Genomic Profile

```http
POST /v1/profiles HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "speciesCode": "CANIS_FAMILIARIS",
  "petIdentification": {
    "registeredName": "Champion Golden Star",
    "microchipId": "123456789012345",
    "birthDate": "2023-05-15",
    "sex": "MALE"
  },
  "sampleMetadata": {
    "sampleId": "SAMPLE-2025-001234",
    "collectionDate": "2025-12-15T10:30:00Z",
    "sampleType": "BUCCAL_SWAB",
    "laboratory": {
      "name": "VetGen Laboratories",
      "certifications": ["ISO 17025", "AAHA Certified"]
    }
  },
  "privacyConsent": {
    "ownerConsent": true,
    "researchParticipation": false,
    "dataSharing": {
      "allowBreedingDatabase": true,
      "allowResearch": false
    }
  }
}
```

**Response (201 Created):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "status": "CREATED",
  "createdAt": "2025-12-18T14:30:00Z",
  "uploadUrl": {
    "vcfFile": "https://upload.pet-genome.wia.org/vcf/PGP-ABCD12345678?token=...",
    "bamFile": "https://upload.pet-genome.wia.org/bam/PGP-ABCD12345678?token=...",
    "expiresAt": "2025-12-18T18:30:00Z"
  },
  "webhookUrl": "https://api.pet-genome.wia.org/v1/webhooks/profiles/PGP-ABCD12345678",
  "_links": {
    "self": "/v1/profiles/PGP-ABCD12345678",
    "upload": "/v1/profiles/PGP-ABCD12345678/upload",
    "analysis": "/v1/profiles/PGP-ABCD12345678/analyze"
  }
}
```

#### Retrieve Genomic Profile

```http
GET /v1/profiles/{profileId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Accept: application/json
```

**Response (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "version": "1.0.0",
  "speciesCode": "CANIS_FAMILIARIS",
  "petIdentification": {
    "registeredName": "Champion Golden Star",
    "microchipId": "123456789012345",
    "birthDate": "2023-05-15",
    "sex": "MALE",
    "neutered": false
  },
  "genomicData": {
    "sequencingMethod": "WHOLE_GENOME_SEQUENCING",
    "referenceGenome": {
      "assembly": "CanFam4",
      "version": "GCA_011100685.1"
    },
    "coverage": {
      "meanDepth": 32.5,
      "percentageAbove30x": 95.2
    },
    "variants": {
      "totalVariants": 3245678,
      "snpCount": 3102456,
      "indelCount": 143222,
      "vcfFileUri": "https://storage.pet-genome.wia.org/vcf/PGP-ABCD12345678.vcf.gz"
    }
  },
  "breedInformation": {
    "primaryBreed": {
      "breedName": "Labrador Retriever",
      "breedCode": "AKC-122",
      "percentage": 87.5,
      "confidence": 0.96
    },
    "ancestryComposition": [
      {
        "breedName": "Labrador Retriever",
        "percentage": 87.5,
        "generationsBack": 1
      },
      {
        "breedName": "Golden Retriever",
        "percentage": 12.5,
        "generationsBack": 3
      }
    ]
  },
  "healthMarkers": {
    "geneticDiseaseRisks": [
      {
        "diseaseName": "Progressive Retinal Atrophy",
        "diseaseCode": "OMIA-001424",
        "riskLevel": "CARRIER",
        "inheritancePattern": "AUTOSOMAL_RECESSIVE",
        "affectedGenes": ["PRCD"]
      }
    ],
    "pharmacogenomics": [
      {
        "drugName": "Ivermectin",
        "gene": "ABCB1",
        "responseType": "NORMAL_METABOLIZER"
      }
    ]
  },
  "qualityMetrics": {
    "overallQualityScore": 96.5,
    "contaminationCheck": true,
    "sexConsistency": true,
    "callRate": 0.992
  },
  "lastUpdated": "2025-12-18T14:30:00Z"
}
```

#### Update Genomic Profile

```http
PATCH /v1/profiles/{profileId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "petIdentification": {
    "neutered": true
  },
  "privacyConsent": {
    "researchParticipation": true
  }
}
```

**Response (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "status": "UPDATED",
  "updatedAt": "2025-12-18T15:00:00Z",
  "updatedFields": ["petIdentification.neutered", "privacyConsent.researchParticipation"]
}
```

#### Delete Genomic Profile

```http
DELETE /v1/profiles/{profileId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
X-Confirmation-Token: CONFIRM-DELETE-PGP-ABCD12345678
```

**Response (204 No Content)**

### 3.2 Variant Analysis Endpoints

#### Query Variants

```http
GET /v1/profiles/{profileId}/variants?gene=PRCD&significance=PATHOGENIC HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**Query Parameters:**
- `chromosome`: Filter by chromosome (e.g., `chr1`, `chrX`)
- `startPosition`: Start position (integer)
- `endPosition`: End position (integer)
- `gene`: Gene symbol
- `significance`: Variant significance (PATHOGENIC, BENIGN, etc.)
- `type`: Variant type (SNP, INDEL, SV)
- `limit`: Results per page (default: 100, max: 1000)
- `offset`: Pagination offset

**Response (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "query": {
    "gene": "PRCD",
    "significance": "PATHOGENIC"
  },
  "totalResults": 1,
  "variants": [
    {
      "variantId": "var_prcd_c5g_a",
      "chromosome": "chr9",
      "position": 69632163,
      "referenceAllele": "G",
      "alternateAllele": "A",
      "genotype": "0/1",
      "significance": "PATHOGENIC",
      "gene": "PRCD",
      "transcript": "ENSCAFT00000012345",
      "consequence": "MISSENSE",
      "aminoAcidChange": "p.Cys2Tyr",
      "associatedConditions": ["Progressive Retinal Atrophy"],
      "omiaId": "OMIA-001424",
      "alleleFrequency": {
        "overall": 0.023,
        "breedSpecific": {
          "Labrador Retriever": 0.045,
          "Cocker Spaniel": 0.078
        }
      },
      "clinicalInterpretation": {
        "zygosity": "HETEROZYGOUS",
        "clinicalImpact": "CARRIER",
        "recommendation": "Pre-breeding genetic counseling recommended"
      }
    }
  ],
  "pagination": {
    "limit": 100,
    "offset": 0,
    "hasMore": false
  }
}
```

#### Annotate Custom Variants

```http
POST /v1/variants/annotate HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "speciesCode": "CANIS_FAMILIARIS",
  "referenceGenome": "CanFam4",
  "variants": [
    {
      "chromosome": "chr9",
      "position": 69632163,
      "referenceAllele": "G",
      "alternateAllele": "A"
    },
    {
      "chromosome": "chr14",
      "position": 23515518,
      "referenceAllele": "ATAG",
      "alternateAllele": "-"
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "annotatedVariants": [
    {
      "inputVariant": {
        "chromosome": "chr9",
        "position": 69632163,
        "referenceAllele": "G",
        "alternateAllele": "A"
      },
      "annotations": {
        "gene": "PRCD",
        "consequence": "MISSENSE",
        "significance": "PATHOGENIC",
        "omiaId": "OMIA-001424",
        "affectedDisease": "Progressive Retinal Atrophy",
        "breedPrevalence": {
          "Labrador Retriever": 0.045
        }
      }
    },
    {
      "inputVariant": {
        "chromosome": "chr14",
        "position": 23515518,
        "referenceAllele": "ATAG",
        "alternateAllele": "-"
      },
      "annotations": {
        "gene": "ABCB1",
        "consequence": "FRAMESHIFT",
        "significance": "PATHOGENIC",
        "omiaId": "OMIA-000347",
        "affectedDisease": "Multidrug Resistance (MDR1)",
        "pharmacogenomicImpact": [
          "Ivermectin sensitivity",
          "Loperamide sensitivity"
        ]
      }
    }
  ]
}
```

### 3.3 Breed Analysis Endpoints

#### Analyze Breed Composition

```http
POST /v1/profiles/{profileId}/breed/analyze HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "analysisType": "COMPREHENSIVE",
  "includeAncestry": true,
  "includeHaplotypes": true,
  "kBreeds": 5
}
```

**Response (202 Accepted):**
```json
{
  "analysisId": "BREED-ANALYSIS-001234",
  "profileId": "PGP-ABCD12345678",
  "status": "PROCESSING",
  "estimatedCompletionTime": "2025-12-18T14:45:00Z",
  "webhookUrl": "https://your-app.com/webhooks/breed-analysis",
  "_links": {
    "status": "/v1/profiles/PGP-ABCD12345678/breed/analyze/BREED-ANALYSIS-001234",
    "cancel": "/v1/profiles/PGP-ABCD12345678/breed/analyze/BREED-ANALYSIS-001234/cancel"
  }
}
```

#### Get Breed Analysis Results

```http
GET /v1/profiles/{profileId}/breed/analyze/{analysisId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "analysisId": "BREED-ANALYSIS-001234",
  "profileId": "PGP-ABCD12345678",
  "status": "COMPLETED",
  "completedAt": "2025-12-18T14:42:30Z",
  "results": {
    "breedComposition": [
      {
        "breedName": "Labrador Retriever",
        "breedCode": "AKC-122",
        "percentage": 62.5,
        "confidence": 0.95,
        "chromosomalContribution": {
          "autosomal": 62.8,
          "xChromosome": 61.2
        }
      },
      {
        "breedName": "Golden Retriever",
        "breedCode": "AKC-111",
        "percentage": 25.0,
        "confidence": 0.92
      },
      {
        "breedName": "Chesapeake Bay Retriever",
        "breedCode": "AKC-123",
        "percentage": 12.5,
        "confidence": 0.78
      }
    ],
    "ancestryTimeline": [
      {
        "generation": 1,
        "estimatedYear": "2020-2021",
        "dominantBreed": "Labrador Retriever",
        "percentage": 62.5
      },
      {
        "generation": 2,
        "estimatedYear": "2017-2018",
        "breeds": [
          {"breedName": "Labrador Retriever", "percentage": 50},
          {"breedName": "Golden Retriever", "percentage": 50}
        ]
      }
    ],
    "haplotypeAnalysis": {
      "maternalHaplogroup": "H1a",
      "paternalHaplogroup": "Y2b",
      "mitochondrialOrigin": "Western European lineage"
    },
    "geneticDiversity": {
      "heterozygosity": 0.342,
      "inbreedingCoefficient": 0.023,
      "effectivePopulationSize": 450,
      "interpretation": "Healthy genetic diversity"
    }
  }
}
```

#### Validate Breed Claim

```http
POST /v1/profiles/{profileId}/breed/validate HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "claimedBreed": "Labrador Retriever",
  "registrationNumber": "AKC-SR-12345678",
  "requirePurebred": true,
  "purebreadThreshold": 87.5
}
```

**Response (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "claimedBreed": "Labrador Retriever",
  "validated": true,
  "geneticBreedPercentage": 92.3,
  "purebreadThreshold": 87.5,
  "confidence": 0.96,
  "matchQuality": "EXCELLENT",
  "recommendation": "Breed claim validated",
  "registrationEligibility": {
    "akc": true,
    "fci": true,
    "notes": "Meets purebred standards for registration"
  }
}
```

### 3.4 Health Analysis Endpoints

#### Analyze Disease Risk

```http
POST /v1/profiles/{profileId}/health/analyze HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "analysisScope": "COMPREHENSIVE",
  "includeCarrierStatus": true,
  "includePharmacogenomics": true,
  "specificDiseases": ["OMIA-001424", "OMIA-000347"]
}
```

**Response (200 OK):**
```json
{
  "analysisId": "HEALTH-ANALYSIS-001234",
  "profileId": "PGP-ABCD12345678",
  "analysisDate": "2025-12-18T14:30:00Z",
  "results": {
    "overallHealthScore": 85,
    "geneticDiseaseRisks": [
      {
        "diseaseName": "Progressive Retinal Atrophy",
        "diseaseCode": "OMIA-001424",
        "riskLevel": "CARRIER",
        "inheritancePattern": "AUTOSOMAL_RECESSIVE",
        "affectedGenes": ["PRCD"],
        "genotype": "c.5G>A (0/1)",
        "penetrance": 1.0,
        "recommendation": "Not affected; carrier status - genetic counseling for breeding",
        "breedPrevalence": 0.045
      }
    ],
    "pharmacogenomics": [
      {
        "drugName": "Ivermectin",
        "gene": "ABCB1",
        "variant": "c.227_230delATAG",
        "genotype": "0/0",
        "responseType": "NORMAL_METABOLIZER",
        "clinicalRecommendation": "Standard dosing safe",
        "adverseReactionRisk": "LOW"
      }
    ],
    "traits": [
      {
        "traitName": "Coat Color",
        "category": "COAT_COLOR",
        "prediction": "Yellow/Golden",
        "confidence": 0.98,
        "genotypicBasis": "ee genotype at MC1R"
      },
      {
        "traitName": "Exercise-Induced Collapse Susceptibility",
        "category": "PERFORMANCE",
        "prediction": "Not susceptible",
        "confidence": 0.99,
        "genotypicBasis": "Clear for DNM1 variant"
      }
    ]
  },
  "breedSpecificRisks": {
    "breed": "Labrador Retriever",
    "commonConditions": [
      {
        "condition": "Hip Dysplasia",
        "geneticComponent": "POLYGENIC",
        "riskScore": 0.42,
        "recommendation": "Monitor and maintain healthy weight"
      }
    ]
  }
}
```

#### Get Pharmacogenomic Report

```http
GET /v1/profiles/{profileId}/pharmacogenomics HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "reportDate": "2025-12-18T14:30:00Z",
  "drugResponses": [
    {
      "drugName": "Ivermectin",
      "drugClass": "Antiparasitic",
      "gene": "ABCB1",
      "variant": "c.227_230delATAG",
      "genotype": "0/0",
      "phenotype": "NORMAL_METABOLIZER",
      "recommendation": "STANDARD_DOSING",
      "safetyLevel": "SAFE",
      "alternativeDrugs": []
    },
    {
      "drugName": "Acepromazine",
      "drugClass": "Sedative",
      "gene": "ABCB1",
      "variant": "c.227_230delATAG",
      "genotype": "0/0",
      "phenotype": "NORMAL_METABOLIZER",
      "recommendation": "STANDARD_DOSING",
      "safetyLevel": "SAFE"
    }
  ],
  "contraindications": [],
  "dosageAdjustments": [],
  "veterinaryGuidance": "No pharmacogenomic contraindications identified. Standard drug protocols appropriate."
}
```

### 3.5 Ancestry and Lineage Endpoints

#### Calculate Relatedness

```http
POST /v1/relatedness/calculate HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "profileId1": "PGP-ABCD12345678",
  "profileId2": "PGP-EFGH87654321",
  "method": "KINSHIP_COEFFICIENT"
}
```

**Response (200 OK):**
```json
{
  "profileId1": "PGP-ABCD12345678",
  "profileId2": "PGP-EFGH87654321",
  "kinshipCoefficient": 0.25,
  "relationship": "HALF_SIBLINGS",
  "confidence": 0.94,
  "sharedSegments": {
    "totalLength": 875000000,
    "numberOfSegments": 23,
    "longestSegment": 125000000
  },
  "interpretation": "Likely half-siblings (share one parent)",
  "alternativeRelationships": [
    {
      "relationship": "GRANDPARENT_GRANDCHILD",
      "probability": 0.05
    },
    {
      "relationship": "AUNT_UNCLE_NIECE_NEPHEW",
      "probability": 0.03
    }
  ]
}
```

#### Build Pedigree

```http
POST /v1/profiles/{profileId}/pedigree HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "generations": 3,
  "includeGenotypes": false,
  "inferMissingRelationships": true
}
```

**Response (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "pedigreeId": "PED-2025-001234",
  "generations": 3,
  "individuals": [
    {
      "id": "PGP-ABCD12345678",
      "name": "Champion Golden Star",
      "sex": "MALE",
      "generation": 0,
      "inbreedingCoefficient": 0.023
    },
    {
      "id": "PGP-SIRE-001",
      "name": "Grand Champion Sire",
      "sex": "MALE",
      "generation": 1,
      "relationship": "FATHER",
      "confidence": 0.99
    },
    {
      "id": "PGP-DAM-001",
      "name": "Champion Dam",
      "sex": "FEMALE",
      "generation": 1,
      "relationship": "MOTHER",
      "confidence": 0.99
    }
  ],
  "relationships": [
    {
      "individual1": "PGP-ABCD12345678",
      "individual2": "PGP-SIRE-001",
      "relationship": "OFFSPRING_PARENT",
      "confidence": 0.99
    }
  ],
  "pedigreeCoefficient": 0.023,
  "foundersCount": 8
}
```

### 3.6 Research and Population Endpoints

#### Contribute to Research Database

```http
POST /v1/research/contribute HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "profileId": "PGP-ABCD12345678",
  "studyId": "STUDY-CANINE-HEALTH-2025",
  "anonymize": true,
  "includeRawData": false,
  "consentConfirmation": "CONSENT-2025-12-18-001"
}
```

**Response (201 Created):**
```json
{
  "contributionId": "CONTRIB-001234",
  "studyId": "STUDY-CANINE-HEALTH-2025",
  "anonymizedId": "ANON-987654321",
  "status": "ACCEPTED",
  "dataShared": ["genotypes", "breed_info", "health_markers"],
  "acknowledgment": "Thank you for contributing to canine health research"
}
```

#### Query Population Statistics

```http
GET /v1/population/statistics?species=CANIS_FAMILIARIS&breed=Labrador%20Retriever HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "species": "CANIS_FAMILIARIS",
  "breed": "Labrador Retriever",
  "sampleSize": 15234,
  "statistics": {
    "averageHeterozygosity": 0.345,
    "averageInbreedingCoefficient": 0.031,
    "effectivePopulationSize": 425,
    "commonDiseaseVariants": [
      {
        "disease": "Progressive Retinal Atrophy",
        "variantId": "PRCD_c.5G>A",
        "carrierFrequency": 0.045,
        "affectedFrequency": 0.002
      },
      {
        "disease": "Exercise-Induced Collapse",
        "variantId": "DNM1_c.767G>T",
        "carrierFrequency": 0.032,
        "affectedFrequency": 0.001
      }
    ],
    "geneticDiversityTrend": {
      "2020": 0.352,
      "2021": 0.348,
      "2022": 0.346,
      "2023": 0.345,
      "2024": 0.345,
      "2025": 0.345
    }
  }
}
```

### 3.7 File Upload and Processing

#### Upload Genomic Data File

```http
POST /v1/profiles/{profileId}/upload HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: multipart/form-data

--boundary
Content-Disposition: form-data; name="fileType"

VCF
--boundary
Content-Disposition: form-data; name="file"; filename="sample.vcf.gz"
Content-Type: application/gzip

[Binary VCF file content]
--boundary--
```

**Response (202 Accepted):**
```json
{
  "uploadId": "UPLOAD-001234",
  "profileId": "PGP-ABCD12345678",
  "fileType": "VCF",
  "fileSize": 245678901,
  "status": "PROCESSING",
  "checksumMd5": "5d41402abc4b2a76b9719d911017c592",
  "estimatedProcessingTime": "PT15M",
  "_links": {
    "status": "/v1/profiles/PGP-ABCD12345678/upload/UPLOAD-001234"
  }
}
```

#### Check Upload Status

```http
GET /v1/profiles/{profileId}/upload/{uploadId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "uploadId": "UPLOAD-001234",
  "profileId": "PGP-ABCD12345678",
  "status": "COMPLETED",
  "processingSteps": [
    {
      "step": "FILE_VALIDATION",
      "status": "COMPLETED",
      "completedAt": "2025-12-18T14:31:00Z"
    },
    {
      "step": "VARIANT_EXTRACTION",
      "status": "COMPLETED",
      "completedAt": "2025-12-18T14:38:00Z",
      "variantsExtracted": 3245678
    },
    {
      "step": "QUALITY_CONTROL",
      "status": "COMPLETED",
      "completedAt": "2025-12-18T14:40:00Z",
      "qualityScore": 96.5
    },
    {
      "step": "ANNOTATION",
      "status": "COMPLETED",
      "completedAt": "2025-12-18T14:45:00Z",
      "annotatedVariants": 3245678
    }
  ],
  "completedAt": "2025-12-18T14:45:00Z"
}
```

### 3.8 Batch Operations

#### Batch Profile Creation

```http
POST /v1/batch/profiles HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "batchId": "BATCH-2025-12-18-001",
  "profiles": [
    {
      "speciesCode": "CANIS_FAMILIARIS",
      "petIdentification": {
        "registeredName": "Pet 1",
        "microchipId": "111111111111111"
      }
    },
    {
      "speciesCode": "FELIS_CATUS",
      "petIdentification": {
        "registeredName": "Pet 2",
        "microchipId": "222222222222222"
      }
    }
  ]
}
```

**Response (202 Accepted):**
```json
{
  "batchId": "BATCH-2025-12-18-001",
  "totalProfiles": 2,
  "status": "PROCESSING",
  "createdProfiles": [
    {
      "profileId": "PGP-BATCH001-001",
      "microchipId": "111111111111111",
      "status": "CREATED"
    },
    {
      "profileId": "PGP-BATCH001-002",
      "microchipId": "222222222222222",
      "status": "CREATED"
    }
  ],
  "_links": {
    "status": "/v1/batch/profiles/BATCH-2025-12-18-001"
  }
}
```

### 3.9 Webhook Notifications

#### Register Webhook

```http
POST /v1/webhooks HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://your-app.com/webhooks/pet-genome",
  "events": [
    "profile.created",
    "profile.updated",
    "analysis.completed",
    "upload.completed",
    "health.alert"
  ],
  "secret": "whsec_1234567890abcdef",
  "active": true
}
```

**Response (201 Created):**
```json
{
  "webhookId": "WEBHOOK-001234",
  "url": "https://your-app.com/webhooks/pet-genome",
  "events": [
    "profile.created",
    "profile.updated",
    "analysis.completed",
    "upload.completed",
    "health.alert"
  ],
  "active": true,
  "createdAt": "2025-12-18T14:30:00Z"
}
```

#### Webhook Payload Example

```json
{
  "event": "analysis.completed",
  "timestamp": "2025-12-18T14:45:00Z",
  "webhookId": "WEBHOOK-001234",
  "data": {
    "analysisId": "BREED-ANALYSIS-001234",
    "profileId": "PGP-ABCD12345678",
    "analysisType": "BREED_COMPOSITION",
    "status": "COMPLETED",
    "resultsUrl": "https://api.pet-genome.wia.org/v1/profiles/PGP-ABCD12345678/breed/analyze/BREED-ANALYSIS-001234"
  },
  "signature": "sha256=5d41402abc4b2a76b9719d911017c592"
}
```

### 3.10 Export and Reporting

#### Generate Veterinary Report

```http
POST /v1/profiles/{profileId}/reports/veterinary HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "format": "PDF",
  "sections": [
    "BREED_IDENTIFICATION",
    "HEALTH_SCREENING",
    "PHARMACOGENOMICS",
    "RECOMMENDATIONS"
  ],
  "includeVisualizations": true,
  "language": "en"
}
```

**Response (200 OK):**
```json
{
  "reportId": "REPORT-VET-001234",
  "profileId": "PGP-ABCD12345678",
  "format": "PDF",
  "status": "GENERATED",
  "downloadUrl": "https://reports.pet-genome.wia.org/vet/REPORT-VET-001234.pdf",
  "expiresAt": "2025-12-25T14:30:00Z",
  "fileSize": 2456789
}
```

#### Export to Third-Party Format

```http
POST /v1/profiles/{profileId}/export HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "format": "PEDIGREE_SOFTWARE",
  "targetSystem": "BREEDER_PRO_V3",
  "includeGenotypes": true
}
```

**Response (200 OK):**
```json
{
  "exportId": "EXPORT-001234",
  "format": "PEDIGREE_SOFTWARE",
  "downloadUrl": "https://exports.pet-genome.wia.org/EXPORT-001234.xml",
  "expiresAt": "2025-12-25T14:30:00Z"
}
```

---

## 4. Error Handling

### 4.1 Standard Error Response

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "The request is missing required field: speciesCode",
    "details": {
      "field": "speciesCode",
      "reason": "Field is required but was not provided"
    },
    "requestId": "req_1234567890abcdef",
    "timestamp": "2025-12-18T14:30:00Z",
    "documentation": "https://docs.pet-genome.wia.org/errors/INVALID_REQUEST"
  }
}
```

### 4.2 Error Codes

| HTTP Status | Error Code | Description | Retry |
|-------------|------------|-------------|-------|
| 400 | INVALID_REQUEST | Malformed request or missing required fields | No |
| 401 | UNAUTHORIZED | Invalid or missing authentication credentials | No |
| 403 | FORBIDDEN | Insufficient permissions for requested operation | No |
| 404 | NOT_FOUND | Requested resource does not exist | No |
| 409 | CONFLICT | Resource already exists or state conflict | No |
| 422 | VALIDATION_ERROR | Request validation failed | No |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests | Yes |
| 500 | INTERNAL_ERROR | Server error | Yes |
| 503 | SERVICE_UNAVAILABLE | Service temporarily unavailable | Yes |

---

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1703001600
X-RateLimit-Window: 3600
```

### 5.2 Rate Limit Tiers

| Tier | Requests/Hour | Requests/Day | Concurrent Uploads |
|------|---------------|--------------|-------------------|
| Free | 100 | 1,000 | 1 |
| Basic | 1,000 | 10,000 | 5 |
| Professional | 10,000 | 100,000 | 20 |
| Enterprise | Custom | Custom | Custom |

---

## 6. Code Examples

### 6.1 Python SDK Example

```python
from pet_genome_sdk import PetGenomeClient
from pet_genome_sdk.models import GenomicProfile, BreedAnalysis

# Initialize client
client = PetGenomeClient(
    api_key='pgp_live_1234567890abcdef',
    environment='production'
)

# Create genomic profile
profile = client.profiles.create(
    species_code='CANIS_FAMILIARIS',
    pet_identification={
        'registered_name': 'Champion Golden Star',
        'microchip_id': '123456789012345',
        'birth_date': '2023-05-15',
        'sex': 'MALE'
    },
    sample_metadata={
        'sample_id': 'SAMPLE-2025-001234',
        'collection_date': '2025-12-15T10:30:00Z',
        'sample_type': 'BUCCAL_SWAB'
    }
)

print(f"Created profile: {profile.profile_id}")

# Upload VCF file
with open('sample.vcf.gz', 'rb') as vcf_file:
    upload = client.profiles.upload_file(
        profile_id=profile.profile_id,
        file_type='VCF',
        file=vcf_file
    )

# Wait for processing
upload.wait_for_completion(timeout=900)  # 15 minutes

# Analyze breed composition
breed_analysis = client.breed.analyze(
    profile_id=profile.profile_id,
    analysis_type='COMPREHENSIVE',
    include_ancestry=True
)

# Wait for results
results = breed_analysis.wait_for_results()

# Print breed composition
for breed in results.breed_composition:
    print(f"{breed.breed_name}: {breed.percentage}%")

# Get health analysis
health = client.health.analyze(
    profile_id=profile.profile_id,
    analysis_scope='COMPREHENSIVE'
)

# Check for pathogenic variants
for disease in health.genetic_disease_risks:
    if disease.risk_level in ['AFFECTED', 'HIGH']:
        print(f"WARNING: {disease.disease_name} - {disease.risk_level}")

# Calculate relatedness
relatedness = client.relatedness.calculate(
    profile_id1=profile.profile_id,
    profile_id2='PGP-EFGH87654321'
)

print(f"Relationship: {relatedness.relationship}")
print(f"Kinship coefficient: {relatedness.kinship_coefficient}")

# Generate veterinary report
report = client.reports.generate_veterinary(
    profile_id=profile.profile_id,
    format='PDF',
    sections=['BREED_IDENTIFICATION', 'HEALTH_SCREENING']
)

# Download report
report.download('veterinary_report.pdf')
```

### 6.2 JavaScript/Node.js SDK Example

```javascript
const { PetGenomeClient } = require('@wia/pet-genome-sdk');

// Initialize client
const client = new PetGenomeClient({
  apiKey: 'pgp_live_1234567890abcdef',
  environment: 'production'
});

// Create genomic profile
async function analyzeePetGenome() {
  try {
    // Create profile
    const profile = await client.profiles.create({
      speciesCode: 'CANIS_FAMILIARIS',
      petIdentification: {
        registeredName: 'Champion Golden Star',
        microchipId: '123456789012345',
        birthDate: '2023-05-15',
        sex: 'MALE'
      },
      sampleMetadata: {
        sampleId: 'SAMPLE-2025-001234',
        collectionDate: '2025-12-15T10:30:00Z',
        sampleType: 'BUCCAL_SWAB'
      }
    });

    console.log(`Created profile: ${profile.profileId}`);

    // Upload VCF file
    const fs = require('fs');
    const vcfStream = fs.createReadStream('sample.vcf.gz');

    const upload = await client.profiles.uploadFile({
      profileId: profile.profileId,
      fileType: 'VCF',
      file: vcfStream
    });

    // Wait for processing
    await upload.waitForCompletion({ timeout: 900000 }); // 15 minutes

    // Analyze breed composition
    const breedAnalysis = await client.breed.analyze({
      profileId: profile.profileId,
      analysisType: 'COMPREHENSIVE',
      includeAncestry: true
    });

    // Get results
    const results = await breedAnalysis.getResults();

    // Print breed composition
    results.breedComposition.forEach(breed => {
      console.log(`${breed.breedName}: ${breed.percentage}%`);
    });

    // Health analysis
    const health = await client.health.analyze({
      profileId: profile.profileId,
      analysisScope: 'COMPREHENSIVE'
    });

    // Check for high-risk conditions
    health.geneticDiseaseRisks
      .filter(d => ['AFFECTED', 'HIGH'].includes(d.riskLevel))
      .forEach(disease => {
        console.log(`WARNING: ${disease.diseaseName} - ${disease.riskLevel}`);
      });

    // Generate report
    const report = await client.reports.generateVeterinary({
      profileId: profile.profileId,
      format: 'PDF',
      sections: ['BREED_IDENTIFICATION', 'HEALTH_SCREENING']
    });

    console.log(`Report available: ${report.downloadUrl}`);

  } catch (error) {
    console.error('Error:', error.message);
    if (error.code === 'RATE_LIMIT_EXCEEDED') {
      const retryAfter = error.retryAfter;
      console.log(`Rate limited. Retry after ${retryAfter} seconds`);
    }
  }
}

analyzePetGenome();
```

### 6.3 Webhook Handler Example

```python
from flask import Flask, request, jsonify
import hmac
import hashlib

app = Flask(__name__)

WEBHOOK_SECRET = 'whsec_1234567890abcdef'

def verify_webhook_signature(payload, signature):
    """Verify webhook signature"""
    expected_signature = hmac.new(
        WEBHOOK_SECRET.encode(),
        payload,
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(
        f"sha256={expected_signature}",
        signature
    )

@app.route('/webhooks/pet-genome', methods=['POST'])
def handle_webhook():
    # Get signature from header
    signature = request.headers.get('X-Webhook-Signature')

    # Get raw payload
    payload = request.get_data()

    # Verify signature
    if not verify_webhook_signature(payload, signature):
        return jsonify({'error': 'Invalid signature'}), 401

    # Parse event
    event = request.json

    # Handle different event types
    if event['event'] == 'analysis.completed':
        handle_analysis_completed(event['data'])
    elif event['event'] == 'profile.created':
        handle_profile_created(event['data'])
    elif event['event'] == 'health.alert':
        handle_health_alert(event['data'])

    return jsonify({'status': 'received'}), 200

def handle_analysis_completed(data):
    """Handle analysis completion"""
    profile_id = data['profileId']
    analysis_id = data['analysisId']

    print(f"Analysis {analysis_id} completed for profile {profile_id}")

    # Fetch results
    # Send notification to user
    # Update database
    pass

def handle_profile_created(data):
    """Handle profile creation"""
    profile_id = data['profileId']
    print(f"New profile created: {profile_id}")
    pass

def handle_health_alert(data):
    """Handle health alert"""
    profile_id = data['profileId']
    alert_type = data['alertType']
    severity = data['severity']

    print(f"Health alert for {profile_id}: {alert_type} (Severity: {severity})")

    # Send urgent notification to veterinarian
    # Log alert
    pass

if __name__ == '__main__':
    app.run(port=3000)
```

### 6.4 Batch Processing Example

```python
import asyncio
from pet_genome_sdk import AsyncPetGenomeClient

async def process_batch_profiles(microchip_ids):
    """Process multiple profiles concurrently"""
    client = AsyncPetGenomeClient(
        api_key='pgp_live_1234567890abcdef'
    )

    async def process_single_profile(microchip_id):
        try:
            # Create profile
            profile = await client.profiles.create(
                species_code='CANIS_FAMILIARIS',
                pet_identification={
                    'microchip_id': microchip_id
                }
            )

            # Upload VCF
            vcf_path = f'data/{microchip_id}.vcf.gz'
            upload = await client.profiles.upload_file(
                profile_id=profile.profile_id,
                file_type='VCF',
                file_path=vcf_path
            )

            # Wait for processing
            await upload.wait_for_completion()

            # Analyze
            breed_analysis = await client.breed.analyze(
                profile_id=profile.profile_id
            )

            results = await breed_analysis.get_results()

            return {
                'microchip_id': microchip_id,
                'profile_id': profile.profile_id,
                'primary_breed': results.breed_composition[0].breed_name,
                'status': 'SUCCESS'
            }

        except Exception as e:
            return {
                'microchip_id': microchip_id,
                'status': 'ERROR',
                'error': str(e)
            }

    # Process all profiles concurrently
    tasks = [process_single_profile(mid) for mid in microchip_ids]
    results = await asyncio.gather(*tasks)

    # Summary
    success_count = sum(1 for r in results if r['status'] == 'SUCCESS')
    print(f"Processed {len(results)} profiles: {success_count} successful")

    return results

# Run batch processing
microchip_ids = ['123456789012345', '234567890123456', '345678901234567']
results = asyncio.run(process_batch_profiles(microchip_ids))

for result in results:
    print(f"{result['microchip_id']}: {result['status']}")
```

### 6.5 Real-time Variant Streaming

```python
from pet_genome_sdk import PetGenomeClient

client = PetGenomeClient(api_key='pgp_live_1234567890abcdef')

# Stream pathogenic variants
def stream_pathogenic_variants(profile_id):
    """Stream pathogenic variants as they are identified"""

    # Open streaming connection
    stream = client.variants.stream_pathogenic(
        profile_id=profile_id,
        significance_filter=['PATHOGENIC', 'LIKELY_PATHOGENIC']
    )

    print(f"Streaming pathogenic variants for {profile_id}...")

    for variant in stream:
        print(f"\nPathogenic variant detected:")
        print(f"  Gene: {variant.gene}")
        print(f"  Variant: {variant.chromosome}:{variant.position}")
        print(f"  Significance: {variant.significance}")
        print(f"  Associated disease: {variant.associated_conditions}")

        # Send alert
        send_veterinary_alert(profile_id, variant)

    print("Streaming completed")

def send_veterinary_alert(profile_id, variant):
    """Send alert to veterinarian"""
    # Implementation for sending alerts
    pass

# Start streaming
stream_pathogenic_variants('PGP-ABCD12345678')
```

---

## 7. Pagination

### 7.1 Cursor-based Pagination

```http
GET /v1/profiles?limit=100&cursor=eyJpZCI6IlBHUC0xMjM0NTY3OCJ9 HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "limit": 100,
    "hasMore": true,
    "nextCursor": "eyJpZCI6IlBHUC05ODc2NTQzMiJ9",
    "previousCursor": null
  }
}
```

---

## 8. API Versioning and Deprecation

### 8.1 Deprecation Headers

```http
HTTP/1.1 200 OK
X-API-Deprecated: true
X-API-Sunset-Date: 2026-06-18
X-API-Deprecation-Info: https://docs.pet-genome.wia.org/deprecations/v1-profiles-endpoint
Link: </v2/profiles>; rel="successor-version"
```

---

## 9. Service Level Agreement (SLA)

| Metric | Target | Measurement |
|--------|--------|-------------|
| API Uptime | 99.9% | Monthly |
| Response Time (p95) | < 500ms | Per endpoint |
| Response Time (p99) | < 1000ms | Per endpoint |
| Data Processing Time | < 30min | Per 30x WGS |
| Support Response | < 4 hours | Business hours |

---

## 10. Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | Initial API specification | WIA Standards Committee |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
