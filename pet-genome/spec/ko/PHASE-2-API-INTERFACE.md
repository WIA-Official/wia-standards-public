# WIA-PET-002: Pet Genome API 인터페이스 명세서
## Phase 2: API 인터페이스 표준

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. 소개

### 1.1 목적
본 명세서는 반려동물 유전체 데이터 서비스를 위한 RESTful API 인터페이스를 정의하여, 유전체 검사 연구소, 수의 클리닉, 번식 조직 및 반려동물 소유자 애플리케이션 간의 표준화된 통신을 가능하게 합니다. API는 안전한 데이터 교환, 실시간 분석 요청, 기존 수의학 및 번식 관리 시스템과의 통합을 지원합니다.

### 1.2 적용 범위
본 표준은 다음을 다룹니다:
- 유전체 데이터 작업을 위한 RESTful API 엔드포인트
- 인증 및 권한 부여 메커니즘
- 요청 및 응답 데이터 형식
- 비동기 처리를 위한 Webhook 알림
- 속도 제한 및 할당량 관리
- 오류 처리 및 상태 코드
- 일괄 처리 작업
- 실시간 스트리밍 인터페이스
- 타사 통합 패턴

### 1.3 기본 URL 구조

```
프로덕션: https://api.pet-genome.wia.org/v1
스테이징: https://api-staging.pet-genome.wia.org/v1
샌드박스: https://api-sandbox.pet-genome.wia.org/v1
```

### 1.4 API 버전 관리 전략

- **URL 기반 버전 관리**: `/v1/`, `/v2/`
- **하위 호환성**: 24개월 동안 유지
- **사용 중단 알림**: 제거 전 최소 6개월
- **버전 마이그레이션 가이드**: 중단 변경 사항에 대해 제공

---

## 2. 인증 및 권한 부여

### 2.1 인증 방법

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

**응답:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "genomic:read genomic:write breed:analyze",
  "refresh_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6..."
}
```

#### API Key 인증

```http
GET /v1/profiles/PGP-ABCD12345678 HTTP/1.1
Host: api.pet-genome.wia.org
X-API-Key: pgp_live_1234567890abcdef
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 2.2 권한 범위

| Scope | 설명 | 접근 수준 |
|-------|-------------|--------------|
| `genomic:read` | 유전체 프로파일 및 변이 읽기 | 읽기 전용 |
| `genomic:write` | 유전체 데이터 생성 및 업데이트 | 쓰기 |
| `genomic:delete` | 유전체 프로파일 삭제 | 삭제 |
| `breed:analyze` | 품종 식별 서비스 접근 | 실행 |
| `health:read` | 건강 마커 및 질병 위험 읽기 | 읽기 전용 |
| `health:write` | 건강 마커 해석 업데이트 | 쓰기 |
| `ancestry:compute` | 혈통 분석 수행 | 실행 |
| `research:contribute` | 연구 데이터베이스에 데이터 제출 | 쓰기 |
| `admin:manage` | 관리 작업 | 관리자 |

### 2.3 보안 헤더

```http
X-API-Key: pgp_live_1234567890abcdef
Authorization: Bearer {access_token}
X-Request-ID: req_1234567890abcdef
X-Idempotency-Key: idem_1234567890abcdef
X-Client-Version: PetGenomeSDK/1.2.3
```

---

## 3. 핵심 API 엔드포인트

### 3.1 유전체 프로파일 관리

#### 유전체 프로파일 생성

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

**응답 (201 Created):**
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

#### 유전체 프로파일 조회

```http
GET /v1/profiles/{profileId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
Accept: application/json
```

**응답 (200 OK):**
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

#### 유전체 프로파일 업데이트

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

**응답 (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "status": "UPDATED",
  "updatedAt": "2025-12-18T15:00:00Z",
  "updatedFields": ["petIdentification.neutered", "privacyConsent.researchParticipation"]
}
```

#### 유전체 프로파일 삭제

```http
DELETE /v1/profiles/{profileId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
X-Confirmation-Token: CONFIRM-DELETE-PGP-ABCD12345678
```

**응답 (204 No Content)**

### 3.2 변이 분석 엔드포인트

#### 변이 쿼리

```http
GET /v1/profiles/{profileId}/variants?gene=PRCD&significance=PATHOGENIC HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
- `chromosome`: 염색체로 필터링 (예: `chr1`, `chrX`)
- `startPosition`: 시작 위치 (정수)
- `endPosition`: 종료 위치 (정수)
- `gene`: 유전자 심볼
- `significance`: 변이 의미 (PATHOGENIC, BENIGN 등)
- `type`: 변이 유형 (SNP, INDEL, SV)
- `limit`: 페이지당 결과 수 (기본값: 100, 최대: 1000)
- `offset`: 페이지네이션 오프셋

**응답 (200 OK):**
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
        "recommendation": "번식 전 유전 상담 권장"
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

#### 사용자 정의 변이 주석

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

**응답 (200 OK):**
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
          "Ivermectin 민감성",
          "Loperamide 민감성"
        ]
      }
    }
  ]
}
```

### 3.3 품종 분석 엔드포인트

#### 품종 구성 분석

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

**응답 (202 Accepted):**
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

#### 품종 분석 결과 조회

```http
GET /v1/profiles/{profileId}/breed/analyze/{analysisId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
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
      "mitochondrialOrigin": "서유럽 계통"
    },
    "geneticDiversity": {
      "heterozygosity": 0.342,
      "inbreedingCoefficient": 0.023,
      "effectivePopulationSize": 450,
      "interpretation": "건강한 유전적 다양성"
    }
  }
}
```

#### 품종 주장 검증

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

**응답 (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "claimedBreed": "Labrador Retriever",
  "validated": true,
  "geneticBreedPercentage": 92.3,
  "purebreadThreshold": 87.5,
  "confidence": 0.96,
  "matchQuality": "EXCELLENT",
  "recommendation": "품종 주장 검증됨",
  "registrationEligibility": {
    "akc": true,
    "fci": true,
    "notes": "등록을 위한 순혈 기준 충족"
  }
}
```

### 3.4 건강 분석 엔드포인트

#### 질병 위험 분석

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

**응답 (200 OK):**
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
        "recommendation": "발병하지 않음; 보인자 상태 - 번식을 위한 유전 상담",
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
        "clinicalRecommendation": "표준 용량 안전",
        "adverseReactionRisk": "LOW"
      }
    ],
    "traits": [
      {
        "traitName": "털 색상",
        "category": "COAT_COLOR",
        "prediction": "노란색/금색",
        "confidence": 0.98,
        "genotypicBasis": "MC1R에서 ee 유전자형"
      },
      {
        "traitName": "운동 유발 허탈 민감성",
        "category": "PERFORMANCE",
        "prediction": "민감하지 않음",
        "confidence": 0.99,
        "genotypicBasis": "DNM1 변이에 대해 정상"
      }
    ]
  },
  "breedSpecificRisks": {
    "breed": "Labrador Retriever",
    "commonConditions": [
      {
        "condition": "고관절 이형성증",
        "geneticComponent": "POLYGENIC",
        "riskScore": 0.42,
        "recommendation": "모니터링 및 건강한 체중 유지"
      }
    ]
  }
}
```

#### 약물유전체학 보고서 조회

```http
GET /v1/profiles/{profileId}/pharmacogenomics HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
```json
{
  "profileId": "PGP-ABCD12345678",
  "reportDate": "2025-12-18T14:30:00Z",
  "drugResponses": [
    {
      "drugName": "Ivermectin",
      "drugClass": "항기생충제",
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
      "drugClass": "진정제",
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
  "veterinaryGuidance": "약물유전체학 금기사항이 확인되지 않았습니다. 표준 약물 프로토콜이 적절합니다."
}
```

### 3.5 혈통 및 계통 엔드포인트

#### 관련성 계산

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

**응답 (200 OK):**
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
  "interpretation": "이복형제일 가능성 높음 (한 부모 공유)",
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

#### 혈통도 구축

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

**응답 (200 OK):**
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

### 3.6 연구 및 집단 엔드포인트

#### 연구 데이터베이스 기여

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

**응답 (201 Created):**
```json
{
  "contributionId": "CONTRIB-001234",
  "studyId": "STUDY-CANINE-HEALTH-2025",
  "anonymizedId": "ANON-987654321",
  "status": "ACCEPTED",
  "dataShared": ["genotypes", "breed_info", "health_markers"],
  "acknowledgment": "개 건강 연구에 기여해 주셔서 감사합니다"
}
```

#### 집단 통계 쿼리

```http
GET /v1/population/statistics?species=CANIS_FAMILIARIS&breed=Labrador%20Retriever HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
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

### 3.7 파일 업로드 및 처리

#### 유전체 데이터 파일 업로드

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

[바이너리 VCF 파일 내용]
--boundary--
```

**응답 (202 Accepted):**
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

#### 업로드 상태 확인

```http
GET /v1/profiles/{profileId}/upload/{uploadId} HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
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

### 3.8 일괄 작업

#### 일괄 프로파일 생성

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

**응답 (202 Accepted):**
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

### 3.9 Webhook 알림

#### Webhook 등록

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

**응답 (201 Created):**
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

#### Webhook 페이로드 예제

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

### 3.10 내보내기 및 보고

#### 수의학 보고서 생성

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
  "language": "ko"
}
```

**응답 (200 OK):**
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

#### 타사 형식으로 내보내기

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

**응답 (200 OK):**
```json
{
  "exportId": "EXPORT-001234",
  "format": "PEDIGREE_SOFTWARE",
  "downloadUrl": "https://exports.pet-genome.wia.org/EXPORT-001234.xml",
  "expiresAt": "2025-12-25T14:30:00Z"
}
```

---

## 4. 오류 처리

### 4.1 표준 오류 응답

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "요청에 필수 필드가 없습니다: speciesCode",
    "details": {
      "field": "speciesCode",
      "reason": "필드가 필수이지만 제공되지 않았습니다"
    },
    "requestId": "req_1234567890abcdef",
    "timestamp": "2025-12-18T14:30:00Z",
    "documentation": "https://docs.pet-genome.wia.org/errors/INVALID_REQUEST"
  }
}
```

### 4.2 오류 코드

| HTTP 상태 | 오류 코드 | 설명 | 재시도 |
|-------------|------------|-------------|-------|
| 400 | INVALID_REQUEST | 잘못된 요청 또는 필수 필드 누락 | 아니오 |
| 401 | UNAUTHORIZED | 잘못되었거나 누락된 인증 자격 증명 | 아니오 |
| 403 | FORBIDDEN | 요청한 작업에 대한 권한 부족 | 아니오 |
| 404 | NOT_FOUND | 요청한 리소스가 존재하지 않음 | 아니오 |
| 409 | CONFLICT | 리소스가 이미 존재하거나 상태 충돌 | 아니오 |
| 422 | VALIDATION_ERROR | 요청 검증 실패 | 아니오 |
| 429 | RATE_LIMIT_EXCEEDED | 너무 많은 요청 | 예 |
| 500 | INTERNAL_ERROR | 서버 오류 | 예 |
| 503 | SERVICE_UNAVAILABLE | 서비스 일시적으로 사용 불가 | 예 |

---

## 5. 속도 제한

### 5.1 속도 제한 헤더

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1703001600
X-RateLimit-Window: 3600
```

### 5.2 속도 제한 계층

| 계층 | 시간당 요청 | 일일 요청 | 동시 업로드 |
|------|---------------|--------------|-------------------|
| Free | 100 | 1,000 | 1 |
| Basic | 1,000 | 10,000 | 5 |
| Professional | 10,000 | 100,000 | 20 |
| Enterprise | 맞춤형 | 맞춤형 | 맞춤형 |

---

## 6. 코드 예제

### 6.1 Python SDK 예제

```python
from pet_genome_sdk import PetGenomeClient
from pet_genome_sdk.models import GenomicProfile, BreedAnalysis

# 클라이언트 초기화
client = PetGenomeClient(
    api_key='pgp_live_1234567890abcdef',
    environment='production'
)

# 유전체 프로파일 생성
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

print(f"생성된 프로파일: {profile.profile_id}")

# VCF 파일 업로드
with open('sample.vcf.gz', 'rb') as vcf_file:
    upload = client.profiles.upload_file(
        profile_id=profile.profile_id,
        file_type='VCF',
        file=vcf_file
    )

# 처리 대기
upload.wait_for_completion(timeout=900)  # 15분

# 품종 구성 분석
breed_analysis = client.breed.analyze(
    profile_id=profile.profile_id,
    analysis_type='COMPREHENSIVE',
    include_ancestry=True
)

# 결과 대기
results = breed_analysis.wait_for_results()

# 품종 구성 출력
for breed in results.breed_composition:
    print(f"{breed.breed_name}: {breed.percentage}%")

# 건강 분석 조회
health = client.health.analyze(
    profile_id=profile.profile_id,
    analysis_scope='COMPREHENSIVE'
)

# 병원성 변이 확인
for disease in health.genetic_disease_risks:
    if disease.risk_level in ['AFFECTED', 'HIGH']:
        print(f"경고: {disease.disease_name} - {disease.risk_level}")

# 관련성 계산
relatedness = client.relatedness.calculate(
    profile_id1=profile.profile_id,
    profile_id2='PGP-EFGH87654321'
)

print(f"관계: {relatedness.relationship}")
print(f"친족 계수: {relatedness.kinship_coefficient}")

# 수의학 보고서 생성
report = client.reports.generate_veterinary(
    profile_id=profile.profile_id,
    format='PDF',
    sections=['BREED_IDENTIFICATION', 'HEALTH_SCREENING']
)

# 보고서 다운로드
report.download('veterinary_report.pdf')
```

### 6.2 JavaScript/Node.js SDK 예제

```javascript
const { PetGenomeClient } = require('@wia/pet-genome-sdk');

// 클라이언트 초기화
const client = new PetGenomeClient({
  apiKey: 'pgp_live_1234567890abcdef',
  environment: 'production'
});

// 유전체 프로파일 생성
async function analyzePetGenome() {
  try {
    // 프로파일 생성
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

    console.log(`생성된 프로파일: ${profile.profileId}`);

    // VCF 파일 업로드
    const fs = require('fs');
    const vcfStream = fs.createReadStream('sample.vcf.gz');

    const upload = await client.profiles.uploadFile({
      profileId: profile.profileId,
      fileType: 'VCF',
      file: vcfStream
    });

    // 처리 대기
    await upload.waitForCompletion({ timeout: 900000 }); // 15분

    // 품종 구성 분석
    const breedAnalysis = await client.breed.analyze({
      profileId: profile.profileId,
      analysisType: 'COMPREHENSIVE',
      includeAncestry: true
    });

    // 결과 조회
    const results = await breedAnalysis.getResults();

    // 품종 구성 출력
    results.breedComposition.forEach(breed => {
      console.log(`${breed.breedName}: ${breed.percentage}%`);
    });

    // 건강 분석
    const health = await client.health.analyze({
      profileId: profile.profileId,
      analysisScope: 'COMPREHENSIVE'
    });

    // 고위험 상태 확인
    health.geneticDiseaseRisks
      .filter(d => ['AFFECTED', 'HIGH'].includes(d.riskLevel))
      .forEach(disease => {
        console.log(`경고: ${disease.diseaseName} - ${disease.riskLevel}`);
      });

    // 보고서 생성
    const report = await client.reports.generateVeterinary({
      profileId: profile.profileId,
      format: 'PDF',
      sections: ['BREED_IDENTIFICATION', 'HEALTH_SCREENING']
    });

    console.log(`보고서 사용 가능: ${report.downloadUrl}`);

  } catch (error) {
    console.error('오류:', error.message);
    if (error.code === 'RATE_LIMIT_EXCEEDED') {
      const retryAfter = error.retryAfter;
      console.log(`속도 제한됨. ${retryAfter}초 후 재시도`);
    }
  }
}

analyzePetGenome();
```

### 6.3 Webhook 핸들러 예제

```python
from flask import Flask, request, jsonify
import hmac
import hashlib

app = Flask(__name__)

WEBHOOK_SECRET = 'whsec_1234567890abcdef'

def verify_webhook_signature(payload, signature):
    """Webhook 서명 확인"""
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
    # 헤더에서 서명 가져오기
    signature = request.headers.get('X-Webhook-Signature')

    # 원시 페이로드 가져오기
    payload = request.get_data()

    # 서명 확인
    if not verify_webhook_signature(payload, signature):
        return jsonify({'error': '잘못된 서명'}), 401

    # 이벤트 파싱
    event = request.json

    # 다양한 이벤트 유형 처리
    if event['event'] == 'analysis.completed':
        handle_analysis_completed(event['data'])
    elif event['event'] == 'profile.created':
        handle_profile_created(event['data'])
    elif event['event'] == 'health.alert':
        handle_health_alert(event['data'])

    return jsonify({'status': 'received'}), 200

def handle_analysis_completed(data):
    """분석 완료 처리"""
    profile_id = data['profileId']
    analysis_id = data['analysisId']

    print(f"프로파일 {profile_id}에 대한 분석 {analysis_id} 완료")

    # 결과 가져오기
    # 사용자에게 알림 보내기
    # 데이터베이스 업데이트
    pass

def handle_profile_created(data):
    """프로파일 생성 처리"""
    profile_id = data['profileId']
    print(f"새 프로파일 생성됨: {profile_id}")
    pass

def handle_health_alert(data):
    """건강 알림 처리"""
    profile_id = data['profileId']
    alert_type = data['alertType']
    severity = data['severity']

    print(f"{profile_id}에 대한 건강 알림: {alert_type} (심각도: {severity})")

    # 수의사에게 긴급 알림 전송
    # 알림 기록
    pass

if __name__ == '__main__':
    app.run(port=3000)
```

### 6.4 일괄 처리 예제

```python
import asyncio
from pet_genome_sdk import AsyncPetGenomeClient

async def process_batch_profiles(microchip_ids):
    """여러 프로파일을 동시에 처리"""
    client = AsyncPetGenomeClient(
        api_key='pgp_live_1234567890abcdef'
    )

    async def process_single_profile(microchip_id):
        try:
            # 프로파일 생성
            profile = await client.profiles.create(
                species_code='CANIS_FAMILIARIS',
                pet_identification={
                    'microchip_id': microchip_id
                }
            )

            # VCF 업로드
            vcf_path = f'data/{microchip_id}.vcf.gz'
            upload = await client.profiles.upload_file(
                profile_id=profile.profile_id,
                file_type='VCF',
                file_path=vcf_path
            )

            # 처리 대기
            await upload.wait_for_completion()

            # 분석
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

    # 모든 프로파일을 동시에 처리
    tasks = [process_single_profile(mid) for mid in microchip_ids]
    results = await asyncio.gather(*tasks)

    # 요약
    success_count = sum(1 for r in results if r['status'] == 'SUCCESS')
    print(f"{len(results)}개 프로파일 처리: {success_count}개 성공")

    return results

# 일괄 처리 실행
microchip_ids = ['123456789012345', '234567890123456', '345678901234567']
results = asyncio.run(process_batch_profiles(microchip_ids))

for result in results:
    print(f"{result['microchip_id']}: {result['status']}")
```

### 6.5 실시간 변이 스트리밍

```python
from pet_genome_sdk import PetGenomeClient

client = PetGenomeClient(api_key='pgp_live_1234567890abcdef')

# 병원성 변이 스트리밍
def stream_pathogenic_variants(profile_id):
    """식별된 병원성 변이를 스트리밍"""

    # 스트리밍 연결 열기
    stream = client.variants.stream_pathogenic(
        profile_id=profile_id,
        significance_filter=['PATHOGENIC', 'LIKELY_PATHOGENIC']
    )

    print(f"{profile_id}에 대한 병원성 변이 스트리밍 중...")

    for variant in stream:
        print(f"\n병원성 변이 감지됨:")
        print(f"  유전자: {variant.gene}")
        print(f"  변이: {variant.chromosome}:{variant.position}")
        print(f"  의미: {variant.significance}")
        print(f"  관련 질병: {variant.associated_conditions}")

        # 알림 전송
        send_veterinary_alert(profile_id, variant)

    print("스트리밍 완료")

def send_veterinary_alert(profile_id, variant):
    """수의사에게 알림 전송"""
    # 알림 전송 구현
    pass

# 스트리밍 시작
stream_pathogenic_variants('PGP-ABCD12345678')
```

---

## 7. 페이지네이션

### 7.1 커서 기반 페이지네이션

```http
GET /v1/profiles?limit=100&cursor=eyJpZCI6IlBHUC0xMjM0NTY3OCJ9 HTTP/1.1
Host: api.pet-genome.wia.org
Authorization: Bearer {access_token}
```

**응답:**
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

## 8. API 버전 관리 및 사용 중단

### 8.1 사용 중단 헤더

```http
HTTP/1.1 200 OK
X-API-Deprecated: true
X-API-Sunset-Date: 2026-06-18
X-API-Deprecation-Info: https://docs.pet-genome.wia.org/deprecations/v1-profiles-endpoint
Link: </v2/profiles>; rel="successor-version"
```

---

## 9. 서비스 수준 계약 (SLA)

| 지표 | 목표 | 측정 |
|--------|--------|-------------|
| API 가동 시간 | 99.9% | 월간 |
| 응답 시간 (p95) | < 500ms | 엔드포인트별 |
| 응답 시간 (p99) | < 1000ms | 엔드포인트별 |
| 데이터 처리 시간 | < 30분 | 30x WGS당 |
| 지원 응답 | < 4시간 | 업무 시간 |

---

## 10. 버전 이력

| Version | Date | 변경사항 | 작성자 |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | 초기 API 명세서 | WIA 표준 위원회 |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
