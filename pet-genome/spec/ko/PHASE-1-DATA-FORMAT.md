# WIA-PET-001: Pet Genome 데이터 형식 명세서
## Phase 1: 데이터 형식 표준

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. 소개

### 1.1 목적
본 명세서는 반려동물 유전 정보를 위한 표준화된 데이터 형식을 정의하여, 수의학 유전체학 연구소, 반려동물 DNA 검사 서비스, 번식 프로그램 및 연구 기관 간의 상호 운용성을 가능하게 합니다. Pet Genome 데이터 형식 표준은 반려동물 유전체학 생태계 전반에 걸쳐 유전자 서열, 품종 식별, 건강 마커 및 혈통 데이터의 일관된 표현을 보장합니다.

### 1.2 적용 범위
본 표준은 다음을 다룹니다:
- 반려동물(개, 고양이, 말, 조류, 이국적인 애완동물)을 위한 DNA 서열 데이터 형식
- 반려동물 전용 유전체학을 위한 Variant Call Format (VCF) 확장
- 참조 게놈 명세
- 품종 식별 및 혈통 추적
- 유전적 건강 마커 및 질병 소인
- 혈통 및 유산 분석 데이터 구조
- 유전적 다양성 지표
- 복제 및 유전적 보존 데이터
- 종간 호환성 표준
- 유전 데이터에 대한 개인정보 보호 및 동의 메타데이터

### 1.3 대상 사용자
- 수의학 유전체학 연구소
- 반려동물 DNA 검사 서비스 제공업체
- 동물 번식 조직
- 수의 클리닉 및 병원
- 반려동물 보험 회사
- 동물 연구 기관
- 반려동물 소유자 및 브리더
- 동물 유전 상담사

### 1.4 설계 원칙
1. **상호운용성**: 서로 다른 시스템 간의 원활한 데이터 교환
2. **정확성**: 유전 정보의 정밀한 표현
3. **개인정보 보호**: 반려동물 소유자 및 유전 데이터 보호
4. **확장성**: 새로운 유전체 기술 지원
5. **종 독립성**: 다양한 반려동물 종에 적용 가능
6. **하위 호환성**: 레거시 유전체 데이터 형식 지원

---

## 2. 핵심 데이터 구조

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
      "description": "반려동물 게놈 프로파일의 고유 식별자"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "시맨틱 버전 관리를 따르는 스키마 버전"
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
      "description": "과학적 종 분류"
    },
    "petIdentification": {
      "type": "object",
      "properties": {
        "registeredName": {
          "type": "string",
          "maxLength": 200,
          "description": "반려동물의 등록명"
        },
        "microchipId": {
          "type": "string",
          "pattern": "^[0-9]{15}$",
          "description": "마이크로칩 식별 번호"
        },
        "registrationNumber": {
          "type": "string",
          "description": "품종 등록 번호"
        },
        "birthDate": {
          "type": "string",
          "format": "date",
          "description": "출생일"
        },
        "sex": {
          "type": "string",
          "enum": ["MALE", "FEMALE", "INTERSEX", "UNKNOWN"],
          "description": "성별"
        },
        "neutered": {
          "type": "boolean",
          "description": "중성화 여부"
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
          ],
          "description": "시퀀싱 방법"
        },
        "referenceGenome": {
          "type": "object",
          "properties": {
            "assembly": {
              "type": "string",
              "description": "참조 게놈 어셈블리 (예: CanFam3.1, CanFam4, Felis_catus_9.0)"
            },
            "version": {
              "type": "string",
              "description": "참조 게놈 버전"
            },
            "source": {
              "type": "string",
              "format": "uri",
              "description": "참조 게놈 출처 URL"
            }
          }
        },
        "coverage": {
          "type": "object",
          "description": "시퀀싱 커버리지 정보",
          "properties": {
            "meanDepth": {
              "type": "number",
              "minimum": 0,
              "description": "평균 깊이"
            },
            "medianDepth": {
              "type": "number",
              "minimum": 0,
              "description": "중앙값 깊이"
            },
            "percentageAbove10x": {
              "type": "number",
              "minimum": 0,
              "maximum": 100,
              "description": "10x 이상 커버리지 비율"
            },
            "percentageAbove30x": {
              "type": "number",
              "minimum": 0,
              "maximum": 100,
              "description": "30x 이상 커버리지 비율"
            }
          }
        },
        "variants": {
          "type": "object",
          "description": "변이 데이터",
          "properties": {
            "vcfFileUri": {
              "type": "string",
              "format": "uri",
              "description": "변이 호출을 포함하는 VCF 파일의 URI"
            },
            "totalVariants": {
              "type": "integer",
              "minimum": 0,
              "description": "총 변이 수"
            },
            "snpCount": {
              "type": "integer",
              "minimum": 0,
              "description": "SNP 수"
            },
            "indelCount": {
              "type": "integer",
              "minimum": 0,
              "description": "Indel 수"
            },
            "structuralVariants": {
              "type": "integer",
              "minimum": 0,
              "description": "구조적 변이 수"
            },
            "clinicalVariants": {
              "type": "array",
              "description": "임상적 의미가 있는 변이",
              "items": {
                "$ref": "#/definitions/ClinicalVariant"
              }
            }
          }
        },
        "rawDataUri": {
          "type": "string",
          "format": "uri",
          "description": "원시 시퀀싱 데이터의 URI (FASTQ/BAM)"
        }
      }
    },
    "breedInformation": {
      "type": "object",
      "description": "품종 정보",
      "properties": {
        "primaryBreed": {
          "type": "object",
          "description": "주요 품종",
          "properties": {
            "breedName": {
              "type": "string",
              "description": "품종명"
            },
            "breedCode": {
              "type": "string",
              "description": "AKC/FCI/TICA 표준 품종 코드"
            },
            "percentage": {
              "type": "number",
              "minimum": 0,
              "maximum": 100,
              "description": "품종 비율 (%)"
            },
            "confidence": {
              "type": "number",
              "minimum": 0,
              "maximum": 1,
              "description": "신뢰도 점수"
            }
          }
        },
        "ancestryComposition": {
          "type": "array",
          "description": "혈통 구성",
          "items": {
            "type": "object",
            "properties": {
              "breedName": {
                "type": "string",
                "description": "품종명"
              },
              "percentage": {
                "type": "number",
                "minimum": 0,
                "maximum": 100,
                "description": "비율 (%)"
              },
              "generationsBack": {
                "type": "integer",
                "minimum": 1,
                "description": "몇 세대 전"
              }
            }
          }
        },
        "breedGroup": {
          "type": "string",
          "description": "품종 그룹 (예: Sporting, Hound, Working, Terrier)"
        },
        "wildcatAncestry": {
          "type": "object",
          "description": "야생 고양이 혈통 (고양이용)",
          "properties": {
            "wildcatSpecies": {
              "type": "string",
              "description": "야생 고양이 종"
            },
            "percentage": {
              "type": "number",
              "description": "야생 혈통 비율"
            }
          }
        }
      }
    },
    "healthMarkers": {
      "type": "object",
      "description": "건강 마커",
      "properties": {
        "geneticDiseaseRisks": {
          "type": "array",
          "description": "유전 질환 위험",
          "items": {
            "$ref": "#/definitions/DiseaseRisk"
          }
        },
        "pharmacogenomics": {
          "type": "array",
          "description": "약물유전체학 정보",
          "items": {
            "$ref": "#/definitions/DrugResponse"
          }
        },
        "traits": {
          "type": "array",
          "description": "유전적 특성",
          "items": {
            "$ref": "#/definitions/GeneticTrait"
          }
        }
      }
    },
    "sampleMetadata": {
      "type": "object",
      "description": "샘플 메타데이터",
      "required": ["collectionDate", "sampleType", "laboratory"],
      "properties": {
        "sampleId": {
          "type": "string",
          "description": "샘플 식별자"
        },
        "collectionDate": {
          "type": "string",
          "format": "date-time",
          "description": "샘플 채취 날짜"
        },
        "sampleType": {
          "type": "string",
          "enum": [
            "BLOOD",
            "SALIVA",
            "BUCCAL_SWAB",
            "TISSUE",
            "HAIR_FOLLICLE"
          ],
          "description": "샘플 유형"
        },
        "laboratory": {
          "type": "object",
          "description": "검사 실험실",
          "properties": {
            "name": {
              "type": "string",
              "description": "실험실명"
            },
            "certifications": {
              "type": "array",
              "description": "인증",
              "items": {
                "type": "string"
              }
            },
            "contactInfo": {
              "type": "object",
              "description": "연락처 정보"
            }
          }
        },
        "processingDate": {
          "type": "string",
          "format": "date-time",
          "description": "처리 날짜"
        }
      }
    },
    "privacyConsent": {
      "type": "object",
      "description": "개인정보 동의",
      "properties": {
        "ownerConsent": {
          "type": "boolean",
          "description": "소유자 동의 여부"
        },
        "researchParticipation": {
          "type": "boolean",
          "description": "연구 참여 동의"
        },
        "dataSharing": {
          "type": "object",
          "description": "데이터 공유 설정",
          "properties": {
            "allowBreedingDatabase": {
              "type": "boolean",
              "description": "번식 데이터베이스 공유 허용"
            },
            "allowResearch": {
              "type": "boolean",
              "description": "연구 목적 공유 허용"
            },
            "allowCommercial": {
              "type": "boolean",
              "description": "상업적 사용 허용"
            }
          }
        },
        "consentDate": {
          "type": "string",
          "format": "date-time",
          "description": "동의 날짜"
        }
      }
    },
    "qualityMetrics": {
      "type": "object",
      "description": "품질 지표",
      "properties": {
        "overallQualityScore": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "전체 품질 점수"
        },
        "contaminationCheck": {
          "type": "boolean",
          "description": "오염 검사 통과"
        },
        "sexConsistency": {
          "type": "boolean",
          "description": "성별 일관성"
        },
        "callRate": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "호출 비율"
        }
      }
    }
  },
  "definitions": {
    "ClinicalVariant": {
      "type": "object",
      "description": "임상 변이",
      "properties": {
        "variantId": {
          "type": "string",
          "description": "변이 식별자"
        },
        "chromosome": {
          "type": "string",
          "description": "염색체"
        },
        "position": {
          "type": "integer",
          "description": "위치"
        },
        "referenceAllele": {
          "type": "string",
          "description": "참조 대립유전자"
        },
        "alternateAllele": {
          "type": "string",
          "description": "대체 대립유전자"
        },
        "genotype": {
          "type": "string",
          "pattern": "^(0|1|2)/(0|1|2)$",
          "description": "유전자형"
        },
        "significance": {
          "type": "string",
          "enum": [
            "PATHOGENIC",
            "LIKELY_PATHOGENIC",
            "UNCERTAIN",
            "LIKELY_BENIGN",
            "BENIGN"
          ],
          "description": "임상적 의미"
        },
        "associatedConditions": {
          "type": "array",
          "description": "관련 질환",
          "items": {
            "type": "string"
          }
        }
      }
    },
    "DiseaseRisk": {
      "type": "object",
      "description": "질병 위험",
      "properties": {
        "diseaseName": {
          "type": "string",
          "description": "질병명"
        },
        "diseaseCode": {
          "type": "string",
          "description": "OMIA (Online Mendelian Inheritance in Animals) 코드"
        },
        "riskLevel": {
          "type": "string",
          "enum": ["HIGH", "MODERATE", "LOW", "CARRIER"],
          "description": "위험 수준"
        },
        "inheritancePattern": {
          "type": "string",
          "enum": [
            "AUTOSOMAL_DOMINANT",
            "AUTOSOMAL_RECESSIVE",
            "X_LINKED",
            "POLYGENIC"
          ],
          "description": "유전 패턴"
        },
        "affectedGenes": {
          "type": "array",
          "description": "영향받는 유전자",
          "items": {
            "type": "string"
          }
        },
        "penetrance": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "침투도"
        }
      }
    },
    "DrugResponse": {
      "type": "object",
      "description": "약물 반응",
      "properties": {
        "drugName": {
          "type": "string",
          "description": "약물명"
        },
        "gene": {
          "type": "string",
          "description": "관련 유전자"
        },
        "variant": {
          "type": "string",
          "description": "변이"
        },
        "responseType": {
          "type": "string",
          "enum": [
            "NORMAL_METABOLIZER",
            "POOR_METABOLIZER",
            "RAPID_METABOLIZER",
            "ADVERSE_REACTION_RISK"
          ],
          "description": "반응 유형"
        },
        "clinicalRecommendation": {
          "type": "string",
          "description": "임상 권장사항"
        }
      }
    },
    "GeneticTrait": {
      "type": "object",
      "description": "유전적 특성",
      "properties": {
        "traitName": {
          "type": "string",
          "description": "특성명"
        },
        "category": {
          "type": "string",
          "enum": [
            "PHYSICAL",
            "BEHAVIORAL",
            "PERFORMANCE",
            "COAT_COLOR",
            "SIZE"
          ],
          "description": "특성 카테고리"
        },
        "prediction": {
          "type": "string",
          "description": "예측 결과"
        },
        "confidence": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "신뢰도"
        }
      }
    }
  }
}
```

### 2.2 Variant Call Format (VCF) 확장

#### 반려동물 전용 VCF Header 필드

```
##fileformat=VCFv4.3
##fileDate=20251218
##source=PetGenomeAnalyzer_v1.0
##reference=CanFam4.0
##INFO=<ID=BREED_AF,Number=A,Type=Float,Description="특정 품종에서의 대립유전자 빈도">
##INFO=<ID=OMIA,Number=1,Type=String,Description="OMIA 질병 식별자">
##INFO=<ID=TRAIT,Number=.,Type=String,Description="관련 표현형 특성">
##INFO=<ID=PATHOGENIC,Number=0,Type=Flag,Description="알려진 병원성 변이">
##INFO=<ID=CARRIER_FREQ,Number=1,Type=Float,Description="집단에서의 보인자 빈도">
##FORMAT=<ID=GT,Number=1,Type=String,Description="유전자형">
##FORMAT=<ID=DP,Number=1,Type=Integer,Description="읽기 깊이">
##FORMAT=<ID=GQ,Number=1,Type=Integer,Description="유전자형 품질">
##FORMAT=<ID=PL,Number=G,Type=Integer,Description="Phred-scaled 유전자형 우도">
```

### 2.3 품종 혈통 데이터 형식

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

## 3. 참조 게놈 표준

### 3.1 지원되는 참조 어셈블리

| 종 (Species) | 일반명 | Assembly | Version | 출시일 | URL |
|---------|-------------|----------|---------|--------------|-----|
| Canis familiaris | 개 | CanFam4 | GCA_011100685.1 | 2020-03 | https://www.ncbi.nlm.nih.gov/assembly/GCF_011100685.1 |
| Canis familiaris | 개 | CanFam3.1 | GCA_000002285.2 | 2011-09 | https://www.ncbi.nlm.nih.gov/assembly/GCF_000002285.3 |
| Felis catus | 고양이 | Felis_catus_9.0 | GCA_000181335.4 | 2017-11 | https://www.ncbi.nlm.nih.gov/assembly/GCF_000181335.3 |
| Equus caballus | 말 | EquCab3.0 | GCA_002863925.1 | 2018-01 | https://www.ncbi.nlm.nih.gov/assembly/GCF_002863925.1 |
| Oryctolagus cuniculus | 토끼 | OryCun2.0 | GCA_000003625.1 | 2009-04 | https://www.ncbi.nlm.nih.gov/assembly/GCF_000003625.3 |

### 3.2 염색체 명명 규칙

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

## 4. 건강 마커 표준

### 4.1 질병 위험 분류

| 위험 수준 | 유전자형 해석 | 임상 조치 | 검사 빈도 |
|------------|------------------------|-----------------|-------------------|
| AFFECTED | 병원성 변이에 대해 동형접합 | 즉시 수의사 상담 | 연간 모니터링 |
| HIGH_RISK | 우성 병원성 변이에 대해 이형접합 | 예방적 치료 권장 | 반기별 모니터링 |
| CARRIER | 열성 변이에 대해 이형접합 | 번식 시 고려사항 | 번식 전 검사 |
| MODERATE | 다유전자 위험 요소 존재 | 생활습관 개선 | 2-3년마다 |
| LOW | 보호 또는 양성 변이 | 표준 관리 | 5년마다 |
| UNKNOWN | 의미 불확실한 새로운 변이 | 유전 상담 | 필요시 |

### 4.2 일반적인 유전 질환 Schema

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
        "야맹증",
        "진행성 시력 상실",
        "완전한 실명 (진행 단계)"
      ],
      "ageOfOnset": "3-5세",
      "penetrance": 1.0,
      "treatmentAvailable": false,
      "preventiveScreening": "번식 전 검사 권장"
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
      "clinicalImpact": "약물 민감성",
      "affectedDrugs": [
        "Ivermectin",
        "Loperamide",
        "Vincristine",
        "Doxorubicin"
      ],
      "penetrance": 1.0,
      "clinicalManagement": "금기 약물 회피",
      "pharmacogenomicGuidance": "대체 약물 사용"
    }
  ]
}
```

---

## 5. 코드 예제

### 5.1 Pet Genome Profile 파싱

```python
import json
from typing import Dict, List, Optional
from datetime import datetime

class PetGenomeProfile:
    """WIA Pet Genome Profile 데이터 형식 파서"""

    def __init__(self, profile_data: Dict):
        self.data = profile_data
        self.profile_id = profile_data['profileId']
        self.species = profile_data['speciesCode']

    def get_breed_composition(self) -> List[Dict]:
        """품종 혈통 정보 추출"""
        breed_info = self.data.get('breedInformation', {})
        ancestry = breed_info.get('ancestryComposition', [])

        return sorted(ancestry,
                     key=lambda x: x.get('percentage', 0),
                     reverse=True)

    def get_disease_risks(self, risk_level: Optional[str] = None) -> List[Dict]:
        """유전 질병 위험 마커 조회"""
        health_markers = self.data.get('healthMarkers', {})
        diseases = health_markers.get('geneticDiseaseRisks', [])

        if risk_level:
            diseases = [d for d in diseases
                       if d.get('riskLevel') == risk_level]

        return diseases

    def get_pharmacogenomic_alerts(self) -> List[Dict]:
        """약물 반응 예측 조회"""
        health_markers = self.data.get('healthMarkers', {})
        return health_markers.get('pharmacogenomics', [])

    def calculate_diversity_score(self) -> float:
        """유전적 다양성 지수 계산"""
        breed_comp = self.get_breed_composition()

        if not breed_comp:
            return 0.0

        # Shannon 다양성 지수
        diversity = 0.0
        for breed in breed_comp:
            p = breed.get('percentage', 0) / 100.0
            if p > 0:
                diversity -= p * math.log(p)

        return diversity

    def is_suitable_for_breeding(self) -> Dict:
        """유전 건강에 기반한 번식 적합성 평가"""
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
        """번식 권장사항 생성"""
        recommendations = []

        if affected:
            recommendations.append("번식 금지: 유전 질환 발병")
        if high_risk:
            recommendations.append("번식 전 유전 상담 필요")
        if carriers:
            recommendations.append(f"{len(carriers)}개 질환 보인자 - 교배 상대 검사 필요")

        return recommendations if recommendations else ["번식 프로그램에 적합"]

# 사용 예제
with open('pet_genome_profile.json', 'r') as f:
    profile_data = json.load(f)

pet_genome = PetGenomeProfile(profile_data)

# 품종 구성 조회
breeds = pet_genome.get_breed_composition()
print(f"주요 품종: {breeds[0]['breedName']} ({breeds[0]['percentage']}%)")

# 질병 위험 확인
high_risk = pet_genome.get_disease_risks(risk_level='HIGH')
for disease in high_risk:
    print(f"경고: {disease['diseaseName']} - {disease['riskLevel']}")

# 번식 평가
breeding_assessment = pet_genome.is_suitable_for_breeding()
print(f"번식 적합: {breeding_assessment['suitable']}")
```

### 5.2 VCF 변이 분석

```python
import vcf
from typing import List, Dict

class PetVCFAnalyzer:
    """반려동물 전용 주석이 있는 VCF 파일 분석"""

    def __init__(self, vcf_path: str):
        self.vcf_reader = vcf.Reader(open(vcf_path, 'r'))

    def extract_pathogenic_variants(self) -> List[Dict]:
        """VCF에서 모든 병원성 변이 추출"""
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
        """특정 품종에 대한 대립유전자 빈도 계산"""
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
        """복합 이형접합 질병 변이 식별"""
        gene_variants = {}

        for record in self.vcf_reader:
            if record.samples and record.samples[0]['GT'] == '0/1':
                gene = record.INFO.get('GENE', 'UNKNOWN')
                if gene not in gene_variants:
                    gene_variants[gene] = []
                gene_variants[gene].append(record)

        # 여러 이형접합 변이가 있는 유전자 찾기
        compound_het = []
        for gene, variants in gene_variants.items():
            if len(variants) >= 2:
                compound_het.append({
                    'gene': gene,
                    'variant_count': len(variants),
                    'variants': [f"{v.CHROM}:{v.POS}" for v in variants]
                })

        return compound_het

# 사용 예제
analyzer = PetVCFAnalyzer('pet_genome.vcf')

# 병원성 변이 찾기
pathogenic = analyzer.extract_pathogenic_variants()
print(f"{len(pathogenic)}개의 병원성 변이 발견")

for variant in pathogenic:
    print(f"  {variant['chromosome']}:{variant['position']} - {variant['omia']}")
```

### 5.3 품종 예측 알고리즘

```python
import numpy as np
from sklearn.mixture import GaussianMixture
from typing import List, Tuple

class BreedIdentificationEngine:
    """머신러닝 기반 품종 식별"""

    def __init__(self, reference_panel_path: str):
        self.reference_panel = self._load_reference_panel(reference_panel_path)
        self.breed_models = {}

    def _load_reference_panel(self, path: str) -> Dict:
        """품종 참조 유전자형 로드"""
        import pickle
        with open(path, 'rb') as f:
            return pickle.load(f)

    def predict_breed_composition(self,
                                  genotype_data: np.ndarray,
                                  k_breeds: int = 5) -> List[Dict]:
        """
        혼합 분석을 사용한 품종 구성 예측

        Args:
            genotype_data: SNP 유전자형 행렬 (샘플 x 마커)
            k_breeds: 추정할 조상 품종 수

        Returns:
            백분율이 포함된 품종 예측 목록
        """
        # ADMIXTURE 유사 알고리즘
        gmm = GaussianMixture(n_components=k_breeds,
                            covariance_type='diagonal',
                            random_state=42)
        gmm.fit(genotype_data)

        # 혈통 비율 가져오기
        ancestry_props = gmm.predict_proba(genotype_data)

        # 성분을 알려진 품종에 매칭
        breed_assignments = self._match_components_to_breeds(
            gmm.means_, ancestry_props
        )

        return breed_assignments

    def _match_components_to_breeds(self,
                                   component_means: np.ndarray,
                                   ancestry_props: np.ndarray) -> List[Dict]:
        """혈통 성분을 알려진 품종에 매칭"""
        breed_results = []

        for i, component in enumerate(component_means):
            # 참조 패널에서 가장 가까운 품종 찾기
            breed_match = self._find_closest_breed(component)
            percentage = np.mean(ancestry_props[:, i]) * 100

            if percentage > 5.0:  # 보고 임계값
                breed_results.append({
                    'breedName': breed_match['name'],
                    'breedCode': breed_match['code'],
                    'percentage': round(percentage, 2),
                    'confidence': breed_match['confidence']
                })

        # 백분율로 정렬
        breed_results.sort(key=lambda x: x['percentage'], reverse=True)

        # 100%로 정규화
        total = sum([b['percentage'] for b in breed_results])
        for breed in breed_results:
            breed['percentage'] = (breed['percentage'] / total) * 100

        return breed_results

    def _find_closest_breed(self, component_signature: np.ndarray) -> Dict:
        """참조 패널에서 가장 가까운 품종 찾기"""
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
        """주장된 품종을 유전 데이터와 검증"""
        predicted_breeds = self.predict_breed_composition(genotype_data)

        # 주장된 품종이 상위 예측에 있는지 확인
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

### 5.4 유전적 다양성 계산기

```python
import numpy as np
from scipy.stats import chi2
from typing import Dict, List

class GeneticDiversityAnalyzer:
    """반려동물 집단에 대한 유전적 다양성 지표 계산"""

    def __init__(self, genotype_matrix: np.ndarray, sample_ids: List[str]):
        """
        Args:
            genotype_matrix: 유전자형 행렬 (샘플 x SNP), 0,1,2로 코딩됨
            sample_ids: 샘플 식별자 목록
        """
        self.genotypes = genotype_matrix
        self.sample_ids = sample_ids
        self.n_samples = len(sample_ids)
        self.n_snps = genotype_matrix.shape[1]

    def calculate_heterozygosity(self) -> Dict:
        """관찰 및 기대 이형접합도 계산"""
        # 관찰 이형접합도 (이형접합 유전자형 비율)
        het_genotypes = np.sum(self.genotypes == 1, axis=0)
        ho = np.mean(het_genotypes / self.n_samples)

        # 기대 이형접합도 (Hardy-Weinberg)
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
        """근친교배 계수 (F) 계산"""
        het_stats = self.calculate_heterozygosity()
        ho = het_stats['observedHeterozygosity']
        he = het_stats['expectedHeterozygosity']

        # F = (He - Ho) / He
        f_coeff = (he - ho) / he if he > 0 else 0

        # 동형접합 연속 구간 (ROH) 분석
        roh_segments = self._detect_roh_segments()

        return {
            'inbreedingCoefficient': round(f_coeff, 4),
            'rohBasedInbreeding': self._calculate_roh_f(roh_segments),
            'rohSegments': len(roh_segments),
            'totalRohLength': sum([s['length'] for s in roh_segments]),
            'interpretation': self._interpret_inbreeding(f_coeff)
        }

    def _calculate_allele_frequencies(self) -> np.ndarray:
        """각 SNP에 대한 소수 대립유전자 빈도 계산"""
        allele_counts = np.sum(self.genotypes, axis=0)
        total_alleles = self.n_samples * 2
        freq = allele_counts / total_alleles

        # 소수 대립유전자 빈도 반환
        return np.minimum(freq, 1 - freq)

    def _detect_roh_segments(self, min_snps: int = 50) -> List[Dict]:
        """동형접합 연속 구간 탐지"""
        roh_segments = []

        for sample_idx in range(self.n_samples):
            genotypes = self.genotypes[sample_idx, :]

            current_run = []
            for snp_idx, gt in enumerate(genotypes):
                if gt != 1:  # 동형접합
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

            # 마지막 구간 확인
            if len(current_run) >= min_snps:
                roh_segments.append({
                    'sampleId': self.sample_ids[sample_idx],
                    'startSnp': current_run[0],
                    'endSnp': current_run[-1],
                    'length': len(current_run)
                })

        return roh_segments

    def _calculate_roh_f(self, roh_segments: List[Dict]) -> float:
        """ROH 기반 근친교배 계산"""
        if not roh_segments:
            return 0.0

        # 2.5 Gb의 게놈 길이 가정 (개 게놈)
        genome_length = 2500000000
        total_roh = sum([s['length'] for s in roh_segments])

        # 물리적 길이 추정 (대략적 근사)
        physical_roh = (total_roh / self.n_snps) * genome_length

        return round(physical_roh / genome_length, 4)

    def _interpret_heterozygosity(self, ho: float, he: float) -> str:
        """이형접합도 값 해석"""
        if ho < he * 0.85:
            return "낮은 다양성 - 근친교배 가능성"
        elif ho > he * 1.05:
            return "높은 다양성 - 외래교배 집단"
        else:
            return "정상적인 다양성 수준"

    def _interpret_inbreeding(self, f: float) -> str:
        """근친교배 계수 해석"""
        if f < 0.05:
            return "낮은 근친교배"
        elif f < 0.125:
            return "중간 근친교배 (사촌 수준)"
        elif f < 0.25:
            return "높은 근친교배 (형제자매 수준)"
        else:
            return "매우 높은 근친교배 - 건강 문제 우려"

    def calculate_effective_population_size(self) -> int:
        """유효 집단 크기 추정"""
        # 연관 불균형 기반
        r2_values = self._calculate_ld()

        # Ne = 1 / (3 * mean(r2))
        mean_r2 = np.mean(r2_values)
        ne = int(1 / (3 * mean_r2)) if mean_r2 > 0 else 0

        return max(ne, 1)

    def _calculate_ld(self) -> np.ndarray:
        """인접 SNP 간 연관 불균형 (r²) 계산"""
        r2_values = []

        # 효율성을 위해 1000개 SNP 쌍 샘플링
        sample_size = min(1000, self.n_snps - 1)
        indices = np.random.choice(self.n_snps - 1, sample_size, replace=False)

        for i in indices:
            gt1 = self.genotypes[:, i]
            gt2 = self.genotypes[:, i + 1]

            # r² 계산
            r2 = self._calculate_r2(gt1, gt2)
            if not np.isnan(r2):
                r2_values.append(r2)

        return np.array(r2_values)

    def _calculate_r2(self, gt1: np.ndarray, gt2: np.ndarray) -> float:
        """두 SNP 간 r² 계산"""
        # 대립유전자 수로 변환
        p1 = np.mean(gt1) / 2
        p2 = np.mean(gt2) / 2

        # 일배체형 빈도 계산
        d = np.mean(gt1 * gt2) / 4 - p1 * p2

        # r² = D² / (p1 * (1-p1) * p2 * (1-p2))
        denominator = p1 * (1 - p1) * p2 * (1 - p2)

        if denominator > 0:
            return (d ** 2) / denominator
        else:
            return np.nan

# 사용 예제
genotypes = np.random.randint(0, 3, size=(100, 10000))  # 100개 샘플, 10k SNP
sample_ids = [f"PET-{i:04d}" for i in range(100)]

analyzer = GeneticDiversityAnalyzer(genotypes, sample_ids)

# 지표 계산
het_results = analyzer.calculate_heterozygosity()
print(f"관찰 이형접합도: {het_results['observedHeterozygosity']}")
print(f"기대 이형접합도: {het_results['expectedHeterozygosity']}")

inbreeding = analyzer.calculate_inbreeding_coefficient()
print(f"근친교배 계수: {inbreeding['inbreedingCoefficient']}")
print(f"ROH 구간 탐지: {inbreeding['rohSegments']}")

ne = analyzer.calculate_effective_population_size()
print(f"유효 집단 크기: {ne}")
```

### 5.5 데이터 내보내기 및 변환

```python
import json
import csv
from datetime import datetime
from typing import Dict, List

class PetGenomeExporter:
    """다양한 형식으로 반려동물 게놈 데이터 내보내기"""

    def __init__(self, genome_profile: Dict):
        self.profile = genome_profile

    def export_to_pedigree_format(self, output_path: str):
        """혈통 관리 소프트웨어용 내보내기"""
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
                'sire': None,  # 연결 예정
                'dam': None    # 연결 예정
            }
        }

        # 건강 상태 분류
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
        """수의학 임상 보고서 생성"""
        report = f"""
수의학 유전 건강 보고서
================================
생성 일시: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

환자 정보
-------------------
프로파일 ID: {self.profile['profileId']}
종: {self.profile['speciesCode']}
마이크로칩: {self.profile.get('petIdentification', {}).get('microchipId', 'N/A')}

품종 식별
--------------------
"""
        breeds = self.profile.get('breedInformation', {}).get('ancestryComposition', [])
        for breed in breeds[:5]:
            report += f"  {breed['breedName']}: {breed['percentage']:.1f}%\n"

        report += "\n유전 건강 검사\n"
        report += "------------------------\n"

        diseases = self.profile.get('healthMarkers', {}).get('geneticDiseaseRisks', [])

        # 위험 수준별로 그룹화
        for risk_level in ['AFFECTED', 'HIGH', 'CARRIER', 'MODERATE']:
            relevant = [d for d in diseases if d.get('riskLevel') == risk_level]
            if relevant:
                report += f"\n{risk_level} 위험:\n"
                for disease in relevant:
                    report += f"  - {disease['diseaseName']}\n"
                    report += f"    유전 패턴: {disease.get('inheritancePattern', '알 수 없음')}\n"
                    if 'affectedGenes' in disease:
                        report += f"    유전자: {', '.join(disease['affectedGenes'])}\n"

        report += "\n약물유전체학 고려사항\n"
        report += "------------------------------\n"

        pharmacogenomics = self.profile.get('healthMarkers', {}).get('pharmacogenomics', [])
        for drug_response in pharmacogenomics:
            if drug_response.get('responseType') in ['POOR_METABOLIZER', 'ADVERSE_REACTION_RISK']:
                report += f"  경고: {drug_response['drugName']}\n"
                report += f"    유형: {drug_response['responseType']}\n"
                report += f"    권장사항: {drug_response.get('clinicalRecommendation', '전문의 상담')}\n\n"

        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(report)

    def export_to_csv_summary(self, output_path: str):
        """요약 데이터를 CSV로 내보내기"""
        with open(output_path, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)

            # 헤더
            writer.writerow(['카테고리', '항목', '값', '세부사항'])

            # 기본 정보
            writer.writerow(['식별', '프로파일 ID', self.profile['profileId'], ''])
            writer.writerow(['식별', '종', self.profile['speciesCode'], ''])

            # 품종
            breeds = self.profile.get('breedInformation', {}).get('ancestryComposition', [])
            for breed in breeds:
                writer.writerow(['품종', breed['breedName'],
                               f"{breed['percentage']:.2f}%",
                               f"신뢰도: {breed.get('confidence', 'N/A')}"])

            # 건강 마커
            diseases = self.profile.get('healthMarkers', {}).get('geneticDiseaseRisks', [])
            for disease in diseases:
                writer.writerow(['건강', disease['diseaseName'],
                               disease['riskLevel'],
                               disease.get('inheritancePattern', '')])

    def _extract_inbreeding_coefficient(self) -> float:
        """프로파일에서 근친교배 계수 추출"""
        diversity = self.profile.get('breedInformation', {}).get('geneticDiversity', {})
        return diversity.get('inbreedingCoefficient', 0.0)

# 사용 예제
with open('pet_genome_profile.json', 'r') as f:
    profile = json.load(f)

exporter = PetGenomeExporter(profile)

# 다양한 형식으로 내보내기
exporter.export_to_pedigree_format('pedigree_export.json')
exporter.export_to_veterinary_report('vet_report.txt')
exporter.export_to_csv_summary('genome_summary.csv')
```

---

## 6. 데이터 개인정보 보호 및 보안

### 6.1 개인정보 보호 요구사항

| 데이터 카테고리 | 개인정보 수준 | 암호화 | 보관 기간 | 소유자 접근 |
|--------------|--------------|------------|------------------|--------------|
| 원시 유전체 데이터 | 매우 민감 | AES-256 | 10년 | 전체 접근 |
| 처리된 변이 | 민감 | AES-256 | 15년 | 전체 접근 |
| 품종 정보 | 중간 | 전송 중 TLS | 무기한 | 전체 접근 |
| 건강 마커 | 매우 민감 | AES-256 | 평생 + 5년 | 전체 접근 |
| 소유자 신원 | 개인식별정보 | AES-256 | 동의에 따름 | 제한적 |
| 연구 데이터 (익명화) | 공개 | 없음 | 무기한 | 집계만 |

### 6.2 동의 관리 Schema

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

## 7. 품질 관리 표준

### 7.1 시퀀싱 품질 지표

| 지표 | 최소 임계값 | 권장 | 우수 |
|--------|------------------|-------------|-----------|
| 평균 커버리지 깊이 | 10x | 30x | 50x |
| ≥10x 비율 | 85% | 95% | 98% |
| Call Rate | 95% | 98% | 99.5% |
| Ti/Tv 비율 (WGS) | 2.0 | 2.1-2.3 | 2.2 |
| Het/Hom 비율 | 1.3-1.8 | 1.4-1.6 | 1.5 |
| 오염률 | <3% | <1% | <0.5% |
| 성별 일관성 | 100% | 100% | 100% |

---

## 8. 파일 형식 명세

### 8.1 지원되는 파일 유형

- **FASTQ**: 원시 시퀀싱 리드
- **BAM/CRAM**: 정렬된 시퀀스 데이터
- **VCF/BCF**: 변이 호출
- **GFF3/GTF**: 게놈 주석
- **FASTA**: 참조 게놈
- **BED**: 게놈 영역
- **JSON**: 구조화된 유전체 메타데이터
- **HDF5**: 대규모 유전체 행렬

---

## 9. 상호운용성 표준

### 9.1 외부 시스템과의 통합

- **NCBI Sequence Read Archive (SRA)**: 원시 데이터 저장
- **European Nucleotide Archive (ENA)**: 국제 데이터 공유
- **OMIA (Online Mendelian Inheritance in Animals)**: 질병 변이 데이터베이스
- **AKC/FCI 레지스트리**: 품종 검증
- **수의학 진료 관리 시스템**: 임상 통합
- **반려동물 보험 플랫폼**: 위험 평가 (동의 하에)

---

## 10. 버전 이력

| Version | Date | 변경사항 | 작성자 |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | 초기 명세서 초안 | WIA 표준 위원회 |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
