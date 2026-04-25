# WIA-CRYO-ASSET Phase 1: 데이터 형식 명세서

**버전:** 1.0.0
**상태:** 초안
**최종 수정일:** 2025-12-18
**대표 색상:** #06B6D4 (Cyan)

## 1. 소개

### 1.1 목적

WIA 극저온 자산 관리 표준(WIA-CRYO-ASSET)은 극저온 보존 및 소생 과정 동안 자산을 관리, 보호 및 이전하기 위한 포괄적인 데이터 형식, 프로토콜 및 인터페이스를 정의합니다. 이 Phase 1 명세서는 안전한 자산 등록, 추적 및 최종 이전에 필요한 기본 데이터 구조를 확립합니다.

### 1.2 적용 범위

본 명세서는 다음을 다룹니다:

- 자산 등록 및 분류 데이터 형식
- 소유권 검증 및 암호학적 증명 구조
- 신탁 및 재산 통합 데이터 모델
- 디지털 및 물리적 자산 메타데이터 표준
- 시간 잠금 자산 이전 메커니즘
- 소생 트리거 조건 명세
- 다중 서명 및 수탁 약정

### 1.3 핵심 개념

**극저온 보존 기간**: 개인이 극저온 보존 상태에 있는 기간으로, 이 기간 동안 자산을 관리, 보호하고 소생 시 이전을 준비해야 합니다.

**자산 수탁자**: 보존 기간 동안 자산을 관리할 책임이 있는 신뢰할 수 있는 개체(개인, 조직 또는 스마트 계약)입니다.

**소생 이벤트**: 극저온 보존으로부터의 성공적인 소생을 나타내는 검증된 의료 및 법적 이벤트로, 자산 이전 프로토콜을 트리거합니다.

**시간 잠금 자산**: 시간적, 의료적 또는 법적 조건에 기반한 조건부 접근 제한이 있는 자산입니다.

## 2. 핵심 데이터 구조

### 2.1 자산 등록부 스키마

자산 등록부는 극저온 보존 중인 개인의 모든 등록된 자산을 포함하는 기본 데이터 구조입니다.

```json
{
  "$schema": "https://wia.dev/schemas/cryo-asset/v1/asset-registry.json",
  "registryId": "AR-2025-1234567890-ABCDEF",
  "version": "1.0.0",
  "status": "active",
  "created": "2025-12-18T10:30:00Z",
  "lastModified": "2025-12-18T10:30:00Z",
  "subject": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "홍길동",
    "dateOfBirth": "1985-06-15",
    "nationality": ["KOR", "USA"],
    "identityProof": {
      "type": "biometric_hash",
      "algorithm": "SHA3-512",
      "value": "a7f3c9e2b8d4f1a6e9c3b7d2f5a8e1c4b9d6f3a7e2c8b5d1f4a9e6c3b8d7f2a5",
      "biometricTypes": ["fingerprint", "iris", "dna"],
      "timestamp": "2025-12-18T10:00:00Z",
      "certificationAuthority": "WIA-BIO-CERT-001"
    },
    "preservationDetails": {
      "facility": "Alcor Life Extension Foundation",
      "facilityId": "ALCOR-AZ-001",
      "preservationDate": "2025-12-20T14:00:00Z",
      "preservationType": "vitrification",
      "contractNumber": "ALCOR-2025-12345",
      "medicalDirector": "Dr. Sarah Chen, MD",
      "emergencyContact": {
        "name": "김영희",
        "relationship": "spouse",
        "phone": "+82-10-1234-5678",
        "email": "younghee.kim@example.com"
      }
    }
  },
  "assets": {
    "financial": [],
    "realEstate": [],
    "intellectual": [],
    "digital": [],
    "personal": [],
    "business": []
  },
  "custodians": [],
  "revivalConditions": {},
  "legalFramework": {},
  "encryption": {
    "algorithm": "AES-256-GCM",
    "keyDerivation": "PBKDF2-SHA512",
    "iterations": 100000,
    "publicKey": "-----BEGIN PUBLIC KEY-----\nMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEA...\n-----END PUBLIC KEY-----"
  },
  "signatures": []
}
```

### 2.2 금융 자산 스키마

금융 자산에는 은행 계좌, 투자 포트폴리오, 암호화폐, 주식, 채권 및 기타 화폐 수단이 포함됩니다.

```json
{
  "assetId": "FA-2025-BTC-001",
  "assetType": "financial",
  "category": "cryptocurrency",
  "status": "active",
  "registered": "2025-12-18T10:35:00Z",
  "asset": {
    "name": "비트코인 보유분",
    "description": "주요 비트코인 지갑 및 거래소 계정",
    "cryptocurrency": {
      "symbol": "BTC",
      "network": "bitcoin",
      "wallets": [
        {
          "walletId": "WALLET-BTC-001",
          "type": "cold_storage",
          "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
          "balance": {
            "amount": "12.45678901",
            "unit": "BTC",
            "usdValue": 523456.78,
            "lastUpdated": "2025-12-18T10:30:00Z"
          },
          "accessMethod": "multisig",
          "requiredSignatures": 3,
          "totalSignatures": 5,
          "signatories": [
            {
              "id": "SIG-001",
              "type": "individual",
              "name": "홍길동 (대상자)",
              "publicKey": "xpub6D4BDPcP2GT577Vvch3R8wDkScZWzQzMMUm3PWbmWvVJrZwQY4VUNgqFJPMM3No2dFDFGTZ4BuFgoC8YVmzGmHqNdqJpdqMCAUf4NqfJ7JZ"
            },
            {
              "id": "SIG-002",
              "type": "custodian",
              "name": "신탁회사 알파",
              "publicKey": "xpub6ASAVgeehLbnwdqV6UKMHVzgqAG8Gr6riv3Fxxpj8ksbH9ebxaEyBLZ85ySDhKiLDBrQSARLq1uNRts8RuJiHjaDMBU4Zn9h8LZNnBC5y4a"
            },
            {
              "id": "SIG-003",
              "type": "custodian",
              "name": "재산 변호사",
              "publicKey": "xpub6FHa3pjLCk84BayeJxFW2SP4XRrFd1JYnxeLeU8EqN3vDfZmbqBqaGJAyiLjTAwm6ZLRQUMv1ZACTj37sR62cfN7fe5JnJ7dh8zL4fiyLHV"
            },
            {
              "id": "SIG-004",
              "type": "family",
              "name": "김영희 (배우자)",
              "publicKey": "xpub6D2jNUGkFVjXDJNPkPeRhYTfGQEfyhVWiCMTLXZJbCrEHJjsVrVKYgvhzJkLaXW8R1MfLf9KjSv3pJqzDqANpVJdXAKJz9FmXh3CvBJPMNo"
            },
            {
              "id": "SIG-005",
              "type": "revival_executor",
              "name": "WIA 소생 프로토콜 계약",
              "publicKey": "xpub6ERApfZwUBLFQtHZ2RqPvJFmxz8RLvXGrknDQxYJN8CJ6z4f8XZ7XqPrJPqJqdzTpXqvGRDxNnN8BJJBLKqJLKJqdzTpXqvGRDxNnN8BJJ"
            }
          ],
          "recoveryMethod": {
            "type": "shamir_secret_sharing",
            "threshold": 3,
            "shares": 5,
            "shareHolders": ["SIG-001", "SIG-002", "SIG-003", "SIG-004", "SIG-005"]
          },
          "timelock": {
            "enabled": true,
            "conditions": [
              {
                "type": "medical_verification",
                "description": "극저온 보존으로부터의 검증된 소생",
                "requiredDocuments": ["medical_certification", "neural_function_assessment"]
              },
              {
                "type": "legal_verification",
                "description": "소생 후 법적 지위를 인정하는 법원 명령",
                "jurisdiction": "Arizona, USA"
              },
              {
                "type": "identity_verification",
                "description": "보존 전 기록과 일치하는 생체인식 검증",
                "methods": ["fingerprint", "iris", "dna", "neural_pattern"]
              }
            ]
          }
        }
      ]
    }
  },
  "valuation": {
    "totalValue": 935796.90,
    "currency": "USD",
    "lastAssessed": "2025-12-18T10:30:00Z",
    "assessmentMethod": "market_rate",
    "volatilityRating": "high"
  },
  "management": {
    "strategy": "hold",
    "allowedActions": ["monitor", "rebalance_on_critical_threshold"],
    "prohibitedActions": ["sell", "transfer", "lend"],
    "rebalanceThresholds": {
      "maxValueDrop": 0.50,
      "maxValueIncrease": 5.0,
      "reviewTrigger": "quarterly"
    }
  },
  "custodian": {
    "primary": "신탁회사 알파",
    "backup": ["재산 변호사", "가족 신탁"],
    "custodyAgreementId": "CUSTODY-2025-001"
  }
}
```

### 2.3 부동산 자산 스키마

```json
{
  "assetId": "RE-2025-001",
  "assetType": "realEstate",
  "category": "residential",
  "status": "active",
  "registered": "2025-12-18T10:40:00Z",
  "asset": {
    "name": "주거용 주택",
    "propertyType": "single_family_home",
    "address": {
      "street": "강남대로 123",
      "city": "서울",
      "district": "강남구",
      "zipCode": "06000",
      "country": "KOR",
      "coordinates": {
        "latitude": 37.4979,
        "longitude": 127.0276
      }
    },
    "legalDescription": "서울특별시 강남구 역삼동 123번지, 지적도 제45호, 공부상 면적 250평방미터",
    "parcelNumber": "123-45-678-90",
    "deed": {
      "recordingNumber": "2020-0123456",
      "recordingDate": "2020-03-15",
      "registryOffice": "서울 강남구청",
      "documentType": "소유권이전등기",
      "owner": "홍길동",
      "documentHash": "sha256:b94d27b9934d3e08a52e52d7da7dabfac484efe37a5380ee9088f7ace2efcde9"
    },
    "ownership": {
      "type": "sole_ownership",
      "ownerName": "홍길동",
      "ownershipPercentage": 100.0,
      "vestingType": "완전소유권"
    },
    "physical": {
      "yearBuilt": 2018,
      "squareFootage": 2500,
      "bedrooms": 4,
      "bathrooms": 3,
      "parking": "지하주차장 2대",
      "lotSize": 250,
      "lotUnit": "평방미터"
    },
    "financial": {
      "purchasePrice": 1500000000,
      "purchaseDate": "2020-03-15",
      "currentValue": 1950000000,
      "assessedValue": 1920000000,
      "currency": "KRW",
      "lastAppraisal": {
        "date": "2025-06-01",
        "value": 1950000000,
        "appraiser": "한국부동산감정원",
        "appraisalId": "APP-2025-456789"
      }
    },
    "encumbrances": {
      "mortgages": [
        {
          "lender": "국민은행",
          "accountNumber": "****-****-****-5678",
          "originalAmount": 900000000,
          "currentBalance": 685000000,
          "interestRate": 3.2,
          "monthlyPayment": 4500000,
          "loanType": "20년 고정금리",
          "originationDate": "2020-03-15",
          "maturityDate": "2040-03-15"
        }
      ]
    },
    "propertyTax": {
      "annualAmount": 12000000,
      "paymentSchedule": "semi_annual",
      "lastPaymentDate": "2025-09-30",
      "nextPaymentDate": "2026-03-31",
      "currency": "KRW"
    }
  },
  "management": {
    "occupancyStatus": "owner_occupied",
    "duringPreservation": {
      "strategy": "rent_to_trusted_tenant",
      "manager": {
        "name": "서울부동산관리",
        "license": "PM-SEOUL-123456",
        "contact": "+82-2-1234-5678",
        "email": "info@seoulpropmgmt.co.kr",
        "managementFee": "월 임대료의 8%"
      },
      "rentalTerms": {
        "targetMonthlyRent": 5000000,
        "currency": "KRW",
        "leaseType": "annual",
        "tenantScreening": "comprehensive"
      }
    }
  }
}
```

### 2.4 지적재산권 자산 스키마

```json
{
  "assetId": "IP-2025-001",
  "assetType": "intellectual",
  "category": "patent",
  "status": "active",
  "registered": "2025-12-18T10:45:00Z",
  "asset": {
    "name": "신경 인터페이스 데이터 압축 방법",
    "type": "utility_patent",
    "patent": {
      "patentNumber": "KR-10-2234567",
      "title": "신경 인터페이스 데이터 스트림의 무손실 압축 방법 및 시스템",
      "filingDate": "2022-03-15",
      "issueDate": "2024-08-20",
      "expirationDate": "2042-03-15",
      "jurisdiction": "대한민국",
      "patentOffice": "특허청",
      "inventors": [
        {
          "name": "홍길동",
          "citizenship": "KOR",
          "contributionPercentage": 75.0
        },
        {
          "name": "이영희",
          "citizenship": "KOR",
          "contributionPercentage": 25.0
        }
      ],
      "assignee": {
        "name": "뉴럴테크이노베이션 주식회사",
        "type": "corporation",
        "jurisdiction": "대한민국",
        "ownershipStructure": {
          "hongGilDong": 80.0,
          "leeYoungHee": 20.0
        }
      },
      "claims": 23,
      "independentClaims": 3
    },
    "commercialization": {
      "status": "licensed",
      "licenses": [
        {
          "licenseId": "LIC-2024-001",
          "licensee": "메드테크글로벌 주식회사",
          "type": "exclusive",
          "territory": "worldwide",
          "field": "medical_devices",
          "effectiveDate": "2024-09-01",
          "expirationDate": "2034-08-31",
          "royalty": {
            "type": "percentage",
            "rate": 5.5,
            "minimumAnnual": 50000000,
            "currency": "KRW",
            "paymentSchedule": "quarterly"
          },
          "revenue": {
            "2024Q4": 75000000,
            "2025Q1": 82000000,
            "2025Q2": 89000000,
            "2025Q3": 95000000,
            "totalToDate": 341000000,
            "currency": "KRW"
          }
        }
      ]
    }
  }
}
```

## 3. 자산 분류 시스템

### 3.1 자산 유형 분류표

| 자산 유형 | 카테고리 | 하위 카테고리 | 유동성 | 복잡도 | 우선순위 |
|-----------|----------|-------------|--------|--------|---------|
| 금융 | 현금 | 은행계좌 | 높음 | 낮음 | 중요 |
| 금융 | 현금 | 머니마켓펀드 | 높음 | 낮음 | 중요 |
| 금융 | 투자 | 주식 | 높음 | 중간 | 높음 |
| 금융 | 투자 | 채권 | 중간 | 중간 | 높음 |
| 금융 | 암호화폐 | Bitcoin | 높음 | 높음 | 높음 |
| 금융 | 암호화폐 | Ethereum | 높음 | 높음 | 높음 |
| 금융 | 퇴직연금 | 개인연금 | 낮음 | 중간 | 중요 |
| 부동산 | 주거용 | 주택 | 낮음 | 높음 | 중요 |
| 부동산 | 주거용 | 임대부동산 | 낮음 | 높음 | 높음 |
| 부동산 | 상업용 | 사무실건물 | 낮음 | 높음 | 중간 |
| 지적재산권 | 특허 | 실용특허 | 낮음 | 높음 | 높음 |
| 지적재산권 | 저작권 | 소프트웨어 | 중간 | 높음 | 높음 |
| 지적재산권 | 상표권 | 워드마크 | 낮음 | 낮음 | 중간 |
| 디지털 | 도메인 | .com/.net/.kr | 중간 | 낮음 | 중간 |
| 디지털 | SNS | 계정 | 중간 | 중간 | 낮음 |
| 개인자산 | 차량 | 자동차 | 중간 | 낮음 | 낮음 |
| 사업자산 | 지분 | 법인주식 | 중간 | 높음 | 높음 |

### 3.2 우선순위 레벨 및 처리

| 우선순위 | 설명 | 응답 시간 | 모니터링 빈도 | 수탁자 요구사항 |
|----------|-----|----------|--------------|---------------|
| 중요 | 보존 비용, 법적 준수 또는 즉각적인 가족 지원에 필수적인 자산 | 24시간 | 매일 | 주 수탁자 + 백업 + 법률 |
| 높음 | 적극적 관리 또는 수익 창출이 필요한 중요한 가치 자산 | 1주 | 매주 | 주 수탁자 + 백업 |
| 중간 | 정기적 검토 및 표준 보호가 필요한 가치 있는 자산 | 1개월 | 매월 | 주 수탁자 |
| 낮음 | 감정적 또는 적은 금전적 가치를 가진 개인 물품 | 1분기 | 분기별 | 주 수탁자 |

## 4. 암호화 표준

### 4.1 암호화 요구사항

모든 민감한 자산 데이터는 다음 표준을 사용하여 암호화되어야 합니다:

```json
{
  "encryptionStandards": {
    "symmetric": {
      "algorithm": "AES-256-GCM",
      "keySize": 256,
      "mode": "GCM",
      "ivSize": 96,
      "tagSize": 128,
      "keyRotation": "annual"
    },
    "asymmetric": {
      "algorithm": "RSA-4096",
      "keySize": 4096,
      "padding": "OAEP",
      "hashFunction": "SHA-256",
      "mgf": "MGF1"
    },
    "keyDerivation": {
      "function": "PBKDF2",
      "hashFunction": "SHA-512",
      "iterations": 100000,
      "saltSize": 256
    },
    "hashing": {
      "algorithm": "SHA3-512",
      "iterations": 1,
      "purpose": "integrity_verification"
    },
    "quantumResistant": {
      "enabled": true,
      "algorithm": "CRYSTALS-Kyber",
      "securityLevel": 5,
      "notes": "장기 보존 시나리오에 권장됨"
    }
  }
}
```

### 4.2 디지털 서명 스키마

```json
{
  "signature": {
    "signatureId": "SIG-2025-ASSET-001",
    "algorithm": "ECDSA",
    "curve": "secp256k1",
    "hashFunction": "SHA3-256",
    "timestamp": "2025-12-18T10:30:00Z",
    "signer": {
      "id": "CRYO-IND-2025-987654",
      "name": "홍길동",
      "role": "subject",
      "publicKey": "04a7b1c3d5e7f9a2b4c6d8e0f1a3b5c7d9e1f3a5b7c9d1e3f5a7b9c1d3e5f7a9b1c3d5e7f9a2b4c6d8e0f1a3b5c7d9e1f3a5b7c9d1e3f5a7b9c1d3",
      "certificate": {
        "issuer": "WIA 인증기관",
        "serialNumber": "WIA-CERT-2025-001234",
        "validFrom": "2025-01-01T00:00:00Z",
        "validTo": "2026-01-01T00:00:00Z"
      }
    },
    "signatureValue": "3045022100f7a9c8b6d4e2f0a8c6b4d2e0f9a7c5b3d1e9f7a5c3b1d9e7f5a3c1b9e7f5a3c10220a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
    "witnesses": [
      {
        "witnessId": "WIT-001",
        "name": "김영희",
        "role": "spouse",
        "timestamp": "2025-12-18T10:31:00Z"
      },
      {
        "witnessId": "WIT-002",
        "name": "이철수 변호사",
        "role": "attorney",
        "timestamp": "2025-12-18T10:32:00Z"
      }
    ]
  }
}
```

## 5. 신원 및 접근 제어

### 5.1 생체인식 신원 스키마

```json
{
  "biometricIdentity": {
    "subjectId": "CRYO-IND-2025-987654",
    "created": "2025-12-18T09:00:00Z",
    "certificationAuthority": "WIA 생체인식 인증센터",
    "biometrics": [
      {
        "type": "fingerprint",
        "samples": 10,
        "hands": ["left", "right"],
        "fingers": ["thumb", "index", "middle", "ring", "pinky"],
        "algorithm": "minutiae_extraction",
        "templateFormat": "ISO_19794-2",
        "quality": 95,
        "hash": "sha3-512:a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8",
        "encrypted": true
      },
      {
        "type": "iris",
        "samples": 4,
        "eyes": ["left", "right"],
        "algorithm": "daugman_algorithm",
        "templateFormat": "ISO_19794-6",
        "quality": 98,
        "hash": "sha3-512:b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9",
        "encrypted": true
      },
      {
        "type": "dna",
        "samples": 1,
        "sequencingMethod": "whole_genome_sequencing",
        "coverage": "30x",
        "referenceGenome": "GRCh38",
        "markers": 1000000,
        "hash": "sha3-512:c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0",
        "encrypted": true,
        "laboratory": "한국게놈연구소",
        "certificationDate": "2025-12-10"
      }
    ],
    "compositeHash": "sha3-512:a7f3c9e2b8d4f1a6e9c3b7d2f5a8e1c4b9d6f3a7e2c8b5d1f4a9e6c3b8d7f2a5c1e4b8d6f3a9e7c2b5d8f1a4e9c6b3d7f2a8e5c1b4d9f6a3e7c2",
    "verificationThreshold": {
      "minimumBiometricMatches": 3,
      "minimumConfidenceScore": 0.95,
      "allowedBiometricTypes": ["fingerprint", "iris", "dna", "neural_pattern"],
      "fallbackMethods": ["legal_documentation", "witness_testimony"]
    }
  }
}
```

### 5.2 다중 서명 승인 스키마

```json
{
  "authorizationPolicy": {
    "policyId": "AUTH-POLICY-001",
    "assetScope": "all_assets",
    "created": "2025-12-18T10:30:00Z",
    "expiresOnRevival": true,
    "rules": [
      {
        "ruleId": "RULE-001",
        "action": "view_asset_information",
        "description": "자산 정보 조회",
        "requiredSignatures": 1,
        "authorizedRoles": ["subject", "primary_custodian", "backup_custodian", "legal_representative"],
        "timeRestrictions": null
      },
      {
        "ruleId": "RULE-002",
        "action": "modify_asset_metadata",
        "description": "자산 메타데이터 수정",
        "requiredSignatures": 2,
        "authorizedRoles": ["primary_custodian", "legal_representative"],
        "timeRestrictions": null,
        "approvalTimeout": "7 days"
      },
      {
        "ruleId": "RULE-003",
        "action": "transfer_asset_value_under_10000",
        "description": "1만 달러 미만 자산 이전",
        "requiredSignatures": 2,
        "authorizedRoles": ["primary_custodian", "backup_custodian"],
        "timeRestrictions": null,
        "approvalTimeout": "3 days",
        "purposeRestrictions": ["preservation_expenses", "emergency_family_support", "asset_maintenance"]
      },
      {
        "ruleId": "RULE-004",
        "action": "transfer_asset_value_over_10000",
        "description": "1만 달러 이상 자산 이전",
        "requiredSignatures": 3,
        "authorizedRoles": ["primary_custodian", "backup_custodian", "legal_representative"],
        "timeRestrictions": null,
        "approvalTimeout": "14 days",
        "purposeRestrictions": ["preservation_expenses", "emergency_family_support", "court_ordered"],
        "courtApprovalRequired": true
      },
      {
        "ruleId": "RULE-005",
        "action": "sell_or_liquidate_asset",
        "description": "자산 매각 또는 청산",
        "requiredSignatures": 4,
        "authorizedRoles": ["primary_custodian", "backup_custodian", "legal_representative", "family_representative"],
        "timeRestrictions": "only_after_1_year_preservation",
        "approvalTimeout": "30 days",
        "courtApprovalRequired": true,
        "purposeRestrictions": ["critical_preservation_expenses", "court_ordered"]
      }
    ]
  }
}
```

## 6. 코드 예제

### 6.1 자산 등록부 생성

```python
import hashlib
import json
from datetime import datetime
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.backends import default_backend

class CryoAssetRegistry:
    """극저온 자산 등록부 관리 클래스"""

    def __init__(self, individual_id, legal_name, date_of_birth):
        self.registry_id = self.generate_registry_id(individual_id)
        self.version = "1.0.0"
        self.status = "active"
        self.created = datetime.utcnow().isoformat() + "Z"
        self.subject = {
            "individualId": individual_id,
            "legalName": legal_name,
            "dateOfBirth": date_of_birth
        }
        self.assets = {
            "financial": [],
            "realEstate": [],
            "intellectual": [],
            "digital": [],
            "personal": [],
            "business": []
        }
        self.private_key, self.public_key = self.generate_key_pair()

    def generate_registry_id(self, individual_id):
        """등록부 ID 생성"""
        timestamp = str(int(datetime.utcnow().timestamp()))
        data = f"{individual_id}-{timestamp}".encode()
        hash_hex = hashlib.sha256(data).hexdigest()[:6].upper()
        return f"AR-2025-{timestamp}-{hash_hex}"

    def generate_key_pair(self):
        """RSA 키 쌍 생성"""
        private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=4096,
            backend=default_backend()
        )
        public_key = private_key.public_key()
        return private_key, public_key

    def add_financial_asset(self, asset_data):
        """금융 자산 추가"""
        asset_id = f"FA-2025-{asset_data['category'].upper()}-{len(self.assets['financial']) + 1:03d}"
        asset = {
            "assetId": asset_id,
            "assetType": "financial",
            "status": "active",
            "registered": datetime.utcnow().isoformat() + "Z",
            **asset_data
        }
        self.assets['financial'].append(asset)
        return asset_id

    def sign_registry(self):
        """등록부 서명"""
        registry_json = json.dumps(self.to_dict(), sort_keys=True)
        registry_hash = hashlib.sha3_256(registry_json.encode()).digest()

        signature = self.private_key.sign(
            registry_hash,
            padding.PSS(
                mgf=padding.MGF1(hashes.SHA256()),
                salt_length=padding.PSS.MAX_LENGTH
            ),
            hashes.SHA256()
        )

        return signature.hex()

# 사용 예제
registry = CryoAssetRegistry(
    individual_id="CRYO-IND-2025-987654",
    legal_name="홍길동",
    date_of_birth="1985-06-15"
)

# 비트코인 자산 추가
btc_asset = registry.add_financial_asset({
    "category": "cryptocurrency",
    "asset": {
        "name": "비트코인 보유분",
        "cryptocurrency": {
            "symbol": "BTC",
            "wallets": [
                {
                    "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
                    "balance": {"amount": "12.45678901", "unit": "BTC"}
                }
            ]
        }
    }
})

print(f"등록부 생성: {registry.registry_id}")
print(f"비트코인 자산 추가: {btc_asset}")
```

### 6.2 생체인식 해시 생성

```python
import hashlib
from typing import List, Dict
from datetime import datetime

class BiometricHasher:
    """생체인식 데이터 해시 생성기"""

    def __init__(self):
        self.algorithm = "sha3-512"

    def hash_biometric_sample(self, biometric_data: bytes, salt: bytes) -> str:
        """생체인식 샘플 해시 생성"""
        combined = salt + biometric_data
        hash_obj = hashlib.sha3_512(combined)
        return f"{self.algorithm}:{hash_obj.hexdigest()}"

    def create_composite_hash(self, biometric_hashes: List[str]) -> str:
        """복합 생체인식 해시 생성"""
        combined = "".join(sorted(biometric_hashes))
        hash_obj = hashlib.sha3_512(combined.encode())
        return f"{self.algorithm}:{hash_obj.hexdigest()}"

    def generate_biometric_identity(self, samples: Dict[str, bytes]) -> Dict:
        """생체인식 신원 생성"""
        salt = hashlib.sha256(str(datetime.utcnow().timestamp()).encode()).digest()

        biometric_hashes = []
        biometrics = []

        for biometric_type, data in samples.items():
            bio_hash = self.hash_biometric_sample(data, salt)
            biometric_hashes.append(bio_hash)

            biometrics.append({
                "type": biometric_type,
                "hash": bio_hash,
                "encrypted": True
            })

        composite_hash = self.create_composite_hash(biometric_hashes)

        return {
            "biometrics": biometrics,
            "compositeHash": composite_hash,
            "salt": salt.hex()
        }

# 사용 예제
hasher = BiometricHasher()

# 시뮬레이션된 생체인식 데이터
samples = {
    "fingerprint": b"지문_템플릿_데이터...",
    "iris": b"홍채_패턴_데이터...",
    "dna": b"DNA_시퀀스_데이터..."
}

identity = hasher.generate_biometric_identity(samples)
print(f"복합 생체인식 해시: {identity['compositeHash'][:80]}...")
```

### 6.3 다중 서명 트랜잭션 빌더

```javascript
const crypto = require('crypto');
const { ec: EC } = require('elliptic');
const ec = new EC('secp256k1');

class MultiSigAssetTransaction {
    /**
     * 다중 서명 자산 트랜잭션 관리 클래스
     */
    constructor(assetId, action, requiredSignatures) {
        this.transactionId = this.generateTransactionId();
        this.assetId = assetId;
        this.action = action;
        this.requiredSignatures = requiredSignatures;
        this.signatures = [];
        this.timestamp = new Date().toISOString();
        this.status = 'pending';
    }

    generateTransactionId() {
        const timestamp = Date.now();
        const random = crypto.randomBytes(8).toString('hex');
        return `TX-${timestamp}-${random.toUpperCase()}`;
    }

    getTransactionHash() {
        const data = JSON.stringify({
            transactionId: this.transactionId,
            assetId: this.assetId,
            action: this.action,
            timestamp: this.timestamp
        });
        return crypto.createHash('sha3-256').update(data).digest('hex');
    }

    addSignature(signerId, privateKeyHex) {
        if (this.signatures.length >= this.requiredSignatures) {
            throw new Error('모든 필수 서명이 이미 수집되었습니다');
        }

        const keyPair = ec.keyFromPrivate(privateKeyHex, 'hex');
        const txHash = this.getTransactionHash();
        const signature = keyPair.sign(txHash);

        this.signatures.push({
            signerId: signerId,
            signature: signature.toDER('hex'),
            timestamp: new Date().toISOString(),
            publicKey: keyPair.getPublic('hex')
        });

        if (this.signatures.length === this.requiredSignatures) {
            this.status = 'ready_to_execute';
        }

        return this.signatures.length;
    }

    verifySignatures() {
        const txHash = this.getTransactionHash();

        for (const sig of this.signatures) {
            const keyPair = ec.keyFromPublic(sig.publicKey, 'hex');
            const verified = keyPair.verify(txHash, sig.signature);

            if (!verified) {
                return false;
            }
        }

        return this.signatures.length === this.requiredSignatures;
    }

    execute() {
        if (!this.verifySignatures()) {
            throw new Error('유효하지 않거나 불충분한 서명');
        }

        this.status = 'executed';
        this.executedAt = new Date().toISOString();

        return {
            success: true,
            transactionId: this.transactionId,
            executedAt: this.executedAt
        };
    }
}

// 사용 예제
const transaction = new MultiSigAssetTransaction(
    'FA-2025-BTC-001',
    '보존_비용_이전',
    3
);

console.log(`트랜잭션 생성: ${transaction.transactionId}`);
```

---

**弘益人間 (홍익인간)** - 인류에 이로움
© 2025 WIA
MIT 라이선스
