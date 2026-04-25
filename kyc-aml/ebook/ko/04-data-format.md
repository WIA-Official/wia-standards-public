# 4장: 데이터 형식 사양

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:
- WIA KYC/AML 표준의 JSON 스키마 구조 이해
- 고객 프로필, 확인 결과 및 위험 평가 데이터 모델 작업
- 표준화된 데이터 형식을 사용하여 시스템 간 상호 운용성 구현
- 확장 메커니즘을 사용하여 사용자 정의 필드 추가
- 국제 지원을 위한 데이터 검증 및 형식 지정 규칙 적용

---

## 개요

이 장에서는 WIA KYC/AML 표준에서 사용되는 JSON 스키마를 자세히 설명합니다. 표준화된 데이터 형식은 시스템, 공급업체 및 금융 기관 간의 원활한 상호 운용성을 가능하게 합니다.

---

## 스키마 설계 원칙

### 1. 일관성
- 균일한 명명 규칙 (필드용 camelCase)
- 표준 데이터 유형 (ISO 날짜, 통화 코드)
- 엔티티 전반의 예측 가능한 구조

### 2. 확장성
- `extensions` 객체를 통한 사용자 정의 필드 지원
- 스키마 진화를 위한 버전 필드
- 유연성을 위한 선택적 필드

### 3. 검증
- 필수 필드 대 선택적 필드 명확히 정의
- 제한된 선택을 위한 열거형 값
- 형식 사양 (정규식, 길이, 범위)

### 4. 국제 지원
- 이름 및 주소에 대한 유니코드 지원
- `language` 필드를 통한 여러 언어
- 국가 코드 (ISO 3166-1 alpha-3)
- 통화 코드 (ISO 4217)

---

## 핵심 데이터 스키마

### 1. 개인 고객 프로필

**스키마:** `CustomerProfile-Individual-v1.0`

```json
{
  "schemaVersion": "1.0",
  "customerId": "CUST-789012",
  "type": "individual",
  "createdAt": "2025-01-09T10:30:00Z",
  "updatedAt": "2025-01-09T10:30:00Z",
  "status": "active",

  "personalInfo": {
    "title": "Mr",
    "firstName": "John",
    "middleName": "Michael",
    "lastName": "Smith",
    "fullName": "John Michael Smith",
    "preferredName": "John",
    "formerNames": [
      {
        "fullName": "John Smith Jr.",
        "type": "birth",
        "validFrom": "1985-06-15",
        "validTo": "2010-08-20"
      }
    ],
    "dateOfBirth": "1985-06-15",
    "gender": "male",
    "nationality": ["USA"],
    "citizenship": "USA",
    "countryOfBirth": "USA",
    "placeOfBirth": {
      "city": "New York",
      "state": "NY",
      "country": "USA"
    },
    "maritalStatus": "married",
    "dependents": 2
  },

  "identityDocuments": [
    {
      "documentId": "DOC-001",
      "type": "passport",
      "number": "N1234567",
      "issuingCountry": "USA",
      "issuingAuthority": "U.S. Department of State",
      "issueDate": "2020-06-14",
      "expiryDate": "2030-06-14",
      "placeOfIssue": "New York, NY",
      "verified": true,
      "verificationMethod": "automated_idv",
      "verificationDate": "2025-01-09T10:15:00Z",
      "verificationProvider": "Jumio",
      "documentImages": {
        "front": "https://secure-storage.example.com/docs/doc-001-front.enc",
        "back": null,
        "portrait": "https://secure-storage.example.com/docs/doc-001-portrait.enc"
      }
    },
    {
      "documentId": "DOC-002",
      "type": "drivers_license",
      "number": "D1234567",
      "issuingCountry": "USA",
      "issuingAuthority": "California DMV",
      "issuingState": "CA",
      "issueDate": "2022-03-10",
      "expiryDate": "2027-06-15",
      "verified": true,
      "verificationMethod": "manual_review",
      "verificationDate": "2025-01-09T10:20:00Z"
    }
  ],

  "biometricData": {
    "faceImageId": "BIO-FACE-001",
    "faceVerified": true,
    "faceVerificationDate": "2025-01-09T10:15:00Z",
    "livenessCheckPassed": true,
    "fingerprintCaptured": false
  },

  "contactInfo": {
    "email": {
      "address": "john.smith@example.com",
      "verified": true,
      "verifiedDate": "2025-01-09T09:45:00Z",
      "primary": true
    },
    "phone": {
      "number": "+1-555-0123",
      "countryCode": "+1",
      "type": "mobile",
      "verified": true,
      "verifiedDate": "2025-01-09T09:50:00Z",
      "primary": true
    },
    "alternativeContacts": [
      {
        "type": "email",
        "value": "j.smith@work.example.com",
        "verified": false
      }
    ]
  },

  "addresses": [
    {
      "addressId": "ADDR-001",
      "type": "residential",
      "primary": true,
      "street": "123 Main Street",
      "unit": "Apt 4B",
      "city": "San Francisco",
      "state": "CA",
      "postalCode": "94102",
      "country": "USA",
      "countryCode": "USA",
      "verified": true,
      "verificationMethod": "utility_bill",
      "verificationDate": "2025-01-09T10:25:00Z",
      "residingSince": "2020-01-15"
    }
  ],

  "employment": {
    "status": "employed",
    "employer": {
      "name": "Tech Corp Inc.",
      "industry": "Software Development",
      "address": {
        "street": "456 Market St",
        "city": "San Francisco",
        "state": "CA",
        "country": "USA"
      }
    },
    "occupation": "Software Engineer",
    "title": "Senior Developer",
    "employedSince": "2018-03-01",
    "annualIncome": {
      "amount": 150000,
      "currency": "USD",
      "verified": false
    }
  },

  "financialProfile": {
    "sourceOfFunds": ["employment", "investments"],
    "sourceOfWealth": ["employment", "inheritance"],
    "estimatedNetWorth": {
      "range": "500000-1000000",
      "currency": "USD"
    },
    "expectedAccountActivity": {
      "monthlyDeposits": {
        "count": "5-10",
        "amount": {
          "min": 1000,
          "max": 15000,
          "currency": "USD"
        }
      },
      "monthlyWithdrawals": {
        "count": "10-20",
        "amount": {
          "min": 500,
          "max": 10000,
          "currency": "USD"
        }
      },
      "internationalTransactions": true,
      "typicalCountries": ["USA", "CAN", "GBR"]
    }
  },

  "riskProfile": {
    "overallRisk": {
      "category": "low",
      "score": 25,
      "assessmentDate": "2025-01-09T10:30:00Z",
      "assessmentMethod": "automated_scoring_v2.1",
      "nextReviewDate": "2027-01-09"
    },
    "factors": [
      {
        "dimension": "geographic",
        "score": 10,
        "details": "저위험 관할권, 고위험 연결 없음"
      },
      {
        "dimension": "occupation",
        "score": 15,
        "details": "표준 기업 고용"
      },
      {
        "dimension": "transactionProfile",
        "score": 20,
        "details": "소득과 일치하는 예상 활동"
      }
    ]
  },

  "screening": {
    "sanctionsScreening": {
      "status": "clear",
      "lastScreened": "2025-01-09T10:12:00Z",
      "nextScheduled": "2025-01-10T10:12:00Z",
      "listsChecked": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"]
    },
    "pepScreening": {
      "status": "not_pep",
      "lastScreened": "2025-01-09T10:12:00Z",
      "tier": null
    },
    "adverseMedia": {
      "status": "clear",
      "lastScreened": "2025-01-09T10:12:00Z"
    }
  },

  "relationships": {
    "powerOfAttorney": [],
    "authorizedSigners": [],
    "beneficiaries": [],
    "relatedCustomers": []
  },

  "compliance": {
    "cddLevel": "standard",
    "cddCompletedDate": "2025-01-09T10:30:00Z",
    "cddExpiryDate": "2027-01-09",
    "eddRequired": false,
    "taxResidency": ["USA"],
    "taxIdentificationNumbers": [
      {
        "type": "SSN",
        "country": "USA",
        "number": "XXX-XX-1234",
        "verified": true
      }
    ],
    "fatcaStatus": {
      "applicable": true,
      "classification": "US_person",
      "reportable": false
    },
    "crsStatus": {
      "applicable": false
    }
  },

  "onboarding": {
    "channel": "web",
    "date": "2025-01-09T09:30:00Z",
    "ipAddress": "192.0.2.100",
    "deviceFingerprint": "fp-abc123def456",
    "referralSource": "organic_search",
    "approvedBy": "system_auto_approval",
    "approvalDate": "2025-01-09T10:30:00Z"
  },

  "metadata": {
    "dataClassification": "highly_confidential",
    "retentionPolicy": "7_years_post_closure",
    "consentRecords": [
      {
        "type": "data_processing",
        "version": "1.2",
        "consentedAt": "2025-01-09T09:35:00Z",
        "ipAddress": "192.0.2.100"
      },
      {
        "type": "marketing_communications",
        "version": "1.0",
        "consentedAt": "2025-01-09T09:36:00Z",
        "opted": false
      }
    ]
  },

  "extensions": {
    "customField1": "value",
    "institutionSpecificData": {}
  }
}
```

---

### 2. 기업 고객 프로필 (KYB)

**스키마:** `CustomerProfile-Business-v1.0`

```json
{
  "schemaVersion": "1.0",
  "customerId": "CORP-456789",
  "type": "business",
  "createdAt": "2025-01-09T11:00:00Z",
  "updatedAt": "2025-01-09T11:00:00Z",
  "status": "active",

  "companyInfo": {
    "legalName": "Example Corp Inc.",
    "tradingNames": ["Example Corp", "Ex Corp"],
    "formerNames": [],
    "registrationNumber": "12-3456789",
    "registrationCountry": "USA",
    "registrationState": "Delaware",
    "registrationDate": "2010-03-15",
    "incorporationType": "C-Corporation",
    "taxIdentificationNumber": "98-7654321",
    "lei": "LEI1234567890ABCDEF12",
    "website": "https://example-corp.com",
    "businessType": "private",
    "publiclyListed": false,
    "stockExchange": null,
    "tickerSymbol": null
  },

  "businessActivity": {
    "primaryIndustry": "Software Development",
    "industryCodes": {
      "naics": "541511",
      "sic": "7371",
      "nace": "62.01"
    },
    "businessDescription": "금융 서비스 회사를 위한 엔터프라이즈 소프트웨어 솔루션 제공, 규정 준수 및 위험 관리 플랫폼 전문",
    "productServices": [
      "RegTech 소프트웨어",
      "AML 규정 준수 솔루션",
      "위험 관리 플랫폼"
    ],
    "targetMarkets": ["북미", "유럽", "아시아 태평양"],
    "estimatedAnnualRevenue": {
      "amount": 25000000,
      "currency": "USD",
      "year": 2024
    },
    "numberOfEmployees": 150,
    "businessModel": "B2B SaaS"
  },

  "registeredAddress": {
    "street": "789 Corporate Blvd",
    "suite": "Suite 1000",
    "city": "Wilmington",
    "state": "DE",
    "postalCode": "19801",
    "country": "USA"
  },

  "operationalAddresses": [
    {
      "type": "headquarters",
      "street": "456 Market St",
      "floor": "12th Floor",
      "city": "San Francisco",
      "state": "CA",
      "postalCode": "94105",
      "country": "USA",
      "primary": true
    },
    {
      "type": "branch",
      "street": "10 Downing Street",
      "city": "London",
      "postalCode": "SW1A 2AA",
      "country": "GBR"
    }
  ],

  "contactInfo": {
    "email": "info@example-corp.com",
    "phone": "+1-555-0199",
    "website": "https://example-corp.com"
  },

  "ownershipStructure": {
    "beneficialOwners": [
      {
        "customerId": "CUST-111111",
        "type": "individual",
        "name": "Jane Doe",
        "dateOfBirth": "1975-05-20",
        "nationality": "USA",
        "ownershipPercentage": 55.0,
        "ownershipType": "direct",
        "votingRights": 55.0,
        "controlMechanism": ["majority_ownership", "voting_rights"],
        "isPep": false,
        "acquisitionDate": "2010-03-15",
        "verificationStatus": "verified",
        "verificationDate": "2025-01-09T11:00:00Z"
      },
      {
        "customerId": "CUST-222222",
        "type": "individual",
        "name": "Robert Johnson",
        "dateOfBirth": "1978-09-12",
        "nationality": "USA",
        "ownershipPercentage": 45.0,
        "ownershipType": "direct",
        "votingRights": 45.0,
        "controlMechanism": ["significant_ownership", "voting_rights"],
        "isPep": false,
        "acquisitionDate": "2010-03-15",
        "verificationStatus": "verified",
        "verificationDate": "2025-01-09T11:00:00Z"
      }
    ],
    "ownershipThreshold": 25.0,
    "complexStructure": false,
    "trusts": [],
    "nominees": []
  },

  "controlPersons": [
    {
      "customerId": "CUST-111111",
      "role": "ceo",
      "title": "Chief Executive Officer",
      "appointedDate": "2010-03-15",
      "authority": ["executive_decisions", "contract_signing", "banking_authority"]
    },
    {
      "customerId": "CUST-333333",
      "role": "cfo",
      "title": "Chief Financial Officer",
      "appointedDate": "2015-06-01",
      "authority": ["financial_decisions", "banking_authority"]
    }
  ],

  "boardOfDirectors": [
    {
      "customerId": "CUST-111111",
      "name": "Jane Doe",
      "position": "Chairman & CEO",
      "appointedDate": "2010-03-15"
    },
    {
      "customerId": "CUST-222222",
      "name": "Robert Johnson",
      "position": "Director",
      "appointedDate": "2010-03-15"
    },
    {
      "customerId": "CUST-444444",
      "name": "Maria Garcia",
      "position": "Independent Director",
      "appointedDate": "2018-01-10"
    }
  ],

  "registrationDocuments": [
    {
      "documentType": "certificate_of_incorporation",
      "documentId": "DOC-CORP-001",
      "issueDate": "2010-03-15",
      "issuingAuthority": "Delaware Division of Corporations",
      "verified": true,
      "verificationDate": "2025-01-09T11:00:00Z"
    },
    {
      "documentType": "articles_of_association",
      "documentId": "DOC-CORP-002",
      "issueDate": "2010-03-15",
      "verified": true
    },
    {
      "documentType": "good_standing_certificate",
      "documentId": "DOC-CORP-003",
      "issueDate": "2024-12-01",
      "expiryDate": "2025-12-01",
      "verified": true
    }
  ],

  "licenses": [
    {
      "type": "business_license",
      "number": "BL-2010-12345",
      "issuingAuthority": "City of San Francisco",
      "issueDate": "2015-01-01",
      "expiryDate": "2026-01-01",
      "jurisdiction": "San Francisco, CA",
      "status": "active"
    }
  ],

  "financialInformation": {
    "bankAccounts": [
      {
        "accountId": "ACC-001",
        "institution": "Example Bank",
        "country": "USA",
        "currency": "USD",
        "accountType": "business_checking",
        "openedDate": "2015-01-15"
      }
    ],
    "sourceOfFunds": ["operating_revenue", "venture_capital"],
    "expectedAccountActivity": {
      "monthlyTransactions": {
        "count": "100-500",
        "volume": {
          "min": 500000,
          "max": 3000000,
          "currency": "USD"
        }
      },
      "internationalTransactions": true,
      "typicalCountries": ["USA", "GBR", "SGP", "DEU"],
      "largeTransactions": {
        "frequency": "monthly",
        "averageSize": 100000
      }
    }
  },

  "riskProfile": {
    "overallRisk": {
      "category": "medium",
      "score": 45,
      "assessmentDate": "2025-01-09T11:00:00Z",
      "nextReviewDate": "2026-01-09"
    },
    "factors": [
      {
        "dimension": "geographic",
        "score": 30,
        "details": "고위험 지역을 포함한 여러 관할권에서 운영"
      },
      {
        "dimension": "industry",
        "score": 40,
        "details": "소프트웨어/기술 부문 - 중간 위험"
      },
      {
        "dimension": "complexity",
        "score": 35,
        "details": "국제 운영, 여러 자회사"
      },
      {
        "dimension": "ownership",
        "score": 25,
        "details": "명확한 소유 구조, PEP 없음"
      }
    ]
  },

  "screening": {
    "entityScreening": {
      "status": "clear",
      "lastScreened": "2025-01-09T11:00:00Z",
      "listsChecked": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"]
    },
    "uboScreening": {
      "status": "clear",
      "lastScreened": "2025-01-09T11:00:00Z",
      "allUbosCleared": true
    },
    "adverseMedia": {
      "status": "clear",
      "lastScreened": "2025-01-09T11:00:00Z"
    }
  },

  "compliance": {
    "cddLevel": "enhanced",
    "cddCompletedDate": "2025-01-09T11:00:00Z",
    "cddExpiryDate": "2026-01-09",
    "eddRequired": true,
    "eddReason": "high_transaction_volume",
    "eddCompletedDate": "2025-01-09T12:00:00Z"
  }
}
```

---

### 3. 신원 확인 결과

**스키마:** `IdentityVerificationResult-v1.0`

```json
{
  "schemaVersion": "1.0",
  "verificationId": "IDV-2025-001234",
  "customerId": "CUST-789012",
  "initiatedAt": "2025-01-09T10:10:00Z",
  "completedAt": "2025-01-09T10:15:00Z",
  "status": "verified",
  "overallResult": "pass",
  "confidenceScore": 0.94,

  "verificationMethod": "automated_with_manual_review",
  "provider": "Jumio",

  "documentVerification": {
    "documentId": "DOC-001",
    "documentType": "passport",
    "documentNumber": "N1234567",
    "issuingCountry": "USA",

    "authenticityCheck": {
      "status": "pass",
      "confidence": 0.96,
      "methods": ["mrz_validation", "security_features", "template_matching", "font_analysis"],
      "securityFeatures": {
        "hologram": "detected",
        "watermark": "detected",
        "microprint": "detected",
        "uvFeatures": "detected"
      }
    },

    "dataExtraction": {
      "status": "success",
      "confidence": 0.98,
      "extractedData": {
        "firstName": "JOHN",
        "middleName": "MICHAEL",
        "lastName": "SMITH",
        "dateOfBirth": "1985-06-15",
        "documentNumber": "N1234567",
        "nationality": "USA",
        "sex": "M",
        "issueDate": "2020-06-14",
        "expiryDate": "2030-06-14",
        "placeOfBirth": "NEW YORK, NY, USA"
      },
      "mrzData": {
        "line1": "P<USASMITH<<JOHN<MICHAEL<<<<<<<<<<<<<<<<<<<<",
        "line2": "N1234567<9USA8506154M3006145<<<<<<<<<<<<<<02"
      },
      "mrzVerified": true
    },

    "tamperingCheck": {
      "status": "pass",
      "confidence": 0.95,
      "indicators": []
    },

    "expiryCheck": {
      "status": "pass",
      "expiryDate": "2030-06-14",
      "daysUntilExpiry": 1982
    },

    "documentImages": {
      "original": "https://secure.example.com/img-original.enc",
      "cropped": "https://secure.example.com/img-cropped.enc",
      "faceExtract": "https://secure.example.com/img-face.enc"
    }
  },

  "biometricVerification": {
    "type": "facial",

    "livenessDetection": {
      "status": "pass",
      "confidence": 0.93,
      "method": "active",
      "challenges": ["smile", "turn_left", "blink"],
      "videoRecorded": true,
      "videoUrl": "https://secure.example.com/liveness-video.enc"
    },

    "faceMatching": {
      "status": "match",
      "similarity": 0.91,
      "threshold": 0.85,
      "documentFaceQuality": 0.88,
      "selfieFaceQuality": 0.92,
      "algorithm": "FaceNet_v2"
    }
  },

  "dataValidation": {
    "nameValidation": {
      "status": "pass",
      "matchScore": 1.0,
      "providedName": "John Michael Smith",
      "documentName": "JOHN MICHAEL SMITH"
    },

    "dateOfBirthValidation": {
      "status": "pass",
      "providedDOB": "1985-06-15",
      "documentDOB": "1985-06-15",
      "ageVerified": true,
      "minimumAgeRequirement": 18,
      "customerAge": 39
    },

    "addressValidation": {
      "status": "verified",
      "method": "utility_bill_upload",
      "documentType": "electricity_bill",
      "issueDate": "2024-12-15",
      "matchScore": 0.95,
      "providedAddress": "123 Main St, Apt 4B, San Francisco, CA 94102",
      "documentAddress": "123 Main Street, Apartment 4B, San Francisco, CA 94102"
    }
  },

  "databaseChecks": {
    "creditBureauCheck": {
      "provider": "Experian",
      "status": "match",
      "confidence": 0.89,
      "matchedFields": ["name", "dob", "address", "ssn"]
    },

    "emailVerification": {
      "email": "john.smith@example.com",
      "status": "verified",
      "method": "otp",
      "verifiedAt": "2025-01-09T09:45:00Z",
      "emailQuality": {
        "formatValid": true,
        "domainValid": true,
        "smtpValid": true,
        "disposable": false,
        "businessEmail": false
      }
    },

    "phoneVerification": {
      "phone": "+1-555-0123",
      "status": "verified",
      "method": "sms_otp",
      "verifiedAt": "2025-01-09T09:50:00Z",
      "phoneQuality": {
        "valid": true,
        "type": "mobile",
        "carrier": "Verizon",
        "country": "USA",
        "voip": false
      }
    }
  },

  "riskSignals": {
    "fraudIndicators": [],
    "warnings": [],
    "informational": [
      "문서가 5년 이상 후에 만료됨 - 양호한 유효 기간"
    ]
  },

  "manualReview": {
    "required": false,
    "performed": false
  },

  "complianceChecks": {
    "minimumAge": {
      "required": 18,
      "actual": 39,
      "pass": true
    },
    "sanctionsScreening": {
      "performed": true,
      "status": "clear"
    },
    "pepScreening": {
      "performed": true,
      "status": "not_pep"
    }
  },

  "metadata": {
    "ipAddress": "192.0.2.100",
    "userAgent": "Mozilla/5.0...",
    "deviceFingerprint": "fp-abc123",
    "geolocation": {
      "country": "USA",
      "region": "CA",
      "city": "San Francisco"
    },
    "sessionId": "sess-xyz789"
  }
}
```

---

### 4. 위험 평가

**스키마:** `RiskAssessment-v1.0`

```json
{
  "schemaVersion": "1.0",
  "assessmentId": "RISK-2025-001234",
  "customerId": "CUST-789012",
  "assessmentDate": "2025-01-09T10:30:00Z",
  "assessmentType": "initial",
  "assessmentMethod": "automated_scoring_v2.1",
  "assessedBy": "system",

  "overallRisk": {
    "score": 35,
    "category": "low",
    "previousScore": null,
    "previousCategory": null,
    "scoreChange": null
  },

  "riskDimensions": [
    {
      "dimension": "geographic",
      "weight": 0.25,
      "rawScore": 25,
      "weightedScore": 6.25,
      "factors": [
        {
          "factor": "residence_country",
          "value": "USA",
          "score": 10,
          "rationale": "저위험 관할권"
        },
        {
          "factor": "citizenship",
          "value": "USA",
          "score": 10,
          "rationale": "저위험 관할권"
        },
        {
          "factor": "transaction_countries",
          "value": ["USA", "CAN", "GBR"],
          "score": 20,
          "rationale": "모두 저위험 관할권"
        }
      ]
    },
    {
      "dimension": "product_service",
      "weight": 0.20,
      "rawScore": 30,
      "weightedScore": 6.0,
      "factors": [
        {
          "factor": "account_type",
          "value": "checking_with_international_wire",
          "score": 35,
          "rationale": "국제 송금 기능이 위험 추가"
        },
        {
          "factor": "expected_volume",
          "value": "medium",
          "score": 25,
          "rationale": "보통 거래량"
        }
      ]
    },
    {
      "dimension": "customer_type",
      "weight": 0.20,
      "rawScore": 20,
      "weightedScore": 4.0,
      "factors": [
        {
          "factor": "customer_category",
          "value": "individual_retail",
          "score": 15,
          "rationale": "개인 소매 고객 - 낮은 복잡성"
        },
        {
          "factor": "occupation",
          "value": "software_engineer",
          "score": 20,
          "rationale": "전문직 - 표준 위험"
        },
        {
          "factor": "employer_type",
          "value": "established_corporation",
          "score": 10,
          "rationale": "평판 좋은 고용주"
        }
      ]
    },
    {
      "dimension": "behavioral",
      "weight": 0.20,
      "rawScore": 15,
      "weightedScore": 3.0,
      "factors": [
        {
          "factor": "source_of_funds",
          "value": "employment",
          "score": 10,
          "rationale": "명확하고 합법적인 출처"
        },
        {
          "factor": "transaction_purpose",
          "value": "personal_banking",
          "score": 15,
          "rationale": "표준 개인 사용"
        },
        {
          "factor": "expected_vs_actual",
          "value": "new_customer",
          "score": 20,
          "rationale": "아직 거래 기록 없음"
        }
      ]
    },
    {
      "dimension": "relationship",
      "weight": 0.15,
      "rawScore": 10,
      "weightedScore": 1.5,
      "factors": [
        {
          "factor": "pep_status",
          "value": false,
          "score": 0,
          "rationale": "PEP 아님"
        },
        {
          "factor": "sanctions_match",
          "value": false,
          "score": 0,
          "rationale": "제재 일치 없음"
        },
        {
          "factor": "adverse_media",
          "value": false,
          "score": 0,
          "rationale": "부정적 미디어 발견 안됨"
        },
        {
          "factor": "known_associates",
          "value": "none",
          "score": 10,
          "rationale": "고위험 연관 없음"
        }
      ]
    }
  ],

  "calculationDetails": {
    "formula": "weighted_average",
    "totalWeight": 1.0,
    "weightedSum": 20.75,
    "adjustments": [
      {
        "type": "fraud_score_adjustment",
        "value": 5,
        "rationale": "신원 확인의 낮은 사기 지표"
      }
    ],
    "finalScore": 35
  },

  "riskMitigationControls": {
    "cddLevel": "standard",
    "transactionLimits": {
      "dailyWithdrawal": {
        "amount": 5000,
        "currency": "USD"
      },
      "monthlyInternationalWire": {
        "amount": 50000,
        "currency": "USD"
      }
    },
    "enhancedMonitoring": false,
    "additionalDocumentation": []
  },

  "recommendations": {
    "approvalDecision": "approve",
    "cddLevel": "standard",
    "reviewFrequency": "biennial",
    "nextReviewDate": "2027-01-09",
    "specialConditions": [],
    "escalation": false
  },

  "regulatoryAlignment": {
    "fatfCompliant": true,
    "jurisdictionSpecific": {
      "USA": {
        "bsaCompliant": true,
        "patriotActCompliant": true
      }
    }
  }
}
```

---

### 5. 거래 모니터링 경보

**스키마:** `TransactionMonitoringAlert-v1.0`

```json
{
  "schemaVersion": "1.0",
  "alertId": "ALERT-2025-123456",
  "customerId": "CUST-789012",
  "generatedAt": "2025-01-09T14:30:00Z",
  "status": "open",
  "priority": "high",

  "alertType": "behavioral_anomaly",
  "scenario": "rapid_funds_movement",
  "scenarioCode": "TM-045",

  "description": "24시간 이내에 대규모 예금 후 빠른 인출",

  "triggeredRules": [
    {
      "ruleId": "TM-045",
      "ruleName": "Rapid Funds Movement",
      "ruleVersion": "2.1",
      "triggerTime": "2025-01-09T14:30:00Z",
      "severity": "high",
      "confidence": 0.87
    }
  ],

  "transactions": [
    {
      "transactionId": "TXN-001-20250109",
      "timestamp": "2025-01-09T10:00:00Z",
      "type": "wire_credit",
      "amount": 45000,
      "currency": "USD",
      "accountId": "ACC-CUST-789012-001",
      "counterparty": {
        "name": "ABC Trading Ltd",
        "account": "XXXX-5678",
        "bank": "Foreign Bank Corp",
        "country": "SGP"
      },
      "description": "컨설팅 서비스에 대한 지불"
    },
    {
      "transactionId": "TXN-002-20250109",
      "timestamp": "2025-01-09T16:30:00Z",
      "type": "wire_debit",
      "amount": 43000,
      "currency": "USD",
      "accountId": "ACC-CUST-789012-001",
      "counterparty": {
        "name": "XYZ Investments",
        "account": "XXXX-9012",
        "bank": "Another Foreign Bank",
        "country": "HKG"
      },
      "description": "투자"
    }
  ],

  "riskIndicators": [
    {
      "indicator": "rapid_turnaround",
      "value": "6.5 hours",
      "threshold": "48 hours",
      "severity": "high"
    },
    {
      "indicator": "withdrawal_percentage",
      "value": "95.6%",
      "threshold": "90%",
      "severity": "medium"
    },
    {
      "indicator": "foreign_counterparties",
      "value": 2,
      "severity": "medium"
    },
    {
      "indicator": "new_relationship",
      "value": "두 당사자 모두 첫 거래",
      "severity": "medium"
    }
  ],

  "customerContext": {
    "riskCategory": "low",
    "accountAge": "1 day",
    "averageMonthlyVolume": null,
    "historicalAlerts": 0,
    "pepStatus": false,
    "occupation": "Software Engineer",
    "expectedActivity": "개인 은행 업무, 가끔 국제 송금"
  },

  "mlScore": {
    "suspicionScore": 0.82,
    "model": "anomaly_detection_v3.2",
    "confidence": 0.87,
    "peerComparison": {
      "percentile": 98,
      "interpretation": "유사한 고객에 비해 매우 비정상적"
    }
  },

  "recommendedActions": [
    "manual_investigation",
    "request_supporting_documentation",
    "contact_customer"
  ],

  "investigation": {
    "required": true,
    "assignedTo": null,
    "assignedAt": null,
    "dueDate": "2025-01-11T14:30:00Z",
    "sla": "48_hours"
  },

  "disposition": {
    "status": "pending",
    "dispositionDate": null,
    "disposedBy": null,
    "outcome": null,
    "notes": null,
    "sarFiled": false
  }
}
```

---

### 6. 스크리닝 결과

**스키마:** `ScreeningResult-v1.0`

```json
{
  "schemaVersion": "1.0",
  "screeningId": "SCR-2025-001234",
  "customerId": "CUST-789012",
  "screeningType": "comprehensive",
  "executedAt": "2025-01-09T10:12:00Z",
  "provider": "Dow Jones Risk & Compliance",

  "searchCriteria": {
    "name": "John Michael Smith",
    "dateOfBirth": "1985-06-15",
    "nationality": "USA",
    "identificationNumbers": [],
    "fuzzyMatching": true,
    "threshold": 85
  },

  "sanctionsScreening": {
    "status": "no_match",
    "listsChecked": [
      {
        "listName": "OFAC_SDN",
        "listVersion": "2025-01-09",
        "recordCount": 12458,
        "matches": 0
      },
      {
        "listName": "OFAC_NON_SDN",
        "listVersion": "2025-01-09",
        "recordCount": 8742,
        "matches": 0
      },
      {
        "listName": "UN_SC_CONSOLIDATED",
        "listVersion": "2025-01-08",
        "recordCount": 1523,
        "matches": 0
      },
      {
        "listName": "EU_SANCTIONS",
        "listVersion": "2025-01-09",
        "recordCount": 2891,
        "matches": 0
      }
    ],
    "totalRecordsChecked": 25614,
    "nextScheduledScreen": "2025-01-10T10:12:00Z"
  },

  "pepScreening": {
    "status": "potential_match",
    "matches": [
      {
        "matchId": "PEP-2025-5678",
        "confidence": 78,
        "requiresReview": true,
        "profile": {
          "name": "Jon Smith",
          "alternateNames": ["Jonathan Smith"],
          "dateOfBirth": "1985-06-10",
          "nationality": "USA",
          "position": "Deputy Minister of Finance",
          "organization": "Ministry of Finance",
          "country": "Hypothetica",
          "pepTier": 2,
          "pepCategory": "government_official",
          "activeFrom": "2020-01-01",
          "activeTo": null,
          "status": "active"
        },
        "matchReasons": [
          "이름 유사성: 92%",
          "생년월일 근접하지만 정확하지 않음 (5일 차이)",
          "동일한 국적"
        ],
        "differentiators": [
          "다른 중간 이름",
          "생년월일 정확히 일치하지 않음",
          "다른 식별 정보 일치하지 않음"
        ],
        "recommendation": "manual_review"
      }
    ]
  },

  "adverseMediaScreening": {
    "status": "clear",
    "articlesReviewed": 0,
    "categories": [
      "financial_crime",
      "fraud",
      "corruption",
      "money_laundering",
      "sanctions_violations"
    ],
    "dateRange": {
      "from": "2020-01-09",
      "to": "2025-01-09"
    }
  },

  "enforcementLists": {
    "status": "no_match",
    "listsChecked": [
      "FBI_Most_Wanted",
      "Interpol_Red_Notices",
      "FinCEN_314a",
      "State_Regulatory_Actions"
    ]
  },

  "overallRisk": "low_risk_pending_review",
  "recommendedAction": "manual_review_pep_match",

  "reviewRequired": true,
  "reviewReason": "잠재적 PEP 일치 확인 필요",
  "reviewDueDate": "2025-01-10T10:12:00Z",

  "continuousMonitoring": {
    "enabled": true,
    "frequency": "daily",
    "nextScheduled": "2025-01-10T10:12:00Z"
  }
}
```

---

### 7. 사례 기록

**스키마:** `CaseRecord-v1.0`

```json
{
  "schemaVersion": "1.0",
  "caseId": "CASE-2025-001234",
  "caseType": "suspicious_activity_investigation",
  "status": "under_investigation",
  "priority": "high",

  "customerId": "CUST-789012",
  "customerName": "John Michael Smith",

  "createdAt": "2025-01-09T14:45:00Z",
  "createdBy": "system",
  "assignedTo": "analyst_jdoe",
  "assignedAt": "2025-01-09T15:00:00Z",

  "trigger": {
    "type": "transaction_monitoring_alert",
    "sourceId": "ALERT-2025-123456",
    "description": "빠른 자금 이동 - 고액"
  },

  "investigation": {
    "startedAt": "2025-01-09T15:00:00Z",
    "investigator": {
      "userId": "analyst_jdoe",
      "name": "Jane Doe",
      "title": "Senior AML Analyst"
    },
    "steps": [
      {
        "stepId": "INV-STEP-001",
        "timestamp": "2025-01-09T15:05:00Z",
        "action": "review_alert_details",
        "performedBy": "analyst_jdoe",
        "notes": "트리거 거래 및 위험 지표 검토",
        "duration": 300
      },
      {
        "stepId": "INV-STEP-002",
        "timestamp": "2025-01-09T15:15:00Z",
        "action": "review_customer_profile",
        "performedBy": "analyst_jdoe",
        "notes": "고객은 신규 (계정 개설 1일 전), 저위험 프로필, 고용된 전문가",
        "duration": 600
      },
      {
        "stepId": "INV-STEP-003",
        "timestamp": "2025-01-09T15:30:00Z",
        "action": "review_transaction_history",
        "performedBy": "analyst_jdoe",
        "notes": "총 2건의 거래만 - 경보를 트리거한 거래. 이전 활동 없음.",
        "duration": 900
      },
      {
        "stepId": "INV-STEP-004",
        "timestamp": "2025-01-09T16:00:00Z",
        "action": "external_database_search",
        "performedBy": "analyst_jdoe",
        "notes": "LexisNexis 검색, 부정적 발견 없음. 고객이 합법적으로 보임.",
        "sources": ["LexisNexis", "Public Records"],
        "duration": 1800
      },
      {
        "stepId": "INV-STEP-005",
        "timestamp": "2025-01-09T16:45:00Z",
        "action": "customer_contact",
        "performedBy": "analyst_jdoe",
        "contactMethod": "phone",
        "outcome": "successful",
        "notes": "고객과 통화. ABC Trading(자금 출처) 및 XYZ Investments(인출 목적)와의 투자 계약을 보여주는 문서 제공. 문서가 합법적으로 보임.",
        "duration": 900,
        "documentsReceived": ["DOC-CASE-001", "DOC-CASE-002"]
      }
    ]
  },

  "evidence": [
    {
      "evidenceId": "DOC-CASE-001",
      "type": "supporting_document",
      "description": "ABC Trading Ltd와의 컨설팅 서비스 계약",
      "receivedFrom": "customer",
      "receivedAt": "2025-01-09T17:00:00Z",
      "fileUrl": "https://secure-storage.example.com/cases/case-2025-001234/doc-001.enc",
      "verified": true,
      "verificationNotes": "계약이 진짜로 보이며 거래 세부 사항과 일치"
    },
    {
      "evidenceId": "DOC-CASE-002",
      "type": "supporting_document",
      "description": "XYZ Investments와의 투자 계약",
      "receivedFrom": "customer",
      "receivedAt": "2025-01-09T17:00:00Z",
      "fileUrl": "https://secure-storage.example.com/cases/case-2025-001234/doc-002.enc",
      "verified": true,
      "verificationNotes": "투자 계약 합법적, XYZ Investments는 등록된 펀드"
    },
    {
      "evidenceId": "SEARCH-001",
      "type": "external_search",
      "description": "LexisNexis 배경 조사",
      "executedAt": "2025-01-09T16:00:00Z",
      "result": "부정적 발견 없음",
      "provider": "LexisNexis"
    }
  ],

  "findings": {
    "summary": "계정 개설 직후 외국 법인 간 대규모 자금의 빠른 이동으로 인해 초기 경보 트리거됨. 조사 결과 합법적인 사업 거래 - 고객은 서비스 지불을 받고 등록된 투자 펀드에 즉시 투자하는 컨설턴트. 제공된 지원 문서가 확인됨. 자금 세탁 또는 의심스러운 활동의 지표 없음.",
    "suspiciousActivityIndicators": [],
    "legitimateActivityIndicators": [
      "명시된 직업(컨설턴트)과 일치",
      "제공된 지원 문서가 확인됨",
      "거래상대방은 합법적인 등록 법인",
      "고객 협조적이고 반응적",
      "설명이 그럴듯하고 증거로 뒷받침됨"
    ]
  },

  "recommendation": {
    "disposition": "close_as_false_positive",
    "rationale": "지원 문서가 있는 합법적인 사업 거래로 설명된 활동. SAR 제출 필요 없음.",
    "additionalMonitoring": "standard",
    "proposedBy": "analyst_jdoe",
    "proposedAt": "2025-01-09T17:30:00Z"
  },

  "supervisorReview": {
    "required": true,
    "reviewedBy": "supervisor_rsmith",
    "reviewedAt": "2025-01-09T18:00:00Z",
    "approved": true,
    "notes": "분석가 평가에 동의. 조사 철저하고 문서 적절. 종료 승인."
  },

  "resolution": {
    "disposition": "closed_false_positive",
    "disposedAt": "2025-01-09T18:00:00Z",
    "disposedBy": "supervisor_rsmith",
    "sarFiled": false,
    "totalInvestigationTime": 10800,
    "finalNotes": "행동 패턴에 따라 경보가 적절하게 생성됨. 조사 결과 활동이 합법적임을 판단. 사례 종료."
  },

  "auditTrail": [
    {
      "timestamp": "2025-01-09T14:45:00Z",
      "action": "case_created",
      "performedBy": "system",
      "details": "경보 ALERT-2025-123456에서 자동 생성"
    },
    {
      "timestamp": "2025-01-09T15:00:00Z",
      "action": "case_assigned",
      "performedBy": "system",
      "details": "analyst_jdoe에 할당"
    },
    {
      "timestamp": "2025-01-09T18:00:00Z",
      "action": "case_closed",
      "performedBy": "supervisor_rsmith",
      "details": "조사 및 감독자 검토 후 거짓 양성으로 종료"
    }
  ]
}
```

---

## 필드 사양

### 공통 데이터 유형

| 유형 | 형식 | 예시 | 설명 |
|------|------|------|------|
| **String** | UTF-8 텍스트 | "John Smith" | 텍스트 필드 |
| **Integer** | 정수 | 42 | 개수, 점수 |
| **Float** | 소수 | 0.95 | 신뢰도 점수, 백분율 |
| **Boolean** | true/false | true | 예/아니오 값 |
| **Date** | YYYY-MM-DD | "2025-01-09" | 날짜만 |
| **DateTime** | ISO 8601 | "2025-01-09T10:30:00Z" | 타임스탬프 |
| **Currency** | ISO 4217 | "USD" | 통화 코드 |
| **Country** | ISO 3166-1 | "USA" | 국가 코드 |
| **Email** | RFC 5322 | "user@example.com" | 이메일 주소 |
| **Phone** | E.164 | "+1-555-0123" | 전화번호 |
| **UUID** | RFC 4122 | "123e4567-e89b..." | 고유 ID |

### 열거형 값

#### 고객 상태
- `active` - 계정 활성
- `inactive` - 일시적으로 비활성
- `suspended` - 조사를 위해 정지
- `closed` - 영구 폐쇄

#### 위험 범주
- `prohibited` - 허용되지 않음
- `high` - 강화된 실사 필요
- `medium` - 표준 실사
- `low` - 간소화된 실사 허용

#### 문서 유형
- `passport` - 여권
- `national_id` - 국민 ID
- `drivers_license` - 운전면허증
- `residence_permit` - 거주 허가
- `birth_certificate` - 출생 증명서
- `utility_bill` - 공과금 청구서
- `bank_statement` - 은행 명세서

#### 확인 상태
- `pass` - 확인 통과
- `fail` - 확인 실패
- `review` - 수동 검토 필요
- `pending` - 진행 중

---

## 확장 메커니즘

모든 스키마는 `extensions` 객체를 통해 사용자 정의 필드를 지원합니다:

```json
{
  "extensions": {
    "institutionSpecific": {
      "internalCustomerId": "INT-12345",
      "branchCode": "SF-001",
      "relationshipManager": "RM-456"
    },
    "vendorSpecific": {
      "vendorA": {
        "vendorCustomerId": "VENDOR-A-789"
      }
    },
    "customRiskFactors": {
      "industrySpecificRisk": 35
    }
  }
}
```

**지침:**
- 표준 스키마에 없는 필드에 `extensions` 사용
- 충돌을 피하기 위해 사용자 정의 필드에 네임스페이스 지정
- API 문서에서 사용자 정의 확장 문서화
- 핵심 기능에 대해 확장에 의존하지 말 것

---

## 주요 요점

1. 📋 **표준화된 JSON 스키마** 모든 핵심 엔티티에 대해
2. 🔄 **일관된 구조** 상호 운용성 가능
3. 🌍 **국제 지원** ISO 표준과 함께
4. 📝 **확장 가능한 설계** 확장 객체를 통해
5. ✅ **명확한 검증** 규칙 및 데이터 유형
6. 📊 **포괄적인 범위** KYC/AML 워크플로의

---

## 복습 질문

1. **개념 이해**: WIA KYC/AML 데이터 스키마에서 `extensions` 객체의 목적은 무엇입니까?

2. **스키마 설계**: 개인 고객 프로필과 기업 고객 프로필(KYB)의 주요 차이점은 무엇입니까?

3. **데이터 검증**: 신원 확인 결과에서 "confidenceScore"는 무엇을 나타내며 어떻게 사용됩니까?

4. **위험 평가**: 전체 위험 점수 계산에서 5가지 위험 차원은 무엇입니까?

5. **실무 적용**: 거래 모니터링 경보를 트리거할 수 있는 위험 지표를 3가지 예로 드십시오.

6. **상호 운용성**: 표준화된 데이터 형식이 시스템 간 상호 운용성을 어떻게 가능하게 합니까?

7. **스키마 진화**: "schemaVersion" 필드가 중요한 이유는 무엇이며 어떻게 사용됩니까?

---

**이전**: [← 3장 - 표준 개요](03-standard-overview.md) | **다음**: [5장 - API 인터페이스 →](05-api-interface.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
