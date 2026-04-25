# Chapter 4: Data Format Specifications

## Overview

This chapter details the JSON schemas used in the WIA KYC/AML Standard. Standardized data formats enable seamless interoperability between systems, vendors, and financial institutions.

---

## Schema Design Principles

### 1. Consistency
- Uniform naming conventions (camelCase for fields)
- Standard data types (ISO dates, currency codes)
- Predictable structure across entities

### 2. Extensibility
- Custom fields supported via `extensions` object
- Version field for schema evolution
- Optional fields for flexibility

### 3. Validation
- Required vs. optional fields clearly defined
- Enum values for constrained choices
- Format specifications (regex, length, range)

### 4. International Support
- Unicode support for names and addresses
- Multiple languages via `language` field
- Country codes (ISO 3166-1 alpha-3)
- Currency codes (ISO 4217)

---

## Core Data Schemas

### 1. Individual Customer Profile

**Schema:** `CustomerProfile-Individual-v1.0`

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
        "details": "Low-risk jurisdiction, no high-risk connections"
      },
      {
        "dimension": "occupation",
        "score": 15,
        "details": "Standard corporate employment"
      },
      {
        "dimension": "transactionProfile",
        "score": 20,
        "details": "Expected activity consistent with income"
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

### 2. Corporate Customer Profile (KYB)

**Schema:** `CustomerProfile-Business-v1.0`

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
    "businessDescription": "Provides enterprise software solutions for financial services companies, specializing in compliance and risk management platforms.",
    "productServices": [
      "RegTech Software",
      "AML Compliance Solutions",
      "Risk Management Platforms"
    ],
    "targetMarkets": ["North America", "Europe", "Asia-Pacific"],
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
        "details": "Operations in multiple jurisdictions including higher-risk regions"
      },
      {
        "dimension": "industry",
        "score": 40,
        "details": "Software/Tech sector - Medium risk"
      },
      {
        "dimension": "complexity",
        "score": 35,
        "details": "International operations, multiple subsidiaries"
      },
      {
        "dimension": "ownership",
        "score": 25,
        "details": "Clear ownership structure, no PEPs"
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

### 3. Identity Verification Result

**Schema:** `IdentityVerificationResult-v1.0`

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
    "
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
      "Document will expire in 5+ years - good validity period"
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

### 4. Risk Assessment

**Schema:** `RiskAssessment-v1.0`

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
          "rationale": "Low-risk jurisdiction"
        },
        {
          "factor": "citizenship",
          "value": "USA",
          "score": 10,
          "rationale": "Low-risk jurisdiction"
        },
        {
          "factor": "transaction_countries",
          "value": ["USA", "CAN", "GBR"],
          "score": 20,
          "rationale": "All low-risk jurisdictions"
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
          "rationale": "International wire capability adds risk"
        },
        {
          "factor": "expected_volume",
          "value": "medium",
          "score": 25,
          "rationale": "Moderate transaction volume"
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
          "rationale": "Individual retail customer - lower complexity"
        },
        {
          "factor": "occupation",
          "value": "software_engineer",
          "score": 20,
          "rationale": "Professional occupation - standard risk"
        },
        {
          "factor": "employer_type",
          "value": "established_corporation",
          "score": 10,
          "rationale": "Reputable employer"
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
          "rationale": "Clear, legitimate source"
        },
        {
          "factor": "transaction_purpose",
          "value": "personal_banking",
          "score": 15,
          "rationale": "Standard personal use"
        },
        {
          "factor": "expected_vs_actual",
          "value": "new_customer",
          "score": 20,
          "rationale": "No transaction history yet"
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
          "rationale": "Not a PEP"
        },
        {
          "factor": "sanctions_match",
          "value": false,
          "score": 0,
          "rationale": "No sanctions matches"
        },
        {
          "factor": "adverse_media",
          "value": false,
          "score": 0,
          "rationale": "No adverse media found"
        },
        {
          "factor": "known_associates",
          "value": "none",
          "score": 10,
          "rationale": "No high-risk associations"
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
        "rationale": "Low fraud indicators from identity verification"
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

### 5. Transaction Monitoring Alert

**Schema:** `TransactionMonitoringAlert-v1.0`

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

  "description": "Large deposit followed by rapid withdrawal within 24 hours",

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
      "description": "Payment for consulting services"
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
      "description": "Investment"
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
      "value": "First transaction with both parties",
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
    "expectedActivity": "Personal banking, occasional international transfers"
  },

  "mlScore": {
    "suspicionScore": 0.82,
    "model": "anomaly_detection_v3.2",
    "confidence": 0.87,
    "peerComparison": {
      "percentile": 98,
      "interpretation": "Highly unusual compared to similar customers"
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

### 6. Screening Result

**Schema:** `ScreeningResult-v1.0`

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
          "Name similarity: 92%",
          "Date of birth close but not exact (5 day difference)",
          "Same nationality"
        ],
        "differentiators": [
          "Different middle name",
          "Date of birth not exact match",
          "No other identifying information matches"
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
  "reviewReason": "Potential PEP match requires verification",
  "reviewDueDate": "2025-01-10T10:12:00Z",

  "continuousMonitoring": {
    "enabled": true,
    "frequency": "daily",
    "nextScheduled": "2025-01-10T10:12:00Z"
  }
}
```

---

### 7. Case Record

**Schema:** `CaseRecord-v1.0`

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
    "description": "Rapid funds movement - high value"
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
        "notes": "Reviewed triggering transactions and risk indicators",
        "duration": 300
      },
      {
        "stepId": "INV-STEP-002",
        "timestamp": "2025-01-09T15:15:00Z",
        "action": "review_customer_profile",
        "performedBy": "analyst_jdoe",
        "notes": "Customer is new (account opened 1 day ago), low risk profile, employed professional",
        "duration": 600
      },
      {
        "stepId": "INV-STEP-003",
        "timestamp": "2025-01-09T15:30:00Z",
        "action": "review_transaction_history",
        "performedBy": "analyst_jdoe",
        "notes": "Only 2 transactions total - the ones that triggered alert. No prior activity.",
        "duration": 900
      },
      {
        "stepId": "INV-STEP-004",
        "timestamp": "2025-01-09T16:00:00Z",
        "action": "external_database_search",
        "performedBy": "analyst_jdoe",
        "notes": "Searched LexisNexis, no negative findings. Customer appears legitimate.",
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
        "notes": "Spoke with customer. Provided documentation showing consulting contract with ABC Trading (source of funds) and investment agreement with XYZ Investments (purpose of withdrawal). Documentation appears legitimate.",
        "duration": 900,
        "documentsReceived": ["DOC-CASE-001", "DOC-CASE-002"]
      }
    ]
  },

  "evidence": [
    {
      "evidenceId": "DOC-CASE-001",
      "type": "supporting_document",
      "description": "Consulting services contract with ABC Trading Ltd",
      "receivedFrom": "customer",
      "receivedAt": "2025-01-09T17:00:00Z",
      "fileUrl": "https://secure-storage.example.com/cases/case-2025-001234/doc-001.enc",
      "verified": true,
      "verificationNotes": "Contract appears authentic, matches transaction details"
    },
    {
      "evidenceId": "DOC-CASE-002",
      "type": "supporting_document",
      "description": "Investment agreement with XYZ Investments",
      "receivedFrom": "customer",
      "receivedAt": "2025-01-09T17:00:00Z",
      "fileUrl": "https://secure-storage.example.com/cases/case-2025-001234/doc-002.enc",
      "verified": true,
      "verificationNotes": "Investment agreement legitimate, XYZ Investments is registered fund"
    },
    {
      "evidenceId": "SEARCH-001",
      "type": "external_search",
      "description": "LexisNexis background check",
      "executedAt": "2025-01-09T16:00:00Z",
      "result": "No negative findings",
      "provider": "LexisNexis"
    }
  ],

  "findings": {
    "summary": "Initial alert triggered due to rapid movement of large funds between foreign entities shortly after account opening. Investigation revealed legitimate business transaction - customer is consultant receiving payment for services and immediately investing in registered investment fund. Supporting documentation provided and verified. No indicators of money laundering or suspicious activity.",
    "suspiciousActivityIndicators": [],
    "legitimateActivityIndicators": [
      "Consistent with stated occupation (consultant)",
      "Supporting documentation provided and verified",
      "Counterparties are legitimate registered entities",
      "Customer cooperative and responsive",
      "Explanation plausible and supported by evidence"
    ]
  },

  "recommendation": {
    "disposition": "close_as_false_positive",
    "rationale": "Activity explained by legitimate business transaction with supporting documentation. No SAR filing warranted.",
    "additionalMonitoring": "standard",
    "proposedBy": "analyst_jdoe",
    "proposedAt": "2025-01-09T17:30:00Z"
  },

  "supervisorReview": {
    "required": true,
    "reviewedBy": "supervisor_rsmith",
    "reviewedAt": "2025-01-09T18:00:00Z",
    "approved": true,
    "notes": "Concur with analyst assessment. Investigation thorough, documentation adequate. Approve closure."
  },

  "resolution": {
    "disposition": "closed_false_positive",
    "disposedAt": "2025-01-09T18:00:00Z",
    "disposedBy": "supervisor_rsmith",
    "sarFiled": false,
    "totalInvestigationTime": 10800,
    "finalNotes": "Alert generated appropriately based on behavioral pattern. Investigation determined activity legitimate. Case closed."
  },

  "auditTrail": [
    {
      "timestamp": "2025-01-09T14:45:00Z",
      "action": "case_created",
      "performedBy": "system",
      "details": "Auto-created from alert ALERT-2025-123456"
    },
    {
      "timestamp": "2025-01-09T15:00:00Z",
      "action": "case_assigned",
      "performedBy": "system",
      "details": "Assigned to analyst_jdoe"
    },
    {
      "timestamp": "2025-01-09T18:00:00Z",
      "action": "case_closed",
      "performedBy": "supervisor_rsmith",
      "details": "Closed as false positive after investigation and supervisor review"
    }
  ]
}
```

---

## Field Specifications

### Common Data Types

| Type | Format | Example | Description |
|------|--------|---------|-------------|
| **String** | UTF-8 text | "John Smith" | Text fields |
| **Integer** | Whole number | 42 | Counts, scores |
| **Float** | Decimal | 0.95 | Confidence scores, percentages |
| **Boolean** | true/false | true | Yes/no values |
| **Date** | YYYY-MM-DD | "2025-01-09" | Dates only |
| **DateTime** | ISO 8601 | "2025-01-09T10:30:00Z" | Timestamps |
| **Currency** | ISO 4217 | "USD" | Currency codes |
| **Country** | ISO 3166-1 | "USA" | Country codes |
| **Email** | RFC 5322 | "user@example.com" | Email addresses |
| **Phone** | E.164 | "+1-555-0123" | Phone numbers |
| **UUID** | RFC 4122 | "123e4567-e89b..." | Unique IDs |

### Enumerated Values

#### Customer Status
- `active` - Account active
- `inactive` - Temporarily inactive
- `suspended` - Suspended for investigation
- `closed` - Permanently closed

#### Risk Categories
- `prohibited` - Not permitted
- `high` - Enhanced due diligence required
- `medium` - Standard due diligence
- `low` - Simplified due diligence acceptable

#### Document Types
- `passport`
- `national_id`
- `drivers_license`
- `residence_permit`
- `birth_certificate`
- `utility_bill`
- `bank_statement`

#### Verification Status
- `pass` - Check passed
- `fail` - Check failed
- `review` - Manual review required
- `pending` - In progress

---

## Extension Mechanism

All schemas support custom fields via the `extensions` object:

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

**Guidelines:**
- Use `extensions` for fields not in standard schema
- Namespace custom fields to avoid conflicts
- Document custom extensions in API documentation
- Don't rely on extensions for core functionality

---

## Key Takeaways

1. 📋 **Standardized JSON schemas** for all core entities
2. 🔄 **Consistent structure** enables interoperability
3. 🌍 **International support** with ISO standards
4. 📝 **Extensible design** via extensions object
5. ✅ **Clear validation** rules and data types
6. 📊 **Comprehensive coverage** of KYC/AML workflows

---

**Previous**: [← Chapter 3 - Standard Overview](03-standard-overview.md) | **Next**: [Chapter 5 - API Interface →](05-api-interface.md)

---

© 2025 WIA Standards Committee
弘익人間 (홍익인간) - Benefit All Humanity
