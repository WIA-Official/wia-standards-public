# WIA-UNI-003: Family Reunion Data Standard v1.0

**Status:** Official Standard  
**Version:** 1.0.0  
**Date:** December 25, 2025  
**Authors:** WIA Standards Committee  
**Category:** UNI (Unification/Peace)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Data Format Specification](#3-data-format-specification)
4. [API Specification](#4-api-specification)
5. [Privacy & Security](#5-privacy--security)
6. [Ethical Guidelines](#6-ethical-guidelines)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the WIA-UNI-003 standard for family reunion data management, providing a comprehensive framework for helping separated families find and reconnect with loved ones across borders, conflicts, and decades.

### 1.2 Design Principles

- **Privacy First:** End-to-end encryption and consent-based data sharing
- **Humanitarian Focus:** Free for refugees and displaced persons
- **Interoperability:** Works with Red Cross, government, and NGO databases
- **Accuracy:** Multi-source verification and confidence scoring
- **Trauma-Informed:** Ethical guidelines for sensitive reunions

### 1.3 Normative References

- GDPR: General Data Protection Regulation
- ICRC Restoring Family Links: Red Cross family tracing protocols
- ISO 27001: Information Security Management
- UNESCO Guidelines on Separated Children

---

## 2. Scope

### 2.1 In Scope

- Person data schemas and family relationship graphs
- DNA profile formats and matching algorithms
- Photo recognition and age progression
- Multi-database search and synchronization
- Privacy-preserving computation protocols

### 2.2 Out of Scope

- DNA sequencing hardware/lab operations
- Legal adoption procedures
- Immigration/visa processing
- Financial reunification support

---

## 3. Data Format Specification

### 3.1 Person Data Schema

```json
{
  "personId": "UUID v4",
  "names": [
    {
      "fullName": "string",
      "language": "ISO 639-1 code",
      "script": "Latn|Kore|Arab|Cyrl",
      "isPrimary": "boolean"
    }
  ],
  "birthDate": {
    "date": "ISO 8601 date or partial (YYYY or YYYY-MM)",
    "isApproximate": "boolean",
    "accuracy": "exact|year|decade"
  },
  "birthPlace": {
    "city": "string",
    "region": "string",
    "country": "ISO 3166-1 alpha-2",
    "coordinates": "optional lat/long"
  },
  "gender": "MALE|FEMALE|OTHER|UNKNOWN",
  "currentStatus": "SEEKING|FOUND|DECEASED|UNKNOWN",
  "lastKnownLocation": "Location object",
  "separationDetails": {
    "separationDate": "ISO 8601 date",
    "separationEvent": "KOREAN_WAR|SYRIAN_CRISIS|NATURAL_DISASTER|etc",
    "circumstances": "string"
  },
  "biometricData": {
    "dnaProfileId": "UUID reference",
    "photoIds": ["UUID array"],
    "fingerprints": "optional"
  },
  "relationships": [
    {
      "relationshipId": "UUID",
      "relatedPersonId": "UUID",
      "relationshipType": "PARENT|CHILD|SIBLING|SPOUSE|etc",
      "confidence": "0-100",
      "verified": "boolean",
      "source": "DNA|PHOTO|DOCUMENT|TESTIMONY"
    }
  ],
  "privacySettings": {
    "allowDNAMatching": "boolean",
    "allowPhotoMatching": "boolean",
    "allowContactByMatches": "boolean",
    "dataRetentionYears": "integer|INDEFINITE"
  },
  "consentGiven": "boolean",
  "consentDate": "ISO 8601 timestamp",
  "createdAt": "ISO 8601 timestamp",
  "updatedAt": "ISO 8601 timestamp"
}
```

### 3.2 DNA Profile Format

```json
{
  "dnaProfileId": "UUID",
  "personId": "UUID",
  "sampleType": "SALIVA|BLOOD|BUCCAL_SWAB",
  "collectionDate": "ISO 8601 date",
  "autosomalDNA": {
    "format": "VCF|23ANDME|ANCESTRYDNA",
    "snpCount": "integer",
    "dataHash": "SHA-256 hash of encrypted data",
    "encryptedData": "base64 encoded encrypted genetic data"
  },
  "yDNA": {
    "haplogroup": "string",
    "strMarkers": "encrypted object"
  },
  "mtDNA": {
    "haplogroup": "string",
    "sequence": "encrypted string"
  },
  "ethnicity": {
    "primary": "string",
    "percentages": "object"
  },
  "qualityScore": "0-100",
  "processingLab": "string",
  "privacyTier": "FULL_MATCH|HASH_ONLY|NO_SHARING"
}
```

### 3.3 Photo Metadata

```json
{
  "photoId": "UUID",
  "personId": "UUID",
  "uploadDate": "ISO 8601 timestamp",
  "photoDate": "ISO 8601 date (approximate)",
  "photoAge": "integer (age of person in photo)",
  "facialEmbedding": "encrypted 512-d vector",
  "quality": "EXCELLENT|GOOD|FAIR|POOR",
  "restorationApplied": "boolean",
  "ageProgressionApplied": "boolean",
  "metadata": {
    "location": "optional",
    "occasion": "optional string",
    "description": "string"
  }
}
```

---

## 4. API Specification

### 4.1 Search API

**Endpoint:** `POST /api/v1/search`

**Request:**
```json
{
  "searchCriteria": {
    "name": "string",
    "birthDate": "ISO 8601 date (approximate)",
    "birthPlace": "string",
    "lastKnownLocation": "string",
    "separationEvent": "string",
    "familyMembers": ["names array"]
  },
  "searchOptions": {
    "fuzzyMatching": "boolean",
    "maxResults": "integer (default 50)",
    "confidenceThreshold": "0-100 (default 70)"
  }
}
```

**Response:**
```json
{
  "matches": [
    {
      "personId": "UUID",
      "matchScore": "0-100",
      "matchReasons": ["NAME_MATCH", "DATE_MATCH", "LOCATION_MATCH"],
      "person": "Person object",
      "relationshipProbability": "object"
    }
  ],
  "totalMatches": "integer",
  "searchId": "UUID (for tracking)"
}
```

### 4.2 DNA Matching API

**Endpoint:** `POST /api/v1/dna/match`

**Request:**
```json
{
  "dnaProfileId": "UUID",
  "matchThreshold": "centiMorgans (default 7cM)",
  "relationshipTypes": ["PARENT", "SIBLING", "COUSIN"]
}
```

**Response:**
```json
{
  "matches": [
    {
      "matchedPersonId": "UUID",
      "sharedDNA": "percentage",
      "sharedSegments": "centiMorgans",
      "relationship": "FULL_SIBLING|HALF_SIBLING|PARENT|etc",
      "confidence": "0-100",
      "chromosomeData": "encrypted segment details"
    }
  ]
}
```

---

## 5. Privacy & Security

### 5.1 Encryption Requirements

- AES-256-GCM for data at rest
- TLS 1.3 for data in transit
- End-to-end encryption for DNA and photos
- Homomorphic encryption for DNA matching

### 5.2 Consent Management

- Explicit opt-in for all data sharing
- Granular privacy controls (DNA, photos, contact)
- Right to withdraw consent at any time
- Right to delete all data (GDPR Article 17)

### 5.3 Access Controls

- Role-based access control (RBAC)
- Multi-factor authentication required
- Audit logging of all data access
- Geographic data residency compliance

---

## 6. Ethical Guidelines

### 6.1 Informed Consent

- Age-appropriate consent forms (18+)
- Guardian consent for minors
- Cultural adaptation of consent materials
- Translation into 50+ languages

### 6.2 Trauma-Informed Care

- Professional mediators for first contact
- Emotional support resources
- Preparation for potential negative outcomes
- Respect for those who decline reunion

### 6.3 Child Protection

- Best interests of the child paramount
- Verification of legal guardianship
- Protection from trafficking/exploitation
- Coordination with child welfare authorities

---

**Copyright © 2025 SmileStory Inc. / WIA**  
**弘益人間 (홍익인간) · Benefit All Humanity**
