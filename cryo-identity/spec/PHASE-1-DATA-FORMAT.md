# CRYO-IDENTITY Phase 1: Data Format Specification

## 1. Core Identity Document

### 1.1 Master Identity Schema

```typescript
interface CryoIdentityDocument {
  // Document Metadata
  "@context": string[];
  "type": ["VerifiableCredential", "CryoIdentityCredential"];
  "id": string;  // urn:uuid:cryo:identity:{uuid}
  "issuer": string;
  "issuanceDate": string;  // ISO 8601
  "expirationDate"?: string;

  // Core Subject
  "credentialSubject": {
    "subjectId": string;  // Links to CRYO-PRESERVATION subject
    "identityId": string;  // CRYOID-{year}-{facility}-{sequence}

    // Legal Identity
    "legalIdentity": LegalIdentityRecord;

    // Personal Identity
    "personalIdentity": PersonalIdentityRecord;

    // Digital Identity
    "digitalIdentity": DigitalIdentityRecord;

    // Biometric Identity
    "biometricIdentity": BiometricIdentityRecord;
  };

  // Cryptographic Proof
  "proof": ProofObject;
}
```

### 1.2 Legal Identity Record

```typescript
interface LegalIdentityRecord {
  // Primary Name
  "legalName": {
    "givenName": string;
    "middleName"?: string;
    "familyName": string;
    "suffix"?: string;
    "prefix"?: string;
  };

  // Alternative Names
  "alternativeNames"?: {
    "type": "MAIDEN" | "ALIAS" | "NICKNAME" | "PROFESSIONAL" | "PREVIOUS";
    "name": string;
    "validFrom"?: string;
    "validTo"?: string;
  }[];

  // Birth Record
  "birthRecord": {
    "dateOfBirth": string;  // YYYY-MM-DD
    "placeOfBirth": {
      "country": string;  // ISO 3166-1 alpha-3
      "region"?: string;
      "city"?: string;
    };
    "birthCertificateRef"?: string;
  };

  // Citizenship
  "citizenship": {
    "country": string;  // ISO 3166-1 alpha-3
    "type": "BIRTH" | "NATURALIZED" | "DUAL" | "STATELESS";
    "documentNumber"?: string;
    "issueDate"?: string;
    "expiryDate"?: string;
  }[];

  // Government Identifications
  "governmentIds": GovernmentIdRecord[];

  // Family Relationships
  "familyRelationships": FamilyRelationshipRecord[];

  // Marital Status
  "maritalStatus": {
    "status": "SINGLE" | "MARRIED" | "DIVORCED" | "WIDOWED" | "SEPARATED";
    "spouse"?: PersonReference;
    "marriageDate"?: string;
    "marriagePlace"?: string;
  };
}

interface GovernmentIdRecord {
  "idType": "PASSPORT" | "NATIONAL_ID" | "DRIVERS_LICENSE" | "SSN" | "TAX_ID" | "OTHER";
  "country": string;
  "number": string;  // Encrypted
  "issueDate": string;
  "expiryDate"?: string;
  "issuer": string;
  "documentHash": string;  // Hash of original document
}

interface FamilyRelationshipRecord {
  "relationship": "PARENT" | "CHILD" | "SIBLING" | "SPOUSE" | "GRANDPARENT" | "GRANDCHILD";
  "person": PersonReference;
  "biological": boolean;
  "legal": boolean;
  "verificationStatus": "VERIFIED" | "UNVERIFIED" | "CLAIMED";
}

interface PersonReference {
  "name": string;
  "cryoId"?: string;  // If also preserved
  "externalId"?: string;
  "dateOfBirth"?: string;
  "status": "LIVING" | "DECEASED" | "PRESERVED" | "UNKNOWN";
}
```

### 1.3 Personal Identity Record

```typescript
interface PersonalIdentityRecord {
  // Life Timeline
  "lifeTimeline": LifeEventRecord[];

  // Personality Profile
  "personalityProfile"?: {
    "assessmentType": "MBTI" | "BIG_FIVE" | "ENNEAGRAM" | "CUSTOM";
    "assessmentDate": string;
    "results": Record<string, any>;
    "assessor"?: string;
  }[];

  // Values and Beliefs
  "valuesAndBeliefs": {
    "coreValues": string[];
    "religiousAffiliation"?: string;
    "politicalViews"?: string;
    "ethicalPrinciples"?: string[];
  };

  // Preferences
  "preferences": {
    "languages": {
      "language": string;  // ISO 639-1
      "proficiency": "NATIVE" | "FLUENT" | "INTERMEDIATE" | "BASIC";
    }[];
    "cultural": string[];
    "lifestyle": string[];
  };

  // Skills and Expertise
  "skillsAndExpertise": SkillRecord[];

  // Education
  "education": EducationRecord[];

  // Career
  "career": CareerRecord[];

  // Memory Archives
  "memoryArchives"?: MemoryArchiveReference[];
}

interface LifeEventRecord {
  "eventId": string;
  "eventType": "BIRTH" | "EDUCATION" | "CAREER" | "RELATIONSHIP" |
               "ACHIEVEMENT" | "HEALTH" | "TRAVEL" | "OTHER";
  "date": string;
  "description": string;
  "location"?: string;
  "significance": "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";
  "attachments"?: AttachmentReference[];
}

interface SkillRecord {
  "skillName": string;
  "category": "PROFESSIONAL" | "TECHNICAL" | "CREATIVE" | "PHYSICAL" | "SOCIAL";
  "proficiencyLevel": 1 | 2 | 3 | 4 | 5;  // 1=Novice, 5=Expert
  "yearsExperience": number;
  "certifications"?: string[];
  "lastPracticed": string;
}

interface EducationRecord {
  "institution": string;
  "degree": string;
  "field": string;
  "startDate": string;
  "endDate"?: string;
  "status": "COMPLETED" | "IN_PROGRESS" | "INCOMPLETE";
  "gpa"?: number;
  "achievements"?: string[];
}

interface CareerRecord {
  "employer": string;
  "position": string;
  "industry": string;
  "startDate": string;
  "endDate"?: string;
  "responsibilities": string[];
  "achievements"?: string[];
}

interface MemoryArchiveReference {
  "archiveId": string;
  "archiveType": "JOURNAL" | "PHOTO" | "VIDEO" | "AUDIO" | "DOCUMENT" | "DIGITAL";
  "description": string;
  "dateRange": {
    "from": string;
    "to": string;
  };
  "storageLocation": string;
  "accessCredentials"?: string;  // Encrypted
  "format": string;
  "sizeBytes": number;
  "integrityHash": string;
}
```

### 1.4 Digital Identity Record

```typescript
interface DigitalIdentityRecord {
  // Decentralized Identifiers
  "dids": {
    "primary": string;  // did:wia:cryo:{identifier}
    "secondary"?: string[];
    "recovery": string[];
  };

  // Credential Wallet
  "credentialWallet": {
    "walletId": string;
    "walletType": "HARDWARE" | "SOFTWARE" | "CUSTODIAL";
    "provider": string;
    "publicKey": string;
    "recoveryMechanism": "SEED_PHRASE" | "SOCIAL_RECOVERY" | "GUARDIAN" | "MULTI_SIG";
    "guardians"?: GuardianRecord[];
  };

  // Online Accounts
  "onlineAccounts": OnlineAccountRecord[];

  // Digital Asset Registry
  "digitalAssets": DigitalAssetRecord[];

  // Communication Channels
  "communicationChannels": {
    "email": string[];
    "phone": string[];
    "messaging": {
      "platform": string;
      "identifier": string;
    }[];
    "social": {
      "platform": string;
      "username": string;
      "url"?: string;
    }[];
  };

  // Cryptographic Keys
  "cryptographicKeys": {
    "keyId": string;
    "keyType": "RSA" | "ECDSA" | "ED25519" | "PQC";
    "purpose": "SIGNING" | "ENCRYPTION" | "AUTHENTICATION";
    "publicKey": string;
    "privateKeyLocation": "VAULT" | "HARDWARE" | "ESCROW";
    "rotationPolicy": string;
  }[];
}

interface GuardianRecord {
  "guardianId": string;
  "guardianType": "PERSON" | "ORGANIZATION" | "SMART_CONTRACT";
  "name": string;
  "contactInfo": string;  // Encrypted
  "threshold": number;  // Required for recovery
  "relationshipToSubject": string;
}

interface OnlineAccountRecord {
  "accountId": string;
  "platform": string;
  "username": string;
  "email"?: string;
  "url"?: string;
  "accountType": "SOCIAL" | "FINANCIAL" | "PROFESSIONAL" | "ENTERTAINMENT" | "UTILITY";
  "status": "ACTIVE" | "SUSPENDED" | "ARCHIVED" | "DELETED";
  "createdDate"?: string;
  "lastAccess"?: string;
  "recoveryOptions": string[];
  "accessCredentialVault": string;  // Reference to credential storage
}

interface DigitalAssetRecord {
  "assetId": string;
  "assetType": "CRYPTOCURRENCY" | "NFT" | "DOMAIN" | "VIRTUAL_PROPERTY" |
               "DIGITAL_CONTENT" | "SOFTWARE_LICENSE" | "DATA";
  "name": string;
  "description"?: string;
  "platform"?: string;
  "walletAddress"?: string;
  "contractAddress"?: string;
  "tokenId"?: string;
  "quantity": number | string;
  "estimatedValue"?: {
    "amount": number;
    "currency": string;
    "asOfDate": string;
  };
  "accessMethod": string;
  "legalOwnership": "SOLE" | "JOINT" | "TRUST" | "ESTATE";
}
```

### 1.5 Biometric Identity Record

```typescript
interface BiometricIdentityRecord {
  // DNA Profile
  "dnaProfile": {
    "profileId": string;
    "sequenceType": "FULL_GENOME" | "EXOME" | "SNP_PANEL" | "STR";
    "sequenceDate": string;
    "laboratory": string;
    "storageFormat": "FASTQ" | "BAM" | "VCF" | "FASTA";
    "storageLocation": string;
    "integrityHash": string;
    "markers": {
      "marker": string;
      "value": string;
    }[];
  };

  // Fingerprints
  "fingerprints"?: {
    "digit": "RIGHT_THUMB" | "RIGHT_INDEX" | "RIGHT_MIDDLE" | "RIGHT_RING" | "RIGHT_PINKY" |
             "LEFT_THUMB" | "LEFT_INDEX" | "LEFT_MIDDLE" | "LEFT_RING" | "LEFT_PINKY";
    "template": string;  // Encrypted biometric template
    "format": "ISO_19794_2" | "ANSI_378" | "PROPRIETARY";
    "quality": number;  // 0-100
    "captureDate": string;
  }[];

  // Facial Geometry
  "facialGeometry"?: {
    "templateId": string;
    "template": string;  // Encrypted
    "format": "ISO_19794_5" | "FERET" | "PROPRIETARY";
    "captureDate": string;
    "lightingCondition": string;
    "pose": "FRONTAL" | "PROFILE_LEFT" | "PROFILE_RIGHT";
    "referencePhotos": string[];  // Encrypted references
  };

  // Iris Patterns
  "irisPatterns"?: {
    "eye": "LEFT" | "RIGHT";
    "template": string;  // Encrypted
    "format": "ISO_19794_6" | "PROPRIETARY";
    "captureDate": string;
    "quality": number;
  }[];

  // Voice Print
  "voicePrint"?: {
    "templateId": string;
    "template": string;  // Encrypted
    "captureDate": string;
    "language": string;
    "duration": number;  // seconds
    "sampleRate": number;
    "format": string;
  };

  // Physical Characteristics
  "physicalCharacteristics": {
    "height": number;  // cm
    "weight": number;  // kg at preservation
    "bloodType": string;
    "eyeColor": string;
    "hairColor": string;
    "skinTone": string;
    "distinguishingMarks": {
      "type": "SCAR" | "BIRTHMARK" | "TATTOO" | "PIERCING" | "OTHER";
      "location": string;
      "description": string;
      "imageRef"?: string;
    }[];
  };
}
```

## 2. Identity Vault Structure

### 2.1 Vault Configuration

```typescript
interface IdentityVault {
  "vaultId": string;
  "subjectId": string;
  "identityId": string;

  "configuration": {
    "encryptionScheme": "AES-256-GCM" | "CHACHA20-POLY1305";
    "keyManagement": "THRESHOLD" | "MULTI_SIG" | "GUARDIAN";
    "redundancy": number;  // Number of copies
    "storageLocations": StorageLocationRecord[];
    "accessPolicy": AccessPolicyRecord;
  };

  "contents": {
    "coreIdentity": EncryptedPayload;
    "legalDocuments": EncryptedPayload[];
    "personalRecords": EncryptedPayload[];
    "digitalCredentials": EncryptedPayload[];
    "biometricData": EncryptedPayload[];
    "memoryArchives": EncryptedPayload[];
  };

  "accessLog": AccessLogEntry[];
  "integrityProofs": IntegrityProof[];
}

interface StorageLocationRecord {
  "locationId": string;
  "locationType": "PRIMARY" | "BACKUP" | "COLD_STORAGE";
  "provider": string;
  "geographic": {
    "region": string;
    "jurisdiction": string;
  };
  "status": "ACTIVE" | "SYNCING" | "OFFLINE" | "RETIRED";
  "lastSync": string;
  "integrityVerified": string;
}

interface AccessPolicyRecord {
  "defaultAccess": "NONE" | "READ_ONLY" | "FULL";

  "authorizedAccessors": {
    "accessorId": string;
    "accessorType": "GUARDIAN" | "FACILITY" | "LEGAL_REP" | "FAMILY" | "SYSTEM";
    "accessLevel": "METADATA" | "PARTIAL" | "FULL";
    "permissions": string[];
    "validFrom": string;
    "validUntil"?: string;
    "conditions"?: string[];
  }[];

  "emergencyAccess": {
    "enabled": boolean;
    "threshold": number;  // Number of guardians required
    "waitingPeriod": number;  // Hours
    "notificationRequired": boolean;
  };

  "revivalAccess": {
    "automaticGrant": boolean;
    "verificationRequired": string[];
    "integrationProtocol": string;
  };
}

interface EncryptedPayload {
  "payloadId": string;
  "contentType": string;
  "encryptedData": string;
  "encryptionKeyId": string;
  "nonce": string;
  "createdAt": string;
  "lastModified": string;
  "integrityHash": string;
}
```

## 3. Identity Continuity Records

### 3.1 Status Tracking

```typescript
interface IdentityContinuityRecord {
  "recordId": string;
  "identityId": string;
  "subjectId": string;

  "currentStatus": {
    "identityState": "ACTIVE" | "PRESERVED" | "SUSPENDED" | "REVIVED" | "TERMINATED";
    "preservationDate": string;
    "lastVerification": string;
    "nextScheduledVerification": string;
  };

  "legalStatus": {
    "jurisdictions": {
      "jurisdiction": string;
      "status": "RECOGNIZED" | "SUSPENDED" | "UNRECOGNIZED";
      "legalDocuments": string[];
      "notes": string;
    }[];
    "powerOfAttorney": {
      "grantee": PersonReference;
      "scope": string[];
      "validUntil": string;
      "documentRef": string;
    }[];
  };

  "assetStatus": {
    "financialAssets": "FROZEN" | "MANAGED" | "TRUST" | "ESTATE";
    "propertyAssets": "FROZEN" | "MANAGED" | "TRUST" | "ESTATE";
    "digitalAssets": "FROZEN" | "MANAGED" | "TRUST" | "ESTATE";
    "managingEntity"?: string;
    "trustDocument"?: string;
  };

  "relationshipStatus": {
    "activeRelationships": number;
    "designatedContacts": PersonReference[];
    "communicationProxy"?: string;
  };
}
```

### 3.2 Update History

```typescript
interface IdentityUpdateRecord {
  "updateId": string;
  "identityId": string;
  "timestamp": string;

  "updateType": "SCHEDULED_VERIFICATION" | "LEGAL_UPDATE" | "ASSET_UPDATE" |
                "RELATIONSHIP_UPDATE" | "SYSTEM_MIGRATION" | "EMERGENCY";

  "changes": {
    "field": string;
    "previousValue": any;
    "newValue": any;
    "reason": string;
  }[];

  "authorizedBy": {
    "authorizer": string;
    "authorizerType": "GUARDIAN" | "LEGAL_REP" | "SYSTEM" | "COURT";
    "authorization": string;
  };

  "verification": {
    "method": string;
    "verifiedBy": string;
    "verificationProof": string;
  };
}
```

## 4. Revival Identity Package

### 4.1 Revival Preparation

```typescript
interface RevivalIdentityPackage {
  "packageId": string;
  "identityId": string;
  "subjectId": string;

  "preparationDate": string;
  "preparedBy": string;

  "coreIdentityBundle": {
    "legalName": string;
    "dateOfBirth": string;
    "primaryCitizenship": string;
    "governmentIds": GovernmentIdRecord[];
    "biometricVerification": string[];
  };

  "restorationGuidance": {
    "priorityDocuments": string[];
    "recommendedSequence": {
      "step": number;
      "action": string;
      "dependencies": string[];
      "estimatedDuration": string;
    }[];
    "potentialChallenges": {
      "challenge": string;
      "mitigation": string;
    }[];
  };

  "integrationSupport": {
    "languagePreferences": string[];
    "culturalContext": string;
    "familyContacts": PersonReference[];
    "professionalNetwork": PersonReference[];
    "financialAdvisors": string[];
    "legalRepresentation": string[];
  };

  "personalContext": {
    "lifeSnapshot": string;  // Summary of life at preservation
    "personalMessages": {
      "from": string;
      "message": string;  // Encrypted
      "deliveryCondition": string;
    }[];
    "orientationMaterials": string[];
  };
}
```

---
*CRYO-IDENTITY Phase 1 Specification v1.0.0*
