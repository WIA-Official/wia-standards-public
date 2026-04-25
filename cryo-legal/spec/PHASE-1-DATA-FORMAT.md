# CRYO-LEGAL Phase 1: Data Format Specification

## Overview

This document defines the data structures for legal status management of cryopreserved individuals.

## Core Data Structures

### Legal Status Record

```typescript
interface LegalStatusRecord {
  recordId: string;
  subjectId: string;
  identityId: string;
  currentStatus: LegalStatus;
  statusHistory: StatusTransition[];
  deathCertificate: DeathCertificate;
  preservationDeclaration: PreservationDeclaration;
  restorationCertificate?: RestorationCertificate;
  jurisdictions: JurisdictionRecord[];
  documents: LegalDocument[];
  guardianship: GuardianshipRecord;
  lastUpdated: string;
  createdAt: string;
}

type LegalStatus =
  | 'LIVING'
  | 'LEGALLY_DECEASED'
  | 'PRESERVED_STATUS'
  | 'REVIVAL_PENDING'
  | 'LEGALLY_RESTORED'
  | 'PERMANENT_DEATH';

interface StatusTransition {
  transitionId: string;
  fromStatus: LegalStatus;
  toStatus: LegalStatus;
  transitionDate: string;
  authority: LegalAuthority;
  documentRef: string;
  reason: string;
  witnesses: Witness[];
}
```

### Death Certificate

```typescript
interface DeathCertificate {
  certificateId: string;
  certificateNumber: string;
  jurisdiction: Jurisdiction;

  deceasedInfo: {
    fullName: string;
    dateOfBirth: string;
    placeOfBirth: string;
    nationality: string[];
    identificationNumbers: IdentificationNumber[];
  };

  deathInfo: {
    dateOfDeath: string;
    timeOfDeath: string;
    placeOfDeath: string;
    causeOfDeath: string;
    mannerOfDeath: MannerOfDeath;
    certifyingPhysician: string;
    physicianLicense: string;
  };

  preservationAnnotation?: {
    annotated: boolean;
    annotationDate: string;
    annotationType: 'CRYOPRESERVATION_PENDING' | 'CRYOPRESERVED';
    facilityId: string;
    facilityName: string;
  };

  issuingAuthority: {
    name: string;
    title: string;
    jurisdiction: string;
    registrationNumber: string;
  };

  issueDate: string;
  registrationDate: string;
  certified: boolean;
  certificationDate: string;
}

type MannerOfDeath =
  | 'NATURAL'
  | 'ACCIDENT'
  | 'SUICIDE'
  | 'HOMICIDE'
  | 'UNDETERMINED'
  | 'PENDING_INVESTIGATION';

interface IdentificationNumber {
  type: 'SSN' | 'NATIONAL_ID' | 'PASSPORT' | 'DRIVERS_LICENSE' | 'OTHER';
  number: string;
  issuingCountry: string;
  verified: boolean;
}
```

### Preservation Declaration

```typescript
interface PreservationDeclaration {
  declarationId: string;
  subjectId: string;

  declarationType: DeclarationType;

  legalBasis: {
    statute: string;
    jurisdiction: Jurisdiction;
    effectiveDate: string;
    expirationDate?: string;
  };

  preservationDetails: {
    facilityId: string;
    facilityName: string;
    facilityJurisdiction: string;
    preservationDate: string;
    preservationMethod: string;
    expectedDuration: string;
  };

  legalEffects: {
    deathCertificateStatus: 'SUSPENDED' | 'ANNOTATED' | 'UNCHANGED';
    propertyRights: PropertyRightsStatus;
    contractualObligations: ContractStatus;
    familyRelations: FamilyRelationsStatus;
    votingRights: 'SUSPENDED' | 'TERMINATED';
    taxObligations: TaxStatus;
  };

  conditions: {
    revivalConditions: string[];
    terminationConditions: string[];
    reviewPeriod: string;
  };

  issuingAuthority: LegalAuthority;
  issueDate: string;
  effectiveDate: string;
  signatures: Signature[];
}

type DeclarationType =
  | 'JUDICIAL_ORDER'
  | 'ADMINISTRATIVE_DECLARATION'
  | 'NOTARIZED_DECLARATION'
  | 'TREATY_BASED';

interface PropertyRightsStatus {
  status: 'SUSPENDED' | 'TRUST_MANAGED' | 'ESTATE_DISTRIBUTED';
  trustId?: string;
  trusteeId?: string;
  reviewDate: string;
}

interface ContractStatus {
  status: 'SUSPENDED' | 'TERMINATED' | 'CONTINUED';
  exceptions: string[];
}

interface FamilyRelationsStatus {
  status: 'PRESERVED' | 'MODIFIED';
  marriageStatus: 'SUSPENDED' | 'DISSOLVED' | 'PRESERVED';
  parentalRights: 'SUSPENDED' | 'TRANSFERRED' | 'PRESERVED';
  guardianshipAssigned: boolean;
}

interface TaxStatus {
  status: 'SUSPENDED' | 'FINAL_RETURN_FILED' | 'TRUST_FILING';
  lastFilingYear: number;
  trustTaxId?: string;
}
```

### Restoration Certificate

```typescript
interface RestorationCertificate {
  certificateId: string;
  subjectId: string;
  identityId: string;

  revivalInfo: {
    revivalProcedureId: string;
    revivalDate: string;
    revivalFacility: string;
    revivalJurisdiction: string;
    outcomeAssessment: string;
  };

  identityVerification: {
    verificationMethod: IdentityVerificationMethod[];
    verificationDate: string;
    verifiedBy: string;
    confidenceScore: number;
    biometricMatch: boolean;
  };

  restoredRights: {
    legalPersonhood: boolean;
    propertyRights: boolean;
    contractualCapacity: boolean;
    familyRelations: RestoredFamilyRelations;
    votingRights: boolean;
    professionalLicenses: LicenseRestoration[];
  };

  newIdentifiers: {
    newIdNumber?: string;
    idContinuity: boolean;
    socialSecurityStatus: 'RESTORED' | 'NEW_NUMBER' | 'PENDING';
    passportStatus: 'RESTORED' | 'NEW_ISSUED' | 'PENDING';
  };

  conditions: {
    probationaryPeriod?: string;
    medicalReview: boolean;
    mentalCapacityReview: boolean;
    financialReview: boolean;
  };

  issuingAuthority: LegalAuthority;
  issueDate: string;
  effectiveDate: string;
  signatures: Signature[];
  courtApproval?: CourtApproval;
}

type IdentityVerificationMethod =
  | 'BIOMETRIC_DNA'
  | 'BIOMETRIC_FINGERPRINT'
  | 'BIOMETRIC_FACIAL'
  | 'BIOMETRIC_IRIS'
  | 'DOCUMENT_VERIFICATION'
  | 'WITNESS_TESTIMONY'
  | 'CRYO_IDENTITY_VERIFICATION';

interface RestoredFamilyRelations {
  maritalStatus: 'RESTORED' | 'DISSOLVED' | 'NOT_APPLICABLE';
  spouseConsent: boolean;
  parentalRights: 'RESTORED' | 'MODIFIED' | 'TERMINATED';
  childrenAcknowledged: string[];
}

interface LicenseRestoration {
  licenseType: string;
  originalLicenseId: string;
  status: 'RESTORED' | 'REQUIRES_REEXAMINATION' | 'EXPIRED' | 'NOT_APPLICABLE';
  restorationDate?: string;
  conditions?: string[];
}

interface CourtApproval {
  courtName: string;
  caseNumber: string;
  judgeName: string;
  approvalDate: string;
  orderText: string;
}
```

### Jurisdiction Record

```typescript
interface JurisdictionRecord {
  jurisdiction: Jurisdiction;
  recognitionStatus: RecognitionStatus;
  localStatusEquivalent: string;
  registrationId: string;
  registrationDate: string;
  lastVerified: string;
  documents: JurisdictionDocument[];
  contacts: JurisdictionContact[];
}

interface Jurisdiction {
  country: string;
  countryCode: string;
  region?: string;
  locality?: string;
  legalSystem: LegalSystem;
  treatyMember: boolean;
  bilateralAgreements: string[];
}

type LegalSystem =
  | 'COMMON_LAW'
  | 'CIVIL_LAW'
  | 'RELIGIOUS_LAW'
  | 'CUSTOMARY_LAW'
  | 'MIXED';

type RecognitionStatus =
  | 'FULL_RECOGNITION'
  | 'PARTIAL_RECOGNITION'
  | 'PENDING_RECOGNITION'
  | 'NOT_RECOGNIZED'
  | 'TREATY_BASED';

interface JurisdictionDocument {
  documentId: string;
  documentType: string;
  localReference: string;
  issueDate: string;
  validUntil?: string;
  certified: boolean;
}

interface JurisdictionContact {
  authority: string;
  contactType: string;
  name: string;
  title: string;
  email: string;
  phone: string;
}
```

### Guardianship Record

```typescript
interface GuardianshipRecord {
  guardianshipId: string;
  subjectId: string;
  status: GuardianshipStatus;

  guardians: Guardian[];

  scope: {
    personalDecisions: boolean;
    medicalDecisions: boolean;
    financialDecisions: boolean;
    legalRepresentation: boolean;
    revivalDecisions: boolean;
  };

  appointment: {
    appointmentDate: string;
    appointingAuthority: LegalAuthority;
    courtOrder?: string;
    expirationDate?: string;
  };

  duties: GuardianDuty[];
  reportingRequirements: ReportingRequirement[];

  succession: {
    successorGuardians: Guardian[];
    successionRules: string;
  };

  termination?: {
    terminationDate: string;
    reason: TerminationReason;
    authority: LegalAuthority;
  };
}

type GuardianshipStatus =
  | 'ACTIVE'
  | 'SUSPENDED'
  | 'TERMINATED'
  | 'TRANSFERRED';

interface Guardian {
  guardianId: string;
  name: string;
  relationship: string;
  priority: number;
  contactInfo: ContactInfo;
  appointmentDate: string;
  qualifications: string[];
  bondAmount?: number;
  bondProvider?: string;
}

interface GuardianDuty {
  dutyType: string;
  description: string;
  frequency: string;
  reportingRequired: boolean;
}

interface ReportingRequirement {
  reportType: string;
  frequency: string;
  recipient: string;
  lastReport?: string;
  nextDue: string;
}

type TerminationReason =
  | 'REVIVAL_SUCCESSFUL'
  | 'PERMANENT_DEATH'
  | 'GUARDIAN_RESIGNATION'
  | 'GUARDIAN_REMOVAL'
  | 'COURT_ORDER';
```

### Legal Document

```typescript
interface LegalDocument {
  documentId: string;
  documentType: LegalDocumentType;
  title: string;

  content: {
    format: 'PDF' | 'TEXT' | 'STRUCTURED';
    hash: string;
    hashAlgorithm: string;
    size: number;
    encryptionStatus: 'ENCRYPTED' | 'PLAINTEXT';
  };

  metadata: {
    language: string;
    version: string;
    previousVersionId?: string;
    createdAt: string;
    createdBy: string;
    lastModified: string;
    modifiedBy: string;
  };

  authentication: {
    signed: boolean;
    signatures: Signature[];
    notarized: boolean;
    notarizationDetails?: NotarizationDetails;
    apostille?: ApostilleDetails;
  };

  jurisdiction: Jurisdiction;
  effectiveDate: string;
  expirationDate?: string;
  status: DocumentStatus;
}

type LegalDocumentType =
  | 'DEATH_CERTIFICATE'
  | 'PRESERVATION_DECLARATION'
  | 'RESTORATION_CERTIFICATE'
  | 'COURT_ORDER'
  | 'GUARDIANSHIP_ORDER'
  | 'TRUST_DOCUMENT'
  | 'POWER_OF_ATTORNEY'
  | 'CONSENT_DOCUMENT'
  | 'IDENTITY_VERIFICATION'
  | 'INTERNATIONAL_RECOGNITION';

type DocumentStatus =
  | 'DRAFT'
  | 'PENDING_SIGNATURE'
  | 'ACTIVE'
  | 'SUPERSEDED'
  | 'REVOKED'
  | 'EXPIRED';

interface Signature {
  signerId: string;
  signerName: string;
  signerRole: string;
  signatureDate: string;
  signatureType: 'HANDWRITTEN' | 'DIGITAL' | 'ELECTRONIC';
  digitalSignature?: string;
  certificateId?: string;
  verified: boolean;
}

interface NotarizationDetails {
  notaryId: string;
  notaryName: string;
  notaryJurisdiction: string;
  notarizationDate: string;
  notaryCommission: string;
  commissionExpiration: string;
  sealNumber: string;
}

interface ApostilleDetails {
  apostilleNumber: string;
  issuingCountry: string;
  issuingAuthority: string;
  issueDate: string;
  targetCountry?: string;
}
```

### Legal Authority

```typescript
interface LegalAuthority {
  authorityId: string;
  authorityType: AuthorityType;
  name: string;
  jurisdiction: Jurisdiction;

  officialInfo: {
    officialName: string;
    title: string;
    department: string;
    appointmentDate: string;
    termExpiration?: string;
  };

  contact: {
    address: string;
    phone: string;
    email: string;
    website: string;
  };

  powers: {
    canIssueDeath: boolean;
    canIssuePreservation: boolean;
    canIssueRestoration: boolean;
    canAppointGuardian: boolean;
    jurisdictionalLimits: string[];
  };

  verification: {
    verified: boolean;
    verificationDate: string;
    verificationMethod: string;
  };
}

type AuthorityType =
  | 'COURT'
  | 'GOVERNMENT_AGENCY'
  | 'VITAL_RECORDS_OFFICE'
  | 'MEDICAL_EXAMINER'
  | 'NOTARY_PUBLIC'
  | 'INTERNATIONAL_BODY';
```

### Witness

```typescript
interface Witness {
  witnessId: string;
  name: string;
  relationship: string;
  contactInfo: ContactInfo;
  identificationProvided: boolean;
  identificationTypes: string[];
  witnessDate: string;
  statement?: string;
  signature: Signature;
}

interface ContactInfo {
  address: string;
  city: string;
  region: string;
  country: string;
  postalCode: string;
  phone: string;
  email: string;
}
```

## JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/cryo-legal/v1",
  "definitions": {
    "LegalStatus": {
      "type": "string",
      "enum": [
        "LIVING",
        "LEGALLY_DECEASED",
        "PRESERVED_STATUS",
        "REVIVAL_PENDING",
        "LEGALLY_RESTORED",
        "PERMANENT_DEATH"
      ]
    },
    "LegalStatusRecord": {
      "type": "object",
      "required": ["recordId", "subjectId", "currentStatus"],
      "properties": {
        "recordId": { "type": "string" },
        "subjectId": { "type": "string" },
        "currentStatus": { "$ref": "#/definitions/LegalStatus" },
        "statusHistory": {
          "type": "array",
          "items": { "$ref": "#/definitions/StatusTransition" }
        }
      }
    },
    "StatusTransition": {
      "type": "object",
      "required": ["transitionId", "fromStatus", "toStatus", "transitionDate"],
      "properties": {
        "transitionId": { "type": "string" },
        "fromStatus": { "$ref": "#/definitions/LegalStatus" },
        "toStatus": { "$ref": "#/definitions/LegalStatus" },
        "transitionDate": { "type": "string", "format": "date-time" }
      }
    }
  }
}
```

---

*WIA Technical Committee - Cryopreservation Working Group*
