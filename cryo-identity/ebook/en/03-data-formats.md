# Chapter 3: Data Formats and Schemas

## 3.1 Overview

The WIA Cryo Identity Standard defines comprehensive data structures for managing identities in cryopreservation contexts. This chapter details the core schemas, validation rules, and data modeling patterns.

```typescript
import { z } from 'zod';

// Data format overview
const dataFormatOverview = {
  coreSchemas: [
    'Subject',              // Primary identity entity
    'Identifier',           // Identity credentials
    'Profile',              // Personal information
    'Biometric',            // Physical identifiers
    'Verification',         // Identity verification
    'Relationship',         // Subject connections
    'Directive'             // Identity instructions
  ],
  supportingSchemas: [
    'AuditEvent',           // Change tracking
    'ConsentRecord',        // Consent documentation
    'AccessGrant',          // Permission records
    'PrivacyPreference'     // Subject preferences
  ],
  validationLevels: [
    'Syntactic',            // Format validation
    'Semantic',             // Meaning validation
    'Contextual'            // Business rule validation
  ]
};
```

## 3.2 Core Identity Schemas

### 3.2.1 Subject Schema

```typescript
// Subject type enumeration
export const SubjectTypeSchema = z.enum([
  'individual',     // Adult with full capacity
  'minor',          // Subject under legal age
  'incapacitated',  // Limited capacity
  'posthumous'      // Deceased subject
]);

export type SubjectType = z.infer<typeof SubjectTypeSchema>;

// Subject status enumeration
export const SubjectStatusSchema = z.enum([
  'active',         // Currently active
  'suspended',      // Temporarily inactive
  'deceased',       // Subject has died
  'anonymized',     // Identity removed
  'merged'          // Combined with another
]);

export type SubjectStatus = z.infer<typeof SubjectStatusSchema>;

// Complete subject schema
export const SubjectSchema = z.object({
  id: z.string().uuid(),
  type: SubjectTypeSchema,
  status: SubjectStatusSchema,

  identifiers: z.array(z.lazy(() => SubjectIdentifierSchema)).min(1),
  profile: z.lazy(() => SubjectProfileSchema),
  biometrics: z.array(z.lazy(() => BiometricDataSchema)).optional(),

  specimens: z.array(z.lazy(() => SpecimenLinkSchema)),
  relationships: z.array(z.lazy(() => RelationshipSchema)).optional(),
  directives: z.array(z.lazy(() => IdentityDirectiveSchema)).optional(),

  consent: z.object({
    primaryConsentId: z.string().uuid(),
    additionalConsents: z.array(z.string().uuid())
  }),

  privacy: z.object({
    level: z.enum(['standard', 'enhanced', 'anonymous']),
    preferences: z.record(z.boolean()).optional()
  }),

  metadata: z.object({
    createdAt: z.string().datetime(),
    createdBy: z.string(),
    updatedAt: z.string().datetime().optional(),
    updatedBy: z.string().optional(),
    version: z.number().positive()
  }),

  history: z.array(z.lazy(() => SubjectEventSchema))
});

export type Subject = z.infer<typeof SubjectSchema>;
```

### 3.2.2 Identifier Schema

```typescript
// Identifier type enumeration
export const IdentifierTypeSchema = z.enum([
  'internal',       // System-generated UUID
  'national-id',    // Government ID number
  'passport',       // Passport number
  'medical-record', // Hospital/clinic record
  'donor-id',       // Donation registry ID
  'anonymous'       // De-identified code
]);

export type IdentifierType = z.infer<typeof IdentifierTypeSchema>;

// Subject identifier schema
export const SubjectIdentifierSchema = z.object({
  id: z.string().uuid(),
  type: IdentifierTypeSchema,
  value: z.string().min(1).max(100),

  issuer: z.object({
    name: z.string(),
    country: z.string().length(2), // ISO 3166-1 alpha-2
    type: z.enum(['government', 'healthcare', 'facility', 'registry'])
  }).optional(),

  validity: z.object({
    validFrom: z.string().datetime(),
    validTo: z.string().datetime().optional(),
    indefinite: z.boolean().default(false)
  }),

  verification: z.object({
    verified: z.boolean(),
    verifiedAt: z.string().datetime().optional(),
    verifiedBy: z.string().optional(),
    method: z.string().optional(),
    confidence: z.number().min(0).max(100).optional()
  }),

  flags: z.object({
    primary: z.boolean().default(false),
    active: z.boolean().default(true),
    sensitive: z.boolean().default(false)
  }),

  metadata: z.object({
    addedAt: z.string().datetime(),
    addedBy: z.string(),
    source: z.string().optional()
  })
});

export type SubjectIdentifier = z.infer<typeof SubjectIdentifierSchema>;

// Identifier validation rules
export const identifierValidationRules: Record<IdentifierType, RegExp | ((value: string) => boolean)> = {
  'internal': /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i,
  'national-id': (value) => value.length >= 6 && value.length <= 20,
  'passport': /^[A-Z0-9]{6,12}$/,
  'medical-record': (value) => value.length >= 4 && value.length <= 30,
  'donor-id': /^[A-Z0-9-]{8,20}$/,
  'anonymous': /^ANON-[A-Z0-9]{8,16}$/
};
```

### 3.2.3 Profile Schema

```typescript
// Legal name schema
export const LegalNameSchema = z.object({
  given: z.string().min(1).max(100),
  middle: z.string().max(100).optional(),
  family: z.string().min(1).max(100),
  suffix: z.string().max(20).optional(),

  script: z.enum(['latin', 'korean', 'chinese', 'japanese', 'arabic', 'cyrillic', 'other']).optional(),
  romanized: z.object({
    given: z.string(),
    family: z.string()
  }).optional(),

  previous: z.array(z.object({
    given: z.string(),
    middle: z.string().optional(),
    family: z.string(),
    changedAt: z.string().datetime(),
    reason: z.enum(['marriage', 'divorce', 'legal-change', 'adoption', 'other'])
  })).optional()
});

export type LegalName = z.infer<typeof LegalNameSchema>;

// Address schema
export const AddressSchema = z.object({
  type: z.enum(['residential', 'mailing', 'work', 'temporary']),
  street: z.string(),
  city: z.string(),
  state: z.string().optional(),
  postalCode: z.string(),
  country: z.string().length(2),
  validFrom: z.string().datetime().optional(),
  validTo: z.string().datetime().optional()
});

export type Address = z.infer<typeof AddressSchema>;

// Contact details schema
export const ContactDetailsSchema = z.object({
  email: z.string().email().optional(),
  phone: z.string().optional(),
  alternatePhone: z.string().optional(),
  addresses: z.array(AddressSchema),
  preferred: z.enum(['email', 'phone', 'mail']),
  doNotContact: z.boolean().default(false)
});

export type ContactDetails = z.infer<typeof ContactDetailsSchema>;

// Next of kin schema
export const NextOfKinSchema = z.object({
  id: z.string().uuid(),
  name: z.string(),
  relationship: z.enum([
    'spouse', 'parent', 'child', 'sibling',
    'grandparent', 'grandchild', 'aunt-uncle',
    'cousin', 'legal-guardian', 'designated-agent', 'other'
  ]),
  contact: ContactDetailsSchema,
  priority: z.number().positive(),
  authorized: z.object({
    forEmergency: z.boolean(),
    forDecisions: z.boolean(),
    forInformation: z.boolean(),
    scope: z.array(z.string()).optional()
  }),
  verification: z.object({
    verified: z.boolean(),
    verifiedAt: z.string().datetime().optional(),
    documentation: z.string().optional()
  })
});

export type NextOfKin = z.infer<typeof NextOfKinSchema>;

// Subject profile schema
export const SubjectProfileSchema = z.object({
  legalName: LegalNameSchema,
  dateOfBirth: z.string().regex(/^\d{4}-\d{2}-\d{2}$/),
  placeOfBirth: z.object({
    city: z.string(),
    state: z.string().optional(),
    country: z.string().length(2)
  }).optional(),

  nationality: z.array(z.string().length(2)),
  gender: z.enum(['male', 'female', 'non-binary', 'other', 'undisclosed']).optional(),

  contactInfo: ContactDetailsSchema.optional(),
  nextOfKin: z.array(NextOfKinSchema).optional(),

  medicalInfo: z.object({
    bloodType: z.enum(['A+', 'A-', 'B+', 'B-', 'AB+', 'AB-', 'O+', 'O-', 'unknown']).optional(),
    allergies: z.array(z.string()).optional(),
    conditions: z.array(z.string()).optional()
  }).optional(),

  preferences: z.object({
    language: z.string().length(2),
    timezone: z.string(),
    notifications: z.record(z.boolean())
  }).optional()
});

export type SubjectProfile = z.infer<typeof SubjectProfileSchema>;
```

### 3.2.4 Biometric Schema

```typescript
// Biometric type enumeration
export const BiometricTypeSchema = z.enum([
  'fingerprint',    // Fingerprint minutiae
  'facial',         // Facial geometry
  'iris',           // Iris pattern
  'dna',            // DNA profile
  'voice',          // Voice print
  'palm',           // Palm print
  'retina'          // Retinal scan
]);

export type BiometricType = z.infer<typeof BiometricTypeSchema>;

// Biometric data schema
export const BiometricDataSchema = z.object({
  id: z.string().uuid(),
  type: BiometricTypeSchema,

  template: z.object({
    format: z.string(), // e.g., 'ISO-19794-2', 'ANSI-378'
    version: z.string(),
    data: z.string(), // Base64 encoded template
    hash: z.string()  // SHA-256 hash for integrity
  }),

  capture: z.object({
    capturedAt: z.string().datetime(),
    capturedBy: z.string(),
    equipment: z.object({
      manufacturer: z.string(),
      model: z.string(),
      serialNumber: z.string().optional()
    }).optional(),
    location: z.string().optional()
  }),

  quality: z.object({
    score: z.number().min(0).max(100),
    acceptableForVerification: z.boolean(),
    degradationDetected: z.boolean().default(false)
  }),

  status: z.object({
    active: z.boolean().default(true),
    supersededBy: z.string().uuid().optional(),
    expiresAt: z.string().datetime().optional()
  }),

  metadata: z.object({
    addedAt: z.string().datetime(),
    updatedAt: z.string().datetime().optional()
  })
});

export type BiometricData = z.infer<typeof BiometricDataSchema>;

// DNA profile schema (specialized biometric)
export const DNAProfileSchema = z.object({
  type: z.literal('dna'),
  profileType: z.enum(['str', 'snp', 'mtdna', 'y-str']),

  markers: z.array(z.object({
    locus: z.string(),
    alleles: z.array(z.number())
  })),

  laboratory: z.object({
    name: z.string(),
    accreditation: z.string().optional(),
    reportNumber: z.string()
  }),

  collectionInfo: z.object({
    sampleType: z.enum(['blood', 'saliva', 'buccal', 'hair', 'other']),
    collectedAt: z.string().datetime(),
    collectedBy: z.string()
  }),

  chainOfCustody: z.array(z.object({
    action: z.string(),
    actor: z.string(),
    timestamp: z.string().datetime(),
    location: z.string().optional()
  }))
});

export type DNAProfile = z.infer<typeof DNAProfileSchema>;
```

## 3.3 Relationship and Link Schemas

### 3.3.1 Specimen Link Schema

```typescript
// Specimen link status
export const SpecimenLinkStatusSchema = z.enum([
  'active',         // Link is active
  'inactive',       // Link is inactive
  'disposed',       // Specimen disposed
  'transferred'     // Transferred to another facility
]);

// Specimen link schema
export const SpecimenLinkSchema = z.object({
  id: z.string().uuid(),
  specimenId: z.string(),
  type: z.string(), // e.g., 'oocyte', 'sperm', 'embryo', 'tissue'

  facility: z.object({
    id: z.string(),
    name: z.string(),
    country: z.string().length(2)
  }),

  storage: z.object({
    location: z.string(),
    container: z.string().optional(),
    position: z.string().optional()
  }).optional(),

  consent: z.object({
    consentId: z.string().uuid(),
    purposes: z.array(z.string()),
    restrictions: z.array(z.string()).optional()
  }),

  linking: z.object({
    linkedAt: z.string().datetime(),
    linkedBy: z.string(),
    method: z.enum(['registration', 'donation', 'transfer', 'inheritance']),
    documentation: z.string().optional()
  }),

  status: SpecimenLinkStatusSchema,

  history: z.array(z.object({
    action: z.string(),
    timestamp: z.string().datetime(),
    actor: z.string(),
    notes: z.string().optional()
  }))
});

export type SpecimenLink = z.infer<typeof SpecimenLinkSchema>;
```

### 3.3.2 Relationship Schema

```typescript
// Relationship type enumeration
export const RelationshipTypeSchema = z.enum([
  'parent',           // Parent of the subject
  'child',            // Child of the subject
  'sibling',          // Sibling
  'spouse',           // Spouse or partner
  'donor-recipient',  // Donor-recipient relationship
  'genetic',          // Genetic relationship
  'legal-guardian',   // Legal guardianship
  'surrogate',        // Surrogacy relationship
  'co-parent'         // Co-parenting arrangement
]);

export type RelationshipType = z.infer<typeof RelationshipTypeSchema>;

// Relationship schema
export const RelationshipSchema = z.object({
  id: z.string().uuid(),
  relatedSubjectId: z.string().uuid(),
  type: RelationshipTypeSchema,

  details: z.object({
    subtype: z.string().optional(), // e.g., 'biological', 'adoptive'
    direction: z.enum(['subject-to-related', 'related-to-subject', 'bidirectional']),
    startDate: z.string().datetime().optional(),
    endDate: z.string().datetime().optional()
  }),

  verification: z.object({
    verified: z.boolean(),
    verifiedAt: z.string().datetime().optional(),
    verifiedBy: z.string().optional(),
    method: z.enum(['documentation', 'dna-test', 'legal-declaration', 'self-declared']),
    evidence: z.array(z.string()).optional()
  }),

  permissions: z.object({
    viewProfile: z.boolean().default(false),
    viewSpecimens: z.boolean().default(false),
    makeDecisions: z.boolean().default(false),
    receiveNotifications: z.boolean().default(false)
  }).optional(),

  metadata: z.object({
    createdAt: z.string().datetime(),
    createdBy: z.string(),
    notes: z.string().optional()
  })
});

export type Relationship = z.infer<typeof RelationshipSchema>;
```

### 3.3.3 Identity Directive Schema

```typescript
// Directive type enumeration
export const DirectiveTypeSchema = z.enum([
  'name-change',          // Future name change instruction
  'anonymization',        // Request for anonymization
  'access-restriction',   // Access limitations
  'posthumous',           // Post-death instructions
  'research-consent',     // Research participation wishes
  'succession',           // Specimen inheritance
  'disposal'              // Specimen disposal wishes
]);

export type DirectiveType = z.infer<typeof DirectiveTypeSchema>;

// Identity directive schema
export const IdentityDirectiveSchema = z.object({
  id: z.string().uuid(),
  type: DirectiveTypeSchema,

  content: z.object({
    instruction: z.string(),
    parameters: z.record(z.unknown()).optional(),
    conditions: z.array(z.string()).optional()
  }),

  validity: z.object({
    effectiveFrom: z.string().datetime(),
    effectiveTo: z.string().datetime().optional(),
    triggerEvent: z.string().optional() // e.g., 'death', 'incapacity'
  }),

  execution: z.object({
    automaticExecution: z.boolean().default(false),
    requiresApproval: z.boolean().default(true),
    approvers: z.array(z.string()).optional()
  }),

  legal: z.object({
    legalBasis: z.string().optional(),
    witnesses: z.array(z.object({
      name: z.string(),
      date: z.string().datetime(),
      signature: z.string().optional()
    })).optional(),
    notarized: z.boolean().default(false),
    jurisdiction: z.string().optional()
  }),

  status: z.object({
    current: z.enum(['draft', 'active', 'executed', 'revoked', 'expired']),
    executedAt: z.string().datetime().optional(),
    revokedAt: z.string().datetime().optional()
  }),

  metadata: z.object({
    createdAt: z.string().datetime(),
    createdBy: z.string(),
    updatedAt: z.string().datetime().optional()
  })
});

export type IdentityDirective = z.infer<typeof IdentityDirectiveSchema>;
```

## 3.4 Verification Schemas

### 3.4.1 Verification Request Schema

```typescript
// Verification purpose
export const VerificationPurposeSchema = z.enum([
  'specimen-access',      // Accessing stored specimens
  'profile-update',       // Updating identity information
  'consent-change',       // Modifying consent
  'delegation',           // Delegating access
  'information-request',  // Requesting information
  'transfer',             // Transferring specimens
  'disposal'              // Disposing specimens
]);

// Verification method
export const VerificationMethodSchema = z.enum([
  'document',             // Document verification
  'biometric',            // Biometric matching
  'knowledge',            // Knowledge-based questions
  'possession',           // Token/device possession
  'third-party'           // External authority
]);

// Verification request schema
export const VerificationRequestSchema = z.object({
  id: z.string().uuid(),
  subjectId: z.string().uuid(),
  purpose: VerificationPurposeSchema,

  requestor: z.object({
    userId: z.string(),
    role: z.string(),
    facility: z.string(),
    ipAddress: z.string().optional()
  }),

  requiredMethods: z.array(VerificationMethodSchema).min(1),
  completedMethods: z.array(z.object({
    method: VerificationMethodSchema,
    completedAt: z.string().datetime(),
    result: z.enum(['pass', 'fail', 'inconclusive']),
    confidence: z.number().min(0).max(100),
    details: z.record(z.unknown()).optional()
  })),

  status: z.enum(['pending', 'in-progress', 'passed', 'failed', 'expired', 'cancelled']),

  timestamps: z.object({
    createdAt: z.string().datetime(),
    expiresAt: z.string().datetime(),
    completedAt: z.string().datetime().optional()
  }),

  result: z.object({
    passed: z.boolean().optional(),
    overallConfidence: z.number().min(0).max(100).optional(),
    approvedBy: z.string().optional(),
    notes: z.string().optional()
  }).optional()
});

export type VerificationRequest = z.infer<typeof VerificationRequestSchema>;
```

### 3.4.2 Verification Result Schema

```typescript
// Verification result schema
export const VerificationResultSchema = z.object({
  requestId: z.string().uuid(),
  subjectId: z.string().uuid(),

  outcome: z.enum(['verified', 'not-verified', 'requires-review']),

  methodResults: z.array(z.object({
    method: VerificationMethodSchema,
    status: z.enum(['pass', 'fail', 'skipped', 'error']),
    confidence: z.number().min(0).max(100),
    details: z.object({
      documentType: z.string().optional(),
      biometricMatch: z.number().optional(),
      questionsCorrect: z.number().optional(),
      totalQuestions: z.number().optional()
    }).optional(),
    timestamp: z.string().datetime()
  })),

  overallConfidence: z.number().min(0).max(100),

  riskAssessment: z.object({
    level: z.enum(['low', 'medium', 'high']),
    factors: z.array(z.string()),
    recommendations: z.array(z.string())
  }).optional(),

  authorization: z.object({
    granted: z.boolean(),
    scope: z.array(z.string()),
    validUntil: z.string().datetime(),
    restrictions: z.array(z.string()).optional()
  }).optional(),

  audit: z.object({
    verifiedAt: z.string().datetime(),
    verifiedBy: z.string(),
    systemVersion: z.string()
  })
});

export type VerificationResult = z.infer<typeof VerificationResultSchema>;
```

## 3.5 Audit and Event Schemas

### 3.5.1 Subject Event Schema

```typescript
// Event type enumeration
export const EventTypeSchema = z.enum([
  'created',          // Subject created
  'updated',          // Profile updated
  'verified',         // Identity verified
  'suspended',        // Subject suspended
  'reactivated',      // Subject reactivated
  'merged',           // Merged with another
  'anonymized',       // Identity anonymized
  'deceased',         // Death recorded
  'specimen-linked',  // Specimen linked
  'specimen-unlinked',// Specimen unlinked
  'relationship-added', // Relationship added
  'directive-added',  // Directive added
  'access-granted',   // Access granted
  'access-revoked'    // Access revoked
]);

export type EventType = z.infer<typeof EventTypeSchema>;

// Subject event schema
export const SubjectEventSchema = z.object({
  id: z.string().uuid(),
  subjectId: z.string().uuid(),
  type: EventTypeSchema,

  timestamp: z.string().datetime(),

  actor: z.object({
    userId: z.string(),
    role: z.string(),
    facility: z.string().optional(),
    ipAddress: z.string().optional()
  }),

  description: z.string(),

  changes: z.array(z.object({
    field: z.string(),
    before: z.unknown(),
    after: z.unknown()
  })).optional(),

  related: z.object({
    verificationId: z.string().uuid().optional(),
    specimenId: z.string().optional(),
    relationshipId: z.string().uuid().optional(),
    directiveId: z.string().uuid().optional()
  }).optional(),

  metadata: z.object({
    source: z.enum(['api', 'ui', 'system', 'import']),
    version: z.number(),
    correlationId: z.string().optional()
  })
});

export type SubjectEvent = z.infer<typeof SubjectEventSchema>;
```

## 3.6 Data Validation

### 3.6.1 Validation Utilities

```typescript
// Validation result type
interface ValidationResult<T> {
  success: boolean;
  data?: T;
  errors?: ValidationError[];
}

interface ValidationError {
  path: string;
  message: string;
  code: string;
}

// Identity data validator
export class CryoIdentityDataValidator {
  // Validate subject
  static validateSubject(data: unknown): ValidationResult<Subject> {
    try {
      const result = SubjectSchema.parse(data);
      return { success: true, data: result };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          success: false,
          errors: error.errors.map(err => ({
            path: err.path.join('.'),
            message: err.message,
            code: err.code
          }))
        };
      }
      throw error;
    }
  }

  // Validate identifier
  static validateIdentifier(data: unknown): ValidationResult<SubjectIdentifier> {
    try {
      const result = SubjectIdentifierSchema.parse(data);

      // Additional business rule validation
      const typeValidation = identifierValidationRules[result.type];
      if (typeValidation) {
        const isValid = typeof typeValidation === 'function'
          ? typeValidation(result.value)
          : typeValidation.test(result.value);

        if (!isValid) {
          return {
            success: false,
            errors: [{
              path: 'value',
              message: `Invalid format for identifier type ${result.type}`,
              code: 'INVALID_FORMAT'
            }]
          };
        }
      }

      return { success: true, data: result };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          success: false,
          errors: error.errors.map(err => ({
            path: err.path.join('.'),
            message: err.message,
            code: err.code
          }))
        };
      }
      throw error;
    }
  }

  // Validate verification request
  static validateVerificationRequest(
    data: unknown
  ): ValidationResult<VerificationRequest> {
    try {
      const result = VerificationRequestSchema.parse(data);

      // Business rule: Check method requirements
      if (result.requiredMethods.length === 0) {
        return {
          success: false,
          errors: [{
            path: 'requiredMethods',
            message: 'At least one verification method is required',
            code: 'INSUFFICIENT_METHODS'
          }]
        };
      }

      return { success: true, data: result };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          success: false,
          errors: error.errors.map(err => ({
            path: err.path.join('.'),
            message: err.message,
            code: err.code
          }))
        };
      }
      throw error;
    }
  }

  // Cross-field validation
  static validateSubjectConsistency(subject: Subject): ValidationError[] {
    const errors: ValidationError[] = [];

    // Check for primary identifier
    const hasPrimary = subject.identifiers.some(id => id.flags.primary);
    if (!hasPrimary) {
      errors.push({
        path: 'identifiers',
        message: 'Subject must have at least one primary identifier',
        code: 'NO_PRIMARY_IDENTIFIER'
      });
    }

    // Check minor has guardian
    if (subject.type === 'minor') {
      const hasGuardian = subject.relationships?.some(
        r => r.type === 'legal-guardian' && r.permissions?.makeDecisions
      );
      if (!hasGuardian) {
        errors.push({
          path: 'relationships',
          message: 'Minor subject must have a legal guardian relationship',
          code: 'MINOR_NO_GUARDIAN'
        });
      }
    }

    // Check deceased has death date in directive
    if (subject.status === 'deceased') {
      const hasDeathDirective = subject.directives?.some(
        d => d.type === 'posthumous' && d.status.current === 'active'
      );
      if (!hasDeathDirective) {
        errors.push({
          path: 'directives',
          message: 'Deceased subject should have posthumous directive',
          code: 'DECEASED_NO_DIRECTIVE'
        });
      }
    }

    return errors;
  }
}
```

## 3.7 Summary

This chapter defined the comprehensive data schemas for the WIA Cryo Identity Standard:

| Schema Category | Key Schemas | Purpose |
|----------------|-------------|---------|
| Core Identity | Subject, Identifier, Profile | Primary identity representation |
| Biometrics | BiometricData, DNAProfile | Physical identity verification |
| Relationships | SpecimenLink, Relationship | Identity connections |
| Directives | IdentityDirective | Subject instructions |
| Verification | VerificationRequest/Result | Identity verification |
| Audit | SubjectEvent | Change tracking |

Key design principles:
- Strong typing with Zod validation
- Extensible schema structure
- Comprehensive audit trailing
- Privacy-aware design
- International compatibility

---

© 2025 WIA Standards. All rights reserved.
