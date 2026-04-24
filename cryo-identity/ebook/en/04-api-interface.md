# Chapter 4: API Interface

## 4.1 Overview

The WIA Cryo Identity Standard provides comprehensive APIs for identity management, verification, and privacy-preserving operations. This chapter details REST, GraphQL, and WebSocket interfaces for building identity management systems.

```typescript
// API architecture overview
const apiArchitecture = {
  restAPI: {
    version: 'v1',
    baseUrl: '/api/v1/identity',
    authentication: 'OAuth 2.0 + JWT',
    rateLimit: '1000 requests/minute'
  },
  graphQL: {
    endpoint: '/graphql',
    subscriptions: '/graphql/ws',
    introspection: 'development-only'
  },
  webSocket: {
    endpoint: '/ws/identity',
    protocols: ['verification', 'notifications'],
    heartbeat: '30 seconds'
  },
  features: [
    'Subject management',
    'Identity verification',
    'Relationship management',
    'Consent tracking',
    'Audit logging'
  ]
};
```

## 4.2 REST API Design

### 4.2.1 Subject Management API

```typescript
import express, { Router, Request, Response, NextFunction } from 'express';
import { z } from 'zod';

// Subject router
function createSubjectRouter(): Router {
  const router = Router();

  // List subjects
  router.get('/',
    requirePermission('subject:read'),
    validateRequest(z.object({
      query: z.object({
        page: z.coerce.number().min(1).default(1),
        limit: z.coerce.number().min(1).max(100).default(20),
        status: SubjectStatusSchema.optional(),
        type: SubjectTypeSchema.optional(),
        search: z.string().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { page, limit, status, type, search } = req.query;

      const subjects = await subjectService.findAll({
        pagination: { page, limit },
        filters: { status, type, search }
      });

      res.json({
        data: subjects.items,
        pagination: {
          page: subjects.page,
          limit: subjects.limit,
          total: subjects.total,
          totalPages: subjects.totalPages
        }
      });
    }
  );

  // Get subject by ID
  router.get('/:subjectId',
    requirePermission('subject:read'),
    validateRequest(z.object({
      params: z.object({
        subjectId: z.string().uuid()
      }),
      query: z.object({
        include: z.array(z.enum([
          'identifiers', 'profile', 'biometrics',
          'specimens', 'relationships', 'directives'
        ])).optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { subjectId } = req.params;
      const { include } = req.query;

      const subject = await subjectService.findById(subjectId, { include });

      if (!subject) {
        res.status(404).json({
          error: 'Not Found',
          message: `Subject not found: ${subjectId}`
        });
        return;
      }

      // Apply privacy filters
      const filtered = await privacyService.filterSubjectData(
        subject,
        req.user!,
        req.query.include || []
      );

      res.json({ data: filtered });
    }
  );

  // Register new subject
  router.post('/',
    requirePermission('subject:create'),
    validateRequest(z.object({
      body: CreateSubjectSchema
    })),
    async (req: Request, res: Response) => {
      // Duplicate check
      const existing = await subjectService.findByIdentifier(
        req.body.identifiers[0]
      );

      if (existing) {
        res.status(409).json({
          error: 'Conflict',
          message: 'Subject with this identifier already exists',
          existingId: existing.id
        });
        return;
      }

      const subject = await subjectService.create(req.body, {
        createdBy: req.user!.id,
        facility: req.user!.facility
      });

      // Audit log
      await auditService.log({
        action: 'SUBJECT_CREATED',
        subjectId: subject.id,
        actor: req.user!.id,
        details: { type: req.body.type }
      });

      res.status(201).json({
        data: subject,
        message: 'Subject registered successfully'
      });
    }
  );

  // Update subject
  router.put('/:subjectId',
    requirePermission('subject:update'),
    requireVerification('profile-update'),
    validateRequest(z.object({
      params: z.object({
        subjectId: z.string().uuid()
      }),
      body: UpdateSubjectSchema
    })),
    async (req: Request, res: Response) => {
      const { subjectId } = req.params;

      const subject = await subjectService.update(subjectId, req.body, {
        updatedBy: req.user!.id,
        verificationId: req.verificationId
      });

      res.json({
        data: subject,
        message: 'Subject updated successfully'
      });
    }
  );

  // Update subject status
  router.patch('/:subjectId/status',
    requirePermission('subject:status'),
    validateRequest(z.object({
      params: z.object({
        subjectId: z.string().uuid()
      }),
      body: z.object({
        status: SubjectStatusSchema,
        reason: z.string(),
        effectiveDate: z.string().datetime().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { subjectId } = req.params;
      const { status, reason, effectiveDate } = req.body;

      const subject = await subjectService.updateStatus(subjectId, {
        status,
        reason,
        effectiveDate,
        updatedBy: req.user!.id
      });

      res.json({
        data: subject,
        message: `Subject status changed to ${status}`
      });
    }
  );

  // Merge subjects
  router.post('/:subjectId/merge',
    requirePermission('subject:merge'),
    requireVerification('subject-merge'),
    validateRequest(z.object({
      params: z.object({
        subjectId: z.string().uuid()
      }),
      body: z.object({
        mergeWithId: z.string().uuid(),
        strategy: z.enum(['keep-primary', 'keep-secondary', 'combine']),
        reason: z.string()
      })
    })),
    async (req: Request, res: Response) => {
      const { subjectId } = req.params;
      const { mergeWithId, strategy, reason } = req.body;

      const merged = await subjectService.merge(subjectId, mergeWithId, {
        strategy,
        reason,
        mergedBy: req.user!.id,
        verificationId: req.verificationId
      });

      res.json({
        data: merged,
        message: 'Subjects merged successfully'
      });
    }
  );

  return router;
}

// Create subject schema
const CreateSubjectSchema = z.object({
  type: SubjectTypeSchema,
  identifiers: z.array(z.object({
    type: IdentifierTypeSchema,
    value: z.string(),
    issuer: z.object({
      name: z.string(),
      country: z.string().length(2),
      type: z.enum(['government', 'healthcare', 'facility', 'registry'])
    }).optional(),
    primary: z.boolean().default(false)
  })).min(1),
  profile: z.object({
    legalName: LegalNameSchema,
    dateOfBirth: z.string().regex(/^\d{4}-\d{2}-\d{2}$/),
    nationality: z.array(z.string().length(2)).min(1),
    gender: z.enum(['male', 'female', 'non-binary', 'other', 'undisclosed']).optional()
  }),
  consent: z.object({
    consentId: z.string().uuid(),
    purposes: z.array(z.string())
  }),
  guardian: z.object({
    subjectId: z.string().uuid().optional(),
    name: z.string(),
    relationship: z.string(),
    contact: ContactDetailsSchema
  }).optional() // Required for minors
});

// Update subject schema
const UpdateSubjectSchema = z.object({
  profile: z.object({
    legalName: LegalNameSchema.optional(),
    contactInfo: ContactDetailsSchema.optional(),
    nextOfKin: z.array(NextOfKinSchema).optional()
  }).optional(),
  privacy: z.object({
    level: z.enum(['standard', 'enhanced', 'anonymous']),
    preferences: z.record(z.boolean())
  }).optional()
});
```

### 4.2.2 Identity Verification API

```typescript
// Verification router
function createVerificationRouter(): Router {
  const router = Router();

  // Initiate verification
  router.post('/initiate',
    requirePermission('verification:initiate'),
    validateRequest(z.object({
      body: z.object({
        subjectId: z.string().uuid(),
        purpose: VerificationPurposeSchema,
        methods: z.array(VerificationMethodSchema).min(1),
        context: z.object({
          specimenId: z.string().optional(),
          facilityId: z.string().optional(),
          reason: z.string().optional()
        }).optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { subjectId, purpose, methods, context } = req.body;

      // Get verification requirements
      const requirements = await verificationService.getRequirements(
        subjectId,
        purpose
      );

      // Validate requested methods meet requirements
      if (!verificationService.methodsMeetRequirements(methods, requirements)) {
        res.status(400).json({
          error: 'Insufficient Methods',
          message: 'Requested methods do not meet verification requirements',
          required: requirements
        });
        return;
      }

      const session = await verificationService.initiate({
        subjectId,
        purpose,
        methods,
        context,
        requestor: req.user!
      });

      res.status(201).json({
        data: {
          sessionId: session.id,
          status: session.status,
          methods: session.pendingMethods,
          expiresAt: session.expiresAt
        }
      });
    }
  );

  // Submit verification step
  router.post('/:sessionId/submit',
    validateRequest(z.object({
      params: z.object({
        sessionId: z.string().uuid()
      }),
      body: z.object({
        method: VerificationMethodSchema,
        data: z.record(z.unknown())
      })
    })),
    async (req: Request, res: Response) => {
      const { sessionId } = req.params;
      const { method, data } = req.body;

      const result = await verificationService.submitStep(sessionId, method, data);

      if (result.sessionComplete) {
        res.json({
          data: {
            sessionId,
            status: result.status,
            passed: result.passed,
            confidence: result.confidence,
            authorization: result.authorization
          }
        });
      } else {
        res.json({
          data: {
            sessionId,
            status: 'in-progress',
            completedMethods: result.completedMethods,
            pendingMethods: result.pendingMethods
          }
        });
      }
    }
  );

  // Submit document for verification
  router.post('/:sessionId/document',
    upload.single('document'),
    validateRequest(z.object({
      params: z.object({
        sessionId: z.string().uuid()
      }),
      body: z.object({
        documentType: z.enum(['passport', 'national-id', 'drivers-license', 'other']),
        side: z.enum(['front', 'back']).optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { sessionId } = req.params;
      const { documentType, side } = req.body;

      if (!req.file) {
        res.status(400).json({ error: 'No document uploaded' });
        return;
      }

      const result = await verificationService.verifyDocument(sessionId, {
        type: documentType,
        side,
        file: req.file
      });

      res.json({
        data: {
          verified: result.verified,
          confidence: result.confidence,
          extractedData: result.extractedData,
          warnings: result.warnings
        }
      });
    }
  );

  // Submit biometric for verification
  router.post('/:sessionId/biometric',
    validateRequest(z.object({
      params: z.object({
        sessionId: z.string().uuid()
      }),
      body: z.object({
        type: BiometricTypeSchema,
        template: z.string(), // Base64 encoded
        captureInfo: z.object({
          equipment: z.string().optional(),
          quality: z.number().min(0).max(100)
        })
      })
    })),
    async (req: Request, res: Response) => {
      const { sessionId } = req.params;
      const { type, template, captureInfo } = req.body;

      const result = await verificationService.verifyBiometric(sessionId, {
        type,
        template,
        captureInfo
      });

      res.json({
        data: {
          matched: result.matched,
          confidence: result.confidence,
          liveness: result.livenessDetected
        }
      });
    }
  );

  // Get verification session status
  router.get('/:sessionId',
    validateRequest(z.object({
      params: z.object({
        sessionId: z.string().uuid()
      })
    })),
    async (req: Request, res: Response) => {
      const { sessionId } = req.params;

      const session = await verificationService.getSession(sessionId);

      if (!session) {
        res.status(404).json({
          error: 'Not Found',
          message: 'Verification session not found'
        });
        return;
      }

      res.json({ data: session });
    }
  );

  // Cancel verification session
  router.delete('/:sessionId',
    validateRequest(z.object({
      params: z.object({
        sessionId: z.string().uuid()
      }),
      body: z.object({
        reason: z.string()
      })
    })),
    async (req: Request, res: Response) => {
      const { sessionId } = req.params;
      const { reason } = req.body;

      await verificationService.cancelSession(sessionId, {
        reason,
        cancelledBy: req.user!.id
      });

      res.json({ message: 'Verification session cancelled' });
    }
  );

  return router;
}
```

### 4.2.3 Relationship Management API

```typescript
// Relationship router
function createRelationshipRouter(): Router {
  const router = Router();

  // Get subject relationships
  router.get('/subject/:subjectId',
    requirePermission('relationship:read'),
    validateRequest(z.object({
      params: z.object({
        subjectId: z.string().uuid()
      }),
      query: z.object({
        type: RelationshipTypeSchema.optional(),
        verified: z.coerce.boolean().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { subjectId } = req.params;
      const { type, verified } = req.query;

      const relationships = await relationshipService.findBySubject(subjectId, {
        type,
        verified
      });

      res.json({ data: relationships });
    }
  );

  // Add relationship
  router.post('/',
    requirePermission('relationship:create'),
    validateRequest(z.object({
      body: z.object({
        subjectId: z.string().uuid(),
        relatedSubjectId: z.string().uuid(),
        type: RelationshipTypeSchema,
        details: z.object({
          subtype: z.string().optional(),
          startDate: z.string().datetime().optional()
        }).optional(),
        evidence: z.array(z.string()).optional()
      })
    })),
    async (req: Request, res: Response) => {
      const relationship = await relationshipService.create(req.body, {
        createdBy: req.user!.id
      });

      res.status(201).json({
        data: relationship,
        message: 'Relationship created successfully'
      });
    }
  );

  // Verify relationship
  router.post('/:relationshipId/verify',
    requirePermission('relationship:verify'),
    validateRequest(z.object({
      params: z.object({
        relationshipId: z.string().uuid()
      }),
      body: z.object({
        method: z.enum(['documentation', 'dna-test', 'legal-declaration', 'third-party']),
        evidence: z.array(z.string()),
        notes: z.string().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { relationshipId } = req.params;
      const { method, evidence, notes } = req.body;

      const verified = await relationshipService.verify(relationshipId, {
        method,
        evidence,
        notes,
        verifiedBy: req.user!.id
      });

      res.json({
        data: verified,
        message: 'Relationship verified successfully'
      });
    }
  );

  // Update relationship permissions
  router.patch('/:relationshipId/permissions',
    requirePermission('relationship:update'),
    requireVerification('relationship-permission-change'),
    validateRequest(z.object({
      params: z.object({
        relationshipId: z.string().uuid()
      }),
      body: z.object({
        permissions: z.object({
          viewProfile: z.boolean().optional(),
          viewSpecimens: z.boolean().optional(),
          makeDecisions: z.boolean().optional(),
          receiveNotifications: z.boolean().optional()
        })
      })
    })),
    async (req: Request, res: Response) => {
      const { relationshipId } = req.params;
      const { permissions } = req.body;

      const updated = await relationshipService.updatePermissions(
        relationshipId,
        permissions,
        {
          updatedBy: req.user!.id,
          verificationId: req.verificationId
        }
      );

      res.json({
        data: updated,
        message: 'Permissions updated successfully'
      });
    }
  );

  // End relationship
  router.delete('/:relationshipId',
    requirePermission('relationship:delete'),
    validateRequest(z.object({
      params: z.object({
        relationshipId: z.string().uuid()
      }),
      body: z.object({
        reason: z.string(),
        endDate: z.string().datetime().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { relationshipId } = req.params;
      const { reason, endDate } = req.body;

      await relationshipService.end(relationshipId, {
        reason,
        endDate,
        endedBy: req.user!.id
      });

      res.json({ message: 'Relationship ended successfully' });
    }
  );

  return router;
}
```

## 4.3 GraphQL API

### 4.3.1 GraphQL Schema

```graphql
# GraphQL Schema for Cryo Identity

scalar DateTime
scalar JSON

# Subject types
type Subject {
  id: ID!
  type: SubjectType!
  status: SubjectStatus!

  identifiers: [SubjectIdentifier!]!
  profile: SubjectProfile!
  biometrics: [BiometricData!]

  specimens: [SpecimenLink!]!
  relationships: [Relationship!]
  directives: [IdentityDirective!]

  history(limit: Int = 20): [SubjectEvent!]!

  createdAt: DateTime!
  updatedAt: DateTime
}

enum SubjectType {
  INDIVIDUAL
  MINOR
  INCAPACITATED
  POSTHUMOUS
}

enum SubjectStatus {
  ACTIVE
  SUSPENDED
  DECEASED
  ANONYMIZED
  MERGED
}

type SubjectIdentifier {
  id: ID!
  type: IdentifierType!
  value: String!
  issuer: Issuer
  verified: Boolean!
  primary: Boolean!
  validFrom: DateTime!
  validTo: DateTime
}

enum IdentifierType {
  INTERNAL
  NATIONAL_ID
  PASSPORT
  MEDICAL_RECORD
  DONOR_ID
  ANONYMOUS
}

type Issuer {
  name: String!
  country: String!
  type: String!
}

type SubjectProfile {
  legalName: LegalName!
  dateOfBirth: String!
  placeOfBirth: PlaceOfBirth
  nationality: [String!]!
  gender: String
  contactInfo: ContactDetails
  nextOfKin: [NextOfKin!]
}

type LegalName {
  given: String!
  middle: String
  family: String!
  suffix: String
  previous: [PreviousName!]
}

type PreviousName {
  given: String!
  middle: String
  family: String!
  changedAt: DateTime!
  reason: String!
}

type PlaceOfBirth {
  city: String!
  state: String
  country: String!
}

type ContactDetails {
  email: String
  phone: String
  addresses: [Address!]!
  preferred: String!
}

type Address {
  type: String!
  street: String!
  city: String!
  state: String
  postalCode: String!
  country: String!
}

type NextOfKin {
  id: ID!
  name: String!
  relationship: String!
  contact: ContactDetails!
  priority: Int!
  authorized: NextOfKinAuthorization!
}

type NextOfKinAuthorization {
  forEmergency: Boolean!
  forDecisions: Boolean!
  forInformation: Boolean!
}

type BiometricData {
  id: ID!
  type: BiometricType!
  quality: BiometricQuality!
  capturedAt: DateTime!
  active: Boolean!
}

enum BiometricType {
  FINGERPRINT
  FACIAL
  IRIS
  DNA
  VOICE
}

type BiometricQuality {
  score: Float!
  acceptableForVerification: Boolean!
}

type SpecimenLink {
  id: ID!
  specimenId: String!
  type: String!
  facility: Facility!
  status: String!
  linkedAt: DateTime!
}

type Facility {
  id: ID!
  name: String!
  country: String!
}

type Relationship {
  id: ID!
  relatedSubject: Subject!
  type: RelationshipType!
  verified: Boolean!
  verifiedAt: DateTime
  permissions: RelationshipPermissions
}

enum RelationshipType {
  PARENT
  CHILD
  SIBLING
  SPOUSE
  DONOR_RECIPIENT
  GENETIC
  LEGAL_GUARDIAN
  SURROGATE
  CO_PARENT
}

type RelationshipPermissions {
  viewProfile: Boolean!
  viewSpecimens: Boolean!
  makeDecisions: Boolean!
  receiveNotifications: Boolean!
}

type IdentityDirective {
  id: ID!
  type: DirectiveType!
  content: DirectiveContent!
  status: DirectiveStatus!
  effectiveFrom: DateTime!
  effectiveTo: DateTime
}

enum DirectiveType {
  NAME_CHANGE
  ANONYMIZATION
  ACCESS_RESTRICTION
  POSTHUMOUS
  RESEARCH_CONSENT
  SUCCESSION
  DISPOSAL
}

type DirectiveContent {
  instruction: String!
  conditions: [String!]
}

enum DirectiveStatus {
  DRAFT
  ACTIVE
  EXECUTED
  REVOKED
  EXPIRED
}

type SubjectEvent {
  id: ID!
  type: String!
  timestamp: DateTime!
  actor: String!
  description: String!
  changes: JSON
}

# Verification types
type VerificationSession {
  id: ID!
  subjectId: ID!
  purpose: String!
  status: VerificationStatus!
  methods: [VerificationMethodStatus!]!
  expiresAt: DateTime!
  result: VerificationResult
}

enum VerificationStatus {
  PENDING
  IN_PROGRESS
  PASSED
  FAILED
  EXPIRED
  CANCELLED
}

type VerificationMethodStatus {
  method: String!
  status: String!
  confidence: Float
  completedAt: DateTime
}

type VerificationResult {
  passed: Boolean!
  confidence: Float!
  authorization: Authorization
}

type Authorization {
  granted: Boolean!
  scope: [String!]!
  validUntil: DateTime!
}

# Queries
type Query {
  # Subject queries
  subject(id: ID!): Subject
  subjects(
    filter: SubjectFilter
    pagination: PaginationInput
  ): SubjectConnection!

  searchSubjects(
    query: String!
    type: SubjectType
    limit: Int = 10
  ): [Subject!]!

  # Verification queries
  verificationSession(id: ID!): VerificationSession
  verificationHistory(
    subjectId: ID!
    limit: Int = 20
  ): [VerificationSession!]!

  # Relationship queries
  relationships(
    subjectId: ID!
    type: RelationshipType
  ): [Relationship!]!

  # Directive queries
  directives(
    subjectId: ID!
    type: DirectiveType
    status: DirectiveStatus
  ): [IdentityDirective!]!
}

input SubjectFilter {
  status: SubjectStatus
  type: SubjectType
  hasSpecimens: Boolean
  facilityId: ID
}

input PaginationInput {
  page: Int = 1
  limit: Int = 20
}

type SubjectConnection {
  items: [Subject!]!
  pageInfo: PageInfo!
}

type PageInfo {
  page: Int!
  limit: Int!
  total: Int!
  totalPages: Int!
  hasNextPage: Boolean!
}

# Mutations
type Mutation {
  # Subject mutations
  registerSubject(input: RegisterSubjectInput!): Subject!
  updateSubject(id: ID!, input: UpdateSubjectInput!): Subject!
  updateSubjectStatus(
    id: ID!
    status: SubjectStatus!
    reason: String!
  ): Subject!
  mergeSubjects(
    primaryId: ID!
    secondaryId: ID!
    strategy: MergeStrategy!
    reason: String!
  ): Subject!

  # Identifier mutations
  addIdentifier(
    subjectId: ID!
    input: AddIdentifierInput!
  ): SubjectIdentifier!
  verifyIdentifier(
    identifierId: ID!
    method: String!
    evidence: [String!]
  ): SubjectIdentifier!

  # Verification mutations
  initiateVerification(
    input: InitiateVerificationInput!
  ): VerificationSession!
  submitVerificationStep(
    sessionId: ID!
    method: String!
    data: JSON!
  ): VerificationSession!
  cancelVerification(
    sessionId: ID!
    reason: String!
  ): Boolean!

  # Relationship mutations
  addRelationship(input: AddRelationshipInput!): Relationship!
  verifyRelationship(
    relationshipId: ID!
    method: String!
    evidence: [String!]!
  ): Relationship!
  updateRelationshipPermissions(
    relationshipId: ID!
    permissions: PermissionsInput!
  ): Relationship!
  endRelationship(
    relationshipId: ID!
    reason: String!
  ): Boolean!

  # Directive mutations
  addDirective(input: AddDirectiveInput!): IdentityDirective!
  updateDirectiveStatus(
    directiveId: ID!
    status: DirectiveStatus!
    reason: String
  ): IdentityDirective!
}

input RegisterSubjectInput {
  type: SubjectType!
  identifiers: [IdentifierInput!]!
  profile: ProfileInput!
  consentId: ID!
}

input IdentifierInput {
  type: IdentifierType!
  value: String!
  issuer: IssuerInput
  primary: Boolean
}

input IssuerInput {
  name: String!
  country: String!
  type: String!
}

input ProfileInput {
  legalName: LegalNameInput!
  dateOfBirth: String!
  nationality: [String!]!
  gender: String
}

input LegalNameInput {
  given: String!
  middle: String
  family: String!
  suffix: String
}

input UpdateSubjectInput {
  profile: ProfileUpdateInput
  privacy: PrivacyInput
}

input ProfileUpdateInput {
  contactInfo: ContactInfoInput
  nextOfKin: [NextOfKinInput!]
}

input ContactInfoInput {
  email: String
  phone: String
  addresses: [AddressInput!]
  preferred: String
}

input AddressInput {
  type: String!
  street: String!
  city: String!
  state: String
  postalCode: String!
  country: String!
}

input NextOfKinInput {
  name: String!
  relationship: String!
  contact: ContactInfoInput!
  priority: Int!
  authorization: AuthorizationInput!
}

input AuthorizationInput {
  forEmergency: Boolean!
  forDecisions: Boolean!
  forInformation: Boolean!
}

input PrivacyInput {
  level: String!
  preferences: JSON
}

enum MergeStrategy {
  KEEP_PRIMARY
  KEEP_SECONDARY
  COMBINE
}

input AddIdentifierInput {
  type: IdentifierType!
  value: String!
  issuer: IssuerInput
  primary: Boolean
}

input InitiateVerificationInput {
  subjectId: ID!
  purpose: String!
  methods: [String!]!
  context: JSON
}

input AddRelationshipInput {
  subjectId: ID!
  relatedSubjectId: ID!
  type: RelationshipType!
  subtype: String
  evidence: [String!]
}

input PermissionsInput {
  viewProfile: Boolean
  viewSpecimens: Boolean
  makeDecisions: Boolean
  receiveNotifications: Boolean
}

input AddDirectiveInput {
  subjectId: ID!
  type: DirectiveType!
  instruction: String!
  conditions: [String!]
  effectiveFrom: DateTime!
  effectiveTo: DateTime
}

# Subscriptions
type Subscription {
  # Real-time verification updates
  verificationProgress(sessionId: ID!): VerificationSession!

  # Subject change notifications
  subjectUpdated(subjectId: ID!): Subject!

  # Relationship notifications
  relationshipChanged(subjectId: ID!): Relationship!
}
```

## 4.4 WebSocket Real-time API

### 4.4.1 WebSocket Server

```typescript
import { WebSocketServer, WebSocket } from 'ws';

// WebSocket message types
interface WSMessage {
  type: string;
  payload: any;
}

// Identity WebSocket Server
export class IdentityWebSocketServer {
  private wss: WebSocketServer;
  private clients: Map<string, WSClient> = new Map();

  constructor(server: any) {
    this.wss = new WebSocketServer({ server, path: '/ws/identity' });
    this.wss.on('connection', this.handleConnection.bind(this));
  }

  private async handleConnection(ws: WebSocket, req: any) {
    const token = this.extractToken(req);
    const user = await this.authenticateToken(token);

    if (!user) {
      ws.close(4001, 'Unauthorized');
      return;
    }

    const clientId = this.generateClientId();
    const client: WSClient = {
      id: clientId,
      ws,
      user,
      subscriptions: new Set()
    };

    this.clients.set(clientId, client);

    ws.on('message', (data) => this.handleMessage(client, data));
    ws.on('close', () => this.handleDisconnect(client));

    this.send(ws, {
      type: 'connected',
      payload: { clientId }
    });
  }

  private handleMessage(client: WSClient, data: any) {
    const message: WSMessage = JSON.parse(data.toString());

    switch (message.type) {
      case 'subscribe-verification':
        this.subscribeToVerification(client, message.payload.sessionId);
        break;

      case 'subscribe-subject':
        this.subscribeToSubject(client, message.payload.subjectId);
        break;

      case 'unsubscribe':
        client.subscriptions.delete(message.payload.topic);
        break;

      default:
        this.send(client.ws, {
          type: 'error',
          payload: { message: `Unknown message type: ${message.type}` }
        });
    }
  }

  // Broadcast verification update
  broadcastVerificationUpdate(sessionId: string, update: any) {
    const topic = `verification:${sessionId}`;
    for (const client of this.clients.values()) {
      if (client.subscriptions.has(topic)) {
        this.send(client.ws, {
          type: 'verification-update',
          payload: update
        });
      }
    }
  }

  // Broadcast subject update
  broadcastSubjectUpdate(subjectId: string, update: any) {
    const topic = `subject:${subjectId}`;
    for (const client of this.clients.values()) {
      if (client.subscriptions.has(topic)) {
        this.send(client.ws, {
          type: 'subject-update',
          payload: update
        });
      }
    }
  }

  private subscribeToVerification(client: WSClient, sessionId: string) {
    client.subscriptions.add(`verification:${sessionId}`);
    this.send(client.ws, {
      type: 'subscribed',
      payload: { topic: `verification:${sessionId}` }
    });
  }

  private subscribeToSubject(client: WSClient, subjectId: string) {
    client.subscriptions.add(`subject:${subjectId}`);
    this.send(client.ws, {
      type: 'subscribed',
      payload: { topic: `subject:${subjectId}` }
    });
  }

  private send(ws: WebSocket, message: WSMessage) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }

  private handleDisconnect(client: WSClient) {
    this.clients.delete(client.id);
  }

  private extractToken(req: any): string | null {
    const url = new URL(req.url, `http://${req.headers.host}`);
    return url.searchParams.get('token');
  }

  private async authenticateToken(token: string | null): Promise<any> {
    if (!token) return null;
    // Token verification logic
    return {};
  }

  private generateClientId(): string {
    return `ws_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

interface WSClient {
  id: string;
  ws: WebSocket;
  user: any;
  subscriptions: Set<string>;
}
```

## 4.5 Summary

This chapter detailed the comprehensive API interfaces for the WIA Cryo Identity Standard:

| API Type | Endpoint | Features |
|----------|----------|----------|
| REST API | /api/v1/identity | CRUD operations, verification |
| GraphQL | /graphql | Flexible queries, subscriptions |
| WebSocket | /ws/identity | Real-time updates |

Key API features:
- Subject lifecycle management
- Multi-factor verification workflows
- Relationship management
- Privacy-aware data access
- Complete audit logging

---

© 2025 WIA Standards. All rights reserved.
