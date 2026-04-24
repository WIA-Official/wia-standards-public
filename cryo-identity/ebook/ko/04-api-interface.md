# 제4장: API 인터페이스

## 개요

WIA-CRYO-IDENTITY 표준은 다양한 클라이언트 요구사항을 지원하기 위해 REST API, GraphQL, 그리고 WebSocket 기반의 실시간 인터페이스를 제공합니다. 이 장에서는 각 API의 상세 구현과 사용 방법을 다룹니다.

## REST API

### 대상자 관리 API

```typescript
import express, { Router, Request, Response, NextFunction } from 'express';

// 라우터 설정
const subjectRouter = Router();

// 인증 미들웨어
const authMiddleware = async (req: Request, res: Response, next: NextFunction) => {
  const token = req.headers.authorization?.replace('Bearer ', '');
  if (!token) {
    return res.status(401).json({ error: '인증 토큰이 필요합니다' });
  }

  try {
    const decoded = await verifyToken(token);
    req.user = decoded;
    next();
  } catch (error) {
    return res.status(401).json({ error: '유효하지 않은 토큰입니다' });
  }
};

// 권한 검증 미들웨어
const requirePermission = (permission: string) => {
  return (req: Request, res: Response, next: NextFunction) => {
    if (!req.user?.permissions?.includes(permission)) {
      return res.status(403).json({ error: '권한이 없습니다' });
    }
    next();
  };
};

// 대상자 생성
subjectRouter.post(
  '/',
  authMiddleware,
  requirePermission('subjects:create'),
  async (req: Request, res: Response) => {
    try {
      const validationResult = SubjectSchema.safeParse(req.body);
      if (!validationResult.success) {
        return res.status(400).json({
          error: '유효성 검사 실패',
          details: validationResult.error.errors
        });
      }

      const subject = await subjectService.create(validationResult.data);

      await auditLogger.log({
        action: 'SUBJECT_CREATED',
        subjectId: subject.id,
        userId: req.user.id,
        details: { type: subject.type }
      });

      return res.status(201).json({
        success: true,
        data: subject
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 대상자 조회
subjectRouter.get(
  '/:id',
  authMiddleware,
  requirePermission('subjects:read'),
  async (req: Request, res: Response) => {
    try {
      const { id } = req.params;
      const { include } = req.query;

      const subject = await subjectService.findById(id, {
        include: include ? (include as string).split(',') : undefined
      });

      if (!subject) {
        return res.status(404).json({ error: '대상자를 찾을 수 없습니다' });
      }

      await auditLogger.log({
        action: 'SUBJECT_ACCESSED',
        subjectId: id,
        userId: req.user.id
      });

      return res.json({
        success: true,
        data: subject
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 대상자 업데이트
subjectRouter.patch(
  '/:id',
  authMiddleware,
  requirePermission('subjects:update'),
  async (req: Request, res: Response) => {
    try {
      const { id } = req.params;

      const existingSubject = await subjectService.findById(id);
      if (!existingSubject) {
        return res.status(404).json({ error: '대상자를 찾을 수 없습니다' });
      }

      const validationResult = SubjectSchema.partial().safeParse(req.body);
      if (!validationResult.success) {
        return res.status(400).json({
          error: '유효성 검사 실패',
          details: validationResult.error.errors
        });
      }

      const updated = await subjectService.update(id, validationResult.data);

      await auditLogger.log({
        action: 'SUBJECT_UPDATED',
        subjectId: id,
        userId: req.user.id,
        details: { fields: Object.keys(validationResult.data) }
      });

      return res.json({
        success: true,
        data: updated
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 대상자 삭제 (소프트 삭제)
subjectRouter.delete(
  '/:id',
  authMiddleware,
  requirePermission('subjects:delete'),
  async (req: Request, res: Response) => {
    try {
      const { id } = req.params;
      const { reason } = req.body;

      const existingSubject = await subjectService.findById(id);
      if (!existingSubject) {
        return res.status(404).json({ error: '대상자를 찾을 수 없습니다' });
      }

      await subjectService.softDelete(id, {
        deletedBy: req.user.id,
        reason
      });

      await auditLogger.log({
        action: 'SUBJECT_DELETED',
        subjectId: id,
        userId: req.user.id,
        details: { reason }
      });

      return res.json({
        success: true,
        message: '대상자가 삭제되었습니다'
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 대상자 목록 조회
subjectRouter.get(
  '/',
  authMiddleware,
  requirePermission('subjects:read'),
  async (req: Request, res: Response) => {
    try {
      const {
        page = '1',
        limit = '20',
        type,
        status,
        search,
        sortBy = 'createdAt',
        sortOrder = 'desc'
      } = req.query;

      const result = await subjectService.findAll({
        page: parseInt(page as string),
        limit: Math.min(parseInt(limit as string), 100),
        filters: {
          type: type as string,
          status: status as string,
          search: search as string
        },
        sort: {
          field: sortBy as string,
          order: sortOrder as 'asc' | 'desc'
        }
      });

      return res.json({
        success: true,
        data: result.subjects,
        pagination: {
          page: result.page,
          limit: result.limit,
          total: result.total,
          totalPages: result.totalPages
        }
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 대상자 병합
subjectRouter.post(
  '/:id/merge',
  authMiddleware,
  requirePermission('subjects:merge'),
  async (req: Request, res: Response) => {
    try {
      const { id } = req.params;
      const { sourceId, strategy } = req.body;

      if (!sourceId) {
        return res.status(400).json({ error: '원본 대상자 ID가 필요합니다' });
      }

      const merged = await subjectService.merge(id, sourceId, {
        strategy: strategy || 'target-priority',
        userId: req.user.id
      });

      await auditLogger.log({
        action: 'SUBJECTS_MERGED',
        subjectId: id,
        userId: req.user.id,
        details: { sourceId, strategy }
      });

      return res.json({
        success: true,
        data: merged
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 대상자 상태 변경
subjectRouter.post(
  '/:id/status',
  authMiddleware,
  requirePermission('subjects:update'),
  async (req: Request, res: Response) => {
    try {
      const { id } = req.params;
      const { status, reason } = req.body;

      const validStatuses = ['pending', 'active', 'suspended', 'inactive', 'deceased'];
      if (!validStatuses.includes(status)) {
        return res.status(400).json({ error: '유효하지 않은 상태입니다' });
      }

      const updated = await subjectService.updateStatus(id, status, {
        reason,
        userId: req.user.id
      });

      await auditLogger.log({
        action: 'SUBJECT_STATUS_CHANGED',
        subjectId: id,
        userId: req.user.id,
        details: { newStatus: status, reason }
      });

      return res.json({
        success: true,
        data: updated
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 오류 처리 헬퍼
function handleError(res: Response, error: unknown): Response {
  console.error('API Error:', error);

  if (error instanceof ValidationError) {
    return res.status(400).json({
      error: '유효성 검사 오류',
      details: error.details
    });
  }

  if (error instanceof NotFoundError) {
    return res.status(404).json({ error: error.message });
  }

  if (error instanceof ConflictError) {
    return res.status(409).json({ error: error.message });
  }

  return res.status(500).json({ error: '내부 서버 오류' });
}
```

### 인증 API

```typescript
const verificationRouter = Router();

// 인증 세션 생성
verificationRouter.post(
  '/sessions',
  authMiddleware,
  requirePermission('verification:create'),
  async (req: Request, res: Response) => {
    try {
      const { subjectId, context } = req.body;

      if (!subjectId) {
        return res.status(400).json({ error: '대상자 ID가 필요합니다' });
      }

      const session = await verificationService.createSession(subjectId, {
        purpose: context?.purpose || 'general',
        initiator: req.user.id,
        facility: req.user.facilityId,
        riskLevel: context?.riskLevel || 'medium'
      });

      return res.status(201).json({
        success: true,
        data: session
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 인증 세션 조회
verificationRouter.get(
  '/sessions/:sessionId',
  authMiddleware,
  requirePermission('verification:read'),
  async (req: Request, res: Response) => {
    try {
      const { sessionId } = req.params;

      const session = await verificationService.getSession(sessionId);
      if (!session) {
        return res.status(404).json({ error: '인증 세션을 찾을 수 없습니다' });
      }

      return res.json({
        success: true,
        data: session
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 인증 수행
verificationRouter.post(
  '/sessions/:sessionId/verify',
  authMiddleware,
  requirePermission('verification:execute'),
  async (req: Request, res: Response) => {
    try {
      const { sessionId } = req.params;
      const { method, data } = req.body;

      if (!method || !data) {
        return res.status(400).json({ error: '인증 방법과 데이터가 필요합니다' });
      }

      const result = await verificationService.executeVerification(
        sessionId,
        method,
        data
      );

      await auditLogger.log({
        action: 'VERIFICATION_EXECUTED',
        sessionId,
        userId: req.user.id,
        details: { method, result: result.status }
      });

      return res.json({
        success: true,
        data: result
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 문서 인증 제출
verificationRouter.post(
  '/sessions/:sessionId/document',
  authMiddleware,
  requirePermission('verification:execute'),
  upload.fields([
    { name: 'frontImage', maxCount: 1 },
    { name: 'backImage', maxCount: 1 }
  ]),
  async (req: Request, res: Response) => {
    try {
      const { sessionId } = req.params;
      const { documentType } = req.body;
      const files = req.files as { [fieldname: string]: Express.Multer.File[] };

      if (!documentType || !files.frontImage) {
        return res.status(400).json({
          error: '문서 유형과 전면 이미지가 필요합니다'
        });
      }

      const result = await verificationService.executeVerification(
        sessionId,
        'document',
        {
          documentType,
          frontImage: files.frontImage[0].buffer,
          backImage: files.backImage?.[0]?.buffer
        }
      );

      return res.json({
        success: true,
        data: result
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 생체정보 인증 제출
verificationRouter.post(
  '/sessions/:sessionId/biometric',
  authMiddleware,
  requirePermission('verification:execute'),
  async (req: Request, res: Response) => {
    try {
      const { sessionId } = req.params;
      const { modality, samples, livenessData } = req.body;

      if (!modality || !samples || samples.length === 0) {
        return res.status(400).json({
          error: '생체정보 양식과 샘플이 필요합니다'
        });
      }

      const result = await verificationService.executeVerification(
        sessionId,
        'biometric',
        {
          modality,
          samples: samples.map((s: any) => ({
            data: Buffer.from(s.data, 'base64'),
            capturedAt: new Date(s.capturedAt),
            deviceInfo: s.deviceInfo
          })),
          livenessData
        }
      );

      return res.json({
        success: true,
        data: result
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 인증 이력 조회
verificationRouter.get(
  '/subjects/:subjectId/history',
  authMiddleware,
  requirePermission('verification:read'),
  async (req: Request, res: Response) => {
    try {
      const { subjectId } = req.params;
      const { limit = '10', offset = '0' } = req.query;

      const history = await verificationService.getHistory(subjectId, {
        limit: parseInt(limit as string),
        offset: parseInt(offset as string)
      });

      return res.json({
        success: true,
        data: history
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);
```

### 관계 관리 API

```typescript
const relationshipRouter = Router();

// 관계 생성
relationshipRouter.post(
  '/',
  authMiddleware,
  requirePermission('relationships:create'),
  async (req: Request, res: Response) => {
    try {
      const validationResult = RelationshipSchema.safeParse(req.body);
      if (!validationResult.success) {
        return res.status(400).json({
          error: '유효성 검사 실패',
          details: validationResult.error.errors
        });
      }

      const relationship = await relationshipService.create(
        validationResult.data
      );

      await auditLogger.log({
        action: 'RELATIONSHIP_CREATED',
        subjectId: validationResult.data.subjectId,
        userId: req.user.id,
        details: {
          relatedSubjectId: validationResult.data.relatedSubjectId,
          type: validationResult.data.relationshipType
        }
      });

      return res.status(201).json({
        success: true,
        data: relationship
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 대상자의 관계 조회
relationshipRouter.get(
  '/subjects/:subjectId',
  authMiddleware,
  requirePermission('relationships:read'),
  async (req: Request, res: Response) => {
    try {
      const { subjectId } = req.params;
      const { type, includeInactive } = req.query;

      const relationships = await relationshipService.findBySubject(
        subjectId,
        {
          type: type as string,
          includeInactive: includeInactive === 'true'
        }
      );

      return res.json({
        success: true,
        data: relationships
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 관계 업데이트
relationshipRouter.patch(
  '/:id',
  authMiddleware,
  requirePermission('relationships:update'),
  async (req: Request, res: Response) => {
    try {
      const { id } = req.params;

      const existing = await relationshipService.findById(id);
      if (!existing) {
        return res.status(404).json({ error: '관계를 찾을 수 없습니다' });
      }

      const updated = await relationshipService.update(id, req.body);

      await auditLogger.log({
        action: 'RELATIONSHIP_UPDATED',
        relationshipId: id,
        userId: req.user.id
      });

      return res.json({
        success: true,
        data: updated
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);

// 관계 종료
relationshipRouter.post(
  '/:id/terminate',
  authMiddleware,
  requirePermission('relationships:update'),
  async (req: Request, res: Response) => {
    try {
      const { id } = req.params;
      const { reason, effectiveDate } = req.body;

      const terminated = await relationshipService.terminate(id, {
        reason,
        effectiveDate: effectiveDate ? new Date(effectiveDate) : new Date(),
        terminatedBy: req.user.id
      });

      await auditLogger.log({
        action: 'RELATIONSHIP_TERMINATED',
        relationshipId: id,
        userId: req.user.id,
        details: { reason }
      });

      return res.json({
        success: true,
        data: terminated
      });
    } catch (error) {
      return handleError(res, error);
    }
  }
);
```

## GraphQL API

### 스키마 정의

```typescript
import { gql } from 'apollo-server-express';

const typeDefs = gql`
  # 스칼라 타입
  scalar DateTime
  scalar JSON
  scalar Upload

  # 열거형
  enum SubjectType {
    individual
    minor
    incapacitated
    posthumous
  }

  enum SubjectStatus {
    pending
    active
    suspended
    inactive
    deceased
  }

  enum IdentifierType {
    internal
    national_id
    passport
    medical_record
    donor_id
    anonymous
  }

  enum VerificationLevel {
    basic
    standard
    enhanced
    maximum
  }

  enum VerificationMethod {
    document
    biometric
    knowledge
    possession
    third_party
  }

  enum VerificationStatus {
    pending
    in_progress
    completed
    failed
    expired
  }

  enum BiometricModality {
    fingerprint
    facial
    iris
    voice
    palm
  }

  enum RelationshipType {
    spouse
    parent
    child
    sibling
    guardian
    legal_representative
    emergency_contact
    next_of_kin
    authorized_agent
  }

  # 입력 타입
  input LegalNameInput {
    given: String!
    family: String!
    middle: String
    prefix: String
    suffix: String
    preferredName: String
    nativeScript: String
  }

  input IdentifierInput {
    type: IdentifierType!
    value: String!
    issuingAuthority: String
    issueDate: DateTime
    expirationDate: DateTime
  }

  input ContactInfoInput {
    phones: [PhoneInput!]
    emails: [EmailInput!]
    preferredLanguage: String
    preferredContactMethod: String
  }

  input PhoneInput {
    type: String!
    number: String!
    primary: Boolean
  }

  input EmailInput {
    type: String!
    address: String!
    primary: Boolean
  }

  input SubjectProfileInput {
    legalName: LegalNameInput!
    dateOfBirth: DateTime
    placeOfBirth: String
    nationality: String
    gender: String
    contactInfo: ContactInfoInput
  }

  input CreateSubjectInput {
    type: SubjectType!
    identifiers: [IdentifierInput!]!
    profile: SubjectProfileInput!
    facilityId: ID!
  }

  input UpdateSubjectInput {
    profile: SubjectProfileInput
    status: SubjectStatus
  }

  input VerificationContextInput {
    purpose: String!
    riskLevel: String
    specimenIds: [ID!]
    operationType: String
  }

  input BiometricSampleInput {
    data: String!  # Base64 encoded
    capturedAt: DateTime!
    deviceId: String
  }

  input SubjectFilterInput {
    type: SubjectType
    status: SubjectStatus
    search: String
    facilityId: ID
  }

  input PaginationInput {
    page: Int
    limit: Int
    sortBy: String
    sortOrder: String
  }

  # 객체 타입
  type LegalName {
    given: String!
    family: String!
    middle: String
    prefix: String
    suffix: String
    preferredName: String
    nativeScript: String
    fullName: String!
  }

  type Identifier {
    id: ID!
    type: IdentifierType!
    value: String!
    issuingAuthority: String
    issueDate: DateTime
    expirationDate: DateTime
    verified: Boolean!
    verifiedAt: DateTime
  }

  type ContactInfo {
    phones: [Phone!]
    emails: [Email!]
    preferredLanguage: String
    preferredContactMethod: String
  }

  type Phone {
    type: String!
    number: String!
    primary: Boolean!
  }

  type Email {
    type: String!
    address: String!
    verified: Boolean!
    primary: Boolean!
  }

  type SubjectProfile {
    legalName: LegalName!
    dateOfBirth: DateTime
    placeOfBirth: String
    nationality: String
    gender: String
    contactInfo: ContactInfo
  }

  type Subject {
    id: ID!
    externalId: String
    type: SubjectType!
    status: SubjectStatus!
    identifiers: [Identifier!]!
    profile: SubjectProfile!
    relationships: [Relationship!]
    specimens: [Specimen!]
    verificationHistory: [VerificationResult!]
    currentVerification: VerificationResult
    createdAt: DateTime!
    updatedAt: DateTime!
    facilityId: ID!
  }

  type Relationship {
    id: ID!
    subject: Subject!
    relatedSubject: Subject!
    relationshipType: RelationshipType!
    isPrimary: Boolean!
    legalAuthority: String
    validFrom: DateTime!
    validUntil: DateTime
    verified: Boolean!
  }

  type Specimen {
    id: ID!
    type: String!
    status: String!
    collectionDate: DateTime!
    location: String
  }

  type VerificationSession {
    id: ID!
    subjectId: ID!
    subject: Subject!
    requestedLevel: VerificationLevel!
    requiredMethods: [VerificationMethod!]!
    completedMethods: [VerificationMethod!]!
    status: VerificationStatus!
    context: JSON!
    results: [VerificationResult!]
    createdAt: DateTime!
    expiresAt: DateTime!
    completedAt: DateTime
  }

  type VerificationResult {
    id: ID!
    sessionId: ID!
    method: VerificationMethod!
    status: VerificationStatus!
    confidence: Float!
    timestamp: DateTime!
    expiresAt: DateTime
    metadata: JSON
  }

  type BiometricTemplate {
    id: ID!
    subjectId: ID!
    modality: BiometricModality!
    quality: Float!
    isActive: Boolean!
    capturedAt: DateTime!
    expiresAt: DateTime
  }

  # 페이지네이션
  type SubjectConnection {
    edges: [SubjectEdge!]!
    pageInfo: PageInfo!
    totalCount: Int!
  }

  type SubjectEdge {
    node: Subject!
    cursor: String!
  }

  type PageInfo {
    hasNextPage: Boolean!
    hasPreviousPage: Boolean!
    startCursor: String
    endCursor: String
  }

  # 뮤테이션 결과
  type MutationResult {
    success: Boolean!
    message: String
    errors: [String!]
  }

  type SubjectMutationResult {
    success: Boolean!
    subject: Subject
    errors: [String!]
  }

  type VerificationSessionResult {
    success: Boolean!
    session: VerificationSession
    errors: [String!]
  }

  type VerificationExecutionResult {
    success: Boolean!
    result: VerificationResult
    sessionCompleted: Boolean!
    errors: [String!]
  }

  # 쿼리
  type Query {
    # 대상자 쿼리
    subject(id: ID!): Subject
    subjectByIdentifier(type: IdentifierType!, value: String!): Subject
    subjects(
      filter: SubjectFilterInput
      pagination: PaginationInput
    ): SubjectConnection!

    # 인증 쿼리
    verificationSession(id: ID!): VerificationSession
    verificationHistory(subjectId: ID!, limit: Int): [VerificationResult!]!

    # 관계 쿼리
    relationships(subjectId: ID!): [Relationship!]!
    relationship(id: ID!): Relationship

    # 생체정보 쿼리
    biometricTemplates(subjectId: ID!): [BiometricTemplate!]!

    # 시스템 쿼리
    systemHealth: HealthStatus!
  }

  # 뮤테이션
  type Mutation {
    # 대상자 뮤테이션
    createSubject(input: CreateSubjectInput!): SubjectMutationResult!
    updateSubject(id: ID!, input: UpdateSubjectInput!): SubjectMutationResult!
    deleteSubject(id: ID!, reason: String): MutationResult!
    mergeSubjects(targetId: ID!, sourceId: ID!): SubjectMutationResult!

    # 인증 뮤테이션
    createVerificationSession(
      subjectId: ID!
      context: VerificationContextInput!
    ): VerificationSessionResult!

    executeDocumentVerification(
      sessionId: ID!
      documentType: String!
      frontImage: Upload!
      backImage: Upload
    ): VerificationExecutionResult!

    executeBiometricVerification(
      sessionId: ID!
      modality: BiometricModality!
      samples: [BiometricSampleInput!]!
    ): VerificationExecutionResult!

    cancelVerificationSession(
      sessionId: ID!
      reason: String
    ): MutationResult!

    # 관계 뮤테이션
    createRelationship(
      subjectId: ID!
      relatedSubjectId: ID!
      type: RelationshipType!
      legalAuthority: String
    ): Relationship!

    terminateRelationship(
      id: ID!
      reason: String
    ): MutationResult!

    # 생체정보 뮤테이션
    enrollBiometric(
      subjectId: ID!
      modality: BiometricModality!
      samples: [BiometricSampleInput!]!
    ): BiometricTemplate!

    deactivateBiometric(id: ID!): MutationResult!
  }

  # 서브스크립션
  type Subscription {
    # 인증 이벤트
    verificationStatusChanged(sessionId: ID!): VerificationSession!
    verificationCompleted(subjectId: ID!): VerificationResult!

    # 대상자 이벤트
    subjectStatusChanged(facilityId: ID!): Subject!
  }

  type HealthStatus {
    status: String!
    version: String!
    uptime: Int!
    components: [ComponentHealth!]!
  }

  type ComponentHealth {
    name: String!
    status: String!
    latency: Int
  }
`;
```

### 리졸버 구현

```typescript
import { GraphQLUpload } from 'graphql-upload';
import { withFilter } from 'graphql-subscriptions';

const resolvers = {
  Upload: GraphQLUpload,

  DateTime: new GraphQLScalarType({
    name: 'DateTime',
    description: 'ISO 8601 DateTime',
    serialize(value: Date) {
      return value.toISOString();
    },
    parseValue(value: string) {
      return new Date(value);
    },
    parseLiteral(ast) {
      if (ast.kind === Kind.STRING) {
        return new Date(ast.value);
      }
      return null;
    }
  }),

  Query: {
    subject: async (_: any, { id }: { id: string }, context: Context) => {
      context.requirePermission('subjects:read');
      return subjectService.findById(id);
    },

    subjectByIdentifier: async (
      _: any,
      { type, value }: { type: string; value: string },
      context: Context
    ) => {
      context.requirePermission('subjects:read');
      return subjectService.findByIdentifier(type, value);
    },

    subjects: async (
      _: any,
      { filter, pagination }: { filter?: any; pagination?: any },
      context: Context
    ) => {
      context.requirePermission('subjects:read');

      const result = await subjectService.findAll({
        page: pagination?.page || 1,
        limit: Math.min(pagination?.limit || 20, 100),
        filters: filter,
        sort: {
          field: pagination?.sortBy || 'createdAt',
          order: pagination?.sortOrder || 'desc'
        }
      });

      return {
        edges: result.subjects.map((subject: any, index: number) => ({
          node: subject,
          cursor: Buffer.from(`${result.page}:${index}`).toString('base64')
        })),
        pageInfo: {
          hasNextPage: result.page < result.totalPages,
          hasPreviousPage: result.page > 1,
          startCursor: result.subjects.length > 0
            ? Buffer.from(`${result.page}:0`).toString('base64')
            : null,
          endCursor: result.subjects.length > 0
            ? Buffer.from(`${result.page}:${result.subjects.length - 1}`).toString('base64')
            : null
        },
        totalCount: result.total
      };
    },

    verificationSession: async (
      _: any,
      { id }: { id: string },
      context: Context
    ) => {
      context.requirePermission('verification:read');
      return verificationService.getSession(id);
    },

    verificationHistory: async (
      _: any,
      { subjectId, limit }: { subjectId: string; limit?: number },
      context: Context
    ) => {
      context.requirePermission('verification:read');
      return verificationService.getHistory(subjectId, { limit: limit || 10 });
    },

    relationships: async (
      _: any,
      { subjectId }: { subjectId: string },
      context: Context
    ) => {
      context.requirePermission('relationships:read');
      return relationshipService.findBySubject(subjectId);
    },

    biometricTemplates: async (
      _: any,
      { subjectId }: { subjectId: string },
      context: Context
    ) => {
      context.requirePermission('biometrics:read');
      return biometricService.getTemplates(subjectId);
    },

    systemHealth: async () => {
      return healthService.getStatus();
    }
  },

  Mutation: {
    createSubject: async (
      _: any,
      { input }: { input: any },
      context: Context
    ) => {
      context.requirePermission('subjects:create');

      try {
        const subject = await subjectService.create({
          ...input,
          createdBy: context.user.id
        });

        return {
          success: true,
          subject
        };
      } catch (error) {
        return {
          success: false,
          errors: [error instanceof Error ? error.message : '알 수 없는 오류']
        };
      }
    },

    updateSubject: async (
      _: any,
      { id, input }: { id: string; input: any },
      context: Context
    ) => {
      context.requirePermission('subjects:update');

      try {
        const subject = await subjectService.update(id, input);
        return { success: true, subject };
      } catch (error) {
        return {
          success: false,
          errors: [error instanceof Error ? error.message : '알 수 없는 오류']
        };
      }
    },

    createVerificationSession: async (
      _: any,
      { subjectId, context: verificationContext }: { subjectId: string; context: any },
      context: Context
    ) => {
      context.requirePermission('verification:create');

      try {
        const session = await verificationService.createSession(subjectId, {
          ...verificationContext,
          initiator: context.user.id,
          facility: context.user.facilityId
        });

        return { success: true, session };
      } catch (error) {
        return {
          success: false,
          errors: [error instanceof Error ? error.message : '알 수 없는 오류']
        };
      }
    },

    executeDocumentVerification: async (
      _: any,
      { sessionId, documentType, frontImage, backImage }: any,
      context: Context
    ) => {
      context.requirePermission('verification:execute');

      try {
        const front = await frontImage;
        const back = backImage ? await backImage : null;

        const frontBuffer = await streamToBuffer(front.createReadStream());
        const backBuffer = back ? await streamToBuffer(back.createReadStream()) : undefined;

        const result = await verificationService.executeVerification(
          sessionId,
          'document',
          { documentType, frontImage: frontBuffer, backImage: backBuffer }
        );

        const session = await verificationService.getSession(sessionId);

        return {
          success: true,
          result,
          sessionCompleted: session?.status === 'completed'
        };
      } catch (error) {
        return {
          success: false,
          errors: [error instanceof Error ? error.message : '알 수 없는 오류']
        };
      }
    },

    executeBiometricVerification: async (
      _: any,
      { sessionId, modality, samples }: any,
      context: Context
    ) => {
      context.requirePermission('verification:execute');

      try {
        const result = await verificationService.executeVerification(
          sessionId,
          'biometric',
          {
            modality,
            samples: samples.map((s: any) => ({
              data: Buffer.from(s.data, 'base64'),
              capturedAt: new Date(s.capturedAt),
              deviceInfo: { type: 'unknown', manufacturer: 'unknown', model: 'unknown', certifications: [] }
            }))
          }
        );

        const session = await verificationService.getSession(sessionId);

        return {
          success: true,
          result,
          sessionCompleted: session?.status === 'completed'
        };
      } catch (error) {
        return {
          success: false,
          errors: [error instanceof Error ? error.message : '알 수 없는 오류']
        };
      }
    }
  },

  Subscription: {
    verificationStatusChanged: {
      subscribe: withFilter(
        () => pubsub.asyncIterator(['VERIFICATION_STATUS_CHANGED']),
        (payload, variables) => {
          return payload.verificationStatusChanged.id === variables.sessionId;
        }
      )
    },

    verificationCompleted: {
      subscribe: withFilter(
        () => pubsub.asyncIterator(['VERIFICATION_COMPLETED']),
        (payload, variables) => {
          return payload.verificationCompleted.subjectId === variables.subjectId;
        }
      )
    }
  },

  // 타입 리졸버
  Subject: {
    relationships: async (subject: any, _: any, context: Context) => {
      return relationshipService.findBySubject(subject.id);
    },

    specimens: async (subject: any, _: any, context: Context) => {
      return specimenService.findBySubject(subject.id);
    },

    verificationHistory: async (subject: any, _: any, context: Context) => {
      return verificationService.getHistory(subject.id, { limit: 10 });
    },

    currentVerification: async (subject: any, _: any, context: Context) => {
      const history = await verificationService.getHistory(subject.id, { limit: 1 });
      return history[0] || null;
    }
  },

  LegalName: {
    fullName: (name: any) => {
      const parts = [name.prefix, name.given, name.middle, name.family, name.suffix]
        .filter(Boolean);
      return parts.join(' ');
    }
  },

  Relationship: {
    subject: async (relationship: any) => {
      return subjectService.findById(relationship.subjectId);
    },

    relatedSubject: async (relationship: any) => {
      return subjectService.findById(relationship.relatedSubjectId);
    }
  }
};
```

## WebSocket API

### 실시간 인증 업데이트

```typescript
import { Server as WebSocketServer } from 'ws';
import { createServer } from 'http';

class VerificationWebSocketServer {
  private wss: WebSocketServer;
  private clients: Map<string, WebSocket[]> = new Map();

  constructor(server: ReturnType<typeof createServer>) {
    this.wss = new WebSocketServer({ server, path: '/ws/verification' });
    this.initialize();
  }

  private initialize(): void {
    this.wss.on('connection', (ws, req) => {
      const token = this.extractToken(req);
      if (!token) {
        ws.close(4001, '인증 필요');
        return;
      }

      this.handleConnection(ws, token);
    });
  }

  private async handleConnection(ws: WebSocket, token: string): Promise<void> {
    try {
      const user = await this.verifyToken(token);

      ws.on('message', async (message: string) => {
        try {
          const data = JSON.parse(message);
          await this.handleMessage(ws, user, data);
        } catch (error) {
          this.sendError(ws, '잘못된 메시지 형식');
        }
      });

      ws.on('close', () => {
        this.removeClient(ws);
      });

      this.sendMessage(ws, {
        type: 'connected',
        userId: user.id,
        timestamp: new Date().toISOString()
      });

    } catch (error) {
      ws.close(4003, '인증 실패');
    }
  }

  private async handleMessage(
    ws: WebSocket,
    user: User,
    data: WebSocketMessage
  ): Promise<void> {
    switch (data.type) {
      case 'subscribe_session':
        await this.subscribeToSession(ws, data.sessionId);
        break;

      case 'unsubscribe_session':
        await this.unsubscribeFromSession(ws, data.sessionId);
        break;

      case 'subscribe_subject':
        await this.subscribeToSubject(ws, data.subjectId);
        break;

      case 'ping':
        this.sendMessage(ws, { type: 'pong', timestamp: new Date().toISOString() });
        break;

      default:
        this.sendError(ws, `알 수 없는 메시지 유형: ${data.type}`);
    }
  }

  private async subscribeToSession(ws: WebSocket, sessionId: string): Promise<void> {
    const key = `session:${sessionId}`;
    if (!this.clients.has(key)) {
      this.clients.set(key, []);
    }
    this.clients.get(key)!.push(ws);

    this.sendMessage(ws, {
      type: 'subscribed',
      channel: key,
      timestamp: new Date().toISOString()
    });
  }

  private async unsubscribeFromSession(ws: WebSocket, sessionId: string): Promise<void> {
    const key = `session:${sessionId}`;
    const clients = this.clients.get(key);
    if (clients) {
      const index = clients.indexOf(ws);
      if (index !== -1) {
        clients.splice(index, 1);
      }
    }

    this.sendMessage(ws, {
      type: 'unsubscribed',
      channel: key,
      timestamp: new Date().toISOString()
    });
  }

  private async subscribeToSubject(ws: WebSocket, subjectId: string): Promise<void> {
    const key = `subject:${subjectId}`;
    if (!this.clients.has(key)) {
      this.clients.set(key, []);
    }
    this.clients.get(key)!.push(ws);

    this.sendMessage(ws, {
      type: 'subscribed',
      channel: key,
      timestamp: new Date().toISOString()
    });
  }

  // 외부에서 호출하여 클라이언트에게 브로드캐스트
  broadcastSessionUpdate(sessionId: string, data: any): void {
    const key = `session:${sessionId}`;
    const clients = this.clients.get(key) || [];

    for (const client of clients) {
      if (client.readyState === WebSocket.OPEN) {
        this.sendMessage(client, {
          type: 'session_update',
          sessionId,
          data,
          timestamp: new Date().toISOString()
        });
      }
    }
  }

  broadcastVerificationResult(subjectId: string, result: any): void {
    const key = `subject:${subjectId}`;
    const clients = this.clients.get(key) || [];

    for (const client of clients) {
      if (client.readyState === WebSocket.OPEN) {
        this.sendMessage(client, {
          type: 'verification_result',
          subjectId,
          result,
          timestamp: new Date().toISOString()
        });
      }
    }
  }

  private removeClient(ws: WebSocket): void {
    for (const [key, clients] of this.clients.entries()) {
      const index = clients.indexOf(ws);
      if (index !== -1) {
        clients.splice(index, 1);
      }
      if (clients.length === 0) {
        this.clients.delete(key);
      }
    }
  }

  private sendMessage(ws: WebSocket, message: any): void {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }

  private sendError(ws: WebSocket, error: string): void {
    this.sendMessage(ws, {
      type: 'error',
      error,
      timestamp: new Date().toISOString()
    });
  }

  private extractToken(req: any): string | null {
    const url = new URL(req.url, 'http://localhost');
    return url.searchParams.get('token');
  }

  private async verifyToken(token: string): Promise<User> {
    // 토큰 검증 로직
    return {} as User;
  }
}

interface WebSocketMessage {
  type: string;
  sessionId?: string;
  subjectId?: string;
  [key: string]: any;
}

interface User {
  id: string;
  facilityId: string;
  permissions: string[];
}
```

## 요약

이 장에서 다룬 내용:

1. **REST API**: 대상자, 인증, 관계 관리를 위한 완전한 RESTful 엔드포인트
2. **GraphQL API**: 유연한 쿼리와 뮤테이션을 위한 스키마 및 리졸버
3. **WebSocket API**: 실시간 인증 상태 업데이트

다음 장에서는 상세 인증 프로토콜 구현을 다룹니다.
