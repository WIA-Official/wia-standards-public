# API 인터페이스

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## REST API 설계

### API 기본 구조

```typescript
/**
 * WIA 냉동보존 REST API v1.0
 *
 * Base URL: https://api.wia-cryo.org/v1
 * 인증: Bearer Token (JWT)
 * 데이터 형식: JSON
 */

import { Router, Request, Response, NextFunction } from 'express';
import { CompleteSpecimen, CompleteFreezingProtocol } from './schemas';

/**
 * API 응답 기본 형식
 */
export interface ApiResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    messageKr: string;
    details?: any;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
    version: string;
  };
}

/**
 * 페이지네이션 응답
 */
export interface PaginatedResponse<T> {
  success: boolean;
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalItems: number;
    hasNext: boolean;
    hasPrevious: boolean;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
  };
}

/**
 * API 에러 코드
 */
export enum ApiErrorCode {
  UNAUTHORIZED = 'UNAUTHORIZED',
  FORBIDDEN = 'FORBIDDEN',
  NOT_FOUND = 'NOT_FOUND',
  VALIDATION_ERROR = 'VALIDATION_ERROR',
  CONFLICT = 'CONFLICT',
  INTERNAL_ERROR = 'INTERNAL_ERROR',
  SPECIMEN_NOT_FOUND = 'SPECIMEN_NOT_FOUND',
  PROTOCOL_NOT_FOUND = 'PROTOCOL_NOT_FOUND',
  STORAGE_FULL = 'STORAGE_FULL',
  INVALID_STATUS_TRANSITION = 'INVALID_STATUS_TRANSITION',
}

/**
 * 에러 메시지 맵 (한영)
 */
export const ErrorMessages: Record<ApiErrorCode, { en: string; kr: string }> = {
  [ApiErrorCode.UNAUTHORIZED]: {
    en: 'Authentication required',
    kr: '인증이 필요합니다',
  },
  [ApiErrorCode.FORBIDDEN]: {
    en: 'Access denied',
    kr: '접근 권한이 없습니다',
  },
  [ApiErrorCode.NOT_FOUND]: {
    en: 'Resource not found',
    kr: '리소스를 찾을 수 없습니다',
  },
  [ApiErrorCode.VALIDATION_ERROR]: {
    en: 'Validation failed',
    kr: '유효성 검사 실패',
  },
  [ApiErrorCode.CONFLICT]: {
    en: 'Resource conflict',
    kr: '리소스 충돌',
  },
  [ApiErrorCode.INTERNAL_ERROR]: {
    en: 'Internal server error',
    kr: '서버 내부 오류',
  },
  [ApiErrorCode.SPECIMEN_NOT_FOUND]: {
    en: 'Specimen not found',
    kr: '검체를 찾을 수 없습니다',
  },
  [ApiErrorCode.PROTOCOL_NOT_FOUND]: {
    en: 'Protocol not found',
    kr: '프로토콜을 찾을 수 없습니다',
  },
  [ApiErrorCode.STORAGE_FULL]: {
    en: 'Storage capacity full',
    kr: '저장 공간이 가득 찼습니다',
  },
  [ApiErrorCode.INVALID_STATUS_TRANSITION]: {
    en: 'Invalid status transition',
    kr: '잘못된 상태 전환',
  },
};
```

### 검체 관리 API

```typescript
/**
 * 검체 API 라우터
 */
export class SpecimenApiRouter {
  private router: Router;

  constructor() {
    this.router = Router();
    this.setupRoutes();
  }

  private setupRoutes(): void {
    // 검체 생성
    this.router.post('/specimens', this.createSpecimen);

    // 검체 조회
    this.router.get('/specimens/:id', this.getSpecimen);

    // 검체 목록 조회
    this.router.get('/specimens', this.listSpecimens);

    // 검체 수정
    this.router.patch('/specimens/:id', this.updateSpecimen);

    // 검체 상태 변경
    this.router.post('/specimens/:id/status', this.updateStatus);

    // 검체 삭제 (소프트 삭제)
    this.router.delete('/specimens/:id', this.deleteSpecimen);

    // 검체 이력 조회
    this.router.get('/specimens/:id/history', this.getHistory);

    // 검체 위치 조회
    this.router.get('/specimens/:id/location', this.getLocation);

    // 검체 위치 변경
    this.router.post('/specimens/:id/location', this.updateLocation);
  }

  /**
   * POST /api/v1/specimens
   * 새로운 검체 등록
   */
  private async createSpecimen(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const specimenData = req.body;

      // 데이터 검증
      const validation = CompleteSpecimenSchema.safeParse(specimenData);
      if (!validation.success) {
        res.status(400).json({
          success: false,
          error: {
            code: ApiErrorCode.VALIDATION_ERROR,
            message: 'Invalid specimen data',
            messageKr: '검체 데이터가 유효하지 않습니다',
            details: validation.error.errors,
          },
        } as ApiResponse);
        return;
      }

      // 검체 등록
      const specimen = await this.specimenService.create(validation.data);

      // 이벤트 로깅
      await this.auditService.log({
        eventType: 'SPECIMEN_CREATED',
        userId: req.user.id,
        resourceId: specimen.specimenId,
        action: 'Created new specimen',
        actionKr: '새로운 검체 등록',
      });

      res.status(201).json({
        success: true,
        data: specimen,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.id,
          version: 'v1',
        },
      } as ApiResponse<CompleteSpecimen>);
    } catch (error) {
      next(error);
    }
  }

  /**
   * GET /api/v1/specimens/:id
   * 검체 상세 조회
   */
  private async getSpecimen(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const { id } = req.params;

      const specimen = await this.specimenService.findById(id);

      if (!specimen) {
        res.status(404).json({
          success: false,
          error: {
            code: ApiErrorCode.SPECIMEN_NOT_FOUND,
            message: 'Specimen not found',
            messageKr: '검체를 찾을 수 없습니다',
          },
        } as ApiResponse);
        return;
      }

      // 접근 권한 확인
      if (!this.hasAccessPermission(req.user, specimen)) {
        res.status(403).json({
          success: false,
          error: {
            code: ApiErrorCode.FORBIDDEN,
            message: 'Access denied',
            messageKr: '접근 권한이 없습니다',
          },
        } as ApiResponse);
        return;
      }

      res.json({
        success: true,
        data: specimen,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.id,
          version: 'v1',
        },
      } as ApiResponse<CompleteSpecimen>);
    } catch (error) {
      next(error);
    }
  }

  /**
   * GET /api/v1/specimens
   * 검체 목록 조회 (페이지네이션, 필터링)
   *
   * Query Parameters:
   * - page: 페이지 번호 (기본값: 1)
   * - pageSize: 페이지 크기 (기본값: 20, 최대: 100)
   * - type: 검체 유형 필터
   * - status: 상태 필터
   * - patientId: 환자 ID 필터
   * - fromDate: 시작일 필터
   * - toDate: 종료일 필터
   * - sortBy: 정렬 필드 (기본값: createdAt)
   * - sortOrder: 정렬 순서 (asc, desc)
   */
  private async listSpecimens(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const {
        page = 1,
        pageSize = 20,
        type,
        status,
        patientId,
        fromDate,
        toDate,
        sortBy = 'createdAt',
        sortOrder = 'desc',
      } = req.query;

      // 페이지네이션 검증
      const validatedPage = Math.max(1, Number(page));
      const validatedPageSize = Math.min(100, Math.max(1, Number(pageSize)));

      // 필터 구성
      const filters: any = {};
      if (type) filters.type = type;
      if (status) filters.status = status;
      if (patientId) filters.patientId = patientId;
      if (fromDate || toDate) {
        filters.dateRange = {
          from: fromDate ? new Date(fromDate as string) : undefined,
          to: toDate ? new Date(toDate as string) : undefined,
        };
      }

      // 검체 조회
      const result = await this.specimenService.findAll({
        page: validatedPage,
        pageSize: validatedPageSize,
        filters,
        sortBy: sortBy as string,
        sortOrder: sortOrder as 'asc' | 'desc',
        userId: req.user.id, // 접근 권한 필터링
      });

      res.json({
        success: true,
        data: result.items,
        pagination: {
          page: validatedPage,
          pageSize: validatedPageSize,
          totalPages: result.totalPages,
          totalItems: result.totalItems,
          hasNext: validatedPage < result.totalPages,
          hasPrevious: validatedPage > 1,
        },
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.id,
        },
      } as PaginatedResponse<CompleteSpecimen>);
    } catch (error) {
      next(error);
    }
  }

  /**
   * PATCH /api/v1/specimens/:id
   * 검체 정보 수정
   */
  private async updateSpecimen(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const { id } = req.params;
      const updates = req.body;

      const specimen = await this.specimenService.findById(id);
      if (!specimen) {
        res.status(404).json({
          success: false,
          error: {
            code: ApiErrorCode.SPECIMEN_NOT_FOUND,
            message: 'Specimen not found',
            messageKr: '검체를 찾을 수 없습니다',
          },
        } as ApiResponse);
        return;
      }

      // 권한 확인
      if (!this.hasUpdatePermission(req.user, specimen)) {
        res.status(403).json({
          success: false,
          error: {
            code: ApiErrorCode.FORBIDDEN,
            message: 'Update permission denied',
            messageKr: '수정 권한이 없습니다',
          },
        } as ApiResponse);
        return;
      }

      // 업데이트
      const updated = await this.specimenService.update(id, updates);

      // 감사 로그
      await this.auditService.log({
        eventType: 'SPECIMEN_UPDATED',
        userId: req.user.id,
        resourceId: id,
        action: 'Updated specimen',
        actionKr: '검체 정보 수정',
        changes: { before: specimen, after: updated },
      });

      res.json({
        success: true,
        data: updated,
        metadata: {
          timestamp: new Date().toISOString(),
          requestId: req.id,
          version: 'v1',
        },
      } as ApiResponse<CompleteSpecimen>);
    } catch (error) {
      next(error);
    }
  }

  /**
   * POST /api/v1/specimens/:id/status
   * 검체 상태 변경
   */
  private async updateStatus(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const { id } = req.params;
      const { status, reason } = req.body;

      const specimen = await this.specimenService.findById(id);
      if (!specimen) {
        res.status(404).json({
          success: false,
          error: {
            code: ApiErrorCode.SPECIMEN_NOT_FOUND,
            message: 'Specimen not found',
            messageKr: '검체를 찾을 수 없습니다',
          },
        } as ApiResponse);
        return;
      }

      // 상태 전환 유효성 검증
      const isValidTransition = this.validateStatusTransition(
        specimen.status,
        status
      );

      if (!isValidTransition) {
        res.status(400).json({
          success: false,
          error: {
            code: ApiErrorCode.INVALID_STATUS_TRANSITION,
            message: `Cannot transition from ${specimen.status} to ${status}`,
            messageKr: `${specimen.statusKr}에서 ${status}(으)로 전환할 수 없습니다`,
          },
        } as ApiResponse);
        return;
      }

      // 상태 업데이트
      const updated = await this.specimenService.updateStatus(id, status, {
        changedBy: req.user.id,
        reason,
      });

      res.json({
        success: true,
        data: updated,
      } as ApiResponse<CompleteSpecimen>);
    } catch (error) {
      next(error);
    }
  }

  /**
   * GET /api/v1/specimens/:id/history
   * 검체 이력 조회
   */
  private async getHistory(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const { id } = req.params;

      const history = await this.specimenService.getHistory(id);

      res.json({
        success: true,
        data: history,
      } as ApiResponse);
    } catch (error) {
      next(error);
    }
  }

  // Helper methods
  private hasAccessPermission(user: any, specimen: CompleteSpecimen): boolean {
    // 권한 로직 구현
    return true;
  }

  private hasUpdatePermission(user: any, specimen: CompleteSpecimen): boolean {
    // 수정 권한 로직 구현
    return true;
  }

  private validateStatusTransition(from: string, to: string): boolean {
    // 상태 전환 검증 로직
    const validTransitions: Record<string, string[]> = {
      REGISTERED: ['PREPARED', 'DISPOSED'],
      PREPARED: ['FREEZING', 'DISPOSED'],
      FREEZING: ['FROZEN', 'QUARANTINE'],
      FROZEN: ['THAWING', 'DISPOSED'],
      THAWING: ['THAWED', 'QUARANTINE'],
      THAWED: ['TRANSFERRED', 'DISPOSED'],
      QUARANTINE: ['DISPOSED'],
    };

    return validTransitions[from]?.includes(to) ?? false;
  }

  // Service instances (의존성 주입 필요)
  private specimenService: any;
  private auditService: any;

  getRouter(): Router {
    return this.router;
  }
}
```

### 프로토콜 관리 API

```typescript
/**
 * 프로토콜 API 라우터
 */
export class ProtocolApiRouter {
  private router: Router;

  constructor() {
    this.router = Router();
    this.setupRoutes();
  }

  private setupRoutes(): void {
    // 동결 프로토콜
    this.router.get('/protocols/freezing', this.listFreezingProtocols);
    this.router.get('/protocols/freezing/:id', this.getFreezingProtocol);
    this.router.post('/protocols/freezing', this.createFreezingProtocol);
    this.router.patch('/protocols/freezing/:id', this.updateFreezingProtocol);

    // 해동 프로토콜
    this.router.get('/protocols/thawing', this.listThawingProtocols);
    this.router.get('/protocols/thawing/:id', this.getThawingProtocol);
    this.router.post('/protocols/thawing', this.createThawingProtocol);

    // 프로토콜 실행
    this.router.post('/protocols/freezing/:id/execute', this.executeFreezingProtocol);
    this.router.post('/protocols/thawing/:id/execute', this.executeThawingProtocol);
  }

  /**
   * GET /api/v1/protocols/freezing
   * 동결 프로토콜 목록 조회
   */
  private async listFreezingProtocols(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const { specimenType, method, status = 'active' } = req.query;

      const protocols = await this.protocolService.findFreezingProtocols({
        specimenType: specimenType as string,
        method: method as string,
        status: status as string,
      });

      res.json({
        success: true,
        data: protocols,
      } as ApiResponse);
    } catch (error) {
      next(error);
    }
  }

  /**
   * POST /api/v1/protocols/freezing/:id/execute
   * 동결 프로토콜 실행
   */
  private async executeFreezingProtocol(
    req: Request,
    res: Response,
    next: NextFunction
  ): Promise<void> {
    try {
      const { id } = req.params;
      const { specimenId } = req.body;

      // 프로토콜 조회
      const protocol = await this.protocolService.findFreezingProtocolById(id);
      if (!protocol) {
        res.status(404).json({
          success: false,
          error: {
            code: ApiErrorCode.PROTOCOL_NOT_FOUND,
            message: 'Protocol not found',
            messageKr: '프로토콜을 찾을 수 없습니다',
          },
        } as ApiResponse);
        return;
      }

      // 검체 조회
      const specimen = await this.specimenService.findById(specimenId);
      if (!specimen) {
        res.status(404).json({
          success: false,
          error: {
            code: ApiErrorCode.SPECIMEN_NOT_FOUND,
            message: 'Specimen not found',
            messageKr: '검체를 찾을 수 없습니다',
          },
        } as ApiResponse);
        return;
      }

      // 프로토콜 실행
      const execution = await this.protocolService.executeFreezingProtocol(
        protocol,
        specimen,
        {
          performedBy: req.user.id,
          startedAt: new Date().toISOString(),
        }
      );

      res.json({
        success: true,
        data: execution,
      } as ApiResponse);
    } catch (error) {
      next(error);
    }
  }

  private protocolService: any;
  private specimenService: any;

  getRouter(): Router {
    return this.router;
  }
}
```

## GraphQL API

### GraphQL 스키마

```typescript
/**
 * GraphQL 스키마 정의
 */
import { gql } from 'apollo-server-express';

export const typeDefs = gql`
  """
  검체 유형
  """
  enum SpecimenType {
    SPERM          # 정자
    OOCYTE         # 난자
    EMBRYO         # 배아
    CORD_BLOOD     # 제대혈
    TISSUE         # 조직
    STEM_CELL      # 줄기세포
    OVARIAN_TISSUE # 난소조직
    TESTICULAR_TISSUE # 고환조직
    BONE_MARROW    # 골수
    PERIPHERAL_BLOOD # 말초혈
  }

  """
  검체 상태
  """
  enum SpecimenStatus {
    REGISTERED   # 등록됨
    PREPARED     # 준비됨
    FREEZING     # 동결중
    FROZEN       # 동결완료
    THAWING      # 해동중
    THAWED       # 해동완료
    TRANSFERRED  # 이식됨
    DISPOSED     # 폐기됨
    QUARANTINE   # 격리중
  }

  """
  보존 방법
  """
  enum PreservationMethod {
    SLOW_FREEZING   # 완만동결
    VITRIFICATION   # 유리화동결
    RAPID_FREEZING  # 급속동결
    CONTROLLED_RATE # 프로그램동결
  }

  """
  검체 품질 정보
  """
  type SpecimenQuality {
    volume: Float!
    concentration: Float
    motility: Float
    viability: Float!
    morphology: String
  }

  """
  보존 정보
  """
  type PreservationInfo {
    method: PreservationMethod!
    methodKr: String!
    startDate: String!
    endDate: String
    cryoprotectant: String!
    coolingRate: String!
    performedBy: String!
  }

  """
  저장 위치
  """
  type StorageLocation {
    tankId: String!
    tankName: String
    canisterId: String!
    caneId: String!
    position: String!
    temperature: Float!
  }

  """
  검체
  """
  type Specimen {
    specimenId: ID!
    type: SpecimenType!
    typeKr: String!
    status: SpecimenStatus!
    statusKr: String!
    quality: SpecimenQuality!
    preservation: PreservationInfo!
    storage: StorageLocation!
    barcodeId: String!
    rfidTag: String
    createdAt: String!
    updatedAt: String!
  }

  """
  검체 목록 응답
  """
  type SpecimenConnection {
    edges: [SpecimenEdge!]!
    pageInfo: PageInfo!
    totalCount: Int!
  }

  type SpecimenEdge {
    node: Specimen!
    cursor: String!
  }

  type PageInfo {
    hasNextPage: Boolean!
    hasPreviousPage: Boolean!
    startCursor: String
    endCursor: String
  }

  """
  동결 프로토콜
  """
  type FreezingProtocol {
    protocolId: ID!
    name: String!
    nameKr: String!
    version: String!
    specimenTypes: [SpecimenType!]!
    method: PreservationMethod!
    methodKr: String!
    steps: [ProtocolStep!]!
    status: ProtocolStatus!
  }

  """
  프로토콜 단계
  """
  type ProtocolStep {
    stepNumber: Int!
    name: String!
    nameKr: String!
    duration: Float!
    temperature: Float
    action: String!
    actionKr: String!
  }

  enum ProtocolStatus {
    DRAFT
    ACTIVE
    DEPRECATED
  }

  """
  검체 입력
  """
  input CreateSpecimenInput {
    type: SpecimenType!
    patientId: String!
    collectionDate: String!
    quality: QualityInput!
    preservationMethod: PreservationMethod!
  }

  input QualityInput {
    volume: Float!
    concentration: Float
    motility: Float
    viability: Float!
    morphology: String
  }

  """
  검체 필터
  """
  input SpecimenFilter {
    type: SpecimenType
    status: SpecimenStatus
    fromDate: String
    toDate: String
  }

  """
  쿼리
  """
  type Query {
    """
    검체 조회
    """
    specimen(id: ID!): Specimen

    """
    검체 목록 조회 (커서 기반 페이지네이션)
    """
    specimens(
      first: Int
      after: String
      filter: SpecimenFilter
    ): SpecimenConnection!

    """
    바코드로 검체 조회
    """
    specimenByBarcode(barcodeId: String!): Specimen

    """
    동결 프로토콜 조회
    """
    freezingProtocol(id: ID!): FreezingProtocol

    """
    동결 프로토콜 목록
    """
    freezingProtocols(
      specimenType: SpecimenType
      method: PreservationMethod
    ): [FreezingProtocol!]!

    """
    저장소 현황
    """
    storageStatus: StorageStatus!
  }

  type StorageStatus {
    totalCapacity: Int!
    currentOccupancy: Int!
    availableSpace: Int!
    utilizationRate: Float!
    tankStatuses: [TankStatus!]!
  }

  type TankStatus {
    tankId: String!
    tankName: String!
    capacity: Int!
    occupancy: Int!
    temperature: Float!
    liquidNitrogenLevel: Float!
    status: String!
  }

  """
  뮤테이션
  """
  type Mutation {
    """
    검체 생성
    """
    createSpecimen(input: CreateSpecimenInput!): Specimen!

    """
    검체 상태 변경
    """
    updateSpecimenStatus(
      id: ID!
      status: SpecimenStatus!
      reason: String
    ): Specimen!

    """
    검체 위치 변경
    """
    updateSpecimenLocation(
      id: ID!
      tankId: String!
      position: String!
    ): Specimen!

    """
    검체 삭제 (소프트 삭제)
    """
    deleteSpecimen(id: ID!): Boolean!

    """
    동결 프로토콜 실행
    """
    executeFreezingProtocol(
      protocolId: ID!
      specimenId: ID!
    ): ProtocolExecution!
  }

  type ProtocolExecution {
    executionId: ID!
    protocolId: ID!
    specimenId: ID!
    status: ExecutionStatus!
    startedAt: String!
    completedAt: String
    steps: [StepExecution!]!
  }

  type StepExecution {
    stepNumber: Int!
    status: ExecutionStatus!
    startedAt: String!
    completedAt: String
  }

  enum ExecutionStatus {
    PENDING
    IN_PROGRESS
    COMPLETED
    FAILED
  }

  """
  구독
  """
  type Subscription {
    """
    검체 상태 변경 구독
    """
    specimenStatusChanged(specimenId: ID!): Specimen!

    """
    알람 구독
    """
    alarmTriggered(tankId: String): Alarm!

    """
    프로토콜 실행 진행 상황
    """
    protocolExecutionProgress(executionId: ID!): ProtocolExecution!
  }

  type Alarm {
    alarmId: ID!
    alarmType: String!
    alarmTypeKr: String!
    severity: String!
    severityKr: String!
    message: String!
    messageKr: String!
    triggeredAt: String!
  }
`;
```

### GraphQL 리졸버

```typescript
/**
 * GraphQL 리졸버
 */
import { PubSub } from 'graphql-subscriptions';

const pubsub = new PubSub();

export const resolvers = {
  Query: {
    /**
     * 검체 조회
     */
    specimen: async (_: any, { id }: { id: string }, context: any) => {
      // 인증 확인
      if (!context.user) {
        throw new Error('인증이 필요합니다');
      }

      const specimen = await context.dataSources.specimenAPI.getSpecimen(id);

      // 권한 확인
      if (!context.user.canAccess(specimen)) {
        throw new Error('접근 권한이 없습니다');
      }

      return specimen;
    },

    /**
     * 검체 목록 조회
     */
    specimens: async (
      _: any,
      { first = 20, after, filter }: any,
      context: any
    ) => {
      if (!context.user) {
        throw new Error('인증이 필요합니다');
      }

      const result = await context.dataSources.specimenAPI.getSpecimens({
        first,
        after,
        filter,
        userId: context.user.id,
      });

      return {
        edges: result.items.map((item: any) => ({
          node: item,
          cursor: item.cursor,
        })),
        pageInfo: {
          hasNextPage: result.hasNextPage,
          hasPreviousPage: result.hasPreviousPage,
          startCursor: result.startCursor,
          endCursor: result.endCursor,
        },
        totalCount: result.totalCount,
      };
    },

    /**
     * 저장소 현황
     */
    storageStatus: async (_: any, __: any, context: any) => {
      if (!context.user) {
        throw new Error('인증이 필요합니다');
      }

      return await context.dataSources.storageAPI.getStatus();
    },
  },

  Mutation: {
    /**
     * 검체 생성
     */
    createSpecimen: async (_: any, { input }: any, context: any) => {
      if (!context.user) {
        throw new Error('인증이 필요합니다');
      }

      const specimen = await context.dataSources.specimenAPI.create({
        ...input,
        createdBy: context.user.id,
      });

      // 이벤트 발행
      pubsub.publish('SPECIMEN_CREATED', { specimen });

      return specimen;
    },

    /**
     * 검체 상태 변경
     */
    updateSpecimenStatus: async (
      _: any,
      { id, status, reason }: any,
      context: any
    ) => {
      if (!context.user) {
        throw new Error('인증이 필요합니다');
      }

      const specimen = await context.dataSources.specimenAPI.updateStatus(
        id,
        status,
        {
          changedBy: context.user.id,
          reason,
        }
      );

      // 이벤트 발행
      pubsub.publish('SPECIMEN_STATUS_CHANGED', {
        specimenStatusChanged: specimen,
      });

      return specimen;
    },
  },

  Subscription: {
    /**
     * 검체 상태 변경 구독
     */
    specimenStatusChanged: {
      subscribe: (_: any, { specimenId }: any) => {
        return pubsub.asyncIterator(['SPECIMEN_STATUS_CHANGED']);
      },
    },

    /**
     * 알람 구독
     */
    alarmTriggered: {
      subscribe: (_: any, { tankId }: any) => {
        return pubsub.asyncIterator(['ALARM_TRIGGERED']);
      },
    },
  },
};
```

## WebSocket 실시간 통신

```typescript
/**
 * WebSocket 서버 구현
 */
import WebSocket from 'ws';
import { Server } from 'http';

/**
 * WebSocket 메시지 타입
 */
export enum WsMessageType {
  // 클라이언트 -> 서버
  SUBSCRIBE = 'SUBSCRIBE',
  UNSUBSCRIBE = 'UNSUBSCRIBE',
  PING = 'PING',

  // 서버 -> 클라이언트
  PONG = 'PONG',
  STATUS_UPDATE = 'STATUS_UPDATE',
  ALARM = 'ALARM',
  TEMPERATURE_UPDATE = 'TEMPERATURE_UPDATE',
  PROTOCOL_PROGRESS = 'PROTOCOL_PROGRESS',
}

/**
 * WebSocket 메시지 인터페이스
 */
export interface WsMessage {
  type: WsMessageType;
  data?: any;
  timestamp: string;
}

/**
 * WebSocket 서버
 */
export class CryoWebSocketServer {
  private wss: WebSocket.Server;
  private subscriptions: Map<string, Set<WebSocket>>;

  constructor(server: Server) {
    this.wss = new WebSocket.Server({ server, path: '/ws' });
    this.subscriptions = new Map();
    this.setupHandlers();
  }

  private setupHandlers(): void {
    this.wss.on('connection', (ws: WebSocket, req: any) => {
      console.log('새로운 WebSocket 연결');

      // 인증 처리
      const token = this.extractToken(req);
      if (!token || !this.verifyToken(token)) {
        ws.close(4001, '인증 실패');
        return;
      }

      // Ping/Pong 핸들러
      ws.on('ping', () => {
        ws.pong();
      });

      // 메시지 핸들러
      ws.on('message', (message: string) => {
        try {
          const parsed: WsMessage = JSON.parse(message);
          this.handleMessage(ws, parsed);
        } catch (error) {
          console.error('메시지 파싱 오류:', error);
        }
      });

      // 연결 종료 핸들러
      ws.on('close', () => {
        this.cleanupSubscriptions(ws);
        console.log('WebSocket 연결 종료');
      });

      // 에러 핸들러
      ws.on('error', (error) => {
        console.error('WebSocket 에러:', error);
      });
    });
  }

  /**
   * 메시지 처리
   */
  private handleMessage(ws: WebSocket, message: WsMessage): void {
    switch (message.type) {
      case WsMessageType.SUBSCRIBE:
        this.handleSubscribe(ws, message.data);
        break;

      case WsMessageType.UNSUBSCRIBE:
        this.handleUnsubscribe(ws, message.data);
        break;

      case WsMessageType.PING:
        this.sendMessage(ws, { type: WsMessageType.PONG });
        break;

      default:
        console.warn('알 수 없는 메시지 타입:', message.type);
    }
  }

  /**
   * 구독 처리
   */
  private handleSubscribe(ws: WebSocket, data: any): void {
    const { channel } = data;

    if (!this.subscriptions.has(channel)) {
      this.subscriptions.set(channel, new Set());
    }

    this.subscriptions.get(channel)!.add(ws);
    console.log(`구독 추가: ${channel}`);
  }

  /**
   * 구독 취소 처리
   */
  private handleUnsubscribe(ws: WebSocket, data: any): void {
    const { channel } = data;

    if (this.subscriptions.has(channel)) {
      this.subscriptions.get(channel)!.delete(ws);
      console.log(`구독 취소: ${channel}`);
    }
  }

  /**
   * 구독 정리
   */
  private cleanupSubscriptions(ws: WebSocket): void {
    this.subscriptions.forEach((subs) => {
      subs.delete(ws);
    });
  }

  /**
   * 메시지 전송
   */
  private sendMessage(ws: WebSocket, message: Partial<WsMessage>): void {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        ...message,
        timestamp: new Date().toISOString(),
      }));
    }
  }

  /**
   * 채널로 브로드캐스트
   */
  public broadcast(channel: string, message: Partial<WsMessage>): void {
    const subscribers = this.subscriptions.get(channel);

    if (subscribers) {
      subscribers.forEach((ws) => {
        this.sendMessage(ws, message);
      });
    }
  }

  /**
   * 온도 업데이트 브로드캐스트
   */
  public broadcastTemperatureUpdate(tankId: string, temperature: number): void {
    this.broadcast(`tank:${tankId}`, {
      type: WsMessageType.TEMPERATURE_UPDATE,
      data: {
        tankId,
        temperature,
        timestamp: new Date().toISOString(),
      },
    });
  }

  /**
   * 알람 브로드캐스트
   */
  public broadcastAlarm(alarm: any): void {
    this.broadcast('alarms', {
      type: WsMessageType.ALARM,
      data: alarm,
    });
  }

  /**
   * 프로토콜 진행 상황 브로드캐스트
   */
  public broadcastProtocolProgress(executionId: string, progress: any): void {
    this.broadcast(`protocol:${executionId}`, {
      type: WsMessageType.PROTOCOL_PROGRESS,
      data: progress,
    });
  }

  private extractToken(req: any): string | null {
    const url = new URL(req.url, `http://${req.headers.host}`);
    return url.searchParams.get('token');
  }

  private verifyToken(token: string): boolean {
    // JWT 검증 로직
    return true;
  }
}
```

---

**문서 버전**: 1.0
**최종 수정**: 2025-01-11
**작성자**: WIA Standards Committee

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
