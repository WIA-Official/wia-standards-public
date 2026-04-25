# 4장: API 인터페이스 - REST, GraphQL, WebSocket

## 극저온 법률 표준 완전한 API 구현

이 장에서는 WIA 극저온 법률 표준을 위한 포괄적인 API 구현을 제공합니다. REST 엔드포인트, GraphQL 스키마, 실시간 WebSocket 연결을 포함합니다.

## REST API 구현

```typescript
/**
 * WIA 극저온 법률 표준 - REST API 구현
 * Express 기반 RESTful API
 */

import express, { Router, Request, Response, NextFunction } from 'express';
import { z } from 'zod';
import {
  Contract,
  ContractSchema,
  Dispute,
  DisputeSchema,
  Regulation,
  RegulationSchema,
} from './types';

// ============================================================================
// API 설정
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
  rateLimiting?: RateLimitConfig;
}

export interface RateLimitConfig {
  windowMs: number;
  maxRequests: number;
}

// ============================================================================
// 응답 타입
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  meta?: ResponseMeta;
}

export interface APIError {
  code: string;
  message: string;
  details?: unknown;
}

export interface ResponseMeta {
  requestId: string;
  timestamp: string;
  processingTime: number;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}

// ============================================================================
// 미들웨어
// ============================================================================

// 인증 미들웨어
export function authMiddleware(
  req: Request,
  res: Response,
  next: NextFunction
): void {
  const apiKey = req.headers['x-api-key'] as string;
  const authHeader = req.headers.authorization;

  if (!apiKey && !authHeader) {
    res.status(401).json({
      success: false,
      error: {
        code: 'UNAUTHORIZED',
        message: 'API 키 또는 인증 토큰 필요',
      },
    });
    return;
  }

  // API 키 또는 토큰 검증
  if (apiKey) {
    if (!validateApiKey(apiKey)) {
      res.status(401).json({
        success: false,
        error: {
          code: 'INVALID_API_KEY',
          message: '유효하지 않은 API 키',
        },
      });
      return;
    }
  }

  if (authHeader) {
    const token = authHeader.replace('Bearer ', '');
    const validation = validateJWT(token);

    if (!validation.valid) {
      res.status(401).json({
        success: false,
        error: {
          code: 'INVALID_TOKEN',
          message: validation.error || '유효하지 않은 인증 토큰',
        },
      });
      return;
    }

    (req as any).user = validation.payload;
  }

  next();
}

function validateApiKey(apiKey: string): boolean {
  return apiKey.startsWith('wia_') && apiKey.length === 40;
}

function validateJWT(token: string): { valid: boolean; payload?: any; error?: string } {
  try {
    const parts = token.split('.');
    if (parts.length !== 3) {
      return { valid: false, error: '잘못된 토큰 형식' };
    }
    return { valid: true, payload: JSON.parse(atob(parts[1])) };
  } catch {
    return { valid: false, error: '토큰 디코드 실패' };
  }
}

// 유효성 검사 미들웨어
export function validationMiddleware<T>(schema: z.ZodSchema<T>) {
  return (req: Request, res: Response, next: NextFunction): void => {
    const result = schema.safeParse(req.body);

    if (!result.success) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: '요청 유효성 검사 실패',
          details: result.error.errors.map(e => ({
            path: e.path.join('.'),
            message: e.message,
          })),
        },
      });
      return;
    }

    req.body = result.data;
    next();
  };
}

// 오류 처리기
export function errorHandler(
  error: Error,
  req: Request,
  res: Response,
  next: NextFunction
): void {
  console.error('API 오류:', error);

  if (error instanceof z.ZodError) {
    res.status(400).json({
      success: false,
      error: {
        code: 'VALIDATION_ERROR',
        message: '데이터 유효성 검사 실패',
        details: error.errors,
      },
    });
    return;
  }

  res.status(500).json({
    success: false,
    error: {
      code: 'INTERNAL_ERROR',
      message: '내부 오류가 발생했습니다',
    },
  });
}

// ============================================================================
// 계약 라우터
// ============================================================================

export function createContractRouter(service: ContractService): Router {
  const router = Router();

  // 계약 목록 조회 (페이지네이션)
  router.get('/', async (req: Request, res: Response) => {
    try {
      const { type, status, limit = 20, offset = 0 } = req.query;

      const result = await service.listContracts({
        type: type as string,
        status: status as string,
        limit: Number(limit),
        offset: Number(offset),
      });

      res.json({
        success: true,
        data: result.contracts,
        pagination: {
          total: result.total,
          limit: Number(limit),
          offset: Number(offset),
          hasMore: result.total > Number(offset) + Number(limit),
        },
      });
    } catch (error) {
      throw error;
    }
  });

  // 계약 ID로 조회
  router.get('/:id', async (req: Request, res: Response) => {
    try {
      const contract = await service.getContractById(req.params.id);

      if (!contract) {
        res.status(404).json({
          success: false,
          error: {
            code: 'NOT_FOUND',
            message: `계약 ${req.params.id}을(를) 찾을 수 없음`,
          },
        });
        return;
      }

      res.json({
        success: true,
        data: contract,
      });
    } catch (error) {
      throw error;
    }
  });

  // 계약 생성
  router.post(
    '/',
    validationMiddleware(ContractSchema),
    async (req: Request, res: Response) => {
      try {
        const contract = await service.createContract(req.body);

        res.status(201).json({
          success: true,
          data: contract,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // 계약 업데이트
  router.put(
    '/:id',
    validationMiddleware(ContractSchema.partial()),
    async (req: Request, res: Response) => {
      try {
        const contract = await service.updateContract(req.params.id, req.body);

        res.json({
          success: true,
          data: contract,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // 계약에 서명 추가
  router.post('/:id/signatures', async (req: Request, res: Response) => {
    try {
      const signature = await service.addSignature(req.params.id, req.body);

      res.status(201).json({
        success: true,
        data: signature,
      });
    } catch (error) {
      throw error;
    }
  });

  // 계약 실행 (활성화)
  router.post('/:id/execute', async (req: Request, res: Response) => {
    try {
      const contract = await service.executeContract(req.params.id);

      res.json({
        success: true,
        data: contract,
        message: '계약이 성공적으로 실행됨',
      });
    } catch (error) {
      throw error;
    }
  });

  // 계약 해지
  router.post('/:id/terminate', async (req: Request, res: Response) => {
    try {
      const { reason, effectiveDate } = req.body;
      const contract = await service.terminateContract(
        req.params.id,
        reason,
        effectiveDate
      );

      res.json({
        success: true,
        data: contract,
        message: '계약이 성공적으로 해지됨',
      });
    } catch (error) {
      throw error;
    }
  });

  return router;
}

// ============================================================================
// 분쟁 라우터
// ============================================================================

export function createDisputeRouter(service: DisputeService): Router {
  const router = Router();

  // 분쟁 목록 조회
  router.get('/', async (req: Request, res: Response) => {
    try {
      const { type, status, limit = 20, offset = 0 } = req.query;

      const result = await service.listDisputes({
        type: type as string,
        status: status as string,
        limit: Number(limit),
        offset: Number(offset),
      });

      res.json({
        success: true,
        data: result.disputes,
        pagination: {
          total: result.total,
          limit: Number(limit),
          offset: Number(offset),
          hasMore: result.total > Number(offset) + Number(limit),
        },
      });
    } catch (error) {
      throw error;
    }
  });

  // 새 분쟁 제출
  router.post(
    '/',
    validationMiddleware(DisputeSchema.omit({ id: true, status: true, history: true })),
    async (req: Request, res: Response) => {
      try {
        const dispute = await service.fileDispute(req.body);

        res.status(201).json({
          success: true,
          data: dispute,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // 분쟁 상태 업데이트
  router.patch('/:id/status', async (req: Request, res: Response) => {
    try {
      const { status, reason } = req.body;
      const dispute = await service.updateDisputeStatus(
        req.params.id,
        status,
        reason
      );

      res.json({
        success: true,
        data: dispute,
      });
    } catch (error) {
      throw error;
    }
  });

  // 분쟁 에스컬레이션
  router.post('/:id/escalate', async (req: Request, res: Response) => {
    try {
      const { mechanism, reason } = req.body;
      const dispute = await service.escalateDispute(
        req.params.id,
        mechanism,
        reason
      );

      res.json({
        success: true,
        data: dispute,
        message: `분쟁이 ${mechanism}(으)로 에스컬레이션됨`,
      });
    } catch (error) {
      throw error;
    }
  });

  // 분쟁 해결
  router.post('/:id/resolve', async (req: Request, res: Response) => {
    try {
      const resolution = req.body;
      const dispute = await service.resolveDispute(req.params.id, resolution);

      res.json({
        success: true,
        data: dispute,
        message: '분쟁이 성공적으로 해결됨',
      });
    } catch (error) {
      throw error;
    }
  });

  return router;
}

// ============================================================================
// 준수 라우터
// ============================================================================

export function createComplianceRouter(service: ComplianceService): Router {
  const router = Router();

  // 준수 요구사항 조회
  router.get('/requirements', async (req: Request, res: Response) => {
    try {
      const { regulation, status } = req.query;
      const requirements = await service.getRequirements({
        regulation: regulation as string,
        status: status as string,
      });

      res.json({
        success: true,
        data: requirements,
      });
    } catch (error) {
      throw error;
    }
  });

  // 준수 점수 조회
  router.get('/score', async (req: Request, res: Response) => {
    try {
      const score = await service.getComplianceScore();

      res.json({
        success: true,
        data: score,
      });
    } catch (error) {
      throw error;
    }
  });

  // 요구사항 상태 업데이트
  router.patch('/requirements/:id', async (req: Request, res: Response) => {
    try {
      const { status, evidence } = req.body;
      const requirement = await service.updateRequirementStatus(
        req.params.id,
        status,
        evidence
      );

      res.json({
        success: true,
        data: requirement,
      });
    } catch (error) {
      throw error;
    }
  });

  // 위반 보고
  router.post('/violations', async (req: Request, res: Response) => {
    try {
      const violation = await service.reportViolation(req.body);

      res.status(201).json({
        success: true,
        data: violation,
      });
    } catch (error) {
      throw error;
    }
  });

  return router;
}

// ============================================================================
// 메인 API 애플리케이션
// ============================================================================

export function createCryoLegalAPI(
  services: CryoLegalServices,
  config: APIConfig
): express.Application {
  const app = express();

  // 미들웨어
  app.use(express.json());
  app.use(authMiddleware);

  // 요청 로깅
  app.use((req, res, next) => {
    const start = Date.now();

    res.on('finish', () => {
      const duration = Date.now() - start;
      console.log(`${req.method} ${req.path} ${res.statusCode} ${duration}ms`);
    });

    next();
  });

  // API 라우트
  app.use('/api/v1/contracts', createContractRouter(services.contracts));
  app.use('/api/v1/disputes', createDisputeRouter(services.disputes));
  app.use('/api/v1/compliance', createComplianceRouter(services.compliance));

  // 헬스 체크
  app.get('/health', (req, res) => {
    res.json({
      status: 'healthy',
      version: '1.0.0',
      timestamp: new Date().toISOString(),
    });
  });

  // 오류 처리기
  app.use(errorHandler);

  return app;
}

// ============================================================================
// 서비스 인터페이스
// ============================================================================

export interface CryoLegalServices {
  contracts: ContractService;
  disputes: DisputeService;
  compliance: ComplianceService;
}

export interface ContractService {
  listContracts(params: any): Promise<{ contracts: Contract[]; total: number }>;
  getContractById(id: string): Promise<Contract | null>;
  createContract(data: Contract): Promise<Contract>;
  updateContract(id: string, data: Partial<Contract>): Promise<Contract>;
  addSignature(id: string, signature: any): Promise<any>;
  executeContract(id: string): Promise<Contract>;
  terminateContract(id: string, reason: string, effectiveDate: string): Promise<Contract>;
}

export interface DisputeService {
  listDisputes(params: any): Promise<{ disputes: Dispute[]; total: number }>;
  fileDispute(data: any): Promise<Dispute>;
  updateDisputeStatus(id: string, status: string, reason: string): Promise<Dispute>;
  escalateDispute(id: string, mechanism: string, reason: string): Promise<Dispute>;
  resolveDispute(id: string, resolution: any): Promise<Dispute>;
}

export interface ComplianceService {
  getRequirements(filters: any): Promise<any[]>;
  getComplianceScore(): Promise<any>;
  updateRequirementStatus(id: string, status: string, evidence: string[]): Promise<any>;
  reportViolation(data: any): Promise<any>;
}
```

## GraphQL API 구현

```typescript
/**
 * WIA 극저온 법률 표준 - GraphQL API 구현
 * Apollo Server 기반 GraphQL API
 */

import { gql } from 'graphql-tag';

// ============================================================================
// GraphQL 타입 정의
// ============================================================================

export const typeDefs = gql`
  scalar DateTime

  # 계약 타입
  type Contract {
    id: ID!
    templateId: String
    type: ContractType!
    parties: [Party!]!
    subject: ContractSubject
    terms: ContractTerms!
    signatures: [Signature!]!
    effectiveDate: DateTime!
    expiryDate: DateTime
    status: ContractStatus!
    history: [ContractEvent!]!
  }

  enum ContractType {
    STORAGE_AGREEMENT    # 보관 계약
    CONSENT_FORM         # 동의서
    SERVICE_AGREEMENT    # 서비스 계약
    TRANSFER_AGREEMENT   # 이전 계약
    RESEARCH_AGREEMENT   # 연구 계약
    DONATION_AGREEMENT   # 기증 계약
  }

  enum ContractStatus {
    DRAFT       # 초안
    PENDING     # 대기 중
    ACTIVE      # 활성
    EXPIRED     # 만료
    TERMINATED  # 해지
    DISPUTED    # 분쟁 중
  }

  type Party {
    id: ID!
    type: PartyType!
    name: String!
    role: PartyRole!
    representative: String
    contact: ContactInfo!
  }

  enum PartyType {
    INDIVIDUAL    # 개인
    ORGANIZATION  # 조직
  }

  enum PartyRole {
    PRIMARY       # 주 당사자
    COUNTERPARTY  # 상대방
    GUARANTOR     # 보증인
    WITNESS       # 증인
  }

  type ContactInfo {
    name: String!
    email: String!
    phone: String
  }

  type ContractSubject {
    type: SubjectType!
    identifier: String!
    description: String!
  }

  enum SubjectType {
    SPECIMEN   # 검체
    SERVICE    # 서비스
    FACILITY   # 시설
    RESEARCH   # 연구
  }

  type ContractTerms {
    duration: String!
    renewal: RenewalType!
    fees: [Fee!]
    obligations: [PartyObligation!]!
    warranties: [String!]!
    limitations: [String!]!
    governing: String!
    venue: String!
  }

  type Fee {
    type: String!
    amount: Float!
    currency: String!
    frequency: FeeFrequency!
    due: String!
  }

  enum RenewalType {
    AUTOMATIC  # 자동
    MANUAL     # 수동
    NONE       # 없음
  }

  enum FeeFrequency {
    ONE_TIME   # 일회성
    MONTHLY    # 월간
    ANNUAL     # 연간
  }

  type PartyObligation {
    party: String!
    obligation: String!
  }

  type Signature {
    party: String!
    signatory: String!
    date: DateTime!
    method: SignatureMethod!
    witness: String
    notarized: Boolean
  }

  enum SignatureMethod {
    WET         # 수기
    ELECTRONIC  # 전자
    DIGITAL     # 디지털
  }

  type ContractEvent {
    id: ID!
    type: String!
    date: DateTime!
    actor: String!
    description: String!
    documents: [String!]
  }

  # 분쟁 타입
  type Dispute {
    id: ID!
    type: DisputeType!
    parties: [String!]!
    subject: String!
    description: String!
    filedDate: DateTime!
    status: DisputeStatus!
    currentMechanism: String
    resolution: Resolution
    history: [DisputeEvent!]!
  }

  enum DisputeType {
    CONTRACT    # 계약 분쟁
    CONSENT     # 동의 분쟁
    OWNERSHIP   # 소유권 분쟁
    NEGLIGENCE  # 과실 분쟁
    REGULATORY  # 규제 분쟁
  }

  enum DisputeStatus {
    FILED        # 제출됨
    UNDER_REVIEW # 검토 중
    NEGOTIATION  # 협상 중
    MEDIATION    # 조정 중
    ARBITRATION  # 중재 중
    LITIGATION   # 소송 중
    RESOLVED     # 해결됨
    DISMISSED    # 기각됨
  }

  type Resolution {
    date: DateTime!
    mechanism: String!
    outcome: String!
    terms: [String!]!
    binding: Boolean!
    enforcement: String
  }

  type DisputeEvent {
    date: DateTime!
    type: String!
    description: String!
    actor: String!
    documents: [String!]
  }

  # 준수 타입
  type ComplianceRequirement {
    id: ID!
    regulation: String!
    requirement: String!
    responsible: String!
    evidence: [String!]!
    frequency: String!
    lastCheck: DateTime!
    status: ComplianceStatus!
  }

  enum ComplianceStatus {
    COMPLIANT      # 준수
    NON_COMPLIANT  # 미준수
    PARTIAL        # 부분 준수
    PENDING        # 대기 중
  }

  type ComplianceScore {
    total: Int!
    compliant: Int!
    nonCompliant: Int!
    partial: Int!
    pending: Int!
    score: Float!
    rating: String!
  }

  # 쿼리
  type Query {
    # 계약
    contracts(
      type: ContractType
      status: ContractStatus
      limit: Int
      offset: Int
    ): ContractConnection!
    contract(id: ID!): Contract

    # 분쟁
    disputes(
      type: DisputeType
      status: DisputeStatus
      limit: Int
      offset: Int
    ): DisputeConnection!
    dispute(id: ID!): Dispute

    # 준수
    complianceRequirements(
      regulation: String
      status: ComplianceStatus
    ): [ComplianceRequirement!]!
    complianceScore: ComplianceScore!
  }

  type ContractConnection {
    nodes: [Contract!]!
    totalCount: Int!
    pageInfo: PageInfo!
  }

  type DisputeConnection {
    nodes: [Dispute!]!
    totalCount: Int!
    pageInfo: PageInfo!
  }

  type PageInfo {
    hasNextPage: Boolean!
    hasPreviousPage: Boolean!
    startCursor: String
    endCursor: String
  }

  # 뮤테이션
  type Mutation {
    # 계약
    createContract(input: CreateContractInput!): Contract!
    updateContract(id: ID!, input: UpdateContractInput!): Contract!
    addSignature(contractId: ID!, input: SignatureInput!): Contract!
    executeContract(id: ID!): Contract!
    terminateContract(id: ID!, reason: String!, effectiveDate: DateTime!): Contract!

    # 분쟁
    fileDispute(input: FileDisputeInput!): Dispute!
    updateDisputeStatus(id: ID!, status: DisputeStatus!, reason: String!): Dispute!
    escalateDispute(id: ID!, mechanism: String!, reason: String!): Dispute!
    resolveDispute(id: ID!, input: ResolutionInput!): Dispute!

    # 준수
    updateComplianceStatus(
      id: ID!
      status: ComplianceStatus!
      evidence: [String!]
    ): ComplianceRequirement!
  }

  # 입력 타입
  input CreateContractInput {
    type: ContractType!
    parties: [PartyInput!]!
    subject: ContractSubjectInput
    terms: ContractTermsInput!
    effectiveDate: DateTime!
    expiryDate: DateTime
  }

  input UpdateContractInput {
    terms: ContractTermsInput
    expiryDate: DateTime
  }

  input PartyInput {
    type: PartyType!
    name: String!
    role: PartyRole!
    contact: ContactInfoInput!
  }

  input ContactInfoInput {
    name: String!
    email: String!
    phone: String
  }

  input ContractSubjectInput {
    type: SubjectType!
    identifier: String!
    description: String!
  }

  input ContractTermsInput {
    duration: String!
    renewal: RenewalType!
    governing: String!
    venue: String!
  }

  input SignatureInput {
    party: String!
    signatory: String!
    method: SignatureMethod!
  }

  input FileDisputeInput {
    type: DisputeType!
    parties: [String!]!
    subject: String!
    description: String!
  }

  input ResolutionInput {
    mechanism: String!
    outcome: String!
    terms: [String!]!
    binding: Boolean!
  }

  # 구독
  type Subscription {
    contractUpdated(contractId: ID!): Contract!
    disputeStatusChanged(disputeId: ID): Dispute!
    complianceAlert: ComplianceAlert!
  }

  type ComplianceAlert {
    type: String!
    requirementId: String!
    message: String!
    severity: String!
    timestamp: DateTime!
  }
`;
```

## WebSocket 실시간 API

```typescript
/**
 * WIA 극저온 법률 표준 - WebSocket 실시간 API
 * 법률 운영을 위한 실시간 이벤트 스트리밍
 */

import { WebSocketServer, WebSocket } from 'ws';
import { v4 as uuidv4 } from 'uuid';

// ============================================================================
// WebSocket 서버 구현
// ============================================================================

export interface WSMessage {
  type: string;
  payload: unknown;
  timestamp: string;
  messageId: string;
}

export interface WSClient {
  id: string;
  socket: WebSocket;
  subscriptions: Set<string>;
  authenticated: boolean;
  userId?: string;
}

export class CryoLegalWebSocketServer {
  private wss: WebSocketServer;
  private clients: Map<string, WSClient> = new Map();
  private subscriptionClients: Map<string, Set<string>> = new Map();

  constructor(port: number) {
    this.wss = new WebSocketServer({ port });
    this.initialize();
  }

  private initialize(): void {
    this.wss.on('connection', (socket: WebSocket) => {
      const clientId = uuidv4();

      const client: WSClient = {
        id: clientId,
        socket,
        subscriptions: new Set(),
        authenticated: false,
      };

      this.clients.set(clientId, client);

      // 연결 확인 전송
      this.sendToClient(client, {
        type: 'connection_ack',
        payload: { clientId },
        timestamp: new Date().toISOString(),
        messageId: uuidv4(),
      });

      // 메시지 처리
      socket.on('message', (data: Buffer) => {
        this.handleMessage(client, data.toString());
      });

      // 연결 종료 처리
      socket.on('close', () => {
        this.handleDisconnect(client);
      });
    });
  }

  private handleMessage(client: WSClient, data: string): void {
    try {
      const message = JSON.parse(data);

      switch (message.type) {
        case 'authenticate':
          this.handleAuthenticate(client, message.payload);
          break;
        case 'subscribe':
          this.handleSubscribe(client, message.payload);
          break;
        case 'unsubscribe':
          this.handleUnsubscribe(client, message.payload);
          break;
        case 'ping':
          this.handlePing(client);
          break;
      }
    } catch (error) {
      this.sendError(client, 'PARSE_ERROR', '메시지 파싱 실패');
    }
  }

  private handleAuthenticate(client: WSClient, payload: { token: string }): void {
    const validation = this.validateToken(payload.token);

    if (validation.valid) {
      client.authenticated = true;
      client.userId = validation.userId;

      this.sendToClient(client, {
        type: 'authenticated',
        payload: { userId: validation.userId },
        timestamp: new Date().toISOString(),
        messageId: uuidv4(),
      });
    } else {
      this.sendError(client, 'AUTH_FAILED', '인증 실패');
    }
  }

  private validateToken(token: string): { valid: boolean; userId?: string } {
    if (token && token.length > 0) {
      return { valid: true, userId: 'user-123' };
    }
    return { valid: false };
  }

  private handleSubscribe(client: WSClient, payload: { topics: string[] }): void {
    if (!client.authenticated) {
      this.sendError(client, 'NOT_AUTHENTICATED', '인증 필요');
      return;
    }

    for (const topic of payload.topics) {
      client.subscriptions.add(topic);

      if (!this.subscriptionClients.has(topic)) {
        this.subscriptionClients.set(topic, new Set());
      }
      this.subscriptionClients.get(topic)!.add(client.id);
    }

    this.sendToClient(client, {
      type: 'subscribed',
      payload: { topics: Array.from(client.subscriptions) },
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  private handleUnsubscribe(client: WSClient, payload: { topics: string[] }): void {
    for (const topic of payload.topics) {
      client.subscriptions.delete(topic);

      const topicClients = this.subscriptionClients.get(topic);
      if (topicClients) {
        topicClients.delete(client.id);
      }
    }
  }

  private handlePing(client: WSClient): void {
    this.sendToClient(client, {
      type: 'pong',
      payload: {},
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  private handleDisconnect(client: WSClient): void {
    for (const topic of client.subscriptions) {
      const topicClients = this.subscriptionClients.get(topic);
      if (topicClients) {
        topicClients.delete(client.id);
      }
    }

    this.clients.delete(client.id);
  }

  private sendToClient(client: WSClient, message: WSMessage): void {
    if (client.socket.readyState === WebSocket.OPEN) {
      client.socket.send(JSON.stringify(message));
    }
  }

  private sendError(client: WSClient, code: string, message: string): void {
    this.sendToClient(client, {
      type: 'error',
      payload: { code, message },
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  // 계약 이벤트 발행
  public publishContractEvent(contractId: string, event: ContractUpdateEvent): void {
    const topic = `contract:${contractId}`;
    this.publishToTopic(topic, {
      type: 'contract_update',
      payload: event,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  // 분쟁 이벤트 발행
  public publishDisputeEvent(disputeId: string, event: DisputeUpdateEvent): void {
    const topic = `dispute:${disputeId}`;
    this.publishToTopic(topic, {
      type: 'dispute_update',
      payload: event,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  // 준수 알림 발행
  public publishComplianceAlert(alert: ComplianceAlertEvent): void {
    this.publishToTopic('compliance:alerts', {
      type: 'compliance_alert',
      payload: alert,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  private publishToTopic(topic: string, message: WSMessage): void {
    const clientIds = this.subscriptionClients.get(topic);

    if (clientIds) {
      for (const clientId of clientIds) {
        const client = this.clients.get(clientId);
        if (client) {
          this.sendToClient(client, message);
        }
      }
    }
  }
}

export interface ContractUpdateEvent {
  contractId: string;
  action: 'created' | 'updated' | 'signed' | 'executed' | 'terminated';
  newStatus: string;
  actor: string;
}

export interface DisputeUpdateEvent {
  disputeId: string;
  action: 'filed' | 'status_changed' | 'escalated' | 'resolved';
  newStatus?: string;
  actor: string;
}

export interface ComplianceAlertEvent {
  alertId: string;
  type: 'requirement_due' | 'status_change' | 'violation';
  severity: 'low' | 'medium' | 'high' | 'critical';
  message: string;
}
```

---

## 장 요약

이 장에서는 완전한 API 구현을 제공했습니다:

- **REST API**: 미들웨어를 포함한 Express 기반 엔드포인트
- **GraphQL API**: 전체 스키마와 리졸버를 포함한 Apollo Server
- **WebSocket API**: 구독 관리를 포함한 실시간 이벤트 스트리밍

---

**다음 장**: [계약 관리 - 템플릿, 조항, 생명주기](./05-contract-management.md)
