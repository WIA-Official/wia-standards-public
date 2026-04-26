# 제4장: 임상의사결정지원 API 인터페이스

## 서비스 아키텍처 및 통합 엔드포인트

### 4.1 CDSS API 아키텍처

WIA-CLINICAL-DECISION-SUPPORT 표준은 의료 시스템 전반에 임상의사결정 기능에 대한 일관된 접근을 제공하는 RESTful 및 실시간 API를 정의합니다.

```typescript
// CDSS REST API 명세
interface CDSSRestAPI {
  version: 'v1';
  baseUrl: '/api/cdss/v1';

  endpoints: {
    recommendations: RecommendationEndpoints;
    alerts: AlertEndpoints;
    knowledge: KnowledgeEndpoints;
    calculators: CalculatorEndpoints;
    analytics: AnalyticsEndpoints;
    admin: AdminEndpoints;
  };

  authentication: {
    type: 'OAuth 2.0 + SMART on FHIR';
    scopes: CDSSAuthScope[];
  };

  rateLimit: {
    requestsPerMinute: 1000;
    burstLimit: 100;
  };
}

// 권장사항 엔드포인트
interface RecommendationEndpoints {
  // POST /recommendations - 권장사항 요청
  createRecommendationRequest: {
    method: 'POST';
    path: '/recommendations';
    request: RecommendationRequest;
    response: RecommendationResponse;
    description: '특정 임상 질문에 대한 CDSS 권장사항 요청';
  };

  // GET /recommendations/{id} - 권장사항 조회
  getRecommendation: {
    method: 'GET';
    path: '/recommendations/{id}';
    response: RecommendationResponse;
    description: 'ID로 특정 권장사항 조회';
  };

  // GET /recommendations - 환자별 목록 조회
  listRecommendations: {
    method: 'GET';
    path: '/recommendations';
    query: {
      patientId: string;
      status?: 'pending' | 'viewed' | 'accepted' | 'rejected';
      category?: string;
      from?: string;
      to?: string;
      limit?: number;
    };
    response: RecommendationListResponse;
  };

  // PUT /recommendations/{id}/action - 권장사항 조치
  updateRecommendationAction: {
    method: 'PUT';
    path: '/recommendations/{id}/action';
    request: RecommendationActionRequest;
    response: RecommendationActionResponse;
    description: '권장사항 수락/거부 등 조치 기록';
  };
}

// 요청/응답 타입
interface RecommendationRequest {
  patientId: string;
  encounterId?: string;
  userId: string;
  userRole: ClinicalRole;
  questionType: ClinicalQuestionType;
  primaryConcern: string;
  additionalContext?: string;
  urgency: 'ROUTINE' | 'URGENT' | 'EMERGENT';
  responsePreferences: {
    maxRecommendations?: number;
    includeAlternatives?: boolean;
    includeEvidence?: boolean;
    explanationLevel?: 'BRIEF' | 'STANDARD' | 'DETAILED';
    language?: string;
  };
  constraints?: ClinicalConstraint[];
}

interface RecommendationResponse {
  id: string;
  requestId: string;
  timestamp: Date;
  status: 'success' | 'partial' | 'error';
  patientId: string;
  questionType: ClinicalQuestionType;

  recommendations: Recommendation[];

  confidence: {
    overall: number;
    dataQuality: number;
    modelConfidence: number;
    applicability: number;
  };

  evidence: {
    level: EvidenceLevel;
    sources: EvidenceSource[];
  };

  explanation: {
    summary: string;
    reasoning: string[];
    keyFactors: KeyFactor[];
    limitations: string[];
  };

  alternatives: AlternativeOption[];
  contraindications: Contraindication[];
  monitoring: MonitoringRecommendation[];

  metadata: {
    processingTime: number;
    modelsUsed: string[];
    knowledgeVersion: string;
    validUntil: Date;
  };
}

interface Recommendation {
  rank: number;
  id: string;
  action: string;
  actionType: ActionType;
  rationale: string;
  strength: 'STRONG' | 'MODERATE' | 'WEAK' | 'CONDITIONAL';
  urgency: 'IMMEDIATE' | 'SOON' | 'ROUTINE' | 'OPTIONAL';
  confidence: number;
  considerations: string[];
  resources: ResourceReference[];
  suggestedOrder?: SuggestedOrder;
}

// 경보 엔드포인트
interface AlertEndpoints {
  // POST /alerts/evaluate - 경보 평가
  evaluateAlerts: {
    method: 'POST';
    path: '/alerts/evaluate';
    request: AlertEvaluationRequest;
    response: AlertEvaluationResponse;
    description: '제공된 데이터에 대해 경보 규칙 평가';
  };

  // GET /alerts - 활성 경보 조회
  listAlerts: {
    method: 'GET';
    path: '/alerts';
    query: {
      patientId: string;
      encounterId?: string;
      severity?: AlertSeverity;
      category?: AlertCategory;
      status?: 'active' | 'acknowledged' | 'overridden' | 'resolved';
      limit?: number;
    };
    response: AlertListResponse;
  };

  // PUT /alerts/{id}/acknowledge - 경보 확인
  acknowledgeAlert: {
    method: 'PUT';
    path: '/alerts/{id}/acknowledge';
    request: AlertAcknowledgeRequest;
    response: AlertAcknowledgeResponse;
  };

  // PUT /alerts/{id}/override - 경보 무시
  overrideAlert: {
    method: 'PUT';
    path: '/alerts/{id}/override';
    request: AlertOverrideRequest;
    response: AlertOverrideResponse;
  };

  // WebSocket /alerts/stream - 실시간 스트림
  streamAlerts: {
    protocol: 'WebSocket';
    path: '/alerts/stream';
    description: '환자/접점에 대한 실시간 경보 스트리밍';
  };
}

interface AlertEvaluationRequest {
  patientId: string;
  encounterId?: string;
  userId: string;
  trigger: AlertTriggerData;
  categories?: AlertCategory[];
  severity?: AlertSeverity;
}

interface AlertTriggerData {
  type: 'MEDICATION_ORDER' | 'LAB_RESULT' | 'VITAL_SIGN' | 'DIAGNOSIS' | 'PROCEDURE';
  data: Record<string, unknown>;
  timestamp: Date;
}

interface AlertEvaluationResponse {
  requestId: string;
  evaluatedRules: number;
  alerts: ClinicalAlert[];
  suppressedAlerts: SuppressedAlert[];
  processingTime: number;
}

interface ClinicalAlert {
  id: string;
  timestamp: Date;
  severity: AlertSeverity;
  category: AlertCategory;
  title: string;
  message: string;
  detail?: string;
  patientId: string;
  encounterId?: string;
  triggeredBy: AlertTrigger;
  recommendedAction: string;
  overrideOptions: OverrideOption[];
  references: Reference[];
  expiresAt?: Date;
  acknowledgementRequired: boolean;
}
```

### 4.2 GraphQL API

```typescript
// CDSS GraphQL 스키마
const cdssGraphQLSchema = `
  # 환자 임상 데이터
  type Patient {
    id: ID!
    demographics: Demographics!
    conditions: [Condition!]!
    medications: [Medication!]!
    allergies: [Allergy!]!
    vitals(limit: Int, from: DateTime): [VitalSign!]!
    labs(limit: Int, category: String): [LabResult!]!
    riskScores: [RiskScore!]!
    recommendations(status: RecommendationStatus): [Recommendation!]!
    alerts(severity: AlertSeverity, status: AlertStatus): [Alert!]!
  }

  type Demographics {
    name: String!
    birthDate: Date!
    age: Int!
    gender: Gender!
    race: [String!]
    ethnicity: String
  }

  # CDSS 권장사항
  type Recommendation {
    id: ID!
    timestamp: DateTime!
    questionType: QuestionType!
    action: String!
    rationale: String!
    strength: RecommendationStrength!
    urgency: Urgency!
    confidence: Float!
    evidence: Evidence!
    explanation: Explanation!
    alternatives: [Alternative!]
    status: RecommendationStatus!
    actionTaken: ActionRecord
  }

  type Evidence {
    level: EvidenceLevel!
    sources: [EvidenceSource!]!
    guidelines: [Guideline!]
  }

  type Explanation {
    summary: String!
    reasoning: [String!]!
    keyFactors: [KeyFactor!]!
    limitations: [String!]
  }

  type KeyFactor {
    factor: String!
    value: String!
    impact: Impact!
    source: String!
  }

  # 경보
  type Alert {
    id: ID!
    timestamp: DateTime!
    severity: AlertSeverity!
    category: AlertCategory!
    title: String!
    message: String!
    detail: String
    recommendedAction: String!
    status: AlertStatus!
    acknowledgement: Acknowledgement
    override: Override
  }

  # 위험 점수
  type RiskScore {
    id: ID!
    scoreType: String!
    name: String!
    value: Float!
    category: RiskCategory!
    calculatedAt: DateTime!
    factors: [RiskFactor!]!
    trend: Trend
    validUntil: DateTime!
  }

  # 쿼리
  type Query {
    # 환자 데이터
    patient(id: ID!): Patient
    patients(ids: [ID!]!): [Patient!]!

    # 권장사항
    recommendation(id: ID!): Recommendation
    recommendations(
      patientId: ID!
      questionType: QuestionType
      status: RecommendationStatus
      limit: Int
    ): [Recommendation!]!

    # 경보
    alert(id: ID!): Alert
    alerts(
      patientId: ID!
      severity: AlertSeverity
      category: AlertCategory
      status: AlertStatus
    ): [Alert!]!

    # 위험 점수
    riskScore(patientId: ID!, scoreType: String!): RiskScore
    riskScores(patientId: ID!): [RiskScore!]!

    # 임상 계산기
    calculate(calculatorId: String!, inputs: JSON!): CalculatorResult!
    calculators(category: String): [Calculator!]!

    # 지식 베이스
    guideline(id: ID!): Guideline
    searchGuidelines(query: String!, specialty: String): [Guideline!]!
    drugInteraction(drug1: String!, drug2: String!): DrugInteraction
  }

  # 뮤테이션
  type Mutation {
    # 권장사항 요청
    requestRecommendation(input: RecommendationInput!): Recommendation!

    # 권장사항 조치
    acceptRecommendation(id: ID!, notes: String): ActionRecord!
    rejectRecommendation(id: ID!, reason: String!): ActionRecord!

    # 경보 조치
    acknowledgeAlert(id: ID!): Alert!
    overrideAlert(id: ID!, reason: OverrideReason!, comment: String): Alert!

    # 피드백
    submitOutcomeFeedback(input: OutcomeFeedbackInput!): FeedbackRecord!
  }

  # 구독(실시간)
  type Subscription {
    # 실시간 경보
    alertStream(patientId: ID, encounterId: ID): Alert!

    # 위험 점수 업데이트
    riskScoreUpdate(patientId: ID!): RiskScore!

    # 권장사항 업데이트
    recommendationUpdate(patientId: ID!): Recommendation!
  }

  # 입력 타입
  input RecommendationInput {
    patientId: ID!
    encounterId: ID
    questionType: QuestionType!
    primaryConcern: String!
    additionalContext: String
    urgency: Urgency!
    preferences: PreferencesInput
  }

  input PreferencesInput {
    maxRecommendations: Int
    includeAlternatives: Boolean
    includeEvidence: Boolean
    explanationLevel: ExplanationLevel
  }

  input OutcomeFeedbackInput {
    recommendationId: ID!
    outcome: Outcome!
    notes: String
    followUpDate: DateTime
  }

  # 열거형
  enum AlertSeverity {
    LOW
    MEDIUM
    HIGH
    CRITICAL
  }

  enum AlertCategory {
    DRUG_INTERACTION
    DRUG_ALLERGY
    LAB_CRITICAL
    VITAL_SIGN
    SCREENING_DUE
    RISK_ALERT
  }

  enum RecommendationStrength {
    STRONG
    MODERATE
    WEAK
    CONDITIONAL
  }

  enum EvidenceLevel {
    HIGH
    MODERATE
    LOW
    VERY_LOW
    EXPERT_OPINION
  }
`;

// GraphQL 리졸버
class CDSSGraphQLResolvers {
  constructor(
    private cdssService: ClinicalDecisionSupportService,
    private patientService: PatientDataService,
    private alertService: AlertService
  ) {}

  Query = {
    patient: async (_: unknown, { id }: { id: string }) => {
      return this.patientService.getPatient(id);
    },

    recommendations: async (
      _: unknown,
      args: { patientId: string; questionType?: string; status?: string; limit?: number }
    ) => {
      return this.cdssService.getRecommendations(args);
    },

    calculate: async (
      _: unknown,
      { calculatorId, inputs }: { calculatorId: string; inputs: Record<string, unknown> }
    ) => {
      return this.cdssService.runCalculator(calculatorId, inputs);
    },

    drugInteraction: async (
      _: unknown,
      { drug1, drug2 }: { drug1: string; drug2: string }
    ) => {
      return this.cdssService.checkDrugInteraction(drug1, drug2);
    }
  };

  Mutation = {
    requestRecommendation: async (
      _: unknown,
      { input }: { input: RecommendationInput },
      context: GraphQLContext
    ) => {
      return this.cdssService.generateRecommendation({
        ...input,
        userId: context.userId,
        userRole: context.userRole
      });
    },

    acknowledgeAlert: async (
      _: unknown,
      { id }: { id: string },
      context: GraphQLContext
    ) => {
      return this.alertService.acknowledge(id, context.userId);
    },

    overrideAlert: async (
      _: unknown,
      { id, reason, comment }: { id: string; reason: string; comment?: string },
      context: GraphQLContext
    ) => {
      return this.alertService.override(id, reason, comment, context.userId);
    }
  };

  Subscription = {
    alertStream: {
      subscribe: async function* (
        _: unknown,
        { patientId, encounterId }: { patientId?: string; encounterId?: string },
        context: GraphQLContext
      ) {
        const stream = this.alertService.getAlertStream(patientId, encounterId);
        for await (const alert of stream) {
          yield { alertStream: alert };
        }
      }
    }
  };
}
```

### 4.3 실시간 WebSocket API

```typescript
// WebSocket 기반 실시간 CDSS 스트리밍
interface CDSSWebSocketAPI {
  endpoint: 'wss://cdss.example.com/ws';

  protocols: {
    realTimeAlerts: RealTimeAlertProtocol;
    patientMonitoring: PatientMonitoringProtocol;
    earlyWarning: EarlyWarningProtocol;
  };
}

// 조기경보시스템 WebSocket 서비스
class EarlyWarningWebSocketService {
  private connections: Map<string, WebSocket> = new Map();
  private ewsCalculator: EarlyWarningScoreCalculator;
  private alertPublisher: AlertPublisher;

  async handleConnection(ws: WebSocket, request: IncomingMessage): Promise<void> {
    const { patientId, encounterId } = this.parseConnectionParams(request);

    // 인증
    const auth = await this.authenticate(request);
    if (!auth.valid) {
      ws.close(4001, '인증 실패');
      return;
    }

    // 연결 등록
    const connectionId = generateUUID();
    this.connections.set(connectionId, ws);

    // 환자 모니터링 구독
    const subscription = this.subscribeToPatientData(patientId, encounterId);

    ws.on('message', async (message) => {
      await this.handleMessage(ws, message, patientId);
    });

    ws.on('close', () => {
      this.connections.delete(connectionId);
      subscription.unsubscribe();
    });

    // 초기 상태 전송
    const initialState = await this.getPatientState(patientId, encounterId);
    ws.send(JSON.stringify({
      type: 'INITIAL_STATE',
      data: initialState
    }));
  }

  private async handleMessage(
    ws: WebSocket,
    message: WebSocket.Data,
    patientId: string
  ): Promise<void> {
    const msg = JSON.parse(message.toString()) as WebSocketMessage;

    switch (msg.type) {
      case 'VITAL_UPDATE':
        await this.processVitalUpdate(ws, patientId, msg.data);
        break;

      case 'SUBSCRIBE_PATIENT':
        await this.subscribeAdditionalPatient(ws, msg.data.patientId);
        break;

      case 'ACKNOWLEDGE_ALERT':
        await this.acknowledgeAlert(msg.data.alertId, msg.data.userId);
        break;

      case 'REQUEST_EWS':
        const ews = await this.calculateEWS(patientId);
        ws.send(JSON.stringify({
          type: 'EWS_RESULT',
          data: ews
        }));
        break;
    }
  }

  private async processVitalUpdate(
    ws: WebSocket,
    patientId: string,
    vitals: VitalSignData
  ): Promise<void> {
    // NEWS2 점수 계산
    const ewsScore = await this.ewsCalculator.calculate(patientId, vitals);

    // 임계값 확인
    if (ewsScore.total >= 7) {
      // 위험 경보
      const alert = await this.createCriticalAlert(patientId, ewsScore);
      this.broadcastAlert(patientId, alert);

    } else if (ewsScore.total >= 5) {
      // 경고
      const alert = await this.createWarningAlert(patientId, ewsScore);
      this.broadcastAlert(patientId, alert);
    }

    // EWS 업데이트 전송
    ws.send(JSON.stringify({
      type: 'EWS_UPDATE',
      data: {
        patientId,
        ewsScore,
        timestamp: new Date().toISOString(),
        trend: ewsScore.trend,
        recommendation: this.getEWSRecommendation(ewsScore)
      }
    }));
  }

  private getEWSRecommendation(ewsScore: EWSScore): EWSRecommendation {
    if (ewsScore.total >= 7) {
      return {
        clinicalResponse: 'EMERGENCY',
        frequency: 'CONTINUOUS',
        escalation: '즉시 RRT 호출, 담당의에게 긴급 연락',
        interventions: [
          '산소 투여 고려',
          '대용량 정맥 접근 확보',
          '전해질 검사 포함 검사 오더',
          '흉부 X-ray 고려'
        ]
      };
    } else if (ewsScore.total >= 5) {
      return {
        clinicalResponse: 'URGENT',
        frequency: 'HOURLY',
        escalation: '담당의에게 긴급 연락, 고위험 환자 라운딩 추가',
        interventions: [
          '활력징후 모니터링 빈도 증가',
          '담당의 임상 평가'
        ]
      };
    } else if (ewsScore.total >= 3) {
      return {
        clinicalResponse: 'MEDIUM',
        frequency: '4시간마다',
        escalation: '담당 간호사/의사에게 알림',
        interventions: [
          '환자 평가',
          '케어 계획 검토'
        ]
      };
    } else {
      return {
        clinicalResponse: 'LOW',
        frequency: '12시간마다',
        escalation: '해당 없음',
        interventions: ['일상적인 모니터링 계속']
      };
    }
  }
}

// NEWS2 점수 계산기
class EarlyWarningScoreCalculator {
  async calculate(patientId: string, vitals: VitalSignData): Promise<EWSScore> {
    const scores: ComponentScore[] = [];

    // 호흡수
    scores.push(this.scoreRespiratoryRate(vitals.respiratoryRate));

    // 산소 포화도
    scores.push(this.scoreSpO2(vitals.spO2, vitals.onOxygen));

    // 보조 산소
    if (vitals.onOxygen) {
      scores.push({ component: 'airOrOxygen', score: 2 });
    } else {
      scores.push({ component: 'airOrOxygen', score: 0 });
    }

    // 체온
    scores.push(this.scoreTemperature(vitals.temperature));

    // 수축기 혈압
    scores.push(this.scoreSystolicBP(vitals.systolicBP));

    // 심박수
    scores.push(this.scoreHeartRate(vitals.heartRate));

    // 의식 수준
    scores.push(this.scoreConsciousness(vitals.consciousness));

    const total = scores.reduce((sum, s) => sum + s.score, 0);

    // 이전 점수 가져오기
    const previousScores = await this.getPreviousScores(patientId);
    const trend = this.calculateTrend(total, previousScores);

    return {
      patientId,
      timestamp: new Date(),
      total,
      components: scores,
      trend,
      riskLevel: this.getRiskLevel(total, scores)
    };
  }

  private scoreRespiratoryRate(rate: number): ComponentScore {
    if (rate <= 8) return { component: 'respiratoryRate', score: 3, value: rate };
    if (rate <= 11) return { component: 'respiratoryRate', score: 1, value: rate };
    if (rate <= 20) return { component: 'respiratoryRate', score: 0, value: rate };
    if (rate <= 24) return { component: 'respiratoryRate', score: 2, value: rate };
    return { component: 'respiratoryRate', score: 3, value: rate };
  }

  private scoreSpO2(spO2: number, onOxygen: boolean): ComponentScore {
    // Scale 1 (정상 목표) 사용
    if (spO2 <= 91) return { component: 'spO2', score: 3, value: spO2 };
    if (spO2 <= 93) return { component: 'spO2', score: 2, value: spO2 };
    if (spO2 <= 95) return { component: 'spO2', score: 1, value: spO2 };
    return { component: 'spO2', score: 0, value: spO2 };
  }

  private scoreTemperature(temp: number): ComponentScore {
    if (temp <= 35.0) return { component: 'temperature', score: 3, value: temp };
    if (temp <= 36.0) return { component: 'temperature', score: 1, value: temp };
    if (temp <= 38.0) return { component: 'temperature', score: 0, value: temp };
    if (temp <= 39.0) return { component: 'temperature', score: 1, value: temp };
    return { component: 'temperature', score: 2, value: temp };
  }

  private scoreSystolicBP(sbp: number): ComponentScore {
    if (sbp <= 90) return { component: 'systolicBP', score: 3, value: sbp };
    if (sbp <= 100) return { component: 'systolicBP', score: 2, value: sbp };
    if (sbp <= 110) return { component: 'systolicBP', score: 1, value: sbp };
    if (sbp <= 219) return { component: 'systolicBP', score: 0, value: sbp };
    return { component: 'systolicBP', score: 3, value: sbp };
  }

  private scoreHeartRate(hr: number): ComponentScore {
    if (hr <= 40) return { component: 'heartRate', score: 3, value: hr };
    if (hr <= 50) return { component: 'heartRate', score: 1, value: hr };
    if (hr <= 90) return { component: 'heartRate', score: 0, value: hr };
    if (hr <= 110) return { component: 'heartRate', score: 1, value: hr };
    if (hr <= 130) return { component: 'heartRate', score: 2, value: hr };
    return { component: 'heartRate', score: 3, value: hr };
  }

  private scoreConsciousness(level: ConsciousnessLevel): ComponentScore {
    if (level === 'ALERT') {
      return { component: 'consciousness', score: 0, value: level };
    }
    return { component: 'consciousness', score: 3, value: level };
  }

  private getRiskLevel(
    total: number,
    components: ComponentScore[]
  ): RiskLevel {
    // 단일 파라미터 3점 확인
    const hasExtreme = components.some(c => c.score === 3);

    if (total >= 7) return 'HIGH';
    if (total >= 5 || hasExtreme) return 'MEDIUM';
    if (total >= 1) return 'LOW';
    return 'NONE';
  }
}
```

### 4.4 오류 처리 및 응답 형식

```typescript
// CDSS API 오류 처리
interface CDSSAPIError {
  error: {
    code: CDSSErrorCode;
    message: string;
    details?: Record<string, unknown>;
    timestamp: Date;
    requestId: string;
    path: string;
  };
}

type CDSSErrorCode =
  | 'AUTHENTICATION_FAILED'
  | 'AUTHORIZATION_DENIED'
  | 'PATIENT_NOT_FOUND'
  | 'INSUFFICIENT_DATA'
  | 'INVALID_REQUEST'
  | 'KNOWLEDGE_NOT_FOUND'
  | 'CALCULATOR_ERROR'
  | 'INFERENCE_ERROR'
  | 'TIMEOUT'
  | 'RATE_LIMITED'
  | 'SERVICE_UNAVAILABLE'
  | 'INTERNAL_ERROR';

// 오류 처리기
class CDSSErrorHandler {
  handleError(error: Error, context: RequestContext): CDSSAPIError {
    const requestId = context.requestId || generateUUID();

    if (error instanceof PatientNotFoundError) {
      return {
        error: {
          code: 'PATIENT_NOT_FOUND',
          message: `환자 ${error.patientId}를 찾을 수 없습니다`,
          details: { patientId: error.patientId },
          timestamp: new Date(),
          requestId,
          path: context.path
        }
      };
    }

    if (error instanceof InsufficientDataError) {
      return {
        error: {
          code: 'INSUFFICIENT_DATA',
          message: '권장사항 생성에 필요한 데이터가 부족합니다',
          details: {
            missingData: error.missingData,
            minimumRequired: error.minimumRequired
          },
          timestamp: new Date(),
          requestId,
          path: context.path
        }
      };
    }

    if (error instanceof InferenceTimeoutError) {
      return {
        error: {
          code: 'TIMEOUT',
          message: '추론 엔진 타임아웃',
          details: {
            timeout: error.timeout,
            partialResults: error.partialResults
          },
          timestamp: new Date(),
          requestId,
          path: context.path
        }
      };
    }

    // 기본 내부 오류
    console.error('CDSS 내부 오류:', error);
    return {
      error: {
        code: 'INTERNAL_ERROR',
        message: '내부 서버 오류가 발생했습니다',
        timestamp: new Date(),
        requestId,
        path: context.path
      }
    };
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT API 인터페이스**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
