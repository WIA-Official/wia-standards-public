# 제5장: 인증 프로토콜

## 개요

냉동보존 분야의 신원 인증은 다층적인 프로토콜을 통해 정확한 대상자 식별을 보장해야 합니다. 이 장에서는 초기 등록부터 재인증까지 포괄적인 인증 방법론을 상세히 다룹니다.

## 인증 아키텍처

### 핵심 인증 시스템

```typescript
import { EventEmitter } from 'events';

// 인증 방법 유형
type VerificationMethod =
  | 'document'
  | 'biometric'
  | 'knowledge'
  | 'possession'
  | 'third-party';

type VerificationStatus =
  | 'pending'
  | 'in-progress'
  | 'completed'
  | 'failed'
  | 'expired';

type VerificationLevel =
  | 'basic'
  | 'standard'
  | 'enhanced'
  | 'maximum';

// 인증 결과 인터페이스
interface VerificationResult {
  id: string;
  sessionId: string;
  subjectId: string;
  method: VerificationMethod;
  status: VerificationStatus;
  confidence: number;
  timestamp: Date;
  expiresAt: Date;
  evidence: VerificationEvidence[];
  reviewer?: ReviewerInfo;
  metadata: Record<string, unknown>;
}

interface VerificationEvidence {
  type: string;
  source: string;
  capturedAt: Date;
  hash: string;
  encryptedData?: Buffer;
}

interface ReviewerInfo {
  id: string;
  name: string;
  role: string;
  reviewedAt: Date;
  decision: 'approved' | 'rejected' | 'escalated';
  notes?: string;
}

// 인증 세션 관리
interface VerificationSession {
  id: string;
  subjectId: string;
  requestedLevel: VerificationLevel;
  requiredMethods: VerificationMethod[];
  completedMethods: VerificationMethod[];
  results: VerificationResult[];
  status: VerificationStatus;
  createdAt: Date;
  updatedAt: Date;
  expiresAt: Date;
  context: VerificationContext;
}

interface VerificationContext {
  purpose: string;
  initiator: string;
  facility: string;
  specimenIds?: string[];
  operationType?: string;
  riskLevel: 'low' | 'medium' | 'high' | 'critical';
}

// 메인 인증 엔진
class VerificationEngine extends EventEmitter {
  private sessions: Map<string, VerificationSession> = new Map();
  private verifiers: Map<VerificationMethod, MethodVerifier> = new Map();
  private policies: VerificationPolicyEngine;
  private auditLogger: AuditLogger;

  constructor(
    private config: VerificationConfig,
    private subjectRepository: SubjectRepository,
    private evidenceStore: EvidenceStore
  ) {
    super();
    this.policies = new VerificationPolicyEngine(config.policies);
    this.auditLogger = new AuditLogger(config.audit);
    this.initializeVerifiers();
  }

  private initializeVerifiers(): void {
    this.verifiers.set('document', new DocumentVerifier(this.config.document));
    this.verifiers.set('biometric', new BiometricVerifier(this.config.biometric));
    this.verifiers.set('knowledge', new KnowledgeVerifier(this.config.knowledge));
    this.verifiers.set('possession', new PossessionVerifier(this.config.possession));
    this.verifiers.set('third-party', new ThirdPartyVerifier(this.config.thirdParty));
  }

  async createSession(
    subjectId: string,
    context: VerificationContext
  ): Promise<VerificationSession> {
    const subject = await this.subjectRepository.findById(subjectId);
    if (!subject) {
      throw new VerificationError('SUBJECT_NOT_FOUND', `대상자를 찾을 수 없습니다: ${subjectId}`);
    }

    const requiredLevel = this.policies.determineRequiredLevel(context);
    const requiredMethods = this.policies.getRequiredMethods(requiredLevel, subject.type);

    const session: VerificationSession = {
      id: this.generateSessionId(),
      subjectId,
      requestedLevel: requiredLevel,
      requiredMethods,
      completedMethods: [],
      results: [],
      status: 'pending',
      createdAt: new Date(),
      updatedAt: new Date(),
      expiresAt: this.calculateExpiration(requiredLevel),
      context
    };

    this.sessions.set(session.id, session);

    await this.auditLogger.log({
      action: 'VERIFICATION_SESSION_CREATED',
      sessionId: session.id,
      subjectId,
      context,
      requiredLevel,
      requiredMethods
    });

    this.emit('sessionCreated', session);
    return session;
  }

  async executeVerification(
    sessionId: string,
    method: VerificationMethod,
    input: VerificationInput
  ): Promise<VerificationResult> {
    const session = this.sessions.get(sessionId);
    if (!session) {
      throw new VerificationError('SESSION_NOT_FOUND', `세션을 찾을 수 없습니다: ${sessionId}`);
    }

    if (session.status === 'expired' || new Date() > session.expiresAt) {
      session.status = 'expired';
      throw new VerificationError('SESSION_EXPIRED', '인증 세션이 만료되었습니다');
    }

    if (!session.requiredMethods.includes(method)) {
      throw new VerificationError('METHOD_NOT_REQUIRED', `이 세션에서 요구하지 않는 방법입니다: ${method}`);
    }

    if (session.completedMethods.includes(method)) {
      throw new VerificationError('METHOD_COMPLETED', `이미 완료된 방법입니다: ${method}`);
    }

    const verifier = this.verifiers.get(method);
    if (!verifier) {
      throw new VerificationError('VERIFIER_NOT_FOUND', `인증기를 찾을 수 없습니다: ${method}`);
    }

    session.status = 'in-progress';
    session.updatedAt = new Date();

    try {
      const result = await verifier.verify(session.subjectId, input);

      // 증거 안전하게 저장
      for (const evidence of result.evidence) {
        await this.evidenceStore.store(session.id, evidence);
      }

      session.results.push(result);

      if (result.status === 'completed' && result.confidence >= this.config.thresholds[method]) {
        session.completedMethods.push(method);
      }

      // 모든 필수 방법 완료 여부 확인
      if (this.isSessionComplete(session)) {
        session.status = 'completed';
        this.emit('sessionCompleted', session);
      } else if (result.status === 'failed') {
        session.status = 'failed';
        this.emit('sessionFailed', session, result);
      }

      session.updatedAt = new Date();

      await this.auditLogger.log({
        action: 'VERIFICATION_EXECUTED',
        sessionId,
        method,
        result: {
          status: result.status,
          confidence: result.confidence
        }
      });

      this.emit('verificationResult', session, result);
      return result;
    } catch (error) {
      session.status = 'failed';
      session.updatedAt = new Date();

      await this.auditLogger.log({
        action: 'VERIFICATION_ERROR',
        sessionId,
        method,
        error: error instanceof Error ? error.message : '알 수 없는 오류'
      });

      throw error;
    }
  }

  private isSessionComplete(session: VerificationSession): boolean {
    return session.requiredMethods.every(method =>
      session.completedMethods.includes(method)
    );
  }

  private generateSessionId(): string {
    return `VS-${Date.now()}-${Math.random().toString(36).substring(2, 11)}`;
  }

  private calculateExpiration(level: VerificationLevel): Date {
    const durations: Record<VerificationLevel, number> = {
      basic: 30 * 60 * 1000,      // 30분
      standard: 60 * 60 * 1000,   // 1시간
      enhanced: 120 * 60 * 1000,  // 2시간
      maximum: 240 * 60 * 1000    // 4시간
    };
    return new Date(Date.now() + durations[level]);
  }

  async getSession(sessionId: string): Promise<VerificationSession | null> {
    return this.sessions.get(sessionId) || null;
  }

  async cancelSession(sessionId: string, reason: string): Promise<void> {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    session.status = 'failed';
    session.updatedAt = new Date();

    await this.auditLogger.log({
      action: 'VERIFICATION_SESSION_CANCELLED',
      sessionId,
      reason
    });

    this.emit('sessionCancelled', session, reason);
  }
}
```

### 문서 인증

```typescript
// 문서 인증 구현
interface DocumentVerifierConfig {
  supportedDocuments: DocumentType[];
  ocrProvider: string;
  validationRules: DocumentValidationRule[];
  expirationBuffer: number; // 만료 전 경고 일수
}

type DocumentType =
  | 'passport'        // 여권
  | 'national-id'     // 주민등록증
  | 'drivers-license' // 운전면허증
  | 'birth-certificate' // 출생증명서
  | 'medical-id'      // 의료 ID
  | 'donor-card';     // 기증자 카드

interface DocumentValidationRule {
  documentType: DocumentType;
  requiredFields: string[];
  formatValidation: Record<string, RegExp>;
  crossReferenceChecks: string[];
}

interface DocumentInput {
  documentType: DocumentType;
  frontImage: Buffer;
  backImage?: Buffer;
  metadata?: Record<string, string>;
}

interface ExtractedDocumentData {
  documentNumber: string;
  documentType: DocumentType;
  issuingCountry: string;
  issuingAuthority?: string;
  issueDate: Date;
  expirationDate?: Date;
  holderName: {
    firstName: string;
    lastName: string;
    middleName?: string;
  };
  dateOfBirth: Date;
  nationality?: string;
  gender?: string;
  address?: string;
  photoHash?: string;
  mrzData?: string;
  rawFields: Record<string, string>;
}

class DocumentVerifier implements MethodVerifier {
  private ocrService: OCRService;
  private validationService: DocumentValidationService;
  private fraudDetector: FraudDetectionService;

  constructor(private config: DocumentVerifierConfig) {
    this.ocrService = new OCRService(config.ocrProvider);
    this.validationService = new DocumentValidationService(config.validationRules);
    this.fraudDetector = new FraudDetectionService();
  }

  async verify(subjectId: string, input: VerificationInput): Promise<VerificationResult> {
    const docInput = input as DocumentInput;
    const startTime = Date.now();

    try {
      // 1단계: OCR을 사용하여 데이터 추출
      const extractedData = await this.extractDocumentData(docInput);

      // 2단계: 문서 진위 검증
      const authenticityScore = await this.validateAuthenticity(docInput, extractedData);

      // 3단계: 사기 지표 확인
      const fraudCheck = await this.fraudDetector.analyze({
        images: [docInput.frontImage, docInput.backImage].filter(Boolean) as Buffer[],
        extractedData
      });

      // 4단계: 대상자 데이터와 교차 검증
      const crossRefScore = await this.crossReferenceWithSubject(subjectId, extractedData);

      // 5단계: 문서 만료 확인
      const expirationStatus = this.checkExpiration(extractedData);

      // 종합 신뢰도 계산
      const confidence = this.calculateConfidence({
        authenticityScore,
        fraudScore: 1 - fraudCheck.riskScore,
        crossRefScore,
        expirationPenalty: expirationStatus.penalty
      });

      // 증거 생성
      const evidence: VerificationEvidence[] = [
        {
          type: 'document-image',
          source: 'front',
          capturedAt: new Date(),
          hash: await this.hashImage(docInput.frontImage),
          encryptedData: await this.encryptEvidence(docInput.frontImage)
        },
        {
          type: 'extracted-data',
          source: 'ocr',
          capturedAt: new Date(),
          hash: await this.hashData(extractedData)
        }
      ];

      if (docInput.backImage) {
        evidence.push({
          type: 'document-image',
          source: 'back',
          capturedAt: new Date(),
          hash: await this.hashImage(docInput.backImage),
          encryptedData: await this.encryptEvidence(docInput.backImage)
        });
      }

      const status: VerificationStatus = confidence >= 0.8 ? 'completed' : 'failed';

      return {
        id: `DR-${Date.now()}`,
        sessionId: '',
        subjectId,
        method: 'document',
        status,
        confidence,
        timestamp: new Date(),
        expiresAt: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000), // 1년
        evidence,
        metadata: {
          documentType: docInput.documentType,
          processingTime: Date.now() - startTime,
          extractedFields: Object.keys(extractedData.rawFields),
          authenticityScore,
          fraudRisk: fraudCheck.riskScore,
          expirationWarning: expirationStatus.warning
        }
      };
    } catch (error) {
      return {
        id: `DR-${Date.now()}`,
        sessionId: '',
        subjectId,
        method: 'document',
        status: 'failed',
        confidence: 0,
        timestamp: new Date(),
        expiresAt: new Date(),
        evidence: [],
        metadata: {
          error: error instanceof Error ? error.message : '알 수 없는 오류',
          processingTime: Date.now() - startTime
        }
      };
    }
  }

  private async extractDocumentData(input: DocumentInput): Promise<ExtractedDocumentData> {
    const ocrResult = await this.ocrService.processDocument({
      frontImage: input.frontImage,
      backImage: input.backImage,
      documentType: input.documentType
    });

    return {
      documentNumber: ocrResult.fields.documentNumber,
      documentType: input.documentType,
      issuingCountry: ocrResult.fields.issuingCountry,
      issuingAuthority: ocrResult.fields.issuingAuthority,
      issueDate: new Date(ocrResult.fields.issueDate),
      expirationDate: ocrResult.fields.expirationDate
        ? new Date(ocrResult.fields.expirationDate)
        : undefined,
      holderName: {
        firstName: ocrResult.fields.firstName,
        lastName: ocrResult.fields.lastName,
        middleName: ocrResult.fields.middleName
      },
      dateOfBirth: new Date(ocrResult.fields.dateOfBirth),
      nationality: ocrResult.fields.nationality,
      gender: ocrResult.fields.gender,
      address: ocrResult.fields.address,
      photoHash: ocrResult.photoHash,
      mrzData: ocrResult.mrzData,
      rawFields: ocrResult.fields
    };
  }

  private checkExpiration(data: ExtractedDocumentData): { penalty: number; warning?: string } {
    if (!data.expirationDate) {
      return { penalty: 0 };
    }

    const now = new Date();
    const daysUntilExpiration = Math.floor(
      (data.expirationDate.getTime() - now.getTime()) / (1000 * 60 * 60 * 24)
    );

    if (daysUntilExpiration < 0) {
      return { penalty: 1, warning: '문서가 만료되었습니다' };
    }

    if (daysUntilExpiration < this.config.expirationBuffer) {
      return {
        penalty: 0.1,
        warning: `문서가 ${daysUntilExpiration}일 후 만료됩니다`
      };
    }

    return { penalty: 0 };
  }

  private calculateConfidence(scores: {
    authenticityScore: number;
    fraudScore: number;
    crossRefScore: number;
    expirationPenalty: number;
  }): number {
    const weights = {
      authenticity: 0.3,
      fraud: 0.3,
      crossRef: 0.4
    };

    const baseScore =
      scores.authenticityScore * weights.authenticity +
      scores.fraudScore * weights.fraud +
      scores.crossRefScore * weights.crossRef;

    return Math.max(0, baseScore - scores.expirationPenalty);
  }

  private async hashImage(image: Buffer): Promise<string> {
    const crypto = await import('crypto');
    return crypto.createHash('sha256').update(image).digest('hex');
  }

  private async hashData(data: unknown): Promise<string> {
    const crypto = await import('crypto');
    return crypto.createHash('sha256').update(JSON.stringify(data)).digest('hex');
  }

  private async encryptEvidence(data: Buffer): Promise<Buffer> {
    // 실제 구현에서는 적절한 암호화 사용
    return data;
  }

  private async validateAuthenticity(
    input: DocumentInput,
    data: ExtractedDocumentData
  ): Promise<number> {
    return 0.95; // 간소화된 구현
  }

  private async crossReferenceWithSubject(
    subjectId: string,
    data: ExtractedDocumentData
  ): Promise<number> {
    return 0.95; // 간소화된 구현
  }
}
```

### 생체정보 인증

```typescript
// 생체정보 인증 구현
interface BiometricVerifierConfig {
  supportedModalities: BiometricModality[];
  matchingThresholds: Record<BiometricModality, number>;
  livenessDetection: LivenessConfig;
  qualityRequirements: QualityConfig;
}

type BiometricModality =
  | 'fingerprint'  // 지문
  | 'facial'       // 얼굴
  | 'iris'         // 홍채
  | 'voice'        // 음성
  | 'palm';        // 손바닥

interface LivenessConfig {
  enabled: boolean;
  challenges: LivenessChallenge[];
  minimumScore: number;
}

type LivenessChallenge =
  | 'blink'           // 눈 깜빡임
  | 'smile'           // 미소
  | 'turn-head'       // 머리 돌리기
  | 'random-digits'   // 랜덤 숫자 읽기
  | 'light-reflection'; // 빛 반사

interface BiometricInput {
  modality: BiometricModality;
  samples: BiometricSample[];
  livenessData?: LivenessData;
}

interface BiometricSample {
  data: Buffer;
  capturedAt: Date;
  deviceInfo: DeviceInfo;
  quality?: number;
}

interface DeviceInfo {
  type: string;
  manufacturer: string;
  model: string;
  certifications: string[];
}

class BiometricVerifier implements MethodVerifier {
  private modalityEngines: Map<BiometricModality, ModalityEngine> = new Map();
  private livenessDetector: LivenessDetector;
  private qualityAssessor: QualityAssessor;
  private templateStore: BiometricTemplateStore;

  constructor(private config: BiometricVerifierConfig) {
    this.initializeEngines();
    this.livenessDetector = new LivenessDetector(config.livenessDetection);
    this.qualityAssessor = new QualityAssessor(config.qualityRequirements);
    this.templateStore = new BiometricTemplateStore();
  }

  private initializeEngines(): void {
    for (const modality of this.config.supportedModalities) {
      this.modalityEngines.set(modality, this.createEngine(modality));
    }
  }

  private createEngine(modality: BiometricModality): ModalityEngine {
    switch (modality) {
      case 'fingerprint':
        return new FingerprintEngine();
      case 'facial':
        return new FacialRecognitionEngine();
      case 'iris':
        return new IrisRecognitionEngine();
      case 'voice':
        return new VoiceRecognitionEngine();
      case 'palm':
        return new PalmRecognitionEngine();
      default:
        throw new Error(`지원하지 않는 양식: ${modality}`);
    }
  }

  async verify(subjectId: string, input: VerificationInput): Promise<VerificationResult> {
    const bioInput = input as BiometricInput;
    const startTime = Date.now();

    try {
      // 1단계: 샘플 품질 확인
      const qualityResults = await this.assessQuality(bioInput);
      if (!qualityResults.acceptable) {
        return this.createFailedResult(subjectId, bioInput.modality,
          '샘플 품질이 기준 미달입니다', startTime);
      }

      // 2단계: 생체 인식 수행
      if (this.config.livenessDetection.enabled) {
        const livenessResult = await this.checkLiveness(bioInput);
        if (!livenessResult.isLive) {
          return this.createFailedResult(subjectId, bioInput.modality,
            '생체 인식 검사 실패', startTime);
        }
      }

      // 3단계: 생체 템플릿 추출
      const engine = this.modalityEngines.get(bioInput.modality);
      if (!engine) {
        throw new Error(`엔진을 찾을 수 없습니다: ${bioInput.modality}`);
      }

      const capturedTemplate = await engine.extractTemplate(bioInput.samples);

      // 4단계: 등록된 템플릿 조회
      const enrolledTemplate = await this.templateStore.getTemplate(
        subjectId,
        bioInput.modality
      );

      if (!enrolledTemplate) {
        return this.createFailedResult(subjectId, bioInput.modality,
          '등록된 템플릿을 찾을 수 없습니다', startTime);
      }

      // 5단계: 매칭 수행
      const matchResult = await engine.match(capturedTemplate, enrolledTemplate);
      const threshold = this.config.matchingThresholds[bioInput.modality];
      const isMatch = matchResult.score >= threshold;

      // 증거 생성
      const evidence: VerificationEvidence[] = [
        {
          type: 'biometric-template',
          source: bioInput.modality,
          capturedAt: new Date(),
          hash: await this.hashTemplate(capturedTemplate)
        },
        {
          type: 'match-result',
          source: 'verification-engine',
          capturedAt: new Date(),
          hash: await this.hashData(matchResult)
        }
      ];

      return {
        id: `BR-${Date.now()}`,
        sessionId: '',
        subjectId,
        method: 'biometric',
        status: isMatch ? 'completed' : 'failed',
        confidence: matchResult.score,
        timestamp: new Date(),
        expiresAt: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000), // 30일
        evidence,
        metadata: {
          modality: bioInput.modality,
          matchScore: matchResult.score,
          threshold,
          qualityScore: qualityResults.score,
          processingTime: Date.now() - startTime
        }
      };
    } catch (error) {
      return this.createFailedResult(subjectId, bioInput.modality,
        error instanceof Error ? error.message : '알 수 없는 오류', startTime);
    }
  }

  private async assessQuality(input: BiometricInput): Promise<QualityResult> {
    const scores: number[] = [];

    for (const sample of input.samples) {
      const score = await this.qualityAssessor.assess(sample, input.modality);
      scores.push(score);
    }

    const averageScore = scores.reduce((a, b) => a + b, 0) / scores.length;
    return {
      acceptable: averageScore >= this.config.qualityRequirements.minimumScore,
      score: averageScore,
      details: scores.map((score, i) => ({ sampleIndex: i, score }))
    };
  }

  private async checkLiveness(input: BiometricInput): Promise<LivenessResult> {
    if (!input.livenessData) {
      return { isLive: false, score: 0 };
    }

    return this.livenessDetector.detect(input.livenessData);
  }

  private createFailedResult(
    subjectId: string,
    modality: BiometricModality,
    reason: string,
    startTime: number
  ): VerificationResult {
    return {
      id: `BR-${Date.now()}`,
      sessionId: '',
      subjectId,
      method: 'biometric',
      status: 'failed',
      confidence: 0,
      timestamp: new Date(),
      expiresAt: new Date(),
      evidence: [],
      metadata: {
        modality,
        failureReason: reason,
        processingTime: Date.now() - startTime
      }
    };
  }

  private async hashTemplate(template: BiometricTemplate): Promise<string> {
    const crypto = await import('crypto');
    return crypto.createHash('sha256').update(template.data).digest('hex');
  }

  private async hashData(data: unknown): Promise<string> {
    const crypto = await import('crypto');
    return crypto.createHash('sha256').update(JSON.stringify(data)).digest('hex');
  }
}

interface QualityResult {
  acceptable: boolean;
  score: number;
  details: { sampleIndex: number; score: number }[];
}

interface LivenessResult {
  isLive: boolean;
  score: number;
  challenges?: { type: LivenessChallenge; passed: boolean }[];
}

interface BiometricTemplate {
  data: Buffer;
  modality: BiometricModality;
  version: string;
  createdAt: Date;
}

interface MatchResult {
  score: number;
  matchedFeatures: number;
  totalFeatures: number;
}
```

### 지식 기반 인증

```typescript
// 지식 기반 인증 (보안 질문)
interface KnowledgeVerifierConfig {
  questionSets: QuestionSet[];
  minimumCorrectAnswers: number;
  maxAttempts: number;
  lockoutDuration: number; // 분
  answerFuzzyMatchThreshold: number;
}

interface QuestionSet {
  id: string;
  name: string;
  category: 'personal' | 'medical' | 'historical' | 'custom';
  questions: KnowledgeQuestion[];
  difficulty: 'easy' | 'medium' | 'hard';
}

interface KnowledgeQuestion {
  id: string;
  text: string;
  textKo?: string; // 한국어 질문
  type: 'text' | 'date' | 'numeric' | 'multiple-choice';
  options?: string[];
  hints?: string[];
  weight: number;
}

interface KnowledgeInput {
  questionId: string;
  answer: string | number | Date;
  timestamp: Date;
}

class KnowledgeVerifier implements MethodVerifier {
  private challengeSessions: Map<string, KnowledgeChallengeSession> = new Map();
  private knowledgeStore: SubjectKnowledgeStore;
  private fuzzyMatcher: FuzzyMatcher;

  constructor(private config: KnowledgeVerifierConfig) {
    this.knowledgeStore = new SubjectKnowledgeStore();
    this.fuzzyMatcher = new FuzzyMatcher(config.answerFuzzyMatchThreshold);
  }

  async verify(subjectId: string, input: VerificationInput): Promise<VerificationResult> {
    const knowledgeInput = input as KnowledgeInput;
    const startTime = Date.now();

    // 챌린지 세션 조회 또는 생성
    let session = this.getChallengeSession(subjectId);
    if (!session) {
      session = await this.createChallengeSession(subjectId);
    }

    // 잠금 상태 확인
    if (session.lockedUntil && new Date() < session.lockedUntil) {
      const remainingMinutes = Math.ceil(
        (session.lockedUntil.getTime() - Date.now()) / (1000 * 60)
      );
      return this.createFailedResult(subjectId,
        `계정이 잠겼습니다. ${remainingMinutes}분 후에 다시 시도하세요.`, startTime);
    }

    // 답변 중인 질문 찾기
    const question = session.questions.find(q => q.questionId === knowledgeInput.questionId);
    if (!question) {
      return this.createFailedResult(subjectId, '유효하지 않은 질문 ID입니다', startTime);
    }

    // 답변 확인
    const matchResult = this.checkAnswer(question, knowledgeInput);

    session.answers.push({
      questionId: knowledgeInput.questionId,
      providedAnswer: knowledgeInput.answer,
      isCorrect: matchResult.isCorrect,
      matchScore: matchResult.score,
      answeredAt: new Date()
    });

    // 모든 질문에 답변했는지 확인
    if (session.answers.length >= session.questions.length) {
      session.completedAt = new Date();

      const correctCount = session.answers.filter(a => a.isCorrect).length;
      const passed = correctCount >= this.config.minimumCorrectAnswers;

      if (!passed) {
        session.attempts++;
        if (session.attempts >= this.config.maxAttempts) {
          session.lockedUntil = new Date(
            Date.now() + this.config.lockoutDuration * 60 * 1000
          );
        }
      }

      const confidence = correctCount / session.questions.length;

      return {
        id: `KR-${Date.now()}`,
        sessionId: '',
        subjectId,
        method: 'knowledge',
        status: passed ? 'completed' : 'failed',
        confidence,
        timestamp: new Date(),
        expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24시간
        evidence: [{
          type: 'knowledge-challenge',
          source: 'challenge-session',
          capturedAt: new Date(),
          hash: await this.hashSession(session)
        }],
        metadata: {
          questionsAsked: session.questions.length,
          correctAnswers: correctCount,
          attempts: session.attempts,
          processingTime: Date.now() - startTime
        }
      };
    }

    // 아직 더 많은 질문에 답변해야 함
    return {
      id: `KR-${Date.now()}`,
      sessionId: '',
      subjectId,
      method: 'knowledge',
      status: 'pending',
      confidence: 0,
      timestamp: new Date(),
      expiresAt: new Date(Date.now() + 60 * 60 * 1000), // 1시간
      evidence: [],
      metadata: {
        questionsAnswered: session.answers.length,
        questionsRemaining: session.questions.length - session.answers.length,
        lastAnswerCorrect: matchResult.isCorrect,
        processingTime: Date.now() - startTime
      }
    };
  }

  private async createChallengeSession(subjectId: string): Promise<KnowledgeChallengeSession> {
    const storedKnowledge = await this.knowledgeStore.getForSubject(subjectId);

    // 저장된 지식 기반으로 질문 선택
    const selectedQuestions = this.selectQuestions(storedKnowledge);

    const session: KnowledgeChallengeSession = {
      id: `KCS-${Date.now()}`,
      subjectId,
      questions: selectedQuestions,
      answers: [],
      attempts: 0,
      startedAt: new Date()
    };

    this.challengeSessions.set(subjectId, session);
    return session;
  }

  private selectQuestions(knowledge: StoredKnowledge): SelectedQuestion[] {
    const selected: SelectedQuestion[] = [];
    const usedCategories = new Set<string>();

    // 다양성을 위해 다른 카테고리에서 질문 선택
    for (const questionSet of this.config.questionSets) {
      if (usedCategories.has(questionSet.category)) continue;

      const availableQuestions = questionSet.questions.filter(q =>
        knowledge.answers.has(q.id)
      );

      if (availableQuestions.length > 0) {
        const randomQuestion = availableQuestions[
          Math.floor(Math.random() * availableQuestions.length)
        ];

        selected.push({
          questionId: randomQuestion.id,
          correctAnswer: knowledge.answers.get(randomQuestion.id)!,
          askedAt: new Date()
        });

        usedCategories.add(questionSet.category);
      }

      if (selected.length >= 5) break; // 최대 5개 질문
    }

    return selected;
  }

  private checkAnswer(
    question: SelectedQuestion,
    input: KnowledgeInput
  ): { isCorrect: boolean; score: number } {
    if (typeof question.correctAnswer === 'string' && typeof input.answer === 'string') {
      const score = this.fuzzyMatcher.compare(
        question.correctAnswer.toLowerCase(),
        input.answer.toLowerCase()
      );
      return {
        isCorrect: score >= this.config.answerFuzzyMatchThreshold,
        score
      };
    }

    if (question.correctAnswer instanceof Date && input.answer instanceof Date) {
      const daysDiff = Math.abs(
        (question.correctAnswer.getTime() - input.answer.getTime()) / (1000 * 60 * 60 * 24)
      );
      const score = daysDiff === 0 ? 1 : daysDiff <= 1 ? 0.9 : 0;
      return { isCorrect: daysDiff <= 1, score };
    }

    const isExactMatch = question.correctAnswer === input.answer;
    return { isCorrect: isExactMatch, score: isExactMatch ? 1 : 0 };
  }

  private getChallengeSession(subjectId: string): KnowledgeChallengeSession | undefined {
    const session = this.challengeSessions.get(subjectId);
    if (session && !session.completedAt) {
      return session;
    }
    return undefined;
  }

  private createFailedResult(
    subjectId: string,
    reason: string,
    startTime: number
  ): VerificationResult {
    return {
      id: `KR-${Date.now()}`,
      sessionId: '',
      subjectId,
      method: 'knowledge',
      status: 'failed',
      confidence: 0,
      timestamp: new Date(),
      expiresAt: new Date(),
      evidence: [],
      metadata: {
        failureReason: reason,
        processingTime: Date.now() - startTime
      }
    };
  }

  private async hashSession(session: KnowledgeChallengeSession): Promise<string> {
    const crypto = await import('crypto');
    const sanitized = {
      ...session,
      questions: session.questions.map(q => ({ ...q, correctAnswer: '[삭제됨]' }))
    };
    return crypto.createHash('sha256').update(JSON.stringify(sanitized)).digest('hex');
  }
}

interface KnowledgeChallengeSession {
  id: string;
  subjectId: string;
  questions: SelectedQuestion[];
  answers: KnowledgeAnswer[];
  attempts: number;
  lockedUntil?: Date;
  startedAt: Date;
  completedAt?: Date;
}

interface SelectedQuestion {
  questionId: string;
  correctAnswer: string | number | Date;
  askedAt: Date;
}

interface KnowledgeAnswer {
  questionId: string;
  providedAnswer: string | number | Date;
  isCorrect: boolean;
  matchScore: number;
  answeredAt: Date;
}

interface StoredKnowledge {
  subjectId: string;
  answers: Map<string, string | number | Date>;
  updatedAt: Date;
}
```

## 재인증 프로토콜

### 정기 재인증

```typescript
// 재인증 스케줄링 및 관리
interface ReVerificationConfig {
  schedules: ReVerificationSchedule[];
  triggerConditions: TriggerCondition[];
  gracePeriod: number; // 예정일 이후 유예 기간 (일)
  escalationRules: EscalationRule[];
}

interface ReVerificationSchedule {
  subjectType: string;
  verificationLevel: VerificationLevel;
  intervalDays: number;
  methods: VerificationMethod[];
}

interface TriggerCondition {
  name: string;
  condition: (subject: Subject, context: any) => boolean;
  resultingLevel: VerificationLevel;
  priority: number;
}

interface EscalationRule {
  daysOverdue: number;
  action: 'reminder' | 'restrict-access' | 'suspend' | 'escalate-review';
  notifyRoles: string[];
}

class ReVerificationManager {
  private scheduler: VerificationScheduler;
  private notificationService: NotificationService;
  private accessController: AccessController;

  constructor(
    private config: ReVerificationConfig,
    private verificationEngine: VerificationEngine,
    private subjectRepository: SubjectRepository
  ) {
    this.scheduler = new VerificationScheduler();
    this.notificationService = new NotificationService();
    this.accessController = new AccessController();
  }

  async scheduleReVerification(
    subjectId: string,
    lastVerification: VerificationResult
  ): Promise<ScheduledVerification> {
    const subject = await this.subjectRepository.findById(subjectId);
    if (!subject) {
      throw new Error(`대상자를 찾을 수 없습니다: ${subjectId}`);
    }

    const schedule = this.findApplicableSchedule(subject, lastVerification);
    const nextDate = new Date(
      lastVerification.timestamp.getTime() + schedule.intervalDays * 24 * 60 * 60 * 1000
    );

    const scheduled: ScheduledVerification = {
      id: `SV-${Date.now()}`,
      subjectId,
      scheduledDate: nextDate,
      dueDate: new Date(nextDate.getTime() + this.config.gracePeriod * 24 * 60 * 60 * 1000),
      requiredLevel: schedule.verificationLevel,
      requiredMethods: schedule.methods,
      status: 'scheduled',
      reminders: this.generateReminderSchedule(nextDate),
      createdAt: new Date()
    };

    await this.scheduler.schedule(scheduled);
    return scheduled;
  }

  private findApplicableSchedule(
    subject: Subject,
    lastVerification: VerificationResult
  ): ReVerificationSchedule {
    const matching = this.config.schedules.find(s =>
      s.subjectType === subject.type
    );

    if (!matching) {
      // 기본 스케줄
      return {
        subjectType: subject.type,
        verificationLevel: 'standard',
        intervalDays: 365,
        methods: ['document', 'biometric']
      };
    }

    return matching;
  }

  private generateReminderSchedule(dueDate: Date): ReminderSchedule[] {
    return [
      { daysBeforeDue: 30, sent: false },
      { daysBeforeDue: 14, sent: false },
      { daysBeforeDue: 7, sent: false },
      { daysBeforeDue: 1, sent: false }
    ];
  }

  async processOverdueVerifications(): Promise<void> {
    const overdue = await this.scheduler.getOverdue();

    for (const scheduled of overdue) {
      const daysOverdue = Math.floor(
        (Date.now() - scheduled.dueDate.getTime()) / (1000 * 60 * 60 * 24)
      );

      for (const rule of this.config.escalationRules) {
        if (daysOverdue >= rule.daysOverdue) {
          await this.executeEscalation(scheduled, rule);
        }
      }
    }
  }

  private async executeEscalation(
    scheduled: ScheduledVerification,
    rule: EscalationRule
  ): Promise<void> {
    switch (rule.action) {
      case 'reminder':
        await this.notificationService.sendReminder(scheduled.subjectId, {
          type: 'reverification-overdue',
          scheduledId: scheduled.id
        });
        break;

      case 'restrict-access':
        await this.accessController.restrictAccess(scheduled.subjectId, {
          reason: 'reverification-overdue',
          level: 'partial'
        });
        break;

      case 'suspend':
        await this.accessController.suspendAccess(scheduled.subjectId, {
          reason: 'reverification-overdue'
        });
        break;

      case 'escalate-review':
        await this.notificationService.escalateToReview(scheduled.subjectId, {
          scheduledId: scheduled.id,
          notifyRoles: rule.notifyRoles
        });
        break;
    }
  }
}

interface ScheduledVerification {
  id: string;
  subjectId: string;
  scheduledDate: Date;
  dueDate: Date;
  requiredLevel: VerificationLevel;
  requiredMethods: VerificationMethod[];
  status: 'scheduled' | 'in-progress' | 'completed' | 'overdue';
  reminders: ReminderSchedule[];
  createdAt: Date;
}

interface ReminderSchedule {
  daysBeforeDue: number;
  sent: boolean;
  sentAt?: Date;
}
```

## 요약

이 장에서 다룬 내용:

1. **핵심 인증 시스템**: 엔진 아키텍처, 세션 관리, 다중 방법 인증
2. **문서 인증**: OCR 추출, 진위 검증, 사기 탐지
3. **생체정보 인증**: 다양한 양식 지원, 생체 인식, 품질 평가
4. **지식 기반 인증**: 챌린지-응답 프로토콜, 퍼지 매칭
5. **재인증**: 정기 및 트리거 기반 재인증, 에스컬레이션 규칙

다음 장에서는 시스템 통합 패턴을 다룹니다.
