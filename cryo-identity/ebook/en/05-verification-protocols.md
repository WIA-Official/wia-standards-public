# Chapter 5: Verification Protocols

## Overview

Identity verification in cryogenic preservation requires multi-layered protocols that ensure accurate subject identification throughout the specimen lifecycle. This chapter details comprehensive verification methodologies, from initial enrollment through ongoing re-verification processes.

## Verification Architecture

### Core Verification System

```typescript
import { EventEmitter } from 'events';

// Verification method types
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

// Verification result interface
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

// Verification session management
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

// Main verification engine
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
      throw new VerificationError('SUBJECT_NOT_FOUND', `Subject ${subjectId} not found`);
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
      throw new VerificationError('SESSION_NOT_FOUND', `Session ${sessionId} not found`);
    }

    if (session.status === 'expired' || new Date() > session.expiresAt) {
      session.status = 'expired';
      throw new VerificationError('SESSION_EXPIRED', 'Verification session has expired');
    }

    if (!session.requiredMethods.includes(method)) {
      throw new VerificationError('METHOD_NOT_REQUIRED', `Method ${method} not required for this session`);
    }

    if (session.completedMethods.includes(method)) {
      throw new VerificationError('METHOD_COMPLETED', `Method ${method} already completed`);
    }

    const verifier = this.verifiers.get(method);
    if (!verifier) {
      throw new VerificationError('VERIFIER_NOT_FOUND', `No verifier for method ${method}`);
    }

    session.status = 'in-progress';
    session.updatedAt = new Date();

    try {
      const result = await verifier.verify(session.subjectId, input);

      // Store evidence securely
      for (const evidence of result.evidence) {
        await this.evidenceStore.store(session.id, evidence);
      }

      session.results.push(result);

      if (result.status === 'completed' && result.confidence >= this.config.thresholds[method]) {
        session.completedMethods.push(method);
      }

      // Check if all required methods are completed
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
        error: error instanceof Error ? error.message : 'Unknown error'
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
      basic: 30 * 60 * 1000,      // 30 minutes
      standard: 60 * 60 * 1000,   // 1 hour
      enhanced: 120 * 60 * 1000,  // 2 hours
      maximum: 240 * 60 * 1000    // 4 hours
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

// Verification configuration
interface VerificationConfig {
  policies: PolicyConfig;
  thresholds: Record<VerificationMethod, number>;
  document: DocumentVerifierConfig;
  biometric: BiometricVerifierConfig;
  knowledge: KnowledgeVerifierConfig;
  possession: PossessionVerifierConfig;
  thirdParty: ThirdPartyVerifierConfig;
  audit: AuditConfig;
}

interface PolicyConfig {
  levelRequirements: Record<VerificationLevel, VerificationMethod[]>;
  contextRules: ContextRule[];
  subjectTypeOverrides: Record<string, VerificationMethod[]>;
}

interface ContextRule {
  condition: (context: VerificationContext) => boolean;
  level: VerificationLevel;
  additionalMethods?: VerificationMethod[];
}
```

### Document Verification

```typescript
// Document verification implementation
interface DocumentVerifierConfig {
  supportedDocuments: DocumentType[];
  ocrProvider: string;
  validationRules: DocumentValidationRule[];
  expirationBuffer: number; // days before expiration to warn
}

type DocumentType =
  | 'passport'
  | 'national-id'
  | 'drivers-license'
  | 'birth-certificate'
  | 'medical-id'
  | 'donor-card';

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
      // Step 1: Extract data using OCR
      const extractedData = await this.extractDocumentData(docInput);

      // Step 2: Validate document authenticity
      const authenticityScore = await this.validateAuthenticity(docInput, extractedData);

      // Step 3: Check for fraud indicators
      const fraudCheck = await this.fraudDetector.analyze({
        images: [docInput.frontImage, docInput.backImage].filter(Boolean) as Buffer[],
        extractedData
      });

      // Step 4: Cross-reference with subject data
      const crossRefScore = await this.crossReferenceWithSubject(subjectId, extractedData);

      // Step 5: Check document expiration
      const expirationStatus = this.checkExpiration(extractedData);

      // Calculate overall confidence
      const confidence = this.calculateConfidence({
        authenticityScore,
        fraudScore: 1 - fraudCheck.riskScore,
        crossRefScore,
        expirationPenalty: expirationStatus.penalty
      });

      // Generate evidence
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
        expiresAt: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000), // 1 year
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
          error: error instanceof Error ? error.message : 'Unknown error',
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

  private async validateAuthenticity(
    input: DocumentInput,
    data: ExtractedDocumentData
  ): Promise<number> {
    const checks: AuthenticityCheck[] = [
      await this.checkSecurityFeatures(input.frontImage),
      await this.checkMRZConsistency(data),
      await this.checkFieldFormats(data),
      await this.checkDigitalSignature(input)
    ];

    const passedChecks = checks.filter(c => c.passed).length;
    return passedChecks / checks.length;
  }

  private async crossReferenceWithSubject(
    subjectId: string,
    data: ExtractedDocumentData
  ): Promise<number> {
    // Implementation would fetch subject and compare
    // Returns similarity score 0-1
    return 0.95;
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
      return { penalty: 1, warning: 'Document has expired' };
    }

    if (daysUntilExpiration < this.config.expirationBuffer) {
      return {
        penalty: 0.1,
        warning: `Document expires in ${daysUntilExpiration} days`
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
    // Implementation would use proper encryption
    return data;
  }

  private async checkSecurityFeatures(image: Buffer): Promise<AuthenticityCheck> {
    return { name: 'security-features', passed: true, confidence: 0.9 };
  }

  private async checkMRZConsistency(data: ExtractedDocumentData): Promise<AuthenticityCheck> {
    return { name: 'mrz-consistency', passed: true, confidence: 0.95 };
  }

  private async checkFieldFormats(data: ExtractedDocumentData): Promise<AuthenticityCheck> {
    return { name: 'field-formats', passed: true, confidence: 0.9 };
  }

  private async checkDigitalSignature(input: DocumentInput): Promise<AuthenticityCheck> {
    return { name: 'digital-signature', passed: true, confidence: 0.85 };
  }
}

interface AuthenticityCheck {
  name: string;
  passed: boolean;
  confidence: number;
}
```

### Biometric Verification

```typescript
// Biometric verification implementation
interface BiometricVerifierConfig {
  supportedModalities: BiometricModality[];
  matchingThresholds: Record<BiometricModality, number>;
  livenessDetection: LivenessConfig;
  qualityRequirements: QualityConfig;
}

type BiometricModality =
  | 'fingerprint'
  | 'facial'
  | 'iris'
  | 'voice'
  | 'palm';

interface LivenessConfig {
  enabled: boolean;
  challenges: LivenessChallenge[];
  minimumScore: number;
}

type LivenessChallenge =
  | 'blink'
  | 'smile'
  | 'turn-head'
  | 'random-digits'
  | 'light-reflection';

interface QualityConfig {
  minimumScore: number;
  resolutionRequirements: Record<BiometricModality, { width: number; height: number }>;
  illuminationThreshold: number;
  sharpnessThreshold: number;
}

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

interface LivenessData {
  challenges: {
    type: LivenessChallenge;
    response: Buffer;
    timestamp: Date;
  }[];
  environmentData?: {
    lighting: number;
    backgroundNoise?: number;
  };
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
        throw new Error(`Unsupported modality: ${modality}`);
    }
  }

  async verify(subjectId: string, input: VerificationInput): Promise<VerificationResult> {
    const bioInput = input as BiometricInput;
    const startTime = Date.now();

    try {
      // Step 1: Check sample quality
      const qualityResults = await this.assessQuality(bioInput);
      if (!qualityResults.acceptable) {
        return this.createFailedResult(subjectId, bioInput.modality,
          'Sample quality below threshold', startTime);
      }

      // Step 2: Perform liveness detection
      if (this.config.livenessDetection.enabled) {
        const livenessResult = await this.checkLiveness(bioInput);
        if (!livenessResult.isLive) {
          return this.createFailedResult(subjectId, bioInput.modality,
            'Liveness check failed', startTime);
        }
      }

      // Step 3: Extract biometric template
      const engine = this.modalityEngines.get(bioInput.modality);
      if (!engine) {
        throw new Error(`No engine for modality: ${bioInput.modality}`);
      }

      const capturedTemplate = await engine.extractTemplate(bioInput.samples);

      // Step 4: Retrieve enrolled template
      const enrolledTemplate = await this.templateStore.getTemplate(
        subjectId,
        bioInput.modality
      );

      if (!enrolledTemplate) {
        return this.createFailedResult(subjectId, bioInput.modality,
          'No enrolled template found', startTime);
      }

      // Step 5: Perform matching
      const matchResult = await engine.match(capturedTemplate, enrolledTemplate);
      const threshold = this.config.matchingThresholds[bioInput.modality];
      const isMatch = matchResult.score >= threshold;

      // Create evidence
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

      if (this.config.livenessDetection.enabled && bioInput.livenessData) {
        evidence.push({
          type: 'liveness-data',
          source: 'liveness-detector',
          capturedAt: new Date(),
          hash: await this.hashData(bioInput.livenessData)
        });
      }

      return {
        id: `BR-${Date.now()}`,
        sessionId: '',
        subjectId,
        method: 'biometric',
        status: isMatch ? 'completed' : 'failed',
        confidence: matchResult.score,
        timestamp: new Date(),
        expiresAt: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000), // 30 days
        evidence,
        metadata: {
          modality: bioInput.modality,
          matchScore: matchResult.score,
          threshold,
          qualityScore: qualityResults.score,
          livenessScore: bioInput.livenessData ?
            await this.livenessDetector.getScore(bioInput.livenessData) : null,
          processingTime: Date.now() - startTime
        }
      };
    } catch (error) {
      return this.createFailedResult(subjectId, bioInput.modality,
        error instanceof Error ? error.message : 'Unknown error', startTime);
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

// Abstract engine interface
interface ModalityEngine {
  extractTemplate(samples: BiometricSample[]): Promise<BiometricTemplate>;
  match(captured: BiometricTemplate, enrolled: BiometricTemplate): Promise<MatchResult>;
}
```

### Knowledge-Based Verification

```typescript
// Knowledge verification for challenge questions
interface KnowledgeVerifierConfig {
  questionSets: QuestionSet[];
  minimumCorrectAnswers: number;
  maxAttempts: number;
  lockoutDuration: number; // minutes
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
  type: 'text' | 'date' | 'numeric' | 'multiple-choice';
  options?: string[]; // For multiple choice
  hints?: string[];
  weight: number; // Importance for scoring
}

interface KnowledgeInput {
  questionId: string;
  answer: string | number | Date;
  timestamp: Date;
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

    // Get or create challenge session
    let session = this.getChallengeSession(subjectId);
    if (!session) {
      session = await this.createChallengeSession(subjectId);
    }

    // Check if locked out
    if (session.lockedUntil && new Date() < session.lockedUntil) {
      const remainingMinutes = Math.ceil(
        (session.lockedUntil.getTime() - Date.now()) / (1000 * 60)
      );
      return this.createFailedResult(subjectId,
        `Account locked. Try again in ${remainingMinutes} minutes`, startTime);
    }

    // Find the question being answered
    const question = session.questions.find(q => q.questionId === knowledgeInput.questionId);
    if (!question) {
      return this.createFailedResult(subjectId, 'Invalid question ID', startTime);
    }

    // Check answer
    const matchResult = this.checkAnswer(question, knowledgeInput);

    session.answers.push({
      questionId: knowledgeInput.questionId,
      providedAnswer: knowledgeInput.answer,
      isCorrect: matchResult.isCorrect,
      matchScore: matchResult.score,
      answeredAt: new Date()
    });

    // Check if all questions answered
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
        expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
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

    // Still more questions to answer
    return {
      id: `KR-${Date.now()}`,
      sessionId: '',
      subjectId,
      method: 'knowledge',
      status: 'pending',
      confidence: 0,
      timestamp: new Date(),
      expiresAt: new Date(Date.now() + 60 * 60 * 1000), // 1 hour
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

    // Select questions based on stored knowledge
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

    // Select questions from different categories for diversity
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

      if (selected.length >= 5) break; // Max 5 questions
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
      questions: session.questions.map(q => ({ ...q, correctAnswer: '[REDACTED]' }))
    };
    return crypto.createHash('sha256').update(JSON.stringify(sanitized)).digest('hex');
  }
}

interface StoredKnowledge {
  subjectId: string;
  answers: Map<string, string | number | Date>;
  updatedAt: Date;
}
```

## Re-Verification Protocols

### Scheduled Re-Verification

```typescript
// Re-verification scheduling and management
interface ReVerificationConfig {
  schedules: ReVerificationSchedule[];
  triggerConditions: TriggerCondition[];
  graceperiod: number; // days after scheduled date
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
      throw new Error(`Subject ${subjectId} not found`);
    }

    const schedule = this.findApplicableSchedule(subject, lastVerification);
    const nextDate = new Date(
      lastVerification.timestamp.getTime() + schedule.intervalDays * 24 * 60 * 60 * 1000
    );

    const scheduled: ScheduledVerification = {
      id: `SV-${Date.now()}`,
      subjectId,
      scheduledDate: nextDate,
      dueDate: new Date(nextDate.getTime() + this.config.graceperiod * 24 * 60 * 60 * 1000),
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
      // Default schedule
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

  async checkTriggeredReVerifications(): Promise<TriggeredVerification[]> {
    const triggered: TriggeredVerification[] = [];
    const subjects = await this.subjectRepository.findAll();

    for (const subject of subjects) {
      for (const condition of this.config.triggerConditions) {
        if (condition.condition(subject, {})) {
          triggered.push({
            subjectId: subject.id,
            trigger: condition.name,
            requiredLevel: condition.resultingLevel,
            priority: condition.priority,
            triggeredAt: new Date()
          });
        }
      }
    }

    return triggered.sort((a, b) => b.priority - a.priority);
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

interface TriggeredVerification {
  subjectId: string;
  trigger: string;
  requiredLevel: VerificationLevel;
  priority: number;
  triggeredAt: Date;
}
```

## Verification Workflow Orchestration

### Complete Verification Flow

```typescript
// End-to-end verification workflow
class VerificationWorkflow {
  constructor(
    private verificationEngine: VerificationEngine,
    private documentVerifier: DocumentVerifier,
    private biometricVerifier: BiometricVerifier,
    private knowledgeVerifier: KnowledgeVerifier,
    private reVerificationManager: ReVerificationManager,
    private auditService: AuditService
  ) {}

  async executeFullVerification(
    subjectId: string,
    context: VerificationContext
  ): Promise<VerificationWorkflowResult> {
    const workflowId = `VW-${Date.now()}-${Math.random().toString(36).substring(2, 8)}`;
    const startTime = Date.now();

    await this.auditService.log({
      action: 'VERIFICATION_WORKFLOW_STARTED',
      workflowId,
      subjectId,
      context
    });

    try {
      // Step 1: Create verification session
      const session = await this.verificationEngine.createSession(subjectId, context);

      // Step 2: Execute required verifications in sequence
      const results: VerificationResult[] = [];

      for (const method of session.requiredMethods) {
        const result = await this.executeMethodVerification(session.id, method, subjectId);
        results.push(result);

        // If critical method fails, abort workflow
        if (result.status === 'failed' && this.isCriticalMethod(method, context)) {
          return this.createWorkflowResult(workflowId, session, results, 'failed', startTime);
        }
      }

      // Step 3: Calculate overall result
      const overallStatus = this.calculateOverallStatus(results, session.requiredMethods);

      // Step 4: Schedule re-verification if successful
      if (overallStatus === 'completed') {
        const lastResult = results[results.length - 1];
        await this.reVerificationManager.scheduleReVerification(subjectId, lastResult);
      }

      return this.createWorkflowResult(workflowId, session, results, overallStatus, startTime);

    } catch (error) {
      await this.auditService.log({
        action: 'VERIFICATION_WORKFLOW_ERROR',
        workflowId,
        subjectId,
        error: error instanceof Error ? error.message : 'Unknown error'
      });

      return {
        workflowId,
        status: 'failed',
        results: [],
        duration: Date.now() - startTime,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  }

  private async executeMethodVerification(
    sessionId: string,
    method: VerificationMethod,
    subjectId: string
  ): Promise<VerificationResult> {
    // In real implementation, this would collect input from the user
    // Here we simulate the verification flow
    const mockInput = await this.collectVerificationInput(method, subjectId);
    return this.verificationEngine.executeVerification(sessionId, method, mockInput);
  }

  private async collectVerificationInput(
    method: VerificationMethod,
    subjectId: string
  ): Promise<VerificationInput> {
    // This would be implemented to collect actual input
    // Returning mock data for demonstration
    switch (method) {
      case 'document':
        return {
          documentType: 'passport',
          frontImage: Buffer.from('mock-image'),
          metadata: {}
        } as DocumentInput;
      case 'biometric':
        return {
          modality: 'facial',
          samples: [{
            data: Buffer.from('mock-biometric'),
            capturedAt: new Date(),
            deviceInfo: {
              type: 'camera',
              manufacturer: 'Generic',
              model: 'HD-1080',
              certifications: ['ISO-19795']
            }
          }]
        } as BiometricInput;
      default:
        return {};
    }
  }

  private isCriticalMethod(method: VerificationMethod, context: VerificationContext): boolean {
    if (context.riskLevel === 'critical') return true;
    if (method === 'biometric' && context.operationType === 'specimen-release') return true;
    return false;
  }

  private calculateOverallStatus(
    results: VerificationResult[],
    requiredMethods: VerificationMethod[]
  ): VerificationStatus {
    const completedMethods = results
      .filter(r => r.status === 'completed')
      .map(r => r.method);

    const allRequired = requiredMethods.every(m => completedMethods.includes(m));
    return allRequired ? 'completed' : 'failed';
  }

  private createWorkflowResult(
    workflowId: string,
    session: VerificationSession,
    results: VerificationResult[],
    status: VerificationStatus,
    startTime: number
  ): VerificationWorkflowResult {
    const averageConfidence = results.length > 0
      ? results.reduce((sum, r) => sum + r.confidence, 0) / results.length
      : 0;

    return {
      workflowId,
      sessionId: session.id,
      subjectId: session.subjectId,
      status,
      level: session.requestedLevel,
      results,
      overallConfidence: averageConfidence,
      duration: Date.now() - startTime,
      completedAt: new Date()
    };
  }
}

interface VerificationWorkflowResult {
  workflowId: string;
  sessionId?: string;
  subjectId: string;
  status: VerificationStatus;
  level?: VerificationLevel;
  results: VerificationResult[];
  overallConfidence?: number;
  duration: number;
  completedAt?: Date;
  error?: string;
}
```

## Summary

This chapter covered:

1. **Core Verification System**: Engine architecture, session management, and multi-method verification
2. **Document Verification**: OCR extraction, authenticity validation, and fraud detection
3. **Biometric Verification**: Multi-modal support with liveness detection and quality assessment
4. **Knowledge Verification**: Challenge-response protocols with fuzzy matching
5. **Re-Verification**: Scheduled and triggered re-verification with escalation rules
6. **Workflow Orchestration**: End-to-end verification flow management

The next chapter covers system integration patterns for connecting identity verification with other cryogenic facility systems.
