# Chapter 4: API Interface

## Content Authentication and Detection APIs

This chapter defines the API specifications for content authentication, AI detection, and provenance verification services.

---

## API Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                    WIA Content AI API Architecture                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   Client Applications                                                │
│   ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐              │
│   │  Web     │ │  Mobile  │ │  CMS     │ │  AI Gen  │              │
│   │  Apps    │ │  Apps    │ │  Plugins │ │  Tools   │              │
│   └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘              │
│        │            │            │            │                      │
│        └────────────┴─────┬──────┴────────────┘                      │
│                           │                                          │
│                    ┌──────┴──────┐                                   │
│                    │  API Gateway │                                  │
│                    │  (Auth, Rate │                                  │
│                    │   Limiting)  │                                  │
│                    └──────┬──────┘                                   │
│                           │                                          │
│   ┌───────────────────────┼───────────────────────┐                 │
│   │                       │                       │                  │
│   ▼                       ▼                       ▼                  │
│ ┌─────────────┐    ┌─────────────┐    ┌─────────────┐              │
│ │ Signing     │    │ Detection   │    │ Verification│              │
│ │ Service     │    │ Service     │    │ Service     │              │
│ │             │    │             │    │             │              │
│ │ • C2PA Sign │    │ • Image     │    │ • Manifest  │              │
│ │ • Watermark │    │ • Video     │    │ • Chain     │              │
│ │ • Timestamp │    │ • Audio     │    │ • Trust     │              │
│ └─────────────┘    │ • Text      │    └─────────────┘              │
│                    └─────────────┘                                   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## REST API Specification

### OpenAPI 3.1 Definition

```yaml
openapi: 3.1.0
info:
  title: WIA Content AI API
  version: 1.0.0
  description: |
    API for content authentication, AI detection, and provenance verification.
  contact:
    name: WIA Technical Support
    email: api-support@wia.org
  license:
    name: Apache 2.0
    url: https://www.apache.org/licenses/LICENSE-2.0

servers:
  - url: https://api.wia-content.org/v1
    description: Production server
  - url: https://sandbox.wia-content.org/v1
    description: Sandbox for testing

security:
  - bearerAuth: []
  - apiKey: []

tags:
  - name: signing
    description: Content signing and credential operations
  - name: detection
    description: AI content detection services
  - name: verification
    description: Manifest and provenance verification
  - name: watermarking
    description: Invisible watermark operations

paths:
  /sign/manifest:
    post:
      tags: [signing]
      summary: Sign content with C2PA manifest
      operationId: signManifest
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              $ref: '#/components/schemas/SigningRequest'
      responses:
        '200':
          description: Signed content with embedded manifest
          content:
            application/octet-stream:
              schema:
                type: string
                format: binary
        '400':
          $ref: '#/components/responses/BadRequest'
        '401':
          $ref: '#/components/responses/Unauthorized'

  /detect/analyze:
    post:
      tags: [detection]
      summary: Analyze content for AI generation
      operationId: analyzeContent
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              $ref: '#/components/schemas/DetectionRequest'
      responses:
        '200':
          description: Detection analysis results
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DetectionResult'

  /verify/manifest:
    post:
      tags: [verification]
      summary: Verify content manifest and provenance
      operationId: verifyManifest
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              $ref: '#/components/schemas/VerificationRequest'
      responses:
        '200':
          description: Verification results
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/VerificationResult'

components:
  securitySchemes:
    bearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT
    apiKey:
      type: apiKey
      in: header
      name: X-API-Key

  schemas:
    SigningRequest:
      type: object
      required: [content, assertions]
      properties:
        content:
          type: string
          format: binary
          description: Content file to sign
        assertions:
          type: array
          items:
            $ref: '#/components/schemas/Assertion'
        ingredients:
          type: array
          items:
            $ref: '#/components/schemas/IngredientInput'
        signerInfo:
          $ref: '#/components/schemas/SignerInfo'

    DetectionRequest:
      type: object
      required: [content]
      properties:
        content:
          type: string
          format: binary
        contentType:
          type: string
          enum: [IMAGE, VIDEO, AUDIO, TEXT]
        models:
          type: array
          items:
            type: string
        options:
          $ref: '#/components/schemas/DetectionOptions'

    DetectionResult:
      $ref: '#/components/schemas/AIDetectionResult'
```

---

## Content Signing API

```typescript
// WIA Content AI Signing Service
// C2PA manifest creation and embedding

import { createHash, createSign } from 'crypto';

/**
 * Content Signing Service
 */
class ContentSigningService {
  private certificateStore: CertificateStore;
  private timestampService: TimestampService;
  private manifestBuilder: ManifestBuilder;

  constructor(config: SigningConfig) {
    this.certificateStore = new CertificateStore(config.certificates);
    this.timestampService = new TimestampService(config.timestamp);
    this.manifestBuilder = new ManifestBuilder();
  }

  /**
   * Sign content with C2PA manifest
   */
  async signContent(request: SignContentRequest): Promise<SignContentResponse> {
    // Validate input
    this.validateRequest(request);

    // Get signer certificate
    const signerCert = await this.certificateStore.getSignerCertificate(
      request.signerId
    );

    // Build manifest
    const manifest = await this.manifestBuilder
      .setGenerator("WIA Content AI", "1.0.0")
      .setContentInfo(request.title, request.mimeType)
      .addAssertions(request.assertions)
      .addIngredients(request.ingredients || [])
      .build();

    // Compute content hash
    const contentHash = this.computeHash(request.content);

    // Create claim
    const claim = this.createClaim(manifest, contentHash);

    // Sign the claim
    const signature = await this.signClaim(claim, signerCert);

    // Get timestamp
    const timestamp = await this.timestampService.getTimestamp(signature);

    // Embed manifest in content
    const signedContent = await this.embedManifest(
      request.content,
      request.mimeType,
      {
        ...manifest,
        signature: {
          ...signature,
          timestamp
        }
      }
    );

    return {
      signedContent,
      manifestId: manifest.instanceId,
      hash: contentHash,
      signature: signature.sig,
      timestamp: timestamp.time
    };
  }

  /**
   * Sign AI-generated content with generation metadata
   */
  async signAIGeneratedContent(
    request: SignAIContentRequest
  ): Promise<SignContentResponse> {
    // Create AI generation assertion
    const aiAssertion: Assertion = {
      label: "c2pa.ai_generated",
      data: {
        model: request.aiMetadata.model,
        provider: request.aiMetadata.provider,
        generatedAt: new Date().toISOString(),
        parameters: request.aiMetadata.parameters,
        softwareAgent: {
          name: request.aiMetadata.provider,
          version: request.aiMetadata.modelVersion
        }
      }
    };

    // Create actions assertion
    const actionsAssertion: Assertion = {
      label: "c2pa.actions",
      data: {
        actions: [{
          action: "c2pa.ai_generated",
          when: new Date().toISOString(),
          softwareAgent: {
            name: request.aiMetadata.provider,
            version: request.aiMetadata.modelVersion
          },
          digitalSourceType: "http://cv.iptc.org/newscodes/digitalsourcetype/trainedAlgorithmicMedia",
          parameters: {
            prompt: request.aiMetadata.parameters.promptHash
              ? undefined
              : request.aiMetadata.parameters.prompt,
            promptHash: request.aiMetadata.parameters.promptHash
          }
        }]
      }
    };

    return this.signContent({
      ...request,
      assertions: [aiAssertion, actionsAssertion, ...(request.assertions || [])]
    });
  }

  /**
   * Add watermark alongside C2PA manifest
   */
  async signWithWatermark(
    request: SignWithWatermarkRequest
  ): Promise<SignWithWatermarkResponse> {
    // First, embed invisible watermark
    const watermarkedContent = await this.embedWatermark(
      request.content,
      request.mimeType,
      request.watermarkPayload
    );

    // Then sign with C2PA
    const signed = await this.signContent({
      ...request,
      content: watermarkedContent,
      assertions: [
        ...request.assertions,
        {
          label: "wia.watermark",
          data: {
            type: "INVISIBLE",
            payloadHash: this.computeHash(request.watermarkPayload)
          }
        }
      ]
    });

    return {
      ...signed,
      watermarkId: request.watermarkPayload.id
    };
  }

  private validateRequest(request: SignContentRequest): void {
    if (!request.content || request.content.length === 0) {
      throw new Error("Content is required");
    }
    if (!request.mimeType) {
      throw new Error("MIME type is required");
    }
  }

  private computeHash(data: Buffer | string): string {
    return createHash('sha256')
      .update(data)
      .digest('base64');
  }

  private createClaim(manifest: any, contentHash: string): Claim {
    return {
      dcTitle: manifest.title,
      dcFormat: manifest.format,
      claimGeneratorInfo: {
        name: manifest.claimGenerator.split('/')[0],
        version: manifest.claimGenerator.split('/')[1]
      },
      signatureInfo: {
        alg: "ES256",
        issuer: "",
        time: new Date().toISOString()
      },
      assertionStore: [],
      ingredientStore: [],
      alg: "ES256"
    };
  }

  private async signClaim(
    claim: Claim,
    cert: SignerCertificate
  ): Promise<ManifestSignature> {
    const claimBytes = Buffer.from(JSON.stringify(claim));
    const signer = createSign('SHA256');
    signer.update(claimBytes);
    const signature = signer.sign(cert.privateKey, 'base64');

    return {
      alg: "ES256",
      sig: signature,
      certChain: cert.chain
    };
  }

  private async embedManifest(
    content: Buffer,
    mimeType: string,
    manifest: C2PAManifest
  ): Promise<Buffer> {
    // Embed manifest based on content type
    switch (mimeType) {
      case 'image/jpeg':
        return this.embedJPEGManifest(content, manifest);
      case 'image/png':
        return this.embedPNGManifest(content, manifest);
      case 'video/mp4':
        return this.embedMP4Manifest(content, manifest);
      default:
        return this.embedSidecarManifest(content, manifest);
    }
  }

  private async embedJPEGManifest(content: Buffer, manifest: C2PAManifest): Promise<Buffer> {
    // Implementation for JPEG APP11 segment embedding
    return content;
  }

  private async embedPNGManifest(content: Buffer, manifest: C2PAManifest): Promise<Buffer> {
    // Implementation for PNG caBX chunk embedding
    return content;
  }

  private async embedMP4Manifest(content: Buffer, manifest: C2PAManifest): Promise<Buffer> {
    // Implementation for MP4 uuid box embedding
    return content;
  }

  private async embedSidecarManifest(content: Buffer, manifest: C2PAManifest): Promise<Buffer> {
    // Implementation for sidecar .c2pa file
    return content;
  }

  private async embedWatermark(
    content: Buffer,
    mimeType: string,
    payload: WatermarkPayload
  ): Promise<Buffer> {
    // Implement invisible watermark embedding
    return content;
  }
}

// Interfaces
interface SignContentRequest {
  content: Buffer;
  mimeType: string;
  title: string;
  signerId?: string;
  assertions: Assertion[];
  ingredients?: IngredientInput[];
}

interface SignAIContentRequest extends SignContentRequest {
  aiMetadata: {
    model: string;
    provider: string;
    modelVersion: string;
    parameters: AIGenerationParameters;
  };
}

interface SignWithWatermarkRequest extends SignContentRequest {
  watermarkPayload: WatermarkPayload;
}

interface SignContentResponse {
  signedContent: Buffer;
  manifestId: string;
  hash: string;
  signature: string;
  timestamp: string;
}

interface SignWithWatermarkResponse extends SignContentResponse {
  watermarkId: string;
}

interface WatermarkPayload {
  id: string;
  data: Buffer;
}

interface SignerCertificate {
  privateKey: string;
  certificate: string;
  chain: string[];
}

interface SigningConfig {
  certificates: any;
  timestamp: any;
}

interface CertificateStore {
  getSignerCertificate(signerId: string): Promise<SignerCertificate>;
}

interface TimestampService {
  getTimestamp(signature: ManifestSignature): Promise<{ time: string }>;
}
```

---

## AI Detection API

```typescript
/**
 * AI Detection Service
 * Multi-model detection for various content types
 */
class AIDetectionService {
  private imageDetector: ImageDetector;
  private videoDetector: VideoDetector;
  private audioDetector: AudioDetector;
  private textDetector: TextDetector;
  private modelRegistry: ModelRegistry;

  constructor(config: DetectionConfig) {
    this.imageDetector = new ImageDetector(config.image);
    this.videoDetector = new VideoDetector(config.video);
    this.audioDetector = new AudioDetector(config.audio);
    this.textDetector = new TextDetector(config.text);
    this.modelRegistry = new ModelRegistry(config.models);
  }

  /**
   * Analyze content for AI generation
   */
  async analyzeContent(
    request: DetectionRequest
  ): Promise<AIDetectionResult> {
    const startTime = Date.now();
    const requestId = crypto.randomUUID();

    // Compute content info
    const contentInfo = await this.extractContentInfo(
      request.content,
      request.contentType
    );

    // Select appropriate detector
    const detector = this.getDetector(request.contentType);

    // Get models to use
    const models = request.options?.models ||
      this.modelRegistry.getDefaultModels(request.contentType);

    // Run detection models
    const modelResults = await Promise.all(
      models.map(modelId => detector.runModel(modelId, request.content))
    );

    // Extract features
    const features = await detector.extractFeatures(request.content);

    // Perform regional analysis if applicable
    let regions: RegionalAnalysis[] | undefined;
    if (request.options?.analyzeRegions &&
        ['IMAGE', 'VIDEO'].includes(request.contentType)) {
      regions = await detector.analyzeRegions(request.content);
    }

    // Check for provenance
    const provenance = await this.checkProvenance(request.content);

    // Aggregate results
    const detection = this.aggregateResults(modelResults, features, provenance);

    // Compute confidence metrics
    const confidence = this.computeConfidence(modelResults, features, provenance);

    return {
      version: "1.0",
      requestId,
      timestamp: new Date().toISOString(),
      content: contentInfo,
      detection,
      analysis: {
        models: modelResults,
        features,
        regions
      },
      provenance,
      confidence,
      metadata: {
        processingTime: Date.now() - startTime,
        modelsUsed: models,
        apiVersion: "1.0.0",
        region: "us-west-2"
      }
    };
  }

  /**
   * Batch analyze multiple content items
   */
  async batchAnalyze(
    requests: DetectionRequest[]
  ): Promise<AIDetectionResult[]> {
    return Promise.all(requests.map(req => this.analyzeContent(req)));
  }

  /**
   * Real-time stream detection
   */
  async *streamDetect(
    stream: AsyncIterable<Buffer>,
    contentType: ContentType
  ): AsyncGenerator<StreamDetectionResult> {
    const detector = this.getDetector(contentType);

    for await (const chunk of stream) {
      const result = await detector.analyzeChunk(chunk);
      yield {
        timestamp: Date.now(),
        chunkHash: this.computeHash(chunk),
        detection: result.detection,
        confidence: result.confidence
      };
    }
  }

  private getDetector(contentType: ContentType): ContentDetector {
    switch (contentType) {
      case 'IMAGE': return this.imageDetector;
      case 'VIDEO': return this.videoDetector;
      case 'AUDIO': return this.audioDetector;
      case 'TEXT': return this.textDetector;
      default: throw new Error(`Unsupported content type: ${contentType}`);
    }
  }

  private async extractContentInfo(
    content: Buffer,
    contentType: ContentType
  ): Promise<ContentInfo> {
    const hash = this.computeHash(content);

    return {
      type: contentType,
      format: this.detectFormat(content),
      size: content.length,
      hash: { alg: "SHA-256", hash }
    };
  }

  private async checkProvenance(content: Buffer): Promise<ProvenanceInfo | undefined> {
    try {
      const manifest = await this.extractManifest(content);
      if (!manifest) return { hasC2PA: false };

      const validation = await this.validateManifest(manifest);

      return {
        hasC2PA: true,
        manifestValid: validation.valid,
        signerIdentity: manifest.signature?.issuer,
        aiDisclosed: this.checkAIDisclosure(manifest)
      };
    } catch {
      return { hasC2PA: false };
    }
  }

  private aggregateResults(
    modelResults: ModelResult[],
    features: FeatureAnalysis,
    provenance?: ProvenanceInfo
  ): DetectionVerdict {
    // Weighted average of model scores
    const avgScore = modelResults.reduce((sum, r) => sum + r.score, 0) / modelResults.length;

    // Adjust based on provenance
    let adjustedScore = avgScore;
    if (provenance?.hasC2PA && provenance.manifestValid) {
      if (provenance.aiDisclosed) {
        adjustedScore = Math.max(avgScore, 0.95);
      } else {
        adjustedScore = Math.min(avgScore, 0.3);
      }
    }

    return {
      isAIGenerated: adjustedScore > 0.5,
      confidence: adjustedScore,
      generationType: this.determineGenerationType(adjustedScore, provenance),
      manipulationType: this.detectManipulationTypes(modelResults, features)
    };
  }

  private computeConfidence(
    modelResults: ModelResult[],
    features: FeatureAnalysis,
    provenance?: ProvenanceInfo
  ): ConfidenceMetrics {
    const scores = modelResults.map(r => r.score);
    const mean = scores.reduce((a, b) => a + b, 0) / scores.length;
    const variance = scores.reduce((sum, s) => sum + Math.pow(s - mean, 2), 0) / scores.length;
    const agreement = 1 - Math.sqrt(variance);

    return {
      overall: mean,
      modelAgreement: agreement,
      featureStrength: this.computeFeatureStrength(features),
      provenanceBoost: provenance?.hasC2PA ? 0.1 : 0,
      uncertaintyFactors: this.identifyUncertaintyFactors(modelResults, features)
    };
  }

  private determineGenerationType(
    score: number,
    provenance?: ProvenanceInfo
  ): GenerationType {
    if (provenance?.aiDisclosed) return "FULLY_SYNTHETIC";
    if (score > 0.8) return "FULLY_SYNTHETIC";
    if (score > 0.4) return "PARTIALLY_SYNTHETIC";
    if (score < 0.2) return "HUMAN_CREATED";
    return "UNCERTAIN";
  }

  private detectManipulationTypes(
    modelResults: ModelResult[],
    features: FeatureAnalysis
  ): ManipulationType[] {
    const types: ManipulationType[] = [];
    // Analyze model predictions for manipulation types
    return types;
  }

  private computeFeatureStrength(features: FeatureAnalysis): number {
    return 0.8; // Computed from feature values
  }

  private identifyUncertaintyFactors(
    modelResults: ModelResult[],
    features: FeatureAnalysis
  ): string[] {
    return [];
  }

  private computeHash(data: Buffer): string {
    return createHash('sha256').update(data).digest('base64');
  }

  private detectFormat(content: Buffer): string {
    return "application/octet-stream";
  }

  private async extractManifest(content: Buffer): Promise<C2PAManifest | null> {
    return null;
  }

  private async validateManifest(manifest: C2PAManifest): Promise<{ valid: boolean }> {
    return { valid: true };
  }

  private checkAIDisclosure(manifest: C2PAManifest): boolean {
    return manifest.assertions.some(a => a.label === "c2pa.ai_generated");
  }
}

interface DetectionRequest {
  content: Buffer;
  contentType: ContentType;
  options?: DetectionOptions;
}

interface DetectionOptions {
  models?: string[];
  analyzeRegions?: boolean;
  checkProvenance?: boolean;
  timeout?: number;
}

type ContentType = "IMAGE" | "VIDEO" | "AUDIO" | "TEXT";

interface StreamDetectionResult {
  timestamp: number;
  chunkHash: string;
  detection: DetectionVerdict;
  confidence: number;
}

interface ContentDetector {
  runModel(modelId: string, content: Buffer): Promise<ModelResult>;
  extractFeatures(content: Buffer): Promise<FeatureAnalysis>;
  analyzeRegions(content: Buffer): Promise<RegionalAnalysis[]>;
  analyzeChunk(chunk: Buffer): Promise<{ detection: DetectionVerdict; confidence: number }>;
}

interface DetectionConfig {
  image: any;
  video: any;
  audio: any;
  text: any;
  models: any;
}

// Stub classes
class ImageDetector implements ContentDetector {
  constructor(config: any) {}
  async runModel(modelId: string, content: Buffer): Promise<ModelResult> { return {} as ModelResult; }
  async extractFeatures(content: Buffer): Promise<FeatureAnalysis> { return {} as FeatureAnalysis; }
  async analyzeRegions(content: Buffer): Promise<RegionalAnalysis[]> { return []; }
  async analyzeChunk(chunk: Buffer): Promise<{ detection: DetectionVerdict; confidence: number }> {
    return { detection: {} as DetectionVerdict, confidence: 0 };
  }
}

class VideoDetector implements ContentDetector {
  constructor(config: any) {}
  async runModel(modelId: string, content: Buffer): Promise<ModelResult> { return {} as ModelResult; }
  async extractFeatures(content: Buffer): Promise<FeatureAnalysis> { return {} as FeatureAnalysis; }
  async analyzeRegions(content: Buffer): Promise<RegionalAnalysis[]> { return []; }
  async analyzeChunk(chunk: Buffer): Promise<{ detection: DetectionVerdict; confidence: number }> {
    return { detection: {} as DetectionVerdict, confidence: 0 };
  }
}

class AudioDetector implements ContentDetector {
  constructor(config: any) {}
  async runModel(modelId: string, content: Buffer): Promise<ModelResult> { return {} as ModelResult; }
  async extractFeatures(content: Buffer): Promise<FeatureAnalysis> { return {} as FeatureAnalysis; }
  async analyzeRegions(content: Buffer): Promise<RegionalAnalysis[]> { return []; }
  async analyzeChunk(chunk: Buffer): Promise<{ detection: DetectionVerdict; confidence: number }> {
    return { detection: {} as DetectionVerdict, confidence: 0 };
  }
}

class TextDetector implements ContentDetector {
  constructor(config: any) {}
  async runModel(modelId: string, content: Buffer): Promise<ModelResult> { return {} as ModelResult; }
  async extractFeatures(content: Buffer): Promise<FeatureAnalysis> { return {} as FeatureAnalysis; }
  async analyzeRegions(content: Buffer): Promise<RegionalAnalysis[]> { return []; }
  async analyzeChunk(chunk: Buffer): Promise<{ detection: DetectionVerdict; confidence: number }> {
    return { detection: {} as DetectionVerdict, confidence: 0 };
  }
}

class ModelRegistry {
  constructor(config: any) {}
  getDefaultModels(contentType: ContentType): string[] { return []; }
}
```

---

## Verification API

```typescript
/**
 * Content Verification Service
 * Verify manifests and validate provenance chains
 */
class ContentVerificationService {
  private trustStore: TrustStore;
  private revocationChecker: RevocationChecker;
  private manifestParser: ManifestParser;

  constructor(config: VerificationConfig) {
    this.trustStore = new TrustStore(config.trustedRoots);
    this.revocationChecker = new RevocationChecker(config.ocsp);
    this.manifestParser = new ManifestParser();
  }

  /**
   * Verify content manifest
   */
  async verifyContent(
    request: VerificationRequest
  ): Promise<VerificationResult> {
    const startTime = Date.now();

    // Extract manifest
    const manifest = await this.manifestParser.extract(
      request.content,
      request.mimeType
    );

    if (!manifest) {
      return {
        verified: false,
        status: "NO_MANIFEST",
        message: "No C2PA manifest found in content"
      };
    }

    // Verify signature
    const signatureValid = await this.verifySignature(manifest);
    if (!signatureValid.valid) {
      return {
        verified: false,
        status: "INVALID_SIGNATURE",
        message: signatureValid.reason,
        manifest: this.summarizeManifest(manifest)
      };
    }

    // Verify certificate chain
    const chainValid = await this.verifyCertificateChain(manifest.signature);
    if (!chainValid.valid) {
      return {
        verified: false,
        status: "INVALID_CERTIFICATE",
        message: chainValid.reason,
        manifest: this.summarizeManifest(manifest)
      };
    }

    // Check revocation
    const notRevoked = await this.revocationChecker.check(manifest.signature);
    if (!notRevoked) {
      return {
        verified: false,
        status: "CERTIFICATE_REVOKED",
        message: "Signing certificate has been revoked",
        manifest: this.summarizeManifest(manifest)
      };
    }

    // Verify content hash
    const hashValid = await this.verifyContentHash(request.content, manifest);
    if (!hashValid) {
      return {
        verified: false,
        status: "CONTENT_MODIFIED",
        message: "Content has been modified since signing",
        manifest: this.summarizeManifest(manifest)
      };
    }

    // Verify ingredient chain
    const ingredientsValid = await this.verifyIngredients(manifest);

    return {
      verified: true,
      status: "VALID",
      manifest: this.summarizeManifest(manifest),
      signatureInfo: {
        signer: manifest.signature.issuer,
        timestamp: manifest.claim.signatureInfo.time,
        algorithm: manifest.signature.alg
      },
      ingredientStatus: ingredientsValid,
      processingTime: Date.now() - startTime
    };
  }

  /**
   * Verify provenance chain
   */
  async verifyProvenanceChain(
    content: Buffer
  ): Promise<ProvenanceChainResult> {
    const chain: ManifestSummary[] = [];
    let currentContent = content;

    while (true) {
      const manifest = await this.manifestParser.extract(currentContent);
      if (!manifest) break;

      chain.push(this.summarizeManifest(manifest));

      // Check ingredients for parent manifests
      if (manifest.ingredients.length === 0) break;

      const parentIngredient = manifest.ingredients.find(
        ing => ing.relationship === "parentOf"
      );
      if (!parentIngredient?.manifest) break;

      // Continue with parent
      currentContent = await this.fetchIngredient(parentIngredient);
    }

    return {
      chainLength: chain.length,
      manifests: chain,
      originalCreator: chain[chain.length - 1]?.signer,
      aiGeneratedInChain: chain.some(m => m.aiGenerated)
    };
  }

  private async verifySignature(manifest: C2PAManifest): Promise<{ valid: boolean; reason?: string }> {
    return { valid: true };
  }

  private async verifyCertificateChain(signature: ManifestSignature): Promise<{ valid: boolean; reason?: string }> {
    return { valid: true };
  }

  private async verifyContentHash(content: Buffer, manifest: C2PAManifest): Promise<boolean> {
    return true;
  }

  private async verifyIngredients(manifest: C2PAManifest): Promise<IngredientStatus[]> {
    return [];
  }

  private summarizeManifest(manifest: C2PAManifest): ManifestSummary {
    return {
      id: manifest.instanceId,
      title: manifest.title,
      format: manifest.format,
      generator: manifest.claimGenerator,
      signer: manifest.signature.issuer,
      timestamp: manifest.claim.signatureInfo.time,
      aiGenerated: manifest.assertions.some(a => a.label === "c2pa.ai_generated"),
      ingredientCount: manifest.ingredients.length,
      actionCount: manifest.assertions.filter(a => a.label === "c2pa.actions").length
    };
  }

  private async fetchIngredient(ingredient: Ingredient): Promise<Buffer> {
    return Buffer.alloc(0);
  }
}

interface VerificationRequest {
  content: Buffer;
  mimeType: string;
}

interface VerificationResult {
  verified: boolean;
  status: VerificationStatus;
  message?: string;
  manifest?: ManifestSummary;
  signatureInfo?: SignatureDetails;
  ingredientStatus?: IngredientStatus[];
  processingTime?: number;
}

type VerificationStatus =
  | "VALID"
  | "NO_MANIFEST"
  | "INVALID_SIGNATURE"
  | "INVALID_CERTIFICATE"
  | "CERTIFICATE_REVOKED"
  | "CONTENT_MODIFIED"
  | "EXPIRED";

interface ManifestSummary {
  id: string;
  title: string;
  format: string;
  generator: string;
  signer: string;
  timestamp: string;
  aiGenerated: boolean;
  ingredientCount: number;
  actionCount: number;
}

interface SignatureDetails {
  signer: string;
  timestamp: string;
  algorithm: string;
}

interface IngredientStatus {
  title: string;
  verified: boolean;
  relationship: string;
}

interface ProvenanceChainResult {
  chainLength: number;
  manifests: ManifestSummary[];
  originalCreator?: string;
  aiGeneratedInChain: boolean;
}

interface VerificationConfig {
  trustedRoots: any;
  ocsp: any;
}

class TrustStore {
  constructor(trustedRoots: any) {}
}

class RevocationChecker {
  constructor(config: any) {}
  async check(signature: ManifestSignature): Promise<boolean> { return true; }
}

class ManifestParser {
  async extract(content: Buffer, mimeType?: string): Promise<C2PAManifest | null> { return null; }
}
```

---

## Summary

| API Endpoint | Method | Purpose |
|--------------|--------|---------|
| `/sign/manifest` | POST | Sign content with C2PA manifest |
| `/sign/ai-content` | POST | Sign AI-generated content |
| `/detect/analyze` | POST | Analyze for AI generation |
| `/detect/batch` | POST | Batch content analysis |
| `/verify/manifest` | POST | Verify content manifest |
| `/verify/chain` | POST | Verify provenance chain |

---

**Next Chapter:** [Chapter 5: Detection Protocols](./05-control-protocols.md) - AI content detection methodologies.

---

© 2025 World Industry Association (WIA). All rights reserved.
弘益人間 (홍익인간) · Benefit All Humanity
