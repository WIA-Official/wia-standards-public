# 제4장: API 인터페이스

## 콘텐츠 인증 및 탐지 API

### 개요

WIA 콘텐츠 AI API는 AI 생성 콘텐츠의 서명, 검증 및 탐지를 위한 RESTful 인터페이스를 제공합니다. 본 장에서는 API 엔드포인트, 인증 방식, 요청/응답 형식 및 SDK 사용법을 상세히 설명합니다.

---

## 4.1 API 개요

### OpenAPI 사양

```yaml
openapi: 3.1.0
info:
  title: WIA 콘텐츠 AI API
  description: AI 생성 콘텐츠 인증 및 탐지 서비스
  version: 1.0.0
  contact:
    name: WIA 기술 지원
    email: support@wia.org
  license:
    name: Apache 2.0
    url: https://www.apache.org/licenses/LICENSE-2.0

servers:
  - url: https://api.contentai.wia.org/v1
    description: 운영 서버
  - url: https://sandbox.contentai.wia.org/v1
    description: 샌드박스 테스트 서버

tags:
  - name: 서명
    description: 콘텐츠 자격 증명 서명 작업
  - name: 검증
    description: 콘텐츠 자격 증명 검증 작업
  - name: 탐지
    description: AI 생성 콘텐츠 탐지 작업
  - name: 워터마크
    description: 워터마크 임베딩 및 추출

security:
  - ApiKeyAuth: []
  - OAuth2:
      - read
      - write

components:
  securitySchemes:
    ApiKeyAuth:
      type: apiKey
      in: header
      name: X-API-Key

    OAuth2:
      type: oauth2
      flows:
        clientCredentials:
          tokenUrl: https://auth.wia.org/oauth/token
          scopes:
            read: 읽기 권한
            write: 쓰기 권한
            admin: 관리자 권한
```

### API 아키텍처

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WIA 콘텐츠 AI API 아키텍처                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  클라이언트                                                                   │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐            │
│  │  웹 앱     │  │ 모바일 앱  │  │ 서버 통합   │  │ CLI 도구   │            │
│  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘            │
│        │               │               │               │                    │
│        └───────────────┴───────────────┴───────────────┘                    │
│                               │                                              │
│                               ▼                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                        API 게이트웨이                               │    │
│  │  • 인증/인가    • 요청 제한    • 로드 밸런싱    • 캐싱              │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                               │                                              │
│        ┌──────────────────────┼──────────────────────┐                      │
│        ▼                      ▼                      ▼                      │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐              │
│  │  서명 서비스  │      │  검증 서비스  │      │  탐지 서비스  │              │
│  └──────────────┘      └──────────────┘      └──────────────┘              │
│        │                      │                      │                      │
│        ▼                      ▼                      ▼                      │
│  ┌──────────────────────────────────────────────────────────────────┐      │
│  │                        공유 인프라                                 │      │
│  │  • PostgreSQL    • Redis    • S3    • HSM                        │      │
│  └──────────────────────────────────────────────────────────────────┘      │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4.2 서명 API

### 콘텐츠 서명

```yaml
paths:
  /sign:
    post:
      tags:
        - 서명
      summary: 콘텐츠에 자격 증명 서명
      description: |
        콘텐츠에 암호화 서명을 생성하고 자격 증명을 반환합니다.
        지원 형식: 이미지, 비디오, 오디오, 텍스트, PDF
      operationId: signContent
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              type: object
              required:
                - content
              properties:
                content:
                  type: string
                  format: binary
                  description: 서명할 콘텐츠 파일
                assertions:
                  type: array
                  items:
                    $ref: '#/components/schemas/Assertion'
                  description: 포함할 어설션 목록
                options:
                  $ref: '#/components/schemas/SigningOptions'
      responses:
        '200':
          description: 성공적으로 서명됨
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/SigningResponse'
        '400':
          $ref: '#/components/responses/BadRequest'
        '401':
          $ref: '#/components/responses/Unauthorized'
        '413':
          $ref: '#/components/responses/PayloadTooLarge'

components:
  schemas:
    SigningOptions:
      type: object
      properties:
        algorithm:
          type: string
          enum: [Ed25519, ES256, RS256]
          default: Ed25519
        embed_credential:
          type: boolean
          default: true
          description: 자격 증명을 콘텐츠에 임베딩
        timestamp:
          type: boolean
          default: true
          description: 타임스탬프 토큰 포함
        watermark:
          type: object
          properties:
            enabled:
              type: boolean
            algorithm:
              type: string
              enum: [DCT, DWT, Neural]
            strength:
              type: number
              minimum: 0
              maximum: 1

    SigningResponse:
      type: object
      properties:
        credential_id:
          type: string
          format: uuid
        content_hash:
          type: string
        signature:
          type: string
        signed_at:
          type: string
          format: date-time
        credential_url:
          type: string
          format: uri
        embedded_content:
          type: string
          format: binary
          description: 자격 증명이 임베딩된 콘텐츠 (선택적)
```

### TypeScript SDK 구현

```typescript
// SDK 클라이언트 구현
import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import FormData from 'form-data';
import { createReadStream } from 'fs';

interface ContentAIClientConfig {
  baseUrl: string;
  apiKey: string;
  timeout?: number;
  retryAttempts?: number;
}

interface SigningOptions {
  algorithm?: 'Ed25519' | 'ES256' | 'RS256';
  embedCredential?: boolean;
  timestamp?: boolean;
  watermark?: {
    enabled: boolean;
    algorithm?: 'DCT' | 'DWT' | 'Neural';
    strength?: number;
  };
}

interface Assertion {
  type: string;
  data: Record<string, unknown>;
}

interface SigningResponse {
  credentialId: string;
  contentHash: string;
  signature: string;
  signedAt: string;
  credentialUrl: string;
  embeddedContent?: Buffer;
}

export class ContentAIClient {
  private client: AxiosInstance;
  private config: ContentAIClientConfig;

  constructor(config: ContentAIClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'X-API-Key': config.apiKey,
        'Accept': 'application/json'
      }
    });

    this.setupInterceptors();
  }

  private setupInterceptors(): void {
    // 요청 인터셉터
    this.client.interceptors.request.use(
      (config) => {
        config.headers['X-Request-ID'] = this.generateRequestId();
        return config;
      },
      (error) => Promise.reject(error)
    );

    // 응답 인터셉터 (재시도 로직)
    this.client.interceptors.response.use(
      (response) => response,
      async (error) => {
        const config = error.config;

        if (!config || !config.retry) {
          config.retry = 0;
        }

        if (
          config.retry < (this.config.retryAttempts || 3) &&
          this.isRetryable(error)
        ) {
          config.retry += 1;
          const delay = Math.pow(2, config.retry) * 1000;
          await this.sleep(delay);
          return this.client(config);
        }

        return Promise.reject(error);
      }
    );
  }

  private isRetryable(error: any): boolean {
    return (
      error.code === 'ECONNABORTED' ||
      error.code === 'ETIMEDOUT' ||
      (error.response && error.response.status >= 500)
    );
  }

  private generateRequestId(): string {
    return `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  // 콘텐츠 서명
  async signContent(
    content: Buffer | string,
    assertions: Assertion[] = [],
    options: SigningOptions = {}
  ): Promise<SigningResponse> {
    const formData = new FormData();

    if (typeof content === 'string') {
      formData.append('content', createReadStream(content));
    } else {
      formData.append('content', content, {
        filename: 'content',
        contentType: 'application/octet-stream'
      });
    }

    if (assertions.length > 0) {
      formData.append('assertions', JSON.stringify(assertions));
    }

    formData.append('options', JSON.stringify(options));

    const response = await this.client.post('/sign', formData, {
      headers: formData.getHeaders()
    });

    return {
      credentialId: response.data.credential_id,
      contentHash: response.data.content_hash,
      signature: response.data.signature,
      signedAt: response.data.signed_at,
      credentialUrl: response.data.credential_url,
      embeddedContent: response.data.embedded_content
        ? Buffer.from(response.data.embedded_content, 'base64')
        : undefined
    };
  }

  // AI 생성 어설션 추가 서명
  async signWithAIAssertion(
    content: Buffer | string,
    aiInfo: {
      generatorName: string;
      modelName: string;
      generationParams?: Record<string, unknown>;
    },
    options: SigningOptions = {}
  ): Promise<SigningResponse> {
    const aiAssertion: Assertion = {
      type: 'ai_generation',
      data: {
        ai_generated: true,
        generator_info: {
          name: aiInfo.generatorName,
          model: aiInfo.modelName,
          version: '1.0'
        },
        generation_parameters: aiInfo.generationParams || {},
        timestamp: new Date().toISOString()
      }
    };

    return this.signContent(content, [aiAssertion], options);
  }
}
```

---

## 4.3 검증 API

### 자격 증명 검증

```yaml
paths:
  /verify:
    post:
      tags:
        - 검증
      summary: 콘텐츠 자격 증명 검증
      description: |
        콘텐츠의 자격 증명을 검증하고 결과를 반환합니다.
        서명 유효성, 인증서 체인, 콘텐츠 무결성을 검사합니다.
      operationId: verifyContent
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              type: object
              required:
                - content
              properties:
                content:
                  type: string
                  format: binary
                credential:
                  type: string
                  description: 별도 자격 증명 (임베딩되지 않은 경우)
                options:
                  $ref: '#/components/schemas/VerificationOptions'
      responses:
        '200':
          description: 검증 완료
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/VerificationResponse'

components:
  schemas:
    VerificationOptions:
      type: object
      properties:
        check_revocation:
          type: boolean
          default: true
        check_timestamp:
          type: boolean
          default: true
        trust_anchors:
          type: array
          items:
            type: string
          description: 사용자 지정 신뢰 앵커 ID

    VerificationResponse:
      type: object
      properties:
        valid:
          type: boolean
        credential_id:
          type: string
        content_hash:
          type: string
        signed_at:
          type: string
          format: date-time
        signer:
          $ref: '#/components/schemas/SignerInfo'
        assertions:
          type: array
          items:
            $ref: '#/components/schemas/VerifiedAssertion'
        certificate_chain:
          type: array
          items:
            $ref: '#/components/schemas/CertificateInfo'
        errors:
          type: array
          items:
            $ref: '#/components/schemas/VerificationError'
        warnings:
          type: array
          items:
            $ref: '#/components/schemas/VerificationWarning'

    SignerInfo:
      type: object
      properties:
        name:
          type: string
        organization:
          type: string
        verified:
          type: boolean

    CertificateInfo:
      type: object
      properties:
        subject:
          type: string
        issuer:
          type: string
        valid_from:
          type: string
          format: date-time
        valid_to:
          type: string
          format: date-time
        is_trust_anchor:
          type: boolean
```

### 검증 서비스 구현

```typescript
// 검증 서비스 구현
interface VerificationResult {
  valid: boolean;
  credentialId: string;
  contentHash: string;
  signedAt: Date;
  signer: SignerInfo;
  assertions: VerifiedAssertion[];
  certificateChain: CertificateInfo[];
  errors: VerificationError[];
  warnings: VerificationWarning[];
}

interface SignerInfo {
  name: string;
  organization: string;
  verified: boolean;
}

interface VerifiedAssertion {
  type: string;
  valid: boolean;
  data: Record<string, unknown>;
}

interface CertificateInfo {
  subject: string;
  issuer: string;
  validFrom: Date;
  validTo: Date;
  isTrustAnchor: boolean;
}

interface VerificationError {
  code: string;
  message: string;
  field?: string;
}

interface VerificationWarning {
  code: string;
  message: string;
}

// ContentAIClient에 검증 메서드 추가
export class ContentAIClient {
  // ... 이전 코드 ...

  async verifyContent(
    content: Buffer | string,
    credential?: string,
    options: VerificationOptions = {}
  ): Promise<VerificationResult> {
    const formData = new FormData();

    if (typeof content === 'string') {
      formData.append('content', createReadStream(content));
    } else {
      formData.append('content', content, {
        filename: 'content',
        contentType: 'application/octet-stream'
      });
    }

    if (credential) {
      formData.append('credential', credential);
    }

    formData.append('options', JSON.stringify(options));

    const response = await this.client.post('/verify', formData, {
      headers: formData.getHeaders()
    });

    return this.mapVerificationResponse(response.data);
  }

  private mapVerificationResponse(data: any): VerificationResult {
    return {
      valid: data.valid,
      credentialId: data.credential_id,
      contentHash: data.content_hash,
      signedAt: new Date(data.signed_at),
      signer: {
        name: data.signer?.name || '',
        organization: data.signer?.organization || '',
        verified: data.signer?.verified || false
      },
      assertions: data.assertions?.map((a: any) => ({
        type: a.type,
        valid: a.valid,
        data: a.data
      })) || [],
      certificateChain: data.certificate_chain?.map((c: any) => ({
        subject: c.subject,
        issuer: c.issuer,
        validFrom: new Date(c.valid_from),
        validTo: new Date(c.valid_to),
        isTrustAnchor: c.is_trust_anchor
      })) || [],
      errors: data.errors || [],
      warnings: data.warnings || []
    };
  }

  // 빠른 검증 (해시 확인만)
  async quickVerify(contentHash: string): Promise<{
    found: boolean;
    credentialId?: string;
    signedAt?: Date;
  }> {
    const response = await this.client.get(`/verify/hash/${contentHash}`);

    return {
      found: response.data.found,
      credentialId: response.data.credential_id,
      signedAt: response.data.signed_at
        ? new Date(response.data.signed_at)
        : undefined
    };
  }
}

interface VerificationOptions {
  checkRevocation?: boolean;
  checkTimestamp?: boolean;
  trustAnchors?: string[];
}
```

---

## 4.4 탐지 API

### AI 콘텐츠 탐지

```yaml
paths:
  /detect:
    post:
      tags:
        - 탐지
      summary: AI 생성 콘텐츠 탐지
      description: |
        콘텐츠가 AI에 의해 생성되었는지 탐지합니다.
        이미지, 비디오, 오디오, 텍스트를 지원합니다.
      operationId: detectAIContent
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              type: object
              required:
                - content
              properties:
                content:
                  type: string
                  format: binary
                content_type:
                  type: string
                  enum: [image, video, audio, text]
                options:
                  $ref: '#/components/schemas/DetectionOptions'
          application/json:
            schema:
              type: object
              required:
                - text
              properties:
                text:
                  type: string
                  minLength: 10
                  maxLength: 100000
                options:
                  $ref: '#/components/schemas/DetectionOptions'
      responses:
        '200':
          description: 탐지 완료
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DetectionResponse'

components:
  schemas:
    DetectionOptions:
      type: object
      properties:
        models:
          type: array
          items:
            type: string
          description: 사용할 탐지 모델 목록
        threshold:
          type: number
          minimum: 0
          maximum: 1
          default: 0.5
        include_details:
          type: boolean
          default: true
        include_frequency_analysis:
          type: boolean
          default: false

    DetectionResponse:
      type: object
      properties:
        content_type:
          type: string
        is_ai_generated:
          type: boolean
        confidence:
          type: number
        verdict:
          type: string
          enum: [ai_generated, human_created, uncertain, manipulated]
        models_used:
          type: array
          items:
            $ref: '#/components/schemas/ModelResult'
        details:
          $ref: '#/components/schemas/DetectionDetails'
        processing_time_ms:
          type: integer

    ModelResult:
      type: object
      properties:
        model_name:
          type: string
        version:
          type: string
        score:
          type: number
        label:
          type: string
        weight:
          type: number

    DetectionDetails:
      type: object
      properties:
        frequency_analysis:
          type: object
        suspicious_regions:
          type: array
          items:
            type: object
        generator_prediction:
          type: object
```

### 탐지 클라이언트 구현

```typescript
// 탐지 API 클라이언트
interface DetectionResult {
  contentType: 'image' | 'video' | 'audio' | 'text';
  isAIGenerated: boolean;
  confidence: number;
  verdict: 'ai_generated' | 'human_created' | 'uncertain' | 'manipulated';
  modelsUsed: ModelResult[];
  details?: DetectionDetails;
  processingTimeMs: number;
}

interface ModelResult {
  modelName: string;
  version: string;
  score: number;
  label: string;
  weight: number;
}

interface DetectionDetails {
  frequencyAnalysis?: {
    highFrequencyRatio: number;
    spectralFlatness: number;
    periodicityScore: number;
  };
  suspiciousRegions?: Array<{
    x: number;
    y: number;
    width: number;
    height: number;
    confidence: number;
    type: string;
  }>;
  generatorPrediction?: {
    predictedGenerator: string;
    confidence: number;
    alternatives: Array<{
      generator: string;
      probability: number;
    }>;
  };
  textAnalysis?: {
    perplexity: number;
    burstiness: number;
    vocabularyRichness: number;
  };
}

interface DetectionOptions {
  models?: string[];
  threshold?: number;
  includeDetails?: boolean;
  includeFrequencyAnalysis?: boolean;
}

// ContentAIClient에 탐지 메서드 추가
export class ContentAIClient {
  // ... 이전 코드 ...

  // 이미지 AI 탐지
  async detectImage(
    image: Buffer | string,
    options: DetectionOptions = {}
  ): Promise<DetectionResult> {
    const formData = new FormData();

    if (typeof image === 'string') {
      formData.append('content', createReadStream(image));
    } else {
      formData.append('content', image, {
        filename: 'image',
        contentType: 'image/jpeg'
      });
    }

    formData.append('content_type', 'image');
    formData.append('options', JSON.stringify(options));

    const response = await this.client.post('/detect', formData, {
      headers: formData.getHeaders()
    });

    return this.mapDetectionResponse(response.data);
  }

  // 텍스트 AI 탐지
  async detectText(
    text: string,
    options: DetectionOptions = {}
  ): Promise<DetectionResult> {
    const response = await this.client.post('/detect', {
      text,
      options
    });

    return this.mapDetectionResponse(response.data);
  }

  // 비디오 AI 탐지
  async detectVideo(
    video: Buffer | string,
    options: DetectionOptions = {}
  ): Promise<DetectionResult> {
    const formData = new FormData();

    if (typeof video === 'string') {
      formData.append('content', createReadStream(video));
    } else {
      formData.append('content', video, {
        filename: 'video',
        contentType: 'video/mp4'
      });
    }

    formData.append('content_type', 'video');
    formData.append('options', JSON.stringify(options));

    const response = await this.client.post('/detect', formData, {
      headers: formData.getHeaders(),
      timeout: 120000 // 비디오는 처리 시간이 더 길어질 수 있음
    });

    return this.mapDetectionResponse(response.data);
  }

  // 오디오 AI 탐지
  async detectAudio(
    audio: Buffer | string,
    options: DetectionOptions = {}
  ): Promise<DetectionResult> {
    const formData = new FormData();

    if (typeof audio === 'string') {
      formData.append('content', createReadStream(audio));
    } else {
      formData.append('content', audio, {
        filename: 'audio',
        contentType: 'audio/wav'
      });
    }

    formData.append('content_type', 'audio');
    formData.append('options', JSON.stringify(options));

    const response = await this.client.post('/detect', formData, {
      headers: formData.getHeaders()
    });

    return this.mapDetectionResponse(response.data);
  }

  // 배치 탐지
  async detectBatch(
    items: Array<{
      content: Buffer | string;
      contentType: 'image' | 'video' | 'audio' | 'text';
      id: string;
    }>,
    options: DetectionOptions = {}
  ): Promise<Array<DetectionResult & { id: string }>> {
    const formData = new FormData();

    for (let i = 0; i < items.length; i++) {
      const item = items[i];

      if (item.contentType === 'text' && typeof item.content === 'string') {
        formData.append(`items[${i}][text]`, item.content);
      } else if (typeof item.content === 'string') {
        formData.append(`items[${i}][content]`, createReadStream(item.content));
      } else {
        formData.append(`items[${i}][content]`, item.content);
      }

      formData.append(`items[${i}][content_type]`, item.contentType);
      formData.append(`items[${i}][id]`, item.id);
    }

    formData.append('options', JSON.stringify(options));

    const response = await this.client.post('/detect/batch', formData, {
      headers: formData.getHeaders(),
      timeout: 300000 // 배치는 더 긴 타임아웃
    });

    return response.data.results.map((result: any) => ({
      ...this.mapDetectionResponse(result),
      id: result.id
    }));
  }

  private mapDetectionResponse(data: any): DetectionResult {
    return {
      contentType: data.content_type,
      isAIGenerated: data.is_ai_generated,
      confidence: data.confidence,
      verdict: data.verdict,
      modelsUsed: data.models_used?.map((m: any) => ({
        modelName: m.model_name,
        version: m.version,
        score: m.score,
        label: m.label,
        weight: m.weight
      })) || [],
      details: data.details ? {
        frequencyAnalysis: data.details.frequency_analysis ? {
          highFrequencyRatio: data.details.frequency_analysis.high_frequency_ratio,
          spectralFlatness: data.details.frequency_analysis.spectral_flatness,
          periodicityScore: data.details.frequency_analysis.periodicity_score
        } : undefined,
        suspiciousRegions: data.details.suspicious_regions,
        generatorPrediction: data.details.generator_prediction ? {
          predictedGenerator: data.details.generator_prediction.predicted_generator,
          confidence: data.details.generator_prediction.confidence,
          alternatives: data.details.generator_prediction.alternatives
        } : undefined,
        textAnalysis: data.details.text_analysis ? {
          perplexity: data.details.text_analysis.perplexity,
          burstiness: data.details.text_analysis.burstiness,
          vocabularyRichness: data.details.text_analysis.vocabulary_richness
        } : undefined
      } : undefined,
      processingTimeMs: data.processing_time_ms
    };
  }
}
```

---

## 4.5 사용 예제

### 전체 워크플로우 예제

```typescript
// 전체 사용 예제
import { ContentAIClient } from '@wia/content-ai-sdk';
import { readFileSync } from 'fs';

async function main() {
  // 클라이언트 초기화
  const client = new ContentAIClient({
    baseUrl: 'https://api.contentai.wia.org/v1',
    apiKey: process.env.WIA_API_KEY!,
    timeout: 60000
  });

  // 1. AI 생성 이미지 서명
  console.log('1. AI 생성 이미지 서명...');
  const imageBuffer = readFileSync('./ai-generated-image.png');

  const signResult = await client.signWithAIAssertion(
    imageBuffer,
    {
      generatorName: 'DALL-E',
      modelName: 'dall-e-3',
      generationParams: {
        prompt_hash: 'sha256:abc123...',
        quality: 'hd',
        size: '1024x1024'
      }
    },
    {
      algorithm: 'Ed25519',
      embedCredential: true,
      timestamp: true,
      watermark: {
        enabled: true,
        algorithm: 'Neural',
        strength: 0.3
      }
    }
  );

  console.log(`자격 증명 ID: ${signResult.credentialId}`);
  console.log(`콘텐츠 해시: ${signResult.contentHash}`);
  console.log(`서명 시간: ${signResult.signedAt}`);

  // 2. 서명된 콘텐츠 검증
  console.log('\n2. 서명된 콘텐츠 검증...');
  const verifyResult = await client.verifyContent(
    signResult.embeddedContent!,
    undefined,
    {
      checkRevocation: true,
      checkTimestamp: true
    }
  );

  console.log(`유효성: ${verifyResult.valid}`);
  console.log(`서명자: ${verifyResult.signer.name}`);
  console.log(`어설션 수: ${verifyResult.assertions.length}`);

  if (verifyResult.errors.length > 0) {
    console.log('오류:', verifyResult.errors);
  }

  // 3. 알려지지 않은 이미지 탐지
  console.log('\n3. 알려지지 않은 이미지 AI 탐지...');
  const unknownImage = readFileSync('./unknown-image.jpg');

  const detectResult = await client.detectImage(unknownImage, {
    models: ['cnn_v2', 'frequency_v1'],
    threshold: 0.5,
    includeDetails: true,
    includeFrequencyAnalysis: true
  });

  console.log(`AI 생성 여부: ${detectResult.isAIGenerated}`);
  console.log(`신뢰도: ${(detectResult.confidence * 100).toFixed(1)}%`);
  console.log(`판정: ${detectResult.verdict}`);
  console.log(`처리 시간: ${detectResult.processingTimeMs}ms`);

  if (detectResult.details?.generatorPrediction) {
    console.log(`예측 생성기: ${detectResult.details.generatorPrediction.predictedGenerator}`);
  }

  // 4. 텍스트 AI 탐지
  console.log('\n4. 텍스트 AI 탐지...');
  const textContent = `
    인공지능은 21세기 가장 혁신적인 기술 중 하나입니다.
    기계 학습과 딥러닝의 발전으로 AI는 다양한 분야에서
    인간의 능력을 보완하고 때로는 능가하고 있습니다.
  `;

  const textDetectResult = await client.detectText(textContent.trim(), {
    includeDetails: true
  });

  console.log(`AI 생성 여부: ${textDetectResult.isAIGenerated}`);
  console.log(`신뢰도: ${(textDetectResult.confidence * 100).toFixed(1)}%`);

  if (textDetectResult.details?.textAnalysis) {
    console.log(`퍼플렉서티: ${textDetectResult.details.textAnalysis.perplexity}`);
    console.log(`버스티니스: ${textDetectResult.details.textAnalysis.burstiness}`);
  }

  // 5. 배치 탐지
  console.log('\n5. 배치 탐지...');
  const batchResults = await client.detectBatch([
    { content: './image1.jpg', contentType: 'image', id: 'img1' },
    { content: './image2.png', contentType: 'image', id: 'img2' },
    { content: '이 텍스트는 AI가 생성한 것인가요?', contentType: 'text', id: 'txt1' }
  ]);

  for (const result of batchResults) {
    console.log(`${result.id}: AI=${result.isAIGenerated}, 신뢰도=${(result.confidence * 100).toFixed(1)}%`);
  }
}

main().catch(console.error);
```

---

## 요약

WIA 콘텐츠 AI API는 다음을 제공합니다:

1. **서명 API** - C2PA 호환 자격 증명 생성
2. **검증 API** - 서명 및 인증서 체인 검증
3. **탐지 API** - 이미지, 비디오, 오디오, 텍스트 AI 탐지
4. **배치 처리** - 대량 콘텐츠 효율적 처리
5. **TypeScript SDK** - 개발자 친화적 클라이언트

---

*© 2025 세계산업협회 (WIA). 모든 권리 보유.*
*弘益人間 (홍익인간) · 널리 인간을 이롭게 하라*
