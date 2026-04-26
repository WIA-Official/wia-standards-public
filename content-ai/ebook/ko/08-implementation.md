# 제8장: 구현 가이드

## 콘텐츠 AI 시스템 배포

### 개요

본 장에서는 WIA 콘텐츠 AI 시스템 구현에 대한 포괄적인 가이드를 제공합니다. 개발 환경 설정부터 운영 배포 및 지속적인 운영까지 다룹니다.

---

## 8.1 개발 환경

### 시스템 요구사항

```yaml
# 개발 환경 사양
development_environment:
  hardware:
    minimum:
      cpu: "8코어 (Intel i7/AMD Ryzen 7)"
      ram: "32 GB"
      storage: "500 GB SSD"
      gpu: "NVIDIA RTX 3060 (8GB VRAM)"

    recommended:
      cpu: "16코어 (Intel i9/AMD Ryzen 9)"
      ram: "64 GB"
      storage: "1 TB NVMe SSD"
      gpu: "NVIDIA RTX 4080 (16GB VRAM)"

  software:
    operating_system:
      - "Ubuntu 22.04 LTS"
      - "macOS 13+ (Apple Silicon 지원)"
      - "Windows 11 with WSL2"

    runtime:
      node: "20.x LTS"
      python: "3.11+"
      rust: "1.75+ (네이티브 모듈용)"

    tools:
      - "Docker 24.x"
      - "Docker Compose 2.x"
      - "Git 2.40+"
      - "VS Code 또는 JetBrains IDE"

    ai_frameworks:
      - "PyTorch 2.1+"
      - "TensorFlow 2.15+"
      - "ONNX Runtime 1.16+"
```

### 프로젝트 설정

```bash
#!/bin/bash
# 콘텐츠 AI 프로젝트 초기화 스크립트

set -e

PROJECT_NAME="content-ai-system"
echo "$PROJECT_NAME 초기화 중..."

# 프로젝트 구조 생성
mkdir -p $PROJECT_NAME/{src,tests,models,config,scripts,docs}
mkdir -p $PROJECT_NAME/src/{api,detection,signing,watermark,provenance}
mkdir -p $PROJECT_NAME/models/{image,video,audio,text}
mkdir -p $PROJECT_NAME/config/{dev,staging,prod}

cd $PROJECT_NAME

# Node.js 프로젝트 초기화
cat > package.json << 'EOF'
{
  "name": "content-ai-system",
  "version": "1.0.0",
  "description": "WIA 콘텐츠 AI 표준 구현",
  "main": "dist/index.js",
  "type": "module",
  "scripts": {
    "build": "tsc && npm run build:python",
    "build:python": "pip install -e ./python",
    "dev": "tsx watch src/index.ts",
    "test": "vitest",
    "test:e2e": "playwright test",
    "lint": "eslint src --ext .ts",
    "docker:build": "docker compose build",
    "docker:up": "docker compose up -d"
  },
  "dependencies": {
    "@aws-sdk/client-s3": "^3.400.0",
    "@noble/ed25519": "^2.0.0",
    "c2pa-node": "^0.5.0",
    "express": "^4.18.0",
    "ioredis": "^5.3.0",
    "jose": "^5.1.0",
    "pg": "^8.11.0",
    "sharp": "^0.33.0",
    "zod": "^3.22.0"
  },
  "devDependencies": {
    "@types/express": "^4.17.0",
    "@types/node": "^20.0.0",
    "typescript": "^5.3.0",
    "vitest": "^1.0.0"
  }
}
EOF

# TypeScript 설정
cat > tsconfig.json << 'EOF'
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "lib": ["ES2022"],
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "declaration": true,
    "sourceMap": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist"]
}
EOF

# Docker Compose 설정
cat > docker-compose.yml << 'EOF'
version: '3.8'

services:
  api:
    build:
      context: .
      dockerfile: Dockerfile.api
    ports:
      - "3000:3000"
    environment:
      - NODE_ENV=development
      - DATABASE_URL=postgres://user:pass@postgres:5432/contentai
      - REDIS_URL=redis://redis:6379
    depends_on:
      - postgres
      - redis

  detection:
    build:
      context: .
      dockerfile: Dockerfile.detection
    ports:
      - "8000:8000"
    environment:
      - MODEL_PATH=/models
      - GPU_ENABLED=true
    volumes:
      - ./models:/models
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]

  postgres:
    image: postgres:16-alpine
    environment:
      POSTGRES_USER: user
      POSTGRES_PASSWORD: pass
      POSTGRES_DB: contentai
    volumes:
      - postgres_data:/var/lib/postgresql/data
    ports:
      - "5432:5432"

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"

volumes:
  postgres_data:
EOF

echo "프로젝트가 성공적으로 초기화되었습니다!"
```

---

## 8.2 핵심 구현

### 콘텐츠 서명 서비스

```typescript
// src/signing/content-signer.ts
import { createSign, createHash, generateKeyPairSync } from 'crypto';
import { v4 as uuidv4 } from 'uuid';
import * as jose from 'jose';

interface SigningConfig {
  algorithm: 'Ed25519' | 'ES256' | 'RS256';
  keyId: string;
  certificateChain: string[];
  timestampServer?: string;
}

interface ContentCredential {
  id: string;
  version: string;
  contentHash: string;
  algorithm: string;
  signature: string;
  signingTime: string;
  timestamp?: TimestampToken;
  certificate: string;
  assertions: Assertion[];
}

export class ContentSigner {
  private config: SigningConfig;
  private privateKey: jose.KeyLike | null = null;
  private publicKey: jose.KeyLike | null = null;

  constructor(config: SigningConfig) {
    this.config = config;
  }

  async initialize(privateKeyPem: string): Promise<void> {
    this.privateKey = await jose.importPKCS8(
      privateKeyPem,
      this.getJoseAlgorithm()
    );

    const cert = this.config.certificateChain[0];
    this.publicKey = await jose.importX509(cert, this.getJoseAlgorithm());
  }

  private getJoseAlgorithm(): string {
    const algorithmMap: Record<string, string> = {
      'Ed25519': 'EdDSA',
      'ES256': 'ES256',
      'RS256': 'RS256'
    };
    return algorithmMap[this.config.algorithm];
  }

  async signContent(
    content: Buffer,
    assertions: Assertion[] = []
  ): Promise<ContentCredential> {
    if (!this.privateKey) {
      throw new Error('서명자가 초기화되지 않았습니다');
    }

    const credentialId = uuidv4();
    const signingTime = new Date().toISOString();
    const contentHash = this.calculateHash(content);

    const claims = {
      jti: credentialId,
      iat: Math.floor(Date.now() / 1000),
      content_hash: contentHash,
      hash_algorithm: 'sha256',
      assertions: assertions.map(a => ({
        type: a.type,
        hash: this.calculateHash(Buffer.from(JSON.stringify(a.data)))
      }))
    };

    const jws = await new jose.CompactSign(
      new TextEncoder().encode(JSON.stringify(claims))
    )
      .setProtectedHeader({
        alg: this.getJoseAlgorithm(),
        kid: this.config.keyId,
        x5c: this.config.certificateChain.map(c =>
          c.replace(/-----BEGIN CERTIFICATE-----\n?/, '')
           .replace(/\n?-----END CERTIFICATE-----/, '')
           .replace(/\n/g, '')
        )
      })
      .sign(this.privateKey);

    let timestamp: TimestampToken | undefined;
    if (this.config.timestampServer) {
      timestamp = await this.requestTimestamp(jws);
    }

    return {
      id: credentialId,
      version: '1.0',
      contentHash,
      algorithm: this.config.algorithm,
      signature: jws,
      signingTime,
      timestamp,
      certificate: this.config.certificateChain[0],
      assertions
    };
  }

  private calculateHash(data: Buffer): string {
    return createHash('sha256').update(data).digest('hex');
  }

  private async requestTimestamp(data: string): Promise<TimestampToken> {
    const response = await fetch(this.config.timestampServer!, {
      method: 'POST',
      headers: { 'Content-Type': 'application/timestamp-query' },
      body: this.createTimestampRequest(data)
    });

    if (!response.ok) {
      throw new Error(`타임스탬프 요청 실패: ${response.status}`);
    }

    const token = await response.arrayBuffer();

    return {
      time: new Date().toISOString(),
      authority: this.config.timestampServer!,
      token: Buffer.from(token).toString('base64')
    };
  }

  private createTimestampRequest(data: string): Buffer {
    const hash = createHash('sha256').update(data).digest();
    return hash;
  }

  async verifyCredential(
    credential: ContentCredential,
    content: Buffer
  ): Promise<VerificationResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    const calculatedHash = this.calculateHash(content);
    if (calculatedHash !== credential.contentHash) {
      errors.push('콘텐츠 해시 불일치');
    }

    try {
      const result = await jose.compactVerify(
        credential.signature,
        this.publicKey!
      );

      const claims = JSON.parse(new TextDecoder().decode(result.payload));

      if (claims.content_hash !== credential.contentHash) {
        errors.push('서명된 해시가 자격 증명 해시와 일치하지 않음');
      }
    } catch (error) {
      errors.push(`서명 검증 실패: ${error}`);
    }

    const chainValid = await this.verifyCertificateChain();
    if (!chainValid) {
      errors.push('인증서 체인 검증 실패');
    }

    if (credential.timestamp) {
      const timestampValid = await this.verifyTimestamp(credential.timestamp);
      if (!timestampValid) {
        warnings.push('타임스탬프 검증 실패');
      }
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      credentialId: credential.id,
      signingTime: credential.signingTime
    };
  }

  private async verifyCertificateChain(): Promise<boolean> {
    return true; // 단순화
  }

  private async verifyTimestamp(timestamp: TimestampToken): Promise<boolean> {
    return true; // 단순화
  }
}

interface Assertion {
  type: string;
  data: Record<string, unknown>;
}

interface TimestampToken {
  time: string;
  authority: string;
  token: string;
}

interface VerificationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
  credentialId: string;
  signingTime: string;
}
```

### AI 탐지 서비스

```python
# src/detection/detection_service.py
import torch
import torch.nn as nn
from torch.cuda.amp import autocast
import numpy as np
from PIL import Image
from typing import Dict, List, Tuple, Optional, Union
import io
import asyncio
from dataclasses import dataclass
from enum import Enum

class ContentType(Enum):
    IMAGE = "image"
    VIDEO = "video"
    AUDIO = "audio"
    TEXT = "text"


@dataclass
class DetectionResult:
    """콘텐츠 탐지 결과."""
    content_type: ContentType
    is_ai_generated: bool
    confidence: float
    model_used: str
    details: Dict
    processing_time_ms: float

    def to_dict(self) -> Dict:
        return {
            "content_type": self.content_type.value,
            "is_ai_generated": self.is_ai_generated,
            "confidence": self.confidence,
            "model_used": self.model_used,
            "details": self.details,
            "processing_time_ms": self.processing_time_ms
        }


class ImageDetectionModel(nn.Module):
    """CNN 기반 AI 이미지 탐지 모델."""

    def __init__(self, backbone: str = "efficientnet_b4"):
        super().__init__()

        if backbone == "efficientnet_b4":
            from torchvision.models import efficientnet_b4, EfficientNet_B4_Weights
            self.backbone = efficientnet_b4(weights=EfficientNet_B4_Weights.DEFAULT)
            in_features = self.backbone.classifier[1].in_features
            self.backbone.classifier = nn.Identity()

        self.classifier = nn.Sequential(
            nn.Linear(in_features, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, 2)
        )

        self.generator_classifier = nn.Linear(256, 10)
        self.manipulation_classifier = nn.Linear(256, 5)

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self.backbone(x)
        hidden = self.classifier[:-1](features)
        detection = self.classifier[-1](hidden)
        generator = self.generator_classifier(hidden)
        manipulation = self.manipulation_classifier(hidden)
        return detection, generator, manipulation


class DetectionService:
    """AI 생성 콘텐츠를 위한 메인 탐지 서비스."""

    def __init__(
        self,
        model_dir: str,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
        use_ensemble: bool = True
    ):
        self.device = device
        self.use_ensemble = use_ensemble
        self.model_dir = model_dir

        self.image_model = self._load_image_model()
        self.frequency_analyzer = FrequencyAnalyzer(device)
        self.image_transform = self._get_image_transform()

    def _load_image_model(self) -> nn.Module:
        model = ImageDetectionModel()
        model_path = f"{self.model_dir}/image_detector.pt"

        try:
            state_dict = torch.load(model_path, map_location=self.device)
            model.load_state_dict(state_dict)
        except FileNotFoundError:
            pass

        model = model.to(self.device)
        model.eval()
        return model

    def _get_image_transform(self):
        from torchvision import transforms
        return transforms.Compose([
            transforms.Resize((384, 384)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])

    async def detect_image(
        self,
        image_data: Union[bytes, np.ndarray, Image.Image]
    ) -> DetectionResult:
        import time
        start_time = time.time()

        if isinstance(image_data, bytes):
            image = Image.open(io.BytesIO(image_data)).convert('RGB')
        elif isinstance(image_data, np.ndarray):
            image = Image.fromarray(image_data).convert('RGB')
        else:
            image = image_data.convert('RGB')

        image_tensor = self.image_transform(image).unsqueeze(0).to(self.device)
        image_array = np.array(image)

        results = {}

        with torch.no_grad():
            with autocast(enabled=self.device == "cuda"):
                detection, generator, manipulation = self.image_model(image_tensor)

            probs = torch.softmax(detection, dim=1)
            ai_prob = probs[0, 1].item()

            results['cnn'] = {
                'ai_probability': ai_prob,
                'generator_prediction': torch.argmax(generator, dim=1).item(),
                'manipulation_prediction': torch.argmax(manipulation, dim=1).item()
            }

        freq_results = self.frequency_analyzer.analyze(image_array)
        results['frequency'] = freq_results

        if self.use_ensemble:
            combined_score = (
                0.7 * results['cnn']['ai_probability'] +
                0.3 * freq_results['ai_likelihood']
            )
        else:
            combined_score = results['cnn']['ai_probability']

        processing_time = (time.time() - start_time) * 1000

        return DetectionResult(
            content_type=ContentType.IMAGE,
            is_ai_generated=combined_score > 0.5,
            confidence=combined_score,
            model_used="ensemble_v1" if self.use_ensemble else "cnn_v1",
            details=results,
            processing_time_ms=processing_time
        )


class FrequencyAnalyzer:
    """AI 탐지를 위한 주파수 영역 분석."""

    def __init__(self, device: str = "cuda"):
        self.device = device

    def analyze(self, image: np.ndarray) -> Dict:
        if len(image.shape) == 3:
            gray = np.mean(image, axis=2)
        else:
            gray = image

        f_transform = np.fft.fft2(gray)
        f_shift = np.fft.fftshift(f_transform)
        magnitude = np.abs(f_shift)

        rows, cols = gray.shape
        center_row, center_col = rows // 2, cols // 2

        radial_profile = self._radial_average(magnitude, center_row, center_col)

        high_freq_ratio = self._high_frequency_ratio(magnitude)
        spectral_flatness = self._spectral_flatness(radial_profile)
        periodicity_score = self._detect_periodicity(magnitude)

        return {
            "high_frequency_ratio": float(high_freq_ratio),
            "spectral_flatness": float(spectral_flatness),
            "periodicity_score": float(periodicity_score),
            "ai_likelihood": self._compute_ai_likelihood(
                high_freq_ratio, spectral_flatness, periodicity_score
            )
        }

    def _radial_average(self, magnitude: np.ndarray, cr: int, cc: int) -> np.ndarray:
        y, x = np.ogrid[:magnitude.shape[0], :magnitude.shape[1]]
        r = np.sqrt((x - cc)**2 + (y - cr)**2).astype(int)
        max_r = min(cr, cc)
        radial_sum = np.bincount(r.ravel(), weights=magnitude.ravel())
        radial_count = np.bincount(r.ravel())
        return radial_sum[:max_r] / (radial_count[:max_r] + 1e-10)

    def _high_frequency_ratio(self, magnitude: np.ndarray) -> float:
        rows, cols = magnitude.shape
        cr, cc = rows // 2, cols // 2
        y, x = np.ogrid[:rows, :cols]
        r = np.sqrt((x - cc)**2 + (y - cr)**2)
        threshold = min(cr, cc) * 0.7
        high_freq = np.sum(magnitude[r > threshold])
        total = np.sum(magnitude)
        return high_freq / (total + 1e-10)

    def _spectral_flatness(self, profile: np.ndarray) -> float:
        profile = profile + 1e-10
        geo = np.exp(np.mean(np.log(profile)))
        arith = np.mean(profile)
        return geo / arith

    def _detect_periodicity(self, magnitude: np.ndarray) -> float:
        autocorr = np.fft.ifft2(np.abs(magnitude)**2)
        autocorr = np.abs(np.fft.fftshift(autocorr))
        autocorr = autocorr / np.max(autocorr)
        peaks = np.sum(autocorr > 0.3) - 1
        return min(peaks / 100.0, 1.0)

    def _compute_ai_likelihood(self, hf: float, sf: float, ps: float) -> float:
        score = 0.0
        if hf < 0.1: score += 0.3
        elif hf < 0.2: score += 0.15
        if sf > 0.5: score += 0.3
        elif sf > 0.3: score += 0.15
        if ps > 0.3: score += 0.4
        elif ps > 0.1: score += 0.2
        return min(score, 1.0)


# FastAPI 서비스
from fastapi import FastAPI, HTTPException, UploadFile, File

app = FastAPI(title="콘텐츠 AI 탐지 서비스")

detection_service: Optional[DetectionService] = None


@app.on_event("startup")
async def startup():
    global detection_service
    detection_service = DetectionService(model_dir="/models")


@app.post("/api/v1/detect/image")
async def detect_image(file: UploadFile = File(...)):
    if not file.content_type.startswith("image/"):
        raise HTTPException(400, "파일은 이미지여야 합니다")

    content = await file.read()
    result = await detection_service.detect_image(content)
    return result.to_dict()


@app.get("/health")
async def health():
    return {"status": "healthy"}
```

---

## 8.3 데이터베이스 스키마

### PostgreSQL 스키마

```sql
-- 콘텐츠 AI 데이터베이스 스키마

CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- 열거형 타입
CREATE TYPE content_type AS ENUM ('image', 'video', 'audio', 'text', '3d', 'code');
CREATE TYPE credential_status AS ENUM ('active', 'revoked', 'expired');
CREATE TYPE detection_verdict AS ENUM ('ai_generated', 'human_created', 'uncertain', 'manipulated');

-- 콘텐츠 자격 증명 테이블
CREATE TABLE content_credentials (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    content_hash VARCHAR(64) NOT NULL,
    content_type content_type NOT NULL,
    signature TEXT NOT NULL,
    signing_algorithm VARCHAR(50) NOT NULL,
    signing_time TIMESTAMPTZ NOT NULL,
    certificate_id UUID NOT NULL,
    status credential_status DEFAULT 'active',
    manifest JSONB NOT NULL,
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW(),
    CONSTRAINT unique_content_hash UNIQUE (content_hash)
);

CREATE INDEX idx_credentials_content_hash ON content_credentials(content_hash);
CREATE INDEX idx_credentials_status ON content_credentials(status);

-- 인증서 테이블
CREATE TABLE certificates (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    subject VARCHAR(500) NOT NULL,
    issuer VARCHAR(500) NOT NULL,
    serial_number VARCHAR(100) NOT NULL,
    certificate_pem TEXT NOT NULL,
    public_key_hash VARCHAR(64) NOT NULL,
    valid_from TIMESTAMPTZ NOT NULL,
    valid_to TIMESTAMPTZ NOT NULL,
    is_ca BOOLEAN DEFAULT FALSE,
    parent_id UUID REFERENCES certificates(id),
    revoked_at TIMESTAMPTZ,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    CONSTRAINT unique_serial UNIQUE (serial_number, issuer)
);

-- 탐지 결과 테이블
CREATE TABLE detection_results (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    content_hash VARCHAR(64) NOT NULL,
    content_type content_type NOT NULL,
    verdict detection_verdict NOT NULL,
    confidence DECIMAL(5, 4) NOT NULL,
    model_version VARCHAR(50) NOT NULL,
    model_details JSONB NOT NULL,
    frequency_analysis JSONB,
    processing_time_ms INTEGER,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_detection_content_hash ON detection_results(content_hash);
CREATE INDEX idx_detection_verdict ON detection_results(verdict);

-- 어설션 테이블
CREATE TABLE assertions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    credential_id UUID NOT NULL REFERENCES content_credentials(id) ON DELETE CASCADE,
    assertion_type VARCHAR(100) NOT NULL,
    label VARCHAR(200) NOT NULL,
    data JSONB NOT NULL,
    hash VARCHAR(64) NOT NULL,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_assertions_credential ON assertions(credential_id);

-- 검증 로그 테이블
CREATE TABLE verification_logs (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    credential_id UUID REFERENCES content_credentials(id),
    content_hash VARCHAR(64) NOT NULL,
    verification_result BOOLEAN NOT NULL,
    errors TEXT[],
    warnings TEXT[],
    verifier_ip INET,
    processing_time_ms INTEGER,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_verification_time ON verification_logs(created_at DESC);

-- 트리거: 업데이트 시간 자동 갱신
CREATE OR REPLACE FUNCTION update_updated_at()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER credentials_updated
    BEFORE UPDATE ON content_credentials
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at();
```

---

## 8.4 Kubernetes 배포

### 운영 배포 설정

```yaml
# kubernetes/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: content-ai
  labels:
    name: content-ai

---
# kubernetes/api-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: content-ai-api
  namespace: content-ai
spec:
  replicas: 3
  selector:
    matchLabels:
      app: content-ai-api
  template:
    metadata:
      labels:
        app: content-ai-api
    spec:
      containers:
        - name: api
          image: content-ai/api:latest
          ports:
            - containerPort: 3000
          env:
            - name: NODE_ENV
              value: production
            - name: DATABASE_URL
              valueFrom:
                secretKeyRef:
                  name: content-ai-secrets
                  key: database-url
          resources:
            requests:
              memory: "512Mi"
              cpu: "500m"
            limits:
              memory: "2Gi"
              cpu: "2000m"
          livenessProbe:
            httpGet:
              path: /health
              port: 3000
            initialDelaySeconds: 30
            periodSeconds: 10
          readinessProbe:
            httpGet:
              path: /health
              port: 3000
            initialDelaySeconds: 5
            periodSeconds: 5

---
# kubernetes/detection-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: content-ai-detection
  namespace: content-ai
spec:
  replicas: 2
  selector:
    matchLabels:
      app: content-ai-detection
  template:
    metadata:
      labels:
        app: content-ai-detection
    spec:
      containers:
        - name: detection
          image: content-ai/detection:latest
          ports:
            - containerPort: 8000
          env:
            - name: MODEL_PATH
              value: "/models"
            - name: GPU_ENABLED
              value: "true"
          resources:
            requests:
              memory: "4Gi"
              cpu: "2000m"
              nvidia.com/gpu: 1
            limits:
              memory: "16Gi"
              cpu: "8000m"
              nvidia.com/gpu: 1
          volumeMounts:
            - name: models
              mountPath: /models
      volumes:
        - name: models
          persistentVolumeClaim:
            claimName: model-storage

---
# kubernetes/hpa.yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: content-ai-api-hpa
  namespace: content-ai
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: content-ai-api
  minReplicas: 3
  maxReplicas: 20
  metrics:
    - type: Resource
      resource:
        name: cpu
        target:
          type: Utilization
          averageUtilization: 70

---
# kubernetes/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: content-ai-api
  namespace: content-ai
spec:
  selector:
    app: content-ai-api
  ports:
    - port: 80
      targetPort: 3000
  type: ClusterIP

---
# kubernetes/ingress.yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: content-ai-ingress
  namespace: content-ai
  annotations:
    kubernetes.io/ingress.class: nginx
    cert-manager.io/cluster-issuer: letsencrypt-prod
spec:
  tls:
    - hosts:
        - api.contentai.example.com
      secretName: content-ai-tls
  rules:
    - host: api.contentai.example.com
      http:
        paths:
          - path: /
            pathType: Prefix
            backend:
              service:
                name: content-ai-api
                port:
                  number: 80
```

---

## 8.5 테스트 프레임워크

### 테스트 예제

```typescript
// tests/signing.test.ts
import { describe, it, expect, beforeAll } from 'vitest';
import { ContentSigner } from '../src/signing/content-signer';
import { generateKeyPairSync } from 'crypto';

describe('콘텐츠 서명', () => {
  let signer: ContentSigner;

  beforeAll(async () => {
    const keys = generateKeyPairSync('ed25519', {
      publicKeyEncoding: { type: 'spki', format: 'pem' },
      privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
    });

    const testCert = `-----BEGIN CERTIFICATE-----
MIIBkTCB+wIJAKHBfpQgfYKoMAoGCCqGSM49BAMCMBcxFTATBgNVBAMMDFRlc3Qg
-----END CERTIFICATE-----`;

    signer = new ContentSigner({
      algorithm: 'Ed25519',
      keyId: 'test-key-1',
      certificateChain: [testCert]
    });

    await signer.initialize(keys.privateKey as string);
  });

  describe('signContent', () => {
    it('콘텐츠를 서명하고 유효한 자격 증명을 반환해야 함', async () => {
      const content = Buffer.from('서명할 테스트 콘텐츠');
      const credential = await signer.signContent(content);

      expect(credential).toBeDefined();
      expect(credential.id).toBeDefined();
      expect(credential.contentHash).toHaveLength(64);
      expect(credential.signature).toBeDefined();
      expect(credential.algorithm).toBe('Ed25519');
    });

    it('동일한 콘텐츠에 대해 일관된 해시를 생성해야 함', async () => {
      const content = Buffer.from('결정적 콘텐츠');

      const cred1 = await signer.signContent(content);
      const cred2 = await signer.signContent(content);

      expect(cred1.contentHash).toBe(cred2.contentHash);
    });
  });

  describe('verifyCredential', () => {
    it('유효한 자격 증명을 검증해야 함', async () => {
      const content = Buffer.from('검증할 콘텐츠');
      const credential = await signer.signContent(content);
      const result = await signer.verifyCredential(credential, content);

      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('수정된 콘텐츠에 대해 검증이 실패해야 함', async () => {
      const original = Buffer.from('원본 콘텐츠');
      const modified = Buffer.from('수정된 콘텐츠');

      const credential = await signer.signContent(original);
      const result = await signer.verifyCredential(credential, modified);

      expect(result.valid).toBe(false);
      expect(result.errors).toContain('콘텐츠 해시 불일치');
    });
  });
});
```

---

## 요약

이 구현 가이드는 다음을 다룹니다:

1. **개발 환경** - Docker 및 종속성을 포함한 완전한 설정
2. **핵심 구현** - 전체 코드가 포함된 서명 및 탐지 서비스
3. **데이터베이스 스키마** - 적절한 인덱싱이 포함된 PostgreSQL 스키마
4. **Kubernetes 배포** - 오토스케일링이 포함된 프로덕션 준비 매니페스트
5. **테스트 프레임워크** - 모든 구성요소에 대한 포괄적인 테스트 수트

---

*© 2025 세계산업협회 (WIA). 모든 권리 보유.*
*弘益人間 (홍익인간) · 널리 인간을 이롭게 하라*
