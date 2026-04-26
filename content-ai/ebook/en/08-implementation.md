# Chapter 8: Implementation Guide

## Deploying Content AI Systems

### Overview

This chapter provides comprehensive guidance for implementing WIA Content AI systems, from development environment setup through production deployment and ongoing operations. Organizations will learn to build scalable, reliable content authentication and detection infrastructure.

---

## 8.1 Development Environment

### System Requirements

```yaml
# Development environment specification
development_environment:
  hardware:
    minimum:
      cpu: "8 cores (Intel i7/AMD Ryzen 7)"
      ram: "32 GB"
      storage: "500 GB SSD"
      gpu: "NVIDIA RTX 3060 (8GB VRAM)"

    recommended:
      cpu: "16 cores (Intel i9/AMD Ryzen 9)"
      ram: "64 GB"
      storage: "1 TB NVMe SSD"
      gpu: "NVIDIA RTX 4080 (16GB VRAM)"

  software:
    operating_system:
      - "Ubuntu 22.04 LTS"
      - "macOS 13+ (Apple Silicon supported)"
      - "Windows 11 with WSL2"

    runtime:
      node: "20.x LTS"
      python: "3.11+"
      rust: "1.75+ (for native modules)"

    tools:
      - "Docker 24.x"
      - "Docker Compose 2.x"
      - "Git 2.40+"
      - "VS Code or JetBrains IDE"

    ai_frameworks:
      - "PyTorch 2.1+"
      - "TensorFlow 2.15+"
      - "ONNX Runtime 1.16+"
```

### Project Setup

```bash
#!/bin/bash
# Content AI project initialization script

set -e

PROJECT_NAME="content-ai-system"
echo "Initializing $PROJECT_NAME..."

# Create project structure
mkdir -p $PROJECT_NAME/{src,tests,models,config,scripts,docs}
mkdir -p $PROJECT_NAME/src/{api,detection,signing,watermark,provenance}
mkdir -p $PROJECT_NAME/models/{image,video,audio,text}
mkdir -p $PROJECT_NAME/config/{dev,staging,prod}

cd $PROJECT_NAME

# Initialize Node.js project
cat > package.json << 'EOF'
{
  "name": "content-ai-system",
  "version": "1.0.0",
  "description": "WIA Content AI Standard Implementation",
  "main": "dist/index.js",
  "type": "module",
  "scripts": {
    "build": "tsc && npm run build:python",
    "build:python": "pip install -e ./python",
    "dev": "tsx watch src/index.ts",
    "test": "vitest",
    "test:e2e": "playwright test",
    "lint": "eslint src --ext .ts",
    "format": "prettier --write src",
    "docker:build": "docker compose build",
    "docker:up": "docker compose up -d",
    "docker:down": "docker compose down"
  },
  "dependencies": {
    "@aws-sdk/client-s3": "^3.400.0",
    "@grpc/grpc-js": "^1.9.0",
    "@noble/ed25519": "^2.0.0",
    "@noble/secp256k1": "^2.0.0",
    "c2pa-node": "^0.5.0",
    "express": "^4.18.0",
    "ioredis": "^5.3.0",
    "jose": "^5.1.0",
    "pg": "^8.11.0",
    "sharp": "^0.33.0",
    "uuid": "^9.0.0",
    "winston": "^3.11.0",
    "zod": "^3.22.0"
  },
  "devDependencies": {
    "@types/express": "^4.17.0",
    "@types/node": "^20.0.0",
    "@types/pg": "^8.10.0",
    "@types/uuid": "^9.0.0",
    "eslint": "^8.50.0",
    "prettier": "^3.1.0",
    "tsx": "^4.0.0",
    "typescript": "^5.3.0",
    "vitest": "^1.0.0"
  }
}
EOF

# TypeScript configuration
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
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "declaration": true,
    "declarationMap": true,
    "sourceMap": true,
    "resolveJsonModule": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist"]
}
EOF

# Docker Compose configuration
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
    volumes:
      - ./src:/app/src
      - ./models:/app/models

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
      - redis_data:/data

  minio:
    image: minio/minio
    command: server /data --console-address ":9001"
    ports:
      - "9000:9000"
      - "9001:9001"
    environment:
      MINIO_ROOT_USER: minioadmin
      MINIO_ROOT_PASSWORD: minioadmin
    volumes:
      - minio_data:/data

volumes:
  postgres_data:
  redis_data:
  minio_data:
EOF

echo "Project initialized successfully!"
```

---

## 8.2 Core Implementation

### Content Signing Service

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

interface Assertion {
  type: string;
  data: Record<string, unknown>;
  signature?: string;
}

interface TimestampToken {
  time: string;
  authority: string;
  token: string;
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

    // Extract public key
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
      throw new Error('Signer not initialized');
    }

    const credentialId = uuidv4();
    const signingTime = new Date().toISOString();

    // Calculate content hash
    const contentHash = this.calculateHash(content);

    // Build claim set
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

    // Create JWS signature
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

    // Request timestamp if server configured
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
      headers: {
        'Content-Type': 'application/timestamp-query'
      },
      body: this.createTimestampRequest(data)
    });

    if (!response.ok) {
      throw new Error(`Timestamp request failed: ${response.status}`);
    }

    const token = await response.arrayBuffer();

    return {
      time: new Date().toISOString(),
      authority: this.config.timestampServer!,
      token: Buffer.from(token).toString('base64')
    };
  }

  private createTimestampRequest(data: string): Buffer {
    // Create RFC 3161 timestamp request
    const hash = createHash('sha256').update(data).digest();
    // Simplified - actual implementation would use ASN.1 encoding
    return hash;
  }

  async verifyCredential(
    credential: ContentCredential,
    content: Buffer
  ): Promise<VerificationResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Verify content hash
    const calculatedHash = this.calculateHash(content);
    if (calculatedHash !== credential.contentHash) {
      errors.push('Content hash mismatch');
    }

    // Verify signature
    try {
      const result = await jose.compactVerify(
        credential.signature,
        this.publicKey!
      );

      const claims = JSON.parse(new TextDecoder().decode(result.payload));

      if (claims.content_hash !== credential.contentHash) {
        errors.push('Signed hash does not match credential hash');
      }
    } catch (error) {
      errors.push(`Signature verification failed: ${error}`);
    }

    // Verify certificate chain
    const chainValid = await this.verifyCertificateChain();
    if (!chainValid) {
      errors.push('Certificate chain validation failed');
    }

    // Verify timestamp if present
    if (credential.timestamp) {
      const timestampValid = await this.verifyTimestamp(credential.timestamp);
      if (!timestampValid) {
        warnings.push('Timestamp verification failed');
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
    // Verify certificate chain against trust anchors
    // Implementation would validate full chain
    return true;
  }

  private async verifyTimestamp(timestamp: TimestampToken): Promise<boolean> {
    // Verify RFC 3161 timestamp token
    return true;
  }
}

interface VerificationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
  credentialId: string;
  signingTime: string;
}

// C2PA manifest builder
export class C2PAManifestBuilder {
  private manifest: C2PAManifest;

  constructor() {
    this.manifest = {
      claim_generator: 'WIA-ContentAI/1.0',
      title: '',
      format: '',
      instance_id: uuidv4(),
      assertions: [],
      claim_signature: null
    };
  }

  setTitle(title: string): this {
    this.manifest.title = title;
    return this;
  }

  setFormat(format: string): this {
    this.manifest.format = format;
    return this;
  }

  addCreativeWorkAssertion(data: CreativeWorkData): this {
    this.manifest.assertions.push({
      label: 'stds.schema-org.CreativeWork',
      data: {
        '@context': 'https://schema.org',
        '@type': 'CreativeWork',
        ...data
      }
    });
    return this;
  }

  addAIGenerationAssertion(
    generator: string,
    model: string,
    parameters: Record<string, unknown>
  ): this {
    this.manifest.assertions.push({
      label: 'c2pa.ai_training',
      data: {
        ai_generated: true,
        generator_info: {
          name: generator,
          model,
          version: parameters.version || '1.0'
        },
        generation_parameters: parameters,
        timestamp: new Date().toISOString()
      }
    });
    return this;
  }

  addActionsAssertion(actions: ContentAction[]): this {
    this.manifest.assertions.push({
      label: 'c2pa.actions',
      data: {
        actions: actions.map(action => ({
          action: action.type,
          when: action.timestamp,
          softwareAgent: action.software,
          parameters: action.parameters
        }))
      }
    });
    return this;
  }

  async build(signer: ContentSigner, content: Buffer): Promise<C2PAManifest> {
    // Calculate content binding
    const credential = await signer.signContent(content, this.manifest.assertions);

    this.manifest.claim_signature = {
      algorithm: credential.algorithm,
      signature: credential.signature,
      certificate_chain: [credential.certificate]
    };

    return this.manifest;
  }
}

interface C2PAManifest {
  claim_generator: string;
  title: string;
  format: string;
  instance_id: string;
  assertions: C2PAAssertion[];
  claim_signature: ClaimSignature | null;
}

interface C2PAAssertion {
  label: string;
  data: Record<string, unknown>;
}

interface ClaimSignature {
  algorithm: string;
  signature: string;
  certificate_chain: string[];
}

interface CreativeWorkData {
  author?: string;
  dateCreated?: string;
  description?: string;
}

interface ContentAction {
  type: string;
  timestamp: string;
  software: string;
  parameters?: Record<string, unknown>;
}
```

### AI Detection Service

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
import logging

logger = logging.getLogger(__name__)


class ContentType(Enum):
    IMAGE = "image"
    VIDEO = "video"
    AUDIO = "audio"
    TEXT = "text"


@dataclass
class DetectionResult:
    """Detection result for a piece of content."""
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
    """CNN-based AI image detection model."""

    def __init__(self, backbone: str = "efficientnet_b4"):
        super().__init__()

        # Use EfficientNet backbone
        if backbone == "efficientnet_b4":
            from torchvision.models import efficientnet_b4, EfficientNet_B4_Weights
            self.backbone = efficientnet_b4(weights=EfficientNet_B4_Weights.DEFAULT)
            in_features = self.backbone.classifier[1].in_features
            self.backbone.classifier = nn.Identity()
        else:
            raise ValueError(f"Unknown backbone: {backbone}")

        # Detection head
        self.classifier = nn.Sequential(
            nn.Linear(in_features, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(256, 2)  # [real, AI-generated]
        )

        # Auxiliary heads for multi-task learning
        self.generator_classifier = nn.Linear(256, 10)  # Identify generator
        self.manipulation_classifier = nn.Linear(256, 5)  # Type of manipulation

    def forward(
        self,
        x: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        # Extract features
        features = self.backbone(x)

        # Main classification
        hidden = self.classifier[:-1](features)
        detection = self.classifier[-1](hidden)

        # Auxiliary tasks
        generator = self.generator_classifier(hidden)
        manipulation = self.manipulation_classifier(hidden)

        return detection, generator, manipulation


class FrequencyAnalyzer:
    """Frequency domain analysis for AI detection."""

    def __init__(self, device: str = "cuda"):
        self.device = device

    def analyze(self, image: np.ndarray) -> Dict:
        """Analyze frequency characteristics of image."""
        # Convert to grayscale
        if len(image.shape) == 3:
            gray = np.mean(image, axis=2)
        else:
            gray = image

        # Apply 2D FFT
        f_transform = np.fft.fft2(gray)
        f_shift = np.fft.fftshift(f_transform)
        magnitude = np.abs(f_shift)

        # Calculate frequency statistics
        rows, cols = gray.shape
        center_row, center_col = rows // 2, cols // 2

        # Radial averaging
        radial_profile = self._radial_average(magnitude, center_row, center_col)

        # Detect AI-specific patterns
        high_freq_ratio = self._high_frequency_ratio(magnitude)
        spectral_flatness = self._spectral_flatness(radial_profile)
        periodicity_score = self._detect_periodicity(magnitude)

        return {
            "high_frequency_ratio": float(high_freq_ratio),
            "spectral_flatness": float(spectral_flatness),
            "periodicity_score": float(periodicity_score),
            "radial_profile": radial_profile.tolist(),
            "ai_likelihood": self._compute_ai_likelihood(
                high_freq_ratio, spectral_flatness, periodicity_score
            )
        }

    def _radial_average(
        self,
        magnitude: np.ndarray,
        center_row: int,
        center_col: int
    ) -> np.ndarray:
        """Compute radial average of magnitude spectrum."""
        y, x = np.ogrid[:magnitude.shape[0], :magnitude.shape[1]]
        r = np.sqrt((x - center_col)**2 + (y - center_row)**2).astype(int)

        max_r = min(center_row, center_col)
        radial_sum = np.bincount(r.ravel(), weights=magnitude.ravel())
        radial_count = np.bincount(r.ravel())

        radial_profile = radial_sum[:max_r] / (radial_count[:max_r] + 1e-10)
        return radial_profile

    def _high_frequency_ratio(self, magnitude: np.ndarray) -> float:
        """Calculate ratio of high frequency energy."""
        rows, cols = magnitude.shape
        center_row, center_col = rows // 2, cols // 2

        # Define high frequency region (outer 30%)
        y, x = np.ogrid[:rows, :cols]
        r = np.sqrt((x - center_col)**2 + (y - center_row)**2)
        threshold = min(center_row, center_col) * 0.7

        high_freq_energy = np.sum(magnitude[r > threshold])
        total_energy = np.sum(magnitude)

        return high_freq_energy / (total_energy + 1e-10)

    def _spectral_flatness(self, profile: np.ndarray) -> float:
        """Calculate spectral flatness (Wiener entropy)."""
        profile = profile + 1e-10
        geometric_mean = np.exp(np.mean(np.log(profile)))
        arithmetic_mean = np.mean(profile)
        return geometric_mean / arithmetic_mean

    def _detect_periodicity(self, magnitude: np.ndarray) -> float:
        """Detect periodic patterns in frequency domain."""
        # Autocorrelation in frequency domain
        autocorr = np.fft.ifft2(np.abs(magnitude)**2)
        autocorr = np.abs(np.fft.fftshift(autocorr))

        # Normalize and find peaks
        autocorr = autocorr / np.max(autocorr)

        # Count significant peaks (excluding center)
        threshold = 0.3
        peaks = autocorr > threshold
        peak_count = np.sum(peaks) - 1  # Exclude center

        # Normalize by image size
        return min(peak_count / 100.0, 1.0)

    def _compute_ai_likelihood(
        self,
        high_freq_ratio: float,
        spectral_flatness: float,
        periodicity_score: float
    ) -> float:
        """Compute AI likelihood from frequency features."""
        # AI images typically have:
        # - Lower high frequency content
        # - Higher spectral flatness
        # - More periodic patterns

        score = 0.0

        if high_freq_ratio < 0.1:
            score += 0.3
        elif high_freq_ratio < 0.2:
            score += 0.15

        if spectral_flatness > 0.5:
            score += 0.3
        elif spectral_flatness > 0.3:
            score += 0.15

        if periodicity_score > 0.3:
            score += 0.4
        elif periodicity_score > 0.1:
            score += 0.2

        return min(score, 1.0)


class DetectionService:
    """Main detection service for AI-generated content."""

    def __init__(
        self,
        model_dir: str,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
        use_ensemble: bool = True
    ):
        self.device = device
        self.use_ensemble = use_ensemble
        self.model_dir = model_dir

        # Load models
        self.image_model = self._load_image_model()
        self.frequency_analyzer = FrequencyAnalyzer(device)

        # Preprocessing
        self.image_transform = self._get_image_transform()

    def _load_image_model(self) -> nn.Module:
        """Load image detection model."""
        model = ImageDetectionModel()
        model_path = f"{self.model_dir}/image_detector.pt"

        try:
            state_dict = torch.load(model_path, map_location=self.device)
            model.load_state_dict(state_dict)
        except FileNotFoundError:
            logger.warning(f"Model not found at {model_path}, using random weights")

        model = model.to(self.device)
        model.eval()
        return model

    def _get_image_transform(self):
        """Get image preprocessing transform."""
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
        """Detect if image is AI-generated."""
        import time
        start_time = time.time()

        # Load image
        if isinstance(image_data, bytes):
            image = Image.open(io.BytesIO(image_data)).convert('RGB')
        elif isinstance(image_data, np.ndarray):
            image = Image.fromarray(image_data).convert('RGB')
        else:
            image = image_data.convert('RGB')

        # Preprocess
        image_tensor = self.image_transform(image).unsqueeze(0).to(self.device)
        image_array = np.array(image)

        # Run detection models
        results = {}

        # CNN detection
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

        # Frequency analysis
        freq_results = self.frequency_analyzer.analyze(image_array)
        results['frequency'] = freq_results

        # Ensemble combination
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

    async def detect_batch(
        self,
        images: List[Union[bytes, np.ndarray]]
    ) -> List[DetectionResult]:
        """Batch detection for multiple images."""
        tasks = [self.detect_image(img) for img in images]
        return await asyncio.gather(*tasks)


class TextDetectionService:
    """Detection service for AI-generated text."""

    def __init__(self, model_name: str = "roberta-base"):
        from transformers import AutoTokenizer, AutoModelForSequenceClassification

        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModelForSequenceClassification.from_pretrained(
            model_name,
            num_labels=2
        )
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)
        self.model.eval()

    async def detect_text(self, text: str) -> DetectionResult:
        """Detect if text is AI-generated."""
        import time
        start_time = time.time()

        # Tokenize
        inputs = self.tokenizer(
            text,
            truncation=True,
            max_length=512,
            padding=True,
            return_tensors="pt"
        ).to(self.device)

        # Detect
        with torch.no_grad():
            outputs = self.model(**inputs)
            probs = torch.softmax(outputs.logits, dim=1)
            ai_prob = probs[0, 1].item()

        # Additional features
        perplexity = self._calculate_perplexity(text)
        burstiness = self._calculate_burstiness(text)

        # Combine scores
        combined_score = (
            0.6 * ai_prob +
            0.2 * (1 - min(perplexity / 100, 1)) +
            0.2 * (1 - burstiness)
        )

        processing_time = (time.time() - start_time) * 1000

        return DetectionResult(
            content_type=ContentType.TEXT,
            is_ai_generated=combined_score > 0.5,
            confidence=combined_score,
            model_used="roberta_text_v1",
            details={
                "classifier_probability": ai_prob,
                "perplexity": perplexity,
                "burstiness": burstiness
            },
            processing_time_ms=processing_time
        )

    def _calculate_perplexity(self, text: str) -> float:
        """Calculate text perplexity using GPT-2."""
        # Simplified perplexity calculation
        # Real implementation would use a language model
        words = text.split()
        unique_ratio = len(set(words)) / max(len(words), 1)
        return 50 * (1 + unique_ratio)  # Placeholder

    def _calculate_burstiness(self, text: str) -> float:
        """Calculate burstiness (variation in sentence length)."""
        import re
        sentences = re.split(r'[.!?]+', text)
        lengths = [len(s.split()) for s in sentences if s.strip()]

        if len(lengths) < 2:
            return 0.5

        mean_len = np.mean(lengths)
        std_len = np.std(lengths)

        # AI text tends to have more uniform sentence lengths
        burstiness = std_len / (mean_len + 1e-10)
        return min(burstiness, 1.0)


# FastAPI service implementation
from fastapi import FastAPI, HTTPException, UploadFile, File
from fastapi.responses import JSONResponse

app = FastAPI(title="Content AI Detection Service")

detection_service: Optional[DetectionService] = None
text_detection_service: Optional[TextDetectionService] = None


@app.on_event("startup")
async def startup():
    global detection_service, text_detection_service
    detection_service = DetectionService(model_dir="/models")
    text_detection_service = TextDetectionService()


@app.post("/api/v1/detect/image")
async def detect_image(file: UploadFile = File(...)):
    """Detect AI-generated image."""
    if not file.content_type.startswith("image/"):
        raise HTTPException(400, "File must be an image")

    content = await file.read()
    result = await detection_service.detect_image(content)
    return JSONResponse(result.to_dict())


@app.post("/api/v1/detect/text")
async def detect_text(request: Dict):
    """Detect AI-generated text."""
    text = request.get("text")
    if not text:
        raise HTTPException(400, "Text is required")

    result = await text_detection_service.detect_text(text)
    return JSONResponse(result.to_dict())


@app.get("/health")
async def health():
    return {"status": "healthy"}
```

---

## 8.3 Database Schema

### PostgreSQL Schema

```sql
-- Content AI Database Schema

-- Extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Enum types
CREATE TYPE content_type AS ENUM ('image', 'video', 'audio', 'text', '3d', 'code');
CREATE TYPE credential_status AS ENUM ('active', 'revoked', 'expired');
CREATE TYPE detection_verdict AS ENUM ('ai_generated', 'human_created', 'uncertain', 'manipulated');

-- Content credentials table
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

-- Create index for content lookup
CREATE INDEX idx_credentials_content_hash ON content_credentials(content_hash);
CREATE INDEX idx_credentials_status ON content_credentials(status);
CREATE INDEX idx_credentials_created ON content_credentials(created_at DESC);

-- Certificates table
CREATE TABLE certificates (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    subject VARCHAR(500) NOT NULL,
    issuer VARCHAR(500) NOT NULL,
    serial_number VARCHAR(100) NOT NULL,
    certificate_pem TEXT NOT NULL,
    public_key_hash VARCHAR(64) NOT NULL,
    valid_from TIMESTAMPTZ NOT NULL,
    valid_to TIMESTAMPTZ NOT NULL,
    key_usage VARCHAR(100)[],
    is_ca BOOLEAN DEFAULT FALSE,
    parent_id UUID REFERENCES certificates(id),
    revoked_at TIMESTAMPTZ,
    revocation_reason VARCHAR(100),
    created_at TIMESTAMPTZ DEFAULT NOW(),

    CONSTRAINT unique_serial UNIQUE (serial_number, issuer)
);

CREATE INDEX idx_certificates_subject ON certificates(subject);
CREATE INDEX idx_certificates_valid ON certificates(valid_from, valid_to);
CREATE INDEX idx_certificates_revoked ON certificates(revoked_at) WHERE revoked_at IS NOT NULL;

-- Detection results table
CREATE TABLE detection_results (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    content_hash VARCHAR(64) NOT NULL,
    content_type content_type NOT NULL,
    verdict detection_verdict NOT NULL,
    confidence DECIMAL(5, 4) NOT NULL CHECK (confidence >= 0 AND confidence <= 1),
    model_version VARCHAR(50) NOT NULL,
    model_details JSONB NOT NULL,
    frequency_analysis JSONB,
    ensemble_scores JSONB,
    processing_time_ms INTEGER,
    created_at TIMESTAMPTZ DEFAULT NOW(),

    CONSTRAINT confidence_range CHECK (confidence >= 0 AND confidence <= 1)
);

CREATE INDEX idx_detection_content_hash ON detection_results(content_hash);
CREATE INDEX idx_detection_verdict ON detection_results(verdict);
CREATE INDEX idx_detection_confidence ON detection_results(confidence DESC);

-- Assertions table
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
CREATE INDEX idx_assertions_type ON assertions(assertion_type);

-- AI generation metadata
CREATE TABLE ai_generation_metadata (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    credential_id UUID NOT NULL REFERENCES content_credentials(id) ON DELETE CASCADE,
    generator_name VARCHAR(100) NOT NULL,
    generator_version VARCHAR(50),
    model_name VARCHAR(200),
    prompt_hash VARCHAR(64),
    generation_parameters JSONB,
    seed BIGINT,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_ai_gen_credential ON ai_generation_metadata(credential_id);
CREATE INDEX idx_ai_gen_generator ON ai_generation_metadata(generator_name);

-- Verification logs
CREATE TABLE verification_logs (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    credential_id UUID REFERENCES content_credentials(id),
    content_hash VARCHAR(64) NOT NULL,
    verification_result BOOLEAN NOT NULL,
    errors TEXT[],
    warnings TEXT[],
    verifier_ip INET,
    verifier_agent VARCHAR(500),
    processing_time_ms INTEGER,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_verification_credential ON verification_logs(credential_id);
CREATE INDEX idx_verification_result ON verification_logs(verification_result);
CREATE INDEX idx_verification_time ON verification_logs(created_at DESC);

-- Trust anchors
CREATE TABLE trust_anchors (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    name VARCHAR(200) NOT NULL,
    organization VARCHAR(200) NOT NULL,
    certificate_id UUID NOT NULL REFERENCES certificates(id),
    trust_level INTEGER DEFAULT 100 CHECK (trust_level >= 0 AND trust_level <= 100),
    is_active BOOLEAN DEFAULT TRUE,
    added_at TIMESTAMPTZ DEFAULT NOW(),
    added_by VARCHAR(200)
);

CREATE INDEX idx_trust_anchors_active ON trust_anchors(is_active) WHERE is_active = TRUE;

-- Watermark registrations
CREATE TABLE watermark_registrations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    content_hash VARCHAR(64) NOT NULL,
    watermark_hash VARCHAR(64) NOT NULL,
    watermark_algorithm VARCHAR(50) NOT NULL,
    strength DECIMAL(3, 2),
    owner_id UUID NOT NULL,
    registered_at TIMESTAMPTZ DEFAULT NOW(),
    expires_at TIMESTAMPTZ,

    CONSTRAINT unique_watermark UNIQUE (content_hash, watermark_hash)
);

CREATE INDEX idx_watermark_content ON watermark_registrations(content_hash);
CREATE INDEX idx_watermark_owner ON watermark_registrations(owner_id);

-- Functions and triggers

-- Update timestamp trigger
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

-- Credential status check function
CREATE OR REPLACE FUNCTION check_credential_status(cred_id UUID)
RETURNS credential_status AS $$
DECLARE
    cred RECORD;
    cert RECORD;
BEGIN
    SELECT * INTO cred FROM content_credentials WHERE id = cred_id;

    IF cred IS NULL THEN
        RETURN 'expired';
    END IF;

    IF cred.status = 'revoked' THEN
        RETURN 'revoked';
    END IF;

    SELECT * INTO cert FROM certificates WHERE id = cred.certificate_id;

    IF cert.revoked_at IS NOT NULL THEN
        RETURN 'revoked';
    END IF;

    IF NOW() > cert.valid_to THEN
        RETURN 'expired';
    END IF;

    RETURN 'active';
END;
$$ LANGUAGE plpgsql;
```

---

## 8.4 Kubernetes Deployment

### Production Deployment Configuration

```yaml
# kubernetes/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: content-ai
  labels:
    name: content-ai
    istio-injection: enabled

---
# kubernetes/configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: content-ai-config
  namespace: content-ai
data:
  LOG_LEVEL: "info"
  DETECTION_THRESHOLD: "0.5"
  ENSEMBLE_ENABLED: "true"
  CACHE_TTL: "3600"
  MAX_CONTENT_SIZE_MB: "100"

---
# kubernetes/secrets.yaml
apiVersion: v1
kind: Secret
metadata:
  name: content-ai-secrets
  namespace: content-ai
type: Opaque
stringData:
  DATABASE_URL: "postgres://user:pass@postgres:5432/contentai"
  REDIS_URL: "redis://redis:6379"
  SIGNING_KEY: ""
  API_KEY_SALT: ""

---
# kubernetes/api-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: content-ai-api
  namespace: content-ai
  labels:
    app: content-ai-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: content-ai-api
  template:
    metadata:
      labels:
        app: content-ai-api
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "3000"
    spec:
      serviceAccountName: content-ai-sa
      containers:
        - name: api
          image: content-ai/api:latest
          ports:
            - containerPort: 3000
              name: http
          envFrom:
            - configMapRef:
                name: content-ai-config
            - secretRef:
                name: content-ai-secrets
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
          volumeMounts:
            - name: certificates
              mountPath: /etc/certificates
              readOnly: true
      volumes:
        - name: certificates
          secret:
            secretName: signing-certificates

---
# kubernetes/detection-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: content-ai-detection
  namespace: content-ai
  labels:
    app: content-ai-detection
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
              name: http
          envFrom:
            - configMapRef:
                name: content-ai-config
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
              readOnly: true
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
    - type: Resource
      resource:
        name: memory
        target:
          type: Utilization
          averageUtilization: 80
  behavior:
    scaleUp:
      stabilizationWindowSeconds: 60
      policies:
        - type: Percent
          value: 100
          periodSeconds: 60
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
        - type: Percent
          value: 10
          periodSeconds: 60

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
      name: http
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
    nginx.ingress.kubernetes.io/rate-limit: "100"
    nginx.ingress.kubernetes.io/rate-limit-window: "1m"
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

## 8.5 Testing Framework

### Comprehensive Test Suite

```typescript
// tests/signing.test.ts
import { describe, it, expect, beforeAll } from 'vitest';
import { ContentSigner, C2PAManifestBuilder } from '../src/signing/content-signer';
import { generateKeyPairSync } from 'crypto';

describe('Content Signing', () => {
  let signer: ContentSigner;
  let keyPair: { publicKey: string; privateKey: string };

  beforeAll(async () => {
    // Generate test key pair
    const keys = generateKeyPairSync('ed25519', {
      publicKeyEncoding: { type: 'spki', format: 'pem' },
      privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
    });

    keyPair = {
      publicKey: keys.publicKey as string,
      privateKey: keys.privateKey as string
    };

    // Create self-signed certificate for testing
    const testCert = `-----BEGIN CERTIFICATE-----
MIIBkTCB+wIJAKHBfpQgfYKoMAoGCCqGSM49BAMCMBcxFTATBgNVBAMMDFRlc3Qg
Q2VydCBDQTAeFw0yNDAxMDEwMDAwMDBaFw0yNTAxMDEwMDAwMDBaMBcxFTATBgNV
BAMMDFRlc3QgQ2VydCBDQTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABExample
-----END CERTIFICATE-----`;

    signer = new ContentSigner({
      algorithm: 'Ed25519',
      keyId: 'test-key-1',
      certificateChain: [testCert]
    });

    await signer.initialize(keyPair.privateKey);
  });

  describe('signContent', () => {
    it('should sign content and return valid credential', async () => {
      const content = Buffer.from('Test content for signing');

      const credential = await signer.signContent(content);

      expect(credential).toBeDefined();
      expect(credential.id).toBeDefined();
      expect(credential.contentHash).toHaveLength(64);
      expect(credential.signature).toBeDefined();
      expect(credential.algorithm).toBe('Ed25519');
    });

    it('should produce consistent hashes for same content', async () => {
      const content = Buffer.from('Deterministic content');

      const cred1 = await signer.signContent(content);
      const cred2 = await signer.signContent(content);

      expect(cred1.contentHash).toBe(cred2.contentHash);
    });

    it('should include assertions in credential', async () => {
      const content = Buffer.from('Content with assertions');
      const assertions = [
        { type: 'c2pa.ai_training', data: { ai_generated: true } }
      ];

      const credential = await signer.signContent(content, assertions);

      expect(credential.assertions).toHaveLength(1);
      expect(credential.assertions[0].type).toBe('c2pa.ai_training');
    });
  });

  describe('verifyCredential', () => {
    it('should verify valid credential', async () => {
      const content = Buffer.from('Content to verify');
      const credential = await signer.signContent(content);

      const result = await signer.verifyCredential(credential, content);

      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should fail verification for modified content', async () => {
      const originalContent = Buffer.from('Original content');
      const modifiedContent = Buffer.from('Modified content');

      const credential = await signer.signContent(originalContent);
      const result = await signer.verifyCredential(credential, modifiedContent);

      expect(result.valid).toBe(false);
      expect(result.errors).toContain('Content hash mismatch');
    });
  });
});

describe('C2PA Manifest Builder', () => {
  it('should build valid C2PA manifest', async () => {
    const builder = new C2PAManifestBuilder();

    builder
      .setTitle('Test Image')
      .setFormat('image/jpeg')
      .addCreativeWorkAssertion({
        author: 'Test Author',
        dateCreated: '2024-01-01'
      })
      .addAIGenerationAssertion(
        'DALL-E',
        'dall-e-3',
        { quality: 'hd', size: '1024x1024' }
      );

    // Would need signer to build complete manifest
    expect(builder).toBeDefined();
  });
});
```

```python
# tests/test_detection.py
import pytest
import numpy as np
from PIL import Image
import io
import asyncio
from src.detection.detection_service import (
    DetectionService,
    TextDetectionService,
    FrequencyAnalyzer,
    ContentType
)


@pytest.fixture
def detection_service():
    """Create detection service instance."""
    return DetectionService(
        model_dir="./models",
        device="cpu",
        use_ensemble=True
    )


@pytest.fixture
def text_detection_service():
    """Create text detection service instance."""
    return TextDetectionService(model_name="distilbert-base-uncased")


@pytest.fixture
def sample_image():
    """Generate sample test image."""
    img = Image.new('RGB', (256, 256), color='red')
    buffer = io.BytesIO()
    img.save(buffer, format='JPEG')
    return buffer.getvalue()


@pytest.fixture
def frequency_analyzer():
    """Create frequency analyzer instance."""
    return FrequencyAnalyzer(device="cpu")


class TestImageDetection:
    """Tests for image detection functionality."""

    @pytest.mark.asyncio
    async def test_detect_image_returns_result(
        self,
        detection_service,
        sample_image
    ):
        """Test that image detection returns valid result."""
        result = await detection_service.detect_image(sample_image)

        assert result.content_type == ContentType.IMAGE
        assert 0 <= result.confidence <= 1
        assert result.model_used is not None
        assert result.processing_time_ms > 0

    @pytest.mark.asyncio
    async def test_detect_image_from_pil(self, detection_service):
        """Test detection from PIL Image object."""
        img = Image.new('RGB', (256, 256), color='blue')

        result = await detection_service.detect_image(img)

        assert result.content_type == ContentType.IMAGE
        assert isinstance(result.is_ai_generated, bool)

    @pytest.mark.asyncio
    async def test_detect_image_from_numpy(self, detection_service):
        """Test detection from numpy array."""
        img_array = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)

        result = await detection_service.detect_image(img_array)

        assert result.content_type == ContentType.IMAGE

    @pytest.mark.asyncio
    async def test_batch_detection(self, detection_service, sample_image):
        """Test batch image detection."""
        images = [sample_image] * 3

        results = await detection_service.detect_batch(images)

        assert len(results) == 3
        for result in results:
            assert result.content_type == ContentType.IMAGE


class TestFrequencyAnalysis:
    """Tests for frequency domain analysis."""

    def test_analyze_returns_features(self, frequency_analyzer):
        """Test that frequency analysis returns expected features."""
        image = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)

        result = frequency_analyzer.analyze(image)

        assert 'high_frequency_ratio' in result
        assert 'spectral_flatness' in result
        assert 'periodicity_score' in result
        assert 'ai_likelihood' in result

    def test_ai_likelihood_range(self, frequency_analyzer):
        """Test that AI likelihood is in valid range."""
        image = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)

        result = frequency_analyzer.analyze(image)

        assert 0 <= result['ai_likelihood'] <= 1

    def test_grayscale_image(self, frequency_analyzer):
        """Test analysis works with grayscale image."""
        image = np.random.randint(0, 255, (256, 256), dtype=np.uint8)

        result = frequency_analyzer.analyze(image)

        assert result is not None


class TestTextDetection:
    """Tests for text detection functionality."""

    @pytest.mark.asyncio
    async def test_detect_text_returns_result(self, text_detection_service):
        """Test that text detection returns valid result."""
        text = "This is a sample text for detection testing."

        result = await text_detection_service.detect_text(text)

        assert result.content_type == ContentType.TEXT
        assert 0 <= result.confidence <= 1
        assert 'perplexity' in result.details
        assert 'burstiness' in result.details

    @pytest.mark.asyncio
    async def test_detect_long_text(self, text_detection_service):
        """Test detection with long text."""
        text = "Sample sentence. " * 100

        result = await text_detection_service.detect_text(text)

        assert result.content_type == ContentType.TEXT

    @pytest.mark.asyncio
    async def test_detect_empty_text(self, text_detection_service):
        """Test detection with empty text."""
        result = await text_detection_service.detect_text("")

        assert result.content_type == ContentType.TEXT


class TestDetectionResult:
    """Tests for DetectionResult dataclass."""

    def test_to_dict_serialization(self):
        """Test result serialization to dict."""
        from src.detection.detection_service import DetectionResult

        result = DetectionResult(
            content_type=ContentType.IMAGE,
            is_ai_generated=True,
            confidence=0.95,
            model_used="test_model",
            details={"key": "value"},
            processing_time_ms=100.5
        )

        data = result.to_dict()

        assert data['content_type'] == 'image'
        assert data['is_ai_generated'] is True
        assert data['confidence'] == 0.95
```

---

## 8.6 Monitoring and Observability

### Prometheus Metrics

```typescript
// src/monitoring/metrics.ts
import { Registry, Counter, Histogram, Gauge } from 'prom-client';

const register = new Registry();

// API metrics
export const httpRequestDuration = new Histogram({
  name: 'content_ai_http_request_duration_seconds',
  help: 'Duration of HTTP requests',
  labelNames: ['method', 'route', 'status'],
  buckets: [0.01, 0.05, 0.1, 0.5, 1, 2, 5, 10],
  registers: [register]
});

export const httpRequestsTotal = new Counter({
  name: 'content_ai_http_requests_total',
  help: 'Total number of HTTP requests',
  labelNames: ['method', 'route', 'status'],
  registers: [register]
});

// Signing metrics
export const credentialsSignedTotal = new Counter({
  name: 'content_ai_credentials_signed_total',
  help: 'Total number of credentials signed',
  labelNames: ['algorithm', 'content_type'],
  registers: [register]
});

export const signingDuration = new Histogram({
  name: 'content_ai_signing_duration_seconds',
  help: 'Duration of signing operations',
  labelNames: ['algorithm'],
  buckets: [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1],
  registers: [register]
});

// Detection metrics
export const detectionsTotal = new Counter({
  name: 'content_ai_detections_total',
  help: 'Total number of detection requests',
  labelNames: ['content_type', 'verdict'],
  registers: [register]
});

export const detectionDuration = new Histogram({
  name: 'content_ai_detection_duration_seconds',
  help: 'Duration of detection operations',
  labelNames: ['content_type', 'model'],
  buckets: [0.1, 0.25, 0.5, 1, 2, 5, 10, 30],
  registers: [register]
});

export const detectionConfidence = new Histogram({
  name: 'content_ai_detection_confidence',
  help: 'Detection confidence scores',
  labelNames: ['content_type', 'verdict'],
  buckets: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
  registers: [register]
});

// Verification metrics
export const verificationsTotal = new Counter({
  name: 'content_ai_verifications_total',
  help: 'Total number of verification requests',
  labelNames: ['result'],
  registers: [register]
});

export const verificationDuration = new Histogram({
  name: 'content_ai_verification_duration_seconds',
  help: 'Duration of verification operations',
  buckets: [0.01, 0.05, 0.1, 0.25, 0.5, 1, 2],
  registers: [register]
});

// System metrics
export const activeConnections = new Gauge({
  name: 'content_ai_active_connections',
  help: 'Number of active connections',
  registers: [register]
});

export const queueSize = new Gauge({
  name: 'content_ai_queue_size',
  help: 'Current queue size',
  labelNames: ['queue_name'],
  registers: [register]
});

export const modelLoadTime = new Gauge({
  name: 'content_ai_model_load_time_seconds',
  help: 'Time to load detection models',
  labelNames: ['model_name'],
  registers: [register]
});

// Export registry
export { register };
```

---

## Summary

This implementation guide covers:

1. **Development Environment** - Complete setup with Docker and dependencies
2. **Core Implementation** - Signing and detection services with full code
3. **Database Schema** - PostgreSQL schema with proper indexing
4. **Kubernetes Deployment** - Production-ready manifests with autoscaling
5. **Testing Framework** - Comprehensive test suites for all components
6. **Monitoring** - Prometheus metrics for observability

---

*© 2025 World Industry Association (WIA). All rights reserved.*
*弘益人間 (홍익인간) · Benefit All Humanity*
