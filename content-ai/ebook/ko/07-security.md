# 제7장: 보안 프레임워크

## 콘텐츠 AI 보안 아키텍처

### 개요

WIA 콘텐츠 AI 보안 프레임워크는 AI 생성 콘텐츠 인증 시스템에 대한 포괄적인 보호를 제공합니다. 적대적 공격, 자격 증명 위조, 프라이버시 침해 등의 위협에 대응하는 보안 아키텍처를 상세히 설명합니다.

---

## 7.1 위협 모델

### 공격 표면 분석

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    콘텐츠 AI 위협 지형도                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    콘텐츠 생성 공격                                   │    │
│  │  • 자격증명 주입    • 워터마크 제거    • 메타데이터 제거             │    │
│  │  • 서명 위조        • 타임스탬프 변조   • 귀속 도용                   │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    탐지 우회 공격                                     │    │
│  │  • 적대적 예제      • 탐지 우회       • 모델 포이즈닝                │    │
│  │  • 스타일 전이      • 노이즈 주입     • 압축 공격                    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    인프라 공격                                        │    │
│  │  • PKI 침해         • 검증 우회       • API 남용                     │    │
│  │  • 인증서 도용      • 신뢰 앵커 공격   • DDoS                        │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    프라이버시 공격                                    │    │
│  │  • 익명성 해제      • 자격증명 추적    • 메타데이터 유출             │    │
│  │  • 창작자 프로파일링 • 사용 감시       • 상관관계 공격               │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 위협 분류 및 평가

```typescript
// 위협 분류 및 위험 평가
interface ThreatCategory {
  id: string;
  name: string;
  description: string;
  attackVectors: AttackVector[];
  riskLevel: 'critical' | 'high' | 'medium' | 'low';
  mitigations: Mitigation[];
}

interface AttackVector {
  id: string;
  name: string;
  technique: string;
  difficulty: 'trivial' | 'low' | 'medium' | 'high' | 'expert';
  impact: 'critical' | 'high' | 'medium' | 'low';
  prerequisites: string[];
  indicators: string[];
}

interface Mitigation {
  id: string;
  name: string;
  type: 'preventive' | 'detective' | 'corrective';
  effectiveness: number; // 0-1
  implementation: string;
}

// 위협 레지스트리
class ThreatRegistry {
  private threats: Map<string, ThreatCategory> = new Map();

  constructor() {
    this.initializeThreats();
  }

  private initializeThreats(): void {
    // 자격 증명 위조 위협
    this.registerThreat({
      id: 'THREAT-001',
      name: '자격 증명 주입 공격',
      description: '공격자가 콘텐츠에 허위 출처 자격 증명을 주입',
      attackVectors: [
        {
          id: 'AV-001-1',
          name: '매니페스트 주입',
          technique: '서명되지 않은 콘텐츠에 위조된 C2PA 매니페스트 주입',
          difficulty: 'medium',
          impact: 'critical',
          prerequisites: ['콘텐츠 파일 접근', 'C2PA 형식 지식'],
          indicators: ['유효하지 않은 서명 체인', '신뢰 앵커 누락']
        },
        {
          id: 'AV-001-2',
          name: '서명 재생',
          technique: '한 콘텐츠의 유효 서명을 다른 콘텐츠에 복사',
          difficulty: 'low',
          impact: 'high',
          prerequisites: ['서명된 콘텐츠 접근', '유사한 콘텐츠 유형'],
          indicators: ['해시 불일치', '콘텐츠-서명 불일치']
        }
      ],
      riskLevel: 'critical',
      mitigations: [
        {
          id: 'MIT-001-1',
          name: '암호화 바인딩',
          type: 'preventive',
          effectiveness: 0.95,
          implementation: 'JUMBF 구조를 사용하여 서명을 콘텐츠 해시에 바인딩'
        },
        {
          id: 'MIT-001-2',
          name: '신뢰 앵커 검증',
          type: 'detective',
          effectiveness: 0.90,
          implementation: '알려진 신뢰 앵커에 대한 인증서 체인 검증'
        }
      ]
    });

    // 탐지 우회 위협
    this.registerThreat({
      id: 'THREAT-002',
      name: '적대적 탐지 우회',
      description: '탐지를 회피하도록 수정된 AI 생성 콘텐츠',
      attackVectors: [
        {
          id: 'AV-002-1',
          name: '적대적 섭동',
          technique: '탐지 모델을 속이기 위한 인지할 수 없는 노이즈 추가',
          difficulty: 'high',
          impact: 'high',
          prerequisites: ['탐지 모델 접근', '그래디언트 계산'],
          indicators: ['비정상적 픽셀 패턴', '통계적 이상']
        },
        {
          id: 'AV-002-2',
          name: '압축 공격',
          technique: 'AI 아티팩트 제거를 위한 과도한 압축/재압축',
          difficulty: 'trivial',
          impact: 'medium',
          prerequisites: ['기본 이미지 편집'],
          indicators: ['품질 저하', '압축 아티팩트']
        }
      ],
      riskLevel: 'high',
      mitigations: [
        {
          id: 'MIT-002-1',
          name: '앙상블 탐지',
          type: 'detective',
          effectiveness: 0.85,
          implementation: '다양한 탐지 모델의 앙상블 사용'
        },
        {
          id: 'MIT-002-2',
          name: '강건한 워터마킹',
          type: 'preventive',
          effectiveness: 0.80,
          implementation: '생성 시 변환에 강건한 보이지 않는 워터마크 임베딩'
        }
      ]
    });
  }

  registerThreat(threat: ThreatCategory): void {
    this.threats.set(threat.id, threat);
  }

  assessRisk(contentType: string, context: SecurityContext): RiskAssessment {
    const applicableThreats = this.getApplicableThreats(contentType, context);

    return {
      overallRisk: this.calculateOverallRisk(applicableThreats),
      threats: applicableThreats.map(t => ({
        threat: t,
        likelihood: this.assessLikelihood(t, context),
        impact: this.assessImpact(t, context),
        residualRisk: this.calculateResidualRisk(t, context)
      })),
      recommendations: this.generateRecommendations(applicableThreats, context)
    };
  }

  private getApplicableThreats(
    contentType: string,
    context: SecurityContext
  ): ThreatCategory[] {
    return Array.from(this.threats.values()).filter(threat =>
      this.isThreatApplicable(threat, contentType, context)
    );
  }

  private isThreatApplicable(
    threat: ThreatCategory,
    contentType: string,
    context: SecurityContext
  ): boolean {
    return true; // 단순화
  }

  private calculateOverallRisk(threats: ThreatCategory[]): number {
    if (threats.length === 0) return 0;

    const riskScores = threats.map(t => {
      switch (t.riskLevel) {
        case 'critical': return 1.0;
        case 'high': return 0.75;
        case 'medium': return 0.5;
        case 'low': return 0.25;
      }
    });

    const maxRisk = Math.max(...riskScores);
    const aggregateFactor = 1 + (threats.length - 1) * 0.1;

    return Math.min(1.0, maxRisk * aggregateFactor);
  }

  private assessLikelihood(
    threat: ThreatCategory,
    context: SecurityContext
  ): number {
    const baseLikelihood = threat.attackVectors.reduce((max, av) => {
      const difficultyScore: Record<string, number> = {
        'trivial': 0.9, 'low': 0.7, 'medium': 0.5, 'high': 0.3, 'expert': 0.1
      };
      return Math.max(max, difficultyScore[av.difficulty]);
    }, 0);

    return baseLikelihood * context.threatEnvironment;
  }

  private assessImpact(
    threat: ThreatCategory,
    context: SecurityContext
  ): number {
    const impactScores: Record<string, number> = {
      'critical': 1.0, 'high': 0.75, 'medium': 0.5, 'low': 0.25
    };
    return impactScores[threat.riskLevel] * context.assetValue;
  }

  private calculateResidualRisk(
    threat: ThreatCategory,
    context: SecurityContext
  ): number {
    const inherentRisk = this.assessLikelihood(threat, context) *
                         this.assessImpact(threat, context);

    const controlEffectiveness = threat.mitigations.reduce((total, m) =>
      total + m.effectiveness * (context.implementedControls.has(m.id) ? 1 : 0),
      0
    ) / threat.mitigations.length;

    return inherentRisk * (1 - controlEffectiveness);
  }

  private generateRecommendations(
    threats: ThreatCategory[],
    context: SecurityContext
  ): SecurityRecommendation[] {
    const recommendations: SecurityRecommendation[] = [];

    for (const threat of threats) {
      for (const mitigation of threat.mitigations) {
        if (!context.implementedControls.has(mitigation.id)) {
          recommendations.push({
            priority: threat.riskLevel,
            control: mitigation,
            threatAddressed: threat.id,
            estimatedEffort: mitigation.type === 'preventive' ? 'high' : 'medium',
            businessJustification: `${threat.name} 위험을 ${
              Math.round(mitigation.effectiveness * 100)
            }% 감소`
          });
        }
      }
    }

    return recommendations.sort((a, b) =>
      this.priorityScore(b.priority) - this.priorityScore(a.priority)
    );
  }

  private priorityScore(priority: string): number {
    const scores: Record<string, number> = {
      'critical': 4, 'high': 3, 'medium': 2, 'low': 1
    };
    return scores[priority] || 0;
  }
}

interface SecurityContext {
  threatEnvironment: number;
  assetValue: number;
  implementedControls: Set<string>;
}

interface RiskAssessment {
  overallRisk: number;
  threats: ThreatAssessment[];
  recommendations: SecurityRecommendation[];
}

interface ThreatAssessment {
  threat: ThreatCategory;
  likelihood: number;
  impact: number;
  residualRisk: number;
}

interface SecurityRecommendation {
  priority: string;
  control: Mitigation;
  threatAddressed: string;
  estimatedEffort: string;
  businessJustification: string;
}
```

---

## 7.2 암호화 보안

### 디지털 서명 인프라

```typescript
import { createSign, createVerify, generateKeyPairSync } from 'crypto';

// 다중 알고리즘 서명 지원
class ContentSignatureManager {
  private supportedAlgorithms = ['Ed25519', 'ECDSA-P256', 'RSA-PSS'];

  generateKeyPair(algorithm: string): KeyPair {
    switch (algorithm) {
      case 'Ed25519':
        return generateKeyPairSync('ed25519', {
          publicKeyEncoding: { type: 'spki', format: 'pem' },
          privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
        });

      case 'ECDSA-P256':
        return generateKeyPairSync('ec', {
          namedCurve: 'prime256v1',
          publicKeyEncoding: { type: 'spki', format: 'pem' },
          privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
        });

      case 'RSA-PSS':
        return generateKeyPairSync('rsa-pss', {
          modulusLength: 4096,
          hashAlgorithm: 'sha256',
          saltLength: 32,
          publicKeyEncoding: { type: 'spki', format: 'pem' },
          privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
        });

      default:
        throw new Error(`지원하지 않는 알고리즘: ${algorithm}`);
    }
  }

  async signContent(
    content: Buffer,
    privateKey: string,
    algorithm: string
  ): Promise<ContentSignature> {
    const timestamp = Date.now();
    const contentHash = await this.hashContent(content);

    // 서명 페이로드 생성
    const payload = this.createSignaturePayload(contentHash, timestamp);

    // 페이로드 서명
    const signature = this.createSignature(payload, privateKey, algorithm);

    return {
      algorithm,
      signature: signature.toString('base64'),
      contentHash,
      timestamp,
      payload
    };
  }

  private async hashContent(content: Buffer): Promise<string> {
    const { subtle } = await import('crypto');
    const hashBuffer = await subtle.digest('SHA-256', content);
    return Buffer.from(hashBuffer).toString('hex');
  }

  private createSignaturePayload(
    contentHash: string,
    timestamp: number
  ): SignaturePayload {
    return {
      version: '1.0',
      contentHash,
      timestamp,
      algorithm: 'SHA-256',
      format: 'C2PA/1.0'
    };
  }

  private createSignature(
    payload: SignaturePayload,
    privateKey: string,
    algorithm: string
  ): Buffer {
    const payloadString = JSON.stringify(payload);

    switch (algorithm) {
      case 'Ed25519':
        const ed25519Sign = createSign('sha512');
        ed25519Sign.update(payloadString);
        return ed25519Sign.sign(privateKey);

      case 'ECDSA-P256':
        const ecdsaSign = createSign('sha256');
        ecdsaSign.update(payloadString);
        return ecdsaSign.sign(privateKey);

      case 'RSA-PSS':
        const rsaSign = createSign('sha256');
        rsaSign.update(payloadString);
        return rsaSign.sign({
          key: privateKey,
          padding: 6,
          saltLength: 32
        });

      default:
        throw new Error(`지원하지 않는 알고리즘: ${algorithm}`);
    }
  }

  verifySignature(
    signature: ContentSignature,
    publicKey: string
  ): SignatureVerificationResult {
    try {
      const payloadString = JSON.stringify(signature.payload);
      const signatureBuffer = Buffer.from(signature.signature, 'base64');

      const verify = createVerify(
        signature.algorithm === 'Ed25519' ? 'sha512' : 'sha256'
      );
      verify.update(payloadString);

      const isValid = verify.verify(publicKey, signatureBuffer);

      return {
        valid: isValid,
        algorithm: signature.algorithm,
        timestamp: new Date(signature.timestamp),
        contentHash: signature.contentHash,
        errors: isValid ? [] : ['서명 검증 실패']
      };
    } catch (error) {
      return {
        valid: false,
        algorithm: signature.algorithm,
        timestamp: new Date(signature.timestamp),
        contentHash: signature.contentHash,
        errors: [`검증 오류: ${error.message}`]
      };
    }
  }
}

interface KeyPair {
  publicKey: string;
  privateKey: string;
}

interface ContentSignature {
  algorithm: string;
  signature: string;
  contentHash: string;
  timestamp: number;
  payload: SignaturePayload;
}

interface SignaturePayload {
  version: string;
  contentHash: string;
  timestamp: number;
  algorithm: string;
  format: string;
}

interface SignatureVerificationResult {
  valid: boolean;
  algorithm: string;
  timestamp: Date;
  contentHash: string;
  errors: string[];
}
```

---

## 7.3 강건한 워터마킹

### 신경망 워터마킹 시스템

```python
import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Tuple, Dict
import numpy as np

class NeuralWatermarkEncoder(nn.Module):
    """
    신경망 기반 보이지 않는 워터마크 인코더.
    다양한 변환에서 살아남는 강건한 워터마크 임베딩.
    """

    def __init__(
        self,
        message_length: int = 128,
        image_channels: int = 3,
        hidden_dim: int = 64
    ):
        super().__init__()

        self.message_length = message_length

        # 메시지 준비 네트워크
        self.message_encoder = nn.Sequential(
            nn.Linear(message_length, hidden_dim * 4),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim * 4, hidden_dim * 16 * 16),
            nn.ReLU(inplace=True)
        )

        # 스킵 연결이 있는 이미지 인코더
        self.image_encoder = nn.Sequential(
            nn.Conv2d(image_channels, hidden_dim, 3, padding=1),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, hidden_dim * 2, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 2, hidden_dim * 4, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True)
        )

        # 융합 네트워크
        self.fusion = nn.Sequential(
            nn.Conv2d(hidden_dim * 4 + hidden_dim, hidden_dim * 4, 3, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 4, hidden_dim * 4, 3, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True)
        )

        # 워터마킹된 이미지 생성 디코더
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(hidden_dim * 4, hidden_dim * 2, 4, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(hidden_dim * 2, hidden_dim, 4, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, image_channels, 3, padding=1),
            nn.Tanh()
        )

        # 강도 제어
        self.strength = nn.Parameter(torch.tensor(0.1))

    def forward(
        self,
        image: torch.Tensor,
        message: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        이미지에 워터마크 임베딩.

        Args:
            image: 입력 이미지 [B, C, H, W] 범위 [-1, 1]
            message: 이진 메시지 [B, message_length]

        Returns:
            워터마킹된 이미지와 잔차
        """
        batch_size = image.size(0)

        # 메시지 인코딩
        message_features = self.message_encoder(message.float())
        message_features = message_features.view(batch_size, -1, 16, 16)

        # 이미지 인코딩
        image_features = self.image_encoder(image)

        # 메시지 특징을 이미지 특징에 맞게 업샘플링
        message_features = F.interpolate(
            message_features,
            size=image_features.shape[2:],
            mode='bilinear',
            align_corners=False
        )

        # 이미지와 메시지 융합
        fused = torch.cat([image_features, message_features], dim=1)
        fused = self.fusion(fused)

        # 잔차로 디코딩
        residual = self.decoder(fused)

        # 강도 제어된 워터마크 적용
        watermarked = image + self.strength * residual
        watermarked = torch.clamp(watermarked, -1, 1)

        return watermarked, residual


class RobustWatermarkingSystem:
    """
    학습 및 추론을 위한 완전한 워터마킹 시스템.
    """

    def __init__(
        self,
        message_length: int = 128,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        self.device = device
        self.message_length = message_length

        self.encoder = NeuralWatermarkEncoder(message_length).to(device)
        self.decoder = NeuralWatermarkDecoder(message_length).to(device)

    def embed_watermark(
        self,
        image: np.ndarray,
        message: bytes
    ) -> np.ndarray:
        """
        이미지에 워터마크 임베딩.

        Args:
            image: RGB 이미지 [H, W, 3] 범위 [0, 255]
            message: 임베딩할 이진 메시지

        Returns:
            워터마킹된 이미지 [H, W, 3] 범위 [0, 255]
        """
        self.encoder.eval()

        # 이미지 준비
        image_tensor = self._preprocess_image(image)

        # 메시지 준비
        message_bits = self._bytes_to_bits(message)
        message_tensor = torch.tensor(message_bits, dtype=torch.float32)
        message_tensor = message_tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            watermarked, _ = self.encoder(image_tensor, message_tensor)

        return self._postprocess_image(watermarked)

    def extract_watermark(
        self,
        image: np.ndarray
    ) -> Tuple[bytes, float]:
        """
        이미지에서 워터마크 추출.

        Returns:
            추출된 메시지와 신뢰도 점수
        """
        self.decoder.eval()

        image_tensor = self._preprocess_image(image)

        with torch.no_grad():
            logits = self.decoder(image_tensor)
            probs = torch.sigmoid(logits)
            bits = (probs > 0.5).float()

            # 신뢰도 계산
            confidence = torch.mean(torch.abs(probs - 0.5) * 2).item()

        message = self._bits_to_bytes(bits.squeeze().cpu().numpy())
        return message, confidence

    def verify_watermark(
        self,
        image: np.ndarray,
        expected_message: bytes,
        threshold: float = 0.9
    ) -> WatermarkVerificationResult:
        """
        이미지에 예상 워터마크가 포함되어 있는지 확인.
        """
        extracted, confidence = self.extract_watermark(image)

        # 비트 정확도 계산
        expected_bits = self._bytes_to_bits(expected_message)
        extracted_bits = self._bytes_to_bits(extracted)

        bit_accuracy = np.mean(
            np.array(expected_bits) == np.array(extracted_bits)
        )

        return WatermarkVerificationResult(
            verified=bit_accuracy >= threshold,
            bit_accuracy=float(bit_accuracy),
            confidence=confidence,
            extracted_message=extracted
        )

    def _preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        """numpy 이미지를 텐서로 변환."""
        tensor = torch.tensor(image, dtype=torch.float32) / 127.5 - 1.0
        tensor = tensor.permute(2, 0, 1).unsqueeze(0)
        return tensor.to(self.device)

    def _postprocess_image(self, tensor: torch.Tensor) -> np.ndarray:
        """텐서를 numpy 이미지로 변환."""
        tensor = tensor.squeeze(0).permute(1, 2, 0)
        image = ((tensor.cpu().numpy() + 1.0) * 127.5).astype(np.uint8)
        return image

    def _bytes_to_bits(self, data: bytes) -> list:
        """바이트를 비트 리스트로 변환."""
        bits = []
        for byte in data:
            for i in range(8):
                bits.append((byte >> (7 - i)) & 1)
        if len(bits) < self.message_length:
            bits.extend([0] * (self.message_length - len(bits)))
        return bits[:self.message_length]

    def _bits_to_bytes(self, bits: np.ndarray) -> bytes:
        """비트 배열을 바이트로 변환."""
        bits = bits.astype(int)
        byte_list = []
        for i in range(0, len(bits), 8):
            byte_val = 0
            for j in range(min(8, len(bits) - i)):
                byte_val = (byte_val << 1) | bits[i + j]
            byte_list.append(byte_val)
        return bytes(byte_list)


class NeuralWatermarkDecoder(nn.Module):
    """신경망 기반 워터마크 디코더."""

    def __init__(self, message_length: int = 128, hidden_dim: int = 64):
        super().__init__()

        self.feature_extractor = nn.Sequential(
            nn.Conv2d(3, hidden_dim, 3, padding=1),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, hidden_dim * 2, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 2, hidden_dim * 4, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1)
        )

        self.message_decoder = nn.Sequential(
            nn.Linear(hidden_dim * 4, hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim * 2, message_length)
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        features = self.feature_extractor(image)
        features = features.view(features.size(0), -1)
        return self.message_decoder(features)


class WatermarkVerificationResult:
    def __init__(
        self,
        verified: bool,
        bit_accuracy: float,
        confidence: float,
        extracted_message: bytes
    ):
        self.verified = verified
        self.bit_accuracy = bit_accuracy
        self.confidence = confidence
        self.extracted_message = extracted_message

    def to_dict(self) -> dict:
        return {
            'verified': self.verified,
            'bit_accuracy': self.bit_accuracy,
            'confidence': self.confidence,
            'extracted_message': self.extracted_message.hex()
        }
```

---

## 7.4 프라이버시 보호

### 익명화 프레임워크

```typescript
// 프라이버시 보호 콘텐츠 자격 증명 시스템
interface PrivacyConfig {
  anonymizationLevel: 'none' | 'partial' | 'full';
  redactableFields: string[];
  selectiveDisclosure: boolean;
  zkProofSupport: boolean;
}

class PrivacyPreservingCredentials {
  private config: PrivacyConfig;

  constructor(config: PrivacyConfig) {
    this.config = config;
  }

  async createAnonymousCredential(
    content: Buffer,
    creator: CreatorIdentity,
    options: CredentialOptions
  ): Promise<AnonymousCredential> {
    // 가명 식별자 생성
    const pseudonym = await this.generatePseudonym(creator, options.context);

    // 신원 커밋먼트 생성 (필요시 나중에 공개 가능)
    const identityCommitment = await this.createIdentityCommitment(
      creator,
      options.revealConditions
    );

    // 선택적 공개 지원 자격 증명 구축
    const credential = await this.buildCredential({
      pseudonym,
      identityCommitment,
      contentHash: await this.hashContent(content),
      timestamp: Date.now(),
      assertions: options.assertions,
      redactedFields: this.config.redactableFields
    });

    // 영지식 증명 생성 (활성화된 경우)
    let zkProofs: ZKProof[] = [];
    if (this.config.zkProofSupport) {
      zkProofs = await this.generateZKProofs(credential, options.proofRequests);
    }

    return {
      credential,
      zkProofs,
      pseudonym,
      revealToken: identityCommitment.revealToken
    };
  }

  private async generatePseudonym(
    creator: CreatorIdentity,
    context: string
  ): Promise<string> {
    // 컨텍스트별 가명으로 교차 플랫폼 추적 방지
    const contextKey = await this.deriveContextKey(creator.privateKey, context);
    const pseudonym = await this.hash(
      `${creator.id}:${context}:${contextKey}`
    );
    return pseudonym;
  }

  private async createIdentityCommitment(
    creator: CreatorIdentity,
    revealConditions: RevealCondition[]
  ): Promise<IdentityCommitment> {
    // 신원에 대한 Pedersen 커밋먼트
    const randomness = crypto.getRandomValues(new Uint8Array(32));
    const commitment = await this.pedersenCommit(
      Buffer.from(creator.id),
      randomness
    );

    // 권한 있는 당사자를 위한 공개 토큰 암호화
    const revealToken = await this.encryptRevealToken(
      { identity: creator.id, randomness: Buffer.from(randomness) },
      revealConditions
    );

    return {
      commitment,
      revealToken,
      conditions: revealConditions
    };
  }

  private async generateZKProofs(
    credential: Credential,
    proofRequests: ProofRequest[]
  ): Promise<ZKProof[]> {
    const proofs: ZKProof[] = [];

    for (const request of proofRequests) {
      switch (request.type) {
        case 'ai_generated':
          proofs.push(await this.proveAIGeneration(credential, request));
          break;
        case 'creator_authorized':
          proofs.push(await this.proveCreatorAuthorization(credential, request));
          break;
        case 'timestamp_range':
          proofs.push(await this.proveTimestampRange(credential, request));
          break;
        case 'content_unmodified':
          proofs.push(await this.proveContentUnmodified(credential, request));
          break;
      }
    }

    return proofs;
  }

  private async proveAIGeneration(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // 생성기를 공개하지 않고 AI 생성 콘텐츠임을 증명
    return {
      type: 'ai_generated',
      proof: 'zk_proof_data',
      publicInputs: {
        isAIGenerated: true,
        confidenceThreshold: request.params.minConfidence
      },
      verificationKey: 'vk_ai_generation'
    };
  }

  private async proveCreatorAuthorization(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // 신원을 공개하지 않고 창작자 권한 증명
    return {
      type: 'creator_authorized',
      proof: 'zk_proof_data',
      publicInputs: {
        isAuthorized: true,
        authorizationLevel: 'verified'
      },
      verificationKey: 'vk_creator_auth'
    };
  }

  private async proveTimestampRange(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // 정확한 시간을 공개하지 않고 타임스탬프가 범위 내임을 증명
    return {
      type: 'timestamp_range',
      proof: 'zk_proof_data',
      publicInputs: {
        afterDate: request.params.minTimestamp,
        beforeDate: request.params.maxTimestamp
      },
      verificationKey: 'vk_timestamp'
    };
  }

  private async proveContentUnmodified(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // 서명 이후 콘텐츠가 수정되지 않았음을 증명
    return {
      type: 'content_unmodified',
      proof: 'zk_proof_data',
      publicInputs: {
        contentCommitment: credential.contentHash
      },
      verificationKey: 'vk_content_integrity'
    };
  }

  async verifyAnonymousCredential(
    credential: AnonymousCredential,
    content: Buffer
  ): Promise<CredentialVerificationResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    // 콘텐츠 해시 검증
    const contentHash = await this.hashContent(content);
    if (contentHash !== credential.credential.contentHash) {
      errors.push('콘텐츠 해시 불일치');
    }

    // ZK 증명 검증
    for (const proof of credential.zkProofs) {
      const valid = await this.verifyZKProof(proof);
      if (!valid) {
        errors.push(`유효하지 않은 ZK 증명: ${proof.type}`);
      }
    }

    // 가명 형식 검증
    if (!this.isValidPseudonym(credential.pseudonym)) {
      warnings.push('가명 형식이 비표준입니다');
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      proofResults: credential.zkProofs.map(p => ({
        type: p.type,
        verified: true
      })),
      anonymityLevel: this.assessAnonymityLevel(credential)
    };
  }

  private async verifyZKProof(proof: ZKProof): Promise<boolean> {
    return true; // 단순화
  }

  private isValidPseudonym(pseudonym: string): boolean {
    return /^[a-f0-9]{64}$/.test(pseudonym);
  }

  private assessAnonymityLevel(credential: AnonymousCredential): string {
    if (credential.zkProofs.length > 0) {
      return 'high';
    }
    if (credential.credential.redactedFields?.length > 0) {
      return 'medium';
    }
    return 'low';
  }

  private async hashContent(content: Buffer): Promise<string> {
    const hashBuffer = await crypto.subtle.digest('SHA-256', content);
    return Buffer.from(hashBuffer).toString('hex');
  }

  private async hash(data: string): Promise<string> {
    const encoder = new TextEncoder();
    const hashBuffer = await crypto.subtle.digest('SHA-256', encoder.encode(data));
    return Buffer.from(hashBuffer).toString('hex');
  }

  private async deriveContextKey(privateKey: string, context: string): Promise<string> {
    return await this.hash(`${privateKey}:${context}`);
  }

  private async pedersenCommit(value: Buffer, randomness: Uint8Array): Promise<string> {
    const combined = Buffer.concat([value, Buffer.from(randomness)]);
    return await this.hash(combined.toString('hex'));
  }

  private async encryptRevealToken(
    token: { identity: string; randomness: Buffer },
    conditions: RevealCondition[]
  ): Promise<EncryptedRevealToken> {
    const encryptedShares: EncryptedShare[] = [];

    for (const condition of conditions) {
      const share = await this.encryptForParty(
        JSON.stringify(token),
        condition.authorizedParty
      );
      encryptedShares.push({
        party: condition.authorizedParty,
        condition: condition.type,
        encryptedData: share
      });
    }

    return { shares: encryptedShares };
  }

  private async encryptForParty(data: string, partyPublicKey: string): Promise<string> {
    return Buffer.from(data).toString('base64'); // 단순화
  }

  private async buildCredential(params: any): Promise<Credential> {
    return {
      version: '1.0',
      ...params,
      signature: 'signature_placeholder'
    };
  }
}

interface CreatorIdentity {
  id: string;
  privateKey: string;
  publicKey: string;
}

interface CredentialOptions {
  context: string;
  assertions: Assertion[];
  revealConditions: RevealCondition[];
  proofRequests: ProofRequest[];
}

interface RevealCondition {
  type: 'legal_request' | 'platform_policy' | 'creator_consent';
  authorizedParty: string;
  expiresAt?: number;
}

interface ProofRequest {
  type: string;
  params: Record<string, any>;
}

interface AnonymousCredential {
  credential: Credential;
  zkProofs: ZKProof[];
  pseudonym: string;
  revealToken: EncryptedRevealToken;
}

interface ZKProof {
  type: string;
  proof: string;
  publicInputs: Record<string, any>;
  verificationKey: string;
}

interface EncryptedRevealToken {
  shares: EncryptedShare[];
}

interface EncryptedShare {
  party: string;
  condition: string;
  encryptedData: string;
}

interface Credential {
  version: string;
  contentHash: string;
  timestamp: number;
  signature: string;
  redactedFields?: string[];
}

interface CredentialVerificationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
  proofResults: { type: string; verified: boolean }[];
  anonymityLevel: string;
}

interface Assertion {
  type: string;
  value: any;
}

interface IdentityCommitment {
  commitment: string;
  revealToken: EncryptedRevealToken;
  conditions: RevealCondition[];
}
```

---

## 7.5 보안 모범 사례

### 구현 체크리스트

```yaml
# 보안 구현 체크리스트
security_checklist:
  암호화:
    - name: "승인된 알고리즘 사용"
      requirement: "서명에 Ed25519, ECDSA-P256 또는 RSA-4096"
      verification: "서명 구성 검토"
      priority: critical

    - name: "안전한 키 저장"
      requirement: "개인 키에 HSM 또는 보안 영역 사용"
      verification: "키 관리 인프라 감사"
      priority: critical

    - name: "인증서 피닝"
      requirement: "신뢰 앵커 인증서 피닝"
      verification: "인증서 검증 테스트"
      priority: high

  인증:
    - name: "다중 인증"
      requirement: "서명 키 접근에 MFA"
      verification: "인증 흐름 검토"
      priority: critical

    - name: "세션 관리"
      requirement: "로테이션이 있는 단기 토큰"
      verification: "토큰 수명 주기 감사"
      priority: high

  탐지_보안:
    - name: "적대적 강건성"
      requirement: "적대적 예제에 대해 훈련됨"
      verification: "적대적 평가 수행"
      priority: high

    - name: "앙상블 탐지"
      requirement: "다양한 탐지 모델 사용"
      verification: "앙상블 불일치 테스트"
      priority: medium

  프라이버시:
    - name: "데이터 최소화"
      requirement: "필요한 데이터만 수집"
      verification: "데이터 수집 검토"
      priority: high

    - name: "익명화"
      requirement: "가명 자격 증명 지원"
      verification: "익명화 기능 테스트"
      priority: medium

  운영:
    - name: "보안 모니터링"
      requirement: "실시간 위협 탐지"
      verification: "알림 시스템 테스트"
      priority: high

    - name: "사고 대응"
      requirement: "문서화된 절차"
      verification: "탁상 훈련 실행"
      priority: high
```

---

## 요약

WIA 콘텐츠 AI 보안 프레임워크는 다음을 제공합니다:

1. **포괄적 위협 모델** - 공격 벡터 및 완화의 체계적 분석
2. **암호화 보안** - 다중 알고리즘 서명 및 인증서 체인 검증
3. **강건한 워터마킹** - 변환에서 살아남는 신경망 워터마크
4. **프라이버시 보호** - 익명화 및 영지식 증명
5. **보안 모니터링** - 실시간 위협 탐지 및 대응

---

*© 2025 세계산업협회 (WIA). 모든 권리 보유.*
*弘益人間 (홍익인간) · 널리 인간을 이롭게 하라*
