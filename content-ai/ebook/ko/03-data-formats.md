# 제3장: 데이터 형식

## 콘텐츠 자격 증명 스키마

### 개요

WIA 콘텐츠 AI 표준은 AI 생성 콘텐츠의 인증, 출처 추적 및 검증을 위한 포괄적인 데이터 형식을 정의합니다. 본 장에서는 C2PA(Coalition for Content Provenance and Authenticity) 표준과의 호환성을 유지하면서 AI 콘텐츠 특화 확장을 제공하는 스키마 사양을 상세히 설명합니다.

---

## 3.1 C2PA 매니페스트 구조

### 핵심 매니페스트 스키마

```typescript
// C2PA 매니페스트 기본 구조
interface C2PAManifest {
  // 매니페스트 메타데이터
  claim_generator: string;           // 생성 도구 식별자
  claim_generator_info?: GeneratorInfo[];
  title?: string;                    // 콘텐츠 제목
  format: string;                    // MIME 타입
  instance_id: string;               // UUID

  // 콘텐츠 바인딩
  dc:format?: string;                // Dublin Core 형식
  dc:title?: string;                 // Dublin Core 제목

  // 어설션 목록
  assertions: C2PAAssertion[];

  // 클레임 서명
  claim_signature: ClaimSignature;

  // 선택적 성분
  ingredients?: Ingredient[];         // 원재료 (파생작 추적)
  redactions?: string[];              // 수정된 어설션
}

// 생성기 정보
interface GeneratorInfo {
  name: string;
  version: string;
  icon?: AssetReference;
}

// 어설션 기본 구조
interface C2PAAssertion {
  label: string;                      // 어설션 유형 식별자
  data: Record<string, unknown>;      // 어설션 데이터
  kind?: 'Json' | 'Cbor' | 'Binary'; // 인코딩 방식
  hash?: string;                      // 데이터 해시
}

// 클레임 서명
interface ClaimSignature {
  algorithm: SignatureAlgorithm;
  certificate_chain: string[];
  signature: string;
  timestamp?: TimestampToken;
  pad?: number;
}

type SignatureAlgorithm =
  | 'Ed25519'
  | 'ES256'
  | 'ES384'
  | 'ES512'
  | 'PS256'
  | 'PS384'
  | 'PS512';

// 타임스탬프 토큰
interface TimestampToken {
  authority: string;
  time: string;
  token: string;
}

// 원재료 (파생작 추적)
interface Ingredient {
  title: string;
  format: string;
  document_id?: string;
  instance_id: string;
  relationship: 'parentOf' | 'componentOf' | 'inputTo';
  thumbnail?: AssetReference;
  c2pa_manifest?: ManifestReference;
  validation_status?: ValidationStatus;
}

interface AssetReference {
  format: string;
  identifier: string;
}

interface ManifestReference {
  url?: string;
  hash: string;
}

interface ValidationStatus {
  code: string;
  message: string;
}
```

### AI 생성 어설션 확장

```typescript
// AI 생성 콘텐츠 어설션
interface AIGenerationAssertion extends C2PAAssertion {
  label: 'c2pa.ai_training' | 'wia.ai_generation';
  data: {
    // AI 생성 여부
    ai_generated: boolean;

    // 생성기 정보
    generator_info: {
      name: string;                    // 예: "DALL-E", "Midjourney"
      version: string;
      model?: string;                  // 모델 식별자
      provider?: string;               // 서비스 제공자
    };

    // 생성 파라미터 (선택적)
    generation_parameters?: {
      prompt_hash?: string;            // 프롬프트 해시 (프라이버시 보호)
      seed?: number;
      guidance_scale?: number;
      steps?: number;
      sampler?: string;
      negative_prompt_hash?: string;
      [key: string]: unknown;
    };

    // 생성 시간
    timestamp: string;                 // ISO 8601

    // AI 기여도 (부분 AI 생성 시)
    ai_contribution?: {
      percentage: number;              // 0-100
      areas?: ContentArea[];           // 수정 영역
    };

    // 학습 데이터 정보 (선택적)
    training_info?: {
      dataset_declaration?: string;
      opted_out_content: boolean;
      compensation_model?: string;
    };
  };
}

// 콘텐츠 영역 정의
interface ContentArea {
  type: 'region' | 'layer' | 'segment';
  coordinates?: BoundingBox | Polygon;
  time_range?: TimeRange;
  description?: string;
}

interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

interface Polygon {
  points: Array<{ x: number; y: number }>;
}

interface TimeRange {
  start: number;
  end: number;
}
```

### 탐지 결과 어설션

```typescript
// AI 탐지 결과 어설션
interface DetectionResultAssertion extends C2PAAssertion {
  label: 'wia.detection_result';
  data: {
    // 탐지 판정
    verdict: DetectionVerdict;

    // 신뢰도 점수
    confidence: number;               // 0.0 - 1.0

    // 탐지 모델 정보
    detector: {
      name: string;
      version: string;
      provider: string;
      model_hash?: string;
    };

    // 탐지 시간
    detection_time: string;

    // 상세 분석 결과
    analysis: {
      // 모델별 점수
      model_scores: ModelScore[];

      // 주파수 분석
      frequency_analysis?: FrequencyAnalysis;

      // 의심 영역
      suspicious_regions?: SuspiciousRegion[];

      // 텍스트 분석 (해당 시)
      text_analysis?: TextAnalysis;
    };

    // 메타데이터
    metadata: {
      processing_time_ms: number;
      input_hash: string;
      analysis_version: string;
    };
  };
}

type DetectionVerdict =
  | 'ai_generated'
  | 'human_created'
  | 'uncertain'
  | 'manipulated'
  | 'hybrid';

interface ModelScore {
  model_name: string;
  score: number;
  label: string;
  weight: number;
}

interface FrequencyAnalysis {
  high_frequency_ratio: number;
  spectral_flatness: number;
  periodicity_score: number;
  dct_artifacts: boolean;
}

interface SuspiciousRegion {
  region: BoundingBox;
  type: string;
  confidence: number;
  description: string;
}

interface TextAnalysis {
  perplexity: number;
  burstiness: number;
  vocabulary_richness: number;
  stylometric_score: number;
}
```

---

## 3.2 콘텐츠 해시 스키마

### 멀티 해시 바인딩

```typescript
// 콘텐츠 해시 바인딩
interface ContentHashBinding {
  // 기본 해시
  primary: {
    algorithm: HashAlgorithm;
    value: string;
    encoding: 'hex' | 'base64' | 'base58';
  };

  // 대체 해시 (상호 운용성)
  alternatives?: Array<{
    algorithm: HashAlgorithm;
    value: string;
    encoding: 'hex' | 'base64' | 'base58';
  }>;

  // 지각 해시 (이미지/비디오용)
  perceptual?: PerceptualHash[];

  // 부분 해시 (대용량 파일용)
  partial?: PartialHash[];
}

type HashAlgorithm =
  | 'sha256'
  | 'sha384'
  | 'sha512'
  | 'sha3-256'
  | 'sha3-512'
  | 'blake2b'
  | 'blake3';

// 지각 해시 (유사 콘텐츠 매칭)
interface PerceptualHash {
  algorithm: PerceptualHashAlgorithm;
  value: string;
  sensitivity: number;             // 0.0 - 1.0
}

type PerceptualHashAlgorithm =
  | 'pHash'
  | 'dHash'
  | 'aHash'
  | 'wHash'
  | 'colorHash';

// 부분 해시 (청크 기반)
interface PartialHash {
  chunk_index: number;
  chunk_size: number;
  offset: number;
  hash: string;
  algorithm: HashAlgorithm;
}
```

### 해시 구현 예제

```typescript
import { createHash } from 'crypto';

// 멀티 알고리즘 해시 생성기
class ContentHashGenerator {
  async generateHashes(
    content: Buffer,
    options: HashOptions = {}
  ): Promise<ContentHashBinding> {
    const primary = await this.computeHash(content, options.primary || 'sha256');

    const alternatives = await Promise.all(
      (options.alternatives || ['sha3-256', 'blake3']).map(algo =>
        this.computeHash(content, algo)
      )
    );

    let perceptual: PerceptualHash[] | undefined;
    if (options.contentType?.startsWith('image/')) {
      perceptual = await this.computePerceptualHashes(content);
    }

    return {
      primary,
      alternatives,
      perceptual
    };
  }

  private async computeHash(
    content: Buffer,
    algorithm: HashAlgorithm
  ): Promise<{ algorithm: HashAlgorithm; value: string; encoding: 'hex' }> {
    let hash: string;

    switch (algorithm) {
      case 'sha256':
      case 'sha384':
      case 'sha512':
        hash = createHash(algorithm).update(content).digest('hex');
        break;

      case 'sha3-256':
      case 'sha3-512':
        hash = createHash(algorithm.replace('-', '')).update(content).digest('hex');
        break;

      case 'blake3':
        // blake3 라이브러리 사용
        const { blake3 } = await import('blake3');
        hash = blake3(content).toString('hex');
        break;

      default:
        throw new Error(`지원하지 않는 알고리즘: ${algorithm}`);
    }

    return { algorithm, value: hash, encoding: 'hex' };
  }

  private async computePerceptualHashes(
    imageBuffer: Buffer
  ): Promise<PerceptualHash[]> {
    // 이미지 처리 라이브러리 사용
    const sharp = await import('sharp');
    const image = sharp(imageBuffer);
    const metadata = await image.metadata();

    // 표준 크기로 리사이즈
    const normalized = await image
      .resize(64, 64, { fit: 'fill' })
      .grayscale()
      .raw()
      .toBuffer();

    return [
      {
        algorithm: 'pHash',
        value: this.computePHash(normalized),
        sensitivity: 0.85
      },
      {
        algorithm: 'dHash',
        value: this.computeDHash(normalized),
        sensitivity: 0.90
      }
    ];
  }

  private computePHash(grayscaleData: Buffer): string {
    // DCT 기반 pHash 구현
    const size = 8;
    const dct = this.computeDCT(grayscaleData, 64, size);

    // 중간값 계산 (DC 성분 제외)
    const values = dct.slice(1);
    const median = this.median(values);

    // 이진화
    let hash = '';
    for (const value of values) {
      hash += value > median ? '1' : '0';
    }

    return BigInt('0b' + hash).toString(16).padStart(16, '0');
  }

  private computeDHash(grayscaleData: Buffer): string {
    // 차이 기반 dHash 구현
    const width = 64;
    const height = 64;
    let hash = '';

    for (let y = 0; y < 8; y++) {
      for (let x = 0; x < 8; x++) {
        const currentPixel = grayscaleData[y * 8 * width / 64 * width + x * 8];
        const nextPixel = grayscaleData[y * 8 * width / 64 * width + (x + 1) * 8];
        hash += currentPixel < nextPixel ? '1' : '0';
      }
    }

    return BigInt('0b' + hash).toString(16).padStart(16, '0');
  }

  private computeDCT(data: Buffer, inputSize: number, outputSize: number): number[] {
    // 단순화된 DCT 구현
    const result: number[] = [];

    for (let u = 0; u < outputSize; u++) {
      for (let v = 0; v < outputSize; v++) {
        let sum = 0;
        for (let x = 0; x < inputSize; x++) {
          for (let y = 0; y < inputSize; y++) {
            const pixel = data[y * inputSize + x];
            sum += pixel *
              Math.cos((2 * x + 1) * u * Math.PI / (2 * inputSize)) *
              Math.cos((2 * y + 1) * v * Math.PI / (2 * inputSize));
          }
        }
        result.push(sum);
      }
    }

    return result;
  }

  private median(values: number[]): number {
    const sorted = [...values].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    return sorted.length % 2 !== 0
      ? sorted[mid]
      : (sorted[mid - 1] + sorted[mid]) / 2;
  }
}

interface HashOptions {
  primary?: HashAlgorithm;
  alternatives?: HashAlgorithm[];
  contentType?: string;
}
```

---

## 3.3 워터마크 스키마

### 워터마크 메타데이터 형식

```typescript
// 워터마크 정보 스키마
interface WatermarkSchema {
  // 워터마크 식별
  id: string;
  version: string;

  // 워터마크 유형
  type: WatermarkType;

  // 임베딩 정보
  embedding: {
    algorithm: WatermarkAlgorithm;
    strength: number;                  // 0.0 - 1.0
    capacity_bits: number;
    error_correction: ErrorCorrectionScheme;
  };

  // 페이로드
  payload: {
    format: 'binary' | 'text' | 'structured';
    encoding: 'base64' | 'hex' | 'utf8';
    data: string;
    checksum: string;
  };

  // 강건성 프로파일
  robustness: {
    compression: RobustnessLevel;
    scaling: RobustnessLevel;
    cropping: RobustnessLevel;
    rotation: RobustnessLevel;
    noise: RobustnessLevel;
    color_adjustment: RobustnessLevel;
  };

  // 검증 정보
  verification: {
    public_key?: string;
    verification_url?: string;
    certificate_chain?: string[];
  };

  // 메타데이터
  metadata: {
    created_at: string;
    creator: string;
    purpose: WatermarkPurpose;
    expiration?: string;
  };
}

type WatermarkType =
  | 'invisible_spatial'      // 공간 영역 워터마크
  | 'invisible_frequency'    // 주파수 영역 워터마크
  | 'invisible_neural'       // 신경망 기반 워터마크
  | 'visible_overlay'        // 가시적 오버레이
  | 'metadata_steganography' // 메타데이터 스테가노그래피
  | 'audio_spectral'         // 오디오 스펙트럼 워터마크
  | 'video_temporal';        // 비디오 시간 워터마크

type WatermarkAlgorithm =
  | 'DCT'                    // 이산 코사인 변환
  | 'DWT'                    // 이산 웨이블릿 변환
  | 'SVD'                    // 특이값 분해
  | 'QIM'                    // 양자화 인덱스 변조
  | 'SS'                     // 스프레드 스펙트럼
  | 'Neural'                 // 신경망 기반
  | 'SynthID';               // Google SynthID

type ErrorCorrectionScheme =
  | 'none'
  | 'hamming'
  | 'reed_solomon'
  | 'ldpc'
  | 'turbo';

type RobustnessLevel =
  | 'none'
  | 'low'
  | 'medium'
  | 'high'
  | 'very_high';

type WatermarkPurpose =
  | 'ownership'              // 소유권 증명
  | 'authenticity'           // 진위성 검증
  | 'tracking'               // 유통 추적
  | 'tamper_detection'       // 변조 탐지
  | 'ai_disclosure';         // AI 생성 공개
```

### 워터마크 임베딩 예제

```python
import numpy as np
from scipy.fftpack import dct, idct
from typing import Dict, Tuple, Optional
import hashlib

class DCTWatermarkEncoder:
    """DCT 기반 워터마크 인코더"""

    def __init__(
        self,
        block_size: int = 8,
        strength: float = 0.1,
        bands: Tuple[int, int] = (4, 7)
    ):
        self.block_size = block_size
        self.strength = strength
        self.bands = bands

    def embed(
        self,
        image: np.ndarray,
        payload: bytes,
        key: bytes
    ) -> Tuple[np.ndarray, Dict]:
        """
        이미지에 워터마크 임베딩

        Args:
            image: RGB 이미지 배열 [H, W, 3]
            payload: 임베딩할 데이터
            key: 암호화 키

        Returns:
            워터마킹된 이미지와 메타데이터
        """
        # 페이로드 준비
        payload_bits = self._prepare_payload(payload, key)

        # YCbCr 변환 (Y 채널에 워터마크)
        ycbcr = self._rgb_to_ycbcr(image)
        y_channel = ycbcr[:, :, 0].astype(np.float64)

        # 블록 단위 처리
        height, width = y_channel.shape
        block_h = height // self.block_size
        block_w = width // self.block_size

        bit_index = 0
        total_capacity = block_h * block_w

        for i in range(block_h):
            for j in range(block_w):
                if bit_index >= len(payload_bits):
                    break

                # 블록 추출
                block = y_channel[
                    i * self.block_size:(i + 1) * self.block_size,
                    j * self.block_size:(j + 1) * self.block_size
                ]

                # DCT 변환
                dct_block = dct(dct(block.T, norm='ortho').T, norm='ortho')

                # 중간 주파수 대역에 비트 임베딩
                bit = payload_bits[bit_index]
                dct_block = self._embed_bit(dct_block, bit)

                # 역 DCT
                y_channel[
                    i * self.block_size:(i + 1) * self.block_size,
                    j * self.block_size:(j + 1) * self.block_size
                ] = idct(idct(dct_block.T, norm='ortho').T, norm='ortho')

                bit_index += 1

        # 결과 이미지 생성
        ycbcr[:, :, 0] = np.clip(y_channel, 0, 255)
        watermarked = self._ycbcr_to_rgb(ycbcr)

        metadata = {
            'algorithm': 'DCT',
            'block_size': self.block_size,
            'strength': self.strength,
            'capacity_bits': total_capacity,
            'used_bits': min(bit_index, len(payload_bits)),
            'payload_hash': hashlib.sha256(payload).hexdigest()
        }

        return watermarked.astype(np.uint8), metadata

    def extract(
        self,
        image: np.ndarray,
        payload_length: int,
        key: bytes
    ) -> Tuple[bytes, float]:
        """
        이미지에서 워터마크 추출

        Returns:
            추출된 페이로드와 신뢰도
        """
        # YCbCr 변환
        ycbcr = self._rgb_to_ycbcr(image)
        y_channel = ycbcr[:, :, 0].astype(np.float64)

        # 비트 추출
        height, width = y_channel.shape
        block_h = height // self.block_size
        block_w = width // self.block_size

        extracted_bits = []
        confidences = []

        for i in range(block_h):
            for j in range(block_w):
                if len(extracted_bits) >= payload_length * 8:
                    break

                block = y_channel[
                    i * self.block_size:(i + 1) * self.block_size,
                    j * self.block_size:(j + 1) * self.block_size
                ]

                dct_block = dct(dct(block.T, norm='ortho').T, norm='ortho')
                bit, confidence = self._extract_bit(dct_block)

                extracted_bits.append(bit)
                confidences.append(confidence)

        # 바이트로 변환
        payload = self._bits_to_bytes(extracted_bits[:payload_length * 8])

        # 오류 정정 및 복호화
        payload = self._decode_payload(payload, key)

        avg_confidence = np.mean(confidences) if confidences else 0

        return payload, avg_confidence

    def _prepare_payload(self, payload: bytes, key: bytes) -> list:
        """페이로드 준비 (암호화 + 오류 정정)"""
        # 체크섬 추가
        checksum = hashlib.sha256(payload).digest()[:4]
        data = payload + checksum

        # XOR 암호화 (실제로는 더 강력한 암호화 사용)
        key_extended = (key * (len(data) // len(key) + 1))[:len(data)]
        encrypted = bytes(a ^ b for a, b in zip(data, key_extended))

        # 비트 변환
        bits = []
        for byte in encrypted:
            for i in range(8):
                bits.append((byte >> (7 - i)) & 1)

        return bits

    def _decode_payload(self, payload: bytes, key: bytes) -> bytes:
        """페이로드 복호화"""
        key_extended = (key * (len(payload) // len(key) + 1))[:len(payload)]
        decrypted = bytes(a ^ b for a, b in zip(payload, key_extended))

        # 체크섬 검증
        data = decrypted[:-4]
        checksum = decrypted[-4:]
        expected_checksum = hashlib.sha256(data).digest()[:4]

        if checksum != expected_checksum:
            raise ValueError("워터마크 체크섬 불일치")

        return data

    def _embed_bit(self, dct_block: np.ndarray, bit: int) -> np.ndarray:
        """DCT 블록에 비트 임베딩"""
        low, high = self.bands

        # 중간 주파수 계수 선택
        coeff_pos = [(low, high), (high, low)]

        # QIM (Quantization Index Modulation) 방식
        for pos in coeff_pos:
            coeff = dct_block[pos]
            quantized = np.round(coeff / self.strength) * self.strength

            if bit == 1:
                dct_block[pos] = quantized + self.strength / 2
            else:
                dct_block[pos] = quantized - self.strength / 2

        return dct_block

    def _extract_bit(self, dct_block: np.ndarray) -> Tuple[int, float]:
        """DCT 블록에서 비트 추출"""
        low, high = self.bands
        coeff_pos = [(low, high), (high, low)]

        votes = []
        for pos in coeff_pos:
            coeff = dct_block[pos]
            remainder = (coeff % self.strength) / self.strength

            if remainder > 0.25 and remainder < 0.75:
                votes.append(1 if remainder > 0.5 else 0)

        if not votes:
            return 0, 0.5

        bit = 1 if sum(votes) > len(votes) / 2 else 0
        confidence = abs(sum(votes) / len(votes) - 0.5) * 2

        return bit, confidence

    def _rgb_to_ycbcr(self, rgb: np.ndarray) -> np.ndarray:
        """RGB를 YCbCr로 변환"""
        matrix = np.array([
            [0.299, 0.587, 0.114],
            [-0.169, -0.331, 0.500],
            [0.500, -0.419, -0.081]
        ])
        offset = np.array([0, 128, 128])

        return np.dot(rgb, matrix.T) + offset

    def _ycbcr_to_rgb(self, ycbcr: np.ndarray) -> np.ndarray:
        """YCbCr를 RGB로 변환"""
        matrix = np.array([
            [1.0, 0.0, 1.403],
            [1.0, -0.344, -0.714],
            [1.0, 1.773, 0.0]
        ])
        offset = np.array([0, -128, -128])

        ycbcr_adjusted = ycbcr + offset
        return np.clip(np.dot(ycbcr_adjusted, matrix.T), 0, 255)

    def _bits_to_bytes(self, bits: list) -> bytes:
        """비트 리스트를 바이트로 변환"""
        bytes_list = []
        for i in range(0, len(bits), 8):
            byte = 0
            for j in range(min(8, len(bits) - i)):
                byte = (byte << 1) | bits[i + j]
            bytes_list.append(byte)
        return bytes(bytes_list)
```

---

## 3.4 JSON 스키마 정의

### 전체 콘텐츠 자격 증명 스키마

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/schemas/content-ai/v1/credential",
  "title": "WIA 콘텐츠 AI 자격 증명",
  "description": "AI 생성 콘텐츠 인증 및 출처 추적을 위한 자격 증명 스키마",
  "type": "object",
  "required": ["version", "content_binding", "assertions", "signature"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "스키마 버전"
    },
    "id": {
      "type": "string",
      "format": "uuid",
      "description": "자격 증명 고유 식별자"
    },
    "content_binding": {
      "$ref": "#/$defs/ContentBinding"
    },
    "assertions": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/Assertion"
      },
      "minItems": 1
    },
    "signature": {
      "$ref": "#/$defs/Signature"
    },
    "metadata": {
      "$ref": "#/$defs/Metadata"
    }
  },
  "$defs": {
    "ContentBinding": {
      "type": "object",
      "required": ["hash"],
      "properties": {
        "hash": {
          "type": "object",
          "required": ["algorithm", "value"],
          "properties": {
            "algorithm": {
              "type": "string",
              "enum": ["sha256", "sha384", "sha512", "sha3-256", "blake3"]
            },
            "value": {
              "type": "string",
              "pattern": "^[a-f0-9]+$"
            }
          }
        },
        "format": {
          "type": "string",
          "description": "MIME 타입"
        },
        "size": {
          "type": "integer",
          "minimum": 0
        }
      }
    },
    "Assertion": {
      "type": "object",
      "required": ["type", "data"],
      "properties": {
        "type": {
          "type": "string",
          "enum": [
            "ai_generation",
            "detection_result",
            "creative_work",
            "actions",
            "watermark",
            "custom"
          ]
        },
        "data": {
          "type": "object"
        },
        "timestamp": {
          "type": "string",
          "format": "date-time"
        }
      }
    },
    "Signature": {
      "type": "object",
      "required": ["algorithm", "value", "certificate"],
      "properties": {
        "algorithm": {
          "type": "string",
          "enum": ["Ed25519", "ES256", "ES384", "RS256", "PS256"]
        },
        "value": {
          "type": "string",
          "description": "Base64 인코딩된 서명"
        },
        "certificate": {
          "type": "string",
          "description": "PEM 형식 인증서"
        },
        "timestamp_token": {
          "type": "object",
          "properties": {
            "authority": { "type": "string" },
            "time": { "type": "string", "format": "date-time" },
            "token": { "type": "string" }
          }
        }
      }
    },
    "Metadata": {
      "type": "object",
      "properties": {
        "created_at": {
          "type": "string",
          "format": "date-time"
        },
        "creator": {
          "type": "string"
        },
        "title": {
          "type": "string"
        },
        "description": {
          "type": "string"
        }
      }
    }
  }
}
```

---

## 요약

WIA 콘텐츠 AI 데이터 형식은 다음을 제공합니다:

1. **C2PA 호환 매니페스트** - 업계 표준과의 상호 운용성
2. **AI 생성 어설션** - AI 콘텐츠 특화 메타데이터
3. **탐지 결과 스키마** - 표준화된 탐지 결과 형식
4. **멀티 해시 바인딩** - 강력한 콘텐츠 무결성 검증
5. **워터마크 스키마** - 다양한 워터마킹 기술 지원

---

*© 2025 세계산업협회 (WIA). 모든 권리 보유.*
*弘益人間 (홍익인간) · 널리 인간을 이롭게 하라*
