# 제5장: 탐지 프로토콜

## AI 콘텐츠 탐지 방법론

### 개요

본 장에서는 WIA 콘텐츠 AI 표준의 핵심 구성요소인 AI 생성 콘텐츠 탐지 방법론을 상세히 설명합니다. 이미지, 비디오, 오디오, 텍스트 각 미디어 유형에 대한 탐지 기술과 앙상블 접근 방식을 다룹니다.

---

## 5.1 탐지 아키텍처

### 다층 탐지 시스템

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WIA 콘텐츠 AI 탐지 아키텍처                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  입력 콘텐츠                                                                  │
│       │                                                                      │
│       ▼                                                                      │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                     전처리 레이어                                    │    │
│  │  • 형식 감지    • 크기 조정    • 정규화    • 품질 평가              │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│       │                                                                      │
│       ├─────────────┬─────────────┬─────────────┬─────────────┐            │
│       ▼             ▼             ▼             ▼             ▼            │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐          │
│  │ CNN     │  │ 주파수  │  │ 통계    │  │ 시맨틱  │  │ 워터마크│          │
│  │ 탐지기  │  │ 분석기  │  │ 분석기  │  │ 분석기  │  │ 탐지기  │          │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘          │
│       │             │             │             │             │            │
│       └─────────────┴─────────────┴─────────────┴─────────────┘            │
│                               │                                              │
│                               ▼                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                      앙상블 융합 레이어                              │    │
│  │  • 가중 평균    • 투표    • 스태킹    • 불일치 탐지                 │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│       │                                                                      │
│       ▼                                                                      │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                      최종 판정 레이어                                │    │
│  │  • 신뢰도 보정    • 임계값 적용    • 불확실성 정량화               │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│       │                                                                      │
│       ▼                                                                      │
│  [탐지 결과]                                                                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 핵심 탐지 파이프라인

```typescript
// 탐지 파이프라인 오케스트레이터
interface DetectionPipeline {
  preprocessors: Preprocessor[];
  detectors: AIDetector[];
  ensemble: EnsembleMethod;
  postprocessor: Postprocessor;
}

interface Preprocessor {
  name: string;
  process(input: ContentInput): ProcessedContent;
}

interface AIDetector {
  name: string;
  version: string;
  contentTypes: ContentType[];
  detect(content: ProcessedContent): DetectorResult;
}

interface EnsembleMethod {
  name: string;
  combine(results: DetectorResult[]): CombinedResult;
}

interface Postprocessor {
  calibrate(result: CombinedResult): FinalResult;
}

type ContentType = 'image' | 'video' | 'audio' | 'text';

interface ContentInput {
  data: Buffer;
  contentType: ContentType;
  metadata?: Record<string, unknown>;
}

interface ProcessedContent {
  original: ContentInput;
  normalized: Buffer;
  features: Record<string, unknown>;
}

interface DetectorResult {
  detectorName: string;
  score: number;           // 0.0 - 1.0 (1.0 = AI 생성)
  confidence: number;      // 탐지기의 신뢰도
  details: Record<string, unknown>;
}

interface CombinedResult {
  score: number;
  confidence: number;
  detectorScores: Map<string, number>;
  disagreement: number;    // 탐지기 간 불일치 정도
}

interface FinalResult {
  isAIGenerated: boolean;
  confidence: number;
  verdict: 'ai_generated' | 'human_created' | 'uncertain' | 'manipulated';
  explanation: string[];
}

// 탐지 파이프라인 실행기
class DetectionPipelineRunner {
  private pipeline: DetectionPipeline;

  constructor(pipeline: DetectionPipeline) {
    this.pipeline = pipeline;
  }

  async execute(input: ContentInput): Promise<FinalResult> {
    // 1. 전처리
    let processed: ProcessedContent = {
      original: input,
      normalized: input.data,
      features: {}
    };

    for (const preprocessor of this.pipeline.preprocessors) {
      processed = preprocessor.process(processed);
    }

    // 2. 병렬 탐지
    const detectorPromises = this.pipeline.detectors
      .filter(d => d.contentTypes.includes(input.contentType))
      .map(detector => detector.detect(processed));

    const detectorResults = await Promise.all(detectorPromises);

    // 3. 앙상블 융합
    const combined = this.pipeline.ensemble.combine(detectorResults);

    // 4. 후처리 및 보정
    const final = this.pipeline.postprocessor.calibrate(combined);

    return final;
  }
}
```

---

## 5.2 이미지 탐지

### CNN 기반 탐지기

```python
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from typing import Dict, Tuple, List
import numpy as np

class CNNImageDetector(nn.Module):
    """
    CNN 기반 AI 이미지 탐지기
    EfficientNet 백본 + 커스텀 분류 헤드
    """

    def __init__(
        self,
        backbone: str = "efficientnet_b4",
        num_classes: int = 2,
        pretrained: bool = True
    ):
        super().__init__()

        # 백본 네트워크 로드
        if backbone == "efficientnet_b4":
            from torchvision.models import efficientnet_b4, EfficientNet_B4_Weights
            weights = EfficientNet_B4_Weights.DEFAULT if pretrained else None
            self.backbone = efficientnet_b4(weights=weights)
            in_features = self.backbone.classifier[1].in_features
            self.backbone.classifier = nn.Identity()
        else:
            raise ValueError(f"지원하지 않는 백본: {backbone}")

        # 분류 헤드
        self.classifier = nn.Sequential(
            nn.Linear(in_features, 1024),
            nn.BatchNorm1d(1024),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(1024, 512),
            nn.BatchNorm1d(512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(512, num_classes)
        )

        # 보조 태스크 헤드 (멀티태스크 학습)
        self.generator_head = nn.Linear(512, 10)  # 생성기 분류
        self.manipulation_head = nn.Linear(512, 5)  # 조작 유형

    def forward(
        self,
        x: torch.Tensor
    ) -> Dict[str, torch.Tensor]:
        # 특징 추출
        features = self.backbone(x)

        # 중간 특징 (보조 태스크용)
        hidden = self.classifier[:-1](features)

        # 주요 분류
        detection = self.classifier[-1](hidden)

        # 보조 태스크
        generator_pred = self.generator_head(hidden)
        manipulation_pred = self.manipulation_head(hidden)

        return {
            'detection': detection,
            'generator': generator_pred,
            'manipulation': manipulation_pred,
            'features': features
        }

    def predict(
        self,
        image: np.ndarray,
        device: str = 'cuda'
    ) -> Dict:
        """단일 이미지 예측"""
        self.eval()

        # 전처리
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((384, 384)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])

        tensor = transform(image).unsqueeze(0).to(device)

        with torch.no_grad():
            outputs = self.forward(tensor)
            probs = F.softmax(outputs['detection'], dim=1)

            ai_prob = probs[0, 1].item()
            generator_pred = torch.argmax(outputs['generator'], dim=1).item()

        return {
            'ai_probability': ai_prob,
            'is_ai_generated': ai_prob > 0.5,
            'predicted_generator': generator_pred,
            'confidence': abs(ai_prob - 0.5) * 2
        }


class FrequencyDomainDetector:
    """
    주파수 영역 분석 기반 AI 탐지기
    GAN 및 확산 모델의 주파수 아티팩트 탐지
    """

    def __init__(self):
        self.name = "FrequencyDomainDetector"
        self.version = "1.0"

    def analyze(self, image: np.ndarray) -> Dict:
        """주파수 영역 분석 수행"""
        # 그레이스케일 변환
        if len(image.shape) == 3:
            gray = np.mean(image, axis=2)
        else:
            gray = image

        # 2D FFT
        f_transform = np.fft.fft2(gray)
        f_shift = np.fft.fftshift(f_transform)
        magnitude = np.abs(f_shift)
        log_magnitude = np.log1p(magnitude)

        # 방사 평균 프로파일
        radial_profile = self._compute_radial_profile(magnitude)

        # 특징 추출
        features = {
            'high_freq_ratio': self._high_frequency_ratio(magnitude),
            'spectral_flatness': self._spectral_flatness(radial_profile),
            'periodicity_score': self._detect_periodicity(magnitude),
            'dct_artifacts': self._detect_dct_artifacts(gray),
            'noise_pattern': self._analyze_noise_pattern(gray)
        }

        # AI 가능성 점수 계산
        ai_likelihood = self._compute_ai_likelihood(features)

        return {
            'features': features,
            'ai_likelihood': ai_likelihood,
            'radial_profile': radial_profile.tolist()
        }

    def _compute_radial_profile(self, magnitude: np.ndarray) -> np.ndarray:
        """방사 평균 계산"""
        rows, cols = magnitude.shape
        center_r, center_c = rows // 2, cols // 2

        y, x = np.ogrid[:rows, :cols]
        r = np.sqrt((x - center_c)**2 + (y - center_r)**2).astype(int)

        max_r = min(center_r, center_c)
        radial_sum = np.bincount(r.ravel(), weights=magnitude.ravel())
        radial_count = np.bincount(r.ravel())

        profile = radial_sum[:max_r] / (radial_count[:max_r] + 1e-10)
        return profile

    def _high_frequency_ratio(self, magnitude: np.ndarray) -> float:
        """고주파 에너지 비율"""
        rows, cols = magnitude.shape
        center_r, center_c = rows // 2, cols // 2

        y, x = np.ogrid[:rows, :cols]
        r = np.sqrt((x - center_c)**2 + (y - center_r)**2)
        threshold = min(center_r, center_c) * 0.7

        high_freq_energy = np.sum(magnitude[r > threshold])
        total_energy = np.sum(magnitude)

        return high_freq_energy / (total_energy + 1e-10)

    def _spectral_flatness(self, profile: np.ndarray) -> float:
        """스펙트럼 평탄도 (위너 엔트로피)"""
        profile = profile + 1e-10
        geometric_mean = np.exp(np.mean(np.log(profile)))
        arithmetic_mean = np.mean(profile)
        return geometric_mean / arithmetic_mean

    def _detect_periodicity(self, magnitude: np.ndarray) -> float:
        """주기적 패턴 탐지"""
        autocorr = np.fft.ifft2(np.abs(magnitude)**2)
        autocorr = np.abs(np.fft.fftshift(autocorr))
        autocorr = autocorr / np.max(autocorr)

        # 중심 제외한 피크 카운트
        threshold = 0.3
        peaks = autocorr > threshold
        peak_count = np.sum(peaks) - 1

        return min(peak_count / 100.0, 1.0)

    def _detect_dct_artifacts(self, gray: np.ndarray) -> bool:
        """DCT 블록 아티팩트 탐지"""
        # 8x8 블록 경계에서 불연속성 검사
        block_size = 8
        rows, cols = gray.shape

        discontinuities = []
        for i in range(block_size, rows - block_size, block_size):
            row_diff = np.abs(gray[i, :] - gray[i-1, :]).mean()
            discontinuities.append(row_diff)

        for j in range(block_size, cols - block_size, block_size):
            col_diff = np.abs(gray[:, j] - gray[:, j-1]).mean()
            discontinuities.append(col_diff)

        if not discontinuities:
            return False

        avg_discontinuity = np.mean(discontinuities)
        return avg_discontinuity > 5.0  # 임계값

    def _analyze_noise_pattern(self, gray: np.ndarray) -> Dict:
        """노이즈 패턴 분석"""
        # 가우시안 차이 (DoG)로 노이즈 추출
        from scipy.ndimage import gaussian_filter

        smooth1 = gaussian_filter(gray, sigma=1.0)
        smooth2 = gaussian_filter(gray, sigma=2.0)
        noise = gray - smooth1

        return {
            'noise_std': float(np.std(noise)),
            'noise_mean': float(np.mean(np.abs(noise))),
            'noise_uniformity': float(np.std(noise) / (np.mean(np.abs(noise)) + 1e-10))
        }

    def _compute_ai_likelihood(self, features: Dict) -> float:
        """특징에서 AI 가능성 계산"""
        score = 0.0

        # 낮은 고주파 비율 (AI 이미지 특성)
        if features['high_freq_ratio'] < 0.1:
            score += 0.25
        elif features['high_freq_ratio'] < 0.2:
            score += 0.15

        # 높은 스펙트럼 평탄도
        if features['spectral_flatness'] > 0.5:
            score += 0.25
        elif features['spectral_flatness'] > 0.3:
            score += 0.15

        # 주기적 패턴
        if features['periodicity_score'] > 0.3:
            score += 0.25
        elif features['periodicity_score'] > 0.1:
            score += 0.15

        # 균일한 노이즈 패턴 (AI 특성)
        if features['noise_pattern']['noise_uniformity'] > 2.0:
            score += 0.25

        return min(score, 1.0)
```

---

## 5.3 딥페이크 탐지

### 비디오 딥페이크 탐지기

```python
import torch
import torch.nn as nn
import numpy as np
from typing import Dict, List, Tuple
import cv2

class DeepfakeDetector:
    """
    비디오 딥페이크 탐지기
    시간적 일관성, 얼굴 분석, 립싱크 검증
    """

    def __init__(self, device: str = 'cuda'):
        self.device = device
        self.face_detector = self._load_face_detector()
        self.temporal_analyzer = TemporalConsistencyAnalyzer()
        self.lip_sync_analyzer = LipSyncAnalyzer()
        self.physiological_analyzer = PhysiologicalSignalAnalyzer()

    def _load_face_detector(self):
        """얼굴 탐지기 로드"""
        # MTCNN 또는 RetinaFace 사용
        from facenet_pytorch import MTCNN
        return MTCNN(device=self.device)

    def detect(
        self,
        video_path: str,
        sample_rate: int = 5
    ) -> Dict:
        """비디오 딥페이크 탐지"""
        # 프레임 추출
        frames = self._extract_frames(video_path, sample_rate)

        if len(frames) < 10:
            return {'error': '프레임 수 부족', 'is_deepfake': None}

        # 얼굴 추출
        faces = self._extract_faces(frames)

        if len(faces) < 5:
            return {'error': '얼굴 탐지 실패', 'is_deepfake': None}

        # 다중 분석 수행
        results = {}

        # 1. 시간적 일관성 분석
        temporal_result = self.temporal_analyzer.analyze(faces)
        results['temporal'] = temporal_result

        # 2. 립싱크 분석 (오디오가 있는 경우)
        audio = self._extract_audio(video_path)
        if audio is not None:
            lip_sync_result = self.lip_sync_analyzer.analyze(faces, audio)
            results['lip_sync'] = lip_sync_result

        # 3. 생리적 신호 분석
        physiological_result = self.physiological_analyzer.analyze(faces)
        results['physiological'] = physiological_result

        # 앙상블 결과
        final_score = self._ensemble_scores(results)

        return {
            'is_deepfake': final_score > 0.5,
            'confidence': final_score,
            'details': results,
            'frame_count': len(frames),
            'face_count': len(faces)
        }

    def _extract_frames(
        self,
        video_path: str,
        sample_rate: int
    ) -> List[np.ndarray]:
        """비디오에서 프레임 추출"""
        frames = []
        cap = cv2.VideoCapture(video_path)

        frame_idx = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            if frame_idx % sample_rate == 0:
                frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            frame_idx += 1

        cap.release()
        return frames

    def _extract_faces(self, frames: List[np.ndarray]) -> List[np.ndarray]:
        """프레임에서 얼굴 추출"""
        faces = []

        for frame in frames:
            boxes, _ = self.face_detector.detect(frame)

            if boxes is not None and len(boxes) > 0:
                # 가장 큰 얼굴 선택
                box = boxes[0].astype(int)
                x1, y1, x2, y2 = box
                face = frame[y1:y2, x1:x2]

                if face.size > 0:
                    faces.append(cv2.resize(face, (224, 224)))

        return faces

    def _extract_audio(self, video_path: str) -> np.ndarray:
        """비디오에서 오디오 추출"""
        try:
            import librosa
            audio, sr = librosa.load(video_path, sr=16000)
            return audio
        except Exception:
            return None

    def _ensemble_scores(self, results: Dict) -> float:
        """다중 분석 결과 앙상블"""
        scores = []
        weights = []

        if 'temporal' in results:
            scores.append(results['temporal']['score'])
            weights.append(0.4)

        if 'lip_sync' in results:
            scores.append(results['lip_sync']['score'])
            weights.append(0.3)

        if 'physiological' in results:
            scores.append(results['physiological']['score'])
            weights.append(0.3)

        if not scores:
            return 0.5

        # 가중 평균
        total_weight = sum(weights)
        weighted_sum = sum(s * w for s, w in zip(scores, weights))

        return weighted_sum / total_weight


class TemporalConsistencyAnalyzer:
    """시간적 일관성 분석기"""

    def analyze(self, faces: List[np.ndarray]) -> Dict:
        """연속 프레임 간 일관성 분석"""
        if len(faces) < 2:
            return {'score': 0.5, 'error': '프레임 부족'}

        inconsistencies = []

        for i in range(len(faces) - 1):
            # 구조적 유사도 계산
            ssim = self._compute_ssim(faces[i], faces[i + 1])

            # 급격한 변화 탐지
            diff = np.abs(faces[i].astype(float) - faces[i + 1].astype(float))
            avg_diff = np.mean(diff)

            # 일관성 점수 (불일관성이 높으면 딥페이크 가능성)
            if ssim < 0.7 or avg_diff > 50:
                inconsistencies.append({
                    'frame_pair': (i, i + 1),
                    'ssim': ssim,
                    'avg_diff': avg_diff
                })

        # 불일관성 비율 계산
        inconsistency_ratio = len(inconsistencies) / (len(faces) - 1)

        return {
            'score': inconsistency_ratio,
            'inconsistent_frames': len(inconsistencies),
            'total_pairs': len(faces) - 1,
            'details': inconsistencies[:10]  # 상위 10개만
        }

    def _compute_ssim(
        self,
        img1: np.ndarray,
        img2: np.ndarray
    ) -> float:
        """SSIM 계산"""
        from skimage.metrics import structural_similarity
        return structural_similarity(
            img1, img2, multichannel=True, data_range=255
        )


class LipSyncAnalyzer:
    """립싱크 분석기"""

    def analyze(
        self,
        faces: List[np.ndarray],
        audio: np.ndarray
    ) -> Dict:
        """오디오와 입술 움직임 동기화 분석"""
        # 입술 영역 추출
        lip_movements = self._extract_lip_movements(faces)

        # 오디오 에너지 추출
        audio_energy = self._extract_audio_energy(audio, len(faces))

        # 상관관계 분석
        if len(lip_movements) != len(audio_energy):
            min_len = min(len(lip_movements), len(audio_energy))
            lip_movements = lip_movements[:min_len]
            audio_energy = audio_energy[:min_len]

        correlation = np.corrcoef(lip_movements, audio_energy)[0, 1]

        # 낮은 상관관계 = 딥페이크 가능성
        # (실제 영상은 높은 상관관계를 보임)
        if np.isnan(correlation):
            correlation = 0.5

        desync_score = 1 - abs(correlation)

        return {
            'score': desync_score,
            'correlation': float(correlation),
            'lip_movement_variance': float(np.var(lip_movements)),
            'audio_energy_variance': float(np.var(audio_energy))
        }

    def _extract_lip_movements(self, faces: List[np.ndarray]) -> np.ndarray:
        """입술 움직임 추출"""
        movements = []

        for i, face in enumerate(faces):
            # 하단 1/3 영역 (입술 근처)
            h, w = face.shape[:2]
            lip_region = face[int(h * 0.6):, :]

            # 움직임 크기 계산
            if i > 0:
                prev_lip = faces[i - 1][int(h * 0.6):, :]
                movement = np.mean(np.abs(
                    lip_region.astype(float) - prev_lip.astype(float)
                ))
            else:
                movement = 0

            movements.append(movement)

        return np.array(movements)

    def _extract_audio_energy(
        self,
        audio: np.ndarray,
        num_frames: int
    ) -> np.ndarray:
        """오디오 에너지 추출"""
        # 프레임 수에 맞게 분할
        frame_length = len(audio) // num_frames

        energies = []
        for i in range(num_frames):
            start = i * frame_length
            end = (i + 1) * frame_length
            segment = audio[start:end]
            energy = np.sqrt(np.mean(segment ** 2))
            energies.append(energy)

        return np.array(energies)


class PhysiologicalSignalAnalyzer:
    """생리적 신호 분석기"""

    def analyze(self, faces: List[np.ndarray]) -> Dict:
        """눈 깜빡임, PPG 등 생리적 신호 분석"""
        # 눈 깜빡임 분석
        blink_result = self._analyze_blinking(faces)

        # 피부색 변화 (PPG 추정)
        ppg_result = self._analyze_ppg(faces)

        # 자연스러움 점수 계산
        naturalness_score = (
            blink_result['naturalness'] * 0.5 +
            ppg_result['naturalness'] * 0.5
        )

        # 비자연스러움 = 딥페이크 가능성
        deepfake_score = 1 - naturalness_score

        return {
            'score': deepfake_score,
            'blink_analysis': blink_result,
            'ppg_analysis': ppg_result,
            'naturalness': naturalness_score
        }

    def _analyze_blinking(self, faces: List[np.ndarray]) -> Dict:
        """눈 깜빡임 패턴 분석"""
        # 눈 영역 추출 및 개폐 상태 분석
        eye_states = []

        for face in faces:
            h, w = face.shape[:2]
            # 상단 1/3 영역 (눈 근처)
            eye_region = face[:int(h * 0.4), :]

            # 밝기 기반 단순 분석
            brightness = np.mean(eye_region)
            eye_states.append(brightness)

        # 깜빡임 빈도 분석
        state_changes = np.diff(eye_states)
        blink_count = np.sum(np.abs(state_changes) > 10)

        # 정상 깜빡임: 분당 15-20회, 30fps 영상 기준
        expected_blinks_per_sec = 0.3
        video_duration_sec = len(faces) / 30  # 가정
        expected_blinks = expected_blinks_per_sec * video_duration_sec

        # 비정상적 깜빡임 패턴 탐지
        blink_ratio = blink_count / (expected_blinks + 1)
        naturalness = 1 - abs(1 - blink_ratio)
        naturalness = max(0, min(1, naturalness))

        return {
            'blink_count': int(blink_count),
            'expected_blinks': expected_blinks,
            'naturalness': naturalness
        }

    def _analyze_ppg(self, faces: List[np.ndarray]) -> Dict:
        """원격 PPG (심박) 신호 분석"""
        # 피부 영역의 색상 변화 추적
        color_signals = []

        for face in faces:
            # 이마/볼 영역
            h, w = face.shape[:2]
            forehead = face[int(h * 0.1):int(h * 0.3), int(w * 0.3):int(w * 0.7)]

            # 녹색 채널 평균 (PPG에 가장 민감)
            if len(forehead.shape) == 3:
                green_mean = np.mean(forehead[:, :, 1])
            else:
                green_mean = np.mean(forehead)

            color_signals.append(green_mean)

        color_signals = np.array(color_signals)

        # 변동성 분석
        if len(color_signals) > 10:
            signal_std = np.std(color_signals)
            signal_range = np.max(color_signals) - np.min(color_signals)

            # 자연스러운 PPG 신호는 적당한 변동성을 보임
            # 너무 낮거나 높은 변동성은 비자연스러움
            normalized_std = signal_std / (signal_range + 1e-10)

            if 0.1 < normalized_std < 0.5:
                naturalness = 1.0
            else:
                naturalness = max(0, 1 - abs(0.3 - normalized_std) * 3)
        else:
            naturalness = 0.5

        return {
            'signal_variability': float(np.std(color_signals)) if len(color_signals) > 0 else 0,
            'naturalness': naturalness
        }
```

---

## 5.4 텍스트 탐지

### AI 텍스트 탐지기

```python
import torch
import torch.nn as nn
from transformers import AutoTokenizer, AutoModelForSequenceClassification
from typing import Dict, List
import numpy as np
import re

class AITextDetector:
    """
    AI 생성 텍스트 탐지기
    퍼플렉서티, 문체 분석, 제로샷 분류 결합
    """

    def __init__(
        self,
        model_name: str = "roberta-base",
        device: str = 'cuda'
    ):
        self.device = device
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModelForSequenceClassification.from_pretrained(
            model_name,
            num_labels=2
        ).to(device)
        self.model.eval()

    def detect(self, text: str) -> Dict:
        """텍스트 AI 탐지"""
        # 기본 검증
        if len(text.strip()) < 20:
            return {'error': '텍스트 너무 짧음', 'is_ai_generated': None}

        # 1. 분류기 예측
        classifier_result = self._classify(text)

        # 2. 퍼플렉서티 분석
        perplexity_result = self._analyze_perplexity(text)

        # 3. 문체 분석
        stylometric_result = self._analyze_stylometry(text)

        # 4. 버스티니스 분석
        burstiness_result = self._analyze_burstiness(text)

        # 앙상블
        combined_score = self._combine_scores({
            'classifier': classifier_result['score'],
            'perplexity': perplexity_result['score'],
            'stylometric': stylometric_result['score'],
            'burstiness': burstiness_result['score']
        })

        return {
            'is_ai_generated': combined_score > 0.5,
            'confidence': combined_score,
            'details': {
                'classifier': classifier_result,
                'perplexity': perplexity_result,
                'stylometric': stylometric_result,
                'burstiness': burstiness_result
            }
        }

    def _classify(self, text: str) -> Dict:
        """분류기 기반 예측"""
        inputs = self.tokenizer(
            text,
            truncation=True,
            max_length=512,
            padding=True,
            return_tensors='pt'
        ).to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)
            probs = torch.softmax(outputs.logits, dim=1)
            ai_prob = probs[0, 1].item()

        return {
            'score': ai_prob,
            'confidence': abs(ai_prob - 0.5) * 2
        }

    def _analyze_perplexity(self, text: str) -> Dict:
        """퍼플렉서티 분석"""
        # 단어 수준 엔트로피 추정
        words = text.split()
        unique_words = set(words)

        if len(words) == 0:
            return {'score': 0.5, 'perplexity': 0}

        # 어휘 다양성
        vocab_richness = len(unique_words) / len(words)

        # 단순 퍼플렉서티 추정
        # AI 텍스트는 일반적으로 낮은 퍼플렉서티
        word_freq = {}
        for word in words:
            word_freq[word] = word_freq.get(word, 0) + 1

        entropy = -sum(
            (f / len(words)) * np.log2(f / len(words) + 1e-10)
            for f in word_freq.values()
        )

        perplexity = 2 ** entropy

        # AI 텍스트는 낮은 퍼플렉서티 경향
        # 정규화된 점수 (낮은 퍼플렉서티 = 높은 AI 가능성)
        normalized_score = 1 - min(perplexity / 100, 1)

        return {
            'score': normalized_score,
            'perplexity': perplexity,
            'vocab_richness': vocab_richness
        }

    def _analyze_stylometry(self, text: str) -> Dict:
        """문체 분석"""
        sentences = re.split(r'[.!?]+', text)
        sentences = [s.strip() for s in sentences if s.strip()]

        if len(sentences) < 2:
            return {'score': 0.5, 'error': '문장 수 부족'}

        # 문장 길이 분포
        lengths = [len(s.split()) for s in sentences]
        avg_length = np.mean(lengths)
        std_length = np.std(lengths)

        # AI 텍스트는 더 균일한 문장 길이 경향
        cv = std_length / (avg_length + 1e-10)  # 변동 계수

        # 구두점 패턴
        punctuation = re.findall(r'[,;:\-]', text)
        punct_ratio = len(punctuation) / len(text.split())

        # 연결어 빈도
        connectives = ['그러나', '따라서', '또한', '게다가', '반면에', '결과적으로']
        connective_count = sum(text.count(c) for c in connectives)
        connective_ratio = connective_count / len(sentences)

        # 종합 점수
        # CV가 낮고, 연결어가 많으면 AI 가능성 높음
        uniformity_score = 1 - min(cv, 1)
        connective_score = min(connective_ratio * 2, 1)

        stylometric_score = (uniformity_score + connective_score) / 2

        return {
            'score': stylometric_score,
            'sentence_length_cv': cv,
            'avg_sentence_length': avg_length,
            'connective_ratio': connective_ratio
        }

    def _analyze_burstiness(self, text: str) -> Dict:
        """버스티니스 분석 (문장 길이 변동)"""
        sentences = re.split(r'[.!?]+', text)
        sentences = [s.strip() for s in sentences if s.strip()]

        if len(sentences) < 3:
            return {'score': 0.5, 'burstiness': 0}

        lengths = [len(s.split()) for s in sentences]

        # 버스티니스 = 표준편차 / 평균
        mean_len = np.mean(lengths)
        std_len = np.std(lengths)
        burstiness = std_len / (mean_len + 1e-10)

        # AI 텍스트는 낮은 버스티니스 (균일한 문장 길이)
        # 인간 텍스트는 더 다양한 문장 길이
        ai_score = 1 - min(burstiness, 1)

        return {
            'score': ai_score,
            'burstiness': burstiness,
            'mean_length': mean_len,
            'std_length': std_len
        }

    def _combine_scores(self, scores: Dict[str, float]) -> float:
        """점수 앙상블"""
        weights = {
            'classifier': 0.4,
            'perplexity': 0.2,
            'stylometric': 0.2,
            'burstiness': 0.2
        }

        combined = sum(scores[k] * weights[k] for k in scores)
        return combined
```

---

## 5.5 앙상블 방법

### 다중 탐지기 융합

```typescript
// 앙상블 융합 전략
interface EnsembleStrategy {
  name: string;
  combine(results: DetectorResult[]): number;
}

class WeightedAverageEnsemble implements EnsembleStrategy {
  name = 'weighted_average';
  private weights: Map<string, number>;

  constructor(weights: Record<string, number>) {
    this.weights = new Map(Object.entries(weights));
  }

  combine(results: DetectorResult[]): number {
    let weightedSum = 0;
    let totalWeight = 0;

    for (const result of results) {
      const weight = this.weights.get(result.detectorName) || 1;
      weightedSum += result.score * weight * result.confidence;
      totalWeight += weight * result.confidence;
    }

    return totalWeight > 0 ? weightedSum / totalWeight : 0.5;
  }
}

class VotingEnsemble implements EnsembleStrategy {
  name = 'voting';
  private threshold: number;

  constructor(threshold: number = 0.5) {
    this.threshold = threshold;
  }

  combine(results: DetectorResult[]): number {
    const votes = results.map(r => r.score > this.threshold ? 1 : 0);
    return votes.reduce((a, b) => a + b, 0) / votes.length;
  }
}

class StackingEnsemble implements EnsembleStrategy {
  name = 'stacking';
  private metaModel: (scores: number[]) => number;

  constructor(metaModel: (scores: number[]) => number) {
    this.metaModel = metaModel;
  }

  combine(results: DetectorResult[]): number {
    const scores = results.map(r => r.score);
    return this.metaModel(scores);
  }
}

// 불일치 탐지기
class DisagreementDetector {
  detectDisagreement(results: DetectorResult[]): {
    hasDisagreement: boolean;
    disagreementScore: number;
    outliers: string[];
  } {
    const scores = results.map(r => r.score);
    const mean = scores.reduce((a, b) => a + b, 0) / scores.length;
    const std = Math.sqrt(
      scores.reduce((sum, s) => sum + (s - mean) ** 2, 0) / scores.length
    );

    const outliers = results
      .filter(r => Math.abs(r.score - mean) > 2 * std)
      .map(r => r.detectorName);

    return {
      hasDisagreement: std > 0.2,
      disagreementScore: std,
      outliers
    };
  }
}
```

---

## 요약

WIA 콘텐츠 AI 탐지 프로토콜은 다음을 제공합니다:

1. **다층 탐지 아키텍처** - 전처리, 탐지, 앙상블, 후처리
2. **이미지 탐지** - CNN + 주파수 분석 결합
3. **딥페이크 탐지** - 시간적 일관성 + 립싱크 + 생리 신호
4. **텍스트 탐지** - 분류기 + 퍼플렉서티 + 문체 분석
5. **앙상블 융합** - 다중 탐지기 결과 통합

---

*© 2025 세계산업협회 (WIA). 모든 권리 보유.*
*弘益人間 (홍익인간) · 널리 인간을 이롭게 하라*
