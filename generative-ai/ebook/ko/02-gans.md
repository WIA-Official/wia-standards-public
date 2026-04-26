# 제2장: 생성적 적대 신경망 (GANs)

## 2.1 GAN 혁명

2014년 Ian Goodfellow와 동료들이 소개한 생성적 적대 신경망(GAN)은 생성 모델링 접근 방식을 근본적으로 변화시켰습니다. 핵심 통찰력은 아름답게 단순하면서도 깊이 강력했습니다: 하나는 샘플을 생성하고 다른 하나는 진위를 평가하는 두 신경망을 대립적으로 학습시킵니다.

### 핵심 개념

GANs는 두 가지 구성 요소로 구성됩니다:

**생성자 (G)**: 랜덤 노이즈를 입력으로 받아 합성 데이터 샘플을 생성합니다. 목표는 판별자를 속이는 사실적인 출력을 만드는 것입니다.

**판별자 (D)**: 학습 세트의 실제 데이터와 생성자의 합성 데이터를 모두 받습니다. 목표는 각 샘플을 실제 또는 가짜로 정확하게 분류하는 것입니다.

이러한 네트워크는 **미니맥스 게임**에 참여합니다:
- 생성자는 판별자의 오류율을 최대화하려고 합니다
- 판별자는 분류 오류를 최소화하려고 합니다
- 학습은 각 네트워크를 업데이트하는 것 사이에서 교대로 진행됩니다

## 2.2 GAN 아키텍처 및 학습

기본 GAN 아키텍처:

```python
class Generator(nn.Module):
    def __init__(self, latent_dim=100, img_channels=3):
        super().__init__()
        # 노이즈에서 이미지로 변환
        self.model = nn.Sequential(
            nn.Linear(latent_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 512),
            nn.ReLU(),
            nn.Linear(512, 1024),
            nn.ReLU(),
            nn.Linear(1024, img_channels * 64 * 64),
            nn.Tanh()
        )

    def forward(self, z):
        img = self.model(z)
        return img.view(img.size(0), img_channels, 64, 64)

class Discriminator(nn.Module):
    def __init__(self, img_channels=3):
        super().__init__()
        self.model = nn.Sequential(
            nn.Linear(img_channels * 64 * 64, 512),
            nn.LeakyReLU(0.2),
            nn.Linear(512, 256),
            nn.LeakyReLU(0.2),
            nn.Linear(256, 1),
            nn.Sigmoid()
        )

    def forward(self, img):
        img_flat = img.view(img.size(0), -1)
        validity = self.model(img_flat)
        return validity
```

### 학습 과정

GAN 학습은 판별자와 생성자를 교대로 업데이트합니다:

1. **판별자 학습**: 실제 이미지와 가짜 이미지를 구분하도록 학습
2. **생성자 학습**: 판별자를 속이는 이미지를 생성하도록 학습

### 학습 과제

- **모드 붕괴**: 생성자가 제한된 다양성만 생성
- **기울기 소실**: 판별자가 너무 좋아지면 학습이 정체
- **학습 불안정성**: 적대적 프로세스가 수렴하지 않고 진동
- **비수렴**: 평형점이 보장되지 않음

## 2.3 고급 GAN 아키텍처

### DCGAN (Deep Convolutional GAN)

합성곱 계층을 사용하여 안정성 향상:

- 풀링 계층을 스트라이드 합성곱으로 대체
- 배치 정규화 사용
- 완전 연결된 숨겨진 계층 제거
- 생성자에서 ReLU, 판별자에서 LeakyReLU 사용

### StyleGAN

스타일 기반 생성으로 전례 없는 제어:

- 매핑 네트워크로 잠재 공간 변환
- 적응형 인스턴스 정규화(AdaIN)
- 스타일 믹싱
- 고해상도 생성 (1024x1024+)

### CycleGAN

페어링되지 않은 이미지 간 변환:

- 두 개의 생성자: G: X→Y and F: Y→X
- 순환 일관성 손실
- 응용: 스타일 전송, 계절 변환, 도메인 적응

## 2.4 평가 지표

### Inception Score (IS)

생성된 이미지의 품질과 다양성 측정

### Fréchet Inception Distance (FID)

생성된 이미지와 실제 이미지의 분포 비교 (낮을수록 좋음)

### Precision and Recall

충실도(precision)와 다양성(recall) 모두 측정

## 2.5 실용적 응용

- 얼굴 생성 및 편집
- 이미지 초해상도
- 데이터 증강
- 창작 도구
- 과학적 응용 (약물 발견, 재료 과학)

## 요약

생성적 적대 신경망은 적대적 학습을 생성 모델링의 강력한 패러다임으로 도입했습니다. 생성자와 판별자 간의 경쟁을 통해 GAN은 많은 도메인에서 놀랍도록 사실적인 합성 데이터를 생성할 수 있습니다.

**주요 요점**:
- GAN은 생성자와 판별자 네트워크 간의 적대적 학습 사용
- 수많은 변형이 안정성, 제어 가능성 및 응용별 요구 사항을 해결
- 학습에는 신중한 조정과 모니터링이 필요
- 평가는 FID, IS 및 precision/recall과 같은 메트릭 사용
- 응용은 창작 도구, 데이터 증강 및 과학적 발견을 포괄
- 과제에는 학습 불안정성, 모드 붕괴 및 평가 어려움이 포함

## 복습 문제

1. GAN의 생성자와 판별자의 역할 설명
2. 모드 붕괴란 무엇이며 왜 발생하는가?
3. StyleGAN이 생성된 이미지에 대한 세밀한 제어를 달성하는 방법은?
4. FID와 Inception Score의 차이점은?
5. GAN의 세 가지 실용적 응용 설명

---

**弘益人間 (홍익인간)** - GAN은 경쟁이 어떻게 개선을 촉진할 수 있는지 보여줍니다. 이러한 강력한 도구를 개발할 때, 합성 의료 데이터를 생성하여 의료를 발전시키고, 인간의 창의성을 증폭시키는 예술적 도구를 만들며, 과학을 발전시키면서 개인 정보를 존중하는 시스템을 구축하는 데 사용되도록 합시다.

---

© 2025 SmileStory Inc. / WIA | WIA-AI-026 생성형 AI 표준
