# 1장: 패션 테크 소개

## 학습 목표

이 장을 마치면 다음을 이해할 수 있습니다:
- 디지털 패션 기술의 진화
- 현대 패션 테크 시스템의 핵심 구성요소
- 가상 피팅과 AR/VR 응용
- AI 기반 트렌드 예측과 추천
- 패션 기술의 비즈니스 영향

---

## 1.1 디지털 패션 혁명

패션 산업은 기술에 의해 변화를 겪고 있습니다. 한때 순수하게 물리적이었던 것이 이제 점점 더 디지털화되고 있습니다:

```
전통 패션 (2020년 이전)          디지털 패션 (2025년 이후)
┌──────────────────────┐              ┌──────────────────────┐
│ 실물 샘플            │              │ 3D 가상 샘플         │
│ 대면 피팅            │    ──────>   │ AR 가상 피팅         │
│ 트렌드 추측          │              │ AI 트렌드 예측       │
│ 사이즈 차트          │              │ 개인화된 사이징      │
│ 선형 공급망          │              │ 블록체인 추적        │
└──────────────────────┘              └──────────────────────┘
```

### 영향

**환경적:**
- 실물 샘플 생산 70% 감소
- 디지털 패턴 최적화를 통한 소재 낭비 50% 감소
- 투명한 탄소 발자국 추적

**비즈니스:**
- 더 나은 핏 예측으로 인한 반품 35-45% 감소
- 60% 더 빠른 디자인-시장 출시 주기
- AR 피팅으로 고객 참여 40% 증가

**소비자:**
- 개인화된 사이즈 추천 (92% 정확도)
- 가상 옷장과 의상 계획
- 메타버스 플랫폼에서 디지털 패션 접근

---

## 1.2 핵심 기술

### 1.2.1 3D 디지털 의류 모델링

현실적인 소재와 물리 효과로 가상 의류 제작:

```typescript
interface DigitalGarment {
  id: string;
  type: 'dress' | 'shirt' | 'pants' | 'jacket' | 'accessory';

  // 3D 에셋 참조
  models: {
    high: string;    // 제품 페이지용 20K-50K 폴리곤
    medium: string;  // 가상 피팅용 5K-10K 폴리곤
    low: string;     // 썸네일용 1K-3K 폴리곤
  };

  // 소재 속성
  materials: Array<{
    type: string;           // 예: "organic_cotton"
    percentage: number;     // 0-100
    properties: {
      density: number;      // g/cm³
      stretch: number;      // 신축 계수 (1.0-2.0)
      roughness: number;    // PBR 거칠기 (0-1)
      sustainability: number; // 점수 0-100
    };
  }>;

  // 물리 시뮬레이션 매개변수
  physics: {
    clothType: 'cotton' | 'silk' | 'denim' | 'leather' | 'knit';
    stiffness: number;      // 스프링 상수 (N/m)
    damping: number;        // 0-1
    weight: number;         // g/m²
  };
}
```

**예제: 가상 드레스 만들기**

```typescript
const summerDress: DigitalGarment = {
  id: 'DRESS-2026-001',
  type: 'dress',

  models: {
    high: 'https://cdn.example.com/dress-001-high.glb',
    medium: 'https://cdn.example.com/dress-001-med.glb',
    low: 'https://cdn.example.com/dress-001-low.glb'
  },

  materials: [
    {
      type: 'organic_cotton',
      percentage: 95,
      properties: {
        density: 1.54,
        stretch: 1.05,
        roughness: 0.6,
        sustainability: 85
      }
    },
    {
      type: 'elastane',
      percentage: 5,
      properties: {
        density: 1.2,
        stretch: 1.8,
        roughness: 0.4,
        sustainability: 28
      }
    }
  ],

  physics: {
    clothType: 'cotton',
    stiffness: 150,      // 중간 강성
    damping: 0.15,       // 가벼운 감쇠
    weight: 180          // g/m²
  }
};
```

### 1.2.2 가상 피팅 시스템

두 가지 주요 접근 방식:

**AR 카메라 오버레이** (모바일)
```
┌─────────────────────────────────┐
│  사용자 카메라 피드             │
│  ┌──────────────────────┐      │
│  │                      │      │
│  │   [신체 감지]        │      │
│  │         │            │      │
│  │         ↓            │      │
│  │   [자세 추정]        │      │
│  │         │            │      │
│  │         ↓            │      │
│  │   [의류 오버레이]    │      │
│  │                      │      │
│  └──────────────────────┘      │
│                                 │
│  성능: 30-60 FPS                │
│  지연시간: <50ms                │
└─────────────────────────────────┘
```

**3D 아바타 피팅** (데스크톱/VR)
```
┌─────────────────────────────────┐
│  가상 피팅룸                    │
│  ┌──────────────────────┐      │
│  │   사용자 아바타      │      │
│  │   (스캔/측정값으로   │      │
│  │    생성)             │      │
│  │         │            │      │
│  │         ↓            │      │
│  │   [천 시뮬레이션]    │      │
│  │         │            │      │
│  │         ↓            │      │
│  │   [사실적 드레이프]  │      │
│  │                      │      │
│  │  [360° 뷰]  [핏]     │      │
│  └──────────────────────┘      │
└─────────────────────────────────┘
```

**TypeScript 구현 예제:**

```typescript
interface VirtualTryOnRequest {
  garmentId: string;
  mode: 'ar_camera' | 'avatar_3d';

  // AR 모드용
  videoStream?: MediaStream;

  // 아바타 모드용
  bodyMeasurements?: {
    height: number;     // cm
    chest: number;      // cm
    waist: number;      // cm
    hips: number;       // cm
  };

  renderQuality: 'low' | 'medium' | 'high';
}

interface VirtualTryOnResponse {
  success: boolean;

  // AR 모드 결과
  renderStream?: MediaStream;

  // 아바타 모드 결과
  avatarUrl?: string;

  // 핏 분석
  fitAnalysis: {
    overallFit: 'too_tight' | 'comfortable' | 'loose';
    confidence: number; // 0-1
    areas: {
      chest: 'tight' | 'comfortable' | 'loose';
      waist: 'tight' | 'comfortable' | 'loose';
      hips: 'tight' | 'comfortable' | 'loose';
      length: 'short' | 'perfect' | 'long';
    };
  };

  sizeRecommendation: {
    currentSize: string;
    recommendedSize: string;
    confidence: number;
    reason: string;
  };
}

// API 사용법
async function tryOnGarment(
  garmentId: string,
  measurements: BodyMeasurements
): Promise<VirtualTryOnResponse> {
  const response = await fetch('https://api.wiastandards.com/fashion/v1/virtual-tryon', {
    method: 'POST',
    headers: {
      'Authorization': 'Bearer YOUR_API_KEY',
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      garmentId,
      mode: 'avatar_3d',
      bodyMeasurements: measurements,
      renderQuality: 'high'
    })
  });

  return await response.json();
}
```

### 1.2.3 AI 기반 추천

패션 AI는 지능형 추천을 위해 여러 데이터 소스를 결합합니다:

```
데이터 소스:
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ 소셜 미디어  │  │ 런웨이 쇼    │  │ 판매 데이터  │
│ (인스타그램, │  │ (파리, NYC,  │  │ (전자상거래, │
│  틱톡)       │  │  밀라노)     │  │  소매)       │
└──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │                 │                 │
       └─────────────────┼─────────────────┘
                         ↓
                 ┌───────────────┐
                 │  AI 모델:     │
                 │  - LSTM       │
                 │  - XGBoost    │
                 │  - 신경망     │
                 └───────┬───────┘
                         ↓
         ┌───────────────────────────┐
         │  출력:                    │
         │  • 트렌드 예측            │
         │  • 스타일 추천            │
         │  • 사이즈 제안            │
         │  • 의상 조합              │
         └───────────────────────────┘
```

**예제: 트렌드 예측 모델**

```typescript
interface TrendPrediction {
  season: string;        // 예: "Spring 2026"
  category: string;      // 예: "dresses"

  predictions: Array<{
    trend: string;       // 예: "A-line midi dresses"
    strength: number;    // 0-1, 예측 신뢰도
    timeframe: string;   // 예: "1-3 months"

    attributes: {
      colors: string[];       // ["coral", "lavender"]
      patterns: string[];     // ["floral", "abstract"]
      styles: string[];       // ["romantic", "minimalist"]
    };

    confidence: number;       // 0-1
    dataSources: {
      socialMedia: number;    // 가중치: 0.40
      runway: number;         // 가중치: 0.30
      retail: number;         // 가중치: 0.30
    };
  }>;
}

// 사용 예제
const springTrends: TrendPrediction = {
  season: "Spring 2026",
  category: "dresses",

  predictions: [
    {
      trend: "디지털 라벤더",
      strength: 0.92,
      timeframe: "1-3개월",

      attributes: {
        colors: ["lavender", "periwinkle", "soft purple"],
        patterns: ["solid", "subtle geometric"],
        styles: ["minimalist", "futuristic", "clean lines"]
      },

      confidence: 0.88,
      dataSources: {
        socialMedia: 0.95,  // 매우 강한 신호
        runway: 0.85,       // 디자이너 채택
        retail: 0.78        // 초기 판매 데이터
      }
    }
  ]
};
```

---

## 1.3 기술을 통한 지속가능성

디지털 패션 기술은 전례 없는 지속가능성 추적을 가능하게 합니다:

### 탄소 발자국 계산

```typescript
interface SustainabilityMetrics {
  // 환경 영향
  carbonFootprint: {
    material: number;        // 소재 생산에서 kg CO₂e
    manufacturing: number;   // 의류 생산에서 kg CO₂e
    transport: number;       // 운송에서 kg CO₂e
    use: number;            // 세탁/건조에서 kg CO₂e
    endOfLife: number;      // 수명 종료 시 kg CO₂e (재활용 시 음수)
    total: number;          // 모든 단계의 합계
  };

  // 물 사용량
  waterFootprint: {
    material: number;        // 소재 생산용 리터
    manufacturing: number;   // 염색/가공용 리터
    use: number;            // 세탁용 리터
    total: number;
  };

  // 점수
  scores: {
    environmental: number;   // 0-100
    social: number;         // 0-100 (노동 조건)
    circular: number;       // 0-100 (재활용 가능성)
    total: number;          // 가중 평균
  };

  // 인증
  certifications: string[]; // ["GOTS", "Fair Trade", "B-Corp"]
}

// 예제: 의류 지속가능성 계산
function calculateSustainability(garment: DigitalGarment): SustainabilityMetrics {
  // 소재 탄소
  const materialCarbon = garment.materials.reduce((total, mat) => {
    const carbonFactor = getCarbonFactor(mat.type); // kg당 kg CO₂e
    const weight = garment.physics.weight / 1000; // g/m²를 kg로 변환
    return total + (weight * mat.percentage / 100 * carbonFactor);
  }, 0);

  // 제조 탄소 (복잡도 기반)
  const manufacturingCarbon = 1.0 * getComplexityFactor(garment.type);

  // 사용 단계 (50회 세탁, 냉수, 자연 건조)
  const useCarbon = 0.15 * 50;

  // 환경 점수 계산
  const environmentalScore = 100 - (
    (materialCarbon / 17) * 35 +           // 최대: 17 kg CO₂e/kg (가죽)
    (garment.waterFootprint / 125000) * 25 + // 최대: 125k L/kg (양모)
    (1 - garment.recyclability) * 20 +
    (garment.chemicalScore / 100) * 20
  );

  return {
    carbonFootprint: {
      material: materialCarbon,
      manufacturing: manufacturingCarbon,
      transport: 0.03,  // 해상 운송
      use: useCarbon,
      endOfLife: -1.0,  // 재활용/기증
      total: materialCarbon + manufacturingCarbon + 0.03 + useCarbon - 1.0
    },
    waterFootprint: {
      material: 2100,
      manufacturing: 50,
      use: 1250,
      total: 3400
    },
    scores: {
      environmental: environmentalScore,
      social: 87,
      circular: 86,
      total: environmentalScore * 0.4 + 87 * 0.3 + 86 * 0.3
    },
    certifications: ['GOTS', 'Fair Trade']
  };
}
```

### 지속가능성 등급 시스템

| 점수 | 등급 | 설명 |
|-------|--------|-------------|
| 90-100 | A+ | 탁월 - 최고 수준의 지속가능성 |
| 80-89 | A | 우수 - 강력한 지속가능성 관행 |
| 70-79 | B | 좋음 - 평균 이상의 지속가능성 |
| 60-69 | C | 보통 - 기본 요구사항 충족 |
| 50-59 | D | 미흡 - 업계 표준 이하 |
| <50 | F | 매우 미흡 - 상당한 개선 필요 |

---

## 1.4 메타버스와 NFT 패션

디지털 패션은 물리적 의류를 넘어 확장됩니다:

### NFT 패션 메타데이터

```typescript
interface NFTFashionItem {
  // 표준 NFT 필드
  name: string;
  description: string;
  image: string;           // IPFS URL
  animationUrl: string;    // 3D 모델 IPFS URL

  // WIA 패션 전용
  wiaFashion: {
    standard: 'WIA-IND-001';
    version: '1.0.0';

    // 멀티 플랫폼 3D 모델
    models: {
      decentraland: string;   // Decentraland용 GLB
      sandbox: string;        // The Sandbox용 VXM
      roblox: string;         // Roblox용 RBXM
      vrChat: string;         // VRChat용 FBX
      web: string;            // 웹 AR용 glTF
    };

    // 희귀도와 속성
    rarity: 'common' | 'uncommon' | 'rare' | 'epic' | 'legendary';
    edition: number;          // 예: 1 of 100

    // 지속가능성 (디지털용!)
    sustainability: {
      digitalOnly: boolean;
      renderingCarbonOffset: boolean;
      sustainableDesign: boolean;
    };

    // 잠금 해제 가능한 콘텐츠
    unlockables: {
      physicalVersion?: boolean;    // 실물 교환 가능
      arFilter?: string;            // Instagram/Snapchat 필터
      designFiles?: string;         // 3D 프린팅용
      exclusiveAccess?: string[];   // 이벤트, 커뮤니티
    };
  };

  // 블록체인 데이터
  contract: string;         // 스마트 컨트랙트 주소
  tokenId: number;
  blockchain: 'ethereum' | 'polygon' | 'solana';
}
```

**디지털 전용 vs. 피지털 패션:**

```
디지털 전용 NFT                  피지털 NFT
┌─────────────────┐              ┌─────────────────┐
│ • 메타버스 사용 │              │ • 실물 아이템   │
│ • 폐기물 없음   │              │ • + NFT 트윈    │
│ • 즉시 사용     │              │ • 인증됨        │
│ • 저렴함        │              │ • 양쪽 세계     │
└─────────────────┘              └─────────────────┘
      ↓                                   ↓
  탄소: 0                         탄소: 실물
  비용: $5-50                     비용: 실물 + NFT
  플랫폼: 다수                    혜택: 소유권 증명
```

---

## 1.5 비즈니스 영향과 ROI

### 핵심 성과 지표

```typescript
interface FashionTechROI {
  // 반품 감소
  returnRate: {
    before: number;      // 예: 30%
    after: number;       // 예: 18%
    reduction: number;   // 40% 감소
    costSavings: number; // 물류 비용 절감액
  };

  // 샘플 감소
  physicalSamples: {
    before: number;      // 예: 컬렉션당 100개 샘플
    after: number;       // 예: 30개 샘플
    reduction: number;   // 70% 감소
    materialSaved: number; // kg 단위 원단
    carbonSaved: number;   // kg CO₂e
  };

  // 고객 참여
  engagement: {
    virtualTryOnUsers: number;        // 방문자 중 %
    timeOnSite: number;               // 초 단위 증가
    conversionRate: number;           // % 증가
    customerSatisfaction: number;     // NPS 점수 변화
  };

  // 시장 출시 시간
  designCycle: {
    before: number;      // 주
    after: number;       // 주
    reduction: number;   // % 더 빠름
  };
}

// 예제 ROI 계산
const fashionTechROI: FashionTechROI = {
  returnRate: {
    before: 30,
    after: 18,
    reduction: 40,
    costSavings: 500000  // 연간 $500K
  },

  physicalSamples: {
    before: 100,
    after: 30,
    reduction: 70,
    materialSaved: 210,     // kg
    carbonSaved: 1239       // kg CO₂e
  },

  engagement: {
    virtualTryOnUsers: 45,     // 방문자의 45%가 AR 시도
    timeOnSite: 180,           // +3분
    conversionRate: 28,        // 28% 증가
    customerSatisfaction: 15   // +15 NPS 포인트
  },

  designCycle: {
    before: 12,
    after: 7,
    reduction: 42  // 42% 더 빠름
  }
};
```

---

## 1.6 산업 채택

### 부문별 사용 사례

**패스트 패션:**
- 가상 샘플링으로 시장 출시 시간 단축
- 신속한 디자인을 위한 AI 트렌드 예측
- 사이즈 추천을 통한 반품 감소

**럭셔리 패션:**
- NFT를 통한 디지털 인증
- 독점 메타버스 컬렉션
- 고가 아이템을 위한 AR 피팅

**지속가능한 브랜드:**
- 투명한 공급망 추적
- 탄소 발자국 라벨링
- 순환 패션 프로그램

**전자상거래:**
- 가상 피팅룸
- 개인화된 추천
- 반품 물류 감소

---

## 복습 문제

이해도를 테스트해보세요:

1. **디지털 의류 모델의 세 가지 주요 구성요소는 무엇입니까?**
   <details>
   <summary>답변</summary>
   3D 메시/지오메트리, 소재 속성 (PBR 텍스처), 물리 시뮬레이션 매개변수 (강성, 감쇠, 무게).
   </details>

2. **다음 값으로 면 드레스의 총 탄소 발자국을 계산하세요:**
   - 소재: 1.77 kg CO₂e
   - 제조: 1.30 kg CO₂e
   - 운송: 0.03 kg CO₂e
   - 사용 (75회 세탁, 냉수): 11.25 kg CO₂e
   - 수명 종료 (기증): -1.00 kg CO₂e

   <details>
   <summary>답변</summary>
   총계 = 1.77 + 1.30 + 0.03 + 11.25 - 1.00 = 13.35 kg CO₂e
   </details>

3. **AR 카메라 피팅과 3D 아바타 피팅의 차이점은 무엇입니까?**
   <details>
   <summary>답변</summary>
   AR 카메라는 실시간 비디오 피드에 의류를 오버레이하고 (실시간, 모바일), 3D 아바타는 사용자의 가상 모델을 사용하여 더 정확한 물리 시뮬레이션을 제공합니다 (데스크톱/VR, 고품질).
   </details>

4. **AI 트렌드 예측에 사용되는 세 가지 데이터 소스를 말해보세요.**
   <details>
   <summary>답변</summary>
   소셜 미디어 (Instagram, TikTok), 디자이너 런웨이 쇼 (패션 위크), 소매 판매 데이터 (전자상거래, POS).
   </details>

5. **가상 피팅이 달성할 수 있는 반품 감소율은 얼마입니까?**
   <details>
   <summary>답변</summary>
   사이즈 추천과 결합 시 반품률 35-45% 감소.
   </details>

---

## 다음 단계

이제 패션 기술의 기본을 이해했으므로, [**2장: 현재 과제**](02-current-challenges.md)에서 이 디지털 변화를 주도하는 현재 과제를 살펴보겠습니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
