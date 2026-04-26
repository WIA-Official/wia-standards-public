# 3장: WIA 패션 기술 표준 개요

## 학습 목표

이 장을 마치면 다음을 이해할 수 있습니다:
- WIA-IND-001의 완전한 아키텍처
- 디지털 패션 시스템 구성 요소 및 상호작용
- 소재 데이터베이스 구조 및 지속가능성 프레임워크
- 트렌드 예측 및 추천을 위한 AI/ML 통합
- 모든 구성 요소의 협업 방식

---

## 3.1 표준 아키텍처

WIA 패션 기술 표준(WIA-IND-001)은 포괄적인 프레임워크를 제공합니다:

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA-IND-001 아키텍처                     │
└─────────────────────────────────────────────────────────────┘

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  디자인 레이어    │  │  디지털 트윈     │  │  소비자 레이어    │
│                  │  │     레이어       │  │                  │
│ • 3D 디자인      │→ │ • 3D 모델        │→ │ • 가상 착용      │
│ • 소재 라이브러리 │  │ • 메타데이터     │  │ • 사이즈 추천    │
│ • 패턴 시스템    │  │ • 시뮬레이션     │  │ • 스타일 AI      │
│ • AI 어시스턴트  │  │ • 렌더링         │  │ • 가상 옷장      │
└──────────────────┘  └──────────────────┘  └──────────────────┘
        ↓                      ↓                      ↓
┌─────────────────────────────────────────────────────────────┐
│                  지속가능성 레이어                           │
│  • 탄소 추적  • 물 발자국  • 사회적 지표                    │
│  • 순환성 점수  • 블록체인 추적성                           │
└─────────────────────────────────────────────────────────────┘
        ↓                      ↓                      ↓
┌─────────────────────────────────────────────────────────────┐
│                      데이터 레이어                           │
│  • JSON 스키마  • 3D 에셋(glTF/FBX)  • REST API            │
│  • 소재 데이터베이스  • NFT 메타데이터  • ML 모델           │
└─────────────────────────────────────────────────────────────┘
```

### 핵심 원칙

1. **상호운용성**: 플랫폼과 시스템 전반에서 작동
2. **지속가능성**: 환경 및 사회적 영향 추적
3. **정확성**: 현실적인 3D 모델 및 착용감 예측
4. **프라이버시**: 신체 치수의 안전한 처리
5. **개방성**: 산업 채택을 위한 개방형 표준

---

## 3.2 시스템 구성 요소

### 3.2.1 디자인 및 제작 레이어

의류가 탄생하는 곳입니다:

```typescript
interface DesignLayer {
  // 3D 디자인 도구 통합
  designTools: {
    software: string[];     // CLO3D, Marvelous Designer, Blender
    formats: string[];      // FBX, OBJ, glTF
    plugins: string[];      // WIA 내보내기 플러그인
  };

  // 소재 라이브러리 접근
  materialLibrary: {
    materials: Material[];  // 500개 이상의 소재 데이터베이스
    search: (criteria: MaterialCriteria) => Material[];
    sustainability: boolean; // 지속가능성으로 필터링
  };

  // 2D 패턴 시스템
  patternSystem: {
    templates: Pattern[];   // 기본 패턴
    customization: boolean; // 패턴 수정
    optimization: boolean;  // AI 폐기물 감소
  };

  // AI 디자인 어시스턴트
  aiAssistant: {
    textToDesign: (prompt: string) => Design3D;
    styleTransfer: (base: Design3D, style: Image) => Design3D;
    variations: (design: Design3D, count: number) => Design3D[];
  };
}

// 예제: AI 지원 디자인 워크플로우
const designWorkflow = {
  // 1. 디자이너가 텍스트 프롬프트 입력
  prompt: "지속가능한 원단의 A라인 미디 드레스, 플러터 슬리브, 코랄 핑크",

  // 2. AI가 초기 디자인 생성
  initialDesigns: [
    { id: 'var1', silhouette: 'A-line', sleeves: 'flutter', color: '#FF6B9D' },
    { id: 'var2', silhouette: 'A-line', sleeves: 'cap', color: '#FF8BA7' },
    { id: 'var3', silhouette: 'fit-flare', sleeves: 'flutter', color: '#FF6B9D' }
  ],

  // 3. 디자이너가 선택 및 수정
  selected: 'var1',
  refinements: {
    neckline: 'v-neck',
    length: 105,  // cm
    material: 'organic_cotton'
  },

  // 4. AI가 최소 폐기물을 위해 패턴 최적화
  patternOptimization: {
    originalEfficiency: 75,    // 사용된 원단 %
    optimizedEfficiency: 87,   // AI 최적화 후
    wasteSaved: 12             // 개선 %
  }
};
```

### 3.2.2 디지털 트윈 레이어

모든 의류에는 디지털 트윈이 있습니다:

```typescript
interface DigitalTwin {
  // 고유 식별자
  id: string;
  wiaStandard: 'WIA-IND-001';
  version: '1.0.0';

  // 다중 LOD의 3D 모델
  models: {
    lod0: {                    // 최고 품질
      url: string;
      format: 'glTF';
      polygons: number;        // 20,000-50,000
      textures: {
        baseColor: string;     // 4K 해상도
        normal: string;
        roughness: string;
        ao: string;
      };
    };
    lod1: {                    // 중간 품질
      url: string;
      polygons: number;        // 5,000-10,000
      textures: {
        baseColor: string;     // 2K 해상도
        normal: string;
      };
    };
    lod2: {                    // 낮은 품질
      url: string;
      polygons: number;        // 1,000-3,000
      textures: {
        baseColor: string;     // 1K 해상도
      };
    };
  };

  // 포괄적인 메타데이터
  metadata: {
    brand: string;
    name: string;
    category: string;
    season: string;
    price: number;
    currency: string;
  };

  // 물리 시뮬레이션 매개변수
  physics: {
    mass: number;              // kg
    stiffness: number;         // N/m
    damping: number;           // 0-1
    stretch: [number, number]; // [날실, 씨실]
    friction: number;          // 표면 마찰
  };

  // 지속가능성 데이터
  sustainability: {
    totalScore: number;        // 0-100
    carbonFootprint: number;   // kg CO₂e
    waterFootprint: number;    // 리터
    certifications: string[];
    materialPassport: string;  // IPFS 해시
  };

  // 렌더링 설정
  rendering: {
    castsShadows: boolean;
    receivesShadows: boolean;
    doubleSided: boolean;
    transparent: boolean;
    alphaMode: 'opaque' | 'mask' | 'blend';
  };
}

// 예제 디지털 트윈
const dressDigitalTwin: DigitalTwin = {
  id: 'WIA-DRESS-2026-12345',
  wiaStandard: 'WIA-IND-001',
  version: '1.0.0',

  models: {
    lod0: {
      url: 'ipfs://Qm.../dress-high.glb',
      format: 'glTF',
      polygons: 35000,
      textures: {
        baseColor: 'ipfs://Qm.../dress-color-4k.png',
        normal: 'ipfs://Qm.../dress-normal-4k.png',
        roughness: 'ipfs://Qm.../dress-rough-4k.png',
        ao: 'ipfs://Qm.../dress-ao-4k.png'
      }
    },
    lod1: {
      url: 'ipfs://Qm.../dress-med.glb',
      polygons: 8000,
      textures: {
        baseColor: 'ipfs://Qm.../dress-color-2k.png',
        normal: 'ipfs://Qm.../dress-normal-2k.png'
      }
    },
    lod2: {
      url: 'ipfs://Qm.../dress-low.glb',
      polygons: 2000,
      textures: {
        baseColor: 'ipfs://Qm.../dress-color-1k.png'
      }
    }
  },

  metadata: {
    brand: 'EcoFashion',
    name: '코랄 여름 드레스',
    category: 'dresses',
    season: '2026 봄',
    price: 89.99,
    currency: 'USD'
  },

  physics: {
    mass: 0.3,          // 300g
    stiffness: 150,     // 중간 강성
    damping: 0.15,
    stretch: [1.05, 1.03], // 약간의 신축성
    friction: 0.4
  },

  sustainability: {
    totalScore: 85,
    carbonFootprint: 8.16,
    waterFootprint: 3400,
    certifications: ['GOTS', 'Fair Trade'],
    materialPassport: 'ipfs://Qm.../passport.json'
  },

  rendering: {
    castsShadows: true,
    receivesShadows: true,
    doubleSided: false,
    transparent: false,
    alphaMode: 'opaque'
  }
};
```

### 3.2.3 소비자 경험 레이어

기술이 고객을 만나는 곳입니다:

```typescript
interface ConsumerExperience {
  // 가상 착용 시스템
  virtualTryOn: {
    modes: ('ar_camera' | 'avatar_3d' | 'photo_upload')[];
    platforms: ('web' | 'ios' | 'android' | 'vr')[];
    features: {
      realtimeRendering: boolean;
      bodyTracking: boolean;
      clothSimulation: boolean;
      multipleAngles: boolean;
      shareToSocial: boolean;
    };
  };

  // 사이즈 추천 엔진
  sizeRecommendation: {
    inputMethods: ('measurements' | 'body_scan' | 'past_purchases')[];
    accuracy: number;              // 92%
    confidence: number;            // 0-1
    fitPrediction: {
      chest: 'tight' | 'comfortable' | 'loose';
      waist: 'tight' | 'comfortable' | 'loose';
      hips: 'tight' | 'comfortable' | 'loose';
      length: 'short' | 'perfect' | 'long';
    };
  };

  // AI 스타일 어시스턴트
  styleAssistant: {
    personalization: {
      styleProfile: string[];      // 사용자가 선호하는 스타일
      colorPalette: string[];      // 좋아하는 색상
      priceRange: [number, number];
      sustainabilityMin: number;   // 최소 점수
    };
    recommendations: {
      algorithm: 'collaborative' | 'content_based' | 'hybrid';
      factors: ('similarity' | 'trending' | 'seasonal' | 'sustainable')[];
      diversity: boolean;          // 에코 챔버 방지
    };
    outfitComposition: {
      colorHarmony: boolean;
      styleCoherence: boolean;
      occasionMatch: boolean;
    };
  };

  // 가상 옷장
  virtualWardrobe: {
    features: ('3d_closet' | 'outfit_planning' | 'wear_tracking' | 'cost_per_wear')[];
    integration: ('calendar' | 'weather' | 'social_events')[];
    sustainability: {
      wardrobeCarbon: number;      // 총 탄소 발자국
      utilityScore: number;        // 아이템 활용도
      gapAnalysis: string[];       // 부족한 다용도 아이템
    };
  };
}
```

### 3.2.4 지속가능성 레이어

환경 및 사회적 영향 추적:

```typescript
interface SustainabilityLayer {
  // 전체 수명주기에 걸친 탄소 추적
  carbonTracking: {
    stages: {
      materialProduction: CarbonCalculation;
      manufacturing: CarbonCalculation;
      transportation: CarbonCalculation;
      usePhase: CarbonCalculation;
      endOfLife: CarbonCalculation;
    };
    total: number;                 // kg CO₂e
    perWear: number;              // 착용당 kg CO₂e
    benchmark: number;            // 업계 평균
  };

  // 소재 영향 평가
  materialImpact: {
    water: number;                // 리터
    energy: number;               // kWh
    chemicals: {
      toxic: boolean;
      list: string[];
      treatment: boolean;         // 폐수 처리 여부
    };
    biodegradable: boolean;
    recyclable: number;           // 0-1 (비율)
  };

  // 사회적 지표
  socialMetrics: {
    fairLabor: boolean;
    livingWage: boolean;
    safeConditions: boolean;
    certifications: string[];     // Fair Trade, B-Corp, SA8000
    auditScore: number;           // 0-100
    transparencyScore: number;    // 0-100
  };

  // 순환 경제 지표
  circularMetrics: {
    durability: number;           // 예상 사용 연수
    repairability: number;        // 0-100 점수
    recyclability: number;        // 0-100 점수
    takebackProgram: boolean;
    secondHandValue: number;      // 예상 재판매 가치
    designForDisassembly: boolean;
  };

  // 블록체인 검증
  blockchain: {
    enabled: boolean;
    contractAddress: string;
    tokenId: number;
    supplyChainStages: Array<{
      stage: string;
      location: string;
      timestamp: string;
      verified: boolean;
      carbonEmitted: number;
    }>;
  };
}

// 예제 지속가능성 계산
function calculateSustainability(garment: DigitalTwin): SustainabilityLayer {
  return {
    carbonTracking: {
      stages: {
        materialProduction: {
          value: 0.63,  // 유기농 면
          method: 'LCA_standard'
        },
        manufacturing: {
          value: 1.0,
          method: 'facility_data'
        },
        transportation: {
          value: 0.03,  // 해상 운송
          method: 'distance_based'
        },
        usePhase: {
          value: 7.5,   // 50회 세탁
          method: 'usage_model'
        },
        endOfLife: {
          value: -1.0,  // 기부
          method: 'circular_credit'
        }
      },
      total: 8.16,
      perWear: 0.109,  // 8.16 / 75회 착용
      benchmark: 13.5  // 업계 평균
    },

    materialImpact: {
      water: 3400,
      energy: 12,
      chemicals: {
        toxic: false,
        list: ['천연 염료', '친환경 유연제'],
        treatment: true
      },
      biodegradable: true,
      recyclable: 0.95
    },

    socialMetrics: {
      fairLabor: true,
      livingWage: true,
      safeConditions: true,
      certifications: ['Fair Trade', 'GOTS', 'SA8000'],
      auditScore: 92,
      transparencyScore: 88
    },

    circularMetrics: {
      durability: 5,        // 5년 예상
      repairability: 85,
      recyclability: 95,
      takebackProgram: true,
      secondHandValue: 35,  // 35달러 예상
      designForDisassembly: true
    },

    blockchain: {
      enabled: true,
      contractAddress: '0x742d35Cc6634C0532925a3b8',
      tokenId: 12345,
      supplyChainStages: [
        {
          stage: '면화 재배',
          location: '타밀나두, 인도',
          timestamp: '2025-01-15T10:00:00Z',
          verified: true,
          carbonEmitted: 0.21
        },
        // ... 더 많은 단계
      ]
    }
  };
}
```

---

## 3.3 AI 및 머신러닝 통합

### 3.3.1 트렌드 예측 시스템

```
트렌드 예측 파이프라인:
┌────────────────────────────────────────┐
│ 데이터 수집 (실시간)                    │
│ • Instagram API: 일 50만 게시물         │
│ • TikTok 트렌드: 비디오 분석            │
│ • Pinterest: 검색 쿼리                  │
│ • 소매 판매: POS 데이터                 │
│ • 런웨이 쇼: 이미지 인식                │
└──────────────┬─────────────────────────┘
               ↓
┌────────────────────────────────────────┐
│ 특징 추출                               │
│ • NLP: 텍스트 분석                      │
│ • 컴퓨터 비전: 이미지 특징              │
│ • 시계열: 판매 패턴                     │
│ • 감성: 긍정/부정                       │
└──────────────┬─────────────────────────┘
               ↓
┌────────────────────────────────────────┐
│ ML 모델                                 │
│ • LSTM: 시계열 예측                     │
│ • XGBoost: 분류                         │
│ • 신경망: 패턴 인식                     │
│ • 앙상블: 결합 예측                     │
└──────────────┬─────────────────────────┘
               ↓
┌────────────────────────────────────────┐
│ 트렌드 예측                             │
│ • 신뢰도: 0-1                           │
│ • 기간: 1-12개월                        │
│ • 카테고리: 색상, 스타일, 아이템        │
│ • 지역별: 지리적 특정 트렌드            │
└────────────────────────────────────────┘
```

**모델 사양:**

```typescript
interface TrendPredictionModel {
  // 시계열용 LSTM
  lstm: {
    architecture: '3개 레이어, 각 128 유닛';
    inputWindow: 90;        // 90일의 이력
    outputHorizon: 180;     // 180일 앞 예측
    features: string[];     // 판매, 언급, 참여
    accuracy: number;       // 78-85%
  };

  // 분류용 XGBoost
  xgboost: {
    trees: 500;
    maxDepth: 7;
    features: string[];     // 색상, 스타일, 가격, 시즌
    accuracy: number;       // 82-88%
  };

  // 앙상블 결합
  ensemble: {
    weights: {
      lstm: 0.40;
      xgboost: 0.35;
      expert: 0.25;
    };
    finalAccuracy: number;  // 85-90%
  };
}
```

### 3.3.2 사이즈 추천 모델

```typescript
interface SizeRecommendationModel {
  // 입력 특징
  inputFeatures: {
    userMeasurements: number[];     // [키, 가슴, 허리, 엉덩이, ...]
    garmentMeasurements: number[];  // 의류의 사이즈 차트
    fabricStretch: number;          // 신축성 계수
    brandSizingHistory: number[];   // 과거 사이즈 데이터
    userPurchaseHistory: any[];     // 사용자의 과거 구매
    similarUsersData: any[];        // 협업 필터링
  };

  // 모델 아키텍처
  model: {
    type: 'XGBoost';
    features: 15;           // 차원
    trees: 300;
    maxDepth: 6;
    training: {
      samples: 100_000_000; // 1억 구매 기록
      validation: 0.2;      // 20% 검증 분할
      crossValidation: 5;   // 5-폴드 CV
    };
  };

  // 성능 지표
  performance: {
    accuracyWithinOneSize: 0.92;  // 92% 정확 또는 인접
    exactMatch: 0.78;              // 78% 정확한 사이즈 일치
    returnReduction: 0.38;         // 38% 반품 감소
    confidenceCalibration: 0.95;   // 잘 보정됨
  };

  // 출력
  output: {
    recommendedSize: string;       // 'M'
    confidence: number;            // 0.88
    fitPrediction: {
      overall: 'true_to_size' | 'runs_small' | 'runs_large';
      details: {
        chest: string;
        waist: string;
        hips: string;
        length: string;
      };
    };
    alternativeSizes: Array<{
      size: string;
      confidence: number;
      note: string;
    }>;
  };
}
```

---

## 3.4 데이터 흐름 아키텍처

### 엔드투엔드 흐름 예제

```
디자인 → 디지털 트윈 → 소비자 → 분석
  ↓          ↓             ↓           ↓
┌────────────────────────────────────────────┐
│ 1. 디자이너가 CLO3D에서 드레스 제작         │
│    • 텍스처와 함께 FBX로 내보내기           │
└──────────────┬─────────────────────────────┘
               ↓
┌────────────────────────────────────────────┐
│ 2. WIA SDK가 디자인 처리                    │
│    • glTF로 변환(LOD0, LOD1, LOD2)         │
│    • 소재 속성 추출                         │
│    • 지속가능성 점수 계산                   │
│    • JSON 메타데이터 생성                   │
└──────────────┬─────────────────────────────┘
               ↓
┌────────────────────────────────────────────┐
│ 3. 디지털 트윈 저장                         │
│    • 3D 모델 → IPFS/CDN                    │
│    • 메타데이터 → 데이터베이스              │
│    • 블록체인 → 소재 여권                   │
└──────────────┬─────────────────────────────┘
               ↓
┌────────────────────────────────────────────┐
│ 4. 소비자 상호작용                          │
│    • 전자상거래 사이트에서 제품 조회        │
│    • 미리보기를 위한 중간 LOD 로드          │
│    • "가상 착용" 클릭                       │
└──────────────┬─────────────────────────────┘
               ↓
┌────────────────────────────────────────────┐
│ 5. 착용 경험                                │
│    • 신체 치수 입력 또는 스캔               │
│    • AI가 M 사이즈 추천(88% 신뢰도)         │
│    • 상세 보기를 위한 고해상도 LOD 로드     │
│    • 천 시뮬레이션이 현실적인 착용감 표시   │
└──────────────┬─────────────────────────────┘
               ↓
┌────────────────────────────────────────────┐
│ 6. 구매 결정                                │
│    • M 사이즈 구매                          │
│    • 완벽한 착용감 = 반품 없음!             │
└──────────────┬─────────────────────────────┘
               ↓
┌────────────────────────────────────────────┐
│ 7. 분석 피드백                              │
│    • 착용감 데이터가 모델 개선              │
│    • 지속가능성 영향 추적                   │
│    • 사용자가 추천 시스템에 추가            │
└────────────────────────────────────────────┘
```

---

## 복습 질문

1. **WIA-IND-001 아키텍처의 네 가지 주요 레이어를 말하시오.**
   <details>
   <summary>답변</summary>
   디자인 레이어, 디지털 트윈 레이어, 소비자 경험 레이어, 지속가능성 레이어.
   </details>

2. **일반적으로 사용되는 세 가지 LOD(Level of Detail) 폴리곤 수는 무엇인가?**
   <details>
   <summary>답변</summary>
   LOD0: 20K-50K 폴리곤(높음), LOD1: 5K-10K(중간), LOD2: 1K-3K(낮음).
   </details>

3. **한 사이즈 이내의 사이즈 추천 정확도는?**
   <details>
   <summary>답변</summary>
   한 사이즈 이내 92% 정확도(정확한 일치 78%).
   </details>

4. **사회적 지표에서 추적되는 세 가지 인증을 나열하시오.**
   <details>
   <summary>답변</summary>
   Fair Trade, GOTS(Global Organic Textile Standard), SA8000, B-Corp.
   </details>

5. **트렌드 예측을 위한 앙상블 가중치 분포는?**
   <details>
   <summary>답변</summary>
   LSTM: 40%, XGBoost: 35%, 전문가 입력: 25%.
   </details>

---

## 다음 단계

아키텍처를 확실히 이해했다면, [**4장: 데이터 형식**](04-data-format.md)에서 JSON 스키마 및 3D 에셋 사양을 배우는 기술적 세부 사항으로 들어가 보겠습니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
