# 6장: 프로토콜 사양

## 학습 목표

이 장을 마치면 다음을 이해할 수 있습니다:
- 지속가능성 계산 알고리즘 및 공식
- 가상 착용 프로토콜 및 천 시뮬레이션
- 트렌드 예측 방법론 및 모델
- 사이즈 추천 알고리즘
- 색상 조화 및 복장 구성 규칙

---

## 6.1 지속가능성 계산 프로토콜

### 6.1.1 탄소 발자국 계산

**소재 생산 탄소**:

```
Carbon_material = Σ (소재_무게 × 소재_탄소계수 × 비율)

여기서:
- 소재_무게: 의류의 원단 kg
- 소재_탄소계수: kg당 kg CO₂e (소재 데이터베이스에서)
- 비율: 혼방의 소재 % (0-100)
```

**TypeScript 구현**:

```typescript
function calculateMaterialCarbon(garment: Garment): number {
  return garment.materials.reduce((totalCarbon, material) => {
    // 소재 데이터베이스에서 탄소 계수 가져오기
    const carbonFactor = getMaterialCarbonFactor(material.type);

    // 이 소재의 무게 계산
    const materialWeight = (garment.totalWeight * material.percentage) / 100;

    // 탄소 기여도 추가
    return totalCarbon + (materialWeight * carbonFactor);
  }, 0);
}

// 소재 탄소 계수 (kg CO₂e per kg)
const CARBON_FACTORS: Record<string, number> = {
  'cotton': 5.9,
  'organic_cotton': 2.1,
  'polyester': 7.0,
  'recycled_polyester': 3.0,
  'nylon': 7.6,
  'wool': 10.5,
  'silk': 6.5,
  'linen': 2.0,
  'tencel': 2.5,
  'elastane': 9.0
};

// 예제
const dress = {
  totalWeight: 0.3,  // 300g
  materials: [
    { type: 'organic_cotton', percentage: 95 },
    { type: 'elastane', percentage: 5 }
  ]
};

const materialCarbon = calculateMaterialCarbon(dress);
// = (0.3 × 0.95 × 2.1) + (0.3 × 0.05 × 9.0)
// = 0.5985 + 0.135
// = 0.7335 kg CO₂e
```

**제조 탄소**:

```typescript
function calculateManufacturingCarbon(
  garment: Garment,
  factory: Factory
): number {
  // 기본 제조 탄소
  const baseCarbon = 1.0;  // kg CO₂e

  // 복잡도 승수
  const complexityFactors = {
    'tshirt': 1.0,
    'shirt': 1.2,
    'dress': 1.3,
    'pants': 1.4,
    'jacket': 1.8,
    'suit': 2.5
  };
  const complexity = complexityFactors[garment.type] || 1.0;

  // 에너지원 조정
  const energyFactors = {
    'renewable': 0.3,      // 70% 감소
    'grid': 1.0,           // 기준선
    'coal': 1.5            // 50% 증가
  };
  const energyFactor = energyFactors[factory.energySource] || 1.0;

  return baseCarbon * complexity * energyFactor;
}

// 예제
const manufacturingCarbon = calculateManufacturingCarbon(
  { type: 'dress' },
  { energySource: 'renewable' }
);
// = 1.0 × 1.3 × 0.3 = 0.39 kg CO₂e
```

**운송 탄소**:

```typescript
function calculateTransportCarbon(
  weight: number,          // kg
  distance: number,        // km
  method: TransportMethod
): number {
  // 운송 탄소 계수 (kg CO₂e per tonne-km)
  const transportFactors = {
    'sea': 0.01,
    'rail': 0.02,
    'truck': 0.06,
    'air': 0.50
  };

  const factor = transportFactors[method];

  // 톤으로 변환하고 계산
  const weightTonnes = weight / 1000;
  return distance * weightTonnes * factor;
}

// 예제: 해상으로 10,000 km 운송되는 0.3 kg 드레스
const transportCarbon = calculateTransportCarbon(0.3, 10000, 'sea');
// = 10,000 × (0.3/1000) × 0.01
// = 0.03 kg CO₂e
```

**사용 단계 탄소**:

```typescript
function calculateUsePhaseCarbon(
  washes: number,
  washingMethod: WashingMethod,
  dryingMethod: DryingMethod
): number {
  // 세탁 주기당 탄소 (kg CO₂e)
  const washingCarbon = {
    'cold_machine': 0.15,
    'warm_machine': 0.30,
    'hot_machine': 0.40,
    'hand_wash': 0.05
  };

  // 건조 주기당 탄소 (kg CO₂e)
  const dryingCarbon = {
    'line_dry': 0.0,
    'tumble_dry_low': 0.40,
    'tumble_dry_high': 0.60
  };

  const washCarbon = washingCarbon[washingMethod] || 0.15;
  const dryCarbon = dryingCarbon[dryingMethod] || 0.0;

  return washes * (washCarbon + dryCarbon);
}

// 예제: 냉수, 자연 건조로 75회 세탁
const usePhaseCarbon = calculateUsePhaseCarbon(75, 'cold_machine', 'line_dry');
// = 75 × (0.15 + 0.0) = 11.25 kg CO₂e
```

**수명 종료 탄소**:

```typescript
function calculateEndOfLifeCarbon(
  weight: number,          // kg
  disposal: DisposalMethod,
  recyclability: number    // 0-1
): number {
  const disposalCarbon = {
    'landfill': 0.5,       // 메탄 배출
    'incineration': 1.0,
    'recycled': -2.0,      // 탄소 크레딧
    'donated': -1.5,       // 수명 연장 크레딧
    'composted': -0.2      // 생분해성 소재용
  };

  const baseCarbon = disposalCarbon[disposal] || 0.5;

  // 실제 재활용 가능성으로 재활용 크레딧 조정
  if (disposal === 'recycled') {
    return baseCarbon * recyclability * weight;
  }

  return baseCarbon * weight;
}

// 예제: 기부된 0.3 kg
const endOfLifeCarbon = calculateEndOfLifeCarbon(0.3, 'donated', 0.95);
// = -1.5 × 0.3 = -0.45 kg CO₂e (크레딧)
```

**총 탄소 발자국**:

```typescript
interface CarbonFootprint {
  material: number;
  manufacturing: number;
  transport: number;
  use: number;
  endOfLife: number;
  total: number;
  perWear: number;
}

function calculateTotalCarbonFootprint(
  garment: Garment,
  lifecycle: LifecycleData
): CarbonFootprint {
  const material = calculateMaterialCarbon(garment);
  const manufacturing = calculateManufacturingCarbon(
    garment,
    lifecycle.factory
  );
  const transport = calculateTransportCarbon(
    garment.totalWeight,
    lifecycle.transportDistance,
    lifecycle.transportMethod
  );
  const use = calculateUsePhaseCarbon(
    lifecycle.expectedWashes,
    lifecycle.washingMethod,
    lifecycle.dryingMethod
  );
  const endOfLife = calculateEndOfLifeCarbon(
    garment.totalWeight,
    lifecycle.disposalMethod,
    garment.recyclability
  );

  const total = material + manufacturing + transport + use + endOfLife;
  const perWear = total / lifecycle.expectedWears;

  return {
    material,
    manufacturing,
    transport,
    use,
    endOfLife,
    total,
    perWear
  };
}
```

### 6.1.2 지속가능성 점수 매기기

**환경 점수 (0-100)**:

```typescript
function calculateEnvironmentalScore(
  carbonFootprint: CarbonFootprint,
  waterFootprint: WaterFootprint,
  garment: Garment
): number {
  // 최대 참조 값 (최악의 경우)
  const MAX_CARBON = 17;      // kg CO₂e/kg (가죽)
  const MAX_WATER = 125000;   // L/kg (울)

  // 탄소 영향 (0-100, 낮을수록 좋음)
  const carbonPerKg = carbonFootprint.total / garment.totalWeight;
  const carbonImpact = (carbonPerKg / MAX_CARBON) * 100;

  // 물 영향 (0-100, 낮을수록 좋음)
  const waterPerKg = waterFootprint.total / garment.totalWeight;
  const waterImpact = (waterPerKg / MAX_WATER) * 100;

  // 화학물질 영향 (소재 데이터베이스에서)
  const chemicalImpact = garment.materials.reduce((impact, mat) => {
    const matChemicalScore = getMaterialChemicalScore(mat.type);
    return impact + (matChemicalScore * mat.percentage / 100);
  }, 0);

  // 폐기물 영향
  const wasteImpact = (1 - garment.recyclability) * 100;

  // 가중치 점수 (역전, 따라서 100이 최고)
  const score = 100 - (
    carbonImpact * 0.35 +
    waterImpact * 0.25 +
    chemicalImpact * 0.20 +
    wasteImpact * 0.20
  );

  return Math.max(0, Math.min(100, score));
}
```

**사회 점수 (0-100)**:

```typescript
function calculateSocialScore(supplyChain: SupplyChainData): number {
  let score = 0;

  // 공정 노동 관행 (40%)
  if (supplyChain.fairLaborCertified) score += 40;
  else if (supplyChain.laborAuditScore) {
    score += (supplyChain.laborAuditScore / 100) * 40;
  }

  // 안전한 근무 조건 (30%)
  if (supplyChain.safetyAuditScore) {
    score += (supplyChain.safetyAuditScore / 100) * 30;
  }

  // 생활 임금 (20%)
  if (supplyChain.livingWageVerified) score += 20;

  // 지역사회 영향 (10%)
  if (supplyChain.communityPrograms) score += 10;

  // 인증 보너스
  const certificationBonus = {
    'Fair Trade': 15,
    'B-Corp': 12,
    'SA8000': 10,
    'GOTS': 10
  };

  supplyChain.certifications?.forEach(cert => {
    if (certificationBonus[cert]) {
      score += certificationBonus[cert];
    }
  });

  return Math.min(100, score);
}
```

**순환 점수 (0-100)**:

```typescript
function calculateCircularScore(garment: Garment): number {
  // 재활용 가능성 (35%)
  const recyclabilityScore = garment.recyclability * 35;

  // 내구성 (30%)
  const durabilityTable = {
    '<1': 20,
    '1-3': 50,
    '3-5': 70,
    '5-10': 85,
    '>10': 95
  };
  const durabilityYears = garment.expectedLifespan;
  let durabilityScore = 50;  // 기본값
  if (durabilityYears < 1) durabilityScore = 20;
  else if (durabilityYears <= 3) durabilityScore = 50;
  else if (durabilityYears <= 5) durabilityScore = 70;
  else if (durabilityYears <= 10) durabilityScore = 85;
  else durabilityScore = 95;
  durabilityScore = (durabilityScore / 100) * 30;

  // 수리 가능성 (20%)
  const repairabilityScore = (garment.repairability / 100) * 20;

  // 생분해성 (15%)
  const biodegradabilityScore = garment.biodegradable ? 15 : 0;

  return recyclabilityScore + durabilityScore +
         repairabilityScore + biodegradabilityScore;
}
```

**총 지속가능성 점수**:

```typescript
function calculateTotalSustainability(
  environmental: number,
  social: number,
  circular: number
): { score: number; rating: string } {
  const total = (
    environmental * 0.40 +
    social * 0.30 +
    circular * 0.30
  );

  const rating =
    total >= 90 ? 'A+' :
    total >= 80 ? 'A' :
    total >= 70 ? 'B' :
    total >= 60 ? 'C' :
    total >= 50 ? 'D' : 'F';

  return { score: total, rating };
}
```

---

## 6.2 가상 착용 프로토콜

### 6.2.1 천 물리 시뮬레이션

**질량-스프링 시스템**:

```typescript
interface ClothParticle {
  position: Vector3;
  velocity: Vector3;
  mass: number;
  pinned: boolean;  // 입자가 고정되었나?
}

interface Spring {
  particleA: number;  // 인덱스
  particleB: number;
  restLength: number;
  stiffness: number;  // k
  damping: number;    // c
}

function simulateCloth(
  particles: ClothParticle[],
  springs: Spring[],
  deltaTime: number
): void {
  // 힘 적용
  particles.forEach((particle, i) => {
    if (particle.pinned) return;

    // 중력
    const gravity = new Vector3(0, -9.81 * particle.mass, 0);
    let force = gravity;

    // 스프링 힘
    springs.forEach(spring => {
      if (spring.particleA === i || spring.particleB === i) {
        const other = spring.particleA === i ?
          particles[spring.particleB] :
          particles[spring.particleA];

        const delta = other.position.sub(particle.position);
        const distance = delta.length();
        const stretch = distance - spring.restLength;

        // 후크의 법칙: F = -k(L - L0)
        const springForce = delta.normalize()
          .scale(spring.stiffness * stretch);

        // 감쇠: F = -c*v
        const relativeVelocity = particle.velocity.sub(other.velocity);
        const dampingForce = relativeVelocity
          .scale(-spring.damping);

        force = force.add(springForce).add(dampingForce);
      }
    });

    // 공기 저항
    const airResistance = particle.velocity.scale(-0.1);
    force = force.add(airResistance);

    // 속도 및 위치 업데이트 (Verlet 적분)
    const acceleration = force.scale(1 / particle.mass);
    particle.velocity = particle.velocity.add(
      acceleration.scale(deltaTime)
    );
    particle.position = particle.position.add(
      particle.velocity.scale(deltaTime)
    );
  });

  // 충돌 감지 및 응답
  particles.forEach(particle => {
    if (particle.pinned) return;

    // 지면 충돌
    if (particle.position.y < 0) {
      particle.position.y = 0;
      particle.velocity.y *= -0.3;  // 감쇠가 있는 반동
    }

    // 신체 충돌 (단순화)
    const bodyDistance = checkBodyCollision(particle.position);
    if (bodyDistance < 0) {
      // 입자를 신체에서 밀어냄
      const normal = getBodyNormal(particle.position);
      particle.position = particle.position.add(
        normal.scale(-bodyDistance)
      );
      // 신체 방향의 속도 성분 제거
      const velocityNormal = particle.velocity.dot(normal);
      if (velocityNormal < 0) {
        particle.velocity = particle.velocity.sub(
          normal.scale(velocityNormal)
        );
      }
    }
  });
}
```

**원단 강성 매개변수**:

```typescript
const FABRIC_PHYSICS: Record<string, PhysicsProperties> = {
  cotton: {
    stiffness: 150,        // N/m
    damping: 0.15,
    density: 1.54,         // g/cm³
    stretch: [1.05, 1.03], // [날실, 씨실]
    friction: 0.4
  },
  silk: {
    stiffness: 75,
    damping: 0.10,
    density: 1.3,
    stretch: [1.08, 1.06],
    friction: 0.2
  },
  denim: {
    stiffness: 400,
    damping: 0.25,
    density: 1.5,
    stretch: [1.15, 1.08],  // 신축성 데님
    friction: 0.6
  },
  leather: {
    stiffness: 800,
    damping: 0.35,
    density: 1.0,
    stretch: [1.02, 1.02],
    friction: 0.5
  },
  knit: {
    stiffness: 60,
    damping: 0.12,
    density: 1.2,
    stretch: [1.5, 1.4],   // 높은 신축성
    friction: 0.3
  }
};
```

### 6.2.2 착용감 분석 알고리즘

```typescript
function analyzeFit(
  garmentMesh: Mesh,
  bodyMesh: Mesh,
  bodyMeasurements: BodyMeasurements
): FitAnalysis {
  // 의류와 신체 사이의 거리 측정
  const distances = measureGarmentBodyDistances(garmentMesh, bodyMesh);

  // 주요 부위 분석
  const chestFit = analyzeFitRegion(distances.chest, 'chest');
  const waistFit = analyzeFitRegion(distances.waist, 'waist');
  const hipsFit = analyzeFitRegion(distances.hips, 'hips');

  // 전체 착용감 평가
  const overall = determineOverallFit([chestFit, waistFit, hipsFit]);

  // 신뢰도 계산
  const confidence = calculateFitConfidence(distances);

  return {
    overall,
    details: {
      chest: chestFit,
      waist: waistFit,
      hips: hipsFit
    },
    confidence,
    measurements: distances
  };
}

function analyzeFitRegion(
  distance: number,  // cm
  region: string
): FitLevel {
  // 이상적인 착용감 범위 (신체와 의류 사이의 공간 cm)
  const fitRanges = {
    chest: {
      very_tight: [-Infinity, 1],
      tight: [1, 3],
      comfortable: [3, 8],
      loose: [8, 15],
      very_loose: [15, Infinity]
    },
    waist: {
      very_tight: [-Infinity, 0.5],
      tight: [0.5, 2],
      comfortable: [2, 6],
      loose: [6, 12],
      very_loose: [12, Infinity]
    },
    hips: {
      very_tight: [-Infinity, 1],
      tight: [1, 3],
      comfortable: [3, 8],
      loose: [8, 15],
      very_loose: [15, Infinity]
    }
  };

  const ranges = fitRanges[region];

  for (const [fit, [min, max]] of Object.entries(ranges)) {
    if (distance >= min && distance < max) {
      return fit as FitLevel;
    }
  }

  return 'comfortable';  // 폴백
}
```

---

## 6.3 사이즈 추천 프로토콜

### 6.3.1 사이즈 예측 알고리즘

```typescript
class SizeRecommendationModel {
  private model: XGBoostModel;

  constructor() {
    this.model = loadPretrainedModel('size-recommendation-v2.model');
  }

  recommend(
    userMeasurements: BodyMeasurements,
    garmentData: Garment,
    userHistory?: PurchaseHistory[]
  ): SizeRecommendation {
    // 특징 추출
    const features = this.extractFeatures(
      userMeasurements,
      garmentData,
      userHistory
    );

    // 모델 예측
    const prediction = this.model.predict(features);

    // 사이즈 추천으로 변환
    return this.interpretPrediction(prediction, garmentData.sizes);
  }

  private extractFeatures(
    user: BodyMeasurements,
    garment: Garment,
    history?: PurchaseHistory[]
  ): number[] {
    const features: number[] = [];

    // 사용자 측정치 (정규화)
    features.push(user.height / 200);         // 0-1 범위
    features.push(user.chest / 150);
    features.push(user.waist / 150);
    features.push(user.hips / 150);

    // 각 사용 가능한 사이즈의 의류 사이즈 측정치
    garment.sizes.available.forEach(size => {
      const sizeMeasurements = garment.sizes.measurements[size];
      features.push(sizeMeasurements.chest / 150);
      features.push(sizeMeasurements.waist / 150);
      features.push(sizeMeasurements.hips / 150);
    });

    // 원단 신축 계수
    const avgStretch = garment.materials.reduce((sum, mat) => {
      return sum + (getMaterialStretch(mat.type) * mat.percentage / 100);
    }, 0);
    features.push(avgStretch);

    // 사용자 구매 이력 (사용 가능한 경우)
    if (history && history.length > 0) {
      const recentPurchases = history.slice(0, 5);
      recentPurchases.forEach(purchase => {
        features.push(purchase.fitScore);  // -1에서 1
      });
      // 5개 미만인 경우 패딩
      while (features.length < 20) {
        features.push(0);
      }
    }

    return features;
  }

  private interpretPrediction(
    prediction: ModelOutput,
    availableSizes: string[]
  ): SizeRecommendation {
    // prediction.probabilities: [XS, S, M, L, XL]에 대한 [0.05, 0.12, 0.68, 0.13, 0.02]
    const sizeIndex = prediction.probabilities
      .indexOf(Math.max(...prediction.probabilities));

    const recommendedSize = availableSizes[sizeIndex];
    const confidence = prediction.probabilities[sizeIndex];

    // 대안 사이즈 (두 번째로 높은 확률)
    const alternatives: AlternativeSize[] = [];
    const sortedProbs = [...prediction.probabilities]
      .map((prob, idx) => ({ prob, size: availableSizes[idx] }))
      .sort((a, b) => b.prob - a.prob)
      .slice(1, 3);  // 상위 2개 대안

    sortedProbs.forEach(alt => {
      if (alt.prob > 0.1) {  // >10% 확률인 경우만
        alternatives.push({
          size: alt.size,
          confidence: alt.prob,
          note: this.getAlternativeNote(alt.size, recommendedSize)
        });
      }
    });

    return {
      size: recommendedSize,
      confidence,
      alternatives,
      reasoning: this.generateReasoning(prediction, recommendedSize)
    };
  }
}
```

---

## 6.4 트렌드 예측 프로토콜

### 6.4.1 LSTM 시계열 모델

```typescript
class TrendPredictionLSTM {
  private model: LSTMNetwork;

  constructor() {
    this.model = new LSTMNetwork({
      layers: [
        { type: 'lstm', units: 128, returnSequences: true },
        { type: 'dropout', rate: 0.2 },
        { type: 'lstm', units: 128, returnSequences: true },
        { type: 'dropout', rate: 0.2 },
        { type: 'lstm', units: 64, returnSequences: false },
        { type: 'dense', units: 32, activation: 'relu' },
        { type: 'dense', units: 1, activation: 'sigmoid' }
      ]
    });
  }

  async predict(
    historicalData: TrendData[],  // 90일
    horizonDays: number           // N일 앞 예측
  ): Promise<TrendPrediction> {
    // 입력 시퀀스 준비
    const features = this.extractTrendFeatures(historicalData);

    // 정규화
    const normalized = this.normalize(features);

    // 예측
    const predictions = await this.model.predict(normalized);

    // 비정규화 및 해석
    return this.interpretPredictions(predictions, horizonDays);
  }

  private extractTrendFeatures(data: TrendData[]): number[][] {
    return data.map(day => [
      day.socialMediaMentions / 1000000,    // 정규화
      day.searchVolume / 100000,
      day.salesVelocity / 10000,
      day.runwayAppearances / 100,
      day.influencerPosts / 1000,
      day.sentimentScore,                    // 0-1
      day.engagementRate                     // 0-1
    ]);
  }
}
```

### 6.4.2 신뢰도 점수 매기기

```typescript
function calculateTrendConfidence(
  prediction: TrendPrediction,
  dataQuality: DataQuality
): number {
  // 데이터 볼륨 점수 (0-1)
  const volumeScore = Math.min(1, dataQuality.totalDataPoints / 100000);

  // 소스 다양성 (0-1)
  const diversityScore = dataQuality.uniqueSources / 10;

  // 역사적 정확도 (0-1)
  const accuracyScore = prediction.modelAccuracy;

  // 전문가 검증 (0-1)
  const expertScore = prediction.expertValidated ? 1 : 0.5;

  // 가중 조합
  const confidence = (
    volumeScore * 0.25 +
    diversityScore * 0.25 +
    accuracyScore * 0.30 +
    expertScore * 0.20
  );

  return confidence;
}
```

---

## 복습 질문

1. **유기농 면과 일반 면의 탄소 계수는 무엇인가?**
   <details>
   <summary>답변</summary>
   유기농 면: 2.1 kg CO₂e/kg, 일반 면: 5.9 kg CO₂e/kg (64% 감소).
   </details>

2. **총 지속가능성 점수 계산 가중치는?**
   <details>
   <summary>답변</summary>
   환경: 40%, 사회: 30%, 순환: 30%.
   </details>

3. **편안한 착용감을 위한 이상적인 가슴 착용감 범위는?**
   <details>
   <summary>답변</summary>
   신체와 의류 사이 3-8 cm 공간.
   </details>

4. **트렌드 예측 모델은 몇 개의 LSTM 레이어를 사용하는가?**
   <details>
   <summary>답변</summary>
   드롭아웃 레이어가 있는 3개의 LSTM 레이어 (128, 128, 64 유닛).
   </details>

5. **트렌드 예측 신뢰도를 결정하는 요인은?**
   <details>
   <summary>답변</summary>
   데이터 볼륨 (25%), 소스 다양성 (25%), 역사적 정확도 (30%), 전문가 검증 (20%).
   </details>

---

## 다음 단계

[**7장: 시스템 통합**](07-system-integration.md)에서 기존 시스템과 이러한 프로토콜을 통합하는 방법을 배우겠습니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
