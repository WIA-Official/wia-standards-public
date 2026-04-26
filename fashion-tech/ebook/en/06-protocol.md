# Chapter 6: Protocol Specifications

## Learning Objectives

By the end of this chapter, you will understand:
- Sustainability calculation algorithms and formulas
- Virtual fitting protocol and cloth simulation
- Trend prediction methodology and models
- Size recommendation algorithms
- Color harmony and outfit composition rules

---

## 6.1 Sustainability Calculation Protocol

### 6.1.1 Carbon Footprint Calculation

**Material Production Carbon**:

```
Carbon_material = Σ (Material_weight × Material_carbon_factor × Percentage)

Where:
- Material_weight: kg of fabric in garment
- Material_carbon_factor: kg CO₂e per kg (from material database)
- Percentage: % of material in blend (0-100)
```

**TypeScript Implementation**:

```typescript
function calculateMaterialCarbon(garment: Garment): number {
  return garment.materials.reduce((totalCarbon, material) => {
    // Get carbon factor from material database
    const carbonFactor = getMaterialCarbonFactor(material.type);

    // Calculate weight of this material
    const materialWeight = (garment.totalWeight * material.percentage) / 100;

    // Add carbon contribution
    return totalCarbon + (materialWeight * carbonFactor);
  }, 0);
}

// Material carbon factors (kg CO₂e per kg)
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

// Example
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

**Manufacturing Carbon**:

```typescript
function calculateManufacturingCarbon(
  garment: Garment,
  factory: Factory
): number {
  // Base manufacturing carbon
  const baseCarbon = 1.0;  // kg CO₂e

  // Complexity multiplier
  const complexityFactors = {
    'tshirt': 1.0,
    'shirt': 1.2,
    'dress': 1.3,
    'pants': 1.4,
    'jacket': 1.8,
    'suit': 2.5
  };
  const complexity = complexityFactors[garment.type] || 1.0;

  // Energy source adjustment
  const energyFactors = {
    'renewable': 0.3,      // 70% reduction
    'grid': 1.0,           // Baseline
    'coal': 1.5            // 50% increase
  };
  const energyFactor = energyFactors[factory.energySource] || 1.0;

  return baseCarbon * complexity * energyFactor;
}

// Example
const manufacturingCarbon = calculateManufacturingCarbon(
  { type: 'dress' },
  { energySource: 'renewable' }
);
// = 1.0 × 1.3 × 0.3 = 0.39 kg CO₂e
```

**Transportation Carbon**:

```typescript
function calculateTransportCarbon(
  weight: number,          // kg
  distance: number,        // km
  method: TransportMethod
): number {
  // Transport carbon factors (kg CO₂e per tonne-km)
  const transportFactors = {
    'sea': 0.01,
    'rail': 0.02,
    'truck': 0.06,
    'air': 0.50
  };

  const factor = transportFactors[method];

  // Convert to tonnes and calculate
  const weightTonnes = weight / 1000;
  return distance * weightTonnes * factor;
}

// Example: 0.3 kg dress shipped 10,000 km by sea
const transportCarbon = calculateTransportCarbon(0.3, 10000, 'sea');
// = 10,000 × (0.3/1000) × 0.01
// = 0.03 kg CO₂e
```

**Use Phase Carbon**:

```typescript
function calculateUsePhaseCarbon(
  washes: number,
  washingMethod: WashingMethod,
  dryingMethod: DryingMethod
): number {
  // Carbon per wash cycle (kg CO₂e)
  const washingCarbon = {
    'cold_machine': 0.15,
    'warm_machine': 0.30,
    'hot_machine': 0.40,
    'hand_wash': 0.05
  };

  // Carbon per drying cycle (kg CO₂e)
  const dryingCarbon = {
    'line_dry': 0.0,
    'tumble_dry_low': 0.40,
    'tumble_dry_high': 0.60
  };

  const washCarbon = washingCarbon[washingMethod] || 0.15;
  const dryCarbon = dryingCarbon[dryingMethod] || 0.0;

  return washes * (washCarbon + dryCarbon);
}

// Example: 75 washes, cold water, line dry
const usePhaseCarbon = calculateUsePhaseCarbon(75, 'cold_machine', 'line_dry');
// = 75 × (0.15 + 0.0) = 11.25 kg CO₂e
```

**End-of-Life Carbon**:

```typescript
function calculateEndOfLifeCarbon(
  weight: number,          // kg
  disposal: DisposalMethod,
  recyclability: number    // 0-1
): number {
  const disposalCarbon = {
    'landfill': 0.5,       // Methane emissions
    'incineration': 1.0,
    'recycled': -2.0,      // Carbon credit
    'donated': -1.5,       // Extended life credit
    'composted': -0.2      // For biodegradable materials
  };

  const baseCarbon = disposalCarbon[disposal] || 0.5;

  // Adjust recycling credit by actual recyclability
  if (disposal === 'recycled') {
    return baseCarbon * recyclability * weight;
  }

  return baseCarbon * weight;
}

// Example: 0.3 kg donated
const endOfLifeCarbon = calculateEndOfLifeCarbon(0.3, 'donated', 0.95);
// = -1.5 × 0.3 = -0.45 kg CO₂e (credit)
```

**Total Carbon Footprint**:

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

### 6.1.2 Sustainability Scoring

**Environmental Score (0-100)**:

```typescript
function calculateEnvironmentalScore(
  carbonFootprint: CarbonFootprint,
  waterFootprint: WaterFootprint,
  garment: Garment
): number {
  // Maximum reference values (worst case)
  const MAX_CARBON = 17;      // kg CO₂e/kg (leather)
  const MAX_WATER = 125000;   // L/kg (wool)

  // Carbon impact (0-100, lower is better)
  const carbonPerKg = carbonFootprint.total / garment.totalWeight;
  const carbonImpact = (carbonPerKg / MAX_CARBON) * 100;

  // Water impact (0-100, lower is better)
  const waterPerKg = waterFootprint.total / garment.totalWeight;
  const waterImpact = (waterPerKg / MAX_WATER) * 100;

  // Chemical impact (from material database)
  const chemicalImpact = garment.materials.reduce((impact, mat) => {
    const matChemicalScore = getMaterialChemicalScore(mat.type);
    return impact + (matChemicalScore * mat.percentage / 100);
  }, 0);

  // Waste impact
  const wasteImpact = (1 - garment.recyclability) * 100;

  // Weighted score (inverted, so 100 is best)
  const score = 100 - (
    carbonImpact * 0.35 +
    waterImpact * 0.25 +
    chemicalImpact * 0.20 +
    wasteImpact * 0.20
  );

  return Math.max(0, Math.min(100, score));
}
```

**Social Score (0-100)**:

```typescript
function calculateSocialScore(supplyChain: SupplyChainData): number {
  let score = 0;

  // Fair labor practices (40%)
  if (supplyChain.fairLaborCertified) score += 40;
  else if (supplyChain.laborAuditScore) {
    score += (supplyChain.laborAuditScore / 100) * 40;
  }

  // Safe working conditions (30%)
  if (supplyChain.safetyAuditScore) {
    score += (supplyChain.safetyAuditScore / 100) * 30;
  }

  // Living wage (20%)
  if (supplyChain.livingWageVerified) score += 20;

  // Community impact (10%)
  if (supplyChain.communityPrograms) score += 10;

  // Certification bonuses
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

**Circular Score (0-100)**:

```typescript
function calculateCircularScore(garment: Garment): number {
  // Recyclability (35%)
  const recyclabilityScore = garment.recyclability * 35;

  // Durability (30%)
  const durabilityTable = {
    '<1': 20,
    '1-3': 50,
    '3-5': 70,
    '5-10': 85,
    '>10': 95
  };
  const durabilityYears = garment.expectedLifespan;
  let durabilityScore = 50;  // default
  if (durabilityYears < 1) durabilityScore = 20;
  else if (durabilityYears <= 3) durabilityScore = 50;
  else if (durabilityYears <= 5) durabilityScore = 70;
  else if (durabilityYears <= 10) durabilityScore = 85;
  else durabilityScore = 95;
  durabilityScore = (durabilityScore / 100) * 30;

  // Repairability (20%)
  const repairabilityScore = (garment.repairability / 100) * 20;

  // Biodegradability (15%)
  const biodegradabilityScore = garment.biodegradable ? 15 : 0;

  return recyclabilityScore + durabilityScore +
         repairabilityScore + biodegradabilityScore;
}
```

**Total Sustainability Score**:

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

## 6.2 Virtual Fitting Protocol

### 6.2.1 Cloth Physics Simulation

**Mass-Spring System**:

```typescript
interface ClothParticle {
  position: Vector3;
  velocity: Vector3;
  mass: number;
  pinned: boolean;  // Is particle fixed?
}

interface Spring {
  particleA: number;  // Index
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
  // Apply forces
  particles.forEach((particle, i) => {
    if (particle.pinned) return;

    // Gravity
    const gravity = new Vector3(0, -9.81 * particle.mass, 0);
    let force = gravity;

    // Spring forces
    springs.forEach(spring => {
      if (spring.particleA === i || spring.particleB === i) {
        const other = spring.particleA === i ?
          particles[spring.particleB] :
          particles[spring.particleA];

        const delta = other.position.sub(particle.position);
        const distance = delta.length();
        const stretch = distance - spring.restLength;

        // Hooke's law: F = -k(L - L0)
        const springForce = delta.normalize()
          .scale(spring.stiffness * stretch);

        // Damping: F = -c*v
        const relativeVelocity = particle.velocity.sub(other.velocity);
        const dampingForce = relativeVelocity
          .scale(-spring.damping);

        force = force.add(springForce).add(dampingForce);
      }
    });

    // Air resistance
    const airResistance = particle.velocity.scale(-0.1);
    force = force.add(airResistance);

    // Update velocity and position (Verlet integration)
    const acceleration = force.scale(1 / particle.mass);
    particle.velocity = particle.velocity.add(
      acceleration.scale(deltaTime)
    );
    particle.position = particle.position.add(
      particle.velocity.scale(deltaTime)
    );
  });

  // Collision detection and response
  particles.forEach(particle => {
    if (particle.pinned) return;

    // Ground collision
    if (particle.position.y < 0) {
      particle.position.y = 0;
      particle.velocity.y *= -0.3;  // Bounce with damping
    }

    // Body collision (simplified)
    const bodyDistance = checkBodyCollision(particle.position);
    if (bodyDistance < 0) {
      // Push particle away from body
      const normal = getBodyNormal(particle.position);
      particle.position = particle.position.add(
        normal.scale(-bodyDistance)
      );
      // Remove velocity component toward body
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

**Fabric Stiffness Parameters**:

```typescript
const FABRIC_PHYSICS: Record<string, PhysicsProperties> = {
  cotton: {
    stiffness: 150,        // N/m
    damping: 0.15,
    density: 1.54,         // g/cm³
    stretch: [1.05, 1.03], // [warp, weft]
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
    stretch: [1.15, 1.08],  // Stretch denim
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
    stretch: [1.5, 1.4],   // High stretch
    friction: 0.3
  }
};
```

### 6.2.2 Fit Analysis Algorithm

```typescript
function analyzeFit(
  garmentMesh: Mesh,
  bodyMesh: Mesh,
  bodyMeasurements: BodyMeasurements
): FitAnalysis {
  // Measure distances between garment and body
  const distances = measureGarmentBodyDistances(garmentMesh, bodyMesh);

  // Analyze key regions
  const chestFit = analyzeFitRegion(distances.chest, 'chest');
  const waistFit = analyzeFitRegion(distances.waist, 'waist');
  const hipsFit = analyzeFitRegion(distances.hips, 'hips');

  // Overall fit assessment
  const overall = determineOverallFit([chestFit, waistFit, hipsFit]);

  // Calculate confidence
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
  // Ideal fit ranges (cm of space between body and garment)
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

  return 'comfortable';  // fallback
}
```

---

## 6.3 Size Recommendation Protocol

### 6.3.1 Size Prediction Algorithm

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
    // Feature extraction
    const features = this.extractFeatures(
      userMeasurements,
      garmentData,
      userHistory
    );

    // Model prediction
    const prediction = this.model.predict(features);

    // Convert to size recommendation
    return this.interpretPrediction(prediction, garmentData.sizes);
  }

  private extractFeatures(
    user: BodyMeasurements,
    garment: Garment,
    history?: PurchaseHistory[]
  ): number[] {
    const features: number[] = [];

    // User measurements (normalized)
    features.push(user.height / 200);         // 0-1 range
    features.push(user.chest / 150);
    features.push(user.waist / 150);
    features.push(user.hips / 150);

    // Garment size measurements for each available size
    garment.sizes.available.forEach(size => {
      const sizeMeasurements = garment.sizes.measurements[size];
      features.push(sizeMeasurements.chest / 150);
      features.push(sizeMeasurements.waist / 150);
      features.push(sizeMeasurements.hips / 150);
    });

    // Fabric stretch factor
    const avgStretch = garment.materials.reduce((sum, mat) => {
      return sum + (getMaterialStretch(mat.type) * mat.percentage / 100);
    }, 0);
    features.push(avgStretch);

    // User purchase history (if available)
    if (history && history.length > 0) {
      const recentPurchases = history.slice(0, 5);
      recentPurchases.forEach(purchase => {
        features.push(purchase.fitScore);  // -1 to 1
      });
      // Pad if less than 5
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
    // prediction.probabilities: [0.05, 0.12, 0.68, 0.13, 0.02] for [XS, S, M, L, XL]
    const sizeIndex = prediction.probabilities
      .indexOf(Math.max(...prediction.probabilities));

    const recommendedSize = availableSizes[sizeIndex];
    const confidence = prediction.probabilities[sizeIndex];

    // Alternative sizes (second highest probability)
    const alternatives: AlternativeSize[] = [];
    const sortedProbs = [...prediction.probabilities]
      .map((prob, idx) => ({ prob, size: availableSizes[idx] }))
      .sort((a, b) => b.prob - a.prob)
      .slice(1, 3);  // Top 2 alternatives

    sortedProbs.forEach(alt => {
      if (alt.prob > 0.1) {  // Only if >10% probability
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

## 6.4 Trend Prediction Protocol

### 6.4.1 LSTM Time Series Model

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
    historicalData: TrendData[],  // 90 days
    horizonDays: number           // Predict N days ahead
  ): Promise<TrendPrediction> {
    // Prepare input sequence
    const features = this.extractTrendFeatures(historicalData);

    // Normalize
    const normalized = this.normalize(features);

    // Predict
    const predictions = await this.model.predict(normalized);

    // Denormalize and interpret
    return this.interpretPredictions(predictions, horizonDays);
  }

  private extractTrendFeatures(data: TrendData[]): number[][] {
    return data.map(day => [
      day.socialMediaMentions / 1000000,    // Normalized
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

### 6.4.2 Confidence Scoring

```typescript
function calculateTrendConfidence(
  prediction: TrendPrediction,
  dataQuality: DataQuality
): number {
  // Data volume score (0-1)
  const volumeScore = Math.min(1, dataQuality.totalDataPoints / 100000);

  // Source diversity (0-1)
  const diversityScore = dataQuality.uniqueSources / 10;

  // Historical accuracy (0-1)
  const accuracyScore = prediction.modelAccuracy;

  // Expert validation (0-1)
  const expertScore = prediction.expertValidated ? 1 : 0.5;

  // Weighted combination
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

## Review Questions

1. **What is the carbon factor for organic cotton vs conventional cotton?**
   <details>
   <summary>Answer</summary>
   Organic cotton: 2.1 kg CO₂e/kg, Conventional cotton: 5.9 kg CO₂e/kg (64% reduction).
   </details>

2. **What are the weights for calculating total sustainability score?**
   <details>
   <summary>Answer</summary>
   Environmental: 40%, Social: 30%, Circular: 30%.
   </details>

3. **What is the ideal chest fit range for comfortable fit?**
   <details>
   <summary>Answer</summary>
   3-8 cm of space between body and garment.
   </details>

4. **How many LSTM layers does the trend prediction model use?**
   <details>
   <summary>Answer</summary>
   3 LSTM layers (128, 128, 64 units) with dropout layers.
   </details>

5. **What factors determine trend prediction confidence?**
   <details>
   <summary>Answer</summary>
   Data volume (25%), source diversity (25%), historical accuracy (30%), and expert validation (20%).
   </details>

---

## Next Steps

Learn how to integrate these protocols with existing systems in [**Chapter 7: System Integration**](07-system-integration.md).

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
