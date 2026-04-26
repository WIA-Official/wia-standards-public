# WIA-SEMI-011 Flexible Display SDK

TypeScript SDK for flexible display calculations and WIA-SEMI-011 standard validation.

## Installation

```bash
npm install @wia/flexible-display-sdk
```

## Usage

```typescript
import {
  FlexibleDisplay,
  BendRadiusCalculator,
  DurabilityCalculator,
  CertificationValidator,
  WIA_CONSTANTS
} from '@wia/flexible-display-sdk';

// Create a flexible display instance
const display = new FlexibleDisplay({
  type: 'foldable',
  bendRadius: 2.5,
  foldCycles: 250000,
  creaseDepth: 0.3,
  screenSize: 7.6,
  resolution: { width: 2208, height: 1768 },
  brightness: 1750,
  refreshRate: 120
});

// Check WIA-SEMI-011 compliance
const isCompliant = display.meetsWIAStandard();
console.log(`WIA-SEMI-011 Compliant: ${isCompliant}`);

// Calculate bend strain
const strain = display.calculateBendStrain(25); // 25μm offset
console.log(`Bend strain: ${strain.toFixed(2)}%`);

// Estimate lifespan
const years = display.estimateLifespan(100); // 100 folds/day
console.log(`Estimated lifespan: ${years.toFixed(1)} years`);

// Calculate minimum bend radius
const minRadius = BendRadiusCalculator.calculateMinRadius('utg', 50, 70);
console.log(`Minimum bend radius: ${minRadius.toFixed(2)} mm`);

// Validate certification
const validation = CertificationValidator.validateStandard(display.specs);
console.log(`Certification tier: ${validation.tier}`);
console.log(`Passed: ${validation.passed}`);
if (validation.issues.length > 0) {
  console.log('Issues:', validation.issues);
}
```

## API Reference

### FlexibleDisplay

Main class for flexible display instances.

**Constructor:**
```typescript
new FlexibleDisplay(specs: FlexibleDisplaySpecs)
```

**Methods:**
- `calculateBendStrain(neutralPlaneOffset: number): number`
- `estimateLifespan(dailyFolds: number): number`
- `meetsWIAStandard(): boolean`

### BendRadiusCalculator

Static utility class for bend radius calculations.

**Methods:**
- `calculateMinRadius(substrate, thickness, modulus): number`
- `calculateStrain(thickness, radius): number`

### DurabilityCalculator

Static utility class for durability estimation.

**Methods:**
- `estimateFoldCycles(baseCycles, degradationFactor, tempFactor, humidityFactor): number`
- `calculateLifespanYears(totalCycles, dailyFolds): number`

### HingeAnalyzer

Static utility class for hinge stress analysis.

**Methods:**
- `analyzeStress(hingeType, foldAngle, torque, gap): object`

### CertificationValidator

Static utility class for WIA-SEMI-011 validation.

**Methods:**
- `validateStandard(specs): { tier, passed, issues }`

## Constants

```typescript
WIA_CONSTANTS = {
  MIN_BEND_RADIUS: 3,     // mm
  MIN_FOLD_CYCLES: 200000,
  MAX_CREASE_DEPTH: 0.5,  // mm
  MIN_BRIGHTNESS: 400,    // cd/m²
  MIN_UNIFORMITY: 85,     // %
  MIN_RESPONSE_TIME: 10,  // ms
  TEMP_RANGE: { min: -10, max: 50 },  // °C
  HUMIDITY_RANGE: { min: 10, max: 90 } // %RH
}
```

## License

MIT

## Support

For questions and support, visit: https://wiabooks.store/tag/wia-flexible-display/

© 2025 World Certification Industry Association | 弘益人間 (Benefit All Humanity)
