# 🚗 WIA-AUTO-021: Vehicle Lightweight Material Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-AUTO-021
> **Version:** 1.0.0
> **Status:** Active
> **Category:** AUTO / Mobility
> **Color:** Orange (#F97316)

---

## 🌟 Overview

The WIA-AUTO-021 standard defines the specifications, testing methods, and computational frameworks for vehicle lightweight materials, including carbon fiber composites, aluminum alloys, high-strength steel, and advanced polymers for automotive applications.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to accelerate the development and adoption of lightweight materials that improve vehicle efficiency, reduce emissions, and enhance safety while promoting sustainability.

## 🎯 Key Features

- **Material Properties Database**: Comprehensive specifications for lightweight materials
- **Weight Reduction Calculations**: Algorithms for calculating weight savings and efficiency gains
- **Strength-to-Weight Analysis**: Performance metrics for material selection
- **Manufacturing Process Standards**: Guidelines for forming, joining, and finishing
- **Crash Safety Validation**: Impact resistance and energy absorption testing
- **Recyclability Assessment**: End-of-life material recovery and reuse protocols

## 📊 Core Concepts

### 1. Specific Strength

```
σ_specific = σ_tensile / ρ
```

Where:
- `σ_specific` = Specific strength (N·m/kg)
- `σ_tensile` = Tensile strength (MPa)
- `ρ` = Material density (kg/m³)

### 2. Weight Reduction Potential

```
ΔW = V × (ρ_original - ρ_new)
```

Where:
- `ΔW` = Weight reduction (kg)
- `V` = Component volume (m³)
- `ρ_original` = Original material density (kg/m³)
- `ρ_new` = New material density (kg/m³)

### 3. Fuel Efficiency Improvement

```
ΔFC = k × (ΔW / W_vehicle) × 100
```

Where:
- `ΔFC` = Fuel consumption reduction (%)
- `k` = Efficiency coefficient (typically 0.6-0.7)
- `ΔW` = Weight reduction (kg)
- `W_vehicle` = Total vehicle weight (kg)

## 🔧 Components

### TypeScript SDK

```typescript
import {
  calculateWeightReduction,
  analyzeMaterialProperties,
  assessStructuralIntegrity,
  calculateCostBenefit
} from '@wia/auto-021';

// Calculate weight reduction for CFRP component
const reduction = calculateWeightReduction({
  component: 'hood',
  originalMaterial: 'steel',
  newMaterial: 'cfrp',
  volume: 0.05 // m³
});

// Analyze material properties
const analysis = analyzeMaterialProperties({
  material: 'aluminum-6061-t6',
  temperature: 25, // °C
  loadCondition: 'tensile'
});

console.log(analysis.specificStrength, analysis.youngModulus);
```

### CLI Tool

```bash
# Calculate weight reduction
wia-auto-021 calc-weight --component hood --from steel --to cfrp

# Analyze material properties
wia-auto-021 material --name aluminum-6061-t6 --property all

# Assess crash safety
wia-auto-021 crash-test --material cfrp --impact-speed 50 --angle 30

# Calculate cost-benefit
wia-auto-021 cost-benefit --material cfrp --quantity 1000 --lifespan 10
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-AUTO-021-v1.0.md](./spec/WIA-AUTO-021-v1.0.md) | Complete specification with material standards |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-auto-021.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/vehicle-lightweight-material

# Run installation script
./install.sh

# Verify installation
wia-auto-021 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/auto-021

# Or yarn
yarn add @wia/auto-021
```

```typescript
import { LightweightMaterialSDK } from '@wia/auto-021';

const sdk = new LightweightMaterialSDK();

// Calculate weight reduction
const result = sdk.calculateWeightReduction({
  component: 'body-panel',
  originalMaterial: 'steel',
  newMaterial: 'aluminum',
  surfaceArea: 2.5 // m²
});

console.log(`Weight reduction: ${result.weightSaved.toFixed(2)} kg`);
console.log(`Fuel savings: ${result.fuelSavings.toFixed(2)}%`);
```

## 🔬 Material Properties

| Material | Density (kg/m³) | Tensile Strength (MPa) | Young's Modulus (GPa) | Specific Strength (kN·m/kg) |
|----------|----------------|------------------------|----------------------|----------------------------|
| Steel (Mild) | 7,850 | 400 | 200 | 51 |
| HSS (AHSS) | 7,850 | 1,500 | 200 | 191 |
| Aluminum 6061-T6 | 2,700 | 310 | 69 | 115 |
| Aluminum 7075-T6 | 2,810 | 572 | 72 | 204 |
| Magnesium AZ31B | 1,770 | 260 | 45 | 147 |
| CFRP (Standard) | 1,600 | 600-1,000 | 150 | 375-625 |
| GFRP | 2,000 | 480 | 35 | 240 |
| Titanium Ti-6Al-4V | 4,430 | 950 | 114 | 214 |

## ⚙️ Manufacturing Processes

1. **Forming Technologies**
   - Stamping and deep drawing (metals)
   - Thermoforming (thermoplastics)
   - Autoclave molding (composites)
   - Resin Transfer Molding (RTM)
   - Hot press forming (HSS)

2. **Joining Methods**
   - Adhesive bonding
   - Self-piercing rivets (SPR)
   - Friction stir welding (FSW)
   - Resistance spot welding (RSW)
   - Laser welding
   - Mechanical fasteners

3. **Surface Treatment**
   - Anodizing (aluminum)
   - Powder coating
   - E-coating
   - Painting and finishing

## 🛡️ Safety Considerations

1. **Crash Safety**: Materials must meet NCAP and IIHS standards
2. **Energy Absorption**: Minimum specific energy absorption: 50 kJ/kg
3. **Structural Integrity**: Safety factor ≥ 1.5 for all load cases
4. **Corrosion Resistance**: Minimum 10-year protection
5. **Temperature Stability**: Performance range: -40°C to +80°C
6. **Fire Safety**: Meet FMVSS 302 flammability standards

## 🌐 WIA Integration

This standard integrates with:
- **WIA-AUTO-001** through **WIA-AUTO-020**: Complete automotive standards suite
- **WIA-INTENT**: Intent-based material selection queries
- **WIA-OMNI-API**: Universal automotive API gateway
- **WIA-SOCIAL**: Material performance data sharing
- **WIA-SUSTAINABILITY**: Lifecycle and recyclability tracking

## 📖 Use Cases

1. **Body Panels**: CFRP/aluminum hoods, doors, fenders (20-40% weight reduction)
2. **Chassis Components**: Aluminum/magnesium subframes (30-50% reduction)
3. **Powertrain**: Aluminum engine blocks and components (40-60% reduction)
4. **Interior**: Glass fiber reinforced thermoplastics (15-25% reduction)
5. **Closures**: Aluminum/composite tailgates and liftgates (30-45% reduction)

## 🌱 Environmental Impact

- **Weight Reduction**: 10% vehicle weight reduction = 6-8% fuel consumption reduction
- **CO₂ Savings**: 100 kg weight reduction = ~8.4g CO₂/km reduction
- **Recyclability**: Aluminum: 95%, Steel: 85%, CFRP: 30% (improving)
- **Energy Savings**: Recycled aluminum uses 95% less energy than primary production

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
