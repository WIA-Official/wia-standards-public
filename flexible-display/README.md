# WIA-SEMI-011: Flexible Display Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> 홍익인간 (弘益人間) - Benefit All Humanity

## Overview

Comprehensive standard for flexible display technology including foldable, rollable, and stretchable displays. This repository contains specifications, testing protocols, interactive tools, and complete documentation for WIA-SEMI-011.

## 🎯 Quick Start

- **Landing Page**: Open `/index.html` for overview and navigation
- **Interactive Simulator**: Access `/simulator/index.html` for testing tools
- **Ebook**: Read complete guides in `/ebook/en/` (English) or `/ebook/ko/` (Korean)
- **Specifications**: Technical docs in `/spec/`
- **TypeScript SDK**: Integration tools in `/api/typescript/`

## 📁 Repository Structure

```
flexible-display/
├── index.html              # Landing page with EN/KO toggle
├── simulator/
│   └── index.html         # 5-tab simulator with 99 languages
├── ebook/
│   ├── en/                # English ebook (9 chapters, 15KB+ each)
│   │   ├── 01-cover.md
│   │   ├── 02-fundamentals.md
│   │   ├── 03-foldable-displays.md
│   │   ├── 04-rollable-displays.md
│   │   ├── 05-stretchable-displays.md
│   │   ├── 06-market-analysis.md
│   │   ├── 07-hinge-engineering.md
│   │   ├── 08-testing-standards.md
│   │   └── 09-future-directions.md
│   └── ko/                # Korean ebook (9 chapters, 15KB+ each)
│       ├── 01-cover.md
│       ├── 02-fundamentals.md
│       ├── 03-foldable.md
│       ├── 04-rollable.md
│       ├── 05-stretchable.md
│       ├── 06-market.md
│       ├── 07-hinge.md
│       ├── 08-testing.md
│       └── 09-future.md
├── spec/                  # Technical specifications
│   ├── WIA-SEMI-011-specification.md
│   ├── WIA-SEMI-011-testing.md
│   ├── WIA-SEMI-011-materials.md
│   └── WIA-SEMI-011-certification.md
├── api/
│   └── typescript/        # TypeScript SDK
│       ├── src/
│       │   ├── types.ts
│       │   └── index.ts
│       ├── package.json
│       └── README.md
└── README.md             # This file
```

## 🔬 Interactive Simulator

The simulator provides 5 specialized tabs for flexible display testing and validation:

### Tab 1: 📊 Flex Specifications
- Bend radius calculator
- Substrate material comparison (UTG, PI, PET, Silicone)
- Industry standards comparison (WIA vs Samsung vs LG vs BOE)

### Tab 2: 🔢 Durability Calculations
- Fold cycle estimation
- Material degradation factor analysis
- Temperature and humidity impact
- Lifecycle prediction with Weibull analysis

### Tab 3: 📡 Hinge Protocols
- Hinge configuration (Multi-Cam, Waterdrop, Teardrop, 360°)
- Fold angle and torque analysis
- Gap tolerance measurement
- Stress distribution visualization

### Tab 4: 🔗 Electronics Integration
- Flexible circuit design (Serpentine, Island-Bridge, Mesh)
- Conductor material selection (Cu, Ag, Graphene, CNT)
- Voltage drop and signal integrity analysis
- Power delivery optimization

### Tab 5: 🧪 Endurance Testing
- WIA-SEMI-011, IEC, MIL-STD, JESD protocols
- Accelerated testing simulation
- Failure mode analysis
- Pass/fail criteria validation

**Languages**: Supports 99 languages including English, Korean, Chinese, Japanese, Spanish, French, German, and 92 others.

## 📖 Complete Ebook

### English Edition (9 Chapters)

1. **Cover & Introduction** - Overview and guide structure
2. **Fundamentals** - Materials science, physics, OLED technology
3. **Foldable Displays** - Samsung Galaxy Fold/Flip, Huawei Mate X, UTG, hinges
4. **Rollable Displays** - LG Rollable, motorized mechanisms, guide rails
5. **Stretchable Displays** - Elastic substrates, micro-LED, wearables
6. **Market Analysis** - Samsung, LG, BOE, market trends, pricing
7. **Hinge Engineering** - Multi-cam, waterdrop, teardrop designs
8. **Testing & Standards** - WIA-SEMI-011 protocols, certification
9. **Future Directions** - Transparent, holographic, brain-computer interfaces

### Korean Edition (9 Chapters)

Complete Korean translation covering the same topics with real Korean content.

## 🎓 Key Technologies Covered

### Foldable Displays
- **Bend Radius**: < 3mm for optimal performance
- **Fold Cycles**: 200,000+ minimum, 300,000+ premium
- **Crease Management**: < 0.5mm depth after 200K cycles
- **Substrates**: UTG (25-50μm), Polyimide, PET
- **Hinges**: Multi-cam, Waterdrop, Teardrop, 360°

### Rollable Displays
- **Roll Mechanisms**: Motorized and manual slide-out
- **Extension Ratio**: 2:1 to 4:1 (compact to expanded)
- **Bend Radius**: 2-8mm continuous curvature
- **Applications**: Smartphones, portable monitors, rollable TVs

### Stretchable Displays
- **Elastic Deformation**: 20-50% biaxial stretching
- **Substrates**: PDMS, TPU, Styrenic block copolymers
- **Conductors**: Serpentine, island-bridge, liquid metal
- **Applications**: Wearables, conformable automotive, e-skin

## 🔧 TypeScript SDK

Production-ready SDK for flexible display calculations and validation.

### Installation

```bash
npm install @wia/flexible-display-sdk
```

### Quick Example

```typescript
import { FlexibleDisplay, CertificationValidator } from '@wia/flexible-display-sdk';

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

console.log(`WIA Compliant: ${display.meetsWIAStandard()}`);
console.log(`Lifespan: ${display.estimateLifespan(100)} years`);

const validation = CertificationValidator.validateStandard(display.specs);
console.log(`Tier: ${validation.tier}`); // "premium"
```

## 📋 Specifications

Technical specifications are provided in `/spec/`:

- **WIA-SEMI-011-specification.md**: Core requirements and classifications
- **WIA-SEMI-011-testing.md**: Testing procedures and protocols
- **WIA-SEMI-011-materials.md**: Substrate and barrier materials
- **WIA-SEMI-011-certification.md**: Certification process and tiers

## 🏆 Certification Tiers

### Standard Tier
- 200,000 fold cycles minimum
- Bend radius < 3mm
- Crease depth < 0.5mm

### Premium Tier
- 300,000 fold cycles
- Enhanced optical performance
- Tighter tolerances

### Ultra Tier
- 500,000+ fold cycles
- Exceptional durability
- Highest quality grade

## 🌏 Industry Leaders

### Samsung Display
- Market leader: 70%+ global share
- Galaxy Z Fold and Z Flip series
- UTG technology pioneer

### LG Display
- Rollable OLED innovation
- 65" Signature OLED R
- Transparent display research

### BOE Technology
- China's largest display maker
- Cost-competitive manufacturing
- Huawei, Xiaomi supplier

## 🎯 WIA-SEMI-011 Standards

### Mechanical Requirements
- Minimum bend radius: < 3mm
- Fold cycles: ≥ 200,000
- Crease depth: < 0.5mm after 200K cycles
- Hinge torque: 0.2-2.0 N·m (±25% variation)

### Optical Requirements
- Luminance: ≥ 400 cd/m² (peak white)
- Uniformity: ≥ 85% (9-point measurement)
- Color gamut: ≥ 90% DCI-P3 or ≥ 100% sRGB
- Contrast: ≥ 5,000:1 (OLED), ≥ 1,000:1 (LCD)

### Environmental Requirements
- Operating temperature: -10°C to +50°C
- Humidity: 10-90% RH (non-condensing)
- Thermal shock: 30 cycles (-20°C to +60°C)
- Water resistance: IPX7/IPX8 (optional)

## 🚀 Market Outlook

- **2024**: $8.2 billion market, 24.5 million units
- **2025**: $11.5 billion (+40%), 35 million units
- **2030**: $52 billion, 180 million units
- **CAGR**: 36.2% (2024-2030)

## 📚 Resources

- **Ebook**: https://wiabooks.store/tag/wia-flexible-display/
- **GitHub**: https://github.com/WIA-Official/wia-standards
- **Standards**: certification@wia-official.org
- **Partnerships**: partnerships@wia-official.org

## 🤝 Contributing

We welcome contributions from the global community:

1. Technical corrections → Submit via GitHub issues
2. Research contributions → Share findings
3. Industry partnerships → Contact partnerships@wia-official.org
4. Translations → Help translate to more languages

## 📄 License

This standard is published under **WIA Open Knowledge License v1.0**:

✅ **Permitted**: Free distribution, translation, educational use, commercial reference, derivative works with attribution

❌ **Prohibited**: Claiming authorship, removing attribution, misrepresenting WIA standards, unauthorized commercial sale

## 🌟 Philosophy

**홍익인간 (弘益人間)** - "Benefit All Humanity"

This standard aims to democratize knowledge about flexible display technology, enabling innovation worldwide and improving quality of life through better devices.

## 📞 Contact

- **Email**: standards@wia-official.org
- **Website**: https://wiabooks.store
- **GitHub**: https://github.com/WIA-Official

---

© 2025 SmileStory Inc. / World Certification Industry Association (WIA)

**홍익인간 (弘益人間) · Benefit All Humanity**

*"The future of displays is not flat—it's flexible, adaptable, and boundless."*
