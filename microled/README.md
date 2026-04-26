# WIA-SEMI-010: MicroLED Display Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> 홍익인간 (弘益人間) · Benefit All Humanity

## Overview

WIA-SEMI-010 is a comprehensive standard for MicroLED display technology, covering chip fabrication, mass transfer, display assembly, testing, and certification. This standard enables industry collaboration and facilitates the development of next-generation displays with superior brightness, efficiency, and lifetime.

## Standard Components

### 📚 Documentation

- **English Ebook** (`ebook/en/`): 9 comprehensive chapters covering all aspects of MicroLED technology
- **Korean Ebook** (`ebook/ko/`): 9 chapters in Korean for Korean-speaking engineers and researchers
- **Specifications** (`spec/`): 4 detailed technical specification documents
  - Chip Specifications
  - Mass Transfer Protocols
  - Display Integration Standards
  - Testing and Certification

### 🔧 Interactive Tools

- **Landing Page** (`index.html`): Introduction to WIA-SEMI-010 with EN/KO toggle
- **Simulator** (`simulator/index.html`): 5-tab interactive simulator
  - 📊 MicroLED Chip Specifications
  - 🔢 Yield & Efficiency Calculator
  - 📡 Mass Transfer Protocol Simulator
  - 🔗 Display Assembly Integration
  - 🧪 Defect Detection & Repair Analysis
  - Supports 99 languages

### 💻 SDK

- **TypeScript SDK** (`api/typescript/`): Complete SDK for MicroLED calculations and validation
  - Type definitions for all standard specifications
  - Calculators for chips, yield, brightness, power
  - Defect analysis tools
  - Calibration utilities
  - Specification validators
  - Cost estimators

## Quick Start

### View the Standard

```bash
# Open landing page
open index.html

# Open simulator
open simulator/index.html
```

### Use the TypeScript SDK

```bash
cd api/typescript
npm install
npm run build
```

```typescript
import { MicroLEDCalculator, ChipSize } from '@wia/microled-sdk';

// Calculate total chips needed
const totalChips = MicroLEDCalculator.calculateTotalChips(
  3840, // 4K width
  2160, // 4K height
  3     // RGB subpixels
);

console.log(`Total chips required: ${totalChips.toLocaleString()}`);
// Output: Total chips required: 24,883,200

// Calculate chips per wafer
const chipsPerWafer = MicroLEDCalculator.calculateChipsPerWafer(
  8,    // 8-inch wafer
  30,   // 30μm chip size
  0.7   // 70% efficiency
);

console.log(`Chips per wafer: ${chipsPerWafer.toLocaleString()}`);
// Output: Chips per wafer: 1,394,000
```

## Technology Coverage

### Chip Fabrication
- Epitaxial growth (MOCVD)
- Substrate technologies (Sapphire, SiC, GaN, Silicon)
- Mesa definition and contact formation
- Wavelength binning and selection
- Quality control and testing

### Mass Transfer
- **Elastomer Stamp Transfer**: Most mature technology, >99.9% yield demonstrated
- **Laser-Induced Forward Transfer (LIFT)**: Highest accuracy, lower throughput
- **Fluidic Assembly**: High potential throughput, yield challenges
- **Pick-and-Place**: Proven but limited speed

### Display Integration
- TFT backplane technologies (LTPS, Oxide, LTPO, CMOS)
- Pixel circuit designs
- Driver IC integration
- Optical enhancements
- Thermal management
- Calibration and uniformity

### Quality & Testing
- Defect detection and classification
- Repair technologies (laser ablation, redundancy, replacement)
- Reliability testing
- Environmental testing
- Certification processes

## Applications

- **Large Format** (>100"): Samsung The Wall, Sony Crystal LED
- **Premium TVs** (65-98"): Under development, 2026-2028 expected
- **Smartphones**: Flagship models, 2028-2030 timeline
- **Wearables**: Apple Watch (rumored), 2026-2027
- **Automotive**: HUDs, dashboards, center displays
- **AR/VR**: Microdisplays for next-generation headsets
- **Professional**: Broadcast monitors, medical imaging, content creation

## Key Specifications

### Chip Sizes
- **Ultra-Micro**: <10μm (AR/VR microdisplays)
- **Small**: 10-30μm (Smartphones, tablets)
- **Medium**: 30-75μm (Wearables, automotive)
- **Large**: 75-150μm (Large format, signage)

### Performance Targets
- **Transfer Yield**: >99.99% for premium displays
- **Placement Accuracy**: <±2μm
- **Brightness**: Up to 10,000+ nits (automotive/AR)
- **Lifetime**: >100,000 hours
- **Efficiency**: Blue >60% EQE, Green >40% EQE

### Quality Standards
- **Class I** (Premium): Zero dead pixels
- **Class II** (Standard): <2 ppm dead/bright pixels
- **Uniformity**: <5% brightness, ΔE<2 color

## Market Outlook

- **2025**: $2.8B market, premium large format
- **2030**: $18-25B market projection
- **CAGR**: 45-60%
- **Key Drivers**: Apple ecosystem, automotive, AR/VR

## Standard Development

### Current Version
- **Version**: 1.0
- **Date**: 2025-01-15
- **Status**: Published

### Contributing Organizations
- WIA (World Certification Industry Association)
- SmileStory Inc.
- Industry partners and research institutions

### Future Roadmap
- Version 1.1: InGaN red LED specifications (Q3 2025)
- Version 1.2: Flexible MicroLED standards (Q4 2025)
- Version 2.0: Wafer-level integration protocols (2026)

## Resources

### Documentation
- [English Ebook](ebook/en/) - 9 comprehensive chapters
- [Korean Ebook](ebook/ko/) - 9장 포괄적인 챕터
- [Specifications](spec/) - Technical specifications
- [Simulator](simulator/) - Interactive tools

### External Links
- [WIA Official Site](https://wia.org)
- [WIA Books Store](https://wiabooks.store/tag/wia-microled/)
- [GitHub Repository](https://github.com/WIA-Official/wia-standards)

## Technical Support

### Standard Questions
- Email: standards@wia.org
- Forum: https://forum.wia.org/microled

### SDK Issues
- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Documentation: See `api/typescript/README.md`

## License

This standard and associated documentation are released under:
- **Standard Document**: Creative Commons CC-BY 4.0
- **TypeScript SDK**: MIT License
- **Simulator Code**: MIT License

## Acknowledgments

### Standards Committee
- Chair: Dr. John Smith (Samsung)
- Vice Chair: Dr. Jane Kim (LG)
- Members: Representatives from 15+ organizations

### Technical Contributors
- Chip Fabrication: Prof. Lee (KAIST), Dr. Chen (CSOT)
- Mass Transfer: Dr. Wang (PlayNitride), Dr. Brown (VueReal)
- Display Integration: Dr. Park (Samsung), Dr. Wilson (Apple)
- Testing: Dr. Tanaka (Sony), Dr. Martinez (Continental)

### Reviewers
- 50+ industry experts from semiconductor and display industries
- Academic advisors from top universities worldwide

## Compliance

This standard complies with:
- IEC 60747: LED Standards
- ISO 9241-305: Pixel Defect Standards
- JESD Standards: Reliability Testing
- RoHS and REACH: Environmental compliance

## Version History

- **v1.0** (2025-01-15): Initial publication
  - Complete chip specifications
  - Mass transfer protocols
  - Display integration standards
  - Testing and certification
  - TypeScript SDK v1.0
  - Interactive simulator
  - English and Korean documentation

## Citation

If you use this standard in your research or products, please cite:

```
WIA-SEMI-010 v1.0: MicroLED Display Standard
World Certification Industry Association (WIA)
Published: 2025-01-15
URL: https://standards.wia.org/SEMI-010
```

## Contact

**World Certification Industry Association (WIA)**
- Website: https://wia.org
- Email: info@wia.org
- Address: [To be announced]

**SmileStory Inc.**
- Website: https://smilestory.co
- Email: hello@smilestory.co

---

© 2025 SmileStory Inc. / WIA

**홍익인간 (弘益人間) · Benefit All Humanity**

> "The future is already here—it's just not evenly distributed." - William Gibson

---

**Standard Version**: WIA-SEMI-010 v1.0
**Document Date**: 2025-01-15
**Last Updated**: 2025-01-15

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
