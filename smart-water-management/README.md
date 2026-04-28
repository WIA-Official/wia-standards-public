# WIA-CITY-020: Smart Water Management Standard 💧

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> 홍익인간 (弘益人間) - Benefit All Humanity

## Overview

WIA-CITY-020 defines standards for intelligent lighting systems in smart cities. This standard enables 60-80% energy savings through LED technology, occupancy sensing, daylight harvesting, and adaptive controls.

## Features

- **High-Efficacy LEDs**: 120-200 lm/W performance
- **Occupancy Sensing**: Automatic on/off based on presence
- **Daylight Harvesting**: Dim lights when natural light is available
- **Circadian Tuning**: Support natural human rhythms (2700K-6500K)
- **Four Certification Levels**: Bronze, Silver, Gold, Platinum

## Quick Start

```bash
# Install the water distribution and quality management toolkit
chmod +x install.sh
./install.sh

# Run the simulator
cd simulator
open index.html

# View the specification
cd spec
cat smart-water-management-v1.0.md
```

## Certification Levels

| Level | Requirements | Energy Savings | Applications |
|-------|--------------|----------------|--------------|
| **Bronze** | LED (100 lm/W+), basic dimming, occupancy | 50-65% | Small offices, residential |
| **Silver** | LED (120 lm/W+), daylight harvesting, wireless | 65-75% | Commercial, schools |
| **Gold** | LED (150 lm/W+), circadian tuning, AI | 75-85% | Hospitals, campuses |
| **Platinum** | LED (180 lm/W+), Li-Fi, full IoT integration | 85%+ | Smart cities |

## API Usage

```typescript
import { SmartLighting, LightingConfig } from '@wia/smart-water-management';

// Initialize system
const lighting = new SmartLighting({
  mode: 'auto',
  brightness: 75,
  colorTemperature: 4000,
  certificationLevel: 'gold'
});

// Start system
await lighting.start();

// Get status
const status = lighting.getStatus();
console.log(`Energy Savings: ${status.energySavings}%`);
```

## CLI Usage

```bash
# Turn on lights
./cli/smart-water-management.sh on

# Set brightness
./cli/smart-water-management.sh brightness --level=50

# Enable daylight harvesting
./cli/smart-water-management.sh daylight --enable

# Get status
./cli/smart-water-management.sh status
```

## Documentation

- **Specification**: [`spec/smart-water-management-v1.0.md`](spec/smart-water-management-v1.0.md)
- **API Reference**: [`api/typescript/README.md`](api/typescript/README.md)
- **E-Book (English)**: [`ebook/en/index.html`](ebook/en/index.html)
- **E-Book (Korean)**: [`ebook/ko/index.html`](ebook/ko/index.html)
- **Simulator**: [`simulator/index.html`](simulator/index.html)

## Key Technologies

- **LEDs**: High-efficacy solid-state lighting (100-200 lm/W)
- **Sensors**: Occupancy, daylight, ambient light
- **Controls**: Continuous dimming, tunable white, scene control
- **Communication**: Zigbee, Bluetooth Mesh, DALI-2, Wi-Fi
- **AI/ML**: Occupancy prediction, energy optimization

## Energy Savings Breakdown

- Occupancy sensing: 30-50%
- Daylight harvesting: 20-40%
- LED efficiency: 60-80% vs incandescent
- **Total Combined**: 70-85%+

## License

© 2025 SmileStory Inc. / WIA
Distributed under the MIT License.

## Philosophy

**홍익인간 (弘益人間) - Benefit All Humanity**

---

**Version**: 1.0.0
**Standard**: WIA-CITY-020
**Category**: Smart City - Lighting Systems
**Emoji**: 💧

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
