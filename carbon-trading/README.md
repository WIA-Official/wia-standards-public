# WIA-FIN-025: Carbon Trading Standard 🌍

> A comprehensive standard for carbon credit trading, verification, and market infrastructure

[![Version](https://img.shields.io/badge/version-2.0-green.svg)](./spec/v2.0.md)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Standard](https://img.shields.io/badge/WIA-FIN--025-22C55E.svg)](https://wia.org/standards/FIN-025)

## Overview

WIA-FIN-025 establishes a comprehensive framework for carbon credit creation, verification, trading, and retirement. This standard enables interoperability between carbon markets, ensures environmental integrity, and facilitates the transition to a net-zero economy.

**Philosophy:** 홍익인간 (弘益人間) (홍익인간) - *Benefit All Humanity*

### Key Features

- 🔐 **Blockchain Security**: Immutable transaction records and smart contract automation
- 📊 **Market Infrastructure**: Real-time price discovery and order matching
- ✅ **Verification Standards**: ISO 14064/14065 compliance with AI-powered automation
- 🌐 **Global Integration**: Paris Agreement Article 6 compatibility
- 📈 **Analytics & Insights**: Market trend analysis and portfolio optimization
- ⚡ **API & Integration**: RESTful API, WebSocket feeds, TypeScript SDK

## Quick Start

### Installation

```bash
npm install @wia/carbon-trading
```

### Basic Usage

```typescript
import CarbonTradingSDK from '@wia/carbon-trading';

const sdk = new CarbonTradingSDK({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create a carbon project
const project = await sdk.projects.create({
  name: 'Amazon Rainforest REDD+',
  type: 'redd+',
  location: {
    country: 'Brazil',
    region: 'Acre'
  },
  methodology: 'VM0006',
  developer: {
    name: 'Forest Conservation Inc',
    contact: 'info@forestconservation.org'
  },
  estimatedAnnualReduction: 500000
});

// Issue carbon credits
const credits = await sdk.credits.issue({
  projectId: project.projectId,
  quantity: 500000,
  vintage: 2024,
  creditType: 'avoidance',
  verificationReport: '...'
});

// Execute a trade
const trade = await sdk.market.createOrder({
  type: 'market',
  side: 'buy',
  quantity: 10000,
  minQualityScore: 85
});

// Retire credits
await sdk.credits.retire(
  credits.credits[0].id,
  5000,
  'Acme Corp',
  'Corporate net-zero commitment 2024'
);
```

## Documentation

### 📖 eBook: Carbon Trading - A Comprehensive Guide

Complete guide to carbon markets and climate finance:

- **[English Version](./ebook/en/)** - 8 chapters covering everything from fundamentals to future trends
- **[Korean Version](./ebook/ko/)** - 한국어 번역본

**Chapters:**
1. Introduction to Carbon Trading
2. Carbon Markets Fundamentals
3. Trading Mechanisms & Instruments
4. Verification & Standards
5. Implementation Guide
6. Regulations & Compliance
7. Case Studies
8. Future of Carbon Trading

### 📋 Technical Specifications

- **[v2.0](./spec/v2.0.md)** (Current) - Latest specification with AI, IoT, blockchain integration
- **[v1.2](./spec/v1.2.md)** (Deprecated) - Previous version
- **[v1.1](./spec/v1.1.md)** (Deprecated) - Legacy version
- **[v1.0](./spec/v1.0.md)** (Deprecated) - Initial release

### 🧪 Interactive Simulator

Try the [Carbon Trading Simulator](./simulator/) to test and validate carbon trading scenarios:

- **Overview**: Market statistics and project types
- **Testing**: Create projects and execute trades
- **Validation**: Verify emissions data
- **Results**: View trading analytics
- **Integration**: API implementation examples

## Architecture

### 4-Phase System

```
┌─────────────────────────────────────────────┐
│  Phase 1: Emission Verification            │
│  IoT sensors, satellite data, auditors     │
├─────────────────────────────────────────────┤
│  Phase 2: Credit Issuance                  │
│  Blockchain tokenization, registry         │
├─────────────────────────────────────────────┤
│  Phase 3: Market Trading                   │
│  Spot, futures, options trading            │
├─────────────────────────────────────────────┤
│  Phase 4: Compliance & Reporting           │
│  Regulatory reporting, audit trails        │
└─────────────────────────────────────────────┘
```

## Core Components

### Project Types

- **🌲 Reforestation & REDD+**: Forest conservation and afforestation
- **⚡ Renewable Energy**: Solar, wind, hydroelectric projects
- **🏭 Energy Efficiency**: Industrial and building efficiency
- **🌾 Sustainable Agriculture**: Soil carbon, methane reduction
- **♻️ Waste Management**: Landfill gas capture, composting

### Credit Types

- **Avoidance**: Preventing emissions (renewable energy, efficiency)
- **Reduction**: Decreasing emissions (fuel switching, process improvements)
- **Removal**: Extracting CO2 from atmosphere (DAC, biochar, reforestation)

### Supported Standards

- ✅ Verra VCS (Verified Carbon Standard)
- ✅ Gold Standard
- ✅ CDM (Clean Development Mechanism)
- ✅ Climate Action Reserve
- ✅ American Carbon Registry
- ✅ Paris Agreement Article 6

## API Reference

### REST API

```bash
# Base URL
https://api.wia.org/carbon/v2

# Authentication
Authorization: Bearer YOUR_API_KEY
```

**Endpoints:**

```
POST   /api/v2/projects          # Create project
GET    /api/v2/projects/{id}     # Get project details
POST   /api/v2/credits/issue     # Issue credits
GET    /api/v2/credits/search    # Search credits
POST   /api/v2/trading/orders    # Create order
GET    /api/v2/market/prices     # Get prices
POST   /api/v2/credits/retire    # Retire credits
POST   /api/v2/verification      # Submit verification
```

### WebSocket API

```typescript
// Connect to real-time feed
await sdk.connectWebSocket(['prices.all', 'trades.VCS', 'verification']);

// Listen for price updates
sdk.on('prices.all', (data) => {
  console.log('Price update:', data);
});

// Listen for trade executions
sdk.on('trades.VCS', (trade) => {
  console.log('Trade executed:', trade);
});
```

## Market Statistics (2024)

| Metric | Value |
|--------|-------|
| **Global Market Size** | $850+ billion |
| **Credits Traded (2024)** | 2+ billion tCO2e |
| **Participating Countries** | 175+ |
| **Active Projects** | 15,000+ |
| **Average Price (EU ETS)** | €80-90 |
| **Average Price (Voluntary)** | $8-25 |
| **Removal Credit Price** | $50-200 |

## Use Cases

### 🏭 Corporate Net Zero
Companies purchasing carbon credits to offset unavoidable emissions and achieve net-zero commitments.

**Example**: Microsoft purchases 1.3M tonnes annually to become carbon negative by 2030.

### 🌲 Reforestation Projects
Forest conservation projects generating verifiable carbon credits from CO2 sequestration.

**Example**: Alto Mayo REDD+ project protects 182,000 hectares of Amazon rainforest, avoiding 3.5M tonnes CO2.

### ⚡ Renewable Energy
Wind, solar, and hydroelectric projects monetizing emission reductions through carbon credit sales.

**Example**: 100MW solar farm in India generates 75K credits/year under CDM methodology.

### 🏛️ Government Compliance
Regulated entities meeting emission reduction targets through carbon trading mechanisms.

**Example**: EU power plant surrenders 2M ETS allowances annually for compliance.

### ✈️ Aviation Offsetting
Airlines offsetting flight emissions through verified carbon credit purchases under CORSIA.

**Example**: Delta invests $1B in carbon credits over 10 years for aviation offsetting.

## Development

### Prerequisites

- Node.js >= 16.0.0
- TypeScript >= 5.0
- npm or yarn

### Local Setup

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/carbon-trading

# Install dependencies
cd api/typescript
npm install

# Build
npm run build

# Run tests
npm test

# Lint
npm run lint
```

### Testing

```bash
# Unit tests
npm test

# Integration tests
npm run test:integration

# Coverage
npm run test:coverage
```

## Integration Examples

### Python

```python
import requests

API_KEY = "your-api-key"
BASE_URL = "https://api.wia.org/carbon/v2"

headers = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json"
}

# Create project
response = requests.post(
    f"{BASE_URL}/projects",
    headers=headers,
    json={
        "name": "Solar Farm India",
        "type": "renewable_energy",
        "location": {"country": "India", "region": "Gujarat"},
        "methodology": "ACM0002",
        "developer": {"name": "Green Energy", "contact": "info@green.com"},
        "estimatedAnnualReduction": 50000
    }
)

project = response.json()
print(f"Project created: {project['projectId']}")
```

### Go

```go
package main

import (
    "bytes"
    "encoding/json"
    "net/http"
)

type Project struct {
    Name string `json:"name"`
    Type string `json:"type"`
    // ... other fields
}

func main() {
    apiKey := "your-api-key"
    baseURL := "https://api.wia.org/carbon/v2"

    project := Project{
        Name: "Solar Farm India",
        Type: "renewable_energy",
    }

    data, _ := json.Marshal(project)
    req, _ := http.NewRequest("POST", baseURL+"/projects", bytes.NewBuffer(data))
    req.Header.Set("Authorization", "Bearer "+apiKey)
    req.Header.Set("Content-Type", "application/json")

    client := &http.Client{}
    resp, _ := client.Do(req)
    defer resp.Body.Close()
}
```

### cURL

```bash
curl -X POST https://api.wia.org/carbon/v2/projects \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Solar Farm India",
    "type": "renewable_energy",
    "location": {"country": "India", "region": "Gujarat"},
    "methodology": "ACM0002",
    "developer": {"name": "Green Energy", "contact": "info@green.com"},
    "estimatedAnnualReduction": 50000
  }'
```

## Contributing

We welcome contributions! Please see our [Contributing Guide](../../CONTRIBUTING.md) for details.

### Development Process

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Support

- 📧 Email: support@wia.org
- 💬 Discord: [WIA Community](https://discord.gg/wia)
- 🐛 Issues: [GitHub Issues](https://github.com/WIA-Official/wia-standards/issues)
- 📖 Docs: [Full Documentation](https://docs.wia.org/standards/FIN-025)

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Related Standards

- [WIA-FIN-001](../fin-001/) - Digital Currency Standard
- [WIA-FIN-010](../fin-010/) - Cross-Border Payments
- [WIA-FIN-020](../fin-020/) - Green Finance
- [WIA-ENV-001](../env-001/) - Environmental Data Standard

## Acknowledgments

Special thanks to:

- Verra for VCS methodology framework
- Gold Standard for quality principles
- UNFCCC for CDM foundation
- ICVCM for Core Carbon Principles
- All contributors and early adopters

---

© 2025 WIA (World Certification Industry Association)

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

*Building a sustainable future through standardized carbon markets*
