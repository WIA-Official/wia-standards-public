# WIA-SOC-010: Electricity Grid Standard

**전 세계 어디서든 인정되는 스마트 전력망 표준**
*Globally recognized smart electricity grid standard*

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA-SOC-010 is a comprehensive open standard for smart electricity grid systems, defining data formats, APIs, communication protocols, and integration patterns. This standard enables interoperability, efficiency, and sustainability across electrical grid ecosystems worldwide.

### Key Features

- **Universal Data Format**: JSON-LD based for semantic interoperability
- **Real-time Monitoring**: Grid status, power quality, and operational metrics
- **Renewable Integration**: Solar, wind, hydro with advanced forecasting
- **Energy Storage Management**: Battery systems, pumped hydro, grid-scale storage
- **Demand Response**: Automated DR programs with real-time pricing
- **Smart Metering**: AMI integration with consumption analytics
- **SCADA Integration**: IEC 61850, DNP3, Modbus, OPC UA support
- **Cloud Ready**: AWS, Azure, Google Cloud compatibility
- **Privacy-First**: GDPR/CCPA compliant, encrypted data
- **Open Source**: MIT licensed reference implementations

---

## Quick Start

### TypeScript API

```bash
cd api/typescript

# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test
```

### Example Usage

```typescript
import { WiaElectricityGrid } from 'wia-soc-010';

const client = new WiaElectricityGrid({
  host: 'api.grid-operator.com',
  bearerToken: 'your-token-here'
});

// Get current grid status
const status = await client.getGridStatus();
console.log(`Load: ${status.currentLoad.value}MW / ${status.capacity.total}MW`);
console.log(`Load Factor: ${status.loadFactor * 100}%`);

// Get renewable generation
const renewables = await client.getRenewableGeneration();
console.log(`Renewable Penetration: ${renewables.penetrationRate * 100}%`);

// Monitor energy storage
const storage = await client.getStorageStatus();
storage.forEach(ess => {
  console.log(`${ess.id}: ${ess.stateOfCharge}% SOC, ${ess.powerFlow.value}MW ${ess.powerFlow.direction}`);
});

// Subscribe to real-time events
const ws = client.subscribeToEvents((event) => {
  console.log(`${event.channel}: ${JSON.stringify(event.data)}`);
}, {
  channels: ['grid-status', 'renewables', 'storage', 'demand-response', 'alerts']
});
```

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data structures and JSON-LD formats |
| 2 | [PHASE-2-API.md](spec/PHASE-2-API.md) | RESTful API and WebSocket specifications |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | Communication protocols and security |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | Cloud, SCADA, and smart city integration |

---

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/system/info` | Get grid operator system information |
| GET | `/grid-status` | Get current grid operational status |
| GET | `/renewables/current` | Get current renewable generation |
| GET | `/renewables/forecast` | Get renewable energy forecast |
| GET | `/storage/status` | Get energy storage system status |
| POST | `/storage/dispatch` | Dispatch storage charging/discharging |
| GET | `/demand-response/events` | Get demand response events |
| POST | `/demand-response/events` | Create new DR event |
| GET | `/power-quality/current` | Get power quality metrics |
| GET | `/meters/{id}/consumption` | Get smart meter consumption |
| GET | `/alerts` | Get system alerts |
| POST | `/alerts/{id}/acknowledge` | Acknowledge alert |
| WS | `/stream` | Real-time event stream |

---

## Interactive Demos

### 🎮 Simulator
Explore smart grid operations with our interactive 5-tab simulator:
- **Load Balancing**: Adjust power distribution across grid zones
- **Renewable Integration**: Monitor solar, wind, hydro generation
- **Demand Response**: Manage peak demand events
- **Grid Stability**: Monitor frequency, voltage, power quality
- **Energy Storage**: Optimize battery charging/discharging

👉 [Launch Simulator](simulator/index.html)

### 📚 E-Book
Comprehensive guide to electricity grid standards:
- Chapter 1: Introduction to Smart Electricity Grids
- Chapter 2: Grid Architecture & Components
- Chapter 3: Renewable Energy Integration
- Chapter 4: Smart Metering & AMI
- Chapter 5: Grid Stability & Power Quality
- Chapter 6: Energy Storage Systems
- Chapter 7: SCADA & Control Systems
- Chapter 8: Future Innovations

👉 [Read E-Book (English)](ebook/en/index.html) | [한국어](ebook/ko/index.html)

---

## Architecture

### Data Flow

```
┌─────────────────┐
│  Smart Meters   │───┐
└─────────────────┘   │
                      │
┌─────────────────┐   │    ┌─────────────────┐
│  Solar/Wind     │───┼───▶│   WIA-SOC-010   │
└─────────────────┘   │    │   API Gateway   │
                      │    └─────────────────┘
┌─────────────────┐   │            │
│  Battery ESS    │───┘            │
└─────────────────┘                │
                                   ▼
                          ┌─────────────────┐
                          │   Cloud Platform│
                          │  (AWS/Azure/GCP)│
                          └─────────────────┘
                                   │
                          ┌────────┴────────┐
                          ▼                 ▼
                   ┌─────────────┐   ┌─────────────┐
                   │  Analytics  │   │ Dashboards  │
                   └─────────────┘   └─────────────┘
```

### Technology Stack

**Frontend:**
- HTML5, CSS3, JavaScript
- Interactive simulators and visualizations
- Multi-language support (99+ languages)

**Backend:**
- RESTful API (JSON-LD)
- WebSocket for real-time events
- OAuth 2.0 + JWT authentication

**Protocols:**
- MQTT (IoT devices)
- IEC 61850 (substations)
- DNP3 (SCADA)
- Modbus TCP (field devices)

**Cloud:**
- AWS (IoT Core, Kinesis, Lambda, S3)
- Azure (IoT Hub, Stream Analytics, Functions)
- GCP (Cloud IoT, Pub/Sub, BigQuery)

---

## Use Cases

### 🌍 Utility-Scale Deployment
Large electric utilities managing transmission and distribution networks with:
- 1,000,000+ smart meters
- 50+ substations
- 500 MW+ renewable generation
- 100 MW+ battery storage

**Benefits:**
- 30-50% reduction in outage duration (SAIDI)
- 20-30% improvement in grid efficiency
- 40%+ renewable energy integration
- $50M+ annual operational savings

### 🏙️ Smart City Integration
Municipal utilities integrating with smart city systems:
- EV charging infrastructure
- Smart building HVAC
- Traffic signal coordination
- Water/waste management

**Benefits:**
- Unified energy management
- Cross-domain optimization
- Enhanced resilience
- Improved sustainability

### 🏠 Residential Prosumers
Homeowners with solar panels, batteries, and smart homes:
- Real-time consumption monitoring
- Dynamic pricing participation
- Peer-to-peer energy trading
- Grid services revenue

**Benefits:**
- 10-20% energy cost savings
- Backup power during outages
- Increased energy independence
- Environmental impact reduction

### 🏭 Industrial Applications
Manufacturing and data centers with:
- On-site generation (CHP, solar)
- Large-scale storage systems
- Flexible loads
- Demand response participation

**Benefits:**
- Reduced demand charges
- Increased power reliability
- Revenue from grid services
- Sustainability compliance

---

## Global Deployment

### 🇺🇸 United States
- **Deployment**: 100M+ smart meters, 16,000+ PMUs
- **Leaders**: California ISO, PJM, ERCOT
- **Standards**: NERC CIP, IEEE 1547
- **Penetration**: 60%+ of households

### 🇪🇺 European Union
- **Deployment**: 200M+ smart meters
- **Leaders**: Italy, Spain, Sweden, Finland
- **Standards**: EU Energy Packages, ENTSO-E
- **Penetration**: 80%+ target achieved in many countries

### 🇨🇳 China
- **Deployment**: 500M+ smart meters (world's largest)
- **Leaders**: State Grid Corporation of China
- **Standards**: GB/T standards
- **Penetration**: Near-complete urban coverage

### 🇰🇷 South Korea
- **Deployment**: Jeju Smart Grid testbed, national rollout
- **Leaders**: KEPCO, K-Grid
- **Standards**: K-MEG, international compliance
- **Penetration**: 30%+ with rapid growth

### 🇯🇵 Japan
- **Deployment**: Post-Fukushima acceleration
- **Leaders**: TEPCO, Kansai Electric
- **Standards**: JIS standards
- **Penetration**: 40%+ and expanding

---

## Security & Privacy

### Encryption
- **Transport**: TLS 1.3 mandatory
- **Data at Rest**: AES-256-GCM
- **Certificates**: X.509 PKI

### Authentication
- OAuth 2.0 + JWT (APIs)
- Client certificates (devices)
- Multi-factor authentication (admin)

### Authorization
- Role-based access control (RBAC)
- Principle of least privilege
- Audit logging

### Privacy
- GDPR compliant data handling
- CCPA consumer rights support
- Data anonymization options
- Customer consent management

### Compliance
- NERC CIP (North America)
- IEC 62351 (cybersecurity)
- ISO 27001 (information security)
- SOC 2 Type II (service organizations)

---

## Performance

### Latency
- **Critical Control**: < 10 ms
- **SCADA Telemetry**: < 100 ms
- **Smart Meter Data**: < 1 second
- **Historical Queries**: < 5 seconds

### Throughput
- **API Requests**: 10,000+ req/sec
- **WebSocket Messages**: 100,000+ msg/sec
- **MQTT Messages**: 1,000,000+ msg/sec
- **Data Ingestion**: 1 TB+/day

### Availability
- **Control Systems**: 99.99% (52 min/year downtime)
- **Data Collection**: 99.9% (8.76 hours/year downtime)
- **APIs**: 99.95% (4.38 hours/year downtime)

### Scalability
- **Meters**: Millions per deployment
- **Devices**: Tens of millions
- **Data Points**: Billions per day
- **Geographic**: Continental-scale grids

---

## Development

### Prerequisites
- Node.js 16+ (TypeScript SDK)
- Python 3.8+ (data analysis)
- Docker (containerized deployment)
- Git (version control)

### Setup

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/electricity-grid

# Install TypeScript SDK
cd api/typescript
npm install
npm run build

# Run tests
npm test

# Start development server
npm run dev
```

### Testing

```bash
# Unit tests
npm test

# Integration tests
npm run test:integration

# E2E tests
npm run test:e2e

# Coverage
npm run test:coverage
```

### Contributing

We welcome contributions! Please see [CONTRIBUTING.md](../../CONTRIBUTING.md) for guidelines.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## Support

### Documentation
- [Technical Specifications](spec/)
- [API Reference](spec/PHASE-2-API.md)
- [Integration Guide](spec/PHASE-4-INTEGRATION.md)
- [E-Book](ebook/en/index.html)

### Community
- **GitHub**: [WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Issues**: [Issue Tracker](https://github.com/WIA-Official/wia-standards/issues)
- **Discussions**: [GitHub Discussions](https://github.com/WIA-Official/wia-standards/discussions)
- **Email**: [standards@wia-official.org](mailto:standards@wia-official.org)

### Commercial Support
- Consulting services
- Training programs
- Enterprise support plans
- Custom development

Contact: [enterprise@wia-official.org](mailto:enterprise@wia-official.org)

---

## License

This standard is released under the MIT License. See [LICENSE](../../LICENSE) for details.

```
MIT License

Copyright (c) 2025 SmileStory Inc. / WIA (World Certification Industry Association)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## Acknowledgments

This standard builds upon the work of:
- International Electrotechnical Commission (IEC)
- Institute of Electrical and Electronics Engineers (IEEE)
- North American Electric Reliability Corporation (NERC)
- European Network of Transmission System Operators for Electricity (ENTSO-E)
- Global smart grid pioneers and practitioners

---

## Roadmap

### Version 1.1 (Q2 2026)
- Enhanced AI/ML integration
- Blockchain for P2P trading
- V2G (Vehicle-to-Grid) support
- Expanded language support

### Version 2.0 (Q4 2026)
- Quantum-safe cryptography
- Advanced microgrid capabilities
- Extended IoT device support
- Improved edge computing integration

### Version 3.0 (2027+)
- Full autonomous grid operations
- Continental-scale coordination
- Space-based solar integration
- Next-generation storage technologies

---

**WIA-SOC-010 Electricity Grid Standard**
Version 1.0.0
© 2025 SmileStory Inc. / WIA
홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity

*Building a sustainable, intelligent, and equitable energy future for all.*
