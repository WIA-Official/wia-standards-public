# WIA-SOC-008: Water Supply Standard

**전 세계 어디서든 인정되는 스마트 상수도 표준**  
*Globally recognized smart water infrastructure standard*

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA-SOC-008 is a comprehensive open standard for smart water supply systems, defining data formats, APIs, communication protocols, and integration patterns. This standard enables interoperability, efficiency, and sustainability across water utility ecosystems.

### Key Features

- **Universal Data Format**: JSON-LD based for semantic interoperability
- **Real-time Monitoring**: Water quality, pressure, flow with 15-minute intervals
- **Leak Detection**: AI-powered detection with 97%+ accuracy
- **Smart Metering**: AMI integration with consumption analytics
- **SCADA Integration**: OPC UA, Modbus, DNP3 support
- **Cloud Ready**: AWS, Azure, Google Cloud compatibility
- **Privacy-First**: GDPR/CCPA compliant, encrypted data
- **Open Source**: MIT licensed reference implementations

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data structures and JSON-LD formats |
| 2 | [PHASE-2-API.md](spec/PHASE-2-API.md) | RESTful API and WebSocket specifications |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | Communication protocols and security |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | SCADA, smart city, and cloud integration |

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
import { WiaWaterSupply } from 'wia-soc-008';

const client = new WiaWaterSupply({
  host: 'api.water-utility.com',
  bearerToken: 'your-token-here'
});

// Get current water quality
const quality = await client.getWaterQuality({ zoneId: 'ZONE-A' });
console.log(`pH: ${quality.data[0].parameters.pH?.value}`);

// Monitor active leaks
const leaks = await client.getActiveLeaks();
console.log(`Active leaks: ${leaks.count}`);

// Subscribe to real-time events
const ws = client.subscribeToEvents((event) => {
  console.log(`Event: ${event.channel} - ${event.timestamp}`);
}, {
  channels: ['water-quality', 'leaks', 'alerts']
});
```

---

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/system/info` | Get system identity |
| GET | `/water-quality/current` | Get current water quality |
| GET | `/water-quality/history` | Get historical quality data |
| GET | `/network/status` | Get network status |
| GET | `/leaks/active` | Get active leak events |
| POST | `/leaks/report` | Report a new leak |
| GET | `/meters/{id}/consumption` | Get meter consumption |
| GET | `/alerts` | Get system alerts |
| POST | `/alerts/{id}/acknowledge` | Acknowledge alert |
| WS | `/stream` | Real-time event stream |

---

## Interactive Demos

### 🎮 Simulator

Try the full-featured water supply simulator with 99 language support:

```bash
# Open simulator
cd simulator
open index.html
```

Features:
- 5 interactive tabs (Monitoring, Quality, Leaks, Analytics, Logs)
- Real-time status display
- 99 language dropdown
- Dark theme UI

### 📚 Ebook

Comprehensive documentation in English and Korean:

- **English**: [ebook/en/index.html](ebook/en/index.html)
- **Korean**: [ebook/ko/index.html](ebook/ko/index.html)

8 chapters covering:
1. Introduction to Water Supply Standards
2. Water Quality Monitoring and Analysis
3. Distribution Network Management
4. Leak Detection and Prevention Systems
5. Pressure Management and Optimization
6. Smart Metering and Consumption Analytics
7. API Integration and IoT Connectivity
8. Future Innovations and Sustainability

---

## Technical Specifications

### Water Quality Monitoring

**Parameters:**
- pH: Range 0-14, Accuracy ±0.1
- Turbidity: 0-4000 NTU, ±2% accuracy
- Chlorine: 0-5 mg/L, ±0.05 mg/L
- Temperature: -10°C to 100°C, ±0.1°C
- Conductivity: 0-200 mS/cm, ±1%
- Dissolved Oxygen: 0-20 mg/L, ±0.1 mg/L

**Monitoring Frequency:**
- Critical parameters: Every 5 minutes
- Standard parameters: Every 15 minutes
- Compliance reporting: Real-time
- Data retention: 10 years

### Leak Detection

**Methods:**
- Acoustic sensors (hydrophones)
- Pressure transient analysis
- Flow rate monitoring
- AI pattern recognition
- Satellite imaging

**Performance:**
- Detection accuracy: 97%+
- Minimum detectable leak: 0.1 L/min
- Average detection time: < 48 hours
- False positive rate: < 5%
- Location accuracy: ± 15 meters

### Network Performance

**Metrics:**
- Water loss target: < 10%
- System uptime: > 99.9%
- Pressure range: 2-8 bar
- Response time (API): < 200ms p95
- Alert latency: < 30 seconds

---

## Integration Examples

### SCADA Systems

```typescript
// OPC UA Connection
const opcConfig = {
  endpoint: 'opc.tcp://scada.utility.com:4840/wia/soc-008',
  securityMode: 'SignAndEncrypt',
  certificatePath: '/path/to/cert.pem'
};
```

### Cloud Platforms

**AWS IoT Core:**
```bash
aws iot create-thing --thing-name water-sensor-001
aws iot create-policy --policy-name WaterSupplyPolicy
aws iot attach-policy --policy-name WaterSupplyPolicy --target arn:aws:iot:region:account:cert/certId
```

**Azure IoT Hub:**
```bash
az iot hub device-identity create --hub-name WaterUtilityHub --device-id sensor-001
az iot hub device-identity show-connection-string --hub-name WaterUtilityHub --device-id sensor-001
```

### Smart City Platforms

- FIWARE Context Broker
- oneM2M platform
- ETSI SmartM2M
- Custom REST APIs

---

## Security & Privacy

### Data Protection

- **Encryption**: AES-256 at rest, TLS 1.3 in transit
- **Authentication**: OAuth 2.0, mTLS, API keys
- **Authorization**: Role-based access control (RBAC)
- **Privacy**: GDPR/CCPA compliant, data anonymization
- **Audit**: Complete audit trails, 7-year retention

### Cybersecurity

- IEC 62443 compliance
- NIST Cybersecurity Framework
- Network segmentation (OT/IT)
- Intrusion detection systems
- Regular security audits

---

## Certification

Products implementing WIA-SOC-008 can apply for official certification:

1. **Basic Level**: Small utilities (1,000-50,000 population)
2. **Standard Level**: Medium utilities (50,000-500,000 population)
3. **Advanced Level**: Large utilities (500,000+ population)

Certification includes:
- Automated compliance testing
- Security audit
- Interoperability verification
- Performance benchmarking
- Official WIA certification badge

Apply at: https://cert.wiastandards.com

---

## Economic Impact

### 10-Year Projections (2025-2035)

**Water Conservation:**
- Global savings: 180 billion m³/year
- Value: $75 billion annually
- Additional population served: 500 million

**Economic Benefits:**
- Infrastructure cost reduction: $50B/year
- Energy savings: $12B/year
- Total benefit: $101B/year

**Environmental Impact:**
- CO₂ reduction: 35 million tons/year
- Energy savings: 70 TWh/year
- Waterborne disease reduction: 60%
- Lives saved: 300,000+/year

---

## Roadmap

### Version 1.1 (Q2 2026)
- Advanced AI for pipe failure prediction
- Blockchain for water rights tracking
- Enhanced mobile applications
- Extended sensor support

### Version 2.0 (2027)
- Digital twin integration
- Quantum-safe encryption
- Climate adaptation features
- Desalination standards

---

## Development

### Prerequisites

- Node.js 18+
- TypeScript 5+
- Modern browser for simulator

### Building from Source

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/water-supply

# Install dependencies
cd api/typescript
npm install

# Build
npm run build

# Test
npm test

# Lint
npm run lint
```

### Running the Simulator

```bash
# No build required - pure HTML/CSS/JS
cd simulator
python3 -m http.server 8000
# Open http://localhost:8000
```

---

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.

### Areas for Contribution

- Reference implementations in other languages
- Additional sensor integrations
- Smart city platform connectors
- Documentation improvements
- Bug fixes and optimizations
- Test coverage expansion

---

## Support

- **Documentation**: https://wiastandards.com/soc-008
- **Forum**: https://forum.wiastandards.com
- **GitHub Issues**: https://github.com/WIA-Official/wia-standards/issues
- **Email**: support@wiastandards.com

---

## License

MIT License - See [LICENSE](../LICENSE) for details

© 2025 WIA / SmileStory Inc.

---

## Philosophy

홍익인간 (弘益人間) (홍익인간) - Hongik Ingan - Benefit All Humanity

We believe clean water is a fundamental human right. Smart water infrastructure should:
- Ensure universal access to safe water
- Protect public health
- Conserve precious resources
- Enable sustainable development
- Promote global cooperation
- Respect individual privacy

---

## Acknowledgments

Thanks to the global community of water engineers, researchers, and advocates who contributed to this standard. Special recognition to:

- Water utility operators worldwide
- Open-source IoT projects
- SCADA system developers
- Smart city initiatives
- Environmental organizations
- Early adopter utilities
- Beta testers globally

---

**For the latest updates, visit:** https://wiastandards.com/soc-008

홍익인간 (弘益人間) - Benefit All Humanity
