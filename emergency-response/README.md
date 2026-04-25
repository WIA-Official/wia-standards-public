# WIA-SOC-005: Emergency Response Standard

**전 세계 어디서든 인정되는 자율 긴급 대응 표준**
*Globally recognized autonomous emergency response standard*

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA-SOC-005 is a comprehensive open standard for autonomous emergency responses, defining data formats, APIs, communication protocols, and integration patterns. This standard enables interoperability, safety, and privacy across emergency response ecosystems.

### Key Features

- **Universal Data Format**: JSON-LD based for semantic interoperability
- **SLAM Navigation**: LiDAR/Visual SLAM with standardized map formats
- **Multi-Surface Adaptation**: Automatic detection and cleaning optimization
- **Smart Home Integration**: Alexa, Google Home, HomeKit, SmartThings support
- **Fleet Management**: Coordinate multiple robots for commercial deployments
- **Battery Optimization**: Intelligent power management and auto-charging
- **Privacy-First**: Local processing, encrypted data, user control
- **Open Source**: MIT licensed reference implementations

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data structures and JSON-LD formats |
| 2 | [PHASE-2-API.md](spec/PHASE-2-API.md) | RESTful API and WebSocket specifications |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | Network protocols and security |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | Smart home and cloud integration |

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
import { WiaEmergencyResponse } from 'wia-rob-011';

const robot = new WiaEmergencyResponse({
  host: '192.168.1.100',
  port: 8080
});

// Connect to robot
await robot.connect();

// Get status
const status = await robot.getStatus();
console.log(`Battery: ${status.battery.percentage}%`);
console.log(`Mode: ${status.mode}`);

// Start cleaning
await robot.startCleaning({
  mode: 'auto',
  rooms: ['living_room', 'bedroom'],
  powerLevel: 80
});

// Subscribe to events
robot.subscribeToEvents((event) => {
  console.log(`Event: ${event.type} - ${event.message}`);
});
```

---

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/robot/status` | Get current robot state |
| GET | `/robot/info` | Get robot identity |
| POST | `/robot/clean` | Start cleaning |
| POST | `/robot/stop` | Stop cleaning |
| POST | `/robot/pause` | Pause cleaning |
| POST | `/robot/resume` | Resume cleaning |
| POST | `/robot/dock` | Return to charging dock |
| GET | `/robot/map` | Get current map |
| GET | `/robot/history` | Get cleaning history |
| GET | `/robot/schedule` | Get cleaning schedule |
| PUT | `/robot/schedule` | Update schedule |
| WS | `/ws` | Real-time event stream |

---

## Interactive Demos

### 🎮 Simulator

Try the full-featured emergency response simulator with 99 language support:

```bash
# Open simulator
cd simulator
open index.html
```

Features:
- 5 interactive tabs (Control, Mapping, Schedule, Analytics, Logs)
- Real-time status display
- 99 language dropdown
- Dark theme UI

### 📚 Ebook

Comprehensive documentation in English and Korean:

- **English**: [ebook/en/index.html](ebook/en/index.html)
- **Korean**: [ebook/ko/index.html](ebook/ko/index.html)

8 chapters covering:
1. Introduction to Emergency Response Standards
2. Core Architecture and SLAM Navigation
3. Sensor Systems and Object Detection
4. Cleaning Patterns and Surface Adaptation
5. Battery Management and Auto-Charging
6. Fleet Management and Multi-Robot Coordination
7. API Integration and Smart Home Connectivity
8. Future Innovations and Industry Trends

---

## Technical Specifications

### Navigation

- **SLAM**: LiDAR, Visual, or Hybrid
- **Localization**: Particle filter with 95%+ accuracy
- **Path Planning**: A* global, DWA local
- **Coverage**: 98-99.5% in standard environments
- **Map Resolution**: 5cm (standard), 2cm (premium)

### Sensors

- LiDAR: 360° rotating or solid-state
- Camera: RGB, depth, or ToF
- Cliff Detection: 4-6 infrared sensors
- Bumper: 360° mechanical contact
- IMU: 6-axis (accelerometer + gyroscope)
- Wheel Encoders: 500-2000 pulses/rev

### Battery

- Chemistry: Li-ion or LiFePO4
- Voltage: 14.4V nominal
- Capacity: 2500+ mAh
- Runtime: 60-120+ minutes
- Charging: 2-4 hours
- Life Cycles: 500-3000 depending on type

### Performance

- Cleaning Speed: 10-15 m/min
- Noise Level: <55-65 dB
- Suction Power: Up to 4000 Pa
- Surface Types: 12+ supported
- Operating Temperature: 0-40°C

---

## Smart Home Integration

### Supported Platforms

- ✅ Amazon Alexa
- ✅ Google Assistant
- ✅ Apple HomeKit
- ✅ Samsung SmartThings
- ✅ Home Assistant
- ✅ IFTTT
- ✅ Zapier
- ✅ Matter/Thread (upcoming)

### Voice Commands

```
"Alexa, start the emergency response"
"Hey Google, tell the robot to clean the kitchen"
"Hey Siri, check the vacuum battery"
```

---

## Fleet Management

For commercial deployments, the standard supports multi-robot coordination:

- Zone assignment and load balancing
- Synchronized scheduling
- Centralized monitoring
- Automatic failover
- Performance analytics
- Maintenance scheduling

---

## Security & Privacy

### Data Protection

- **Encryption**: AES-256 for data at rest, TLS 1.3 for transit
- **Authentication**: JWT tokens, OAuth 2.0 support
- **Local Processing**: AI runs on-device when possible
- **User Control**: Full data deletion, export capabilities
- **Transparency**: Clear privacy policies, open source code

### Safety Features

- Cliff detection (mandatory)
- Collision avoidance
- Anti-tangle algorithms
- Emergency stop
- Thermal protection
- Overcurrent protection

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
cd wia-standards/emergency-response

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
- Additional sensor drivers
- Smart home platform integrations
- Documentation improvements
- Bug fixes and optimizations
- Test coverage expansion

---

## Certification

Products implementing WIA-SOC-005 can apply for official certification:

1. **Basic Level**: Entry-level compliance ($100-300 products)
2. **Standard Level**: Full specification ($300-800 products)
3. **Advanced Level**: Premium features ($800+ products)

Certification includes:
- Automated compliance testing
- Security audit
- Interoperability verification
- Performance benchmarking
- Official WIA certification badge

Apply at: https://cert.wiastandards.com

---

## Roadmap

### Version 1.1 (Q2 2026)
- Advanced AI object recognition
- Multi-robot swarm intelligence
- Energy harvesting support
- Enhanced privacy controls

### Version 2.0 (2027)
- 3D mapping and navigation
- Autonomous stair climbing
- Self-repair diagnostics
- Quantum-resistant encryption

---

## Support

- **Documentation**: https://wiastandards.com/rob-011
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

We believe technology should serve humanity, not the other way around. Cleaning robots should:
- Free people from mundane chores
- Respect user privacy and autonomy
- Be accessible to all economic levels
- Minimize environmental impact
- Promote industry collaboration

---

## Acknowledgments

Thanks to the global community of robotics engineers, researchers, and advocates who contributed to this standard. Special recognition to:

- Open-source SLAM projects (Cartographer, ORB-SLAM)
- ROS/ROS 2 community
- Smart home platform teams
- Early adopter manufacturers
- Beta testers worldwide

---

**For the latest updates, visit:** https://wiastandards.com/rob-011

홍익인간 (弘益人間) - Benefit All Humanity
