# 🚢 WIA-AUTO-015: Autonomous Ship Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-AUTO-015
> **Version:** 1.0.0
> **Status:** Active
> **Category:** AUTO / Mobility
> **Color:** Orange (#F97316)

---

## 🌟 Overview

The WIA-AUTO-015 standard defines the comprehensive framework for autonomous ship operations, including navigation systems, collision avoidance, remote monitoring, sensor integration, and maritime safety protocols compliant with IMO MASS (Maritime Autonomous Surface Ships) regulations.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to enhance maritime safety, reduce human error, optimize fuel efficiency, and make shipping more accessible while maintaining strict adherence to international maritime law and environmental protection.

## 🎯 Key Features

- **IMO MASS Autonomy Levels**: Compliance with international maritime autonomy standards
- **Advanced Navigation**: GPS, ECDIS, AIS integration for precise route planning
- **Collision Avoidance**: COLREG-compliant automatic collision prevention
- **Remote Operations**: Shore-based control centers for monitoring and intervention
- **Sensor Fusion**: Radar, LiDAR, cameras, and sonar for comprehensive awareness
- **Cybersecurity**: Maritime-grade security protocols for critical systems
- **Environmental Monitoring**: Real-time weather, sea state, and route optimization

## 📊 Core Concepts

### 1. Ship Position Calculation

```
Position(t) = Position(t₀) + ∫[t₀ to t] Velocity(τ) dτ
```

Where:
- `Position(t)` = Ship position at time t (latitude, longitude)
- `Velocity(τ)` = Ship velocity vector (speed and heading)
- `t₀` = Initial time
- `t` = Current time

### 2. Collision Risk Assessment (CPA/TCPA)

```
CPA = |Δr × v̂| / |v̂|
TCPA = -Δr · v̂ / |v|²
```

Where:
- `CPA` = Closest Point of Approach (nautical miles)
- `TCPA` = Time to Closest Point of Approach (minutes)
- `Δr` = Relative position vector
- `v̂` = Relative velocity unit vector
- `v` = Relative velocity magnitude

### 3. Great Circle Distance (Navigation)

```
d = R × arccos(sin(φ₁) × sin(φ₂) + cos(φ₁) × cos(φ₂) × cos(Δλ))
```

Where:
- `d` = Distance between two points
- `R` = Earth's radius (3,440.065 nautical miles)
- `φ₁, φ₂` = Latitudes of points 1 and 2
- `Δλ` = Difference in longitudes

### 4. Ship Domain Safety Zone

```
D = L × (1 + 2 × V/V_max)
```

Where:
- `D` = Safety domain radius
- `L` = Ship length
- `V` = Current speed
- `V_max` = Maximum service speed

## 🔧 Components

### TypeScript SDK

```typescript
import {
  AutonomousShipController,
  calculateCollisionRisk,
  planRoute,
  monitorEnvironment
} from '@wia/auto-015';

// Initialize autonomous ship controller
const ship = new AutonomousShipController({
  imo: '9876543',
  mmsi: '123456789',
  length: 300, // meters
  beam: 48,    // meters
  draft: 15    // meters
});

// Calculate collision risk with target vessel
const collision = calculateCollisionRisk({
  ownShip: {
    position: { lat: 35.6762, lon: 139.6503 },
    heading: 90,  // degrees
    speed: 15     // knots
  },
  target: {
    position: { lat: 35.6850, lon: 139.7500 },
    heading: 270,
    speed: 12
  }
});

console.log(`CPA: ${collision.cpa} NM, TCPA: ${collision.tcpa} minutes`);

// Plan optimal route
const route = await planRoute({
  origin: { lat: 35.6762, lon: 139.6503 },
  destination: { lat: 1.2897, lon: 103.8501 },
  weatherConstraints: {
    maxWaveHeight: 6, // meters
    maxWindSpeed: 30  // knots
  },
  optimizationGoal: 'fuel-efficiency'
});
```

### CLI Tool

```bash
# Initialize autonomous ship system
wia-auto-015 init --imo 9876543 --mmsi 123456789

# Calculate collision risk
wia-auto-015 collision-risk \
  --own-pos "35.6762,139.6503" \
  --own-heading 90 --own-speed 15 \
  --target-pos "35.6850,139.7500" \
  --target-heading 270 --target-speed 12

# Plan route with optimization
wia-auto-015 plan-route \
  --from "35.6762,139.6503" \
  --to "1.2897,103.8501" \
  --optimize fuel \
  --max-wave-height 6

# Monitor ship status
wia-auto-015 monitor --interval 5s

# Execute collision avoidance maneuver
wia-auto-015 avoid-collision --strategy "alter-course" --magnitude 15
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-AUTO-015-v1.0.md](./spec/WIA-AUTO-015-v1.0.md) | Complete specification with maritime protocols |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-auto-015.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/autonomous-ship

# Run installation script
./install.sh

# Verify installation
wia-auto-015 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/auto-015

# Or yarn
yarn add @wia/auto-015
```

```typescript
import { AutonomousShipSDK } from '@wia/auto-015';

const sdk = new AutonomousShipSDK({
  imo: '9876543',
  callsign: 'ABCD',
  autonomyLevel: 3 // IMO MASS Level 3
});

// Start autonomous navigation
await sdk.startNavigation({
  destination: { lat: 1.2897, lon: 103.8501 },
  maxSpeed: 18, // knots
  safetyMargin: 2 // nautical miles
});

// Monitor real-time status
sdk.on('collision-warning', (warning) => {
  console.log(`Collision risk detected: ${warning.severity}`);
  console.log(`Action: ${warning.recommendedAction}`);
});
```

## 🌊 IMO MASS Autonomy Levels

| Level | Name | Description | Human Presence |
|-------|------|-------------|----------------|
| 0 | Manual | Traditional ship operation | Onboard crew |
| 1 | On-board Decision Support | Automated processes with crew decision | Onboard crew |
| 2 | Remote Control | Controlled from shore | May have crew |
| 3 | Periodic Unmanned | Autonomous with remote monitoring | No crew (periods) |
| 4 | Fully Autonomous | Complete autonomous operation | No crew |

## ⚓ Maritime Navigation Constants

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| Earth Radius | R | 3,440.065 | nautical miles |
| Speed of Sound (seawater) | c | 1,500 | m/s |
| Safe Passing Distance | D_safe | 2.0 | nautical miles |
| COLREG Visibility Range | V_vis | 12.0 | nautical miles |
| Standard Gravity | g | 9.80665 | m/s² |
| Seawater Density | ρ | 1,025 | kg/m³ |

## ⚠️ Safety Considerations

1. **COLREG Compliance**: All maneuvers must comply with International Regulations for Preventing Collisions at Sea
2. **Redundancy**: Multiple sensor systems for fault tolerance
3. **Human Override**: Manual control capability at all autonomy levels
4. **Geofencing**: Restricted areas and safe operating zones
5. **Emergency Protocols**: Automatic safe harbor procedures
6. **Weather Monitoring**: Real-time severe weather detection and avoidance
7. **Communication**: AIS, VHF, and satellite communication backup

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Natural language ship commands
- **WIA-OMNI-API**: Universal maritime data API
- **WIA-SOCIAL**: Fleet coordination and communication
- **WIA-IoT**: Sensor network integration
- **WIA-BLOCKCHAIN**: Immutable voyage records

## 📖 Use Cases

1. **Container Shipping**: Autonomous transoceanic cargo transport
2. **Short-Sea Shipping**: Coastal and feeder services
3. **Research Vessels**: Autonomous oceanographic surveys
4. **Harbor Operations**: Automated port entry and docking
5. **Emergency Response**: Unmanned search and rescue operations
6. **Environmental Monitoring**: Autonomous patrol and data collection
7. **Icebreaker Operations**: Remote-controlled polar navigation

## 🛡️ Cybersecurity Features

- **Encrypted Communications**: AES-256 for all data transmission
- **Authentication**: Multi-factor authentication for remote access
- **Intrusion Detection**: Real-time security monitoring
- **Firewalls**: Segregated networks for critical systems
- **Audit Logging**: Complete voyage data recording
- **Secure Updates**: Cryptographically signed firmware

## 🌍 Environmental Benefits

- **Fuel Optimization**: AI-driven route planning reduces consumption by 10-15%
- **Emission Reduction**: Efficient operations lower CO₂, NOₓ, SOₓ emissions
- **Speed Optimization**: Weather routing minimizes fuel waste
- **Hull Efficiency**: Real-time trim and ballast optimization
- **Port Efficiency**: Reduced waiting time and optimized berthing

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)
- **IMO MASS**: [IMO Maritime Autonomous Surface Ships](https://www.imo.org/en/MediaCentre/HotTopics/Pages/Autonomous-shipping.aspx)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
