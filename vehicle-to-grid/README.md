# 🔋 WIA-AUTO-029: Vehicle-to-Grid (V2G) Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-AUTO-029
> **Version:** 1.0.0
> **Status:** Active
> **Category:** AUTO / Mobility
> **Color:** Orange (#F97316)

---

## 🌟 Overview

The WIA-AUTO-029 standard defines the comprehensive framework for Vehicle-to-Grid (V2G) technology, enabling bidirectional energy flow between electric vehicles and the power grid. This standard encompasses charging protocols, grid services, energy management, revenue models, and battery health optimization.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to transform electric vehicles into distributed energy resources that stabilize the grid, reduce energy costs, and accelerate the transition to renewable energy while preserving battery longevity.

## 🎯 Key Features

- **Bidirectional Charging**: Seamless power flow from grid-to-vehicle and vehicle-to-grid
- **Grid Services**: Frequency regulation, peak shaving, load balancing, and demand response
- **ISO 15118 Compliance**: Full support for V2G communication protocol
- **Energy Arbitrage**: Automated buy-low, sell-high energy trading strategies
- **Battery Protection**: Advanced algorithms to minimize degradation during V2G operations
- **Smart Grid Integration**: Real-time coordination with grid operators and energy markets
- **Revenue Optimization**: Maximize earnings from grid services and energy sales

## 📊 Core Concepts

### 1. Power Flow Equation

```
P_grid = P_charge - P_discharge
```

Where:
- `P_grid` = Net power to/from grid (positive = charging, negative = discharging)
- `P_charge` = Charging power (kW)
- `P_discharge` = Discharging power (kW)

### 2. State of Charge Management

```
SoC(t) = SoC(t-1) + (η × P × Δt) / C
```

Where:
- `SoC(t)` = State of charge at time t (0-1)
- `η` = Round-trip efficiency (typically 0.85-0.92)
- `P` = Power flow (kW, positive for charging)
- `Δt` = Time interval (hours)
- `C` = Battery capacity (kWh)

### 3. Energy Arbitrage Value

```
Revenue = ∫[P_sell(t) × Price_sell(t) - P_buy(t) × Price_buy(t)] dt
```

Where:
- `P_sell(t)` = Power sold to grid at time t
- `Price_sell(t)` = Selling price ($/kWh)
- `P_buy(t)` = Power bought from grid at time t
- `Price_buy(t)` = Buying price ($/kWh)

### 4. Battery Degradation Model

```
Degradation = α × DoD + β × Cycles + γ × Temperature
```

Where:
- `DoD` = Depth of discharge (0-1)
- `Cycles` = Number of charge/discharge cycles
- `α, β, γ` = Degradation coefficients
- `Temperature` = Battery temperature (°C)

## 🔧 Components

### TypeScript SDK

```typescript
import {
  V2GController,
  GridService,
  EnergyArbitrage,
  BatteryManager
} from '@wia/auto-029';

// Initialize V2G controller
const v2g = new V2GController({
  vehicleId: 'EV-12345',
  batteryCapacity: 75, // kWh
  maxChargePower: 11, // kW
  maxDischargePower: 10, // kW
  minSoC: 0.2, // Reserve 20% for driving
  maxSoC: 0.9 // Protect battery from overcharge
});

// Start grid service
await v2g.startGridService({
  service: 'frequency-regulation',
  duration: 3600, // 1 hour
  compensationRate: 25 // $/hour
});

// Enable energy arbitrage
await v2g.enableArbitrage({
  buyThreshold: 0.10, // Buy when price < $0.10/kWh
  sellThreshold: 0.30, // Sell when price > $0.30/kWh
  targetSoC: 0.7 // Maintain 70% charge for driving needs
});

// Check battery health
const health = await v2g.getBatteryHealth();
console.log(`Battery SoH: ${health.stateOfHealth}%`);
console.log(`Estimated cycles: ${health.cycleCount}`);
```

### CLI Tool

```bash
# Start V2G session
wia-auto-029 start --vehicle EV-12345 --service frequency-regulation

# Monitor current session
wia-auto-029 status

# Calculate arbitrage opportunity
wia-auto-029 arbitrage --buy-price 0.08 --sell-price 0.35 --duration 4h

# Check battery health impact
wia-auto-029 battery-health --cycles 500 --avg-dod 0.4

# Optimize charging schedule
wia-auto-029 optimize --departure 08:00 --target-soc 80

# View revenue report
wia-auto-029 revenue --period 30d
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-AUTO-029-v1.0.md](./spec/WIA-AUTO-029-v1.0.md) | Complete specification with technical details |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-auto-029.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/vehicle-to-grid

# Run installation script
./install.sh

# Verify installation
wia-auto-029 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/auto-029

# Or yarn
yarn add @wia/auto-029
```

```typescript
import { V2GController, calculateArbitrageRevenue } from '@wia/auto-029';

const controller = new V2GController({
  vehicleId: 'MY-EV',
  batteryCapacity: 60,
  maxChargePower: 7.4,
  maxDischargePower: 7.4
});

// Calculate potential revenue
const revenue = calculateArbitrageRevenue({
  batteryCapacity: 60,
  usableCapacity: 40, // 80% - 20% reserve
  buyPrice: 0.08,
  sellPrice: 0.32,
  efficiency: 0.88,
  cyclesPerDay: 1
});

console.log(`Daily revenue: $${revenue.daily.toFixed(2)}`);
console.log(`Monthly revenue: $${revenue.monthly.toFixed(2)}`);
console.log(`Annual revenue: $${revenue.annual.toFixed(2)}`);
```

## ⚡ Grid Services

| Service | Description | Compensation | Response Time |
|---------|-------------|--------------|---------------|
| Frequency Regulation | Maintain 60 Hz frequency | $15-50/MW/hour | < 4 seconds |
| Peak Shaving | Reduce peak demand | $0.20-0.50/kWh | < 1 minute |
| Load Balancing | Balance supply/demand | $10-30/MW/hour | < 10 minutes |
| Spinning Reserve | Emergency backup power | $5-20/MW/hour | < 10 minutes |
| Voltage Support | Regulate voltage levels | $8-25/MW/hour | < 5 seconds |

## 💰 Revenue Models

### 1. Energy Arbitrage
- **Strategy**: Buy electricity during low-price periods, sell during high-price periods
- **Potential**: $20-100/month per vehicle
- **Requirements**: Dynamic pricing plan (time-of-use or real-time pricing)

### 2. Frequency Regulation
- **Strategy**: Provide rapid response to frequency deviations
- **Potential**: $30-150/month per vehicle
- **Requirements**: Fast response capability (< 4 seconds), ISO 15118 support

### 3. Demand Response
- **Strategy**: Reduce charging during peak events
- **Potential**: $10-50/month per vehicle
- **Requirements**: Grid communication, flexible charging schedule

### 4. Capacity Market
- **Strategy**: Reserve battery capacity for grid emergencies
- **Potential**: $15-75/month per vehicle
- **Requirements**: Guaranteed availability, minimum capacity commitment

## 🔋 Battery Health Considerations

### Degradation Factors

1. **Depth of Discharge (DoD)**
   - Optimal: 20-80% SoC range
   - Impact: Each 10% increase in DoD → ~15% more degradation

2. **Cycle Count**
   - Most batteries: 1,000-3,000 cycles (80% capacity retention)
   - V2G recommendation: Limit to 0.5-1.0 cycles/day

3. **Charging Rate**
   - Optimal: 0.3-0.5C (C = capacity)
   - Fast charging > 1C increases degradation by 20-40%

4. **Temperature**
   - Optimal: 20-30°C
   - Above 40°C: Accelerated degradation (2x at 45°C)
   - Below 0°C: Reduced capacity and increased resistance

### Protection Strategies

```typescript
// Configure battery protection
const batteryConfig = {
  minSoC: 0.20,        // Never discharge below 20%
  maxSoC: 0.90,        // Never charge above 90%
  maxDoD: 0.60,        // Limit daily DoD to 60%
  maxCycles: 1.0,      // Maximum 1 full cycle per day
  temperatureMin: 10,  // Minimum operating temperature (°C)
  temperatureMax: 40,  // Maximum operating temperature (°C)
  maxChargePower: 0.5, // Limit to 0.5C charging rate
};
```

## 🌐 WIA Integration

This standard integrates with:
- **WIA-AUTO-001**: Electric Vehicle Charging
- **WIA-AUTO-002**: Battery Management Systems
- **WIA-ENERGY**: Smart grid and renewable energy standards
- **WIA-INTENT**: Intent-based energy management
- **WIA-OMNI-API**: Universal API gateway for grid services

## 📖 Use Cases

1. **Residential Grid Support**: Homeowners earn revenue while parked at home
2. **Commercial Fleet Management**: Reduce electricity costs for delivery fleets
3. **Renewable Energy Storage**: Store excess solar/wind energy during day, discharge at night
4. **Grid Emergency Response**: Provide backup power during outages or extreme weather
5. **Apartment Complex**: Aggregate multiple EVs for enhanced grid services
6. **Workplace Charging**: Office buildings offset electricity costs via employee EVs

## 🔒 Safety and Standards

### Compliance
- **ISO 15118**: V2G communication protocol
- **IEC 61851**: EV charging infrastructure
- **IEEE 2030.1.1**: Smart grid V2G integration
- **UL 1741**: Grid-tied inverter safety
- **SAE J2847/2**: Communication between plug-in vehicles and the utility grid

### Safety Features
- Automatic disconnection on grid fault
- Ground fault protection
- Over-current/over-voltage protection
- Temperature monitoring
- Emergency stop capability
- Anti-islanding protection

## ⚙️ Technical Specifications

| Parameter | Typical Range | Recommended |
|-----------|---------------|-------------|
| AC Charging Power | 3.3 - 22 kW | 7.4 - 11 kW |
| DC Charging Power | 50 - 350 kW | Not for V2G |
| Discharging Power | 3 - 10 kW | 5 - 10 kW |
| Round-trip Efficiency | 85 - 92% | > 88% |
| Response Time | 1 - 10 seconds | < 4 seconds |
| Communication | ISO 15118, OCPP | ISO 15118-20 |
| Grid Frequency | 50 / 60 Hz | Per region |
| Voltage Range | 220 - 240V (AC) | Per region |

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
