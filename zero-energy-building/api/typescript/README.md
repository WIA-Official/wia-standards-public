# @wia/zero-energy-building

TypeScript SDK for monitoring, analyzing, and certifying net-zero energy buildings according to WIA standards.

## Installation

```bash
npm install @wia/zero-energy-building
```

## Features

- 🔋 **Energy Production Tracking** - Monitor solar, wind, geothermal, and other renewable sources
- 📊 **Consumption Analysis** - Track energy use by category (heating, cooling, lighting, etc.)
- ⚖️ **Energy Balance Calculation** - Calculate net-zero status and self-sufficiency rates
- 🏆 **Certification Assessment** - Automated certification level evaluation (Platinum to Bronze)
- 💡 **Optimization Recommendations** - AI-powered suggestions for energy efficiency improvements
- 📡 **Real-time Monitoring** - Continuous energy flow monitoring and anomaly detection
- 🌍 **Carbon Footprint** - Calculate and track building carbon emissions

## Quick Start

```typescript
import { createZeroEnergyBuilding, EnergySourceType, ConsumptionCategory } from '@wia/zero-energy-building';

// Initialize the SDK
const zeb = createZeroEnergyBuilding({
  buildingId: 'building-001',
  enableRealTimeMonitoring: true,
  monitoringInterval: 60 // seconds
});

// Set up building metadata
await zeb.initializeBuilding({
  id: 'building-001',
  name: 'Green Office Complex',
  type: 'commercial',
  location: {
    address: '123 Sustainability St',
    latitude: 37.7749,
    longitude: -122.4194,
    climateZone: 'Zone 3C'
  },
  size: {
    floorArea: 5000, // m²
    volume: 15000, // m³
    stories: 3
  },
  occupancy: 200,
  yearBuilt: 2020
});
```

## Usage Examples

### Track Energy Production

```typescript
// Track solar panel production
await zeb.trackProduction({
  sourceType: EnergySourceType.SOLAR_PV,
  capacity: 100, // kW
  currentOutput: 75, // kW
  dailyProduction: 500, // kWh
  monthlyProduction: 15000, // kWh
  yearlyProduction: 180000, // kWh
  efficiency: 85, // percentage
  timestamp: new Date()
});

// Track wind turbine production
await zeb.trackProduction({
  sourceType: EnergySourceType.WIND,
  capacity: 50,
  currentOutput: 30,
  dailyProduction: 300,
  monthlyProduction: 9000,
  yearlyProduction: 108000,
  efficiency: 78,
  timestamp: new Date()
});
```

### Track Energy Consumption

```typescript
// Track HVAC consumption
await zeb.trackConsumption({
  category: ConsumptionCategory.HEATING,
  currentPower: 25, // kW
  dailyConsumption: 200, // kWh
  monthlyConsumption: 6000, // kWh
  yearlyConsumption: 72000, // kWh
  peakDemand: 35, // kW
  timestamp: new Date()
});

// Track lighting consumption
await zeb.trackConsumption({
  category: ConsumptionCategory.LIGHTING,
  currentPower: 10,
  dailyConsumption: 80,
  monthlyConsumption: 2400,
  yearlyConsumption: 28800,
  peakDemand: 15,
  timestamp: new Date()
});
```

### Calculate Energy Balance

```typescript
// Calculate daily balance
const dailyBalance = await zeb.calculateEnergyBalance('daily');
console.log(`Daily net balance: ${dailyBalance.netBalance} kWh`);
console.log(`Self-sufficiency: ${dailyBalance.selfSufficiencyRate.toFixed(1)}%`);

// Calculate yearly balance
const yearlyBalance = await zeb.calculateEnergyBalance('yearly');
console.log(`Yearly production: ${yearlyBalance.totalProduction} kWh`);
console.log(`Yearly consumption: ${yearlyBalance.totalConsumption} kWh`);
console.log(`Grid import: ${yearlyBalance.gridImport} kWh`);
console.log(`Grid export: ${yearlyBalance.gridExport} kWh`);
```

### Assess Certification Level

```typescript
const assessment = await zeb.assessCertification();

console.log(`Certification Level: ${assessment.level}`);
console.log(`Score: ${assessment.score}/100`);
console.log(`Self-Sufficiency Rate: ${assessment.selfSufficiencyRate.toFixed(1)}%`);
console.log(`Energy Intensity: ${assessment.energyIntensity.toFixed(2)} kWh/m²/year`);
console.log(`Carbon Footprint: ${assessment.carbonFootprint.toFixed(0)} kg CO2/year`);

console.log('\nRecommendations:');
assessment.recommendations.forEach((rec, i) => {
  console.log(`${i + 1}. ${rec}`);
});
```

### Get Optimization Recommendations

```typescript
const recommendations = await zeb.getOptimizationRecommendations();

recommendations.forEach(rec => {
  console.log(`\n${rec.title} (${rec.priority})`);
  console.log(`Category: ${rec.category}`);
  console.log(`Description: ${rec.description}`);
  console.log(`Estimated Savings: ${rec.estimatedSavings} kWh/year`);
  console.log(`Estimated Cost: $${rec.estimatedCost}`);
  console.log(`Payback Period: ${rec.paybackPeriod} years`);
  console.log(`Carbon Reduction: ${rec.carbonReduction} kg CO2/year`);
});
```

### Event Handling

```typescript
import { ZEBEventType } from '@wia/zero-energy-building';

// Listen for energy surplus events
zeb.on(ZEBEventType.ENERGY_SURPLUS, (event) => {
  console.log(`Energy Surplus: ${event.message}`);
  console.log(`Data:`, event.data);
});

// Listen for production anomalies
zeb.on(ZEBEventType.PRODUCTION_ANOMALY, (event) => {
  console.warn(`Production Anomaly: ${event.message}`);
  // Send alert to maintenance team
});

// Listen for consumption spikes
zeb.on(ZEBEventType.CONSUMPTION_SPIKE, (event) => {
  console.warn(`Consumption Spike: ${event.message}`);
  // Trigger demand response
});
```

## API Reference

### Main Class: `WIAZeroEnergyBuilding`

#### Methods

- `initializeBuilding(metadata: BuildingMetadata): Promise<void>` - Initialize building metadata
- `getBuildingMetadata(): BuildingMetadata | null` - Get current building metadata
- `trackProduction(production: EnergyProduction): Promise<void>` - Track energy production
- `trackConsumption(consumption: EnergyConsumption): Promise<void>` - Track energy consumption
- `calculateEnergyBalance(period, date?): Promise<EnergyBalance>` - Calculate energy balance
- `assessCertification(criteria?): Promise<CertificationAssessment>` - Assess certification level
- `getOptimizationRecommendations(): Promise<OptimizationRecommendation[]>` - Get optimization recommendations
- `on(eventType: ZEBEventType, handler: ZEBEventHandler): void` - Register event handler
- `off(eventType: ZEBEventType, handler: ZEBEventHandler): void` - Unregister event handler
- `startMonitoring(): void` - Start real-time monitoring
- `stopMonitoring(): void` - Stop real-time monitoring
- `destroy(): void` - Clean up resources

### Certification Levels

- `PLATINUM` - Score ≥ 95, highest efficiency and net-zero performance
- `GOLD` - Score ≥ 85, excellent energy performance
- `SILVER` - Score ≥ 70, good energy performance
- `BRONZE` - Score ≥ 50, meets minimum net-zero requirements
- `NOT_CERTIFIED` - Score < 50, does not meet requirements

### Event Types

- `ENERGY_SURPLUS` - Building produces more energy than consumed
- `ENERGY_DEFICIT` - Building consumes more energy than produced
- `PRODUCTION_ANOMALY` - Renewable energy system performance issue
- `CONSUMPTION_SPIKE` - Unusual increase in energy consumption
- `SYSTEM_MAINTENANCE` - System maintenance event
- `CERTIFICATION_RENEWAL` - Certification assessment completed
- `OPTIMIZATION_AVAILABLE` - New optimization recommendations available

## TypeScript Support

This package includes full TypeScript type definitions. All types are exported from the main module:

```typescript
import {
  WIAZeroEnergyBuilding,
  EnergyProduction,
  EnergyConsumption,
  EnergyBalance,
  CertificationAssessment,
  CertificationLevel,
  OptimizationRecommendation,
  ZEBEvent,
  ZEBEventType
  // ... and more
} from '@wia/zero-energy-building';
```

## License

MIT

## Support

For issues and questions:
- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Documentation: https://github.com/WIA-Official/wia-standards/tree/main/zero-energy-building

---

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
