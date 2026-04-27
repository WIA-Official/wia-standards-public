# WIA Water Scarcity Response SDK

TypeScript SDK for comprehensive drought management and water emergency response.

## Features

- **Drought Monitoring**: Real-time assessment using SPI, PDSI, and multiple indicators
- **Water Restrictions**: Flexible restriction phases from voluntary to emergency
- **Emergency Response**: Coordinated emergency activation and resource allocation
- **Alternative Supplies**: Manage desalination, recycled water, and emergency reserves
- **Rationing Systems**: Per-capita, rotating, and priority-based rationing
- **Early Warning**: Predictive drought forecasting and alert distribution
- **Event System**: Real-time notifications for all water scarcity events
- **Recovery Planning**: Structured post-drought recovery management

## Installation

```bash
npm install @wia/water-scarcity-response
```

## Quick Start

```typescript
import { WIAWaterScarcityResponse, DroughtLevel, RestrictionPhase } from '@wia/water-scarcity-response';

// Create instance
const wsr = new WIAWaterScarcityResponse({
  region: 'California',
  autoMonitoring: true,
  monitoringInterval: 3600000 // 1 hour
});

// Monitor drought conditions
const metrics = await wsr.assessDroughtConditions();
console.log(`Current drought level: ${metrics.droughtLevel}`);
console.log(`Reservoir storage: ${metrics.reservoirStorage}%`);

// Implement restrictions based on severity
if (metrics.droughtLevel === DroughtLevel.SEVERE_DROUGHT) {
  const restriction = await wsr.implementRestrictions(RestrictionPhase.PHASE_2);
  console.log(`Restrictions active: ${restriction.reductionTarget}% reduction target`);
}

// Register event handlers
wsr.on('drought_level_change', (event) => {
  console.log(`Drought level changed from ${event.previousLevel} to ${event.newLevel}`);
});

wsr.on('restriction_phase_change', (event) => {
  console.log(`Restriction phase changed to ${event.newPhase}`);
});
```

## API Reference

### Class: WIAWaterScarcityResponse

#### Constructor

```typescript
new WIAWaterScarcityResponse(options: {
  region: string;
  autoMonitoring?: boolean;
  monitoringInterval?: number;
})
```

#### Methods

**Drought Monitoring**
- `assessDroughtConditions(): Promise<DroughtMetrics>` - Assess current drought conditions

**Restrictions**
- `implementRestrictions(phase: RestrictionPhase): Promise<WaterRestriction>` - Implement water restrictions
- `getCurrentRestrictions(): WaterRestriction | null` - Get current restrictions

**Emergency Response**
- `activateEmergencyResponse(severity, targetRegions): Promise<EmergencyResponse>` - Activate emergency plan
- `getActiveEmergencyResponse(): EmergencyResponse | null` - Get active emergency

**Alternative Supplies**
- `registerAlternativeSupply(supply): AlternativeSupply` - Register new supply source
- `activateAlternativeSupply(supplyId): Promise<AlternativeSupply>` - Activate supply

**Rationing**
- `implementRationing(plan): Promise<RationingPlan>` - Implement rationing
- `createPerCapitaRationing(litersPerDay): RationingPlan` - Create per-capita plan
- `getRationingPlan(): RationingPlan | null` - Get current plan

**Early Warning**
- `issueEarlyWarning(level, forecastDays): Promise<EarlyWarning>` - Issue warning

**Recovery**
- `createRecoveryPlan(): RecoveryPlan` - Create recovery plan

**Events**
- `on(eventType, handler): void` - Register event handler
- `off(eventType, handler): void` - Remove event handler

**Monitoring**
- `stopAutoMonitoring(): void` - Stop automatic monitoring

## Usage Examples

### Emergency Response

```typescript
// Activate emergency response
const response = await wsr.activateEmergencyResponse('critical', [
  'Los Angeles',
  'San Diego',
  'San Jose'
]);

console.log(`Emergency ${response.id} activated`);
console.log(`${response.actions.length} actions planned`);
console.log(`${response.alternativeSupplies.length} supplies available`);
```

### Alternative Water Supply

```typescript
// Register desalination plant
const plant = wsr.registerAlternativeSupply({
  id: 'desal-001',
  type: AlternativeSupplyType.DESALINATION,
  capacity: 100000000, // 100M liters/day
  currentProduction: 0,
  location: { latitude: 33.7701, longitude: -118.1937 },
  status: 'standby',
  activationTime: 24,
  costPerLiter: 0.002,
  waterQuality: {
    ph: 7.5,
    tds: 500,
    turbidity: 0.5,
    certified: true
  }
});

// Activate when needed
await wsr.activateAlternativeSupply('desal-001');
```

### Water Rationing

```typescript
// Create per-capita rationing (50 liters per day)
const rationing = wsr.createPerCapitaRationing(50);

// Implement the plan
await wsr.implementRationing(rationing);

console.log(`Per capita allocation: ${rationing.perCapitaAllocation}L/day`);
console.log(`Sector priorities:`, rationing.sectorPriorities);
```

### Early Warning System

```typescript
// Issue drought warning
const warning = await wsr.issueEarlyWarning('warning', 30);

console.log(`Warning level: ${warning.level}`);
console.log(`Forecast: ${warning.forecast.droughtLevel} in ${warning.forecast.timeframe} days`);
console.log(`Confidence: ${warning.forecast.confidence * 100}%`);
console.log(`Actions: ${warning.recommendedActions.join(', ')}`);
```

### Event Monitoring

```typescript
// Monitor all events
wsr.on('drought_level_change', (event) => {
  console.log('🌵 Drought level changed:', event.newLevel);
});

wsr.on('restriction_phase_change', (event) => {
  console.log('⚠️ Restrictions updated:', event.newPhase);
});

wsr.on('emergency_activated', (event) => {
  console.log('🚨 Emergency activated:', event.response.severity);
});

wsr.on('supply_activated', (event) => {
  console.log('💧 Supply activated:', event.supply.type);
});

wsr.on('rationing_implemented', (event) => {
  console.log('📊 Rationing active:', event.plan.type);
});

wsr.on('early_warning_issued', (event) => {
  console.log('⚡ Warning issued:', event.warning.level);
});
```

## Types

### DroughtLevel
- `NORMAL`
- `ABNORMALLY_DRY`
- `MODERATE_DROUGHT`
- `SEVERE_DROUGHT`
- `EXTREME_DROUGHT`
- `EXCEPTIONAL_DROUGHT`

### RestrictionPhase
- `NONE`
- `VOLUNTARY`
- `PHASE_1` - 15% reduction
- `PHASE_2` - 25% reduction
- `PHASE_3` - 35% reduction
- `EMERGENCY` - 50% reduction

### AlternativeSupplyType
- `GROUNDWATER`
- `DESALINATION`
- `RECYCLED_WATER`
- `RAINWATER_HARVESTING`
- `WATER_IMPORT`
- `EMERGENCY_RESERVES`

## Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

This SDK is designed to help communities manage water scarcity effectively, protect vulnerable populations, and ensure equitable water distribution during drought emergencies.

## License

MIT

## Contributing

Contributions welcome! Please see our [contributing guidelines](https://github.com/WIA-Official/wia-standards/blob/main/CONTRIBUTING.md).

---

© 2025 SmileStory Inc. / WIA
