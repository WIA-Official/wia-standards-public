# @wia/urban-heat-island

WIA Urban Heat Island Standard - TypeScript SDK for urban temperature management and heat mitigation.

## Installation

```bash
npm install @wia/urban-heat-island
```

## Quick Start

```typescript
import { createUrbanHeatIsland, UHIConfig } from '@wia/urban-heat-island';

// Configure the urban heat island system
const config: UHIConfig = {
  region: {
    name: 'Downtown Metropolitan Area',
    bounds: {
      north: 37.8,
      south: 37.7,
      east: -122.3,
      west: -122.5,
    },
  },
  monitoring: {
    updateInterval: 300, // 5 minutes
    temperatureUnit: 'celsius',
  },
  detection: {
    temperatureThreshold: 35,
    minimumArea: 10000,
    clusteringDistance: 500,
    severityThresholds: {
      low: 35,
      moderate: 38,
      high: 41,
      extreme: 44,
    },
  },
  analysis: {
    enableTrendAnalysis: true,
    historicalDataRetention: 90,
  },
};

// Create instance
const uhi = createUrbanHeatIsland(config);

// Record temperature readings
uhi.recordTemperature({
  value: 38.5,
  unit: 'celsius',
  timestamp: new Date(),
  location: { latitude: 37.75, longitude: -122.4 },
  source: 'sensor',
  reliability: 0.95,
});

// Start monitoring
uhi.startMonitoring();
```

## Features

### 🌡️ Temperature Monitoring

Monitor and map urban temperatures across your city:

```typescript
// Generate temperature map
const map = await uhi.generateTemperatureMap({
  north: 37.8,
  south: 37.7,
  east: -122.3,
  west: -122.5,
}, 100); // 100m resolution

// Get temperature history
const history = uhi.getTemperatureHistory(
  { latitude: 37.75, longitude: -122.4 },
  new Date('2024-01-01'),
  new Date('2024-12-31')
);
```

### 🔥 Hotspot Detection

Automatically detect and analyze heat hotspots:

```typescript
// Detect hotspots
const hotspots = await uhi.detectHotspots();

console.log(`Found ${hotspots.length} heat hotspots`);

// Filter by severity
const extremeHotspots = uhi.getHotspotsBySeverity('extreme');

// Analyze each hotspot
extremeHotspots.forEach(hotspot => {
  console.log(`Hotspot at (${hotspot.location.latitude}, ${hotspot.location.longitude})`);
  console.log(`Average temp: ${hotspot.averageTemperature}°C`);
  console.log(`Affected population: ${hotspot.affectedPopulation}`);
  console.log(`Contributing factors:`);
  hotspot.contributingFactors.forEach(factor => {
    console.log(`  - ${factor.type}: ${(factor.contribution * 100).toFixed(1)}%`);
  });
});
```

### 🛠️ Mitigation Planning

Create comprehensive mitigation plans:

```typescript
// Create mitigation plan
const plan = await uhi.createMitigationPlan(
  'Downtown Heat Reduction Initiative',
  ['hotspot-1', 'hotspot-2', 'hotspot-3'],
  1000000 // $1M budget
);

console.log(`Plan: ${plan.name}`);
console.log(`Total cost: $${plan.totalCost.toLocaleString()}`);
console.log(`Expected cooling: ${plan.estimatedCoolingEffect}°C`);
console.log(`\nStrategies:`);
plan.strategies.forEach(strategy => {
  console.log(`  - ${strategy.name}: ${strategy.estimatedCoolingEffect}°C reduction`);
});

console.log(`\nImplementation timeline:`);
plan.timeline.phases.forEach(phase => {
  console.log(`  ${phase.name} (${phase.duration} months)`);
});
```

### 💚 Cooling Strategies

Evaluate and implement cooling strategies:

```typescript
// Register a custom cooling strategy
uhi.registerStrategy({
  id: 'green-roof-downtown',
  name: 'Downtown Green Roof Program',
  type: 'green-infrastructure',
  description: 'Install green roofs on 50 downtown buildings',
  targetArea: {
    location: { latitude: 37.75, longitude: -122.4 },
    radius: 1000,
  },
  estimatedCoolingEffect: 3.5,
  implementationCost: 750000,
  maintenanceCost: 35000,
  lifespan: 25,
  cobenefits: [
    'Stormwater management',
    'Energy efficiency',
    'Air quality improvement',
    'Biodiversity habitat',
  ],
});

// Evaluate strategy effectiveness
const hotspot = hotspots[0];
const strategy = uhi.getStrategies()[0];
const evaluation = await uhi.evaluateStrategy(strategy, hotspot);

console.log(`Strategy: ${strategy.name}`);
console.log(`Effectiveness score: ${evaluation.effectivenessScore}/100`);
console.log(`Cost-effectiveness: ${evaluation.costEffectiveness.toFixed(2)}°C/$`);
console.log(`Recommendation: ${evaluation.recommendationLevel}`);
```

### 🌡️ Thermal Comfort

Calculate thermal comfort indices:

```typescript
// Calculate UTCI (Universal Thermal Climate Index)
const comfort = uhi.calculateThermalComfort({
  airTemperature: 35,
  relativeHumidity: 60,
  windSpeed: 2.5,
  solarRadiation: 800,
  location: { latitude: 37.75, longitude: -122.4 },
  timestamp: new Date(),
}, 'UTCI');

console.log(`Thermal comfort index: ${comfort.value}`);
console.log(`Stress level: ${comfort.stressLevel}`);
console.log(`Recommendation: ${comfort.recommendation}`);
```

### 📊 Analytics and Trends

Analyze temperature trends over time:

```typescript
// Analyze temperature trend
const trend = uhi.analyzeTemperatureTrend(
  { latitude: 37.75, longitude: -122.4 },
  new Date('2020-01-01'),
  new Date('2024-12-31')
);

console.log(`Temperature trend: ${trend.trend}`);
console.log(`Rate of change: ${trend.rateOfChange.toFixed(2)}°C/year`);
console.log(`Average: ${trend.averageTemperature}°C`);

// Calculate UHI intensity
const intensity = await uhi.calculateUHIIntensity(
  { latitude: 37.75, longitude: -122.4 }, // Urban location
  { latitude: 37.65, longitude: -122.2 }  // Rural reference
);

console.log(`Urban temperature: ${intensity.urbanTemperature}°C`);
console.log(`Rural temperature: ${intensity.ruralReferenceTemperature}°C`);
console.log(`UHI intensity: ${intensity.intensity}°C`);
console.log(`Affected area: ${intensity.affectedArea} km²`);
```

### 🔔 Event Handling

Subscribe to heat-related events:

```typescript
// Listen for hotspot detections
uhi.on('hotspot-detected', (event) => {
  console.log(`⚠️ New hotspot detected!`);
  console.log(`Severity: ${event.severity}`);
  console.log(`Location: ${event.location?.latitude}, ${event.location?.longitude}`);
  console.log(`Message: ${event.message}`);
});

// Listen for temperature threshold exceedances
uhi.on('temperature-threshold-exceeded', (event) => {
  console.log(`🌡️ Temperature threshold exceeded!`);
  console.log(`Reading: ${event.data.reading.value}°C`);
});

// Listen for mitigation milestones
uhi.on('mitigation-milestone', (event) => {
  console.log(`✅ Mitigation milestone reached!`);
  console.log(event.message);
});
```

## API Reference

### Main Class

#### `WIAUrbanHeatIsland`

The main class for urban heat island management.

**Constructor:**
```typescript
new WIAUrbanHeatIsland(config: UHIConfig)
```

**Methods:**

- **Temperature Monitoring**
  - `recordTemperature(reading: TemperatureReading): void`
  - `generateTemperatureMap(bounds, resolution?): Promise<TemperatureMap>`
  - `getTemperatureHistory(location, startDate, endDate): TemperatureReading[]`

- **Hotspot Detection**
  - `detectHotspots(config?): Promise<HeatHotspot[]>`
  - `getHotspots(): HeatHotspot[]`
  - `getHotspotsBySeverity(severity): HeatHotspot[]`

- **Mitigation Planning**
  - `createMitigationPlan(name, hotspotIds, budget): Promise<MitigationPlan>`
  - `getMitigationPlans(): MitigationPlan[]`

- **Cooling Strategies**
  - `evaluateStrategy(strategy, hotspot): Promise<StrategyEvaluation>`
  - `registerStrategy(strategy): void`
  - `getStrategies(): CoolingStrategy[]`

- **Thermal Comfort**
  - `calculateThermalComfort(input, indexType?): ThermalComfortResult`

- **Analytics**
  - `analyzeTemperatureTrend(location, startDate, endDate): TemperatureTrend`
  - `calculateUHIIntensity(urbanLocation, ruralLocation): Promise<UHIIntensity>`

- **Event Handling**
  - `on(eventType, handler): void`
  - `off(eventType, handler): void`

- **Monitoring Control**
  - `startMonitoring(): void`
  - `stopMonitoring(): void`

### Factory Function

```typescript
createUrbanHeatIsland(config: UHIConfig): WIAUrbanHeatIsland
```

## Types

See the [types documentation](./src/types.ts) for comprehensive type definitions.

## Examples

### Complete Monitoring Setup

```typescript
import { createUrbanHeatIsland } from '@wia/urban-heat-island';

const uhi = createUrbanHeatIsland({
  region: {
    name: 'City Center',
    bounds: { north: 40.8, south: 40.7, east: -73.9, west: -74.0 },
  },
  monitoring: {
    updateInterval: 300,
    temperatureUnit: 'celsius',
  },
  detection: {
    temperatureThreshold: 35,
    minimumArea: 10000,
    clusteringDistance: 500,
    severityThresholds: {
      low: 35,
      moderate: 38,
      high: 41,
      extreme: 44,
    },
  },
  analysis: {
    enableTrendAnalysis: true,
    historicalDataRetention: 90,
  },
});

// Set up event handlers
uhi.on('hotspot-detected', async (event) => {
  const hotspot = event.data.hotspot;

  // Create automatic mitigation plan for extreme hotspots
  if (hotspot.severity === 'extreme') {
    const plan = await uhi.createMitigationPlan(
      `Emergency Response - ${hotspot.id}`,
      [hotspot.id],
      500000
    );

    console.log(`Created emergency mitigation plan: ${plan.name}`);
  }
});

// Start monitoring
uhi.startMonitoring();

// Simulate temperature sensor data
setInterval(() => {
  uhi.recordTemperature({
    value: 30 + Math.random() * 15,
    unit: 'celsius',
    timestamp: new Date(),
    location: {
      latitude: 40.75 + (Math.random() - 0.5) * 0.1,
      longitude: -73.95 + (Math.random() - 0.5) * 0.1,
    },
    source: 'sensor',
    reliability: 0.9,
  });
}, 10000);
```

## Contributing

Contributions are welcome! Please see the [main repository](https://github.com/WIA-Official/wia-standards) for guidelines.

## License

MIT © WIA (World Certification Industry Association)

## Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

---

© 2025 SmileStory Inc. / WIA
