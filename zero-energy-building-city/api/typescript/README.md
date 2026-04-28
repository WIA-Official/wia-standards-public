# WIA Zero Energy Building City SDK

TypeScript SDK for managing city-wide net-zero energy districts, buildings, renewable sources, energy storage, and smart grid integration.

## Installation

```bash
npm install @wia/zero-energy-building-city
```

## Quick Start

```typescript
import { createZEBCitySDK, ZEBCityEventType } from '@wia/zero-energy-building-city';

// Initialize SDK
const sdk = createZEBCitySDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/zero-energy-city/v1'
});

// Get district energy profile
const energyData = await sdk.getDistrictEnergy('district-123');
console.log('District energy:', energyData.data);

// Balance grid
const balanceResult = await sdk.balanceGrid('district-123', {
  priority: 'SUSTAINABILITY',
  target_soc_percent: 80
});

// Forecast demand
const forecast = await sdk.forecastDemand('district-123', 24);
console.log('24-hour demand forecast:', forecast.data);
```

## Event Listeners

```typescript
// Listen to grid events
sdk.on(ZEBCityEventType.GRID_BALANCED, (event) => {
  console.log('Grid balanced:', event.data);
});

sdk.on(ZEBCityEventType.STORAGE_CHARGED, (event) => {
  console.log('Storage charged:', event.data);
});

sdk.on(ZEBCityEventType.CERTIFICATION_AWARDED, (event) => {
  console.log('Certification awarded:', event.data);
});
```

## Core Features

### District Energy Management

```typescript
// List all districts
const districts = await sdk.listDistricts({
  status: 'ACTIVE',
  certified: true
});

// Get comprehensive energy metrics
const energy = await sdk.getDistrictEnergy('district-123');

// Register new district
const newDistrict = await sdk.registerDistrict({
  name: 'Green Valley District',
  class: 'RESIDENTIAL',
  location: { /* ... */ },
  // ... other properties
});
```

### Grid Balancing & Optimization

```typescript
// Real-time grid balancing
const balance = await sdk.balanceGrid('district-123', {
  priority: 'RELIABILITY',
  max_grid_import_kW: 5000
});

// Optimize energy flow
const optimization = await sdk.optimizeFlow('district-123', 24, 'MINIMIZE_COST');

// Get grid connection status
const connection = await sdk.getGridConnection('district-123');
```

### Demand Forecasting

```typescript
// Forecast energy demand
const demandForecast = await sdk.forecastDemand('district-123', 48);

// Forecast renewable production
const productionForecast = await sdk.forecastProduction('solar-farm-456', 24);
```

### Storage Management

```typescript
// Manage storage systems
const result = await sdk.manageStorage('district-123', {
  action: 'OPTIMIZE',
  target_soc_percent: 85,
  duration_hours: 4
});

// Get city-wide storage status
const storage = await sdk.getCityStorage('district-123');

// Get battery details
const battery = await sdk.getBatteryBank('battery-789');
```

### Carbon Tracking

```typescript
// Track carbon footprint
const carbon = await sdk.trackCarbon('district-123', 'DISTRICT', {
  start_date: '2024-01-01',
  end_date: '2024-12-31'
});

// Get net-zero score
const score = await sdk.getNetZeroScore('district-123', 'DISTRICT');

// Calculate carbon offsets
const offsets = await sdk.calculateCarbonOffsets('district-123', 12);
```

### City Metrics

```typescript
// Get comprehensive city metrics
const metrics = await sdk.getCityMetrics('Seoul', {
  start_date: '2024-01-01',
  end_date: '2024-12-31'
});

// Compare districts
const comparison = await sdk.compareDistricts([
  'district-123',
  'district-456',
  'district-789'
]);
```

### Certification

```typescript
// Apply for certification
const cert = await sdk.certifyDistrict(
  'district-123',
  'DISTRICT',
  'LEED Zero',
  {
    verification_period_years: 1,
    verification_method: 'MEASURED',
    supporting_documents: ['doc1.pdf', 'doc2.pdf']
  }
);

// Check certification status
const status = await sdk.getCertification('cert-123');

// List all certifications
const certs = await sdk.listCertifications('district-123', 'DISTRICT');
```

### Urban Planning

```typescript
// Plan district expansion
const expansionPlan = await sdk.planExpansion('district-123', {
  new_buildings: 50,
  additional_area_km2: 2.5,
  target_population_increase: 5000,
  renewable_capacity_increase_MW: 10,
  storage_capacity_increase_MWh: 20,
  timeline_years: 5
});

// Simulate new district
const simulation = await sdk.simulateDistrict({
  location: { lat: 37.5665, lon: 126.9780, city: 'Seoul' },
  area_km2: 5,
  population: 10000,
  building_mix: { 'RESIDENTIAL': 70, 'COMMERCIAL': 30 },
  renewable_mix: { 'SOLAR_PV': 60, 'WIND': 40 }
});
```

### Building Management

```typescript
// Get building details
const building = await sdk.getBuilding('building-123');

// List buildings in district
const buildings = await sdk.listBuildings('district-123', {
  class: 'ZEB',
  building_type: 'RESIDENTIAL'
});

// Register new ZEB building
const newBuilding = await sdk.registerBuilding({
  name: 'Net Zero Tower',
  class: 'ZEB',
  // ... other properties
});
```

### Renewable Energy Sources

```typescript
// Get renewable source
const source = await sdk.getRenewableSource('solar-farm-123');

// List sources in district
const sources = await sdk.listRenewableSources('district-123', 'SOLAR_PV');

// Register solar farm
const solarFarm = await sdk.registerSolarFarm({
  type: 'SOLAR_PV',
  name: 'Solar Valley Farm',
  rated_capacity_kW: 5000,
  // ... other properties
});

// Register wind turbine
const windTurbine = await sdk.registerWindTurbine({
  type: 'WIND',
  name: 'Wind Ridge',
  rated_capacity_kW: 3000,
  // ... other properties
});
```

### Smart City Integration

```typescript
// Get data hub status
const dataHub = await sdk.getDataHub('district-123');

// Get control center
const controlCenter = await sdk.getControlCenter('Seoul');

// Get smart meter data
const meterData = await sdk.getSmartMeterData('meter-123', {
  start_date: '2024-12-01',
  end_date: '2024-12-31'
});
```

## Helper Functions

```typescript
import {
  calculateSelfSufficiency,
  calculateNetZeroBalance,
  isNetZero,
  calculateCarbonIntensity,
  formatEnergy,
  formatPower
} from '@wia/zero-energy-building-city';

// Calculate self-sufficiency
const selfSufficiency = calculateSelfSufficiency(900000, 1000000); // 90%

// Check if net-zero achieved
const netZero = isNetZero(1050000, 1000000, 5); // true

// Calculate carbon intensity
const intensity = calculateCarbonIntensity(50000, 1000000); // 0.05 kg CO2e/kWh

// Format energy values
console.log(formatEnergy(5500000)); // "5.50 GWh"
console.log(formatPower(12500)); // "12.50 MW"
```

## Error Handling

```typescript
import {
  ZEBCityError,
  DistrictNotFoundError,
  GridBalanceError,
  StorageCapacityError,
  CertificationError
} from '@wia/zero-energy-building-city';

try {
  const district = await sdk.getDistrictEnergy('invalid-id');
} catch (error) {
  if (error instanceof DistrictNotFoundError) {
    console.error('District not found:', error.details.districtId);
  } else if (error instanceof ZEBCityError) {
    console.error('SDK error:', error.code, error.message);
  }
}
```

## TypeScript Types

All types are fully documented with JSDoc comments:

```typescript
import type {
  EnergyDistrict,
  ZEBBuilding,
  CityGrid,
  Microgrid,
  SolarFarm,
  WindTurbine,
  BatteryBank,
  SmartMeter,
  CarbonFootprint,
  NetZeroScore,
  Certification
} from '@wia/zero-energy-building-city';
```

## Configuration Options

```typescript
const sdk = createZEBCitySDK({
  apiKey: 'your-api-key',              // Required
  endpoint: 'https://custom-api.com',  // Optional (default: WIA endpoint)
  timeout: 60000,                      // Optional (default: 30000ms)
  retries: 5,                          // Optional (default: 3)
  enableEventEmitter: true             // Optional (default: true)
});
```

## Event Types

- `DISTRICT_UPDATED` - District configuration or status updated
- `GRID_BALANCED` - Grid successfully balanced
- `STORAGE_CHARGED` - Battery/storage charged
- `STORAGE_DISCHARGED` - Battery/storage discharged
- `DEMAND_PEAK` - Peak demand detected in forecast
- `PRODUCTION_PEAK` - Peak production detected in forecast
- `CERTIFICATION_AWARDED` - Certification granted
- `CARBON_MILESTONE` - Carbon neutrality achieved
- `SYSTEM_ERROR` - System error occurred

## License

MIT

## Support

- Documentation: https://wia-official.org/standards/zero-energy-building-city
- Issues: https://github.com/WIA-Official/wia-standards/issues
- Email: support@wia-official.org

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
